"""FastAPI wrapper around StreamVLN implementing the vln_policy wire contract.

Contract (normative copy: src/semantic_nav/vln_policy/DESIGN.md):
  GET  /health -> {"status","backend","model","device"}       (503 if not loaded)
  POST /reset  {"instruction"} -> {"session_id"}
  POST /step   {"session_id","image_jpeg_b64","odom"|null}
            -> {"actions":[...], "done", "latency_ms"}

Inference glue mirrors StreamVLN's own streamvln/http_realworld_server.py
(one frame in -> 4 internal evaluator steps -> 4 discrete actions out), but
the instruction comes from /reset instead of being hardcoded, and the wire
format is ours so other VLN models (NaVILA/NaVid) can implement the same
contract. `odom` is accepted and ignored — StreamVLN is RGB-only.
"""

import base64
import os
import sys
import threading
import time
import uuid

import numpy as np
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

STREAMVLN_ROOT = os.environ.get("STREAMVLN_ROOT", "/opt/streamvln")
MODEL_DIR = os.environ.get("MODEL_DIR", "/opt/streamvln_ckpt")
DEVICE = os.environ.get("DEVICE", "cuda:0")
ATTN_IMPL = os.environ.get("ATTN_IMPL", "flash_attention_2")
NUM_FUTURE_STEPS = int(os.environ.get("NUM_FUTURE_STEPS", "4"))
NUM_FRAMES = int(os.environ.get("NUM_FRAMES", "32"))
NUM_HISTORY = int(os.environ.get("NUM_HISTORY", "8"))
MODEL_MAX_LENGTH = int(os.environ.get("MODEL_MAX_LENGTH", "4096"))
MODEL_REVISION = os.environ.get("MODEL_REVISION", "unknown")
PROTOCOL_VERSION = "2.0"
# upstream loads with low_cpu_mem_usage=False (whole model staged in system
# RAM before .to(device)); set LOW_CPU_MEM=1 on machines with <32 GB RAM
LOW_CPU_MEM = os.environ.get("LOW_CPU_MEM", "") in ("1", "yes", "YES")

# upstream layout: streamvln/model/... imported as `model.` with the package
# dir itself on sys.path (their server relies on script-dir resolution)
sys.path.insert(0, STREAMVLN_ROOT)
sys.path.insert(0, os.path.join(STREAMVLN_ROOT, "streamvln"))

ID_TO_ACTION = {0: "STOP", 1: "FORWARD", 2: "TURN_LEFT", 3: "TURN_RIGHT"}

app = FastAPI(title="StreamVLN server")

_state = {
    "evaluator": None,
    "load_error": None,
    "session_id": None,
    "instruction": "",
}
_lock = threading.Lock()


class ResetRequest(BaseModel):
    instruction: str


class StepRequest(BaseModel):
    session_id: str
    image_jpeg_b64: str
    odom: dict | None = None
    depth_png_b64: str | None = None
    depth_scale_m: float | None = None
    camera_intrinsics: dict | None = None
    image_timestamp_s: float | None = None


def _load_model():
    import argparse

    import torch
    import transformers
    from model.stream_video_vln import StreamVLNForCausalLM
    from streamvln.streamvln_agent import VLNEvaluator

    args = argparse.Namespace(
        model_path=MODEL_DIR,
        num_future_steps=NUM_FUTURE_STEPS,
        num_frames=NUM_FRAMES,
        num_history=NUM_HISTORY,
        model_max_length=MODEL_MAX_LENGTH,
        device=DEVICE,
    )
    tokenizer = transformers.AutoTokenizer.from_pretrained(
        MODEL_DIR, model_max_length=MODEL_MAX_LENGTH, padding_side="right"
    )
    config = transformers.AutoConfig.from_pretrained(MODEL_DIR)
    attn_impl = ATTN_IMPL
    if attn_impl == "flash_attention_2":
        try:
            import flash_attn  # noqa: F401
        except ImportError:
            print("flash-attn unavailable, falling back to sdpa")
            attn_impl = "sdpa"
    model = StreamVLNForCausalLM.from_pretrained(
        MODEL_DIR,
        attn_implementation=attn_impl,
        torch_dtype=torch.bfloat16,
        config=config,
        low_cpu_mem_usage=LOW_CPU_MEM,
    )
    model.model.num_history = NUM_HISTORY
    model.reset(1)
    model.requires_grad_(False)
    model.to(DEVICE)
    model.eval()

    # matches upstream http_realworld_server.py sensor config
    vln_sensor_config = {
        "rgb_height": 1.25,
        "camera_intrinsic": np.array([
            [192.0, 0.0, 191.42857143, 0.0],
            [0.0, 192.0, 191.42857143, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]),
    }
    evaluator = VLNEvaluator(
        vln_sensor_config, model=model, tokenizer=tokenizer, args=args
    )
    # warmup, as upstream does
    evaluator.step(0, np.zeros((480, 640, 3), dtype=np.uint8),
                   "move forward 25 cm", run_model=True)
    evaluator.reset_memory()
    return evaluator


@app.on_event("startup")
def startup():
    try:
        _state["evaluator"] = _load_model()
        print(f"StreamVLN loaded from {MODEL_DIR} on {DEVICE}")
    except Exception as exc:  # noqa: BLE001 — keep serving /health with detail
        _state["load_error"] = f"{type(exc).__name__}: {exc}"
        print(f"MODEL LOAD FAILED: {_state['load_error']}")
        print("Was the image built with STREAMVLN_MODEL=YES ?")


@app.get("/health")
def health():
    if _state["evaluator"] is None:
        raise HTTPException(
            status_code=503,
            detail=f"model not loaded: {_state['load_error']}",
        )
    return {
        "status": "ok",
        "backend": "streamvln",
        "model": os.path.basename(MODEL_DIR.rstrip("/")) or MODEL_DIR,
        "model_revision": MODEL_REVISION,
        "protocol_version": PROTOCOL_VERSION,
        "capabilities": {
            "outputs": ["actions"],
            "rgb": True,
            "depth": False,
            "timings": ["total_ms", "preprocessing_ms", "system2_ms"],
        },
        "device": DEVICE,
    }


@app.post("/reset")
def reset(req: ResetRequest):
    if _state["evaluator"] is None:
        raise HTTPException(503, f"model not loaded: {_state['load_error']}")
    instruction = req.instruction.strip()
    if not instruction:
        raise HTTPException(400, "instruction must not be blank")
    with _lock:
        _state["evaluator"].reset_memory()
        _state["session_id"] = uuid.uuid4().hex
        _state["instruction"] = instruction
    print(f"reset: instruction='{instruction}'")
    return {"session_id": _state["session_id"]}


@app.post("/step")
def step(req: StepRequest):
    if _state["evaluator"] is None:
        raise HTTPException(503, f"model not loaded: {_state['load_error']}")
    import cv2

    total_started = time.perf_counter()
    preprocessing_started = time.perf_counter()
    jpeg = np.frombuffer(base64.b64decode(req.image_jpeg_b64), dtype=np.uint8)
    bgr = cv2.imdecode(jpeg, cv2.IMREAD_COLOR)
    if bgr is None:
        raise HTTPException(400, "image_jpeg_b64 did not decode as an image")
    preprocessing_ms = (time.perf_counter() - preprocessing_started) * 1000.0
    # upstream feeds BGR (PIL RGB reversed); cv2 already decodes to BGR

    inference_started = time.perf_counter()
    with _lock:
        if req.session_id != _state["session_id"]:
            raise HTTPException(409, "unknown session_id (server was reset)")
        evaluator = _state["evaluator"]
        action_seq = None
        # one frame in -> NUM_FUTURE_STEPS internal steps, model regenerates
        # every num_future_steps'th step (upstream realworld loop)
        for _ in range(NUM_FUTURE_STEPS):
            ret_action, _gen_time, _llm_out = evaluator.step(
                0, bgr, _state["instruction"],
                run_model=(evaluator.step_id % NUM_FUTURE_STEPS == 0),
            )
            if ret_action is not None:
                action_seq = ret_action
            evaluator.step_id += 1
    system2_ms = (time.perf_counter() - inference_started) * 1000.0

    ids = [int(a) for a in (action_seq if action_seq is not None else [0])]
    actions = []
    done = False
    for aid in ids:
        actions.append(ID_TO_ACTION.get(aid, "STOP"))
        if aid == 0:
            done = True
            break

    total_ms = (time.perf_counter() - total_started) * 1000.0
    print(f"step: {actions} done={done} {total_ms:.0f}ms")
    return {
        "actions": actions,
        "done": done,
        # Retained for protocol-v1 clients.
        "latency_ms": total_ms,
        "timings": {
            "total_ms": total_ms,
            "preprocessing_ms": preprocessing_ms,
            "system1_ms": None,
            "system2_ms": system2_ms,
        },
    }
