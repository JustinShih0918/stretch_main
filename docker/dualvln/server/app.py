"""Owned protocol-v2 adapter for InternVLA-N1 DualVLN.

This deliberately does not reuse the upstream Flask example: that example
hard-codes an instruction and calibration and has no session contract.
"""

import argparse
import base64
import binascii
import math
import os
import threading
import time
import types
import uuid

import cv2
import numpy as np
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field

INTERNNAV_REVISION = "7a5c62400ac45b313d9b709c740b64191556a242"
CHECKPOINT_REVISION = os.environ.get(
    "CHECKPOINT_REVISION", "a698a9e898b4001621a319e1bc89f02ec715cc86"
)
MODEL_DIR = os.environ.get("MODEL_DIR", "/opt/dualvln_ckpt")
DEVICE = os.environ.get("DEVICE", "cuda:0")
PROTOCOL_VERSION = "2.0"
ACTION_NAMES = {
    0: "STOP", 1: "FORWARD", 2: "TURN_LEFT", 3: "TURN_RIGHT"
}
LOOK_DOWN = 5

app = FastAPI(title="DualVLN protocol adapter")
_lock = threading.Lock()
_state = {
    "agent": None,
    "session_id": None,
    "instruction": "",
    "load_error": None,
    "component_timings": None,
}


class ResetRequest(BaseModel):
    instruction: str = Field(min_length=1)


class StepRequest(BaseModel):
    session_id: str
    image_jpeg_b64: str
    depth_png_b64: str
    depth_scale_m: float = Field(gt=0.0)
    camera_intrinsics: dict
    image_timestamp_s: float | None = None
    odom: dict | None = None


def _install_timing_hooks(agent):
    """Measure the real S1/S2 calls without changing pinned upstream code."""
    original_s1 = agent.step_s1
    original_s2 = agent.step_s2

    def timed_s1(self, *args, **kwargs):
        started = time.perf_counter()
        try:
            return original_s1(*args, **kwargs)
        finally:
            _state["component_timings"]["system1_ms"] += (
                time.perf_counter() - started
            ) * 1000.0

    def timed_s2(self, *args, **kwargs):
        started = time.perf_counter()
        try:
            return original_s2(*args, **kwargs)
        finally:
            _state["component_timings"]["system2_ms"] += (
                time.perf_counter() - started
            ) * 1000.0

    agent.step_s1 = types.MethodType(timed_s1, agent)
    agent.step_s2 = types.MethodType(timed_s2, agent)


def _load_model():
    from internnav.agent.internvla_n1_agent_realworld import (
        InternVLAN1AsyncAgent,
    )

    args = argparse.Namespace(
        device=DEVICE,
        model_path=MODEL_DIR,
        resize_w=int(os.environ.get("RESIZE_W", "384")),
        resize_h=int(os.environ.get("RESIZE_H", "384")),
        num_history=int(os.environ.get("NUM_HISTORY", "8")),
        plan_step_gap=int(os.environ.get("PLAN_STEP_GAP", "0")),
    )
    agent = InternVLAN1AsyncAgent(args)
    _install_timing_hooks(agent)
    _state["component_timings"] = {"system1_ms": 0.0, "system2_ms": 0.0}
    # Warm the exact paired RGB/depth inference surface used below.
    agent.step(
        np.zeros((480, 640, 3), dtype=np.uint8),
        np.zeros((480, 640), dtype=np.float32),
        np.eye(4),
        "warm up",
        intrinsic=np.array([
            [585.0, 0.0, 320.0, 0.0],
            [0.0, 585.0, 240.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]),
    )
    agent.reset()
    agent.last_s2_idx = -100
    return agent


@app.on_event("startup")
def startup():
    try:
        _state["agent"] = _load_model()
        print(f"DualVLN loaded from {MODEL_DIR} on {DEVICE}")
    except Exception as exc:  # noqa: BLE001
        _state["load_error"] = f"{type(exc).__name__}: {exc}"
        print(f"MODEL LOAD FAILED: {_state['load_error']}")


@app.get("/health")
def health():
    if _state["agent"] is None:
        raise HTTPException(503, f"model not loaded: {_state['load_error']}")
    return {
        "status": "ok",
        "backend": "dualvln",
        "model": "InternRobotics/InternVLA-N1-DualVLN",
        "model_revision": CHECKPOINT_REVISION,
        "internnav_revision": INTERNNAV_REVISION,
        "protocol_version": PROTOCOL_VERSION,
        "capabilities": {
            "outputs": ["actions", "trajectory"],
            "rgb": True,
            "depth": True,
            "camera_intrinsics": True,
            "timings": [
                "total_ms", "preprocessing_ms", "system1_ms", "system2_ms"
            ],
        },
        "device": DEVICE,
    }


@app.post("/reset")
def reset(req: ResetRequest):
    if _state["agent"] is None:
        raise HTTPException(503, f"model not loaded: {_state['load_error']}")
    instruction = req.instruction.strip()
    if not instruction:
        raise HTTPException(400, "instruction must not be blank")
    with _lock:
        agent = _state["agent"]
        agent.reset()
        # Upstream reset omits this scheduler field; carrying it over can skip
        # System 2 at the start of the next session.
        agent.last_s2_idx = -100
        _state["instruction"] = instruction
        _state["session_id"] = uuid.uuid4().hex
        session_id = _state["session_id"]
    return {"session_id": session_id}


def _decode_observation(req):
    started = time.perf_counter()
    try:
        rgb_bytes = base64.b64decode(req.image_jpeg_b64, validate=True)
        depth_bytes = base64.b64decode(req.depth_png_b64, validate=True)
    except (binascii.Error, ValueError) as exc:
        raise HTTPException(400, "image payload is not valid base64") from exc
    bgr = cv2.imdecode(np.frombuffer(rgb_bytes, np.uint8), cv2.IMREAD_COLOR)
    depth_units = cv2.imdecode(
        np.frombuffer(depth_bytes, np.uint8), cv2.IMREAD_UNCHANGED
    )
    if bgr is None or depth_units is None or depth_units.ndim != 2:
        raise HTTPException(400, "RGB JPEG or depth PNG did not decode")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    depth = depth_units.astype(np.float32) * float(req.depth_scale_m)
    if rgb.shape[:2] != depth.shape:
        raise HTTPException(400, "RGB and depth dimensions must match")

    required = ("fx", "fy", "cx", "cy", "width", "height")
    if any(name not in req.camera_intrinsics for name in required):
        raise HTTPException(400, "camera_intrinsics is incomplete")
    values = [float(req.camera_intrinsics[name]) for name in required[:4]]
    if not all(math.isfinite(value) for value in values):
        raise HTTPException(400, "camera intrinsics must be finite")
    fx, fy, cx, cy = values
    if fx <= 0.0 or fy <= 0.0:
        raise HTTPException(400, "camera focal lengths must be positive")
    if (
        int(req.camera_intrinsics["width"]) != rgb.shape[1]
        or int(req.camera_intrinsics["height"]) != rgb.shape[0]
    ):
        raise HTTPException(400, "camera calibration dimensions do not match RGB")
    intrinsic = np.array([
        [fx, 0.0, cx, 0.0],
        [0.0, fy, cy, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
    pose = np.eye(4)
    if req.odom is not None:
        try:
            x = float(req.odom["x"])
            y = float(req.odom["y"])
            yaw = float(req.odom["yaw"])
        except (KeyError, TypeError, ValueError) as exc:
            raise HTTPException(400, "odom must contain finite x, y, yaw") from exc
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise HTTPException(400, "odom must contain finite x, y, yaw")
        pose[:2, :2] = [[math.cos(yaw), -math.sin(yaw)],
                        [math.sin(yaw), math.cos(yaw)]]
        pose[0, 3], pose[1, 3] = x, y
    preprocessing_ms = (time.perf_counter() - started) * 1000.0
    return rgb, depth, pose, intrinsic, preprocessing_ms


def _action_ids(output):
    """Upstream returns a list (possibly empty) or None for actions."""
    if output.output_action is None:
        return None
    return [int(value) for value in output.output_action]


def _response_from_output(output):
    ids = _action_ids(output)
    if ids is not None:
        unknown = [value for value in ids if value not in ACTION_NAMES]
        if unknown:
            detail = f"model returned unknown action IDs {unknown}"
            if LOOK_DOWN in unknown:
                detail += " (look-down was already retried once)"
            raise HTTPException(500, detail)
        actions = [ACTION_NAMES[value] for value in ids]
        if "STOP" in actions:
            actions = actions[:actions.index("STOP") + 1]
        return {"actions": actions, "done": "STOP" in actions}
    if output.output_trajectory is None:
        raise HTTPException(500, "model returned neither actions nor a trajectory")
    trajectory = np.asarray(output.output_trajectory)
    if trajectory.ndim != 2 or trajectory.shape[0] == 0 or trajectory.shape[1] < 2:
        raise HTTPException(500, "model returned a malformed trajectory")
    trajectory = trajectory[:, :2].astype(float)
    if not np.all(np.isfinite(trajectory)):
        raise HTTPException(500, "model returned a non-finite trajectory")
    return {"trajectory": trajectory.tolist(), "done": False}


def _agent_step(rgb, depth, pose, intrinsic, look_down):
    return _state["agent"].step(
        rgb, depth, pose, _state["instruction"], intrinsic=intrinsic,
        look_down=look_down,
    )


@app.post("/step")
def step(req: StepRequest):
    if _state["agent"] is None:
        raise HTTPException(503, f"model not loaded: {_state['load_error']}")
    rgb, depth, pose, intrinsic, preprocessing_ms = _decode_observation(req)
    total_started = time.perf_counter()
    with _lock:
        if req.session_id != _state["session_id"]:
            raise HTTPException(409, "unknown session_id (server was reset)")
        _state["component_timings"] = {"system1_ms": 0.0, "system2_ms": 0.0}
        output = _agent_step(rgb, depth, pose, intrinsic, False)
        if _action_ids(output) == [LOOK_DOWN]:
            output = _agent_step(rgb, depth, pose, intrinsic, True)
        if _action_ids(output) == []:
            # The language head emitted neither coordinates nor an action
            # token. Both agent outputs are cleared at that point, so one more
            # step re-runs System 2 on the newest history instead of aborting
            # a 300 s benchmark episode on a single unparseable reply.
            output = _agent_step(rgb, depth, pose, intrinsic, False)
            if _action_ids(output) == []:
                raise HTTPException(
                    500, "model produced no parseable action twice in a row"
                )
        response = _response_from_output(output)
        components = dict(_state["component_timings"])
    total_ms = (time.perf_counter() - total_started) * 1000.0 + preprocessing_ms
    response["timings"] = {
        "total_ms": total_ms,
        "preprocessing_ms": preprocessing_ms,
        "system1_ms": components["system1_ms"],
        "system2_ms": components["system2_ms"],
    }
    return response
