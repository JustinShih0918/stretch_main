"""The gateway's side of OSG: the runner in the OSG container, and the
pre-flight check before a run.

The runner (control_panel/osg_runner/runner.py) answers HTTP on the Thor's
loopback; this polls its state once a second for the page and forwards start /
stop requests. The check combines what only HTTP can see (the perception
servers, ollama, the runner, the API key) with what only ROS can see (the
robot's half, `RosIO.ros_checks`).
"""
from __future__ import annotations

import json
import os
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import List, Optional

import yaml

PRESETS_FILE = Path(__file__).resolve().parent / "osg_presets.yaml"


class OsgRunner:
    def __init__(self, url: Optional[str] = None):
        self.url = (url or os.environ.get("PANEL_OSG_RUNNER", "http://127.0.0.1:18790")).rstrip("/")
        self.state: Optional[dict] = None
        self.stamp = 0.0
        self.error = ""
        threading.Thread(target=self._poll, daemon=True, name="osg-runner").start()

    def _poll(self):
        while True:
            try:
                with urllib.request.urlopen(self.url + "/state", timeout=2) as r:
                    self.state = json.loads(r.read())
                    self.stamp = time.time()
                    self.error = ""
            except (urllib.error.URLError, OSError, ValueError) as e:
                self.error = str(getattr(e, "reason", e))
            time.sleep(1.0)

    def alive(self) -> bool:
        return time.time() - self.stamp < 3.0

    def request(self, req: dict, timeout: float = 30.0) -> (bool, str):
        data = json.dumps(req).encode()
        rq = urllib.request.Request(self.url + "/request", data=data,
                                    headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(rq, timeout=timeout) as r:
                out = json.loads(r.read())
            return bool(out.get("ok")), str(out.get("message", ""))
        except (urllib.error.URLError, OSError, ValueError) as e:
            return False, f"OSG runner unreachable at {self.url}: {getattr(e, 'reason', e)} " \
                          f"(Thor: docker compose ... up -d thor)"

    def snapshot(self) -> dict:
        return {"alive": self.alive(), "error": self.error if not self.alive() else "",
                "state": self.state if self.alive() else None}


def http_up(url: str, timeout: float = 1.5) -> (bool, str):
    """Anything answering HTTP counts as up (a 404 or 405 is still a server)."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return True, f"HTTP {r.status}"
    except urllib.error.HTTPError as e:
        return True, f"HTTP {e.code}"
    except (urllib.error.URLError, OSError) as e:
        return False, str(getattr(e, "reason", e))


def http_checks(runner: OsgRunner, need_inspection: bool) -> List[dict]:
    out = []

    def add(key, label, ok, detail="", hard=True):
        out.append({"key": key, "label": label, "ok": ok, "detail": detail, "hard": hard})

    add("runner", "OSG 容器（runner）", runner.alive(),
        "" if runner.alive() else f"{runner.url}: {runner.error} → Thor 上 up -d thor")
    ports = [p.strip() for p in os.environ.get("PANEL_PERCEPTION_PORTS", "13182:blip2itm,13185:ram").split(",") if p.strip()]
    for item in ports:
        port, _, name = item.partition(":")
        ok, why = http_up(f"http://127.0.0.1:{port}/{name}")
        add(f"perception_{port}", f"感知模型 {name or port} :{port}", ok,
            why if ok else f"{why} → 啟動「感知模型」，載入約 90 秒")
    ollama = os.environ.get("PANEL_OLLAMA_URL", "http://127.0.0.1:11435")
    model = os.environ.get("PANEL_OLLAMA_MODEL", "qwen2.5:7b")
    if ollama:
        try:
            with urllib.request.urlopen(ollama.rstrip("/") + "/api/tags", timeout=2) as r:
                names = [m.get("name", "") for m in json.loads(r.read()).get("models", [])]
            add("ollama", f"ollama 有 {model}", model in names,
                "" if model in names else f"沒有 {model}：docker exec osg-ollama ollama pull {model}")
        except (urllib.error.URLError, OSError, ValueError) as e:
            add("ollama", "ollama", False, f"{getattr(e, 'reason', e)} → up -d ollama")
    else:
        add("ollama", "ollama", None, "PANEL_OLLAMA_URL 未設定，略過")
    key = bool((runner.state or {}).get("nvidia_api_key"))
    add("nvidia_key", "VLM 金鑰 NVIDIA_API_KEY", key if need_inspection else (key or None),
        "" if key else "滅火器檢查需要：source ~/.nvidia_api.env 後重新 up -d thor", hard=need_inspection)
    return out


def load_presets() -> dict:
    try:
        return yaml.safe_load(PRESETS_FILE.read_text()) or {}
    except (OSError, yaml.YAMLError) as e:
        return {"error": str(e)}
