"""HTTP client backend for a StreamVLN inference server (docker/vln).

Wire contract (normative copy in vln_policy/DESIGN.md):
  GET  /health -> {"status": "ok", "backend": ..., "model": ..., "device": ...}
  POST /reset  {"instruction": str} -> {"session_id": str}
  POST /step   {"session_id": str, "image_jpeg_b64": str,
                "odom": {"x","y","yaw"}|null}
            -> {"actions": ["FORWARD"|...], "done": bool, "latency_ms": float}
"""

import base64
import time
from typing import Optional

import requests

from .base import (
    BackendError,
    OdomPose,
    StepResult,
    VLNBackend,
    validate_actions,
)


class StreamVLNHttpBackend(VLNBackend):
    name = "streamvln"
    requires_rgb = True

    def __init__(
        self,
        server_url: str = "http://localhost:18080",
        timeout_s: float = 30.0,
        jpeg_quality: int = 85,
        reset_retries: int = 3,
    ):
        self.server_url = server_url.rstrip("/")
        self.timeout_s = float(timeout_s)
        self.jpeg_quality = int(jpeg_quality)
        self.reset_retries = max(1, int(reset_retries))
        self._session_id = None

    # -- helpers ----------------------------------------------------------
    def _post(self, path: str, payload: dict) -> dict:
        try:
            resp = requests.post(
                f"{self.server_url}{path}", json=payload,
                timeout=self.timeout_s,
            )
        except requests.RequestException as exc:
            raise BackendError(
                f"{self.name} server unreachable at {self.server_url}{path}: "
                f"{exc}"
            ) from exc
        if resp.status_code != 200:
            raise BackendError(
                f"{self.name} server {path} returned HTTP {resp.status_code}: "
                f"{resp.text[:200]}"
            )
        try:
            return resp.json()
        except ValueError as exc:
            raise BackendError(
                f"{self.name} server {path} returned non-JSON body"
            ) from exc

    def _encode_jpeg_b64(self, rgb) -> str:
        import cv2  # deferred: only HTTP backends need it

        ok, buf = cv2.imencode(
            ".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            raise BackendError("failed to JPEG-encode RGB frame")
        return base64.b64encode(buf.tobytes()).decode("ascii")

    # -- VLNBackend -------------------------------------------------------
    def health(self) -> dict:
        try:
            resp = requests.get(
                f"{self.server_url}/health", timeout=self.timeout_s
            )
            resp.raise_for_status()
            return resp.json()
        except (requests.RequestException, ValueError) as exc:
            raise BackendError(
                f"{self.name} health check failed at {self.server_url}: {exc}"
            ) from exc

    def reset(self, instruction: str) -> None:
        last_exc = None
        for attempt in range(self.reset_retries):
            try:
                reply = self._post("/reset", {"instruction": instruction})
                self._session_id = str(reply.get("session_id", ""))
                if not self._session_id:
                    raise BackendError(
                        f"{self.name} /reset reply missing session_id"
                    )
                return
            except BackendError as exc:
                last_exc = exc
                if attempt < self.reset_retries - 1:
                    time.sleep(2.0 ** attempt)
        raise last_exc

    def step(self, rgb, odom: Optional[OdomPose]) -> StepResult:
        if self._session_id is None:
            raise BackendError(f"{self.name} step() before reset()")
        if rgb is None:
            raise BackendError(f"{self.name} step() requires an RGB frame")
        payload = {
            "session_id": self._session_id,
            "image_jpeg_b64": self._encode_jpeg_b64(rgb),
            "odom": (
                {"x": odom.x, "y": odom.y, "yaw": odom.yaw} if odom else None
            ),
        }
        reply = self._post("/step", payload)
        actions = validate_actions(reply.get("actions", []))
        if not actions:
            raise BackendError(f"{self.name} /step returned no actions")
        latency = reply.get("latency_ms")
        detail = f"latency {latency:.0f}ms" if latency is not None else ""
        return StepResult(
            actions=actions, done=bool(reply.get("done", False)),
            detail=detail,
        )
