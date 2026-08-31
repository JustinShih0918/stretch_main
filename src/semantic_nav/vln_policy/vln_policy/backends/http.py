"""Shared HTTP protocol-v2 client used by all remote VLN backends."""

import base64
import math
import threading
import time
from typing import Optional

import requests

from .base import (
    BackendError,
    CameraIntrinsics,
    OdomPose,
    StepResult,
    StepTimings,
    TrajectoryPoint,
    VLNBackend,
    validate_actions,
)


def _optional_finite_ms(value, field: str) -> Optional[float]:
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise BackendError(f"invalid timing field {field}: {value!r}") from exc
    if not math.isfinite(result) or result < 0.0:
        raise BackendError(f"invalid timing field {field}: {value!r}")
    return result


def parse_timings(reply: dict, client_ms: float) -> StepTimings:
    """Accept protocol-v2 nested timings and legacy ``latency_ms``."""
    raw = reply.get("timings") or {}
    if not isinstance(raw, dict):
        raise BackendError("/step timings must be an object")

    def first(*names):
        for name in names:
            if name in raw:
                return raw[name]
            if name in reply:
                return reply[name]
        return None

    return StepTimings(
        client_ms=_optional_finite_ms(client_ms, "client_ms"),
        total_ms=_optional_finite_ms(
            first("total_ms", "latency_ms"), "total_ms"
        ),
        preprocessing_ms=_optional_finite_ms(
            first("preprocessing_ms", "preprocess_ms"), "preprocessing_ms"
        ),
        system1_ms=_optional_finite_ms(
            first("system1_ms", "system_1_ms"), "system1_ms"
        ),
        system2_ms=_optional_finite_ms(
            first("system2_ms", "system_2_ms"), "system2_ms"
        ),
    )


def validate_trajectory(points) -> list:
    if not isinstance(points, list) or not points:
        raise BackendError("/step trajectory must be a non-empty list")
    result = []
    for index, point in enumerate(points):
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise BackendError(
                f"trajectory point {index} must be exactly [x_m, y_m]"
            )
        try:
            x, y = float(point[0]), float(point[1])
        except (TypeError, ValueError) as exc:
            raise BackendError(
                f"trajectory point {index} contains a non-number"
            ) from exc
        if not math.isfinite(x) or not math.isfinite(y):
            raise BackendError(f"trajectory point {index} is not finite")
        result.append(TrajectoryPoint(x=x, y=y))
    return result


class HttpVLNBackend(VLNBackend):
    """Session-oriented JSON/base64 client with strict response validation."""

    name = "http"

    def __init__(
        self,
        server_url: str,
        timeout_s: float = 30.0,
        jpeg_quality: int = 85,
        reset_retries: int = 3,
    ):
        self.server_url = server_url.rstrip("/")
        self.timeout_s = float(timeout_s)
        self.jpeg_quality = int(jpeg_quality)
        self.reset_retries = max(1, int(reset_retries))
        self._session_id = None
        # New instructions may arrive while an old request is completing.
        # Serialize the old step and the new reset so model history is always
        # cleared after the stale computation, never before it.
        self._request_lock = threading.Lock()

    def _post(self, path: str, payload: dict):
        started = time.monotonic()
        try:
            response = requests.post(
                f"{self.server_url}{path}", json=payload,
                timeout=self.timeout_s,
            )
        except requests.RequestException as exc:
            raise BackendError(
                f"{self.name} server unreachable at "
                f"{self.server_url}{path}: {exc}"
            ) from exc
        elapsed_ms = (time.monotonic() - started) * 1000.0
        if response.status_code != 200:
            raise BackendError(
                f"{self.name} server {path} returned HTTP "
                f"{response.status_code}: {response.text[:200]}"
            )
        try:
            reply = response.json()
        except ValueError as exc:
            raise BackendError(
                f"{self.name} server {path} returned non-JSON body"
            ) from exc
        if not isinstance(reply, dict):
            raise BackendError(f"{self.name} server {path} returned non-object JSON")
        return reply, elapsed_ms

    def _encode_jpeg_b64(self, rgb) -> str:
        import cv2

        ok, buffer = cv2.imencode(
            ".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            raise BackendError("failed to JPEG-encode RGB frame")
        return base64.b64encode(buffer.tobytes()).decode("ascii")

    @staticmethod
    def _encode_depth_png_b64(depth) -> str:
        import cv2
        import numpy as np

        array = np.asarray(depth)
        if array.ndim != 2:
            raise BackendError("depth frame must be a single-channel image")
        if array.dtype != np.uint16:
            if not np.all(np.isfinite(array)):
                raise BackendError("depth frame contains NaN or infinity")
            array = np.clip(array, 0, 65535).astype(np.uint16)
        ok, buffer = cv2.imencode(".png", array)
        if not ok:
            raise BackendError("failed to PNG-encode depth frame")
        return base64.b64encode(buffer.tobytes()).decode("ascii")

    def health(self) -> dict:
        try:
            response = requests.get(
                f"{self.server_url}/health", timeout=self.timeout_s
            )
            response.raise_for_status()
            reply = response.json()
        except (requests.RequestException, ValueError) as exc:
            raise BackendError(
                f"{self.name} health check failed at {self.server_url}: {exc}"
            ) from exc
        if not isinstance(reply, dict):
            raise BackendError(f"{self.name} health reply is not an object")
        return reply

    def reset(self, instruction: str) -> None:
        with self._request_lock:
            last_error = None
            for attempt in range(self.reset_retries):
                try:
                    reply, _ = self._post(
                        "/reset", {"instruction": instruction}
                    )
                    self._session_id = str(reply.get("session_id", ""))
                    if not self._session_id:
                        raise BackendError(
                            f"{self.name} /reset reply missing session_id"
                        )
                    return
                except BackendError as exc:
                    last_error = exc
                    if attempt < self.reset_retries - 1:
                        time.sleep(2.0 ** attempt)
            raise last_error

    def step(
        self,
        rgb,
        odom: Optional[OdomPose],
        *,
        depth=None,
        depth_scale_m: Optional[float] = None,
        intrinsics: Optional[CameraIntrinsics] = None,
        image_timestamp_s: Optional[float] = None,
    ) -> StepResult:
        with self._request_lock:
            return self._step_locked(
                rgb, odom, depth=depth, depth_scale_m=depth_scale_m,
                intrinsics=intrinsics,
                image_timestamp_s=image_timestamp_s,
            )

    def _step_locked(
        self,
        rgb,
        odom: Optional[OdomPose],
        *,
        depth=None,
        depth_scale_m: Optional[float] = None,
        intrinsics: Optional[CameraIntrinsics] = None,
        image_timestamp_s: Optional[float] = None,
    ) -> StepResult:
        if self._session_id is None:
            raise BackendError(f"{self.name} step() before reset()")
        if rgb is None:
            raise BackendError(f"{self.name} step() requires an RGB frame")
        if self.requires_depth and (depth is None or intrinsics is None):
            raise BackendError(
                f"{self.name} step() requires synchronized depth and camera intrinsics"
            )

        payload = {
            "session_id": self._session_id,
            "image_jpeg_b64": self._encode_jpeg_b64(rgb),
            "odom": (
                {"x": odom.x, "y": odom.y, "yaw": odom.yaw}
                if odom else None
            ),
        }
        if depth is not None:
            try:
                scale = float(depth_scale_m)
            except (TypeError, ValueError) as exc:
                raise BackendError("depth_scale_m is required with depth") from exc
            if not math.isfinite(scale) or scale <= 0.0:
                raise BackendError("depth_scale_m must be finite and positive")
            payload["depth_png_b64"] = self._encode_depth_png_b64(depth)
            payload["depth_scale_m"] = scale
        if intrinsics is not None:
            payload["camera_intrinsics"] = intrinsics.as_dict()
        if image_timestamp_s is not None:
            timestamp = float(image_timestamp_s)
            if not math.isfinite(timestamp):
                raise BackendError("image_timestamp_s must be finite")
            payload["image_timestamp_s"] = timestamp

        reply, client_ms = self._post("/step", payload)
        has_actions = "actions" in reply
        has_trajectory = "trajectory" in reply
        if has_actions == has_trajectory:
            raise BackendError(
                "/step response must contain exactly one of actions or trajectory"
            )

        actions = validate_actions(reply["actions"]) if has_actions else []
        trajectory = (
            validate_trajectory(reply["trajectory"]) if has_trajectory else []
        )
        if has_actions and not actions:
            raise BackendError("/step actions must be a non-empty list")
        timings = parse_timings(reply, client_ms)
        latency = timings.total_ms
        detail = f"latency {latency:.0f}ms" if latency is not None else ""
        return StepResult(
            actions=actions,
            trajectory=trajectory,
            done=bool(reply.get("done", False)),
            detail=detail,
            timings=timings,
            image_timestamp_s=image_timestamp_s,
        )
