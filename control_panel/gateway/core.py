"""The gateway's logic that needs no ROS: map images, teleop safety, poses.

Kept apart from `ros_io.py` so it is unit-tested without a ROS graph
(control_panel/tests/test_core.py).
"""
from __future__ import annotations

import math
import struct
import time
import zlib
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np


# ------------------------------------------------------------------ images
def encode_png(img: np.ndarray) -> bytes:
    """8-bit grayscale (H, W), RGB (H, W, 3) or RGBA (H, W, 4) -> PNG bytes. No PIL:
    the bridge image the gateway runs in does not carry it."""
    img = np.ascontiguousarray(img, dtype=np.uint8)
    if img.ndim == 2:
        h, w = img.shape
        color_type, channels = 0, 1
    elif img.ndim == 3 and img.shape[2] in (3, 4):
        h, w, channels = img.shape
        color_type = 2 if channels == 3 else 6
    else:
        raise ValueError(f"need (H, W), (H, W, 3) or (H, W, 4), got {img.shape}")
    rows = img.reshape(h, w * channels)
    raw = np.hstack([np.zeros((h, 1), np.uint8), rows]).tobytes()  # filter 0 per row

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack("!I", len(data)) + tag + data + struct.pack("!I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    ihdr = struct.pack("!IIBBBBB", w, h, 8, color_type, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def occupancy_to_rgba(data, width: int, height: int) -> np.ndarray:
    """nav_msgs/OccupancyGrid data -> RGBA image, row 0 = the map's TOP (max y).

    OccupancyGrid row 0 is the bottom (y = origin.y), so the rows are flipped.
    Unknown (-1) is transparent so the page's own background shows through;
    free is white, occupied dark, and anything in between shaded."""
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)[::-1]
    rgba = np.zeros((height, width, 4), np.uint8)
    known = grid >= 0
    occ = np.clip(grid, 0, 100).astype(np.float32) / 100.0
    shade = (245 - occ * 215).astype(np.uint8)  # 245 free .. 30 occupied
    rgba[..., 0] = rgba[..., 1] = rgba[..., 2] = shade
    rgba[..., 3] = np.where(known, 255, 0).astype(np.uint8)
    return rgba


# ------------------------------------------------------------------- poses
def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    """(x, y, z, w) for a rotation of `yaw` about z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ------------------------------------------------------------------ teleop
@dataclass
class TeleopGuard:
    """What the gateway may put on cmd_vel, given what the page last asked.

    A phone is an unreliable joystick: the page can freeze, Wi-Fi can drop,
    the tab can be backgrounded. So a command only stands while it is FRESH
    (`hold_s`); the page re-sends while a finger is down, and anything older is
    a stop. After the stop has gone out once the guard goes quiet, so it does
    not fight Nav2 for the base when nobody is driving. The driver has its own
    0.5 s cmd_vel timeout behind this one (stretch_driver `timeout`).

    Speeds are clamped here, not trusted from the page.
    """

    max_linear: float = 0.3     # m/s
    max_angular: float = 1.0    # rad/s
    hold_s: float = 0.4
    _vx: float = 0.0
    _wz: float = 0.0
    _stamp: float = field(default=-1e9)
    _sent_stop: bool = True

    def command(self, vx: float, wz: float, now: Optional[float] = None) -> None:
        now = time.monotonic() if now is None else now
        vx = 0.0 if not math.isfinite(vx) else vx
        wz = 0.0 if not math.isfinite(wz) else wz
        self._vx = max(-self.max_linear, min(self.max_linear, float(vx)))
        self._wz = max(-self.max_angular, min(self.max_angular, float(wz)))
        self._stamp = now
        self._sent_stop = False

    def release(self) -> None:
        """The page let go (or closed): stop at the next tick."""
        self._stamp = -1e9

    @property
    def active(self) -> bool:
        return not self._sent_stop

    def tick(self, now: Optional[float] = None) -> Optional[Tuple[float, float]]:
        """The twist to publish now, or None to publish nothing."""
        now = time.monotonic() if now is None else now
        if now - self._stamp <= self.hold_s:
            return self._vx, self._wz
        if not self._sent_stop:
            self._sent_stop = True
            return 0.0, 0.0
        return None


# -------------------------------------------------------------------- head
# The Stretch 3 head's joint ranges (rad), a little inside the driver's own
# so a step at the end stops there instead of being rejected.
HEAD_PAN = (-3.9, 1.5)      # + turns the camera to the robot's left
HEAD_TILT = (-1.5, 0.5)     # + looks up
HEAD_PRESETS = {"forward": (0.0, 0.0), "floor": (0.0, -0.8)}


def head_target(current: Tuple[Optional[float], Optional[float]], d_pan: float = 0.0, d_tilt: float = 0.0,
                preset: Optional[str] = None) -> Tuple[float, float]:
    """Where to send the head: a preset, or the current pose plus a step,
    clamped to the joint ranges. An unknown current pose counts as 0."""
    if preset is not None:
        if preset not in HEAD_PRESETS:
            raise ValueError(f"unknown head preset {preset!r}")
        pan, tilt = HEAD_PRESETS[preset]
    else:
        pan, tilt = (current[0] or 0.0) + d_pan, (current[1] or 0.0) + d_tilt
    return (min(max(pan, HEAD_PAN[0]), HEAD_PAN[1]), min(max(tilt, HEAD_TILT[0]), HEAD_TILT[1]))


def clean_name(name: str) -> str:
    """A map name safe as a file name on the robot: letters, digits, - and _."""
    out = "".join(c if (c.isalnum() or c in "-_") else "_" for c in str(name).strip())
    return out.strip("_")[:64]
