"""The best views of the target the search collected on its way in.

Where the run STOPs is the worst place to judge an object from. The Stretch's
head camera looks level from 1.3 m, so at the approach's stop distance the
base of a floor-standing object is below the frame: the picture the robot has
when it is done can be a picture of half the object. The approach itself passed through every
distance on the way -- so keep the views as they go by, and judge from those.

A view is ranked by how much of the object it shows and how well:

  a box touching the image edge is a partial view (the object is cut off),
  and counts for a fraction of a whole one;
  a bigger box shows more detail, up to `full_h_frac` of the image height,
  past which the object is no longer getting more legible, only closer;
  the detector's score breaks ties, since a confident naming is usually a
  clear view.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from ..core.labels import normalize_label


# eq=False: views are compared by identity. The generated __eq__ would compare
# the frames, and `view in buffer` on two numpy arrays raises.
@dataclass(eq=False)
class TargetView:
    rgb: np.ndarray            # (H, W, 3) uint8, the frame the pipeline consumed
    bbox_xyxy: np.ndarray      # (4,) float, the target's box on it
    score: float
    step: int
    truncated: bool
    quality: float
    # Where the object in the box is, in world coordinates, when the frame's
    # depth could place it. With two of the target's class in a run, this is
    # what tells the views of the one inspected from the views of the other.
    xyz: Optional[np.ndarray] = None

    @property
    def h_frac(self) -> float:
        return float(self.bbox_xyxy[3] - self.bbox_xyxy[1]) / float(self.rgb.shape[0])

    def crop(self, pad: float, min_aspect: float = 0.0) -> np.ndarray:
        """The box plus `pad` of its own size on every side, clipped to the frame.

        A tall, narrow object (an extinguisher) gives a sliver of a crop that
        leaves out the floor beside it; `min_aspect` widens the crop about the
        box's centre until it is at least that many times as wide as it is tall.
        """
        h, w = self.rgb.shape[:2]
        x1, y1, x2, y2 = (float(v) for v in self.bbox_xyxy)
        px, py = pad * (x2 - x1), pad * (y2 - y1)
        x1, y1, x2, y2 = x1 - px, y1 - py, x2 + px, y2 + py
        short = min_aspect * (y2 - y1) - (x2 - x1)
        if short > 0:
            x1, x2 = x1 - short / 2, x2 + short / 2
        x1, y1 = max(0, int(x1)), max(0, int(y1))
        x2, y2 = min(w, int(np.ceil(x2))), min(h, int(np.ceil(y2)))
        return np.ascontiguousarray(self.rgb[y1:y2, x1:x2])


class TargetViewBuffer:
    """Keeps the `keep` best views of `target` across a run.

    `observe` is fed every frame's detections (DebugStream's `on_detections`
    hook, which already runs the detector each step for the live view), so
    collecting costs nothing the run was not already paying.
    """

    TRUNCATED_WEIGHT = 0.3

    def __init__(self, target: str, keep: int = 6, border_px: int = 6,
                 min_h_frac: float = 0.12, full_h_frac: float = 0.55) -> None:
        self.target = normalize_label(target)
        self.keep = int(keep)
        self.border_px = int(border_px)
        self.min_h_frac = float(min_h_frac)
        self.full_h_frac = float(full_h_frac)
        self.views: List[TargetView] = []
        self.n_seen = 0          # frames the target was named on, kept or not

    # With a frame, views are kept per place: a cell this wide (m) on the
    # ground. Two extinguishers two metres apart must not compete for the same
    # `keep` slots, or the one seen closer on the way in evicts the other.
    PLACE_M = 0.75

    def observe(self, rgb: np.ndarray, detections, step: int, frame=None) -> Optional[TargetView]:
        """Consider one frame. Returns the best view kept from it, if any.

        Without `frame`, the frame's best detection of the target is one view.
        With it, every detection of the target is a view placed in the world
        (`xyz`), and up to `keep` are kept per place, so `best(near=...)` can
        choose the views of one particular object."""
        dets = [d for d in detections
                if normalize_label(d.label) == self.target and getattr(d, "source", "detector") == "detector"]
        if not dets:
            return None
        self.n_seen += 1
        if frame is None:
            dets = [max(dets, key=lambda d: float(d.score))]
        kept = []
        for det in dets:
            view = self._view(rgb, det, step)
            if view.h_frac < self.min_h_frac:
                continue
            if frame is not None:
                from ..objects.instances import detection_xyz

                view.xyz = detection_xyz(det, frame)
            self.views.append(view)
            kept.append(view)
        self.views.sort(key=lambda v: (v.quality, v.step), reverse=True)
        per_place: dict = {}
        survivors = []
        for v in self.views:
            key = self._place(v)
            if per_place.get(key, 0) < self.keep:
                per_place[key] = per_place.get(key, 0) + 1
                survivors.append(v)
        self.views = survivors
        kept = [v for v in kept if v in self.views]
        return max(kept, key=lambda v: v.quality) if kept else None

    def best(self, n: int = 1, near=None, max_m: float = 1.0, accept=None) -> List[TargetView]:
        """The `n` best views; with `near` (a world point), only views placed
        within `max_m` of it on the ground; with `accept` (a predicate on a
        world point), only views it accepts -- the inspection passes "this is
        the instance the robot stopped at, not the one beside it". When no view
        could be placed at all (no depth), both are ignored rather than leaving
        nothing."""
        views = self.views
        if (near is not None or accept is not None) and any(v.xyz is not None for v in views):
            from ..mapping.costmap import PLANE

            views = [v for v in views if v.xyz is not None]
            if near is not None:
                p = np.asarray(near, dtype=float)[list(PLANE)]
                views = [v for v in views
                         if float(np.linalg.norm(np.asarray(v.xyz)[list(PLANE)] - p)) <= float(max_m)]
            if accept is not None:
                views = [v for v in views if accept(v.xyz)]
        return list(views[: max(0, int(n))])

    def _place(self, view: TargetView):
        if view.xyz is None:
            return None
        from ..mapping.costmap import PLANE

        xy = np.asarray(view.xyz, dtype=float)[list(PLANE)]
        return tuple(np.floor(xy / self.PLACE_M).astype(int).tolist())

    def _view(self, rgb: np.ndarray, det, step: int) -> TargetView:
        h, w = rgb.shape[:2]
        box = np.asarray(det.bbox_xyxy, dtype=float)
        b = self.border_px
        truncated = bool(box[0] <= b or box[1] <= b or box[2] >= w - b or box[3] >= h - b)
        h_frac = float(box[3] - box[1]) / float(h)
        size = min(h_frac, self.full_h_frac) / self.full_h_frac
        quality = size * float(det.score) * (self.TRUNCATED_WEIGHT if truncated else 1.0)
        return TargetView(rgb=np.array(rgb, dtype=np.uint8, copy=True), bbox_xyxy=box,
                          score=float(det.score), step=int(step), truncated=truncated,
                          quality=float(quality))


def bearing_deg(T_wc: np.ndarray, target_xyz) -> float:
    """How far to turn the base, in degrees, for the camera to face `target_xyz`.

    Both in the pipeline's world (x, y-up, z); the camera's optical axis is
    T_wc's z column (OpenCV). Measured in ROS's ground plane (x, -z), so
    positive is counter-clockwise -- a left turn -- as `turn_left` takes it.
    """
    T = np.asarray(T_wc, dtype=float)
    fx, fy = T[0, 2], -T[2, 2]
    d = np.asarray(target_xyz, dtype=float) - T[:3, 3]
    dx, dy = d[0], -d[2]
    return float(np.degrees(np.arctan2(fx * dy - fy * dx, fx * dx + fy * dy)))
