"""2D occupancy costmap built from depth frames.

World plane axes are (x, z) with y as height (habitat convention, y-up).
Grid values: -1 unknown, 0 free, 100 occupied. The grid auto-grows.
"""
from __future__ import annotations

from typing import Callable, List, Optional

import numpy as np

from ..core.geometry import backproject
from ..core.types import FrameData

UNKNOWN, FREE, OCCUPIED = -1, 0, 100
PLANE = (0, 2)  # world axes forming the ground plane
HEIGHT_AXIS = 1


class Costmap2D:
    def __init__(self, resolution: float = 0.05, size_m: float = 20.0) -> None:
        self.resolution = resolution
        n = int(size_m / resolution)
        self.grid = np.full((n, n), UNKNOWN, dtype=np.int8)
        self.origin = np.array([-size_m / 2.0, -size_m / 2.0])  # world xy of grid[0, 0]
        self._grow_listeners: List[Callable[[int, int, int, int], None]] = []

    def add_grow_listener(self, fn: Callable[[int, int, int, int], None]) -> None:
        self._grow_listeners.append(fn)

    # ------------------------------------------------------------- transforms

    def world_to_grid(self, xy: np.ndarray) -> np.ndarray:
        return np.floor((np.asarray(xy) - self.origin) / self.resolution).astype(int)

    def grid_to_world(self, rc: np.ndarray) -> np.ndarray:
        return self.origin + (np.asarray(rc, dtype=float) + 0.5) * self.resolution

    def in_bounds(self, rc: np.ndarray) -> bool:
        return 0 <= rc[0] < self.grid.shape[0] and 0 <= rc[1] < self.grid.shape[1]

    def clear_footprint(self, xy: np.ndarray, radius_m: float) -> int:
        """The robot is standing here, so these cells are free. Returns how
        many cells changed.

        What Nav2's costmaps do as a matter of course and the simulator never
        needed: its agent sees the floor at its feet, a head camera at 1.3 m
        does not, and a glossy floor or the robot's own mast in a tilted view
        puts obstacle speckle exactly where the base is. Measured on the
        Stretch (outputs/20260922_232853): 23 of 101 poses on cells the map
        called OCCUPIED, 57 on free islands under 50 cells, 98 free
        components -- and wavefront frontier detection, which grows from the
        robot's own component, found nothing from a 9-cell island.
        """
        r = int(np.ceil(float(radius_m) / self.resolution))
        if r <= 0:
            return 0
        rc = self.world_to_grid(xy)
        if not self.in_bounds(rc):
            return 0
        h, w = self.grid.shape
        r0, r1 = max(0, rc[0] - r), min(h, rc[0] + r + 1)
        c0, c1 = max(0, rc[1] - r), min(w, rc[1] + r + 1)
        rr, cc = np.ogrid[r0:r1, c0:c1]
        disc = (rr - rc[0]) ** 2 + (cc - rc[1]) ** 2 <= r * r
        block = self.grid[r0:r1, c0:c1]
        changed = int((disc & (block != FREE)).sum())
        block[disc] = FREE
        return changed

    def ensure_contains(self, xy: np.ndarray, margin_m: float = 2.0) -> None:
        rc = self.world_to_grid(xy)
        m = int(margin_m / self.resolution)
        h, w = self.grid.shape
        if 0 + m <= rc[0] < h - m and 0 + m <= rc[1] < w - m:
            return
        # Double the grid, keeping content centered
        new = np.full((h * 2, w * 2), UNKNOWN, dtype=np.int8)
        new[h // 2 : h // 2 + h, w // 2 : w // 2 + w] = self.grid
        self.grid = new
        self.origin = self.origin - np.array([h // 2, w // 2]) * self.resolution
        for listener in self._grow_listeners:
            listener(h, w, h // 2, w // 2)
        self.ensure_contains(xy, margin_m)

    # ---------------------------------------------------------------- update

    def update(
        self,
        frame: FrameData,
        floor_y: float,
        obstacle_low: float = 0.2,
        obstacle_high: float = 1.5,
        max_range: float = 5.0,
        stride: int = 4,
        pts: Optional[np.ndarray] = None,
    ) -> None:
        """Fold one frame into the grid, banding points by height above
        `floor_y`, the ground plane. Points outside the band are dropped.

        `pts` accepts an already-backprojected (N, 3) world point cloud, for a
        caller that has one; the backprojection dominates mapping cost.
        """
        cam_xy = frame.camera_position[list(PLANE)]
        self.ensure_contains(cam_xy, margin_m=max_range + 1.0)

        if pts is None:
            pts = backproject(
                frame.depth, frame.intrinsics, frame.T_wc, stride=stride, max_depth=max_range
            )
        if pts.shape[0] == 0:
            return
        rel_h = pts[:, HEIGHT_AXIS] - floor_y
        xy = pts[:, list(PLANE)]

        floor_mask = (rel_h > -0.3) & (rel_h < obstacle_low)
        obst_mask = (rel_h >= obstacle_low) & (rel_h < obstacle_high)

        cam_rc = self.world_to_grid(cam_xy)
        self._raycast_batch(cam_rc, xy[floor_mask], xy[obst_mask])
        # The agent's own cell is free by construction
        if self.in_bounds(cam_rc):
            self.grid[cam_rc[0], cam_rc[1]] = FREE

    def _raycast_batch(
        self, cam_rc: np.ndarray, floor_xy: np.ndarray, obst_xy: np.ndarray
    ) -> None:
        """Vectorized raycasting: all rays sampled as a (M, N) grid of cells.
        Intermediate unknown/free cells become FREE; rays stop at the first
        already-occupied cell (never carve walls); obstacle endpoints are
        stamped OCCUPIED last. The per-point python Bresenham this replaces
        dominated the control loop (~1.6 s/frame)."""
        ends = []
        occ_flags = []
        for pts, occ in ((floor_xy, False), (obst_xy, True)):
            if pts.shape[0] == 0:
                continue
            rc = np.floor((pts - self.origin) / self.resolution).astype(np.int64)
            rc, idx = np.unique(rc, axis=0, return_index=True)
            ends.append(rc)
            occ_flags.append(np.full(rc.shape[0], occ))
        if not ends:
            return
        end_rc = np.concatenate(ends)  # (N, 2)
        end_occ = np.concatenate(occ_flags)  # (N,)
        h, w = self.grid.shape
        inb = (end_rc[:, 0] >= 0) & (end_rc[:, 0] < h) & (end_rc[:, 1] >= 0) & (end_rc[:, 1] < w)
        end_rc, end_occ = end_rc[inb], end_occ[inb]
        if end_rc.shape[0] == 0:
            return

        delta = end_rc - cam_rc[None, :]
        n_steps = int(np.abs(delta).max()) + 1
        # 2x supersampling closes diagonal gaps a true Bresenham would fill
        m = min(2 * n_steps + 1, 4096)
        t = np.linspace(0.0, 1.0, m)[:, None, None]
        samples = np.rint(cam_rc[None, None, :] + t * delta[None, :, :]).astype(np.int64)
        rr = np.clip(samples[..., 0], 0, h - 1)
        cc = np.clip(samples[..., 1], 0, w - 1)

        vals = self.grid[rr, cc]  # (M, N)
        blocked = vals == OCCUPIED
        any_blocked = blocked.any(axis=0)
        first_block = np.where(any_blocked, blocked.argmax(axis=0), m)  # (N,)
        step_idx = np.arange(m)[:, None]
        visible = step_idx < first_block[None, :]  # strictly before the wall

        self.grid[rr[visible], cc[visible]] = FREE
        # Obstacle endpoints whose ray was not blocked earlier
        reached = ~any_blocked | (first_block >= m - 1)
        stamp = end_occ & reached
        self.grid[end_rc[stamp, 0], end_rc[stamp, 1]] = OCCUPIED

    # ------------------------------------------------------------------ views

    def inflated(self, radius_m: float) -> np.ndarray:
        """Boolean obstacle map dilated by radius (for planning)."""
        from scipy import ndimage

        obst = self.grid == OCCUPIED
        r = max(1, int(round(radius_m / self.resolution)))
        struct = _disk(r)
        return ndimage.binary_dilation(obst, structure=struct)

    def free_mask(self) -> np.ndarray:
        return self.grid == FREE

    def coverage_cells(self) -> int:
        return int((self.grid != UNKNOWN).sum())


def grow_aligned(
    arr: np.ndarray, old_h: int, old_w: int, off_r: int, off_c: int, fill=0
) -> np.ndarray:
    """Grow a grid-aligned side array after its costmap doubles in size."""
    new = np.full((old_h * 2, old_w * 2), fill, dtype=arr.dtype)
    new[off_r : off_r + old_h, off_c : off_c + old_w] = arr
    return new


def _disk(r: int) -> np.ndarray:
    y, x = np.ogrid[-r : r + 1, -r : r + 1]
    return x * x + y * y <= r * r


def nearest_free_xy(costmap: "Costmap2D", xy: np.ndarray) -> np.ndarray:
    """Nearest FREE cell to a (possibly occupied) world point -- the closest
    pose the agent can actually stand at.

    Used wherever a goal is derived from an OBJECT rather than from free space:
    a tabletop object's centre is an occupied cell inside the furniture, and
    driving to it strands the follower against the desk. Measured over 42
    episodes, the 10 that ended on such a goal scored SR 0.100 against 0.516 for
    the rest.
    """
    rc = costmap.world_to_grid(xy)
    h, w = costmap.grid.shape
    rad = int(1.5 / costmap.resolution)
    r0, r1 = max(0, rc[0] - rad), min(h, rc[0] + rad + 1)
    c0, c1 = max(0, rc[1] - rad), min(w, rc[1] + rad + 1)
    free = np.argwhere(costmap.grid[r0:r1, c0:c1] == FREE)
    if free.shape[0] == 0:
        return xy
    free_world = costmap.grid_to_world(free + np.array([r0, c0]))
    d = np.linalg.norm(free_world - xy, axis=1)
    return free_world[int(np.argmin(d))]


def cell_status(costmap: "Costmap2D", xy: np.ndarray) -> str:
    """Costmap classification of a world point: free/occupied/unknown/oob."""
    rc = costmap.world_to_grid(xy)
    h, w = costmap.grid.shape
    if not (0 <= rc[0] < h and 0 <= rc[1] < w):
        return "oob"
    v = costmap.grid[rc[0], rc[1]]
    return {FREE: "free", OCCUPIED: "occupied", UNKNOWN: "unknown"}.get(int(v), str(int(v)))
