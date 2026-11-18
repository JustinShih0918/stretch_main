"""The agent's map: one occupancy grid and everything sized to it.

`room_labels` is kept beside its costmap because the two must share a shape --
a segmentation taken before the grid grew is re-run rather than indexed. The
planner and the optional semantic value map are built per map for the same
reason: both hold state in the costmap's grid frame.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .costmap import Costmap2D
from .room_seg import VoronoiRoomSegmenter


@dataclass
class MapLayer:
    costmap: Costmap2D
    segmenter: VoronoiRoomSegmenter
    room_labels: Optional[np.ndarray] = None
    # World height of the ground plane the costmap bands points against.
    # Latched from the first frame (camera height minus `agent.camera_height`),
    # or restored from a prior-map snapshot so both passes band alike.
    floor_y: Optional[float] = None
    planner: object = None
    value_map: object = None

    @classmethod
    def build(
        cls,
        resolution_m: float = 0.05,
        room_seg_kwargs: Optional[dict] = None,
        planner_factory=None,
        value_map_factory=None,
    ) -> "MapLayer":
        costmap = Costmap2D(resolution=resolution_m)
        return cls(
            costmap=costmap,
            segmenter=VoronoiRoomSegmenter(**dict(room_seg_kwargs or {})),
            planner=planner_factory() if planner_factory is not None else None,
            value_map=(
                value_map_factory(costmap) if value_map_factory is not None else None
            ),
        )
