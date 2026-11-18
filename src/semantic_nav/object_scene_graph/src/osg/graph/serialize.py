"""Scene graph serialization: compact hierarchy text for LLM prompts and
JSON for logging / ScanNet-style graph evaluation (milestone M7).
"""
from __future__ import annotations

from collections import Counter, defaultdict
from typing import Dict, List

import numpy as np

from .scene_graph import ObjectNodeView, SceneGraph

NEAR_DIST_M = 1.5


def _near_pairs(objs: List[ObjectNodeView]) -> Dict[int, List[str]]:
    near = defaultdict(list)
    for i in range(len(objs)):
        for j in range(len(objs)):
            if i == j:
                continue
            d = np.linalg.norm(objs[i].center - objs[j].center)
            if d < NEAR_DIST_M:
                near[i].append(objs[j].label)
    return near


def to_prompt_text(sg: SceneGraph, max_near: int = 2) -> str:
    """e.g. ``Room 2 (bedroom): bed x1, lamp x2 (near bed), wardrobe x1``"""
    return _rooms_text(sg, sorted(sg.rooms.keys()), max_near)


def _rooms_text(sg: SceneGraph, room_ids, max_near: int) -> str:
    lines = []
    for rid in room_ids:
        room = sg.rooms[rid]
        objs = sg.objects_in_room(rid)
        if not objs:
            continue
        near = _near_pairs(objs)
        by_label: Dict[str, List[int]] = defaultdict(list)
        for idx, o in enumerate(objs):
            by_label[o.label].append(idx)
        parts = []
        for label, idxs in sorted(by_label.items()):
            near_labels = Counter()
            for idx in idxs:
                near_labels.update(near.get(idx, []))
            near_labels.pop(label, None)
            suffix = ""
            if near_labels:
                top = ", ".join(l for l, _ in near_labels.most_common(max_near))
                suffix = f" (near {top})"
            parts.append(f"{label} x{len(idxs)}{suffix}")
        name = f" ({room.label})" if room.label else ""
        lines.append(f"Room {rid}{name}: " + ", ".join(parts))

    # Room 0 is "unassigned", not a real room, so it is never in room_ids and
    # has to be appended explicitly.
    unassigned = sg.objects_in_room(0)
    if unassigned:
        counts = Counter(o.label for o in unassigned)
        lines.append("Hallway/other: "
                     + ", ".join(f"{l} x{n}" for l, n in sorted(counts.items())))
    if not lines:
        return "(no objects mapped yet)"
    return "\n".join(lines)


def to_json(sg: SceneGraph) -> dict:
    return {
        "rooms": [
            {
                "id": r.id,
                "label": r.label,
                "centroid_xy": r.centroid_xy.tolist(),
                "n_cells": r.n_cells,
                "container_ids": sorted(r.container_ids),
            }
            for r in sg.rooms.values()
        ],
        "containers": [
            {
                "id": c.id,
                "label": c.label,
                "track_ids": sorted(c.track_ids),
                "center": c.center.tolist(),
                "top_h": round(c.top_h, 3),
                "area_m2": round(c.area_m2, 4),
                "room_id": c.room_id,
                "object_ids": sorted(c.object_ids),
            }
            for c in sorted(sg.containers.values(), key=lambda c: c.id)
        ],
        "objects": [
            {
                "track_id": o.track_id,
                "label": o.label,
                "center": o.center.tolist(),
                "room_id": o.room_id,
                "n_obs": o.n_obs,
                "container_id": o.container_id,
                "p_rel": None if o.p_rel is None else [round(v, 3) for v in o.p_rel],
            }
            for o in sg.objects
        ],
    }
