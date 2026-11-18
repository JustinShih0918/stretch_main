#!/usr/bin/env python3
"""A pretend OSG run, for testing the panel's OSG tab with no robot or models.

    python3 control_panel/tools/fake_osg_run.py map|search --tag lab --target "fire extinguisher #1"

Publishes what scripts/run_robot.py publishes through the bridge -- /osg/mission
(searching -> checking -> inspecting -> verdict), /osg/scene_graph_json (objects
appearing in tools/fake_stretch.py's floor, two extinguishers indexed #1 and
#2) and /osg/inspection_view -- prints `[robot]` lines like a real run, and on
a map pass writes outputs/robot_maps/<tag>.json in the current directory.
A search pass for a tag with no map fails as the real one does.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

OBJECTS = [  # (label, x, y, z) in fake_stretch's floor
    ("chair", 2.0, 4.5, 0.45), ("table", 3.6, 1.6, 0.7), ("fire extinguisher", 6.8, 0.6, 0.35),
    ("fire extinguisher", 9.5, 6.4, 0.35), ("cup", 3.5, 1.5, 0.8), ("bottle", 11.5, 6.9, 0.9),
]
MOVED_1 = (11.8, 2.0)   # where #1 is on the search pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["map", "search"])
    ap.add_argument("--tag", required=True)
    ap.add_argument("--target", required=True)
    ap.add_argument("--steps", type=int, default=40)
    a = ap.parse_args()
    a.target = a.target.strip("'")                  # it arrives as a quoted Hydra string
    path = Path("outputs/robot_maps") / f"{a.tag}.json"
    if a.mode == "search" and not path.exists():
        print(f"FileNotFoundError: no map at {path}. Run the map pass first.", flush=True)
        sys.exit(1)
    rclpy.init()
    node = Node("fake_osg_run")
    mission = node.create_publisher(String, "/osg/mission", 20)
    graph = node.create_publisher(String, "/osg/scene_graph_json", 5)
    view = node.create_publisher(Image, "/osg/inspection_view", 2)
    time.sleep(1.5)                                  # let discovery match the gateway
    target, want = a.target, 0
    if "#" in target:
        target, k = target.rsplit("#", 1)
        target, want = target.strip(), int(k)

    def say(**payload):
        mission.publish(String(data=json.dumps(payload)))

    def objects(seen: int, moved: bool):
        out, k = [], 0
        for i, (label, x, y, z) in enumerate(OBJECTS[:seen]):
            o = {"id": 40 + i, "label": label, "xyz": [x, y, z], "p": 0.9, "n_obs": 5,
                 "target": label == target}
            if label == "fire extinguisher":
                k += 1
                if k == 1 and moved:
                    o["xyz"] = [MOVED_1[0], MOVED_1[1], z]
                o["name"] = f"{label} #{k}"
                if want:
                    o["target"] = k == want
            out.append(o)
        return out

    print(f"[robot] {a.mode} run, target={a.target!r}, map={path}", flush=True)
    say(stage="searching", target=a.target, map_mode=a.mode)
    for step in range(a.steps):
        seen = min(len(OBJECTS), 1 + step * len(OBJECTS) // max(1, a.steps // 2))
        g = {"target": target, "step": step, "state": "EXPLORE", "objects": objects(seen, a.mode == "search"),
             "instances": [{"name": "fire extinguisher #1", "index": 1, "track_id": 42, "moved": int(a.mode == "search")},
                           {"name": "fire extinguisher #2", "index": 2, "track_id": 43, "moved": 0}],
             "rooms": [{"id": 1, "label": "room", "xy": [2.0, 3.0]}, {"id": 2, "label": "room", "xy": [10.0, 3.0]}]}
        graph.publish(String(data=json.dumps(g)))
        if step == a.steps // 2 and a.mode == "search":
            print("[robot] fire extinguisher #1 moved: [6.8, 0.6] -> [11.8, 2.0] (5.06 m; track 42 -> 57)", flush=True)
        time.sleep(0.25)
    if a.mode == "map":
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({"fake": True, "target": a.target, "objects": len(OBJECTS)}))
        print(f"[robot] map saved: {path}", flush=True)
        print(f"[robot] {a.steps} steps -> outputs/fake_{a.mode}", flush=True)
        return
    where = list(MOVED_1) if want == 1 else [9.5, 6.4]
    say(stage="checking", target=a.target, where=where)
    time.sleep(1.0)
    say(stage="inspecting", target=a.target, where=where)
    img = np.zeros((240, 160, 3), np.uint8)
    img[:, :] = (60, 70, 80)
    img[40:200, 55:105] = (190, 30, 30)              # a red cylinder
    img[60:120, 60:100] = (120, 80, 40)              # rust
    view.publish(Image(height=240, width=160, encoding="rgb8", step=480, data=img.tobytes()))
    print(f"[robot] inspecting the {a.target} at {where}: 2 view(s) -> fake VLM", flush=True)
    time.sleep(1.5)
    say(stage="verdict", target=a.target, object=a.target, condition="bad", confidence=0.8,
        issues=["Rust or corrosion on cylinder body"], summary="Cylinder shows rust; needs maintenance.",
        where=where)
    print("[robot] inspection: BAD (0.80) -- Rust or corrosion on cylinder body", flush=True)
    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
