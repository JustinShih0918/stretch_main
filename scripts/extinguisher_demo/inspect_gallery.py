#!/usr/bin/env python3
"""Damage gallery: the same VLM inspection, one extinguisher defect at a time.

The recorded takes show a whole mission per extinguisher. This shows the inspection
step alone across many kinds of damage, quickly: with the robot parked where a
take stopped it (facing the extinguisher), each condition is switched in live
(open_extinguisher.py's control file without a token: no reset, the robot stays put),
and the robot's camera is run through the same steps a mission ends with --
YOLOE with the mission's vocabulary, `TargetViewBuffer` to choose the views,
`ConditionInspector` to ask the VLM. Nothing is staged for the model beyond the
extinguisher itself; what it is shown is what a mission stopped there would show it.

    # after a take, with the robot still parked at the extinguisher:
    source ~/.nvidia_api.env
    cd src/semantic_nav/object_scene_graph
    PYTHONPATH=src /opt/conda/envs/habitat/bin/python \\
        ../../../scripts/extinguisher_demo/inspect_gallery.py corroded hose_detached discharged \\
        fallen blocked --bridge 127.0.0.1:18765 --target-xy=-7.5,5.12 \\
        --out ../../../videos/extinguisher_demo/gallery

With Isaac and the model servers resident there may be no GPU memory left for a
second YOLOE: add `--set detector.device=cpu --set detector.half=false`.

Per condition, into <out>/<condition>/: `inspection.json` (the report, as a
mission writes it), `inspection_view*.jpg` (exactly what the VLM saw),
`robot_frame.jpg` (the camera frame with YOLOE's boxes) and `viewport.jpg`
(Isaac's third-person view). `--no-vlm` stops before the VLM call, to check
framing and detection. compile_demo.py --gallery cuts these into the video.

Run from OSG's root (Hydra's config path) in the habitat env, like run_robot.py.
"""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

OSG = Path(__file__).resolve().parents[2] / "src" / "semantic_nav" / "object_scene_graph"
sys.path.insert(0, str(OSG / "src"))


def grab_viewport(display: str, crop: str, path: Path) -> None:
    size, _, xy = crop.partition("+")
    subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "x11grab",
                    "-draw_mouse", "0", "-video_size", size, "-i", f"{display}+{xy}",
                    "-frames:v", "1", "-q:v", "2", str(path)], check=True)


def stand_off(env, target_xy, dist_m: float, frame: str, height_m: float = 0.33) -> None:
    """Park the camera `dist_m` from the target, on the line it is on now, facing it.

    Where a mission stops depends on how it approached, and the gallery wants
    every condition shot from one pose a mission could stop at. The head camera
    is 1.3 m up and looks level: much closer than 1.5 m the extinguisher's stand
    leaves the bottom of the frame, and from 1.9 m the extinguisher is too small
    for the VLM to see a missing hose. Nav2 places the base, not the camera, so the offset
    between them is learnt from the first goal and the second one corrects it.
    """
    from osg.inspection import bearing_deg

    hx, hy = target_xy

    def camera_xy():
        T = env.observe().T_wc
        return float(T[0, 3]), float(-T[2, 3])                   # pipeline world -> ROS map

    offset = 0.0                                                 # base behind the camera, m
    for _ in range(2):
        cx, cy = camera_xy()
        d = float(np.hypot(cx - hx, cy - hy))
        if abs(d - dist_m) <= 0.15:
            break
        vx, vy = (cx - hx) / d, (cy - hy) / d
        goal_d = dist_m + offset
        gx, gy = hx + goal_d * vx, hy + goal_d * vy
        print(f"[gallery] camera {d:.2f} m from the target; Nav2 to ({gx:.2f}, {gy:.2f})",
              flush=True)
        env.transport.send_goal(gx, gy, float(np.arctan2(-vy, -vx)), frame)
        t0, state = time.time(), "?"
        while time.time() - t0 < 90:
            time.sleep(1.0)
            state = str(env.transport.nav_status().get("state"))
            if state in ("succeeded", "aborted", "canceled", "rejected"):
                break
        print(f"[gallery] Nav2: {state}", flush=True)
        cx, cy = camera_xy()
        offset = goal_d - float(np.hypot(cx - hx, cy - hy))
    turn = bearing_deg(env.observe().T_wc, (hx, height_m, -hy))
    if abs(turn) >= 3.0:
        env.transport.execute("turn_left" if turn > 0 else "turn_right", 0.0, abs(turn))
    T = env.observe().T_wc
    print(f"[gallery] camera at {np.hypot(T[0, 3] - hx, -T[2, 3] - hy):.2f} m, "
          f"{bearing_deg(T, (hx, height_m, -hy)):+.1f} deg off the target", flush=True)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("conditions", nargs="+")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--bridge", default=os.environ.get("BRIDGE_ADDR", "127.0.0.1:18765"))
    ap.add_argument("--control", type=Path,
                    default=Path(os.environ.get("EXTINGUISHER_CONTROL",
                                                Path.home() / "sim_nav_ws/extinguisher/condition")))
    ap.add_argument("--settle", type=float, default=4.0,
                    help="seconds after a switch before the first frame (render + camera latency)")
    ap.add_argument("--frames", type=int, default=6, help="camera frames per condition")
    ap.add_argument("--restore", default="good", help="condition to leave the extinguisher in")
    ap.add_argument("--display", default=os.environ.get("ISAAC_DISPLAY", ":5"))
    ap.add_argument("--viewport-crop", default=os.environ.get("VIEWPORT_CROP", "1735x976+116,66"))
    ap.add_argument("--no-vlm", action="store_true", help="detect and save views, skip the VLM")
    ap.add_argument("--target-xy", default="-7.5,4.3",
                    help="the extinguisher in the ROS map, x,y (pass as --target-xy=-7.5,4.3)")
    ap.add_argument("--stand-off", type=float, default=1.5,
                    help="camera-to-extinguisher metres to shoot from (the extinguisher, its "
                         "stand and the floor in front in frame); 0 = stay where the robot is")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="extra Hydra override, e.g. detector.device=cpu when the GPU is full")
    args = ap.parse_args()

    from hydra import compose, initialize_config_dir

    from osg.core.config import register_configs
    from osg.eval.visualize import overlay_segmentation
    from osg.inspection import ConditionInspector, TargetViewBuffer
    from osg.perception.vocabulary import target_vocabulary
    from osg.pipeline.components import build_detector
    from osg.sim.ros2_env import Ros2Env

    register_configs()
    with initialize_config_dir(config_dir=str(OSG / "configs"), version_base="1.3"):
        cfg = compose(config_name="config", overrides=[
            "+experiment=isaac_extinguisher_inspect", f"ros2.bridge_addr={args.bridge}", *args.set])
    icfg = cfg.inspection
    if not args.no_vlm and str(icfg.api_key) in ("", "missing"):
        sys.exit("NVIDIA_API_KEY is not set (source ~/.nvidia_api.env)")
    target = str(cfg.ros2.target)
    detector = build_detector(cfg)
    detector.set_vocabulary(target_vocabulary(target, cfg.detector.vocabulary))
    inspector = None if args.no_vlm else ConditionInspector(icfg, target)
    env = Ros2Env(cfg)
    env.reset()
    if args.stand_off > 0:
        stand_off(env, tuple(float(v) for v in args.target_xy.split(",")), args.stand_off,
                  str(cfg.ros2.goal_frame))
    args.out.mkdir(parents=True, exist_ok=True)
    summary = []
    try:
        for condition in args.conditions:
            args.control.write_text(condition + "\n")
            time.sleep(args.settle)
            out = args.out / condition
            out.mkdir(parents=True, exist_ok=True)
            views = TargetViewBuffer(target, keep=int(icfg.keep_views),
                                     border_px=int(icfg.border_px),
                                     min_h_frac=float(icfg.min_bbox_h_frac))
            frame = dets = None
            for k in range(args.frames):
                frame = env.observe()
                dets = detector.detect(frame.rgb)
                views.observe(frame.rgb, dets, k + 1)
            cv2.imwrite(str(out / "robot_frame.jpg"), overlay_segmentation(frame.rgb, dets, target))
            grab_viewport(args.display, args.viewport_crop, out / "viewport.jpg")
            chosen = views.best(int(icfg.n_views))
            images = (inspector or ConditionInspector(icfg, target, client=object())).images(chosen)
            for old in out.glob("inspection_view*.jpg"):
                old.unlink()
            for i, im in enumerate(images):
                cv2.imwrite(str(out / f"inspection_view{i}.jpg"), np.ascontiguousarray(im[..., ::-1]))
            line = (f"[gallery] {condition}: {target} in {views.n_seen}/{args.frames} frames, "
                    f"{len(chosen)} view(s), box h={chosen[0].h_frac:.2f}" if chosen else
                    f"[gallery] {condition}: no usable view of the {target} "
                    f"({views.n_seen}/{args.frames} frames named it)")
            print(line, flush=True)
            if inspector is None:
                continue
            report = inspector.inspect(chosen)
            record = dict(report.to_dict(), n_target_frames=views.n_seen, gallery_condition=condition)
            (out / "inspection.json").write_text(json.dumps(record, indent=2))
            verdict = (f"FAILED: {report.error}" if report.error else
                       f"{report.condition.upper()} ({report.confidence:.2f}) in {report.latency_s}s"
                       f"{' [fallback]' if report.fallback else ''} -- "
                       f"{'; '.join(report.issues) or 'no defects'} -- {report.summary}")
            print(f"[gallery] {condition}: {verdict}", flush=True)
            summary.append({"condition": condition, "verdict": report.condition,
                            "confidence": report.confidence, "error": report.error})
    finally:
        args.control.write_text(args.restore + "\n")
        env.transport.close()
    if summary:
        (args.out / "summary.json").write_text(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
