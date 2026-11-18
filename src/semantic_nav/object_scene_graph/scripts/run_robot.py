"""Run the pipeline on a real robot, over ROS 2 / Nav2.

Two runs make an experiment, differing in one field:

    python scripts/run_robot.py +experiment=stretch3_map      # explore, keep the map
    python scripts/run_robot.py +experiment=stretch3_search   # load it, find the thing

That is the protocol the whole dynamic-scene line is built around
(docs/history/DYNAMIC_SCENES.md): run 1 leaves a scene graph and its occupancy map;
run 2 starts believing it, and the object has moved. The difference on a robot
is only that the world changes because somebody moved a chair rather than
because a script re-authored a layout.

Separate from `run_eval.py` because a run here is not an evaluation: there is
no episode dataset, no ground truth, nothing to score. What it shares is
everything that matters -- `build_env`, `build_run_components`, `build_agent`
and `run_episode` are the same functions the benchmark uses, so the robot runs
the measured pipeline and not a copy of it.

Needs the bridge up first:

    bash scripts/ros2/bridge.sh                   # a robot, or
    bash scripts/ros2/check_pipeline.sh           # the fake one
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

import hydra
from omegaconf import DictConfig, OmegaConf

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from osg.core.config import register_configs  # noqa: E402

register_configs()


@hydra.main(config_path="../configs", config_name="config", version_base="1.3")
def main(cfg: DictConfig) -> None:
    # Imported late, as run_eval.py does, so `--help` does not load habitat.
    from osg.eval.episode import run_episode
    from osg.core.profiler import Profiler
    from osg.eval.record import (build_episode_record, detector_identity,
                                 episode_tag)
    from osg.eval.visualize import save_topdown
    from osg.graph.map_store import apply_map, load_map, save_map
    from osg.pipeline.components import (build_agent, build_env,
                                         build_run_components,
                                         probe_served_models)

    if str(cfg.eval.mode) != "ros2":
        raise ValueError(
            f"run_robot.py drives a robot, so eval.mode must be 'ros2' "
            f"(got {cfg.eval.mode!r}). Use +experiment=stretch3_map.")
    map_mode = str(cfg.ros2.map_mode)
    if map_mode not in ("map", "search"):
        raise ValueError(f"ros2.map_mode must be map|search, got {map_mode!r}")

    out_dir = Path(cfg.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    OmegaConf.save(cfg, out_dir / "config.yaml")

    # Everything below is written AS IT HAPPENS, not when the run ends. Two
    # runs were lost by their shell closing: no episodes.jsonl, no map, and a
    # stdout log that -- piped through tee -- had been sitting in Python's
    # 8 KB block buffer. So: line-buffered stdout, SIGTERM (kill, docker stop,
    # a closing terminal's hangup) treated as Ctrl-C so the record and the map
    # still get written, and the mapping pass checkpoints its map below.
    import signal
    import sys

    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(line_buffering=True)

    def _stop(signum, _frame):
        raise KeyboardInterrupt(f"signal {signum}")

    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, _stop)

    # Same refusal as a benchmark run: a model server that is down is not a
    # degraded run, and on a robot it is a robot driving on a dead detector.
    probe_served_models(cfg)

    env = build_env(cfg)
    # The same kind of refusal as probe_served_models, for the robot's side:
    # a run that would drive found out Nav2 was missing at its first goal --
    # `send_goal: RuntimeError: no navigate_to_pose action server` -- after
    # every model had loaded. Ask the bridge first; it costs nothing.
    info = env.transport.ping()
    if not info.get("nav_server", True):
        raise SystemExit(
            f"no {cfg.ros2.nav_action} action server on the robot. Launch Nav2 there "
            "(stretch_nav2 navigation.launch.py) and check `ros2 action info "
            f"{cfg.ros2.nav_action}` shows a server, then rerun.")
    # A robot on another DDS vendor is the failure that LOOKS like a working
    # robot: topics cross vendors, services and actions do not, so every Nav2
    # goal silently times out and the base never moves (bridge_node.robot_rmw).
    foreign = {v: n for v, n in (info.get("robot_rmw") or {}).items()
               if v != "cyclonedds" and any(name != "osg_bridge" for name in n)}
    if foreign:
        raise SystemExit(
            "the robot's nodes are not on CycloneDDS: "
            + "; ".join(f"{v}: {', '.join(sorted(set(n)))}" for v, n in foreign.items())
            + ". Actions and services do not cross DDS vendors, so Nav2 would never "
            "answer a goal. Relaunch the robot's stack (driver, camera, Nav2, SLAM) with "
            "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp and the CycloneDDS env exported in "
            "EVERY shell, then rerun.")
    # Which map Nav2 plans on. slam_toolbox = the robot maps as it goes (the
    # mapping protocol); map_server = AMCL on a map it was given, which must
    # then be localised BEFORE the run (docs/THOR.md, "Mapping with slam_toolbox").
    map_pubs = info.get("map_publishers")
    if map_pubs is not None:
        print(f"[robot] /map published by: {', '.join(map_pubs) or 'nobody (no map yet?)'}")
    components = build_run_components(cfg, env=env)
    env.attach_driver(components["pointnav"])
    # "fire extinguisher #1": one instance of a class the scene graph tells
    # apart (scene_graph.instance_labels, osg/objects/instances.py). The class
    # is the target everywhere else; the index only narrows which object.
    from osg.objects.instances import split_target

    target, instance = split_target(env.target_category())
    if instance:
        cfg.agent.target_instance = instance
    instance = int(cfg.agent.target_instance)
    mission = f"{target} #{instance}" if instance else target
    map_path = Path(cfg.ros2.map_dir) / f"{cfg.ros2.map_tag}.json"

    print(f"[robot] {map_mode} run, target={mission!r}, map={map_path}")
    profiler = Profiler()
    agent = build_agent(
        cfg, components, target,
        keyframe_dir=str(out_dir / "keyframes" / f"{cfg.ros2.map_tag}")
        if cfg.eval.save_viz else None,
        profiler=profiler,
    )

    frame = env.reset()
    episode = env.current_episode

    if map_mode == "search":
        if not map_path.exists():
            raise FileNotFoundError(
                f"no map at {map_path}. Run +experiment=stretch3_map first, "
                f"or point ros2.map_tag at one that exists.")
        n_tracks = apply_map(
            agent, load_map(map_path),
            max_log_odds=float(cfg.scene_graph.presence.reload_max_log_odds),
            initial_floor_y=float(frame.camera_position[1]) - float(cfg.agent.camera_height),
        )
        print(f"[robot] restored {n_tracks} tracks")
    if getattr(agent, "instances", None) is not None:
        for inst in agent.instances.instances():
            track = agent.object_layer.get(inst.track_id)
            where = None if track is None else _where(agent, track)
            print(f"[robot] {inst.name} at {where}")

    # The simulator's debug video, live: rotated frame, detections, the
    # two-panel view, all on /osg/* for scripts/ros2/osg.rviz. Always on for a
    # robot -- a person is watching -- and it never costs a step (debug_stream.py).
    from osg.eval.debug_stream import DebugStream

    checkpoint_every = int(cfg.ros2.map_checkpoint_steps)

    printed = {"instance_events": len(getattr(agent, "instance_events", None) or []),
               "off_plane": 0}

    def _checkpoint(step: int) -> None:
        # A robot on its side still answers every step: Isaac's Stretch clipped a
        # wheelchair at step 69, went over backwards, and the run spent its other
        # 230 steps "wedged" with the camera on the ceiling (outputs/20261113_191906).
        # The camera leaving its standing height is the tell; end the run there.
        if float(getattr(agent, "_off_plane_m", 0.0) or 0.0) > FELL_M:
            printed["off_plane"] += 1
            if printed["off_plane"] >= FELL_STEPS and getattr(env, "abort_reason", None) is None:
                env.abort(f"the robot fell over at step {step} (its camera is "
                          f"{agent._off_plane_m:.2f} m off its standing height)")
                print(f"[robot] {env.abort_reason}: ending the run", flush=True)
        else:
            printed["off_plane"] = 0
        # Instances indexed or moved since the last step ("fire extinguisher #1
        # moved ..."), as they happen.
        events = getattr(agent, "instance_events", None) or []
        for ev in events[printed["instance_events"]:]:
            if ev["event"] == "moved":
                print(f"[robot] {ev['name']} moved: {ev['from_xy']} -> {ev['to_xy']} "
                      f"({ev['distance_m']} m; track {ev['from_track']} -> {ev['to_track']})", flush=True)
            else:
                print(f"[robot] {ev['name']} {ev['event']} at {ev['xy']}"
                      + (f" ({ev['why']}; track {ev['from_track']} -> {ev['track_id']})" if ev.get("why") else ""),
                      flush=True)
        printed["instance_events"] = len(events)
        # The map so far, at the path the search pass will read. A run that dies
        # at step 150 then leaves a step-140 map instead of none; a proper end
        # overwrites it with the final one below.
        if map_mode == "map" and checkpoint_every > 0 and step % checkpoint_every == 0:
            save_map(map_path, agent, scene=str(cfg.ros2.map_tag))
            print(f"[robot] map checkpoint at step {step} -> {map_path}", flush=True)

    # The condition inspection (inspection group, off unless a preset turns it
    # on): the best views of the target are kept as the search goes by them,
    # and judged once it has stopped there.
    inspector = views = None
    if bool(cfg.inspection.enabled):
        from osg.inspection import ConditionInspector, TargetViewBuffer

        inspector = ConditionInspector(cfg.inspection, target)
        views = TargetViewBuffer(target, keep=int(cfg.inspection.keep_views),
                                 border_px=int(cfg.inspection.border_px),
                                 min_h_frac=float(cfg.inspection.min_bbox_h_frac))
        _mission(env, {"stage": "searching", "target": mission, "map_mode": map_mode})

    debug = DebugStream(env, log_path=out_dir / "stream.jsonl", on_step=_checkpoint,
                        on_detections=None if views is None else views.observe)
    looked: dict = {}

    def _stop_check(_frame) -> bool:
        """A STOP the run can still take back (`eval.attempts` > 1): is there a
        {target} where it stopped? The look the inspection takes anyway -- face
        it, a few frames, back off if too close -- is taken now, and if the
        detector names none there the stop becomes a failed attempt: the agent
        drops that track and searches on. Measured: a far-placement run stopped
        at a shelf of red-labelled binders that only the region proposer had
        called an extinguisher, found no view of one, and ended the mission
        there (outputs/20261113_191403)."""
        env.halt()
        seen = _look_at_stop(cfg, env, agent, components["detector"], views, target,
                             agent.step_count, stage="checking")
        looked.clear()
        looked.update(seen)
        if seen["chosen"] or seen["track"] is None:
            return True
        print(f"[robot] stopped at {seen['name']} at {seen['where']}, but the detector names "
              f"no {target} there: not it, searching on", flush=True)
        _mission(env, {"stage": "resumed", "target": mission, "rejected": seen["name"],
                       "where": seen["where"]})
        return False

    if inspector is not None and int(cfg.eval.attempts) > 1:
        env.stop_check = _stop_check
    try:
        outcome = run_episode(cfg, env, agent, episode, target, frame,
                              components["detector"], debug)
        if outcome.attempts_used > 1:
            print(f"[robot] {outcome.attempts_used - 1} stop(s) taken back: no {target} there",
                  flush=True)
        if inspector is not None:
            try:
                _inspect(cfg, env, agent, components["detector"], inspector, views,
                         target, outcome.steps, out_dir,
                         on_graph=lambda: debug.publish_graph(agent, target), looked=looked)
            except Exception as exc:  # noqa: BLE001 -- loud, but the map run keeps its map
                import traceback

                traceback.print_exc()
                print(f"[robot] inspection CRASHED: {exc!r}", flush=True)
                _mission(env, {"stage": "verdict", "condition": "unknown", "target": target,
                               "error": f"inspection crashed: {exc!r}"})
    except KeyboardInterrupt as exc:
        print(f"\n[robot] interrupted ({exc or 'Ctrl-C'}) -- stopping the base")
        outcome = None
    finally:
        debug.close()
        env.close()

    driver = components["pointnav"]
    print(f"[robot] {getattr(driver, 'n_goals_sent', 0)} goals posted to Nav2, "
          f"{getattr(driver, 'n_aborts', 0)} refused, {getattr(driver, 'n_escapes', 0)} wedged and backed out")

    if map_mode == "map":
        path = save_map(map_path, agent, scene=str(cfg.ros2.map_tag))
        print(f"[robot] wrote {path}")

    if outcome is None:
        return

    scorer = components["scorer"]
    verifier = components["verifier"]
    rec = build_episode_record(
        cfg=cfg, episode=episode, env=env, agent=agent, outcome=outcome,
        target=target, detector_identity=detector_identity(cfg),
        metrics=env.metrics(), profiler=profiler, scorer=scorer,
        scorer_before=(0, 0, None),
        verifier=verifier, verifier_before=(0, 0),
    )
    if getattr(agent, "instances", None) is not None:
        # Which track was which object at the end, and every index and move seen.
        rec["instances"] = agent.instances.to_dict()["instances"]
        rec["instance_events"] = list(agent.instance_events)
        rec["target_instance"] = int(cfg.agent.target_instance)
    with open(out_dir / "episodes.jsonl", "a") as f:
        f.write(json.dumps(_json_safe(rec)) + "\n")
    # Per-stage timings, as the simulator runner writes them, so
    # `scripts/analyze_latency.py` reads a robot run too.
    profiler.write_csv(str(out_dir / "timing.csv"))
    if cfg.eval.save_viz:
        save_topdown(
            str(out_dir / "viz" / f"{episode_tag(episode)}.png"),
            agent.costmap, outcome.trajectory, scene_graph=agent.scene_graph,
            title=f"{cfg.ros2.map_tag} target={target} steps={outcome.steps}",
        )
    print(f"[robot] {outcome.steps} steps, {outcome.wall_time_s}s -> {out_dir}")


MISSION_TOPIC = "/osg/mission"
# The camera this far off its standing height for this many steps in a row is a
# robot that fell over, not a head tilt (centimetres).
FELL_M = 0.5
FELL_STEPS = 2


def _mission(env, payload: dict) -> None:
    """Where the mission is (searching / inspecting / verdict / not_found), as
    JSON on /osg/mission for a display. Best effort, like the debug stream."""
    try:
        env.transport.publish_text(MISSION_TOPIC, json.dumps(_json_safe(payload)))
    except Exception as exc:  # noqa: BLE001 -- a display never ends a run
        print(f"[robot] mission status not published ({exc!r})")


def _where(agent, track) -> list:
    from osg.eval.debug_stream import DebugStream

    x, y, _ = DebugStream._ros(agent.object_layer.center_of(track))
    return [round(x, 2), round(y, 2)]


def _look_at_stop(cfg, env, agent, detector, views, target, steps, stage="inspecting") -> dict:
    """Face what the run stopped at and look at it: which one it is, and the views
    of it the inspection can use (none: the detector does not name a {target}
    there). Announces `stage` on /osg/mission: "checking" while the stop can
    still be taken back, "inspecting" once the VLM is next."""
    from osg.eval.debug_stream import DebugStream
    from osg.inspection import bearing_deg

    where = None
    track = agent.object_layer.get(agent._candidate_id) if agent._candidate_id is not None else None
    if track is not None:
        x, y, _ = DebugStream._ros(agent.object_layer.center_of(track))
        where = [round(x, 2), round(y, 2)]
    # Which one it is, when the class is told apart by instance. The registry
    # is brought up to date first: the stop is often the keyframe that settles
    # a move.
    reg = getattr(agent, "instances", None)
    name = target
    if reg is not None and reg.covers(target) and track is not None:
        agent.instance_events += reg.update(agent.object_layer, step=steps)
        name = reg.name_of(track)
    _mission(env, {"stage": stage, "target": name, "where": where,
                   "model": str(cfg.inspection.model)})
    if track is not None and bool(cfg.inspection.face_target):
        turn = bearing_deg(env.observe().T_wc, agent.object_layer.center_of(track))
        if abs(turn) >= float(cfg.inspection.face_min_deg):
            print(f"[robot] turning {turn:+.0f} deg to face the {target}", flush=True)
            env.transport.execute("turn_left" if turn > 0 else "turn_right", 0.0, abs(turn))
    for k in range(int(cfg.inspection.final_frames)):
        frame = env.observe()
        views.observe(frame.rgb, detector.detect(frame.rgb), steps + 1 + k, frame=frame)
    # With another of its class in the building, only the views of the one the
    # robot stopped at: the best view of the run can be of the other one.
    accept = None
    if reg is not None and reg.covers(target) and track is not None:
        match_m = float(cfg.scene_graph.instance_match_m)
        # Within reach of the track it stopped at, and no nearer another
        # instance's: two extinguishers can stand closer than `match_m`.
        accept = lambda xyz: reg.belongs(xyz, track.id, agent.object_layer, match_m)  # noqa: E731
    chosen = views.best(int(cfg.inspection.n_views), accept=accept)
    if not chosen and track is not None:
        # Nothing usable, and the stop overshot: at the sim's driving speed one
        # step can cover 1.5 m, so the terminal stop fired at 0.95 m with the
        # extinguisher's base below the frame and the detector unable to name it
        # (outputs/20261113_015719). Back out to the stop distance, the way the
        # robot came in, and look again.
        from osg.mapping.costmap import PLANE

        here = env.observe().camera_position[list(PLANE)]
        gap = agent.object_layer.nearest_point_dist_xy(track, here, float(cfg.agent.terminal_percentile))
        if gap is None:
            gap = float(np.linalg.norm(agent.object_layer.center_of(track)[list(PLANE)] - here))
        back = min(1.0, float(cfg.agent.terminal_stop_m) - float(gap))
        if back > 0.15:
            print(f"[robot] {gap:.2f} m is too close to see the whole {name}: backing up {back:.2f} m",
                  flush=True)
            env.transport.execute("move_backward", back, 0.0)
            for k in range(int(cfg.inspection.final_frames)):
                frame = env.observe()
                views.observe(frame.rgb, detector.detect(frame.rgb),
                              steps + 1 + int(cfg.inspection.final_frames) + k, frame=frame)
            chosen = views.best(int(cfg.inspection.n_views), accept=accept)
    return {"track": track, "track_id": None if track is None else int(track.id), "step": int(steps),
            "where": where, "name": name, "reg": reg, "chosen": chosen}


def _inspect(cfg, env, agent, detector, inspector, views, target, steps, out_dir,
             on_graph=None, looked=None) -> None:
    """The search is over. If it ended AT the target, judge the target's condition.

    The views are the best the approach collected (osg/inspection/views.py)
    plus a last look from the stop pose (`_look_at_stop`, already taken when the
    run checked the stop: `looked`); the verdict, the views it was given and a
    card of both go into the run directory and onto /osg/*.
    """
    import cv2

    from osg.inspection import render_card

    if getattr(getattr(agent, "state", None), "name", "") != "DONE":
        want = int(cfg.agent.target_instance)
        why = getattr(env, "abort_reason", None)
        _mission(env, {"stage": "not_found", "target": f"{target} #{want}" if want else target,
                       "reason": why})
        print(f"[robot] inspection skipped: {why or f'the run did not stop at a {target}'}")
        return
    if (looked and looked.get("track_id") is not None
            and looked.get("track_id") == agent._candidate_id and looked.get("step") == agent.step_count):
        seen = looked
        _mission(env, {"stage": "inspecting", "target": seen["name"], "where": seen["where"],
                       "model": str(cfg.inspection.model)})
    else:
        seen = _look_at_stop(cfg, env, agent, detector, views, target, agent.step_count)
    track, where, name, reg, chosen = (seen["track"], seen["where"], seen["name"], seen["reg"],
                                       seen["chosen"])
    want = int(cfg.agent.target_instance)
    if (chosen and reg is not None and want and reg.covers(target) and track is not None
            and reg.index_of(track.id) is None):
        # The robot was sent for #k, #k is gone from where it was, and the detector
        # has now named an extinguisher here that no instance explains (the track
        # it came to may be the region proposer's, which the registry's own rules
        # never bind): this is #k, moved.
        event = reg.claim(target, want, track.id, agent.object_layer, step=steps,
                          why="named by the detector at the stop")
        if event is not None:
            agent.instance_events.append(event)
            name = reg.name_of(track)
            print(f"[robot] {event['name']} moved: {event['from_xy']} -> {event['to_xy']} "
                  f"({event['distance_m']} m; confirmed at the stop)", flush=True)
            # The display announced "fire extinguisher ?": show it the move, and
            # which one is being inspected.
            if on_graph is not None:
                on_graph()
            _mission(env, {"stage": "inspecting", "target": name, "where": where,
                           "model": str(cfg.inspection.model)})
    if want and reg is not None and reg.covers(target) and track is not None:
        if reg.index_of(track.id) is None and reg.same_as(track, target, want, agent.object_layer):
            # A second track of the extinguisher #k was just found as (the region
            # proposer's): it is #k. Say so to the display, which was told "?".
            name = reg.get(target, want).name
            _mission(env, {"stage": "inspecting", "target": name, "where": where,
                           "model": str(cfg.inspection.model)})
        elif reg.index_of(track.id) != want:
            print(f"[robot] WARNING: stopped at {name}, the mission was {target} #{want}", flush=True)
            if reg.index_of(track.id) is not None:
                # Another instance, known as such: its verdict is not this mission's
                # answer, and reporting it as one is the mistake the indices exist to
                # prevent (#2 judged GOOD for the #1 mission, outputs/20261113_200152).
                why = f"stopped at {name}, not {target} #{want}: no verdict for the mission"
                _mission(env, {"stage": "not_found", "target": f"{target} #{want}", "reason": why})
                print(f"[robot] inspection skipped: {why}", flush=True)
                return
    images = inspector.images(chosen)
    for i, im in enumerate(images):
        cv2.imwrite(str(out_dir / f"inspection_view{i}.jpg"), np.ascontiguousarray(im[..., ::-1]))
    if images:
        try:
            env.transport.publish_image("/osg/inspection_view", images[0], "rgb8")
        except Exception:  # noqa: BLE001 -- display only
            pass
    print(f"[robot] inspecting the {name} at {where}: {len(chosen)} view(s) from steps "
          f"{[v.step for v in chosen]} ({views.n_seen} frames named it) -> {cfg.inspection.model}",
          flush=True)
    report = inspector.inspect(chosen)
    record = dict(report.to_dict(), where=where, n_target_frames=views.n_seen, object=name)
    with open(out_dir / "inspection.json", "w") as f:
        json.dump(_json_safe(record), f, indent=2)
    card = render_card(report, images[0] if images else None)
    cv2.imwrite(str(out_dir / "inspection_card.jpg"), card)
    try:
        env.transport.publish_image("/osg/inspection", card, "bgr8")
    except Exception:  # noqa: BLE001 -- display only
        pass
    _mission(env, dict(record, stage="verdict", reasoning=report.reasoning[-1200:]))
    if report.error:
        print(f"[robot] inspection FAILED: {report.error}", flush=True)
    else:
        print(f"[robot] inspection: {report.condition.upper()} ({report.confidence:.2f}) "
              f"in {report.latency_s}s -- {'; '.join(report.issues) or 'no defects'} "
              f"-- {report.summary}", flush=True)


def _json_safe(value):
    """Non-finite floats -> null, so the record is JSON anything can read.

    A robot run has no ground truth, so `success`/`spl`/`distance_to_goal`
    arrive as NaN -- honestly "not measured" in memory, but `json.dumps` writes
    a bare `NaN` token that only Python accepts back.
    """
    import math

    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    return value


if __name__ == "__main__":
    main()
