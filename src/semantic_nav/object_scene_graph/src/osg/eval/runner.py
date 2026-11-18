"""The run: build the stack once, drive every episode, summarise.

This file is deliberately thin. Everything it calls is a module named after what
it does -- what the run is MADE of is `pipeline/components.py`, what one episode
DOES is `episode.py`, what gets written down is `record.py` -- so the shape of a
run is legible here in one screen and no mechanism has to be understood to read
it.
"""
from __future__ import annotations

import json
import dataclasses
from pathlib import Path

from ..core.config import resolve_navigation
from ..core.profiler import Profiler
from ..pipeline.components import (
    build_agent,
    build_env,
    build_run_components,
    unload_ollama_models,
)
from ..pipeline.components import build_scorer
from .debug_video import DebugVideo
from .episode import run_episode
from .metrics import (
    aggregate,
    dynamic_summary,
    per_category,
)
from .prior_map import save_map_for_scene
from .record import (
    build_episode_record,
    detector_identity,
    episode_tag,
)
from .visualize import save_topdown


def _episode_uid(episode) -> str:
    """Return a scene-qualified key because Habitat episode ids repeat."""
    return f"{str(episode.scene_id).split('/')[-1]}:{episode.episode_id}"


def _algorithm_fingerprint(cfg) -> dict:
    """Serialize every behavior-bearing typed setting for paired A/B audits."""
    from ..core.config import AgentConfig, ExplorationConfig, VerificationConfig
    from omegaconf import OmegaConf

    def plain(value):
        if OmegaConf.is_config(value):
            return OmegaConf.to_container(value, resolve=True)
        if isinstance(value, tuple):
            return [plain(item) for item in value]
        if isinstance(value, list):
            return [plain(item) for item in value]
        if isinstance(value, dict):
            return {str(key): plain(item) for key, item in value.items()}
        return value

    out = {}
    for schema, node, prefix in (
        (ExplorationConfig, cfg.exploration, ""),
        (VerificationConfig, cfg.verification, "verify_"),
        (AgentConfig, cfg.agent, ""),
    ):
        for field in dataclasses.fields(schema):
            value = getattr(node, field.name)
            out[prefix + field.name] = plain(value)
    out.update({
        "detector_weights": str(cfg.detector.weights),
        "detector_imgsz": int(cfg.detector.imgsz),
        "room_classifier": str(cfg.scene_graph.room_classifier),
        "room_erode_iters": int(cfg.scene_graph.room_erode_iters),
        "fp_retraction": bool(cfg.scene_graph.fp_retraction),
    })
    return out


def run_eval(cfg) -> dict:
    out_dir = Path(cfg.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    unload_ollama_models(cfg)
    from ..pipeline.components import probe_served_models

    probe_served_models(cfg)
    env = build_env(cfg)
    components = build_run_components(cfg, env=env)
    algorithm_holder = {
        "algorithm": {
            # Explicit field inventory: the values are populated by the typed
            # serializer, while these names make additions reviewable in diffs.
            # "affinity_cache" "affinity_grounded" "affinity_llm"
            # "approach_abandon_steps" "approach_false_arrival_m"
            # "approach_navigable_goal" "approach_retarget_m"
            # "approach_close_clearance_m" "approach_close_source"
            # "approach_retarget_max" "approach_scan_turns"
            # "approach_stop_at_best_m" "approach_to_viewpoint"
            # "value_strict"
            # "ram_url" "ram_tags"
            # "close_look_before_absence" "close_look_by_belief"
            # "close_look_face_turns" "close_look_hold_steps"
            # "close_look_max_per_episode" "close_look_max_steps"
            # "close_look_min_belief" "close_look_opportunistic"
            # "close_look_ring_m" "close_look_trigger_range_m"
            # "continuity_weight" "detector_imgsz" "detector_weights"
            # "escape_window" "extractor"
            # "fp_retraction"
            # "frontier_commit" "frontier_cost_free_cell" "frontier_desc"
            # "frontier_desc_match_m" "frontier_goal_free_cell"
            # "frontier_min_cells" "frontier_reachability_gate"
            # "frontier_stick_rule"
            # "info_gain_weight" "knowledge_prior" "knowledge_weight"
            # "los_visibility_penalty" "navigation"
            # "nearby_distance_m" "pointnav_approach_creep_m" "pointnav_arrival_m"
            # "pointnav_depth_shape" "pointnav_stop_means_blocked"
            # "pointnav_creep_stall_eps" "pointnav_creep_stall_steps"
            # "pointnav_stop_radius" "pointnav_weights" "policy" "ranker"
            # "ranker_every_steps" "ranker_topk" "reachable_via_viewpoint"
            # "reselect_every" "room_classifier" "room_erode_iters"
            # "search_arrival_m" "search_detect_prob"
            # "search_drop_proximity_after_absence" "search_face_turns"
            # "explore_after_failed_attempt_steps"
            # "profile_sync"
            # "search_frontier_weight" "search_glance_detect_prob"
            # "search_glance_floor" "search_glance_range_m" "search_max_steps"
            # "search_posterior" "search_proximity_floor" "search_proximity_len_m"
            # "search_room_saturation" "search_room_saturation_floor"
            # "search_room_saturation_free" "search_same_room_bonus"
            # "search_surface_mass" "search_unreached_credit"
            # "value_blip2_timeout_s" "value_blip2_url"
            # "verify_api_key" "verify_base_url"
            # "verify_viewpoint_min_clearance_m"
            # "verify_seen_counts_within_m"
            # "giveup_after_steps" "giveup_min_progress_m"
            # region_proposal.*: enabled, weights, backbone, tau, admit_score,
            # pad_frac, min_area_px, max_area_frac, max_regions,
            # only_when_unnamed, max_per_episode, require_never_named,
            # unnamed_keyframes, use_for_absence, every_keyframe, commits,
            # commit_min_obs, commit_tau
            # "select_every"
            # "selector"
            # "terminal_percentile" "terminal_requires_detection"
            # "terminal_rule" "terminal_stop_m" "unreachable_restrike_m"
            # "use_habitat_navmesh" "value_argmax" "value_map" "value_model"
            # "value_weight" "verify_abandon_below_p" "verify_absence_categories_max"
            # "verify_absence_max_range_m" "verify_absence_on_arrival"
            # "verify_absence_only" "verify_absence_requires_expectation"
            # "verify_absence_use_vlm" "verify_accept_confidence"
            # "verify_approach_recheck" "verify_approach_recheck_thresh"
            # "verify_center_before_verify" "verify_choice_mode"
            # "verify_detector_absence_recall" "verify_min_bbox_px"
            # "verify_min_obs" "verify_min_score" "verify_rank_candidates_by_presence"
            # "verify_commit_max_above_storey_m" "verify_commit_high_min_obs"
            # "verify_ring_radius_extent_aware"
            # "verify_retire_stale_twins_after_absence" "verify_stop_at_stale_anchor_once"
            # "verify_stale_stop_at_nearest_free" "verify_failed_attempt_disables_place"
            # "verify_relook_after_stale_stop"
            # "reachable_via_nearest_free" "navmesh_snap_on_agent_island"
            # "approach_close_last_metre_m" "approach_arrival_m"
            # "verify_target_bypasses_bbox_gate"
            # "verify_terminal" "verify_unreachable_is_absorbing" "verify_vlm_q"
            # "verify_vlm_recall" "viewpoint_stop_m" "voronoi_goal_near_m"
            **_algorithm_fingerprint(cfg),
        },
    }
    detector = components["detector"]
    scorer = components["scorer"]
    verifier = components["verifier"]
    identity = detector_identity(cfg)
    if verifier is not None and cfg.eval.debug_frames:
        verifier.debug_dir = str(out_dir / "verify_debug")

    n_total = len(env.env.episodes)
    n_run = n_total if cfg.eval.num_episodes < 0 else min(cfg.eval.num_episodes, n_total)
    wanted = set(cfg.eval.episode_ids) if cfg.eval.episode_ids else None
    if wanted is not None:
        n_run = len(wanted) if cfg.eval.num_episodes < 0 else min(
            int(cfg.eval.num_episodes), len(wanted)
        )

    results: list = []
    episodes_file = out_dir / "episodes.jsonl"
    profiler_all = Profiler(sync=bool(getattr(cfg.eval, "profile_sync", False)))

    for _dataset_i in range(n_total):
        if len(results) >= n_run:
            break
        frame = env.reset()
        episode = env.current_episode
        uid = _episode_uid(episode)
        if (
            wanted is not None
            and uid not in wanted
            and str(episode.episode_id) not in wanted
        ):
            continue
        ep_i = len(results)
        target = env.target_category()
        ep_tag = episode_tag(episode)
        if verifier is not None:
            verifier.debug_tag = ep_tag

        # The scorer and verifier are built once and shared, so their counters
        # are cumulative; the record reports deltas against these.
        scorer_before = (scorer.n_calls, scorer.n_errors, scorer.last_error)
        verifier_before = (
            (verifier.n_calls, verifier.n_errors) if verifier is not None else (0, 0)
        )
        # frontier.id/room.id restart from 0/1 each episode (a fresh extractor
        # and segmenter per NavAgent), but the scorer's caches are keyed by
        # those same small ints and persist across the run -- without this, a
        # new episode can inherit a stale score from an unrelated scene the
        # moment an id collides.
        scorer.reset()

        profiler = Profiler(sync=bool(getattr(cfg.eval, "profile_sync", False)))
        agent = build_agent(
            cfg, components, target,
            keyframe_dir=str(out_dir / "keyframes" / ep_tag) if cfg.eval.save_viz else None,
            profiler=profiler,
            nav_fn=(env.action_to_goal if components["navigation"] == "navmesh" else None),
            reachable_fn=(env.is_reachable if components["navigation"] == "navmesh" else None),
            # NOT gated on the mover. Its only consumer is the closing walk
            # (`approach.py:_close_last_metre`), which is switched by
            # `agent.approach_close_last_metre_m` and is 0 by default. Gating it
            # on navmesh mode made the flag silently dead under pointnav -- an
            # arm built to price what the closing walk is worth ran bit-identical
            # to the arm without it, which is how this was found.
            nearest_navigable_fn=getattr(env, "nearest_navigable_xy", None),
        )
        debug = DebugVideo(cfg, out_dir, ep_tag) if cfg.eval.debug_frames else None
        outcome = run_episode(cfg, env, agent, episode, target, frame, detector, debug)
        if debug is not None:
            debug.close()

        save_map_for_scene(cfg, agent, episode)
        rec = build_episode_record(
            cfg=cfg, episode=episode, env=env, agent=agent, outcome=outcome,
            target=target, detector_identity=identity, metrics=env.metrics(),
            profiler=profiler, scorer=scorer, scorer_before=scorer_before,
            verifier=verifier, verifier_before=verifier_before,
        )
        results.append(rec)
        with open(episodes_file, "a") as f:
            f.write(json.dumps(rec) + "\n")
        for name, samples in profiler._samples.items():
            for sample in samples:
                profiler_all.add(name, sample)

        if cfg.eval.save_viz:
            save_topdown(
                str(out_dir / "viz" / f"{ep_tag}.png"),
                agent.costmap,
                outcome.trajectory,
                scene_graph=agent.scene_graph,
                title=f"ep {episode.episode_id} target={target} "
                      f"success={rec['success']:.0f} spl={rec['spl']:.2f}",
            )
        print(
            f"[{ep_i + 1}/{n_run}] ep={episode.episode_id} target={target} "
            f"success={rec['success']:.0f} spl={rec['spl']:.3f} "
            f"steps={outcome.steps} fps={rec['control_fps']}"
        )

    summary = _summarise(
        cfg, env, results, identity, profiler_all,
        algorithm=algorithm_holder["algorithm"],
    )
    with open(out_dir / "summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    profiler_all.write_csv(str(out_dir / "timing.csv"))
    scorer.shutdown()
    env.close()
    print(json.dumps(summary["metrics"], indent=2))
    return summary


def _summarise(cfg, env, results, identity, profiler_all, algorithm=None) -> dict:
    """`dynamic` is the block that answers the dynamic-scene questions: how long
    the map took to stop believing a moved object, how often the agent committed
    to a goal it had already disproved, and how often a ghost survived."""
    summary = {
        "config": {
            "eval_mode": str(cfg.eval.mode),
            "scorer": cfg.exploration.frontier_text_scorer,
            "verification": cfg.verification.enabled,
            "detector": identity,
            "dataset_version": cfg.eval.dataset_version,
            "split": cfg.eval.split,
            "success_distance": cfg.agent.success_distance,
            "navigation": resolve_navigation(cfg.agent),
            "policy": str(getattr(cfg.agent, "policy", "nav_agent")),
        },
        "algorithm": algorithm or _algorithm_fingerprint(cfg),
        "metrics": aggregate(results),
        "dynamic": dynamic_summary(results),
        "per_category": per_category(results),
        "timing": profiler_all.report(),
        "pipeline_fps": round(profiler_all.fps("control_loop"), 2),
    }
    benchmark_metadata = getattr(env, "benchmark_metadata", None)
    if callable(benchmark_metadata):
        summary["benchmark"] = benchmark_metadata()
    return summary
