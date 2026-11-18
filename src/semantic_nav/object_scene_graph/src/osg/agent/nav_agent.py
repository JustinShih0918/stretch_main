"""NavAgent: the control loop behind a single act(frame) -> action call.

What one step does, in order:

    costmap.update        depth -> occupancy, banded above the latched floor height
    on_keyframe           detector -> object layer -> presence beliefs, and a
                          glance at every surface in plain view
    candidates.check      does any mapped track now clear the gates and become
                          a goal (presence, identity, evidence, reachability)
    dispatch              the handler for whichever state the agent is in

The state machine and the two literal actions live in `state.py`. Everything a
state does lives beside it:

    exploration/strategy.py   where to go next -- frontiers and mapped surfaces
                              competing under one index
    agent/candidate.py        what becomes a goal, and the pre-approach verify
    agent/approach.py         the terminal walk and the stop decision
    verification/absence.py   "I got there and it was not there", as evidence

What is left here is the wiring, the FSM state itself, and the few things every
state needs: the costmap, a plan, a path to follow, and a detection of the
target in the current frame. NavAgent is the only owner of FSM state -- `state`,
`_goal_xy`, `_current_path`, `_candidate_id`, `_target_obj_xy`, the deadline --
which is why the handlers read and write it through their `nav` back-reference
rather than each keeping a copy.
"""
from __future__ import annotations

from typing import List, Optional

import numpy as np

from ..core.profiler import Profiler
from ..core.config import resolve_navigation
from ..core.types import Detection, FrameData
from ..exploration.async_scorer import AsyncScorer
from ..exploration.ascent_selector import FrontierCommitState
from ..exploration.selector import frontier_goal_xy
from ..exploration.strategy import ExplorationStrategy, WorldView
from ..graph.scene_graph import SceneGraph
from ..mapping.costmap import PLANE, Costmap2D
from ..mapping.map_layer import MapLayer
from ..objects.instances import InstanceRegistry
from ..objects.object_layer import ObjectLayer
from ..perception.detector import Detector
from ..perception.vocabulary import target_vocabulary
from ..pipeline.beliefs import build_affinity_prior, build_presence_filter
from ..perception.keyframe import KeyframeSelector, KeyframeStore
from ..planning.controller import WaypointController, agent_heading
from ..planning.escape import ActionHistoryEscape
from ..planning.planner import PlanResult, StraightLinePlanner
from ..planning.voronoi_planner import HybridVoronoiPlanner
from ..verification.absence import AbsenceSensor, AbsenceVerdict
from ..verification.viewpoint import ViewpointPlanner
from .approach import ApproachPolicy
from .candidate import CandidatePolicy
from .close_look import CloseLookPolicy
from ..core.labels import normalize_label
from ..graph.containers import CONTAINER_CATEGORIES
from .state import STOP_ACTION, TURN_ACTION, State

def _horizontal_radius_m(track) -> float:
    """The track's ground-plane extent, from its fitted ellipsoid.

    The semi-axes are in the ellipsoid's own frame, so the honest ground-plane
    radius is the largest of them: a box seen edge-on has its long axis
    horizontal whichever way R points, and over-estimating the radius pushes the
    viewpoint ring outwards, which is the safe direction -- a ring that is too
    big costs a little distance, one that is too small lands in an occupied cell
    and loses the viewpoint entirely.
    """
    ellipsoid = getattr(track, "ellipsoid", None)
    axes = getattr(ellipsoid, "axes", None)
    if axes is None:
        return 0.0
    try:
        radius = float(np.max(np.abs(np.asarray(axes, dtype=float))))
    except (TypeError, ValueError):
        return 0.0
    return radius if np.isfinite(radius) else 0.0


class NavAgent:
    def __init__(
        self,
        cfg,
        detector: Detector,
        scorer: AsyncScorer,
        verifier,  # always None in old-algorithm mode (no VLM verifier); kept for call-site compat
        target_category: str,
        keyframe_dir: Optional[str] = None,
        profiler: Optional[Profiler] = None,
        nav_fn=None,
        reachable_fn=None,
        nearest_navigable_fn=None,
        pointnav=None,
        ranker=None,
        room_classifier=None,
        image_text=None,
        feature_memory=None,
        region_proposer=None,
        gate_itm=None,
        ram=None,
    ) -> None:
        self.cfg = cfg
        self.detector = detector
        self.scorer = scorer
        self.verifier = verifier
        # Habitat-navmesh driving (old-stack alignment): nav_fn(goal_xy) returns
        # the next discrete action toward goal_xy on Habitat's navmesh, or None
        # when arrived/unreachable. When set, it replaces the costmap planner +
        # controller for all goal-following. The costmap is still built (for
        # frontier extraction / scene graph), only navigation switches.
        self._nav_fn = nav_fn
        self._reachable_fn = reachable_fn
        # nearest_navigable_fn(xy, None) -> the navmesh point nearest to xy on
        # the agent's island, for closing the last metre of an approach.
        self._nearest_navigable_fn = nearest_navigable_fn
        self.navigation = resolve_navigation(cfg.agent)
        self._use_navmesh = nav_fn is not None and self.navigation == "navmesh"
        if pointnav is None and self.navigation == "pointnav":
            from ..planning.pointnav_driver import build_pointnav

            pointnav = build_pointnav(cfg)
        self.pointnav = pointnav
        self._driver = nav_fn if self._use_navmesh else self.pointnav
        self._direct_approach = self._driver is not None
        self.ranker = ranker
        self.room_classifier = room_classifier
        self.image_text = image_text
        self.gate_itm = gate_itm
        # RAM++ tags are an ASCENT prompt input; accepted so the construction
        # contract holds, unused here.
        self.ram = ram
        ascent_selector = str(getattr(cfg.exploration, "selector", "utility")) == "ascent"
        self.commit_state = (
            FrontierCommitState(quantise_m=cfg.exploration.frontier_dedup_m)
            if ascent_selector or bool(getattr(cfg.exploration, "frontier_commit", False))
            else None
        )
        self._approach_recheck = image_text is not None and bool(
            getattr(cfg.verification, "approach_recheck", False)
        )
        self._approach_recheck_thresh = float(
            getattr(cfg.verification, "approach_recheck_thresh", 0.0)
        )
        self._frontier_desc = str(getattr(cfg.exploration, "frontier_desc", "graph"))
        self.frontier_semantics = None
        if self._frontier_desc in ("frame", "frame_objects"):
            from ..exploration.frontier_semantics import FrontierSemantics

            self.frontier_semantics = FrontierSemantics(
                match_radius_m=float(
                    getattr(cfg.exploration, "frontier_desc_match_m", 1.0)
                ),
                fov_rad=np.radians(float(getattr(cfg.eval, "hfov_deg", 79.0))),
                max_range_m=float(cfg.mapping.max_range_m),
            )
        # Terminal-view verification mode: skip the pre-approach best_crop VLM
        # call and instead verify the live close-up frame at the STOP decision
        # (see _do_approach). Requires a verifier; no-op when verifier is None.
        self._terminal_verify = (
            verifier is not None and bool(cfg.verification.terminal)
        )
        self.profiler = profiler or Profiler()
        # Appearance matching (objects/feature_memory.py), or None. Kept apart
        # from `image_text`: any non-None value THERE turns the semantic value
        # map on for every frame, which is a different mechanism entirely.
        self.feature_memory = feature_memory
        # Class-agnostic proposals for the objects the detector never names
        # (perception/region_proposer.py). None unless region_proposal.enabled.
        self.region_proposer = region_proposer
        self._region_admits = 0
        self._region_cache = None
        self._region_kf = 0          # keyframes seen this episode
        self._region_named = False   # has the LABEL path ever named the target
        # One entry per appearance commit: which surface, which track, how
        # much it looked like the query, and how many it beat.
        self.feature_pick_log: list = []
        # Debug hook: if set, called with (frame, dets) every keyframe right
        # after the detections that feed object_layer.update() are computed
        # -- lets diagnostics observe exactly what the scene graph is built
        # from without duplicating the keyframe-timing logic. None by default
        # (zero cost, never called).
        self.on_keyframe_detections = None

        # Counters the whole stack writes into; the handlers share the dict, so
        # reset() clears it in place rather than rebinding it.
        self.stats: dict = {}
        # The map: one costmap, its room segmentation, a planner and an
        # optional semantic value map, all in one grid frame. Rebuilt per
        # episode by reset().
        self._value_map_factory = None
        if image_text is not None:
            from ..mapping.value_map import ValueMap2D

            self._value_map_factory = lambda costmap: ValueMap2D(
                costmap, max_depth_m=cfg.mapping.max_range_m
            )
        self.map = self._new_map()
        self.selection_planner = (
            StraightLinePlanner()
            if self._driver is not None
            and not bool(cfg.agent.frontier_reachability_gate)
            else self.planner
        )
        self.object_layer = ObjectLayer(
            feature_memory=feature_memory,
            assoc_score_thresh=cfg.scene_graph.assoc_score_thresh,
            assoc_depth_gate_m=cfg.scene_graph.assoc_depth_gate_m,
            assoc_category_gate=cfg.scene_graph.assoc_category_gate,
            min_obs_for_refine=cfg.scene_graph.min_obs_for_refine,
            refine_every=cfg.scene_graph.refine_every,
            refine_max_center_move_m=cfg.scene_graph.refine_max_center_move_m,
            link_dist_m=cfg.scene_graph.link_dist_m,
            link_max_frame_gap=cfg.scene_graph.link_max_frame_gap,
            min_det_score=cfg.scene_graph.min_det_score,
            min_det_bbox_px=cfg.scene_graph.min_det_bbox_px,
            confirm_baseline_m=cfg.scene_graph.confirm_baseline_m,
            repeat_view_discount=cfg.scene_graph.repeat_view_discount,
            presence_filter=build_presence_filter(cfg),
            target_bypasses_gates=cfg.scene_graph.target_bypasses_gates,
            max_range_m=cfg.mapping.max_range_m,
            fp_disable_radius_m=cfg.scene_graph.fp_disable_radius_m,
            cloud_stride=cfg.scene_graph.cloud_stride,
            cloud_cap=cfg.scene_graph.cloud_cap,
            min_valid_depth_frac=cfg.scene_graph.min_valid_depth_frac,
        )
        # Objects of these classes are told apart one by one (objects/instances.py),
        # so two of them seen side by side must not be linked into one.
        self.object_layer.unlinked_labels = {
            normalize_label(l) for l in (cfg.scene_graph.instance_labels or [])}
        self.object_layer.instance_assoc_gate_m = float(cfg.scene_graph.instance_assoc_gate_m)
        self.scene_graph = SceneGraph(
            container_top_h_m=tuple(cfg.scene_graph.container_top_h_m),
            container_min_area_m2=cfg.scene_graph.container_min_area_m2,
            container_support_tol_m=cfg.scene_graph.container_support_tol_m,
            container_min_obs=cfg.scene_graph.container_min_obs,
            container_min_score=cfg.scene_graph.container_min_score,
            container_merge_m=cfg.scene_graph.container_merge_m,
            containers_floor_relative=bool(getattr(
                cfg.scene_graph, "containers_floor_relative", False)),
            container_merge_sigma=float(
                getattr(cfg.scene_graph, "container_merge_sigma", 0.0) or 0.0
            ),
        )
        self.keyframes = KeyframeStore(save_dir=keyframe_dir)
        self.kf_selector = KeyframeSelector(
            cfg.scene_graph.keyframe_trans_m, cfg.scene_graph.keyframe_rot_deg
        )
        self.controller = WaypointController(forward_m=cfg.agent.forward_m)
        self.viewpoint_planner = ViewpointPlanner(
            list(cfg.verification.ring_radii_m),
            min_clearance_m=float(getattr(cfg.verification, "viewpoint_min_clearance_m", 0.0) or 0.0),
        )
        # "I walked there and it was not there" as evidence, from two sensors
        # with different error rates. See verification/absence.py.
        self.absence = AbsenceSensor(cfg, verifier, self.profiler, self.stats)
        # The two state handlers. Each owns what only it uses; NavAgent stays
        # the single owner of FSM state.
        self.approach = ApproachPolicy(self)
        self.candidates = CandidatePolicy(self)
        self.close_look = CloseLookPolicy(self)
        # Where to go next -- frontiers and mapped surfaces under one index.
        # It owns everything an exploration round remembers; see
        # exploration/strategy.py.
        self.exploration = ExplorationStrategy(
            cfg, self.selection_planner, scorer, self.viewpoint_planner,
            build_affinity_prior(cfg), self.stats, self.profiler,
        )

        self.reset(target_category)

    # ------------------------------------------------------------------- map

    def _new_map(self) -> MapLayer:
        cfg = self.cfg
        return MapLayer.build(
            resolution_m=cfg.mapping.resolution_m,
            room_seg_kwargs=dict(
                min_room_radius_m=cfg.scene_graph.room_min_radius_m,
                door_width_m=cfg.scene_graph.room_door_width_m,
            ),
            planner_factory=lambda: HybridVoronoiPlanner(
                collision_m=cfg.agent.agent_radius + cfg.mapping.inflate_margin_m,
                goal_near_m=cfg.exploration.voronoi_goal_near_m,
                inflate_radius_m=cfg.agent.agent_radius + cfg.mapping.inflate_margin_m,
            ),
            value_map_factory=self._value_map_factory,
        )

    @property
    def costmap(self) -> Costmap2D:
        return self.map.costmap

    @property
    def planner(self) -> HybridVoronoiPlanner:
        return self.map.planner

    @property
    def _room_labels(self):
        return self.map.room_labels

    @_room_labels.setter
    def _room_labels(self, labels) -> None:
        self.map.room_labels = labels

    # Per-episode exploration telemetry, recorded by eval/runner.py.
    @property
    def search_log_events(self) -> list:
        return self.exploration.search_log_events

    @property
    def frontier_select_log(self) -> list:
        return self.exploration.frontier_select_log

    @property
    def giveup_log(self) -> list:
        return self.exploration.giveup_log

    @property
    def _current_frontier(self):  # eval/runner.py's debug video draws it
        return self.exploration.current_frontier

    @_current_frontier.setter
    def _current_frontier(self, frontier) -> None:
        self.exploration.current_frontier = frontier

    @property
    def _frontier_reach_m(self) -> float:
        base = float(self.exploration.frontier_reach_m)
        return max(base, float(getattr(self.pointnav, "stop_radius", 0.0)))

    # Per-episode approach telemetry, recorded by eval/runner.py.
    @property
    def approach_bbox_log(self) -> list:
        return self.approach.bbox_log

    @property
    def approach_stop_reason(self):
        return self.approach.stop_reason

    @property
    def approach_diag(self) -> dict:
        return self.approach.diag

    @property
    def approach_retarget_log(self) -> list:
        return self.approach.retarget_log

    @property
    def goal_commit_log(self) -> list:
        return self.candidates.goal_commit_log

    @property
    def candidate_reject_log(self) -> list:
        return self.candidates.reject_log

    @property
    def close_look_log(self) -> list:
        return self.close_look.log

    @property
    def glance_ranges(self) -> dict:
        return {str(k): round(float(v), 3) for k, v in self.exploration.glance_ranges.items()}

    # ------------------------------------------------------------------ reset

    def reset(self, target_category: str) -> None:
        self.target = target_category
        self.state = State.INIT
        self.step_count = 0
        self._scan_steps_left = (
            int(round(360.0 / self.cfg.agent.turn_deg)) if self.cfg.agent.initial_scan else 0
        )
        self.map = self._new_map()
        # A fresh map carries a fresh planner. Keep the ordinary selection path
        # attached to it; the straight-line selector is intentionally independent.
        if not isinstance(self.selection_planner, StraightLinePlanner):
            self.selection_planner = self.planner
        self._kf_count = 0
        self._current_path: Optional[np.ndarray] = None
        self._candidate_id: Optional[int] = None
        self._goal_xy: Optional[np.ndarray] = None
        self._last_action: Optional[str] = None
        self._goto_deadline = 10**9
        self._target_obj_xy: Optional[np.ndarray] = None
        self._agent_xy: Optional[np.ndarray] = None
        self._target_cloud_xy: Optional[np.ndarray] = None
        self._room_votes: list = []
        self.stats.clear()
        self.stats.update({"plan_ok": 0, "plan_fail": 0, "select_none": 0, "select_ok": 0})
        # Phase 2 instrumentation (docs/history/DYNAMIC_SCENES.md): when the map STOPPED
        # believing in something, and what it believed at the moment it
        # committed to a goal. Belief latency and stale-goal rate are computed
        # from these two logs plus the relocation step the env records.
        self.presence_events: List[dict] = []
        self._disbelieved: set = set()
        # "fire extinguisher #1", "#2": which track IS which object, and the
        # moves seen (objects/instances.py). None unless the preset names
        # classes in `scene_graph.instance_labels`.
        labels = list(self.cfg.scene_graph.instance_labels or [])
        self.instances: Optional[InstanceRegistry] = InstanceRegistry(
            labels,
            fragment_m=float(self.cfg.scene_graph.instance_fragment_m),
            missing_p=float(self.cfg.scene_graph.presence.min_presence),
            # Two sightings at least, whatever the candidate gate accepts: one
            # frame's false positive must not become "#3" or claim a missing one.
            min_obs=max(2, int(self.cfg.verification.min_obs)),
            max_height_m=float(getattr(self.cfg.scene_graph, "instance_max_height_m", 0.0) or 0.0),
        ) if labels else None
        self.target_instance = int(getattr(self.cfg.agent, "target_instance", 0) or 0)
        self.instance_events: List[dict] = []
        self._stale_stop_used = False
        self._stale_stop_pending = False
        self._relook = None  # (key, centre_xy, radius_m, exclude_xy, label)
        self._explore_hold_until = 0  # see `rearm`
        self.state_log = []
        self.approach.reset()
        self.candidates.reset()
        self.close_look.reset()
        self.kf_selector.reset()
        self.controller.reset()
        self.exploration.reset()
        self._escape = ActionHistoryEscape(int(self.cfg.agent.escape_window))
        self._progress_ref_step = 0
        self._progress_ref_xy = np.zeros(2)
        self._frontier_ref_dist = None
        self._approach_start_step = 0
        self._terminal_last_xy = None
        self._terminal_min_d = float("inf")
        self._terminal_stalls = 0
        self._approach_itm_max = 0.0
        self._approach_itm_n = 0
        self.approach_recheck_max = None
        self._last_itm = 0.0
        if self.commit_state is not None:
            self.commit_state.reset()
        if self.frontier_semantics is not None:
            self.frontier_semantics.reset()
        self.detector.set_vocabulary(
            target_vocabulary(self.target, self.cfg.detector.vocabulary)
        )
        if self.region_proposer is not None:
            self.region_proposer.set_target(self.target)
            self._region_admits = 0
            self._region_cache = None
            self._region_kf = 0
            self._region_named = False
            self.object_layer.set_proposal_text(self.region_proposer.text)
        if self.feature_memory is not None:
            # The prompt changes once an episode, so the text encoder runs once
            # an episode. Centres come from the object layer, which resolves a
            # linked component rather than one ellipsoid.
            self.feature_memory.set_target(self.target)
            self.feature_memory.bind_centres(
                lambda t: self.object_layer.center_of(t)[list(PLANE)]
            )
            self.feature_pick_log = []
        self.object_layer.set_target(self.target)
        self.object_layer.keep_cloud_labels = {self.target}
        if self.pointnav is not None:
            self.pointnav.reset()
        for component in (self.ranker, self.room_classifier, self.image_text):
            reset = getattr(component, "reset", None)
            if callable(reset):
                reset()

    def rearm(self, max_steps: int) -> None:
        """Give the agent another attempt without giving it a new map.

        Everything learned survives -- presence beliefs, searched surfaces,
        objects mapped along the way -- because that carry-over is the whole
        point of retrying, and it is what separates this from resetting. Only
        the navigation state goes back: the committed candidate is released and
        the agent returns to EXPLORE with its remaining step budget.

        The belief work belongs to the caller (eval/attempts.py), because what a
        failed attempt is WORTH is a protocol question, not an agent one.
        """
        self._schedule_relook()
        self._candidate_id = None
        self._target_obj_xy = None
        self._goal_xy = None
        self._current_path = None
        self.approach.at_viewpoint = False
        self.approach.scan_turns_left = 0
        self.approach.scan_expected = 0
        self.approach.stop_reason = None
        self.close_look.abort()
        self._stale_stop_pending = False
        self.state = State.EXPLORE
        self._goto_deadline = self.step_count + int(max_steps)
        self.stats["attempts"] = self.stats.get("attempts", 1) + 1
        # EXPLORE is not the same thing as an exploration ROUND. `_act_inner`
        # runs `candidates.check` before the EXPLORE branch on every step, so
        # the next same-label track commits immediately and the round -- rate
        # limited to one per `exploration.select_every` steps -- never happens.
        # Hold the commit open long enough for one, and make that one run now
        # rather than at the rate limit's convenience.
        hold = int(getattr(self.cfg.agent, "explore_after_failed_attempt_steps", 0))
        if hold > 0:
            self._explore_hold_until = self.step_count + hold
            force = getattr(self.exploration, "force_select_next", None)
            if callable(force):
                force()

    # ------------------------------------------------------------------- act

    def _take_operator_waypoint(self, world) -> bool:
        """`ros2.waypoint_xy`: the first exploration goal is where the operator
        said, facing `waypoint_yaw_deg`; the search resumes from there. Taken
        once per run."""
        ros = getattr(self.cfg, "ros2", None)
        xy = getattr(ros, "waypoint_xy", None)
        if not xy or getattr(self, "_waypoint_taken", False):
            return False
        from ..ros2.frames import ros_xy_to_pipeline

        goal = ros_xy_to_pipeline(float(xy[0]), float(xy[1]))
        yaw_deg = getattr(ros, "waypoint_yaw_deg", None)
        backend = getattr(getattr(self, "pointnav", None), "backend", None)
        if yaw_deg is not None and hasattr(backend, "set_goal_yaw"):
            backend.set_goal_yaw(goal, float(np.radians(float(yaw_deg))))
        self._waypoint_taken = True
        self._goal_xy = goal
        self._current_path = None
        self._candidate_id = None
        self.exploration.current_frontier = None
        self.exploration.note_progress(world)
        self.state = State.GOTO_FRONTIER
        self.stats["operator_waypoint"] = [float(xy[0]), float(xy[1]),
                                           None if yaw_deg is None else float(yaw_deg)]
        return True

    def _schedule_relook(self) -> None:
        """A stale stop just failed: the object is probably still on that
        surface. Remember the surface so the next attempt looks at it first,
        from somewhere other than where the agent stood."""
        if not bool(self.cfg.verification.relook_after_stale_stop):
            return
        if self._relook is not None or not self._stale_stop_used:
            return
        track = self.object_layer.get(self._candidate_id) if self._candidate_id is not None else None
        if track is None or not getattr(track, "from_prior", False) or track.seen_live:
            return
        here = None if self._agent_xy is None else np.asarray(self._agent_xy, dtype=float).copy()
        centre = np.asarray(self.object_layer.center_of(track), dtype=float)[list(PLANE)]
        key, radius, label = -int(track.id), _horizontal_radius_m(track), str(track.label)
        view = next((o for o in getattr(self.scene_graph, "objects", []) if int(o.track_id) == int(track.id)), None)
        cid = getattr(view, "container_id", None) if view is not None else None
        node = self.scene_graph.containers.get(int(cid)) if cid is not None else None
        if node is not None:
            from .close_look import _container_radius_m

            key = int(cid)
            centre = np.asarray(node.center, dtype=float)[list(PLANE)]
            radius = _container_radius_m(self, node)
            label = str(node.label)
        self._relook = (key, centre, radius, here, label)
        self.stats["relook_scheduled"] = self.stats.get("relook_scheduled", 0) + 1

    def _start_relook(self, frame: FrameData) -> Optional[str]:
        if self._relook is None or self.close_look.active:
            return None
        key, centre, radius, exclude_xy, label = self._relook
        self._relook = None
        self.stats["relook_started"] = self.stats.get("relook_started", 0) + 1
        self.close_look.start(key, centre, radius, "explore", reason="relook",
                              label=label, exclude_xy=exclude_xy)
        return self.close_look.step(frame)

    def act(self, frame: FrameData) -> str:
        self.step_count += 1
        prev_state = self.state
        with self.profiler.timeit("control_loop"):
            action = self._act_inner(frame)
        if self._escape.window > 0 and not (
            action == STOP_ACTION and self.state is State.DONE
        ):
            action = self._escape(action)
        if self.state != prev_state:
            self.state_log.append((self.step_count, self.state.value))
        self._last_action = action
        return action

    def _act_inner(self, frame: FrameData) -> str:
        self._agent_xy = frame.camera_position[list(PLANE)].copy()
        if self.pointnav is not None:
            self.pointnav.observe(frame)
        standing_y = float(frame.camera_position[1] - self.cfg.agent.camera_height)
        if self.map.floor_y is None:
            # The ground plane is latched on the first frame (or restored from
            # a prior map) and every later frame is banded against it.
            self.map.floor_y = standing_y
        floor_y = float(self.map.floor_y)
        # The strategy plans on this map's planner; a prior map or a reset may
        # have replaced it since the last step.
        if not isinstance(self.selection_planner, StraightLinePlanner):
            self.selection_planner = self.planner
        self.exploration.planner = self.selection_planner
        self._floor_y = floor_y
        self._off_plane_m = abs(standing_y - floor_y)
        reject_m = float(getattr(self.cfg.mapping, "floor_reject_m", 0.0))
        off_map = reject_m > 0.0 and self._off_plane_m > reject_m

        if off_map:
            self.stats["frames_off_plane"] = self.stats.get("frames_off_plane", 0) + 1
        else:
            with self.profiler.timeit("costmap"):
                self.costmap.update(
                    frame,
                    floor_y=floor_y,
                    obstacle_low=self.cfg.mapping.obstacle_low_m,
                    obstacle_high=self.cfg.mapping.obstacle_high_m,
                    max_range=self.cfg.mapping.max_range_m,
                    stride=self.cfg.mapping.depth_stride,
                )
                clear_m = float(getattr(self.cfg.mapping, "footprint_clear_m", 0.0) or 0.0)
                if clear_m > 0.0:
                    n_cleared = self.costmap.clear_footprint(self._agent_xy, clear_m)
                    self.stats["footprint_cells_cleared"] = (
                        self.stats.get("footprint_cells_cleared", 0) + n_cleared)
            if self.image_text is not None:
                self._update_value_map(frame, self.map)
            # Tests and external callers may inject an image-text scorer
            # without enabling the semantic value map. Preserve that legacy
            # approach-recheck path in that case; when a value map exists the
            # single score above already feeds both mechanisms.
            if (
                self.state is State.APPROACH
                and self.image_text is not None
                and self.map.value_map is None
            ):
                scores = self.image_text.score(
                    frame.rgb, [self.target.replace("_", " ")]
                )
                if len(scores):
                    self._last_itm = float(scores[0])
                    self._approach_itm_max = max(
                        self._approach_itm_max, self._last_itm
                    )
                    self._approach_itm_n += 1
        self.controller.observe_progress(
            frame.T_wc, self._last_action, self.costmap, self.step_count
        )
        if self.controller.stuck:
            self.controller.stuck = False
            self._current_path = None  # force replan

        if self.kf_selector.is_keyframe(frame.T_wc):
            self._on_keyframe(frame)
            if self.feature_memory is not None:
                # Mechanism counters ride out with the rest; on this benchmark
                # SR cannot resolve anything under about three trials, so an arm
                # that cannot be seen in a counter cannot be judged at all.
                self.stats.update(self.feature_memory.counters)
            if self.cfg.exploration.search_posterior:
                with self.profiler.timeit("glance"):
                    self.exploration.glance(self._world(frame))
            with self.profiler.timeit("close_look"):
                self.close_look.maybe_opportunistic(frame)

        # Candidate target check happens in every state except terminal ones
        if self.state in (State.INIT, State.EXPLORE, State.GOTO_FRONTIER) or (
            self.state is State.CLOSE_LOOK and self.close_look.resume == "explore"
        ):
            if self.step_count <= self._explore_hold_until:
                # A failed attempt is holding the commit open so one real
                # exploration round can happen (`rearm`). A hold, not a ban.
                self.stats["explore_hold_steps"] = (
                    self.stats.get("explore_hold_steps", 0) + 1)
            else:
                with self.profiler.timeit("candidates"):
                    self.candidates.check(frame.camera_position[list(PLANE)])
        if self.close_look.active and self.state is not State.CLOSE_LOOK:
            self.close_look.interrupted()  # a commit pre-empted the look
        if self.state == State.CLOSE_LOOK:
            return self.close_look.step(frame)

        if self.state == State.INIT:
            if self._scan_steps_left > 0:
                self._scan_steps_left -= 1
                return TURN_ACTION
            self.state = State.EXPLORE

        if self.state == State.EXPLORE:
            relook = self._start_relook(frame)
            if relook is not None:
                return relook
            # Look at the surface before the selection round scores it
            # searched -- the selection round retires it on arrival, and
            # the belief update should rest on a frame that shows it.
            facing = self.exploration.face_surface(self._world(frame))
            if facing is not None:
                return facing
            self._explore(frame)
            if self.state == State.EXPLORE:  # nothing selectable
                return TURN_ACTION  # keep looking around; map will grow

        if self.state == State.GOTO_FRONTIER:
            reselect = int(getattr(self.cfg.exploration, "reselect_every", 0))
            if reselect > 0 and self.step_count % reselect == 0:
                self._select_new_frontier(frame)
            if self.exploration.maybe_give_up(self._world(frame)):
                self._current_path = None
                self.state = State.EXPLORE
                return self._act_inner_post_transition(frame)
            action = self._follow_path(frame)
            if action is not None:
                return action
            self.exploration.current_frontier = None
            self.state = State.EXPLORE
            return self._act_inner_post_transition(frame)

        if self.state == State.GOTO_VERIFY_VIEW:
            # Same terminal semantics as GOTO_TARGET: with discrete actions
            # the agent rarely lands exactly on the viewpoint — verify once
            # we are near it, the path is consumed, or the deadline passes.
            agent_xy = frame.camera_position[list(PLANE)]
            near_view = (
                self._goal_xy is not None
                and np.linalg.norm(agent_xy - self._goal_xy) < 0.35
            )
            if near_view or self.step_count > self._goto_deadline:
                self.state = State.VERIFYING
            else:
                action = self._follow_path(frame)
                if action is not None:
                    return action
                self.state = State.VERIFYING

        if self.state == State.VERIFYING:
            return self.candidates.verify(frame)

        if self.state == State.APPROACH:
            return self._do_approach(frame)

        return STOP_ACTION

    def _act_inner_post_transition(self, frame: FrameData) -> str:
        """Re-enter EXPLORE logic once after a state transition (no recursion
        beyond one level: EXPLORE either picks a path or turns in place)."""
        self._explore(frame)
        if self.state == State.GOTO_FRONTIER:
            action = self._follow_path(frame)
            if action is not None:
                return action
        return TURN_ACTION

    # -------------------------------------------------------------- keyframes

    def _active_container_tracks(self):
        """The tracks making up the surface the search is currently inspecting.

        A container is a view over tracks rather than an entity of its own, and
        an L-shaped sofa is two ellipsoids under one anchor id, so the answer is
        a list -- projecting only the representative would crop half the sofa.
        """
        cid = getattr(self.exploration, "search_container", None)
        if cid is None:
            return None
        node = self.scene_graph.containers.get(int(cid))
        return list(node.track_ids) if node is not None else None

    def _foveate(self, frame: FrameData, dets: list) -> list:
        """A second detector pass over the container surfaces in view.

        Reported as the DECISION, not the state: `foveate_added` counts only
        detections the whole-frame pass did not already have, and
        `foveate_added_target` only those of the episode's target -- the arm's
        entire claim. An arm that fires constantly and adds no target is a null,
        and has to be legible as one.
        """
        from ..perception.foveate import container_regions, foveated_detect, merge

        sg = self.cfg.scene_graph
        only_ids = None
        if sg.foveate_active_only:
            only_ids = self._active_container_tracks()
            if not only_ids:
                return dets
        regions = container_regions(
            self.object_layer, frame, CONTAINER_CATEGORIES,
            max_range_m=float(sg.foveate_max_range_m),
            min_px=float(sg.foveate_min_bbox_px),
            max_regions=int(sg.foveate_max_regions),
            only_ids=only_ids,
        )
        if not regions:
            return dets
        self.stats["foveate_regions"] = self.stats.get("foveate_regions", 0) + len(regions)
        extra = foveated_detect(self.detector, frame.rgb, regions,
                                pad=float(sg.foveate_pad))
        merged, n_added = merge(dets, extra)
        if n_added:
            self.stats["foveate_added"] = self.stats.get("foveate_added", 0) + n_added
            want = normalize_label(self.target)
            hits = sum(1 for d in merged[len(dets):]
                       if normalize_label(d.label) == want)
            if hits:
                self.stats["foveate_added_target"] = (
                    self.stats.get("foveate_added_target", 0) + hits
                )
        return merged

    def _on_keyframe(self, frame: FrameData) -> None:
        self._kf_count += 1
        with self.profiler.timeit("detector"):
            dets = self.detector.detect(frame.rgb)
            if self.cfg.scene_graph.foveate_containers:
                dets = self._foveate(frame, dets)
        if self.frontier_semantics is not None:
            room = self.room_classifier.classify(frame.rgb) if self.room_classifier else None
            heading = agent_heading(frame.T_wc)
            self.frontier_semantics.observe(
                self.step_count,
                room,
                [d.label for d in dets],
                camera_xy=frame.camera_position[list(PLANE)],
                heading_xy=np.array([np.cos(heading), np.sin(heading)]),
            )
        if self.room_classifier is not None:
            self._room_votes.append((
                frame.camera_position[list(PLANE)].copy(),
                self.room_classifier.classify(frame.rgb),
            ))
        named_dets = list(dets or [])
        dets = self._propose_regions(frame, dets)
        if self.on_keyframe_detections is not None:
            # The ground-truth view counts what the DETECTOR saw; proposals
            # have their own counters.
            self.on_keyframe_detections(frame, named_dets)
        with self.profiler.timeit("object_layer"):
            self.object_layer.update(frame, dets)
        self.keyframes.add(frame)
        if self.instances is not None:
            self.instances.floor_y = self.map.floor_y
            self.instance_events += self.instances.update(self.object_layer, step=self.step_count)
            self.candidates.follow_registry()

        pf = self.object_layer.presence_filter
        if pf is not None:
            # Surfaced per episode so the mechanism is measurable on real runs:
            # how much evidence the beliefs rest on, and how many objects the
            # agent has actually looked for and failed to find.
            self.stats["presence_expected"] = pf.n_expected
            self.stats["presence_negative"] = pf.n_negative
            self.stats["presence_positive"] = pf.n_positive
            self.stats["presence_disbelieved"] = sum(
                1 for t in self.object_layer.tracks() if t.presence.p < 0.1
            )
            for track in self.object_layer.tracks():
                if track.presence.p >= 0.1 or track.id in self._disbelieved:
                    continue
                # First crossing only: the step here is what "belief latency"
                # is measured against, so a belief that dips, recovers and dips
                # again must not reset the clock.
                self._disbelieved.add(track.id)
                self.presence_events.append(
                    {
                        "step": int(self.step_count),
                        "track_id": int(track.id),
                        "label": str(track.label),
                        "center": [float(v) for v in self.object_layer.center_of(track)],
                        "p": round(float(track.presence.p), 4),
                        "n_missed": int(track.presence.n_missed),
                    }
                )

        if self._kf_count % self.cfg.scene_graph.room_seg_every_kf == 1:
            with self.profiler.timeit("room_seg"):
                self._room_labels = self.map.segmenter.segment(self.costmap)
        if self._room_labels is not None:
            if self._room_labels.shape != self.costmap.grid.shape:
                self._room_labels = self.map.segmenter.segment(self.costmap)
            with self.profiler.timeit("scene_graph"):
                self.scene_graph.rebuild(
                    self._room_labels, self.costmap, self.object_layer,
                    floor_y=float(self.map.floor_y or 0.0),
                )
                self._label_rooms(self.map)

    def _label_rooms(self, layer) -> None:
        from collections import Counter

        if not self._room_votes or layer.room_labels is None:
            return
        votes = {}
        for xy, name in self._room_votes:
            rc = layer.costmap.world_to_grid(xy)
            if not layer.costmap.in_bounds(rc):
                continue
            local = int(layer.room_labels[rc[0], rc[1]])
            if local > 0:
                votes.setdefault(local, Counter())[name] += 1
        self.stats["rooms_total"] = len(self.scene_graph.rooms)
        for room_id, counter in votes.items():
            room = self.scene_graph.rooms.get(room_id)
            if room is not None:
                room.label = counter.most_common(1)[0][0]
        self.stats["rooms_labelled"] = sum(
            1 for room in self.scene_graph.rooms.values() if room.label
        )

    def _world(self, frame: FrameData) -> WorldView:
        """What the exploration strategy is allowed to see this round.

        Built per call rather than held: `goal_xy` belongs to the FSM.
        """
        return WorldView(
            frame=frame,
            step=self.step_count,
            agent_xy=frame.camera_position[list(PLANE)],
            costmap=self.costmap,
            scene_graph=self.scene_graph,
            object_layer=self.object_layer,
            keyframes=self.keyframes,
            target=self.target,
            goal_xy=self._goal_xy,
            value_map=self.map.value_map,
            floor_y=float(self.map.floor_y or 0.0),
        )

    def _explore(self, frame: FrameData) -> None:
        """Run one selection round and act on what it chose.

        The strategy decides where; this applies it. Both kinds of choice enter
        GOTO_FRONTIER, because that is the only state that follows `_goal_xy` --
        a surface chosen but left in EXPLORE is a surface never visited, which
        is the defect that invalidated every C3 result before it was found.
        """
        world = self._world(frame)
        if self._take_operator_waypoint(world):
            return
        choice = self.exploration.select(world)
        if choice is None:
            return
        self._goal_xy = choice.goal_xy
        self._current_path = choice.path
        self.exploration.note_progress(world)
        self.state = State.GOTO_FRONTIER

    # ------------------------------------------------------------- candidates

    def _seen_this_approach(self) -> bool:
        """Was the target confirmed visible during this approach, closely
        enough for that to protect the stop from the absence reading?

        `verification.seen_counts_within_m` is the range beyond which a
        sighting does not count; 0 is the shipped rule, any range. Measured
        on the robot (outputs/20260922_191444): one 0.36-score "sports ball"
        at 4.9 m, 34 steps before arrival, and the run STOPped at an empty
        spot while both ball tracks' beliefs had already fallen to 0.44 --
        the reading that would have abandoned was never taken because of
        that one far frame. Detector recall beyond 3 m is 0.24-0.26
        (`absence_max_range_m`), so a far sighting is a weak claim either way.
        """
        if self.approach.last_good_xy is None:
            return False
        lim = float(getattr(self.cfg.verification, "seen_counts_within_m", 0.0) or 0.0)
        rng = getattr(self.approach, "last_good_range_m", None)
        if lim > 0.0 and rng is not None and float(rng) > lim:
            self.stats["far_sighting_ignored"] = self.stats.get("far_sighting_ignored", 0) + 1
            return False
        return True

    def _absence_at_arrival(self, frame: FrameData, reason: str,
                            from_look: bool = False) -> Optional[str]:
        """The approach is ending and the target was never seen. Say so.

        Returns an action when the candidate is abandoned (the caller must not
        STOP), or None to let the normal termination proceed. A track that has
        been seen at some point during this approach is left alone -- the target
        was there, so this is a geometry or timing problem, not absence.

        The reading itself is verification/absence.py; what is here is applying
        its verdict to the FSM.
        """
        track = (
            self.object_layer.get(self._candidate_id)
            if self._candidate_id is not None else None
        )
        if track is None or self._seen_this_approach():
            return None
        if self._stale_stop_pending and not from_look:
            # Granted at the end of the close look; the approach walked back to
            # its ring and this is the arrival the STOP was promised for.
            self._stale_stop_pending = False
            return None
        if (
            bool(self.cfg.verification.stop_at_stale_anchor_once)
            and not self._stale_stop_used
            and bool(getattr(track, "from_prior", False))
            and not track.seen_live
        ):
            # The stale anchor is the answer more often than not, and a stop
            # here costs one attempt of three. Returning None lets the
            # approach STOP; the protocol scores it and, if it fails, applies
            # the negative reading (eval/attempts.py). From the close look the
            # agent stands on the 1.5 m ring, too far to score, so the look's
            # own path re-approaches the tight ring and the STOP is taken on
            # that arrival instead (`_stale_stop_pending`).
            self._stale_stop_used = True
            self.stats["stale_anchor_stop"] = self.stats.get("stale_anchor_stop", 0) + 1
            if (bool(self.cfg.verification.stale_stop_at_nearest_free)
                    and not bool(getattr(self.approach, "closed", False))):
                # (skipped when the approach already closed the last metre on
                # the navmesh, which gets nearer than the costmap's free cell)
                # Not here: as close to the old position as the furniture allows.
                from ..mapping.costmap import nearest_free_xy

                centre = np.asarray(self.object_layer.center_of(track), dtype=float)[list(PLANE)]
                goal = np.asarray(nearest_free_xy(self.costmap, centre), dtype=float)
                here = frame.camera_position[list(PLANE)]
                if float(np.linalg.norm(goal - here)) > 0.2:
                    self._goal_xy = goal
                    self._current_path = None
                    self.approach.path_goal = None
                    self.approach.at_viewpoint = False
                    self.approach.steps_left = max(int(self.approach.steps_left), 60)
                    self._goto_deadline = max(int(self._goto_deadline), self.step_count + 60)
                    self._stale_stop_pending = True
                    self.stats["stale_stop_reaimed"] = self.stats.get("stale_stop_reaimed", 0) + 1
                    self.state = State.APPROACH
                    return TURN_ACTION
            self._stale_stop_pending = bool(from_look)
            return None
        verdict = self.absence.observe(
            track, self.target, frame, self.object_layer.presence_filter,
            self.approach.scan_expected, reason,
        )
        if verdict is None or not verdict.abandon:
            verdict = self._instance_gone_verdict(track, reason)
            if verdict is None:
                return None
        self.presence_events.append(
            {
                "step": int(self.step_count),
                "track_id": int(track.id),
                "label": str(track.label),
                "center": [float(v) for v in self.object_layer.center_of(track)],
                "p": round(float(verdict.p), 4),
                "n_missed": int(track.presence.n_missed),
                **verdict.event,
            }
        )
        # Deliberately NOT blacklisted -- see verification/absence.py.
        self._candidate_id = None
        self._target_obj_xy = None
        self.state = State.EXPLORE
        return TURN_ACTION

    # ---------------------------------------------------------------- helpers

    def _best_target_detection(self, frame: FrameData) -> Optional[Detection]:
        """The detector's best detection of the target on this frame, or None.

        This is the choke point the approach stop, the close look and the
        exploration check all ask, and -- through the close look -- it is what
        decides that a committed track is ABSENT. Asking only the label path
        there is circular for exactly the objects the proposal stage exists
        for: the detector that could not name the object is re-asked whether
        the object is present, says no, and the track is retired.

        Measured on in_anchor__0117__banana: the agent walked to the banana,
        looked from 1.5 m, was told "not detected", dropped the track's belief
        from 0.95 to 0.433 and committed elsewhere -- in an episode where the
        proposal stage had already admitted 16 regions.
        """
        target = normalize_label(self.target)
        with self.profiler.timeit("detector"):
            dets = self.detector.detect(frame.rgb)
        matches = [
            d for d in dets
            if normalize_label(d.label) == target and d.score > 0.25
        ]
        if matches:
            matches = self._of_committed_instance(matches, frame)
        if matches:
            return max(matches, key=lambda d: d.score)
        # The fallthrough is for the track whose identity rests on appearance.
        # This function also stops and steers every approach, ends the close
        # look and gates the terminal stop, so a proposal answering it while
        # the agent works a track the DETECTOR named lets a region on the
        # wrong object do all of that. Measured with the stage on every
        # keyframe: a named red plate approach stopped at 1.52 m on a region,
        # a tin can scored a false stop at step 46 on its prior-map track.
        # A proposal-only track gets the proposal sensor; a named one gets the
        # detector, exactly as before the stage existed.
        if not self._working_a_proposal_track():
            return None
        return self._region_detection(frame)

    def _instance_gone_verdict(self, track, reason: str) -> Optional[AbsenceVerdict]:
        """Abandon an arrival at an instance the map already disbelieves.

        The arrival reading wants a frame that would have shown the object
        (`absence_requires_expectation`); from the stop pose there may be none,
        and then the STOP stands. With one object of the class that costs a
        wasted stop. With two it reports an empty place as the object asked for
        -- measured in Isaac (outputs/20261113_011412): #1 moved, the presence
        filter at p=0.02 on its old track from five expected-and-missed frames on
        the way in, the close look silent, and the run ended DONE where #1 used
        to stand. Here the walk in has already been the reading: the track is
        below the candidate gate's `min_presence` and none of the frames on this
        approach showed it (`_seen_this_approach`, instance-filtered)."""
        reg = self.instances
        if reg is None or not reg.covers(self.target) or not reg.is_track_missing(track):
            return None
        track.absence_arrivals += 1
        track.identity_rejections += 1
        self.stats["instance_absent_on_arrival"] = (
            self.stats.get("instance_absent_on_arrival", 0) + 1)
        return AbsenceVerdict(abandon=True, p=float(track.presence.p),
                              event={"cause": f"instance_disbelieved:{reason}"})

    def _of_committed_instance(self, dets: List[Detection], frame: FrameData) -> List[Detection]:
        """The detections that are the committed track's object, when its class
        is one whose objects are told apart (objects/instances.py); all of them
        otherwise, or when nothing is committed. A detection with no depth to
        place it is not counted: with two of them in the room, "an extinguisher
        is in view" is not evidence that THIS one is."""
        reg = self.instances
        if reg is None or self._candidate_id is None or not reg.covers(self.target):
            return dets
        from ..objects.instances import detection_xyz

        match_m = float(self.cfg.scene_graph.instance_match_m)
        kept = [d for d in dets
                if reg.belongs(detection_xyz(d, frame), self._candidate_id,
                               self.object_layer, match_m)]
        if len(kept) < len(dets):
            self.stats["other_instance_in_view"] = (
                self.stats.get("other_instance_in_view", 0) + len(dets) - len(kept))
        return kept

    def _working_a_proposal_track(self) -> bool:
        cid = getattr(self, "_candidate_id", None)
        if cid is None:
            return False
        track = self.object_layer.get(int(cid))
        return bool(track is not None and getattr(track, "proposal_only", False))

    def _region_active(self) -> bool:
        """Is the proposal stage a fallback for THIS episode?

        Per-frame gating was the defect: on a trial where the detector works the
        target is absent from most individual frames, so a per-frame test fires
        on nearly all of them. Measured on the full 107, that took the 21
        perception trials from 0 to 6 and the other 86 from 57 to 41, with 13 of
        the 18 lost trials exhausting all three attempts on regions the stage
        had admitted.

        Episode-level instead: once the label path has named the target even
        once, the detector can see this object and the stage stays off.
        """
        rp = self.region_proposer
        if rp is None:
            return False
        cfg = rp.cfg
        if self._region_admits >= int(cfg.max_per_episode):
            return False
        if bool(getattr(cfg, "every_keyframe", False)):
            return True
        if bool(getattr(cfg, "require_never_named", False)) and self._region_named:
            return False
        return self._region_kf >= int(getattr(cfg, "unnamed_keyframes", 0) or 0)

    def _region_detection(self, frame: FrameData) -> Optional[Detection]:
        """The proposal stage's answer for this frame, computed at most once.

        Cached on the frame id because this is called several times per step --
        the approach stop, the close look and the absence sensor each ask -- and
        a segment-plus-encode per call would multiply the stage's cost by the
        number of askers rather than by the number of frames.
        """
        rp = self.region_proposer
        if rp is None or self.target is None or not bool(rp.cfg.use_for_absence):
            return None
        if not self._region_active():
            return None
        key = int(getattr(frame, "frame_id", -1))
        cached = getattr(self, "_region_cache", None)
        if cached is not None and cached[0] == key:
            return cached[1]
        with self.profiler.timeit("region_proposal"):
            det = rp.propose(frame.rgb)
        self._region_cache = (key, det)
        self.stats.update(rp.counters)
        return det

    def _target_visible(self, frame: FrameData) -> bool:
        """Does the detector see the target category in the current view?"""
        return self._best_target_detection(frame) is not None

    @staticmethod
    def _detection_depth(det: Detection, frame: FrameData) -> Optional[float]:
        """Median metric depth (m) over the detection's mask, using only valid
        depth pixels; None if too few valid samples (mask off the depth range)."""
        ys, xs = np.nonzero(det.mask)
        if ys.size == 0:
            return None
        d = frame.depth[ys, xs]
        valid = d > 1e-3
        if int(valid.sum()) < 8:
            return None
        return float(np.median(d[valid]))

    def _plan_to(
        self, frame: FrameData, goal_xy: np.ndarray, goal_tolerance_m: Optional[float] = None
    ) -> None:
        agent_xy = frame.camera_position[list(PLANE)]
        with self.profiler.timeit("planner"):
            result: PlanResult = self.planner.plan(
                self.costmap, agent_xy, goal_xy, goal_tolerance_m
            )
        self._current_path = result.path if result.success else None
        self.stats["plan_ok" if result.success else "plan_fail"] += 1

    def _follow_path(self, frame: FrameData) -> Optional[str]:
        goal = (
            frontier_goal_xy(
                self.exploration.current_frontier, self.costmap,
                self.exploration.goal_prefer_free,
            )
            if self.state == State.GOTO_FRONTIER
            and self.exploration.current_frontier is not None
            else self._goal_xy
        )
        if goal is None:
            return None
        if self._use_navmesh:
            # Drive on Habitat's navmesh. None = arrived-or-unreachable; if we're
            # still far from a frontier goal, block it (as the stub-block does).
            action = self._nav_fn(goal, None)
            if action is None and self.state == State.GOTO_FRONTIER:
                self.exploration.retire_pursued(self._world(frame), goal)
            return action
        if self.pointnav is not None:
            step = self.pointnav.step(goal)
            if step.action is None and self.state == State.GOTO_FRONTIER:
                if step.reason == "policy_stop" and not bool(
                    self.cfg.agent.pointnav_stop_means_blocked
                ):
                    self.stats["pointnav_stop_forced_forward"] = (
                        self.stats.get("pointnav_stop_forced_forward", 0) + 1
                    )
                    return "move_forward"
                if float(
                    np.linalg.norm(frame.camera_position[list(PLANE)] - goal)
                ) > self._frontier_reach_m:
                    # ``retire_pursued`` owns both the blacklist update and the
                    # associated counter.  Doing either here would double count
                    # a PointNav policy stop.
                    pass
                self.exploration.retire_pursued(self._world(frame), goal)
            return step.action
        if self._current_path is None:
            self._plan_to(frame, goal)
            if self._current_path is None:
                if self.state == State.GOTO_FRONTIER:
                    self.exploration.block(
                        self.exploration.current_frontier, 50, self.step_count
                    )
                return None
        action = self.controller.act(frame.T_wc, self._current_path)
        if action is None:
            self._current_path = None
            if self.state == State.GOTO_FRONTIER:
                self.exploration.retire_pursued(self._world(frame), goal)
        return action

    # ------------------------------------------------ ASCENT compatibility API

    @property
    def frontier_extractor(self):
        return self.exploration.frontier_extractor

    @property
    def _progress_ref_step(self) -> int:
        return int(self.exploration.progress_ref_step)

    @_progress_ref_step.setter
    def _progress_ref_step(self, value: int) -> None:
        self.exploration.progress_ref_step = int(value)

    @property
    def _progress_ref_xy(self) -> np.ndarray:
        return self.exploration.progress_ref_xy

    @_progress_ref_xy.setter
    def _progress_ref_xy(self, value: np.ndarray) -> None:
        self.exploration.progress_ref_xy = np.asarray(value, dtype=float)

    @property
    def _select_every(self) -> int:
        return int(getattr(self.cfg.exploration, "select_every", 5))

    @property
    def _reselect_every(self) -> int:
        return int(getattr(self.cfg.exploration, "reselect_every", 0))

    @property
    def _last_select_step(self) -> int:
        return self.exploration._last_select_step

    @_last_select_step.setter
    def _last_select_step(self, value: int) -> None:
        self.exploration._last_select_step = int(value)

    def _select_new_frontier(self, frame: FrameData) -> None:
        prev = self._current_frontier
        before = self.exploration._last_select_step
        progress_step = self._progress_ref_step
        progress_xy = self._progress_ref_xy.copy()
        frontier_dist = self._frontier_ref_dist
        self._explore(frame)
        if self.exploration._last_select_step == before:
            return
        current = self._current_frontier
        if current is None:
            return
        same = prev is not None and float(
            np.linalg.norm(prev.centroid_xy - current.centroid_xy)
        ) < 0.5
        if same:
            self._progress_ref_step = progress_step
            self._progress_ref_xy = progress_xy
            self._frontier_ref_dist = frontier_dist
        else:
            self.stats["frontier_switch"] = self.stats.get("frontier_switch", 0) + 1
            self._progress_ref_step = self.step_count
            self._progress_ref_xy = frame.camera_position[list(PLANE)].copy()
            self._frontier_ref_dist = None

    def _frontier_consumed(self, frontier) -> bool:
        if frontier is None:
            return False
        goal = frontier_goal_xy(frontier, self.costmap)
        rc = self.costmap.world_to_grid(goal)
        radius = max(1, int(round(self.cfg.agent.agent_radius / self.costmap.resolution)))
        h, w = self.costmap.grid.shape
        r0, r1 = max(0, rc[0] - radius), min(h, rc[0] + radius + 1)
        c0, c1 = max(0, rc[1] - radius), min(w, rc[1] + radius + 1)
        if r0 >= r1 or c0 >= c1:
            return False
        from ..mapping.costmap import UNKNOWN

        return not bool((self.costmap.grid[r0:r1, c0:c1] == UNKNOWN).any())

    def _frontier_stalled(
        self, agent_xy: np.ndarray, frontier, stick_m: float, stick_steps: int
    ) -> bool:
        if stick_steps <= 0:
            return False
        if self.cfg.agent.frontier_stick_rule == "closing":
            goal = frontier_goal_xy(frontier, self.costmap) if frontier else self._goal_xy
            if goal is None:
                return False
            distance = float(np.linalg.norm(agent_xy - goal))
            if self._frontier_ref_dist is None:
                self._frontier_ref_dist = distance
                self._progress_ref_step = self.step_count
                return False
            if abs(self._frontier_ref_dist - distance) > stick_m:
                self._frontier_ref_dist = distance
                self._progress_ref_step = self.step_count
                return False
            return self.step_count - self._progress_ref_step >= stick_steps
        if self.step_count - self._progress_ref_step < stick_steps:
            return False
        moved = float(np.linalg.norm(agent_xy - self._progress_ref_xy))
        self._progress_ref_step = self.step_count
        self._progress_ref_xy = agent_xy.copy()
        return moved < stick_m

    def _nearest_point_stop(self, agent_xy: np.ndarray) -> Optional[str]:
        track = self.object_layer.get(self._candidate_id) if self._candidate_id is not None else None
        if track is None:
            return None
        distance = self.object_layer.nearest_point_dist_xy(
            track, agent_xy, float(self.cfg.agent.terminal_percentile)
        )
        if distance is None or distance >= float(self.cfg.agent.terminal_engage_m):
            return None
        if distance <= float(self.cfg.agent.terminal_stop_m):
            return "nearest_point"
        moved = self._terminal_last_xy is not None and float(
            np.linalg.norm(agent_xy - self._terminal_last_xy)
        ) > 0.05
        self._terminal_last_xy = agent_xy.copy()
        if not moved:
            return None
        if abs(distance - self._terminal_min_d) < float(self.cfg.agent.terminal_progress_eps):
            self._terminal_stalls += 1
            if self._terminal_stalls >= int(self.cfg.agent.terminal_stall_steps):
                return "nearest_point_stalled"
        else:
            self._terminal_stalls = 0
            self._terminal_min_d = min(self._terminal_min_d, distance)
        return None

    def _propose_regions(self, frame: FrameData, dets):
        """Add a class-agnostic proposal for the target, when one is warranted.

        The stage returns an ordinary `Detection` under the target label, so
        admission, the presence filter, the identity channel, the candidate
        gate, the VLM and the attempt protocol all judge it exactly as they
        judge the detector's own output. Nothing here can stop an approach or
        score a trial by itself.

        It runs only when the detector produced nothing for the target on this
        keyframe -- the case it exists for -- and at most
        `max_per_episode` times, because at the measured 85% precision an
        unbounded stage would write a lot of wrong tracks.
        """
        rp = self.region_proposer
        if rp is None or self.target is None:
            return dets
        cfg = rp.cfg
        self._region_kf += 1
        want = normalize_label(self.target)
        named_now = any(normalize_label(d.label) == want for d in dets or [])
        if named_now:
            # The detector can see this object. Whatever else is true of the
            # episode, it does not need a fallback -- and a fallback that runs
            # anyway competes with a detector that was about to succeed.
            self._region_named = True
        if not self._region_active():
            return dets
        if bool(cfg.only_when_unnamed) and named_now:
            return dets
        key = int(getattr(frame, "frame_id", -1))
        cached = getattr(self, "_region_cache", None)
        if cached is not None and cached[0] == key:
            det = cached[1]
        else:
            with self.profiler.timeit("region_proposal"):
                det = rp.propose(frame.rgb)
            self._region_cache = (key, det)
        self.stats.update(rp.counters)
        if det is None:
            return dets
        self._region_admits += 1
        self.stats["region_admits"] = self._region_admits
        return list(dets or []) + [det]

    def _update_value_map(self, frame: FrameData, layer) -> None:
        """Score the current view and fuse it into the map's value layer.

        The image-text component is constructed only for explicit
        ``exploration.value_map`` configurations. Keeping the update here,
        after the costmap has grown, guarantees that value/confidence arrays
        remain aligned with the occupancy grid. A stride avoids repeatedly scoring
        effectively identical frames and preserves the approach re-check
        telemetry from the same scalar.
        """
        value_map = getattr(layer, "value_map", None)
        if self.image_text is None or value_map is None:
            return
        stride = max(1, int(getattr(self.cfg.exploration, "value_stride", 1)))
        if self.step_count % stride:
            return
        prompt = str(
            getattr(
                self.cfg.exploration,
                "value_prompt",
                "Seems like there is a {target} ahead.",
            )
        ).format(target=self.target.replace("_", " "))
        with self.profiler.timeit("value_map"):
            scores = self.image_text.score(frame.rgb, [prompt])
            if len(scores) == 0:
                return
            value = float(scores[0])
            value_map.update(frame, value)
        self.stats["value_calls"] = self.stats.get("value_calls", 0) + 1
        self._last_itm = value
        if self.state is State.APPROACH:
            self._approach_itm_max = max(self._approach_itm_max, value)
            self._approach_itm_n += 1





    def _recheck_rejects(self, stop_reason: str) -> bool:
        """Apply ASCENT's latched image-text gate before committing a stop."""
        self.approach_recheck_max = float(self._approach_itm_max)
        if not self._approach_recheck:
            return False
        self.stats["recheck_calls"] = self.stats.get("recheck_calls", 0) + 1
        if self._approach_itm_n == 0:
            self.stats["recheck_no_obs"] = self.stats.get("recheck_no_obs", 0) + 1
            return False
        if self._approach_itm_max >= self._approach_recheck_thresh:
            self.stats["recheck_pass"] = self.stats.get("recheck_pass", 0) + 1
            return False
        self.stats["recheck_reject"] = self.stats.get("recheck_reject", 0) + 1
        self.stats[f"recheck_reject_{stop_reason}"] = (
            self.stats.get(f"recheck_reject_{stop_reason}", 0) + 1
        )
        if self._candidate_id is not None:
            self.object_layer.blacklist(self._candidate_id)
        self._candidate_id = None
        self._target_obj_xy = None
        self._goal_xy = None
        self._current_path = None
        self.state = State.EXPLORE
        return True

    def _commit_terminal_stop(self, stop_reason: str) -> str:
        if self._recheck_rejects(stop_reason):
            return TURN_ACTION
        self.state = State.DONE
        self.approach.stop_reason = stop_reason
        return STOP_ACTION

    def _start_approach(
        self,
        obj_xy: np.ndarray,
        agent_xy: Optional[np.ndarray] = None,
    ) -> None:
        """Start the terminal approach on `obj_xy`."""
        self._approach_itm_max = 0.0
        self._approach_itm_n = 0
        self.approach_recheck_max = None
        self._approach_start_step = self.step_count
        here = agent_xy if agent_xy is not None else self._agent_xy
        obj_radius_m = 0.0
        if self._candidate_id is not None:
            track = self.object_layer.get(self._candidate_id)
            if track is not None:
                if here is not None:
                    nearest = self.object_layer.nearest_point_xy(track, here)
                    self._target_cloud_xy = (
                        None if nearest is None else np.asarray(nearest, dtype=float).copy()
                    )
                obj_radius_m = _horizontal_radius_m(track)
        self.approach.start(obj_xy, agent_xy, obj_radius_m=obj_radius_m)

    def _abandon_approach(self) -> str:
        disabled = self._candidate_id is not None and self.object_layer.disable_target(
            self._candidate_id
        )
        if not disabled and self._target_obj_xy is not None:
            self.object_layer.disable_place(self._target_obj_xy, self.target)
        self.stats["approach_abandon"] = self.stats.get("approach_abandon", 0) + 1
        self._candidate_id = self._target_obj_xy = self._goal_xy = None
        self._current_path = None
        self.state = State.EXPLORE
        return TURN_ACTION

    def _do_approach(self, frame: FrameData) -> str:
        budget = int(getattr(self.cfg.agent, "approach_abandon_steps", 0))
        if (
            self._reachable_fn is None and budget > 0
            and self.step_count - self._approach_start_step >= budget
        ):
            return self._abandon_approach()
        if self.cfg.agent.terminal_rule == "nearest_point":
            reason = self._nearest_point_stop(frame.camera_position[list(PLANE)])
            if reason is not None and (
                not self.cfg.agent.terminal_requires_detection
                or self._target_visible(frame)
            ):
                return self._commit_terminal_stop(reason)
        if hasattr(self, "_approach_steps_left"):
            self.approach.steps_left = self._approach_steps_left
        action = self.approach.step(frame)
        self._approach_steps_left = self.approach.steps_left
        if action == STOP_ACTION:
            return self._commit_terminal_stop(self.approach.stop_reason or "approach")
        return action

    def _follow_to(self, frame: FrameData, goal_xy: np.ndarray) -> Optional[str]:
        if self.pointnav is not None:
            creep = (
                float(self.cfg.agent.pointnav_approach_creep_m)
                if self.state is State.APPROACH else 0.0
            )
            # `.step` rather than `__call__`: the action is the same, but the
            # REASON is the only way to tell "I am there" from "the network gave
            # up" from "I stopped closing", and an arm whose whole question is
            # why approaches do not terminate cannot be read without it.
            with self.profiler.timeit("mover"):
                step = self.pointnav.step(
                    goal_xy, creep_below=creep,
                    stop_radius=float(self.cfg.agent.pointnav_arrival_m),
                )
            self.stats[f"pointnav_{step.reason}"] = (
                self.stats.get(f"pointnav_{step.reason}", 0) + 1
            )
            # How often the goal moved far enough to wipe the policy's
            # recurrent state.
            self.stats["pointnav_resets"] = int(
                getattr(self.pointnav, "n_resets", 0) or 0)
            return step.action
        return self.approach.follow_to(frame, goal_xy)
