# Vendored: Object_scene_graph

A snapshot of https://github.com/JustinShih0918/Object_scene_graph, copied here so it can be
modified for stretch_main without touching the original repo. **This copy is the one to edit.**
It does not track upstream automatically.

| | |
|---|---|
| Source | `JustinShih0918/Object_scene_graph`, branch `feat/ros2` |
| Commit | `44c1ec6a4ff0240f145d64b122b6c6ab4de9e1f8` |
| Copied | 2026-10-19, via `git archive` (no history) |
| Cherry-picked | `e4cd164`, `29fb05e` from `feat/mfv2-benchmark` (2026-11-12; see the last entry below) |

## Local changes vs. upstream

- Added `configs/experiment/isaac_hospital_map.yaml` and `configs/ros2/isaac_hospital.yaml`
  (Isaac hospital preset, recovered from the sim container used for the 2026-09-23 demo).
- Added `COLCON_IGNORE`: this is a plain Python project, not a ROS package (the bridge runs
  via `scripts/ros2/bridge.sh`), and its ASCENT third_party tree must not be picked up by colcon.
- Dropped `.claude/settings.local.json` (machine-local permissions); `.claude/skills/` kept.
- `relative_work/ascent` stays a git submodule (Zeying-Gong/ascent, pinned at `8f7bbf9`),
  registered in stretch_main's `.gitmodules`.
- Pinned `isaac_hospital_map` in `tests/unit/golden/experiment_fingerprints.json` (regenerated;
  that one line is the only change).
- **Removed what stretch_main doesn't use** (all still upstream if needed):
  - `docker/`: OSG runs in stretch_main's sim container instead. `docker/ascent-requirements.txt`
    moved to `requirements/ascent-requirements.txt` (read by `install_osg_envs.sh`).
    **The Thor deployment came back later** (2026-11-18), outside this directory: upstream's
    `Dockerfile.thor`, `Dockerfile.bridge`, `compose.thor.yaml` (as `compose.yaml`),
    `entrypoint.thor.sh` and `thor-model-requirements.txt` are in stretch_main's
    `docker/thor/`, building from this directory (`thor-model-requirements.txt` and the
    entrypoint arrive as the extra build context `thor`); `docker/stretch/nav2_params_slam.yaml`
    is stretch_main's `docker/deploy/nav2_params_slam.yaml`; the DDS file is stretch_main's own
    `docker/cyclonedds-eth.xml`, no longer a copy. `docs/THOR.md` and `docs/ROS2.md` point at
    those paths.
  - `tests/unit/test_docker_build_context.py`: it only tested the removed Dockerfiles.
  - `scripts/legacy/`: one-off probes, which upstream's own rule says to delete.
  - `scripts/video/` and `docs/VIDEO.md`: the narrated paper video. This is not our demo
    recorder, which is `scripts/scene_graph_demo/`.
  - `docs/DEMO_FLOW.md`: the Jetson Thor two-floor demo checklist, which depends on the removed
    Thor compose files.
- Docs: a "vendored copy" note at the top of `README.md`, `docs/SETUP.md` and `docs/USAGE.md`
  (their docker instructions describe upstream and don't apply here). `CLAUDE.md`'s
  Environment section now describes the sim container. Links to removed files are dropped.

Unit tests after these changes: 1493 passed. One known error remains: `test_room_classifier`
needs the gitignored `data/priors/place365_room_map.json`.

- **Single-floor, single-policy (2026-10-19).** Multi-floor support and the ASCENT
  transcription were removed; what is left is the dynamic scene graph on one storey.
  - Removed the ASCENT transcription: `src/navigation/`, `agent/ascent_agent.py`,
    `agent/world_model.py`, the `ascent`/`ascentnav` policies, the ASCENT obstacle-map
    prior (`ycb.obstacle_map_*`, `eval/obstacle_map_viz.py`), RedNet, the GroundingDINO
    stair client, and their presets, scripts and tests. `nav_agent` is the only policy
    and the default (`configs/agent/s71.yaml`). Kept, because `nav_agent` uses them: the
    `ascent` selector/ranker, the served-model clients, the `ascent` conda env and the
    `relative_work/ascent` submodule.
  - Removed multi-floor: `mapping/floors.py`, `floor_stack.py`, `portals.py`, `stairs.py`,
    `agent/floor_policy.py`, `climb.py`, `stair_sense.py`, `exploration/floor_planner.py`,
    `perception/stair_seg.py`, `eval/floors.py`, the `floor` config group and every
    climb/stair/storey field, `/osg/floor` and `scripts/ros2/switch_floor.sh`, the
    virtual-storey pose lift, the wait-for-carry action, cross-floor YCB episode selection,
    the floor prior and floor LLM prompt, and the floor fields of `episodes.jsonl`.
  - New `mapping/map_layer.py`: one costmap, room segmenter, planner and value map, and the
    ground-plane height latched on the first frame (or restored from a snapshot). The scene
    graph is `room -> container -> object`; `containers_floor_relative` now measures surface
    tops from that latched height.
  - `graph/map_store.py` writes schema v3 (one layer + `floor_y`); it still reads v1 and
    single-floor v2 snapshots, and refuses a multi-floor v2 one.
  - Presets: floor-only presets deleted (`mf5_*`, `ycb_dynamic_multifloor`, the
    `ycb_authored_15_fused*` ladder, `floor_schedule_habitat`, the stair/climb archive rungs,
    `dualmap_osg_unified` and others that became duplicates); `ycb_authored_15` now composes
    on `ycb_dynamic_best`; floor keys stripped from the rest. Goldens regenerated.
  - Docs: `docs/history/` multi-floor logs deleted (`MULTI_FLOOR`, `CROSS_FLOOR_CLIMB`,
    `CROSS_ANCHOR_*`, `FUSED_PIPELINE`, `FUSION_RESULTS*`); living docs updated; the paper
    and the remaining history describe upstream and carry a note saying so.
  - Unit tests after this change: 788 passed, 8 failed, 7 skipped. The 8 failures existed
    before it (7 need `habitat_sim`; `test_object_distance_rule` fails at upstream too).

- **Condition inspection after the search (2026-11-10)**, for stretch_main's
  fire-extinguisher demo (`scripts/extinguisher_demo/` at the repo root). Off unless a
  preset turns it on.
  - New `src/osg/inspection/`: `views.py` (`TargetViewBuffer`, the best views of the target
    the search passes on its way in: whole over cut-off, close over far), `condition.py`
    (`ConditionInspector`: one OpenAI-compatible VLM call over those crops plus a context
    frame, JSON verdict good/bad + issues; reasoning that exhausts `max_tokens` is asked
    again without thinking and marked `fallback`; a failed call, an empty answer or a
    non-verdict is `unknown` with the reason, never a default) and `card.py`.
  - New `inspection` config group (`core/config/inspection.py`, `configs/inspection/{off,nim}.yaml`;
    `off` in `config.yaml`'s defaults). `nim` is NVIDIA's Nemotron 3.5 Super VL on
    build.nvidia.com and reads `NVIDIA_API_KEY`.
  - New preset `configs/experiment/isaac_extinguisher_inspect.yaml`: `isaac_hospital_map`
    with target `fire extinguisher`, `inspection=nim`, an extinguisher checklist, the
    terminal stop at 1.4 m instead of 0.4 m so the last look frames the whole object, and
    crops shaped for a small, tall object. Two new keys, both off by default:
    `crop_min_aspect` (0.75 here) widens a tall, narrow crop toward square so the floor
    beside the object stays in it, and `crop_upscale_px` (768 here) enlarges a small crop
    before it is sent. (The demo was first built for a
    fire hydrant, `isaac_hydrant_inspect`; that preset was replaced the same day.)
  - `scripts/run_robot.py`: when `inspection.enabled`, collects target views through the
    debug stream, and once the episode has ended in DONE turns to face the target
    (`inspection.face_target`, `views.bearing_deg`), takes `final_frames` fresh frames,
    asks the VLM, writes `inspection.json` / `inspection_view*.jpg` / `inspection_card.jpg`
    into the run directory and publishes progress on `/osg/mission`. An inspection that
    raises is printed and published as `unknown`; a map run still saves its map.
  - `sim/ros2_env.py`: `Ros2Env.observe()`, a fresh frame without acting.
  - ROS bridge: a `publish_text` op (latched `std_msgs/String`, `/osg/*` only) in
    `bridge_node.py`, `transport.py`, `wire.py` and `loopback.py`.
  - `eval/debug_stream.py`: publishes the scene graph as JSON on `/osg/scene_graph_json`
    (room -> container -> object, robot, trajectory; ROS coordinates) and takes an
    `on_detections` hook. Both guarded like the existing views.
  - Goldens regenerated. `config_snapshot.json` gains the `inspection.*` keys;
    `experiment_fingerprints.json` changes for every preset only because each now carries
    the (disabled) `inspection` group: with that key removed, all 48 existing presets hash
    exactly as before (checked 2026-11-10, and again after `crop_min_aspect` and
    `crop_upscale_px` were added). `isaac_extinguisher_inspect` is added.
  - New `tests/unit/test_inspection.py` (27 tests). Unit suite after this change: 825
    passed, 5 skipped; `test_room_classifier` still errors at collection without
    `data/priors/place365_room_map.json`.

- **A detection mostly beyond depth range places no track (2026-11-10)**, opt-in. New
  `scene_graph.min_valid_depth_frac` (default 0 = off, so every existing preset composes and
  runs as before): `ObjectLayer.update` skips a detection when less than that fraction of
  its mask has a depth reading, and counts it as `funnel["obs_beyond_range"]`. Depth past
  `eval.depth_max_m` reads 0, so a far object's mask carries only the depth of whatever edge
  is in front of it. In the extinguisher demo's map run a 0.66 m extinguisher seen 8 m off
  past a reception desk became a track on the desk corner, 7.6 m short of it, and the run
  stopped there with no usable view. `isaac_extinguisher_inspect` sets it to 0.5. Plumbed
  through `nav_agent.py`; three tests in `tests/unit/test_terminal_stop.py`. Unit suite: 828
  passed, 5 skipped (`test_room_classifier` still needs its data file). Goldens regenerated:
  `config_snapshot.json` gains the key, and with it removed every other preset's fingerprint
  is unchanged.

- **Upstream's control-loop speedups (2026-11-12).** `feat/ros2` has not moved since `44c1ec6`;
  upstream's newest branch is `feat/mfv2-benchmark` (`ef5fdd6`, 2026-11-04), 62 commits on.
  Nearly all of those are multi-floor (storey belief, stair routes, climbing), the Benchmark v2
  campaign and its stop-gate ablations, which this single-floor copy does not carry. Two were
  taken, cherry-picked onto this copy so git merged them 3-way:
  - `29fb05e` perf: bit-identical hot-path speedups. One cached mask scan per detection
    (`core.geometry.mask_pixels` / `det_ellipse`), the ellipsoid refine residual over all views
    at once (`objects.optimization`, `Ellipsoid.project` with a precomputed Q and P), a
    vectorised near-pair prefilter in `objects.linking.relink`, device-resident masks in
    `perception.region_proposer`, and CLIP preprocessing as a device-side lookup table in
    `perception.feature_encoder`. Upstream measured the control loop at 574 -> 340 ms.
    Conflicts resolved by hand: `graph/scene_graph.py` (the bincount room centroids, the
    container-geometry memo and the indexed nearest-room search were ported onto this copy's
    single-floor `rebuild`/`_nearest_room`; upstream's are keyed by floor), `objects/linking.py`
    (imports). `feature_encoder.py` is upstream's file as of `29fb05e`, so it also brings
    `d29b70b`'s `_reparameterize` (timm's fold when the `mobileclip` package is absent; with it
    installed, as in the sim container, nothing changes). `tests/unit/test_hot_path_equivalence.py`
    is upstream's with its two scene-graph tests moved to the single-floor API.
  - `e4cd164` latency: p90/p99 in `core/profiler.py` and `timing.csv`,
    `scripts/analyze_latency.py`, and a "Measuring latency" section in `docs/THOR.md`.
  - Local, not upstream: `scripts/run_robot.py` now writes `timing.csv` into the run directory.
    `analyze_latency.py` says robot runs write the same profiler output as simulator runs,
    but `run_robot.py` (here and upstream) never wrote it, so a robot run had nothing to read.
  - Unit suite: 842 passed, 5 skipped (`test_room_classifier` still needs its data file); all
    14 new tests pass, including the bit-identical device-preprocess one under torch. No
    config or golden changed.
  - Measured in Isaac (2026-11-13), one `isaac_extinguisher_inspect` search take each on the
    same stack, back to back, pre-merge code then merged (`scripts/analyze_latency.py` on
    `outputs/20261113_002619` and `_002754`; the GPU was shared with Isaac and another job):
    scene_graph 52.7 -> 9.3 ms, object_layer 77.8 -> 35.7 ms, region_proposal 400 -> 362 ms
    per call; control loop mean 469 -> 354 ms (2.13 -> 2.82 Hz), median 337 -> 206 ms. Both
    found the extinguisher and judged it GOOD. One take per arm, so the loop totals are
    noisy; the two CPU stages are the firm part.

- **Instances: "fire extinguisher #1", "#2" (2026-11-13)**, for the live demo with more than one
  extinguisher. Off unless a preset lists the class in `scene_graph.instance_labels`
  (`isaac_extinguisher_inspect` does). Measured first in Isaac with two extinguishers 2.1 m apart
  on one wall, #1 then moved 3.3 m to a row of chairs and a search sent for an extinguisher
  (`outputs/20261113_010555`): the robot drove to #1's empty place, saw #2 two metres beyond it,
  counted that as the target, stopped at the empty place and inspected #2 (GOOD) as the one it
  had been sent to.
  - New `objects/instances.py`: `InstanceRegistry` binds an index to the track that is that
    object, every keyframe: a track within `instance_fragment_m` (0.4) of an instance's and
    never detected beside it in one frame is a fragment of it; an instance whose track (and
    fragments) the presence filter disbelieves (< `presence.min_presence`) is missing; a
    believed, unexplained track of the label is claimed by a missing instance (a "moved"
    event, nearest first); a new index is given only when no known instance could be it
    (none missing, all seen this run); a believed instance is never re-bound. Also
    `detection_xyz` (a detection's world point from depth), `belongs` (a sighting is this
    track's object: within `instance_match_m`, no nearer another instance) and
    `split_target` ("fire extinguisher #2" -> index 2).
  - Agent (`nav_agent.py`, `candidate.py`): `agent.target_instance` (0 = any). The candidate
    check keeps only the named instance's track, or -- once it is missing -- unexplained
    tracks of the label; never another instance's. `_best_target_detection`, which stops the
    approach, ends the close look and protects a stop from the absence reading, counts only
    detections that `belongs` to the committed track. An arrival at an instance the presence
    filter already disbelieves is abandoned even when the arrival reading has no frame that
    would have shown it (`_instance_gone_verdict`).
  - Instance labels are never linked (`relink(skip_labels=...)`, `ObjectLayer.unlinked_labels`):
    two extinguishers side by side are co-observed, and linking reports both at the midpoint.
  - Instance labels are associated in 3D (`DataAssociator.associate(xyz_labels=...)`,
    `scene_graph.instance_assoc_gate_m` 0.45): a detection placed by depth joins a track of
    its label within the gate, nearest pair first, or starts its own; such tracks are never
    matched in the image. Measured without it (outputs/20261113_013326): #1 put down 0.65 m
    beside #2 took #2's track in the image, #2 started a new one, and the indices swapped.
  - "Co-observed" (the fragment rule's two-objects test) means two detections in one frame
    whose image ellipses are apart (> 1.2 summed std along the line between them). A half
    hidden extinguisher seen along the wall behind another gave a second, overlapping track
    0.23 m from #2, which the frame-id-only test indexed as "#3" (outputs/20261113_012324).
  - An approach to an instance-class track it has not once seen stops at `terminal_stop_m`
    from the track to take its absence reading (close look, then `_absence_at_arrival`),
    once per approach (`ApproachPolicy._at_standoff_unseen`). With Nav2 the goal is the
    object's centre and the stop is its depth; with the object moved there is no depth, and
    the Stretch drove on to its old place 6 cm from a wall, pushed into it and fell over
    (outputs/20261113_013913, base_link's up axis horizontal in Isaac).
  - The inspection backs out to `terminal_stop_m` and looks again when it has no usable view
    and the stop overshot (one step can cover 1.5 m at the sim's speed; a stop at 0.95 m left
    the extinguisher's base below the frame, outputs/20261113_015719). This needed a
    `move_backward` action on the base (`bridge_node.py`, metered on odometry like
    `move_forward`; `loopback.py`). If the mission's instance is missing and the detector then
    names an extinguisher at the stop that no instance holds, the stopped-at track is claimed
    for it (`InstanceRegistry.claim`) -- the track the robot followed there may be the region
    proposer's, which the registry's own rules never bind. After such a claim the scene-graph
    JSON is published again (`DebugStream.publish_graph`) and the inspection re-announced by
    name: the display had been told "fire extinguisher ?" and never saw the move. A stop on a
    track no rule binds (the region proposer's) within `instance_fragment_m` of the mission's
    instance, and nearer it than any other, is that instance (`InstanceRegistry.same_as`): #1
    claimed at the chairs, the stop 0.15 m away on a proposer track, reported as "?" with a
    false WARNING (outputs/20261113_175735).
  - An instance remembers where it was put (its anchor: indexed, moved to, re-anchored, or its
    track's centre when a map loads) and is re-bound, as the one exception to "never re-bound",
    when its believed track has wandered more than half `instance_fragment_m` (0.2 m) off that
    place while another believed track of its label that no other instance holds sits within
    `instance_fragment_m` of it and nearer it (`InstanceRegistry._undrift`, a "reanchored"
    event with `drift_m`). Also seen with #1 at the chairs: #2's track pulled 0.46 m toward the
    cart beside it and #2's own detections, on a track 0.24 m from its place, indexed "#3"
    (outputs/20261113_162238). Recorded
    take, #1 put 0.65 m from #2 (outputs/20261113_154556): #1 seen alone joined #2's track
    within the 3D gate, the refit pulled that track 0.45 m toward #1, #2's own detections
    started a fresh track at #2's place, missing #1 claimed it, and #2 was inspected as #1
    (GOOD). The same placement had passed earlier that day (outputs/20261113_015031).
    The track an instance is taken off is retired (never indexed, claimed, offered as a
    candidate, or a fragment): it holds both objects' detections and sat between them, where
    it was indexed "#3" (outputs/20261113_160353).
  - `scene_graph.instance_max_height_m` (0 = off; 1.0 in `isaac_extinguisher_inspect`): a track
    of an instance class centred higher above the floor is never indexed, claimed, offered as a
    candidate (`InstanceRegistry.plausible`, also on the appearance pick) or taken as the
    instance at the stop. Moving #1 to seeded far spots (`scripts/extinguisher_demo/
    far_placements.py`), the hospital's red FIRE EXTINGUISHER wall signs were named
    extinguishers from across the lobby (tracks at 1.36-1.51 m, every real one 0.20-0.45 m):
    three of five runs went to one, and one claimed it as #1 moved (outputs/20261113_184240).
  - `ros2.nav_wedged_s` (0 = off; 6 s in `isaac_extinguisher_inspect`): `Nav2Driver` backs out
    (two `move_backward`, a turn, alternating sides) and posts the goal again when the base has
    neither moved 0.1 m nor turned 15 deg for that long with a goal outstanding. In two of the
    five far runs the Stretch drove into a supply cart and a height scale whose low frames
    Nav2's costmap misses and sat there 200 steps, every goal accepted and impossible
    (outputs/20261113_182013, _182916). `run_robot.py` prints the count. Goldens regenerated:
    the two keys; with them removed every preset's fingerprint is unchanged (49 of 49).
  - A STOP the robot can take back. `Ros2Env.attempt_scored` asks a `stop_check(frame)` the
    run installs (else False, as before), `Ros2Env.halt()` cancels the Nav2 goal without
    ending the episode. `run_robot.py` installs one when the inspection is on and
    `eval.attempts` > 1: it takes the look the inspection always took (`_look_at_stop`, split
    out of `_inspect`: face the track, a few frames, back off if too close; "checking" on
    /osg/mission) and, if the detector names no target there for that track, the stop is a
    failed attempt: `eval/attempts.py` re-arms the agent and, with
    `verification.failed_attempt_disables_place`, strikes off the place ("resumed" on
    /osg/mission). `_inspect` reuses the look when the stop was checked. A far-placement run
    stopped at a shelf of red-labelled binders only the region proposer had called an
    extinguisher (proposal_sim 0.29), found no view of one and ended the mission there
    (outputs/20261113_191403). `isaac_extinguisher_inspect` sets `eval.attempts: 4`; its
    fingerprint is the only one that changes.
  - A robot that fell over ends the run. `Ros2Env.abort(reason)` makes `episode_over` true and
    keeps `abort_reason`; `run_robot.py` calls it when the camera is more than 0.5 m off its
    standing height (`NavAgent._off_plane_m`) two steps running, and says why in the log and
    on /osg/mission. Isaac's Stretch clipped a wheelchair at step 69 under Nav2, went over
    backwards, and spent its other 230 steps "wedged" with the camera on the ceiling
    (outputs/20261113_191906).
  - An approach follows the registry (`CandidatePolicy.follow_registry`, called after every
    registry update in `NavAgent._on_keyframe`). Approaching for #k a track no instance held,
    the registry answering a keyframe later retargets the approach onto #k's believed track,
    or, when the committed track has become another instance's, drops the approach to
    EXPLORE with nothing held against that track. Three far-placement runs stopped at their
    commit after the registry had moved on (outputs/20261113_200152: the track at #2's place
    became #2's two steps after the commit and #2 was inspected for the #1 mission;
    _202629: #1 claimed on the commit then re-anchored onto its neighbour; _203830: #1
    claimed on a track 0.56 m from the commit). As a backstop `run_robot.py` gives no
    verdict at a stop on another instance's track ("no verdict for the mission").
  - `inspection.brighten_below` / `brighten_to` (0 = off; 0.35 / 0.45 in
    `isaac_extinguisher_inspect`): an image whose median CIELAB lightness is under the bar is
    lifted to `brighten_to` by a gamma on L before the model sees it
    (`inspection/condition.py _brighten`). The corroded extinguisher, found in unlit spots,
    was judged "good" from crops at median L 0.08 and 0.14 ("the lighting is dim, making it
    hard to see ... rust"); the same images lifted were judged "bad". Lifted, three earlier
    verdicts kept their answer (BAD 0.85 / GOOD 0.90 / BAD 0.80) and the intact one darkened
    to L 0.08 and 0.14 stayed GOOD. Goldens regenerated: the two keys; with them removed every
    preset's fingerprint is unchanged (49 of 49).
  - The inspection's whole-frame context image outlines the inspected object in green and the
    prompt says to judge only that one (`inspection/condition.py`): shown both, the VLM
    described the other extinguisher's rust as the inspected one's (outputs/20261113_013326).
  - `graph/map_store.py` saves `instances` (an additive key) and restores them; a map saved
    before them is indexed on load in creation order.
  - Inspection (`inspection/views.py`, `scripts/run_robot.py`): with a frame, every target
    detection is a view placed in the world, kept per 0.75 m place so one extinguisher's
    views cannot evict the other's; the views judged are only those `belongs` accepts for
    the track the robot stopped at. `DebugStream` passes the frame to its detection hook
    (a hook without a `frame` parameter is still called as before).
  - Display: the scene-graph JSON names objects ("fire extinguisher #1", "?" while
    undecided) and lists the instances; RViz labels use the names; only the named instance
    is the target. `run_robot.py` reads "#k" off `ros2.target`, prints indexing and moves as
    they happen, and writes `instances`, `instance_events` and `target_instance` into the
    episode record and the instance's name into `inspection.json`.
  - New `tests/unit/test_instances.py`; `test_eval_uid.py` exempts `target_instance` (a
    mission parameter, recorded per episode). Goldens regenerated: three
    `scene_graph.instance_*` keys and `agent.target_instance`; with them removed every
    preset's fingerprint is unchanged (49 of 49).
  - `configs/experiment/stretch3_extinguisher.yaml` (new): the fire-extinguisher demo on the
    real Stretch, both passes by `ros2.map_mode` -- `stretch3_map` plus `inspection: nim`,
    `scene_graph.instance_labels: [fire extinguisher]`, `instance_max_height_m`,
    `min_valid_depth_frac`, `eval.attempts: 4` and the inspection crop/checklist from
    `isaac_extinguisher_inspect`, without that preset's Isaac-tuned stop distance, wedge
    back-out and brightening. Started by stretch_main's control panel (`control_panel/osg_runner`).
    Goldens: one fingerprint added, none changed.

- `scripts/serve_perception.sh`: stopping the servers now reaps them by what they are, not
  only by killing the tmux session. tmux daemonizes, so the five `model_api.*_out` servers
  are not in the process group of whoever ran the script: killing the session -- or
  stretch_main's control panel stopping its `perception` unit -- left them running and
  holding :13182-:13186, and the next start then died one line into Flask with "Address
  already in use" while the panel still showed the unit up. `--stop` and the start path both
  `pkill -f 'model_api\.[a-z_]*_out'` (SIGTERM, then SIGKILL after 10 s), and a start whose
  ports are still answering afterwards -- something this script cannot kill -- exits 1 with
  the ports named instead of launching servers that cannot bind.
- **The real robot stops 1 m from the target** (2026-11-21). `stretch3_map` (so also
  `stretch3_search` / `stretch3_extinguisher`) sets `agent.terminal_stop_m: 1.0`,
  `terminal_engage_m: 1.5` and `success_distance: 1.0`; it inherited the sim-tuned 0.4 / 1.0 /
  0.1 from `agent/s71.yaml`. `isaac_hospital_map`, which builds on `stretch3_map`, pins the old
  three so the Isaac presets compose exactly as before. Fingerprints of the three `stretch3_*`
  presets regenerated.
- **Keyframes twice as often on the real robot** (2026-11-21): `stretch3_map` sets
  `scene_graph.keyframe_trans_m: 0.15` / `keyframe_rot_deg: 15.0` (default 0.25 / 30);
  `isaac_hospital_map` pins the defaults. Fingerprints regenerated.

Add an entry above whenever you change something here, so a later re-sync from upstream
knows what to keep.

## Pulling upstream changes later

This copy has diverged too far (single-floor) for a whole-branch `git apply`. Replay it on
upstream's history in a scratch clone and cherry-pick only what is wanted, so each pick is a
real 3-way merge:

```bash
git clone --filter=blob:none https://github.com/JustinShih0918/Object_scene_graph.git osg && cd osg
git log --oneline 29fb05e..origin/<branch>             # what is new since the last pick
git checkout -b vendored 44c1ec6
git ls-files | grep -v '^relative_work/ascent$' | xargs git rm -q --
(cd <stretch_main>/src/semantic_nav/object_scene_graph && git ls-files -co --exclude-standard -z \
   | grep -zv '^relative_work/ascent$' | tar --null -T - -cf -) | tar -xf -
git add -A && git commit -m "stretch_main copy"
git cherry-pick -x <commit>...                         # resolve against the single-floor API
git diff --binary HEAD~<n> HEAD > /tmp/osg.patch       # n = commits picked
git -C <stretch_main> apply --directory=src/semantic_nav/object_scene_graph /tmp/osg.patch
```

Then log the picks here and in the table above.

Running it with Isaac Sim: see [scripts/scene_graph_demo/README.md](../../../scripts/scene_graph_demo/README.md).
