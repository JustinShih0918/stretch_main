# `src/semantic_nav/` — Action-aware semantic traversability

Implements the idea from **arXiv:2310.08873** ("Interactive Navigation in
Environments with Traversable Obstacles Using Large Language and Vision-Language
Models") as a **Nav2-native layered-costmap** pipeline: the robot can plan a path
*through* obstacles that are physically traversable (curtains, grass) instead of
treating them as hard `LETHAL` obstacles.

This group is **not** part of the CI build subset (CI stays `--packages-up-to
bt_engine`). It is built in the **sim** image now, and is designed to be built in
the **deploy** image for the real robot later (see *Real robot* below).

## Data flow

```
/rgb ─► VLM perception node ─► /semantic_detection (SemanticDetection2D: bbox+label+traversable)
                                      │
/depth + /camera_info + TF ─► projection_node ─► /semantic_regions (SemanticRegionArray: ground polygons)
                                      │
/laser_scan ─► ObstacleLayer (marks curtain 254) ─► SemanticTraversabilityLayer
                                                       (overwrite 254 → traversable_cost inside polygon)
                                                     ─► InflationLayer ─► planner/controller ─► /cmd_vel
```

The LiDAR `ObstacleLayer` still marks the curtain `LETHAL`; our layer runs
**after** it and **before** `InflationLayer`, clearing only the cells inside the
camera-derived polygon of a class flagged `traversable`.

## Packages

### `semantic_traversability/` (C++, `ament_cmake`) — environment-agnostic
- **`SemanticTraversabilityLayer`** (`nav2_costmap_2d::Layer`, pluginlib):
  subscribes `SemanticRegionArray`, and in `updateCosts()` overwrites every
  master-grid cell that is inside a traversable polygon **and** currently
  `>= override_threshold` (default `254`) with `traversable_cost`.
  - Params: `enabled`, `polygon_topic` (`/semantic_regions`),
    `traversable_cost` (**`-1` = honor each region's `cost`; `>=0` forces this
    value**; `0` = FREESPACE, matching the paper), `override_threshold` (`254`),
    `transform_tolerance`.
- **`projection_node`**: deprojects the detection's pixels with `/depth` +
  `/camera_info`, transforms to `target_frame` (default `odom`), drops Z, and
  publishes the convex-hull ground footprint. Handles `32FC1`(m) and `16UC1`(mm)
  depth. Params: `depth_topic`, `camera_info_topic`, `regions_topic`,
  `target_frame`, `camera_optical_frame`, `pixel_step`, `min/max_depth`.
  It also includes confirmed-region memory for close-range detector drop-outs:
  after `region_confirmation_hits` spatially consistent detections within
  `region_match_distance_m`, the region is republished for `region_hold_sec`
  seconds (`<0` holds forever). Unconfirmed candidates expire after
  `pending_region_ttl_sec`, so one-frame false positives are not held.

### `semantic_perception/` (Python, `ament_python`) — sim / off-board
- **`locate_anything_node`**: default open-set VLM backend. Runs
  `nvidia/LocateAnything-3B`, parses generated labeled boxes, and publishes the
  existing `SemanticDetection2D` contract. Its checkpoint is cached at
  `/opt/locate_anything/LocateAnything-3B` when the sim image is built with
  `LOCATE_ANYTHING=YES`. Generated boxes have synthetic confidence `1.0`
  because LocateAnything does not emit calibrated detector scores.
- **`grounding_dino_node`**: open-set VLM. Builds its prompt from
  `config/semantic_targets.yaml` (or `~/instruction`), runs Grounding DINO on
  `/rgb`, publishes one `SemanticDetection2D` per box with the static
  `traversable` attribute. Torch + weights are installed only when the image is
  built with `GROUNDING_DINO=YES` (see `docker/sim/modules/install_grounding_dino.sh`);
  the node degrades gracefully (logs + no detections) if the model is absent.
- **`static_region_publisher`**: milestone-1 helper that publishes a
  hand-authored traversable polygon directly on `/semantic_regions`, so the
  costmap layer + planner can be validated **without** the VLM.
- **`image_rotation`**: shared right-angle RGB correction (`rgb_rotation`
  param, default `clockwise_90`, same convention as `vln_policy`). The Stretch
  head camera publishes its frame rolled sideways, which the open-set VLMs
  ground poorly on, so both perception nodes rotate the image **upright before
  inference** and map the detected boxes **back to raw-camera pixels** before
  publishing `SemanticDetection2D`. That back-mapping is what keeps
  `projection_node` correct: `/depth` and `/camera_info` are never rotated, so
  the published detection contract stays in camera pixel coordinates
  regardless of this setting. `detection_viz_node` takes the same parameter and
  publishes `/semantic_detection_viz` in the upright orientation the model saw.
  Use `rgb_rotation:=none` with a camera that already publishes upright.

### `vln_policy/` (Python, `ament_python`) — Vision-Language Navigation
- **`vln_agent_node`**: instruction-driven VLN agent with swappable model
  backends (StreamVLN via HTTP to `docker/vln/`, scripted `dummy`, `navila`
  adapter slot) and selectable executors (`cmd_vel` velocity bursts or
  relative waypoints to nav2 `navigate_to_pose`). Standalone demo:
  `./run_vln_demo.sh` (repo root). See [vln_policy/README.md](vln_policy/README.md)
  and the normative HTTP contract in [vln_policy/DESIGN.md](vln_policy/DESIGN.md).

### Interfaces
Added to the vendored `src/common/btcpp_ros2_interfaces/`:
`SemanticDetection2D`, `SemanticRegion`, `SemanticRegionArray`, `VlnStatus`.

## Run (Isaac Sim)

Build the sim image (Compose enables LocateAnything by default), then inside
the container:

```bash
# Milestone 1 — no ML: prove the layer + planner with a static polygon.
ros2 launch stretch3_navigation semantic_navigation.launch.py perception:=static
#   -> watch the curtain cells clear in RViz; send a goal behind it; a path
#      appears. Set semantic_traversability_layer.enabled:=False to see it fail.

# Full pipeline — LocateAnything (default), with the RViz scene.
ros2 launch stretch3_navigation semantic_navigation.launch.py rviz:=true

# Require 3 matching detections before holding a region forever.
ros2 launch stretch3_navigation semantic_navigation.launch.py \
    region_confirmation_hits:=3 region_hold_sec:=-1.0

# Optional Grounding DINO backend.
ros2 launch stretch3_navigation semantic_navigation.launch.py \
    perception:=dino

# Camera already upright (no sideways roll to correct).
ros2 launch stretch3_navigation semantic_navigation.launch.py \
    rgb_rotation:=none
```

### RViz scene (`rviz:=true`)

`stretch3_navigation/rviz/semantic_navigation.rviz` (fixed frame `world`)
shows, in one window:

| Display | Topic | What it tells you |
|---|---|---|
| **RGB + Detections** | `/semantic_detection_viz` | upright camera frame with the VLM boxes + `TRAVERSABLE`/`BLOCKED` labels |
| **Global Costmap** | `/global_costmap/costmap` | curtain cells cleared to `traversable_cost` by the semantic layer |
| **Local Costmap** | `/local_costmap/costmap` | what the controller actually drives against |
| **Robot Frames (TF)** + **Robot Footprint** | `/tf`, `/local_costmap/published_footprint` | robot pose and body outline |
| **LaserScan** | `/laser_scan` | the LETHAL marks the semantic layer overrides |
| **Global Plan** / **Goal Pose** | `/plan`, `/goal_pose` | the path through the traversable region |

Both costmap displays subscribe **transient-local** (as nav2's own
`nav2_default_view.rviz` does); a volatile subscription can sit empty waiting
for the next periodic publish. An empty **RGB + Detections** panel means
`detection_viz_node` is publishing nothing at all — it republishes the plain
(rotated) frame even when the VLM finds nothing, so check `/rgb` is flowing
and the node is alive, not the detector.

A `RobotModel` display is included but **disabled**: nothing in this stack
publishes `/robot_description` (Isaac Sim supplies TF only). Enable it if you
run a `robot_state_publisher` alongside. Point `rviz_config:=/path/to.rviz` at
your own file to override the scene.

Backend-specific model parameters live in
`semantic_perception/config/locate_anything_params.yaml` and
`semantic_perception/config/grounding_dino_params.yaml`. To test a local
variant without editing the package config, pass
`perception_params_file:=/path/to/custom_backend_params.yaml`.

Switch the active landmark by publishing to `/semantic_instruction`, including
through the `SetSemanticInstruction` behavior-tree node.

### Tuning note
The default planner/controller (NavFn + DWB) are intentionally left unchanged.
NavFn barely weights intermediate costs, so the default `traversable_cost: 0`
(FREESPACE) is what makes it reliably cross. To get true *risk-aware* routing
(cross only when the detour is longer), raise `traversable_cost` (~100) **and**
switch the planner to a cost-aware one (e.g. Smac2D) — out of scope here.

## Real robot (Stretch 3 — later phase)

The C++ `semantic_traversability` package is environment-agnostic and is the
reusable core. To bring this up on the real robot:

1. **Build it in the deploy image**: add `semantic_traversability` (and, if the
   VLM runs on-robot, `semantic_perception`) to the `--packages-up-to` list in
   `docker/deploy/docker-compose.yaml`; add any system deps to
   `docker/deploy/Dockerfile`.
2. **Add the layer to the deploy params**: insert `semantic_traversability_layer`
   after the obstacle/voxel layer and before inflation in
   `src/deploy/stretch_nav2/config/nav2_voxel_params.yaml` (via a params overlay —
   do **not** edit the vendored file in place). The on-robot stack localizes in
   `map` (AMCL) rather than sim's ground-truth `world`; set `target_frame`
   accordingly (`map` or `odom`).
3. **Perception**: run either VLM node against the robot's
   RealSense D435i (already in the Stretch URDF) — likely **off-board** given
   on-robot GPU limits — or consume detections from
   [`hello-robot/stretch_ai`](https://github.com/hello-robot/stretch_ai) and adapt
   them into `SemanticDetection2D` (same topic contract, so the projection node
   and costmap layer are unchanged).
4. **Frames/calibration**: verify the camera optical-frame convention and the
   `map → odom → base_link → camera_*` TF chain; set `camera_optical_frame` on the
   projection node if the depth image's `header.frame_id` is not the optical frame.
```
