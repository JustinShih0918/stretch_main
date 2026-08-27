# Action-Aware Semantic Traversability — Design & Test Guide

Design notes, pipeline internals, risks, and a step-by-step test procedure for
the `src/semantic_nav/` feature. Companion to the user-facing
[README.md](README.md). Implements arXiv:2310.08873 ("Interactive Navigation in
Environments with Traversable Obstacles Using LLMs and VLMs") as a Nav2-native
layered-costmap pipeline.

---

## 1. Goal & how it maps to the paper

The robot should plan a path **through** physically-traversable obstacles
(curtains, grass) instead of treating them as hard `LETHAL` cells.

| Paper element | Our implementation |
|---|---|
| LLM parses instruction → `{landmark, action-aware attribute}` | **Static config** `semantic_targets.yaml` (`curtain: traversable`). `SetSemanticInstruction` BT node + `~/instruction` topic are the hook where an LLM plugs in later. |
| VLM → landmark bounding boxes | `locate_anything_node` (default) or `grounding_dino_node`, open-set and text-promptable. |
| LiDAR points projected into image, segmented traversable/untraversable | We **don't** segment the cloud. RGB-D `projection_node` turns the detection into a ground polygon; the costmap layer clears the LiDAR-marked cells inside it. |
| Action-aware costmap (traversable pts → cost 0) | `SemanticTraversabilityLayer` overwrites LETHAL→`traversable_cost` (default 0) inside the polygon. |
| A* finds a feasible path | Unchanged Nav2 planner (NavfnPlanner). |

**Why the design differs:** the paper segments the point cloud *before* the
costmap. We keep the standard LiDAR `ObstacleLayer` (it still marks the curtain)
and add a *post-process* layer that clears it. This is modular, requires no
changes to the sensor pipeline, and is reusable across sim and the real robot.

---

## 2. Pipeline (nodes, topics, frames)

```
                 ┌──────────────────────┐
   /rgb ────────►│  grounding_dino_node │ (Python, GPU)
                 │  prompt = curtain    │
                 └─────────┬────────────┘
                           │ /semantic_detection   (SemanticDetection2D)
                           ▼
 /depth ───────► ┌──────────────────────┐
 /camera_info ──►│   projection_node    │ (C++)
   TF (cam→odom) │  deproject + hull    │
                 └─────────┬────────────┘
                           │ /semantic_regions     (SemanticRegionArray, frame=odom)
                           ▼
 /laser_scan ──► ObstacleLayer ─► SemanticTraversabilityLayer ─► InflationLayer
                 (marks 254)      (254→traversable_cost in poly)   (inflate rest)
                           │ master costmap
                           ▼
                 Nav2 planner (NavFn) / controller (DWB) ─► /cmd_vel
```

### Topic contract

| Topic | Type | Producer | Consumer | QoS |
|---|---|---|---|---|
| `/rgb` | `sensor_msgs/Image` | Isaac Sim | grounding_dino_node | sensor |
| `/depth` | `sensor_msgs/Image` (32FC1 m or 16UC1 mm) | Isaac Sim | projection_node | sensor |
| `/camera_info` | `sensor_msgs/CameraInfo` | Isaac Sim | projection_node | sensor |
| `/semantic_detection` | `SemanticDetection2D` | grounding_dino_node | projection_node | default (10) |
| `/semantic_regions` | `SemanticRegionArray` | projection_node (or static pub) | costmap layer | **transient_local** (latched) |
| `~/instruction` | `std_msgs/String` | SetSemanticInstruction / CLI | grounding_dino_node | 1 |

> `transient_local` on `/semantic_regions` matters: the costmap layers subscribe
> with matching durability so a late-joining costmap still receives the last
> published region.

> `SemanticDetection2D` boxes are **always in raw `/rgb` pixel coordinates**.
> The perception nodes' `rgb_rotation` parameter (default `clockwise_90`, to
> upright the sideways Stretch head camera) applies only to the image handed to
> the VLM; the resulting boxes are rotated back before publishing, because
> `projection_node` deprojects them against the unrotated `/depth` +
> `/camera_info`. Any new detector must publish in the same raw-pixel frame.

### Frames

- Global costmap frame: **`world`**; local costmap frame: **`odom`**.
- projection_node publishes polygons in **`target_frame` (default `odom`)**.
- The layer transforms `polygon.header.frame_id → costmap global frame` via TF
  each cycle, so publishing in `odom` works for both costmaps (TF has
  `world↔odom`).

---

## 3. Component internals

### 3.1 VLM nodes (`semantic_perception`)
- `locate_anything_node` is the default launch backend. It loads the cached
  `nvidia/LocateAnything-3B` checkpoint, detects configured target categories,
  parses `<ref>label</ref><box>...</box>` output, and publishes confidence
  `1.0` because the generated format has no detector score.
- Both learned backends subscribe to private `~/instruction`; the semantic
  navigation launch remaps this to the backend-independent
  `/semantic_instruction` topic.
- Builds the caption from `semantic_targets.yaml` keys joined by `" . "`
  for Grounding DINO, or uses the target list for LocateAnything.
- For each predicted box (normalized `cx,cy,w,h`), converts to pixel
  `x,y,width,height`, matches the phrase back to a known label (substring), and
  sets `traversable` from the config map.
- **Lazy import**: torch / groundingdino / cv_bridge are imported on first image.
  If unavailable, it logs an error and publishes nothing — the rest of the
  pipeline still runs (use `static_region_publisher` instead).
- Common wiring params (`rgb_topic`, `detection_topic`, `targets_file`) are set
  by `semantic_navigation.launch.py`. Backend/model params are loaded from YAML:
  `locate_anything_params.yaml` or `grounding_dino_params.yaml`. Use the launch
  argument `perception_params_file:=...` to point at a custom backend config.

### 3.2 `projection_node` (semantic_traversability)
- Caches latest `/depth` + `/camera_info`; on each detection:
  1. Look up TF `optical_frame → target_frame`.
  2. For pixels in the bbox (stride `pixel_step`, default 4) with valid depth in
     `[min_depth, max_depth]`, deproject: `X=(u-cx)Z/fx, Y=(v-cy)Z/fy, Z=Z`.
  3. Transform each 3D point to `target_frame`, drop Z.
  4. Convex hull (Andrew monotone chain) → polygon, publish.
- `optical_frame` = `camera_optical_frame` param if set, else
  `depth.header.frame_id`.
- Key params: `depth_topic`, `camera_info_topic`, `regions_topic`,
  `target_frame`, `camera_optical_frame`, `pixel_step`, `min_depth` (0.15),
  `max_depth` (6.0), `transform_tolerance` (0.2).

### 3.3 `SemanticTraversabilityLayer` (semantic_traversability)
- `onInitialize`: declare params, subscribe `polygon_topic` (transient_local).
- `updateBounds`: transform every traversable region to the costmap global
  frame, expand the update window to their XY union.
- `updateCosts`: for each region, clamp its world bbox to map cells
  (`worldToMapEnforceBounds`), and for every cell that is **inside the polygon**
  (ray-cast test) **and** has current cost `>= override_threshold` (254), set it
  to `traversable_cost`.
- Cost source: `traversable_cost >= 0` forces that value (single knob);
  `traversable_cost < 0` honors each region's own `cost` field.
- Params: `enabled`, `polygon_topic` (`/semantic_regions`), `traversable_cost`
  (`0.0`), `override_threshold` (`254`), `transform_tolerance` (`0.2`).

### 3.4 `SetSemanticInstruction` (bt_nav)
- `RosTopicPubNode<std_msgs/String>`; publishes the `instruction` port to the
  given `topic_name`, returning SUCCESS. Registered in `BTEngine::registerNodes`.
- Not in the default tree; see `bt_engine/bt/semantic_tree.xml`.

---

## 4. Concerns / risks (read before testing)

1. **Camera optical-frame convention.** Deprojection assumes the depth image's
   `header.frame_id` follows the optical convention (x-right, y-down, z-forward).
   If Isaac publishes depth under a body frame (`camera_link`, x-forward), the
   polygon lands in the wrong place. **Mitigation:** set
   `camera_optical_frame:=<the optical frame>` on projection_node. Verify with
   `ros2 run tf2_tools view_frames` and by visualizing `/semantic_regions` in
   RViz against the actual curtain.

2. **Depth/RGB alignment & encoding.** projection_node assumes the depth image is
   pixel-aligned with the RGB the VLM saw, and encoded `32FC1`(m) or `16UC1`(mm).
   If Isaac's RealSense outputs unaligned or a different encoding, points are
   wrong/dropped. **Check:** `ros2 topic echo /depth --field encoding` and
   confirm `/rgb` and `/depth` share resolution.

3. **NavFn barely weights intermediate cost.** With the planner intentionally
   left as NavFn, only `traversable_cost: 0` reliably yields a path *through*.
   A value like 100 won't behave "risk-aware" under NavFn. **If** you later want
   risk-aware detours, switch to Smac2D (out of scope; controller/planner kept
   as-is per request).

4. **Inflation around cleared cells.** The layer clears LETHAL cells, but
   `InflationLayer` runs after it and inflates whatever LETHAL cells remain at
   the polygon border. If the polygon under-covers the curtain, leftover lethal
   edges may inflate back over the gap. **Mitigation:** make the detection/
   polygon slightly generous; confirm the cleared corridor is wider than the
   robot footprint (`robot_radius 0.22`, `inflation_radius 0.55`).

5. **Grounding DINO weight download / GPU.** The image must be built with
   `GROUNDING_DINO=YES`; weights are pulled from HuggingFace at build time
   (network). torch wheel index defaults to cu121 — adjust `TORCH_INDEX_URL` if
   the container CUDA differs. **Mitigation:** validate the whole costmap path
   with `perception:=static` first; only then bring up the VLM.

6. **Latency / rate.** VLM inference is slow (100s of ms). The region is latched,
   so a stale polygon persists until the next detection — fine for static
   curtains, not for moving ones. Costmap clears only update when a new region
   arrives or the costmap re-reads (rolling window).

7. **`use_sim_time`.** All semantic nodes must run with `use_sim_time:=True` in
   sim or TF lookups fail. The launch file sets this; if you run nodes manually,
   pass it.

8. **CI is intentionally untouched.** These packages are NOT in the CI subset
   (`--packages-up-to bt_engine`). Don't add them there — `nav2_costmap_2d` /
   torch aren't in the CI image.

---

## 5. Step-by-step test plan

### Step 0 — Build (inside the sim container)
```bash
cd $ROS2_WS
colcon build --symlink-install \
  --packages-up-to bt_engine stretch3_navigation \
                   semantic_traversability semantic_perception
source install/setup.bash
```
**Expect:** all four packages build. Common failure: missing `nav2_costmap_2d`
(install `ros-humble-navigation2`) or `cv_bridge` (from the DINO module).

### Step 1 — Interfaces sanity
```bash
ros2 interface show btcpp_ros2_interfaces/msg/SemanticRegionArray
ros2 interface show btcpp_ros2_interfaces/msg/SemanticDetection2D
```
**Expect:** the message definitions print (rosidl generated them).

### Step 2 — Plugin is discoverable
```bash
ros2 plugin list | grep -i semantic        # or:
grep -r SemanticTraversabilityLayer install/*/share/*/nav2_*plugin*.xml
```
**Expect:** `semantic_traversability::SemanticTraversabilityLayer` registered for
base `nav2_costmap_2d::Layer`.

### Step 3 — Milestone 1: costmap + planner, NO ML
Bring up Isaac Sim (publishing `/laser_scan`, `/odom`, TF), then:
```bash
ros2 launch stretch3_navigation semantic_navigation.launch.py perception:=static
```
Tune the static polygon to your scene if needed:
```bash
ros2 param set /static_region_publisher polygon "[1.5,-0.2, 2.5,-0.2, 2.5,0.2, 1.5,0.2]"
```
**Checks:**
- `ros2 topic echo /semantic_regions --once` → one traversable region in `world`.
- In RViz (global costmap): the curtain cells that were red/LETHAL turn free
  inside the polygon.
- Nav2 logs show `semantic_traversability_layer` loaded in both costmaps.

### Step 4 — A/B proof (the paper's Fig. 6)
Send a goal **behind** the curtain (RViz "Nav2 Goal" or):
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: world}, pose: {position: {x: 3.0, y: 0.0}}}}"
```
- **Layer ON:** a path is planned straight through the curtain; robot drives.
- **Layer OFF** (`ros2 param set /global_costmap/global_costmap
  semantic_traversability_layer.enabled false`): no valid path / robot stops at
  the curtain.

### Step 5 — Milestone 2: full VLM pipeline
```bash
ros2 launch stretch3_navigation semantic_navigation.launch.py \
  perception:=dino
```
**Checks (in order — isolates failures):**
1. `ros2 topic hz /rgb /depth /camera_info` → all flowing.
2. `ros2 topic echo /semantic_detection` → a `curtain` box with `traversable=true`
   when the curtain is in view.
3. `ros2 topic echo /semantic_regions` → polygon roughly over the real curtain
   (visualize in RViz to confirm placement — this validates Concern #1).
4. Repeat Step 4's A/B with live perception.

### Step 6 — Interactive prompt (BT)
```bash
ros2 run bt_engine bt_engine --ros-args \
  -p bt_xml_path:=$(ros2 pkg prefix bt_engine)/share/bt_engine/bt/semantic_tree.xml
ros2 topic pub --once /start std_msgs/Empty "{}"
```
**Expect:** `SetSemanticInstruction` publishes `curtain` to
`/semantic_instruction`, then `NavigateToPose` runs. Try switching the
prompt live:
```bash
ros2 topic pub --once /semantic_instruction std_msgs/String "{data: 'grass'}"
```

### Step 7 — Cost-value behavior (optional)
Toggle the written cost to confirm semantics:
```bash
# free pass (paper)        risk-flavored (NavFn won't truly detour)
ros2 param set /global_costmap/global_costmap semantic_traversability_layer.traversable_cost 0
ros2 param set /global_costmap/global_costmap semantic_traversability_layer.traversable_cost 100
```
(Param is read at init; reload the costmap or relaunch for it to take effect.)

---

## 6. Real robot (later)
See [README.md](README.md) §"Real robot". The C++ `semantic_traversability`
package is reusable as-is; only the perception source, frames, and the deploy
nav2 params overlay change.
