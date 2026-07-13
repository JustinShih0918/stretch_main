# Semantic Nav — Verification Log

Companion to [DESIGN.md](DESIGN.md). Records what has been tested, what passed,
what failed (and how it was fixed), and what still needs to be run.

Environment: Isaac Sim 5.1 (running), sim container, `perception:=static` (no ML),
RViz2 costmap visualization, 2026-06-17.

---

## Results

### ✅ Step 1 — Interface definitions

```bash
ros2 interface show btcpp_ros2_interfaces/msg/SemanticRegionArray
ros2 interface show btcpp_ros2_interfaces/msg/SemanticDetection2D
```

Both printed cleanly. `rosidl` generated the types correctly.

---

### ✅ Step 2 — Plugin discoverable

```bash
grep -r SemanticTraversabilityLayer install/*/share/*/nav2_*plugin*.xml
```

Found in `install/semantic_traversability/share/semantic_traversability/nav2_semantic_traversability_plugin.xml`.
`semantic_traversability::SemanticTraversabilityLayer` registered for base
`nav2_costmap_2d::Layer`. ✓

---

### ✅ Step 3 — Milestone 1: static polygon, no ML

```bash
ros2 launch stretch3_navigation semantic_navigation.launch.py perception:=static
```

**Layer loaded in both costmaps** (from launch log):
```
[local_costmap]  [semantic_traversability_layer] SemanticTraversabilityLayer up:
                  topic=/semantic_regions traversable_cost=0 override_threshold=254
[global_costmap] [semantic_traversability_layer] SemanticTraversabilityLayer up:
                  topic=/semantic_regions traversable_cost=0 override_threshold=254
```

`/semantic_regions` echoed a traversable `curtain` polygon (4-point rectangle in
`world` frame). ✓

**Depth encoding check:** `/depth` → `32FC1` (meters). projection_node supports
this encoding. ✓

**Isaac Sim topics confirmed live:**
`/rgb`, `/depth`, `/camera_info`, `/laser_scan`, `/odom`, `/tf`, `/clock`. ✓

---

### ✅ Step 4 — A/B costmap proof (RViz observed)

Polygon was repositioned to cover the actual wall in the scene (found via laser
scan analysis: wall at x ≈ 9.5 in world frame). Final polygon:
`x=9.0–10.0, y=±0.8` (1.6 m wide, wider than 2 × inflation_radius = 1.1 m).

**Layer ON:** Gap visible in global costmap in RViz — LETHAL cells inside the
polygon cleared to FREESPACE. Navigation goal to `(11.0, 0.0)` **succeeded** ✓

**Layer OFF (`ros2 param set ... enabled false`):** Gap closed in RViz — wall
cells restored to LETHAL. Navigation goal to `(11.0, 0.0)` blocked. ✓

---

### 🐛 Bug found & fixed — `enabled` param not live-reloadable

**Symptom:** `ros2 param set /global_costmap/global_costmap
semantic_traversability_layer.enabled false` had no effect; the costmap gap
stayed clear even with the layer "disabled."

**Root cause:** `onInitialize()` read `enabled_` once via `get_parameter()` with
no parameter-change callback registered. The member was never updated at runtime.

**Fix** (`src/semantic_traversability/src/semantic_traversability_layer.cpp`):
Added `add_on_set_parameters_callback` at the end of `onInitialize()` that
updates `enabled_`, `traversable_cost_`, and `override_threshold_` whenever any
of those parameters change. The callback handle is stored in a new
`param_cb_handle_` member (header) so it is not garbage-collected.

After rebuilding (`colcon build --packages-select semantic_traversability`),
toggling `enabled` via `ros2 param set` takes effect immediately on the next
costmap update cycle. ✓

---

## Still to verify

### Step 5 — Full VLM pipeline

LocateAnything is the default backend and is enabled by the sim Compose build:

```bash
docker compose -f docker/sim/compose.yaml build
ros2 launch stretch3_navigation semantic_navigation.launch.py
```

The cached checkpoint should exist at
`/opt/locate_anything/LocateAnything-3B`. To validate the upstream worker
directly:

```bash
cd src/models/Eagle/Embodied
LOCATE_ANYTHING_MODEL_PATH=/opt/locate_anything/LocateAnything-3B python3 test.py
```

Grounding DINO remains available as an optional backend:

**Blocker:** Image was not built with `GROUNDING_DINO=YES`; weights are absent
at `/opt/grounding_dino/`.

```bash
# Rebuild image with VLM support:
docker compose -f docker/sim/compose.yaml build --build-arg GROUNDING_DINO=YES

# Then inside container:
ros2 launch stretch3_navigation semantic_navigation.launch.py \
  perception:=dino
```

Grounding DINO model paths and thresholds are configured in
`semantic_perception/config/grounding_dino_params.yaml`. Use
`perception_params_file:=/path/to/custom_grounding_dino_params.yaml` for an
experiment-specific override.

**Checks to run (in order — isolates failure):**
1. `ros2 topic hz /rgb /depth /camera_info` — all flowing?
2. `ros2 topic echo /semantic_detection` — `curtain` box, `traversable=true`?
3. `ros2 topic echo /semantic_regions` — polygon visible in RViz, aligned with
   real curtain? (validates camera optical-frame convention, DESIGN.md Concern #1)
4. Repeat Step 4 A/B with live perception.

---

### Step 6 — BT integration (`SetSemanticInstruction`)

```bash
ros2 run bt_engine bt_engine --ros-args \
  -p bt_xml_path:=$(ros2 pkg prefix bt_engine)/share/bt_engine/bt/semantic_tree.xml
ros2 topic pub --once /start std_msgs/Empty "{}"
```

**Expect:** `SetSemanticInstruction` publishes `curtain` to
`/semantic_instruction`, then `NavigateToPose` navigates to
`(3.0, 0.0)`. Try switching prompt live:

```bash
ros2 topic pub --once /semantic_instruction std_msgs/String "{data: 'grass'}"
```

Not yet tested (requires VLM or a stub).

---

### Step 7 — Cost-value knob

```bash
ros2 param set /global_costmap/global_costmap \
  semantic_traversability_layer.traversable_cost 100
```

With NavFn this has no meaningful effect (DESIGN.md §4 Concern #3). Would need
Smac2D planner to observe true risk-aware detour behaviour. Not tested.

---

### Step 8 — projection_node end-to-end (camera → polygon alignment)

Requires VLM or a `SemanticDetection2D` injected manually:

```bash
ros2 topic pub --once /semantic_detection btcpp_ros2_interfaces/msg/SemanticDetection2D \
  '{header: {frame_id: camera_optical_frame}, label: "curtain",
    traversable: true, confidence: 1.0,
    x: 200, y: 100, width: 200, height: 300}'
```

Then verify `/semantic_regions` polygon overlaps the real curtain in RViz.
Validates deprojection math and TF chain (`camera_optical_frame → odom`).

---

### Step 9 — Real robot (future)

See [README.md](README.md) §"Real robot". Requires:
- `semantic_traversability` added to deploy image build
- Nav2 params overlay for `map` frame (AMCL, not sim ground-truth `world`)
- Camera optical-frame verification on RealSense D435i TF chain
- Off-board or on-robot Grounding DINO inference
