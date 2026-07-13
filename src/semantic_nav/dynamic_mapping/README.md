# Dynamic SLAM Runtime Map Updating

This package adds a lightweight dynamic-mapping layer for the Isaac Sim SLAM
pipeline. It keeps RTAB-Map's internal map and database unchanged, then
publishes a cleaned runtime occupancy grid for Nav2.

The design is conservative: a semantic detection can identify a candidate
dynamic-object footprint, but it cannot clear the persistent navigation map by
itself. A stale occupied cell is cleared from the published `/map` only after
later LiDAR scans repeatedly pass through that cell from distinct robot poses.

## Pipeline

```text
Isaac Sim:
  /rgb
  /depth
  /camera_info
  /laser_scan
  /odom
  /tf
      |
      +--> rtabmap_slam/rtabmap --------------> /rtabmap/map
      |
      +--> LocateAnything --> /semantic_detection
      |                         |
      +--> projection_node <----+
                |
                +-----------------------------> /semantic_regions

/rtabmap/map + /laser_scan + /semantic_regions + TF
      |
      +--> dynamic_map_updater_node ----------> /map
                                              +-> /dynamic_map/cleared_cells
                                              +-> /dynamic_map/change_events

Nav2 global_costmap StaticLayer <------------- /map
Nav2 local_costmap ObstacleLayer <------------ /laser_scan
```

The global costmap plans against the cleaned `/map`. In the dynamic SLAM config,
semantic traversability runs after inflation in both costmaps. That final layer
clears inflated and lethal costs inside confirmed `traversable: true` regions
and a configurable margin around them, while preserving unknown cells. This
keeps ordinary dynamic objects as live collision obstacles while preventing a
confirmed traversable object, such as a curtain, from reappearing as cost when
the robot gets close or passes it.

## Packages And Files

- `dynamic_mapping`
  - `dynamic_map_updater_node`: ROS node that subscribes to raw SLAM map,
    semantic regions, LiDAR scans, and TF.
  - `dynamic_map_core`: testable grid conversion, ray tracing, polygon, label,
    and absence-evidence logic.
- `stretch3_navigation/launch/dynamic_slam.launch.py`
  - Starts RTAB-Map, semantic perception/projection, the updater, Nav2, and the
    `world -> map` static transform needed in Isaac Sim.
- `stretch3_navigation/config/dynamic_map_updater.yaml`
  - Thresholds, topics, frames, and dynamic labels.
- `stretch3_navigation/config/nav2_dynamic_slam_params.yaml`
  - Nav2 config where the global costmap uses `StaticLayer` on `/map`, while
    the local costmap uses `ObstacleLayer -> InflationLayer ->
    SemanticTraversabilityLayer`. The global costmap uses `StaticLayer ->
    InflationLayer -> SemanticTraversabilityLayer`. The semantic layer's
    `clear_margin_m` should be at least the inflation radius when the traversable
    object footprint is slightly smaller than the inflated cost field in RViz.

## Behavior

The updater maintains:

- Latest raw SLAM map from `/rtabmap/map`.
- A candidate-cell mask from `/semantic_regions` for configured dynamic labels.
- Per-cell absence evidence from LiDAR free-space ray crossings.
- Per-cell traversable-object evidence from repeated semantic observations of
  configured traversable labels.

For stale dynamic objects, a cell can be cleared in the published `/map` only
when all of these are true:

- The raw SLAM map currently marks the cell occupied.
- The cell lies inside a semantic region whose label is in `dynamic_labels`.
- Later LiDAR rays pass through the cell as free space.
- Evidence reaches:
  - `min_absence_hits: 5`
  - `min_distinct_poses: 3`
  - `min_pose_separation_m: 0.25`
  - `ray_pass_through_margin_m: 0.15`

For traversable obstacles, such as curtains, the updater uses a separate
Khronos-inspired confirmation path. A semantic region proposes cells, but the
cleaned `/map` clears them permanently only after repeated observations from
distinct robot poses:

- The raw SLAM map currently marks the cell occupied.
- The cell lies inside a region with `traversable: true`.
- The label is in `traversable_labels`.
- Evidence reaches:
  - `min_traversable_observations: 4`
  - `min_traversable_distinct_poses: 3`
  - `traversable_min_pose_separation_m: 0.25`

Once this threshold is met, the cleaned `/map` continues publishing that cell as
free even when the object moves behind the robot and the camera no longer sees
it. This prevents the global planner from re-blocking the only exit path after
the robot passes the traversable object.

If the raw map later no longer marks that cell occupied, or the cell no longer
belongs to a dynamic semantic region, its absence evidence is reset.

Unknown cells remain unknown (`-1`). Cleared stale occupied cells are published
as free (`0`) only in the cleaned `/map`; `/rtabmap/map` is not modified.

## Run In Isaac Sim

Start Isaac Sim first. The expected topics are:

- `/rgb` (`sensor_msgs/Image`)
- `/depth` (`sensor_msgs/Image`)
- `/camera_info` (`sensor_msgs/CameraInfo`)
- `/laser_scan` (`sensor_msgs/LaserScan`)
- `/odom` (`nav_msgs/Odometry`)
- `/tf`, `/tf_static`
- `/clock`

Build and source the workspace:

```bash
colcon build --symlink-install --packages-up-to stretch3_navigation
source install/setup.bash
```

Launch the full dynamic SLAM stack:

```bash
ros2 launch stretch3_navigation dynamic_slam.launch.py \
  rgb_topic:=/rgb \
  depth_topic:=/depth \
  camera_info_topic:=/camera_info \
  scan_topic:=/laser_scan
```

The default perception backend is LocateAnything. Alternatives:

```bash
# Use Grounding DINO.
ros2 launch stretch3_navigation dynamic_slam.launch.py perception:=dino

# Require 3 matching detections before semantic memory holds a region forever.
ros2 launch stretch3_navigation dynamic_slam.launch.py \
  region_confirmation_hits:=3 \
  region_hold_sec:=-1.0

# Skip ML and publish /semantic_regions yourself.
ros2 launch stretch3_navigation dynamic_slam.launch.py perception:=none
```

## Verify

Confirm the stack is up:

```bash
ros2 node list
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
```

Expected active nodes include:

- `/rtabmap`
- `/dynamic_map_updater_node`
- `/semantic_projection_node`
- `/locate_anything_node` when `perception:=locate_anything`
- `/planner_server`
- `/controller_server`

Check topic flow:

```bash
ros2 topic list -t | grep -E 'rtabmap/map|/map|semantic_regions|dynamic_map'
ros2 topic echo /rtabmap/map --once
ros2 topic echo /map --once
ros2 topic echo /semantic_regions --once
ros2 topic echo /dynamic_map/cleared_cells --once
```

Expected:

- `/rtabmap/map` publishes the raw RTAB-Map occupancy grid.
- `/map` publishes the cleaned runtime occupancy grid in `map` frame.
- `/semantic_regions` publishes projected ground polygons in `map` frame. With
  memory enabled, regions appear only after confirmation and continue to be
  republished through close-range detector drop-outs.
- `/dynamic_map/cleared_cells` publishes an occupancy grid where cleared cells
  are `0` and all other cells are `-1`.

Check that a single semantic detection does not clear the map:

```bash
python3 - <<'PY'
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

rclpy.init()
node = rclpy.create_node("dynamic_map_probe")
msgs = {}
qos = QoSProfile(depth=1)
qos.history = HistoryPolicy.KEEP_LAST
qos.reliability = ReliabilityPolicy.RELIABLE
qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

for name, topic in [
    ("raw", "/rtabmap/map"),
    ("clean", "/map"),
    ("debug", "/dynamic_map/cleared_cells"),
]:
    node.create_subscription(
        OccupancyGrid, topic, lambda msg, n=name: msgs.__setitem__(n, msg), qos)

deadline = node.get_clock().now().nanoseconds + 10_000_000_000
while rclpy.ok() and set(msgs) != {"raw", "clean", "debug"}:
    if node.get_clock().now().nanoseconds > deadline:
        break
    rclpy.spin_once(node, timeout_sec=0.2)

for name in ["raw", "clean", "debug"]:
    msg = msgs.get(name)
    if msg is None:
        print(f"{name}: missing")
        continue
    data = list(msg.data)
    print(
        f"{name}: frame={msg.header.frame_id} "
        f"size={msg.info.width}x{msg.info.height} "
        f"unknown={data.count(-1)} free={data.count(0)} "
        f"occupied={sum(1 for v in data if v >= 65)}")

if "raw" in msgs and "clean" in msgs:
    pairs = list(zip(msgs["raw"].data, msgs["clean"].data))
    print("raw_vs_clean_diffs=" + str(sum(1 for a, b in pairs if a != b)))
    print("raw_occupied_cleared_to_free=" + str(
        sum(1 for a, b in pairs if a >= 65 and b == 0)))
if "debug" in msgs:
    print("debug_cleared_cells=" + str(sum(1 for v in msgs["debug"].data if v == 0)))

node.destroy_node()
rclpy.shutdown()
PY
```

Immediately after startup, it is normal for `raw_vs_clean_diffs=0` and
`debug_cleared_cells=0`. Cells should clear only after the robot revisits a
previously occupied dynamic-object region and LiDAR rays repeatedly pass through
that space.

## Dynamic Clearing Test Procedure

Use this manual scenario in Isaac Sim:

1. Start Isaac Sim and launch `dynamic_slam.launch.py`.
2. Place a dynamic object in the scene using one of the configured labels, such
   as `person`, `chair`, `cart`, or `box`.
3. Let RTAB-Map observe and map the object so occupied cells appear in
   `/rtabmap/map`.
4. Remove the object from the scene.
5. Drive or command the robot through several viewpoints that produce LiDAR rays
   through the stale occupied space.
6. Watch:
   - `/rtabmap/map`: remains the raw SLAM map.
- `/map`: clears stale occupied cells after evidence thresholds are met.
- `/dynamic_map/cleared_cells`: shows the cleared cells.
- `/dynamic_map/change_events`: emits a message when new cells are cleared.

The local costmap should still block any non-traversable object that is
currently present and seen by `/laser_scan`. Confirmed traversable objects are
cleared locally and globally by `SemanticTraversabilityLayer`, including the
inflated cost around the object footprint.

## Configuration

Default updater params are in
`src/sim/stretch3_navigation/config/dynamic_map_updater.yaml`:

```yaml
raw_map_topic: /rtabmap/map
scan_topic: /laser_scan
semantic_regions_topic: /semantic_regions
output_map_topic: /map
map_frame: map
base_frame: base_link

dynamic_labels:
  - person
  - chair
  - cart
  - box

enable_traversable_persistence: True
traversable_labels:
  - curtain
  - grass
min_traversable_observations: 4
min_traversable_distinct_poses: 3
traversable_min_pose_separation_m: 0.25
traversable_confidence_radius_m: 0.10

min_absence_hits: 5
min_distinct_poses: 3
min_pose_separation_m: 0.25
ray_pass_through_margin_m: 0.15
occupied_threshold: 65
```

For the real robot, the likely first changes are:

- `scan_topic: /scan_filtered`
- `raw_map_topic`: whatever SLAM backend publishes the raw map
- `map_frame`: usually `map`
- `base_frame`: usually `base_link`

## Tests

Run package tests:

```bash
colcon test --packages-select dynamic_mapping --event-handlers console_direct+
```

Covered core behavior:

- Occupancy-grid world/cell conversion.
- Bresenham ray tracing.
- Absence evidence accumulation and reset.
- Case-insensitive dynamic label filtering.
- Polygon containment.

## Known Limitations

- This package does not rewrite RTAB-Map's database or internal map.
- Semantic detections are candidate masks only; LiDAR free-space evidence is
  the clearing authority.
- Dynamic clearing requires robot motion or sufficiently distinct scan poses.
- `/dynamic_map/change_events` is a debug `std_msgs/String` topic, not a stable
  API contract.
- The first implementation targets 2D occupancy-grid navigation. It is intended
  to stay backend-agnostic enough to replace `/rtabmap/map` with another raw map
  source later.
