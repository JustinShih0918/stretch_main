# Dynamic Mapping Design

This package provides a runtime map updater for dynamic SLAM. It sits between
the SLAM backend and Nav2, keeps the SLAM map unchanged, and publishes a cleaned
navigation map that can remove stale or traversable obstacles only after
conservative evidence has accumulated.

The design is intentionally lightweight. It borrows the main safety idea from
Khronos-style dynamic mapping: a single semantic detection is not enough to
rewrite navigable space. A cell is cleared only after repeated evidence from
later observations.

## Goals

- Keep RTAB-Map's internal map and database unchanged.
- Publish a cleaned `/map` for Nav2's global costmap.
- Remove stale dynamic objects only after LiDAR free-space rays repeatedly pass
  through the occupied cells.
- Persistently clear confirmed traversable objects, such as curtains or grass,
  so they do not reappear as blocking cost when the camera no longer sees them.
- Keep live obstacle marking active in the local costmap for non-traversable
  objects.

## Non-Goals

- This package does not edit the SLAM backend database.
- It does not perform 3D object tracking or mesh-level reconstruction.
- It does not trust visual detections as direct proof that map cells should be
  removed.
- It does not disable local collision protection for ordinary obstacles.

## System Context

```text
RTAB-Map / SLAM backend
  /rtabmap/map
      |
      v
dynamic_map_updater_node ----------------------> /map
  ^              ^              ^                  |
  |              |              |                  v
  |              |              |          Nav2 global StaticLayer
  |              |              |
  |              |              +---------- TF: map -> base_link and scan frame
  |              |
  |              +------------------------- /laser_scan
  |
  +---------------------------------------- /semantic_regions

semantic_traversability_layer:
  runs inside Nav2 costmaps after inflation and clears confirmed traversable
  regions plus a margin around them.
```

The map updater publishes the occupancy grid consumed by the Nav2 global
`StaticLayer`. Nav2's local costmap still uses the live LiDAR `ObstacleLayer`, so
currently present non-traversable objects remain collision obstacles.

## Inputs And Outputs

Inputs:

- `raw_map_topic`, default `/rtabmap/map`
  - Raw occupancy grid from the SLAM backend.
- `scan_topic`, default `/laser_scan`
  - 2D LiDAR scan used as the authority for free-space absence evidence.
- `semantic_regions_topic`, default `/semantic_regions`
  - Ground-plane polygons from the semantic projection pipeline.
- TF
  - `map_frame -> base_frame`
  - `map_frame -> scan frame`
  - `map_frame -> semantic region frame`

Outputs:

- `output_map_topic`, default `/map`
  - Cleaned runtime occupancy grid for Nav2.
- `cleared_cells_topic`, default `/dynamic_map/cleared_cells`
  - Debug occupancy grid. Cleared cells are `0`; all other cells are `-1`.
- `change_events_topic`, default `/dynamic_map/change_events`
  - Debug string messages when cells become cleared or traversable-confirmed.

## Data Model

The node stores the latest raw occupancy grid and derives a compact grid-aligned
state from it:

- `GridGeometry`
  - Width, height, resolution, and map origin.
- `candidate_cells_`
  - Cells inside semantic regions whose labels match `dynamic_labels`.
- `traversable_candidate_cells_`
  - Cells inside confirmed semantic regions that are `traversable: true` and
    match `traversable_labels`.
- `EvidenceGrid evidence_`
  - LiDAR absence evidence for stale dynamic-object removal.
- `EvidenceGrid traversable_evidence_`
  - Semantic confirmation evidence for permanent traversable-object clearing.

`EvidenceGrid` tracks per-cell:

- `absence_hits`
  - Number of evidence updates for the cell.
- `distinct_poses`
  - Robot poses separated by at least `min_pose_separation_m`.
- `cleared`
  - True once both hit and distinct-pose thresholds are met.

The same generic `EvidenceGrid` is used for two meanings:

- For dynamic objects, an evidence hit means a LiDAR free-space ray crossed a
  candidate occupied cell.
- For traversable objects, an evidence hit means a configured traversable region
  observed the occupied cell from the current robot pose.

## Raw Map Handling

When a raw map arrives:

1. The node records the grid and frame.
2. If geometry changed, all masks and evidence grids are resized and reset.
3. If geometry is unchanged, evidence is reset for cells that are no longer
   occupied in the raw map.
4. Candidate masks are rebuilt from the latest semantic regions.
5. A cleaned map is republished.

The raw map is never modified. The cleaned map starts as a copy of the raw map
and then overrides selected occupied cells to free (`0`).

Unknown cells stay unknown unless a cell is explicitly cleared from an occupied
raw-map value.

## Semantic Candidate Mask

Each semantic region is transformed into the map frame. Its polygon is rasterized
onto the occupancy grid by testing each cell center against the polygon.

Two masks are built:

- Dynamic-object mask
  - Region label must match `dynamic_labels`.
  - Used with LiDAR absence evidence.
- Traversable-object mask
  - `enable_traversable_persistence` must be true.
  - Region must have `traversable: true`.
  - Region label must match `traversable_labels`, unless that list is empty.
  - Used with semantic confirmation evidence.

For traversable regions, the rasterization bounding box is expanded by
`traversable_confidence_radius_m`. This makes confirmation less brittle when the
projected polygon is slightly smaller than the object footprint.

## Dynamic Object Clearing

This path removes stale map cells for configured dynamic labels such as people,
chairs, carts, or boxes.

A raw occupied cell becomes eligible only if:

- It is currently occupied in `/rtabmap/map`.
- It lies inside a semantic region whose normalized label is in
  `dynamic_labels`.

For each valid LiDAR scan ray:

1. The scan origin and endpoint are transformed into the map frame.
2. Bresenham ray tracing enumerates cells from the scan origin to the endpoint.
3. The endpoint cell is treated as a possible current obstacle observation. If
   it is a candidate occupied cell, its absence evidence is reset.
4. Cells before the endpoint are treated as observed free space, excluding the
   final `ray_pass_through_margin_m` near the endpoint.
5. Candidate occupied cells crossed by that free-space part of the ray receive
   one evidence hit at the current robot pose.

A cell is cleared in `/map` once:

- `absence_hits >= min_absence_hits`
- `distinct_poses.size() >= min_distinct_poses`

The distinct-pose check prevents the robot from clearing a cell because many
nearly identical rays arrived from one stationary viewpoint.

## Traversable Object Persistence

This path handles objects that are physically traversable but visually
intermittent, for example a curtain that the camera fails to detect when the
robot is too close or after the robot has passed it.

The problem is different from stale dynamic-object removal. Here the object may
still be present, and LiDAR may continue seeing it. The intended behavior is to
allow navigation through it after the system has enough evidence that it is
traversable.

When `/semantic_regions` arrives:

1. Traversable candidate cells are rebuilt.
2. The robot pose is looked up in the map frame.
3. Each raw occupied traversable candidate cell receives one confirmation hit.
4. A cell becomes persistently cleared after:
   - `min_traversable_observations`
   - `min_traversable_distinct_poses`
   - `traversable_min_pose_separation_m`

Once confirmed, a traversable cell remains cleared in the cleaned `/map` even if
the latest camera frame no longer contains that region. This is the memory
mechanism that prevents the global planner from re-blocking a passage after the
object moves behind the robot or becomes too close to detect.

## Publishing The Cleaned Map

Publishing starts with:

```text
cleaned = raw_map
debug = all unknown
```

For each cell:

- If it is a dynamic candidate occupied cell and LiDAR absence evidence is
  cleared, write `cleaned[i] = 0`.
- If it is a raw occupied cell and traversable evidence is cleared, write
  `cleaned[i] = 0`.
- For either cleared case, write `debug[i] = 0`.

Everything else remains exactly as it appears in the raw SLAM map.

## Nav2 Costmap Interaction

The cleaned `/map` fixes the global planner's static view of the world, but Nav2
also has layered costmaps that can reintroduce cost from live sensors.

The dynamic SLAM Nav2 config uses:

```text
global_costmap:
  StaticLayer -> InflationLayer -> SemanticTraversabilityLayer

local_costmap:
  ObstacleLayer -> InflationLayer -> SemanticTraversabilityLayer
```

The semantic traversability layer runs after inflation. It clears costs inside
confirmed `traversable: true` polygons and within `clear_margin_m` around their
edges. The margin should be at least the inflation radius when RViz still shows
an inflated cost halo around the traversable object.

`clear_unknown` is false by default so the layer does not convert unknown space
into free space.

## Safety Properties

- A single visual detection cannot clear stale dynamic objects from `/map`.
- Stale dynamic cells require free-space LiDAR evidence from multiple robot
  poses.
- Traversable-object persistence requires repeated semantic confirmation from
  multiple robot poses.
- Unknown cells are preserved.
- Raw SLAM output remains available on `/rtabmap/map` for debugging and future
  backend changes.
- Non-traversable live obstacles remain handled by the local LiDAR obstacle
  layer.

## Failure Modes And Diagnostics

No `/map` output:

- Check that `/rtabmap/map` is publishing.
- Check TF from the raw map frame to `base_frame`.

No cleared cells:

- Check `/semantic_regions` has polygons in a transformable frame.
- Check labels match `dynamic_labels` or `traversable_labels` after
  lowercasing.
- Check raw map cells under the region are actually occupied
  (`>= occupied_threshold`).
- Move the robot enough to satisfy the distinct-pose thresholds.

Cost still appears in RViz:

- Determine whether the cost is from the global or local costmap display.
- For global cost, confirm the global `StaticLayer` uses `/map`.
- For local cost, confirm `SemanticTraversabilityLayer` is after
  `InflationLayer`.
- Increase `clear_margin_m` if inflated cost remains around the polygon.
- Confirm `clear_unknown` is false unless unknown clearing is explicitly wanted.

## Extension Points

- Replace RTAB-Map with another SLAM backend by changing `raw_map_topic`.
- Add labels through `dynamic_labels` and `traversable_labels`.
- Tune confirmation strictness through the hit and distinct-pose thresholds.
- Add richer debug messages to `/dynamic_map/change_events`.
- Add per-class thresholds if different object categories need different
  evidence policies.

