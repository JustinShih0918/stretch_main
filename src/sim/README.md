# `src/sim/` — Isaac Sim simulation packages

Packages built into the **sim** image ([docker/sim/](../../docker/sim/)) for the
Isaac Sim 5.1 environment. The shared BehaviorTree engine
([src/common/](../common/)) drives the `navigate_to_pose` server these bring up.

Ported from [j3soon/ros2-essentials](https://github.com/j3soon/ros2-essentials)
`stretch3_ws` (which this repo's maintainer co-owns).

## `stretch3_navigation/`

`ament_cmake` package that installs launch + config + rviz only (no compiled
code). Expects Isaac Sim to publish `world -> odom -> base_link` TF, `/laser_scan`
and `/odom`.

- `launch/navigation.launch.py` — pure nav2 bringup (no SLAM)
- `launch/cartographer.launch.py` — Cartographer 2D SLAM + nav2
- `launch/rtabmap.launch.py` — RTAB-Map SLAM + nav2
- `launch/rviz.launch.py` — RViz with `rviz/stretch3.rviz`
- `config/` — `nav2_params.yaml`, `cartographer.lua`, `rtabmap.yaml`

## `stretch_urdf/`

hello-robot's URDF-generation pip library. **No `package.xml`** → colcon
ignores it during build; it's kept for parity with `stretch3_ws` and for URDF
tooling. Isaac USD assets live at the repo-root [`isaacsim/assets/`](../../isaacsim/assets/).

To re-sync from upstream:

```bash
git clone --depth 1 https://github.com/j3soon/ros2-essentials.git /tmp/ros2-essentials
cp -R /tmp/ros2-essentials/stretch3_ws/src/stretch3_navigation src/sim/
cp -R /tmp/ros2-essentials/stretch3_ws/src/stretch_urdf        src/sim/
```
## Automated benchmark resets

The simulation image includes `ros-humble-simulation-interfaces`. Start
Isaac Sim 5.1 with `isaac-sim-ros-control` so the documented
`isaacsim.ros2.sim_control` extension exposes `/set_entity_state`; then open
the hospital USD and press Play. The VLN benchmark deliberately fails if the
service or reset-confirming odometry is absent.
