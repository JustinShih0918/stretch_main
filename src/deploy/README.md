# `src/deploy/` — on-robot navigation & SLAM

Packages built into the **deploy** image ([docker/deploy/](../../docker/deploy/))
and run on the physical Stretch robot. The shared BehaviorTree engine
(`bt_engine` / `bt_nav` in [src/common/](../common/)) drives the
`navigate_to_pose` action server these packages bring up.

## `stretch_nav2/` (vendored)

- **Upstream:** [hello-robot/stretch_ros2](https://github.com/hello-robot/stretch_ros2)
  (branch `humble`), package `stretch_nav2`.
- **License:** Apache License 2.0 — see [stretch_nav2/LICENSE.md](stretch_nav2/LICENSE.md).
  Copyright 2020–2024 Hello Robot Inc. Vendored verbatim; do not edit in place —
  re-sync from upstream instead.
- **What it provides:** nav2 bringup + `slam_toolbox` (online/offline mapping) +
  AMCL localization, with Stretch-tuned configs
  (`config/nav2_params.yaml`, `config/mapper_params_online_async.yaml`,
  `config/nav2_voxel_params.yaml`).

### Runtime dependency on the robot's hello-robot install

`stretch_nav2/launch/navigation.launch.py` includes `stretch_core`'s
`stretch_driver.launch.py` and `rplidar.launch.py`. **`stretch_core` (the
hardware driver) and the calibrated URDF are NOT vendored here** — they come
from the robot's own hello-robot ROS 2 workspace, which must be sourced as an
underlay before launching (see [docker/deploy/](../../docker/deploy/)).

To re-sync the vendored copy:

```bash
git clone --depth 1 -b humble https://github.com/hello-robot/stretch_ros2.git /tmp/stretch_ros2
rm -rf src/deploy/stretch_nav2
cp -R /tmp/stretch_ros2/stretch_nav2 src/deploy/stretch_nav2
```
