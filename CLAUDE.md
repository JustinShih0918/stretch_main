# CLAUDE.md — Project notes for future Claude sessions

## What this repo is

A minimal ROS 2 (Humble) workspace that demonstrates **BehaviorTree.CPP v4 + behaviortree_ros2** ticking a tree which calls **nav2's standard `navigate_to_pose` action`**. It is intentionally trimmed-down compared to the much larger `ref/` reference code.

Top-level layout — every dir at repo root that has a `package.xml` is a colcon package:

```
stretch_main/
├── btcpp_ros2_interfaces/     # vendored interfaces pkg (action/srv/msg)
├── behaviortree_ros2/         # vendored BT.CPP↔ROS2 wrapper (RosActionNode, RosNodeParams, ...)
├── engine/   → pkg "bt_engine" # the BT runner
├── nav/      → pkg "bt_nav"    # NavigateToPose BT action node
├── docker/Dockerfile.ci       # CI image (also used for local docker compose)
├── docker/docker-compose.yaml # `build` (one-shot colcon) and `dev` (shell) services using Dockerfile.ci
├── ref/                       # old reference code, NOT a package, NOT built
├── engine/bt/main_tree.xml    # default BT XML
├── run.sh                     # tmux launcher
└── .github/workflows/ci.yml
```

## Critical project quirks

1. **`behaviortree_ros2` is vendored at the repo root** as a single package (not the upstream monorepo — just the wrapper package itself). It is NOT in the apt repos for Humble; do not try to `apt install ros-humble-behaviortree-ros2`.
2. **`btcpp_ros2_interfaces/` is a vendored copy** (not a submodule) — it has custom `.action` files (`FirmwareMission`, `Navigation`, etc.) that don't exist upstream. **Do not replace it with the upstream version.** If interfaces are missing, add them here. `behaviortree_ros2/package.xml` declares a `<depend>btcpp_ros2_interfaces</depend>` that is satisfied by this local copy.
3. **`ref/` is reference-only.** Contains an older, much larger `bt_engine.cpp` that uses a `dock_robot` action and project-specific logic (team/robot/plan parsing, startup service, game timer). It is **not built** (no top-level `package.xml`). Use it as a *pattern reference* — don't try to compile it.
4. **`docker/` only has `Dockerfile.ci`.** The legacy dev `Dockerfile` and `scripts/` dir (vnc / cli_tools / settings / micro_ros) were removed. `docker-compose.yaml` was rewritten to reuse `Dockerfile.ci` via a YAML anchor (`x-ci-image`); two services: `build` (one-shot `colcon build --packages-up-to bt_engine`) and `dev` (interactive shell). Workspace mounts at `/ws`. Run with `docker compose -f docker/docker-compose.yaml run --rm <build|dev>`.
5. `Dockerfile.ci` does **not** install `behaviortree_ros2` from apt (it's vendored). It also does not install `libfmt-dev` or `libboost-dev` explicitly — they come transitively via `ros-humble-behaviortree-cpp` and `ros-humble-navigation2`. If those are ever removed, add the libs back.

## Architecture

### `bt_engine` ([engine/](engine/))
- `BTEngine : rclcpp::Node`
- Single executable `bt_engine`
- Lifecycle:
  1. Constructor: declare params (`bt_xml_path`, `tree_name`, `tick_rate_hz`), subscribe to `/start` (`std_msgs/Empty`)
  2. `init()`: call after `make_shared` so `shared_from_this()` works → builds `BT::RosNodeParams`
  3. `registerNodes()`: `factory.registerNodeType<bt_nav::NavigateToPoseAction>("NavigateToPose", params_)`
  4. `buildTree()`: load XML, `factory.createTree(...)`
  5. Block on `spin_some` until `start_received_` flips true
  6. `runTree()`: `tree.tickOnce()` in a `rclcpp::Rate` loop until status != RUNNING
- `/start` topic type is `std_msgs/Empty` (chosen for simplest one-line publish)

### `bt_nav` ([nav/](nav/))
- One BT node: `bt_nav::NavigateToPoseAction`
- Inherits `BT::RosActionNode<nav2_msgs::action::NavigateToPose>` (from vendored `behaviortree_ros2`)
- Default action server name: `"navigate_to_pose"` (nav2 standard)
- Ports: `x` (double), `y` (double), `yaw` (double rad, default 0), `frame_id` (string, default `"map"`)
- Goal built with `tf2::Quaternion::setRPY(0, 0, yaw)` → `geometry_msgs::PoseStamped`
- Compiled as a SHARED library + exported via `ament_export_targets`; `bt_engine` `find_package(bt_nav)` and links it

## Build / run cheat sheet

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to bt_engine
source install/setup.bash

# need a navigate_to_pose action server, e.g. fake one:
ros2 run nav2_util fake_action_server navigate_to_pose &
./run.sh   # tmux: left runs engine, right pane press 's' to send /start
```

CI mirrors this: build `Dockerfile.ci` → `colcon build --packages-up-to bt_engine`. No source-clone step; everything we need is in-tree.

## Common tasks → where to edit

| Task | Files |
|---|---|
| Add a BT node | new pkg under repo root *or* extend `bt_nav`; register it in `engine/src/bt_engine.cpp::registerNodes()` |
| Change tree | `engine/bt/main_tree.xml` (rebuild to install) or pass `bt_xml_path` param |
| Add ROS interface (msg/srv/action) | `btcpp_ros2_interfaces/` + update its `CMakeLists.txt` |
| Add a system dep | `docker/Dockerfile.ci` (the only Docker image left) |
| Change colcon args | `.github/workflows/ci.yml` "colcon build" step |

## Things to avoid

- Don't compile `ref/` — it expects a different node graph (DockRobot action, custom interfaces, params). It would need significant porting.
- Don't replace local `btcpp_ros2_interfaces/` with the upstream `BehaviorTree.ROS2/btcpp_ros2_interfaces` — local has more action types.
- Don't add behavior tree node registration outside `BTEngine::registerNodes()` — keeps the engine the single source of truth.
- Don't skip `init()` after constructing `BTEngine` — `RosNodeParams.nh` needs the shared_ptr.
- `BT::RosActionNode::providedPorts()` must call `providedBasicPorts({...})` (it injects standard server-name etc. ports). Don't return raw ports.
- Don't `apt install ros-humble-behaviortree-ros2` — there is no such package; we vendor the source.
