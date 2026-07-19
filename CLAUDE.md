# CLAUDE.md — Project notes for future Claude sessions

## What this repo is

A ROS 2 (Humble) workspace whose core is a **BehaviorTree.CPP v4 + behaviortree_ros2** engine ticking a tree that calls **nav2's standard `navigate_to_pose` action**. The same BT engine is the **shared orchestrator** for two runtime environments: an Isaac Sim simulation and the real Stretch robot.

All colcon packages live under `src/`, grouped by role (the grouping is cosmetic — colcon recurses all of `src/`; per-image selection is done with `--packages-up-to`):

```
stretch_main/
├── src/
│   ├── common/                   # built in EVERY image (CI builds only these)
│   │   ├── btcpp_ros2_interfaces/  # vendored interfaces pkg (action/srv/msg)
│   │   ├── behaviortree_ros2/      # vendored BT.CPP↔ROS2 wrapper (RosActionNode, ...)
│   │   ├── bt_engine/              # the BT runner (was engine/)
│   │   └── bt_nav/                 # NavigateToPose BT action node (was nav/)
│   ├── sim/                       # Isaac Sim packages (from j3soon/ros2-essentials stretch3_ws)
│   │   ├── stretch3_navigation/    # nav2 + cartographer/rtabmap launch+config for Isaac
│   │   └── stretch_urdf/           # hello-robot URDF gen tool (pip pkg, NO package.xml → colcon ignores)
│   ├── semantic_nav/              # action-aware semantic traversability (arXiv:2310.08873)
│   │   ├── semantic_traversability/  # C++ nav2 costmap Layer plugin + projection node (env-agnostic)
│   │   ├── semantic_perception/      # Python Grounding DINO node + static region test publisher
│   │   └── vln_policy/               # VLN agent: swappable backends (StreamVLN HTTP) + cmd_vel/nav2 executors
│   └── deploy/                    # vendored from hello-robot/stretch_ros2 (Apache-2.0)
│       └── stretch_nav2/           # on-robot nav2 + slam_toolbox + AMCL
├── docker/
│   ├── ci/      Dockerfile.ci + docker-compose.yaml   # minimal build/test image
│   ├── sim/     Dockerfile + compose.yaml + modules/  # Isaac Sim 5.1 image (GPU/X11)
│   ├── deploy/  Dockerfile + docker-compose.yaml      # on-robot nav/slam image
│   └── vln/     Dockerfile + compose.yaml + server/   # StreamVLN inference server (GPU 1, no ROS)
├── isaacsim/assets/              # Stretch USD assets for Isaac Sim
├── src/common/bt_engine/bt/main_tree.xml   # default BT XML
├── ref/                          # old reference code, NOT a package, NOT built
├── run.sh                        # tmux launcher
└── .github/workflows/ci.yml
```

## Critical project quirks

1. **`src/common/behaviortree_ros2` is vendored** as a single package (not the upstream monorepo — just the wrapper package itself). It is NOT in the apt repos for Humble; do not try to `apt install ros-humble-behaviortree-ros2`.
2. **`src/common/btcpp_ros2_interfaces/` is a vendored copy** (not a submodule) — it has custom `.action` files (`FirmwareMission`, `Navigation`, etc.) that don't exist upstream. **Do not replace it with the upstream version.** If interfaces are missing, add them here. `behaviortree_ros2/package.xml` declares a `<depend>btcpp_ros2_interfaces</depend>` satisfied by this local copy.
3. **`src/deploy/stretch_nav2` is vendored verbatim** from hello-robot/stretch_ros2 (branch `humble`, Apache-2.0 — keep `LICENSE.md`). Don't edit in place; re-sync from upstream (see [src/deploy/README.md](src/deploy/README.md)). Its `navigation.launch.py` pulls `stretch_core` (the hardware driver) + calibrated URDF from the **robot's own hello-robot install** at runtime — those are NOT vendored or built here.
4. **`src/sim/` is ported from `j3soon/ros2-essentials/stretch3_ws`** (which the maintainer co-owns). `stretch3_navigation` is the Isaac Sim nav stack; `stretch_urdf` has no `package.xml` (pip lib) so colcon ignores it. Isaac USD assets live at repo-root `isaacsim/assets/`.
5. **`ref/` is reference-only.** Older, larger `bt_engine.cpp` using a `dock_robot` action + project logic. **Not built** (no `package.xml`). Pattern reference only — don't compile it.
6. **Three Docker environments under `docker/`** (see "Docker environments" below). The `docker/sim/Dockerfile` `COPY`s install scripts from `docker/sim/modules/` — in upstream ros2-essentials those are hard-links to a repo-level `docker_modules/`; here they are **vendored copies** (materialized, not symlinks). If you add/update a sim install step, edit the copy under `docker/sim/modules/`.
7. CI / deploy Dockerfiles do **not** install `behaviortree_ros2` from apt (vendored), nor `libfmt-dev`/`libboost-dev` explicitly — they come transitively via `ros-humble-behaviortree-cpp` and `ros-humble-navigation2`. If those are removed, add the libs back.
8. **VLN model inference never runs in a ROS image.** `docker/vln/` is a separate, ROS-free container (StreamVLN pins torch 2.1.2/cu121 vs sim's cu128) that runs on any GPU machine (GPU selected via `VLN_GPU_ID`, default 0; use 1 when sharing the Isaac machine); the ROS side (`src/semantic_nav/vln_policy/`) is a thin HTTP client pointed at it with `server_url`. The wire contract is normative in `src/semantic_nav/vln_policy/DESIGN.md` — swap models by implementing that contract, never by adding model deps to ROS packages. Demo: `./run_vln_demo.sh` + Isaac playing `isaacsim/assets/stretch3_og_hospital.usda`.

## Architecture

### `bt_engine` ([src/common/bt_engine/](src/common/bt_engine/))
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

### `bt_nav` ([src/common/bt_nav/](src/common/bt_nav/))
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

CI mirrors this: build `docker/ci/Dockerfile.ci` → `colcon build --packages-up-to bt_engine`. No source-clone step; everything we need is in-tree.

## Docker environments

Three independent images under `docker/`, run from the repo root. Each builds a different package subset via `--packages-up-to`; CI builds only `common`.

| Env | Path | Image purpose | Build subset | Run |
|---|---|---|---|---|
| `ci` | `docker/ci/` | minimal build/test (CI + local parity) | `bt_engine` | `docker compose -f docker/ci/docker-compose.yaml run --rm <build\|dev>` |
| `sim` | `docker/sim/` | Isaac Sim 5.1 (GPU/X11/privileged) | `bt_engine stretch3_navigation semantic_traversability semantic_perception` | `docker compose -f docker/sim/compose.yaml run --rm stretch3-ws` |
| `deploy` | `docker/deploy/` | on-robot nav/slam | `bt_engine stretch_nav2` | `docker compose -f docker/deploy/docker-compose.yaml run --rm <build\|nav\|bt\|dev>` |

- Sim first-launch auto-build is scoped in `docker/sim/.bashrc` (only common + `stretch3_navigation`, NOT the deploy packages which need robot-only deps).
- Sim compose mounts the repo at `/home/user/stretch_main` (= `$ROS2_WS`); ci/deploy mount at `/ws`. All use host networking.

## Common tasks → where to edit

| Task | Files |
|---|---|
| Add a BT node | new pkg under `src/common/` *or* extend `bt_nav`; register it in `src/common/bt_engine/src/bt_engine.cpp::registerNodes()` |
| Change tree | `src/common/bt_engine/bt/main_tree.xml` (rebuild to install) or pass `bt_xml_path` param |
| Add ROS interface (msg/srv/action) | `src/common/btcpp_ros2_interfaces/` + update its `CMakeLists.txt` |
| Add a CI system dep | `docker/ci/Dockerfile.ci` |
| Add a sim install step | `docker/sim/modules/*.sh` (vendored copies) + `docker/sim/Dockerfile` |
| Add a deploy system dep | `docker/deploy/Dockerfile` |
| Tune sim nav2/SLAM | `src/sim/stretch3_navigation/config/` + `launch/` |
| Semantic traversability (costmap layer / VLM perception) | `src/semantic_nav/` (see [src/semantic_nav/README.md](src/semantic_nav/README.md)); layer wired into `src/sim/stretch3_navigation/config/nav2_params.yaml` |
| Re-sync vendored deploy nav/slam | see [src/deploy/README.md](src/deploy/README.md) |
| Change colcon args | `.github/workflows/ci.yml` "colcon build" step |

## Things to avoid

- Don't compile `ref/` — it expects a different node graph (DockRobot action, custom interfaces, params). It would need significant porting.
- Don't replace local `src/common/btcpp_ros2_interfaces/` with the upstream `BehaviorTree.ROS2/btcpp_ros2_interfaces` — local has more action types.
- Don't edit `src/deploy/stretch_nav2/` by hand — it's vendored verbatim (Apache-2.0). Re-sync from upstream instead.
- Don't try to build the deploy or sim packages in the CI image — CI is `--packages-up-to bt_engine` on purpose (Isaac/robot deps aren't present).
- Don't symlink `docker/sim/modules/` back to an external `docker_modules/` — they are intentionally vendored copies so the sim image builds standalone.
- Don't add behavior tree node registration outside `BTEngine::registerNodes()` — keeps the engine the single source of truth.
- Don't skip `init()` after constructing `BTEngine` — `RosNodeParams.nh` needs the shared_ptr.
- `BT::RosActionNode::providedPorts()` must call `providedBasicPorts({...})` (it injects standard server-name etc. ports). Don't return raw ports.
- Don't `apt install ros-humble-behaviortree-ros2` — there is no such package; we vendor the source.
