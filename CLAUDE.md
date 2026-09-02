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
│   ├── vlm/     Dockerfile + compose.yaml + modules/  # semantic perception on Jetson AGX Thor
│   └── vln/     Dockerfile(.jetson) + compose(.jetson).yaml + server/  # StreamVLN inference server (no ROS)
├── isaacsim/assets/              # Stretch USD assets for Isaac Sim
├── src/common/bt_engine/bt/main_tree.xml   # default BT XML
├── ref/                          # old reference code, NOT a package, NOT built
├── run.sh                        # tmux launcher (BT engine)
├── run_vln_demo.sh               # VLN in Isaac Sim   ─┐ shared tmux layout in
├── run_vln_robot.sh              # VLN on the robot   ─┘ scripts/vln_tmux.sh
└── .github/workflows/ci.yml
```

## Critical project quirks

1. **`src/common/behaviortree_ros2` is vendored** as a single package (not the upstream monorepo — just the wrapper package itself). It is NOT in the apt repos for Humble; do not try to `apt install ros-humble-behaviortree-ros2`.
2. **`src/common/btcpp_ros2_interfaces/` is a vendored copy** (not a submodule) — it has custom `.action` files (`FirmwareMission`, `Navigation`, etc.) that don't exist upstream. **Do not replace it with the upstream version.** If interfaces are missing, add them here. `behaviortree_ros2/package.xml` declares a `<depend>btcpp_ros2_interfaces</depend>` satisfied by this local copy.
3. **`src/deploy/stretch_nav2` is vendored verbatim** from hello-robot/stretch_ros2 (branch `humble`, Apache-2.0 — keep `LICENSE.md`). Don't edit in place; re-sync from upstream (see [src/deploy/README.md](src/deploy/README.md)). Its `navigation.launch.py` pulls `stretch_core` (the hardware driver) + calibrated URDF from the **robot's own hello-robot install** at runtime — those are NOT vendored or built here.
4. **`src/sim/` is ported from `j3soon/ros2-essentials/stretch3_ws`** (which the maintainer co-owns). `stretch3_navigation` is the Isaac Sim nav stack; `stretch_urdf` has no `package.xml` (pip lib) so colcon ignores it. Isaac USD assets live at repo-root `isaacsim/assets/`.
5. **`ref/` is reference-only.** Older, larger `bt_engine.cpp` using a `dock_robot` action + project logic. **Not built** (no `package.xml`). Pattern reference only — don't compile it.
6. **Docker environments under `docker/`** (see "Docker environments" below). The `docker/sim/Dockerfile` `COPY`s install scripts from `docker/sim/modules/` — in upstream ros2-essentials those are hard-links to a repo-level `docker_modules/`; here they are **vendored copies** (materialized, not symlinks). If you add/update a sim install step, edit the copy under `docker/sim/modules/`. `docker/vlm/modules/` follows the same rule: it holds vendored copies of the sim modules it needs (some Thor-adapted), never symlinks.
7. CI / deploy Dockerfiles do **not** install `behaviortree_ros2` from apt (vendored), nor `libfmt-dev`/`libboost-dev` explicitly — they come transitively via `ros-humble-behaviortree-cpp` and `ros-humble-navigation2`. If those are removed, add the libs back.
8. **VLN model inference never runs in a ROS image.** `docker/vln/` is a separate, ROS-free container (StreamVLN pins transformers 4.45.1, conflicting with the sim image's 4.57); the ROS side (`src/semantic_nav/vln_policy/`) is a thin HTTP client pointed at it with `server_url`. **Two mutually exclusive variants** (same server, same wire contract, different base + pins — both bind :18080): `Dockerfile` + `compose.yaml` for **x86_64 dGPU** (torch 2.1.2/cu121, flash-attn, GPU picked with `VLN_GPU_ID`, checkpoint baked in, tags `stretch-main-vln:latest`); `Dockerfile.jetson` + `compose.jetson.yaml` for **Jetson AGX Thor** (JetPack 7 / CUDA 13 / sm_110, tags `stretch-main-vln:jetson`). On the Thor variant, note: the base must be `nvcr.io/nvidia/pytorch:25.10-py3` — the aarch64 **SBSA** tag, *not* the `-igpu` tags, which are JetPack 6 / Orin and refuse to run on Thor; there is no `VLN_GPU_ID` (one unified-memory GPU); it runs `ATTN_IMPL=sdpa` (no sm_110 flash-attn wheel); and the ~15 GB checkpoint is bind-mounted from the host (`STREAMVLN_CKPT_DIR`) rather than baked in. The wire contract is normative in `src/semantic_nav/vln_policy/DESIGN.md` — swap models by implementing that contract, never by adding model deps to ROS packages. Demo: `./run_vln_demo.sh` + Isaac playing `isaacsim/assets/stretch3_og_hospital.usda`.
   **Sim and robot are separate entry points over the same nodes** (both drive the same three-pane tmux layout from `scripts/vln_tmux.sh`): `./run_vln_demo.sh` → `vln_demo.launch.py` + `config/vln_agent_params.yaml`, `use_sim_time:=True`, and it *brings nav2 up itself* in `execution_mode:=nav2`; `./run_vln_robot.sh` → `vln_robot.launch.py` + `config/vln_robot_params.yaml`, `use_sim_time:=False`, and it starts **only** the VLN nodes because the camera driver and nav2 already run on the robot (a second nav2 would fight the first over `/cmd_vel`). The robot script runs from **`docker/deploy`** — services `vln-console` (its command *is* `./run_vln_robot.sh`) and `vln` (same launch, headless); that image therefore builds `vln_policy` and adds `cv-bridge`/`python3-opencv`/`python3-requests`/`tmux` on top of the nav stack. The StreamVLN server is reached over HTTP (`VLN_SERVER_URL`), everything else over DDS, so `ROS_DOMAIN_ID` + `RMW_IMPLEMENTATION` must match every participant. The deploy compose pins `VLN_INSTALL_BASE=/ws/install` because the bind-mounted workspace also carries the Thor's `install_vlm/`; absent that pin `scripts/vln_tmux.sh` picks the first candidate tree that actually contains `vln_policy` (`VLN_INSTALL_CANDIDATES` per script). **RMW interop, verified on the lab robot:** the Stretch's own stack runs stock Humble = **Fast-DDS**, while every container here sets `rmw_cyclonedds_cpp`. Topics interoperate across DDS vendors, **services and actions do not** (different reply correlation), so the symptom is a connection that looks perfectly healthy — 38 nodes discovered, `/odom` and TF flowing — while every `navigate_to_pose` goal and service call silently gets no reply. Hence `nav2_goal_interface:=topic` (default in `vln_robot.launch.py`): publish the waypoint as a `PoseStamped` on `/goal_pose` like RViz's goal tool does, and complete the batch on odometry (moved → within tolerance → stood still for `nav2_arrival_settle_s`) since a topic goal returns no result. `action` mode stays the better choice once both ends share an RMW. **Image transport:** the robot's camera is subscribed over `<topic>/compressed` (`rgb_transport`, default `compressed` on the robot, `raw` in sim where Isaac publishes no JPEG) — measured 2700 KiB/frame raw vs 261 KiB compressed at 1280x720/15 Hz, which took the displayed frame from 9.6 s to 0.96 s behind. `vln_viz_node` likewise publishes only `/vln/viz_image/compressed` by default (`viz_image_transport`); the raw HUD was 2 MB/frame at 10 Hz. RViz2 (Humble) picks the transport from the *topic name* — there is no transport-hint property, so `vln_demo.rviz` names the `/compressed` topic directly, and `docker/deploy` installs `image-transport-plugins` so RViz can decode it. Our own nodes decode with `cv2` and need no plugin. The robot's topic/action/frame names are one `ROBOT_*` constant block at the top of `vln_robot.launch.py` — edit there, not in the params YAML, which deliberately holds no topic keys (the launch would override them anyway).
9. **`docker/vlm/` is the on-robot (Jetson AGX Thor) home of the semantic-perception nodes** — the same `semantic_perception` + `semantic_traversability` packages the sim image builds, run against the robot's RealSense instead of Isaac. Unlike `docker/vln/` it *is* a ROS image, and that forces an awkward base: the workspace is **Humble (jammy only)**, while Thor is **sm_110 / CUDA 13**, and the NGC PyTorch images carrying an sm_110 torch are **Ubuntu 24.04 (Jazzy)**. Resolution: `ubuntu:22.04` + Humble from apt + torch from the **cu130 aarch64 (SBSA) wheel index** — verified on the hardware to give `arch_list [... sm_110 ...]`, `capability (11, 0)`. Do **not** switch it to plain PyPI torch (CUDA 12.x, tops out at sm_90 on aarch64), and keep `NVIDIA_VISIBLE_DEVICES=all` — with `runtime: nvidia` alone the toolkit injects nothing and `torch.cuda.is_available()` is False. Grounding DINO, flash-attn and librealsense are deliberately absent (no aarch64/sm_110 story, and the camera driver runs on the robot); `decord` conversely MUST be present and is built from source (no aarch64 wheel) because LocateAnything's remote code imports it at module level and `AutoProcessor.from_pretrained` fails without it. Unlike `docker/vln`'s Thor variant, the ~7 GB `nvidia/LocateAnything-3B` checkpoint **is baked into the image** by `modules/install_locate_anything.sh` (~28 GB image) so no manual `hf download` is needed; `LOCATE_ANYTHING_MODEL=NO` + the commented-out volume in `compose.yaml` is the host-copy escape hatch. That module also uninstalls the user-site setuptools — pip's 78.x shadows jammy's 59.6.0 and breaks every ament build with `canonicalize_version() got an unexpected keyword argument`. colcon uses dedicated `build_vlm/ install_vlm/ log_vlm/` bases because this image runs as uid 1000 while ci/deploy build the same bind-mounted workspace as root. Pipeline settings (topics, target frame, region memory, model options) are NOT on the compose command line: they live in one params file, `docker/vlm/config/vlm_pipeline.yaml`, loaded by all three services via `--params-file $VLM_CONFIG`; `config/isaac.yaml` is the same pipeline pointed at Isaac. Params files do not layer, so a copy must repeat every key, and the top-level keys must match the node names compose starts (`locate_anything_node`, `semantic_projection_node`, `detection_viz_node`). The `locate_anything_node` `worker_path` param points at `src/models/Eagle/Embodied` — NVIDIA's Eagle repo, **not** vendored: it is a gitignored sparse clone (`git clone --depth 1 --filter=blob:none --sparse https://github.com/NVlabs/Eagle src/models/Eagle` + `sparse-checkout set Embodied`, ~2.6 MB) supplying `locateanything_worker.py`. Verified on Thor: model load 9.6 s, `detect(generation_mode="hybrid")` 1.5 s/frame, `flash_attn` and `magi_attention` both falling back to sdpa. See `docker/vlm/README.md`.

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

Independent images under `docker/`, run from the repo root. Each builds a different package subset via `--packages-up-to`; CI builds only `common`. (`vln` builds no ROS packages at all — see quirk 8.)

| Env | Path | Image purpose | Build subset | Run |
|---|---|---|---|---|
| `ci` | `docker/ci/` | minimal build/test (CI + local parity) | `bt_engine` | `docker compose -f docker/ci/docker-compose.yaml run --rm <build\|dev>` |
| `sim` | `docker/sim/` | Isaac Sim 5.1 (GPU/X11/privileged) | `bt_engine stretch3_navigation semantic_traversability semantic_perception` | `docker compose -f docker/sim/compose.yaml run --rm stretch3-ws` |
| `deploy` | `docker/deploy/` | on-robot nav/slam | `bt_engine stretch_nav2` | `docker compose -f docker/deploy/docker-compose.yaml run --rm <build\|nav\|bt\|dev>` |
| `vlm` | `docker/vlm/` | VLM semantic perception on Jetson AGX Thor; also builds `vln_policy` for `docker/vln`'s agent services | `semantic_perception semantic_traversability vln_policy` | `docker compose -f docker/vlm/compose.yaml up -d` (services `perception`/`projection`/`viz`, plus `build`/`dev` under profile `tools`) |

- Sim first-launch auto-build is scoped in `docker/sim/.bashrc` (only common + `stretch3_navigation`, NOT the deploy packages which need robot-only deps).
- Sim and vlm compose mount the repo at `/home/user/stretch_main` (= `$ROS2_WS`); ci/deploy mount at `/ws`. All use host networking.

## Common tasks → where to edit

| Task | Files |
|---|---|
| Add a BT node | new pkg under `src/common/` *or* extend `bt_nav`; register it in `src/common/bt_engine/src/bt_engine.cpp::registerNodes()` |
| Change tree | `src/common/bt_engine/bt/main_tree.xml` (rebuild to install) or pass `bt_xml_path` param |
| Add ROS interface (msg/srv/action) | `src/common/btcpp_ros2_interfaces/` + update its `CMakeLists.txt` |
| Add a CI system dep | `docker/ci/Dockerfile.ci` |
| Add a sim install step | `docker/sim/modules/*.sh` (vendored copies) + `docker/sim/Dockerfile` |
| Add a deploy system dep | `docker/deploy/Dockerfile` |
| Add an on-Thor perception dep | `docker/vlm/modules/*.sh` (vendored copies) + `docker/vlm/Dockerfile` |
| Retune the on-Thor pipeline (topics, frames, model opts) | `docker/vlm/config/vlm_pipeline.yaml` (no rebuild; `docker compose ... restart`) |
| Tune sim nav2/SLAM | `src/sim/stretch3_navigation/config/` + `launch/` |
| Point the VLN pipeline at the robot's topics/action | `ROBOT_*` block at the top of `src/semantic_nav/vln_policy/launch/vln_robot.launch.py` (tuning: `config/vln_robot_params.yaml`) |
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
- Don't subscribe to camera topics with the default (RELIABLE) QoS in `semantic_perception` — `realsense2_camera` publishes BEST_EFFORT and a RELIABLE subscriber silently receives nothing (Isaac Sim hides this: its bridge publishes RELIABLE). Use `BEST_EFFORT / KEEP_LAST / depth=1`, as the three perception nodes now do; `depth=1` (not the stock `qos_profile_sensor_data`, depth 5) keeps the ~1.5 s/frame VLM off stale frames.
- Don't `apt install ros-humble-behaviortree-ros2` — there is no such package; we vendor the source.
