# Docker environments

Independent images, each building a different subset of the colcon workspace
(`vln/` builds none — it carries no ROS at all). Run all commands **from the
repo root**.

| Env | Purpose | Build subset | Command |
|---|---|---|---|
| [`ci/`](ci/) | Minimal build/test image (CI + local parity) | `bt_engine` | `docker compose -f docker/ci/docker-compose.yaml run --rm build` |
| [`sim/`](sim/) | Isaac Sim 5.1 simulation (GPU/X11/privileged) | `bt_engine stretch3_navigation` | `docker compose -f docker/sim/compose.yaml run --rm stretch3-ws` |
| [`deploy/`](deploy/) | On-robot nav2 + slam_toolbox + the VLN agent | `bt_engine stretch_nav2 vln_policy` | `docker compose -f docker/deploy/docker-compose.yaml run --rm build` |
| [`vlm/`](vlm/) | VLM semantic perception on Jetson AGX Thor | `semantic_perception semantic_traversability` | `docker compose -f docker/vlm/compose.yaml up -d` |
| [`vln/`](vln/) | StreamVLN inference server (ROS-free, HTTP) | — | `docker compose -f docker/vln/compose.yaml up -d` |

The BehaviorTree engine (`bt_engine` / `bt_nav`, in [`../src/common/`](../src/common/))
is shared across all three: it drives the `navigate_to_pose` action server that
sim (Isaac + nav2) or deploy (robot nav2) brings up.

## ci

Same image used by `.github/workflows/ci.yml`. Services: `build` (one-shot
colcon, mirrors CI) and `dev` (interactive shell). Workspace mounts at `/ws`.

## sim

Ported from [j3soon/ros2-essentials](https://github.com/j3soon/ros2-essentials)
`stretch3_ws`. Requires an NVIDIA GPU + `nvidia-container-toolkit`. The install
scripts under [`sim/modules/`](sim/modules/) are vendored copies (upstream uses
hard-links to a repo-level `docker_modules/`). First run:

```bash
docker compose -f docker/sim/compose.yaml up -d volume-instantiation
docker compose -f docker/sim/compose.yaml run --rm stretch3-ws
```

## deploy

Runs on the physical Stretch. Builds the vendored `stretch_nav2` (nav2 +
slam_toolbox + AMCL). The Stretch hardware driver (`stretch_core`) and
calibrated URDF come from the robot's own hello-robot install — source that as
an underlay first. Services: `build`, `nav`, `bt`, `vln`, `vln-console`, `dev`.

`vln` / `vln-console` run the VLN agent (`../src/semantic_nav/vln_policy/`)
against the camera and nav2 already running on the robot — `vln_robot.launch.py`
starts neither. `vln-console` is the three-pane `./run_vln_robot.sh` tmux
console; `vln` is the same launch headless. The model stays in the `vln/`
image and is reached over HTTP (`VLN_SERVER_URL`).

The image also carries `rviz2` for `rviz:=true`. The compose mounts the X
socket and GDM's auth cookie (`/run/user/1000/gdm/Xauthority` by default —
override with `XAUTHORITY=`), so only `DISPLAY` normally has to be passed.
Rendering is software GL — there is no GPU passthrough here.
