# Docker environments

Three independent images, each building a different subset of the colcon
workspace. Run all commands **from the repo root**.

| Env | Purpose | Build subset | Command |
|---|---|---|---|
| [`ci/`](ci/) | Minimal build/test image (CI + local parity) | `bt_engine` | `docker compose -f docker/ci/docker-compose.yaml run --rm build` |
| [`sim/`](sim/) | Isaac Sim 5.1 simulation (GPU/X11/privileged) | `bt_engine stretch3_navigation` | `docker compose -f docker/sim/compose.yaml run --rm stretch3-ws` |
| [`deploy/`](deploy/) | On-robot nav2 + slam_toolbox | `bt_engine stretch_nav2` | `docker compose -f docker/deploy/docker-compose.yaml run --rm build` |

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
an underlay first. Services: `build`, `nav`, `bt`, `dev`.
