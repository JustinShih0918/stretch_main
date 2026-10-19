# Scene-graph (OSG) in Isaac Sim

This guide covers running the scene-graph object search against stretch_main's Isaac
hospital: how the pieces connect, the one-time setup, and bringing the stack up each session.
**To record demo videos, see [RECORDING.md](RECORDING.md).**

The scene-graph pipeline is [Object_scene_graph](https://github.com/JustinShih0918/Object_scene_graph)
(OSG), the ASCENT-style object-search agent that builds an object scene graph while it
explores. It is **vendored** at
[src/semantic_nav/object_scene_graph/](../../src/semantic_nav/object_scene_graph/) so we can
change it here without touching the original repo (see its `VENDORED.md`). This directory
holds the glue that runs it against stretch_main's Isaac hospital and Nav2.

## How it fits together

All of it runs in **one container**, the stretch_main sim container (`docker/sim`). No second
container is involved, and ROS is never on the network between containers.

```
stretch-main-sim container (host network, CycloneDDS)
├─ Isaac Sim 5.1          stretch3_og_hospital.usda   → /rgb /depth /camera_info /tf /clock, ← /cmd_vel
├─ Nav2                   stretch3_navigation          ← navigate_to_pose (frame: world)
├─ static TF              camera_link → camera_color_optical_frame
├─ osg_bridge             system python3.10 + rclpy    (OSG scripts/ros2/bridge.sh)
│     ▲  TCP 127.0.0.1:18765
│     ▼
├─ run_robot.py           conda "habitat" env, py3.9   (OSG scripts/run_robot.py; RedNet, MobileCLIP, …)
└─ ollama                 127.0.0.1:11434              (local LLM used by OSG)
```

**Why there's a socket.** rclpy is built for Ubuntu 22.04's python 3.10, and OSG's pipeline
needs python 3.9 because habitat-sim 0.3.1 pins it. So OSG splits into two processes:
- `osg_bridge` is a ROS node on the system python. It subscribes to the camera and TF and
  sends goals to Nav2's `navigate_to_pose`.
- `run_robot.py` is the pipeline. It talks to the bridge over a local TCP socket
  (`bridge listening on ('127.0.0.1', 18765)` / `pipeline connected` in the bridge log).

**Where OSG lives.** OSG is at `src/semantic_nav/object_scene_graph/`. The sim container
bind-mounts the repo, so inside the container that is
`/home/user/stretch_main/src/semantic_nav/object_scene_graph`, and `install_osg_envs.sh`
symlinks `/workspace` to it because OSG's configs expect `/workspace`. The September 2026
videos were recorded from an earlier, separate clone at `/home/user/Object_scene_graph` that
existed only inside the container. The vendored copy replaces it.

## Files

| File | What |
|---|---|
| `run_sg_demo.sh` | The demo recorder. Per target: reset Isaac → restart bridge → clear costmaps → drive to spawn → `ffmpeg` screen capture → `run_robot.py` → stop. See [RECORDING.md](RECORDING.md) |
| `reset_pose.py` | Sends a Nav2 goal to world (0, 0, yaw 0) and checks that the robot arrived (< 0.3 m, < 20°) |
| `open_hospital.py` | Isaac `--exec` script: opens the hospital USD and presses Play |
| `install_osg_envs.sh` | One-off: installs OSG's `habitat` + `ascent` conda envs natively in the sim container (pip pins in OSG's `requirements/`) |
| `RECORDING.md` | How to record demo videos: steps, output files, options, checking takes, troubleshooting |

The Isaac-specific OSG configs are in the vendored tree:
`configs/experiment/isaac_hospital_map.yaml` and `configs/ros2/isaac_hospital.yaml`. They are
local additions that don't exist upstream.

`configs/ros2/isaac_hospital.yaml` records how the sim differs from the real robot:
- Isaac topic names, and `/depth` in 32FC1 metres
- the camera is rolled 90° (`rotate_deg: -90`)
- there is no `map` frame, so planning happens in `world`
- the image header's frame is `camera_link`, which is not an optical frame, so the static TF
  below is needed

The experiment preset `isaac_hospital_map` sets a 720×1280 image after rotation, a 59.2° HFOV
and a 1.3 m camera height.

## One-time setup (inside the sim container)

```bash
docker compose -f docker/sim/compose.yaml run --rm stretch3-ws     # or start the existing container

cd ~/stretch_main
git submodule update --init src/semantic_nav/object_scene_graph/relative_work/ascent   # ASCENT, for the ascent env
./scripts/scene_graph_demo/install_osg_envs.sh                     # conda envs, /workspace link
# model weights go under src/semantic_nav/object_scene_graph/data/weights (gitignored):
# RedNet rednet_semmap_mp3d_40.pth; HF models such as MobileCLIP-S2 are fetched into
# data/weights/hf on first run
sudo apt-get install -y ffmpeg xdotool                              # recorder deps
curl -fsSL https://ollama.com/install.sh | sh                       # local LLM server
```

## Bring the stack up (each session)

**Every** process, Isaac included, needs `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

Run Isaac on its own X display (the recorder uses `:4`). The bridge must be in a tmux window
named `osg_sim:bridge`, with its output tee'd to `/home/user/sim_nav_ws/bridge.log`. The
recorder respawns that window after each Isaac reset and waits for `bridge listening` in
that log.

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp      # before creating the session so every window inherits it
source /opt/ros/humble/setup.bash && source ~/stretch_main/install/setup.bash
mkdir -p ~/sim_nav_ws                             # logs; the recorder reads bridge.log here
tmux new-session -d -s osg_sim -n isaac
# isaac:  DISPLAY=:4 ./isaac-sim.sh --exec ~/stretch_main/scripts/scene_graph_demo/open_hospital.py
tmux new-window -t osg_sim -n nav2   'ros2 launch stretch3_navigation navigation.launch.py 2>&1 | tee ~/sim_nav_ws/nav2.log'
tmux new-window -t osg_sim -n tf     'ros2 run tf2_ros static_transform_publisher --frame-id camera_link \
                                        --child-frame-id camera_color_optical_frame --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5'
tmux new-window -t osg_sim -n ollama 'ollama serve 2>&1 | tee ~/sim_nav_ws/ollama.log'
tmux new-window -t osg_sim -n bridge 'cd /workspace && bash scripts/ros2/bridge.sh +experiment=isaac_hospital_map 2>&1 | tee ~/sim_nav_ws/bridge.log'
```

## Run a single search (no recording)

```bash
cd /workspace
PYTHONPATH=$PWD/src /opt/conda/envs/habitat/bin/python scripts/run_robot.py \
    +experiment=isaac_hospital_map ros2.target='trash can' ros2.map_tag=isaac_hospital_trash_can
```

To record it, see [RECORDING.md](RECORDING.md).

## Things to know

- **Restart the bridge after every Isaac Stop/Play.** Its TF buffer runs on wall time and
  keeps the previous session's larger-stamped transforms. Isaac's clock restarts at 0, so the
  bridge would keep using the old transforms. `run_sg_demo.sh` handles the restart; when
  running by hand, respawn the `bridge` window yourself.
- **`/map published by: nobody`** is expected in the sim. Isaac publishes ground-truth
  `world → odom → base_link` and there is no map frame.
- **`POSE JUMP`** means the localisation moved, not the robot. Everything mapped before that
  step is misplaced.
- **To change OSG,** edit `src/semantic_nav/object_scene_graph/` directly and log the change
  in its `VENDORED.md`. Run its unit tests with
  `cd /workspace && PYTHONPATH=src python -m pytest tests/unit -q` (in the habitat env).
