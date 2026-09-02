# vln_policy — Vision-Language Navigation agent

Standalone VLN verification pipeline: a natural-language instruction drives
the Stretch robot through a swappable VLN model (StreamVLN first), with the
current commanded action visible live. No BT engine involved; the future
`FollowInstruction` BT node will wrap this same agent (see
[DESIGN.md](DESIGN.md)).

```
/vln_instruction ─► vln_agent_node ─► backend (HTTP) ─► StreamVLN server
      (String)          │                                 (docker/vln, x86 or Thor)
                        │  discrete actions: FORWARD / TURN_LEFT / TURN_RIGHT / STOP
                        ├─► cmd_vel executor  (velocity bursts, closed-loop /odom)
                        └─► nav2 executor     (relative waypoint -> navigate_to_pose)
      /vln/status ◄─────┘  (VlnStatus: state, current_action, pending, step_count)
```

Two entry points run the same nodes against different worlds:

| | Isaac Sim | real Stretch robot |
|---|---|---|
| script | `./run_vln_demo.sh` | `./run_vln_robot.sh` |
| runs in | the sim container | `docker/deploy`'s `vln` / `vln-console` service |
| launch | `vln_demo.launch.py` | `vln_robot.launch.py` |
| params | `config/vln_agent_params.yaml` | `config/vln_robot_params.yaml` |
| nav2 | launched by us (`stretch3_navigation` params) | already running on the robot — never started here |
| camera | Isaac bridge (`/rgb`, RELIABLE) | `realsense2_camera` (BEST_EFFORT) |
| `use_sim_time` | `True` | `False` |

## Quickstart (Isaac Sim hospital scene)

1. **Sim** — launch Isaac Sim, open `isaacsim/assets/stretch3_og_hospital.usda`,
   press Play. Verify `/rgb` and `/odom` are publishing.
2. **Model server** (any machine with ~17 GB of free VRAM; needs only docker
   + nvidia-container-toolkit + this repo, no ROS) — for `backend:=streamvln`,
   pick the variant matching the hardware:

   ```bash
   # x86_64 dGPU (VLN_GPU_ID=1 shares the Isaac Sim machine, Isaac keeps GPU 0)
   docker compose -f docker/vln/compose.yaml up -d
   # NVIDIA Jetson AGX Thor (JetPack 7 / CUDA 13 / sm_110)
   docker compose -f docker/vln/compose.jetson.yaml up -d

   curl localhost:18080/health        # wait for {"status":"ok",...}
   ```

   The two are mutually exclusive — same container name, same port. The Thor
   variant bind-mounts the ~15 GB checkpoint from the host instead of baking
   it into the image; see the header of `docker/vln/compose.jetson.yaml` for
   how to fetch it. The server is plain HTTP — keep port 18080 inside the lab
   network.
3. **Demo** (sim container, workspace built):

   ```bash
   ./run_vln_demo.sh                  # lab server 140.114.89.63 (default)
   ./run_vln_demo.sh server_url:=http://localhost:18080       # local server
   VLN_SERVER_URL=http://other-host:18080 ./run_vln_demo.sh   # another server
   ```

   From the sim machine, `curl http://<server-ip>:18080/health` first to
   confirm reachability (firewall).

   Type an instruction in the bottom-right pane, e.g.
   `walk down the hallway, turn left at the reception desk and stop`.
   The right pane streams `/vln/status`; the robot moves in the sim.

No GPU / no server? `./run_vln_demo.sh backend:=dummy` replays a scripted
action sequence through the same executors.

## Real robot (`./run_vln_robot.sh`)

On the robot the camera driver and nav2 come from the hello-robot stack, so
`vln_robot.launch.py` starts **only** the VLN nodes and attaches to whatever
is already publishing. Starting a second nav2 would fight the first one over
`/cmd_vel`, which is why the robot launch has no bringup of its own.

**Where each piece runs** — same split as the semantic-perception pipeline:

```
hello-robot stack   camera + nav2
                      │  DDS: image, odom / cmd_vel or navigate_to_pose
docker/deploy       vln_agent_node + vln_viz_node      (vln / vln-console)
                      │  HTTP :18080
docker/vln          StreamVLN server                   (this host or another)
```

Everything but the HTTP hop is DDS on the host network, so every participant
must share a subnet, a `ROS_DOMAIN_ID` and an RMW implementation (CycloneDDS
by default here, in `docker/vlm` and in the hello-robot install). If the agent
and the robot are on different machines over a marginal link, prefer
`execution_mode:=nav2`: the `cmd_vel` executor closes its loop on odometry
across the network, while nav2 closes its control loop on the robot itself.

1. **Wire the names once.** Edit the `ROBOT_*` block at the top of
   [launch/vln_robot.launch.py](launch/vln_robot.launch.py):

   ```python
   ROBOT_RGB_TOPIC     = "/camera/color/image_raw"
   ROBOT_ODOM_TOPIC    = "/odom"
   ROBOT_CMD_VEL_TOPIC = "/stretch/cmd_vel"
   ROBOT_NAV2_ACTION   = "navigate_to_pose"
   ROBOT_GOAL_FRAME    = "odom"     # frame of nav2-mode waypoints
   ROBOT_MARKER_FRAME  = "odom"     # RViz fixed frame for /vln/*
   ROBOT_RGB_ROTATION  = "none"     # the sim scene needs clockwise_90
   ```

   These become the defaults of the matching launch arguments and are handed
   to both the agent and the viz node, so one edit covers both. Confirm them
   against the running robot first:

   ```bash
   ros2 topic list | grep -Ei 'image|odom|cmd_vel'
   ros2 topic info -v /your/image/topic     # type + QoS
   ros2 action list | grep navigate
   ```

   Speeds, timeouts and the episode cap are separate, in
   [config/vln_robot_params.yaml](config/vln_robot_params.yaml) (slower than
   the sim defaults: `v_lin` 0.15, `v_ang` 0.4).

2. **On the robot** — its own camera driver, and nav2 for `execution_mode:=nav2`.
   Nothing here starts them.

3. **On the Thor: model server** (docker/vln, Jetson variant):

   ```bash
   docker compose -f docker/vln/compose.jetson.yaml up -d
   curl localhost:18080/health        # wait for {"status":"ok",...}
   ```

4. **The VLN nodes** — `docker/deploy`, which builds and runs them:

   ```bash
   C="docker compose -f docker/deploy/docker-compose.yaml"
   $C build && $C run --rm build          # image + colcon, after any edit

   $C run --rm vln-console                # tmux console (= ./run_vln_robot.sh)
   $C run --rm vln-console ./run_vln_robot.sh execution_mode:=nav2
   $C run --rm vln-console ./run_vln_robot.sh backend:=dummy \
       dummy_actions:=FORWARD,TURN_LEFT,STOP
   ```

   `VLN_SERVER_URL` points at a StreamVLN server that is not on this host.

   Headless instead — the launch alone, publish instructions yourself:

   ```bash
   $C run --rm vln                        # ros2 launch vln_robot.launch.py
   ros2 topic pub --once /vln_instruction std_msgs/msg/String \
       "{data: 'go down the hallway and stop at the door'}"
   ros2 run vln_policy vln_status_monitor
   ```

   Before the first launch, confirm the container actually sees the robot:

   ```bash
   ros2 topic hz <the robot's image topic>
   ros2 action list | grep navigate
   ```
   Nothing listed = a `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` mismatch or a
   routing problem, not a VLN issue.

The robot moves as soon as an instruction is sent — keep the runstop within
reach, and prefer `backend:=dummy` in an open area for the first wiring
check (it exercises the topics without the model).

## RViz visualization

`vln_viz_node` runs by default (`viz:=false` to disable) and renders:

* **`/vln/viz_image/compressed`** — the camera frame the model sees, with a HUD:
  instruction, state (color-coded), step count, server latency, and the
  action batch with the currently executing action highlighted.
* **`/vln/viz_markers`** (odom frame) — the commanded batch drawn as a green
  trajectory ribbon on the floor + an orange arrow for the final heading,
  and a floating state label above the robot.
* **`/vln/path`** — breadcrumbs of the actually executed motion (compare
  against the ribbon to see command vs. execution). The path is cleared on
  every new `/vln_instruction` and stops recording at `DONE` or `ERROR`, so it
  only shows the current episode.

Open RViz preconfigured with all of it:

```bash
./run_vln_demo.sh server_url:=http://140.114.89.63:18080 rviz:=true   # sim
./run_vln_robot.sh rviz:=true                                         # robot
```

On the robot, `rviz2` is in the deploy image and the compose mounts the X
socket plus GDM's auth cookie (`/run/user/1000/gdm/Xauthority`) for you, so
only the display has to be right:

```bash
DISPLAY=:1 docker compose -f docker/deploy/docker-compose.yaml run --rm \
    vln-console ./run_vln_robot.sh rviz:=true
```

Another uid or display manager? Override the mount's source with
`XAUTHORITY=$(ps -ef | grep -m1 "[X]org" | grep -o "/.*Xauthority")`. A wrong
or missing cookie shows up as "Authorization required, but no authorization
protocol specified"; `xhost +local:root` is the blunter fix.
The deploy image has no GPU passthrough, so RViz renders through software GL —
usable, not fast. The shipped config expects `/laser_scan`; the robot's lidar
is usually `/scan`, so repoint that display or drop it.

or add the displays to an existing RViz session (fixed frame `odom`; config
lives at `vln_policy/config/vln_demo.rviz`). In Nav2 mode the config also
shows the global/local costmaps, laser scan, and robot footprint. The Nav2
global/local plan displays are included but disabled by default to avoid
overlaying extra long lines on `/vln/path`; enable them when debugging Nav2.
All VLN snapshot displays use a keep-last depth of one.

The top-right tmux pane is a compact latest-only status page. It refreshes in
place instead of retaining every `/vln/status` heartbeat.

The hospital scene publishes its camera rolled sideways, so the launch
defaults `rgb_rotation:=clockwise_90`. The same correction is applied to the
JPEG sent to StreamVLN and `/vln/viz_image`, ensuring RViz shows exactly the
upright orientation used by the model. Use `rgb_rotation:=none` with a camera
that already publishes upright images.

## Image latency

Measured on the lab robot (RealSense at 1280x720, 15 Hz, over the wired
robot↔Thor link), age = arrival time minus the frame's own header stamp:

| stage | before | after |
|---|---|---|
| camera stream on the wire | 2700 KiB/frame, 333 Mbit/s | 261 KiB, 32 Mbit/s (`rgb_transport:=compressed`) |
| `/vln/viz_image` age | 9.6 s | 0.96 s |
| delay added by `vln_viz_node` | 321 ms | 123 ms |
| HUD frame published | 2131 KiB raw | 33 KiB JPEG |

What produced that, and the knobs:

* **`rgb_transport:=compressed`** (default in `vln_robot.launch.py`)
  subscribes to `<rgb_topic>/compressed`, the JPEG the RealSense driver
  already publishes — a tenth of the bytes for the same pixels. `raw` stays
  the default in sim, where Isaac publishes no compressed topic. Decoding is
  done with `cv2`, so no `image_transport` plugin is needed on the
  subscriber side.
* **`viz_image_transport`** (default `compressed`) — the HUD used to be
  republished as a full-resolution raw image: 2 MB per frame, 175 Mbit/s at
  10 Hz, for a picture only a human looks at. RViz reads
  `/vln/viz_image/compressed`; the shipped config already points there
  (RViz2 picks the transport from the topic name, and the deploy image
  installs `image-transport-plugins` so it can decode). `raw` or `both` if
  something else needs the uncompressed topic.
* **`viz_decode_reduction`** (default 2) decodes the JPEG at 1/N scale —
  libjpeg does it during decoding, so it is much cheaper than decoding full
  size and resizing after. The HUD is downscaled anyway
  (`viz_image_max_width`, default 640).
* The HUD also skips re-encoding a frame it already published, so a timer
  running faster than the camera costs nothing.

**Check the link before blaming the pipeline.** The remaining ~0.8 s of
camera age on our robot was not ROS at all: an `ssh -Y` session to the robot
was forwarding a remote RViz over X11 at **1.2 Gbit/s** on the same 2.5 Gb/s
wired link, starving the camera stream (frame rate halved, 15 → 7.5 Hz). Run
RViz locally against the DDS topics instead of forwarding a remote one, and
sanity-check the link with:

```bash
# bytes arriving on the wired NIC, with nothing of ours subscribed
awk '/enP2p1s0/ {print $2}' /proc/net/dev; sleep 10; awk '/enP2p1s0/ {print $2}' /proc/net/dev
```

Lowering the camera's resolution or frame rate in the robot's RealSense
launch is the other big lever, and the only one that helps every subscriber
at once.

## Backends (`backend:=`)

| name | what it is | needs |
|---|---|---|
| `streamvln` | HTTP client to the StreamVLN server (`docker/vln/`) | that server (x86 or Thor) |
| `dummy` | scripted action replay (`dummy_actions:=FORWARD,TURN_LEFT,STOP`) | nothing |
| `navila` | adapter slot for a NaVILA/NaVid server speaking the same contract (port 18081) | that server (not vendored yet) |

Swapping models = standing up another server that implements the wire
contract in [DESIGN.md](DESIGN.md) and pointing `server_url` at it.

## Execution modes (`execution_mode:=`)

* `cmd_vel` (default) — each discrete action becomes a velocity burst,
  terminated by odometry displacement (`forward_step_m` / `turn_step_deg`),
  matching StreamVLN's own real-robot deployment. Most standalone; no nav2
  needed.
* `nav2` — each action batch is folded into one relative waypoint sent to
  nav2 (goals in the `odom` frame; in sim the launch brings nav2 up with
  `stretch3_navigation`'s params). Costmaps — including the
  semantic_traversability layer — get veto power over the motion.

### Step geometry (`forward_step_m` / `turn_step_deg`)

How far one action token moves the robot. The VLN-CE reference the pretrained
policies assume is **0.25 m / 15°**, which is what `vln_agent_params.yaml`
(sim) uses. `vln_robot_params.yaml` scales it to **0.5 m / 30°**, because in
`nav2` mode a whole batch collapses into ONE waypoint: doubling the step
doubles the leg nav2 drives and halves the number of accelerate/decelerate
cycles for the same intended path — visibly smoother on the real robot.

The trade-off is real. The policy still believes a token is 0.25 m / 15°, so
the robot travels further than it asked for and corrects on the next frame;
push it too far and trajectories get coarse and start overshooting turns. Set
both back to `0.25` / `15.0` for exact VLN-CE behaviour.

Three places must agree, and all three read the same two parameters: the
executor (motion), `vln_viz_node` (the preview ribbon — hence the
`vln_viz_node` section in the params files), and the reverse-command parser
(`back up 1 m` is 4 steps at 0.25 m but 2 at 0.5 m). In `cmd_vel` mode a
longer step also takes longer at the same speed, so keep `action_timeout_s`
above `forward_step_m / v_lin`.

  Two ways to hand the waypoint over, `nav2_goal_interface:=`:

  | | `action` | `topic` |
  |---|---|---|
  | how | `navigate_to_pose` action goal | publishes `PoseStamped` on `/goal_pose`, exactly what RViz's goal tool does |
  | completion | nav2's result: success / abort / cancel | odometry: the robot moved, entered the tolerance, and stood still for `nav2_arrival_settle_s` |
  | abort visible? | yes | no — surfaces as the goal timeout |
  | cancel | real action cancel | re-publishes the current pose to preempt |
  | needs | the same RMW on both ends | nothing (topics interoperate across DDS vendors) |

  Use `action` when the robot and this node run the same
  `RMW_IMPLEMENTATION`; it is strictly more informative. Use `topic` when
  they differ — see the RMW note in Troubleshooting. `vln_robot.launch.py`
  defaults to `topic` because the lab robot runs Fast-DDS while our
  containers run CycloneDDS.

  The arrival check deliberately requires the robot to *move first*: a
  single-step batch can be shorter than the 0.3 m tolerance (it always is at
  the default 0.25 m step), and a turn-only batch never leaves the ball at
  all, so a naive proximity test would report instant arrival and the robot
  would never set off.

## Robot-relative reverse commands

Simple direct instructions such as `move backward`, `back up 50 cm`, and
`reverse 1 meter` are interpreted locally instead of being sent to
StreamVLN. They become odometry-closed-loop `BACKWARD` actions, quantized to
the same `forward_step_m` used by `FORWARD`. In the default `cmd_vel` mode
this publishes negative `linear.x`; in `nav2` mode it requests a relative
waypoint behind the robot and lets the planner choose the path.

The rule is deliberately narrow: `go to the back of the room`, `go back to
the kitchen`, and similar room/place-relative instructions still go to the
visual navigation model. Direct reverse motion uses the forward-camera-blind
side of the robot, so it should only be used where lidar/costmap coverage or
operator supervision makes that safe.

## Key topics

Sim names below; on the robot the three input/output topics come from the
`ROBOT_*` block in `launch/vln_robot.launch.py`. The `/vln/*` topics are the
same everywhere.

| topic | type | direction |
|---|---|---|
| `/vln_instruction` | `std_msgs/String` | in — starts/restarts an episode |
| `/rgb` | `sensor_msgs/Image` | in — streamed to the model |
| `/odom` | `nav_msgs/Odometry` | in — executor feedback |
| `/vln/status` | `btcpp_ros2_interfaces/VlnStatus` | out — full live status |
| `/vln/current_action` | `std_msgs/String` | out — bare action token |
| `/cmd_vel` | `geometry_msgs/Twist` | out (cmd_vel mode) |
| `/vln/viz_image` | `sensor_msgs/Image` | out — camera + HUD overlay (viz node) |
| `/vln/viz_markers` | `visualization_msgs/MarkerArray` | out — action ribbon/arrow + state text (viz node) |
| `/vln/path` | `nav_msgs/Path` | out — executed breadcrumb path (viz node) |

`/vln_instruction` is deliberately separate from `/semantic_instruction`
(perception prompts): a VLN instruction has episode-reset semantics.

## Parameters

See [config/vln_agent_params.yaml](config/vln_agent_params.yaml) (sim) and
[config/vln_robot_params.yaml](config/vln_robot_params.yaml) (robot).
Notables: `max_steps` (episode cap, 150), `action_timeout_s` (per-action
watchdog, 6 s sim / 8 s robot — also catches odometry silence),
`v_lin`/`v_ang` (burst speeds), `nav2_action_name` + `goal_frame` (nav2 mode).

The camera subscriptions in both nodes are `BEST_EFFORT / KEEP_LAST / depth 1`:
`realsense2_camera` publishes BEST_EFFORT and a RELIABLE subscriber would
receive nothing from it, while Isaac's RELIABLE publisher matches either way.

## Tests

```bash
colcon build --packages-up-to vln_policy
python3 -m pytest src/semantic_nav/vln_policy/test/ -q   # no GPU/sim needed
```

`test_backends.py` doubles as the client-side spec of the wire contract.

## Troubleshooting

* `state: ERROR`, detail "server unreachable" — the model server is down;
  `docker compose -f docker/vln/compose.yaml up -d`, then send the
  instruction again (the node stays alive and recovers per episode).
* `/health` returns 503 — image built without the checkpoint; rebuild with
  `STREAMVLN_MODEL: "YES"` in `docker/vln/compose.yaml`.
* Robot never moves in cmd_vel mode — check `/odom` is publishing; the
  executor refuses to move blind and times out after `action_timeout_s`.
* **Goals never reach the robot, but topics look fine** — everything reads
  correctly (`ros2 topic list`, `/odom`, TF) yet nav2 goals, service calls and
  action feedback silently do nothing. Almost always an **RMW mismatch**:
  topics interoperate across DDS vendors, services (and therefore actions) do
  not, because Fast-DDS and CycloneDDS correlate replies differently. Check
  both ends with `echo $RMW_IMPLEMENTATION`; stock Humble is
  `rmw_fastrtps_cpp`, our containers set `rmw_cyclonedds_cpp`. Fix by matching
  them, or keep `nav2_goal_interface:=topic`, which needs only topics.
* `state: ERROR`, detail mentions `navigate_to_pose` — no nav2 action server.
  In sim, `execution_mode:=nav2` makes the launch bring nav2 up; on the robot
  nav2 must already be running, and its action name must match
  `nav2_action_name` (`ros2 action list | grep navigate`).
* On the robot, `waiting for image on <topic>` forever — the topic name in
  the `ROBOT_*` block is wrong, or the camera driver isn't up. Check with
  `ros2 topic hz <topic>`; a QoS mismatch is no longer possible from our
  side (we subscribe BEST_EFFORT).
