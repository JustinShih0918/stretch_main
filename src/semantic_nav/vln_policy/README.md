# vln_policy — Vision-Language Navigation agent

Standalone VLN verification pipeline: a natural-language instruction drives
the Stretch robot through a swappable VLN model (StreamVLN first), with the
current commanded action visible live. No BT engine involved; the future
`FollowInstruction` BT node will wrap this same agent (see
[DESIGN.md](DESIGN.md)).

```
/vln_instruction ─► vln_agent_node ─► backend (HTTP) ─► StreamVLN server
      (String)          │                                 (docker/vln, GPU 1)
                        │  discrete actions: FORWARD / TURN_LEFT / TURN_RIGHT / STOP
                        ├─► cmd_vel executor  (velocity bursts, closed-loop /odom)
                        └─► nav2 executor     (relative waypoint -> navigate_to_pose)
      /vln/status ◄─────┘  (VlnStatus: state, current_action, pending, step_count)
```

## Quickstart (Isaac Sim hospital scene)

1. **Sim** — launch Isaac Sim, open `isaacsim/assets/stretch3_og_hospital.usda`,
   press Play. Verify `/rgb` and `/odom` are publishing.
2. **Model server** (any GPU machine with ~24 GB VRAM; needs only docker +
   nvidia-container-toolkit + this repo, no ROS) — for `backend:=streamvln`:

   ```bash
   docker compose -f docker/vln/compose.yaml up -d
   curl localhost:18080/health        # wait for {"status":"ok",...}
   ```

   Sharing the Isaac Sim machine instead? `VLN_GPU_ID=1 docker compose ...`
   so Isaac keeps GPU 0. The server is plain HTTP — keep port 18080 inside
   the lab network.
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

## RViz visualization

`vln_viz_node` runs by default (`viz:=false` to disable) and renders:

* **`/vln/viz_image`** — the camera frame the model sees, with a HUD:
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
./run_vln_demo.sh server_url:=http://140.114.89.63:18080 rviz:=true
```

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

## Backends (`backend:=`)

| name | what it is | needs |
|---|---|---|
| `streamvln` | HTTP client to the StreamVLN server (`docker/vln/`) | server on GPU 1 |
| `dummy` | scripted action replay (`dummy_actions:=FORWARD,TURN_LEFT,STOP`) | nothing |
| `navila` | adapter slot for a NaVILA/NaVid server speaking the same contract (port 18081) | that server (not vendored yet) |

Swapping models = standing up another server that implements the wire
contract in [DESIGN.md](DESIGN.md) and pointing `server_url` at it.

## Execution modes (`execution_mode:=`)

* `cmd_vel` (default) — each discrete action becomes a velocity burst,
  terminated by odometry displacement (0.25 m / 15°), matching StreamVLN's
  own real-robot deployment. Most standalone; no nav2 needed.
* `nav2` — each action batch is folded into one relative waypoint sent to
  `navigate_to_pose` (goals in the `odom` frame; brings up nav2 with
  `stretch3_navigation`'s params). Costmaps — including the
  semantic_traversability layer — get veto power over the motion.

## Robot-relative reverse commands

Simple direct instructions such as `move backward`, `back up 50 cm`, and
`reverse 1 meter` are interpreted locally instead of being sent to
StreamVLN. They become odometry-closed-loop `BACKWARD` actions, quantized to
the same 0.25 m action step used by `FORWARD`. In the default `cmd_vel` mode
this publishes negative `linear.x`; in `nav2` mode it requests a relative
waypoint behind the robot and lets the planner choose the path.

The rule is deliberately narrow: `go to the back of the room`, `go back to
the kitchen`, and similar room/place-relative instructions still go to the
visual navigation model. Direct reverse motion uses the forward-camera-blind
side of the robot, so it should only be used where lidar/costmap coverage or
operator supervision makes that safe.

## Key topics

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

See [config/vln_agent_params.yaml](config/vln_agent_params.yaml). Notables:
`max_steps` (episode cap, 150), `action_timeout_s` (per-action watchdog,
6 s — also catches odometry silence), `v_lin`/`v_ang` (burst speeds).

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
* `state: ERROR`, detail mentions `navigate_to_pose` — nav2 isn't up
  (nav2 mode requires `execution_mode:=nav2` so the launch includes it).
