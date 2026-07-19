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
2. **Model server** (host, GPU 1) — only for `backend:=streamvln`:

   ```bash
   docker compose -f docker/vln/compose.yaml up -d
   curl localhost:18080/health        # wait for {"status":"ok",...}
   ```
3. **Demo** (sim container, workspace built):

   ```bash
   ./run_vln_demo.sh                  # streamvln + cmd_vel (defaults)
   ```

   Type an instruction in the bottom-right pane, e.g.
   `walk down the hallway, turn left at the reception desk and stop`.
   The right pane streams `/vln/status`; the robot moves in the sim.

No GPU / no server? `./run_vln_demo.sh backend:=dummy` replays a scripted
action sequence through the same executors.

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

## Key topics

| topic | type | direction |
|---|---|---|
| `/vln_instruction` | `std_msgs/String` | in — starts/restarts an episode |
| `/rgb` | `sensor_msgs/Image` | in — streamed to the model |
| `/odom` | `nav_msgs/Odometry` | in — executor feedback |
| `/vln/status` | `btcpp_ros2_interfaces/VlnStatus` | out — full live status |
| `/vln/current_action` | `std_msgs/String` | out — bare action token |
| `/cmd_vel` | `geometry_msgs/Twist` | out (cmd_vel mode) |

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
