# vln_policy

Remote StreamVLN/DualVLN inference for the Stretch robot, with a shared ROS
trajectory follower and a reset-controlled Isaac Sim benchmark. No model
dependency is installed in a ROS image.

## Start a model server

On the remote GPU computer, choose one or keep both warm:

```bash
docker compose -f docker/vln/compose.yaml up -d
curl http://localhost:18080/health

docker compose -f docker/dualvln/compose.yaml up -d
curl http://localhost:18082/health
```

StreamVLN is served on 18080. DualVLN uses the official checkpoint and pinned
InternNav adapter on 18082. Both containers are ROS-free.

## Switch backends

With Isaac Sim playing the hospital scene:

```bash
./run_vln_demo.sh backend:=streamvln
./run_vln_demo.sh backend:=dualvln
```

The launch defaults to `execution_mode:=trajectory`, so both commands use the
same velocity limits and controller. Select remote hosts without editing YAML:

```bash
STREAMVLN_SERVER_URL=http://stream-host:18080 \
  ./run_vln_demo.sh backend:=streamvln

DUALVLN_SERVER_URL=http://dual-host:18082 \
  ./run_vln_demo.sh backend:=dualvln
```

An explicit `server_url:=http://...` takes precedence. A backend switch starts
a new launch process/session; mid-episode switching is unsupported.

DualVLN needs synchronized `/rgb` and `/depth` plus `/camera_info`. StreamVLN
uses the same RGB stream. The hospital asset's default `clockwise_90` rotation
is applied consistently to RGB, depth, and calibration.

Legacy modes remain available:

```bash
./run_vln_demo.sh backend:=streamvln execution_mode:=cmd_vel
./run_vln_demo.sh backend:=streamvln execution_mode:=nav2
./run_vln_demo.sh backend:=dummy dummy_actions:=FORWARD,TURN_LEFT,STOP
```

## Benchmark

The simulation image installs `simulation_interfaces`. Start Isaac Sim through
the wrapper that enables `isaacsim.ros2.sim_control`, open
`isaacsim/assets/stretch3_og_hospital.usda`, and press Play:

```bash
isaac-sim-ros-control
ros2 service type /set_entity_state
```

Review and physically verify every start, goal, and navmesh reference polyline
in `config/benchmark_manifest_v1.yaml` before collecting final SPL numbers.
Then run:

```bash
./run_vln_dummy_benchmark_smoke.sh
```

That first exercises the real Isaac reset service, odometry confirmation, and
shared follower with the dummy backend. After it passes, run both models:

```bash
STREAMVLN_SERVER_URL=http://stream-host:18080 \
DUALVLN_SERVER_URL=http://dual-host:18082 \
./run_vln_benchmark.sh
```

The runner records both `/health` payloads, alternates model order across five
repetitions, starts each measurement only after reset-confirming odometry and
remote-session reset, and writes:

- `manifest.yaml`
- `metadata.json`
- `steps.jsonl`
- `episodes.csv`
- `summary.json`

Success is model STOP within 1 m or 3 m; passing through a goal does not count.
Executed length is integrated from raw world-frame odometry. Shortest length
comes only from the manifest reference polyline. See [DESIGN.md](DESIGN.md)
for the protocol and exact metric definitions.

A failed launch or trial aborts the run, but every trial already collected is
still written to the result directory with `"complete": false` and the error
in `metadata.json`. Only a run that finishes every route/backend/repetition
and passes the prompt-hash check is marked complete.

## Main ROS interfaces

| name | type | purpose |
|---|---|---|
| `/vln_instruction` | `std_msgs/String` | start an instruction |
| `/rgb` | `sensor_msgs/Image` | shared RGB observation |
| `/depth` | `sensor_msgs/Image` | DualVLN synchronized depth |
| `/camera_info` | `sensor_msgs/CameraInfo` | rotated calibration |
| `/odom` | `nav_msgs/Odometry` | closed-loop feedback/metrics |
| `/cmd_vel` | `geometry_msgs/Twist` | shared follower output |
| `/vln/status` | `VlnStatus` | latest correlated status |
| `/vln/inference_step` | `VlnInferenceStep` | one completed model update |

The status includes episode ID, completed model steps, concurrent
inference/motion flags, terminal reason, and controller-tick count while
retaining the original fields.

## Tests

```bash
colcon build --packages-up-to vln_policy --symlink-install
colcon test --packages-select vln_policy
colcon test-result --verbose
```

The tests are GPU/simulator-free: remote servers, Isaac's
`/set_entity_state`, and the agent's own model calls are all faked. The reset
sequencing test needs `simulation_interfaces` (present in the sim image) and
skips itself elsewhere. A real collection additionally requires the Isaac
`/set_entity_state` smoke test and verified route geometry.
