# VLN policy design and wire contract

`vln_agent_node` is the model-free ROS boundary between remote VLN inference
and robot motion. StreamVLN and DualVLN share one HTTP client and one
odometry-closed-loop trajectory follower. Model libraries and checkpoints
never enter the ROS/simulation image.

## Protocol v2 (backward compatible)

All servers implement:

- `GET /health`
- `POST /reset`
- `POST /step`

`/health` returns `status`, `backend`, `model`, `model_revision`,
`protocol_version`, `device`, and a `capabilities` object. A server that is
still loading returns HTTP 503.

`/reset` accepts an instruction and must clear every episode-specific history
or cache. It returns a new opaque session ID; issuing another reset makes the
old ID invalid.

```json
{"instruction": "Turn left at reception and stop by the door."}
```

`/step` accepts the protocol-v1 RGB/odometry fields plus optional synchronized
depth, calibration, and image time:

```json
{
  "session_id": "opaque",
  "image_jpeg_b64": "...",
  "depth_png_b64": "...",
  "depth_scale_m": 0.001,
  "camera_intrinsics": {
    "fx": 585.0, "fy": 585.0, "cx": 320.0, "cy": 240.0,
    "width": 640, "height": 480
  },
  "image_timestamp_s": 123.45,
  "odom": {"x": 0.0, "y": 0.0, "yaw": 0.0}
}
```

Depth PNG values multiplied by `depth_scale_m` produce metres. RGB, depth,
and intrinsics are transformed by the same configured right-angle rotation.

A successful response contains exactly one output:

```json
{
  "actions": ["FORWARD", "TURN_LEFT"],
  "done": false,
  "timings": {
    "total_ms": 270.0,
    "preprocessing_ms": 5.0,
    "system1_ms": null,
    "system2_ms": 250.0
  }
}
```

or:

```json
{
  "trajectory": [[0.0, 0.0], [0.25, 0.02], [0.5, 0.08]],
  "done": false,
  "timings": {
    "total_ms": 300.0,
    "preprocessing_ms": 5.0,
    "system1_ms": 30.0,
    "system2_ms": 250.0
  }
}
```

Trajectory points are finite metres in the robot frame at request time: x is
forward and y is left. The action vocabulary remains `STOP`, `FORWARD`,
`TURN_LEFT`, and `TURN_RIGHT`, with 0.25 m / 15 degree geometry. Protocol-v1
StreamVLN responses with `actions`, `done`, and `latency_ms` remain valid.
Unknown actions, malformed/non-finite trajectories, ambiguous responses, stale
sessions, and invalid timings become a recoverable agent `ERROR` state and an
immediate zero velocity.

## Execution and scheduling

`execution_mode:=trajectory` is the controlled A/B mode. StreamVLN actions
are converted one-for-one into relative `(x, y, yaw)` waypoints; zero-distance
turn waypoints preserve explicit 15 degree rotations. DualVLN points remain a
continuous path and atomically replace its prior plan.

The follower runs at 20 Hz with:

- 0.25 m/s maximum linear and 0.5 rad/s maximum angular speed
- 0.35 m lookahead
- 0.12 m final-position and 5 degree explicit-turn tolerances
- 0.5 m/s² linear and 1.0 rad/s² angular acceleration limits
- a six-second physical no-progress watchdog and a separate odometry-loss
  timeout

Every cancel, STOP, timeout, backend error, and shutdown path commands zero.
`cmd_vel` and `nav2` execution remain available for compatibility.

StreamVLN requests the next update after the action-derived path finishes.
DualVLN allows one request at a time and requests a fresh synchronized
observation no faster than every 0.3 seconds while its prior path is moving.
Inference and motion are therefore independent flags in `VlnStatus`.

## ROS interfaces

- `/vln_instruction` (`std_msgs/String`) starts an episode.
- `/vln/status` (`VlnStatus`) is the latest agent snapshot.
- `/vln/inference_step` (`VlnInferenceStep`) is one event per completed HTTP
  step, including episode/step IDs, poses, output, and timing fields.
- `/vln_agent_node/prepare_episode` resets a remote session without starting
  the measurement window; the benchmark publishes the matching instruction
  only after reset completion.
- `/vln_agent_node/cancel` cancels inference correlation and motion.

## Benchmark contract

The versioned YAML manifest owns world-frame starts/goals and verified
reference polylines. Validation rejects paths whose endpoints differ from the
route endpoints by more than 0.25 m. Shortest length is the reference-polyline
length, never straight-line distance or a rolling costmap estimate.

Each trial cancels navigation, zeros velocity, calls Isaac Sim 5.1's standard
`/set_entity_state`, waits for five consecutive odometry samples within 5 cm
and 3 degrees, resets the model, and then starts recording as it publishes the
instruction. Reset and warm-up motion are excluded.

Repetitions 1/3/5 run StreamVLN then DualVLN; 2/4 reverse the order. The
result directory contains only the manifest snapshot, metadata, model-step
JSONL, episode CSV, and summary JSON. Success requires model STOP within the
radius. For radius r:

`SPL_r = success_r * shortest_path / max(shortest_path, executed_path)`

The runner separately reports camera input Hz, completed updates per episode
second, client request/response Hz, server compute FPS, available S1/S2 FPS,
and trajectory-control Hz. Raw control ticks are counted but not written as
rows.
