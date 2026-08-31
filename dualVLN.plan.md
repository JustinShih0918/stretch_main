## User

A previous agent produced the plan below to accomplish the user's task. Implement the plan in a fresh context. Treat the plan as the source of user intent, re-read files as needed, and carry the work through implementation and verification.

# StreamVLN / DualVLN Switching and Isaac Sim Benchmark

## Summary

Add `backend:=dualvln` as a launch-time alternative to `streamvln`, while keeping all model inference on remote computers. Both models will produce robot-relative trajectories consumed by one shared ROS trajectory follower:

- StreamVLN actions are converted losslessly into trajectory waypoints.
- DualVLN trajectories remain continuous.
- Both use identical velocity limits and controller settings.
- A benchmark runner resets Isaac Sim, sends identical prompts, records model steps from instruction start through STOP, and computes SPL and FPS metrics.

## Implementation Changes

### Model servers and HTTP contract

- Preserve the existing `/health`, `/reset`, and `/step` endpoints while extending `/step` additively:
  - Request adds optional synchronized `depth_png_b64`, `depth_scale_m`, camera intrinsics, and image timestamp.
  - Response contains exactly one of `actions` or a robot-relative `trajectory`.
  - Add structured timings: total, preprocessing, System 1, and System 2 latency.
  - `/health` reports protocol version, model revision, and capabilities.
- Refactor the ROS HTTP client into a shared implementation used by `streamvln` and `dualvln`; continue accepting the legacy StreamVLN response format.
- Create a separate ROS-free DualVLN container on port `18082`, using the official checkpoint and pinning the inspected InternNav revision `7a5c62400ac45b313d9b709c740b64191556a242`.
- Repair the upstream example-server limitations in the owned adapter:
  - Take the instruction from `/reset` instead of hard-coding it.
  - Validate session IDs and clear model history on reset.
  - Supply paired RGB/depth and actual camera intrinsics.
  - Convert DualVLN action IDs to the existing action vocabulary when it emits STOP/turn/forward instead of a trajectory.
  - Handle its look-down request using the upstream second-pass behavior.
  - Return trajectories as finite `[x_m, y_m]` points in the robot frame.
- Keep StreamVLN on `18080`. Support `STREAMVLN_SERVER_URL`, `DUALVLN_SERVER_URL`, and an explicit `server_url:=` override.

### Shared trajectory execution

- Introduce an internal command union: discrete actions, relative trajectory, or STOP.
- Convert StreamVLN action batches into `(x, y, yaw)` waypoint sequences using the existing 0.25 m and 15° geometry.
- Add a common odometry-closed-loop trajectory follower:
  - 20 Hz control rate.
  - Maximum 0.25 m/s linear and 0.5 rad/s angular velocity.
  - 0.35 m lookahead, 0.12 m final-position tolerance, and 5° explicit-turn tolerance.
  - Acceleration limiting and a six-second no-progress/odometry watchdog.
  - Atomic trajectory replacement for DualVLN replanning.
  - Immediate zero velocity on STOP, cancellation, timeout, or backend error.
- Keep the existing `cmd_vel` and `nav2` modes for compatibility; use the new `trajectory` execution mode for the controlled A/B benchmark.
- Preserve each model’s inference schedule:
  - StreamVLN requests another step after its current action-derived path finishes.
  - DualVLN permits one outstanding request and replans from the latest observation every 0.3 seconds while following the previous path.
- Extend agent status to expose an episode ID, model-step count, inference-active flag, motion-active flag, and terminal reason without removing existing fields.

### Easy switching

- Add `dualvln` to the backend registry and launch choices.
- Support:
  - `./run_vln_demo.sh backend:=streamvln`
  - `./run_vln_demo.sh backend:=dualvln`
- Select the corresponding URL automatically; switching never requires editing YAML or Python.
- Do not support mid-episode switching. Every backend change starts a fresh process and model session.

## Benchmark and Recording

### Route manifest

Add a versioned YAML manifest containing:

- Benchmark and scene IDs.
- Isaac entity path, default `/World/stretch3`.
- Five repetitions, 300-second episode timeout, and success radii `[1.0, 3.0]`.
- For each route:
  - Stable route ID.
  - Exact instruction string.
  - World-frame start position and yaw.
  - World-frame goal position.
  - Verified shortest reference polyline.

Compute shortest-path length from the reference polyline. Reject manifests whose path endpoints do not match the start and goal within 0.25 m. Do not use straight-line distance or the current rolling Nav2 costmap for SPL.

### Isaac reset and orchestration

- Install `ros-humble-simulation-interfaces` in the simulation image and enable Isaac Sim 5.1’s `isaacsim.ros2.sim_control` extension. Use its standard `SetEntityState` service instead of maintaining a custom teleport extension. [Isaac Sim simulation-control documentation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_simulation_control.html)
- Before every trial:
  1. Cancel navigation and publish zero velocity.
  2. Teleport the configured entity with zero twist.
  3. Wait up to ten seconds for five consecutive odometry samples within 5 cm and 3° of the start pose.
  4. Reset the model session.
  5. Start recording and publish the instruction.
- Add a benchmark script that alternates launch-time order across repetitions:
  - Repetitions 1, 3, 5: StreamVLN then DualVLN.
  - Repetitions 2, 4: DualVLN then StreamVLN.
- Keep both remote servers warm; record their `/health` metadata before testing.

### Recorded data

The measurement window begins when the instruction is published and ends on model STOP/done, timeout, or error. Reset and warm-up are excluded.

Write one result directory containing:

- `manifest.yaml`: immutable manifest snapshot.
- `metadata.json`: git revision, server health/model revisions, parameters, and timestamps.
- `steps.jsonl`: one row per completed model `/step`, including backend, route/repetition, request and response poses/times, returned actions or trajectory, client/server/System 1/System 2 latency, and done state.
- `episodes.csv`: one row per trial containing terminal reason, final navigation error, executed path length, shortest-path length, SPL at 1 m and 3 m, model-step count, duration, and rate metrics.
- `summary.json`: per-route and aggregate mean and standard deviation for success, SPL, navigation error, duration, step count, and FPS.

Compute:

- `SPL_r = success_r × shortest_path / max(shortest_path, executed_path)`.
- Success only when the model stops within the selected radius; merely passing through the goal does not count.
- Executed path length by integrating raw world-frame odometry after reset confirmation.
- Separate rates:
  - Camera input Hz.
  - Completed model updates per episode second.
  - Client request/response Hz.
  - Server compute FPS from completed responses divided by accumulated server compute time.
  - DualVLN System 1 and System 2 FPS when their timing is available.
  - Trajectory-follower control Hz.
- Do not save a row for every control tick; retain only counts/rates in episode summaries.

## Public Interfaces

- Launch parameters add `backend:=dualvln`, `execution_mode:=trajectory`, `depth_topic`, `camera_info_topic`, backend-specific server URLs, and trajectory-controller settings.
- Add a typed `/vln/inference_step` event carrying episode/model-step identity, output type and size, timing fields, actions, and trajectory points.
- Extend `VlnStatus` with episode correlation and concurrent inference/motion flags.
- The HTTP extension remains backward compatible: legacy action-only StreamVLN servers continue working.

## Test Plan

- Unit-test legacy and trajectory HTTP responses, invalid sessions, malformed trajectories, timing parsing, depth encoding, synchronized RGB/depth, rotated intrinsics, and backend URL selection.
- Unit-test action-to-trajectory geometry, pure rotations, curved paths, trajectory replacement, STOP cancellation, odometry loss, and no-progress timeout.
- Unit-test manifest validation, reference-path length, odometry path integration, both SPL radii, step-window boundaries, and all FPS formulas.
- Integration-test both backend names using fake remote servers and confirm identical prompts and shared follower parameters.
- Test reset sequencing against a fake `SetEntityState` service, then smoke-test the real Isaac service and odometry verification.
- Acceptance test one short hospital route with the dummy backend before running both real models.
- The full benchmark passes when every route has five rows per backend, prompt hashes match across models, no reset motion enters the metrics, all trials terminate safely, and SPL/FPS can be recomputed from saved data.

## Assumptions

- StreamVLN receives the shared RGB stream; DualVLN receives the same RGB plus synchronized Isaac depth because the currently published deployment code feeds depth to System 1. This sensor difference is recorded in metadata. [DualVLN model card](https://huggingface.co/InternRobotics/InternVLA-N1-DualVLN), [InternNav repository](https://github.com/InternRobotics/InternNav)
- `/rgb`, `/depth`, `/camera_info`, `/odom`, and `/cmd_vel` are available from the hospital USD. Missing camera calibration or reset services causes the benchmark to fail explicitly.
- Operators will populate and verify route coordinates and reference polylines before collecting final SPL results.
- GPU provisioning and checkpoint installation occur only on the remote inference computer and remain outside the ROS simulation image.