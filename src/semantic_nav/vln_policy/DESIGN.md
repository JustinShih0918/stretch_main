# vln_policy design

## Why

Verify VLN capability in Isaac Sim (hospital scene) before wiring it into
the BT engine. Target architecture (from the project plan):

```
instruction ─► BT engine (future FollowInstruction BT node)
                   │
                   ▼
           VLN action server            ◄── /rgb (D435i / head camera)
        StreamVLN: discrete actions / short-range waypoints
                   │
                   ▼
        waypoint -> NavigateToPose (bt_nav)   or   short actions -> controller
                   │
                   ▼
        global/local costmap (obstacle + semantic_traversability
        + dynamic_mapping-cleaned /map) = safety veto layer
```

This package is the standalone middle of that diagram: swappable model
backends, both execution paths, live status output. The BT wrapper comes
later (see "Future BT hook").

## Architecture

```
             ┌────────────────────────── vln_agent_node ──────────────────────────┐
/vln_instruction ─► state machine (20 Hz):                                        │
             │   IDLE ─► RESETTING ─► [ THINKING ─► EXECUTING ]* ─► DONE / ERROR  │
/rgb ────────►   latest-frame cache      │              │                         │
/odom ───────►   OdomPose cache          │              │                         │
             │                   VLNBackend.step()   executor.tick()              │
             │                   (worker thread,        │                         │
             │                    HTTP or scripted)     ├─ CmdVelExecutor ─► /cmd_vel
             │                                          └─ Nav2WaypointExecutor ─► navigate_to_pose
             │   /vln/status + /vln/current_action on every transition + 2 Hz     │
             └────────────────────────────────────────────────────────────────────┘
```

* **Backends** (`vln_policy/backends/`): `VLNBackend` ABC with
  `reset(instruction)` / `step(rgb, odom) -> StepResult(actions, done,
  detail)`. Registry in `backends/__init__.py` (lazy factories, mirroring
  `PERCEPTION_BACKENDS` in `semantic_navigation.launch.py`). The action
  vocabulary and its geometry (`FORWARD_M = 0.25`, `TURN_RAD = 15°`) live in
  `backends/base.py` and are shared with the executors.
* **Executors** (`action_executor.py`): pure Python, ROS I/O injected as
  callbacks, unit-tested without rclpy.
  * `CmdVelExecutor`: one action at a time as a velocity burst; completion
    by odometry displacement, not time; per-action watchdog
    (`action_timeout_s`) that also catches odometry silence; zero-twist
    between actions and on any stop path.
  * `Nav2WaypointExecutor`: folds a whole action batch into one SE(2)
    relative pose (`compose_relative`), expressed in the `odom` frame at
    submit time, sent as a single `navigate_to_pose` goal. STOP truncates
    the batch. Nav2's costmaps (semantic_traversability included) can veto.
* **Model inference is out-of-process.** StreamVLN pins python/torch
  versions incompatible with the ROS image, and a 7B model must not load
  inside a ROS callback context. The node only speaks HTTP.

## Wire contract (normative)

Every VLN model server must implement exactly this; the ROS side never
changes when the model does. `test/test_backends.py` is the executable
client-side spec; `docker/vln/server/app.py` is the reference server.

### `GET /health`

* 200 `{"status": "ok", "backend": "<name>", "model": "<id>", "device": "cuda:0"}`
* 503 with a JSON `detail` when the model is not loaded.

### `POST /reset` — start an episode

Request: `{"instruction": "<natural language>"}`
Response 200: `{"session_id": "<opaque>"}`

Must clear all episode state (frame history, KV cache, memory tokens). A
later `/reset` invalidates prior sessions (single-session servers are fine
— StreamVLN's KV cache is global).

### `POST /step` — one frame in, an action batch out

Request:
```json
{"session_id": "<from /reset>",
 "image_jpeg_b64": "<base64 JPEG, robot's forward RGB>",
 "odom": {"x": 0.0, "y": 0.0, "yaw": 0.0} | null}
```
Response 200:
```json
{"actions": ["FORWARD", "TURN_LEFT", "TURN_RIGHT", "STOP"],
 "done": false, "latency_ms": 270.0}
```

* `actions`: 1..N tokens from exactly {STOP, FORWARD, TURN_LEFT,
  TURN_RIGHT}; geometry fixed at 0.25 m / 15°. Models with continuous
  outputs (NaVILA velocities, NaVid distances) must quantize server-side —
  the adapter owns that mapping, not the robot client.
* `STOP` anywhere ends the episode (`done` should agree; the client
  truncates at STOP regardless).
* Errors: 409 unknown/stale `session_id`; 400 undecodable image; 503 model
  not loaded. Any non-200 surfaces as a `BackendError` → agent `ERROR`
  state → recoverable by the next instruction.
* JSON + base64 (no multipart/websockets) on purpose: the ROS client needs
  only `requests`, and any exchange is replayable with `curl`.

## Why an owned wrapper instead of StreamVLN's own server

`streamvln/http_realworld_server.py` upstream is Go2-specific: one
`/eval_vln` multipart endpoint, the instruction **hardcoded** in the file,
Flask state as globals. Wrapping StreamVLN's `VLNEvaluator` behind our
contract costs ~200 lines (`docker/vln/server/app.py` reuses their exact
load + step loop) and buys: instruction-per-episode, a documented seam for
NaVILA/NaVid, and contract tests that run without a GPU.

## Serving layout

The model server runs wherever a GPU with ~17 GB free lives — point the agent
at it with `server_url`. Two interchangeable deployments, same wire contract:
GPU 1 of the sim machine (`docker/vln/compose.yaml`, `VLN_GPU_ID=1`, so Isaac
Sim keeps GPU 0), or a Jetson AGX Thor (`docker/vln/compose.jetson.yaml`).
Either way the container sees a single GPU, so it is always `cuda:0` inside. The server binds
0.0.0.0:18080 with host networking; it is unauthenticated HTTP, so keep the
port inside the lab network. The image is ROS-free; the sim container is
model-free — this mirrors StreamVLN's own deployment (robot streams to a
remote 4090 over HTTP with ~0.2 s network overhead, which the per-batch
execution model absorbs).

## Future BT hook (out of scope here)

Phase 2 wraps this agent in a `FollowInstruction` action server:

* new `FollowInstruction.action` in `btcpp_ros2_interfaces`
  (goal: instruction string; feedback: VlnStatus; result: final state),
* `vln_agent_node` grows an action-server front end beside the topic one,
* a `bt_nav`-style `RosActionNode` registered in
  `BTEngine::registerNodes()`, so trees can do
  `<FollowInstruction instruction="..."/>` — with nav2 + semantic
  costmap + dynamic_mapping as the safety veto per the target architecture.
