# `docker/vlm` — VLM perception on NVIDIA Jetson AGX Thor

Runs the **GPU half** of [`src/semantic_nav/`](../../src/semantic_nav/) on the
robot's companion compute instead of inside Isaac Sim. Same ROS packages, same
topics — only the camera topic names, the target frame and the torch build
differ from the sim setup.

```
                Thor (this image)                         robot (docker/deploy)
 /rgb ───────► locate_anything_node ─► /semantic_detection
                       │
 /depth ──────► projection_node ─────► /semantic_regions ──► nav2
 /camera_info           │                                    (SemanticTraversabilityLayer)
 + TF                   └────────────► /semantic_detection_viz  ──► planner ─► /cmd_vel

```

Everything is plain DDS on the host network, so the Thor and the robot must
share a subnet, a `ROS_DOMAIN_ID` **and** an RMW implementation (both default
to CycloneDDS here and in `docker/deploy`).

## Quick start (on the Thor)

```bash
# the LocateAnything worker code — NVIDIA's Eagle repo, not vendored here
# (gitignored: it is a nested clone). locate_anything_node imports
# Embodied/locateanything_worker.py from it, so it must land at this exact path
# (or change worker_path in config/vlm_pipeline.yaml). The workspace is
# bind-mounted, so cloning it on the host is enough; sparse checkout keeps it
# at ~2.6 MB:
git clone --depth 1 --filter=blob:none --sparse \
    https://github.com/NVlabs/Eagle src/models/Eagle
git -C src/models/Eagle sparse-checkout set Embodied

# from the repo root:
docker compose -f docker/vlm/compose.yaml build             # incl. the ~7 GB checkpoint
docker compose -f docker/vlm/compose.yaml run --rm build     # colcon, once
docker compose -f docker/vlm/compose.yaml up -d              # perception + projection + viz
docker compose -f docker/vlm/compose.yaml logs -f perception

```

**The model checkpoint is not your problem**: `modules/install_locate_anything.sh`
downloads `nvidia/LocateAnything-3B` during the image build (public repo, no HF
token) and bakes it in at `/opt/locate_anything/LocateAnything-3B`, which is
exactly where `locate_anything_params.yaml` looks. The download retries 3× and
the build fails loudly if the resulting tree is incomplete, rather than leaving
you a container that only reports "model unavailable" at runtime.

That costs image size (~28 GB). To trade it back for a host copy, build with
`LOCATE_ANYTHING_MODEL: "NO"` and uncomment the `/opt/locate_anything` volume in
`compose.yaml`.

Interactive shell (builds the workspace on first launch, like the sim image):

```bash
docker compose -f docker/vlm/compose.yaml run --rm dev
```

## What it builds

`--packages-up-to semantic_perception semantic_traversability` (which pulls in
the vendored `btcpp_ros2_interfaces`). Not `bt_engine`, not `stretch_nav2`,
not `vln_policy`: those run on the robot, in [`../deploy/`](../deploy/).

colcon writes to **`build_vlm/ install_vlm/ log_vlm/`**, not the default
`build/ install/ log/`. The workspace is bind-mounted and the other images
build into it too — `docker/deploy` and `docker/ci` run as **root**, this one as
**uid 1000** — so one shared install tree means permission errors and a mix of
artifacts from two ROS environments. The bases are set in both `compose.yaml`
and `.bashrc`; keep them in sync.

`semantic_traversability` also builds the **costmap layer plugin**, but the
layer has to be loaded *inside the nav2 process*, which lives in
`docker/deploy`. To actually use it on the robot, add `semantic_traversability`
to that image's `--packages-up-to` list and insert
`semantic_traversability_layer` after the obstacle/voxel layer in the nav2
params overlay (step 1–2 of *Real robot* in
[`src/semantic_nav/README.md`](../../src/semantic_nav/README.md)).

## Configuration

Everything about the pipeline — topic names, target frame, region memory, model
options — lives in **one params file**, [`config/vlm_pipeline.yaml`](config/vlm_pipeline.yaml).
All three services read it; nothing is configured on the compose command line
any more.

```bash
$EDITOR docker/vlm/config/vlm_pipeline.yaml
docker compose -f docker/vlm/compose.yaml restart
```

No rebuild is needed: the workspace is built `--symlink-install` and the file is
read from the bind-mounted repo, so a restart is enough.

Its defaults are the **RealSense D435i** topics published by the Stretch driver
(`/camera/color/image_raw`, `/camera/aligned_depth_to_color/*`) with
`target_frame: map` — `map` rather than sim's ground-truth `odom` because the
robot localizes with AMCL, which keeps the polygons attached to the world across
localization corrections.

### Several setups side by side

Copy the file and select it with `VLM_CONFIG` (a path *inside* the container,
where the repo is mounted at `/home/user/stretch_main`).
[`config/isaac.yaml`](config/isaac.yaml) ships as a worked example — the same
pipeline pointed at Isaac Sim (`/rgb`, `/depth`, `/camera_info`, `odom`):

```bash
VLM_CONFIG=/home/user/stretch_main/docker/vlm/config/isaac.yaml \
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  docker compose -f docker/vlm/compose.yaml up -d
```

A ROS 2 params file is **not** layered over another one, so a copy must be
self-contained — `isaac.yaml` repeats every key rather than only the ones that
differ. The top-level keys are node names and must match the names compose
starts the nodes under (`locate_anything_node`, `semantic_projection_node`,
`detection_viz_node`); rename one without renaming the other and its parameters
are silently ignored.

### Still environment variables

These are container-level, not pipeline settings, so they stay in the
environment:

| Env var | Default | Purpose |
|---|---|---|
| `VLM_CONFIG` | `.../config/vlm_pipeline.yaml` | which params file the services load |
| `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` | `0` / `rmw_cyclonedds_cpp` | must match the robot |
| `CYCLONEDDS_URI` | `.../docker/cyclonedds-eth.xml` | DDS config; see below |
| `CYCLONEDDS_IFACE` | `enP2p1s0` | this Thor's wired NIC |
| `LOCATE_ANYTHING_DIR` | `~/models/locate_anything` | host checkpoint dir — only if you opt out of the baked-in weights |

## Images go over the wired link

A 1280x720 `rgb8` frame is 2.76 MB — ~663 Mbit/s at 30 Hz, which Wi-Fi cannot
carry. DDS is therefore pinned to the Ethernet NIC by
[`docker/cyclonedds-eth.xml`](../cyclonedds-eth.xml) (Thor `192.168.100.2`,
robot `192.168.100.1`), not the `autodetermine` config baked into the image.

**Both ends must be pinned.** The robot's stack — the `docker/deploy` services
*and* anything from its native hello-robot install (the RealSense driver, nav2)
— needs the same two variables, with its own NIC name:

```bash
export CYCLONEDDS_URI=<repo>/docker/cyclonedds-eth.xml
export CYCLONEDDS_IFACE=<robot wired NIC>   # ip -brief addr
```

Pin one side only and the two never discover each other: `ros2 topic list`
drops to the local topics. That is the symptom to look for.

Raise the host socket buffers as well — the stock 208 kB limit is smaller than
a single frame, which shows up as stuttering or missing images, not an error:

```bash
echo 'net.core.rmem_max=134217728' | sudo tee /etc/sysctl.d/60-cyclonedds.conf
sudo sysctl --system
```

`network_mode: host` means the host value is what the containers get.

Cheaper still: `/camera/color/image_raw/compressed` already exists, so
`ros2 run image_transport republish compressed raw ...` on this side cuts the
wire traffic ~30x. The VLM only consumes ~0.7 fps.

Switch the active landmark at runtime by publishing to `/semantic_instruction`;
the traversable/blocking attribute map is
`semantic_perception/config/semantic_targets.yaml`.

## Why this base image

This is the awkward part, and it is why this directory exists instead of a
`docker/sim` build arg:

* the workspace is **ROS 2 Humble**, which only has **Ubuntu 22.04** debs;
* Thor is **sm_110** with a **CUDA 13** driver, and the NGC PyTorch images that
  ship an sm_110 torch (used by [`docker/vln`](../vln/)) are Ubuntu 24.04 —
  Jazzy territory.

Resolution: keep jammy + Humble from apt, and take sm_110 from the **PyTorch
cu130 aarch64 (SBSA) wheels**. JetPack 7 uses the ordinary SBSA CUDA stack, so
no Jetson-specific wheel index is needed and the image carries no CUDA toolkit
— the wheels vendor the CUDA 13 runtime, the driver comes from the host via
`runtime: nvidia`.

Verified in this exact configuration on the Thor (`ubuntu:22.04`, `--runtime
nvidia`, `NVIDIA_VISIBLE_DEVICES=all`):

```
torch 2.13.0+cu130
arch_list ['sm_80', 'sm_90', 'sm_100', 'sm_110', 'sm_120']
```

The plain PyPI aarch64 wheels are CUDA 12.x builds that stop at `sm_90`: they
report "GPU not supported" on Thor and only limp along by JIT-ing `compute_90`
PTX. Do not "simplify" the install to `pip install torch`.

`NVIDIA_VISIBLE_DEVICES` is not optional either — with `runtime: nvidia` but no
visible-devices setting, the toolkit injects nothing and
`torch.cuda.is_available()` is `False` on a machine whose GPU works fine.

### The setuptools trap

`install_locate_anything.sh` ends by **uninstalling the user-site setuptools**,
which looks gratuitous and is not. The pip installs pull setuptools 78 into
`~/.local`, where it shadows jammy's 59.6.0; ROS Humble's `ament_cmake_python`
then runs `setup.py egg_info` against it, and it resolves `packaging` to the
system 21.3:

```
TypeError: canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'
Failed   <<< btcpp_ros2_interfaces
```

...which takes down the whole `colcon build`. The script asserts afterwards
that the shadowing is gone, so a future dependency bump fails at image-build
time with a clear message instead of at workspace-build time with that one.

## Verified on hardware

Built and run on a Jetson AGX Thor (L4T R39.2, driver 595.78, CUDA 13.2):

* `torch 2.13.0+cu130`, `numpy 1.26.4`, `decord 0.6.0`, `setuptools 59.6.0`
  (system, unshadowed), `cv_bridge` + `cv2 4.5.4` + `transformers 4.57.1`
  importable together;
* `torch.cuda.is_available() True`, device `NVIDIA Thor`, capability `(11, 0)`,
  bf16 matmul and `scaled_dot_product_attention` both run;
* the baked-in checkpoint loads: `AutoConfig` → `LocateAnythingConfig` and
  `AutoProcessor` → `LocateAnythingProcessor` through `trust_remote_code`;
* `colcon build` → `3 packages finished` (`btcpp_ros2_interfaces`,
  `semantic_perception`, `semantic_traversability`);
* `docker compose up -d` → all three nodes start and discover each other over
  CycloneDDS on the host network (`/locate_anything_node`,
  `/semantic_projection_node`, `/detection_viz_node`; topics
  `/semantic_detection`, `/semantic_regions`, `/semantic_detection_viz`,
  `/semantic_instruction`);
* **real inference**, via the Eagle worker against the baked-in checkpoint:
  model load **9.6 s**, `detect(..., generation_mode="hybrid")` **1.5 s** on a
  640x480 frame, returning the `<ref>label</ref><box>...</box>` format that
  `locate_anything_parser.parse_labeled_boxes` consumes. Both `flash_attn` and
  `magi_attention` log a fallback to `sdpa` and inference proceeds — expected
  on sm_110, and the reason neither is installed.

* **the whole pipeline, driven by a fake RGB-D camera** (a photo + a constant
  2.0 m depth plane + intrinsics + a static `map -> camera_color_optical_frame`
  TF, published at 0.5 Hz): `/semantic_detection` carried `person`
  (traversable) and `chair` (blocked) boxes, `projection_node` turned them into
  `/semantic_regions` polygons at x ~ 1.85-2.15 m in `map` — the geometry a
  2.0 m plane in front of a camera 1.0 m up should produce — and
  `/semantic_detection_viz` published rgb8 frames at ~10 Hz. Verified with the
  camera publishing **BEST_EFFORT** (as a RealSense does) and again with
  **RELIABLE** (as Isaac's bridge does).

`generation_mode` accepts `fast`, `slow` or `hybrid` (the packaged default);
there is no `ar` mode.

### Camera QoS (fixed, but know why)

That fake-camera test first ran with **zero frames delivered**: the image
subscriptions in `semantic_perception` used the default RELIABLE QoS, which is
incompatible with a BEST_EFFORT publisher. DDS does not error — the camera just
logs one `incompatible QoS ... RELIABILITY` line at discovery and the node waits
forever. Isaac Sim hides this because its ROS 2 bridge publishes RELIABLE.

`locate_anything_node`, `grounding_dino_node` and `detection_viz_node` now
subscribe with `BEST_EFFORT / KEEP_LAST / depth=1`, which matches a RealSense
*and* Isaac. If you add another node that consumes camera topics here, do the
same — and keep `depth=1` rather than the stock `qos_profile_sensor_data`
(depth 5), so a ~1.5 s inference step never works through a backlog of stale
frames.

Not yet exercised: a physical camera on the robot.

### Deliberate omissions

* **Grounding DINO** — `groundingdino-py` has no aarch64 wheel and its
  MultiScaleDeformableAttention CUDA extension needs a full CUDA toolkit that
  this image does not carry (`docker/sim/modules/install_cuda_toolkit.sh`
  explicitly bails out on arm64). Use LocateAnything on Thor; compare backends
  in the sim image on x86.
* **flash-attn** — no sm_110 aarch64 wheel; a source build needs hours and
  >32 GB of build RAM. The worker runs on torch's `sdpa` path. Same trade-off
  as `docker/vln/Dockerfile.jetson`.
* **decord** — *not* omitted, though it looks omittable. There is no aarch64
  wheel (neither `decord` nor the `eva-decord` fork), and this node only ever
  passes single frames, so dropping it seems free — but LocateAnything's
  `processing_locateanything.py` imports it at module level and transformers'
  `trust_remote_code` loader runs `check_imports()` over that file, so
  `AutoProcessor.from_pretrained()` fails before a frame is ever decoded. The
  module therefore builds decord 0.6.0 from source, which works because jammy
  ships ffmpeg 4.4 (decord 0.6 does not build against ffmpeg 5+) — one more
  reason this image stays on 22.04.
* **RealSense / librealsense** — the camera driver runs on the robot
  (`docker/deploy`), not here. `docker/sim/modules/install_realsense.sh` is a
  ~30 min source build and would only add weight.
* **Isaac Sim, Isaac Lab, Isaac ROS, Gazebo, cartographer, rtabmap** — sim-only.

## Files

| File | Origin |
|---|---|
| `modules/install_ros.sh` | vendored copy of `docker/sim/modules/install_ros.sh`, unmodified |
| `modules/install_locate_anything.sh` | Thor-adapted copy of the sim module (cu130 wheels, decord from source, checkpoint baked in) |
| `config/vlm_pipeline.yaml` | the pipeline params file — topics, frames, region memory, model options |
| `config/isaac.yaml` | same pipeline pointed at Isaac Sim, as a copy-me template |
| `cyclonedds.xml` | copy of `docker/sim/cyclonedds.xml` |
| `.bashrc` | trimmed copy of `docker/sim/.bashrc` (no Gazebo/Isaac; builds the perception subset) |

Like `docker/sim/modules/`, these are **vendored copies, not symlinks** — the
image must build standalone. If you change an install step in `docker/sim`,
decide explicitly whether it also belongs here.
