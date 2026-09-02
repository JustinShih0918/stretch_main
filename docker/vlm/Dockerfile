##############################################################################
# On-Thor VLM perception image — NVIDIA Jetson AGX Thor (JetPack 7 / L4T r39).
#
# Runs the semantic-navigation perception half of src/semantic_nav/ on the
# robot's companion compute instead of inside Isaac Sim:
#
#   /rgb   -> locate_anything_node -> /semantic_detection
#   /depth + /camera_info + TF -> projection_node -> /semantic_regions
#
# nav2 (with SemanticTraversabilityLayer), the BT engine and the Stretch
# hardware driver stay on the robot in docker/deploy — this image is the GPU
# side only, and talks to them over DDS on the host network.
#
# Build (from repo root):
#   docker compose -f docker/vlm/compose.yaml build
#
# --- Base image choice (this is the part that bites) -------------------------
# Thor is compute capability sm_110 with a CUDA 13 driver, and the workspace is
# ROS 2 Humble, which only has Ubuntu 22.04 (jammy) debs. Those two pull in
# opposite directions:
#
#   * The NGC PyTorch images used by docker/vln/Dockerfile.jetson ship a
#     sm_110 torch, but they are Ubuntu 24.04 — no Humble, only Jazzy. That
#     would fork this workspace onto a second ROS distro.
#   * So: keep jammy + Humble from apt, and get sm_110 from the PyTorch
#     cu130 aarch64 (SBSA) wheels instead (modules/install_locate_anything.sh).
#     JetPack 7 uses the regular SBSA CUDA stack, so no Jetson-specific wheel
#     index is needed, and the container needs no CUDA toolkit of its own —
#     the wheels vendor the CUDA 13 runtime and the driver comes from the host
#     through `runtime: nvidia`.
#
# Do NOT "simplify" this to the plain PyPI torch wheels: on aarch64 those are
# CUDA 12.x builds that top out at sm_90, print "GPU not supported" on Thor and
# only limp along by JIT-ing compute_90 PTX.
#
# Nothing here is Thor-specific beyond the wheel index, so an Orin (sm_87,
# JetPack 6) build works by passing TORCH_INDEX_URL=.../whl/cu126.
##############################################################################

# Base Image: https://hub.docker.com/_/ubuntu/tags?name=22.04
ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE}
# Automatic platform arg; install_ros.sh requires it to be set.
# Ref: https://docs.docker.com/reference/dockerfile/#automatic-platform-args-in-the-global-scope
ARG TARGETARCH

# Keep downloaded packages for caching purposes
# Ref: https://github.com/moby/buildkit/blob/master/frontend/dockerfile/docs/reference.md#example-cache-apt-packages
RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# Upgrade packages
# Ref: https://pythonspeed.com/articles/security-updates-in-docker/
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get upgrade -y \
    && rm -rf /var/lib/apt/lists/*

# Install common tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    curl \
    git \
    htop \
    iputils-ping \
    less \
    nano \
    net-tools \
    netcat-openbsd \
    python3-pip \
    tmux \
    tree \
    unzip \
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/*

# ROS version configuration
ARG ROS_DISTRO=humble
ENV ROS_DISTRO=$ROS_DISTRO
# Vendored copy of docker/sim/modules/install_ros.sh (unmodified).
COPY modules/install_ros.sh /tmp/install_ros.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    /tmp/install_ros.sh && rm /tmp/install_ros.sh
# Set the CycloneDDS configuration file
ENV CYCLONEDDS_URI=/home/user/cyclonedds.xml

# Setup the required capabilities for the container runtime
# Ref: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html#driver-capabilities
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Build dependencies of the semantic-navigation subset:
#   navigation2 -> nav2_costmap_2d headers for the SemanticTraversabilityLayer
#                  (the layer .so is built here so the robot-side nav2 can load
#                  it from the shared install/ tree; nav2 itself runs on the
#                  robot, not in this container)
#   cv-bridge   -> sensor_msgs/Image <-> numpy for the perception nodes
#   image-transport / rqt-image-view -> inspecting /semantic_detection_viz
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-rqt-image-view \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Arguments for the default user
ARG USERNAME=user
ARG USER_UID=1000

# Install sudo and create a user with sudo privileges
# Ref: https://stackoverflow.com/a/65434659
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y sudo \
    && useradd -m -s /bin/bash -u $USER_UID -G sudo $USERNAME \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME

# Create user bin directories specified in `~/.profile`
# Together with .bashrc, this allows invoking executables in the same shell after installing to either location
RUN mkdir -p /home/$USERNAME/bin /home/$USERNAME/.local/bin
# Create the cache directories with correct ownership. Both are mounted later
# (pip: a build cache mount; huggingface: a named volume in compose.yaml) and
# docker seeds a named volume from the image path, ownership included — if the
# path is missing here the volume lands root-owned and the model download
# fails with EACCES.
RUN mkdir -p /home/$USERNAME/.cache/pip /home/$USERNAME/.cache/huggingface

# LocateAnything (nvidia/LocateAnything-3B) — the default semantic-perception
# backend, Thor-adapted (cu130 wheels, decord built from source, no flash-attn).
#
# The ~7 GB checkpoint is downloaded into the image here (public repo, no HF
# token needed), so the container is usable straight after `compose build` with
# no manual download step. That puts the image around 28 GB; set
# LOCATE_ANYTHING_MODEL=NO to skip it and mount weights over
# /opt/locate_anything at runtime instead.
ARG LOCATE_ANYTHING="YES"
ARG LOCATE_ANYTHING_MODEL="YES"
ARG TORCH_INDEX_URL=""
COPY --chown=$USERNAME:$USERNAME \
     modules/install_locate_anything.sh /tmp/install_locate_anything.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    --mount=type=cache,target=/home/$USERNAME/.cache/pip,sharing=private,uid=$USER_UID,gid=$USER_UID \
    --mount=type=cache,target=/home/$USERNAME/.cache/huggingface,sharing=private,uid=$USER_UID,gid=$USER_UID \
    /tmp/install_locate_anything.sh && rm /tmp/install_locate_anything.sh

# image_transport plugins: compressed / compressedDepth / theora. Without them
# the only declared transport is image_transport/raw, and anything asking for
# another one fails at runtime rather than falling back — rviz2's Image display
# defaults to `compressed` and aborts with "Unable to load plugin for transport
# 'image_transport/compressed_sub' ... Declared types are image_transport/raw_sub".
# It is also what `image_transport republish compressed raw` needs to shrink the
# camera stream on the wire (see docker/vlm/README.md).
#
# Deliberately placed AFTER the checkpoint layer above: adding a package to the
# earlier apt layer invalidates it and forces the ~7 GB model download to run
# again. Layer order carries no meaning here, build time does.
USER root
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*
USER $USERNAME

# Grounding DINO (the alternative backend) is deliberately NOT installed here:
# groundingdino-py has no aarch64 wheel and its MultiScaleDeformableAttention
# CUDA extension has to be compiled against a full CUDA toolkit, which this
# image does not carry (see the base-image note above). Use LocateAnything on
# Thor; run the DINO backend in docker/sim on an x86 box if you need to
# compare the two.

COPY --chown=$USERNAME:$USERNAME .bashrc /home/$USERNAME/.bashrc
COPY --chown=$USERNAME:$USERNAME cyclonedds.xml /home/$USERNAME/cyclonedds.xml

WORKDIR /home/$USERNAME/stretch_main
ENTRYPOINT []
CMD ["/bin/bash"]
