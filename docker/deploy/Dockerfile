##############################################################################
# On-robot (deploy) image.
#
# Builds the shared BT engine (bt_engine / bt_nav) together with the vendored
# hello-robot nav/slam stack (stretch_nav2 -> nav2 + slam_toolbox + AMCL), so
# the same BehaviorTree that runs in sim can drive the real Stretch robot,
# plus vln_policy — the VLN agent's ROS half (an HTTP client; the model runs
# in docker/vln).
#
# NOTE: the Stretch hardware driver (stretch_core) and the calibrated URDF come
# from the robot's own hello-robot install and are overlaid at runtime — they
# are NOT built into this image. stretch_nav2's navigation.launch.py pulls in
# stretch_core / rplidar launches from that underlay.
##############################################################################

ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

# keep downloaded .deb files so buildx cache mounts are useful
RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

##############################################################################
# Build toolchain
##############################################################################
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --fix-missing --no-install-recommends \
        build-essential \
        cmake \
        git \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep

##############################################################################
# bt_engine / bt_nav build dependencies (same set as docker/ci/Dockerfile.ci)
##############################################################################
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --fix-missing --no-install-recommends \
        libyaml-cpp-dev \
        libjsoncpp-dev \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-msgs \
        ros-${ROS_DISTRO}-nav2-util \
        ros-${ROS_DISTRO}-behaviortree-cpp \
        ros-${ROS_DISTRO}-generate-parameter-library

##############################################################################
# stretch_nav2 runtime dependencies (nav2 bringup + slam_toolbox + teleop)
##############################################################################
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --fix-missing --no-install-recommends \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-nav2-amcl \
        ros-${ROS_DISTRO}-nav2-map-server \
        ros-${ROS_DISTRO}-slam-toolbox \
        ros-${ROS_DISTRO}-joy \
        ros-${ROS_DISTRO}-teleop-twist-joy \
        ros-${ROS_DISTRO}-teleop-twist-keyboard

##############################################################################
# vln_policy: the ROS side of the VLN agent (src/semantic_nav/vln_policy).
#
# No model ever loads here — it is an HTTP client to the StreamVLN server in
# docker/vln, plus the RViz HUD renderer:
#   cv-bridge / python3-opencv -> sensor_msgs/Image <-> numpy for the HUD
#   python3-requests           -> the backend's HTTP client
#   tmux                       -> ./run_vln_robot.sh's three-pane console
#   rviz2                      -> the launch's rviz:=true (vln_demo.rviz)
#   image-transport-plugins    -> lets rviz2 (and `image_transport republish`)
#                                 read the camera's /compressed topics. Our own
#                                 nodes decode JPEG with cv2 and do not need it,
#                                 but without it rviz2 can only take the raw
#                                 stream — 2.7 MB/frame off the robot.
# nav2_msgs (the nav2 executor's action type) already comes with navigation2
# above. rviz2 needs a display: docker-compose.yaml passes DISPLAY and mounts
# the X11 socket, and the host may need `xhost +local:root` once.
##############################################################################
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --fix-missing --no-install-recommends \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-image-transport-plugins \
        python3-opencv \
        python3-requests \
        tmux

SHELL ["/bin/bash", "-c"]
CMD ["/bin/bash"]
