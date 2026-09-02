#!/usr/bin/env bash
# Launch the VLN pipeline against the **real Stretch robot**, in a tmux
# session. For the Isaac Sim demo use ./run_vln_demo.sh instead.
#
# Where this runs: docker/deploy's `vln-console` service — the command that
# service runs is exactly this script:
#   docker compose -f docker/deploy/docker-compose.yaml run --rm vln-console
# The camera and nav2 come from elsewhere (the robot's hello-robot stack) and
# are reached over DDS, so every participant must share a subnet, a
# ROS_DOMAIN_ID and an RMW implementation. The StreamVLN server (docker/vln)
# is reached over HTTP — VLN_SERVER_URL, default localhost:18080.
#
#   Left pane   : ros2 launch vln_policy vln_robot.launch.py <passthrough args>
#   Top right   : compact latest-only /vln/status monitor
#   Bottom right: type an instruction + Enter to publish /vln_instruction
#
# Usage — arguments after the service name replace the default command:
#   C="docker compose -f docker/deploy/docker-compose.yaml"
#   $C run --rm vln-console
#   $C run --rm vln-console ./run_vln_robot.sh execution_mode:=nav2
#   $C run --rm vln-console ./run_vln_robot.sh backend:=dummy \
#         dummy_actions:=FORWARD,TURN_LEFT,STOP
#   VLN_SERVER_URL=http://<host>:18080 $C run --rm vln-console
# or, from a shell already inside that container / a native ROS workspace:
#   ./run_vln_robot.sh [execution_mode:=nav2] [backend:=dummy] ...
#
# Prerequisites — this script starts ONLY the VLN nodes:
#   * on the robot: its camera driver, and for execution_mode:=nav2 its nav2
#     (nothing here brings either up; a second nav2 would fight the first one
#     over cmd_vel)
#   * the inference server, for backend:=streamvln (docker/vln, on this host
#     or another) —
#       docker compose -f docker/vln/compose.jetson.yaml up -d
#       curl localhost:18080/health                     # wait for "ok"
#   * the workspace built:
#       docker compose -f docker/deploy/docker-compose.yaml run --rm build
#   * check the wiring reaches the robot before launching:
#       ros2 topic hz <the robot's image topic>
#       ros2 action list | grep navigate
#
# The robot's topic/action/frame names are the ROBOT_* block at the top of
# src/semantic_nav/vln_policy/launch/vln_robot.launch.py; per-run overrides
# can also be passed here, e.g. ./run_vln_robot.sh rgb_topic:=/camera/...
#
# SAFETY: the robot moves as soon as an instruction is sent. Keep the runstop
# within reach and start in an open area — the forward camera cannot see
# behind the robot, which matters for reverse commands.

set -e

VLN_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VLN_SESSION=vln_robot
VLN_LAUNCH_FILE=vln_robot.launch.py
# docker/deploy builds into install/; docker/vlm (Thor perception) uses
# install_vlm/ in the same bind-mounted workspace. The deploy compose pins
# VLN_INSTALL_BASE=/ws/install, which overrides this order.
VLN_INSTALL_CANDIDATES=(install install_vlm)
LAUNCH_ARGS=("$@")

# shellcheck source=scripts/vln_tmux.sh
. "${VLN_WORKSPACE}/scripts/vln_tmux.sh"

# The model server normally runs on this same Thor; VLN_SERVER_URL or an
# explicit server_url:= argument points somewhere else.
vln_apply_default_server_url "${VLN_SERVER_URL:-http://localhost:18080}"
vln_tmux_start
