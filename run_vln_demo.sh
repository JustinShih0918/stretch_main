#!/usr/bin/env bash
# Launch the VLN demo **in Isaac Sim** in a tmux session.
# For the real robot use ./run_vln_robot.sh instead.
#
#   Left pane   : ros2 launch vln_policy vln_demo.launch.py <passthrough args>
#   Top right   : compact latest-only /vln/status monitor
#   Bottom right: type an instruction + Enter to publish /vln_instruction
#
# Usage (inside the sim container, workspace built):
#   ./run_vln_demo.sh                                   # lab StreamVLN + cmd_vel
#   ./run_vln_demo.sh backend:=dummy                    # no model needed
#   ./run_vln_demo.sh backend:=streamvln execution_mode:=nav2
#   VLN_SERVER_URL=http://other-host:18080 ./run_vln_demo.sh
#
# Prerequisites:
#   * Isaac Sim playing isaacsim/assets/stretch3_og_hospital.usda
#     (publishes /rgb /odom /tf, subscribes cmd_vel)
#   * for backend:=streamvln — the inference server (docker/vln), either on
#     this machine or remote:
#       docker compose -f docker/vln/compose.yaml up -d     # on the GPU machine
#       curl http://<server-ip>:18080/health                # wait for "ok"
#     remote server: ./run_vln_demo.sh server_url:=http://<server-ip>:18080
#
# execution_mode:=nav2 brings up nav2 itself (stretch3_navigation params) —
# unlike the robot script, where nav2 is already running.

set -e

VLN_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VLN_SESSION=vln_demo
VLN_LAUNCH_FILE=vln_demo.launch.py
# The sim container builds into install/ (and only that tree has
# stretch3_navigation, needed by execution_mode:=nav2).
VLN_INSTALL_CANDIDATES=(install)
LAUNCH_ARGS=("$@")

# shellcheck source=scripts/vln_tmux.sh
. "${VLN_WORKSPACE}/scripts/vln_tmux.sh"

vln_apply_default_server_url
vln_tmux_start
