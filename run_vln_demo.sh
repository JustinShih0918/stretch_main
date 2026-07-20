#!/usr/bin/env bash
# Launch the standalone VLN demo in a tmux session.
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

set -e

SESSION=vln_demo
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_FILE="${WORKSPACE_ROOT}/install/setup.bash"
LAUNCH_ARGS=("$@")

# This workspace normally uses the lab inference server.  An explicit ROS
# launch argument wins, followed by VLN_SERVER_URL, then the lab default.
# Keep localhost available via server_url:=http://localhost:18080.
BACKEND_NAME=streamvln
HAS_SERVER_URL=false
for arg in "${LAUNCH_ARGS[@]}"; do
  case "${arg}" in
    backend:=*) BACKEND_NAME="${arg#backend:=}" ;;
    server_url:=*) HAS_SERVER_URL=true ;;
  esac
done
if [ "${BACKEND_NAME}" = streamvln ] && [ "${HAS_SERVER_URL}" = false ]; then
  LAUNCH_ARGS+=(
    "server_url:=${VLN_SERVER_URL:-http://140.114.89.63:18080}"
  )
fi

# Quote every passthrough argument for the shell tmux creates for the pane.
LAUNCH_ARGS_SHELL=""
for arg in "${LAUNCH_ARGS[@]}"; do
  printf -v quoted_arg '%q' "${arg}"
  LAUNCH_ARGS_SHELL+=" ${quoted_arg}"
done

if [ ! -f "${SETUP_FILE}" ]; then
  echo "ERROR: ${SETUP_FILE} not found."
  echo "Run from the colcon workspace root after building, e.g.:"
  echo "  colcon build --packages-up-to vln_policy"
  exit 1
fi

if ! command -v tmux >/dev/null 2>&1; then
  echo "ERROR: tmux is not installed."
  exit 1
fi

tmux kill-session -t "${SESSION}" 2>/dev/null || true

# Keep the launch/status panes open after their ROS process exits.  Besides
# preserving the three-pane layout, this leaves startup errors visible (an
# early nav2 launch failure used to make the whole left pane disappear).
LAUNCH_CMD=". '${SETUP_FILE}' && \
  ros2 launch vln_policy vln_demo.launch.py${LAUNCH_ARGS_SHELL}; \
  launch_rc=\$?; \
  if [ \$launch_rc -ne 0 ]; then \
    echo; echo \"VLN/Nav2 launch exited with status \$launch_rc\"; \
  fi; \
  exec bash"
STATUS_CMD=". '${SETUP_FILE}' && \
  ros2 run vln_policy vln_status_monitor; \
  status_rc=\$?; \
  if [ \$status_rc -ne 0 ]; then \
    echo; echo \"VLN status monitor exited with status \$status_rc\"; \
  fi; \
  exec bash"
INSTRUCT_CMD=". '${SETUP_FILE}' && \
  echo '== VLN instruction pane =='; \
  echo 'Type a navigation instruction + <Enter> to start an episode'; \
  echo '(a new instruction cancels the running one; q + <Enter> quits)'; \
  while IFS= read -r line; do \
    case \"\$line\" in \
      q) break ;; \
      '') ;; \
      *) ros2 topic pub --once -w 1 /vln_instruction std_msgs/msg/String \
           \"{data: '\$line'}\" >/dev/null && echo \"sent: \$line\" ;; \
    esac; \
  done; \
  exec bash"

# Use explicit pane targets so tmux configuration cannot change the layout:
# left = launcher, right-top = status, right-bottom = instruction.
tmux new-session  -d -s "${SESSION}" -n main "${LAUNCH_CMD}"
tmux split-window -h -t "${SESSION}:main" "${STATUS_CMD}"
tmux split-window -v -t "${SESSION}:main.1" "${INSTRUCT_CMD}"
tmux select-pane  -t "${SESSION}:main.2"
tmux attach       -t "${SESSION}"
