#!/usr/bin/env bash
# Launch the standalone VLN demo in a tmux session.
#
#   Left pane   : ros2 launch vln_policy vln_demo.launch.py <passthrough args>
#   Top right   : ros2 topic echo /vln/status   (live command output)
#   Bottom right: type an instruction + Enter to publish /vln_instruction
#
# Usage (inside the sim container, workspace built):
#   ./run_vln_demo.sh                                   # streamvln + cmd_vel
#   ./run_vln_demo.sh backend:=dummy                    # no model needed
#   ./run_vln_demo.sh backend:=streamvln execution_mode:=nav2
#
# Prerequisites:
#   * Isaac Sim playing isaacsim/assets/stretch3_og_hospital.usda
#     (publishes /rgb /odom /tf, subscribes cmd_vel)
#   * for backend:=streamvln — the inference server on GPU 1:
#       docker compose -f docker/vln/compose.yaml up -d
#       curl localhost:18080/health   # wait for {"status":"ok",...}

set -e

SESSION=vln_demo
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_FILE="${WORKSPACE_ROOT}/install/setup.bash"
LAUNCH_ARGS="$*"

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

LAUNCH_CMD="source '${SETUP_FILE}' && \
  ros2 launch vln_policy vln_demo.launch.py ${LAUNCH_ARGS}; exec bash"
STATUS_CMD="source '${SETUP_FILE}' && \
  echo '== /vln/status (live command output) =='; \
  ros2 topic echo /vln/status; exec bash"
INSTRUCT_CMD="source '${SETUP_FILE}' && \
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

tmux new-session  -d -s "${SESSION}" -n main "bash -c \"${LAUNCH_CMD}\""
tmux split-window -h -t "${SESSION}:main" "bash -c \"${STATUS_CMD}\""
tmux split-window -v -t "${SESSION}:main.1" "bash -c \"${INSTRUCT_CMD}\""
tmux select-pane  -t "${SESSION}:main.2"
tmux attach       -t "${SESSION}"
