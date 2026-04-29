#!/usr/bin/env bash
# Launch the BT engine in a tmux session.
# Left  pane: ros2 run bt_engine bt_engine
# Right pane: type 's' + Enter to publish std_msgs/Empty on /start

set -e

SESSION=bt_demo
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_FILE="${WORKSPACE_ROOT}/install/setup.bash"

if [ ! -f "${SETUP_FILE}" ]; then
  echo "ERROR: ${SETUP_FILE} not found."
  echo "Run from the colcon workspace root after building, e.g.:"
  echo "  colcon build --packages-select btcpp_ros2_interfaces bt_nav bt_engine"
  exit 1
fi

if ! command -v tmux >/dev/null 2>&1; then
  echo "ERROR: tmux is not installed."
  exit 1
fi

tmux kill-session -t "${SESSION}" 2>/dev/null || true

ENGINE_CMD="source '${SETUP_FILE}' && ros2 run bt_engine bt_engine; exec bash"
TRIGGER_CMD="source '${SETUP_FILE}' && \
  echo '== BT engine trigger pane =='; \
  echo 'Type s + <Enter> to publish std_msgs/Empty on /start'; \
  echo 'Type q + <Enter> to quit this pane'; \
  while IFS= read -r line; do \
    case \"\$line\" in \
      s) ros2 topic pub --once /start std_msgs/msg/Empty '{}' ;; \
      q) break ;; \
      *) echo \"unknown input: \$line (use s or q)\" ;; \
    esac; \
  done; \
  exec bash"

tmux new-session  -d -s "${SESSION}" -n main "bash -c \"${ENGINE_CMD}\""
tmux split-window -h -t "${SESSION}:main" "bash -c \"${TRIGGER_CMD}\""
tmux select-pane  -t "${SESSION}:main.1"
tmux attach       -t "${SESSION}"
