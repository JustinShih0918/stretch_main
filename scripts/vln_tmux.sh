#!/usr/bin/env bash
# Shared tmux plumbing for the two VLN launchers:
#
#   ../run_vln_demo.sh   -> Isaac Sim   (vln_demo.launch.py)
#   ../run_vln_robot.sh  -> real robot  (vln_robot.launch.py)
#
# Sourced, never executed. The caller sets:
#
#   VLN_SESSION      tmux session name
#   VLN_LAUNCH_FILE  launch file inside the vln_policy package
#   VLN_WORKSPACE    colcon workspace root
#   LAUNCH_ARGS      array of ros2 launch passthrough arguments
#   VLN_INSTALL_CANDIDATES  (optional) install trees to try, in order
#
# then calls vln_apply_default_server_url (optional) and vln_tmux_start.

# Fill in server_url for the model-server backends unless the caller already
# passed one. Explicit launch argument wins, then $VLN_SERVER_URL, then the
# lab server. Keep localhost reachable via server_url:=http://localhost:18080.
vln_apply_default_server_url() {
  local default_url="${1:-${VLN_SERVER_URL:-http://140.114.89.63:18080}}"
  local backend=streamvln
  local has_server_url=false
  local arg
  for arg in "${LAUNCH_ARGS[@]}"; do
    case "${arg}" in
      backend:=*) backend="${arg#backend:=}" ;;
      server_url:=*) has_server_url=true ;;
    esac
  done
  # navila is left alone on purpose: it speaks the same contract but on its
  # own server/port, so it must be given server_url explicitly.
  if [ "${backend}" = streamvln ] && [ "${has_server_url}" = false ]; then
    LAUNCH_ARGS+=("server_url:=${default_url}")
  fi
}

# Left pane: the launch. Top right: latest-only /vln/status. Bottom right:
# type an instruction + Enter to publish /vln_instruction.
vln_tmux_start() {
  # Which colcon install tree to source. One workspace can hold several:
  # docker/vlm (Jetson Thor) builds into install_vlm/ precisely to stay out of
  # the root-owned install/ that ci/deploy/sim use. Take the first candidate
  # that actually contains vln_policy, so a half-built tree is skipped rather
  # than sourced into a confusing "package not found". VLN_INSTALL_BASE forces
  # a specific one; VLN_INSTALL_CANDIDATES is the caller's preference order.
  local candidates=(install install_vlm)
  if [ "${#VLN_INSTALL_CANDIDATES[@]}" -gt 0 ]; then
    candidates=("${VLN_INSTALL_CANDIDATES[@]}")
  fi
  if [ -n "${VLN_INSTALL_BASE}" ]; then
    candidates=("${VLN_INSTALL_BASE}")
  fi

  local setup_file="" candidate
  for candidate in "${candidates[@]}"; do
    case "${candidate}" in
      /*) ;;
      *) candidate="${VLN_WORKSPACE}/${candidate}" ;;
    esac
    # Both colcon layouts: merge-install (share/<pkg>) and the isolated
    # default (<pkg>/share/<pkg>), which is what docker/vlm produces.
    if [ -f "${candidate}/setup.bash" ] &&
       { [ -d "${candidate}/share/vln_policy" ] ||
         [ -d "${candidate}/vln_policy/share/vln_policy" ]; }; then
      setup_file="${candidate}/setup.bash"
      break
    fi
  done

  if [ -z "${setup_file}" ]; then
    echo "ERROR: no colcon install tree with vln_policy under ${VLN_WORKSPACE}"
    echo "       (tried: ${candidates[*]})"
    echo "Build it first:"
    echo "  colcon build --packages-up-to vln_policy"
    echo "  # docker/vlm (Thor):"
    echo "  #   docker compose -f docker/vlm/compose.yaml run --rm build"
    return 1
  fi
  echo "Sourcing ${setup_file}"
  if ! command -v tmux >/dev/null 2>&1; then
    echo "ERROR: tmux is not installed."
    return 1
  fi

  # Quote every passthrough argument for the shell tmux creates for the pane.
  local launch_args_shell="" quoted_arg arg
  for arg in "${LAUNCH_ARGS[@]}"; do
    printf -v quoted_arg '%q' "${arg}"
    launch_args_shell+=" ${quoted_arg}"
  done

  tmux kill-session -t "${VLN_SESSION}" 2>/dev/null || true

  # Keep the launch/status panes open after their ROS process exits. Besides
  # preserving the three-pane layout, this leaves startup errors visible (an
  # early nav2 launch failure used to make the whole left pane disappear).
  local launch_cmd=". '${setup_file}' && \
    ros2 launch vln_policy ${VLN_LAUNCH_FILE}${launch_args_shell}; \
    launch_rc=\$?; \
    if [ \$launch_rc -ne 0 ]; then \
      echo; echo \"VLN launch exited with status \$launch_rc\"; \
    fi; \
    exec bash"
  local status_cmd=". '${setup_file}' && \
    ros2 run vln_policy vln_status_monitor; \
    status_rc=\$?; \
    if [ \$status_rc -ne 0 ]; then \
      echo; echo \"VLN status monitor exited with status \$status_rc\"; \
    fi; \
    exec bash"
  local instruct_cmd=". '${setup_file}' && \
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

  # Explicit pane targets so tmux configuration cannot change the layout:
  # left = launcher, right-top = status, right-bottom = instruction.
  tmux new-session  -d -s "${VLN_SESSION}" -n main "${launch_cmd}"
  tmux split-window -h -t "${VLN_SESSION}:main" "${status_cmd}"
  tmux split-window -v -t "${VLN_SESSION}:main.1" "${instruct_cmd}"
  tmux select-pane  -t "${VLN_SESSION}:main.2"
  tmux attach       -t "${VLN_SESSION}"
}
