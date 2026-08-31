#!/usr/bin/env bash
set -e

# Isaac Sim 5.1's documented startup switch for isaacsim.ros2.sim_control.
exec "${ISAACSIM_PATH}/isaac-sim.sh" \
  --/isaac/startup/ros_sim_control_extension=True "$@"
