#!/usr/bin/env bash
set -e

if [ -z "$IFACE" ]; then
    echo "Error: No active wired network interface found."
    echo "Please specify it manually, e.g.: $0 eth0"
    exit 1
fi

echo "Starting supervisor on interface: $IFACE"

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="$HOME/stretch_main/docker/cyclonedds-eth.xml"
export CYCLONEDDS_IFACE=enp114s0

python3 "$HOME/stretch_main/control_panel/supervisor/supervisor.py"