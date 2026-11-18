#!/usr/bin/env bash
# Move a fire extinguisher in the running Isaac scene (open_extinguisher.py applies
# it within half a second, while the scene plays; it stays there across resets).
#
#   ./scripts/extinguisher_demo/move_extinguisher.sh 1 -7.6 1.85 -80   # #1 to the waiting chairs
#   ./scripts/extinguisher_demo/move_extinguisher.sh 1 home            # #1 back to its wall
#   ./scripts/extinguisher_demo/move_extinguisher.sh 2 -8.95 5.12      # #2 along the wall, yaw kept
#
# x, y are Isaac world = ROS map metres; yaw in degrees (100 faces the lobby from the
# north wall, -80 faces it from the chairs). #2 exists only on the two-extinguisher
# stage (EXTINGUISHER_STAGE=.../stretch3_og_hospital_two_extinguishers.usda).
set -euo pipefail
LOGS=${LOGS:-$HOME/sim_nav_ws/extinguisher}
EXTINGUISHER_CONTROL=${EXTINGUISHER_CONTROL:-$LOGS/condition}
MOVE=$EXTINGUISHER_CONTROL.move
[ $# -ge 2 ] || { sed -n '2,11p' "$0" >&2; exit 1; }
case "$1" in 1|2) ;; *) echo "which extinguisher: 1 or 2, got '$1'" >&2; exit 1 ;; esac
before=$(cat "$MOVE.ack" 2>/dev/null || true)
echo "$*" > "$MOVE"
for _ in $(seq 1 20); do
  now=$(cat "$MOVE.ack" 2>/dev/null || true)
  [ -n "$now" ] && [ "$now" != "$before" ] && { echo "moved: $*"; exit 0; }
  sleep 0.25
done
echo "Isaac did not acknowledge the move within 5 s (is open_extinguisher.py running with" \
     "EXTINGUISHER_CONTROL=$EXTINGUISHER_CONTROL?)" >&2
exit 1
