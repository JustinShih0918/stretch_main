#!/usr/bin/env bash
# The whole panel against the fake robot, on a ROS domain of its own so it
# cannot touch a real robot or a simulator on this machine:
#
#   bash control_panel/tests/e2e_fake.sh            # run the scripted check
#   KEEP=1 bash control_panel/tests/e2e_fake.sh     # ...and leave it up: http://localhost:18080
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
# Its own domain, whatever the shell exports (a sourced robot setup often sets 0):
# the fake publishes /stretch/cmd_vel, which a real robot or a simulator obeys.
export ROS_DOMAIN_ID=${PANEL_TEST_DOMAIN:-77} ROS_LOCALHOST_ONLY=1
export PANEL_PORT=${PANEL_PORT:-18080}
export PANEL_MAPS_DIR=${PANEL_MAPS_DIR:-$(mktemp -d /tmp/panel_fake_maps.XXXX)}
export PANEL_LOG_DIR=${PANEL_LOG_DIR:-$PANEL_MAPS_DIR/logs}
LOGS=$PANEL_LOG_DIR; mkdir -p "$LOGS"
if curl -s -o /dev/null "http://127.0.0.1:$PANEL_PORT/"; then
  echo "[e2e] port $PANEL_PORT is taken (a panel left running?): set PANEL_PORT or stop it" >&2; exit 2
fi

python3 supervisor/supervisor.py --units tests/fake_units.yaml > "$LOGS/supervisor.out" 2>&1 &
SUP=$!
python3 -m gateway --port "$PANEL_PORT" > "$LOGS/gateway.out" 2>&1 &
GW=$!
cleanup() { kill -INT $SUP 2>/dev/null; wait $SUP 2>/dev/null; kill $GW 2>/dev/null; wait $GW 2>/dev/null; }
trap cleanup EXIT
for _ in $(seq 50); do curl -sf "http://127.0.0.1:$PANEL_PORT/api/health" >/dev/null && break; sleep 0.2; done

echo "[e2e] domain $ROS_DOMAIN_ID, panel http://127.0.0.1:$PANEL_PORT, maps $PANEL_MAPS_DIR"
python3 tests/e2e_fake.py
if [ -n "$KEEP" ]; then echo "[e2e] left running; Ctrl-C to stop"; wait $GW; fi
