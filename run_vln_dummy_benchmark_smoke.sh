#!/usr/bin/env bash
# One reset/teleport/measurement smoke trial without a model server.
set -e

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_FILE="${WORKSPACE_ROOT}/install/setup.bash"
if [ ! -f "${SETUP_FILE}" ]; then
  echo "ERROR: build vln_policy before running this smoke test."
  exit 1
fi
source "${SETUP_FILE}"

OUTPUT_FILE="${1:-${WORKSPACE_ROOT}/dummy_benchmark_smoke.json}"
MANIFEST="$(ros2 pkg prefix vln_policy)/share/vln_policy/config/benchmark_manifest_v1.yaml"

ros2 launch vln_policy vln_demo.launch.py \
  backend:=dummy execution_mode:=trajectory \
  dummy_actions:=FORWARD,FORWARD,FORWARD,FORWARD,FORWARD,FORWARD,FORWARD,FORWARD,STOP \
  viz:=false rviz:=false use_sim_time:=True &
LAUNCH_PID=$!

cleanup() {
  kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

for _ in $(seq 1 60); do
  if ros2 node list 2>/dev/null | grep -qx /vln_agent_node; then
    break
  fi
  sleep 0.5
done
if ! ros2 node list 2>/dev/null | grep -qx /vln_agent_node; then
  echo "ERROR: /vln_agent_node did not start."
  exit 1
fi

ros2 run vln_policy vln_benchmark_trial --ros-args \
  -p "manifest:=${MANIFEST}" \
  -p route_id:=hospital_short_hallway_01 \
  -p backend:=dummy \
  -p repetition:=1 \
  -p "output_file:=${OUTPUT_FILE}" \
  -p use_sim_time:=True

echo "Dummy benchmark smoke result: ${OUTPUT_FILE}"
