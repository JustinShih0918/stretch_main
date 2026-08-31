#!/usr/bin/env bash
# Run the five-repetition StreamVLN/DualVLN benchmark after Isaac is playing.
set -e

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_FILE="${WORKSPACE_ROOT}/install/setup.bash"
if [ ! -f "${SETUP_FILE}" ]; then
  echo "ERROR: build vln_policy before running the benchmark."
  exit 1
fi

source "${SETUP_FILE}"
exec ros2 run vln_policy vln_benchmark "$@"
