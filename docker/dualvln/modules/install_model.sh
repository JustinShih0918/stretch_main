#!/usr/bin/env bash
set -e

if [ -z "${DUALVLN_MODEL:-}" ]; then
  echo "Skipping DualVLN checkpoint (set DUALVLN_MODEL=YES to download)."
  exit 0
fi

huggingface-cli download \
  --revision "${CHECKPOINT_REVISION}" \
  --local-dir /opt/dualvln_ckpt \
  InternRobotics/InternVLA-N1-DualVLN
