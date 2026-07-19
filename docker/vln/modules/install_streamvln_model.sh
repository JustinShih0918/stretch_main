#!/bin/bash
set -e

# Download the StreamVLN real-world checkpoint for the docker/vln server.
#
# Runs inside the Dockerfile during build and is gated so the base image can
# still be built without the multi-GB model (same pattern as
# docker/sim/modules/install_locate_anything.sh).

if [ "${STREAMVLN_MODEL,,}" != "yes" ] && [ "${STREAMVLN_MODEL,,}" != "y" ]; then
    echo "Skipping StreamVLN checkpoint download (set STREAMVLN_MODEL=YES to enable)"
    exit 0
fi

MODEL_ID="${STREAMVLN_MODEL_ID:-mengwei0427/StreamVLN_Video_qwen_1_5_r2r_rxr_envdrop_scalevln_real_world}"
MODEL_DIR="${STREAMVLN_MODEL_DIR:-/opt/streamvln_ckpt}"

pip install --no-cache-dir "huggingface_hub[cli]"
mkdir -p "${MODEL_DIR}"

# Cache all weights so runtime does not require network.
hf download \
    "${MODEL_ID}" \
    --local-dir "${MODEL_DIR}"

echo "StreamVLN checkpoint installed:"
echo "  model: ${MODEL_ID}"
echo "  path:  ${MODEL_DIR}"
