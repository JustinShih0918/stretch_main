#!/bin/bash
set -e

# Install LocateAnything inference dependencies and cache the model checkpoint
# for semantic_perception/locate_anything_node.py.
#
# This script is intended to be run inside the Dockerfile during build and is
# gated so the base sim image can still be built without the multi-GB model.

if [ "${LOCATE_ANYTHING,,}" != "yes" ] && [ "${LOCATE_ANYTHING,,}" != "y" ]; then
    echo "Skipping LocateAnything installation (set LOCATE_ANYTHING=YES to enable)"
    exit 0
fi

echo "Installing LocateAnything inference runtime"

# cv_bridge converts sensor_msgs/Image to the RGB numpy array consumed by PIL.
sudo apt-get update && sudo apt-get install -y \
    ros-${ROS_DISTRO:-humble}-cv-bridge \
    python3-opencv \
    && sudo rm -rf /var/lib/apt/lists/*

# CUDA 12.8 is the default sim-toolkit version. The index remains overridable
# for builders using another CUDA/PyTorch combination.
TORCH_INDEX="${TORCH_INDEX_URL:-https://download.pytorch.org/whl/cu128}"
pip3 install --index-url "${TORCH_INDEX}" torch torchvision

# Inference-only subset of Embodied/pyproject.toml. Training, evaluation, web
# UI, telemetry, datasets, and DeepSpeed dependencies are intentionally omitted.
pip3 install \
    "Pillow>=9.1" \
    "numpy>=1.25,<2" \
    "transformers==4.57.1" \
    "tokenizers==0.22.0" \
    "sentencepiece==0.2.0" \
    "accelerate==1.5.2" \
    "peft==0.12.0" \
    einops \
    einops-exts \
    "timm>=1.0.11" \
    lmdb \
    decord \
    regex \
    requests \
    shortuuid \
    safetensors \
    huggingface_hub

MODEL_ID="${LOCATE_ANYTHING_MODEL_ID:-nvidia/LocateAnything-3B}"
MODEL_DIR="${LOCATE_ANYTHING_MODEL_DIR:-/opt/locate_anything/LocateAnything-3B}"
sudo mkdir -p "${MODEL_DIR}"
sudo chown -R "$(id -u):$(id -g)" "$(dirname "${MODEL_DIR}")"

# Cache all custom model code and weights so runtime does not require network.
hf download \
    "${MODEL_ID}" \
    --local-dir "${MODEL_DIR}"

echo "LocateAnything installed:"
echo "  model: ${MODEL_ID}"
echo "  path:  ${MODEL_DIR}"
