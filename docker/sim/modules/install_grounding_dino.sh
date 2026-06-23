#!/bin/bash
set -e

# Install Grounding DINO (open-set VLM, arXiv:2303.05499) for the action-aware
# semantic perception node (semantic_perception/grounding_dino_node.py).
# Gated behind the GROUNDING_DINO build arg so the base sim image stays lean.
#
# This script is intended to be run inside the Dockerfile during build.

if [ "${GROUNDING_DINO,,}" != "yes" ] && [ "${GROUNDING_DINO,,}" != "y" ]; then
    echo "Skipping Grounding DINO installation (set GROUNDING_DINO=YES to enable)"
    exit 0
fi

echo "Installing Grounding DINO + deps for ROS distro: ${ROS_DISTRO:-humble}"

# cv_bridge bridges sensor_msgs/Image <-> numpy for the perception node.
sudo apt-get update && sudo apt-get install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    python3-opencv \
    && sudo rm -rf /var/lib/apt/lists/*

# Torch: install a CUDA build. CUDA_TOOLKIT_VERSION is set earlier in the image;
# pick the matching wheel index, defaulting to cu121.
TORCH_INDEX="${TORCH_INDEX_URL:-https://download.pytorch.org/whl/cu121}"
pip3 install --index-url "${TORCH_INDEX}" torch torchvision

# Grounding DINO python package + runtime deps.
# Pin transformers<5 (groundingdino uses BERT APIs removed in 5.x),
# numpy<2 (system matplotlib was compiled against numpy 1.x ABI),
# and Pillow>=9.1 (transformers 4.x needs PIL.Image.Resampling).
pip3 install "Pillow>=9.1" "numpy<2" "transformers>=4.24,<5" \
    groundingdino-py addict yapf timm

# Fetch the SwinT-OGC checkpoint + config to a known location.
WEIGHTS_DIR="${GROUNDING_DINO_DIR:-/opt/grounding_dino}"
sudo mkdir -p "${WEIGHTS_DIR}"
sudo chown "$(id -u):$(id -g)" "${WEIGHTS_DIR}"
wget -q -O "${WEIGHTS_DIR}/groundingdino_swint_ogc.pth" \
    https://huggingface.co/ShilongLiu/GroundingDINO/resolve/main/groundingdino_swint_ogc.pth
wget -q -O "${WEIGHTS_DIR}/GroundingDINO_SwinT_OGC.py" \
    https://raw.githubusercontent.com/IDEA-Research/GroundingDINO/main/groundingdino/config/GroundingDINO_SwinT_OGC.py

echo "Grounding DINO installed:"
echo "  model_config:  ${WEIGHTS_DIR}/GroundingDINO_SwinT_OGC.py"
echo "  model_weights: ${WEIGHTS_DIR}/groundingdino_swint_ogc.pth"
echo "Pass these to grounding_dino_node via the launch model_config/model_weights args."
