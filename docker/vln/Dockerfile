# StreamVLN inference server image.
#
# Deliberately separate from docker/sim: StreamVLN pins torch 2.1.2 / cu121 /
# transformers 4.45.1, which conflicts with the Isaac Sim image (cu128,
# transformers 4.57). The ROS side only talks HTTP (contract in
# src/semantic_nav/vln_policy/DESIGN.md), so this image carries no ROS at all.
#
# x86_64 dGPU variant. For NVIDIA Jetson AGX Thor (JetPack 7, aarch64 /
# CUDA 13 / sm_110) use ./Dockerfile.jetson — this base tops out at sm_90.
#
# Build (from repo root):
#   docker compose -f docker/vln/compose.yaml build
#
# devel base (not runtime): flash-attn compiles against nvcc at build time.
FROM pytorch/pytorch:2.1.2-cuda12.1-cudnn8-devel

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        git curl ca-certificates fonts-dejavu-core libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# StreamVLN source, pinned. Serving needs the model/agent code only —
# habitat-sim/habitat-lab (benchmark eval) are NOT installed.
ARG STREAMVLN_REPO=https://github.com/InternRobotics/StreamVLN.git
ARG STREAMVLN_COMMIT=main
RUN git clone "${STREAMVLN_REPO}" /opt/streamvln \
    && cd /opt/streamvln \
    && git checkout "${STREAMVLN_COMMIT}" \
    && rm -rf .git

# Inference subset of StreamVLN's requirements.txt (torch ships with the base
# image; deepspeed/habitat/training deps omitted).
# av: upstream pins 14.4.0, but that version ships no Linux wheels on PyPI
# (source-only, needs ffmpeg 7 + pkg-config); 12.3.0 is the last manylinux
# wheel and covers the import surface LLaVA-Video uses — the server only
# feeds single JPEG frames, never decodes video files.
RUN pip install --no-cache-dir \
        "transformers==4.45.1" \
        "accelerate==0.28.0" \
        "peft==0.5.0" \
        "timm==1.0.15" \
        "einops==0.6.1" \
        "einops-exts==0.0.4" \
        "open-clip-torch==2.32.0" \
        "decord==0.6.0" \
        "av==12.3.0" \
        "numpy==1.26.1" \
        "opencv-python-headless==4.9.0.80" \
        Pillow \
        fastapi \
        "uvicorn[standard]"

# flash-attn matching torch 2.1.2/cu121. Best effort: the server falls back
# to sdpa attention (ATTN_IMPL env) if the import fails at runtime.
RUN pip install --no-cache-dir flash-attn==2.5.9.post1 --no-build-isolation \
    || echo "WARNING: flash-attn install failed; run with ATTN_IMPL=sdpa"

# Deps StreamVLN imports but does not list in requirements.txt (they leak in
# via habitat in the upstream conda env). Exact closure of the modules the
# server touches: streamvln_agent needs quaternion + depth_camera_filtering +
# omegaconf; everything else guarded by try/except upstream.
# numpy-quaternion 2023.0.4 = last numpy<2 pin with cp310 wheels.
RUN pip install --no-cache-dir \
        "numpy-quaternion==2023.0.4" \
        "omegaconf==2.3.0" \
        "git+https://github.com/naokiyokoyama/depth_camera_filtering@39d6e2f391c8b2198a67ad96f94bf6da0acd48a0"

# Checkpoint download, gated so the image can build without the ~16 GB model.
ARG STREAMVLN_MODEL=""
ARG STREAMVLN_MODEL_ID=mengwei0427/StreamVLN_Video_qwen_1_5_r2r_rxr_envdrop_scalevln_real_world
COPY modules/install_streamvln_model.sh /tmp/install_streamvln_model.sh
RUN --mount=type=cache,target=/root/.cache/huggingface \
    STREAMVLN_MODEL="${STREAMVLN_MODEL}" \
    STREAMVLN_MODEL_ID="${STREAMVLN_MODEL_ID}" \
    bash /tmp/install_streamvln_model.sh \
    && rm /tmp/install_streamvln_model.sh

COPY server/ /opt/vln_server/
WORKDIR /opt/vln_server

ENV MODEL_DIR=/opt/streamvln_ckpt \
    STREAMVLN_ROOT=/opt/streamvln \
    PORT=18080

CMD ["sh", "-c", "uvicorn app:app --host 0.0.0.0 --port ${PORT}"]
