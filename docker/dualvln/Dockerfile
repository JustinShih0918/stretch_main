# ROS-free DualVLN inference image. InternNav and the checkpoint are pinned.
FROM pytorch/pytorch:2.5.1-cuda12.4-cudnn9-devel

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        git curl ca-certificates libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

ARG INTERNNAV_REPO=https://github.com/InternRobotics/InternNav.git
ARG INTERNNAV_COMMIT=7a5c62400ac45b313d9b709c740b64191556a242
RUN git clone "${INTERNNAV_REPO}" /opt/internnav \
    && cd /opt/internnav \
    && git checkout "${INTERNNAV_COMMIT}" \
    && pip install --no-cache-dir -e ".[internvla_n1]" \
    && rm -rf .git

ARG CHECKPOINT_REVISION=a698a9e898b4001621a319e1bc89f02ec715cc86
ARG DUALVLN_MODEL=""
COPY modules/install_model.sh /tmp/install_model.sh
RUN --mount=type=cache,target=/root/.cache/huggingface \
    DUALVLN_MODEL="${DUALVLN_MODEL}" \
    CHECKPOINT_REVISION="${CHECKPOINT_REVISION}" \
    bash /tmp/install_model.sh \
    && rm /tmp/install_model.sh

COPY server/ /opt/dualvln_server/
WORKDIR /opt/dualvln_server
ENV MODEL_DIR=/opt/dualvln_ckpt \
    CHECKPOINT_REVISION=${CHECKPOINT_REVISION} \
    DEVICE=cuda:0 \
    PORT=18082 \
    PYTHONPATH=/opt/internnav
CMD ["sh", "-c", "uvicorn app:app --host 0.0.0.0 --port ${PORT}"]
