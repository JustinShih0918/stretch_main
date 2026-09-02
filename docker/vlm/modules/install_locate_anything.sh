#!/bin/bash
set -e

# Install the LocateAnything inference runtime for
# semantic_perception/locate_anything_node.py on NVIDIA Jetson AGX Thor
# (JetPack 7 / L4T r39, aarch64, CUDA 13, compute capability sm_110).
#
# Thor-adapted copy of docker/sim/modules/install_locate_anything.sh. Deltas
# from the sim version, all forced by the aarch64 / sm_110 target:
#
#   torch    -- installed from the cu130 index, NOT cu128. The default PyPI
#               aarch64 wheels are CUDA 12.x builds that top out at sm_90:
#               they print "GPU not supported" on Thor and only limp along by
#               JIT-ing compute_90 PTX. The cu130 aarch64 (SBSA) wheels carry
#               real sm_110 kernels. JetPack 7 uses the regular SBSA CUDA
#               stack, so no Jetson-specific wheel index is needed.
#   decord   -- built from source, because no aarch64 wheel exists for either
#               `decord` or `eva-decord` (x86_64 manylinux + macOS only). It
#               cannot simply be dropped: LocateAnything's
#               processing_locateanything.py has a top-level `import decord`,
#               and transformers' trust_remote_code path runs check_imports()
#               over that file, so AutoProcessor.from_pretrained() fails with
#               "requires the following packages ... decord" before any frame
#               is ever decoded. See the build block below.
#   flash-attn -- not installed: no sm_110 aarch64 wheel exists, and a source
#               build needs hours plus >32 GB of build RAM. The worker runs on
#               torch's sdpa path instead.
#   numpy    -- still pinned <2: cv_bridge and python3-opencv on Ubuntu 22.04
#               (ROS Humble) are compiled against the numpy 1.x ABI.
#
# The ~7 GB checkpoint is downloaded here, at image-build time, so a fresh
# clone needs no manual `hf download` step before the container can run. Set
# LOCATE_ANYTHING_MODEL=NO to skip it (and then supply the weights yourself by
# mounting them over the model dir).

if [ "${LOCATE_ANYTHING,,}" != "yes" ] && [ "${LOCATE_ANYTHING,,}" != "y" ]; then
    echo "Skipping LocateAnything installation (set LOCATE_ANYTHING=YES to enable)"
    exit 0
fi

echo "Installing LocateAnything inference runtime (aarch64 / CUDA 13 / sm_110)"

# pip runs as the unprivileged user, so console scripts (notably `hf`) land in
# ~/.local/bin, which is not on the default PATH of a docker RUN step.
export PATH="${HOME}/.local/bin:${PATH}"

# cv_bridge converts sensor_msgs/Image to the RGB numpy array consumed by PIL.
sudo apt-get update && sudo apt-get install -y \
    ros-${ROS_DISTRO:-humble}-cv-bridge \
    python3-opencv \
    && sudo rm -rf /var/lib/apt/lists/*

# CUDA 13 wheels: the only ones with sm_110 kernels for Thor. Overridable for
# builders on another Jetson generation (Orin is sm_87 -> cu126/cu128).
TORCH_INDEX="${TORCH_INDEX_URL:-https://download.pytorch.org/whl/cu130}"
pip3 install --index-url "${TORCH_INDEX}" torch torchvision

# Inference-only subset of Embodied/pyproject.toml. Training, evaluation, web
# UI, telemetry, datasets, and DeepSpeed dependencies are intentionally
# omitted (decord is handled separately below — it has no aarch64 wheel).
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
    regex \
    requests \
    shortuuid \
    safetensors \
    huggingface_hub

# ROS 2 Humble's ament_cmake_python builds every ament_python / rosidl package
# by shelling out to `setup.py egg_info` against the *system* setuptools
# (59.6.0). The pip installs above drag a modern setuptools (78.x) into
# ~/.local, which shadows it and then resolves `packaging` to jammy's system
# 21.3, giving:
#     TypeError: canonicalize_version() got an unexpected keyword argument
#                'strip_trailing_zero'
# ...on btcpp_ros2_interfaces, i.e. the whole colcon build fails. The same
# breakage hits any *later* `pip install` of a legacy setup.py project — decord
# below is exactly that — so this has to run BEFORE the decord build, not at the
# end of the script. Drop the user-site copy (nothing in the inference stack
# needs it at runtime) and assert the shadowing is really gone, so a future
# dependency bump fails here, loudly, instead of at `colcon build` time.
pip3 uninstall -y setuptools >/dev/null 2>&1 || true
python3 -c "import setuptools, sys; \
assert '/.local/' not in setuptools.__file__, \
    'user-site setuptools still shadows the system one: ' + setuptools.__file__; \
print('setuptools', setuptools.__version__, setuptools.__file__)"

# decord, from source. Neither `decord` nor the `eva-decord` fork publishes an
# aarch64 Linux wheel (x86_64 manylinux + macOS only), and it is not optional:
# LocateAnything's processing_locateanything.py imports it at module level, and
# transformers' trust_remote_code loader runs check_imports() over that file, so
# AutoProcessor.from_pretrained() raises
#     ImportError: This modeling file requires the following packages that were
#                  not found in your environment: decord
# even though this node only ever passes single PIL frames and never touches
# the video-decode path.
#
# decord 0.6.0 wants ffmpeg 4.x — which is exactly what jammy ships (4.4), so
# this builds cleanly here. It would NOT on a newer Ubuntu; that is one more
# reason this image stays on 22.04 (see the base-image note in the Dockerfile).
DECORD_VERSION="${DECORD_VERSION:-v0.6.0}"
echo "Building decord ${DECORD_VERSION} from source (no aarch64 wheel exists)"
sudo apt-get update && sudo apt-get install -y \
    build-essential \
    cmake \
    libavcodec-dev \
    libavdevice-dev \
    libavfilter-dev \
    libavformat-dev \
    libavutil-dev \
    python3-dev \
    && sudo rm -rf /var/lib/apt/lists/*

git clone --quiet --recursive --depth 1 --branch "${DECORD_VERSION}" \
    https://github.com/dmlc/decord /tmp/decord
mkdir -p /tmp/decord/build
cd /tmp/decord/build
# USE_CUDA=0: NVDEC hardware decode is pointless here (no video path) and its
# build wants the full CUDA toolkit, which this image deliberately lacks.
cmake .. -DUSE_CUDA=0 -DCMAKE_BUILD_TYPE=Release
make -j"$(nproc)"
# Install the shared library where decord's find_lib_path() looks, so the
# python package keeps working after /tmp/decord is deleted.
sudo cp libdecord.so /usr/local/lib/
sudo ldconfig
cd /tmp/decord/python
pip3 install .
cd /
rm -rf /tmp/decord
python3 -c "import decord; print('decord', decord.__version__)"


MODEL_DIR="${LOCATE_ANYTHING_MODEL_DIR:-/opt/locate_anything/LocateAnything-3B}"
sudo mkdir -p "${MODEL_DIR}"
sudo chown -R "$(id -u):$(id -g)" "$(dirname "${MODEL_DIR}")"

# The checkpoint is downloaded by default: the container is meant to be usable
# straight after `docker compose build`, with no manual `hf download` and no
# host directory to prepare. Opt out with LOCATE_ANYTHING_MODEL=NO if you would
# rather keep the image ~7 GB smaller and mount the weights over
# ${MODEL_DIR%/*} yourself.
case "${LOCATE_ANYTHING_MODEL,,}" in
    no|n|false|0|skip)
        echo "Skipping LocateAnything checkpoint download"
        echo "  (LOCATE_ANYTHING_MODEL=${LOCATE_ANYTHING_MODEL}; mount the"
        echo "   weights over ${MODEL_DIR%/*} at runtime instead)"
        exit 0
        ;;
esac

MODEL_ID="${LOCATE_ANYTHING_MODEL_ID:-nvidia/LocateAnything-3B}"

echo "Downloading ${MODEL_ID} (~7 GB) to ${MODEL_DIR}"

# `hf` is the current CLI name; older huggingface_hub only ships
# `huggingface-cli`. Accept either so a pin bump does not break the build.
if command -v hf >/dev/null 2>&1; then
    HF_CLI=hf
elif command -v huggingface-cli >/dev/null 2>&1; then
    HF_CLI=huggingface-cli
else
    echo "ERROR: neither 'hf' nor 'huggingface-cli' found on PATH" >&2
    exit 1
fi

# Retry: this is a multi-GB pull over whatever network the build host has, and
# a single dropped connection should not cost the whole image build. Downloads
# resume, so a retry only fetches what is missing.
for attempt in 1 2 3; do
    if "${HF_CLI}" download "${MODEL_ID}" --local-dir "${MODEL_DIR}"; then
        break
    fi
    if [ "${attempt}" = "3" ]; then
        echo "ERROR: checkpoint download failed after 3 attempts" >&2
        exit 1
    fi
    echo "Download attempt ${attempt} failed; retrying in 10s..."
    sleep 10
done

# Fail the build here rather than at runtime if the download silently produced
# an incomplete tree (the node would otherwise just log "model unavailable"
# on every frame). Loading the config + processor also exercises the
# trust_remote_code path, which is what discovers missing python deps such as
# decord — better here than on the robot.
python3 - "${MODEL_DIR}" <<'PY'
import pathlib, sys
d = pathlib.Path(sys.argv[1])
missing = [f for f in ("config.json", "preprocessor_config.json") if not (d / f).is_file()]
weights = list(d.glob("*.safetensors")) + list(d.glob("*.bin"))
if missing or not weights:
    sys.exit(f"ERROR: incomplete checkpoint in {d}: missing={missing} weights={len(weights)}")
print(f"checkpoint OK: {len(weights)} weight shard(s)")

from transformers import AutoConfig, AutoProcessor
cfg = AutoConfig.from_pretrained(str(d), trust_remote_code=True)
proc = AutoProcessor.from_pretrained(str(d), trust_remote_code=True)
print(f"loads OK: {type(cfg).__name__} / {type(proc).__name__}")
PY

echo "LocateAnything installed:"
echo "  model: ${MODEL_ID}"
echo "  path:  ${MODEL_DIR}"
