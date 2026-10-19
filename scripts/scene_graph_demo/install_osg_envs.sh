#!/usr/bin/env bash
# Install Object_scene_graph's two conda envs (habitat + ascent) natively inside the
# stretch_main sim container -- a reproduction of
# upstream Object_scene_graph's docker/Dockerfile (not vendored).
# One-off. Also links /workspace -> the vendored OSG (its configs use /workspace).
#
#   ./scripts/scene_graph_demo/install_osg_envs.sh
set -eo pipefail
REPO=${OSG_REPO:-$(cd "$(dirname "$0")/../.." && pwd)/src/semantic_nav/object_scene_graph}
sudo ln -sfn $REPO /workspace
if [ ! -x /opt/conda/bin/conda ]; then
  sudo mkdir -p /opt/conda && sudo chown $(id -u):$(id -g) /opt/conda
  curl -L https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh -o /tmp/miniforge.sh
  bash /tmp/miniforge.sh -b -u -p /opt/conda && rm /tmp/miniforge.sh
fi
export PATH=/opt/conda/bin:$PATH
unset PYTHONPATH LD_LIBRARY_PATH_EXTRA
echo "=== habitat env"
if [ ! -x /opt/conda/envs/habitat/bin/python ]; then
  conda create -y -n habitat python=3.9 cmake=3.14.0
  conda install -y -n habitat habitat-sim=0.3.1 withbullet headless -c conda-forge -c aihabitat
fi
H=/opt/conda/envs/habitat/bin
[ -d /opt/habitat-lab ] || { sudo mkdir -p /opt/habitat-lab && sudo chown $(id -u):$(id -g) /opt/habitat-lab && git clone --depth 1 --branch v0.3.1 https://github.com/facebookresearch/habitat-lab.git /opt/habitat-lab; }
$H/pip install -e /opt/habitat-lab/habitat-lab
$H/pip install torch==2.4.1 torchvision==0.19.1 --index-url https://download.pytorch.org/whl/cu121
$H/pip install "numpy<2" "ultralytics==8.3.*" openai "hydra-core>=1.3" omegaconf \
   scipy scikit-image scikit-learn "opencv-python==4.10.0.84" \
   pytest pytest-timeout imageio matplotlib pillow pyyaml tqdm anthropic "rerun-sdk"
$H/pip install git+https://github.com/ultralytics/CLIP.git
echo "=== ascent env"
if [ ! -x /opt/conda/envs/ascent/bin/python ]; then
  conda create -y -n ascent python=3.9 cmake=3.14.0
  conda install -y -n ascent habitat-sim=0.3.1 withbullet headless -c conda-forge -c aihabitat
fi
A=/opt/conda/envs/ascent/bin
printf 'numpy==1.23.5\n' > /tmp/pip-constraints.txt
export PIP_CONSTRAINT=/tmp/pip-constraints.txt
$A/pip install --no-deps -r $REPO/requirements/ascent-requirements.txt --extra-index-url https://download.pytorch.org/whl/cu118
cd $REPO/relative_work/ascent
for pkg in third_party/habitat-lab/habitat-lab third_party/habitat-lab/habitat-baselines \
           third_party/MobileSAM third_party/vlfm third_party/frontier_exploration \
           third_party/depth_camera_filtering; do
  $A/pip install --no-deps -e "$pkg"
done
echo "=== nvcc 11.8 inside ascent (for GroundingDINO; system CUDA is 12.8)"
conda install -y -n ascent -c "nvidia/label/cuda-11.8.0" cuda-nvcc cuda-cudart-dev cuda-libraries-dev
export CUDA_HOME=/opt/conda/envs/ascent TORCH_CUDA_ARCH_LIST="8.6;8.9"
cd third_party/GroundingDINO && $A/python setup.py build_ext --inplace && cd ../..
$A/python -c "import site, pathlib; pathlib.Path(site.getsitepackages()[0], '_ascent_gdino.pth').write_text('$REPO/relative_work/ascent/third_party/GroundingDINO\n')"
$A/python -c "import site, pathlib; pathlib.Path(site.getsitepackages()[0], '_ascent_ram.pth').write_text('$REPO/relative_work/ascent/third_party/recognize-anything\n')"
$A/pip uninstall -y opencv-python opencv-contrib-python 2>/dev/null || true
unset PIP_CONSTRAINT
cd / && $A/python -c "import torch, habitat_sim, transformers, numpy, cv2; import lavis.models, mobile_sam, groundingdino.util.inference, ram; from groundingdino import _C; print('ASCENT ENV OK', torch.__version__, numpy.__version__, torch.cuda.is_available())"
$H/python -c "import torch, habitat_sim, ultralytics; print('HABITAT ENV OK', torch.__version__, torch.cuda.is_available())"
echo "=== ALL ENVS DONE"
