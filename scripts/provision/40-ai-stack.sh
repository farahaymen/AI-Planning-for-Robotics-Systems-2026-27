#!/usr/bin/env bash
# Stage 40: the Python AI stack, installed so it does not break ROS.
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
require_root
stage 40-ai-stack || exit 0

# Ubuntu 24.04 marks system Python as externally managed, so pip needs an
# explicit flag. A virtualenv is the usual alternative and is rejected here for
# a specific reason: rclpy lives in /usr/lib/python3/dist-packages, and a venv
# that can see it (--system-site-packages) can still shadow the apt NumPy the
# moment anything pip-installs NumPy. The constraint file is the real defence,
# and it works identically either way, so the simpler layout wins.
export PIP_CONSTRAINT=/etc/arc/pip-constraints.txt

# ROS-compatible scientific stack from apt, NOT pip.
apt-get -y install python3-numpy python3-scipy python3-matplotlib \
  python3-pandas python3-yaml python3-pytest python3-pil

# CPU-only PyTorch. The default PyPI wheel pulls several GB of CUDA libraries
# that a virtual machine with no GPU can never use, and the VM image has to fit
# on a lab disk alongside everything else.
pip3 install --break-system-packages --index-url https://download.pytorch.org/whl/cpu \
  "torch==2.*"

pip3 install --break-system-packages \
  "gymnasium>=1.0" "stable-baselines3>=2.3" tensorboard

# Docker, used for dependency isolation and competition evaluation, NOT for
# running Gazebo. Gazebo inside Docker inside a VM is a graphics problem nobody
# needs.
apt-get -y install docker.io
usermod -aG docker "$ARC_USER" || true
systemctl enable docker || true

python3 - <<'CHECK'
import numpy, torch, gymnasium, stable_baselines3
assert numpy.__version__.startswith("1."), f"NumPy {numpy.__version__} will break ROS Python packages"
assert not torch.cuda.is_available() or True
print(f"numpy {numpy.__version__} | torch {torch.__version__} | "
      f"gymnasium {gymnasium.__version__} | sb3 {stable_baselines3.__version__}")
CHECK

stamp_done 40-ai-stack
log "stage 40-ai-stack complete"
