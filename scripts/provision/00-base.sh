#!/usr/bin/env bash
# Stage 00: base OS, locale, tools, course directories, environment profile.
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
require_root
stage 00-base || exit 0

apt-get update
apt-get -y upgrade
apt-get -y install \
  locales software-properties-common curl wget gnupg lsb-release ca-certificates \
  git build-essential cmake python3-pip python3-venv python3-dev \
  mesa-utils x11-apps net-tools htop tree unzip zip jq \
  pandoc texlive-latex-recommended texlive-fonts-recommended lmodern

locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
add-apt-repository -y universe

# Course directories. Fixed paths, because every manual refers to them by name.
sudo -u "$ARC_USER" mkdir -p \
  "$ARC_HOME/arc_ws/src" "$ARC_HOME/arc_ws/.arc" \
  "$ARC_HOME/arc_ws/maps" "$ARC_HOME/arc_ws/bags" "$ARC_HOME/arc_ws/results"

# Environment profile. Sourced by every login shell, so a student never has to
# remember to set any of this, and course-check can verify it.
cat > /etc/profile.d/arc-course.sh <<'PROFILE'
# Autonomous Robotics Course environment. Do not edit; regenerate with provisioning.
export ARC_VM_RELEASE="2026.1"
export ARC_SEAT_ID="${ARC_SEAT_ID:-$(hostname | tr -cd '0-9' | tail -c3)}"
export ARC_SEAT_ID="${ARC_SEAT_ID:-1}"

# DDS discovery isolation. Without this, thirty machines on one lab subnet
# discover each other and students drive each other's robots.
# ROS_LOCALHOST_ONLY was deprecated; this is the current mechanism.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=$(( (ARC_SEAT_ID % 100) + 1 ))
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Protect the apt-managed NumPy that ROS Python packages are built against.
export PIP_CONSTRAINT=/etc/arc/pip-constraints.txt

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi
if [ -f "$HOME/arc_ws/install/setup.bash" ]; then
  source "$HOME/arc_ws/install/setup.bash"
fi
export PYTHONPATH="$HOME/arc_ws/src/arc-course:${PYTHONPATH:-}"
PROFILE

mkdir -p /etc/arc
# ROS 2 Jazzy on Noble is built against NumPy 1.26. Letting pip pull NumPy 2
# breaks ROS Python packages in ways that surface much later as import errors.
cat > /etc/arc/pip-constraints.txt <<'CONSTRAINTS'
numpy<2
CONSTRAINTS

stamp_done 00-base
log "stage 00-base complete"
