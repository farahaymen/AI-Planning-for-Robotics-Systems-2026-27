#!/usr/bin/env bash
# Stage 10: ROS 2 Jazzy from the official apt source.
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
require_root
stage 10-ros2 || exit 0

# The ros2-apt-source package is the CURRENT official method. Older guides add a
# keyring and a sources.list entry by hand; that still works but is no longer
# what the documentation describes and does not self-update.
ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' | awk -F\" '{print $4}')"
[[ -n "$ROS_APT_SOURCE_VERSION" ]] || die "could not resolve ros-apt-source version (network?)"

CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")"
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
dpkg -i /tmp/ros2-apt-source.deb
apt-get update

apt-get -y install ros-jazzy-desktop ros-dev-tools \
  python3-colcon-common-extensions python3-rosdep python3-vcstool

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi
sudo -u "$ARC_USER" rosdep update

stamp_done 10-ros2
log "stage 10-ros2 complete. Verify: ros2 --help"
