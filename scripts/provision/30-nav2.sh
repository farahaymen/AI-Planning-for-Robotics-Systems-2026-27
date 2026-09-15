#!/usr/bin/env bash
# Stage 30: navigation, mapping, localisation and control stack.
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
require_root
stage 30-nav2 || exit 0

apt-get -y install \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-mppi-controller ros-jazzy-nav2-collision-monitor \
  ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager ros-jazzy-diff-drive-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-tf2-tools ros-jazzy-tf2-ros \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-rqt ros-jazzy-rqt-common-plugins \
  ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-rviz2

stamp_done 30-nav2
log "stage 30-nav2 complete. Verify: ros2 pkg prefix nav2_bringup"
