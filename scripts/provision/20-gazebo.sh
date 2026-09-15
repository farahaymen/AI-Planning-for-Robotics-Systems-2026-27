#!/usr/bin/env bash
# Stage 20: Gazebo Harmonic and the ROS bridge.
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
require_root
stage 20-gazebo || exit 0

# Harmonic is the version officially paired with Jazzy, so it comes from the ROS
# repository rather than from packages.osrfoundation.org. Mixing the two sources
# is a common way to end up with two incompatible Gazebo installs.
apt-get -y install \
  ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image ros-jazzy-ros-gz-interfaces \
  ros-jazzy-gz-ros2-control

# Graphics fallback, applied per session by course-check rather than globally,
# so a machine with working acceleration is not forced into software rendering.
cat > /etc/arc/graphics-tier-b.sh <<'TIERB'
# Source this on a machine where hardware OpenGL is unreliable.
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export QT_X11_NO_MITSHM=1
export ARC_GRAPHICS_TIER=B
echo "[arc] Tier B: software rendering. Launch Gazebo headless with headless:=true."
TIERB

stamp_done 20-gazebo
log "stage 20-gazebo complete. Verify: gz sim --versions"
