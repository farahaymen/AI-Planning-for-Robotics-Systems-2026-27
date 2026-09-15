#!/usr/bin/env bash
# Stage 50: the course workspace, built and ready.
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
stage 50-workspace || exit 0

ARC_REPO="${ARC_REPO:-https://github.com/YOUR-ORG/arc-course.git}"
WS="$ARC_HOME/arc_ws"

if [[ ! -d "$WS/src/arc-course/.git" ]]; then
  sudo -u "$ARC_USER" git clone "$ARC_REPO" "$WS/src/arc-course" \
    || die "clone failed. Set ARC_REPO to your course repository."
fi

sudo -u "$ARC_USER" bash -c "
  source /opt/ros/jazzy/setup.bash
  cd '$WS'
  rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
  colcon build --symlink-install
"

install -m 0755 "$WS/src/arc-course/scripts/course-check" /usr/local/bin/course-check
install -m 0755 "$WS/src/arc-course/scripts/course-smoke-test" /usr/local/bin/course-smoke-test

stamp_done 50-workspace
log "stage 50-workspace complete. Verify: course-check"
