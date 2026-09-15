#!/usr/bin/env bash
# Run the provisioning stages in order.
#
#   sudo ./provision.sh            all stages up to 50
#   sudo ./provision.sh 30         one stage
#
# Stage 60 is deliberately NOT included in the default run: it destroys caches
# and machine identity and must only run after validation has passed.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

run() { echo; echo "=== $1 ==="; bash "$HERE/$1"; }

if [[ $# -gt 0 ]]; then
  run "$(ls "$HERE" | grep "^$1-")"
  exit 0
fi

for s in 00-base.sh 10-ros2.sh 20-gazebo.sh 30-nav2.sh 40-ai-stack.sh 50-workspace.sh; do
  run "$s"
done

echo
echo "Provisioning complete. Now, in a NEW login shell:"
echo "  course-check          verifies everything is present"
echo "  course-smoke-test     verifies the robot actually works"
echo "Only after both pass, run: sudo ./60-finalise.sh"
