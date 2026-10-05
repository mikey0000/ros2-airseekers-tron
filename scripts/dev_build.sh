#!/usr/bin/env bash
# Build + test the workspace on an x86_64 dev host inside docker (no mower needed).
#   ./scripts/dev_build.sh                 # colcon build (all packages)
#   ./scripts/dev_build.sh test            # colcon test + result summary
#   ./scripts/dev_build.sh shell           # interactive shell in the image
#   ./scripts/dev_build.sh build --packages-select mower_mission
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_ROOT="$(dirname "$SCRIPT_DIR")"
IMG=mower:humble-dev-amd64
cd "$STACK_ROOT"
if ! docker image inspect "$IMG" >/dev/null 2>&1 || [ "${REBUILD_IMAGE:-0}" = 1 ]; then
  docker build -f docker/Dockerfile.dev-amd64 -t "$IMG" docker
fi
mode="${1:-build}"; shift || true
run() { docker run --rm -i ${TTY:+-t} --network host -v "$STACK_ROOT":/work -w /work "$IMG" bash -c "source /opt/ros/humble/setup.bash; $*"; }
case "$mode" in
  shell) TTY=1 run bash ;;
  build) run "colcon build --symlink-install --event-handlers console_cohesion+ $*" ;;
  test)  run "source install/setup.bash; colcon test --event-handlers console_cohesion+ $* ; colcon test-result --verbose" ;;
  *) echo "unknown mode $mode" >&2; exit 2 ;;
esac
