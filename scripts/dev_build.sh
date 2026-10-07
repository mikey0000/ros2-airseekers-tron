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
  # Fields2Cover v3 layer (built once, ~2 min at -j2); see scripts/build_f2c.sh.
  docker image inspect mower-f2c:v3-amd64 >/dev/null 2>&1 || ARCH=amd64 "$SCRIPT_DIR/build_f2c.sh"
  base="ros@$(docker manifest inspect ros:humble-ros-base-jammy | python3 -c "
import json,sys; d=json.load(sys.stdin)
print(next(m['digest'] for m in d['manifests'] if m['platform'].get('os')=='linux' and m['platform']['architecture']=='amd64'))")"
  DOCKER_BUILDKIT=0 docker build --build-arg BASE="$base" -f docker/Dockerfile.dev-amd64 -t "$IMG" docker
fi
mode="${1:-build}"; shift || true
# Host networking + default domain 0 put dev-host test nodes on the robot's DDS graph over the
# LAN (2026-10-07: a test feeder's /odometry/filtered_map latched a lethal boundary stop).
run() { docker run --rm -i ${TTY:+-t} --network host -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID="${DEV_ROS_DOMAIN_ID:-77}" -v "$STACK_ROOT":/work -w /work "$IMG" bash -c "source /opt/ros/humble/setup.bash; $*"; }
case "$mode" in
  shell) TTY=1 run bash ;;
  build) run "colcon build --symlink-install --event-handlers console_cohesion+ $*" ;;
  test)  run "source install/setup.bash; colcon test --event-handlers console_cohesion+ $* ; colcon test-result --verbose" ;;
  *) echo "unknown mode $mode" >&2; exit 2 ;;
esac
