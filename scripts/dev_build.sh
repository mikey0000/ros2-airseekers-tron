#!/usr/bin/env bash
# Build + test the workspace on an x86_64 dev host inside docker (no mower needed).
#   ./scripts/dev_build.sh                 # colcon build (all packages)
#   ./scripts/dev_build.sh test            # colcon test + result summary
#   ./scripts/dev_build.sh shell           # interactive shell in the image
#   ./scripts/dev_build.sh clean           # wipe build/ install/ log/ (distro switch)
#   ./scripts/dev_build.sh build --packages-select mower_mission
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_ROOT="$(dirname "$SCRIPT_DIR")"
IMG=mower:jazzy-dev-amd64
ROSDISTRO=jazzy
cd "$STACK_ROOT"
mode="${1:-build}"; shift || true
if ! docker image inspect "$IMG" >/dev/null 2>&1 || [ "${REBUILD_IMAGE:-0}" = 1 ]; then
  # Fields2Cover v3 layer (built once, ~2 min at -j2); see scripts/build_f2c.sh.
  # The layer is distro-specific — it links against its base distro's libs (Noble's
  # libgdal/libtinyxml2 — a Jammy-built tree fails to link in the Noble image, the
  # "libgdal.so.30 not found" trap) — so rebuild when it predates the Jazzy/Noble move.
  f2c_distro="$(docker image inspect --format '{{index .Config.Labels "com.airseekers.ros_distro"}}' mower-f2c:v3-amd64 2>/dev/null || true)"
  if [ "$f2c_distro" != "jazzy" ]; then
    echo "notice: mower-f2c:v3-amd64 is not a Jazzy/Noble layer (label=$f2c_distro); rebuilding Fields2Cover v3"
    ARCH=amd64 "$SCRIPT_DIR/build_f2c.sh"
  fi
  base="ros@$(docker manifest inspect ros:jazzy-ros-base-noble | python3 -c "
import json,sys; d=json.load(sys.stdin)
print(next(m['digest'] for m in d['manifests'] if m['platform'].get('os')=='linux' and m['platform']['architecture']=='amd64'))")"
  DOCKER_BUILDKIT=0 docker build --build-arg BASE="$base" -f docker/Dockerfile.dev-amd64 -t "$IMG" docker
fi
# Host networking + default domain 0 put dev-host test nodes on the robot's DDS graph over the
# LAN (2026-10-07: a test feeder's /odometry/filtered_map latched a lethal boundary stop).
run() { docker run --rm -i ${TTY:+-t} --network host -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID="${DEV_ROS_DOMAIN_ID:-77}" -v "$STACK_ROOT":/work -w /work "$IMG" bash -c "source /opt/ros/jazzy/setup.bash; $*"; }
# A build/ tree left from another ROS distro (e.g. the Humble -> Jazzy move) pins
# /opt/ros/humble paths in its CMakeCache (gtest_vendor, FastRTPS) and fails configure
# inside this image. Stale artifacts are unusable across a distro/ABI change: wipe and
# rebuild rather than fail cryptically. ALLOW_STALE_BUILD=1 keeps the cache and proceeds.
if [ "$mode" != clean ] && [ "${ALLOW_STALE_BUILD:-0}" != 1 ]; then
  stale="$(grep -rhoE '/opt/ros/[a-z]+' build/*/CMakeCache.txt 2>/dev/null | sort -u | grep -v "/opt/ros/$ROSDISTRO" || true)"
  if [ -n "$stale" ]; then
    echo "notice: build/ has a stale CMake cache from another ROS distro ($(echo $stale | tr '\n' ' ')); cleaning build/ install/ log/"
    run "rm -rf build install log"
  fi
fi
case "$mode" in
  shell) TTY=1 run bash ;;
  clean) run "rm -rf build install log" ;;
  build) run "colcon build --symlink-install --event-handlers console_cohesion+ $*" ;;
  test)  run "source install/setup.bash; colcon test --event-handlers console_cohesion+ $* ; colcon test-result --verbose" ;;
  *) echo "unknown mode $mode" >&2; exit 2 ;;
esac
