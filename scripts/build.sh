#!/usr/bin/env bash
# Build the ROS 2 workspace inside the Jazzy container (colcon).
# Host stays on 20.04/Noetic; only the container is Noble/Jazzy.
set -euo pipefail

# Mower kernel lacks CONFIG_POSIX_MQUEUE: BuildKit RUN steps fail mounting mqueue,
# so force the legacy docker builder (see docs/deployment.md).
export DOCKER_BUILDKIT=0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_ROOT="$(dirname "$SCRIPT_DIR")"
ROSDISTRO=jazzy

cd "$STACK_ROOT"

# A build/ tree left from another ROS distro (e.g. the Humble -> Jazzy move) pins
# /opt/ros/humble paths in its CMakeCache (gtest_vendor, FastRTPS) and fails configure
# inside the Jazzy image. Stale artifacts are unusable across a distro/ABI change: wipe
# and rebuild. ALLOW_STALE_BUILD=1 keeps the cache and proceeds.
if [ "${ALLOW_STALE_BUILD:-0}" != 1 ]; then
  stale="$(grep -rhoE '/opt/ros/[a-z]+' build/*/CMakeCache.txt 2>/dev/null | sort -u | grep -v "/opt/ros/$ROSDISTRO" || true)"
  if [ -n "$stale" ]; then
    echo "notice: build/ has a stale CMake cache from another ROS distro ($(echo $stale | tr '\n' ' ')); cleaning build/ install/ log/"
    docker compose -f docker/docker-compose.yml run --rm mower_jazzy \
      bash -c 'rm -rf /work/build /work/install /work/log'
  fi
fi

# Extra args pass through to colcon, e.g.: ./scripts/build.sh --packages-select mower_bringup
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml run --rm mower_jazzy \
  bash -c 'set -e; source /opt/ros/jazzy/setup.bash; cd /work; colcon build --symlink-install "$@"' _ "$@"

echo "OK: build finished; overlay at ros2_stack/src -> /work/install (inside container mounts)."
