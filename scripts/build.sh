#!/usr/bin/env bash
# Build the ROS 2 workspace inside the Humble container (colcon).
# Host stays on 20.04/Noetic; only the container is Jammy/Humble.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$STACK_ROOT"

# Extra args pass through to colcon, e.g.: ./scripts/build.sh --packages-select mower_bringup
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml run --rm mower_humble \
  bash -c 'set -e; source /opt/ros/humble/setup.bash; cd /work; colcon build --symlink-install "$@"' _ "$@"

echo "OK: build finished; overlay at ros2_stack/src -> /work/install (inside container mounts)."
