#!/usr/bin/env bash
# Run the ROS 2 stack inside the Humble container.
# Usage:
#   ./scripts/run_stack.sh                                   # interactive shell in container
#   ./scripts/run_stack.sh <pkg> <launch.py> [launch args]  # ros2 launch <pkg> <launch.py> ...
set -euo pipefail

# See build.sh: mower kernel lacks CONFIG_POSIX_MQUEUE, keep BuildKit off.
export DOCKER_BUILDKIT=0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$STACK_ROOT"

# Allow GUI tools (rviz) when a display is present; harmless over SSH without X.
if [ -n "${DISPLAY:-}" ]; then
  xhost +local:root >/dev/null 2>&1 || true
fi

if [ "$#" -eq 0 ]; then
  exec docker compose -f docker/docker-compose.yml run --rm mower_humble \
    bash -c 'source /opt/ros/humble/setup.bash; [ -f /work/install/setup.bash ] && source /work/install/setup.bash; exec bash'
fi

PKG="$1"; shift
LAUNCH_FILE="$1"; shift || true

docker compose -f docker/docker-compose.yml run --rm mower_humble \
  bash -c 'source /opt/ros/humble/setup.bash; [ -f /work/install/setup.bash ] && source /work/install/setup.bash; exec ros2 launch "$0" "$1" "$@"' \
  "$PKG" "$LAUNCH_FILE" "$@"
