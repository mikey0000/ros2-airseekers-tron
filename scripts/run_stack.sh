#!/usr/bin/env bash
# Run the ROS 2 stack inside the Humble container (foreground, Ctrl-C to stop).
# Usage:
#   ./scripts/run_stack.sh [launch args]   # ros2 launch mower_bringup mower.launch.py [args]
#                                          #   e.g. teleop:=false navigation:=true
#   ./scripts/run_stack.sh shell           # interactive shell in the container
#
# The systemd unit / `docker compose up -d` runs the same launch detached; stop it
# first (docker compose -f docker/docker-compose.yml down) or the two stacks will
# fight over the serial ports.
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

if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx mower_humble; then
  echo "WARNING: mower_humble (compose service) is already running the stack." >&2
fi

ENV_SETUP='source /opt/ros/humble/setup.bash; [ -f /work/install/setup.bash ] && source /work/install/setup.bash'

if [ "${1:-}" = "shell" ]; then
  exec docker compose -f docker/docker-compose.yml run --rm mower_humble \
    bash -c "$ENV_SETUP; exec bash"
fi

exec docker compose -f docker/docker-compose.yml run --rm mower_humble \
  bash -c "$ENV_SETUP; exec ros2 launch mower_bringup mower.launch.py \"\$@\"" bash "$@"
