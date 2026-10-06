#!/usr/bin/env bash
# Deploy / build / run the Humble stack on the mower itself (RK3588, 4 GB RAM, Ubuntu 20.04).
#
#   ./scripts/deploy_to_mower.sh sync      # rsync this tree -> mower:$REMOTE_DIR (no build artefacts)
#   ./scripts/deploy_to_mower.sh image     # docker compose build (arm64 Humble image) ON the mower
#   ./scripts/deploy_to_mower.sh build     # colcon build inside the container (low parallelism: 4 GB RAM)
#   ./scripts/deploy_to_mower.sh up|down   # start / stop the stack (+ GUI if gui compose file exists)
#   ./scripts/deploy_to_mower.sh logs      # follow container logs
#   ./scripts/deploy_to_mower.sh shell     # interactive shell in the running container
#   ./scripts/deploy_to_mower.sh all       # sync + image + build
#
# Env: MOWER_HOST (192.168.1.105) MOWER_USER (airseekers) SSHPASS (password; or use ssh keys)
#      REMOTE_DIR (/userdata/ros2_stack: the root fs on the device is full, /userdata has space)
#
# The device root filesystem is ~100% full; everything (sources, docker data) must live on /userdata.
# Nothing here drives the robot: the stack starts with the cutter off and no mission layer.
set -euo pipefail

MOWER_HOST="${MOWER_HOST:-192.168.1.105}"
MOWER_USER="${MOWER_USER:-airseekers}"
REMOTE_DIR="${REMOTE_DIR:-/userdata/ros2_stack}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_ROOT="$(dirname "$SCRIPT_DIR")"
SKIP_PKGS="open_vins ov_core ov_msckf ov_eval ov_init ov_data"

if [ -n "${SSHPASS:-}" ] && command -v sshpass >/dev/null; then
  SSH=(sshpass -e ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10)
  RSYNC_RSH="sshpass -e ssh -o StrictHostKeyChecking=accept-new"
else
  SSH=(ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10)
  RSYNC_RSH="ssh -o StrictHostKeyChecking=accept-new"
fi
remote() { "${SSH[@]}" "$MOWER_USER@$MOWER_HOST" "$@"; }
# Extra compose fragments: GUI always (if present), video relay with VIDEO=1
compose() { remote "cd $REMOTE_DIR && DOCKER_BUILDKIT=0 docker compose -f docker/docker-compose.yml $( [ -f "$STACK_ROOT/docker/docker-compose.gui.yml" ] && echo -f docker/docker-compose.gui.yml ) $( [ "${VIDEO:-0}" = 1 ] && echo -f docker/docker-compose.video.yml ) $*"; }

do_sync() {
  remote "mkdir -p $REMOTE_DIR"
  rsync -az --delete --info=stats1 -e "$RSYNC_RSH" \
    --exclude build/ --exclude install/ --exclude log/ --exclude .git/ \
    --exclude '__pycache__/' --exclude '.pytest_cache/' \
    --exclude third_party/mowglinext/gui/web/node_modules/ \
    --exclude third_party/mowglinext/gui/web/dist/ \
    --exclude third_party/mowglinext/gui/mowglinext \
    --exclude config/gui/mowgli_robot.yaml \
    "$STACK_ROOT/" "$MOWER_USER@$MOWER_HOST:$REMOTE_DIR/"
  # config/gui/mowgli_robot.yaml is DEVICE-OWNED (datum, GUI-saved settings): never overwrite it;
  # seed it from the repo copy only when the device has none.
  remote "[ -f $REMOTE_DIR/config/gui/mowgli_robot.yaml ] || cp $REMOTE_DIR/config/gui/mowgli_robot.yaml.seed $REMOTE_DIR/config/gui/mowgli_robot.yaml 2>/dev/null || true"
  # open_vins is a relative symlink into ../ros2_port_handoff which is not synced: drop it remotely
  remote "rm -f $REMOTE_DIR/src/open_vins"
  write_deployed_rev
}

# DEPLOYED_REV: what crash records / scripts/crash_report.sh report as the deployed version
# (git HEAD + dirty count when this tree is a git checkout, else a content hash of src/ launch/ config/).
write_deployed_rev() {
  local rev
  if git -C "$STACK_ROOT" rev-parse HEAD >/dev/null 2>&1; then
    rev="git:$(git -C "$STACK_ROOT" rev-parse HEAD) dirty:$(git -C "$STACK_ROOT" status --porcelain | wc -l)"
  else
    rev="nogit sha256:$(cd "$STACK_ROOT" && find src launch config scripts -type f ! -path '*/__pycache__/*' \
          -print0 | sort -z | xargs -0 sha256sum | sha256sum | cut -c1-16)"
  fi
  remote "echo '$rev synced $(date -Iseconds) by $(whoami)@$(hostname)' > $REMOTE_DIR/DEPLOYED_REV"
}

do_image() {
  # BuildKit is off: the device kernel lacks CONFIG_POSIX_MQUEUE (see docs/deployment.md).
  # Logs live outside $REMOTE_DIR: `sync` uses --delete and would remove them.
  remote "mkdir -p /userdata/ros2/logs"
  compose "build 2>&1 | tee /userdata/ros2/logs/image_build.log | tail -5"
}

do_build() {
  # 4 GB RAM, no swap: keep both colcon and make at 2 jobs; rosidl + Fields2Cover are the heavy bits.
  remote "mkdir -p /userdata/ros2/logs"
  compose "run --rm mower_humble bash -c 'source /opt/ros/humble/setup.bash && cd /work && MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2 --event-handlers console_cohesion+ --packages-skip $SKIP_PKGS 2>&1 | tee /userdata/ros2/logs/colcon_build.log | tail -40'"
}

case "${1:-}" in
  sync)  do_sync ;;
  image) do_image ;;
  build) do_build ;;
  all)   do_sync; do_image; do_build ;;
  up)    remote "mkdir -p /userdata/ros2/maps /userdata/ros2/calibration /userdata/ros2/gui_db"; compose "up -d" ;;
  down)  compose "down" ;;
  logs)  compose "logs -f --tail 200" ;;
  shell) remote -t "docker exec -it mower_humble bash" ;;
  *) sed -n 2,14p "$0"; exit 2 ;;
esac
