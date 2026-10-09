#!/usr/bin/env bash
# Build OpenVINS (ov_core, ov_init, ov_msckf) for arm64 ON THE MOWER, in a one-off container,
# with the stack STOPPED. Run from the dev host. See docs/vio.md.
#
#   SSHPASS=... ./scripts/build_openvins_mower.sh check   # preflight only, changes nothing
#   SSHPASS=... ./scripts/build_openvins_mower.sh build   # sync sources + build (30-60 min)
#   SSHPASS=... ./scripts/build_openvins_mower.sh verify  # ov_msckf executable present?
#
# Why this shape (load-190 incident: never compile C++ inside the running stack):
#   * REFUSES to run while mower_jazzy is up: the owner stops it first
#     (./scripts/deploy_to_mower.sh down), then brings it back with `up` afterwards.
#   * Low-memory recipe for a 3.9 GB / no-swap RK3588: one colcon worker, make -j1,
#     container capped at 2.5 GB / 2 CPUs (an OOM kills the build, not the host),
#     -O2 without debug info, GCC's GC tuned to collect early, BUILD_TESTING off.
#     ov_msckf's Eigen-heavy TUs peak at ~1.2 GB each with these flags.
#   * Sources: the third_party/open_vins git submodule (pinned upstream rpng/open_vins;
#     `git submodule update --init` after cloning). third_party/ is COLCON_IGNOREd, so the
#     normal stack build never compiles it; a patched copy goes to
#     /userdata/ros2/src_ext/open_vins, outside the stack's colcon tree. Patches: config/vio/patches/*.patch (image topics use
#     SensorDataQoS, otherwise ov_msckf never receives stereo_cam's best-effort frames).
#   * Output goes into the stack's install space (/work/install/ov_*), so the normal
#     `source /work/install/setup.bash` finds ov_msckf; build tree in /userdata/ros2/build_ov.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_ROOT="$(dirname "$SCRIPT_DIR")"
MOWER="${MOWER:-root@192.168.1.105}"
IMAGE="${IMAGE:-mower:jazzy}"
OV_SRC="${OV_SRC:-$STACK_ROOT/third_party/open_vins}"
[ -f "$OV_SRC/ov_msckf/package.xml" ] || { echo "OpenVINS sources missing at $OV_SRC: run git submodule update --init third_party/open_vins" >&2; exit 1; }
REMOTE_SRC=/userdata/ros2/src_ext
REMOTE_BUILD=/userdata/ros2/build_ov
MIN_AVAIL_MB=3000
: "${SSHPASS:?export SSHPASS (mower password)}"
export SSHPASS
SSH_OPTS=(-o StrictHostKeyChecking=no -o LogLevel=ERROR)
remote() { sshpass -e ssh "${SSH_OPTS[@]}" "$MOWER" "$@"; }

preflight() {
  echo "== preflight on $MOWER"
  if remote "docker ps -q -f name=^mower_jazzy\$ -f status=running" | grep -q .; then
    echo "REFUSING: mower_jazzy is running. Stop the stack first (deploy_to_mower.sh down)." >&2
    exit 1
  fi
  if remote "docker ps -q -f name=^ov_build\$" | grep -q .; then
    echo "REFUSING: an ov_build container is already running." >&2; exit 1
  fi
  local avail
  avail=$(remote "awk '/MemAvailable/ {print int(\$2/1024)}' /proc/meminfo")
  echo "MemAvailable ${avail} MB (need >= ${MIN_AVAIL_MB})"
  [ "$avail" -ge "$MIN_AVAIL_MB" ] || { echo "REFUSING: not enough free memory." >&2; exit 1; }
  remote "uptime; df -h /userdata | tail -1; docker image inspect $IMAGE >/dev/null && echo image $IMAGE ok"
}

sync_sources() {
  echo "== staging patched OpenVINS from $OV_SRC"
  local tmp; tmp=$(mktemp -d)
  trap 'rm -rf "$tmp"' RETURN
  rsync -a --exclude .git --exclude docs --exclude 'Dockerfile*' "$OV_SRC/" "$tmp/open_vins/"
  # ov_eval/ov_data are not needed on the robot.
  touch "$tmp/open_vins/ov_eval/COLCON_IGNORE" "$tmp/open_vins/ov_data/COLCON_IGNORE"
  for p in "$STACK_ROOT"/config/vio/patches/*.patch; do
    echo "   applying $(basename "$p")"; (cd "$tmp/open_vins" && patch -p1 --forward < "$p")
  done
  remote "mkdir -p $REMOTE_SRC $REMOTE_BUILD"
  rsync -a --delete -e "sshpass -e ssh ${SSH_OPTS[*]}" "$tmp/open_vins/" "$MOWER:$REMOTE_SRC/open_vins/"
}

build() {
  echo "== building in one-off container (expect 30-60 min; watch with: ssh $MOWER docker logs -f ov_build)"
  remote "docker run --rm --name ov_build --memory 2500m --memory-swap 2500m --cpus 2 \
      -v /userdata/ros2_stack:/work -v $REMOTE_SRC:/ov_src:ro -v $REMOTE_BUILD:/ov_build \
      $IMAGE bash -c 'source /opt/ros/jazzy/setup.bash && export MAKEFLAGS=-j1 && \
        nice -n 10 colcon build --base-paths /ov_src --build-base /ov_build \
          --install-base /work/install --packages-up-to ov_msckf --parallel-workers 1 \
          --event-handlers console_direct+ \
          --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
            \"-DCMAKE_CXX_FLAGS=-O2 -g0 --param ggc-min-expand=20 --param ggc-min-heapsize=131072\"'"
}

verify() {
  remote "ls -d /userdata/ros2_stack/install/ov_msckf/lib/ov_msckf/run_subscribe_msckf && \
          du -sh /userdata/ros2_stack/install/ov_core /userdata/ros2_stack/install/ov_init /userdata/ros2_stack/install/ov_msckf"
}

case "${1:-check}" in
  check)  preflight ;;
  build)  preflight; sync_sources; build; verify
          echo "== done. Bring the stack back with: ./scripts/deploy_to_mower.sh up (vio stays off: vio:=false)" ;;
  verify) verify ;;
  *) sed -n 2,8p "$0"; exit 2 ;;
esac
