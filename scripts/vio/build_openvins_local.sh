#!/usr/bin/env bash
# Build OpenVINS (ov_core, ov_init, ov_msckf + the ros2_serial_msckf bag runner) on the amd64
# dev box in mower:jazzy-dev-amd64, for the offline VIO evaluation (scripts/vio/eval_vio_bag.sh).
# Nothing here touches the mower; the arm64 build is scripts/build_openvins_mower.sh.
#
#   scripts/vio/build_openvins_local.sh          # stage + build (about 4 min at -j6, incremental after)
#   scripts/vio/build_openvins_local.sh verify   # executables present?
#
# Output: $VIO_BUILD (default <stack>/.vio_build, gitignored): src/open_vins (patched copy),
# build/, install/, log/. The image mounts it at /vio, so source /vio/install/setup.bash there.
# Env: VIO_BUILD, JOBS (default 6), IMG (default mower:jazzy-dev-amd64), OV_SRC.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
VIO_BUILD="${VIO_BUILD:-$STACK_ROOT/.vio_build}"
JOBS="${JOBS:-6}"
IMG="${IMG:-mower:jazzy-dev-amd64}"

verify() {
  local ok=0
  for x in run_subscribe_msckf ros2_serial_msckf; do
    if [ -x "$VIO_BUILD/install/ov_msckf/lib/ov_msckf/$x" ]; then echo "ok   $x"
    else echo "MISSING $x"; ok=1; fi
  done
  return $ok
}

build() {
  mkdir -p "$VIO_BUILD/src"
  touch "$VIO_BUILD/COLCON_IGNORE"   # the stack's own colcon build must not find these packages
  # Stage fresh, then sync by checksum: files that did not change keep their mtime, so a
  # rebuild only recompiles what the sources or patches changed.
  local tmp; tmp=$(mktemp -d)
  trap 'rm -rf "$tmp"' RETURN
  "$SCRIPT_DIR/stage_openvins.sh" "$tmp/open_vins"
  rsync -a -c --delete "$tmp/open_vins/" "$VIO_BUILD/src/open_vins/"
  docker run --rm --network none --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$VIO_BUILD":/vio -w /vio "$IMG" bash -c "source /opt/ros/jazzy/setup.bash && \
      MAKEFLAGS=-j$JOBS colcon --log-base /vio/log build --base-paths /vio/src \
        --build-base /vio/build --install-base /vio/install --packages-up-to ov_msckf \
        --event-handlers console_cohesion+ \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DOV_ROS2_SERIAL=ON \
        2>&1 | tail -25"
  verify
}

case "${1:-build}" in
  build)  build ;;
  verify) verify ;;
  *) sed -n 2,13p "$0"; exit 2 ;;
esac
