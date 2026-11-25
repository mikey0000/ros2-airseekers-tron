#!/usr/bin/env bash
# Stage a patched copy of OpenVINS for a build (docs/vio.md section 4). Shared by
# scripts/build_openvins_mower.sh (arm64, on the mower) and scripts/vio/build_openvins_local.sh
# (amd64 dev image).
#
#   scripts/vio/stage_openvins.sh DEST      # DEST must not exist; DEST/ov_core, ov_msckf, ...
#
# * sources: the third_party/open_vins submodule (pinned rpng/open_vins; OV_SRC overrides);
# * ov_eval / ov_data get COLCON_IGNORE (not needed on the robot or for the evaluation);
# * config/vio/patches/*.patch applied in order (image QoS, deep IMU queue, serial runner CMake);
# * config/vio/serial/*.cpp copied into ov_msckf/src (built only with -DOV_ROS2_SERIAL=ON).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
OV_SRC="${OV_SRC:-$STACK_ROOT/third_party/open_vins}"
dest="${1:?usage: $0 DEST}"
[ -f "$OV_SRC/ov_msckf/package.xml" ] || {
  echo "OpenVINS sources missing at $OV_SRC: run git submodule update --init third_party/open_vins" >&2; exit 1; }
[ -e "$dest" ] && { echo "$dest exists: stage into a new directory" >&2; exit 1; }
mkdir -p "$dest"
rsync -a --exclude .git --exclude docs --exclude 'Dockerfile*' "$OV_SRC/" "$dest/"
touch "$dest/ov_eval/COLCON_IGNORE" "$dest/ov_data/COLCON_IGNORE"
for p in "$STACK_ROOT"/config/vio/patches/*.patch; do
  echo "   applying $(basename "$p")"
  (cd "$dest" && patch -s -p1 --forward < "$p")
done
cp "$STACK_ROOT"/config/vio/serial/*.cpp "$dest/ov_msckf/src/"
