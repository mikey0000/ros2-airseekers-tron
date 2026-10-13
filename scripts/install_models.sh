#!/usr/bin/env bash
# Install the NPU models (and optionally a matching librknnrt.so) for det_ros / seg_ros.
#
# Source: ros2_port_handoff/11_perception_models/*.rknn (no source; vendor exports,
#         compiled with rknn-toolkit2 2.3.0, light classifier 2.3.2).
# Target: /userdata/ros2/models/ on the mower (persistent partition, bind-mounted into the
#         Jazzy container by docker/docker-compose.yml as /userdata/ros2).
#
#   ./scripts/install_models.sh                    # rsync models -> $MOWER_USER@$MOWER_HOST
#   ./scripts/install_models.sh --runtime          # + download librknnrt.so 2.3.0 ->
#                                                  #   /userdata/ros2/lib/librknnrt.so
#   ./scripts/install_models.sh --local DIR        # copy into a local dir (on the device,
#                                                  #   or for dev: models_dir:=DIR)
#   ./scripts/install_models.sh --dry-run          # print what would happen
#
# Env: MOWER_HOST (192.168.1.105) MOWER_USER (airseekers) SSHPASS (optional)
#      MODELS_SRC (default ../../ros2_port_handoff/11_perception_models)
#      DEST (default /userdata/ros2/models)  RKNN_RT_VERSION (default 2.3.0)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_ROOT="$(dirname "$SCRIPT_DIR")"
MOWER_HOST="${MOWER_HOST:-192.168.1.105}"
MOWER_USER="${MOWER_USER:-airseekers}"
MODELS_SRC="${MODELS_SRC:-$STACK_ROOT/../ros2_port_handoff/11_perception_models}"
DEST="${DEST:-/userdata/ros2/models}"
LIB_DEST="${LIB_DEST:-/userdata/ros2/lib}"
RKNN_RT_VERSION="${RKNN_RT_VERSION:-2.3.0}"
RT_URL="https://github.com/airockchip/rknn-toolkit2/raw/v${RKNN_RT_VERSION}/rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so"
# det_ros + seg_ros defaults first; the rest are kept for experiments.
MODELS=(best_large_0208.rknn pplite-seg_20260630-1-6cls.rknn
        best_small_0208.rknn pplite-seg_20260606.rknn model_1.rknn)

LOCAL=""; RUNTIME=0; DRY=0
while [ $# -gt 0 ]; do
  case "$1" in
    --local) LOCAL="$2"; shift 2 ;;
    --runtime) RUNTIME=1; shift ;;
    --dry-run) DRY=1; shift ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown arg $1" >&2; exit 2 ;;
  esac
done

run() { if [ "$DRY" = 1 ]; then echo "+ $*"; else "$@"; fi; }

files=()
for m in "${MODELS[@]}"; do
  if [ -f "$MODELS_SRC/$m" ]; then files+=("$MODELS_SRC/$m"); else echo "[models] missing $MODELS_SRC/$m" >&2; fi
done
[ ${#files[@]} -gt 0 ] || { echo "[models] no models found under $MODELS_SRC" >&2; exit 1; }

rt_tmp=""
if [ "$RUNTIME" = 1 ]; then
  rt_tmp="${TMPDIR:-/tmp}/librknnrt-${RKNN_RT_VERSION}.so"
  run curl -fsSL -o "$rt_tmp" "$RT_URL"
fi

if [ -n "$LOCAL" ]; then
  run mkdir -p "$LOCAL"
  run cp -v "${files[@]}" "$LOCAL/"
  [ -n "$rt_tmp" ] && run install -D -m 0644 "$rt_tmp" "$LOCAL/../lib/librknnrt.so"
  echo "[models] installed into $LOCAL (det_ros/seg_ros: models_dir:=$LOCAL)"
  exit 0
fi

if [ -n "${SSHPASS:-}" ] && command -v sshpass >/dev/null; then
  SSH=(sshpass -e ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10)
  RSH="sshpass -e ssh -o StrictHostKeyChecking=accept-new"
else
  SSH=(ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10)
  RSH="ssh -o StrictHostKeyChecking=accept-new"
fi
TARGET="$MOWER_USER@$MOWER_HOST"
run "${SSH[@]}" "$TARGET" "mkdir -p $DEST $LIB_DEST"
run rsync -av --checksum -e "$RSH" "${files[@]}" "$TARGET:$DEST/"
if [ -n "$rt_tmp" ]; then
  run rsync -av -e "$RSH" "$rt_tmp" "$TARGET:$LIB_DEST/librknnrt.so"
fi
run "${SSH[@]}" "$TARGET" "ls -la $DEST $LIB_DEST; strings $LIB_DEST/librknnrt.so 2>/dev/null | grep -m1 'librknnrt version' || true; strings /usr/lib/librknnrt.so 2>/dev/null | grep -m1 'librknnrt version' || true"
echo "[models] done. Compose must mount /userdata/ros2 (it does) and librknnrt.so at /usr/lib/librknnrt.so."
