#!/usr/bin/env bash
# Idempotent install of the Rockchip rkaiq ISP tuning (IQ) file for the OA cameras.
#
# The mower's left/right obstacle-avoidance cameras are GalaxyCore GC2093 sensors
# (confirmed from the device tree: /i2c@feab0000/gc2093b_1@37 and /i2c@fead0000/gc2093b_1@37).
# rkaiq (camera_engine_rkaiq, the Rockchip 3A engine) reads a per-sensor tuning JSON from
# /etc/iqfiles by default; without it the OA cameras produce wrong colour/exposure.
#
# Also required (not installed by this script — they live in the BSP kernel/modules):
#   * gc2093.ko            — the GC2093 sensor V4L2 subdev driver
#   * camera_engine_rkaiq  — the 3A userspace (librkaiq.so + rkaiq_3A_server)
#   * video_rkisp.ko       — Rockchip ISP driver (already used by the stereo front)
#
# Usage:
#   IQ_SRC=/abs/path/to/iqfiles  IQ_DIR=/etc/iqfiles  ./setup_camera_iq.sh
set -euo pipefail

# --- config ---------------------------------------------------------------
IQ_DIR="${IQ_DIR:-/etc/iqfiles}"                 # where rkaiq looks for tunings
IQ_FILE="${IQ_FILE:-gc2093_MY_default.json}"     # sensor tuning to install

# Locate the source iqfiles dir: explicit env, then relative to this script's repo.
if [[ -n "${IQ_SRC:-}" ]]; then
    SRC="$IQ_SRC"
else
    SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/iqfiles"
fi

echo "[camera-iq] source      : $SRC"
echo "[camera-iq] iq dir       : $IQ_DIR"

# --- checks ---------------------------------------------------------------
if [[ ! -f "$SRC/$IQ_FILE" ]]; then
    echo "[camera-iq] ERROR: $IQ_FILE not found under $SRC" >&2
    echo "[camera-iq]        run with IQ_SRC=/path/to/iqfiles pointing at the mower-dumped set" >&2
    exit 1
fi

# --- install --------------------------------------------------------------
mkdir -p "$IQ_DIR"
if [[ ! -f "$IQ_DIR/$IQ_FILE" ]]; then
    cp "$SRC/$IQ_FILE" "$IQ_DIR/$IQ_FILE"
    echo "[camera-iq] installed $IQ_FILE -> $IQ_DIR/"
else
    echo "[camera-iq] $IQ_FILE already present (skip)"
fi

# --- required-but-not-installed dependencies ------------------------------
warn_missing() {
    local label="$1"; shift
    local found=0
    for p in "$@"; do
        [[ -e "$p" ]] && found=1 && break
    done
    if [[ "$found" -eq 0 ]]; then
        echo "[camera-iq] WARN: $label not found ($*) — OA cameras will not stream" >&2
    else
        echo "[camera-iq] ok: $label present"
    fi
}
warn_missing "gc2093 sensor driver" \
    "/lib/modules/$(uname -r)/*/gc2093.ko" "/lib/modules/$(uname -r)/gc2093.ko"
warn_missing "rkaiq 3A engine" \
    "/usr/bin/rkaiq_3A_server" "/usr/bin/rkisp_demo" "/usr/lib/librkaiq.so"

echo "[camera-iq] done. Restart the rkaiq/media pipeline to pick up the tuning."
