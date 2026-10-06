#!/usr/bin/env bash
# Front (Metoak XC9080) stereo watchdog. Runs ON THE MOWER HOST as root (systemd unit
# config/systemd/mower-stereo-watchdog.service; install with scripts/install_stereo_watchdog.sh).
#
# Problem (2026-10-06): after a power-loss reboot the vendor cam.service runs
# /usr/metoak/metoak/mo_init.sh too early, the XC9080/Simor i2c init does not take, and the
# stereo channel (rkcif-mipi-lvds3, /dev/video22) delivers no frames: stereo_cam's capture
# loop times out every 2 s and re-opens the device, so the kernel logs
# "rkcif-mipi-lvds3: stream[0] start streaming" over and over. Re-running mo_init.sh by hand
# fixed it.
#
# Detection is host-only (no ROS): a healthy stream starts once and keeps running, a dead one
# restarts every few seconds. If the restart count grows by >= RESTARTS within WINDOW seconds,
# re-run mo_init.sh. At most MAX_INITS re-inits per boot, so a genuinely broken camera cannot
# loop the init script (which also power-cycles the rear USB camera).
#
# Usage: stereo_watchdog.sh [watch|check]
#   watch  (default) loop forever (systemd)
#   check  one window, print the verdict, never runs mo_init.sh
set -uo pipefail

MODE="${1:-watch}"
WINDOW="${STEREO_WD_WINDOW:-20}"
RESTARTS="${STEREO_WD_RESTARTS:-4}"
MAX_INITS="${STEREO_WD_MAX_INITS:-3}"
SETTLE="${STEREO_WD_SETTLE:-30}"
INIT=/usr/metoak/metoak/mo_init.sh
PATTERN='rkcif-mipi-lvds3: stream\[0\] start streaming'

log() { echo "[stereo-wd] $*"; }
starts() { dmesg 2>/dev/null | grep -c "$PATTERN"; }

window_restarts() {
    local a b
    a=$(starts); sleep "$WINDOW"; b=$(starts)
    echo $((b - a))
}

if [[ "$MODE" == "check" ]]; then
    n=$(window_restarts)
    if (( n >= RESTARTS )); then log "DEAD: $n stream restarts in ${WINDOW}s"; exit 1; fi
    log "ok: $n stream restarts in ${WINDOW}s"; exit 0
fi

[[ -x "$INIT" || -f "$INIT" ]] || { log "$INIT missing; nothing to do"; exit 0; }
log "watching (window ${WINDOW}s, threshold ${RESTARTS}, max ${MAX_INITS} re-inits/boot)"
sleep "$SETTLE"     # let cam.service and the stack come up first
inits=0
while true; do
    n=$(window_restarts)
    if (( n >= RESTARTS )); then
        if (( inits >= MAX_INITS )); then
            log "stereo still dead after ${inits} re-inits; giving up until reboot"
            exec sleep infinity
        fi
        inits=$((inits + 1))
        log "stereo dead ($n restarts in ${WINDOW}s): re-running mo_init.sh (${inits}/${MAX_INITS})"
        sh "$INIT" 2>&1 | grep -E 'succeed|error|fail' | sed 's/^/[stereo-wd] mo_init: /'
        sleep "$SETTLE"
    fi
done
