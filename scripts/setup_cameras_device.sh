#!/usr/bin/env bash
# Host-side camera prerequisites for the ROS 2 stack (run ON THE MOWER HOST, not in docker).
#
# Verified 2026-10-06 on the stock rootfs: everything the OA (rkisp) cameras need is
# already brought up at boot by the vendor image, so the default mode only CHECKS:
#
#   * /dev/{left_oa,right_oa,rear}_camera udev symlinks (video53 / video44 / video62)
#   * rkaiq_3A.service -> /usr/bin/rkaiq_3A_server running (Rockchip 3A: AE/AWB/... for
#     both ISPs; without it rkisp still streams but exposure/colour are not regulated)
#   * IQ tuning /etc/iqfiles/gc2093_MY_default.json present (scripts/setup_camera_iq.sh)
#   * media graphs linked: rkcif-mipi-lvds{,5} -> rkisp-isp-subdev -> rkisp_mainpath
#     ("[ENABLED]" in `media-ctl -p`) -- the kernel DT sets these up, no media-ctl -l needed
#   * nobody else holds the video nodes (one streaming process per V4L2 node)
#
# The container (`mower_humble`, --privileged, host network) then just opens the nodes:
# the OA nodes are V4L2 *multi-planar* (driver rkisp_v6, UYVY/NV16/NV61/NV21/NV12,
# 32x32..1920x1080, 30 fps), captured by mower_cameras/v4l2_cam.
#
# Usage:  scripts/setup_cameras_device.sh [check|start-3a]
#   check     (default) read-only checks, exit 1 on a hard failure
#   start-3a  if rkaiq_3A_server is not running, start it the way the vendor init script
#             does (/etc/init.d/rkaiq_3A.sh start; needs root). Does not enable services.
set -uo pipefail

MODE="${1:-check}"
fail=0
ok()   { echo "[cameras] ok:   $*"; }
warn() { echo "[cameras] WARN: $*" >&2; }
bad()  { echo "[cameras] FAIL: $*" >&2; fail=1; }

for cam in left_oa_camera right_oa_camera rear_camera; do
    if [[ -e "/dev/$cam" ]]; then
        ok "/dev/$cam -> $(readlink "/dev/$cam") ($(cat "/sys/class/video4linux/$(readlink "/dev/$cam")/name" 2>/dev/null))"
    else
        bad "/dev/$cam missing (udev rules: scripts/99-mower-cameras.rules)"
    fi
done

if pgrep -x rkaiq_3A_server >/dev/null; then
    ok "rkaiq_3A_server running (pid $(pgrep -x rkaiq_3A_server | head -1))"
elif [[ "$MODE" == "start-3a" ]]; then
    echo "[cameras] starting rkaiq_3A_server via /etc/init.d/rkaiq_3A.sh start"
    /etc/init.d/rkaiq_3A.sh start && sleep 1
    pgrep -x rkaiq_3A_server >/dev/null && ok "rkaiq_3A_server started" \
        || bad "rkaiq_3A_server did not start"
else
    warn "rkaiq_3A_server not running: OA images will be un-regulated (dark/green)." \
         "Run: sudo systemctl start rkaiq_3A  (or: $0 start-3a)"
fi

[[ -f /etc/iqfiles/gc2093_MY_default.json ]] && ok "IQ file gc2093_MY_default.json" \
    || warn "/etc/iqfiles/gc2093_MY_default.json missing (scripts/setup_camera_iq.sh)"

if command -v media-ctl >/dev/null; then
    for m in /dev/media*; do
        drv=$(media-ctl -d "$m" -p 2>/dev/null | awk '/^driver/{print $2; exit}')
        [[ "$drv" == rkisp* ]] || continue
        node=$(media-ctl -d "$m" -p 2>/dev/null | awk '/entity .*rkisp_mainpath/{f=1} f&&/device node/{print $4; exit}')
        if media-ctl -d "$m" -p 2>/dev/null | grep -q -E '<- "rkcif-mipi-lvds[0-9]*":0 \[ENABLED'; then
            ok "$m ($drv): rkcif -> isp linked, mainpath $node"
        else
            bad "$m ($drv): rkcif -> rkisp-isp-subdev link not enabled"
        fi
    done
else
    warn "media-ctl not installed; skipping graph check"
fi

if [[ $EUID -eq 0 ]] && command -v fuser >/dev/null; then
    for cam in left_oa_camera right_oa_camera rear_camera; do
        holders=$(fuser "/dev/$(readlink "/dev/$cam")" 2>/dev/null | xargs)
        [[ -n "$holders" ]] && warn "/dev/$cam held by pid(s) $holders" \
            "($(ps -o comm= -p ${holders// /,} | tr '\n' ' '))"
    done
fi

exit $fail
