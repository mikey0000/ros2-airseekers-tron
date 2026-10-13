#!/usr/bin/env bash
# Install + start the boot-time stereo host setup ON THE MOWER HOST (needs root):
#   * /etc/systemd/system/mower-stereo-host.service runs scripts/stereo_host_boot.sh once per
#     boot (after cam.service, before docker): IIO-mode stereo IMU + 8 MiB DDS socket buffers;
#   * /etc/sysctl.d/90-ros2-dds.conf persists the socket buffer sizes as well, so they hold
#     even if the unit is ever disabled.
# Opt-in host change, like install_mower_service.sh / install_stereo_watchdog.sh.
# Run from the stack checkout on the robot:
#   sudo scripts/install_stereo_host_setup.sh               install / refresh
#   sudo scripts/install_stereo_host_setup.sh --uninstall   remove the unit + sysctl file and put
#                                                           the live host back to vendor state
#                                                           (netlink IMU mode, 208 KiB buffers;
#                                                           the stack loses the stereo IMU until
#                                                           re-installed)
set -euo pipefail
cd "$(dirname "$0")/.."
[[ $EUID -eq 0 ]] || { echo "run as root (sudo)"; exit 1; }
UNIT=mower-stereo-host.service
SYSCTL=/etc/sysctl.d/90-ros2-dds.conf

if [[ ${1:-} == --uninstall ]]; then
    systemctl disable --now "$UNIT" 2>/dev/null || true
    rm -f "/etc/systemd/system/$UNIT" "$SYSCTL"
    systemctl daemon-reload
    sh scripts/setup_stereo_host.sh revert
    echo "uninstalled $UNIT and $SYSCTL"
    exit 0
fi

install -m 0644 "config/systemd/$UNIT" /etc/systemd/system/
printf 'net.core.rmem_max=8388608\nnet.core.wmem_max=8388608\n' > "$SYSCTL"
chmod 0644 "$SYSCTL"
sysctl -q -p "$SYSCTL"
chmod 0755 scripts/stereo_host_boot.sh scripts/setup_stereo_host.sh

systemctl daemon-reload
systemctl enable --now "$UNIT"
systemctl --no-pager status "$UNIT" | head -8
echo
sh scripts/setup_stereo_host.sh status
echo "Installed $UNIT (enabled; runs once per boot). Check: journalctl -u mower-stereo-host"
