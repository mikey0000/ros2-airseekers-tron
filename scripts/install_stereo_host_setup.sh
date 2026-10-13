#!/usr/bin/env bash
# Install + start the boot-time stereo host setup ON THE MOWER HOST (needs root):
#   * /etc/systemd/system/mower-stereo-host.service runs scripts/stereo_host_boot.sh once per
#     boot (after cam.service, before docker): IIO-mode stereo IMU + 8 MiB DDS socket buffers;
#   * /etc/sysctl.d/90-ros2-dds.conf persists the socket buffer sizes as well, so they hold
#     even if the unit is ever disabled.
# Opt-in host change, like install_mower_service.sh / install_stereo_watchdog.sh.
# Run from the stack checkout on the robot:  sudo scripts/install_stereo_host_setup.sh
set -euo pipefail
cd "$(dirname "$0")/.."
[[ $EUID -eq 0 ]] || { echo "run as root (sudo)"; exit 1; }

install -m 0644 config/systemd/mower-stereo-host.service /etc/systemd/system/
printf 'net.core.rmem_max=8388608\nnet.core.wmem_max=8388608\n' > /etc/sysctl.d/90-ros2-dds.conf
chmod 0644 /etc/sysctl.d/90-ros2-dds.conf
sysctl -q -p /etc/sysctl.d/90-ros2-dds.conf
chmod 0755 scripts/stereo_host_boot.sh scripts/setup_stereo_host.sh

systemctl daemon-reload
systemctl enable --now mower-stereo-host.service
systemctl --no-pager status mower-stereo-host.service | head -8
echo
sh scripts/setup_stereo_host.sh status
echo "Installed mower-stereo-host.service (enabled; runs once per boot). Check: journalctl -u mower-stereo-host"
