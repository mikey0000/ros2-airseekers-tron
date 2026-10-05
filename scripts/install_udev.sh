#!/usr/bin/env bash
# Install the mower udev rules + camera udev rules so /dev/serial_mower etc.
# and /dev/left_oa_camera etc. exist as stable symlinks. Run once as root.
set -euo pipefail
cd "$(dirname "$0")/.."
install -m 0644 config/udev/mower.rules /etc/udev/rules.d/
if [ -f config/cameras/99-mower-cameras.rules ]; then
  install -m 0644 config/cameras/99-mower-cameras.rules /etc/udev/rules.d/
fi
udevadm control --reload
udevadm trigger
ls -l /dev/serial_mower /dev/serial_imu /dev/serial_rtk 2>/dev/null || true
