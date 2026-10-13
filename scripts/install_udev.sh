#!/usr/bin/env bash
# Install the mower udev rules + camera udev rules so /dev/serial_mower etc.
# and /dev/left_oa_camera etc. exist as stable symlinks. Run once as root.
#   sudo scripts/install_udev.sh               install / refresh
#   sudo scripts/install_udev.sh --uninstall   put the rules back the way they were
#
# /etc/udev/rules.d/mower.rules is VENDOR-SHIPPED on the Tron (June 2025, the Noetic stack
# uses the same /dev/serial_* names; config/udev/mower.rules is a copy of it), so it is never
# simply deleted: a pre-existing file that differs from ours is saved as mower.rules.vendor
# and restored on --uninstall; an identical one is left alone; one we created from nothing
# (marker .ros2_stack-created-mower.rules) is removed. 99-mower-cameras.rules is ours only.
set -euo pipefail
cd "$(dirname "$0")/.."
D=/etc/udev/rules.d
MARK=$D/.ros2_stack-created-mower.rules

reload() { udevadm control --reload; udevadm trigger; }   # trigger drops symlinks no rule claims

if [[ ${1:-} == --uninstall ]]; then
  rm -f "$D/99-mower-cameras.rules"
  if [[ -f $D/mower.rules.vendor ]]; then
    mv -f "$D/mower.rules.vendor" "$D/mower.rules"; echo "restored vendor $D/mower.rules"
  elif [[ -f $MARK ]]; then
    rm -f "$D/mower.rules"; echo "removed $D/mower.rules (we created it)"
  else
    echo "kept $D/mower.rules (vendor-shipped, identical to ours)"
  fi
  rm -f "$MARK"
  reload
  echo "uninstalled udev rules"
  exit 0
fi

if [[ ! -f $D/mower.rules ]]; then
  touch "$MARK"
elif ! cmp -s "$D/mower.rules" config/udev/mower.rules && [[ ! -f $D/mower.rules.vendor ]]; then
  cp -a "$D/mower.rules" "$D/mower.rules.vendor"; echo "saved vendor rules as $D/mower.rules.vendor"
fi
install -m 0644 config/udev/mower.rules "$D/"
if [ -f scripts/99-mower-cameras.rules ]; then
  install -m 0644 scripts/99-mower-cameras.rules "$D/"
fi
reload
ls -l /dev/serial_mower /dev/serial_imu /dev/serial_rtk 2>/dev/null || true
