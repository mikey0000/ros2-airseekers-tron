#!/usr/bin/env bash
# Install the power-button shutdown helper ON THE MOWER HOST (needs root). docs/buttons.md "Power off".
#
#   sudo bash /userdata/ros2_stack/scripts/install_power_button.sh            # install (DRY_RUN=1)
#   sudo bash /userdata/ros2_stack/scripts/install_power_button.sh --uninstall
#
# Installs:
#   /usr/local/sbin/mower-poweroff-host                 (src/base_keys/base_keys/poweroff_host.py)
#   /etc/systemd/system/mower-poweroff.{path,service}   (config/systemd/)
#   /etc/default/mower-poweroff                         (kept if it already exists)
#   /usr/lib/systemd/system-shutdown/mower-mcu-poweroff (docker/host/power/)
#   /usr/bin/sys_close  (vendor kernel-module hook; original saved as /usr/bin/sys_close.vendor)
#   /userdata/ros2/power/                               (request directory, bind-mounted in mower_humble)
# Nothing here powers anything off: the default config is DRY_RUN=1, MCU_POWER_CUT=0.
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ ${1:-} == --uninstall ]]; then
    systemctl disable --now mower-poweroff.path 2>/dev/null || true
    rm -f /etc/systemd/system/mower-poweroff.path /etc/systemd/system/mower-poweroff.service \
          /usr/lib/systemd/system-shutdown/mower-mcu-poweroff /usr/local/sbin/mower-poweroff-host
    if [[ -f /usr/bin/sys_close.vendor ]]; then mv -f /usr/bin/sys_close.vendor /usr/bin/sys_close; fi
    rm -f /run/mower-poweroff/mcu_cut
    systemctl daemon-reload
    echo "uninstalled (kept /etc/default/mower-poweroff and /userdata/ros2/power)"
    exit 0
fi

install -d -m 0755 /userdata/ros2/power
rm -f /userdata/ros2/power/shutdown_request          # never start with a pending request
install -m 0755 src/base_keys/base_keys/poweroff_host.py /usr/local/sbin/mower-poweroff-host
install -m 0644 config/systemd/mower-poweroff.path config/systemd/mower-poweroff.service \
        /etc/systemd/system/
if [[ ! -f /etc/default/mower-poweroff ]]; then
    install -m 0644 docker/host/power/mower-poweroff.default /etc/default/mower-poweroff
fi
install -d /usr/lib/systemd/system-shutdown
install -m 0755 docker/host/power/mower-mcu-poweroff /usr/lib/systemd/system-shutdown/
if [[ ! -f /usr/bin/sys_close.vendor ]]; then cp -a /usr/bin/sys_close /usr/bin/sys_close.vendor; fi
install -m 0755 docker/host/power/sys_close /usr/bin/sys_close
systemctl daemon-reload
systemctl enable --now mower-poweroff.path
systemctl --no-pager status mower-poweroff.path | head -4
echo "config: /etc/default/mower-poweroff"; grep -E '^[A-Z_]+=' /etc/default/mower-poweroff
