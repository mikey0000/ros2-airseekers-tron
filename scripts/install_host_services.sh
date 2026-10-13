#!/usr/bin/env bash
# Install, inspect or REMOVE every one-off HOST piece the stack needs, in one go. Run ON THE
# MOWER HOST as root from the stack checkout (docs/deployment.md "Host services"):
#
#   sudo /userdata/ros2_stack/scripts/install_host_services.sh              # install / refresh all
#   sudo /userdata/ros2_stack/scripts/install_host_services.sh --boot-unit  # also mower-ros2.service
#   sudo /userdata/ros2_stack/scripts/install_host_services.sh status       # just show the units
#   sudo /userdata/ros2_stack/scripts/install_host_services.sh manifest     # every host path we own,
#                                                                           # present / absent
#   sudo /userdata/ros2_stack/scripts/install_host_services.sh uninstall    # remove ALL of it and put
#                                                                           # the host back to vendor
#                                                                           # state (see below)
#
# Each step is its own idempotent installer with a matching --uninstall, so re-running after a
# stack sync refreshes the unit files and scripts in place, and `uninstall` undoes exactly
# what `install` did:
#   1. udev rules            scripts/install_udev.sh              /dev/serial_*, /dev/*_camera symlinks
#   2. stereo host setup     scripts/install_stereo_host_setup.sh mower-stereo-host.service: IIO-mode
#                                                                 stereo IMU + 8 MiB DDS socket buffers
#                                                                 at every boot (+ /etc/sysctl.d)
#   3. stereo watchdog       scripts/install_stereo_watchdog.sh   mower-stereo-watchdog.service: re-init
#                                                                 the front stereo when it stops streaming
#   4. power button          scripts/install_power_button.sh      mower-poweroff.path/.service (DRY_RUN=1
#                                                                 by default, see docs/buttons.md)
#   5. (opt-in) boot unit    scripts/install_mower_service.sh     mower-ros2.service: compose up at boot
#                                                                 (docker's restart policy already does
#                                                                 this for mower_jazzy; only needed when
#                                                                 compose is not left running)
#
# `uninstall` runs the five --uninstall steps in reverse order: units stopped + disabled + removed,
# sysctl file removed, our udev rules removed / the vendor mower.rules restored, vendor /usr/bin/sys_close restored,
# power-button config + request dir removed, and the live IMU driver reloaded in the vendor
# netlink mode with the vendor 208 KiB socket buffers (so the host matches a fresh boot without
# us, immediately, not only after a reboot). It never touches the vendor units (cam.service, ...),
# the docker install, /userdata/ros2_stack or /userdata/ros2 beyond /userdata/ros2/power, and it
# does not stop the running container. Expect the stack to lose the stereo IMU after it.
# Nothing here commands motion.
set -euo pipefail
cd "$(dirname "$0")/.."
UNITS=(mower-stereo-host mower-stereo-watchdog mower-poweroff.path mower-ros2)
# Every host path an installer creates or replaces. Keep in sync with the install_*.sh scripts.
# (/etc/udev/rules.d/mower.rules itself is vendor-shipped and only refreshed, see install_udev.sh)
MANIFEST=(
    /etc/udev/rules.d/99-mower-cameras.rules
    /etc/udev/rules.d/mower.rules.vendor
    /etc/udev/rules.d/.ros2_stack-created-mower.rules
    /etc/systemd/system/mower-stereo-host.service
    /etc/sysctl.d/90-ros2-dds.conf
    /etc/systemd/system/mower-stereo-watchdog.service
    /etc/systemd/system/mower-poweroff.path
    /etc/systemd/system/mower-poweroff.service
    /etc/default/mower-poweroff
    /usr/local/sbin/mower-poweroff-host
    /usr/lib/systemd/system-shutdown/mower-mcu-poweroff
    /usr/bin/sys_close.vendor
    /userdata/ros2/power
    /etc/systemd/system/mower-ros2.service
)

status() {
    echo "== host units"
    local en ac
    for u in "${UNITS[@]}"; do
        en=$(systemctl is-enabled "$u" 2>/dev/null || true)
        ac=$(systemctl is-active "$u" 2>/dev/null || true)
        printf '  %-24s enabled=%-10s active=%s\n' "$u" "${en:-not-installed}" "${ac:-n/a}"
    done
    echo "== stereo host"; sh scripts/setup_stereo_host.sh status 2>/dev/null || true
    echo "== udev"; ls -l /dev/serial_mower /dev/serial_imu /dev/serial_rtk 2>/dev/null || echo "  serial symlinks missing"
}

manifest() {
    echo "== host paths owned by the stack (present = installed)"
    local n=0
    for p in "${MANIFEST[@]}"; do
        if [[ -e $p ]]; then printf '  present  %s\n' "$p"; n=$((n + 1)); else printf '  absent   %s\n' "$p"; fi
    done
    if [[ -f /etc/udev/rules.d/.ros2_stack-created-mower.rules ]]; then
        echo "  /etc/udev/rules.d/mower.rules was created by us (removed on uninstall)"
    elif [[ -f /etc/udev/rules.d/mower.rules.vendor ]]; then
        echo "  /etc/udev/rules.d/mower.rules is OURS (vendor copy saved as mower.rules.vendor)"
    else
        echo "  /etc/udev/rules.d/mower.rules is vendor-shipped (kept on uninstall)"
    fi
    if [[ -f /usr/bin/sys_close.vendor ]]; then
        echo "  /usr/bin/sys_close is OUR forwarder (vendor copy saved as /usr/bin/sys_close.vendor)"
    else
        echo "  /usr/bin/sys_close is the vendor original"
    fi
    echo "  $n/${#MANIFEST[@]} present"
}

case ${1:-} in
    status)   status; exit 0;;
    manifest) manifest; exit 0;;
esac
[[ $EUID -eq 0 ]] || { echo "run as root (sudo)"; exit 1; }

if [[ ${1:-} == uninstall ]]; then
    echo "### boot unit";              bash scripts/install_mower_service.sh --uninstall
    echo "### power button";           bash scripts/install_power_button.sh --uninstall --purge
    echo "### stereo watchdog";        bash scripts/install_stereo_watchdog.sh --uninstall
    echo "### stereo host setup";      bash scripts/install_stereo_host_setup.sh --uninstall
    echo "### udev rules";             bash scripts/install_udev.sh --uninstall
    echo
    manifest
    echo "Host back to vendor state. Reinstall: $0"
    exit 0
fi

echo "### 1/4 udev rules";            bash scripts/install_udev.sh
echo "### 2/4 stereo host setup";     bash scripts/install_stereo_host_setup.sh
echo "### 3/4 stereo watchdog";       bash scripts/install_stereo_watchdog.sh
echo "### 4/4 power button (DRY_RUN)"; bash scripts/install_power_button.sh
if [[ ${1:-} == --boot-unit ]]; then
    echo "### boot unit";             bash scripts/install_mower_service.sh
fi
echo
status
