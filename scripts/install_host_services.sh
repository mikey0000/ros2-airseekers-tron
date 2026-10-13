#!/usr/bin/env bash
# Install every one-off HOST piece the stack needs, in one go. Run ON THE MOWER HOST as root
# from the stack checkout (docs/deployment.md "Host services"):
#
#   sudo /userdata/ros2_stack/scripts/install_host_services.sh            # install / refresh all
#   sudo /userdata/ros2_stack/scripts/install_host_services.sh status     # just show the units
#   sudo /userdata/ros2_stack/scripts/install_host_services.sh --boot-unit  # also mower-ros2.service
#
# Each step is its own idempotent installer, so re-running after a stack sync refreshes the
# unit files and scripts in place:
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
# Nothing here touches the running container or commands motion.
set -euo pipefail
cd "$(dirname "$0")/.."
UNITS=(mower-stereo-host mower-stereo-watchdog mower-poweroff.path mower-ros2)

status() {
    echo "== host units"
    for u in "${UNITS[@]}"; do
        printf '  %-24s enabled=%-9s active=%s\n' "$u" \
            "$(systemctl is-enabled "$u" 2>/dev/null || echo n/a)" \
            "$(systemctl is-active "$u" 2>/dev/null || echo n/a)"
    done
    echo "== stereo host"; sh scripts/setup_stereo_host.sh status 2>/dev/null || true
    echo "== udev"; ls -l /dev/serial_mower /dev/serial_imu /dev/serial_rtk 2>/dev/null || echo "  serial symlinks missing"
}

if [[ ${1:-} == status ]]; then status; exit 0; fi
[[ $EUID -eq 0 ]] || { echo "run as root (sudo)"; exit 1; }

echo "### 1/4 udev rules";            bash scripts/install_udev.sh
echo "### 2/4 stereo host setup";     bash scripts/install_stereo_host_setup.sh
echo "### 3/4 stereo watchdog";       bash scripts/install_stereo_watchdog.sh
echo "### 4/4 power button (DRY_RUN)"; bash scripts/install_power_button.sh
if [[ ${1:-} == --boot-unit ]]; then
    echo "### boot unit";             bash scripts/install_mower_service.sh
fi
echo
status
