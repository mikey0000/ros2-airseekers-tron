#!/usr/bin/env bash
# Install + enable the example systemd unit for the mower ROS 2 stack.
#   sudo scripts/install_mower_service.sh               install / refresh (enabled, not started)
#   sudo scripts/install_mower_service.sh --uninstall   disable and remove the unit. The unit is
#                                                       only disabled, not stopped: its ExecStop
#                                                       is `docker compose down`, which would take
#                                                       the running stack with it.
set -euo pipefail
cd "$(dirname "$0")/.."
UNIT=mower-ros2.service

if [[ ${1:-} == --uninstall ]]; then
    systemctl disable "$UNIT" 2>/dev/null || true
    rm -f "/etc/systemd/system/$UNIT"
    systemctl daemon-reload
    echo "uninstalled $UNIT (containers left as they are; docker's restart policy still applies)"
    exit 0
fi

install -m 0644 config/systemd/mower-ros2.service.example "/etc/systemd/system/$UNIT"
systemctl daemon-reload
systemctl enable "$UNIT"
echo "Installed $UNIT (enabled). Start with: systemctl start mower-ros2"
