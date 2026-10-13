#!/usr/bin/env bash
# Install + start the front-stereo watchdog unit ON THE MOWER HOST (needs root).
#   sudo scripts/install_stereo_watchdog.sh               install / refresh
#   sudo scripts/install_stereo_watchdog.sh --uninstall   stop, disable and remove the unit
set -euo pipefail
cd "$(dirname "$0")/.."
UNIT=mower-stereo-watchdog.service

if [[ ${1:-} == --uninstall ]]; then
    systemctl disable --now "$UNIT" 2>/dev/null || true
    rm -f "/etc/systemd/system/$UNIT"
    systemctl daemon-reload
    echo "uninstalled $UNIT"
    exit 0
fi

install -m 0644 "config/systemd/$UNIT" /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now "$UNIT"
systemctl --no-pager status "$UNIT" | head -5
