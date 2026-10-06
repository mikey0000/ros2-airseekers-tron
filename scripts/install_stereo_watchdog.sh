#!/usr/bin/env bash
# Install + start the front-stereo watchdog unit ON THE MOWER HOST (needs root).
set -euo pipefail
cd "$(dirname "$0")/.."
install -m 0644 config/systemd/mower-stereo-watchdog.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now mower-stereo-watchdog.service
systemctl --no-pager status mower-stereo-watchdog.service | head -5
