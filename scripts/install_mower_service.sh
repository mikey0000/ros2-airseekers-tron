#!/usr/bin/env bash
# Install + enable the example systemd unit for the mower ROS 2 stack.
set -euo pipefail
cd "$(dirname "$0")/.."
install -m 0644 config/systemd/mower-ros2.service.example /etc/systemd/system/mower-ros2.service
systemctl daemon-reload
systemctl enable mower-ros2.service
echo "Installed mower-ros2.service (enabled). Start with: systemctl start mower-ros2"
