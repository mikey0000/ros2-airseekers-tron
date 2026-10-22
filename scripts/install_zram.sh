#!/usr/bin/env bash
# Install compressed RAM swap ON THE MOWER HOST (needs root). docs/host_zram.md.
#
#   sudo bash /userdata/ros2_stack/scripts/install_zram.sh              # install / refresh
#   sudo bash /userdata/ros2_stack/scripts/install_zram.sh --uninstall  # swapoff + remove
#
# Installs:
#   /etc/systemd/system/zram-swap.service   (docker/host/zram-swap.service: /dev/zram0, 1.5 GiB, zstd)
#   /etc/sysctl.d/92-zram-vm.conf           (docker/host/92-zram-vm.conf: swappiness 150, page-cluster 0)
# zram lives in RAM: nothing is written to the eMMC (no writeback device is configured).
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ ${1:-} == --uninstall ]]; then
    systemctl disable --now zram-swap 2>/dev/null || true
    rm -f /etc/systemd/system/zram-swap.service /etc/sysctl.d/92-zram-vm.conf
    systemctl daemon-reload
    sysctl -q -w vm.swappiness=60 vm.page-cluster=3   # Ubuntu 20.04 defaults
    echo "uninstalled (zram swap off, swappiness 60, page-cluster 3)"
    exit 0
fi

if [[ ! -e /sys/block/zram0 ]]; then modprobe zram || { echo "kernel has no zram support" >&2; exit 1; }; fi
grep -qw zstd /sys/block/zram0/comp_algorithm || { echo "zram has no zstd on this kernel" >&2; exit 1; }
install -m 0644 docker/host/zram-swap.service /etc/systemd/system/
install -m 0644 docker/host/92-zram-vm.conf /etc/sysctl.d/
systemctl daemon-reload
systemctl enable zram-swap
systemctl restart zram-swap
sysctl -q -p /etc/sysctl.d/92-zram-vm.conf
swapon --show
