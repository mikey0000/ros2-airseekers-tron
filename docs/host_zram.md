# Compressed RAM swap (zram) on the mower host

The owner approved this on 2026-10-22. It is option Q1 in `docs/analysis/2026-10-22_memory_footprint.md`, which is on `ws/mem-diet`.

## Why
- The RK3588 has 3.9 GB of RAM and no swap. Only about 300 MB was available at idle, and about 290 MB while mowing.
- With no swap, the kernel can only evict executable and page-cache pages. Under memory pressure that becomes a refault storm, and the mower froze on 2026-10-22: ssh timed out and it needed a power cycle.

## What it does
- zram is swap held in RAM. Cold anonymous pages are compressed with zstd, at roughly 3:1, and stay in memory.
- **Nothing is written to the eMMC,** because no writeback device is configured. So there is no flash wear, unlike swap on the eMMC.
- **Size:** `/dev/zram0` takes up to 1.5 GiB of uncompressed pages, which is about 0.5 GiB of real RAM when full.
- **Swap priority:** 100.
- **Sysctls:**
  - `vm.swappiness 150` prefers swapping cold heap pages over dropping code pages;
  - `vm.page-cluster 0` disables readahead, which is pointless for RAM-backed swap.

The kernel already supports it: `/sys/block/zram0` exists, and `comp_algorithm` lists zstd.

## Install
It is part of the standard host install (`docs/deployment.md`, "Host services"): `sudo /userdata/ros2_stack/scripts/install_host_services.sh` installs it with the other host pieces, and `uninstall` removes it.

To install zram on its own: `sudo /userdata/ros2_stack/scripts/install_zram.sh`; undo it with `--uninstall`.

The equivalent manual steps, after syncing so that the files are in `/userdata/ros2_stack/docker/host/`:
```
sudo install -m 644 /userdata/ros2_stack/docker/host/zram-swap.service /etc/systemd/system/
sudo install -m 644 /userdata/ros2_stack/docker/host/92-zram-vm.conf /etc/sysctl.d/
sudo systemctl daemon-reload
sudo systemctl enable --now zram-swap
sudo sysctl -p /etc/sysctl.d/92-zram-vm.conf
swapon --show     # expect /dev/zram0, 1.5G, prio 100
zramctl           # algorithm zstd
```
It survives reboots: the service is enabled.

## Check after a mow
```
zramctl                     # DATA vs COMPR: the compression ratio
grep -E 'MemAvailable|SwapFree' /proc/meminfo
```

## Revert
```
sudo systemctl disable --now zram-swap
sudo rm /etc/systemd/system/zram-swap.service /etc/sysctl.d/92-zram-vm.conf
sudo sysctl -w vm.swappiness=60 vm.page-cluster=3
```
