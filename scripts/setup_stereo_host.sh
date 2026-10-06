#!/bin/sh
# Host prerequisites of the stereo / VIO path. Run on the mower HOST as root (not in the
# container), once per boot (both settings are runtime-only). Idempotent; reversible.
#
# 1. UDP socket buffer cap (net.core.rmem_max / wmem_max -> 8 MiB). mower_humble runs Fast DDS
#    UDP-only (config/cameras/fastdds_no_shm.xml requests 8 MiB buffers) but the kernel clamped
#    them to 208 KiB: one 300 KB mono8 eye filled a subscriber's buffer and the second eye of
#    the pair, sent right after it, was dropped (measured 2026-10-07: 10 Hz left, 2.9 Hz right,
#    2.9 Hz exact pairs; with 8 MiB: 10.02 Hz pairs, 0 unpaired). Subscribers created before
#    the change keep their small buffer until restarted.
#    Persist: echo 'net.core.rmem_max=8388608' + wmem_max into /etc/sysctl.d/90-ros2-dds.conf.
#
# 2. Metoak stereo IMU (ICM-40608, i2c 8-0068) in IIO mode for mower_cameras/stereo_imu.
#
# Why: /usr/metoak/metoak/mo_init.sh (cam.service) loads the Metoak inv_icm42600 build with
# Repot_m=1, i.e. samples go to a vendor netlink socket and NO IIO device is created
# (the "driver loaded but not attached" symptom). Repot_m=0 = plain IIO:
# icm40608-gyro + icm40608-accel with FIFO-backed /dev/iio:deviceN buffers.
# TimeStamp_m=1 (ktime_get_ts64 = CLOCK_MONOTONIC) is kept: same clock as the V4L2 buffers.
# Unbind/bind alone does NOT work in netlink mode (the netlink socket leaks: probe -1).
#
#   setup_stereo_host.sh          apply both (no-op where already done)
#   setup_stereo_host.sh status
#   setup_stereo_host.sh revert   vendor netlink IMU mode + 208 KiB socket buffers
# Persistent alternative (needs owner approval, it edits a vendor file): in
# /usr/metoak/metoak/mo_init.sh change "Repot_m=1" to "Repot_m=0" on the inv-icm42600 line.
set -eu
KO=/lib/kernel/driver
mode=${1:-iio}
has_iio() { grep -qs 'icm40608-gyro' /sys/bus/iio/devices/iio:device*/name; }
reload() {
  rmmod inv_icm42600_i2c 2>/dev/null || true
  rmmod inv_icm42600 2>/dev/null || true
  insmod $KO/inv-icm42600.ko TimeStamp_m=1 Repot_m=$1
  insmod $KO/inv-icm42600-i2c.ko
  sleep 1
}
BUF=8388608
case "$mode" in
  iio) for k in rmem_max wmem_max; do
         [ "$(sysctl -n net.core.$k)" -ge $BUF ] || sysctl -qw net.core.$k=$BUF; done
       echo "net.core.rmem_max=$(sysctl -n net.core.rmem_max) wmem_max=$(sysctl -n net.core.wmem_max)";;
esac
case "$mode" in
  status) echo "net.core.rmem_max=$(sysctl -n net.core.rmem_max) wmem_max=$(sysctl -n net.core.wmem_max)"
          has_iio && echo "IIO mode: $(grep -l icm40608 /sys/bus/iio/devices/iio:device*/name | xargs -n1 dirname | xargs -n1 basename | tr '\n' ' ')" || echo "not in IIO mode";;
  iio)    if has_iio; then echo "already in IIO mode"; else reload 0; has_iio && echo "IIO mode on" || { echo "FAILED: no IIO device"; dmesg | tail -5; exit 1; }; fi;;
  revert) reload 1; sysctl -qw net.core.rmem_max=212992 net.core.wmem_max=212992
          echo "vendor netlink mode, 208 KiB socket buffers";;
  *) echo "usage: $0 [iio|status|revert]"; exit 2;;
esac
