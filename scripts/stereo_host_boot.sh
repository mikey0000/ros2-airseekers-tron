#!/usr/bin/env bash
# Boot-time wrapper for scripts/setup_stereo_host.sh, run ON THE MOWER HOST as root by
# mower-stereo-host.service (config/systemd/mower-stereo-host.service, installed by
# scripts/install_stereo_host_setup.sh).
#
# Both settings the setup script applies are runtime-only and are undone by every reboot:
#   * net.core.rmem_max / wmem_max -> 8 MiB (Fast DDS UDP buffers; subscribers created
#     before the change keep the small buffer, hence Before=docker.service in the unit);
#   * the Metoak ICM-40608 stereo IMU re-loaded in IIO mode (cam.service's mo_init.sh loads
#     it in the vendor netlink mode, which creates no IIO device: 2026-10-13 the heading
#     aligner ran a whole mow on the WIT gyro because of this after a reboot).
#
# cam.service runs mo_init.sh asynchronously, so wait until the vendor driver is loaded
# before swapping it (reloading before mo_init.sh touched it would be undone by mo_init.sh).
# Idempotent: "already in IIO mode" is success.
set -u
SETUP=${SETUP:-/userdata/ros2_stack/scripts/setup_stereo_host.sh}
WAIT_S=${WAIT_S:-90}
log() { echo "[stereo-host] $*"; }

[[ -f "$SETUP" ]] || { log "$SETUP missing; nothing to do"; exit 0; }

for ((i = 0; i < WAIT_S; i++)); do
    grep -qs '^inv_icm42600 ' /proc/modules && break
    sleep 1
done
grep -qs '^inv_icm42600 ' /proc/modules || log "vendor IMU driver not loaded after ${WAIT_S}s; applying anyway"

# The module appearing is not the end of mo_init.sh: cam.service is Type=simple, so the
# unit's After=cam.service orders nothing, and mo_init.sh keeps configuring the camera and
# the vendor IMU for a few more seconds. Field 2026-10-28: this script reloaded at 10:25:54,
# mo_init.sh exited at 10:25:57, the reload failed in the same second and the whole mow ran
# on the WIT gyro. Wait for cam.service to finish, then retry the reload a few times.
for ((; i < WAIT_S; i++)); do
    case "$(systemctl is-active cam.service 2>/dev/null)" in
        active|activating|reloading) sleep 1 ;;
        *) break ;;
    esac
done
sleep "${SETTLE_S:-3}"

TRIES=${TRIES:-5}
for ((n = 1; n <= TRIES; n++)); do
    out=$(sh "$SETUP" iio 2>&1); rc=$?
    log "attempt $n/$TRIES: $out"
    (( rc == 0 )) && break
    sleep 5
done
if (( rc != 0 )); then
    log "IIO mode FAILED after $TRIES attempts (rc=$rc); the stack's stereo_imu node will keep retrying and the heading aligner stays on the WIT gyro"
    exit $rc
fi
sh "$SETUP" status 2>&1 | sed 's/^/[stereo-host] /'
