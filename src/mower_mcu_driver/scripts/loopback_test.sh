#!/usr/bin/env bash
# Offline loopback test for mower_mcu_driver: socat pty pair + synthetic MCU + the real node.
# No mower hardware and no root needed (unless you use --dev).
#
# Run it inside the Jazzy container:
#
#   ./scripts/build.sh                                  # from ros2_stack/
#   ./scripts/run_stack.sh                              # shell in the container
#   src/mower_mcu_driver/scripts/loopback_test.sh       # this script (Ctrl-C stops everything)
#
# Then, from a second shell (./scripts/run_stack.sh again):
#
#   ros2 topic hz /battery
#   ros2 topic echo /odom
#   ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/TwistStamped \
#       "{twist: {linear: {x: 0.3}, angular: {z: 0.2}}}"
#   ros2 topic echo /mower_sensor_info     # once mower_interfaces exists
#
# Options:
#   --imu   fake MCU also emits MODULE_IMU (9) frames so /imu is exercised
#           (the real firmware does not appear to send them)
#   --dev   use the vendor-style /dev/serial_mower + /dev/serial_mower_fake links instead of
#           /tmp - needs sudo and must not run while mower-base.service is up
set -euo pipefail

WITH_IMU=0
USE_DEV=0
for arg in "$@"; do
  case "$arg" in
    --imu) WITH_IMU=1 ;;
    --dev) USE_DEV=1 ;;
    -h|--help) sed -n '2,26p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown option: $arg (try --help)" >&2; exit 1 ;;
  esac
done

command -v socat >/dev/null || { echo "socat is required (apt install socat)" >&2; exit 1; }
command -v ros2  >/dev/null || { echo "ros2 not found - run inside the Jazzy container" >&2; exit 1; }

TMP_DIR=""
if [ "$USE_DEV" = 1 ]; then
  SUDO="sudo"
  HOST_PTY=/dev/serial_mower
  FAKE_PTY=/dev/serial_mower_fake
else
  SUDO=""
  TMP_DIR="$(mktemp -d)"
  HOST_PTY="$TMP_DIR/mower_serial"
  FAKE_PTY="$TMP_DIR/mower_serial_fake"
fi

SOCAT_PID=""
FAKE_PID=""
cleanup() {
  set +e
  [ -n "$FAKE_PID" ] && kill "$FAKE_PID" 2>/dev/null
  [ -n "$SOCAT_PID" ] && $SUDO kill "$SOCAT_PID" 2>/dev/null
  [ -n "$TMP_DIR" ] && rm -rf "$TMP_DIR"
}
trap cleanup EXIT INT TERM

echo "loopback: creating pty pair $HOST_PTY <-> $FAKE_PTY"
$SUDO socat -d -d "PTY,link=$HOST_PTY,raw,echo=0,mode=666" \
                "PTY,link=$FAKE_PTY,raw,echo=0,mode=666" 2>/dev/null &
SOCAT_PID=$!

for _ in $(seq 1 50); do
  [ -e "$HOST_PTY" ] && [ -e "$FAKE_PTY" ] && break
  sleep 0.1
done
[ -e "$HOST_PTY" ] && [ -e "$FAKE_PTY" ] || { echo "socat failed to create the pty pair" >&2; exit 1; }

IMU_ARGS=()
[ "$WITH_IMU" = 1 ] && IMU_ARGS=(--imu)

echo "loopback: starting fake MCU on $FAKE_PTY"
ros2 run mower_mcu_driver fake_mcu --port "$FAKE_PTY" "${IMU_ARGS[@]+"${IMU_ARGS[@]}"}" &
FAKE_PID=$!
sleep 0.5

echo "loopback: starting mower_mcu_driver on $HOST_PTY (Ctrl-C to stop)"
echo "  try: ros2 topic hz /battery  |  ros2 topic echo /odom"
ros2 run mower_mcu_driver mcu_node --ros-args -p "port:=$HOST_PTY" -p "heartbeat_period:=0.1"
