#!/usr/bin/env python3
"""Synthetic Airseekers MCU for offline testing (no mower hardware needed).

Pairs with ``scripts/loopback_test.sh``: socat creates two connected pty endpoints, the
driver node opens one (``port`` parameter) and this tool opens the other, so everything the
node writes can be read back as if it came from the real MCU.

It speaks the same framing/checksum/escape code as the driver (imported from
:mod:`mower_mcu_driver.mcu_node`, single source of truth) and emits the sub-packets the real
firmware sends:

  * ``SpeedData`` (4) + ``SensorInfo`` (10) at ~60 Hz  -- the measured speed echoes whatever
    the host commanded, so ``/odom`` moves when you publish ``/cmd_vel``
  * ``BatteryInfo`` (5) + ``VersionInfo`` (12) + BMS version (24) + ``MotorsInfo`` (13)
    batched into one frame once per second, exactly like the real captures
  * ``Log`` (19) once at start-up
  * ``ImuData`` (9) at 10 Hz **only** with ``--imu``: the real firmware does not appear to
    send module 9 at all (the IMU is a separate WIT JY61P on ``/dev/serial_imu``), and the
    driver is expected to publish nothing on ``/imu`` without it.

Usage (normally via ``scripts/loopback_test.sh``)::

    python3 -m mower_mcu_driver.fake_mcu --port /tmp/mower_serial_fake --imu
"""

import argparse
import math
import os
import select
import struct
import sys
import time

# Import the driver's framing/struct code (single source of truth). Works both from an
# installed overlay and straight from the source tree.
try:
    from mower_mcu_driver.mcu_node import (BATTERY_FMT, BMS_FMT, FrameParser, IMU_FMT,
                                           MOD_BATTERY, MOD_BMS_VERSION, MOD_IMU, MOD_LOG,
                                           MOD_MOTORS, MOD_SENSOR, MOD_SPEED, MOD_VERSION,
                                           MOTOR_FMT, SENSOR_FMT, TYPE_HEARTBEAT,
                                           TYPE_MOWER_ROS, TYPE_ROS_MOWER, VERSION_FMT,
                                           build_frame, speed_payload)
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from mower_mcu_driver.mcu_node import (BATTERY_FMT, BMS_FMT, FrameParser, IMU_FMT,
                                           MOD_BATTERY, MOD_BMS_VERSION, MOD_IMU, MOD_LOG,
                                           MOD_MOTORS, MOD_SENSOR, MOD_SPEED, MOD_VERSION,
                                           MOTOR_FMT, SENSOR_FMT, TYPE_HEARTBEAT,
                                           TYPE_MOWER_ROS, TYPE_ROS_MOWER, VERSION_FMT,
                                           build_frame, speed_payload)

# Static-ish telemetry, values taken from live captures in 10_ros_live_snapshot/.
BATTERY_RAW = (248, 5, 99, 0, 15, 0)      # voltage 0.1V, current(?), %, dock_ok, degC, error
VERSION_RAW = (0, 6, 36, 0, 6, 34, 0, 9, 50)  # cutter, chassis, rtk
BMS_RAW = (1, 18, 0)
MOTORS_RAW = (                 # captured on the Tron at rest, 2026-10-06 (battery 19.7 V)
    (0, 0, 1969, 30, 0),       # cutter: 19.69 V, 30 C, idle
    (0, 3582, 12798, 40, 1),   # left drive: holding (running), voltage ~655 counts/V
    (0, -617, 12798, 34, 1),   # right drive
    (19695, -5536, 0, 0, 0),   # height (not populated on this hardware: garbage)
)
SENSOR_RAW = (0, 0, 0, 0, 0, 0, 1, 1, 0, 0)   # cutter_size=1 (small), press_module=1


class Port:
    """Minimal bidirectional port: pyserial when available, plain pty fds otherwise."""

    def __init__(self, path):
        self._serial = None
        try:
            import serial
            self._serial = serial.Serial(path, 115200, timeout=0.02)
        except ImportError:
            self._fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except Exception as exc:
            raise SystemExit('fake_mcu: cannot open %s: %s' % (path, exc))

    def read(self, size=4096):
        if self._serial is not None:
            return self._serial.read(size) or b''
        ready, _, _ = select.select([self._fd], [], [], 0.0)
        if not ready:
            return b''
        try:
            return os.read(self._fd, size)
        except BlockingIOError:
            return b''

    def write(self, data):
        if self._serial is not None:
            self._serial.write(data)
            return
        view = memoryview(data)
        while view:
            _, ready, _ = select.select([], [self._fd], [], 1.0)
            if not ready:
                return
            try:
                written = os.write(self._fd, view)
            except BlockingIOError:
                continue
            view = view[written:]

    def close(self):
        if self._serial is not None:
            self._serial.close()
        else:
            os.close(self._fd)


def motors_payload():
    return b''.join(struct.pack(MOTOR_FMT, *motor) for motor in MOTORS_RAW)


def imu_payload(t):
    """Stationary-ish IMU: ~1 g on Z, heading wobbling slowly so /odom yaw moves."""
    yaw_raw = int(2000.0 * math.sin(t * 0.2))          # +-11 deg
    accz_raw = int(32768 / 16)                          # 1 g
    return struct.pack(IMU_FMT, 0, 0, yaw_raw, 0, 0, accz_raw, 0, 0, 0)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Synthetic Airseekers MCU (loopback test)')
    parser.add_argument('--port', default='/tmp/mower_serial_fake',
                        help='pty end owned by the fake MCU (default: %(default)s)')
    parser.add_argument('--imu', action='store_true',
                        help='also emit MODULE_IMU (9) frames; off by default because the '
                             'real firmware does not send them')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='stop after N seconds (0 = run until interrupted); used by the '
                             'automated loopback test')
    args = parser.parse_args(argv)

    port = Port(args.port)
    parser_rx = FrameParser()
    start = time.monotonic()
    cmd_linear = cmd_angular = 0.0
    heartbeats = host_frames = 0
    next_fast = next_slow = next_report = next_imu = time.monotonic()

    port.write(build_frame([(TYPE_MOWER_ROS, MOD_LOG, b'fake MCU online')]))
    print('fake_mcu: listening on %s (--imu=%s)' % (args.port, args.imu))

    try:
        while True:
            now = time.monotonic()

            for type_id, mod_id, payload in parser_rx.feed(port.read()):
                host_frames += 1
                if type_id == TYPE_HEARTBEAT:
                    heartbeats += 1
                elif type_id == TYPE_ROS_MOWER and mod_id == MOD_SPEED \
                        and len(payload) == struct.calcsize('<ff'):
                    cmd_linear, cmd_angular = struct.unpack('<ff', payload)

            if now >= next_fast:
                # 60 Hz: measured speed (echo of the command) + sensor flags
                next_fast = now + 1.0 / 60.0
                port.write(build_frame([
                    (TYPE_MOWER_ROS, MOD_SPEED, speed_payload(cmd_linear, cmd_angular)),
                    (TYPE_MOWER_ROS, MOD_SENSOR, struct.pack(SENSOR_FMT, *SENSOR_RAW)),
                ]))

            if now >= next_slow:
                # 1 Hz: the batched telemetry frame the real MCU sends
                next_slow = now + 1.0
                port.write(build_frame([
                    (TYPE_MOWER_ROS, MOD_BATTERY, struct.pack(BATTERY_FMT, *BATTERY_RAW)),
                    (TYPE_MOWER_ROS, MOD_VERSION, struct.pack(VERSION_FMT, *VERSION_RAW)),
                    (TYPE_MOWER_ROS, MOD_BMS_VERSION, struct.pack(BMS_FMT, *BMS_RAW)),
                    (TYPE_MOWER_ROS, MOD_MOTORS, motors_payload()),
                ]))

            if args.imu and now >= next_imu:
                # 10 Hz: only with --imu (the real firmware does not send MODULE_IMU)
                next_imu = now + 0.1
                port.write(build_frame([(TYPE_MOWER_ROS, MOD_IMU, imu_payload(now - start))]))

            if now >= next_report:
                next_report = now + 1.0
                print('fake_mcu: host_frames=%d heartbeats=%d cmd=(%.2f m/s, %.2f rad/s)'
                      % (host_frames, heartbeats, cmd_linear, cmd_angular))
            if args.duration > 0.0 and now - start > args.duration:
                break
            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        port.write(build_frame([(TYPE_ROS_MOWER, MOD_SPEED, speed_payload(0.0, 0.0))]))
        port.close()
        print('fake_mcu: stopped')
    return 0


if __name__ == '__main__':
    sys.exit(main())
