#!/usr/bin/env python3
"""ROS 2 Humble driver node for the Airseekers Tron chassis/cutter MCU serial bus.

The MCU sits behind ``/dev/serial_mower`` (``ttyS9``, RK3588 UART9, 115200 8N1, no flow
control).  The cutter board is the gateway: the chassis board (wheels, bumper, lift, stop)
hangs off it over a second UART, so everything below reaches both boards.

Wire format -- this file is a faithful port of the reference implementation
``ros2_port_handoff/01_mcu_protocol/mcu_frame_decode.py`` and the recovered struct layouts in
``ros2_port_handoff/01_mcu_protocol/protocol_definitions_recovered.h`` (read both):

    0xA5 | total_len | { sub_len | type_id | mod_id | payload[sub_len] }+ | checksum | 0x5A

    * ``total_len``  = number of bytes from the first ``sub_len`` through the last payload byte
    * ``checksum``   = (sum of bytes from ``total_len`` through the last payload byte) & 0xFF
    * one frame may carry several sub-packets back to back
      (battery + bms_version + version + motors arrive together once per second)
    * byte stuffing is applied to everything between SOF and EOF:
          0xA5 -> 0xF5 0x01,  0x5A -> 0xF5 0x02,  0xF5 -> 0xF5 0x03
    * all multi-byte payload fields are LITTLE-ENDIAN (packed structs, Cortex-M / aarch64)

Direction / type ids: host->MCU ``TYPE_ROS_MOWER (0)``, MCU->host ``TYPE_MOWER_ROS (8)``,
keepalive ``TYPE_HEARTBEAT (255)``.  Module ids used here: speed 4, battery 5, imu 9,
sensor 10, version 12, motors 13, log 19, bms_version 24.

Published topics
----------------
``/battery``            sensor_msgs/BatteryState           from ``MODULE_BATTERY`` (5) @ ~1 Hz
``/odom``               nav_msgs/Odometry                  dead reckoned from ``SpeedData`` + IMU yaw
``/imu``                sensor_msgs/Imu                    placeholder, only while the MCU emits
                                                           ``MODULE_IMU`` (9); otherwise nothing is
                                                           published (the real IMU is a WIT JY61P on
                                                           ``/dev/serial_imu``, see wit_imu_driver)
``/mower_sensor_info``  mower_interfaces/MowerSensorInfo   only if the ``mower_interfaces`` package
                                                           has been created, else skipped entirely

Subscribed topics
-----------------
``/cmd_vel``            geometry_msgs/Twist                -> one ``SpeedData`` (MODULE_SPEED, 4)
                                                           per message (event-driven, see below)
``/cmd_vel_stamped``    geometry_msgs/TwistStamped         same path (vendor's second input)

Parameters
----------
``port`` ``/dev/serial_mower``   serial device (loopback test: a socat pty)
``baud`` ``115200``              serial baud rate
``odom_frame`` ``odom`` / ``base_frame`` ``base_link`` / ``imu_frame`` ``imu_link`` / ``frame_id`` ``base_link``
``heartbeat_period`` ``0.1``     **period is UNKNOWN on hardware** -- see TODO below
``speed_cmd_rate`` ``20.0``      Hz of the **debug** SpeedData stream (only with ``speed_stream_enabled``)
``speed_stream_enabled`` ``false`` debug A/B only: stream the last command at ``speed_cmd_rate``
``cmd_vel_timeout`` ``0.5``      s of ``/cmd_vel`` silence after a non-zero command -> stop sequence
``stop_frames`` ``3`` / ``stop_frame_spacing_s`` ``0.1``  zero frames of one stop sequence
``speed_timeout`` ``0.5``        s before measured ``SpeedData`` is considered stale
``odom_rate`` ``50.0``           Hz for ``/odom``
``use_imu_yaw`` ``true``         use MCU ``ImuData`` yaw for heading (falls back to integration)
``linear_scale`` ``1.0`` / ``angular_scale`` ``1.0``   host-side ``/cmd_vel`` -> ``SpeedData`` map
``linear_max`` ``0.3`` / ``angular_max`` ``0.3``       clamp (m/s, rad/s; vendor PID clamp)
``battery_voltage_scale`` ``0.1``  raw 0.1 V units -> volts (set 1.0 to mimic the stock node)
``battery_current_scale`` ``-0.1`` raw 0.1 A, positive = discharging -> ROS amperes (negative
                                 = discharging, REP/BatteryState convention)

Known gaps -- tracked as TODOs, do not treat this driver as safety-complete yet
-------------------------------------------------------------------------------
* **SpeedData TX policy = vendor pass-through + explicit stop** (``docs/wheel_control_semantics.md``
  section 5).  The vendor ``mower_base_node`` sends exactly one SpeedData per received
  ``/cmd_vel`` (no timer, no host PID, no brake, nothing while idle).  ``PidControllerROS`` in
  ``libpid_controller.so`` is NOT a wheel-velocity loop: it is the position-level
  ``rotate``/``moveStraight`` primitive used by the bumper back-off, publishing ``/cmd_vel``.
  We add one rule the vendor got from its producers: after a non-zero command, a zero
  command / ``cmd_vel_timeout`` silence / interlock sends ``stop_frames`` zeros
  ``stop_frame_spacing_s`` apart, then the wire goes silent again.  Measured SpeedData is
  telemetry + odometry only and never feeds back into the TX path.
* **Heartbeat period unknown.** ``Heartbeat`` is a wall-clock stamp and the MCU most likely
  stops the motors when it stops arriving, but neither the period nor the timeout were
  recovered from the binary.  We default to 100 ms (the BLE link uses 100 ms too) purely as a
  starting point -- characterise this on hardware with the wheels off the ground first.
* **E-stop passthrough TODO.** The MCU enforces estop / lift / bumper cut-offs itself; the
  host side surface is the ``/clear_estop`` service (``std_srvs/Empty``) plus whatever frame
  the vendor uses to forward an app/ROS stop.  No host->MCU estop frame has been identified
  in the protocol, so nothing is wired up here yet.  ``std_srvs`` is therefore not a
  dependency of this package -- add it together with the passthrough.
* Odometry is open-loop integration of ``SpeedData`` (no wheel encoder feedback beyond what
  the MCU reports in ``SpeedData``), so it drifts.  ``/reset_odom`` and ``/tf`` broadcasting
  are not implemented yet.

Offline test
------------
``scripts/loopback_test.sh`` builds a socat pty pair and runs a synthetic MCU
(:mod:`mower_mcu_driver.fake_mcu`) against this node -- no hardware, no root, see README.md.
"""

import array
import collections
import fcntl
import math
import os
import select
import struct
import termios
import threading
import time
import traceback
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
try:
    from rclpy.qos import QoSDurabilityPolicy
except ImportError:  # pragma: no cover - ros_stubs
    QoSDurabilityPolicy = None

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool, Int16, String
from std_srvs.srv import Empty, Trigger

from mower_mcu_driver.rain import RainDetector
from mower_mcu_driver.rate_gate import ChangeOrPeriodGate, activity_low_power

try:  # real rclpy only (the offline tests run against ros_stubs)
    from rclpy.serialization import deserialize_message
    from mower_mcu_driver.sub_pump import (SubscriptionPump, flat_serializer, odometry_bytes,
                                           parse_imu)
except ImportError:  # pragma: no cover - stubbed ROS
    deserialize_message = parse_imu = flat_serializer = odometry_bytes = None
    SubscriptionPump = None
try:  # Optional: only exists once ros2_stack/src/mower_interfaces has been created.
    from mower_interfaces.msg import MowerSensorInfo
except ImportError:  # pragma: no cover - depends on workspace build order
    MowerSensorInfo = None
try:
    from mower_interfaces.msg import MowerBaseDevStatus
    from mower_interfaces.srv import ChargingControl, CutterControl
except ImportError:  # pragma: no cover
    MowerBaseDevStatus = ChargingControl = CutterControl = None
try:
    from mower_interfaces.srv import SetCutterHeight
except ImportError:  # pragma: no cover - interfaces built before SetCutterHeight existed
    SetCutterHeight = None


# ---------------------------------------------------------------------------
# Framing (constants and codec mirror mcu_frame_decode.py)
# ---------------------------------------------------------------------------
SOF, EOF, ESC = 0xA5, 0x5A, 0xF5
ESC_MAP = {0x01: SOF, 0x02: EOF, 0x03: ESC}

TYPE_ROS_MOWER = 0      # host -> MCU
TYPE_MOWER_ROS = 8      # MCU -> host
TYPE_HEARTBEAT = 255    # host -> MCU keepalive

MOD_ALL = 0           # MODULE_ALL: addressed to the whole MCU (heartbeat uses this)
MOD_CHARGE = 3        # host->MCU ChargeControl (1 B) via /charging
MOD_SPEED = 4
MOD_BATTERY = 5
MOD_IMU = 9
MOD_SENSOR = 10
MOD_CUTTER = 11       # host->MCU CutterControl (12 B) via /cutter_control
MOD_VERSION = 12
MOD_MOTORS = 13
MOD_CALIB = 18
MOD_LOG = 19
MOD_BMS_VERSION = 24

# struct formats, from protocol_definitions_recovered.h (all little-endian, packed)
SPEED_FMT = '<ff'                 # SpeedData (8):  linear m/s, angular rad/s
MOTOR_CTRL_FMT = '<BBHH'          # MotorControl (6): enable, direction, speed, position
CHARGE_FMT = '<B'                 # ChargeControl (1): enable
SENSOR_CTRL_FMT = '<8B'           # SensorInfoControl (8): per-sensor enable mask (?)
BATTERY_FMT = '<HhBbbI'           # BatteryInfo (11)
SENSOR_FMT = '<10B'               # SensorInfo (10)
IMU_FMT = '<9h'                   # ImuData (18) - pitch, roll, yaw, accx..z, gyrox..z
VERSION_FMT = '<9B'               # VersionInfo (9): cutter/chassis/rtk major,minor,patch
BMS_FMT = '<3B'                   # BMS Version (3)
CALIB_FMT = '<3B'                 # MCCalib (3)
MOTOR_FMT = '<hhhbb'              # MotorInfo (8): speed, current, voltage, temperature, status
HEARTBEAT_FMT = '<8B'             # year, month, day, hour, minute, second, ms_h, ms_l
MOTOR_NAMES = ('cutter', 'left', 'right', 'height')

# MotorInfo units (docs/mcu_protocol_spec.md "MotorInfo units").  Source: vendor ROS 1
# mower_msgs/MotorInfo.msg comments (speed rpm, current 10 mA, voltage 10 mV, temp degC) and
# the vendor mower_base::Motors::infoProcess, which republishes current/voltage as raw/100.
# Those units hold for the CUTTER board only.  The two drive (chassis) boards use other
# scalings, so they are converted here into the same message units:
#   * voltage: raw counts proportional to the battery voltage, ~655 counts/V (two captures:
#     16245 @ 24.7 V battery, 12798 @ 19.7 V battery) -> treated as 1/655.36 V (Q?.16/100).
#     Empirical, +-1 %; the vendor node published these as "161 V" (raw/100, a vendor bug).
#   * current: unknown.  Recovered header says mA; 10 mA would mean 35 A at standstill, so
#     mA is used (left ~3.6 A / right ~-0.6 A while the drive is holding, status RUNNING).
#     UNVERIFIED - needs a capture while driving.
MOTOR_CURRENT_A_PER_COUNT = {'cutter': 0.01, 'left': 0.001, 'right': 0.001, 'height': 0.01}
MOTOR_VOLTAGE_V_PER_COUNT = {'cutter': 0.01, 'left': 1.0 / 655.36, 'right': 1.0 / 655.36,
                             'height': 0.01}
# MotorStatus.status is a SIGNED int8 (vendor mower_msgs/MotorStatus.msg): 0 idle,
# 1 running (drive boards report 1 whenever enabled, also at standstill), 2 locking,
# negative = fault.  The old "1 = over-current" reading came from a mis-commented header.
MOTOR_STATUS_TEXT = {0: 'idle', 1: 'running', 2: 'locking', -1: 'error', -2: 'over-current',
                     -3: 'over-voltage', -4: 'under-voltage', -5: 'over-temperature',
                     -6: 'stalled', -7: 'overload'}

# WIT-Motion JY61P scaling convention used by ImuData (marked (?) in the recovered header:
# angle*32768/180, acc*32768/16 g, gyro*32768/2000 dps).
WIT_RAW_ANGLE_TO_RAD = math.radians(180.0) / 32768.0
WIT_RAW_GYRO_TO_RAD = math.radians(2000.0) / 32768.0
WIT_RAW_ACC_TO_MS2 = 16.0 * 9.80665 / 32768.0


def decode_motors(payload):
    """MODULE_MOTORS payload (32 B) -> {name: dict} in SI units, or None if mis-sized.

    Each dict has ``speed_rpm``, ``current_a``, ``voltage_v``, ``temperature_c``, ``status``
    (signed MotorStatus code) and ``raw`` (the unpacked wire tuple).  ``height`` is not
    populated on this hardware (garbage) but is decoded anyway.
    """
    size = struct.calcsize(MOTOR_FMT)
    if len(payload) != size * len(MOTOR_NAMES):
        return None
    out = {}
    for idx, name in enumerate(MOTOR_NAMES):
        raw = struct.unpack(MOTOR_FMT, payload[idx * size:(idx + 1) * size])
        speed, current, voltage, temperature, status = raw
        out[name] = {
            'speed_rpm': speed,
            'current_a': current * MOTOR_CURRENT_A_PER_COUNT[name],
            'voltage_v': voltage * MOTOR_VOLTAGE_V_PER_COUNT[name],
            'temperature_c': temperature,
            'status': status,
            'raw': raw,
        }
    return out


def _clamp_i16(value):
    return max(-32768, min(32767, int(round(value))))


def motor_msg_fields(si):
    """SI motor dict -> mower_interfaces/MotorInfo field units (int16 10 mA / 10 mV)."""
    return {'speed': si['speed_rpm'], 'current': _clamp_i16(si['current_a'] * 100.0),
            'voltage': _clamp_i16(si['voltage_v'] * 100.0),
            'temperature': si['temperature_c'], 'status': si['status']}


def escape(data):
    """Byte-stuff SOF/EOF/ESC between the frame delimiters (see module docstring)."""
    out = bytearray()
    for x in data:
        if x == SOF:
            out += b'\xF5\x01'
        elif x == EOF:
            out += b'\xF5\x02'
        elif x == ESC:
            out += b'\xF5\x03'
        else:
            out.append(x)
    return bytes(out)


def unescape(data):
    """Inverse of :func:`escape`."""
    out = bytearray()
    i = 0
    while i < len(data):
        if data[i] == ESC and i + 1 < len(data):
            out.append(ESC_MAP.get(data[i + 1], data[i + 1]))
            i += 2
        else:
            out.append(data[i])
            i += 1
    return bytes(out)


def build_frame(subpackets):
    """Encode ``[(type_id, mod_id, payload_bytes), ...]`` into one wire-ready frame."""
    body = bytearray()
    for type_id, mod_id, payload in subpackets:
        if len(payload) > 255:
            raise ValueError('sub-packet payload too long: %d bytes' % len(payload))
        body += bytes([len(payload), type_id, mod_id]) + bytes(payload)
    inner = bytes([len(body)]) + bytes(body)
    checksum = sum(inner) & 0xFF
    return bytes([SOF]) + escape(inner + bytes([checksum])) + bytes([EOF])


def split_subpackets(inner):
    """``inner`` = total_len .. checksum.  Returns ``[(type_id, mod_id, payload), ...]``."""
    body = inner[1:-1]
    out = []
    k = 0
    while k + 3 <= len(body):
        length, type_id, mod_id = body[k], body[k + 1], body[k + 2]
        out.append((type_id, mod_id, body[k + 3:k + 3 + length]))
        k += 3 + length
    return out


class FrameParser:
    """Incremental RX parser: feed raw serial bytes, get validated sub-packets back.

    Mirrors ``split_frames()`` in the reference decoder: resynchronise on ``0xA5``, take the
    next ``0x5A`` as EOF (raw ``0x5A`` never appears inside a frame because it is stuffed),
    unescape, then verify length and checksum before accepting anything.
    """

    MAX_BUFFER = 8192  # never grow without bound if the line is pure garbage

    def __init__(self):
        self._buffer = bytearray()
        self.frames_ok = 0
        self.frames_bad = 0

    def feed(self, data):
        """Append bytes and return the list of ``(type_id, mod_id, payload)`` decoded."""
        self._buffer += data
        if len(self._buffer) > self.MAX_BUFFER:
            # Keep only the tail: a frame is far shorter than this.
            del self._buffer[:-self.MAX_BUFFER]

        out = []
        buf = self._buffer
        i = consumed = 0
        while True:
            sof = buf.find(b'\xA5', i)  # SOF is never stuffed -> always a whole byte
            if sof < 0:
                consumed = len(buf)
                break
            eof = buf.find(b'\x5A', sof + 1)
            if eof < 0:
                break  # incomplete frame, wait for more bytes
            inner = unescape(bytes(buf[sof + 1:eof]))
            if (len(inner) >= 5
                    and inner[0] == len(inner) - 2
                    and (sum(inner[:-1]) & 0xFF) == inner[-1]):
                out.extend(split_subpackets(inner))
                self.frames_ok += 1
                consumed = eof + 1
                i = eof + 1
            else:
                # Bogus SOF (or a truncated frame whose real EOF was eaten): skip one byte
                # and resynchronise.
                self.frames_bad += 1
                consumed = sof + 1
                i = sof + 1
        if consumed:
            del buf[:consumed]
        return out


def heartbeat_payload(now=None):
    """Build the 8-byte ``Heartbeat`` payload (wall-clock stamp, year is year-2000 (?))."""
    now = now or datetime.now()
    ms = now.microsecond // 1000
    return struct.pack(HEARTBEAT_FMT, (now.year - 2000) & 0xFF, now.month, now.day,
                       now.hour, now.minute, now.second, (ms >> 8) & 0xFF, ms & 0xFF)


CUTTER_HEIGHT_MIN_MM = 30
CUTTER_HEIGHT_MAX_MM = 90
CUTTER_HEIGHT_DEFAULT_MM = 90     # vendor's resting (CutterOFF) height


def clamp_height_mm(mm):
    """Clamp a deck height to the vendor range (max(30, min(90, mm)))."""
    return max(CUTTER_HEIGHT_MIN_MM, min(CUTTER_HEIGHT_MAX_MM, int(mm)))


def cutter_payload(cutter_enable, cutter_direction, cutter_speed, height_enable,
                   height_direction, height_position):
    """``CutterControl`` = cutter MotorControl + height MotorControl (12 B).

    MotorControl.speed is 0-100 % (the cutter has also been seen with 1000 = raw); the
    height motor's position is an ABSOLUTE deck height in mm (vendor range 30-90, bench
    verified 2026-10-07). Values are clamped to uint16 here, semantics are the
    MCU's (see docs/mcu_protocol_spec.md section 6).
    """
    def u16(v):
        return max(0, min(0xFFFF, int(v)))
    cutter = struct.pack(MOTOR_CTRL_FMT, 1 if cutter_enable else 0, 1 if cutter_direction else 0,
                         u16(cutter_speed), 0)
    height = struct.pack(MOTOR_CTRL_FMT, 1 if height_enable else 0, 1 if height_direction else 0,
                         0, u16(height_position))
    return cutter + height


def imu_payload(roll, pitch, yaw, acc, gyro):
    """``ImuData`` (18 B, MODULE_IMU, host->MCU): pitch, roll, yaw, accx..z, gyrox..z as int16 in
    WIT raw counts (angle*32768/180 deg, acc*32768/16 g, gyro*32768/2000 dps).

    The vendor host forwards the WIT JY61P readings to the MCU at ~44 Hz (mcu_tap capture
    2026-10-05: 3180 ImuData frames in 72 s, gyro fields always 0). The MCU uses them for its
    own tilt/lift logic, so without this stream it reports the mower as lifted.
    """
    def clamp(v):
        return max(-32768, min(32767, int(round(v))))
    ang = lambda r: clamp(r / math.pi * 32768.0)                       # rad -> counts
    acc_c = lambda a: clamp(a / (16.0 * 9.80665) * 32768.0)            # m/s^2 -> counts
    gyr_c = lambda g: clamp(g / math.radians(2000.0) * 32768.0)        # rad/s -> counts
    return struct.pack(IMU_FMT, ang(pitch), ang(roll), ang(yaw),
                       acc_c(acc[0]), acc_c(acc[1]), acc_c(acc[2]),
                       gyr_c(gyro[0]), gyr_c(gyro[1]), gyr_c(gyro[2]))


def _quat_to_rpy(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def charge_payload(enable):
    return struct.pack(CHARGE_FMT, 1 if enable else 0)


def speed_payload(linear, angular):
    """Build the 8-byte ``SpeedData`` payload (m/s, rad/s)."""
    return struct.pack(SPEED_FMT, float(linear), float(angular))


def _diag6(a, b, c, d, e, f):
    """Row-major 6x6 covariance matrix with ``a..f`` on the diagonal."""
    cov = [0.0] * 36
    for idx, val in enumerate((a, b, c, d, e, f)):
        cov[idx * 6 + idx] = val
    return cov


def _diag3(a, b, c):
    """Row-major 3x3 covariance matrix (``sensor_msgs/Imu`` fields) with ``a..c`` on the diagonal."""
    return [a, 0.0, 0.0, 0.0, b, 0.0, 0.0, 0.0, c]


def _yaw_to_quaternion(yaw):
    """Planar yaw -> ``geometry_msgs/Quaternion`` (roll = pitch = 0)."""
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class _TermiosSerial:
    """Tiny pyserial-compatible fallback (``read``/``write``/``in_waiting``/``close``).

    ``python3-serial`` may be missing in a freshly pulled ``ros:humble`` image, and the
    offline loopback test only ever talks to a pty - so if pyserial is not importable we
    configure the tty ourselves in raw 8N1 mode with the ``termios`` stdlib module.  On a pty
    the baud rate is meaningless; on a real UART it still gets applied.
    """

    SPEEDS = {9600: termios.B9600, 19200: termios.B19200, 38400: termios.B38400,
              57600: termios.B57600, 115200: termios.B115200, 230400: termios.B230400}

    def __init__(self, path, baud):
        self._fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(self._fd)
            speed = _TermiosSerial.SPEEDS.get(baud, termios.B115200)
            attrs[0] = 0                                        # iflag: no input processing
            attrs[1] = 0                                        # oflag: no output post-processing
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # 8N1, no hw flow control
            attrs[3] = 0                                        # lflag: raw (no canon, no echo)
            attrs[4] = attrs[5] = speed                         # ispeed, ospeed
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
            termios.tcflush(self._fd, termios.TCIOFLUSH)
        except termios.error:
            pass  # not a real tty (should not happen) - leave the line discipline alone

    @property
    def in_waiting(self):
        buf = array.array('i', [0])
        try:
            fcntl.ioctl(self._fd, termios.FIONREAD, buf, True)
        except OSError:
            return 0
        return buf[0]

    def read(self, size):
        ready, _, _ = select.select([self._fd], [], [], 0.02)
        if not ready:
            return b''
        try:
            return os.read(self._fd, size)
        except BlockingIOError:
            return b''

    def write(self, data):
        view = memoryview(data)
        written = 0
        while view:
            _, ready, _ = select.select([], [self._fd], [], 1.0)
            if not ready:
                break
            try:
                count = os.write(self._fd, view)
            except BlockingIOError:
                continue
            view = view[count:]
            written += count
        return written

    def fileno(self):
        return self._fd

    def close(self):
        os.close(self._fd)


class McuNode(Node):
    """ROS 2 wrapper around the Airseekers MCU serial protocol."""

    def __init__(self):
        super().__init__('mower_mcu_driver')

        # ---- parameters ---------------------------------------------------
        self.declare_parameter('port', '/dev/serial_mower')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('frame_id', 'base_link')
        # Heartbeat period 100 ms CONFIRMED from the vendor tap capture (2026-10-05, 718 beats,
        # mean 0.100 s). The MCU failsafe timeout is still uncharacterised.
        self.declare_parameter('heartbeat_period', 0.1)
        self.declare_parameter('speed_cmd_rate', 20.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('speed_timeout', 0.5)
        self.declare_parameter('odom_rate', 50.0)
        self.declare_parameter('use_imu_yaw', True)
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)
        # Vendor PidControllerROS clamps its /cmd_vel output to +-0.3; mirror it until the
        # wheel scale/sign has been characterised wheels-up.
        self.declare_parameter('linear_max', 0.3)
        self.declare_parameter('angular_max', 0.3)
        # SpeedData TX policy (docs/wheel_control_semantics.md section 5): one frame per
        # /cmd_vel message; after a non-zero command, a zero / timeout / interlock sends
        # stop_frames zeros stop_frame_spacing_s apart, then silence. Idle = silence.
        self.declare_parameter('stop_frames', 3)
        self.declare_parameter('stop_frame_spacing_s', 0.1)
        # DEBUG ONLY (A/B experiments): true restores a periodic stream of the last command
        # at speed_cmd_rate (zeros once stale/interlocked).  Read fresh every tick, so it can
        # be flipped live with `ros2 param set`; switching it off ends with a stop sequence.
        self.declare_parameter('speed_stream_enabled', False)
        # Telemetry: measured vs commanded velocity at this rate (0 disables); /mcu/sent_speed
        # is published once per SpeedData frame that actually left the host.
        self.declare_parameter('speed_telemetry_rate_hz', 5.0)
        # Host-side interlock: stop wheels + cutter while lift / e-stop (and optionally
        # bumper) are asserted, in addition to whatever the MCU firmware enforces.
        self.declare_parameter('interlock_on_lift', True)
        self.declare_parameter('interlock_on_stop', True)
        self.declare_parameter('interlock_on_bumper', False)
        self.declare_parameter('cutter_default_speed', 100)   # MotorControl.speed (% or raw)
        # Deck height sent with blade OFF: 0 = hold the last requested height (default; least
        # disruptive mid-mow), 30-90 = that height (90 mimics the vendor's CutterOFF).
        self.declare_parameter('cutter_off_height_mm', 0)
        # Forward the WIT IMU to the MCU like the vendor host (ImuData @ ~44 Hz; see imu_payload).
        self.declare_parameter('forward_imu', True)
        self.declare_parameter('forward_imu_topic', '/imu/data')
        self.declare_parameter('forward_imu_rate', 45.0)
        # Cutter firmware 0.6.36 lift logic (mcu_decompile analysis 2026-10-06): lift is asserted
        # while |pitch| > ~37 deg or |roll| > ~38 deg of the forwarded ImuData (int16 pitch, roll
        # in deg*32767/180), plus two hall sensors; once latched > ~17 s only the module-10 clear
        # frame (or a power cycle) releases it, and only while the IMU is inside the window.
        # Upright must therefore be sent as pitch ~ 0, roll ~ 0. Offsets exist only for a sensor
        # mounted in a different orientation than the WIT frame assumes.
        self.declare_parameter('forward_imu_roll_offset_deg', 0.0)
        self.declare_parameter('forward_imu_pitch_offset_deg', 0.0)
        self.declare_parameter('battery_voltage_scale', 0.1)
        # SensorInfo arrives at ~100 Hz. /mower_base/status is published for every frame;
        # /mower_sensor_info (read only by gui_bridge at 5 Hz) and /estop (twist_mux lock with
        # timeout 0, gui_bridge acts on changes) are published immediately whenever their
        # content changes and otherwise as a keepalive at this rate. 0 = every frame (old).
        self.declare_parameter('sensor_info_rate_hz', 10.0)
        # /mower_base/status is published per SensorInfo frame (~100 Hz). While
        # /mission/activity is docked_idle/idle it is limited to this rate; any change of a
        # field (bumper, lift, stop, dock, ...) is still published at once.
        self.declare_parameter('status_idle_rate_hz', 10.0)
        # While /mission/activity is docked_idle/idle the serial RX is drained on this period
        # instead of waking on every chunk (frames queue in the kernel buffer; /cmd_vel and
        # the periodic tasks still wake the loop at once). <= 0 disables.
        self.declare_parameter('rx_idle_period', 0.02)
        self.declare_parameter('estop_rate_hz', 10.0)
        # BatteryInfo.current: 0.1 A units (vendor PowerManager logs raw*100 as mA), positive
        # while discharging (raw 7 off-dock at rest). BatteryState wants negative = discharge.
        self.declare_parameter('battery_current_scale', -0.1)
        # Rain ADC (vendor DevHealthHandler::rainSensorProcess, see rain.py). Dry reads
        # ~4092 (pulled up); wet pulls it into [rain_valid_min, rain_wet_below].
        self.declare_parameter('rain_path', '/dev/rain')
        self.declare_parameter('rain_poll_hz', 2.0)
        self.declare_parameter('rain_wet_below', 4000)     # vendor rain_max
        self.declare_parameter('rain_dry_above', 4000)     # vendor: no hysteresis band
        self.declare_parameter('rain_valid_min', 2000)     # vendor rain_min (below = fault)
        self.declare_parameter('rain_debounce_s', 5.0)
        self.declare_parameter('rain_clear_s', 600.0)

        def param(name):
            return self.get_parameter(name).value

        self.port = str(param('port'))
        self.baud = int(param('baud'))
        self.odom_frame = str(param('odom_frame'))
        self.base_frame = str(param('base_frame'))
        self.imu_frame = str(param('imu_frame'))
        self.frame_id = str(param('frame_id'))
        self.heartbeat_period = float(param('heartbeat_period'))
        self.cmd_vel_timeout = float(param('cmd_vel_timeout'))
        self.speed_timeout = float(param('speed_timeout'))
        self.use_imu_yaw = bool(param('use_imu_yaw'))
        self.linear_scale = float(param('linear_scale'))
        self.angular_scale = float(param('angular_scale'))
        self.linear_max = float(param('linear_max'))
        self.angular_max = float(param('angular_max'))
        self.battery_voltage_scale = float(param('battery_voltage_scale'))
        self.battery_current_scale = float(param('battery_current_scale'))
        rate = float(param('sensor_info_rate_hz'))
        self._sensor_info_period = 1.0 / rate if rate > 0 else 0.0
        rate = float(param('estop_rate_hz'))
        self._estop_period = 1.0 / rate if rate > 0 else 0.0
        self._sensor_info_pub_t = -1e9
        self._sensor_info_pub_key = None
        self._estop_pub_t = -1e9
        self._estop_pub_value = None
        self.interlock_on_lift = bool(param('interlock_on_lift'))
        self.interlock_on_stop = bool(param('interlock_on_stop'))
        self.interlock_on_bumper = bool(param('interlock_on_bumper'))
        self.stop_frames = max(1, int(param('stop_frames')))
        self.stop_frame_spacing = float(param('stop_frame_spacing_s'))
        rate = float(param('speed_telemetry_rate_hz'))
        self._speed_telemetry_period = 1.0 / rate if rate > 0 else 0.0
        self._speed_telemetry_t = -1e9
        self.cutter_default_speed = int(param('cutter_default_speed'))
        self.cutter_off_height_mm = int(param('cutter_off_height_mm'))
        self.forward_imu = bool(param('forward_imu'))
        self.forward_imu_rate = float(param('forward_imu_rate'))
        self.forward_imu_roll_offset = math.radians(float(param('forward_imu_roll_offset_deg')))
        self.forward_imu_pitch_offset = math.radians(float(param('forward_imu_pitch_offset_deg')))
        self._last_imu_fwd = 0.0
        self._imu_fwd_count = 0
        speed_cmd_rate = float(param('speed_cmd_rate'))
        odom_rate = float(param('odom_rate'))

        # ---- state --------------------------------------------------------
        # One lock serialises everything that touches the port or the driver state: the
        # I/O thread (serial RX + periodic TX/odom), the subscription pump thread and the
        # executor (services only). This keeps the old single-threaded semantics.
        self._lock = threading.RLock()
        self._io_thread = None
        self._io_stop = False
        self._ser = None
        self._last_open_attempt = 0.0
        self._parser = FrameParser()

        self._cmd_linear = 0.0            # last /cmd_vel, already mapped + clamped
        self._cmd_angular = 0.0
        self._cmd_stamp = 0.0             # monotonic time of the last /cmd_vel

        self._meas_linear = 0.0           # last measured SpeedData from the MCU
        self._meas_angular = 0.0
        self._meas_stamp = 0.0

        # ---- SpeedData TX state (event-driven; see _speed_tick) ---------------
        # /cmd_vel callbacks only queue the mapped command and wake the I/O thread, which
        # writes the frame.  Every queued message produces at most one frame.
        self._mono = time.monotonic      # injectable for tests
        self._cmd_queue = collections.deque()
        self._moving = False              # last frame sent was non-zero
        self._last_nonzero_t = 0.0        # arrival time of the last non-zero command
        self._stop_left = 0               # zero frames still owed by the stop sequence
        self._stop_next_t = 0.0
        self._streaming = False           # debug stream active (speed_stream_enabled)
        self._stream_next_t = 0.0
        self._stream_period = 1.0 / max(speed_cmd_rate, 1.0)
        self._wake_r, self._wake_w = os.pipe()
        os.set_blocking(self._wake_r, False)
        os.set_blocking(self._wake_w, False)

        self._imu_yaw = None              # last MCU ImuData heading (rad)
        self._imu_stamp = 0.0
        self._have_imu = False            # set once MODULE_IMU was seen at all

        self._x = 0.0                     # dead-reckoned pose
        self._y = 0.0
        self._yaw = 0.0
        self._last_reckon = time.monotonic()

        self._sensor_info = None          # last SensorInfo, cached for /mower_sensor_info
        self._versions = {}               # 'cutter'/'chassis'/'rtk' -> (major, minor, patch)
        self._versions_key = ()           # tuple(self._versions.items()), rebuilt on change
        self._sensor_payload = None       # raw SensorInfo bytes of self._sensor_info
        self._motors_payload = None       # raw motors bytes of self._motors
        self._battery = None              # last BatteryInfo dict
        self._motors = {}                 # motor name -> MotorInfo msg-unit values
        self._motors_si = {}              # motor name -> decode_motors() SI dict
        self._mower_sensor_info_pub_warned = False

        self._cutter_cmd = None           # last CutterControl payload sent (bytes) or None
        self._height_mm = CUTTER_HEIGHT_DEFAULT_MM   # last requested deck height, always sent
        self._cutter_requested_on = False # host wants the blade on (re-sent after interlock)
        self._cutter_speed = 0            # speed of the last blade-ON request (for set_height)
        self._height_published = None     # last value latched on /cutter/height_mm
        self._charging_enabled = False
        self._interlock_latched = False   # True while an interlock has forced motion off
        self._estop_latched = False       # host-side e-stop latch (/estop topic), cleared by /clear_estop

        self._rain = RainDetector(param('rain_wet_below'), param('rain_dry_above'),
                                  param('rain_valid_min'), param('rain_debounce_s'),
                                  param('rain_clear_s'),
                                  window=max(1, int(round(2.0 * float(param('rain_poll_hz'))))))
        self._rain_path = str(param('rain_path'))
        self._rain_fd = None
        self._rain_ok = False             # last sysfs read succeeded
        self._rain_err_t = 0.0

        # ---- publishers / subscribers -------------------------------------
        sensor_qos = QoSProfile(depth=10,
                                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                history=QoSHistoryPolicy.KEEP_LAST)

        self._battery_pub = self.create_publisher(BatteryState, '/battery', 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        # MCU ImuData is a *copy* of the WIT IMU the MCU receives; the real IMU topic
        # (/imu/data) is wit_imu_driver's. Keep this one off the main name.
        self._imu_pub = self.create_publisher(Imu, '/mcu/imu', sensor_qos)
        self._estop_pub = self.create_publisher(Bool, '/estop', 10)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST)
        if QoSDurabilityPolicy is not None:
            latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._rain_pub = self.create_publisher(Bool, '/rain', latched)
        self._rain_published = None
        # Commanded deck height (no deck-position telemetry exists): latched, published on
        # startup and whenever the remembered height changes.
        self._height_pub = self.create_publisher(Int16, '/cutter/height_mm', latched)
        self._publish_height()
        self._status_pub = None
        if MowerBaseDevStatus is not None:
            self._status_pub = self.create_publisher(MowerBaseDevStatus, '/mower_base/status', 10)

        # /mower_sensor_info only if the interface package has been created; otherwise skip.
        self._sensor_pub = None
        if MowerSensorInfo is not None:
            self._sensor_pub = self.create_publisher(MowerSensorInfo, '/mower_sensor_info', 10)
            self.get_logger().info('/mower_sensor_info enabled (mower_interfaces found)')
        else:
            self.get_logger().warn(
                'mower_interfaces not available -> /mower_sensor_info disabled '
                '(build ros2_stack/src/mower_interfaces and relaunch to enable it)')

        # Telemetry: what the MCU reports (measured) and what /cmd_vel asked for (commanded)
        # at speed_telemetry_rate_hz; /mcu/sent_speed once per SpeedData frame on the wire.
        self._meas_speed_pub = self.create_publisher(TwistStamped, '/mcu/measured_speed', 10)
        self._cmd_speed_pub = self.create_publisher(TwistStamped, '/mcu/commanded_speed', 10)
        self._sent_speed_pub = self.create_publisher(TwistStamped, '/mcu/sent_speed', 10)

        # Humble convention (twist_mux 4.3 / Nav2 Humble): unstamped Twist on /cmd_vel.
        # /cmd_vel_stamped mirrors the vendor node's second input.
        # With real rclpy the subscriptions are served by a lean pump thread (see sub_pump.py)
        # instead of the executor; the 100 Hz /imu/data alone cost ~10 % of a core there.
        self._pump = SubscriptionPump(self, 'mcu_inputs') if SubscriptionPump else None
        self._subscribe(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self._subscribe(TwistStamped, '/cmd_vel_stamped', self._on_cmd_vel_stamped, 10)
        self._subscribe(Bool, '/estop_request', self._on_estop_request, 10)
        if self.forward_imu:
            if self._pump is not None:
                # raw: only the <= 45 Hz samples that pass the rate limit are deserialized
                self._pump.subscribe(Imu, str(param('forward_imu_topic')),
                                     self._on_imu_forward_raw, sensor_qos, raw=True,
                                     lock=self._lock)
            else:
                self.create_subscription(Imu, str(param('forward_imu_topic')),
                                         self._on_imu_forward, sensor_qos)

        # ---- services (vendor base_driver_node contract) ---------------------
        if CutterControl is not None:
            self.create_service(CutterControl, '/cutter_control',
                                self._locked(self._srv_cutter_control))
        if ChargingControl is not None:
            self.create_service(ChargingControl, '/charging', self._locked(self._srv_charging))
        self.create_service(Empty, '/clear_estop', self._locked(self._srv_clear_estop))
        self.create_service(Trigger, '/cutter_off', self._locked(self._srv_cutter_off))
        if SetCutterHeight is not None:
            self.create_service(SetCutterHeight, '/cutter/set_height',
                                self._locked(self._srv_set_height))
        # Fill light (host PWM, owned by fill_light_node): mirror its read-back into
        # /mower_sensor_info.is_fill_light_on like the vendor getFillLightStatus().
        self._fill_light_on = False
        self.create_subscription(Bool, '/fill_light/state', self._on_fill_light_state, latched)
        self._low_power = False
        self._status_gate = ChangeOrPeriodGate()
        self._status_idle_period = 1.0 / max(0.1, float(param('status_idle_rate_hz')))
        self._rx_idle_period = float(param('rx_idle_period'))
        self.create_subscription(String, '/mission/activity', self._on_activity, latched)

        # The two 100/50 Hz publishers send pre-serialized CDR (sub_pump serializers, unit
        # tested against rclpy): a typed publish costs ~0.4 ms more CPU here (message object,
        # clock stamp, Python->C conversion). Typed publishing remains the fallback.
        self._status_ser = None
        self._fast_odom = False
        if SubscriptionPump is not None:
            if MowerBaseDevStatus is not None:
                self._status_ser = flat_serializer(MowerBaseDevStatus)
            self._fast_odom = odometry_bytes is not None
            if bool(self.get_parameter('use_sim_time').value):
                self._stamp_ns = lambda: self.get_clock().now().nanoseconds
            else:
                self._stamp_ns = time.time_ns       # ROS time == system time without sim time

        # ---- periodic work ----------------------------------------------------
        # Run by the I/O thread (start_io()), not by rclpy timers: the 100 Hz RX poll plus
        # the 50/20/10 Hz timers were ~180 executor wakes/s (~40 % of a core). The I/O thread
        # blocks in select() on the port and dispatches frames the moment they arrive.
        self._periodic = [
            (self.heartbeat_period, self._send_heartbeat),
            (0.02, self._speed_tick),          # stop-sequence spacing / cmd_vel timeout
            (1.0 / max(odom_rate, 1.0), self._reckon),
        ]
        if float(self.get_parameter('rain_poll_hz').value) > 0.0:
            self._periodic.append((1.0 / float(self.get_parameter('rain_poll_hz').value),
                                   self._rain_tick))

        self.get_logger().info(
            'mower_mcu_driver up: %s @ %d baud, heartbeat every %.0f ms (period UNKNOWN), '
            'odom %.0f Hz, SpeedData: one frame per /cmd_vel, clamp %.2f m/s %.2f rad/s, '
            'stop = %d zeros @ %.0f ms then silence'
            % (self.port, self.baud, self.heartbeat_period * 1000.0, odom_rate,
               self.linear_max, self.angular_max, self.stop_frames,
               self.stop_frame_spacing * 1000.0))

    # ------------------------------------------------------------- threading
    def _locked(self, fn):
        def wrapper(*args):
            with self._lock:
                return fn(*args)
        wrapper.__name__ = getattr(fn, '__name__', 'locked')
        return wrapper

    def _subscribe(self, msg_type, topic, callback, qos):
        if self._pump is not None:
            return self._pump.subscribe(msg_type, topic, callback, qos, lock=self._lock)
        return self.create_subscription(msg_type, topic, callback, qos)

    def start_io(self):
        """Start the serial I/O thread and the subscription pump (called by main())."""
        if self._io_thread is None:
            self._io_stop = False
            self._io_thread = threading.Thread(target=self._io_loop, name='mcu_io',
                                               daemon=True)
            self._io_thread.start()
        if self._pump is not None:
            self._pump.start()

    def stop_io(self, timeout=2.0):
        self._io_stop = True
        if self._pump is not None:
            self._pump.stop(timeout)
        if self._io_thread is not None:
            self._io_thread.join(timeout)
            self._io_thread = None

    def _io_loop(self):
        """Serial RX as frames arrive + the periodic heartbeat / SpeedData / odometry."""
        now = time.monotonic()
        due = [now + period for period, _fn in self._periodic]
        errors = set()
        while not self._io_stop:
            try:
                timeout = max(0.0, min(due) - time.monotonic())
                ser = self._ser
                if ser is None:
                    with self._lock:
                        self._open_serial()        # rate-limited to one attempt per 2 s
                    if self._ser is None:
                        select.select([self._wake_r], [], [], min(timeout, 0.05))
                else:
                    try:
                        fd = ser.fileno()
                    except Exception:  # noqa: BLE001 - port object without an fd
                        fd = None
                    if fd is None:
                        select.select([self._wake_r], [], [], min(timeout, 0.005))
                        readable = True
                    elif self._low_power and self._rx_idle_period > 0.0:
                        # idle: drain in blocks, not per chunk (one wake per rx_idle_period)
                        select.select([self._wake_r], [], [], min(timeout, self._rx_idle_period))
                        readable = True
                    else:
                        ready = select.select([fd, self._wake_r], [], [], timeout)[0]
                        readable = fd in ready
                    if readable:
                        with self._lock:
                            if self._ser is ser:
                                self._poll_serial()
                if self._drain_wake():
                    with self._lock:
                        self._speed_tick()         # /cmd_vel arrived: send it right now
                now = time.monotonic()
                for idx, (period, fn) in enumerate(self._periodic):
                    if now >= due[idx]:
                        due[idx] += period
                        if due[idx] <= now:        # fell behind: do not burst to catch up
                            due[idx] = now + period
                        with self._lock:
                            fn()
            except Exception as exc:  # noqa: BLE001 - keep the link alive, log once per type
                key = type(exc).__name__
                if key not in errors:
                    errors.add(key)
                    self.get_logger().error('mcu I/O loop error (logged once per type):\n%s'
                                            % traceback.format_exc())
                time.sleep(0.01)

    # ------------------------------------------------------------------ serial
    def _open_serial(self):
        """Open the port, retrying forever so launch files survive a late udev symlink."""
        now = time.monotonic()
        if self._ser is not None or now - self._last_open_attempt < 2.0:
            return
        self._last_open_attempt = now
        try:
            import serial  # pyserial (python3-serial)
        except ImportError:
            serial = None
        try:
            if serial is not None:
                self._ser = serial.Serial(self.port, self.baud, timeout=0.02, write_timeout=1.0)
                backend = 'pyserial'
            else:
                self._ser = _TermiosSerial(self.port, self.baud)
                backend = 'termios fallback (python3-serial not installed)'
        except Exception as exc:  # FileNotFoundError, PermissionError, SerialException, ...
            self.get_logger().warn('cannot open %s: %s (retrying in 2 s)' % (self.port, exc),
                                   throttle_duration_sec=10.0)
            return
        self.get_logger().info('opened %s @ %d baud via %s' % (self.port, self.baud, backend))

    def _close_serial(self, reason):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
            self.get_logger().error('serial port closed: %s' % reason)

    def _write(self, data):
        if self._ser is None:
            return
        try:
            self._ser.write(data)
        except Exception as exc:
            self._close_serial('write failed: %s' % exc)

    def _poll_serial(self):
        """Drain the RX FIFO and dispatch whatever complete frames came in."""
        if self._ser is None:
            self._open_serial()
            return
        try:
            waiting = self._ser.in_waiting
            chunk = self._ser.read(waiting or 1)
        except Exception as exc:
            self._close_serial('read failed: %s' % exc)
            return
        if not chunk:
            return
        for type_id, mod_id, payload in self._parser.feed(chunk):
            self._dispatch(type_id, mod_id, payload)

    # ------------------------------------------------------------------- RX
    def _dispatch(self, type_id, mod_id, payload):
        if type_id == TYPE_HEARTBEAT:
            return  # MCU->host heartbeat/ack, nothing to do yet
        if type_id != TYPE_MOWER_ROS:
            self.get_logger().debug('ignoring type=%d mod=%d' % (type_id, mod_id),
                                    throttle_duration_sec=5.0)
            return
        if mod_id == MOD_SPEED:
            self._on_speed_data(payload)
        elif mod_id == MOD_BATTERY:
            self._on_battery(payload)
        elif mod_id == MOD_IMU:
            self._on_imu(payload)
        elif mod_id == MOD_SENSOR:
            self._on_sensor_info(payload)
        elif mod_id == MOD_VERSION:
            self._on_version(payload)
        elif mod_id == MOD_BMS_VERSION:
            pass  # BMS firmware version, no ROS home yet
        elif mod_id == MOD_MOTORS:
            self._on_motors(payload)
        elif mod_id == MOD_LOG:
            self.get_logger().info('MCU: %s' % payload.decode('utf-8', 'replace'))
        elif mod_id == MOD_CALIB:
            if len(payload) == struct.calcsize(CALIB_FMT):
                calib, left, right = struct.unpack(CALIB_FMT, payload)
                self.get_logger().info('MCU calibration: calib=%d left=%d right=%d'
                                       % (calib, left, right))
            else:
                self.get_logger().info('MCU calibration result: %s' % payload.hex())
        else:
            # module 7 (uptime/tick ?) and anything else: counted but not decoded
            self.get_logger().debug('unhandled module %d (%d bytes)' % (mod_id, len(payload)),
                                    throttle_duration_sec=10.0)

    def _on_speed_data(self, payload):
        if len(payload) != struct.calcsize(SPEED_FMT):
            return
        self._meas_linear, self._meas_angular = struct.unpack(SPEED_FMT, payload)
        self._meas_stamp = time.monotonic()

    def _on_battery(self, payload):
        if len(payload) != struct.calcsize(BATTERY_FMT):
            return
        (voltage, current, percentage, dock_ok, temperature,
         error) = struct.unpack(BATTERY_FMT, payload)
        self._battery = {
            'voltage': voltage, 'current': current, 'percentage': percentage,
            'dock_ok': dock_ok, 'temperature': temperature, 'error': error,
        }
        self._publish_battery(self._battery)

    def _publish_battery(self, raw):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        # BatteryInfo.voltage is in 0.1 V units (248 -> 24.8 V). NOTE: the stock node's
        # /battery sample in 10_ros_live_snapshot shows 247.0 V, i.e. it published the raw
        # value; battery_voltage_scale:=1.0 reproduces that if you need byte-for-byte parity.
        msg.voltage = raw['voltage'] * self.battery_voltage_scale
        # BatteryInfo.current: 0.1 A, positive = discharging (see battery_current_scale).
        msg.current = raw['current'] * self.battery_current_scale
        msg.percentage = max(0.0, min(100.0, raw['percentage'])) / 100.0
        msg.temperature = float(raw['temperature'])
        msg.present = True
        msg.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_CHARGING
                                   if raw['dock_ok'] else
                                   BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        self._battery_pub.publish(msg)

    def _on_imu(self, payload):
        """MCU ``ImuData`` (MODULE_IMU) -> placeholder ``/imu``.

        ``SensorInfoControl`` is the host->MCU direction of module 10 and carries no IMU
        data; the only IMU sub-packet on this bus is ``ImuData`` (module 9).  If the firmware
        never emits it, this callback is never called and **nothing is published** on /imu -
        the real IMU is the WIT JY61P handled by wit_imu_driver, exactly as on the stock node.
        """
        if len(payload) != struct.calcsize(IMU_FMT):
            return
        (pitch, roll, yaw, accx, accy, accz,
         gyrox, gyroy, gyroz) = struct.unpack(IMU_FMT, payload)
        if not self._have_imu:
            self._have_imu = True
            self.get_logger().info('MCU MODULE_IMU (9) seen -> publishing placeholder /imu')

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        q = self._euler_to_quaternion(
            pitch * WIT_RAW_ANGLE_TO_RAD, roll * WIT_RAW_ANGLE_TO_RAD,
            yaw * WIT_RAW_ANGLE_TO_RAD)
        msg.orientation.x, msg.orientation.y = q[0], q[1]
        msg.orientation.z, msg.orientation.w = q[2], q[3]
        msg.orientation_covariance = _diag3(0.05, 0.05, 0.1)
        msg.angular_velocity.x = gyrox * WIT_RAW_GYRO_TO_RAD
        msg.angular_velocity.y = gyroy * WIT_RAW_GYRO_TO_RAD
        msg.angular_velocity.z = gyroz * WIT_RAW_GYRO_TO_RAD
        msg.angular_velocity_covariance = _diag3(0.02, 0.02, 0.02)
        msg.linear_acceleration.x = accx * WIT_RAW_ACC_TO_MS2
        msg.linear_acceleration.y = accy * WIT_RAW_ACC_TO_MS2
        msg.linear_acceleration.z = accz * WIT_RAW_ACC_TO_MS2
        msg.linear_acceleration_covariance = _diag3(0.1, 0.1, 0.1)
        self._imu_pub.publish(msg)

        # Heading source for the odometry dead reckoner (sign convention needs validating
        # on hardware - WIT yaw is often compass-style, see TODO in _reckon()).
        self._imu_yaw = yaw * WIT_RAW_ANGLE_TO_RAD
        self._imu_stamp = time.monotonic()

    @staticmethod
    def _euler_to_quaternion(roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        return (sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy)

    def _on_sensor_info(self, payload):
        if len(payload) != struct.calcsize(SENSOR_FMT):
            return
        payload = bytes(payload)
        if payload != self._sensor_payload:   # 100 Hz, almost always unchanged
            self._sensor_info = dict(zip(_SENSOR_NAMES, struct.unpack(SENSOR_FMT, payload)))
            self._sensor_payload = payload
        self._apply_interlock()
        self._publish_sensor_info()
        self._publish_dev_status()

    def _rain_flag(self, s):
        """Host ADC trigger when /dev/rain is readable (vendor overrides the MCU flag with
        it, DevStatus::setRainStatus); the MCU SensorInfo.rain byte otherwise."""
        if self._rain.value is not None:
            return bool(self._rain.triggered)
        return bool(s['rain'])

    def _read_rain(self):
        """One sysfs read (pread on a kept-open fd; iio re-samples on every read)."""
        try:
            if self._rain_fd is None:
                self._rain_fd = os.open(self._rain_path, os.O_RDONLY)
            return int(os.pread(self._rain_fd, 16, 0).strip())
        except (OSError, ValueError) as exc:
            if self._rain_fd is not None:
                try:
                    os.close(self._rain_fd)
                except OSError:
                    pass
                self._rain_fd = None
            now = time.monotonic()
            if self._rain_ok or now - self._rain_err_t > 300.0:
                self._rain_err_t = now
                self.get_logger().warn('rain sensor %s unreadable: %s' % (self._rain_path, exc))
            self._rain_ok = False
            return None

    def _rain_tick(self):
        raw = self._read_rain()
        if raw is None:
            return
        self._rain_ok = True
        before = self._rain.triggered
        triggered = self._rain.update(raw, time.monotonic())
        if triggered != before:
            self.get_logger().warn('rain %s (filtered ADC %d)'
                                   % ('DETECTED' if triggered else 'cleared', self._rain.value))
        if triggered != self._rain_published:
            self._rain_published = triggered
            msg = Bool()
            msg.data = triggered
            self._rain_pub.publish(msg)
            if before != triggered and self._sensor_info is not None:
                self._publish_dev_status()
                self._publish_sensor_info()

    def _on_version(self, payload):
        if len(payload) != struct.calcsize(VERSION_FMT):
            return
        vals = struct.unpack(VERSION_FMT, payload)
        for idx, key in enumerate(('cutter', 'chassis', 'rtk')):
            self._versions[key] = tuple(vals[idx * 3:idx * 3 + 3])
        self._versions_key = tuple(self._versions.items())

    def _on_motors(self, payload):
        payload = bytes(payload)
        if payload == self._motors_payload:   # unchanged (idle): keep the decoded values
            return
        decoded = decode_motors(payload)
        if decoded is None:
            return
        self._motors_si = decoded
        self._motors_payload = payload
        # self._motors holds MowerSensorInfo.MotorInfo units (rpm, 10 mA, 10 mV, degC) with
        # the drive-board scalings already normalised (see MOTOR_*_PER_COUNT).
        for name, si in decoded.items():
            self._motors[name] = motor_msg_fields(si)

    def _on_fill_light_state(self, msg):
        self._fill_light_on = bool(msg.data)

    def _publish_sensor_info(self):
        """Publish ``/mower_sensor_info`` if the interface package exists.

        Split into "build" + :meth:`_fill_sensor_info` so a malformed message can never take
        the timer callback down: a warning is logged once and publishing simply stops.
        """
        if self._sensor_pub is None or self._sensor_info is None or MowerSensorInfo is None:
            return
        # Rate limit (see sensor_info_rate_hz), but never delay a change of the sensor flags,
        # versions, battery flags or the motion/cutting state.
        now = time.monotonic()
        cutter = self._motors.get('cutter')
        linear, angular, _measured = self._speed(now)
        key = (self._sensor_payload or tuple(self._sensor_info.values()), self._versions_key,
               self._rain.triggered, self._fill_light_on,
               (self._battery or {}).get('dock_ok'), (self._battery or {}).get('error'),
               bool(cutter and abs(cutter['speed']) > 500),
               abs(linear) > 1e-2 or abs(angular) > 1e-2,
               now - self._cmd_stamp < self.cmd_vel_timeout)
        if key == self._sensor_info_pub_key and \
                now - self._sensor_info_pub_t < self._sensor_info_period:
            return
        self._sensor_info_pub_key = key
        self._sensor_info_pub_t = now
        try:
            msg = MowerSensorInfo()
        except Exception as exc:  # pragma: no cover - defensive
            if not self._mower_sensor_info_pub_warned:
                self.get_logger().warn('cannot build MowerSensorInfo: %s' % exc)
                self._mower_sensor_info_pub_warned = True
            return
        try:
            self._fill_sensor_info(msg)
        except Exception as exc:  # pragma: no cover - defensive
            if not self._mower_sensor_info_pub_warned:
                self.get_logger().error('cannot fill MowerSensorInfo: %s' % exc)
                self._mower_sensor_info_pub_warned = True
            return
        self._sensor_pub.publish(msg)

    def _on_activity(self, msg):
        self._low_power = activity_low_power(msg.data)

    def _publish_dev_status(self):
        """``/mower_base/status`` (MowerBaseDevStatus): the compact flag set that
        bumper_controller and the mission layer consume."""
        if self._status_pub is None or self._sensor_info is None:
            return
        s = self._sensor_info
        if self._low_power:
            # idle: change-or-10 Hz (the typed fallback path below is never rate limited)
            key = tuple(self._dev_status_values(s).values())
            if not self._status_gate.allow(key, time.monotonic(), self._status_idle_period):
                return
        if self._status_ser is not None:
            self._status_pub.publish(self._status_ser(self._dev_status_values(s),
                                                      self._stamp_ns()))
            return
        try:
            msg = MowerBaseDevStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            cutter = self._motors.get('cutter')
            msg.is_cutting = bool(cutter and abs(cutter['speed']) > 500)
            now = time.monotonic()
            linear, angular, measured = self._speed(now)
            msg.is_moving = bool(measured and (abs(linear) > 1e-2 or abs(angular) > 1e-2))
            msg.is_cmd_moving = bool(now - self._cmd_stamp < self.cmd_vel_timeout
                                     and (abs(self._cmd_linear) > 1e-2 or abs(self._cmd_angular) > 1e-2))
            msg.is_docking_done = bool(self._battery and self._battery.get('dock_ok'))
            msg.is_charging = bool(self._battery and self._battery.get('dock_ok')
                                   and self._charging_enabled)
            msg.bumper_routing_enabled = True
            msg.battery_gate_open = bool(s['battery_gate'])
            msg.press_module = bool(s['press_module'])
            msg.stop_triggered = bool(s['stop']) or self._estop_latched
            msg.bumper_triggered = bool(s['bumper'] or s['bumper_l'] or s['bumper_r'])
            msg.left_bumper_triggered = bool(s['bumper_l'])
            msg.right_bumper_triggered = bool(s['bumper_r'])
            msg.rain_triggered = self._rain_flag(s)
            msg.lift_triggered = bool(s['lift'])
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().error('cannot fill MowerBaseDevStatus: %s' % exc,
                                    throttle_duration_sec=30.0)
            return
        self._status_pub.publish(msg)

    def _dev_status_values(self, s):
        """Field values of /mower_base/status, identical to the typed path above."""
        cutter = self._motors.get('cutter')
        now = time.monotonic()
        linear, angular, measured = self._speed(now)
        dock_ok = bool(self._battery and self._battery.get('dock_ok'))
        return {
            'is_cutting': bool(cutter and abs(cutter['speed']) > 500),
            'is_moving': bool(measured and (abs(linear) > 1e-2 or abs(angular) > 1e-2)),
            'is_cmd_moving': bool(now - self._cmd_stamp < self.cmd_vel_timeout
                                  and (abs(self._cmd_linear) > 1e-2
                                       or abs(self._cmd_angular) > 1e-2)),
            'is_docking_done': dock_ok,
            'is_charging': bool(dock_ok and self._charging_enabled),
            'bumper_routing_enabled': True,
            'battery_gate_open': bool(s['battery_gate']),
            'press_module': bool(s['press_module']),
            'stop_triggered': bool(s['stop']) or self._estop_latched,
            'bumper_triggered': bool(s['bumper'] or s['bumper_l'] or s['bumper_r']),
            'left_bumper_triggered': bool(s['bumper_l']),
            'right_bumper_triggered': bool(s['bumper_r']),
            'rain_triggered': self._rain_flag(s),
            'lift_triggered': bool(s['lift']),
        }

    def _fill_sensor_info(self, msg):
        """Fill every field of ``msg`` from the cached MCU state.

        Every assignment goes through :func:`_set_if`, so a ``mower_interfaces`` whose field
        set differs from what is documented here still works instead of raising.
        """
        if hasattr(msg, 'header'):
            msg.header.stamp = self.get_clock().now().to_msg()
        s = self._sensor_info
        _set_if(msg, 'bumper_triggered', bool(s['bumper'] or s['bumper_l'] or s['bumper_r']))
        _set_if(msg, 'rain_triggered', self._rain_flag(s))
        if self._rain.value is not None:
            _set_if(msg, 'rain_sensor_value', max(0, min(0xFFFF, int(self._rain.value))))
        _set_if(msg, 'lift_triggered', bool(s['lift']))
        _set_if(msg, 'is_fill_light_on', bool(self._fill_light_on))
        _set_if(msg, 'stop_triggered', bool(s['stop']))
        _set_if(msg, 'battery_gate_open', bool(s['battery_gate']))
        _set_if(msg, 'press_module', bool(s['press_module']))
        _set_if(msg, 'cutter_size', int(s['cutter_size']))

        if self._battery:
            _set_if(msg, 'battery_temperature', int(self._battery['temperature']) & 0xFF)
            _set_if(msg, 'battery_error', int(self._battery['error']))
            _set_if(msg, 'is_charging', bool(self._battery['dock_ok']))

        for key, prefix in (('cutter', 'cutter_board_version'),
                            ('chassis', 'chassis_board_version'),
                            ('rtk', 'rtk_board_version')):
            if key in self._versions:
                major, minor, patch = self._versions[key]
                # Matches the live snapshot: "v0.6.36"/"v0.6.34" for cutter/chassis, "0.9.50"
                # for the rtk board.
                version = ('v' if key != 'rtk' else '') + '%d.%d.%d' % (major, minor, patch)
                _set_if(msg, prefix, version)

        now = time.monotonic()
        linear, angular, measured = self._speed(now)
        _set_if(msg, 'is_moving', measured
                and (abs(linear) > 1e-2 or abs(angular) > 1e-2))
        _set_if(msg, 'is_cmd_moving',
                now - self._cmd_stamp < self.cmd_vel_timeout
                and (abs(self._cmd_linear) > 1e-2 or abs(self._cmd_angular) > 1e-2))
        # Vendor DevStatus::judgmentCutterStatus: cutting == cutter rpm > 500.
        cutter = self._motors.get('cutter')
        _set_if(msg, 'is_cutting', bool(cutter and abs(cutter['speed']) > 500))
        _set_if(msg, 'is_docking_done', bool(self._battery and self._battery.get('dock_ok')))
        # TODO: key_pressed, bumper_routing_*,
        # is_docking_done and the MotorInfo sub-messages need sources that are not on this
        # bus (buttons on /dev/keyboard, fill light in the light node, /charging state).

        for name in MOTOR_NAMES:
            raw = self._motors.get(name)
            if raw is None:
                continue
            # wire name 'cutter' -> message field 'cutter_motor'
            motor_field = name + '_motor'
            try:
                motor = type(getattr(msg, motor_field))()
            except AttributeError:
                continue  # this interface package has no such field
            for field, value in raw.items():
                if field == 'status':
                    # MotorInfo.status is a MotorStatus sub-message on the bus
                    status_msg = getattr(motor, 'status', None)
                    if status_msg is not None and hasattr(status_msg, 'status'):
                        status_msg.status = value
                    continue
                if not _set_if(motor, field, value):
                    alias = _MOTOR_FIELD_ALIASES.get(field)
                    if alias:
                        _set_if(motor, alias, value)
            _set_if(msg, motor_field, motor)

    # ------------------------------------------------------------------- TX
    def _on_cmd_vel(self, msg):
        """Queue one SpeedData for this message and wake the I/O thread to send it."""
        now = self._mono()
        self._cmd_linear, self._cmd_angular = self._map_command(msg)
        self._cmd_stamp = now
        self._cmd_queue.append((self._cmd_linear, self._cmd_angular, now))
        try:
            os.write(self._wake_w, b'x')
        except (BlockingIOError, OSError):
            pass                                   # pipe full: a wake is already pending

    def _on_cmd_vel_stamped(self, msg):
        self._on_cmd_vel(msg.twist)

    def _drain_wake(self):
        try:
            return bool(os.read(self._wake_r, 4096))
        except (BlockingIOError, OSError):
            return False

    def _map_command(self, twist):
        """Scale + clamp ``/cmd_vel`` to the ``SpeedData`` that goes on the wire.

        This is the whole command path: no host PID, no brake (the vendor sends the raw
        ``float32(lin.x), float32(ang.z)``; we additionally clamp to +-``*_max``).
        """
        linear = float(twist.linear.x) * self.linear_scale
        angular = float(twist.angular.z) * self.angular_scale
        linear = max(-self.linear_max, min(self.linear_max, linear))
        angular = max(-self.angular_max, min(self.angular_max, angular))
        return linear, angular

    def _send_heartbeat(self):
        """Keepalive every ``heartbeat_period`` (100 ms, confirmed by the vendor tap)."""
        self._write(build_frame([(TYPE_HEARTBEAT, MOD_ALL, heartbeat_payload())]))

    @staticmethod
    def _is_zero(linear, angular):
        return abs(linear) <= 1e-6 and abs(angular) <= 1e-6

    def _send_speed_frame(self, linear, angular):
        self._write(build_frame([(TYPE_ROS_MOWER, MOD_SPEED, speed_payload(linear, angular))]))
        self._sent_speed_pub.publish(self._twist_msg(linear, angular))

    def _start_stop(self, now, reason):
        """First zero now, the remaining ``stop_frames - 1`` every ``stop_frame_spacing``."""
        if self._moving:
            self.get_logger().info('SpeedData stop (%s): %d zero frames, then silence'
                                   % (reason, self.stop_frames))
        self._moving = False
        self._send_speed_frame(0.0, 0.0)
        self._stop_left = self.stop_frames - 1
        self._stop_next_t = now + self.stop_frame_spacing

    def _speed_tick(self, now=None):
        """SpeedData TX state machine; runs in the I/O thread (wake + every 20 ms).

        * each queued ``/cmd_vel`` -> exactly one frame (vendor pass-through), except
          zeros while already stopped (idle = silence) and anything while interlocked;
        * zero after non-zero, ``cmd_vel_timeout`` silence after non-zero, or an interlock
          engaging while moving -> stop sequence (``stop_frames`` zeros), then silence;
        * after an interlock clears nothing is sent until a fresh non-zero command.
        """
        if now is None:
            now = self._mono()
        blocked = self._interlock_active() or self._estop_latched

        streaming = bool(self.get_parameter('speed_stream_enabled').value)
        if streaming != self._streaming:
            self._streaming = streaming
            self.get_logger().warn('SpeedData debug stream %s'
                                   % ('ENABLED (%.0f Hz)' % (1.0 / self._stream_period)
                                      if streaming else 'DISABLED (event-driven)'))
            if streaming:
                self._stream_next_t = now
            else:
                # Never leave a non-zero setpoint latched in the MCU when the stream stops.
                self._cmd_queue.clear()
                self._start_stop(now, 'debug stream disabled')
        if streaming:
            self._cmd_queue.clear()
            if now >= self._stream_next_t:
                self._stream_next_t = now + self._stream_period
                linear, angular = self._commanded()
                self._send_speed_frame(linear, angular)
                self._moving = not self._is_zero(linear, angular)
                self._last_nonzero_t = self._cmd_stamp
            self._publish_speed_telemetry(now)
            return

        while self._cmd_queue:
            linear, angular, stamp = self._cmd_queue.popleft()
            if blocked:
                self.get_logger().warn('cmd_vel dropped: interlock / e-stop active',
                                       throttle_duration_sec=5.0)
                continue
            if not self._is_zero(linear, angular):
                self._stop_left = 0                # a new motion command cancels a stop
                self._send_speed_frame(linear, angular)
                self._moving = True
                self._last_nonzero_t = stamp
            elif self._moving:
                self._start_stop(now, 'zero cmd_vel')
            # zero while stopped / stopping: send nothing

        if self._moving:
            if blocked:
                self._start_stop(now, 'interlock')
            elif now - self._last_nonzero_t > self.cmd_vel_timeout:
                self._start_stop(now, 'cmd_vel timeout')
        if self._stop_left > 0 and now >= self._stop_next_t:
            self._send_speed_frame(0.0, 0.0)
            self._stop_left -= 1
            self._stop_next_t += self.stop_frame_spacing
        self._publish_speed_telemetry(now)

    def _twist_msg(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        return msg

    def _publish_speed_telemetry(self, now):
        """Measured and commanded body velocity at ``speed_telemetry_rate_hz`` (observational)."""
        if self._speed_telemetry_period <= 0.0:
            return
        if now - self._speed_telemetry_t < self._speed_telemetry_period:
            return
        self._speed_telemetry_t = now
        self._meas_speed_pub.publish(self._twist_msg(self._meas_linear, self._meas_angular))
        cmd_linear, cmd_angular = self._commanded()
        self._cmd_speed_pub.publish(self._twist_msg(cmd_linear, cmd_angular))

    def _commanded(self):
        if self._interlock_active() or self._estop_latched:
            return 0.0, 0.0
        if self._mono() - self._cmd_stamp > self.cmd_vel_timeout:
            return 0.0, 0.0
        return self._cmd_linear, self._cmd_angular

    # --------------------------------------------------------------- safety
    def _interlock_active(self):
        """True while a configured safety input is asserted (lift / e-stop / bumper)."""
        s = self._sensor_info
        if not s:
            return False
        if self.interlock_on_lift and s['lift']:
            return True
        if self.interlock_on_stop and s['stop']:
            return True
        if self.interlock_on_bumper and (s['bumper'] or s['bumper_l'] or s['bumper_r']):
            return True
        return False

    def _apply_interlock(self):
        """Called on every SensorInfo: force the blade off while an interlock is active."""
        active = self._interlock_active() or self._estop_latched
        if active and not self._interlock_latched:
            self._interlock_latched = True
            self.get_logger().warn('interlock asserted (lift/stop/bumper/estop): wheels + cutter off')
            self._send_cutter(False, 0)
        elif not active and self._interlock_latched:
            self._interlock_latched = False
            self.get_logger().info('interlock released; cutter stays OFF until re-requested')
            self._cutter_requested_on = False
        now = time.monotonic()
        if active != self._estop_pub_value or now - self._estop_pub_t >= self._estop_period:
            self._estop_pub_value = active
            self._estop_pub_t = now
            self._estop_pub.publish(Bool(data=bool(active)))

    def _send_cutter(self, enable, speed, height_mm=None):
        """Send CutterControl. The height slot always carries an absolute deck height in mm
        (never 0, which would be out of range / drive the deck to its lowest point).

        ``height_mm`` given: clamp to 30-90 and remember it. Otherwise the remembered height
        is re-sent; with the blade OFF and ``cutter_off_height_mm`` > 0 that height is sent
        instead (without overwriting the remembered one). NOTE: the vendor's CutterOFF
        (behaviors_master/cutter_control.xml) sends 90; we hold the last height by default.
        Like the vendor (mower_bt_nodes CutterControl::onRunning) the height motor's
        enable/direction/speed are 0; only position matters.
        """
        if height_mm is not None:
            self._height_mm = clamp_height_mm(height_mm)
        h = self._height_mm
        if not enable and height_mm is None and self.cutter_off_height_mm > 0:
            h = clamp_height_mm(self.cutter_off_height_mm)
        payload = cutter_payload(enable, False, speed if enable else 0, False, False, h)
        self._cutter_cmd = payload
        self._write(build_frame([(TYPE_ROS_MOWER, MOD_CUTTER, payload)]))
        self._publish_height()

    def _publish_height(self):
        if self._height_mm != self._height_published:
            self._height_published = self._height_mm
            self._height_pub.publish(Int16(data=int(self._height_mm)))

    def _on_imu_forward_raw(self, data):
        if time.monotonic() - self._last_imu_fwd < 1.0 / max(self.forward_imu_rate, 1.0):
            return
        self._on_imu_forward(parse_imu(data) or deserialize_message(data, Imu))

    def _on_imu_forward(self, msg):
        """Relay the host IMU to the MCU as ``ImuData`` at <= forward_imu_rate Hz.

        ``forward_imu`` is re-read each call so the stream can be toggled live with
        `ros2 param set` while A/B-ing which TX stream drives wheel motion.
        """
        if not bool(self.get_parameter('forward_imu').value):
            return
        now = time.monotonic()
        if now - self._last_imu_fwd < 1.0 / max(self.forward_imu_rate, 1.0):
            return
        self._last_imu_fwd = now
        q = msg.orientation
        roll, pitch, yaw = _quat_to_rpy(q.x, q.y, q.z, q.w)
        roll = math.remainder(roll + self.forward_imu_roll_offset, 2.0 * math.pi)
        pitch = math.remainder(pitch + self.forward_imu_pitch_offset, 2.0 * math.pi)
        a, g = msg.linear_acceleration, msg.angular_velocity
        self._write(build_frame([(TYPE_ROS_MOWER, MOD_IMU,
                                  imu_payload(roll, pitch, yaw, (a.x, a.y, a.z), (g.x, g.y, g.z)))]))
        self._imu_fwd_count += 1
        if self._imu_fwd_count == 1:
            self.get_logger().info('forwarding host IMU to the MCU as ImuData (module 9)')

    def _on_estop_request(self, msg):
        if msg.data and not self._estop_latched:
            self._estop_latched = True
            self.get_logger().error('host e-stop requested: latching motion + cutter off')
            self._send_cutter(False, 0)
        self._apply_interlock()
        self._speed_tick()                         # stop sequence now if moving (no zero at rest)

    # --------------------------------------------------------------- services
    def _srv_cutter_control(self, req, resp):
        """mower_interfaces/CutterControl: {MotorControl cutter, MotorControl height}."""
        if self._interlock_active() or self._estop_latched:
            self.get_logger().warn('cutter_control refused: interlock active')
            resp.result = False
            return resp
        cutter_on = bool(req.cutter.enable)
        speed = int(req.cutter.speed) if req.cutter.speed else self.cutter_default_speed
        height = int(req.height.position) if req.height.enable else None
        self._cutter_requested_on = cutter_on
        if cutter_on:
            self._cutter_speed = speed
        self._send_cutter(cutter_on, speed, height)
        self.get_logger().info('cutter %s speed=%d height=%d mm%s'
                               % ('ON' if cutter_on else 'OFF', speed, self._height_mm,
                                  '' if height is not None else ' (held)'))
        resp.result = True
        return resp

    def _srv_set_height(self, req, resp):
        """mower_interfaces/SetCutterHeight: change only the deck height.

        NOTE /cutter_control is the vendor contract: cutter.enable=false there means blade
        OFF, so it cannot be used for a height-only change while mowing. This service
        re-sends the CURRENT blade state (on + its speed, or off) with the new height.
        """
        if self._interlock_active() or self._estop_latched:
            resp.ok = False
            resp.message = 'refused: interlock / e-stop latched'
            resp.height_mm = int(self._height_mm)
            self.get_logger().warn('set_height refused: interlock active')
            return resp
        h = clamp_height_mm(req.height_mm)
        on = bool(self._cutter_requested_on)
        self._send_cutter(on, self._cutter_speed or self.cutter_default_speed, h)
        resp.ok = True
        resp.height_mm = int(self._height_mm)
        resp.message = 'height %d mm (blade %s)' % (self._height_mm, 'on' if on else 'off')
        if h != int(req.height_mm):
            resp.message += ', clamped from %d' % int(req.height_mm)
        self.get_logger().info('set_height: ' + resp.message)
        return resp

    def _srv_cutter_off(self, req, resp):
        self._cutter_requested_on = False
        self._send_cutter(False, 0)
        resp.success = True
        resp.message = 'cutter off'
        return resp

    def _srv_charging(self, req, resp):
        self._charging_enabled = bool(req.enable_charging)
        self._write(build_frame([(TYPE_ROS_MOWER, MOD_CHARGE, charge_payload(self._charging_enabled))]))
        self.get_logger().info('charging %s' % ('enabled' if self._charging_enabled else 'disabled'))
        # mower_interfaces/ChargingControl has an EMPTY response; assigning resp.result raised
        # AttributeError, which killed mcu_node on every /charging call (respawned with the
        # flag reset). Set it only if a future interface version adds the field.
        _set_if(resp, 'result', True)
        return resp

    def _srv_clear_estop(self, req, resp):
        """Clear the e-stop: host-side latch plus the vendor's MCU clear frame.

        Recovered from ``mower_base_node`` ``clearEStopROS`` (native_decompile ... :19943): a
        ``SensorInfoControl`` sub-packet (module 10, 8 bytes, all zero) queued to the MCU.
        """
        self._estop_latched = False
        self._write(build_frame([(TYPE_ROS_MOWER, MOD_SENSOR, bytes(8))]))
        self.get_logger().info('host e-stop latch cleared; MCU clear frame (module 10, 8x00) sent')
        self._apply_interlock()
        return resp

    def _speed(self, now):
        """Best available body velocity as ``(linear, angular, measured)``.

        Prefers the MCU's own ``SpeedData`` estimate; once that goes stale we degrade to the
        last commanded velocity (and report ``measured=False`` so callers can flag it).
        """
        if now - self._meas_stamp < self.speed_timeout:
            return self._meas_linear, self._meas_angular, True
        if now - self._cmd_stamp < self.cmd_vel_timeout:
            return self._cmd_linear, self._cmd_angular, False  # degraded: commanded
        return 0.0, 0.0, False

    # ------------------------------------------------------------- odometry
    def _reckon(self):
        """Dead-reckon and publish ``/odom``.

        Integration of ``SpeedData`` (linear m/s + angular rad/s, the MCU's own estimate of
        the body motion).  Heading prefers the MCU ``ImuData`` yaw when it is fresh; if no
        IMU data ever arrives we simply integrate the angular rate instead, and if neither
        measurement is fresh we fall back to the *commanded* velocity and say so through a
        inflated twist covariance.
        """
        now = time.monotonic()
        dt = now - self._last_reckon
        self._last_reckon = now
        if dt <= 0.0:
            return
        dt = min(dt, 0.5)  # do not jump across a scheduling hiccup

        linear, angular, measured = self._speed(now)

        # 1) translate along the previous heading
        self._x += linear * math.cos(self._yaw) * dt
        self._y += linear * math.sin(self._yaw) * dt
        # 2) heading: absolute MCU yaw if available, else integrate the yaw rate
        if self.use_imu_yaw and self._imu_yaw is not None and now - self._imu_stamp < 1.0:
            self._yaw = self._imu_yaw  # TODO: validate WIT yaw sign / axis on hardware
        else:
            self._yaw = self._yaw + angular * dt
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

        _, _, qz, qw = _yaw_to_quaternion(self._yaw)
        twist_cov = _TWIST_COVARIANCE_MEASURED if measured else _TWIST_COVARIANCE_COMMANDED
        if self._fast_odom:
            self._odom_pub.publish(odometry_bytes(
                self._stamp_ns(), self.odom_frame, self.base_frame,
                (self._x, self._y, 0.0), (0.0, 0.0, qz, qw),
                (linear, 0.0, 0.0), (0.0, 0.0, angular), _POSE_COVARIANCE, twist_cov))
            return
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = _POSE_COVARIANCE
        msg.twist.twist.linear.x = linear
        msg.twist.twist.angular.z = angular
        msg.twist.covariance = (_TWIST_COVARIANCE_MEASURED if measured
                                else _TWIST_COVARIANCE_COMMANDED)
        self._odom_pub.publish(msg)

    # -------------------------------------------------------------- shutdown
    def shutdown(self):
        if self._ser is not None:
            # Politely stop the wheels and the blade before dropping the link.
            self._write(build_frame([(TYPE_ROS_MOWER, MOD_CUTTER, cutter_payload(False, False, 0, False, False, 0))]))
            self._write(build_frame([(TYPE_ROS_MOWER, MOD_SPEED, speed_payload(0.0, 0.0))]))
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None


_SENSOR_NAMES = ('bumper', 'rain', 'lift', 'stop', 'power_off', 'battery_gate',
                 'cutter_size', 'press_module', 'bumper_r', 'bumper_l')


def _set_if(msg, name, value):
    """Set ``msg.name = value`` only when the interface has that field.

    Returns ``True`` when the assignment happened.  ``TypeError`` (field present but with a
    different type, e.g. a nested message where we have a raw int) is swallowed: the interface
    package is being written by another agent and its exact shape is not frozen yet.
    """
    if not name or not hasattr(msg, name):
        return False
    try:
        setattr(msg, name, value)
    except TypeError:
        return False
    return True


# Raw MotorInfo names -> names used by mower_interfaces/MotorInfo (vendor msg uses speed_rpm).
# The vendor MotorInfo nests a MotorStatus message; the wire int8 is copied into .status.
_MOTOR_FIELD_ALIASES = {'speed': 'speed_rpm'}


# Dead reckoning covariances (row-major 6x6).  Unobserved axes are marked with 1e6.
_POSE_COVARIANCE = _diag6(0.05, 0.05, 0.10, 0.50, 0.50, 0.10)
_TWIST_COVARIANCE_MEASURED = _diag6(0.02, 1e6, 1e6, 1e6, 1e6, 0.05)
_TWIST_COVARIANCE_COMMANDED = _diag6(1.00, 1e6, 1e6, 1e6, 1e6, 1.00)


def main(args=None):
    try:  # crash records -> /userdata/ros2/crashes (docs/crash_recovery.md)
        from mower_control.crash_record import install as _install_crash_record
        _install_crash_record('mower_mcu_driver')
    except ImportError:
        pass
    rclpy.init(args=args)
    node = McuNode()
    node.start_io()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_io()
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
