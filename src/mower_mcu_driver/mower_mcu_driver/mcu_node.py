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
``/cmd_vel``            geometry_msgs/TwistStamped         -> ``SpeedData`` (MODULE_SPEED, 4)

Parameters
----------
``port`` ``/dev/serial_mower``   serial device (loopback test: a socat pty)
``baud`` ``115200``              serial baud rate
``odom_frame`` ``odom`` / ``base_frame`` ``base_link`` / ``imu_frame`` ``imu_link`` / ``frame_id`` ``base_link``
``heartbeat_period`` ``0.1``     **period is UNKNOWN on hardware** -- see TODO below
``speed_cmd_rate`` ``20.0``      Hz at which ``SpeedData`` commands are streamed to the MCU
``cmd_vel_timeout`` ``0.5``      s before a stale ``/cmd_vel`` is replaced by a zero command
``speed_timeout`` ``0.5``        s before measured ``SpeedData`` is considered stale
``odom_rate`` ``50.0``           Hz for ``/odom``
``use_imu_yaw`` ``true``         use MCU ``ImuData`` yaw for heading (falls back to integration)
``linear_scale`` ``1.0`` / ``angular_scale`` ``1.0``   host-side ``/cmd_vel`` -> ``SpeedData`` map
``linear_max`` ``1.5`` / ``angular_max`` ``1.5``       clamp (m/s, rad/s)
``battery_voltage_scale`` ``0.1``  raw 0.1 V units -> volts (set 1.0 to mimic the stock node)
``battery_current_scale`` ``1.0``  raw -> amperes (**unit still unknown (?)**)

Known gaps -- tracked as TODOs, do not treat this driver as safety-complete yet
-------------------------------------------------------------------------------
* **PID TODO.** The stock node runs a host-side wheel PID from ``libpid_controller.so``
  (gains in ``mower_base_pkg/config/base.yaml``: linear kp/ki/kd 0.9/0.3/1.0, angular
  0.9/0.3/0.5, both clamped to +-0.3) while the MCU only accepts linear/angular.  Until that
  library is ported or reverse engineered, this node applies a plain linear/angular
  scale+clamp mapping (see :meth:`McuNode._map_command`).  It is *not* equivalent.
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
import fcntl
import math
import os
import select
import struct
import termios
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu

try:  # Optional: only exists once ros2_stack/src/mower_interfaces has been created.
    from mower_interfaces.msg import MowerSensorInfo
except ImportError:  # pragma: no cover - depends on workspace build order
    MowerSensorInfo = None


# ---------------------------------------------------------------------------
# Framing (constants and codec mirror mcu_frame_decode.py)
# ---------------------------------------------------------------------------
SOF, EOF, ESC = 0xA5, 0x5A, 0xF5
ESC_MAP = {0x01: SOF, 0x02: EOF, 0x03: ESC}

TYPE_ROS_MOWER = 0      # host -> MCU
TYPE_MOWER_ROS = 8      # MCU -> host
TYPE_HEARTBEAT = 255    # host -> MCU keepalive

MOD_ALL = 0           # MODULE_ALL: addressed to the whole MCU (heartbeat uses this)
MOD_CHARGE = 3        # not sent yet: /charging service -> ChargeControl is TODO
MOD_SPEED = 4
MOD_BATTERY = 5
MOD_IMU = 9
MOD_SENSOR = 10
MOD_CUTTER = 11       # not sent yet: /cutter_control service -> CutterControl is TODO
MOD_VERSION = 12
MOD_MOTORS = 13
MOD_CALIB = 18
MOD_LOG = 19
MOD_BMS_VERSION = 24

# struct formats, from protocol_definitions_recovered.h (all little-endian, packed)
SPEED_FMT = '<ff'                 # SpeedData (8):  linear m/s, angular rad/s
BATTERY_FMT = '<HhBbbI'           # BatteryInfo (11)
SENSOR_FMT = '<10B'               # SensorInfo (10)
IMU_FMT = '<9h'                   # ImuData (18) - pitch, roll, yaw, accx..z, gyrox..z
VERSION_FMT = '<9B'               # VersionInfo (9): cutter/chassis/rtk major,minor,patch
BMS_FMT = '<3B'                   # BMS Version (3)
CALIB_FMT = '<3B'                 # MCCalib (3)
MOTOR_FMT = '<hhhbB'              # MotorInfo (8): speed, current, voltage, temperature, status
HEARTBEAT_FMT = '<8B'             # year, month, day, hour, minute, second, ms_h, ms_l
MOTOR_NAMES = ('cutter', 'left', 'right', 'height')

# WIT-Motion JY61P scaling convention used by ImuData (marked (?) in the recovered header:
# angle*32768/180, acc*32768/16 g, gyro*32768/2000 dps).
WIT_RAW_ANGLE_TO_RAD = math.radians(180.0) / 32768.0
WIT_RAW_GYRO_TO_RAD = math.radians(2000.0) / 32768.0
WIT_RAW_ACC_TO_MS2 = 16.0 * 9.80665 / 32768.0


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


def speed_payload(linear, angular):
    """Build the 8-byte ``SpeedData`` payload (m/s, rad/s)."""
    return struct.pack(SPEED_FMT, float(linear), float(angular))


def _diag6(a, b, c, d, e, f):
    """Row-major 6x6 covariance matrix with ``a..f`` on the diagonal."""
    cov = [0.0] * 36
    for idx, val in enumerate((a, b, c, d, e, f)):
        cov[idx * 6 + idx] = val
    return cov


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
        # TODO(unknown): the real heartbeat period / MCU failsafe timeout were never recovered.
        self.declare_parameter('heartbeat_period', 0.1)
        self.declare_parameter('speed_cmd_rate', 20.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('speed_timeout', 0.5)
        self.declare_parameter('odom_rate', 50.0)
        self.declare_parameter('use_imu_yaw', True)
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('linear_max', 1.5)
        self.declare_parameter('angular_max', 1.5)
        self.declare_parameter('battery_voltage_scale', 0.1)
        self.declare_parameter('battery_current_scale', 1.0)

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
        speed_cmd_rate = float(param('speed_cmd_rate'))
        odom_rate = float(param('odom_rate'))

        # ---- state --------------------------------------------------------
        self._ser = None
        self._last_open_attempt = 0.0
        self._parser = FrameParser()

        self._cmd_linear = 0.0            # last /cmd_vel, already mapped + clamped
        self._cmd_angular = 0.0
        self._cmd_stamp = 0.0             # monotonic time of the last /cmd_vel

        self._meas_linear = 0.0           # last measured SpeedData from the MCU
        self._meas_angular = 0.0
        self._meas_stamp = 0.0

        self._imu_yaw = None              # last MCU ImuData heading (rad)
        self._imu_stamp = 0.0
        self._have_imu = False            # set once MODULE_IMU was seen at all

        self._x = 0.0                     # dead-reckoned pose
        self._y = 0.0
        self._yaw = 0.0
        self._last_reckon = time.monotonic()

        self._sensor_info = None          # last SensorInfo, cached for /mower_sensor_info
        self._versions = {}               # 'cutter'/'chassis'/'rtk' -> (major, minor, patch)
        self._battery = None              # last BatteryInfo dict
        self._motors = {}                 # motor name -> dict of raw values
        self._mower_sensor_info_pub_warned = False

        # ---- publishers / subscribers -------------------------------------
        sensor_qos = QoSProfile(depth=10,
                                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                history=QoSHistoryPolicy.KEEP_LAST)

        self._battery_pub = self.create_publisher(BatteryState, '/battery', 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._imu_pub = self.create_publisher(Imu, '/imu', sensor_qos)

        # /mower_sensor_info only if the interface package has been created; otherwise skip.
        self._sensor_pub = None
        if MowerSensorInfo is not None:
            self._sensor_pub = self.create_publisher(MowerSensorInfo, '/mower_sensor_info', 10)
            self.get_logger().info('/mower_sensor_info enabled (mower_interfaces found)')
        else:
            self.get_logger().warn(
                'mower_interfaces not available -> /mower_sensor_info disabled '
                '(build ros2_stack/src/mower_interfaces and relaunch to enable it)')

        # TODO(estop): passthrough not wired yet, see module docstring.
        # TODO: the stock node also accepts unstamped geometry_msgs/Twist on /cmd_vel.
        self.create_subscription(TwistStamped, '/cmd_vel', self._on_cmd_vel, 10)

        # ---- timers -------------------------------------------------------
        # Default single-threaded executor: all timers/callbacks touch the serial port, so no
        # locking is needed as long as rclpy.spin() (not a MultiThreadedExecutor) is used.
        self.create_timer(0.01, self._poll_serial)                       # 100 Hz RX poll
        self.create_timer(self.heartbeat_period, self._send_heartbeat)
        self.create_timer(1.0 / max(speed_cmd_rate, 1.0), self._send_speed)
        self.create_timer(1.0 / max(odom_rate, 1.0), self._reckon)

        self.get_logger().info(
            'mower_mcu_driver up: %s @ %d baud, heartbeat every %.0f ms (period UNKNOWN), '
            'cmd_vel %.0f Hz, odom %.0f Hz'
            % (self.port, self.baud, self.heartbeat_period * 1000.0,
               speed_cmd_rate, odom_rate))

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
        # TODO(unknown): BatteryInfo.current units are unrecovered - mA? 0.1 A?  raw for now.
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
        msg.orientation_covariance = _diag6(0.05, 0.05, 0.05, 0.0, 0.0, 0.1)
        msg.angular_velocity.x = gyrox * WIT_RAW_GYRO_TO_RAD
        msg.angular_velocity.y = gyroy * WIT_RAW_GYRO_TO_RAD
        msg.angular_velocity.z = gyroz * WIT_RAW_GYRO_TO_RAD
        msg.angular_velocity_covariance = _diag6(0.02, 0.02, 0.02, 0.0, 0.0, 0.02)
        msg.linear_acceleration.x = accx * WIT_RAW_ACC_TO_MS2
        msg.linear_acceleration.y = accy * WIT_RAW_ACC_TO_MS2
        msg.linear_acceleration.z = accz * WIT_RAW_ACC_TO_MS2
        msg.linear_acceleration_covariance = _diag6(0.1, 0.1, 0.1, 0.0, 0.0, 0.0)
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
        names = ('bumper', 'rain', 'lift', 'stop', 'power_off', 'battery_gate',
                 'cutter_size', 'press_module', 'bumper_r', 'bumper_l')
        self._sensor_info = dict(zip(names, struct.unpack(SENSOR_FMT, payload)))
        self._publish_sensor_info()

    def _on_version(self, payload):
        if len(payload) != struct.calcsize(VERSION_FMT):
            return
        vals = struct.unpack(VERSION_FMT, payload)
        for idx, key in enumerate(('cutter', 'chassis', 'rtk')):
            self._versions[key] = tuple(vals[idx * 3:idx * 3 + 3])

    def _on_motors(self, payload):
        if len(payload) != struct.calcsize(MOTOR_FMT) * len(MOTOR_NAMES):
            return
        fields = ('speed', 'current', 'voltage', 'temperature', 'status')
        for idx, name in enumerate(MOTOR_NAMES):
            chunk = payload[idx * 8:(idx + 1) * 8]
            self._motors[name] = dict(zip(fields, struct.unpack(MOTOR_FMT, chunk)))

    def _publish_sensor_info(self):
        """Publish ``/mower_sensor_info`` if the interface package exists.

        Split into "build" + :meth:`_fill_sensor_info` so a malformed message can never take
        the timer callback down: a warning is logged once and publishing simply stops.
        """
        if self._sensor_pub is None or self._sensor_info is None or MowerSensorInfo is None:
            return
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

    def _fill_sensor_info(self, msg):
        """Fill every field of ``msg`` from the cached MCU state.

        Every assignment goes through :func:`_set_if`, so a ``mower_interfaces`` whose field
        set differs from what is documented here still works instead of raising.
        """
        if hasattr(msg, 'header'):
            msg.header.stamp = self.get_clock().now().to_msg()
        s = self._sensor_info
        _set_if(msg, 'bumper_triggered', bool(s['bumper'] or s['bumper_l'] or s['bumper_r']))
        _set_if(msg, 'rain_triggered', bool(s['rain']))
        _set_if(msg, 'lift_triggered', bool(s['lift']))
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
        # Heuristic: cutter spinning == cutting (no explicit cutter state on this bus yet).
        cutter = self._motors.get('cutter')
        _set_if(msg, 'is_cutting', bool(cutter and abs(cutter['speed']) > 0))
        # TODO: key_pressed, is_fill_light_on, rain_sensor_value, bumper_routing_*,
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
                if not _set_if(motor, field, value):
                    alias = _MOTOR_FIELD_ALIASES.get(field)
                    if alias:
                        _set_if(motor, alias, value)
            _set_if(msg, motor_field, motor)

    # ------------------------------------------------------------------- TX
    def _on_cmd_vel(self, msg):
        self._cmd_linear, self._cmd_angular = self._map_command(msg.twist)
        self._cmd_stamp = time.monotonic()

    def _map_command(self, twist):
        """Simple host-side ``/cmd_vel`` -> ``SpeedData`` mapping.

        TODO(port): the stock node runs a real controller here (``libpid_controller.so``,
        gains in ``mower_base_pkg/config/base.yaml``: linear 0.9/0.3/1.0, angular 0.9/0.3/0.5,
        both clamped to +-0.3).  Until that is ported this is only a scale + clamp; expect
        different dynamics and no wheel-level closed loop.
        """
        linear = float(twist.linear.x) * self.linear_scale
        angular = float(twist.angular.z) * self.angular_scale
        linear = max(-self.linear_max, min(self.linear_max, linear))
        angular = max(-self.angular_max, min(self.angular_max, angular))
        return linear, angular

    def _send_heartbeat(self):
        """Keepalive.  Fixed period by parameter because the real one is UNKNOWN (see doc)."""
        self._write(build_frame([(TYPE_HEARTBEAT, MOD_ALL, heartbeat_payload())]))

    def _send_speed(self):
        """Stream the current ``SpeedData`` command (zeros once /cmd_vel goes stale).

        Streaming zeros instead of stopping the stream makes a stale command fail safe on
        our side; the MCU additionally has its own (uncharacterised) command timeout.
        """
        linear, angular = self._commanded()
        self._write(build_frame([(TYPE_ROS_MOWER, MOD_SPEED, speed_payload(linear, angular))]))

    def _commanded(self):
        if time.monotonic() - self._cmd_stamp > self.cmd_vel_timeout:
            return 0.0, 0.0
        return self._cmd_linear, self._cmd_angular

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

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        _, _, qz, qw = _yaw_to_quaternion(self._yaw)
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
            # Politely stop the wheels before dropping the link.
            self._write(build_frame([(TYPE_ROS_MOWER, MOD_SPEED, speed_payload(0.0, 0.0))]))
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None


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
# TODO: the vendor MotorInfo also nests a MotorStatus message where the wire format has a
# single uint8; that field needs a converter once the interface package is final.
_MOTOR_FIELD_ALIASES = {'speed': 'speed_rpm'}


# Dead reckoning covariances (row-major 6x6).  Unobserved axes are marked with 1e6.
_POSE_COVARIANCE = _diag6(0.05, 0.05, 0.10, 0.50, 0.50, 0.10)
_TWIST_COVARIANCE_MEASURED = _diag6(0.02, 1e6, 1e6, 1e6, 1e6, 0.05)
_TWIST_COVARIANCE_COMMANDED = _diag6(1.00, 1e6, 1e6, 1e6, 1e6, 1.00)


def main(args=None):
    rclpy.init(args=args)
    node = McuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
