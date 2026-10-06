#!/usr/bin/env python3
"""Slew-rate limiter for ``/cmd_vel`` with a command watchdog (ROS 2 Humble).

Port of the MowgliNext ``cmd_vel_slew.hpp`` pattern -- see
``ros2_stack/docs/mowglinext_baseline.md`` section 4.3, item 9: shape the
commanded twist *before* it reaches the firmware so the MCU never sees a step
change in linear/angular velocity, and echo the exact applied command for
rosbag analysis.

Subscribed topics
-----------------
``/cmd_vel_raw``   geometry_msgs/Twist          unshaped command in (twist_mux,
                                                     teleop, Nav2, ...)

Published topics
---------------
``/cmd_vel``       geometry_msgs/Twist          slew-limited command out; this
                                                     is what ``mower_mcu_driver``
                                                     subscribes to

Parameters
----------
``max_linear_accel``   ``0.5``    m/s^2 cap on linear.x acceleration
``max_angular_accel``  ``1.0``    rad/s^2 cap on angular.z acceleration
``cmd_timeout``        ``0.5``    s after the last ``/cmd_vel_raw`` the output is
                                  forced to exactly zero (immediate exact stop,
                                  MowgliNext style) and held there until a fresh
                                  command arrives
``rate``               ``20.0``   Hz output/update rate
``frame_id``           ``base_link``  frame stamped on the output
``applied_log_period`` ``1.0``    s between ``cmd_vel applied`` log lines
                                  (0 = every tick)

Straight-line shaping (all live-settable, defaults neutral = no behaviour change;
see :class:`DriveShaper`). Applied to the *target* angular.z before slewing:

``angular_trim_radps``     ``0.0``   rad/s added to angular.z while |linear.x| > 0.02,
                                     signed with linear.x (a wheel-torque imbalance
                                     pulls the other way when reversing)
``angular_deadband_radps`` ``0.0``   |angular.z| below this is zeroed while
                                     |linear.x| > 0.02 (joystick drift)
``heading_hold``           ``false`` lock the IMU yaw when the angular command goes to
                                     ~0 while driving (|linear.x| > 0.05) and steer
                                     back onto it
``heading_hold_kp``        ``1.0``   rad/s per rad of yaw error
``heading_hold_kd``        ``0.1``   rad/s per rad/s of gyro z (damping)
``heading_hold_max_radps`` ``0.15``  clamp on the heading-hold correction
``robot_settings_file``    ``$MOWER_ROBOT_SETTINGS_FILE`` or
                           ``/work/config/gui/mowgli_robot.yaml``: the GUI
                                     settings yaml; the keys above found there
                                     override the defaults at startup (the GUI
                                     pushes them live on save; the launch file does
                                     not pass parameters to this node)

Extra input: ``/imu/data`` (sensor_msgs/Imu, sampled once per tick; only used by
heading hold, which releases when the IMU is older than 0.3 s).

Behaviour
---------
* Fixed-rate loop.  Each tick moves the applied linear.x / angular.z at most
  ``accel * dt`` toward the target; every other twist component passes through
  unchanged.
* Every published command is echoed to the log at full float precision (the
  MowgliNext ``/cmd_vel_applied`` idea, as a log line instead of a topic),
  throttled to ``applied_log_period`` (default 1 s; 0 = every tick, the old
  20 Hz echo). Silence with ``--ros-args --log-level cmd_vel_slew:=warn``.
* Watchdog: when no ``/cmd_vel_raw`` has arrived within ``cmd_timeout``, the
  applied velocity is set to exactly 0.0 (not slewed) and zero is published
  every tick until a fresh command arrives; recovery then slews up from zero.

CPU (RK3588): ``/cmd_vel_raw`` is a sampled input of a ``SubscriptionPump``
read once per tick with its receive time (no executor wake per message), the
loop runs on a ``PeriodicRunner`` thread instead of an rclpy timer, and
``/cmd_vel`` is published as pre-serialized Twist bytes. Idle cost went from
~12 % of a core to ~1 %.
"""

import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu

from mower_control.fast_msgs import parse_twist, twist_bytes
from mower_control.sub_pump import PeriodicRunner, SubscriptionPump, parse_imu


def slew(current, target, max_accel, dt):
    """Move ``current`` toward ``target`` by at most ``max_accel * dt``."""
    delta = target - current
    max_delta = max_accel * dt
    if abs(delta) <= max_delta:
        return target
    return current + math.copysign(max_delta, delta)


def wrap_angle(a):
    """``a`` wrapped to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# Straight-line shaping parameters: name -> (default, type). The names double as
# the GUI settings keys (mowgli_robot.yaml; gui pkg/api/param_bindings.go).
SHAPER_PARAMS = {
    'angular_trim_radps': (0.0, float),
    'angular_deadband_radps': (0.0, float),
    'heading_hold': (False, bool),
    'heading_hold_kp': (1.0, float),
    'heading_hold_kd': (0.1, float),
    'heading_hold_max_radps': (0.15, float),
}

TRIM_MIN_LINEAR = 0.02      # m/s: trim / deadband only while actually driving
HOLD_MIN_LINEAR = 0.05      # m/s: heading hold only above this
HOLD_ZERO_ANGULAR = 1e-3    # rad/s: "zero" angular command when no deadband is set
IMU_MAX_AGE = 0.3           # s: heading hold releases on an older IMU sample


class DriveShaper:
    """Trim, deadband and heading hold on the commanded angular.z (pure logic)."""

    def __init__(self, **params):
        for name, (default, _kind) in SHAPER_PARAMS.items():
            setattr(self, name, params.get(name, default))
        self.yaw_ref = None     # locked yaw while heading hold is engaged

    @property
    def holding(self):
        return self.yaw_ref is not None

    def reset(self):
        self.yaw_ref = None

    def shape(self, lin, ang, yaw=None, gyro_z=0.0):
        """Shaped angular.z for target (lin, ang); ``yaw`` None = no fresh IMU."""
        driving = abs(lin) > TRIM_MIN_LINEAR
        deadband = max(0.0, self.angular_deadband_radps)
        straight = abs(ang) < (deadband if deadband > 0.0 else HOLD_ZERO_ANGULAR)
        if driving and deadband > 0.0 and abs(ang) < deadband:
            ang = 0.0
        if (self.heading_hold and straight and abs(lin) > HOLD_MIN_LINEAR
                and yaw is not None):
            if self.yaw_ref is None:
                self.yaw_ref = yaw
            limit = abs(self.heading_hold_max_radps)
            corr = (self.heading_hold_kp * wrap_angle(self.yaw_ref - yaw)
                    - self.heading_hold_kd * gyro_z)
            ang = max(-limit, min(limit, corr))
        else:
            self.yaw_ref = None
        if driving and self.angular_trim_radps:
            ang += math.copysign(self.angular_trim_radps, lin)
        return ang


def default_settings_file():
    return os.environ.get('MOWER_ROBOT_SETTINGS_FILE', '/work/config/gui/mowgli_robot.yaml')


def settings_overrides(path):
    """{param: value} of the SHAPER_PARAMS keys found in the GUI settings yaml."""
    if not path:
        return {}
    try:
        import yaml
        with open(path, encoding='utf-8') as f:
            doc = yaml.safe_load(f) or {}
    except Exception:  # noqa: BLE001 - missing/unreadable file: defaults apply
        return {}
    flat = {}
    if isinstance(doc, dict):
        for section in doc.values():
            params = section.get('ros__parameters') if isinstance(section, dict) else None
            if isinstance(params, dict):
                flat.update(params)
    out = {}
    for name, (_default, kind) in SHAPER_PARAMS.items():
        value = flat.get(name)
        if kind is bool and isinstance(value, bool):
            out[name] = value
        elif kind is float and isinstance(value, (int, float)) and not isinstance(value, bool):
            out[name] = float(value)
    return out


class CmdVelSlewNode(Node):
    """Slew-limits ``/cmd_vel_raw`` -> ``/cmd_vel`` and enforces the watchdog."""

    def __init__(self):
        super().__init__('cmd_vel_slew')

        self.declare_parameter('max_linear_accel', 0.5)
        self.declare_parameter('max_angular_accel', 1.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('applied_log_period', 1.0)
        self.declare_parameter('robot_settings_file', default_settings_file())
        for name, (default, _kind) in SHAPER_PARAMS.items():
            self.declare_parameter(name, default)
        overrides = settings_overrides(str(self.get_parameter('robot_settings_file').value))
        if overrides:
            self.set_parameters([Parameter(k, value=v)
                                 for k, v in overrides.items()])
        self._shaper = DriveShaper(**{n: self.get_parameter(n).value for n in SHAPER_PARAMS})
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self._imu = None              # (yaw, gyro_z) of the newest /imu/data
        self._imu_time = None

        self._max_linear_accel = float(self.get_parameter('max_linear_accel').value)
        self._max_angular_accel = float(self.get_parameter('max_angular_accel').value)
        self._cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self._rate = float(self.get_parameter('rate').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._log_period = float(self.get_parameter('applied_log_period').value)

        # Applied (slewed) state, the last raw target, watchdog bookkeeping (monotonic s).
        self._cur_lin = 0.0
        self._cur_ang = 0.0
        self._tgt = None              # (lx, ly, lz, ax, ay, az) of the last /cmd_vel_raw
        self._last_cmd_time = None
        self._last_tick = None
        self._next_log = 0.0

        qos = QoSProfile(depth=10,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST)
        # Sampled latest-value input: only the newest command matters each tick, and its
        # receive time (not the poll time) feeds the watchdog.
        self._pump = SubscriptionPump(self, 'cmd_vel_slew_inputs')
        self._pump.subscribe(Twist, '/cmd_vel_raw', self._on_cmd_vel_raw,
                             QoSProfile(depth=1,
                                        reliability=QoSReliabilityPolicy.RELIABLE,
                                        history=QoSHistoryPolicy.KEEP_LAST),
                             parser=parse_twist, sampled=True, with_receipt=True)
        self._pump.subscribe(Imu, '/imu/data', self._on_imu,
                             QoSProfile(depth=1,
                                        reliability=QoSReliabilityPolicy.RELIABLE,
                                        history=QoSHistoryPolicy.KEEP_LAST),
                             parser=parse_imu, sampled=True, with_receipt=True)
        self._pump.start()
        self._pub = self.create_publisher(Twist, '/cmd_vel', qos)

        self._runner = PeriodicRunner(self, [(1.0 / self._rate, self._tick)], 'cmd_vel_slew')
        self._runner.start()
        self.get_logger().info(
            'cmd_vel_slew ready: max_linear_accel=%.3f m/s^2, '
            'max_angular_accel=%.3f rad/s^2, cmd_timeout=%.3f s, rate=%.1f Hz'
            % (self._max_linear_accel, self._max_angular_accel,
               self._cmd_timeout, self._rate))
        self.get_logger().info('cmd_vel_slew shaping: %s%s' % (
            {n: getattr(self._shaper, n) for n in SHAPER_PARAMS},
            ' (from %s)' % sorted(overrides) if overrides else ''))

    def shutdown(self):
        self._runner.stop()
        self._pump.stop()

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def _on_cmd_vel_raw(self, msg, receipt):
        """Store the raw target; the periodic loop does the slewing/publishing."""
        a, b = msg.linear, msg.angular
        self._tgt = (a.x, a.y, a.z, b.x, b.y, b.z)
        self._last_cmd_time = receipt

    def _on_imu(self, msg, receipt):
        q = msg.orientation
        self._imu = (yaw_from_quaternion(q.x, q.y, q.z, q.w), msg.angular_velocity.z)
        self._imu_time = receipt

    def _on_set_parameters(self, params):
        for p in params:
            if p.name not in SHAPER_PARAMS:
                continue
            kind = SHAPER_PARAMS[p.name][1]
            value = p.value
            if kind is float and isinstance(value, int) and not isinstance(value, bool):
                value = float(value)
            if not isinstance(value, kind) or (kind is float and not math.isfinite(value)):
                return SetParametersResult(successful=False,
                                           reason='%s must be a %s' % (p.name, kind.__name__))
        for p in params:
            if p.name in SHAPER_PARAMS:
                kind = SHAPER_PARAMS[p.name][1]
                setattr(self._shaper, p.name, kind(p.value))
                if p.name == 'heading_hold' and not p.value:
                    self._shaper.reset()
                self.get_logger().info('cmd_vel_slew: %s = %r' % (p.name, p.value))
        return SetParametersResult(successful=True)

    def _tick(self):
        self._pump.poll()
        out = self.step(time.monotonic())
        if out is not None:
            self._pub.publish(twist_bytes(*out))

    def step(self, now):
        """One loop iteration at monotonic time ``now``; returns the Twist tuple to publish."""
        if self._last_tick is None:
            self._last_tick = now
            return None
        dt = now - self._last_tick
        self._last_tick = now
        if dt <= 0.0:
            return None

        stale = (self._last_cmd_time is None or
                 now - self._last_cmd_time > self._cmd_timeout)

        if stale:
            # Immediate exact stop (MowgliNext cmd_vel_slew): no slewing to zero.
            self._cur_lin = 0.0
            self._cur_ang = 0.0
            tgt_lin, tgt_ang = 0.0, 0.0
            if getattr(self, '_shaper', None) is not None:
                self._shaper.reset()
        else:
            tgt_lin = self._tgt[0]
            tgt_ang = self._tgt[5]
            shaper = getattr(self, '_shaper', None)
            if shaper is not None:
                imu_time = getattr(self, '_imu_time', None)
                fresh = imu_time is not None and now - imu_time <= IMU_MAX_AGE
                yaw, gyro_z = self._imu if fresh else (None, 0.0)
                was = shaper.holding
                tgt_ang = shaper.shape(tgt_lin, tgt_ang, yaw, gyro_z)
                if shaper.holding != was:
                    self.get_logger().info('heading hold %s (yaw_ref=%s)' % (
                        'engaged' if shaper.holding else 'released', shaper.yaw_ref))
            self._cur_lin = slew(self._cur_lin, tgt_lin, self._max_linear_accel, dt)
            self._cur_ang = slew(self._cur_ang, tgt_ang, self._max_angular_accel, dt)

        # Pass through every component we do not shape.
        src = self._tgt if self._tgt is not None else (0.0,) * 6
        out = (self._cur_lin, src[1], src[2], src[3], src[4], self._cur_ang)

        # Exact echo: full float precision, throttled (see applied_log_period).
        if now >= self._next_log:
            self._next_log = now + self._log_period
            self.get_logger().info(
                'cmd_vel applied: linear.x={!r} angular.z={!r} '
                '(target linear.x={!r} angular.z={!r})'.format(
                    self._cur_lin, self._cur_ang, tgt_lin, tgt_ang))
        return out

    _slew = staticmethod(slew)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSlewNode()
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
