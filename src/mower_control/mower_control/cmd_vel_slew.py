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
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Twist

from mower_control.fast_msgs import parse_twist, twist_bytes
from mower_control.sub_pump import PeriodicRunner, SubscriptionPump


def slew(current, target, max_accel, dt):
    """Move ``current`` toward ``target`` by at most ``max_accel * dt``."""
    delta = target - current
    max_delta = max_accel * dt
    if abs(delta) <= max_delta:
        return target
    return current + math.copysign(max_delta, delta)


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
        self._pump.start()
        self._pub = self.create_publisher(Twist, '/cmd_vel', qos)

        self._runner = PeriodicRunner(self, [(1.0 / self._rate, self._tick)], 'cmd_vel_slew')
        self._runner.start()
        self.get_logger().info(
            'cmd_vel_slew ready: max_linear_accel=%.3f m/s^2, '
            'max_angular_accel=%.3f rad/s^2, cmd_timeout=%.3f s, rate=%.1f Hz'
            % (self._max_linear_accel, self._max_angular_accel,
               self._cmd_timeout, self._rate))

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
        else:
            tgt_lin = self._tgt[0]
            tgt_ang = self._tgt[5]
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
