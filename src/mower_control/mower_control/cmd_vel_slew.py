#!/usr/bin/env python3
"""Slew-rate limiter for ``/cmd_vel`` with a command watchdog (ROS 2 Humble).

Port of the MowgliNext ``cmd_vel_slew.hpp`` pattern -- see
``ros2_stack/docs/mowglinext_baseline.md`` section 4.3, item 9: shape the
commanded twist *before* it reaches the firmware so the MCU never sees a step
change in linear/angular velocity, and echo the exact applied command for
rosbag analysis.

Subscribed topics
-----------------
``/cmd_vel_raw``   geometry_msgs/TwistStamped   unshaped command in (twist_mux,
                                                     teleop, Nav2, ...)

Published topics
---------------
``/cmd_vel``       geometry_msgs/TwistStamped   slew-limited command out; this
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

Behaviour
---------
* Fixed-rate loop.  Each tick moves the applied linear.x / angular.z at most
  ``accel * dt`` toward the target; every other twist component passes through
  unchanged.
* Every published command is echoed to the log at full float precision (the
  MowgliNext ``/cmd_vel_applied`` idea, as a log line instead of a topic).
  This is the cheapest possible debugging tool when comparing commanded vs
  applied; silence with ``--ros-args --log-level cmd_vel_slew:=warn`` if the
  20 Hz echo is too chatty.
* Watchdog: when no ``/cmd_vel_raw`` has arrived within ``cmd_timeout``, the
  applied velocity is set to exactly 0.0 (not slewed) and zero is published
  every tick until a fresh command arrives; recovery then slews up from zero.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import TwistStamped


class CmdVelSlewNode(Node):
    """Slew-limits ``/cmd_vel_raw`` -> ``/cmd_vel`` and enforces the watchdog."""

    def __init__(self):
        super().__init__('cmd_vel_slew')

        self.declare_parameter('max_linear_accel', 0.5)
        self.declare_parameter('max_angular_accel', 1.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('frame_id', 'base_link')

        self._max_linear_accel = float(self.get_parameter('max_linear_accel').value)
        self._max_angular_accel = float(self.get_parameter('max_angular_accel').value)
        self._cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self._rate = float(self.get_parameter('rate').value)
        self._frame_id = str(self.get_parameter('frame_id').value)

        # Applied (slewed) state, the last raw target, watchdog bookkeeping.
        self._cur_lin = 0.0
        self._cur_ang = 0.0
        self._tgt_msg = None
        self._last_cmd_time = None
        self._last_tick = None

        qos = QoSProfile(depth=10,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(TwistStamped, '/cmd_vel_raw',
                                 self._on_cmd_vel_raw, qos)
        self._pub = self.create_publisher(TwistStamped, '/cmd_vel', qos)

        self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            'cmd_vel_slew ready: max_linear_accel=%.3f m/s^2, '
            'max_angular_accel=%.3f rad/s^2, cmd_timeout=%.3f s, rate=%.1f Hz'
            % (self._max_linear_accel, self._max_angular_accel,
               self._cmd_timeout, self._rate))

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def _on_cmd_vel_raw(self, msg):
        """Store the raw target; the timer loop does the slewing/publishing."""
        self._tgt_msg = msg
        self._last_cmd_time = self.get_clock().now()

    def _tick(self):
        now = self.get_clock().now()
        if self._last_tick is None:
            self._last_tick = now
            return
        dt = (now - self._last_tick).nanoseconds * 1e-9
        self._last_tick = now
        if dt <= 0.0:
            return

        stale = (self._last_cmd_time is None or
                 (now - self._last_cmd_time).nanoseconds * 1e-9 > self._cmd_timeout)

        if stale:
            # Immediate exact stop (MowgliNext cmd_vel_slew): no slewing to zero.
            self._cur_lin = 0.0
            self._cur_ang = 0.0
            tgt_lin, tgt_ang = 0.0, 0.0
        else:
            tgt_lin = self._tgt_msg.twist.linear.x
            tgt_ang = self._tgt_msg.twist.angular.z
            self._cur_lin = self._slew(self._cur_lin, tgt_lin,
                                       self._max_linear_accel, dt)
            self._cur_ang = self._slew(self._cur_ang, tgt_ang,
                                       self._max_angular_accel, dt)

        out = TwistStamped()
        out.header.stamp = now.to_msg()
        out.header.frame_id = self._frame_id
        if self._tgt_msg is not None:
            # Pass through every component we do not shape.
            src = self._tgt_msg.twist
            out.twist.linear.y = src.linear.y
            out.twist.linear.z = src.linear.z
            out.twist.angular.x = src.angular.x
            out.twist.angular.y = src.angular.y
        out.twist.linear.x = self._cur_lin
        out.twist.angular.z = self._cur_ang
        self._pub.publish(out)

        # Exact echo: full float precision, one line per published command.
        self.get_logger().info(
            'cmd_vel applied: linear.x={!r} angular.z={!r} '
            '(target linear.x={!r} angular.z={!r})'.format(
                self._cur_lin, self._cur_ang, tgt_lin, tgt_ang))

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _slew(current, target, max_accel, dt):
        """Move ``current`` toward ``target`` by at most ``max_accel * dt``."""
        delta = target - current
        max_delta = max_accel * dt
        if abs(delta) <= max_delta:
            return target
        return current + math.copysign(max_delta, delta)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSlewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
