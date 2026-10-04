#!/usr/bin/env python3
"""Wheel-slip ("dig") detector with an RTK-fixed gate (ROS 2 Humble).

Port of the MowgliNext ``dig_detector.hpp`` pattern -- see
``ros2_stack/docs/mowglinext_baseline.md`` section 4.3, item 8: compare the
commanded twist against the measured twist and, when the mismatch persists for
a configurable window, latch a dig-stall flag.  MowgliNext gates the latch on
``DigTrustSigma`` from ``/gps/status``; this port gates on the ``/fix_status``
string published by ``um960_gps_driver`` (``quality=RTK_FIXED`` for NMEA GGA
quality 4, ``solution=INS_RTKFIXED`` for Unicore BESTNAV posType 17 -- both
contain the substring ``RTK_FIXED``).

Subscribed topics
-----------------
``/cmd_vel``      geometry_msgs/TwistStamped   commanded twist (output of
                                                  ``mower_control/cmd_vel_slew``)
``/wheel_vel``    geometry_msgs/TwistStamped   measured twist -- preferred source
                                                  (published by the vendor
                                                  ``mower_base`` stack at ~100 Hz)
``/odom``         nav_msgs/Odometry             measured twist -- fallback used
                                                  when ``/wheel_vel`` has no
                                                  publisher (our
                                                  ``mower_mcu_driver`` publishes
                                                  this)
``/fix_status``   std_msgs/String               RTK quality from
                                                  ``um960_gps_driver``

Published topics
----------------
``/dig_stall``    std_msgs/Bool                latched wheel-slip flag,
                                                  QoS(1).transient_local() so
                                                  late joiners see the latch

Parameters
----------
``slip_threshold``    ``0.3``    m/s (also applied to rad/s) -- |measured -
                                 commanded| above this on linear.x or angular.z
                                 is a slip condition
``slip_window``       ``2.0``    s of continuous slip before the latch is raised
``require_rtk_fixed``  ``true``  only latch while /fix_status reports RTK_FIXED
``measured_topic``    ``''``    override the measured source; empty = auto
                                 (prefer /wheel_vel, fall back to /odom)
``discovery_timeout`` ``5.0``    s to wait for a /wheel_vel publisher before
                                 falling back to /odom
``publish_rate``      ``10.0``   Hz latch-status publish rate
``fix_timeout``       ``5.0``    s without a /fix_status message before warning

Behaviour
---------
* The measured source is auto-detected at startup: if ``/wheel_vel`` has a
  publisher it is subscribed, otherwise the node falls back to ``/odom`` after
  ``discovery_timeout`` (override with the ``measured_topic`` param).
* A slip condition is ``|measured - commanded| > slip_threshold`` on linear.x
  or angular.z.  The condition must hold continuously for ``slip_window`` seconds
  before ``/dig_stall`` latches true; any sample below the threshold restarts
  the window.  The latch is never cleared by this node (MowgliNext clears on
  charger contact / escape displacement -- port that with dig_escalation later).
* With ``require_rtk_fixed`` the window does not advance unless the latest
  ``/fix_status`` contains ``RTK_FIXED``; a missing or stale ``/fix_status``
  suppresses the latch and logs a warning.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class SlipDetectorNode(Node):
    """Compares commanded vs measured twist and latches /dig_stall on slip."""

    def __init__(self):
        super().__init__('slip_detector')

        self.declare_parameter('slip_threshold', 0.3)
        self.declare_parameter('slip_window', 2.0)
        self.declare_parameter('require_rtk_fixed', True)
        self.declare_parameter('measured_topic', '')
        self.declare_parameter('discovery_timeout', 5.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('fix_timeout', 5.0)

        self._slip_threshold = float(self.get_parameter('slip_threshold').value)
        self._slip_window = float(self.get_parameter('slip_window').value)
        self._require_rtk_fixed = bool(self.get_parameter('require_rtk_fixed').value)
        self._measured_topic = str(self.get_parameter('measured_topic').value)
        self._discovery_timeout = float(self.get_parameter('discovery_timeout').value)
        self._publish_rate = float(self.get_parameter('publish_rate').value)
        self._fix_timeout = float(self.get_parameter('fix_timeout').value)

        self._cmd = None            # (linear.x, angular.z) of last /cmd_vel
        self._meas = None           # (linear.x, angular.z) of last measured twist
        self._fix_status = None     # last /fix_status string
        self._fix_time = None       # monotonic time of last /fix_status
        self._fix_warned = False
        self._latched = False
        self._violation_start = None
        self._measured_sub = None

        self._qos10 = QoSProfile(depth=10,
                                 reliability=QoSReliabilityPolicy.RELIABLE,
                                 history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(TwistStamped, '/cmd_vel', self._on_cmd_vel,
                                 self._qos10)
        self.create_subscription(String, '/fix_status', self._on_fix_status,
                                 self._qos10)

        # Latched flag: late joiners must see it on connect.
        dig_qos = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._dig_pub = self.create_publisher(Bool, '/dig_stall', dig_qos)

        self.create_timer(1.0 / self._publish_rate, self._tick)

        # Resolve the measured source (auto-detect /wheel_vel vs /odom).
        self._discovery_deadline = time.monotonic() + self._discovery_timeout
        self.create_timer(0.5, self._resolve_measured_topic)
        self.get_logger().info(
            'slip_detector ready: slip_threshold=%.3f, slip_window=%.1f s, '
            'require_rtk_fixed=%s, publish_rate=%.1f Hz'
            % (self._slip_threshold, self._slip_window,
               self._require_rtk_fixed, self._publish_rate))

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def _on_cmd_vel(self, msg):
        self._cmd = (msg.twist.linear.x, msg.twist.angular.z)

    def _on_meas_twist(self, msg):
        self._meas = (msg.twist.linear.x, msg.twist.angular.z)

    def _on_meas_odom(self, msg):
        self._meas = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def _on_fix_status(self, msg):
        self._fix_status = msg.data
        self._fix_time = time.monotonic()
        self._fix_warned = False

    def _resolve_measured_topic(self):
        """Pick /wheel_vel (preferred) or /odom (fallback) and subscribe."""
        if self._measured_sub is not None:
            return  # already resolved
        topic = self._measured_topic
        if not topic:
            if self.get_publishers_info_by_topic('/wheel_vel'):
                topic = '/wheel_vel'
            elif time.monotonic() > self._discovery_deadline:
                topic = '/odom'
                self.get_logger().warn(
                    'no /wheel_vel publisher within %.1f s; falling back to /odom'
                    % self._discovery_timeout)
            else:
                return  # keep waiting
        if topic == '/odom':
            self._measured_sub = self.create_subscription(
                Odometry, topic, self._on_meas_odom, self._qos10)
        else:
            self._measured_sub = self.create_subscription(
                TwistStamped, topic, self._on_meas_twist, self._qos10)
        self.get_logger().info('measured twist source: %s' % topic)

    # ------------------------------------------------------------------
    # main loop
    # ------------------------------------------------------------------
    def _tick(self):
        now = time.monotonic()
        if self._latched:
            self._publish_dig(True)
            return
        if self._cmd is None or self._meas is None:
            return

        if self._require_rtk_fixed:
            if self._fix_status is None:
                if not self._fix_warned:
                    self.get_logger().warn(
                        'require_rtk_fixed=true but no /fix_status received yet; '
                        'dig latch is suppressed')
                    self._fix_warned = True
                return
            if now - self._fix_time > self._fix_timeout:
                if not self._fix_warned:
                    self.get_logger().warn(
                        '/fix_status is stale (%.1f s); dig latch is suppressed'
                        % self._fix_timeout)
                    self._fix_warned = True
                return
            if 'RTK_FIXED' not in self._fix_status:
                # Not RTK-fixed: MowgliNext does not accumulate slip without the
                # GNSS trust gate.
                self._violation_start = None
                return

        d_lin = abs(self._meas[0] - self._cmd[0])
        d_ang = abs(self._meas[1] - self._cmd[1])
        if d_lin > self._slip_threshold or d_ang > self._slip_threshold:
            if self._violation_start is None:
                self._violation_start = now
                self.get_logger().info(
                    'slip condition started: |d_linear|=%.3f |d_angular|=%.3f '
                    '(threshold %.3f)' % (d_lin, d_ang, self._slip_threshold))
            elif now - self._violation_start >= self._slip_window:
                self._latched = True
                self.get_logger().warn(
                    'DIG STALL latched: |d_linear|=%.3f |d_angular|=%.3f for '
                    '>= %.1f s (rtk_fixed=true)'
                    % (d_lin, d_ang, self._slip_window))
                self._publish_dig(True)
        else:
            if self._violation_start is not None:
                self.get_logger().info(
                    'slip condition cleared after %.1f s'
                    % (now - self._violation_start))
            self._violation_start = None
        self._publish_dig(False)

    def _publish_dig(self, latched):
        self._dig_pub.publish(Bool(data=latched))


def main(args=None):
    rclpy.init(args=args)
    node = SlipDetectorNode()
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
