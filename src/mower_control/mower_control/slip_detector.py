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
``/cmd_vel``      geometry_msgs/Twist          commanded twist (output of
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

Stuck guard (2026-10-07 lawn-damage incident: the robot dug a hole for ~45 s
while Nav2 kept commanding and nothing stopped the wheels)
-------------------------------------------------------------------------------
:class:`StuckDetector` (pure logic) runs on the same tick:

* commanded motion (applied ``/cmd_vel`` |v| > ``stuck_cmd_linear`` or |w| >
  ``stuck_cmd_angular``) held for ``stuck_window_s`` while the RTK-fused EKF
  pose (``/odometry/filtered_map``) moved < ``stuck_min_progress_m`` and turned
  < ``stuck_min_heading_deg`` => STUCK (reason ``stuck``);
* wheel-odometry distance / EKF distance > ``stuck_slip_ratio`` over the window
  (wheel distance >= ``stuck_slip_min_wheel_m``) => STUCK (reason ``spinning``);
* ``/stuck`` (std_msgs/Bool, latched) stays true until the EKF pose moved
  > ``stuck_clear_m`` from where it latched, ``/stuck_guard/suppress`` (Bool,
  latched; the mission's escape manoeuvre) is true, or ``/stuck_guard/reset``
  (Bool, any message) arrives; each latch also publishes ``/stuck_event``
  (String JSON ``{x, y, phase, commanded, ts, reason, ...}``).

``mower_control/cmd_vel_slew`` forces zero while ``/stuck`` is true;
``mower_mission`` runs the escape / STUCK_NEEDS_HELP logic.

CPU (RK3588): every input is a sampled input of a ``SubscriptionPump`` (depth
1, read once per tick, newest message only; /odom and /wheel_vel parsed
straight from the CDR bytes), the loop runs on a ``PeriodicRunner`` thread
instead of rclpy timers and ``/dig_stall`` is published pre-serialized. The
node no longer wakes the rclpy executor per message (/odom alone is 50 Hz).
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from mower_control.fast_msgs import (bool_bytes, parse_string, parse_twist,
                                     parse_twist_stamped)
from mower_control.sub_pump import (PeriodicRunner, SubscriptionPump, flat_parser,
                                    parse_odometry)

_DIG_TRUE = bool_bytes(True)
_DIG_FALSE = bool_bytes(False)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class StuckDetector:
    """Pure stuck / spinning detector (see module docstring). ``update`` returns
    ``None`` or the event dict of a new latch."""

    def __init__(self, window_s=4.0, min_progress_m=0.05, min_heading_deg=10.0,
                 cmd_linear=0.03, cmd_angular=0.1, clear_m=0.3, slip_ratio=3.0,
                 slip_min_wheel_m=0.2, pose_timeout_s=1.0, gap_s=1.5):
        self.window_s = float(window_s)
        self.min_progress_m = float(min_progress_m)
        self.min_heading = math.radians(float(min_heading_deg))
        self.cmd_linear = float(cmd_linear)
        self.cmd_angular = float(cmd_angular)
        self.clear_m = float(clear_m)
        self.slip_ratio = float(slip_ratio)
        self.slip_min_wheel_m = float(slip_min_wheel_m)
        self.pose_timeout_s = float(pose_timeout_s)
        # Brief sub-threshold / zero command gaps (<= gap_s) do NOT restart the window:
        # a stalled RPP rotate-to-heading dithers |w| through 0 every ~1 s, which used to
        # clear the buffer forever (2026-10-07 live incident, no latch for 4+ min).
        self.gap_s = float(gap_s)
        self._last_cmd_t = None
        self.stuck = False
        self.latch_pose = None
        self.reason = ''
        self._buf = []          # [(t, x, y, yaw, wheel_cum)] of the commanded window
        self._wheel_cum = 0.0
        self._last_t = None

    def reset(self):
        self.stuck = False
        self.latch_pose = None
        self.reason = ''
        self._buf = []
        self._last_cmd_t = None

    def commanded(self, cmd):
        return cmd is not None and (abs(cmd[0]) > self.cmd_linear or abs(cmd[1]) > self.cmd_angular)

    def update(self, now, cmd, pose, pose_t, wheel_v, suppressed=False):
        """``cmd`` (v, w) applied command, ``pose`` (x, y, yaw) EKF map pose received at
        ``pose_t``, ``wheel_v`` measured wheel linear speed (m/s, None = unknown)."""
        dt = 0.0 if self._last_t is None else max(0.0, min(0.5, now - self._last_t))
        self._last_t = now
        if wheel_v is not None:
            self._wheel_cum += abs(float(wheel_v)) * dt
        if suppressed:
            self.reset()
            return None
        fresh = pose is not None and pose_t is not None and now - pose_t <= self.pose_timeout_s
        if self.stuck:
            if fresh and math.hypot(pose[0] - self.latch_pose[0],
                                    pose[1] - self.latch_pose[1]) > self.clear_m:
                self.reset()
            return None
        if not fresh:
            self._buf = []
            return None
        if not self.commanded(cmd):
            if self._last_cmd_t is None or now - self._last_cmd_t > self.gap_s:
                self._buf = []
            return None
        self._last_cmd_t = now
        self._buf.append((now, pose[0], pose[1], pose[2], self._wheel_cum))
        while len(self._buf) > 1 and now - self._buf[1][0] >= self.window_s:
            self._buf.pop(0)
        t0, x0, y0, yaw0, w0 = self._buf[0]
        if now - t0 < self.window_s:
            return None
        moved = max(math.hypot(b[1] - x0, b[2] - y0) for b in self._buf)
        turned = max(abs(_wrap(b[3] - yaw0)) for b in self._buf)
        ekf_d = math.hypot(pose[0] - x0, pose[1] - y0)
        wheel_d = self._wheel_cum - w0
        reason = ''
        if moved < self.min_progress_m and turned < self.min_heading:
            reason = 'stuck'
        elif (wheel_d >= self.slip_min_wheel_m
              and wheel_d > self.slip_ratio * max(ekf_d, 1e-3)):
            reason = 'spinning'
        if not reason:
            return None
        self.stuck = True
        self.reason = reason
        self.latch_pose = (pose[0], pose[1])
        self._buf = []
        return {'x': round(pose[0], 3), 'y': round(pose[1], 3), 'reason': reason,
                'commanded': [round(cmd[0], 3), round(cmd[1], 3)],
                'ekf_moved_m': round(moved, 3), 'turned_deg': round(math.degrees(turned), 1),
                'wheel_m': round(wheel_d, 3), 'window_s': self.window_s}


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
        # stuck guard (StuckDetector)
        self.declare_parameter('stuck_guard', True)
        self.declare_parameter('stuck_window_s', 4.0)
        self.declare_parameter('stuck_min_progress_m', 0.05)
        self.declare_parameter('stuck_min_heading_deg', 10.0)
        self.declare_parameter('stuck_cmd_linear', 0.03)
        self.declare_parameter('stuck_cmd_angular', 0.1)
        self.declare_parameter('stuck_clear_m', 0.3)
        self.declare_parameter('stuck_slip_ratio', 3.0)
        self.declare_parameter('stuck_slip_min_wheel_m', 0.2)
        self.declare_parameter('stuck_cmd_gap_s', 1.5)
        self.declare_parameter('stuck_pose_topic', '/odometry/filtered_map')

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
        self._fix_time = None       # monotonic receive time of last /fix_status
        self._fix_warned = False
        self._latched = False
        self._violation_start = None
        self._measured_sub = None
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self._stuck_enabled = bool(gp('stuck_guard'))
        self._stuck = StuckDetector(
            window_s=gp('stuck_window_s'), min_progress_m=gp('stuck_min_progress_m'),
            min_heading_deg=gp('stuck_min_heading_deg'), cmd_linear=gp('stuck_cmd_linear'),
            cmd_angular=gp('stuck_cmd_angular'), clear_m=gp('stuck_clear_m'),
            slip_ratio=gp('stuck_slip_ratio'), slip_min_wheel_m=gp('stuck_slip_min_wheel_m'),
            gap_s=gp('stuck_cmd_gap_s'))
        self._ekf_pose = None       # (x, y, yaw) of the newest EKF map pose
        self._ekf_t = None
        self._suppressed = False
        self._reset_req = False
        self._phase = ''
        self._stuck_last = None

        # Latest-value inputs: depth 1, taken only when the tick consumes them.
        self._sample_qos = QoSProfile(depth=1,
                                      reliability=QoSReliabilityPolicy.RELIABLE,
                                      history=QoSHistoryPolicy.KEEP_LAST)
        self._pump = SubscriptionPump(self, 'slip_inputs')
        self._pump.subscribe(Twist, '/cmd_vel', self._on_cmd_vel, self._sample_qos,
                             parser=parse_twist, sampled=True)
        self._pump.subscribe(String, '/fix_status', self._on_fix_status, self._sample_qos,
                             parser=parse_string, sampled=True, with_receipt=True)
        latched_in = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                history=QoSHistoryPolicy.KEEP_LAST,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._pump.subscribe(Odometry, str(gp('stuck_pose_topic')), self._on_ekf,
                             self._sample_qos, parser=parse_odometry, sampled=True,
                             with_receipt=True)
        self._pump.subscribe(Bool, '/stuck_guard/suppress', self._on_suppress, latched_in,
                             parser=flat_parser(Bool), sampled=True)
        self._pump.subscribe(Bool, '/stuck_guard/reset', self._on_reset, self._sample_qos,
                             parser=flat_parser(Bool), sampled=True)
        from mowgli_interfaces.msg import HighLevelStatus
        self._pump.subscribe(HighLevelStatus, '/behavior_tree_node/high_level_status',
                             self._on_hl, self._sample_qos, sampled=True)
        # Only sampled entries: start() runs no thread, so the measured source can still be
        # added once it is resolved.
        self._pump.start()

        # Latched flag: late joiners must see it on connect.
        dig_qos = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._dig_pub = self.create_publisher(Bool, '/dig_stall', dig_qos)
        self._stuck_pub = self.create_publisher(Bool, '/stuck', dig_qos)
        self._stuck_event_pub = self.create_publisher(String, '/stuck_event', dig_qos)

        # Resolve the measured source (auto-detect /wheel_vel vs /odom) on the same thread
        # as the tick, so the pump is never polled while a subscription is being added.
        self._discovery_deadline = time.monotonic() + self._discovery_timeout
        self._runner = PeriodicRunner(
            self, [(1.0 / self._publish_rate, self._tick),
                   (0.5, self._resolve_measured_topic)], 'slip_detector')
        self._runner.start()
        self.get_logger().info(
            'slip_detector ready: slip_threshold=%.3f, slip_window=%.1f s, '
            'require_rtk_fixed=%s, publish_rate=%.1f Hz'
            % (self._slip_threshold, self._slip_window,
               self._require_rtk_fixed, self._publish_rate))

    def shutdown(self):
        self._runner.stop()
        self._pump.stop()

    # ------------------------------------------------------------------
    # callbacks (run from SubscriptionPump.poll() inside _tick)
    # ------------------------------------------------------------------
    def _on_cmd_vel(self, msg):
        self._cmd = (msg.linear.x, msg.angular.z)

    def _on_meas_twist(self, msg):
        self._meas = (msg.twist.linear.x, msg.twist.angular.z)

    def _on_meas_odom(self, msg):
        self._meas = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def _on_ekf(self, msg, receipt):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._ekf_pose = (p.x, p.y, yaw)
        self._ekf_t = receipt

    def _on_suppress(self, msg):
        self._suppressed = bool(msg.data)

    def _on_reset(self, msg):
        self._reset_req = True

    def _on_hl(self, msg):
        self._phase = str(msg.state_name)

    def _on_fix_status(self, msg, receipt):
        self._fix_status = msg.data
        self._fix_time = receipt
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
            self._measured_sub = self._pump.subscribe(
                Odometry, topic, self._on_meas_odom, self._sample_qos,
                parser=parse_odometry, sampled=True)
        else:
            self._measured_sub = self._pump.subscribe(
                TwistStamped, topic, self._on_meas_twist, self._sample_qos,
                parser=parse_twist_stamped, sampled=True)
        self.get_logger().info('measured twist source: %s' % topic)

    # ------------------------------------------------------------------
    # main loop
    # ------------------------------------------------------------------
    def _tick(self):
        self._pump.poll()
        now = time.monotonic()
        self.step(now)
        if getattr(self, '_stuck_enabled', False):
            self.stuck_step(now)

    def stuck_step(self, now):
        """Run the StuckDetector at ``now``; publish /stuck every tick, /stuck_event on a latch."""
        det = self._stuck
        if self._reset_req:
            self._reset_req = False
            if det.stuck:
                self.get_logger().info('stuck guard: latch reset by the mission')
            det.reset()
        was = det.stuck
        ev = det.update(now, self._cmd, self._ekf_pose, self._ekf_t,
                        self._meas[0] if self._meas is not None else None,
                        suppressed=self._suppressed)
        if ev is not None:
            ev['phase'] = self._phase
            ev['ts'] = time.time()
            self.get_logger().error(
                'STUCK (%s) at (%.2f, %.2f) in %s: commanded v=%.2f w=%.2f for %.1f s, '
                'EKF moved %.3f m / %.1f deg, wheels %.2f m -> /stuck latched'
                % (ev['reason'], ev['x'], ev['y'], ev['phase'], ev['commanded'][0],
                   ev['commanded'][1], ev['window_s'], ev['ekf_moved_m'], ev['turned_deg'],
                   ev['wheel_m']))
            self._stuck_event_pub.publish(String(data=json.dumps(ev, sort_keys=True)))
        elif was and not det.stuck:
            self.get_logger().info('stuck guard: cleared (%s)' % (
                'suppressed for an escape' if self._suppressed else 'robot moved'))
        self._stuck_last = det.stuck
        self._stuck_pub.publish(_DIG_TRUE if det.stuck else _DIG_FALSE)

    def step(self, now):
        """Evaluate the slip condition at monotonic time ``now`` and publish /dig_stall."""
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
                return  # (used to fall through and publish False on the latching tick)
        else:
            if self._violation_start is not None:
                self.get_logger().info(
                    'slip condition cleared after %.1f s'
                    % (now - self._violation_start))
            self._violation_start = None
        self._publish_dig(False)

    def _publish_dig(self, latched):
        self._dig_pub.publish(_DIG_TRUE if latched else _DIG_FALSE)


def main(args=None):
    try:  # crash records -> /userdata/ros2/crashes (docs/crash_recovery.md)
        from mower_control.crash_record import install as _install_crash_record
        _install_crash_record('slip_detector')
    except ImportError:
        pass
    rclpy.init(args=args)
    node = SlipDetectorNode()
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
