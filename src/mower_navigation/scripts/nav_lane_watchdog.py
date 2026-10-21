#!/usr/bin/env python3
# Copyright 2026 ROS 2 port team
# SPDX-License-Identifier: Apache-2.0
"""Nav-lane WATCHDOG (2026-10-20, sim gate r5 hard ideal, body-aware coverage leg 26).

The controller's commands reach the wheels through
``/cmd_vel_nav_raw -> velocity_smoother -> /cmd_vel_nav_smoothed -> collision_monitor ->
/cmd_vel_nav``. In that run the lane went silent for good right after a Spin: the controller
kept publishing 0.175 m/s for the rest of the mow, /cmd_vel_nav carried nothing but the
behavior server's spins, the robot never moved again and 21 legs were lost. Neither node
logged anything and the root cause is NOT known (/cmd_vel_nav_smoothed was not recorded).

This is a watchdog, not a fix: it detects the lane silently dropping moving commands, logs it
loudly (ERROR, naming the silent stage) and recovers it in a bounded way by cycling the silent
lifecycle node (deactivate -> activate through its change_state service), at most
``max_recoveries`` times per ``window_s`` with ``cooldown_s`` between attempts. A
collision_monitor that is legitimately holding a stop (its state topic reports STOP /
SLOWDOWN / APPROACH / LIMIT) is never cycled. ``recover`` defaults to false (log only):
without a known root cause, cycling a node could override a real stop.
"""

import time

MOVING = 0.02           # |v| or |w| above this is a moving command


def node_now(node):
    """time.monotonic, or the node's ROS clock in seconds under use_sim_time (mower_sim
    publishes /clock, possibly faster than real time). Under sim time it first blocks until
    the first /clock and seeds the node clock with it (no stamp on a zero clock)."""
    if not node.get_parameter('use_sim_time').value:
        return time.monotonic
    from rclpy.time import Time
    from rclpy.wait_for_message import wait_for_message
    from rosgraph_msgs.msg import Clock as ClockMsg
    clock = node.get_clock()
    while not clock.now().nanoseconds and node.context.ok():
        ok, msg = wait_for_message(ClockMsg, node, '/clock', time_to_wait=10.0)
        if ok and (msg.clock.sec or msg.clock.nanosec) and not clock.now().nanoseconds:
            clock.set_ros_time_override(Time.from_msg(msg.clock))
        elif not ok:
            node.get_logger().warn('use_sim_time: waiting for /clock')
    return lambda: clock.now().nanoseconds * 1e-9


class LaneWatchdog:
    """Pure decision logic (no ROS). Times in seconds (monotonic)."""

    def __init__(self, stall_s=2.0, max_recoveries=3, window_s=600.0, cooldown_s=10.0,
                 hold_grace_s=5.0):
        self.stall_s = float(stall_s)
        self.max_recoveries = int(max_recoveries)
        self.window_s = float(window_s)
        self.cooldown_s = float(cooldown_s)
        self.last_raw_moving = None
        self.moving_since = None        # raw moving continuously since (gap < 0.5 s)
        self.last_smoothed = None
        self.last_nav = None
        self.cm_holding = False         # collision_monitor reports an active stop / slowdown
        # 2026-10-21 field: "commanding motion for 68.4 s but collision_monitor forwards
        # nothing" during a real stop_zone hold that flickered stop <-> slowdown. A hold
        # reported within hold_grace_s still counts as a real stop.
        self.hold_grace_s = float(hold_grace_s)
        self.cm_hold_t = None
        self.recoveries = []

    def on_raw(self, v, w, now):
        if abs(v) > MOVING or abs(w) > MOVING:
            if self.last_raw_moving is None or now - self.last_raw_moving > 0.5:
                self.moving_since = now
            self.last_raw_moving = now

    def on_smoothed(self, now):
        self.last_smoothed = now

    def on_nav(self, now):
        self.last_nav = now

    def on_cm_state(self, action_type, now=None):
        self.cm_holding = int(action_type) != 0
        if self.cm_holding and now is not None:
            self.cm_hold_t = now

    def _silent(self, last, now):
        return last is None or now - last >= self.stall_s

    def check(self, now):
        """None, or the stage that silently drops the controller's moving commands:
        'velocity_smoother' or 'collision_monitor'."""
        if self.last_raw_moving is None or now - self.last_raw_moving > 0.5 or \
                self.moving_since is None or now - self.moving_since < self.stall_s:
            return None
        if not self._silent(self.last_nav, now):
            return None
        if self._silent(self.last_smoothed, now):
            return 'velocity_smoother'
        if self.cm_holding or (self.cm_hold_t is not None
                               and now - self.cm_hold_t < self.hold_grace_s):
            return None             # a real stop: not ours to override
        return 'collision_monitor'

    def may_recover(self, now):
        self.recoveries = [t for t in self.recoveries if now - t < self.window_s]
        if len(self.recoveries) >= self.max_recoveries:
            return False
        if self.recoveries and now - self.recoveries[-1] < self.cooldown_s:
            return False
        self.recoveries.append(now)
        return True


def main(args=None):
    import rclpy
    from geometry_msgs.msg import Twist
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from nav2_msgs.msg import CollisionMonitorState
    from rclpy.node import Node

    class NavLaneWatchdog(Node):
        def __init__(self):
            super().__init__('nav_lane_watchdog')
            self._now = node_now(self)
            d = self.declare_parameter
            self.wd = LaneWatchdog(d('stall_s', 2.0).value, d('max_recoveries', 3).value,
                                   d('window_s', 600.0).value, d('cooldown_s', 10.0).value,
                                   d('hold_grace_s', 5.0).value)
            self.enabled = bool(d('recover', False).value)
            self.create_subscription(Twist, d('raw_topic', '/cmd_vel_nav_raw').value,
                                     lambda m: self.wd.on_raw(m.linear.x, m.angular.z,
                                                              self._now()), 10)
            self.create_subscription(Twist, d('smoothed_topic', '/cmd_vel_nav_smoothed').value,
                                     lambda _m: self.wd.on_smoothed(self._now()), 10)
            self.create_subscription(Twist, d('nav_topic', '/cmd_vel_nav').value,
                                     lambda _m: self.wd.on_nav(self._now()), 10)
            self.create_subscription(
                CollisionMonitorState,
                d('cm_state_topic', '/collision_monitor_state').value,
                lambda m: self.wd.on_cm_state(m.action_type, time.monotonic()), 10)
            self.lane_clients = {n: self.create_client(ChangeState, '/%s/change_state' % n)
                            for n in ('velocity_smoother', 'collision_monitor')}
            self.busy = False
            self.create_timer(0.5, self.on_tick)

        def on_tick(self):
            now = self._now()
            stage = self.wd.check(now)
            if stage is None or self.busy:
                return
            self.get_logger().error(
                'NAV LANE STALL (watchdog): the controller has been commanding motion for '
                '%.1f s but %s forwards nothing (/cmd_vel_nav silent)'
                % (now - self.wd.moving_since, stage), throttle_duration_sec=5.0)
            if not self.enabled or not self.wd.may_recover(now):
                return
            cli = self.lane_clients[stage]
            if not cli.service_is_ready():
                self.get_logger().error('nav lane watchdog: %s/change_state not available'
                                        % stage)
                return
            self.get_logger().error('nav lane watchdog: cycling %s (deactivate -> activate), '
                                    'recovery %d/%d in %.0f s' % (
                                        stage, len(self.wd.recoveries), self.wd.max_recoveries,
                                        self.wd.window_s))
            self.busy = True
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_DEACTIVATE

            def activated(fut):
                self.busy = False
                ok = fut.result() is not None and fut.result().success
                self.get_logger().error('nav lane watchdog: %s re-activated: %s' % (stage, ok))

            def deactivated(fut):
                act = ChangeState.Request()
                act.transition.id = Transition.TRANSITION_ACTIVATE
                cli.call_async(act).add_done_callback(activated)

            cli.call_async(req).add_done_callback(deactivated)

    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = NavLaneWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
