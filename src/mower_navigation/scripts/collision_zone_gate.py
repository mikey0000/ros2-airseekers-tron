#!/usr/bin/env python3
# Copyright 2026 ROS 2 port team
# SPDX-License-Identifier: Apache-2.0
"""Make the Jazzy collision_monitor's stop / slowdown zones forward-only.

Jazzy 1.3 nav2_collision_monitor STOP / SLOWDOWN polygons act on any command once
enough points are inside them; they do not look at the direction of travel. A front stop
zone would therefore also freeze a coverage reverse leg, or a pivot away from a hedge in
front of the nose (deadlock). This node watches the same command the collision_monitor
filters (``cmd_topic``, the velocity_smoother output) and sets the zones' dynamic
``<zone>.enabled`` parameters:

* on while the robot is commanded forward (linear.x >= ``forward_on_mps``) on a wide arc
  (|angular.z| / linear.x <= ``max_curvature_on``),
* off for reverse, pivots, creeping (linear.x < ``forward_off_mps``) and tight arcs
  (curvature > ``max_curvature_off``); hysteresis between,
* off while a behavior_server Spin / BackUp / DriveOnHeading goal is active (their
  ``<action>/_action/status``) and for ``behavior_hold_s`` after it ends,
* otherwise back on when no command has arrived for ``idle_rearm_s`` (the safe default).

2026-10-21 field session (grass false stops): slow arcing turns (v 0.03-0.09, |w| 0.1-0.3)
kept the zones armed while the nose swung over grass, and the idle re-arm after a Spin
(behavior_server bypasses the smoother) armed them under the first FollowPath command.

The direction-aware ``approach`` polygon is never touched (bumper source only, nav2_params). One SetParameters request per
change, and the state is re-sent every ``resync_s`` (a respawned collision_monitor starts
from the yaml default). A failed or unanswered (1 s) request is retried no faster than every
``retry_backoff_s``: while the collision_monitor is unconfigured its zone parameters are not
declared yet, and retrying at the 5 Hz tick wrote "parameter 'stop_zone.enabled' cannot be
set because it was not declared" 5x/s into its log for hours (2026-10-13, 28 MB). One WARN
per failure streak (then throttled), one INFO when a set succeeds again.
If this node is down the zones keep their last state (yaml default: on).
"""

import time


class ZoneGate:
    """Pure decision logic (no ROS): wanted zone state from commands, behaviors and time."""

    def __init__(self, forward_on=0.08, forward_off=0.06, idle_rearm_s=1.0,
                 max_curvature_on=1.0, max_curvature_off=1.4, behavior_hold_s=2.0):
        if forward_off > forward_on:
            raise ValueError('forward_off_mps must be <= forward_on_mps')
        if max_curvature_off < max_curvature_on:
            raise ValueError('max_curvature_off must be >= max_curvature_on')
        self.forward_on = float(forward_on)
        self.forward_off = float(forward_off)
        self.idle_rearm_s = float(idle_rearm_s)
        self.k_on = float(max_curvature_on)
        self.k_off = float(max_curvature_off)
        self.behavior_hold_s = float(behavior_hold_s)
        self.wanted = True
        self._last_cmd_t = None
        self._behavior = False
        self._hold_until = None

    def _held(self, now):
        return self._behavior or (self._hold_until is not None and now < self._hold_until)

    def on_cmd(self, linear_x, now, angular_z=0.0):
        self._last_cmd_t = now
        if self._held(now):
            self.wanted = False
            return self.wanted
        w = abs(float(angular_z))
        if linear_x >= self.forward_on and w <= self.k_on * linear_x:
            self.wanted = True
        elif linear_x < self.forward_off or w > self.k_off * max(linear_x, 1e-6):
            self.wanted = False
        return self.wanted

    def on_behavior(self, active, now):
        """A Spin / BackUp / DriveOnHeading goal became active (True) or ended (False)."""
        if active:
            self._behavior = True
            self.wanted = False
        elif self._behavior:
            self._behavior = False
            self._hold_until = now + self.behavior_hold_s
            self._last_cmd_t = now      # idle re-arm counts from the end of the hold
        return self.wanted

    def on_tick(self, now):
        if self._held(now):
            self.wanted = False
            return self.wanted
        self._hold_until = None
        if self._last_cmd_t is not None and now - self._last_cmd_t >= self.idle_rearm_s:
            self.wanted = True
            self._last_cmd_t = None
        return self.wanted


class RetryBackoff:
    """Pure: after a failed set, hold retries off for ``backoff_s``."""

    def __init__(self, backoff_s=5.0):
        self.backoff_s = float(backoff_s)
        self.failures = 0
        self._retry_at = None

    def ready(self, now):
        return self._retry_at is None or now >= self._retry_at

    def failed(self, now):
        """Record a failure; True for the first one of a streak (log it)."""
        self.failures += 1
        self._retry_at = now + self.backoff_s
        return self.failures == 1

    def succeeded(self):
        """Record a success; True if it ends a failure streak."""
        recovered = self.failures > 0
        self.failures = 0
        self._retry_at = None
        return recovered


def main(args=None):
    import rclpy
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from geometry_msgs.msg import Twist
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    class CollisionZoneGate(Node):
        def __init__(self):
            super().__init__('collision_zone_gate')
            d = self.declare_parameter
            cmd_topic = d('cmd_topic', '/cmd_vel_nav_smoothed').value
            cm = d('collision_monitor', 'collision_monitor').value
            self.zones = list(d('zones', ['stop_zone', 'slowdown_zone']).value)
            self.gate = ZoneGate(d('forward_on_mps', 0.08).value,
                                 d('forward_off_mps', 0.06).value,
                                 d('idle_rearm_s', 1.0).value,
                                 d('max_curvature_on', 1.0).value,
                                 d('max_curvature_off', 1.4).value,
                                 d('behavior_hold_s', 2.0).value)
            self.resync_s = float(d('resync_s', 2.0).value)
            self.backoff = RetryBackoff(d('retry_backoff_s', 5.0).value)
            self.applied = None     # last state the collision_monitor confirmed
            self.applied_t = 0.0
            self.pending = None     # (state, future, sent monotonic)
            self.cli = self.create_client(SetParameters, '/%s/set_parameters' % cm.lstrip('/'))
            self.create_subscription(Twist, cmd_topic, self.on_cmd, 10)
            # behavior_server goals publish /cmd_vel_nav directly (no smoother command here)
            self._active = {}
            status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
            for a in d('behavior_actions', ['/spin', '/backup', '/drive_on_heading']).value:
                self.create_subscription(
                    GoalStatusArray, a.rstrip('/') + '/_action/status',
                    lambda m, a=a: self.on_status(a, m), status_qos)
            self.create_timer(0.2, self.on_tick)
            self.get_logger().info('gating %s on %s (forward >= %.2f m/s)'
                                   % (self.zones, cmd_topic, self.gate.forward_on))

        def on_cmd(self, msg):
            self.sync(self.gate.on_cmd(msg.linear.x, time.monotonic(), msg.angular.z))

        def on_status(self, action, msg):
            live = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                    GoalStatus.STATUS_CANCELING)
            self._active[action] = any(st.status in live for st in msg.status_list)
            self.sync(self.gate.on_behavior(any(self._active.values()), time.monotonic()))

        def on_tick(self):
            now = time.monotonic()
            if self.pending is None and now - self.applied_t >= self.resync_s:
                self.applied = None
            self.sync(self.gate.on_tick(now))

        def failed(self, why):
            if self.backoff.failed(time.monotonic()):
                self.get_logger().warn(
                    'collision_monitor zones.enabled not set (%s); retrying every %.0f s '
                    '(normal while it is not configured yet)' % (why, self.backoff.backoff_s))
            else:
                self.get_logger().warn('collision_monitor still refusing zones.enabled (%s, '
                                       '%d attempts)' % (why, self.backoff.failures),
                                       throttle_duration_sec=60.0)

        def sync(self, wanted):
            if self.pending is not None:
                state, fut, sent = self.pending
                if not fut.done():
                    if time.monotonic() - sent < 1.0:
                        return
                    self.cli.remove_pending_request(fut)
                    self.pending = None
                    self.failed('no answer in 1 s')
                    return
                self.pending = None
                res = fut.result()
                if res is not None and all(r.successful for r in res.results):
                    self.applied = state
                    self.applied_t = time.monotonic()
                    if self.backoff.succeeded():
                        self.get_logger().info('collision_monitor accepted zones.enabled=%s'
                                               % state)
                else:
                    self.failed('; '.join(sorted({r.reason for r in res.results}))
                                if res is not None else 'no result')
            if wanted == self.applied:
                return
            if not self.backoff.ready(time.monotonic()):
                return
            if not self.cli.service_is_ready():
                self.get_logger().warn('collision_monitor parameter service not ready',
                                       throttle_duration_sec=10.0)
                return
            req = SetParameters.Request()
            for z in self.zones:
                req.parameters.append(Parameter(
                    name=z + '.enabled',
                    value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=wanted)))
            self.pending = (wanted, self.cli.call_async(req), time.monotonic())
            self.get_logger().debug('zones.enabled -> %s' % wanted)

    rclpy.init(args=args)
    node = CollisionZoneGate()
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
