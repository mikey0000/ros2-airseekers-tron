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

* on while the robot is commanded forward (linear.x >= ``forward_on_mps``),
* off for reverse, pivots and creeping (linear.x < ``forward_off_mps``; hysteresis between),
* back on when no command has arrived for ``idle_rearm_s`` (the safe default).

The direction-aware ``approach`` polygon is never touched. One SetParameters request per
change; a failed or unanswered (1 s) request is retried on the next command / timer tick,
and the state is re-sent every ``resync_s`` (a respawned collision_monitor starts from the
yaml default).
If this node is down the zones keep their last state (yaml default: on).
"""

import time


class ZoneGate:
    """Pure decision logic (no ROS): wanted zone state from commands and time."""

    def __init__(self, forward_on=0.08, forward_off=0.06, idle_rearm_s=1.0):
        if forward_off > forward_on:
            raise ValueError('forward_off_mps must be <= forward_on_mps')
        self.forward_on = float(forward_on)
        self.forward_off = float(forward_off)
        self.idle_rearm_s = float(idle_rearm_s)
        self.wanted = True
        self._last_cmd_t = None

    def on_cmd(self, linear_x, now):
        self._last_cmd_t = now
        if linear_x >= self.forward_on:
            self.wanted = True
        elif linear_x < self.forward_off:
            self.wanted = False
        return self.wanted

    def on_tick(self, now):
        if self._last_cmd_t is not None and now - self._last_cmd_t >= self.idle_rearm_s:
            self.wanted = True
            self._last_cmd_t = None
        return self.wanted


def main(args=None):
    import rclpy
    from geometry_msgs.msg import Twist
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from rclpy.node import Node

    class CollisionZoneGate(Node):
        def __init__(self):
            super().__init__('collision_zone_gate')
            d = self.declare_parameter
            cmd_topic = d('cmd_topic', '/cmd_vel_nav_smoothed').value
            cm = d('collision_monitor', 'collision_monitor').value
            self.zones = list(d('zones', ['stop_zone', 'slowdown_zone']).value)
            self.gate = ZoneGate(d('forward_on_mps', 0.08).value,
                                 d('forward_off_mps', 0.06).value,
                                 d('idle_rearm_s', 1.0).value)
            self.resync_s = float(d('resync_s', 2.0).value)
            self.applied = None     # last state the collision_monitor confirmed
            self.applied_t = 0.0
            self.pending = None     # (state, future, sent monotonic)
            self.cli = self.create_client(SetParameters, '/%s/set_parameters' % cm.lstrip('/'))
            self.create_subscription(Twist, cmd_topic, self.on_cmd, 10)
            self.create_timer(0.2, self.on_tick)
            self.get_logger().info('gating %s on %s (forward >= %.2f m/s)'
                                   % (self.zones, cmd_topic, self.gate.forward_on))

        def on_cmd(self, msg):
            self.sync(self.gate.on_cmd(msg.linear.x, time.monotonic()))

        def on_tick(self):
            now = time.monotonic()
            if self.pending is None and now - self.applied_t >= self.resync_s:
                self.applied = None
            self.sync(self.gate.on_tick(now))

        def sync(self, wanted):
            if self.pending is not None:
                state, fut, sent = self.pending
                if not fut.done():
                    if time.monotonic() - sent < 1.0:
                        return
                    self.cli.remove_pending_request(fut)
                    self.pending = None
                    return
                self.pending = None
                res = fut.result()
                if res is not None and all(r.successful for r in res.results):
                    self.applied = state
                    self.applied_t = time.monotonic()
                else:
                    self.get_logger().warn('collision_monitor refused zones.enabled=%s' % state,
                                           throttle_duration_sec=5.0)
            if wanted == self.applied:
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
