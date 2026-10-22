# SPDX-License-Identifier: GPL-3.0-or-later
"""Node clocks: ``time.monotonic`` in production, the ROS clock under ``use_sim_time``.

:func:`node_clock` gives a node its :class:`NodeClock` (``now`` / ``sleep`` /
``wait_until``); :func:`node_now` is the bare ``now`` callable (the former per-script
``node_now`` helpers of mower_navigation). :data:`WALL_CLOCK` stands in for a NodeClock in
tests that stub a node without ``__init__``.
"""

import threading
import time
import types


class NodeClock:
    """``time.monotonic`` / ``time.sleep`` for a node, or its ROS clock under ``use_sim_time``.

    In production (use_sim_time false) ``now`` IS ``time.monotonic`` and ``sleep`` IS
    ``time.sleep``: nothing changes. In mower_sim (use_sim_time true, the sim publishes /clock,
    possibly faster than real time) ``now`` is the node's ROS time in seconds and ``sleep`` /
    ``wait_until`` block until /clock has advanced that far. The constructor blocks until the
    first /clock message and seeds the node clock with it (the executor is not spinning yet),
    so no stamp is ever taken on a zero / foreign clock.
    """

    def __init__(self, node):
        try:
            self.sim = bool(node.get_parameter('use_sim_time').value)
        except Exception:  # noqa: BLE001 - not declared: wall clock
            self.sim = False
        if not self.sim:
            self.now = time.monotonic
            self.sleep = time.sleep
            return
        from rclpy.clock import JumpThreshold
        from rclpy.duration import Duration
        self._clock = node.get_clock()
        self._ctx = node.context
        self._cv = threading.Condition()
        self._jump = self._clock.create_jump_callback(
            JumpThreshold(min_forward=Duration(nanoseconds=1), min_backward=None,
                          on_clock_change=True), post_callback=self._on_jump)
        self.now = self._sim_now
        self.sleep = self._sim_sleep
        wait_first_clock(node)

    def _sim_now(self):
        return self._clock.now().nanoseconds * 1e-9

    def _on_jump(self, _jump):
        with self._cv:
            self._cv.notify_all()

    def wait_until(self, t, stop=None):
        """Block until ``now() >= t`` (or ``stop`` is set / the context shuts down)."""
        if not self.sim:
            wait = t - time.monotonic()
            if wait > 0:
                if stop is not None:
                    stop.wait(wait)
                else:
                    time.sleep(wait)
            return
        with self._cv:
            while (self._sim_now() < t and self._ctx.ok()
                   and not (stop is not None and stop.is_set())):
                self._cv.wait(0.1)

    def _sim_sleep(self, seconds):
        self.wait_until(self._sim_now() + max(0.0, float(seconds)))


# the production clock as a NodeClock stand-in (tests that stub a node without __init__)
WALL_CLOCK = types.SimpleNamespace(sim=False, now=time.monotonic, sleep=time.sleep)


def wait_first_clock(node, topic='/clock'):
    """use_sim_time: block until the first /clock message and seed the node's ROS clock with
    it (the TimeSource takes over once the executor spins)."""
    clock = node.get_clock()
    if clock.now().nanoseconds:
        return
    from rclpy.time import Time
    from rclpy.wait_for_message import wait_for_message
    from rosgraph_msgs.msg import Clock as ClockMsg
    while node.context.ok():
        ok, msg = wait_for_message(ClockMsg, node, topic, time_to_wait=10.0)
        if ok and (msg.clock.sec or msg.clock.nanosec):
            if not clock.now().nanoseconds:
                clock.set_ros_time_override(Time.from_msg(msg.clock))
            return
        node.get_logger().warn('use_sim_time: waiting for %s' % topic)


def node_clock(node):
    """The node's :class:`NodeClock` (one per node, created on first use)."""
    clk = getattr(node, '_mower_node_clock', None)
    if clk is None:
        clk = NodeClock(node)
        node._mower_node_clock = clk
    return clk


def node_now(node):
    """``time.monotonic``, or the node's ROS clock in seconds under use_sim_time (blocks until
    the first /clock, see :class:`NodeClock`)."""
    return node_clock(node).now
