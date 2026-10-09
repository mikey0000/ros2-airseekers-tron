# SPDX-License-Identifier: Apache-2.0
"""Scenario runner: send a Nav2 NavigateToPose goal against a running sim and
judge the run from the sim's ground truth (/sim/status).

    ros2 run mower_sim scenario navigate --x 8.0 --y 0.0 [--yaw 0.0] [--timeout 180]
    ros2 run mower_sim scenario dock [--no-vision]

Prints one JSON line with the metrics and exits 0 when the run passes.
navigate: goal SUCCEEDED, final ground-truth pose within ``--goal-tol`` of the goal,
no collision, and no stall (robot moved < ``stall_dist`` m for more than
``--max-stall`` s while the goal was active).
dock: /mower_docking/dock succeeded, the sim reports dock contact and charging, no
collision.

``--stub-state`` (default TRANSIT for navigate, RETURNING_HOME for dock; '' = off):
cmd_vel_slew's motion gate only passes wheel commands while the mission is in a
motion phase and re-asserts the latched /motion_enabled. Run the sim with
``mission:=false mission_stub:=true`` and the client stands in for the mission by
publishing /motion_enabled true and /behavior_tree_node/high_level_status with that
state. With the real mission running pass ``--stub-state ''`` and drive it through
its own services instead.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import asdict, dataclass, field
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


@dataclass
class RunMetrics:
    status: str = 'NOT_RUN'
    duration_s: float = 0.0
    final_x: float = math.nan
    final_y: float = math.nan
    goal_error_m: float = math.nan
    collisions: int = 0
    min_clearance_m: Optional[float] = None
    max_stall_s: float = 0.0
    max_abs_y_m: float = 0.0
    path_length_m: float = 0.0
    passed: bool = False
    reasons: List[str] = field(default_factory=list)


STATUS_NAMES = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                GoalStatus.STATUS_CANCELED: 'CANCELED'}


class ScenarioClient(Node):
    def __init__(self, name='mower_sim_scenario', stub_state=''):
        super().__init__(name)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.status = None
        self.create_subscription(String, '/sim/status', self._on_status, 10)
        self.stub_state = stub_state
        if stub_state:
            from mowgli_interfaces.msg import HighLevelStatus
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self._motion_pub = self.create_publisher(Bool, '/motion_enabled', latched)
            self._hl_pub = self.create_publisher(
                HighLevelStatus, '/behavior_tree_node/high_level_status', 10)
            self._hl = HighLevelStatus()
            self._hl.state = HighLevelStatus.HIGH_LEVEL_STATE_AUTONOMOUS
            self._hl.state_name = stub_state
            self._hl.emergency = False
            self.create_timer(0.5, self._stub_tick)
            self._stub_tick()

    def _stub_tick(self):
        self._motion_pub.publish(Bool(data=True))
        self._hl_pub.publish(self._hl)

    def _on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            pass

    def spin_until(self, pred, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if pred():
                return True
        return False

    def wait_ready(self, timeout=90.0) -> bool:
        """Sim status flowing and bt_navigator ACTIVE (its action server exists from
        configure on, but rejects goals until activated)."""
        end = time.monotonic() + timeout
        if not self.spin_until(lambda: self.status is not None, timeout):
            return False
        if not self.nav.wait_for_server(timeout_sec=max(0.1, end - time.monotonic())):
            return False
        cli = self.create_client(GetState, '/bt_navigator/get_state')
        try:
            while time.monotonic() < end:
                if cli.wait_for_service(timeout_sec=1.0):
                    fut = cli.call_async(GetState.Request())
                    if self.spin_until(fut.done, 5.0) and fut.result() is not None and \
                            fut.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
                        # costmaps fill from the first sensor frames
                        self.spin_until(lambda: False, 3.0)
                        return True
                self.spin_until(lambda: False, 1.0)
            return False
        finally:
            self.destroy_client(cli)

    def navigate(self, x, y, yaw=0.0, timeout=180.0, goal_tol=0.5, max_stall=20.0,
                 stall_dist=0.05) -> RunMetrics:
        m = RunMetrics()
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        start_collisions = int((self.status or {}).get('collisions', 0))
        t0 = time.monotonic()
        fut = self.nav.send_goal_async(goal)
        if not self.spin_until(fut.done, 10.0) or not fut.result().accepted:
            m.status = 'REJECTED'
            m.reasons.append('goal rejected')
            return m
        res_fut = fut.result().get_result_async()
        anchor = None          # (t, x, y) of the last time the robot moved stall_dist
        last_xy = None
        while time.monotonic() - t0 < timeout and not res_fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            st = self.status
            if not st:
                continue
            now = time.monotonic()
            xy = (st['x'], st['y'])
            if last_xy is not None:
                m.path_length_m += math.hypot(xy[0] - last_xy[0], xy[1] - last_xy[1])
            last_xy = xy
            m.max_abs_y_m = max(m.max_abs_y_m, abs(xy[1]))
            if anchor is None or math.hypot(xy[0] - anchor[1], xy[1] - anchor[2]) > stall_dist:
                anchor = (now, xy[0], xy[1])
            m.max_stall_s = max(m.max_stall_s, now - anchor[0])
        m.duration_s = time.monotonic() - t0
        if res_fut.done():
            m.status = STATUS_NAMES.get(res_fut.result().status, str(res_fut.result().status))
        else:
            m.status = 'TIMEOUT'
            fut.result().cancel_goal_async()
            self.spin_until(lambda: False, 1.0)
        self.spin_until(lambda: False, 0.5)   # one more status sample
        st = self.status or {}
        m.final_x, m.final_y = st.get('x', math.nan), st.get('y', math.nan)
        m.goal_error_m = math.hypot(m.final_x - x, m.final_y - y)
        m.collisions = int(st.get('collisions', 0)) - start_collisions
        m.min_clearance_m = st.get('min_clearance')
        if m.status != 'SUCCEEDED':
            m.reasons.append('nav result %s' % m.status)
        if not m.goal_error_m <= goal_tol:
            m.reasons.append('final pose %.2f m from the goal (> %.2f)' % (m.goal_error_m,
                                                                         goal_tol))
        if m.collisions:
            m.reasons.append('%d collision(s), last contact %s'
                             % (m.collisions, st.get('last_contact')))
        if m.max_stall_s > max_stall:
            m.reasons.append('stalled %.1f s (> %.1f s)' % (m.max_stall_s, max_stall))
        m.passed = not m.reasons
        return m

    def dock(self, use_vision=True, timeout=240.0) -> RunMetrics:
        from mower_interfaces.action import Dock
        m = RunMetrics()
        client = ActionClient(self, Dock, '/mower_docking/dock')
        if not client.wait_for_server(timeout_sec=30.0):
            m.status = 'NO_SERVER'
            m.reasons.append('/mower_docking/dock not available')
            return m
        goal = Dock.Goal()
        goal.use_vision = bool(use_vision)
        goal.timeout_s = float(timeout)
        start_collisions = int((self.status or {}).get('collisions', 0))
        t0 = time.monotonic()
        fut = client.send_goal_async(goal, feedback_callback=self._on_dock_feedback)
        if not self.spin_until(fut.done, 10.0) or not fut.result().accepted:
            m.status = 'REJECTED'
            m.reasons.append('dock goal rejected')
            return m
        res_fut = fut.result().get_result_async()
        self.spin_until(res_fut.done, timeout + 10.0)
        m.duration_s = time.monotonic() - t0
        self.spin_until(lambda: False, 1.0)
        st = self.status or {}
        m.final_x, m.final_y = st.get('x', math.nan), st.get('y', math.nan)
        m.collisions = int(st.get('collisions', 0)) - start_collisions
        m.min_clearance_m = st.get('min_clearance')
        if not res_fut.done():
            m.status = 'TIMEOUT'
            fut.result().cancel_goal_async()
            m.reasons.append('dock timed out')
        else:
            r = res_fut.result()
            m.status = STATUS_NAMES.get(r.status, str(r.status))
            if not r.result.success:
                m.reasons.append('dock failed: %s' % r.result.message)
        if not st.get('docked'):
            m.reasons.append('sim: no dock contact')
        if not st.get('charging'):
            m.reasons.append('sim: not charging')
        if m.collisions:
            m.reasons.append('%d collision(s)' % m.collisions)
        m.passed = not m.reasons
        return m

    def _on_dock_feedback(self, fb):
        f = fb.feedback
        if f.state != getattr(self, '_last_dock_state', None):
            self._last_dock_state = f.state
            self.get_logger().info('dock: %s (%.2f m, retries %d)'
                                   % (f.state, f.distance_m, f.retries))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    nav = sub.add_parser('navigate', help='NavigateToPose to (x, y, yaw) in map')
    nav.add_argument('--x', type=float, required=True)
    nav.add_argument('--y', type=float, required=True)
    nav.add_argument('--yaw', type=float, default=0.0)
    nav.add_argument('--goal-tol', type=float, default=0.5)
    nav.add_argument('--max-stall', type=float, default=20.0)
    nav.add_argument('--timeout', type=float, default=180.0)
    dock = sub.add_parser('dock', help='/mower_docking/dock from the current pose')
    dock.add_argument('--no-vision', action='store_true')
    dock.add_argument('--timeout', type=float, default=240.0)
    for p in (nav, dock):
        p.add_argument('--ready-timeout', type=float, default=90.0)
        p.add_argument('--stub-state', default=None,
                       help="mission stand-in state ('' = off; default TRANSIT / "
                            "RETURNING_HOME)")
    a, rest = ap.parse_known_args(argv)
    stub = a.stub_state
    if stub is None:
        stub = 'TRANSIT' if a.cmd == 'navigate' else 'RETURNING_HOME'
    rclpy.init(args=rest)
    node = ScenarioClient(stub_state=stub)
    try:
        if not node.wait_ready(a.ready_timeout):
            print(json.dumps({'passed': False, 'reasons': ['sim / Nav2 not ready']}))
            return 1
        if a.cmd == 'navigate':
            m = node.navigate(a.x, a.y, a.yaw, a.timeout, a.goal_tol, a.max_stall)
        else:
            m = node.dock(not a.no_vision, a.timeout)
        print(json.dumps(asdict(m)))
        return 0 if m.passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
