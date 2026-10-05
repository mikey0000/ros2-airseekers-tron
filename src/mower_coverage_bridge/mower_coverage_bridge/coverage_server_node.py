# Copyright 2026 michael
# SPDX-License-Identifier: Apache-2.0
"""coverage_server: /plan_coverage action (mowgli_interfaces) over /coverage/plan.

Adapter between the MowgliNext mission layer, which plans coverage through
the ``mowgli_interfaces/action/PlanCoverage`` action, and our Fields2Cover 2.1
planner, which is the ``mower_interfaces/srv/PlanCoverage`` service of
``mower_coverage``. The geometry work (segments, drivable sub-paths) is in
``splitter.py``.
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Point32, Polygon, PoseStamped
from mower_interfaces.srv import PlanCoverage as PlanCoverageSrv
from mowgli_interfaces.action import PlanCoverage
from nav_msgs.msg import Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mower_coverage_bridge import splitter


def _poly_points(poly):
    return [(float(p.x), float(p.y)) for p in poly.points]


def _copy_polygon(poly):
    out = Polygon()
    out.points = [Point32(x=float(p.x), y=float(p.y), z=0.0) for p in poly.points]
    return out


class CoverageServer(Node):

    def __init__(self):
        super().__init__('coverage_server')
        p = self.declare_parameter
        p('action_name', '/plan_coverage')
        p('service_name', '/coverage/plan')
        p('frame_id', 'map')
        # Must match mowgli_interfaces::coverage_geometry::kSegmentTransitGapM.
        p('transit_gap_m', splitter.DEFAULT_TRANSIT_GAP_M)
        p('turn_split_deg', splitter.DEFAULT_TURN_SPLIT_DEG)
        p('densify_step_m', 0.10)
        p('split_at_keepouts', True)
        p('service_wait_s', 2.0)
        p('service_timeout_s', 30.0)
        # Forwarded to /coverage/plan; <= 0 (or 0 passes) = planner default.
        p('operation_width', 0.0)
        p('headland_width', 0.0)
        p('headland_passes', 0)
        p('min_swath_length', 0.0)

        self._client_group = ReentrantCallbackGroup()
        self._client = self.create_client(
            PlanCoverageSrv, self._str('service_name'), callback_group=self._client_group)
        self._server = ActionServer(
            self, PlanCoverage, self._str('action_name'),
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(
            'coverage_server ready: action %s -> service %s (transit_gap_m=%.2f)' % (
                self._str('action_name'), self._str('service_name'),
                self._float('transit_gap_m')))

    # ---- parameter helpers (read live so `ros2 param set` applies per plan) ----
    def _str(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    def _float(self, name):
        v = self.get_parameter(name).value
        return float(v)

    def _int(self, name):
        return int(self.get_parameter(name).value)

    def _bool(self, name):
        return bool(self.get_parameter(name).value)

    # ---- action ----
    def _on_goal(self, goal):
        if len(goal.outer_boundary.points) < 3:
            self.get_logger().warn('rejecting goal: outer_boundary needs >= 3 points')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _feedback(self, goal_handle, phase):
        fb = PlanCoverage.Feedback()
        fb.phase = phase
        goal_handle.publish_feedback(fb)
        self.get_logger().info('PlanCoverage: %s' % phase)

    def _fail(self, goal_handle, msg, t0, canceled=False):
        res = PlanCoverage.Result()
        res.success = False
        res.message = msg
        res.planning_time_s = time.monotonic() - t0
        if canceled:
            goal_handle.canceled()
        else:
            self.get_logger().error('PlanCoverage aborted: %s' % msg)
            goal_handle.abort()
        return res

    def _call_planner(self, goal, mow_angle_deg):
        """Synchronous /coverage/plan call. Returns (response, error)."""
        req = PlanCoverageSrv.Request()
        req.boundary = _copy_polygon(goal.outer_boundary)
        req.holes = [_copy_polygon(h) for h in goal.obstacles if len(h.points) >= 3]
        req.frame_id = self._str('frame_id')
        req.has_start = False
        req.has_goal = False
        req.operation_width = self._float('operation_width')
        req.headland_width = self._float('headland_width')
        req.headland_passes = self._int('headland_passes')
        req.mow_angle_deg = float(mow_angle_deg)
        req.min_swath_length = self._float('min_swath_length')

        done = threading.Event()
        future = self._client.call_async(req)
        future.add_done_callback(lambda _f: done.set())
        timeout = self._float('service_timeout_s')
        if not done.wait(timeout):
            future.cancel()
            return None, '%s did not answer within %.1f s' % (self._str('service_name'), timeout)
        if future.exception() is not None:
            return None, '%s raised: %s' % (self._str('service_name'), future.exception())
        return future.result(), None

    def _execute(self, goal_handle):
        t0 = time.monotonic()
        goal = goal_handle.request
        service = self._str('service_name')

        self._feedback(goal_handle, 'waiting for %s' % service)
        if not self._client.wait_for_service(timeout_sec=self._float('service_wait_s')):
            return self._fail(
                goal_handle,
                'coverage planner service %s unavailable (is mower_coverage_node running?)'
                % service, t0)

        angle = goal.mow_angle_deg
        if goal.perpendicular and angle >= 0.0:
            angle = (angle + 90.0) % 180.0
        self._feedback(goal_handle, 'planning (mow_angle_deg=%s)' % (
            'auto' if angle < 0.0 else '%.1f' % angle))
        resp, err = self._call_planner(goal, angle)
        if err is None and resp.success and goal.perpendicular and goal.mow_angle_deg < 0.0:
            # AUTO + perpendicular: learn the auto angle, then re-plan at +90 deg.
            segs, _ = splitter.split_segments(
                [(ps.pose.position.x, ps.pose.position.y) for ps in resp.path.poses],
                resp.ring_count, resp.swath_count)
            auto = splitter.swath_angle_deg(segs)
            if auto is not None:
                angle = (auto + 90.0) % 180.0
                self._feedback(goal_handle, 'perpendicular re-plan at %.1f deg' % angle)
                resp, err = self._call_planner(goal, angle)
        if err is not None:
            return self._fail(goal_handle, err, t0)
        if goal_handle.is_cancel_requested:
            return self._fail(goal_handle, 'canceled', t0, canceled=True)
        if not resp.success or len(resp.path.poses) < 2:
            return self._fail(goal_handle, 'planner failed: %s' % resp.message, t0)

        self._feedback(goal_handle, 'splitting %d poses (%d rings, %d swaths)' % (
            len(resp.path.poses), resp.ring_count, resp.swath_count))
        gap = self._float('transit_gap_m')
        fences, holes = [], []
        if self._bool('split_at_keepouts'):
            fences = [_poly_points(goal.outer_boundary)]
            holes = [_poly_points(h) for h in goal.obstacles if len(h.points) >= 3]
        result = splitter.plan(
            [(ps.pose.position.x, ps.pose.position.y) for ps in resp.path.poses],
            ring_count=resp.ring_count, swath_count=resp.swath_count,
            transit_gap_m=gap, turn_split_deg=self._float('turn_split_deg'),
            fences=fences, holes=holes)
        if result.mode != splitter.MODE_STRUCTURAL:
            self.get_logger().warn(
                'planner path did not match its ring/swath counts (%d/%d); '
                'used the heuristic gap/turn split' % (resp.ring_count, resp.swath_count))
        if not result.subpaths:
            return self._fail(goal_handle, 'planner path produced no drivable sub-path', t0)

        res = self._build_result(result, resp)
        res.planning_time_s = time.monotonic() - t0
        summary = splitter.summary(result)
        res.message = '%s | planner: %s' % (summary, resp.message)
        self._feedback(goal_handle, 'done: %s' % summary)
        goal_handle.succeed()
        return res

    def _build_result(self, result, resp):
        frame = self._str('frame_id')
        step = self._float('densify_step_m')
        stamp = self.get_clock().now().to_msg()

        def to_path(points):
            pts = splitter.densify(points, step)
            path = Path()
            path.header.frame_id = frame
            path.header.stamp = stamp
            for (x, y), yaw in zip(pts, splitter.yaws(pts)):
                ps = PoseStamped()
                ps.header = path.header
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.orientation.z = math.sin(yaw / 2.0)
                ps.pose.orientation.w = math.cos(yaw / 2.0)
                path.poses.append(ps)
            return path

        res = PlanCoverage.Result()
        res.success = True
        res.segments = [to_path(s.points) for s in result.segments]
        res.segment_types = [int(s.kind) for s in result.segments]
        res.drivable_subpaths = [to_path(s.points) for s in result.subpaths]
        res.full_path = Path()
        res.full_path.header.frame_id = frame
        res.full_path.header.stamp = stamp
        for sp in res.drivable_subpaths:
            res.full_path.poses.extend(sp.poses)
        # Same convention as upstream mowgli_coverage: length of what is driven
        # blade-on (sub-paths including in-sub-path connectors).
        res.total_distance = float(sum(s.length for s in result.subpaths))
        res.ring_count = result.ring_count
        res.swath_count = result.swath_count
        return res


def main(args=None):
    rclpy.init(args=args)
    node = CoverageServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
