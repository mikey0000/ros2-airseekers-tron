# SPDX-License-Identifier: Apache-2.0
"""Cancelling a running /mower_docking/dock goal publishes zero velocity immediately.

Needs a real ROS 2 Jazzy environment; skipped elsewhere. Runs on an isolated
ROS domain and a test-only cmd_vel topic, so it never reaches twist_mux.
"""

import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('mower_interfaces.action')
pytest.importorskip('nav2_msgs.action')

from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from mower_interfaces.action import Dock  # noqa: E402
from mower_interfaces.msg import MowerBaseDevStatus  # noqa: E402

DOMAIN = 87
CMD = '/test_cmd_vel_docking_cancel'


def test_cancel_publishes_zero_immediately():
    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init(domain_id=DOMAIN, args=[
        '--ros-args', '-p', 'mower_docking:cmd_vel_topic:=' + CMD,
        '-p', 'mower_docking:skip_nav_to_approach:=true',
        '-p', 'mower_docking:allow_blind_docking:=true',
        '-p', 'mower_docking:progress_odom_topic:=/test_no_progress_odom',
        '-p', 'mower_docking:stop_burst_s:=0.3'])
    from mower_docking.docking_node import DockingServer
    stop = threading.Event()
    ex = node = None
    try:
        node = DockingServer()
        assert node.get_parameter('cmd_vel_topic').value == CMD
        helper = rclpy.create_node('dock_cancel_helper')
        cmds = []
        helper.create_subscription(Twist, CMD, lambda m: cmds.append((time.monotonic(),
                                                                      m.linear.x)), 50)
        status_pub = helper.create_publisher(MowerBaseDevStatus, '/mower_base/status', 10)
        odom_pub = helper.create_publisher(Odometry, '/odom', 10)
        client = ActionClient(helper, Dock, '/mower_docking/dock')

        def feed():
            while not stop.is_set():
                status_pub.publish(MowerBaseDevStatus())
                od = Odometry()
                od.pose.pose.orientation.w = 1.0
                odom_pub.publish(od)
                time.sleep(0.02)
        threading.Thread(target=feed, daemon=True).start()
        ex = MultiThreadedExecutor(num_threads=6)
        ex.add_node(node)
        ex.add_node(helper)
        threading.Thread(target=ex.spin, daemon=True).start()

        assert client.wait_for_server(timeout_sec=5.0)
        goal = Dock.Goal()
        goal.use_vision = False
        goal.timeout_s = 60.0
        fut = client.send_goal_async(goal)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 5.0:
            time.sleep(0.01)
        gh = fut.result()
        assert gh is not None and gh.accepted
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0 and not any(v < 0 for _, v in cmds):
            time.sleep(0.01)
        assert any(v < 0 for _, v in cmds), 'docking never reversed'
        t_cancel = time.monotonic()
        gh.cancel_goal_async()
        time.sleep(0.8)
        after = [(t, v) for t, v in cmds if t > t_cancel]
        zeros = [t for t, v in after if v == 0.0]
        assert zeros and zeros[0] - t_cancel < 0.2, 'no zero within 200 ms of cancel'
        # nothing but zeros once the first zero went out
        assert all(v == 0.0 for t, v in after if t >= zeros[0])
    finally:
        stop.set()
        if ex is not None:
            ex.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
