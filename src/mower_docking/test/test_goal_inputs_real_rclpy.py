# SPDX-License-Identifier: Apache-2.0
"""Goal-only inputs: subscribed for a goal, fresh before it starts, gone afterwards.

Needs a real ROS 2 Humble environment; skipped elsewhere.
"""

import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('mower_interfaces.msg')
pytest.importorskip('nav2_msgs.action')

from nav_msgs.msg import Odometry  # noqa: E402
from mower_interfaces.msg import MowerBaseDevStatus  # noqa: E402


@pytest.fixture
def ctx():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _topics(node):
    return {name for name, _ in node.get_topic_names_and_types()}


def _sub_count(node, topic):
    return sum(1 for i in node.get_subscriptions_info_by_topic(topic)
               if i.node_name == 'mower_docking')


def test_goal_inputs_lifecycle(ctx):
    from mower_docking.docking_node import DockingServer

    node = DockingServer()
    helper = rclpy.create_node('dock_inputs_feeder')
    status_pub = helper.create_publisher(MowerBaseDevStatus, '/mower_base/status', 10)
    odom_pub = helper.create_publisher(Odometry, '/odom', 10)
    stop = threading.Event()

    def feed():
        k = 0
        while not stop.is_set():
            st = MowerBaseDevStatus()
            st.is_docking_done = True
            st.lift_triggered = (k % 2 == 0)
            status_pub.publish(st)
            if k % 2 == 0:
                od = Odometry()
                od.pose.pose.position.x = 1.0 + k
                od.pose.pose.orientation.w = 1.0
                odom_pub.publish(od)
            k += 1
            time.sleep(0.01)

    feeder = threading.Thread(target=feed, daemon=True)
    feeder.start()
    try:
        time.sleep(0.5)
        # idle: no subscription to the goal-only topics at all
        assert _sub_count(helper, '/mower_base/status') == 0
        assert _sub_count(helper, '/odom') == 0

        t0 = time.monotonic()
        node._resume_inputs(timeout=3.0)
        assert time.monotonic() - t0 < 3.0, 'inputs did not arrive (discovery?)'
        snap = node._snapshot()
        assert snap.odom is not None and snap.odom.x >= 1.0
        assert snap.status_fresh
        assert snap.contact                     # n consecutive is_docking_done samples
        assert _sub_count(helper, '/mower_base/status') == 1
        assert _sub_count(helper, '/odom') == 1

        node._release_inputs()
        time.sleep(0.5)
        assert _sub_count(helper, '/mower_base/status') == 0
        assert _sub_count(helper, '/odom') == 0
        before = node._status_count
        time.sleep(0.2)
        assert node._status_count == before     # nothing delivered any more
    finally:
        stop.set()
        feeder.join(1.0)
        helper.destroy_node()
        node.destroy_node()
