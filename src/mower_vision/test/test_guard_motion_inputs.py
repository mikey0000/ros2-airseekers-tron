# SPDX-License-Identifier: GPL-3.0-or-later
"""obstacle_guard takes /cmd_vel and /odom in batches (sub_pump) when it handles a tick."""
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('vision_msgs')

from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402

from mower_vision import obstacle_guard_node as og  # noqa: E402


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def test_motion_inputs_batched_with_receive_times(ros):
    guard = og.ObstacleGuard()
    feeder = rclpy.create_node('guard_motion_feeder')
    try:
        cmd = feeder.create_publisher(Twist, '/cmd_vel', 10)
        odom = feeder.create_publisher(Odometry, '/odom', 10)
        deadline = time.monotonic() + 5.0
        while (time.monotonic() < deadline and (cmd.get_subscription_count() == 0
                                                or odom.get_subscription_count() == 0)):
            time.sleep(0.05)
        t0 = time.monotonic()
        tw = Twist()
        tw.linear.x = 0.4
        tw.angular.z = 0.6
        o = Odometry()
        o.twist.twist.angular.z = -0.7
        for _ in range(20):            # nothing reaches the tracker until the node polls
            cmd.publish(tw)
            odom.publish(o)
            time.sleep(0.01)
        assert guard.motion._last_fwd is None
        time.sleep(0.2)
        guard._on_tick()
        assert guard.motion._last_fwd is not None
        assert t0 <= guard.motion._last_fwd <= time.monotonic()
        assert guard.motion._last_left is not None
        assert guard.motion._odom_w == pytest.approx(-0.7)
    finally:
        feeder.destroy_node()
        guard.destroy_node()
