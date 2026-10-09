# SPDX-License-Identifier: GPL-3.0-or-later
"""obstacle_guard rear handling end to end: corridor, dock awareness, latched Bool outputs."""
import json
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('vision_msgs')

from geometry_msgs.msg import PoseStamped, Twist  # noqa: E402
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,  # noqa: E402
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool, String  # noqa: E402
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose  # noqa: E402

from mower_vision import obstacle_guard_node as og  # noqa: E402

LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE)


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


class Rig:
    """Guard + feeder node with helpers; everything runs in real time."""

    def __init__(self):
        self.guard = og.ObstacleGuard()
        self.feeder = rclpy.create_node('rear_feeder')
        f = self.feeder
        self.state_pub = f.create_publisher(String, '/mower_docking/state', 10)
        self.marker_pub = f.create_publisher(PoseStamped, '/mower_docking/marker_pose', 10)
        self.det_pub = f.create_publisher(Detection2DArray, '/ai/det/detections_ranged', 10)
        self.cmd_pub = f.create_publisher(Twist, '/cmd_vel', 10)
        self.blocked = None
        self.watch = None
        self.policy = None
        f.create_subscription(Bool, '/vision/rear_blocked',
                              lambda m: setattr(self, 'blocked', m.data), LATCHED)
        f.create_subscription(Bool, '/vision/rear_watch',
                              lambda m: setattr(self, 'watch', m.data), LATCHED)
        f.create_subscription(String, '/obstacle_policy',
                              lambda m: setattr(self, 'policy', json.loads(m.data)), LATCHED)
        self.dock_state = None
        self.marker = None
        self.person = None
        self.reverse = False

    def spin(self, seconds, until=None):
        """Spin both nodes, republishing the 20 Hz style inputs; stop early on until()."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self.dock_state:
                self.state_pub.publish(String(data=self.dock_state))
            if self.marker:
                self.marker_pub.publish(self.marker)
            if self.reverse:
                tw = Twist()
                tw.linear.x = -0.2
                self.cmd_pub.publish(tw)
            if self.person is not None:
                self.det_pub.publish(self.person)
            for n in (self.feeder, self.guard):      # spin_once runs one callback: drain
                for _ in range(6):
                    rclpy.spin_once(n, timeout_sec=0.0)
            if until is not None and until():
                return True
            time.sleep(0.04)
        return until() if until is not None else False

    def wait_connected(self):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and (
                self.det_pub.get_subscription_count() == 0
                or self.state_pub.get_subscription_count() == 0
                or self.marker_pub.get_subscription_count() == 0):
            rclpy.spin_once(self.guard, timeout_sec=0.02)
        assert self.spin(5.0, lambda: self.watch is not None and self.blocked is not None)

    def set_marker(self, x, y, z):
        m = PoseStamped()
        m.header.frame_id = 'base_link'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        self.marker = m

    def set_person(self, x, y):
        det = Detection2D()
        det.header.frame_id = 'rear_camera'
        det.bbox.center.position.x, det.bbox.center.position.y = 640.0, 500.0
        det.bbox.size_x, det.bbox.size_y = 100.0, 400.0
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = 'person'
        hyp.hypothesis.score = 0.9
        hyp.pose.pose.position.x, hyp.pose.pose.position.y = x, y
        hyp.pose.covariance[0] = 0.01
        det.results.append(hyp)
        arr = Detection2DArray()
        arr.header.frame_id = 'rear_camera'
        arr.detections.append(det)
        self.person = arr

    def close(self):
        self.feeder.destroy_node()
        self.guard.destroy_node()


@pytest.fixture
def rig(ros):
    r = Rig()
    try:
        yield r
    finally:
        r.close()


def test_dock_scenario_behind_ignored_between_blocks(rig):
    rig.wait_connected()
    rig.dock_state = 'DOCKING'
    rig.set_marker(-1.4, 0.0, 0.15)
    assert rig.spin(5.0, lambda: rig.watch is True)
    rig.spin(0.5)                      # let the marker arrive

    rig.set_person(-1.75, 0.0)         # behind the dock
    assert not rig.spin(2.0, lambda: rig.blocked is True)
    assert rig.blocked is False
    assert rig.watch is True
    assert (rig.policy or {}).get('kind', 'none') == 'none'

    rig.set_person(-0.8, 0.0)          # between robot and dock
    assert rig.spin(5.0, lambda: rig.blocked is True)
    assert rig.spin(3.0, lambda: (rig.policy or {}).get('kind') == 'dynamic')
    assert rig.policy['camera'] == 'rear'
    assert rig.policy['class'] == 'person'


def test_no_dock_no_reverse_never_blocks(rig):
    rig.wait_connected()
    rig.set_person(-0.8, 0.0)
    assert not rig.spin(2.0, lambda: rig.blocked is True)
    assert rig.blocked is False
    assert (rig.policy or {}).get('kind', 'none') == 'none'


def test_reversing_without_dock_blocks(rig):
    rig.wait_connected()
    rig.reverse = True
    rig.set_person(-0.8, 0.0)
    assert rig.spin(5.0, lambda: rig.blocked is True)


def test_level_none_disables_rear(rig):
    rig.wait_connected()
    rig.dock_state = 'DOCKING'
    assert rig.spin(5.0, lambda: rig.watch is True)
    res = rig.guard.set_parameters([rclpy.parameter.Parameter('obstacle_detection', value='none')])
    assert res[0].successful
    assert rig.spin(5.0, lambda: rig.watch is False)
    rig.set_person(-0.8, 0.0)
    assert not rig.spin(2.0, lambda: rig.blocked is True)
    assert rig.blocked is False and rig.watch is False
