# SPDX-License-Identifier: GPL-3.0-or-later
"""FollowPath goal paths are stamped 0 so controller_server re-transforms the goal into odom
with the latest map -> odom on every cycle (2026-11-06 slope world: with the send time as
stamp the corner checker judged a goal frozen 0.15-0.2 m off the vertex). Needs the ROS
message packages (skipped elsewhere)."""

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('nav_msgs.msg')
pytest.importorskip('nav2_msgs.action')
pytest.importorskip('mower_interfaces.msg')
pytest.importorskip('mowgli_interfaces.action')


class _Clock:
    def now(self):
        from rclpy.time import Time
        return Time(seconds=1234, nanoseconds=5)


class _Stub:
    """Just enough of MissionNode for _path / _pose_stamped."""

    def __init__(self):
        self._p = {'map_frame': 'map'}

    def get_clock(self):
        return _Clock()

    def _pose_stamped(self, x, y, yaw):
        from mower_mission.mission_node import MissionNode
        return MissionNode._pose_stamped(self, x, y, yaw)


def test_follow_path_goal_is_stamped_zero():
    from mower_mission.mission_node import MissionNode
    path = MissionNode._path(_Stub(), [(0.0, 0.0, 0.0), (1.0, 0.5), (2.0, 1.0, 0.3)])
    assert path.header.frame_id == 'map'
    stamps = [path.header.stamp] + [ps.header.stamp for ps in path.poses]
    assert all(s.sec == 0 and s.nanosec == 0 for s in stamps)
    assert [ps.header.frame_id for ps in path.poses] == ['map'] * 3
    assert path.poses[-1].pose.position.x == 2.0 and path.poses[-1].pose.position.y == 1.0
