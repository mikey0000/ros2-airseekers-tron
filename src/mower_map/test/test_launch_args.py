"""Guard against launch-argument name collisions.

ROS 2 launch includes share one LaunchConfiguration context. nav2.launch.py declares
``odom_topic`` (= /odom, wheel odometry for the EKF); if map_server.launch.py declared the
same name, its default would be ignored and the boundary monitor would judge the robot
against drifting wheel odometry (this happened: every mow tripped the lethal boundary stop).
"""
import os
import re

HERE = os.path.dirname(__file__)
LAUNCH = os.path.join(HERE, '..', 'launch', 'map_server.launch.py')


def test_map_server_launch_does_not_declare_odom_topic():
    src = open(LAUNCH).read()
    declared = re.findall(r"\(\s*'([a-z_]+)'\s*,", src)
    assert 'odom_topic' not in declared
    assert 'map_odom_topic' in declared
    assert "overrides['odom_topic'] = overrides.pop('map_odom_topic')" in src
