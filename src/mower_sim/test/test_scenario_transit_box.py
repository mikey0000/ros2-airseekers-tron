# SPDX-License-Identifier: Apache-2.0
"""Headless regression test for obstacle rerouting (launch_testing, ~1 min).

Launches launch/sim.launch.py on worlds/transit_box.yaml (real Nav2, twist_mux,
cmd_vel_slew, bumper_controller, map server; mission replaced by the scenario
client's stand-in) and sends NavigateToPose (8, 0) from (1.5, 0) with a 0.6 m box
that is not in the map on the straight line. Passes when Nav2 reports SUCCEEDED,
the ground-truth pose ends within 0.5 m of the goal, the footprint never touched
the box, the robot actually went round it, and it never sat still for > 20 s.

Runs under ``colcon test`` in the dev image (needs Nav2 + the built workspace);
skipped where launch_testing / Nav2 are missing. Own DDS domain
(MOWER_SIM_ROS_DOMAIN_ID, default 150 + pid % 50, localhost only) so parallel
runs and the robot never see this graph.
"""

import os
import unittest

import pytest

os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_DOMAIN_ID'] = os.environ.get('MOWER_SIM_ROS_DOMAIN_ID',
                                             str(150 + os.getpid() % 50))

launch_testing = pytest.importorskip('launch_testing')
pytest.importorskip('nav2_msgs')
pytest.importorskip('launch_ros')

import launch_testing.actions  # noqa: E402
from launch import LaunchDescription  # noqa: E402
from launch.actions import IncludeLaunchDescription  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402

GOAL = (8.0, 0.0)


def _sim_launch():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('mower_bringup'), 'launch',
                        'sim.launch.py')


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_sim_launch()),
        launch_arguments={
            'world': 'transit_box.yaml',
            'mission': 'false', 'mission_stub': 'true',
            'gui_bridge': 'false', 'docking': 'false', 'coverage': 'false',
            'log_level': 'warn',
        }.items())
    return LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


class TestTransitBox(unittest.TestCase):

    def test_reroute_around_box(self):
        import rclpy
        from mower_sim.scenario import ScenarioClient
        rclpy.init()
        node = ScenarioClient('test_transit_box', stub_state='TRANSIT')
        try:
            self.assertTrue(node.wait_ready(120.0), 'sim / Nav2 never became ready')
            m = node.navigate(GOAL[0], GOAL[1], 0.0, timeout=180.0, goal_tol=0.5,
                              max_stall=20.0)
            print('transit_box metrics: %s' % m)
            self.assertTrue(m.passed, '; '.join(m.reasons))
            self.assertEqual(m.collisions, 0)
            # it went round the box (box half width 0.3 + robot half width 0.27)
            self.assertGreater(m.max_abs_y_m, 0.4, 'robot did not deviate around the box')
            self.assertIsNotNone(m.min_clearance_m)
            self.assertGreater(m.min_clearance_m, 0.05)
        finally:
            node.destroy_node()
            rclpy.shutdown()
