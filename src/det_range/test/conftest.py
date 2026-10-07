# --- ROS test isolation (keep identical in every package's test/conftest.py) ---------------
# Tests that create real rclpy nodes/publishers must never reach a live robot: on 2026-10-07 a
# dev-host `colcon test` (docker --network host, domain 0) injected /odometry/filtered_map
# x=27,y=0 (test_goal_inputs_real_rclpy feeder) into the mowing robot's graph over the LAN and
# latched BOUNDARY_EMERGENCY_STOP. Force localhost-only discovery on a private domain before
# anything imports rclpy. MOWER_TEST_ROS_DOMAIN_ID overrides the domain.
import os as _iso_os  # noqa: E402
_iso_os.environ['ROS_LOCALHOST_ONLY'] = '1'
_iso_os.environ['ROS_DOMAIN_ID'] = _iso_os.environ.get('MOWER_TEST_ROS_DOMAIN_ID', '77')
# -------------------------------------------------------------------------------------------

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
