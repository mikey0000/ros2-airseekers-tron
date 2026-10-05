#!/usr/bin/env python3
"""IMU publishing against *real* rclpy message types (skipped where only the stubs exist).

The stubs accept any covariance length; real ``sensor_msgs/Imu`` demands 9-element (3x3)
covariances, which is what this guards.
"""

import os
import struct
import sys
import unittest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for path in (PKG_DIR, os.path.join(PKG_DIR, 'test')):
    if path not in sys.path:
        sys.path.insert(0, path)

import ros_stubs  # noqa: E402

ros_stubs.install()

from mower_mcu_driver import mcu_node as mn  # noqa: E402


@unittest.skipIf(ros_stubs.is_stubbed(), 'real rclpy required (stubs do not type-check messages)')
class TestImuRealMessages(unittest.TestCase):

    def test_imu_message_is_valid_and_published(self):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu

        node = mn.McuNode()
        received = []
        helper = rclpy.create_node('imu_probe')
        try:
            helper.create_subscription(Imu, '/mcu/imu', received.append, qos_profile_sensor_data)
            node._on_imu(struct.pack(mn.IMU_FMT, 0, 0, 0, 0, 0, 2048, 0, 0, 0))
            for _ in range(20):
                rclpy.spin_once(helper, timeout_sec=0.05)
                if received:
                    break
            self.assertTrue(received, 'no /mcu/imu message received')
            self.assertEqual(len(received[0].orientation_covariance), 9)
            self.assertAlmostEqual(received[0].linear_acceleration.z, 9.80665, places=3)
        finally:
            helper.destroy_node()
            node.shutdown()
            node.destroy_node()


if __name__ == '__main__':
    unittest.main(verbosity=2)
