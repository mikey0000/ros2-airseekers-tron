#!/usr/bin/env python3
"""The pre-serialized /mower_base/status and /odom publishing equals the typed path.

Skipped where only the ROS stubs exist (the fast path needs real rclpy serialization).
"""

import os
import struct
import sys
import types
import unittest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for path in (PKG_DIR, os.path.join(PKG_DIR, 'test')):
    if path not in sys.path:
        sys.path.insert(0, path)

import ros_stubs  # noqa: E402

ros_stubs.install()

from mower_mcu_driver import mcu_node as mn  # noqa: E402


@unittest.skipIf(ros_stubs.is_stubbed() or mn.SubscriptionPump is None
                 or mn.MowerBaseDevStatus is None, 'real rclpy + mower_interfaces required')
class TestFastPublish(unittest.TestCase):

    def setUp(self):
        self.node = mn.McuNode()
        self.addCleanup(self.node.destroy_node)
        self.node._write = lambda data: None

    def _capture(self, attr):
        out = []
        setattr(self.node, attr, types.SimpleNamespace(publish=out.append))
        return out

    @staticmethod
    def _strip_stamp(msg):
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        return msg

    def test_dev_status_fast_equals_typed(self):
        from rclpy.serialization import deserialize_message
        node = self.node
        node._on_battery(struct.pack(mn.BATTERY_FMT, 248, 5, 99, 1, 15, 0))
        node._on_motors(b''.join(struct.pack(mn.MOTOR_FMT, 900, 0, 2478, 14, 0)
                                 for _ in range(4)))
        node._on_speed_data(mn.speed_payload(0.4, 0.0))
        node._charging_enabled = True
        for flags in ((1, 0, 0, 1, 0, 1, 1, 1, 0, 0), (0, 1, 1, 0, 0, 0, 0, 0, 1, 1),
                      (0,) * 10):
            node._sensor_info = dict(zip(('bumper', 'rain', 'lift', 'stop', 'power_off',
                                          'battery_gate', 'cutter_size', 'press_module',
                                          'bumper_r', 'bumper_l'), flags))
            for latched in (False, True):
                node._estop_latched = latched
                out = self._capture('_status_pub')
                node._publish_dev_status()
                ser, node._status_ser = node._status_ser, None
                try:
                    node._publish_dev_status()
                finally:
                    node._status_ser = ser
                self.assertIsInstance(out[0], bytes)
                fast = deserialize_message(out[0], mn.MowerBaseDevStatus)
                self.assertNotEqual(fast.header.stamp.sec, 0)
                self.assertEqual(self._strip_stamp(fast), self._strip_stamp(out[1]))

    def test_odom_fast_equals_typed(self):
        from rclpy.serialization import deserialize_message
        node = self.node
        node._on_speed_data(mn.speed_payload(0.3, -0.2))
        node._x, node._y, node._yaw = 1.25, -3.5, 0.7
        out = self._capture('_odom_pub')
        node._last_reckon = mn.time.monotonic() - 0.02
        state = (node._x, node._y, node._yaw)
        node._reckon()
        node._x, node._y, node._yaw = state
        node._fast_odom = False
        node._last_reckon = mn.time.monotonic() - 0.02
        node._reckon()
        self.assertIsInstance(out[0], bytes)
        fast = self._strip_stamp(deserialize_message(out[0], mn.Odometry))
        typed = self._strip_stamp(out[1])
        # dt differs by a few microseconds between the two calls: compare with tolerance
        self.assertEqual((fast.header.frame_id, fast.child_frame_id),
                         (typed.header.frame_id, typed.child_frame_id))
        for a, b in ((fast.pose.pose.position.x, typed.pose.pose.position.x),
                     (fast.pose.pose.position.y, typed.pose.pose.position.y),
                     (fast.pose.pose.orientation.z, typed.pose.pose.orientation.z),
                     (fast.pose.pose.orientation.w, typed.pose.pose.orientation.w),
                     (fast.twist.twist.linear.x, typed.twist.twist.linear.x),
                     (fast.twist.twist.angular.z, typed.twist.twist.angular.z)):
            self.assertAlmostEqual(a, b, places=4)
        self.assertEqual(list(fast.pose.covariance), list(typed.pose.covariance))
        self.assertEqual(list(fast.twist.covariance), list(typed.twist.covariance))


if __name__ == '__main__':
    unittest.main(verbosity=2)
