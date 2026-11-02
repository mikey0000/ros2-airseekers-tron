# SPDX-License-Identifier: GPL-3.0-or-later
"""``camera_blocked`` - front camera blocked / dark / uniform monitor (stock parity).

Subscribes ``image_topic`` (front stereo right image, taken raw: only the sampled frames are deserialized) only
while the mission is MOWING
(``/behavior_tree_node/high_level_status``) and samples it at ``sample_hz``; the verdict is
:class:`mower_vision.camera_blocked.BlockedMonitor`. Publishes ``blocked_topic`` (latched
``std_msgs/Bool``: mower_mission caps the speed at ``camera_blocked_speed_mps`` while true)
and raises / clears the ``camera_blocked`` mower_alerts notice.
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from mowgli_interfaces.msg import HighLevelStatus

from mower_vision.camera_blocked import BlockedMonitor, image_stats, is_daylight

PARAMS = {
    'enabled': True,
    'image_topic': '/vio/right/image_raw',
    'high_level_status_topic': '/behavior_tree_node/high_level_status',
    'blocked_topic': '/vision/camera_blocked',
    'notice_topic': '/mower_alerts/notice',
    'active_states': ['MOWING'],
    'daylight_hours': [8.0, 18.0],    # local time [start, end); [] = always
    'sample_hz': 1.0,
    'decimate': 8,
    'camera_blocked_s': 10.0,
    'clear_s': 3.0,
    'std_max': 6.0,
    'mean_min': 12.0,
    'mean_max': 250.0,
}


class CameraBlockedNode(Node):
    def __init__(self):
        super().__init__('camera_blocked')
        for k, v in PARAMS.items():
            self.declare_parameter(k, v)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.mon = BlockedMonitor(g('camera_blocked_s'), g('clear_s'), g('std_max'),
                                  g('mean_min'), g('mean_max'))
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.blocked_pub = self.create_publisher(Bool, g('blocked_topic'), latched)
        self.notice_pub = self.create_publisher(String, g('notice_topic'), 10)
        self.blocked_pub.publish(Bool(data=False))
        self._state = ''
        self._img_sub = None
        self._last_sample = 0.0
        self.create_subscription(HighLevelStatus, g('high_level_status_topic'), self._on_hl, 10)
        self.create_timer(1.0, self._tick)
        self.get_logger().info('camera blocked monitor on %s: %.0f s, std < %.1f, mean '
                               'outside [%.0f, %.0f]' % (g('image_topic'), g('camera_blocked_s'),
                                                         g('std_max'), g('mean_min'),
                                                         g('mean_max')))

    def _active(self):
        return bool(self.get_parameter('enabled').value) and \
            self._state in list(self.get_parameter('active_states').value) and \
            is_daylight(list(self.get_parameter('daylight_hours').value))

    def _on_hl(self, msg):
        self._state = str(msg.state_name)

    def _tick(self):
        active = self._active()
        if active and self._img_sub is None:
            self._img_sub = self.create_subscription(
                Image, self.get_parameter('image_topic').value, self._on_image,
                qos_profile_sensor_data, raw=True)   # bytes: deserialize only sampled frames
        elif not active and self._img_sub is not None:
            self.destroy_subscription(self._img_sub)
            self._img_sub = None
        if not active:
            self._feed(None, False)

    def _on_image(self, raw):
        now = time.monotonic()
        if now - self._last_sample < 1.0 / max(0.1, float(self.get_parameter('sample_hz').value)):
            return
        self._last_sample = now
        msg = deserialize_message(raw, Image)
        stats = image_stats(bytes(msg.data), msg.width, msg.height, msg.step, msg.encoding,
                            int(self.get_parameter('decimate').value))
        self._feed(stats, self._active())

    def _feed(self, stats, active):
        if not self.mon.update(time.monotonic(), stats, active):
            return
        blocked = self.mon.blocked
        self.blocked_pub.publish(Bool(data=blocked))
        if blocked:
            self.get_logger().warn('front camera looks blocked: %s' % self.mon.why)
        else:
            self.get_logger().info('front camera image ok again')
        self.notice_pub.publish(String(data=json.dumps({
            'incident': 'camera_blocked', 'active': blocked, 'kind': 'blocked', 'priority': 3,
            'message': 'cameraBlocked',
            'text': 'Front camera blocked (%s): mowing slowly, clean the lens.' % self.mon.why})))


def main(args=None):
    rclpy.init(args=args)
    node = CameraBlockedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
