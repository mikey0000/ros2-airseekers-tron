"""``seg_ros`` — PP-LiteSeg terrain segmentation node for the Airseekers Tron.

Subscribes to the left/right OA camera streams and runs the shipped 6-class
`pplite-seg_20260630-1-6cls.rknn` on the RK3588S NPU (pinned to core 1).

Publishes per-camera:
- a class-index mask (``sensor_msgs/Image``, mono8, indices 0..5),
- a traversability mask (mono8, 255 navigable / 0 obstacle) for the planner,
- a colour debug overlay (bgr8).
"""

from __future__ import annotations

from typing import List

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from mower_rknn import RknnRunner, RknnUnavailable, bgr_to_rgb_nhwc
from seg_ros.ppseg_postprocess import (argmax_mask, colorize, resize_mask,
                                       traversability_mask)


class SegRosNode(Node):
    def __init__(self):
        super().__init__('seg_ros')

        # ---- parameters ----
        self.declare_parameter('model_path', 'model/pplite-seg_20260630-1-6cls.rknn')
        self.declare_parameter('img_size', [640, 480])  # [width, height]
        self.declare_parameter('core_mask', '1')
        self.declare_parameter('left_topic', '/left_oa_camera/image_raw')
        self.declare_parameter('right_topic', '/right_oa_camera/image_raw')
        self.declare_parameter('publish_overlay', True)

        self.model_path = self.get_parameter('model_path').value
        self.img_size = tuple(int(v) for v in self.get_parameter('img_size').value)
        self.core_mask = self.get_parameter('core_mask').value

        try:
            self.runner = RknnRunner(self.model_path, core_mask=self.core_mask)
        except RknnUnavailable as exc:
            self.get_logger().fatal(str(exc))
            raise

        self.bridge = CvBridge()
        qos = rclpy.qos.QoSProfile(depth=10)
        self.mask_pub = self.create_publisher(Image, '/ai/seg/mask', qos)
        self.trav_pub = self.create_publisher(Image, '/ai/seg/traversability', qos)
        self.overlay_pub = self.create_publisher(Image, '/ai/seg/image_overlay', qos)
        self.subs: List = []
        for topic in (self.get_parameter('left_topic').value,
                      self.get_parameter('right_topic').value):
            self.subs.append(self.create_subscription(
                Image, topic, self._make_cb(topic), qos))

        self.get_logger().info(
            'seg_ros up: model=%s, core_mask=%s, %dx%d',
            self.model_path, self.core_mask, *self.img_size)

    def _make_cb(self, topic):
        def cb(msg: Image):
            frame = msg.header.frame_id or topic.strip('/').replace('/', '_')
            try:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn('decode failed on %s: %s', topic, exc)
                return
            self._segment(bgr, frame, msg.header.stamp)
        return cb

    def _segment(self, bgr: np.ndarray, frame_id: str, stamp) -> None:
        # PP-LiteSeg expects a direct resize (stretch) to the model input, not a
        # letterbox (the mask is mapped back by scaling, not aspect-preserving).
        resize_w, resize_h = self.img_size
        resized = cv2.resize(bgr, (resize_w, resize_h), interpolation=cv2.INTER_LINEAR)
        tensor = bgr_to_rgb_nhwc(resized)[None, ...]  # NHWC uint8 RGB

        outputs = self.runner.run([tensor])
        logits = outputs[0]  # [1, 6, H, W] or [6, H, W]
        class_mask = argmax_mask(logits)          # [H, W] uint8
        trav = traversability_mask(class_mask)     # [H, W] uint8

        # full-resolution masks (nearest-neighbour up) for the planner
        h, w = bgr.shape[:2]
        full_class = resize_mask(class_mask, (w, h))
        full_trav = resize_mask(trav, (w, h))

        def imgmsg(arr, encoding):
            m = self.bridge.cv2_to_imgmsg(arr, encoding=encoding)
            m.header.stamp = stamp
            m.header.frame_id = frame_id
            return m

        self.mask_pub.publish(imgmsg(full_class, 'mono8'))
        self.trav_pub.publish(imgmsg(full_trav, 'mono8'))

        if self.get_parameter('publish_overlay').value:
            overlay = colorize(full_class)
            self.overlay_pub.publish(imgmsg(overlay, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = SegRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.runner.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
