"""``seg_ros`` — PP-LiteSeg terrain segmentation node for the Airseekers Tron.

Subscribes to the left/right OA camera streams and runs the shipped 6-class
`pplite-seg_20260630-1-6cls.rknn` on the RK3588S NPU (pinned to core 1).

Publishes per-camera:
- a class-index mask (``sensor_msgs/Image``, mono8, indices 0..5),
- a traversability mask (mono8, 255 navigable / 0 obstacle) for the planner,
- a colour debug overlay (bgr8).

Startup mirrors ``det_ros`` (``mower_rknn.startup``): ``model_path`` defaults to
``/userdata/ros2/models/pplite-seg_20260630-1-6cls.rknn``, ``models_dir`` is the
fallback directory, and ``dry_run:=true`` keeps the node alive without NPU/model.
"""

from __future__ import annotations

import sys
import time
from typing import List

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import cv2
    from cv_bridge import CvBridge
    _DEP_ERROR = None
except ImportError as _exc:  # pragma: no cover - depends on the image
    _DEP_ERROR = _exc

from mower_rknn import NpuStartupError, bgr_to_rgb_nhwc, prepare_runner
from seg_ros.ppseg_postprocess import (argmax_mask, colorize, resize_mask,
                                       traversability_mask)


class SegRosNode(Node):
    def __init__(self):
        super().__init__('seg_ros')

        # ---- parameters ----
        self.declare_parameter('model_path',
                               '/userdata/ros2/models/pplite-seg_20260630-1-6cls.rknn')
        self.declare_parameter('models_dir', '')
        self.declare_parameter('dry_run', False)
        self.declare_parameter('max_rate_hz', 5.0)  # per camera; 0 = every frame
        self.declare_parameter('img_size', [640, 480])  # [width, height]
        self.declare_parameter('core_mask', '1')
        self.declare_parameter('left_topic', '/left_oa_camera/image_raw')
        self.declare_parameter('right_topic', '/right_oa_camera/image_raw')
        self.declare_parameter('publish_overlay', True)

        self.model_path = self.get_parameter('model_path').value
        self.img_size = tuple(int(v) for v in self.get_parameter('img_size').value)
        self.core_mask = str(self.get_parameter('core_mask').value)
        self.max_rate_hz = float(self.get_parameter('max_rate_hz').value)
        self._last = {}

        self.runner = prepare_runner(
            self.get_logger(), self.model_path, self.get_parameter('models_dir').value,
            self.core_mask, bool(self.get_parameter('dry_run').value))
        if self.runner is None:
            return  # dry_run

        self.bridge = CvBridge()
        qos = rclpy.qos.QoSProfile(depth=10)
        self.mask_pub = self.create_publisher(Image, '/ai/seg/mask', qos)
        self.trav_pub = self.create_publisher(Image, '/ai/seg/traversability', qos)
        self.overlay_pub = self.create_publisher(Image, '/ai/seg/image_overlay', qos)
        self.subs: List = []
        for topic in (self.get_parameter('left_topic').value,
                      self.get_parameter('right_topic').value):
            if topic:
                self.subs.append(self.create_subscription(
                    Image, topic, self._make_cb(topic), rclpy.qos.qos_profile_sensor_data))

        self.get_logger().info(
            f'seg_ros up: core_mask={self.core_mask}, '
            f'input {self.img_size[0]}x{self.img_size[1]}, max {self.max_rate_hz} Hz/camera')

    def _make_cb(self, topic):
        def cb(msg: Image):
            if self.max_rate_hz > 0.0:
                now = time.monotonic()
                if now - self._last.get(topic, 0.0) < 1.0 / self.max_rate_hz:
                    return
                self._last[topic] = now
            frame = msg.header.frame_id or topic.strip('/').split('/')[0]
            try:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f'decode failed on {topic}: {exc}')
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
    if _DEP_ERROR is not None:
        print(f'[seg_ros] FATAL: missing dependency ({_DEP_ERROR}); install '
              'ros-humble-cv-bridge / python3-opencv (docker/Dockerfile.humble).',
              file=sys.stderr)
        sys.exit(2)
    rclpy.init(args=args)
    try:
        node = SegRosNode()
    except NpuStartupError as exc:
        rclpy.logging.get_logger('seg_ros').fatal(str(exc))
        rclpy.try_shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.runner is not None:
            node.runner.release()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
