"""``det_ros`` — YOLOv8 obstacle detection node for the Airseekers Tron.

Subscribes to the left/right OA camera streams and runs the shipped YOLOv8 ``.rknn``
model (``best_large_0208.rknn`` = 22 classes, ``best_small_0208.rknn`` = 2 classes:
``stone``, ``leaf``) on the RK3588S NPU.

Publishes per-camera detections as a standard ``vision_msgs/Detection2DArray`` plus a
debug annotated ``bgr8`` image. See README for the OA-obstacle contract.
"""

from __future__ import annotations

import os
from typing import Dict, List

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (Detection2D, Detection2DArray, ObjectHypothesisWithPose)
from cv_bridge import CvBridge

from mower_rknn import RknnRunner, RknnUnavailable, letterbox
from det_ros.yolo_postprocess import post_process


class DetRosNode(Node):
    def __init__(self):
        super().__init__('det_ros')

        # ---- parameters ----
        self.declare_parameter('model_path', 'model/best_large_0208.rknn')
        self.declare_parameter('classes', [])  # empty -> loaded with model_path+'.list'
        self.declare_parameter('img_size', [480, 640])  # [width, height]
        self.declare_parameter('obj_thresh', 0.25)
        self.declare_parameter('nms_thresh', 0.45)
        self.declare_parameter('core_mask', '0')
        self.declare_parameter('left_topic', '/left_oa_camera/image_raw')
        self.declare_parameter('right_topic', '/right_oa_camera/image_raw')
        self.declare_parameter('publish_annotated', True)

        self.model_path = self.get_parameter('model_path').value
        self.img_size = tuple(int(v) for v in self.get_parameter('img_size').value)
        self.obj_thresh = float(self.get_parameter('obj_thresh').value)
        self.nms_thresh = float(self.get_parameter('nms_thresh').value)
        self.core_mask = self.get_parameter('core_mask').value
        self.classes = self._load_classes()

        # ---- model ----
        try:
            self.runner = RknnRunner(self.model_path, core_mask=self.core_mask)
        except RknnUnavailable as exc:
            self.get_logger().fatal(str(exc))
            raise

        # ---- io ----
        self.bridge = CvBridge()
        qos = rclpy.qos.QoSProfile(depth=10)
        self.det_pub = self.create_publisher(Detection2DArray, '/ai/det/detections', qos)
        self.ann_pub = self.create_publisher(Image, '/ai/det/image_annotated', qos)
        self.subs: List = []
        for topic in (self.get_parameter('left_topic').value,
                      self.get_parameter('right_topic').value):
            self.subs.append(self.create_subscription(
                Image, topic, self._make_cb(topic), qos))

        self.get_logger().info(
            'det_ros up: model=%s, %d classes, core_mask=%s, %dx%d',
            self.model_path, len(self.classes), self.core_mask, *self.img_size)

    # ------------------------------------------------------------------ classes

    def _load_classes(self) -> List[str]:
        declared = self.get_parameter('classes').value
        if declared:
            return [str(c) for c in declared]
        list_file = self.model_path + '.list'
        if os.path.isfile(list_file):
            with open(list_file) as fh:
                return [ln.strip() for ln in fh if ln.strip()]
        self.get_logger().warn('no class list; publishing class indices only')
        return []

    # ------------------------------------------------------------------ callback

    def _make_cb(self, topic):
        def cb(msg: Image):
            # cache the frame_id for the detection header
            frame = msg.header.frame_id or topic.strip('/').replace('/', '_')
            try:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn('decode failed on %s: %s', topic, exc)
                return
            self._detect(bgr, frame, msg.header.stamp)
        return cb

    def _detect(self, bgr: np.ndarray, frame_id: str, stamp) -> None:
        padded, scale, (pad_x, pad_y) = letterbox(bgr, self.img_size)
        tensor = padded[None, ...]  # NHWC uint8 RGB expected by the int8 model
        outputs = self.runner.run([tensor])

        boxes, classes, scores = post_process(
            outputs, img_size=self.img_size,
            obj_thresh=self.obj_thresh, nms_thresh=self.nms_thresh)

        detections = Detection2DArray()
        detections.header.stamp = stamp
        detections.header.frame_id = frame_id

        if boxes is not None:
            inv_scale = 1.0 / scale if scale else 1.0
            for box, cls, score in zip(boxes, classes, scores):
                x1 = (box[0] - pad_x) * inv_scale
                y1 = (box[1] - pad_y) * inv_scale
                x2 = (box[2] - pad_x) * inv_scale
                y2 = (box[3] - pad_y) * inv_scale
                h, w = bgr.shape[:2]
                x1, y1, x2, y2 = (max(0.0, min(float(w), x1)),
                                  max(0.0, min(float(h), y1)),
                                  max(0.0, min(float(w), x2)),
                                  max(0.0, min(float(h), y2)))
                d = Detection2D()
                d.bbox.center.position.x = (x1 + x2) / 2.0
                d.bbox.center.position.y = (y1 + y2) / 2.0
                d.bbox.size_x = float(x2 - x1)
                d.bbox.size_y = float(y2 - y1)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(int(cls))
                label = self.classes[int(cls)] if int(cls) < len(self.classes) else str(int(cls))
                hyp.hypothesis.score = float(score)
                hyp.id = label
                d.results.append(hyp)
                detections.detections.append(d)

        self.det_pub.publish(detections)

        if self.get_parameter('publish_annotated').value:
            vis = self._draw(bgr, boxes, classes, scores, scale, pad_x, pad_y)
            self.ann_pub.publish(self.bridge.cv2_to_imgmsg(vis, encoding='bgr8'))

    def _draw(self, bgr, boxes, classes, scores, scale, pad_x, pad_y):
        vis = bgr.copy()
        if boxes is None:
            return vis
        inv_scale = 1.0 / scale if scale else 1.0
        for box, cls, score in zip(boxes, classes, scores):
            x1 = int((box[0] - pad_x) * inv_scale)
            y1 = int((box[1] - pad_y) * inv_scale)
            x2 = int((box[2] - pad_x) * inv_scale)
            y2 = int((box[3] - pad_y) * inv_scale)
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = self.classes[int(cls)] if int(cls) < len(self.classes) else str(int(cls))
            cv2.putText(vis, f'{label} {score:.2f}', (x1, max(12, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        return vis


def main(args=None):
    rclpy.init(args=args)
    node = DetRosNode()
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
