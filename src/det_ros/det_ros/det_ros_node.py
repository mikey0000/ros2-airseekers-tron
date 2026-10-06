"""``det_ros`` — YOLOv8 obstacle detection node for the Airseekers Tron.

Subscribes to the left/right OA camera streams (``/left_oa_camera/image_raw``,
``/right_oa_camera/image_raw`` from ``launch/cameras.launch.py``) and runs the shipped
YOLOv8 ``.rknn`` model (``best_large_0208.rknn`` = 22 classes) on the RK3588S NPU.

Publishes per-camera detections as ``vision_msgs/Detection2DArray`` on
``/ai/det/detections`` (header.frame_id = source camera frame; each result's
``hypothesis.class_id`` is the class *name*, ``score`` the confidence; bbox in source
image pixels) plus annotated ``bgr8`` images (boxes + labels + scores drawn, same size as
the input): one per camera on ``/<camera_ns>/image_annotated`` (e.g.
``/left_oa_camera/image_annotated``, what the GUI perception page streams) and the merged
debug topic ``/ai/det/image_annotated``. Annotation is on demand: a frame is drawn and
encoded only when one of those topics has a subscriber (same pattern as the camera nodes).

Startup (see ``mower_rknn.startup``): ``model_path`` defaults to the device path
``/userdata/ros2/models/best_large_0208.rknn``; when it is missing the basename is tried
in ``models_dir``. Without ``rknnlite`` / the model the node exits with one clear error,
or with ``dry_run:=true`` stays alive publishing nothing.
"""

from __future__ import annotations

import sys
import time
from typing import List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import cv2  # noqa: F401  (needed by letterbox/_draw)
    from cv_bridge import CvBridge
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
    _DEP_ERROR = None
except ImportError as _exc:  # pragma: no cover - depends on the image
    _DEP_ERROR = _exc

from mower_rknn import NpuStartupError, bgr_to_rgb_nhwc, letterbox, prepare_runner
from det_ros.labels import annotated_topic_for, default_classes, label_for
from det_ros.yolo_postprocess import post_process

DEFAULT_MODEL = '/userdata/ros2/models/best_large_0208.rknn'



class DetRosNode(Node):
    def __init__(self):
        super().__init__('det_ros')

        # ---- parameters ----
        self.declare_parameter('model_path', DEFAULT_MODEL)
        self.declare_parameter('models_dir', '')
        self.declare_parameter('dry_run', False)
        # empty -> model_path + '.list' file, else built-in list for best_large_*
        self.declare_parameter('classes', [''])
        self.declare_parameter('img_size', [480, 640])  # [width, height]
        self.declare_parameter('obj_thresh', 0.25)
        self.declare_parameter('nms_thresh', 0.45)
        self.declare_parameter('core_mask', '0')
        self.declare_parameter('left_topic', '/left_oa_camera/image_raw')
        self.declare_parameter('right_topic', '/right_oa_camera/image_raw')
        # Further cameras, same model / NPU core / per-camera rate cap ('' entries ignored).
        self.declare_parameter('extra_topics', [''])
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('max_rate_hz', 5.0)  # per camera; 0 = every frame

        self.model_path = self.get_parameter('model_path').value
        self.models_dir = self.get_parameter('models_dir').value
        self.img_size = tuple(int(v) for v in self.get_parameter('img_size').value)
        self.obj_thresh = float(self.get_parameter('obj_thresh').value)
        self.nms_thresh = float(self.get_parameter('nms_thresh').value)
        self.core_mask = str(self.get_parameter('core_mask').value)
        self.max_rate_hz = float(self.get_parameter('max_rate_hz').value)
        self.classes = self._load_classes()

        # ---- model (raises NpuStartupError unless dry_run) ----
        self.runner = prepare_runner(
            self.get_logger(), self.model_path, self.models_dir, self.core_mask,
            bool(self.get_parameter('dry_run').value))
        if self.runner is None:
            return  # dry_run: alive, no subscriptions, no publications

        # ---- io ----
        self.bridge = CvBridge()
        qos = rclpy.qos.QoSProfile(depth=10)
        sensor_qos = rclpy.qos.qos_profile_sensor_data
        self.det_pub = self.create_publisher(Detection2DArray, '/ai/det/detections', qos)
        self.ann_pub = self.create_publisher(Image, '/ai/det/image_annotated', qos)
        self._last = {}
        self.subs: List = []
        self.cam_ann_pubs = {}
        topics = [self.get_parameter('left_topic').value,
                  self.get_parameter('right_topic').value]
        topics += [str(t) for t in self.get_parameter('extra_topics').value]
        for topic in topics:
            if topic:
                self.cam_ann_pubs[topic] = self.create_publisher(
                    Image, annotated_topic_for(topic), sensor_qos)
                self.subs.append(self.create_subscription(
                    Image, topic, self._make_cb(topic), sensor_qos))

        self.get_logger().info(
            f'det_ros up: {len(self.classes)} classes, core_mask={self.core_mask}, '
            f'input {self.img_size[0]}x{self.img_size[1]}, max {self.max_rate_hz} Hz/camera')

    # ------------------------------------------------------------------ classes

    def _load_classes(self) -> List[str]:
        import os
        declared = [str(c) for c in self.get_parameter('classes').value if str(c)]
        if declared:
            return declared
        list_file = self.model_path + '.list'
        if os.path.isfile(list_file):
            with open(list_file) as fh:
                return [ln.strip() for ln in fh if ln.strip()]
        builtin = default_classes(self.model_path)
        if not builtin:
            self.get_logger().warning('no class list; publishing class indices only')
        return builtin

    # ------------------------------------------------------------------ callback

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
            self._detect(bgr, frame, msg.header.stamp, self.cam_ann_pubs.get(topic))
        return cb

    def _to_source(self, box, scale, pad_x, pad_y, w, h):
        inv = 1.0 / scale if scale else 1.0
        x1 = min(max((box[0] - pad_x) * inv, 0.0), float(w))
        y1 = min(max((box[1] - pad_y) * inv, 0.0), float(h))
        x2 = min(max((box[2] - pad_x) * inv, 0.0), float(w))
        y2 = min(max((box[3] - pad_y) * inv, 0.0), float(h))
        return float(x1), float(y1), float(x2), float(y2)

    def _detect(self, bgr, frame_id: str, stamp, cam_ann_pub=None) -> None:
        padded, scale, (pad_x, pad_y) = letterbox(bgr, self.img_size)
        # letterbox keeps BGR; the int8 model was calibrated on RGB NHWC
        tensor = bgr_to_rgb_nhwc(padded)[None, ...]
        outputs = self.runner.run([tensor])

        boxes, classes, scores = post_process(
            outputs, img_size=self.img_size,
            obj_thresh=self.obj_thresh, nms_thresh=self.nms_thresh)

        detections = Detection2DArray()
        detections.header.stamp = stamp
        detections.header.frame_id = frame_id
        h, w = bgr.shape[:2]
        src_boxes = []
        if boxes is not None:
            for box, cls, score in zip(boxes, classes, scores):
                x1, y1, x2, y2 = self._to_source(box, scale, pad_x, pad_y, w, h)
                src_boxes.append((x1, y1, x2, y2, int(cls), float(score)))
                d = Detection2D()
                d.header = detections.header
                d.bbox.center.position.x = (x1 + x2) / 2.0
                d.bbox.center.position.y = (y1 + y2) / 2.0
                d.bbox.size_x = x2 - x1
                d.bbox.size_y = y2 - y1
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = label_for(int(cls), self.classes)
                hyp.hypothesis.score = float(score)
                d.results.append(hyp)
                detections.detections.append(d)

        self.det_pub.publish(detections)

        if not self.get_parameter('publish_annotated').value:
            return
        # On demand: draw + encode only for topics somebody listens to.
        targets = [p for p in (cam_ann_pub, self.ann_pub)
                   if p is not None and p.get_subscription_count() > 0]
        if not targets:
            return
        out = self.bridge.cv2_to_imgmsg(self._draw(bgr, src_boxes), encoding='bgr8')
        out.header = detections.header
        for pub in targets:
            pub.publish(out)

    def _draw(self, bgr, src_boxes):
        import cv2 as _cv2
        vis = bgr.copy()
        for x1, y1, x2, y2, cls, score in src_boxes:
            _cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            _cv2.putText(vis, f'{label_for(cls, self.classes)} {score:.2f}',
                         (int(x1), max(12, int(y1) - 6)),
                         _cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        return vis


def main(args=None):
    if _DEP_ERROR is not None:
        print(f'[det_ros] FATAL: missing ROS dependency ({_DEP_ERROR}); install '
              'ros-humble-vision-msgs and ros-humble-cv-bridge (docker/Dockerfile.humble).',
              file=sys.stderr)
        sys.exit(2)
    rclpy.init(args=args)
    try:
        node = DetRosNode()
    except NpuStartupError as exc:
        rclpy.logging.get_logger('det_ros').fatal(str(exc))
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
