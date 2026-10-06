"""``obstacle_guard`` — det_ros detections -> close-obstacle flag (+ optional stop).

Subscribes ``/ai/det/detections`` (``vision_msgs/Detection2DArray`` from ``det_ros``) and
applies the danger-zone rule in :mod:`mower_vision.guard_logic` to whitelisted classes.

Prefers ``/ai/det/detections_ranged`` (``det_range``: same detections, ranged ones carry
``results[0].pose.pose.position`` in base_link and ``pose.covariance[0] > 0``): a ranged
whitelisted detection is close when its planar distance from (``range_origin_x_m``, 0)
is <= ``obstacle_stop_range_m``; unranged ones keep the image-space rule. While ranged
messages arrive (within ``ranged_timeout_s``) the raw ``/ai/det/detections`` are ignored;
otherwise the guard falls back to them.

Publishes:
* ``/vision/obstacle_close``   ``std_msgs/Bool`` on every detection frame and on hold expiry;
* ``/vision/obstacle_markers``  ``visualization_msgs/ImageMarker`` (LINE_LIST, image pixels,
  frame = camera frame): red = close, yellow = whitelisted, grey = other, blue = danger
  line. Add it as an annotation on the camera image in a Foxglove Image panel.

When ``stop_on_close`` is true, on the rising edge of the close state:
* zero ``geometry_msgs/Twist`` on ``/cmd_vel_emergency`` at ``burst_rate_hz`` for
  ``burst_s`` (twist_mux emergency input, priority 100, timeout 0.2 s);
* one ``std_srvs/Trigger`` call to ``/cutter_off`` (``mower_mcu_driver``).
"""

from __future__ import annotations

import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, ColorRGBA
from std_srvs.srv import Trigger
from visualization_msgs.msg import ImageMarker

try:
    from vision_msgs.msg import Detection2DArray
    _DEP_ERROR = None
except ImportError as _exc:  # pragma: no cover
    _DEP_ERROR = _exc

from mower_vision.guard_logic import (DEFAULT_CLASSES, DEFAULT_WHITELIST, Box,
                                      GuardConfig, GuardState, box_outline, class_label,
                                      classify, danger_line, with_image_size)

RED = ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0)
YELLOW = ColorRGBA(r=1.0, g=0.85, b=0.0, a=1.0)
GREY = ColorRGBA(r=0.6, g=0.6, b=0.6, a=1.0)
BLUE = ColorRGBA(r=0.2, g=0.5, b=1.0, a=1.0)


def range_of(det, origin_x=0.0):
    """Planar range (m) of a det_range-tagged detection, or None when unranged."""
    if not det.results:
        return None
    pose = det.results[0].pose
    if len(pose.covariance) < 1 or not pose.covariance[0] > 0.0:
        return None
    p = pose.pose.position
    return math.hypot(float(p.x) - origin_x, float(p.y))


def boxes_from_msg(msg, classes, origin_x=0.0):
    """``Detection2DArray`` -> ``[Box]`` (best hypothesis per detection)."""
    out = []
    for det in msg.detections:
        if not det.results:
            continue
        best = max(det.results, key=lambda r: r.hypothesis.score)
        out.append(Box(label=class_label(best.hypothesis.class_id, classes),
                       score=float(best.hypothesis.score),
                       cx=float(det.bbox.center.position.x),
                       cy=float(det.bbox.center.position.y),
                       w=float(det.bbox.size_x), h=float(det.bbox.size_y),
                       range_m=range_of(det, origin_x)))
    return out


class ObstacleGuard(Node):
    def __init__(self):
        super().__init__('obstacle_guard')
        dp = self.declare_parameter
        dp('detections_topic', '/ai/det/detections')
        dp('ranged_topic', '/ai/det/detections_ranged')   # '' = image-space rule only
        dp('ranged_timeout_s', 1.0)
        dp('obstacle_stop_range_m', 1.0)
        dp('range_origin_x_m', 0.466)   # stereo camera x in base_link: range from the lens
        dp('classes', list(DEFAULT_CLASSES))
        dp('whitelist', list(DEFAULT_WHITELIST))
        dp('y_frac', 0.6)
        dp('w_frac', 0.15)
        dp('min_score', 0.4)
        # Fallback only: the real size per frame_id comes from camera_info.
        dp('image_width', 960)
        dp('image_height', 540)
        dp('camera_info_topics', ['/left_oa_camera/camera_info',
                                  '/right_oa_camera/camera_info'])
        dp('hold_s', 0.5)
        dp('stop_on_close', False)
        dp('burst_s', 1.0)
        dp('burst_rate_hz', 20.0)
        dp('emergency_topic', '/cmd_vel_emergency')
        dp('cutter_off_service', '/cutter_off')
        dp('publish_markers', True)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.classes = [str(c) for c in p('classes')]
        self.cfg = GuardConfig(whitelist=[str(w) for w in p('whitelist')],
                               y_frac=float(p('y_frac')), w_frac=float(p('w_frac')),
                               min_score=float(p('min_score')),
                               image_width=int(p('image_width')),
                               image_height=int(p('image_height')),
                               stop_range_m=float(p('obstacle_stop_range_m')))
        self.range_origin_x = float(p('range_origin_x_m'))
        self.ranged_timeout = float(p('ranged_timeout_s'))
        self._last_ranged = None
        self.state = GuardState(hold_s=float(p('hold_s')), burst_s=float(p('burst_s')))
        self.stop_on_close = bool(p('stop_on_close'))
        self.publish_markers = bool(p('publish_markers'))

        self.close_pub = self.create_publisher(Bool, '/vision/obstacle_close', 10)
        self.marker_pub = self.create_publisher(ImageMarker, '/vision/obstacle_markers', 10)
        self.twist_pub = self.create_publisher(Twist, p('emergency_topic'), 10)
        self.cutter_cli = self.create_client(Trigger, p('cutter_off_service'))
        self._sizes = {}  # frame_id -> (w, h) from camera_info
        for topic in p('camera_info_topics'):
            self.create_subscription(CameraInfo, str(topic), self._on_info,
                                     qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, p('detections_topic'), self._on_raw, 10)
        if p('ranged_topic'):
            self.create_subscription(Detection2DArray, p('ranged_topic'), self._on_ranged, 10)
        self.create_timer(1.0 / float(p('burst_rate_hz')), self._on_tick)
        self._last_published = None

        self.get_logger().info(
            f'obstacle_guard up: whitelist={list(self.cfg.whitelist)} y_frac={self.cfg.y_frac} '
            f'w_frac={self.cfg.w_frac} fallback_image={self.cfg.image_width}x{self.cfg.image_height} '
            f'stop_on_close={self.stop_on_close} ranged={p("ranged_topic") or "off"} '
            f'stop_range={self.cfg.stop_range_m} m')

    def _on_info(self, msg):
        w, h = int(msg.width), int(msg.height)
        if w <= 0 or h <= 0:
            return
        key = msg.header.frame_id
        if self._sizes.get(key) != (w, h):
            self._sizes[key] = (w, h)
            self.get_logger().info(f'image size for frame "{key}": {w}x{h} (camera_info)')

    def _cfg_for(self, frame_id):
        size = self._sizes.get(frame_id)
        return with_image_size(self.cfg, *size) if size else self.cfg

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _publish_close(self, close: bool):
        self.close_pub.publish(Bool(data=bool(close)))
        self._last_published = close

    def _on_raw(self, msg):
        if (self._last_ranged is not None
                and self._now() - self._last_ranged <= self.ranged_timeout):
            return          # det_range is up: its republished copy is used instead
        self._on_dets(msg)

    def _on_ranged(self, msg):
        self._last_ranged = self._now()
        self._on_dets(msg)

    def _on_dets(self, msg):
        boxes = boxes_from_msg(msg, self.classes, self.range_origin_x)
        cfg = self._cfg_for(msg.header.frame_id)
        verdicts = classify(boxes, cfg)
        close_now = any(c for _, _, c in verdicts)
        close, rising = self.state.update(msg.header.frame_id or 'camera', close_now,
                                          self._now())
        self._publish_close(close)
        if rising:
            self._on_rising(verdicts)
        if self.publish_markers:
            self.marker_pub.publish(self._markers(msg.header, verdicts, cfg))

    def _on_tick(self):
        now = self._now()
        close, rising = self.state.tick(now)
        if close != self._last_published and self._last_published is not None:
            self._publish_close(close)
        if rising:
            self._on_rising([])
        if self.stop_on_close and self.state.burst_active(now):
            self.twist_pub.publish(Twist())

    def _on_rising(self, verdicts):
        what = ', '.join(f'{b.label}({b.score:.2f})' for b, _, c in verdicts if c) or 'held'
        if not self.stop_on_close:
            self.get_logger().warning(f'obstacle close: {what} (stop_on_close=false)')
            return
        self.get_logger().warning(
            f'obstacle close: {what} -> {self.state.burst_s:.1f}s zero burst + cutter off')
        self.twist_pub.publish(Twist())
        if self.cutter_cli.service_is_ready():
            self.cutter_cli.call_async(Trigger.Request())
        else:
            self.get_logger().error('cutter_off service not available')

    def _markers(self, header, verdicts, cfg):
        m = ImageMarker()
        m.header = header
        m.ns = 'obstacle_guard'
        m.id = 0
        m.type = ImageMarker.LINE_LIST
        m.action = ImageMarker.ADD
        m.scale = 3.0
        m.outline_color = BLUE
        pts, cols = [], []
        for x, y in danger_line(cfg):
            pts.append(Point(x=x, y=y))
            cols.append(BLUE)
        for box, relevant, close in verdicts:
            colour = RED if close else (YELLOW if relevant else GREY)
            for x, y in box_outline(box):
                pts.append(Point(x=float(x), y=float(y)))
                cols.append(colour)
        m.points = pts
        m.outline_colors = cols
        m.lifetime.nanosec = 500_000_000
        return m


def main(args=None):
    if _DEP_ERROR is not None:
        print(f'[obstacle_guard] FATAL: {_DEP_ERROR}; install ros-humble-vision-msgs.',
              file=sys.stderr)
        sys.exit(2)
    rclpy.init(args=args)
    node = ObstacleGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
