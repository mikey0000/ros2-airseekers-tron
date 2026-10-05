"""``mower_cameras`` — OpenCV/V4L2 fallback camera driver for the Airseekers Tron.

**Canonical producers (do not double-open devices; a V4L2 node can be streamed by one
process only):**

==========================  ==========================================  ================
topic                       canonical producer                          this node
==========================  ==========================================  ================
/left_oa_camera/*           ``v4l2_camera`` in ``launch/cameras.launch.py``  never
/right_oa_camera/*          ``v4l2_camera`` in ``launch/cameras.launch.py``  never
/rear_camera/image_raw,     ``v4l2_camera`` (``rear_driver:=v4l2``,      ``enable_rear``
/rear_camera/camera_info    default) in ``launch/cameras.launch.py``     (off by default)
/vio/{left,right}/image_raw ``stereo_vio_bridge`` (``launch/vio.launch.py``) ``enable_stereo``
                                                                        (off by default)
==========================  ==========================================  ================

Why this node still exists:

* ``enable_rear`` (``cameras.launch.py rear_driver:=opencv``): the rear USB UVC webcam
  (32e6:9221) delivers 1920x1080@30 only as **MJPEG**; ``v4l2_camera`` 0.6 (Humble) cannot
  decode MJPEG (YUYV/UYVY/GREY only), so full-rate 1080p rear video needs this
  OpenCV path (``CAP_PROP_FOURCC=MJPG``). Publishes ``/rear_camera/image_raw`` (bgr8),
  ``/rear_camera/camera_info`` (from ``rear_camera_info_file``) and optionally
  ``/rear_camera/image_raw/compressed`` (JPEG).
* ``enable_stereo``: debug fallback for the Metoak side-by-side stereo when
  ``stereo_vio_bridge`` is not running. Publishes the same topics OpenVINS consumes
  (``/vio/left/image_raw``, ``/vio/right/image_raw``), so never run both.
"""
from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

try:
    import cv2
    import numpy as np  # noqa: F401
except ImportError:  # pragma: no cover - cv2/numpy live in the ROS 2 image
    cv2 = None

STEREO_FRAME_ID = 'vio_camera'
REAR_FRAME_ID = 'rear_camera'


def _to_image(stamp, frame_id, frame):
    """Convert an OpenCV frame to a ROS ``Image`` (bgr8, or mono8 if single-channel)."""
    if frame.ndim == 2 or frame.shape[2] == 1:
        encoding = 'mono8'
        frame = frame.reshape(frame.shape[0], frame.shape[1], 1)
    else:
        encoding = 'bgr8'
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.encoding = encoding
    msg.step = int(frame.strides[0])
    msg.data = frame.tobytes()
    return msg


def _to_compressed(stamp, frame_id, frame, quality):
    """Encode a BGR frame as a JPEG ``CompressedImage``."""
    ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        return None
    msg = CompressedImage()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.format = 'jpeg'
    msg.data = buf.tobytes()
    return msg


def split_side_by_side(frame):
    """Split a side-by-side stereo frame into ``(left, right)`` halves."""
    half = frame.shape[1] // 2
    return frame[:, :half], frame[:, half:2 * half]


def load_camera_info(path, frame_id=''):
    """Parse a ``camera_info_manager`` YAML into a ``CameraInfo`` (``None`` if no path)."""
    if not path:
        return None
    import yaml
    with open(path) as fh:
        data = yaml.safe_load(fh)
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = int(data['image_width'])
    msg.height = int(data['image_height'])
    msg.distortion_model = data.get('distortion_model', 'plumb_bob')
    msg.d = [float(v) for v in data['distortion_coefficients']['data']]
    msg.k = [float(v) for v in data['camera_matrix']['data']]
    msg.r = [float(v) for v in data['rectification_matrix']['data']]
    msg.p = [float(v) for v in data['projection_matrix']['data']]
    return msg


class CameraNode(Node):
    def __init__(self):
        super().__init__('mower_cameras')

        # ---- what to run (both off: the canonical producers live elsewhere) ----
        self.declare_parameter('enable_stereo', False)
        self.declare_parameter('enable_rear', False)
        # ---- devices ----
        # /dev/video11 = /dev/videoSimor raw side-by-side (same default as
        # stereo_vio_bridge v4l2 mode). /dev/video22 (/dev/videoIsp) is the ISP node the
        # Metoak SDK opens (stereo_vio_bridge source_mode:=sdk) -- not usable here.
        self.declare_parameter('stereo_device', '/dev/video11')
        self.declare_parameter('left_device', '')
        self.declare_parameter('right_device', '')
        self.declare_parameter('rear_device', '/dev/rear_camera')
        self.declare_parameter('rear_camera_info_file', '')
        # ---- geometry / rate ----
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('rear_width', 1920)
        self.declare_parameter('rear_height', 1080)
        self.declare_parameter('rear_fps', 30.0)
        self.declare_parameter('rear_fourcc', 'MJPG')
        # ---- options ----
        self.declare_parameter('publish_compressed', False)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('mono', False)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.enable_stereo = bool(p('enable_stereo'))
        self.enable_rear = bool(p('enable_rear'))
        self.publish_compressed = bool(p('publish_compressed'))
        self.jpeg_quality = int(p('jpeg_quality'))
        self.mono = bool(p('mono'))

        qos = rclpy.qos.qos_profile_sensor_data
        self.cap_left = self.cap_right = self.cap_rear = None
        self.split_stereo = False
        self.left_comp_pub = self.right_comp_pub = self.rear_comp_pub = None

        if cv2 is None:
            self.get_logger().error('python3-opencv missing: no capture possible')

        if self.enable_stereo:
            self.left_pub = self.create_publisher(Image, '/vio/left/image_raw', qos)
            self.right_pub = self.create_publisher(Image, '/vio/right/image_raw', qos)
            if self.publish_compressed:
                self.left_comp_pub = self.create_publisher(
                    CompressedImage, '/vio/left/image_raw/compressed', qos)
                self.right_comp_pub = self.create_publisher(
                    CompressedImage, '/vio/right/image_raw/compressed', qos)
            if cv2 is not None:
                self._open_stereo()
            self.create_timer(1.0 / float(p('fps')), self._on_stereo_tick)

        if self.enable_rear:
            self.rear_pub = self.create_publisher(Image, '/rear_camera/image_raw', qos)
            self.rear_info_pub = self.create_publisher(
                CameraInfo, '/rear_camera/camera_info', qos)
            if self.publish_compressed:
                self.rear_comp_pub = self.create_publisher(
                    CompressedImage, '/rear_camera/image_raw/compressed', qos)
            try:
                self.rear_info = load_camera_info(p('rear_camera_info_file'), REAR_FRAME_ID)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f'rear camera_info not loaded: {exc}')
                self.rear_info = None
            if cv2 is not None:
                self.cap_rear = self._open(p('rear_device'), p('rear_width'),
                                           p('rear_height'), p('rear_fourcc'), p('rear_fps'))
            self.create_timer(1.0 / float(p('rear_fps')), self._on_rear_tick)

        if not (self.enable_stereo or self.enable_rear):
            self.get_logger().warning(
                'mower_cameras: enable_stereo and enable_rear are both false; idle. '
                'OA/rear cameras come from launch/cameras.launch.py, VIO stereo from '
                'stereo_vio_bridge.')
        else:
            self.get_logger().info(
                f'mower_cameras up (stereo={self.enable_stereo} split={self.split_stereo}, '
                f'rear={self.enable_rear}, compressed={self.publish_compressed})')

    # ------------------------------------------------------------------ capture

    def _open(self, device, width, height, fourcc='', fps=0.0):
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error(f'could not open {device}')
            return None
        if fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc[:4]))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps:
            cap.set(cv2.CAP_PROP_FPS, fps)
        if self.mono:
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        return cap

    def _open_stereo(self):
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        left_dev, right_dev = p('left_device'), p('right_device')
        w, h = p('width'), p('height')
        if left_dev and right_dev:
            self.cap_left = self._open(left_dev, w, h)
            self.cap_right = self._open(right_dev, w, h)
        else:
            self.cap_left = self._open(p('stereo_device'), w, h)
            self.split_stereo = self.cap_left is not None

    def _grab_stereo(self):
        if self.cap_left is None:
            return None
        ok, frame = self.cap_left.read()
        if not ok:
            return None
        if self.split_stereo:
            return split_side_by_side(frame)
        if self.cap_right is None:
            return None
        ok, right = self.cap_right.read()
        return (frame, right) if ok else None

    # ------------------------------------------------------------------ publish

    def _publish(self, stamp, frame_id, frame, pub, comp_pub):
        pub.publish(_to_image(stamp, frame_id, frame))
        if comp_pub is not None:
            comp = _to_compressed(stamp, frame_id, frame, self.jpeg_quality)
            if comp is not None:
                comp_pub.publish(comp)

    def _on_stereo_tick(self):
        pair = self._grab_stereo()
        if pair is None:
            return
        stamp = self.get_clock().now().to_msg()
        self._publish(stamp, STEREO_FRAME_ID, pair[0], self.left_pub, self.left_comp_pub)
        self._publish(stamp, STEREO_FRAME_ID, pair[1], self.right_pub, self.right_comp_pub)

    def _on_rear_tick(self):
        if self.cap_rear is None:
            return
        ok, frame = self.cap_rear.read()
        if not ok:
            return
        stamp = self.get_clock().now().to_msg()
        self._publish(stamp, REAR_FRAME_ID, frame, self.rear_pub, self.rear_comp_pub)
        if self.rear_info is not None:
            self.rear_info.header.stamp = stamp
            self.rear_info_pub.publish(self.rear_info)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
