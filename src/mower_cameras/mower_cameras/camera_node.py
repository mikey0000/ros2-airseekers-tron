"""``mower_cameras`` — OpenCV/V4L2 fallback camera driver for the Airseekers Tron.

**Canonical producers (do not double-open devices; a V4L2 node can be streamed by one
process only):**

==========================  ==========================================  ================
topic                       canonical producer                          this node
==========================  ==========================================  ================
/left_oa_camera/*           ``v4l2_cam`` (``mower_cameras.v4l2_node``)    never
/right_oa_camera/*          ``v4l2_cam`` (``mower_cameras.v4l2_node``)    never
/rear_camera/image_raw,     this node (``cameras.launch.py``             ``enable_rear``
/rear_camera/camera_info    ``rear_driver:=opencv``, the default)        (off by default)
/vio/{left,right}/image_raw ``stereo_vio_bridge`` (``launch/vio.launch.py``) ``enable_stereo``
                                                                        (off by default)
==========================  ==========================================  ================

Why this node still exists:

* ``enable_rear`` (``cameras.launch.py rear_driver:=opencv``): the rear USB UVC webcam
  (32e6:9221) delivers 1920x1080@30 only as **MJPEG**; ``v4l2_camera`` 0.6 (Humble) cannot
  decode MJPEG (YUYV/UYVY/GREY only), so full-rate rear video needs this node. Capture
  runs in a thread (``CaptureLoop``); ``rear_backend`` ``v4l2`` (default: pure-Python mmap
  reader, JPEG passed through to ``compressed``) or ``opencv``. Publishes ``/rear_camera/image_raw`` (bgr8),
  ``/rear_camera/camera_info`` (from ``rear_camera_info_file``) and optionally
  ``/rear_camera/image_raw/compressed`` (JPEG).
* ``enable_stereo``: debug fallback for the Metoak side-by-side stereo when
  ``stereo_vio_bridge`` is not running. Publishes the same topics OpenVINS consumes
  (``/vio/left/image_raw``, ``/vio/right/image_raw``), so never run both.
"""
from __future__ import annotations

import array

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - cv2/numpy live in the ROS 2 image
    cv2 = np = None

from mower_cameras.image_cdr import ImageCdr
from mower_cameras.v4l2_node import CaptureLoop, to_compressed_msg

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
    frame = np.ascontiguousarray(frame)
    msg.step = int(frame.strides[0])
    # array.array, not bytes: the Humble rclpy setter range-checks every element of a
    # bytes/list value in Python (0.998 s per 1080p bgr8 frame measured on the mower ->
    # the rear topic ran at ~1 Hz); array.array('B') is taken as-is (0.005 s).
    data = array.array('B')
    data.frombytes(frame.data)
    msg.data = data
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
    data = array.array('B')
    data.frombytes(buf.tobytes())
    msg.data = data
    return msg


def split_side_by_side(frame):
    """Split a side-by-side stereo frame into ``(left, right)`` halves."""
    half = frame.shape[1] // 2
    return frame[:, :half], frame[:, half:2 * half]


def scale_camera_info(info, width, height):
    """``info`` rescaled to ``width``x``height`` (same object if the size already matches,
    ``None`` if the aspect ratio differs, i.e. it is not a full-FOV scale)."""
    if info is None or (info.width, info.height) == (width, height):
        return info
    sx, sy = width / info.width, height / info.height
    if abs(sx - sy) > 1e-3:
        return None
    scaled = CameraInfo()
    scaled.header.frame_id = info.header.frame_id
    scaled.width, scaled.height = width, height
    scaled.distortion_model, scaled.d, scaled.r = info.distortion_model, info.d, info.r
    k, pm = list(info.k), list(info.p)
    for i in (0, 1, 2):
        k[i] *= sx
        pm[i] *= sx
    for i in (3, 4, 5):
        k[i] *= sy
        pm[4 + i - 3] *= sy
    pm[3] *= sx
    pm[7] *= sy
    scaled.k, scaled.p = k, pm
    return scaled


def mode_info_path(path, width, height):
    """Per-capture-mode calibration next to ``path``: ``<stem>_<w>x<h>.yaml``.

    UVC modes with another aspect ratio than the calibrated one (rear webcam: 640x480 vs
    the 1920x1080 calibration) are a crop + scale of the sensor that cannot be derived
    from the calibration alone; such a mode gets its own file (``None`` if no path).
    """
    if not path:
        return None
    import os
    stem, ext = os.path.splitext(path)
    return f'{stem}_{int(width)}x{int(height)}{ext or ".yaml"}'


def camera_info_for_mode(info, path, width, height, frame_id=''):
    """``CameraInfo`` for a ``width``x``height`` capture: the mode-specific file
    (:func:`mode_info_path`) if it exists and matches, else ``info`` rescaled
    (:func:`scale_camera_info`; ``None`` when the aspect ratio differs)."""
    import os
    mode_path = mode_info_path(path, width, height)
    if mode_path and os.path.isfile(mode_path):
        mode = load_camera_info(mode_path, frame_id)
        if (mode.width, mode.height) == (width, height):
            return mode
    return scale_camera_info(info, width, height)


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
        # Metoak SDK opens; it streams YUYV 1280x480 side-by-side grey and is what
        # mower_cameras/stereo_cam (the default front-stereo producer) reads.
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
        # v4l2: pure-Python mmap reader (mower_cameras.v4l2), MJPEG passed through to
        # image_raw/compressed for free; opencv: cv2.VideoCapture(CAP_V4L2).
        self.declare_parameter('rear_backend', 'v4l2')
        # ---- options ----
        self.declare_parameter('publish_compressed', False)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('mono', False)
        # rear: decode/publish only while image_raw / compressed / camera_info has
        # subscribers (web_video_server, foxglove subscribe on demand)
        self.declare_parameter('publish_on_demand', True)
        # on_demand: close the rear device after this long without subscribers (USB
        # stream off), reopen on the next subscriber; 0 = keep streaming
        self.declare_parameter('close_when_unused_s', 5.0)
        self.declare_parameter('fast_publish', True)     # pre-serialized Image (bytes)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.enable_stereo = bool(p('enable_stereo'))
        self.enable_rear = bool(p('enable_rear'))
        self.publish_compressed = bool(p('publish_compressed'))
        self.jpeg_quality = int(p('jpeg_quality'))
        self.mono = bool(p('mono'))
        self.on_demand = bool(p('publish_on_demand'))
        self.fast_publish = bool(p('fast_publish'))
        self.rear_opencv = p('rear_backend') == 'opencv'
        self._rear_img = None

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
            self.rear_loop = None
            if cv2 is not None:
                # Dedicated capture thread: a blocking read can never stall the executor.
                self.rear_loop = CaptureLoop(
                    self.get_logger(), p('rear_device'), p('rear_width'), p('rear_height'),
                    p('rear_fourcc'), p('rear_fps'), self._on_rear_frame,
                    backend=p('rear_backend'), name='cap:rear',
                    want=self._rear_wanted if self.on_demand else None,
                    close_when_unused_s=p('close_when_unused_s'))
                self.rear_loop.start()
                self.create_timer(10.0, self._rear_watchdog)
                self._rear_seen = 0

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

    def _scaled_rear_info(self, width, height):
        """Rear camera_info, rescaled if the capture mode has the calibration's aspect."""
        info = self.rear_info
        scaled = camera_info_for_mode(info, self.get_parameter('rear_camera_info_file').value,
                                      width, height, REAR_FRAME_ID)
        if scaled is None:
            self.get_logger().warning(
                f'rear camera_info: no calibration for {width}x{height} (aspect differs from '
                f'{info.width}x{info.height} and no '
                f'{mode_info_path(self.get_parameter("rear_camera_info_file").value, width, height)}'
                '); not publishing camera_info (re-checked every 10 s)',
                throttle_duration_sec=60.0)
        elif scaled is not info:
            self.get_logger().info(
                f'rear camera_info {width}x{height}: fx={scaled.k[0]:.1f} fy={scaled.k[4]:.1f} '
                f'cx={scaled.k[2]:.1f} cy={scaled.k[5]:.1f} (mode file, or a full-FOV rescale '
                f'of {info.width}x{info.height})')
        return scaled

    @staticmethod
    def _subscribed(pub):
        return pub is not None and pub.get_subscription_count() > 0

    def _rear_wanted(self):
        return (self._subscribed(self.rear_pub) or self._subscribed(self.rear_comp_pub)
                or (self.rear_info is not None and self._subscribed(self.rear_info_pub)))

    def _rear_raw(self, now, data, cap):
        """Decode + publish /rear_camera/image_raw; returns the BGR frame."""
        if self.rear_opencv:
            frame = data
        elif self.fast_publish and cap.pixel_format in ('MJPG', 'JPEG', 'UYVY', 'YUYV'):
            img = self._rear_img
            if img is None or (img.width, img.height) != (cap.width, cap.height):
                img = self._rear_img = ImageCdr(REAR_FRAME_ID, cap.height, cap.width)
            from mower_cameras.v4l2 import to_bgr
            frame = to_bgr(data, cap.width, cap.height, cap.bytesperline, cap.pixel_format,
                           dst=img.image)
            if frame is None:
                raise ValueError('undecodable rear frame (corrupt JPEG?)')
            if frame.shape == img.image.shape:
                if frame.ctypes.data != img.image.ctypes.data:
                    img.image[...] = frame
                try:
                    self.rear_pub.publish(img.serialize(now.nanoseconds))
                    return frame
                except TypeError:       # rclpy without publish(bytes)
                    self.fast_publish = False
        else:
            from mower_cameras.v4l2 import to_bgr
            frame = to_bgr(data, cap.width, cap.height, cap.bytesperline, cap.pixel_format)
            if frame is None:
                raise ValueError('undecodable rear frame (corrupt JPEG?)')
        self.rear_pub.publish(_to_image(now.to_msg(), REAR_FRAME_ID, frame))
        return frame

    def _on_rear_frame(self, data, cap):
        """Capture-thread callback: ``data`` is JPEG bytes / a buffer view (v4l2) or a BGR
        image (opencv)."""
        now = self.get_clock().now()
        stamp = now.to_msg()
        od = self.on_demand
        want_raw = not od or self._subscribed(self.rear_pub)
        want_comp = self.rear_comp_pub is not None and (
            not od or self._subscribed(self.rear_comp_pub))
        jpeg = None
        if not self.rear_opencv and cap.pixel_format == 'MJPG':
            jpeg = data
        frame = self._rear_raw(now, data, cap) if want_raw else None
        if want_comp:
            if jpeg is None and frame is None:
                frame = self._rear_raw_frame_only(data, cap)
            comp = (to_compressed_msg(stamp, REAR_FRAME_ID, jpeg) if jpeg is not None
                    else _to_compressed(stamp, REAR_FRAME_ID, frame, self.jpeg_quality))
            if comp is not None:
                self.rear_comp_pub.publish(comp)
        if self.rear_info is None or not (not od or want_raw or want_comp
                                          or self._subscribed(self.rear_info_pub)):
            return
        if frame is not None:
            h, w = frame.shape[:2]
        else:
            w, h = cap.width, cap.height     # v4l2 backend: negotiated size
        t = now.nanoseconds * 1e-9
        if (not hasattr(self, '_rear_info_wh') or self._rear_info_wh != (w, h)
                or (self._rear_info_out is None and t - self._rear_info_t > 10.0)):
            self._rear_info_wh = (w, h)
            self._rear_info_t = t
            self._rear_info_out = self._scaled_rear_info(w, h)
        if self._rear_info_out is not None:
            self._rear_info_out.header.stamp = stamp
            self.rear_info_pub.publish(self._rear_info_out)

    def _rear_raw_frame_only(self, data, cap):
        if self.rear_opencv:
            return data
        from mower_cameras.v4l2 import to_bgr
        frame = to_bgr(data, cap.width, cap.height, cap.bytesperline, cap.pixel_format)
        if frame is None:
            raise ValueError('undecodable rear frame (corrupt JPEG?)')
        return frame

    def _rear_watchdog(self):
        n = self.rear_loop.captured
        if n == self._rear_seen and not self.rear_loop.paused:   # paused = nobody subscribes
            self.get_logger().warning(
                'rear camera: no frame captured in the last 10 s (%s %dx%d %s); see the '
                'capture thread log above' % (self.get_parameter('rear_device').value,
                                              self.get_parameter('rear_width').value,
                                              self.get_parameter('rear_height').value,
                                              self.get_parameter('rear_fourcc').value))
        self._rear_seen = n

    def destroy_node(self):
        if getattr(self, 'rear_loop', None) is not None:
            self.rear_loop.stop()
            self.rear_loop.join(timeout=3.0)
        super().destroy_node()


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
