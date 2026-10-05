"""``mower_cameras`` — minimal V4L2 camera driver for the Airseekers Tron mower.

Opens the front Metoak stereo module and the rear USB UVC camera via V4L2
(``cv2.VideoCapture``) and publishes::

    /vio/left            sensor_msgs/Image   (left eye of the stereo pair)
    /vio/right           sensor_msgs/Image   (right eye of the stereo pair)
    /rear_camera         sensor_msgs/Image   (rear USB UVC camera)

The Metoak stereo module (``/dev/videoSimor`` -> ``video11``) delivers a single
side-by-side frame (e.g. 3240x810 or 1920x360); the node splits it in half to
get the left/right eyes. Alternatively ``left_device`` / ``right_device`` can
point at two independent V4L2 devices.

Encodings: ``bgr8`` by default, ``mono8`` when the ``mono`` parameter is set
(or the captured frame is single-channel). When ``publish_compressed`` is true
an additional ``sensor_msgs/CompressedImage`` (JPEG) is published on
``<topic>/compressed`` for lightweight viewing (Foxglove / web_video_server).

Device map (see ros2_port_handoff/README.md hardware table):
  /dev/videoSimor (video11)   Metoak front stereo, side-by-side frames
  /dev/rear_camera (video62)  rear USB UVC webcam (32e6:9221)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - cv2/numpy live in the ROS 2 image
    cv2 = np = None

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
    ok, buf = cv2.imencode('.jpg', frame,
                           [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        return None
    msg = CompressedImage()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.format = 'jpeg'
    msg.data = buf.tobytes()
    return msg


class CameraNode(Node):
    def __init__(self):
        super().__init__('mower_cameras')

        # ---- devices ----
        self.declare_parameter('stereo_device', '/dev/video11')
        self.declare_parameter('left_device', '')
        self.declare_parameter('right_device', '')
        self.declare_parameter('rear_device', '/dev/rear_camera')
        # ---- geometry / rate ----
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('rear_width', 1920)
        self.declare_parameter('rear_height', 1080)
        self.declare_parameter('rear_fps', 30.0)
        # ---- options ----
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('mono', False)

        self.publish_compressed = self.get_parameter('publish_compressed').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.mono = self.get_parameter('mono').value

        # ---- publishers ----
        qos = rclpy.qos.QoSProfile(depth=10)
        self.left_pub = self.create_publisher(Image, '/vio/left', qos)
        self.right_pub = self.create_publisher(Image, '/vio/right', qos)
        self.rear_pub = self.create_publisher(Image, '/rear_camera', qos)
        if self.publish_compressed:
            self.left_comp_pub = self.create_publisher(
                CompressedImage, '/vio/left/compressed', qos)
            self.right_comp_pub = self.create_publisher(
                CompressedImage, '/vio/right/compressed', qos)
            self.rear_comp_pub = self.create_publisher(
                CompressedImage, '/rear_camera/compressed', qos)

        # ---- sources ----
        self.cap_left = self.cap_right = self.cap_rear = None
        self.split_stereo = False
        if cv2 is not None:
            self._open_cameras()

        fps = self.get_parameter('fps').value
        rear_fps = self.get_parameter('rear_fps').value
        self.stereo_timer = self.create_timer(1.0 / fps, self._on_stereo_tick)
        self.rear_timer = self.create_timer(1.0 / rear_fps, self._on_rear_tick)
        self.get_logger().info(
            'mower_cameras up (split_stereo=%s, compressed=%s, stereo %g Hz, rear %g Hz)',
            self.split_stereo, self.publish_compressed, fps, rear_fps)

    # ------------------------------------------------------------------ capture

    def _open(self, device, width, height):
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error('could not open %s', device)
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if self.mono:
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        return cap

    def _open_cameras(self):
        left_dev = self.get_parameter('left_device').value
        right_dev = self.get_parameter('right_device').value
        w = self.get_parameter('width').value
        h = self.get_parameter('height').value
        if left_dev and right_dev:
            self.cap_left = self._open(left_dev, w, h)
            self.cap_right = self._open(right_dev, w, h)
        else:
            # Side-by-side stereo: one device, split in half.
            self.cap_left = self._open(
                self.get_parameter('stereo_device').value, w, h)
            self.cap_right = None
            self.split_stereo = self.cap_left is not None
        self.cap_rear = self._open(
            self.get_parameter('rear_device').value,
            self.get_parameter('rear_width').value,
            self.get_parameter('rear_height').value)

    def _grab_stereo(self):
        """Return ``(left, right)`` BGR frames, or ``None``."""
        if self.cap_left is None:
            return None
        ok, frame = self.cap_left.read()
        if not ok:
            return None
        if self.split_stereo:
            half = frame.shape[1] // 2
            return frame[:, :half], frame[:, half:]
        ok, right = self.cap_right.read()
        if not ok:
            return None
        return frame, right

    def _grab_rear(self):
        if self.cap_rear is None:
            return None
        ok, frame = self.cap_rear.read()
        return frame if ok else None

    # ------------------------------------------------------------------ publish

    def _publish(self, stamp, frame_id, frame, pub, comp_pub):
        pub.publish(_to_image(stamp, frame_id, frame))
        if self.publish_compressed and comp_pub is not None:
            comp = _to_compressed(stamp, frame_id, frame, self.jpeg_quality)
            if comp is not None:
                comp_pub.publish(comp)

    def _on_stereo_tick(self):
        pair = self._grab_stereo()
        if pair is None:
            return
        stamp = self.get_clock().now().to_msg()
        left, right = pair
        self._publish(stamp, STEREO_FRAME_ID, left,
                      self.left_pub, self.left_comp_pub)
        self._publish(stamp, STEREO_FRAME_ID, right,
                      self.right_pub, self.right_comp_pub)

    def _on_rear_tick(self):
        frame = self._grab_rear()
        if frame is None:
            return
        stamp = self.get_clock().now().to_msg()
        self._publish(stamp, REAR_FRAME_ID, frame,
                      self.rear_pub, self.rear_comp_pub)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
