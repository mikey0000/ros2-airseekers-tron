"""``fake_camera`` — synthetic camera for dev hosts (no V4L2 device needed).

Publishes ``<ns>/image_raw`` (bgr8, moving gradient + box) and ``<ns>/camera_info`` at
``rate`` Hz. Run it in a camera namespace to stand in for a v4l2_camera node, e.g.::

    ros2 run mower_vision fake_camera --ros-args -r __ns:=/rear_camera -p width:=640 -p height:=360

Useful to exercise web_video_server / the RTSP relay / det_ros(dry_run) wiring off-robot.
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def synth_frame(width: int, height: int, t: int) -> np.ndarray:
    x = np.linspace(0, 255, width, dtype=np.uint8)
    frame = np.empty((height, width, 3), dtype=np.uint8)
    frame[:, :, 0] = x[None, :]
    frame[:, :, 1] = np.uint8((t * 4) % 256)
    frame[:, :, 2] = np.linspace(0, 255, height, dtype=np.uint8)[:, None]
    bw, bh = max(width // 8, 1), max(height // 6, 1)
    x0 = (t * 8) % max(width - bw, 1)
    y0 = height // 2
    frame[y0:y0 + bh, x0:x0 + bw] = (255, 255, 255)
    return frame


class FakeCamera(Node):
    def __init__(self):
        super().__init__('fake_camera')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 360)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('frame_id', 'fake_camera')
        self.w = int(self.get_parameter('width').value)
        self.h = int(self.get_parameter('height').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.img_pub = self.create_publisher(Image, 'image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)
        self.t = 0
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)

    def _tick(self):
        frame = synth_frame(self.w, self.h, self.t)
        self.t += 1
        stamp = self.get_clock().now().to_msg()
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = self.frame_id
        img.height, img.width = self.h, self.w
        img.encoding = 'bgr8'
        img.step = self.w * 3
        img.data = frame.tobytes()
        self.img_pub.publish(img)
        info = CameraInfo()
        info.header = img.header
        info.width, info.height = self.w, self.h
        self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = FakeCamera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
