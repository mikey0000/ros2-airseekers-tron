"""``mower_cameras/stereo_cam`` — Metoak front stereo -> ``/vio/{left,right}/image_raw``.

The Metoak module delivers ONE side-by-side frame per exposure. Verified on the mower
(2026-10-06, ``v4l2-ctl --list-formats-ext``): both nodes are rkcif multi-planar,

* ``/dev/video22`` (``/dev/videoIsp``, rkcif-mipi-lvds3): YUYV 1280x480 = two 640x480
  colour eyes side by side (re-probed 2026-10-06: U/V carry real chroma) -> default;
* ``/dev/video11`` (``/dev/videoSimor``, rkcif-mipi-lvds2): max 1920x360, raw sensor data
  that does not decode as an image without the Metoak SDK.

The whole frame is decoded YUYV -> bgr8 once and split; each eye is published as
``bgr8`` 640x480. Capture runs in the shared
:class:`mower_cameras.v4l2_node.CaptureLoop` thread (reconnect, rate cap) and, with
``publish_on_demand`` (default), frames are only copied while ``/vio/left|right/image_raw``
or ``camera_info`` has a subscriber (web_video_server / the GUI Perception page / VIO).

Never run this together with ``stereo_vio_bridge`` (same device, same topics):
``launch/cameras.launch.py`` starts it only for ``stereo:=true vio:=false``.
"""
from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from mower_cameras.v4l2 import split_side_by_side_bgr
from mower_cameras.v4l2_node import CaptureLoop, to_image_msg

STEREO_FRAME_ID = 'vio_camera'


class StereoCamNode(Node):
    def __init__(self):
        super().__init__('stereo_cam')
        d = self.declare_parameter
        d('video_device', '/dev/video22')
        d('width', 1280)                # full side-by-side width (each eye = width/2)
        d('height', 480)
        d('pixel_format', 'YUYV')
        d('fps', 5.0)                   # publish-rate cap
        d('frame_id', STEREO_FRAME_ID)
        d('left_topic', '/vio/left/image_raw')
        d('right_topic', '/vio/right/image_raw')
        d('left_camera_info_file', '')
        d('right_camera_info_file', '')
        d('publish_on_demand', True)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.frame_id = p('frame_id')
        self.on_demand = bool(p('publish_on_demand'))
        qos = rclpy.qos.qos_profile_sensor_data
        lt, rt = p('left_topic'), p('right_topic')
        self.pubs = [self.create_publisher(Image, lt, qos),
                     self.create_publisher(Image, rt, qos)]
        self.info_pubs = [
            self.create_publisher(CameraInfo, lt.rsplit('/', 1)[0] + '/camera_info', qos),
            self.create_publisher(CameraInfo, rt.rsplit('/', 1)[0] + '/camera_info', qos)]
        self.infos = [self._load_info(p('left_camera_info_file')),
                      self._load_info(p('right_camera_info_file'))]
        self.loop = CaptureLoop(self.get_logger(), p('video_device'), p('width'), p('height'),
                                p('pixel_format'), p('fps'), self._on_frame, 'v4l2',
                                name='cap:stereo',
                                want=self._wanted if self.on_demand else None)
        self.loop.start()
        self.get_logger().info(
            f'stereo_cam {p("video_device")} {p("width")}x{p("height")} {p("pixel_format")} '
            f'fps<={p("fps")} -> {lt}, {rt} (bgr8) on_demand={self.on_demand}')

    def _load_info(self, path):
        path = str(path).replace('file://', '', 1)
        if not path:
            return None
        from mower_cameras.camera_node import load_camera_info
        try:
            return load_camera_info(path, self.frame_id)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'camera_info not loaded ({path}): {exc}')
            return None

    @staticmethod
    def _subscribed(pub):
        return pub.get_subscription_count() > 0

    def _wanted(self):
        return any(self._subscribed(x) for x in self.pubs) or any(
            info is not None and self._subscribed(pub)
            for info, pub in zip(self.infos, self.info_pubs))

    def _on_frame(self, data, cap):
        stamp = self.get_clock().now().to_msg()
        eyes = split_side_by_side_bgr(data, cap.width, cap.height, cap.bytesperline,
                                      cap.pixel_format)
        if eyes is None:
            return
        for eye, pub, info, info_pub in zip(eyes, self.pubs, self.infos, self.info_pubs):
            if not self.on_demand or self._subscribed(pub):
                pub.publish(to_image_msg(stamp, self.frame_id, eye))
            if info is not None:
                info.header.stamp = stamp
                info_pub.publish(info)

    def destroy_node(self):
        self.loop.stop()
        self.loop.join(timeout=3.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StereoCamNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
