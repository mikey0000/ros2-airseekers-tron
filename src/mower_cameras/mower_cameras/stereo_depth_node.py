"""``mower_cameras/stereo_depth`` — Metoak hardware disparity -> depth image + PointCloud2.

The Metoak module's Simor ASIC computes stereo disparity on the module and streams it on
``/dev/video11`` (``/dev/videoSimor``, rkcif-mipi-lvds2). Decoded on the mower 2026-10-06
(raw capture, no vendor SDK): the node is opened as RGB3 640x360 (bytesperline 1920),
i.e. 3 bytes per pixel:

* byte 0          disparity bits 0..7
* byte 1 & 0x0F   disparity bits 8..11   (high nibble: unrelated/flags, ignored)
* byte 2          8-bit grey image of the reference eye (rectified)

``disparity_px = raw12 / 32`` (5 fractional bits); 0 = invalid. Scale checked against the
ground plane: with the camera 0.25 m above ground, rows 240 / 340 gave raw 582 / 1317, i.e.
18.2 / 41.2 px -> 1.25 / 0.55 m, matching the geometric slant range (1.25 / 0.59 m).
``Z = bxf / disparity_px`` with ``bxf`` = baseline[mm] x focal[px] from the per-unit
calibration (ros2_port_handoff/08_calibration_identity/stereo_params.yaml: base 60.047 mm,
bxf 22698.4 -> f = 378 px). This video node is independent of ``/dev/video22`` used by
``stereo_cam``, so both nodes run side by side.

Publishes (frame ``stereo_camera_optical``: standard optical axes, z forward, x right,
y down; defined in config/urdf/mower.urdf.xacro):

* ``~/depth/image_raw``  32FC1 metres (0 = invalid), on demand
* ``~/image_mono``       mono8 reference-eye image, on demand
* ``~/points``           PointCloud2 xyz float32, decimated, range-limited (Nav2 obstacle
                         source; always published while anybody subscribes)
"""
from __future__ import annotations

import array

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField

from mower_cameras.v4l2_node import CaptureLoop, to_image_msg

FRAME_ID = 'stereo_camera_optical'


def decode_simor(data, width=640, height=360, bytesperline=1920):
    """Raw RGB3 Simor frame -> (disparity raw12 uint16 HxW, grey uint8 HxW)."""
    buf = np.frombuffer(data, dtype=np.uint8, count=bytesperline * height)
    px = buf.reshape(height, bytesperline)[:, :width * 3].reshape(height, width, 3)
    raw = px[:, :, 0].astype(np.uint16) | ((px[:, :, 1].astype(np.uint16) & 0x0F) << 8)
    return raw, px[:, :, 2]


def disparity_to_depth(raw, bxf_mm, scale=32.0, min_disp_px=1.0):
    """raw12 disparity -> depth in metres (float32, 0 where invalid)."""
    d = raw.astype(np.float32) / float(scale)
    z = np.zeros_like(d)
    ok = d >= min_disp_px
    z[ok] = (float(bxf_mm) / 1000.0) / d[ok]
    return z


def depth_to_points(z, fx, fy, cx, cy, step=4, min_z=0.2, max_z=4.0):
    """Decimated depth -> Nx3 float32 optical-frame points."""
    zs = z[::step, ::step]
    v, u = np.mgrid[0:z.shape[0]:step, 0:z.shape[1]:step]
    ok = (zs >= min_z) & (zs <= max_z)
    zz = zs[ok]
    x = (u[ok].astype(np.float32) - cx) * zz / fx
    y = (v[ok].astype(np.float32) - cy) * zz / fy
    return np.stack([x, y, zz], axis=1).astype(np.float32)


def points_msg(stamp, frame_id, pts):
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = int(pts.shape[0])
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate('xyz')]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = True
    data = array.array('B')
    data.frombytes(np.ascontiguousarray(pts).tobytes())
    msg.data = data
    return msg


def depth_image_msg(stamp, frame_id, z):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = int(z.shape[0]), int(z.shape[1])
    msg.encoding = '32FC1'
    msg.is_bigendian = 0
    msg.step = msg.width * 4
    data = array.array('B')
    data.frombytes(np.ascontiguousarray(z, dtype=np.float32).tobytes())
    msg.data = data
    return msg


class StereoDepthNode(Node):
    def __init__(self):
        super().__init__('stereo_depth')
        d = self.declare_parameter
        d('video_device', '/dev/video11')
        d('width', 640)
        d('height', 360)
        d('pixel_format', 'RGB3')
        d('fps', 10.0)                   # publish-rate cap (sensor runs ~25 Hz)
        d('frame_id', FRAME_ID)
        d('bxf_mm', 22698.4)             # baseline[mm] * focal[px], per-unit calibration
        d('disparity_scale', 32.0)       # raw12 / scale = disparity in px
        d('fx', 378.0)                   # = bxf / baseline (60.047 mm)
        d('fy', 378.0)
        d('cx', 319.5)
        d('cy', 179.5)
        d('cloud_step', 4)               # pixel decimation for the cloud (640x360 -> 160x90)
        d('min_range_m', 0.2)
        d('max_range_m', 4.0)
        d('publish_on_demand', True)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {n: p(n) for n in ('frame_id', 'bxf_mm', 'disparity_scale', 'fx', 'fy',
                                       'cx', 'cy', 'cloud_step', 'min_range_m', 'max_range_m')}
        self.on_demand = bool(p('publish_on_demand'))
        qos = rclpy.qos.qos_profile_sensor_data
        self.depth_pub = self.create_publisher(Image, '~/depth/image_raw', qos)
        self.mono_pub = self.create_publisher(Image, '~/image_mono', qos)
        self.cloud_pub = self.create_publisher(PointCloud2, '~/points', qos)
        self.loop = CaptureLoop(self.get_logger(), p('video_device'), p('width'), p('height'),
                                p('pixel_format'), p('fps'), self._on_frame, 'v4l2',
                                name='cap:simor',
                                want=self._wanted if self.on_demand else None)
        self.loop.start()
        self.get_logger().info(
            f'stereo_depth {p("video_device")} {p("width")}x{p("height")} fps<={p("fps")} '
            f'bxf={self.cfg["bxf_mm"]} scale={self.cfg["disparity_scale"]} '
            f'-> ~/points, ~/depth/image_raw ({self.cfg["frame_id"]})')

    def _wanted(self):
        return any(x.get_subscription_count() > 0
                   for x in (self.depth_pub, self.mono_pub, self.cloud_pub))

    def _on_frame(self, data, cap):
        c = self.cfg
        stamp = self.get_clock().now().to_msg()
        raw, grey = decode_simor(data, cap.width, cap.height, cap.bytesperline or cap.width * 3)
        z = disparity_to_depth(raw, c['bxf_mm'], c['disparity_scale'])
        if not self.on_demand or self.cloud_pub.get_subscription_count() > 0:
            pts = depth_to_points(z, c['fx'], c['fy'], c['cx'], c['cy'], int(c['cloud_step']),
                                  c['min_range_m'], c['max_range_m'])
            self.cloud_pub.publish(points_msg(stamp, c['frame_id'], pts))
        if not self.on_demand or self.depth_pub.get_subscription_count() > 0:
            self.depth_pub.publish(depth_image_msg(stamp, c['frame_id'], z))
        if not self.on_demand or self.mono_pub.get_subscription_count() > 0:
            self.mono_pub.publish(to_image_msg(stamp, c['frame_id'], grey))

    def destroy_node(self):
        self.loop.stop()
        self.loop.join(timeout=3.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
