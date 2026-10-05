"""``stereo_vio_bridge`` — feed the VSLAM/VIO estimator from the Metoak **stereo front**
camera + its embedded ICM-40608 IMU.

The Metoak module is a self-contained stereo pair (XC9080 sensors + on-board "Simor"
depth ASIC) plus a TDK ICM-40608 IMU (the chip at i2c-8/0x68, driven by the
``inv-icm42600`` kernel module — see ``09_platform/i2c8-devices.txt``).

The stereo front is delivered by the RK ISP as **one combined 1280x480 YUV422 stream**
(two 640x480 images side-by-side), *not* two independent V4L2 devices. The SDK's
``MoImage::SpliteImage`` splits it; we do the same here, then stereo-rectify with the
recovered calibration. Publishes the estimator inputs on the topics the OpenVINS config
already declares::

    /vio/imu                sensor_msgs/Imu
    /vio/left/image_raw     sensor_msgs/Image   (+ /vio/left/camera_info)
    /vio/right/image_raw    sensor_msgs/Image   (+ /vio/right/camera_info)

Capture is selected by ``stereo_layout``:

* ``combined`` (default) — one device (``stereo_device``) at ``combined_width`` x
  ``combined_height``, split left/right in software.
* ``separate`` — the old two-device mode (``left_device`` / ``right_device``).

Prerequisite (combined): ``mo_init.sh`` must have run first (``mo_xc9080.ko`` /
``mo_simor.ko`` / ``video_rkisp.ko`` loaded + the i2c init sequence), so the ISP exposes
the 1280x480 stream as a V4L2 node. ``source_mode: sdk`` (consume the SDK's already
rectified frames) is documented but not implemented.
"""
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import CameraInfo, Image, Imu

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - cv2/numpy live in the ROS 2 image
    cv2 = np = None

from stereo_vio_bridge.camera_info_builder import (
    camera_info_dict, distortion, intrinsics, load_camera, load_stereo,
)


def _to_img(time_stamp, frame_id, img):
    """Convert an OpenCV image to a ROS ``Image`` (mono8 or bgr8, auto-detected)."""
    msg = Image()
    msg.header.stamp = time_stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = img.shape[0], img.shape[1]
    msg.encoding = 'bgr8' if (img.ndim == 3 and img.shape[2] == 3) else 'mono8'
    msg.step = int(img.strides[0])
    msg.data = img.tobytes()
    return msg


def _camera_info(time_stamp, frame_id, d):
    """Build a ``CameraInfo`` from a ``camera_info_dict()`` result."""
    ci = CameraInfo()
    ci.header.stamp = time_stamp
    ci.header.frame_id = frame_id
    ci.height = d['height']
    ci.width = d['width']
    ci.distortion_model = d['distortion_model']
    ci.d = d['D']
    ci.k = d['K']
    ci.r = d['R']
    ci.p = d['P']
    return ci


class IioImu:
    """Read a TDK ICM-40608 (or any accel+gyro) IMU via the Linux IIO subsystem.

    The vendor ships ``inv-icm42600.ko`` + ``inv-icm42600-i2c.ko`` (the InvenSense driver
    also covers ICM-40608), so the IMU appears as an ``iio:deviceN`` with ``in_accel_*``
    and ``in_anglvel_*`` channels. raw*scale gives SI (m/s^2, rad/s).
    """

    def __init__(self, device=None):
        self.base = device or self._find()

    @staticmethod
    def _find():
        d = '/sys/bus/iio/devices'
        if not os.path.isdir(d):
            return None
        for name in sorted(os.listdir(d)):
            p = os.path.join(d, name)
            if os.path.isfile(os.path.join(p, 'in_accel_x_raw')) and \
               os.path.isfile(os.path.join(p, 'in_anglvel_x_raw')):
                return p
        return None

    @staticmethod
    def _read(path):
        try:
            with open(path) as fh:
                return fh.read().strip()
        except OSError:
            return None

    def _channel(self, channel, axis):
        if self.base is None:
            return None
        raw = self._read(os.path.join(self.base, f'{channel}_{axis}_raw'))
        if raw is None:
            return None
        scale = self._read(os.path.join(self.base, f'{channel}_scale')) or \
            self._read(os.path.join(self.base, f'{channel}_{axis}_scale'))
        s = float(scale) if scale is not None else 1.0
        return float(raw) * s

    def sample(self):
        """Return ``(ax, ay, az, gx, gy, gz)`` in SI (m/s^2, rad/s), or ``None``."""
        try:
            acc = (self._channel('in_accel', 'x'), self._channel('in_accel', 'y'),
                   self._channel('in_accel', 'z'))
            gyr = (self._channel('in_anglvel', 'x'), self._channel('in_anglvel', 'y'),
                   self._channel('in_anglvel', 'z'))
        except (TypeError, ValueError):
            return None
        vals = acc + gyr
        if any(v is None for v in vals):
            return None
        return vals


class StereoVioBridge(Node):
    def __init__(self):
        super().__init__('stereo_vio_bridge')

        self.declare_parameter('source_mode', 'v4l2')  # v4l2 | sdk
        self.declare_parameter('stereo_layout', 'combined')  # combined | separate
        self.declare_parameter('stereo_device', '/dev/video11')
        self.declare_parameter('left_device', '/dev/video0')
        self.declare_parameter('right_device', '/dev/video1')
        self.declare_parameter('combined_width', 1280)
        self.declare_parameter('combined_height', 480)
        self.declare_parameter('swap_lr', False)  # swap left/right halves if ISP order differs
        self.declare_parameter('publish_mono', True)  # mono8 (VIO wants intensity)
        self.declare_parameter('rectify', True)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('calib_dir', '/userdata/ros2/calibration')
        self.declare_parameter('imu_device', '')
        self.declare_parameter('camera_frame', 'vio_camera')

        self.source_mode = self.get_parameter('source_mode').value
        self.layout = self.get_parameter('stereo_layout').value
        self.rate = self.get_parameter('rate').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.rectify = self.get_parameter('rectify').value
        self.swap_lr = self.get_parameter('swap_lr').value
        self.publish_mono = self.get_parameter('publish_mono').value
        self.calib_dir = self.get_parameter('calib_dir').value

        # ---- publishers ----
        qos = rclpy.qos.QoSProfile(depth=10)
        self.left_pub = self.create_publisher(Image, '/vio/left/image_raw', qos)
        self.right_pub = self.create_publisher(Image, '/vio/right/image_raw', qos)
        self.left_ci_pub = self.create_publisher(CameraInfo, '/vio/left/camera_info', qos)
        self.right_ci_pub = self.create_publisher(CameraInfo, '/vio/right/camera_info', qos)
        self.imu_pub = self.create_publisher(Imu, '/vio/imu', qos)

        # ---- calibration ----
        self.calib = self._load_calibration()
        self._rect_maps = None
        self._rectified_info = None
        if self.rectify and cv2 is not None:
            self._build_rectification()

        # ---- sources ----
        self.capS = self.capL = self.capR = None
        if self.source_mode == 'v4l2' and cv2 is not None:
            self._open_cameras()
        self.imu_src = IioImu(self.get_parameter('imu_device').value or None)

        self.timer = self.create_timer(1.0 / self.rate, self._on_tick)
        self.get_logger().info(
            'stereo_vio_bridge up (%s mode, layout=%s, rectify=%s, mono=%s, %g Hz)',
            self.source_mode, self.layout, self.rectify, self.publish_mono, self.rate)

    # ------------------------------------------------------------------ calib

    def _load_calibration(self):
        try:
            cam0 = load_camera(os.path.join(self.calib_dir, 'cam0.yaml'))
            cam1 = load_camera(os.path.join(self.calib_dir, 'cam1.yaml'))
            r, t, base, bxf = load_stereo(os.path.join(self.calib_dir, 'stereo_params.yaml'))
            return {'cam0': cam0, 'cam1': cam1, 'R': r, 'T': t, 'base': base, 'bxf': bxf}
        except (OSError, KeyError, ValueError) as exc:
            self.get_logger().error('calibration load failed: %s', exc)
            return None

    def _build_rectification(self):
        c = self.calib
        if c is None:
            return
        cam0, cam1 = c['cam0'], c['cam1']
        size = (cam0['width'], cam0['height'])
        k0, d0 = intrinsics(cam0), distortion(cam0)
        k1, d1 = intrinsics(cam1), distortion(cam1)
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            k0, d0, k1, d1, size, c['R'], c['T'], alpha=0)
        self._rect_maps = (
            cv2.initUndistortRectifyMap(k0, d0, R1, P1, size, cv2.CV_16SC2),
            cv2.initUndistortRectifyMap(k1, d1, R2, P2, size, cv2.CV_16SC2),
        )
        self._rectified_info = (P1, P2)

    # ------------------------------------------------------------------ capture

    def _open_cameras(self):
        if self.layout == 'combined':
            dev = self.get_parameter('stereo_device').value
            w = self.get_parameter('combined_width').value
            h = self.get_parameter('combined_height').value
            cap = cv2.VideoCapture(dev)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            else:
                self.get_logger().error('could not open combined device %s', dev)
            self.capS = cap
        else:  # separate — two per-eye devices
            w = self.get_parameter('width').value
            h = self.get_parameter('height').value
            for attr, dev in (('capL', self.get_parameter('left_device').value),
                              ('capR', self.get_parameter('right_device').value)):
                cap = cv2.VideoCapture(dev)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                else:
                    self.get_logger().error('could not open %s', dev)
                setattr(self, attr, cap)

    @staticmethod
    def _as_gray(frame):
        """BGR -> mono8 if requested and the frame is colour."""
        if frame.ndim == 3 and frame.shape[2] == 3:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return frame

    def _grab_combined(self):
        if not (self.capS and self.capS.isOpened()):
            return None
        ok, frame = self.capS.read()
        if not ok:
            return None
        w = frame.shape[1]
        if w < 2:
            return None
        half = w // 2
        left, right = frame[:, :half], frame[:, half:]
        if self.swap_lr:
            left, right = right, left
        if self.publish_mono:
            left = self._as_gray(left)
            right = self._as_gray(right)
        return self._rectify_pair(left, right)

    def _grab_separate(self):
        if not (self.capL and self.capR and self.capL.isOpened() and self.capR.isOpened()):
            return None
        okL, frameL = self.capL.read()
        okR, frameR = self.capR.read()
        if not (okL and okR):
            return None
        if self.swap_lr:
            frameL, frameR = frameR, frameL
        if self.publish_mono:
            frameL = self._as_gray(frameL)
            frameR = self._as_gray(frameR)
        return self._rectify_pair(frameL, frameR)

    def _rectify_pair(self, left, right):
        """Software stereo-rectify if requested. Returns ``(left, right)``."""
        if self._rect_maps is not None:
            left = cv2.remap(left, *self._rect_maps[0], cv2.INTER_LINEAR)
            right = cv2.remap(right, *self._rect_maps[1], cv2.INTER_LINEAR)
        return left, right

    def _grab(self):
        """Pull one left/right frame pair (mono8 or bgr8), or ``None``."""
        if self.layout == 'combined':
            return self._grab_combined()
        return self._grab_separate()

    # ------------------------------------------------------------------ publish

    def _camera_info_msg(self, which, stamp):
        if self.calib is None:
            return None
        if self._rectified_info is not None:
            p = np.zeros((3, 4))
            p[0:3, 0:3] = self._rectified_info[0 if which == 'left' else 1][0:3, 0:3]
            d = {
                'height': self.calib['cam0']['height'],
                'width': self.calib['cam0']['width'],
                'distortion_model': 'plumb_bob',
                'D': [0.0] * 5,
                'K': p[0:3, 0:3].reshape(-1).tolist(),
                'R': [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0],
                'P': p.reshape(-1).tolist(),
            }
            return _camera_info(stamp, self.camera_frame, d)
        d = camera_info_dict(self.calib['cam0' if which == 'left' else 'cam1'])
        return _camera_info(stamp, self.camera_frame, d)

    def _publish_imu(self, stamp):
        sample = self.imu_src.sample()
        if sample is None:
            return
        ax, ay, az, gx, gy, gz = sample
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = 'imu_link'
        msg.linear_acceleration = Vector3(x=ax, y=ay, z=az)
        msg.angular_velocity = Vector3(x=gx, y=gy, z=gz)
        msg.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        msg.orientation_covariance[0] = -1.0  # orientation unavailable (unknown basis)
        self.imu_pub.publish(msg)

    def _on_tick(self):
        if cv2 is None:
            return
        stamp = self.get_clock().now().to_msg()
        pair = self._grab()
        if pair is not None:
            left, right = pair
            self.left_pub.publish(_to_img(stamp, self.camera_frame, left))
            self.right_pub.publish(_to_img(stamp, self.camera_frame, right))
            for which, pub in (('left', self.left_ci_pub), ('right', self.right_ci_pub)):
                ci = self._camera_info_msg(which, stamp)
                if ci is not None:
                    pub.publish(ci)
        self._publish_imu(stamp)


def main(args=None):
    rclpy.init(args=args)
    node = StereoVioBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
