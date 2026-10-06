# SPDX-License-Identifier: GPL-3.0-or-later
"""``det_range`` — range det_ros detections with the front stereo hardware depth.

Inputs (all served by a :class:`SubscriptionPump`, no rclpy executor wakes):

* ``/ai/det/detections``           vision_msgs/Detection2DArray (det_ros), event
* ``/stereo_depth/depth/image_raw`` 32FC1 metres, reference eye (cam0, rectified 640x360,
  frame ``stereo_camera_optical``); kept as raw CDR bytes in a short ring, only the frame
  closest in time to a detection is deserialized
* ``/vio/right/camera_info``       intrinsics/distortion of the source eye (cam1)
* ``/tf_static``                   for ``stereo_camera_optical`` -> ``base_link``

Only detections whose frame is in ``stereo_frames`` (default ``vio_camera``, the right colour
eye) are ranged (see :mod:`det_range.geometry`). The OA cameras (``left_oa_camera``,
``right_oa_camera``) have no extrinsics in the URDF, so their detections pass through
without range.

Outputs:

* ``/ai/det/detections_ranged``  every input detection, bbox unchanged; ranged ones get
  ``results[0].pose.pose.position`` = box centre in ``base_link`` (m) and
  ``results[0].pose.covariance[0]`` = range variance (m^2, > 0). Unranged: covariance 0.
* ``/ai/det/obstacles``          geometry_msgs/PoseArray of the ranged centroids (base_link)
* ``/ai/det/obstacle_points``    PointCloud2 (xyz float32, base_link) of the same points:
  Nav2 local costmap obstacle source.
"""
from __future__ import annotations

import collections
import sys
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_msgs.msg import TFMessage

try:
    from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
    _DEP_ERROR = None
except ImportError as _exc:  # pragma: no cover
    _DEP_ERROR = _exc

from det_range.geometry import (StereoModel, chain_to, quat_to_matrix, range_box)
from det_range.sub_pump import SubscriptionPump, _header


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def cloud_msg(header, pts):
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(pts)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate('xyz')]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = True
    msg.data = np.asarray(pts, dtype=np.float32).reshape(-1, 3).tobytes()
    return msg


class DetRange(Node):
    def __init__(self):
        super().__init__('det_range')
        d = self.declare_parameter
        d('detections_topic', '/ai/det/detections')
        d('depth_topic', '/stereo_depth/depth/image_raw')
        d('camera_info_topic', '/vio/right/camera_info')
        d('output_topic', '/ai/det/detections_ranged')
        d('obstacles_topic', '/ai/det/obstacles')
        d('obstacle_points_topic', '/ai/det/obstacle_points')
        d('stereo_frames', ['vio_camera'])
        d('target_frame', 'base_link')
        d('depth_frame', 'stereo_camera_optical')
        # Fallback pose of depth_frame in target_frame until /tf_static arrives (URDF value).
        d('fallback_xyz', [0.466, 0.0, 0.240])   # matches the URDF (measured 2026-10-06)
        d('fallback_rpy', [-1.5707963, 0.0, -1.5707963])
        # Reference-eye intrinsics of the depth image (stereo_depth node defaults).
        d('ref_fx', 378.0)
        d('ref_fy', 378.0)
        d('ref_cx', 319.5)
        d('ref_cy', 179.5)
        d('bxf_mm', 22698.404296875)
        d('box_frac', 0.5)            # central fraction of the box that is sampled
        d('min_valid_px', 20)
        d('min_range_m', 0.2)
        d('max_range_m', 10.0)
        d('z_guess_m', 2.0)
        d('sigma_disparity_px', 0.25)
        d('max_depth_age_s', 0.5)     # |det stamp - depth stamp| tolerance
        d('depth_ring', 8)
        # Per-area obstacle_detection=none (set live by mower_mission): publish an EMPTY
        # obstacle cloud so the costmap gets no detection marks (the buffer stays fresh).
        d('publish_obstacle_points', True)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.p = p
        self.stereo_frames = {str(f) for f in p('stereo_frames') if f}
        self.target = str(p('target_frame'))
        self.depth_frame = str(p('depth_frame'))
        self.model = StereoModel(ref_k=(float(p('ref_fx')), float(p('ref_fy')),
                                        float(p('ref_cx')), float(p('ref_cy'))),
                                 bxf_m=float(p('bxf_mm')) / 1000.0)
        self._max_dt_ns = int(float(p('max_depth_age_s')) * 1e9)
        self._lock = threading.Lock()
        self._ring = collections.deque(maxlen=int(p('depth_ring')))
        self._static = {}
        self._tf = None
        self._info_seen = False
        x, y, z = (float(v) for v in p('fallback_xyz'))
        r, pi, ya = (float(v) for v in p('fallback_rpy'))
        self._fallback = (self._rpy(r, pi, ya), np.array([x, y, z]))
        self.n_in = self.n_ranged = 0

        self.pub = self.create_publisher(Detection2DArray, p('output_topic'), 10)
        self.pose_pub = self.create_publisher(PoseArray, p('obstacles_topic'), 10)
        self.cloud_pub = self.create_publisher(PointCloud2, p('obstacle_points_topic'), 10)

        self._pump = SubscriptionPump(self, 'det_range_inputs')
        self._pump.subscribe(Image, p('depth_topic'), self._on_depth,
                             qos_profile_sensor_data, raw=True, lock=self._lock)
        self._pump.subscribe(CameraInfo, p('camera_info_topic'), self._on_info,
                             qos_profile_sensor_data, lock=self._lock)
        tf_qos = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pump.subscribe(TFMessage, '/tf_static', self._on_tf_static, tf_qos,
                             lock=self._lock)
        self._pump.subscribe(Detection2DArray, p('detections_topic'), self._on_dets, 10,
                             lock=self._lock)
        self._pump.start()
        self.create_timer(30.0, self._report)
        self.get_logger().info(
            f'det_range: {p("detections_topic")} x {p("depth_topic")} -> {p("output_topic")}, '
            f'{p("obstacles_topic")}, {p("obstacle_points_topic")} (ranged frames '
            f'{sorted(self.stereo_frames)})')

    @staticmethod
    def _rpy(r, p, y):
        cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
        return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                         [-sp, cp * sr, cp * cr]])

    # -------------------------------------------------------------- inputs (pump thread)
    def _on_depth(self, data):
        hdr, _pos = _header(data)
        self._ring.append((_stamp_ns(hdr.stamp), data))

    def _on_info(self, msg):
        if msg.width <= 0 or len(msg.k) < 9 or msg.k[0] <= 0:
            return
        k = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]))
        dd = tuple(float(v) for v in msg.d) if msg.distortion_model in (
            'plumb_bob', 'rational_polynomial') else (0.0,) * 5
        if (k, dd[:5]) != (self.model.src_k, tuple(self.model.src_d)[:5]):
            self.model.src_k, self.model.src_d = k, dd[:5]
            self.get_logger().info(f'source intrinsics from camera_info: K={k} D={dd[:5]}')
        self._info_seen = True

    def _on_tf_static(self, msg):
        for t in msg.transforms:
            q, tr = t.transform.rotation, t.transform.translation
            self._static[t.child_frame_id] = (t.header.frame_id,
                                              quat_to_matrix(q.x, q.y, q.z, q.w),
                                              np.array([tr.x, tr.y, tr.z]))
        tf = chain_to(self._static, self.depth_frame, self.target)
        if tf is not None and self._tf is None:
            self.get_logger().info(f'{self.depth_frame} -> {self.target} from /tf_static: '
                                   f't={np.round(tf[1], 3).tolist()}')
        if tf is not None:
            self._tf = tf

    def _depth_for(self, stamp_ns):
        best = None
        for s, data in self._ring:
            dt = abs(s - stamp_ns)
            if dt <= self._max_dt_ns and (best is None or dt < best[0]):
                best = (dt, data)
        if best is None:
            return None
        img = deserialize_message(best[1], Image)
        if img.encoding != '32FC1' or img.height == 0:
            return None
        arr = np.frombuffer(bytes(img.data), dtype=np.float32)
        return arr.reshape(img.height, img.step // 4)[:, :img.width]

    def _on_dets(self, msg):
        self.n_in += 1
        frame = msg.header.frame_id
        depth = None
        if frame in self.stereo_frames and msg.detections:
            depth = self._depth_for(_stamp_ns(msg.header.stamp))
        rot, trans = self._tf if self._tf is not None else self._fallback
        pa = PoseArray()
        pa.header.stamp = msg.header.stamp
        pa.header.frame_id = self.target
        pts = []
        p = self.p
        for det in msg.detections:
            if not det.results:
                det.results.append(ObjectHypothesisWithPose())
            res = None
            if depth is not None:
                res = range_box(self.model, depth, det.bbox.center.position.x,
                                det.bbox.center.position.y, det.bbox.size_x, det.bbox.size_y,
                                z_guess=float(p('z_guess_m')), frac=float(p('box_frac')),
                                min_valid=int(p('min_valid_px')),
                                z_min=float(p('min_range_m')), z_max=float(p('max_range_m')),
                                sigma_disp_px=float(p('sigma_disparity_px')))
            if res is None:
                continue
            xyz = rot @ np.asarray(res.point_ref) + trans
            r0 = det.results[0]
            r0.pose.pose.position.x, r0.pose.pose.position.y, r0.pose.pose.position.z = (
                float(v) for v in xyz)
            r0.pose.pose.orientation.w = 1.0
            cov = [0.0] * 36
            cov[0] = max(res.variance, 1e-6)
            r0.pose.covariance = cov
            pose = Pose()
            pose.position = r0.pose.pose.position
            pose.orientation.w = 1.0
            pa.poses.append(pose)
            pts.append(xyz)
            self.n_ranged += 1
        self.pub.publish(msg)
        self.pose_pub.publish(pa)
        if not bool(p('publish_obstacle_points')):
            pts = []
        self.cloud_pub.publish(cloud_msg(pa.header, pts))

    def _report(self):
        with self._lock:
            self.get_logger().info(
                f'{self.n_in} detection msgs, {self.n_ranged} ranged boxes, depth ring '
                f'{len(self._ring)}, tf={"static" if self._tf is not None else "fallback"}, '
                f'camera_info={"yes" if self._info_seen else "default cam1"}')

    def destroy_node(self):
        self._pump.stop()
        super().destroy_node()


def main(args=None):
    try:  # crash records -> /userdata/ros2/crashes (docs/crash_recovery.md)
        from mower_control.crash_record import install as _install_crash_record
        _install_crash_record('det_range')
    except ImportError:
        pass
    if _DEP_ERROR is not None:
        print(f'[det_range] FATAL: {_DEP_ERROR}; install ros-humble-vision-msgs.',
              file=sys.stderr)
        sys.exit(2)
    rclpy.init(args=args)
    node = DetRange()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
