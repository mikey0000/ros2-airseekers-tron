"""``nongrass_projector`` — seg_ros class mask -> per-map-cell non-grass / grass votes.

2026-10-09 (docs/grass_segmentation.md). Geometry and labelling live in
:mod:`seg_ros.ground_projection` (pure, unit-tested); this node only gathers the inputs.

Inputs:

* ``/ai/seg/mask`` + ``/ai/seg/confidence`` (mono8, paired by identical stamp) from seg_ros.
  Only frames whose ``frame_id`` is in ``camera_frames`` are used (default ``vio_camera``,
  the front stereo right colour eye; the OA cameras have no extrinsics).
* ``camera_info_topics`` (one per camera; first message only, else the cam1 defaults).
* ``/tf_static`` for ``ref_frame`` -> ``base_link`` (fallback: the URDF value below).
* ``/odometry/filtered_map`` (map frame): robot pose at the mask stamp (nearest within
  ``max_pose_dt_s``). Not /tf: a Python TransformListener at the EKF rate costs a lot of CPU
  on the RK3588 for one lookup every 0.5 s.
* ``/stereo_depth/ground_plane`` (live plane fit; optional) and
  ``/stereo_depth/depth/image_raw`` (raw bytes ring, only the frame nearest the mask stamp
  is deserialized; ``depth_check``).

Output: ``/ai/seg/ground_cells`` PointCloud2 in ``map`` (one point per 0.1 m cell per
accepted frame): float32 ``x y z nongrass weight vx vy`` with ``nongrass`` 1/0, ``weight``
= samples in the cell, ``vx vy`` the camera position in map (the viewpoint, for the
multi-viewpoint confirmation in mower_map). Consumed by mower_map map_server_node
(``nongrass_topic``), which owns the memory, decay, per-area enable and the Nav2 layer.
Frames are dropped while the robot stands still (``MotionGate``) or when the pose / plane
is missing; nothing is published then. ``~/stats`` String JSON every 10 s.
"""
from __future__ import annotations

import collections
import json
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, String
from tf2_msgs.msg import TFMessage

from seg_ros import ground_projection as gp

FIELDS = ('x', 'y', 'z', 'nongrass', 'weight', 'vx', 'vy')


def stamp_s(stamp) -> float:
    return int(stamp.sec) + int(stamp.nanosec) * 1e-9


def cells_cloud(header, cx, cy, nongrass, weight, vx, vy) -> PointCloud2:
    n = len(cx)
    arr = np.zeros((n, len(FIELDS)), np.float32)
    arr[:, 0], arr[:, 1] = cx, cy
    arr[:, 3] = np.asarray(nongrass, dtype=np.float32)
    arr[:, 4] = weight
    arr[:, 5], arr[:, 6] = vx, vy
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = [PointField(name=f, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, f in enumerate(FIELDS)]
    msg.is_bigendian = False
    msg.point_step = 4 * len(FIELDS)
    msg.row_step = msg.point_step * n
    msg.is_dense = True
    msg.data = arr.tobytes()
    return msg


def image_array(msg) -> np.ndarray:
    """mono8 / 32FC1 sensor_msgs/Image -> 2D numpy (no cv_bridge)."""
    if msg.encoding == '32FC1':
        a = np.frombuffer(bytes(msg.data), dtype=np.float32)
        return a.reshape(msg.height, msg.step // 4)[:, :msg.width]
    a = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    return a.reshape(msg.height, msg.step)[:, :msg.width]


class NonGrassProjector(Node):
    def __init__(self):
        super().__init__('nongrass_projector')
        d = self.declare_parameter
        d('mask_topic', '/ai/seg/mask')
        d('confidence_topic', '/ai/seg/confidence')   # '' = trust every class pixel
        d('output_topic', '/ai/seg/ground_cells')
        d('odom_topic', '/odometry/filtered_map')
        d('map_frame', 'map')
        d('base_frame', 'base_link')
        # Cameras, by mask frame_id. Only vio_camera is projectable today (see seg.yaml).
        d('camera_frames', ['vio_camera'])
        d('camera_info_topics', ['/vio/right/camera_info'])
        # Rigid parent of each camera in TF and the camera's pose in it as
        # stereo_params-style rvec/T (X_cam = R X_ref + T), 6 numbers per camera.
        d('ref_frames', ['stereo_camera_optical'])
        d('cam_from_ref', list(gp.STEREO_RVEC) + list(gp.STEREO_T_M))
        # URDF stereo_camera_optical_joint (measured 2026-10-06) until /tf_static arrives.
        d('ref_fallback_xyz', [0.466, 0.0, 0.240])
        d('ref_fallback_rpy', [-1.5547956, 0.0, -1.5707963])
        d('ground_plane_topic', '/stereo_depth/ground_plane')   # '' = URDF flat ground
        d('ground_plane_frame', 'stereo_camera_optical')
        d('ground_plane_max_age_s', 1.0)
        d('ground_plane_max_dh_m', 0.06)
        d('ground_plane_max_angle_deg', 6.0)
        d('depth_check', True)
        d('depth_topic', '/stereo_depth/depth/image_raw')
        d('depth_max_dt_s', 0.15)
        d('depth_tol_m', 0.08)
        d('depth_tol_frac', 0.10)
        d('stride_px', 8)
        d('cell_m', 0.1)
        d('min_range_m', 0.35)
        d('max_range_m', 2.0)
        d('max_lateral_m', 1.5)
        d('min_conf_nongrass', 0.8)
        d('min_conf_grass', 0.6)
        d('nongrass_classes', list(gp.NONGRASS_CLASSES))
        d('grass_classes', list(gp.GRASS_CLASSES))
        d('max_pose_dt_s', 0.1)
        d('min_move_m', 0.15)
        d('min_turn_deg', 10.0)
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        self.map_frame, self.base_frame = p('map_frame'), p('base_frame')
        self.params = gp.ProjectionParams(
            min_range_m=float(p('min_range_m')), max_range_m=float(p('max_range_m')),
            max_lateral_m=float(p('max_lateral_m')),
            min_conf_nongrass=float(p('min_conf_nongrass')),
            min_conf_grass=float(p('min_conf_grass')),
            nongrass_classes=tuple(int(c) for c in p('nongrass_classes')),
            grass_classes=tuple(int(c) for c in p('grass_classes')),
            depth_tol_m=float(p('depth_tol_m')), depth_tol_frac=float(p('depth_tol_frac')))
        self.stride = int(p('stride_px'))
        self.cell = float(p('cell_m'))
        self.gate = gp.MotionGate(float(p('min_move_m')), float(p('min_turn_deg')))

        frames = list(p('camera_frames'))
        infos = list(p('camera_info_topics'))
        refs = list(p('ref_frames'))
        ext = [float(v) for v in p('cam_from_ref')]
        self.cams = {}
        for i, f in enumerate(frames):
            r6 = ext[6 * i:6 * i + 6] if len(ext) >= 6 * i + 6 else [0.0] * 6
            self.cams[f] = {
                'ref': refs[i] if i < len(refs) else f,
                'cam_in_ref': gp.src_in_ref(r6[:3], r6[3:]),
                'k': gp.CAM1_K, 'd': gp.CAM1_D, 'size': gp.CAM1_SIZE, 'info': False,
                'grid': None,
            }
            if i < len(infos) and infos[i]:
                self._sub_info(f, infos[i])

        xyz = [float(v) for v in p('ref_fallback_xyz')]
        rpy = [float(v) for v in p('ref_fallback_rpy')]
        self._fallback_ref = (_rpy_matrix(*rpy), np.array(xyz))
        self._static = {}                       # child -> (parent, R, t)

        latched = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TFMessage, '/tf_static', self._on_tf_static, latched)
        self._poses = collections.deque(maxlen=200)
        self.create_subscription(Odometry, p('odom_topic'), self._on_odom, 20)
        self._plane = None                      # (t_monotonic, (n, d))
        self.plane_frame = p('ground_plane_frame')
        if p('ground_plane_topic'):
            self.create_subscription(Float32MultiArray, p('ground_plane_topic'),
                                     self._on_plane, 5)
        self._depth = collections.deque(maxlen=6)   # (stamp_s, raw bytes)
        if p('depth_check') and p('depth_topic'):
            self.create_subscription(Image, p('depth_topic'), self._on_depth,
                                     qos_profile_sensor_data, raw=True)
        self._conf = collections.OrderedDict()  # stamp_ns -> Image
        self._use_conf = bool(p('confidence_topic'))
        if self._use_conf:
            self.create_subscription(Image, p('confidence_topic'), self._on_conf, 5)
        self.create_subscription(Image, p('mask_topic'), self._on_mask, 5)
        self.pub = self.create_publisher(PointCloud2, p('output_topic'), 5)
        self.stats_pub = self.create_publisher(String, '~/stats', 1)
        self.stats = collections.Counter()
        self.create_timer(10.0, self._publish_stats)
        self.get_logger().info(
            f'nongrass_projector up: cameras {frames}, range {self.params.min_range_m}-'
            f'{self.params.max_range_m} m, conf >= {self.params.min_conf_nongrass} '
            f'(non-grass {list(self.params.nongrass_classes)}), depth_check '
            f'{bool(p("depth_check"))} -> {p("output_topic")}')

    # ------------------------------------------------------------------ inputs
    def _sub_info(self, frame, topic):
        holder = {}

        def cb(msg, frame=frame):
            cam = self.cams[frame]
            if not cam['info'] and msg.width > 0 and msg.k[0] > 0:
                cam['k'] = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
                cam['d'] = tuple(msg.d) if msg.distortion_model in ('plumb_bob',
                                                                   'rational_polynomial') \
                    else ()
                cam['size'] = (int(msg.width), int(msg.height))
                cam['info'], cam['grid'] = True, None
                self.get_logger().info(f'{frame}: intrinsics from {topic} '
                                       f'{cam["size"][0]}x{cam["size"][1]}')
                sub = holder.pop('sub', None)   # static calibration: one message is enough
                if sub is not None:
                    self.destroy_subscription(sub)
        holder['sub'] = self.create_subscription(CameraInfo, topic, cb, qos_profile_sensor_data)

    def _on_tf_static(self, msg):
        for t in msg.transforms:
            q, tr = t.transform.rotation, t.transform.translation
            self._static[t.child_frame_id] = (
                t.header.frame_id, gp.quat_to_matrix(q.x, q.y, q.z, q.w),
                np.array([tr.x, tr.y, tr.z]))

    def _chain(self, child, target):
        acc = (np.eye(3), np.zeros(3))
        frame = child
        for _ in range(16):
            if frame == target:
                return acc
            if frame not in self._static:
                return None
            parent, r, t = self._static[frame]
            acc = gp.compose((r, t), acc)
            frame = parent
        return None

    def _on_odom(self, msg):
        q, pp = msg.pose.pose.orientation, msg.pose.pose.position
        self._poses.append((stamp_s(msg.header.stamp), pp.x, pp.y, pp.z, q.x, q.y, q.z, q.w))

    def _on_plane(self, msg):
        pl = gp.plane_from_fit(list(msg.data))
        if pl is not None:
            self._plane = (time.monotonic(), pl)

    def _on_depth(self, raw):
        # header stamp = first 8 bytes after the 4-byte CDR encapsulation header
        try:
            sec, nsec = np.frombuffer(raw, dtype='<i4', count=2, offset=4)
        except ValueError:
            return
        self._depth.append((int(sec) + int(nsec) * 1e-9, raw))

    def _on_conf(self, msg):
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id)
        self._conf[key] = msg
        while len(self._conf) > 6:
            self._conf.popitem(last=False)

    # ------------------------------------------------------------------ helpers
    def _pose_at(self, t):
        best = None
        for rec in self._poses:
            dt = abs(rec[0] - t)
            if best is None or dt < best[0]:
                best = (dt, rec)
        if best is None or best[0] > float(self.get_parameter('max_pose_dt_s').value):
            return None
        _, (_t, x, y, z, qx, qy, qz, qw) = best
        return gp.quat_to_matrix(qx, qy, qz, qw), np.array([x, y, z])

    def _depth_at(self, t):
        best = None
        for st, raw in self._depth:
            dt = abs(st - t)
            if best is None or dt < best[0]:
                best = (dt, raw)
        if best is None or best[0] > float(self.get_parameter('depth_max_dt_s').value):
            return None
        try:
            img = deserialize_message(best[1], Image)
            return image_array(img) if img.encoding == '32FC1' else None
        except Exception:  # noqa: BLE001 - a bad frame only disables the check this frame
            return None

    def _plane_base(self, ref_in_base):
        """Live ground plane in base_link if fresh and near the URDF prior, else z = 0."""
        prior = gp.flat_ground_base()
        if self._plane is None or not self.plane_frame:
            return prior, 'flat'
        t0, pl = self._plane
        if time.monotonic() - t0 > float(self.get_parameter('ground_plane_max_age_s').value):
            return prior, 'flat'
        # same rigid frame as the camera's ref in the default setup (stereo_camera_optical)
        tf = self._chain(self.plane_frame, self.base_frame) or ref_in_base
        pb = gp.transform_plane(pl[0], pl[1], tf)
        # live n points DOWN from the camera; flip to the prior's convention (up, d ~ 0)
        n, dd = pb
        if float(n @ prior[0]) < 0:
            n, dd = -n, -dd
        if not gp.plane_agrees((n, dd), prior, float(self.get_parameter(
                'ground_plane_max_dh_m').value), float(self.get_parameter(
                'ground_plane_max_angle_deg').value)):
            self.stats['plane_rejected'] += 1
            return prior, 'flat'
        return (n, dd), 'live'

    # ------------------------------------------------------------------ main
    def _on_mask(self, msg):
        cam = self.cams.get(msg.header.frame_id)
        self.stats['masks'] += 1
        if cam is None:
            self.stats['skip_frame'] += 1
            return
        conf_img = None
        if self._use_conf:
            key = (msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id)
            conf_img = self._conf.pop(key, None)
            if conf_img is None:
                self.stats['skip_no_conf'] += 1
                return
        t = stamp_s(msg.header.stamp)
        base_in_map = self._pose_at(t)
        if base_in_map is None:
            self.stats['skip_no_pose'] += 1
            return
        yaw = math.atan2(base_in_map[0][1, 0], base_in_map[0][0, 0])
        if not self.gate.accept(float(base_in_map[1][0]), float(base_in_map[1][1]), yaw):
            self.stats['skip_still'] += 1
            return
        ref_in_base = self._chain(cam['ref'], self.base_frame) or self._fallback_ref
        cam_in_base = gp.compose(ref_in_base, cam['cam_in_ref'])
        classes = image_array(msg)
        h, w = classes.shape
        if cam['grid'] is None or cam['grid'].size != (w, h):
            k = gp.scale_intrinsics(cam['k'], cam['size'], (w, h)) \
                if tuple(cam['size']) != (w, h) else cam['k']
            cam['grid'] = gp.make_ray_grid((w, h), k, cam['d'], self.stride)
        conf = image_array(conf_img) if conf_img is not None else None
        if conf is not None and conf.shape != classes.shape:
            self.stats['skip_conf_size'] += 1
            return
        plane, kind = self._plane_base(ref_in_base)
        self.stats['plane_' + kind] += 1
        depth = self._depth_at(t) if self._depth else None
        base_to_ref = gp.invert(ref_in_base) if depth is not None else None
        votes = gp.project_frame(classes, conf, cam['grid'], cam_in_base, plane, self.params,
                                 depth, base_to_ref)
        self.stats['elevated'] += votes.n_elevated
        mx, my = gp.to_map(votes, base_in_map)
        cx, cy, ng, n = gp.bin_cells(mx, my, votes.vote, self.cell)
        if len(cx) == 0:
            self.stats['empty'] += 1
            return
        view = cam_in_base[1] @ base_in_map[0].T + base_in_map[1]
        header = msg.header
        header.frame_id = self.map_frame
        self.pub.publish(cells_cloud(header, cx, cy, ng, n,
                                     np.full(len(cx), view[0]), np.full(len(cx), view[1])))
        self.stats['frames'] += 1
        self.stats['cells_nongrass'] += int(ng.sum())
        self.stats['cells_grass'] += int((~ng).sum())

    def _publish_stats(self):
        self.stats_pub.publish(String(data=json.dumps(dict(self.stats), sort_keys=True)))


def _rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def main(args=None):
    rclpy.init(args=args)
    node = NonGrassProjector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
