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

Class-size check (2026-10-13): a stereo range of a class with a height prior
(``size_check_classes`` / ``size_check_heights_m``: person, dog, cat) that is implausibly NEAR
for its unclipped box height is replaced by the box-height range with a large variance
(:func:`det_range.geometry.check_class_size`, rules and limits there). Clipped or very tall
boxes keep the stereo range. The detection stays ranged, so obstacle_guard and the Nav2 cloud
see the corrected position.

Monocular frames (``mono_frames``, default ``rear_camera``; 2026-10-09, rear obstacle sensing
while reversing / docking): no depth image, the bbox bottom-centre is intersected with the
ground plane (:func:`det_range.geometry.mono_range_box`, accuracy notes there) using the
camera's ``camera_info`` (``mono_camera_info_topics``, taken once then unsubscribed: a
camera_info subscriber keeps the on-demand rear capture running) and its pose from
``/tf_static`` (fallback ``mono_fallback_pose``). Same output encoding as the stereo ranges.
Mono points stay OUT of ``/ai/det/obstacles`` / ``obstacle_points`` (Nav2 costmap) unless
``mono_to_costmap``: they are coarse and behind the robot.

Outputs:

* ``/ai/det/detections_ranged``  every input detection, bbox unchanged; ranged ones get
  ``results[0].pose.pose.position`` = box centre in ``base_link`` (m) and
  ``results[0].pose.covariance[0]`` = range variance (m^2, > 0). Unranged: covariance 0.
* ``/ai/det/obstacles``          geometry_msgs/PoseArray of the ranged centroids (base_link)
* ``/ai/det/obstacle_points``    PointCloud2 (xyz float32, base_link) of the same points:
  Nav2 local costmap obstacle source.
* ``/ai/det/obstacle_points_static`` the same for the STATIC classes (``static_classes``:
  trunk, stone, wood, ...): a separate costmap layer keeps these marks for 60 s instead of
  5 s, so a trunk stays lethal while it is beside the camera or out of view during a pivot.

Width (option B, 2026-10-23 trunk incident, docs/analysis/2026-10-23_obstacle_nogo_options.md):
a stereo-ranged box is no longer one centroid point on the clouds but a DISC of points
(``width_points``): its visible width = box width x range / fx, centred half a width behind
the measured front face, radius width / 2 + ``width_margin_m`` (geometry.object_disc). The
width is also written to ``results[0].pose.covariance[35]`` (m; the yaw-yaw slot, unused for a
point) of the ranged detection, 0 = unknown, for obstacle_tracker's sized no-go zones.
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

from det_range.geometry import (DEFAULT_SIZE_CLASSES, DEFAULT_SIZE_HEIGHTS_M,
                                DEFAULT_STATIC_CLASSES, StereoModel, best_label, box_edges_ref,
                                chain_to, check_class_size, mono_range_box, object_disc,
                                points_worthy, quat_to_matrix, range_box, rpy_matrix)
from mower_common.sub_pump import SubscriptionPump, _header


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
        # Only named, confident detections reach obstacle_points (costmaps + obstacle_tracker;
        # geometry.points_worthy, 2026-10-21). Empty list = any named class.
        d('points_classes', [''])
        d('points_min_score', 0.5)
        # Option B (2026-10-23): STATIC classes go to their own cloud (60 s costmap decay);
        # every stereo-ranged box is a disc of its measured width (+ margin) instead of one
        # centroid point (module docstring). Empty static_points_topic = one cloud as before.
        d('static_points_topic', '/ai/det/obstacle_points_static')
        d('static_classes', list(DEFAULT_STATIC_CLASSES))
        d('width_points', True)
        d('width_margin_m', 0.10)
        d('width_spacing_m', 0.05)       # = local costmap resolution
        d('max_width_m', 1.5)
        # Monocular ground-plane ranging (rear camera, see module docstring)
        d('mono_frames', ['rear_camera'])
        d('mono_camera_info_topics', ['/rear_camera/camera_info'])   # parallel to mono_frames
        # pose of each mono frame in target_frame until /tf_static has it: 6 values per frame
        # (x y z roll pitch yaw) = URDF rear_camera_joint (TODO(calib) like the URDF)
        d('mono_fallback_pose', [-0.201, 0.0, 0.25, -1.5707963, 0.0, 1.5707963])
        d('mono_ground_z_m', 0.0)        # ground plane z in target_frame (base_link)
        d('mono_max_range_m', 6.0)
        d('mono_min_depression_deg', 2.0)
        d('mono_sigma_px', 3.0)
        d('mono_sigma_pitch_deg', 1.0)
        d('mono_to_costmap', False)
        # Class-size plausibility of stereo ranges (geometry.check_class_size, 2026-10-13: a
        # spurious near disparity on the lawn ranged a person several metres away at 0.91 m).
        # Parallel lists: class name (det_ros label) -> nominal height (m). Empty = off.
        d('size_check_classes', list(DEFAULT_SIZE_CLASSES))
        d('size_check_heights_m', list(DEFAULT_SIZE_HEIGHTS_M))
        d('size_check_min_ratio', 0.5)     # stereo < ratio * box-height range = implausible
        d('size_check_clip_px', 3.0)       # box within this of the top/bottom edge = clipped
        d('size_check_tall_frac', 0.85)    # box >= this fraction of the frame: stereo kept
        d('size_check_sigma_frac', 0.3)    # replaced range: sigma = this * range
        d('source_image_height', 480)      # until camera_info gives it
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
        self.n_size_rejected = 0
        self._size_log_t = 0.0
        self._src_h = float(p('source_image_height'))
        cls = [str(c).strip().lower() for c in p('size_check_classes') if str(c).strip()]
        hts = [float(v) for v in p('size_check_heights_m')]
        if len(cls) != len(hts):
            self.get_logger().error('size_check_classes / size_check_heights_m differ in '
                                    'length: class-size check OFF')
            cls = []
        self._size_heights = dict(zip(cls, hts))
        self.n_mono = 0
        self.mono_frames = [str(f) for f in p('mono_frames') if f]
        fb = [float(v) for v in p('mono_fallback_pose')]
        self._mono_fallback = {}
        for i, f in enumerate(self.mono_frames):
            v = fb[6 * i:6 * i + 6] if len(fb) >= 6 * (i + 1) else fb[:6]
            if len(v) == 6:
                self._mono_fallback[f] = (rpy_matrix(*v[3:6]), np.array(v[0:3]))
        self._mono_tf = {}
        self._mono_info = {}          # frame -> (K4, D5, height)
        self._mono_info_subs = {}
        topics = [str(t) for t in p('mono_camera_info_topics')]
        for f, topic in zip(self.mono_frames, topics):
            if topic:
                self._mono_info_subs[f] = self.create_subscription(
                    CameraInfo, topic, lambda m, _f=f: self._on_mono_info(m, _f),
                    qos_profile_sensor_data)

        self.pub = self.create_publisher(Detection2DArray, p('output_topic'), 10)
        self.pose_pub = self.create_publisher(PoseArray, p('obstacles_topic'), 10)
        self.cloud_pub = self.create_publisher(PointCloud2, p('obstacle_points_topic'), 10)
        self._pts_classes = {str(c).strip().lower() for c in p('points_classes')
                             if str(c).strip()}
        self._static_classes = {str(c).strip().lower() for c in p('static_classes')
                                if str(c).strip()}
        self.static_pub = (self.create_publisher(PointCloud2, p('static_points_topic'), 10)
                           if p('static_points_topic') else None)

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
        if msg.height > 0:
            self._src_h = float(msg.height)
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
        for f in self.mono_frames:
            mt = chain_to(self._static, f, self.target)
            if mt is not None:
                if f not in self._mono_tf:
                    self.get_logger().info(f'{f} -> {self.target} from /tf_static: '
                                           f't={np.round(mt[1], 3).tolist()}')
                self._mono_tf[f] = mt

    def _on_mono_info(self, msg, frame):
        if msg.width <= 0 or len(msg.k) < 9 or msg.k[0] <= 0:
            return
        k = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]))
        dd = tuple(float(v) for v in msg.d)[:5] if msg.distortion_model in (
            'plumb_bob', 'rational_polynomial') else (0.0,) * 5
        with self._lock:
            self._mono_info[frame] = (k, dd, float(msg.height))
        self.get_logger().info(f'{frame} intrinsics from camera_info {msg.width}x{msg.height}: '
                               f'K={k} (unsubscribing)')
        sub = self._mono_info_subs.pop(frame, None)
        if sub is not None:
            self.destroy_subscription(sub)

    def _mono_range(self, frame, det):
        info = self._mono_info.get(frame)
        pose = self._mono_tf.get(frame) or self._mono_fallback.get(frame)
        if info is None or pose is None:
            return None
        p = self.p
        k, dd, ih = info
        return mono_range_box(det.bbox.center.position.x, det.bbox.center.position.y,
                              det.bbox.size_x, det.bbox.size_y, k, dd, pose[0], pose[1], ih,
                              ground_z=float(p('mono_ground_z_m')),
                              min_depression_deg=float(p('mono_min_depression_deg')),
                              max_range_m=float(p('mono_max_range_m')),
                              sigma_px=float(p('mono_sigma_px')),
                              sigma_pitch_deg=float(p('mono_sigma_pitch_deg')))

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
        static_pts = []
        p = self.p
        mono = frame in self.mono_frames
        for det in msg.detections:
            if not det.results:
                det.results.append(ObjectHypothesisWithPose())
            if mono:
                mr = self._mono_range(frame, det)
                if mr is None:
                    continue
                r0 = det.results[0]
                (r0.pose.pose.position.x, r0.pose.pose.position.y,
                 r0.pose.pose.position.z) = mr.point
                r0.pose.pose.orientation.w = 1.0
                cov = [0.0] * 36
                cov[0] = max(mr.variance, 1e-6)
                r0.pose.covariance = cov
                self.n_mono += 1
                if bool(p('mono_to_costmap')):
                    pose = Pose()
                    pose.position = r0.pose.pose.position
                    pose.orientation.w = 1.0
                    pa.poses.append(pose)
                    pts.append(np.asarray(mr.point))
                continue
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
            point_ref, variance = self._size_checked(det, res)
            xyz = rot @ np.asarray(point_ref) + trans
            r0 = det.results[0]
            r0.pose.pose.position.x, r0.pose.pose.position.y, r0.pose.pose.position.z = (
                float(v) for v in xyz)
            r0.pose.pose.orientation.w = 1.0
            disc = self._disc(det, point_ref, rot, trans)
            cov = [0.0] * 36
            cov[0] = max(variance, 1e-6)
            cov[35] = disc.width if disc is not None else 0.0
            r0.pose.covariance = cov
            pose = Pose()
            pose.position = r0.pose.pose.position
            pose.orientation.w = 1.0
            pa.poses.append(pose)
            hyps = [(r.hypothesis.class_id, r.hypothesis.score) for r in det.results]
            if points_worthy(hyps, self._pts_classes, p('points_min_score')):
                out = (static_pts if self.static_pub is not None
                       and best_label(hyps) in self._static_classes else pts)
                if disc is not None and bool(p('width_points')):
                    out.extend(disc.points)
                else:
                    out.append(xyz)
            self.n_ranged += 1
        self.pub.publish(msg)
        self.pose_pub.publish(pa)
        if not bool(p('publish_obstacle_points')):
            pts, static_pts = [], []
        self.cloud_pub.publish(cloud_msg(pa.header, pts))
        if self.static_pub is not None:
            self.static_pub.publish(cloud_msg(pa.header, static_pts))

    def _disc(self, det, point_ref, rot, trans):
        """The ranged box as a disc of its measured width in the target frame
        (geometry.object_disc), or None when the box has no width."""
        b = det.bbox
        z = float(point_ref[2])
        if b.size_x <= 0.0 or z <= 0.0:
            return None
        p = self.p
        edges = [rot @ e + trans for e in box_edges_ref(
            self.model, b.center.position.x, b.center.position.y, b.size_x, z)]
        return object_disc(edges[0], edges[1], trans[:2],
                           margin=float(p('width_margin_m')),
                           spacing=float(p('width_spacing_m')),
                           max_width=float(p('max_width_m')))

    def _size_checked(self, det, res):
        """(point_ref, variance) of a stereo range after the class-size check: an
        implausibly near range is moved along its ray to the box-height range."""
        if not self._size_heights or not det.results:
            return res.point_ref, res.variance
        best = max(det.results, key=lambda r: r.hypothesis.score)
        label = str(best.hypothesis.class_id).strip().lower()
        hm = self._size_heights.get(label)
        if hm is None:
            return res.point_ref, res.variance
        p = self.p
        b = det.bbox
        sc = check_class_size(self.model, res.z, res.variance, b.center.position.x,
                              b.center.position.y, b.size_y, self._src_h, hm,
                              min_ratio=float(p('size_check_min_ratio')),
                              clip_px=float(p('size_check_clip_px')),
                              tall_frac=float(p('size_check_tall_frac')),
                              size_sigma_frac=float(p('size_check_sigma_frac')))
        if sc.verdict != 'implausible':
            return res.point_ref, res.variance
        self.n_size_rejected += 1
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._size_log_t >= 2.0:
            self._size_log_t = now
            self.get_logger().info(
                '%s: stereo %.2f m implausible for a %d px tall box (%.0f %% of %d px, not '
                'clipped; %.1f m nominal -> %.2f m): using the box-height range'
                % (label, res.z, round(b.size_y), 100.0 * b.size_y / max(self._src_h, 1.0),
                   self._src_h, hm, sc.z))
        s = sc.z / res.z if res.z > 0 else 1.0
        return tuple(float(v) * s for v in res.point_ref), sc.variance

    def _report(self):
        with self._lock:
            self.get_logger().info(
                f'{self.n_in} detection msgs, {self.n_ranged} ranged boxes, '
                f'{self.n_mono} mono-ranged, {self.n_size_rejected} stereo ranges replaced '
                f'by the class-size check, depth ring '
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
        print(f'[det_range] FATAL: {_DEP_ERROR}; install ros-jazzy-vision-msgs.',
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
