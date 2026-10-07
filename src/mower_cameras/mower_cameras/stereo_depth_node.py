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
* ``~/points``           PointCloud2 xyz float32, decimated, range-limited, FILTERED: speckle
                         + neighbour noise removal, ground removed (``ground_plane_fit``),
                         temporal persistence (``persist_frames``). Only obstacle points
                         remain (Nav2 obstacle source).
* ``~/ground_plane``     Float32MultiArray [nx, ny, nz, h, inliers, fit_ok] in the optical
                         frame (n points down to the ground, h = camera height above it)
* ``~/stats``            String JSON at 1 Hz: points raw / after noise / after ground /
                         published, fit ok, ms per frame

Filter details: mower_cameras/depth_filters.py. The prior ground plane comes from TF
(base_footprint -> frame_id; URDF pose measured from the depth ground plane 2026-10-06)
with ``ground_prior_height_m`` / ``ground_prior_pitch_up_deg`` as fallback.
"""
from __future__ import annotations

import array
import json
import math
import time

import numpy as np
import rclpy
import rclpy.time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, String

from mower_cameras import depth_filters as dfl
from mower_cameras.activity_gate import ActivityWatch

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
        d('cloud_fps', 5.0)              # filtered cloud rate cap (CPU); depth image keeps fps
        # --- cloud stamping (Nav2 costmap deadlock workaround, 2026-10-07) ---
        # Humble tf2_ros 0.25.x: a cloud that is NOT transformable on arrival in the costmap
        # goes through tf2_ros::Buffer::waitForTransform, which can deadlock against the
        # costmap's TF listener thread (Buffer mutex vs BufferCore transformable_requests
        # mutex, lock-order inversion; gdb-confirmed in controller_server). The EKF's
        # odom->base_link lags wall clock by 5-200 ms, so a cloud stamped now() was ALWAYS
        # ahead of TF and the obstacle layer froze within ~1 s of every start. Stamp the
        # cloud at the latest received odom TF time minus a margin: transformable on arrival,
        # waitForTransform returns immediately and no request is ever left pending.
        d('stamp_tf_parent', 'odom')     # '' = old behaviour (stamp = now)
        d('stamp_tf_child', 'base_link')
        d('stamp_tf_margin_s', 0.05)     # margin under the newest TF stamp
        d('stamp_max_lag_s', 0.3)        # TF older than this (or none): stamp = now - this
        # --- noise filters (before the cloud) ---
        d('speckle_max_size', 200)       # px; disparity blobs smaller than this are dropped
        d('speckle_max_diff_px', 1.0)    # disparity step that splits blobs
        d('min_valid_neighbours', 5)     # of the 3x3 window incl. the pixel itself
        # --- ground removal ---
        d('ground_plane_fit', True)      # RANSAC ground plane each frame
        d('ground_margin_m', 0.12)       # drop points < this above the plane
        d('ground_base_frame', 'base_footprint')
        d('ground_prior_height_m', 0.24)      # fallback when TF is unavailable
        d('ground_prior_pitch_up_deg', 0.92)  # (measured 2026-10-06)
        d('ground_max_tilt_deg', 8.0)    # fit rejected beyond this from the prior
        d('ground_max_dh_m', 0.08)
        # --- temporal ---
        d('persist_frames', 2)           # voxel seen in N consecutive frames
        d('persist_voxel_m', 0.10)
        # --- idle (/mission/activity docked_idle|idle): close /dev/video11, no maths ---
        # Nav2 is idle then (no controller running), so the obstacle source's
        # expected_update_rate gate is irrelevant. Any other activity (incl. UNDOCKING and
        # PREFLIGHT_CHECK, see mower_mission/activity.py) reopens within ~0.2 s + open time.
        d('idle_pause', True)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {n: p(n) for n in ('frame_id', 'bxf_mm', 'disparity_scale', 'fx', 'fy',
                                       'cx', 'cy', 'cloud_step', 'min_range_m', 'max_range_m',
                                       'speckle_max_size', 'speckle_max_diff_px',
                                       'min_valid_neighbours', 'ground_plane_fit',
                                       'ground_margin_m', 'ground_max_tilt_deg',
                                       'ground_max_dh_m')}
        self.on_demand = bool(p('publish_on_demand'))
        self._stamp_parent = str(p('stamp_tf_parent'))
        self._stamp_child = str(p('stamp_tf_child'))
        self._stamp_margin_ns = int(float(p('stamp_tf_margin_s')) * 1e9)
        self._stamp_max_lag_ns = int(float(p('stamp_max_lag_s')) * 1e9)
        self._last_tf_ns = 0
        if self._stamp_parent:
            from tf2_msgs.msg import TFMessage
            # Own light /tf subscription (only reads the stamp of parent->child), independent
            # of the ground-prior TF listener that is dropped once the static chain is known.
            self._tf_msg_type = TFMessage
            self._tf_sub = None
        self.ground_base = p('ground_base_frame')
        cf = float(p('cloud_fps'))
        self.cloud_period = 0.9 / cf if cf > 0 else 0.0
        self._last_cloud = 0.0
        self.prior = dfl.plane_from_pose(p('ground_prior_height_m'),
                                         math.radians(p('ground_prior_pitch_up_deg')))
        self.prior_from_tf = False
        self.plane = self.prior
        self.persist = dfl.Persistence(p('persist_frames'), p('persist_voxel_m'))
        self.rng = np.random.default_rng(0)
        self._stats = {'frames': 0, 'raw': 0, 'denoised': 0, 'obstacle': 0, 'published': 0,
                       'fit_ok': 0, 'ms': 0.0}
        self._tf_buf = None
        try:
            import tf2_ros
            self._tf_buf = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)
        except ImportError:  # pragma: no cover
            self.get_logger().warn('tf2_ros missing: ground prior from parameters')
        qos = rclpy.qos.qos_profile_sensor_data
        self.depth_pub = self.create_publisher(Image, '~/depth/image_raw', qos)
        self.mono_pub = self.create_publisher(Image, '~/image_mono', qos)
        self.cloud_pub = self.create_publisher(PointCloud2, '~/points', qos)
        self.plane_pub = self.create_publisher(Float32MultiArray, '~/ground_plane', 10)
        self.stats_pub = self.create_publisher(String, '~/stats', 10)
        self.create_timer(1.0, self._publish_stats)
        self.loop = CaptureLoop(self.get_logger(), p('video_device'), p('width'), p('height'),
                                p('pixel_format'), p('fps'), self._on_frame, 'v4l2',
                                name='cap:simor',
                                want=self._wanted if self.on_demand else None)
        self.idle_pause = bool(p('idle_pause'))
        self.activity = ActivityWatch(self._on_activity)
        self._on_activity(None)
        self.activity.subscribe(self)
        if self.idle_pause:
            self.loop.pause_fn = lambda: self.activity.low_power
        self.loop.start()
        self.get_logger().info(
            f'stereo_depth {p("video_device")} {p("width")}x{p("height")} fps<={p("fps")} '
            f'bxf={self.cfg["bxf_mm"]} scale={self.cfg["disparity_scale"]} '
            f'-> ~/points, ~/depth/image_raw ({self.cfg["frame_id"]})')

    def _on_activity(self, act):
        """Drop the Python /tf subscription while paused (it deserialises every /tf msg)."""
        paused = self.idle_pause and self.activity.low_power if act is not None else False
        if act is not None:
            self.get_logger().info(f'stereo_depth: activity {act} -> '
                                   f'{"paused" if paused else "streaming"}')
        if not self._stamp_parent:
            return
        if paused and self._tf_sub is not None:
            self.destroy_subscription(self._tf_sub)
            self._tf_sub = None
        elif not paused and self._tf_sub is None:
            self._tf_sub = self.create_subscription(self._tf_msg_type, '/tf', self._on_tf, 50)

    def _wanted(self):
        return any(x.get_subscription_count() > 0
                   for x in (self.depth_pub, self.mono_pub, self.cloud_pub))

    def _on_tf(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == self._stamp_child and t.header.frame_id == self._stamp_parent:
                ns = t.header.stamp.sec * 1_000_000_000 + t.header.stamp.nanosec
                if ns > self._last_tf_ns:
                    self._last_tf_ns = ns

    def cloud_stamp_ns(self, now_ns):
        """Stamp for the cloud: never ahead of the newest odom TF (see stamp_tf_parent)."""
        if not self._stamp_parent:
            return now_ns
        floor = now_ns - self._stamp_max_lag_ns
        if self._last_tf_ns <= floor:
            return floor
        return min(now_ns, self._last_tf_ns - self._stamp_margin_ns)

    def _on_frame(self, data, cap):
        c = self.cfg
        now_ns = self.get_clock().now().nanoseconds
        s_ns = self.cloud_stamp_ns(now_ns)
        stamp = rclpy.time.Time(nanoseconds=s_ns).to_msg()
        raw, grey = decode_simor(data, cap.width, cap.height, cap.bytesperline or cap.width * 3)
        now = time.monotonic()
        if (now - self._last_cloud >= self.cloud_period
                and (not self.on_demand or self.cloud_pub.get_subscription_count() > 0)):
            self._last_cloud = now
            pts = self.filtered_points(raw)
            self.cloud_pub.publish(points_msg(stamp, c['frame_id'], pts))
        if not self.on_demand or self.depth_pub.get_subscription_count() > 0:
            z = disparity_to_depth(raw, c['bxf_mm'], c['disparity_scale'])
            self.depth_pub.publish(depth_image_msg(stamp, c['frame_id'], z))
        if not self.on_demand or self.mono_pub.get_subscription_count() > 0:
            self.mono_pub.publish(to_image_msg(stamp, c['frame_id'], grey))

    def _update_prior_from_tf(self):
        if self.prior_from_tf or self._tf_buf is None:
            return
        try:
            tf = self._tf_buf.lookup_transform(self.ground_base, self.cfg['frame_id'],
                                               rclpy.time.Time())
        except Exception:  # noqa: BLE001 - TF not up yet, keep the parameter prior
            return
        q = tf.transform.rotation
        x, y, zq, w = q.x, q.y, q.z, q.w
        # third row of R(optical -> base): base z axis expressed in the optical frame
        up = np.array([2 * (x * zq - y * w), 2 * (y * zq + x * w), 1 - 2 * (x * x + y * y)])
        n = -up / np.linalg.norm(up)
        self.prior = (n, float(tf.transform.translation.z))
        self.plane = self.prior
        self.prior_from_tf = True
        # The chain is static (URDF fixed joints): stop deserialising every /tf message in
        # Python (no need to keep a Python /tf subscription).
        try:
            self._tf_listener.unregister()
        except Exception:  # noqa: BLE001
            pass
        self._tf_listener = None
        self._tf_buf = None
        self.get_logger().info(f'ground prior from TF: n={np.round(n, 4).tolist()} '
                               f'h={self.prior[1]:.3f} m')

    def filtered_points(self, raw):
        """raw disparity -> obstacle-only optical-frame points (noise + ground removed)."""
        c = self.cfg
        t0 = time.monotonic()
        self._update_prior_from_tf()
        step = int(c['cloud_step'])
        # Filter at half resolution (the cloud keeps every cloud_step-th pixel anyway):
        # 4x fewer pixels for speckle/neighbour; speckle area scales by 1/4.
        sub = 2 if step % 2 == 0 else 1
        r = raw[::sub, ::sub]
        r = dfl.speckle_filter(r, int(c['speckle_max_size']) // (sub * sub),
                               c['speckle_max_diff_px'], c['disparity_scale'])
        valid = dfl.neighbour_filter(r > 0, int(c['min_valid_neighbours']))
        r = np.where(valid, r, 0)
        z = disparity_to_depth(r, c['bxf_mm'], c['disparity_scale'])
        # pixel u' = u / sub  ->  fx' = fx / sub, cx' = cx / sub
        pts = depth_to_points(z, c['fx'] / sub, c['fy'] / sub, c['cx'] / sub, c['cy'] / sub,
                              step // sub, c['min_range_m'], c['max_range_m'])
        st = self._stats
        st['frames'] += 1
        st['denoised'] += len(pts)
        st['raw'] += int(np.count_nonzero(raw[::step, ::step]))
        if c['ground_plane_fit']:
            plane, k, ok = dfl.fit_ground_plane(pts, self.prior,
                                                max_tilt_deg=c['ground_max_tilt_deg'],
                                                max_dh_m=c['ground_max_dh_m'], rng=self.rng)
            self.plane = plane
            st['fit_ok'] += int(ok)
            m = Float32MultiArray()
            m.data = [float(v) for v in plane[0]] + [float(plane[1]), float(k), float(ok)]
            self.plane_pub.publish(m)
        pts = dfl.remove_ground(pts, self.plane, c['ground_margin_m'])
        st['obstacle'] += len(pts)
        pts = self.persist(pts).astype(np.float32)
        st['published'] += len(pts)
        st['ms'] += (time.monotonic() - t0) * 1000.0
        return pts

    def _publish_stats(self):
        st = self._stats
        f = max(st['frames'], 1)
        out = {'frames': st['frames'], 'fit_ok': st['fit_ok'],
               'ms_per_frame': round(st['ms'] / f, 1),
               'plane': [round(float(v), 4) for v in self.plane[0]] + [round(self.plane[1], 3)]}
        for k in ('raw', 'denoised', 'obstacle', 'published'):
            out[k + '_per_frame'] = round(st[k] / f, 1)
        self.stats_pub.publish(String(data=json.dumps(out)))
        for k in st:
            st[k] = 0 if k != 'ms' else 0.0

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
