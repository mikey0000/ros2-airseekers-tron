"""``mower_control/fixed_frame_relay`` - republish robot-frame clouds in odom / map (2026-10-09).

Why: the Jazzy costmap ObstacleLayer feeds every source through a tf2 MessageFilter. A cloud
that is not transformable into the costmap frame when it arrives goes through
``tf2_ros::Buffer::waitForTransform``, which can deadlock (Buffer mutex vs transformable
requests; gdb-confirmed 2026-10-07). Live 2026-10-09 (load 13 on 8 cores): base_link clouds
stamped "now" (the 5 Hz /bumper_cloud heartbeat, /ai/det/obstacle_points) regularly arrived
before the matching odom TF and froze the local costmap's sources for good ("stereo stale").

This node transforms each cloud with the NEWEST odom<-base_link sample it has, stamps the
output at that sample's time and publishes it already in ``odom`` (local costmap) and, when
map->odom is known, in ``map`` (global costmap). Target frame == cloud frame, so the costmap
filter never waits. Robot-frame points lose nothing (the TF sample is <= 1 control period
old). The raw topics stay for consumers that do their own lookups (collision_monitor).
"""
from __future__ import annotations

import array

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage


def tf_matrix(tr):
    q, t = tr.rotation, tr.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), t.x],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), t.y],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), t.z],
        [0.0, 0.0, 0.0, 1.0]])


def cloud_xyz(msg) -> np.ndarray:
    """xyz float32 points of a PointCloud2 (any point_step with float32 x/y/z fields)."""
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3), np.float32)
    off = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
    cols = [raw[:, off[k]:off[k] + 4].copy().view(np.float32).reshape(n) for k in 'xyz']
    return np.stack(cols, axis=1)


def make_cloud(stamp, frame_id, pts) -> PointCloud2:
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = 1, int(pts.shape[0])
    msg.fields = [PointField(name=k, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, k in enumerate('xyz')]
    msg.is_bigendian, msg.point_step = False, 12
    msg.row_step, msg.is_dense = 12 * msg.width, True
    data = array.array('B')
    data.frombytes(np.ascontiguousarray(pts, np.float32).tobytes())
    msg.data = data
    return msg


def relay_plan(src_frame, pts, now_ns, max_lag_ns, base_frame, odom_tf, map_tf, static_tf):
    """-> [(frame, stamp_ns, points)] or [] (TF missing / stale / unknown source frame).
    odom_tf: (ns, 4x4 odom<-base); map_tf: (ns, 4x4 map<-odom, static) or None;
    static_tf: {frame: 4x4 base<-frame} (identity for base_frame itself)."""
    if odom_tf is None or odom_tf[0] <= now_ns - max_lag_ns:
        return []
    T_bs = np.eye(4) if src_frame == base_frame else static_tf.get(src_frame)
    if T_bs is None:
        return []
    ns, T_ob = odom_tf
    T = T_ob @ T_bs
    p = np.asarray(pts, np.float64)
    p_odom = (p @ T[:3, :3].T + T[:3, 3]).astype(np.float32) if len(p) else \
        np.zeros((0, 3), np.float32)
    out = [('odom', ns, p_odom)]
    if map_tf is not None and (map_tf[2] or map_tf[0] > now_ns - max_lag_ns):
        M = map_tf[1]
        p_map = (p_odom.astype(np.float64) @ M[:3, :3].T + M[:3, 3]).astype(np.float32) \
            if len(p_odom) else np.zeros((0, 3), np.float32)
        out.append(('map', ns, p_map))
    return out


class FixedFrameRelay(Node):
    def __init__(self):
        super().__init__('fixed_frame_relay')
        d = self.declare_parameter
        # 'in_topic' -> publishes '<in_topic>_odom' and '<in_topic>_map'
        d('topics', ['/bumper_cloud', '/ai/det/obstacle_points'])
        d('base_frame', 'base_link')
        d('odom_frame', 'odom')
        d('map_frame', 'map')
        d('max_tf_lag_s', 0.3)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.base, self.odom, self.map = p('base_frame'), p('odom_frame'), p('map_frame')
        self.max_lag_ns = int(float(p('max_tf_lag_s')) * 1e9)
        self.odom_tf = None
        self.map_tf = None
        self.static_tf = {}
        self._static_edges = {}
        self.skipped = 0
        self.create_subscription(TFMessage, '/tf', self._on_tf, 50)
        self.create_subscription(TFMessage, '/tf_static', self._on_static,
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pubs = {}
        for t in p('topics'):
            self.pubs[t] = (self.create_publisher(PointCloud2, t + '_odom', 10),
                            self.create_publisher(PointCloud2, t + '_map', 10))
            # reliable + sensor-data sources both work with a best-effort subscription
            self.create_subscription(PointCloud2, t, lambda m, t=t: self._on_cloud(t, m),
                                     qos_profile_sensor_data)
        self.get_logger().info('fixed_frame_relay: %s -> *_odom / *_map' % ', '.join(self.pubs))

    def _on_tf(self, msg):
        for t in msg.transforms:
            ns = t.header.stamp.sec * 1_000_000_000 + t.header.stamp.nanosec
            if t.header.frame_id == self.odom and t.child_frame_id == self.base:
                if self.odom_tf is None or ns > self.odom_tf[0]:
                    self.odom_tf = (ns, tf_matrix(t.transform))
            elif t.header.frame_id == self.map and t.child_frame_id == self.odom:
                if self.map_tf is None or (not self.map_tf[2] and ns > self.map_tf[0]):
                    self.map_tf = (ns, tf_matrix(t.transform), False)

    def _on_static(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == self.map and t.child_frame_id == self.odom:
                self.map_tf = (0, tf_matrix(t.transform), True)
            self._static_edges[t.child_frame_id] = (t.header.frame_id, tf_matrix(t.transform))
        # base <- frame for every static chain that reaches base_frame
        for child in list(self._static_edges):
            T, f, hops = np.eye(4), child, 0
            while f != self.base and f in self._static_edges and hops < 10:
                parent, M = self._static_edges[f]
                T, f, hops = M @ T, parent, hops + 1
            if f == self.base:
                self.static_tf[child] = T

    def _on_cloud(self, topic, msg):
        now_ns = self.get_clock().now().nanoseconds
        try:
            pts = cloud_xyz(msg)
        except Exception:  # noqa: BLE001 - unexpected layout: skip
            return
        plan = relay_plan(msg.header.frame_id, pts, now_ns, self.max_lag_ns, self.base,
                          self.odom_tf, self.map_tf, self.static_tf)
        if not plan:
            self.skipped += 1
            self.get_logger().warn('fixed_frame_relay: no fresh %s<-%s TF for %s (skipped %d)'
                                   % (self.odom, msg.header.frame_id, topic, self.skipped),
                                   throttle_duration_sec=10.0)
            return
        pub_odom, pub_map = self.pubs[topic]
        for frame, ns, p in plan:
            (pub_odom if frame == 'odom' else pub_map).publish(
                make_cloud(rclpy.time.Time(nanoseconds=ns).to_msg(),
                           self.odom if frame == 'odom' else self.map, p))


def main(args=None):
    rclpy.init(args=args)
    node = FixedFrameRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
