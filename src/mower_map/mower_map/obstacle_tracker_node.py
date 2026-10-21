# SPDX-License-Identifier: GPL-3.0-or-later
"""obstacle_tracker_node - auto-mark reliably detected obstacles into the map.

The Airseekers Tron marks live sensor obstacles in the Nav2 costmaps (stereo
depth, ``det_range`` YOLO detections, bumper) and the controller deviates
around them, but those marks are transient: they decay and are raytraced away.
This node closes that gap for *static* obstacles (the kids' toys, balls, tools
left on the lawn): it clusters the ground-removed stereo depth cloud plus the
YOLO-ranged detections, tracks the clusters across frames and, once one is
reliably stable, asks ``/map_server_node/promote_obstacle`` to turn it into a
PENDING keepout polygon. The keepout is rasterised into ``/nav_keepout_mask``
(which the Nav2 global costmap consumes via the keepout filter) for this session
only; the operator accepts it (then it is persisted to ``areas.dat``) or
discards it (review 2026-10-13, B2).

Symmetrically, when the object is gone the node removes it again: if the depth
camera positively sees bare ground in the footprint (``/stereo_depth/clear_points_map``)
or the robot drives up to the spot and finds no obstacle point there, the
keepout is dropped via ``/map_server_node/remove_obstacle``.

Algorithm (mower_map.obstacle_tracker_core, ROS-free and unit tested):

    cluster  ->  track (associate within association_dist_m)
             ->  promote gate: min_observations, persistence_age_s (minutes,
                 still in view), radius <= 1 m, centroid inside a mowing area,
                 no ranged person / animal detection near it, mowing
             ->  remove gate: not a physical point anymore, verified
                 clear/absent for gone_confirmations frames, >= min_lifetime_s

Safety defaults (owner 2026-10-13: "auto obstacle marking should be false by
default as it's not ready yet"): ``auto_promote`` is off, so the node only
tracks and publishes candidates; when switched on, promotions are pending
proposals (never written straight into areas.dat), only while mowing. Only
``SOURCE_TRACKER`` keepouts can be removed, so hand-drawn keepouts are never
touched; stored ``SOURCE_TRACKER`` keepouts are re-adopted at startup so an
accepted one can still be removed once the object is gone. See
config/obstacle_tracker.yaml.

In:  ``obstacle_points``  (default /stereo_depth/points_map, + det_range points)
     ``clear_points``     (/stereo_depth/clear_points_map)
     ``odom_topic``       (/odometry/filtered_map)
     ``replan_topic``     (/map_server_node/replan_needed)
     ``high_level_status_topic`` (/behavior_tree_node/high_level_status, for only_while_mowing)
     ``snapshot_topic``   (/vio/right/image_annotated, the det_ros front overlay)
     ``dynamic_detections_topic`` (/ai/det/detections_ranged, person/animal veto)
     ``~/get_areas_service`` (/map_server_node/get_mowing_area)
Out: /obstacle_tracker/obstacles  (mowgli_interfaces/ObstacleArray, for map_server + GUI)
     ~/markers                   (visualization_msgs/MarkerArray)
     <snapshot_dir>/<id>.jpg    (whole annotated front frame per obstacle, for the GUI)
Srv: /map_server_node/promote_obstacle, /map_server_node/remove_obstacle
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Point, Point32
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

from mowgli_interfaces.msg import (HighLevelStatus, MapObstacleInfo, ObstacleArray,
                                   TrackedObstacle)
from mowgli_interfaces.srv import GetMowingArea, PromoteObstacle, RemoveObstacle

from mower_map.obstacle_snapshot import prune_snapshots, snapshot_ids, sync_snapshots
from mower_map.obstacle_tracker_core import TrackerCore, TrackerParams, keepout_entry
from mower_map.sub_pump import node_clock

try:  # det_ros / det_range output; without vision_msgs the dynamic-class veto is off
    from vision_msgs.msg import Detection2DArray
except ImportError:  # pragma: no cover - always present on the robot image
    Detection2DArray = None

# det_ros/labels.py BEST_LARGE_CLASSES (best_large_0208.rknn export order), as in
# mower_vision.guard_logic.DEFAULT_CLASSES: maps a bare integer class_id to a name.
DET_CLASSES = [
    'person', 'dog', 'cat', 'sports ball', 'hedgehog', 'rabbit', 'stone', 'hoe',
    'shovel', 'manhole', 'brick', 'trashbin', 'toycars', 'potted plant', 'cans',
    'bottle', 'book', 'backpack', 'wood', 'chair', 'trunk', 'dock',
]
# Classes that move by themselves (guard_logic DEFAULT_WHITELIST: the living things the
# model knows). A cluster near one of these is never promoted into a keepout.
DYNAMIC_CLASSES = ['person', 'dog', 'cat', 'hedgehog', 'rabbit']

PARAMS = {
    'enabled': True,
    'obstacle_points_topic': '/stereo_depth/points_map',
    'extra_obstacle_points_topic': '/ai/det/obstacle_points_map',  # YOLO ranged, map frame (may be '')
    'clear_points_topic': '/stereo_depth/clear_points_map',         # may be ''
    'odom_topic': '/odometry/filtered_map',
    'replan_topic': '/map_server_node/replan_needed',
    'high_level_status_topic': '/behavior_tree_node/high_level_status',
    'obstacles_topic': '/obstacle_tracker/obstacles',
    'markers_topic': '~/markers',
    'cloud_max_age_s': 1.5,        # drop a cloud older than this (monotonic)
    'high_level_fresh_s': 5.0,     # high-level status older than this = unknown
    'only_while_mowing': True,     # only promote while HIGH_LEVEL_STATE_AUTONOMOUS
    'step_rate': 1.0,              # tracker update / decision rate (Hz)
    'publish_markers': True,
    # --- clustering / tracking ---
    'resolution': 0.1,
    'merge_cells': 2,
    'association_dist_m': 0.5,
    'persistence_age_s': 120.0,    # B2: minutes, never a person standing still
    'min_observations': 10,
    'min_obstacle_radius_m': 0.05,
    'max_obstacle_radius_m': 1.0,
    'det_radius_m': 0.15,
    'polygon_inflation_m': 0.05,
    'polygon_steps': 12,
    'require_inside_mowing_area': True,
    'max_tracks': 200,
    # --- auto promotion / removal ---
    'auto_promote': False,         # owner 2026-10-13: not ready -> off by default
    'auto_remove': True,
    'min_lifetime_s': 5.0,
    'gone_confirmations': 4,
    'verify_distance_m': 3.0,
    'verify_fov_deg': 60.0,
    'present_radius_m': 0.4,
    'clear_radius_m': 0.4,
    'transient_timeout_s': 30.0,
    'max_track_drift_m': 0.4,
    # --- dynamic-class veto (B2): people / animals are never promoted ---
    # Ranged det_range detections (base_link positions) of these classes, put into the map
    # frame with the latest odometry; a track within dynamic_veto_radius_m of one is never
    # promoted. An unranged one blocks every promotion for dynamic_memory_s. '' = off.
    'dynamic_detections_topic': '/ai/det/detections_ranged',
    'dynamic_classes': list(DYNAMIC_CLASSES),
    'det_classes': list(DET_CLASSES),
    'dynamic_min_score': 0.3,
    'dynamic_memory_s': 10.0,
    'dynamic_veto_radius_m': 1.0,
    # --- map_server plumbing ---
    'promote_service': '/map_server_node/promote_obstacle',
    'remove_service': '/map_server_node/remove_obstacle',
    'get_areas_service': '/map_server_node/get_mowing_area',
    'obstacle_name': 'auto: obstacle',
    'areas_refresh_s': 5.0,
    # --- GUI settings (mowgli_robot.yaml; launch/robot_settings.py BINDINGS) ---
    # The GUI-bound key obstacle_auto_promote overrides auto_promote at startup;
    # '' disables reading (package default stands). The GUI also pushes it live.
    'robot_yaml_path': '',
    # --- obstacle snapshots (whole annotated front frame, shown in the GUI) ---
    # When a new cluster appears the node saves the latest annotated front camera
    # frame to ``<snapshot_dir>/<id>.jpg``; the GUI serves it next to the obstacle
    # in the tracked-obstacles panel. Empty topic = off (e.g. sim, no det_ros).
    'snapshot_enabled': True,
    'snapshot_topic': '/vio/right/image_annotated',
    'snapshot_dir': '/ros2_ws/maps/obstacle_snapshots',
    'snapshot_quality': 80,
    'snapshot_max_age_s': 1.0,   # ignore an annotated frame older than this
}

# TrackerParams keys are a subset of PARAMS; keep the mapping explicit so a
# typo in one place cannot silently keep a default.
_CORE_KEYS = (
    'resolution', 'merge_cells', 'association_dist_m', 'persistence_age_s',
    'min_observations', 'min_obstacle_radius_m', 'max_obstacle_radius_m',
    'det_radius_m', 'polygon_inflation_m', 'polygon_steps',
    'require_inside_mowing_area', 'max_tracks', 'auto_promote', 'auto_remove',
    'min_lifetime_s', 'gone_confirmations', 'verify_distance_m', 'verify_fov_deg',
    'present_radius_m', 'clear_radius_m', 'transient_timeout_s',
    'max_track_drift_m', 'dynamic_veto_radius_m',
)


def cloud_xy(msg: PointCloud2) -> np.ndarray:
    """Extract an (N, 2) map-frame x/y array from a PointCloud2 (dense or
    organized). Non-finite points are dropped. Fast: a single structured view
    over the raw buffer, no message-field iterator."""
    n = int(msg.width) * int(msg.height)
    if n <= 0 or not msg.data:
        return np.empty((0, 2), dtype=float)
    offs = {f.name: int(f.offset) for f in msg.fields if f.name in ('x', 'y')}
    if 'x' not in offs or 'y' not in offs:
        return np.empty((0, 2), dtype=float)
    step = int(msg.point_step)
    w = int(msg.width)
    row_step = int(msg.row_step) or step * w
    buf = np.frombuffer(memoryview(msg.data), dtype=np.uint8)
    if row_step == step * w:
        raw = buf[:n * step].reshape(n, step)
    else:
        raw = np.concatenate([buf[r * row_step:r * row_step + w * step].reshape(w, step)
                              for r in range(int(msg.height))], axis=0)
    x = raw[:, offs['x']:offs['x'] + 4].copy().view('<f4').reshape(-1).astype(float)
    y = raw[:, offs['y']:offs['y'] + 4].copy().view('<f4').reshape(-1).astype(float)
    out = np.column_stack((x, y))
    return out[np.isfinite(out).all(axis=1)]


def dynamic_label(class_id, classes) -> str:
    """det_ros publishes class names; a bare integer index maps through ``classes``."""
    cid = str(class_id).strip()
    if cid.isdigit():
        idx = int(cid)
        if 0 <= idx < len(classes):
            return str(classes[idx]).strip().lower()
    return cid.lower()


def tracker_keepouts(areas_by_index):
    """``{area_index: MapArea}`` -> ``[(area_index, cx, cy, radius, wanted_id)]`` of the
    stored SOURCE_TRACKER keepouts (pending or accepted) of the mowing areas."""
    out = []
    for i, a in sorted(areas_by_index.items()):
        if a.is_navigation_area:
            continue
        for j, poly in enumerate(a.obstacles):
            info = a.obstacle_info[j] if j < len(a.obstacle_info) else None
            if info is None or int(info.source) != MapObstacleInfo.SOURCE_TRACKER:
                continue
            entry = keepout_entry(i, [(p.x, p.y) for p in poly.points], info.name)
            if entry is not None:
                out.append(entry)
    return out


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ObstacleTrackerNode(Node):

    def __init__(self):
        super().__init__('obstacle_tracker')
        self._clk = node_clock(self)   # monotonic / sim time (use_sim_time)
        for name, default in PARAMS.items():
            self.declare_parameter(name, default)

        # GUI-bound settings (mowgli_robot.yaml) win over the declared defaults.
        # Read before the core is built so its startup value is right; the GUI
        # also pushes the same parameter live (see _on_set_parameters).
        self._apply_robot_yaml_settings()

        self.core = TrackerCore(TrackerParams(**{k: self.p(k) for k in _CORE_KEYS}))
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # latest sensor state (monotonic receive time)
        self._obstacle_pts = np.empty((0, 2))
        self._obstacle_time = -1e9
        self._extra_pts = np.empty((0, 2))
        self._extra_time = -1e9
        self._clear_pts = np.empty((0, 2))
        self._clear_time = -1e9
        self._robot_pose = None            # (x, y, yaw)
        self._high_state = None            # HighLevelStatus.state
        self._high_time = -1e9
        self._dynamic = []                 # [(monotonic, x, y)] ranged person/animal, map
        self._dynamic_unranged_time = -1e9

        # mowing areas: (area_index, name, polygon)
        self._areas = ()
        self._areas_time = -1e9
        self._areas_inflight = False
        self._areas_acc = {}

        qos = qos_profile_sensor_data
        self.create_subscription(PointCloud2, self.p('obstacle_points_topic'),
                                 self._on_obstacle_cloud, qos)
        if self.p('extra_obstacle_points_topic'):
            self.create_subscription(PointCloud2, self.p('extra_obstacle_points_topic'),
                                     self._on_extra_cloud, qos)
        if self.p('clear_points_topic'):
            self.create_subscription(PointCloud2, self.p('clear_points_topic'),
                                     self._on_clear_cloud, qos)
        self.create_subscription(Odometry, self.p('odom_topic'), self._on_odom, 10)
        self.create_subscription(Bool, self.p('replan_topic'), self._on_replan, 10)
        self.create_subscription(HighLevelStatus, self.p('high_level_status_topic'),
                                 self._on_high_level, 10)
        if self.p('dynamic_detections_topic') and Detection2DArray is not None:
            self.create_subscription(Detection2DArray, self.p('dynamic_detections_topic'),
                                     self._on_detections, 10)
        elif self.p('dynamic_detections_topic'):
            self.get_logger().warn('vision_msgs missing: dynamic-class promotion veto off')

        # Per-obstacle snapshots: keep the latest whole annotated front frame
        # (raw buffer only) and JPEG-encode it once per new cluster.
        self._snapshot_ids = set()        # ids that already have a file on disk
        self._snapshot_frame = None       # (monotonic, bytes, encoding, w, h, step)
        self._snapshot_enabled = (bool(self.p('snapshot_enabled'))
                                  and bool(self.p('snapshot_topic')))
        # Snapshots of keepouts accepted before a restart are kept (B2): new ids start
        # above every file on disk; files without a re-adopted track are pruned once the
        # first area fetch has re-adopted the stored SOURCE_TRACKER keepouts.
        self._snapshots_pruned = False
        if self._snapshot_enabled:
            on_disk = snapshot_ids(self.p('snapshot_dir'))
            self.core.reserve_ids(max(on_disk) + 1 if on_disk else 1)
            self.create_subscription(Image, self.p('snapshot_topic'),
                                     self._on_snapshot_frame, qos)
            self.get_logger().info(
                'obstacle snapshots: %s -> %s (%d kept from before)'
                % (self.p('snapshot_topic'), self.p('snapshot_dir'), len(on_disk)))

        self._obs_pub = self.create_publisher(ObstacleArray, self.p('obstacles_topic'), 1)
        self._marker_pub = self.create_publisher(MarkerArray, self.p('markers_topic'), 1)

        self._promote_cli = self.create_client(PromoteObstacle, self.p('promote_service'))
        self._remove_cli = self.create_client(RemoveObstacle, self.p('remove_service'))
        self._areas_cli = self.create_client(GetMowingArea, self.p('get_areas_service'))

        self.create_timer(1.0 / max(0.1, float(self.p('step_rate'))), self._on_step)
        self._replan_refresh_cooldown = 0.0
        self.get_logger().info(
            'obstacle_tracker up: promote=%s (pending, only_while_mowing=%s) remove=%s, '
            'obstacle cloud %s, clear %s, dynamic veto %s'
            % (self.p('auto_promote'), self.p('only_while_mowing'), self.p('auto_remove'),
               self.p('obstacle_points_topic'), self.p('clear_points_topic') or 'off',
               self.p('dynamic_detections_topic') or 'off'))

    # ------------------------------------------------------------------
    def p(self, name):
        return self.get_parameter(name).value

    def _apply_robot_yaml_settings(self):
        """Apply GUI-bound keys from ``robot_yaml_path`` over the defaults.

        Only the keys the GUI binds to this node are honoured (BINDINGS in
        launch/robot_settings.py): an unknown or ill-typed value is ignored,
        leaving the package default.
        """
        path = str(self.p('robot_yaml_path') or '')
        if not path:
            return
        from mower_map.area_settings import load_robot_yaml_flat
        value = load_robot_yaml_flat(path).get('obstacle_auto_promote')
        if isinstance(value, bool):
            self.set_parameters([Parameter('auto_promote', value=value)])
            self.get_logger().info('robot settings: auto_promote=%s (from %s)'
                                   % (value, path))

    def _on_set_parameters(self, params):
        """Mirror a live parameter set onto the core's TrackerParams.

        The core is built once from the node parameters, so a GUI push (or a
        ``ros2 param set``) must update the stored copy for the tracker
        tunables to take effect on the next step. Returns success so a bad
        value is not rejected (the core clamps/uses it as given).
        """
        for prm in params:
            if prm.name in _CORE_KEYS and hasattr(self.core.params, prm.name):
                setattr(self.core.params, prm.name, prm.value)
        return SetParametersResult(successful=True)

    # -- subscriptions --------------------------------------------------
    def _on_obstacle_cloud(self, msg):
        self._obstacle_pts = cloud_xy(msg)
        self._obstacle_time = self._clk.now()

    def _on_extra_cloud(self, msg):
        self._extra_pts = cloud_xy(msg)
        self._extra_time = self._clk.now()

    def _on_clear_cloud(self, msg):
        self._clear_pts = cloud_xy(msg)
        self._clear_time = self._clk.now()

    def _on_odom(self, msg):
        pose = msg.pose.pose
        self._robot_pose = (pose.position.x, pose.position.y,
                            yaw_from_quaternion(pose.orientation))

    def _on_replan(self, msg):
        # Areas/masks changed (area edit, our own promote/remove, dock move):
        # refresh the mowing-area polygons so promotion targets stay correct.
        if msg.data and self._clk.now() - self._replan_refresh_cooldown > 2.0:
            self._replan_refresh_cooldown = self._clk.now()
            self._areas_time = -1e9

    def _on_high_level(self, msg):
        self._high_state = int(msg.state)
        self._high_time = self._clk.now()

    def _on_detections(self, msg):
        """Ranged person / animal detections -> map-frame veto points. det_range tags a
        ranged detection with its base_link position in results[0].pose (covariance[0]
        > 0); an unranged one blocks every promotion for dynamic_memory_s."""
        now = self._clk.now()
        dyn = {str(c).strip().lower() for c in (self.p('dynamic_classes') or [])}
        classes = list(self.p('det_classes') or [])
        min_score = float(self.p('dynamic_min_score'))
        for det in msg.detections:
            if not det.results:
                continue
            best = max(det.results, key=lambda r: r.hypothesis.score)
            if float(best.hypothesis.score) < min_score:
                continue
            if dynamic_label(best.hypothesis.class_id, classes) not in dyn:
                continue
            pose = det.results[0].pose
            ranged = len(pose.covariance) > 0 and pose.covariance[0] > 0.0
            if not ranged or self._robot_pose is None:
                self._dynamic_unranged_time = now
                continue
            rx, ry, ryaw = self._robot_pose
            bx, by = float(pose.pose.position.x), float(pose.pose.position.y)
            c, s = math.cos(ryaw), math.sin(ryaw)
            self._dynamic.append((now, rx + c * bx - s * by, ry + s * bx + c * by))

    def _dynamic_points(self, now):
        keep = float(self.p('dynamic_memory_s'))
        self._dynamic = [d for d in self._dynamic if now - d[0] <= keep][-200:]
        if not self._dynamic:
            return np.empty((0, 2))
        return np.array([(x, y) for _, x, y in self._dynamic], dtype=float)

    def _on_snapshot_frame(self, msg):
        # Copy the raw buffer; the JPEG is encoded lazily (once per new cluster)
        # so the idle annotated feed only costs this one copy per frame.
        self._snapshot_frame = (self._clk.now(), bytes(msg.data), msg.encoding,
                                int(msg.width), int(msg.height), int(msg.step))

    def _promotion_allowed(self, now):
        if now - self._dynamic_unranged_time <= float(self.p('dynamic_memory_s')):
            return False   # a person / animal in view we could not place: hold off
        if not bool(self.p('only_while_mowing')):
            return True
        if now - self._high_time > float(self.p('high_level_fresh_s')):
            return False   # status unknown -> don't write to the map
        return self._high_state in (HighLevelStatus.HIGH_LEVEL_STATE_AUTONOMOUS,
                                    HighLevelStatus.HIGH_LEVEL_STATE_MANUAL_MOWING)

    # -- area fetch (async chain) ---------------------------------------
    def _refresh_areas(self):
        if self._areas_inflight or not self._areas_cli.service_is_ready():
            return
        self._areas_inflight = True
        self._areas_acc = {}
        self._request_area(0)

    def _request_area(self, idx):
        if idx > 500:  # paranoia: map_server keeps answering success
            self._finish_areas()
            return
        req = GetMowingArea.Request()
        req.index = int(idx)
        fut = self._areas_cli.call_async(req)
        fut.add_done_callback(lambda f, i=idx: self._on_area_response(f, i))

    def _on_area_response(self, fut, idx):
        try:
            resp = fut.result()
        except Exception as exc:  # noqa: BLE001 - rclpy future
            self.get_logger().warn('get_mowing_area(%d) failed: %s' % (idx, exc))
            self._areas = ()
            self._areas_inflight = False
            return
        if resp is None or not resp.success:
            self._finish_areas()
            self.get_logger().debug('mowing areas: %s'
                                    % ', '.join('%d:%s' % (i, n) for i, n, _ in self._areas))
            return
        self._areas_acc[idx] = resp.area
        self._request_area(idx + 1)

    def _finish_areas(self):
        self._areas = tuple(sorted(
            (i, a.name, [(pt.x, pt.y) for pt in a.area.points])
            for i, a in self._areas_acc.items() if not a.is_navigation_area))
        self._areas_time = self._clk.now()
        self._areas_inflight = False
        # B2: re-adopt stored SOURCE_TRACKER keepouts (accepted before a restart, or a
        # pending one after a tracker respawn) so they can still be auto-removed.
        keepouts = tracker_keepouts(self._areas_acc)
        adopted = self.core.adopt_keepouts(keepouts, self._clk.now())
        if adopted:
            self.get_logger().info('re-adopted %d auto-marked keepout(s): #%s'
                                   % (len(adopted), ', #'.join(str(i) for i in adopted)))
        if self._snapshot_enabled and not self._snapshots_pruned:
            live = {t.id for t in self.core.tracks}
            on_disk = snapshot_ids(self.p('snapshot_dir'))
            self._snapshot_ids |= on_disk & live      # kept: no re-capture
            removed = prune_snapshots(self.p('snapshot_dir'), live)
            self._snapshots_pruned = True
            self.get_logger().info('obstacle snapshots: kept %d, pruned %d stale'
                                   % (len(on_disk & live), removed))

    # -- service calls --------------------------------------------------
    def _send_promote(self, track_id):
        t = self.core.find_track(track_id)
        if t is None or t.area_index < 0:
            return
        if not self._promote_cli.service_is_ready():
            self.core.reject_promotion(track_id)   # retry once map_server is up
            return
        # No polygon -> map_server uses the tracker-snapshot lookup path, which
        # stamps source=SOURCE_TRACKER (needed so ~/remove_obstacle may undo it).
        req = PromoteObstacle.Request()
        req.area_index = int(t.area_index)
        req.obstacle_id = int(t.id)
        # '#<id>' lets a restart re-adopt the keepout under the same id (its snapshot)
        req.name = '%s #%d' % (self.p('obstacle_name'), t.id)
        fut = self._promote_cli.call_async(req)
        fut.add_done_callback(lambda f, tid=track_id: self._on_promote(f, tid))

    def _on_promote(self, fut, track_id):
        t = self.core.find_track(track_id)
        try:
            resp = fut.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('promote_obstacle(#%d) failed: %s' % (track_id, exc))
            self.core.reject_promotion(track_id)
            return
        if resp is not None and resp.success:
            self.core.confirm_promoted(track_id, self._clk.now())
            self.get_logger().info('auto-proposed obstacle #%d (pending review): %s'
                                   % (track_id, resp.message))
        else:
            # e.g. map_server had not yet seen the snapshot, or centroid now in
            # a nav area: un-flag so the next step retries.
            self.core.reject_promotion(track_id)
            if t is not None:
                self.get_logger().debug('promote #%d deferred: %s'
                                        % (track_id, resp.message if resp else 'no response'))

    def _send_remove(self, track_id):
        t = self.core.find_track(track_id)
        if t is None:
            return
        if not self._remove_cli.service_is_ready():
            self.core.reject_removal(track_id)   # retry once map_server is up
            return
        req = RemoveObstacle.Request()
        req.centroid.x = float(t.cx)
        req.centroid.y = float(t.cy)
        req.obstacle_id = 0
        req.max_distance_m = max(float(self.p('association_dist_m')),
                                 float(t.footprint_radius(self.core.params)))
        fut = self._remove_cli.call_async(req)
        fut.add_done_callback(lambda f, tid=track_id: self._on_remove(f, tid))

    def _on_remove(self, fut, track_id):
        try:
            resp = fut.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('remove_obstacle(#%d) failed: %s' % (track_id, exc))
            self.core.reject_removal(track_id)   # keep the track; retry next step
            return
        if resp is not None and resp.success:
            self.get_logger().info('auto-removed obstacle #%d: %s' % (track_id, resp.message))
            self.core.confirm_removed(track_id)
        else:
            # No matching keepout (already gone, e.g. after a restart, or it was
            # never persisted): drop the track so it re-forms from fresh evidence.
            self.core.confirm_removed(track_id)

    # -- main step ------------------------------------------------------
    def _on_step(self):
        if not bool(self.p('enabled')):
            return
        now = self._clk.now()
        if now - self._areas_time > float(self.p('areas_refresh_s')):
            self._refresh_areas()

        age_max = float(self.p('cloud_max_age_s'))
        obstacle_fresh = (now - self._obstacle_time <= age_max
                          or now - self._extra_time <= age_max)
        merged = [self._obstacle_pts] if now - self._obstacle_time <= age_max else []
        if now - self._extra_time <= age_max:
            merged.append(self._extra_pts)
        obstacle_pts = np.vstack(merged) if merged else np.empty((0, 2))
        clear_fresh = now - self._clear_time <= age_max
        clear_pts = self._clear_pts if clear_fresh else np.empty((0, 2))

        result = self.core.step(obstacle_pts, now, self._robot_pose, clear_pts, self._areas,
                                obstacle_fresh=obstacle_fresh, clear_fresh=clear_fresh,
                                allow_promote=self._promotion_allowed(now),
                                dynamic_points=self._dynamic_points(now))

        self._publish_tracks(now)
        self._update_snapshots(now)
        for tid in result['promote']:
            self._send_promote(tid)
        for tid in result['remove']:
            self._send_remove(tid)

    # -- snapshots ------------------------------------------------------
    def _update_snapshots(self, now):
        """Save the annotated front frame for each newly seen obstacle and drop
        the file once its track is gone. The frame is captured the first step a
        cluster appears (when the object is still in the front camera's view)."""
        if not self._snapshot_enabled:
            return
        written = sync_snapshots(self.p('snapshot_dir'),
                                 {t.id for t in self.core.tracks},
                                 self._snapshot_ids, self._snapshot_frame, now,
                                 max_age_s=float(self.p('snapshot_max_age_s')),
                                 quality=int(self.p('snapshot_quality')))
        for tid in written:
            self.get_logger().info('saved obstacle #%d snapshot' % tid)

    # -- publishing -----------------------------------------------------
    def _publish_tracks(self, now):
        arr = ObstacleArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = 'map'
        markers = MarkerArray()
        params = self.core.params
        for i, t in enumerate(self.core.tracks):
            poly = t.footprint(params)
            o = TrackedObstacle()
            o.id = int(t.id)
            o.polygon.points = [Point32(x=float(x), y=float(y)) for x, y in poly]
            o.centroid = Point(x=float(t.cx), y=float(t.cy), z=0.0)
            o.radius = float(t.footprint_radius(params))
            o.first_seen = (self.get_clock().now()
                            - Duration(seconds=now - t.first_seen)).to_msg()
            o.observation_count = int(t.observation_count)
            o.status = TrackedObstacle.PERSISTENT if t.promoted else TrackedObstacle.TRANSIENT
            arr.obstacles.append(o)
            if self.p('publish_markers'):
                markers.markers.append(self._track_marker(i, t, poly))
        self._obs_pub.publish(arr)
        if self.p('publish_markers'):
            self._marker_pub.publish(markers)

    def _track_marker(self, idx, t, poly):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'obstacle_tracker'
        m.id = idx
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.03
        if t.promoted:
            m.color.r, m.color.g, m.color.b = 0.1, 0.4, 1.0
        else:
            m.color.r, m.color.g, m.color.b = 1.0, 0.6, 0.0
        m.color.a = 0.9
        m.pose.orientation.w = 1.0
        pts = list(poly) + [poly[0]]
        for x, y in pts:
            m.points.append(Point(x=float(x), y=float(y), z=0.05))
        return m


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleTrackerNode()
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
