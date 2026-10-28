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
Class-voted no-go zones (option A, 2026-10-23 trunk incident; obstacle_tracker_core): ranged
static-class detections (trunk, stone, wood, ...) with class, score and the det_range width
(covariance[35]), on the planned path ahead (``plan_topics``: the coverage plan from the robot
on + the current route; corridor or pivot sweep, mower_map.nogo_zones) or hit by the bumper
(``bumper_topic``), become PENDING keepouts within seconds, named ``auto: <class> #<id>``.
Not in areas with ``obstacle_detection: none`` (``area_settings_topic``). The GUI's Ignore
(``~/clear_obstacle``) drops one and suppresses its spot; Accept persists it via map_server
(source SOURCE_TRACKER = auto). The zone set + recent static marks go out as JSON on
``zones_topic`` for the mission's pre-pivot check (nogo_zones.pivot_sweep_hits).

Out: /obstacle_tracker/obstacles  (mowgli_interfaces/ObstacleArray, for map_server + GUI)
     /obstacle_tracker/nogo_zones (std_msgs/String JSON, nogo_zones.ZoneSet)
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
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from mowgli_interfaces.msg import (HighLevelStatus, MapObstacleInfo, ObstacleArray,
                                   TrackedObstacle)
from mowgli_interfaces.srv import ClearObstacle, GetMowingArea, PromoteObstacle, RemoveObstacle

from mower_map.obstacle_snapshot import prune_snapshots, snapshot_ids, sync_snapshots
from mower_map.areas import point_in_polygon
from mower_map.nogo_zones import PathCursor, Zone, ZoneSet
from mower_map.obstacle_tracker_core import TrackerCore, TrackerParams, keepout_entry
from mower_common.sub_pump import node_clock

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
    'living_timeout_s': 10.0,
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
    # --- class-voted no-go zones (option A, 2026-10-23; owner to review the defaults) ---
    'class_zones': True,
    'det_min_score': 0.6,
    'det_min_frames': 5,
    'det_window_s': 10.0,
    'det_assoc_m': 0.3,
    'det_vote_min_score': 0.3,
    'static_classes': ['trunk', 'stone', 'wood', 'brick', 'trashbin', 'potted plant',
                       'chair'],
    'never_classes': list(DYNAMIC_CLASSES),
    'class_prior_names': ['trunk', 'stone', 'wood', 'brick', 'trashbin', 'potted plant',
                          'chair'],
    'class_prior_diameters_m': [0.25, 0.15, 0.10, 0.20, 0.45, 0.30, 0.50],
    'default_prior_m': 0.20,
    'size_margin_m': 0.10,
    'max_zone_radius_m': 1.0,
    'resize_step_m': 0.05,
    'require_on_path': True,
    'onpath_pivot_min_turn_deg': 30.0,
    'bumper_evidence_m': 0.5,
    'bumper_zone_hits': 0,
    'bumper_zone_radius_m': 0.25,
    'pending_expire_s': 3600.0,
    'det_gone_confirmations': 8,
    'det_verify_min_m': 0.8,
    'det_verify_fov_deg': 30.0,
    'camera_x_m': 0.466,              # front stereo camera ahead of base_link (URDF)
    'merge_dist_m': 0.3,
    'ignore_radius_m': 0.5,
    # inputs / outputs of the zones
    'plan_topics': ['/coverage/full_plan', '/plan'],   # first = the long coverage plan
    'plan_max_age_s': 600.0,          # a route (/plan) older than this is not "ahead"
    'plan_back_m': 2.0,               # the coverage plan counts from this far behind the robot
    'bumper_topic': '/bumper_cloud_map',
    'bumper_debounce_s': 1.0,
    'area_settings_topic': '/map_server_node/area_settings',
    'zones_topic': '/obstacle_tracker/nogo_zones',
    'mark_memory_s': 60.0,            # static marks in the zone set (= the costmap layer decay)
    'mark_min_score': 0.5,            # = det_range points_min_score
    'mark_margin_m': 0.10,            # = det_range width_margin_m
    'discard_service': '/map_server_node/discard_obstacle',
    # option C (2026-10-23): pending zones hold for the current mow only; when the mission
    # reaches one of these states (HighLevelStatus.state_name) they are dropped (track and
    # pending keepout) unless the operator accepted them
    'drop_pending_at_session_end': True,
    'session_end_states': ['MOWING_COMPLETE', 'MOWING_INCOMPLETE'],
}

# TrackerParams keys are a subset of PARAMS; keep the mapping explicit so a
# typo in one place cannot silently keep a default.
_CORE_KEYS = (
    'resolution', 'merge_cells', 'association_dist_m', 'persistence_age_s',
    'min_observations', 'min_obstacle_radius_m', 'max_obstacle_radius_m',
    'det_radius_m', 'polygon_inflation_m', 'polygon_steps',
    'require_inside_mowing_area', 'max_tracks', 'auto_promote', 'auto_remove',
    'min_lifetime_s', 'gone_confirmations', 'verify_distance_m', 'verify_fov_deg',
    'present_radius_m', 'clear_radius_m', 'transient_timeout_s', 'living_timeout_s',
    'max_track_drift_m', 'dynamic_veto_radius_m',
    'class_zones', 'det_min_score', 'det_min_frames', 'det_window_s', 'det_assoc_m',
    'det_vote_min_score', 'default_prior_m', 'size_margin_m', 'max_zone_radius_m',
    'resize_step_m', 'require_on_path', 'onpath_pivot_min_turn_deg', 'bumper_evidence_m',
    'bumper_zone_hits', 'bumper_zone_radius_m', 'pending_expire_s', 'det_gone_confirmations',
    'det_verify_min_m', 'det_verify_fov_deg', 'merge_dist_m', 'ignore_radius_m', 'camera_x_m',
)
# list-valued core params (converted to tuples)
_CORE_LIST_KEYS = ('static_classes', 'never_classes')


def core_params(get) -> TrackerParams:
    """TrackerParams from a parameter getter (``get(name)``)."""
    kw = {k: get(k) for k in _CORE_KEYS}
    for k in _CORE_LIST_KEYS:
        kw[k] = tuple(str(c).strip().lower() for c in (get(k) or []) if str(c).strip())
    names = [str(c).strip().lower() for c in (get('class_prior_names') or [])]
    dia = [float(v) for v in (get('class_prior_diameters_m') or [])]
    kw['class_prior_m'] = tuple(zip(names, dia))
    return TrackerParams(**kw)


def blocked_areas(settings_json: str, areas) -> set:
    """Indices of the ``areas`` ``(index, name, polygon)`` whose effective
    ``obstacle_detection`` (map_server ~/area_settings snapshot) is ``none``."""
    import json
    try:
        doc = json.loads(settings_json or '{}')
    except ValueError:
        return set()
    default = str((doc.get('defaults') or {}).get('obstacle_detection', 'standard'))
    per = doc.get('areas') or {}
    out = set()
    for (i, name, _poly) in areas:
        level = str((per.get(name) or {}).get('obstacle_detection', default))
        if level.strip().lower() == 'none':
            out.add(int(i))
    return out


def det_observation(det, robot_pose, classes, camera_x=0.466):
    """A det_range ranged detection -> ``(x, y, label, score, width_m)`` in the map frame
    (robot pose (x, y, yaw) of base_link), or None when it is not ranged. det_range ranges
    the FRONT face; with a known width the point is moved half a width further along the
    camera ray, to the object's centre (as det_range's costmap disc)."""
    if not det.results:
        return None
    best = max(det.results, key=lambda r: r.hypothesis.score)
    pose = det.results[0].pose
    cov = pose.covariance
    if robot_pose is None or len(cov) == 0 or cov[0] <= 0.0:
        return None
    rx, ry, ryaw = robot_pose
    bx, by = float(pose.pose.position.x), float(pose.pose.position.y)
    c, s = math.cos(ryaw), math.sin(ryaw)
    width = float(cov[35]) if len(cov) > 35 else 0.0
    if width > 0.0:
        vx, vy = bx - camera_x, by
        n = math.hypot(vx, vy)
        if n > 1e-6:
            bx, by = bx + vx / n * width / 2.0, by + vy / n * width / 2.0
    return (rx + c * bx - s * by, ry + s * bx + c * by,
            dynamic_label(best.hypothesis.class_id, classes),
            float(best.hypothesis.score), max(0.0, width))


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
                out.append(entry + (bool(info.pending),))
    return out


def _latched_qos():
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=ReliabilityPolicy.RELIABLE)


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

        self.core = TrackerCore(core_params(self.p))
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
        self._high_name = ''               # HighLevelStatus.state_name (mission phase)
        self._high_time = -1e9
        self._dynamic = []                 # [(monotonic, x, y)] ranged person/animal, map
        self._dynamic_unranged_time = -1e9
        # class-voted zones
        self._det_buf = []                 # map-frame static/named observations since last step
        self._marks = []                   # [(monotonic, x, y, r)] static-class det discs
        self._plans = {}                   # topic -> (monotonic, (N, 2) array)
        self._cursor = PathCursor(back_m=float(self.p('plan_back_m')))
        self._bumper_hits = []
        self._bumper_time = -1e9
        self._settings_json = ''
        self._area_blocked = set()
        self._blocked_logged = None

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
        topics = [str(t) for t in (self.p('plan_topics') or []) if str(t)]
        for i, topic in enumerate(topics):
            # the coverage plan is latched; /plan (route, Nav2) is volatile: a latched
            # subscription would be QoS-incompatible with it
            self.create_subscription(Path, topic,
                                     lambda m, t=topic, i=i: self._on_plan(t, i == 0, m),
                                     _latched_qos() if i == 0 else 10)
        if self.p('bumper_topic'):
            self.create_subscription(PointCloud2, self.p('bumper_topic'), self._on_bumper, qos)
        if self.p('area_settings_topic'):
            self.create_subscription(String, self.p('area_settings_topic'),
                                     self._on_area_settings, _latched_qos())
        self._zones_pub = (self.create_publisher(String, self.p('zones_topic'), 1)
                           if self.p('zones_topic') else None)
        self.create_service(ClearObstacle, '~/clear_obstacle', self._srv_clear_obstacle)
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
        self._discard_cli = self.create_client(ClearObstacle, self.p('discard_service'))
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
            elif prm.name in _CORE_LIST_KEYS:
                setattr(self.core.params, prm.name,
                        tuple(str(c).strip().lower() for c in prm.value if str(c).strip()))
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
        name = str(msg.state_name)
        if name != self._high_name:
            self._high_name = name
            if bool(self.p('drop_pending_at_session_end')) and \
                    name in list(self.p('session_end_states') or []):
                self._drop_pending(name)

    def _drop_pending(self, why):
        """Session end: drop the pending (not accepted) zones and their keepouts."""
        for t in self.core.drop_pending():
            self.get_logger().info('no-go zone #%d (%s) dropped at the end of the mow (%s): '
                                   'pending, not accepted' % (t.id, t.label(), why))
            self._remove_keepout(t, 'session end')

    def _on_plan(self, topic, is_coverage, msg):
        pts = np.array([(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses],
                       dtype=float).reshape(-1, 2)
        self._plans[topic] = (self._clk.now(), pts, is_coverage)
        if is_coverage:
            self._cursor.set_path(pts)

    def _on_bumper(self, msg):
        pts = cloud_xy(msg)
        now = self._clk.now()
        if len(pts) == 0 or now - self._bumper_time < float(self.p('bumper_debounce_s')):
            return
        self._bumper_time = now
        self._bumper_hits.append((float(pts[:, 0].mean()), float(pts[:, 1].mean())))

    def _on_area_settings(self, msg):
        self._settings_json = msg.data
        self._area_blocked = blocked_areas(self._settings_json, self._areas)

    def _plans_ahead(self, now):
        """Planned paths ahead for the on-path test: the coverage plan from the robot's
        cursor on, plus every other plan topic (route) younger than plan_max_age_s."""
        out = []
        max_age = float(self.p('plan_max_age_s'))
        for _topic, (t, pts, is_cov) in self._plans.items():
            if len(pts) < 2:
                continue
            if is_cov:
                if self._robot_pose is not None:
                    self._cursor.update(self._robot_pose[0], self._robot_pose[1])
                ahead = self._cursor.ahead()
                if len(ahead) >= 2:
                    out.append(ahead)
            elif now - t <= max_age:
                out.append(pts)
        return out

    def _robot_area(self):
        if self._robot_pose is None:
            return -1
        for (i, _n, poly) in self._areas:
            if point_in_polygon(self._robot_pose[0], self._robot_pose[1], poly):
                return int(i)
        return -1

    def _srv_clear_obstacle(self, req, res):
        """GUI Ignore (/obstacle_tracker/clear_obstacle): forget a tracked obstacle and
        suppress new zones at its spot for this session. A PENDING keepout made from it is
        removed from the map too; an accepted one is part of the saved map (edit the area)."""
        t = self.core.find_track(int(req.obstacle_id))
        if t is None:
            res.success = False
            res.message = 'no tracked obstacle #%d' % req.obstacle_id
            return res
        if t.promoted and not t.pending:
            res.success = False
            res.message = ('obstacle #%d is an accepted keepout: edit the area to remove it'
                           % t.id)
            return res
        was_promoted = t.promoted or t.promoting
        self.core.ignore(t.id)
        if was_promoted:
            self._remove_keepout(t, 'ignored')
        res.success = True
        res.message = 'obstacle #%d ignored%s' % (
            t.id, ' and its pending keepout removed' if was_promoted else '')
        self.get_logger().info(res.message)
        return res

    def _remove_keepout(self, t, why, then=None):
        """Remove the SOURCE_TRACKER keepout at a track's spot (pending or not) without
        touching the track state; ``then(ok)`` runs on the result."""
        if not self._remove_cli.service_is_ready():
            if then:
                then(False)
            return
        req = RemoveObstacle.Request()
        req.centroid.x, req.centroid.y = float(t.cx), float(t.cy)
        req.obstacle_id = 0
        req.max_distance_m = max(float(self.p('association_dist_m')),
                                 float(t.promoted_radius or t.footprint_radius(self.core.params)))
        fut = self._remove_cli.call_async(req)

        def done(f):
            try:
                r = f.result()
                ok = bool(r is not None and r.success)
            except Exception:  # noqa: BLE001
                ok = False
            self.get_logger().info('keepout of #%d removed (%s): %s'
                                   % (t.id, why, 'ok' if ok else 'not found'))
            if then:
                then(ok)
        fut.add_done_callback(done)

    def _send_resize(self, track_id):
        """A pending zone's size moved by >= resize_step_m: re-send it. map_server replaces
        the polygon of the PENDING keepout named '... #<id>' in place (no gap in the mask)."""
        t = self.core.find_track(track_id)
        if t is not None:
            self.get_logger().info('no-go zone #%d re-sized %.2f -> %.2f m'
                                   % (t.id, t.promoted_radius,
                                      t.footprint_radius(self.core.params)))
            self._send_promote(track_id, resize=True)

    def _publish_zones(self, now):
        if self._zones_pub is None:
            return
        keep = float(self.p('mark_memory_s'))
        self._marks = [m for m in self._marks if now - m[0] <= keep][-2000:]
        cells = {}
        for (_t, x, y, r) in self._marks:      # one mark per 0.1 m cell (largest radius)
            k = (round(x * 10), round(y * 10))
            if k not in cells or cells[k][2] < r:
                cells[k] = (x, y, r)
        p = self.core.params
        zones = []
        for t in self.core.tracks:
            if t.kind == 'cluster' and not t.promoted:
                continue
            state = ('pending' if t.pending else 'accepted') if t.promoted else 'candidate'
            zones.append(Zone(t.cx, t.cy, t.footprint_radius(p), t.id, t.label(), state,
                              t.score_max))
        zs = ZoneSet(self.get_clock().now().nanoseconds * 1e-9, zones,
                     [Zone(x, y, r, state='mark') for (x, y, r) in cells.values()])
        self._zones_pub.publish(String(data=zs.to_json()))

    def _on_detections(self, msg):
        """Ranged person / animal detections -> map-frame veto points. det_range tags a
        ranged detection with its base_link position in results[0].pose (covariance[0]
        > 0); an unranged one blocks every promotion for dynamic_memory_s."""
        now = self._clk.now()
        dyn = {str(c).strip().lower() for c in (self.p('dynamic_classes') or [])}
        classes = list(self.p('det_classes') or [])
        min_score = float(self.p('dynamic_min_score'))
        static = set(self.core.params.static_classes)
        for det in msg.detections:
            obs = det_observation(det, self._robot_pose, classes, float(self.p('camera_x_m')))
            if obs is not None and obs[2] and not obs[2].lstrip('-').isdigit():
                self._det_buf.append(obs)
                del self._det_buf[:-500]
                if obs[2] in static and obs[3] >= float(self.p('mark_min_score')):
                    r = (obs[4] / 2.0 if obs[4] > 0 else 0.0) + float(self.p('mark_margin_m'))
                    self._marks.append((now, obs[0], obs[1], r))
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
        for k in keepouts:      # accepted in the GUI since: no longer pending
            if k[4]:
                self.core.set_pending(int(k[4]), bool(k[5]))
        self._area_blocked = blocked_areas(self._settings_json, self._areas)
        if self._area_blocked != self._blocked_logged:
            self._blocked_logged = set(self._area_blocked)
            if self._area_blocked:
                self.get_logger().warn(
                    'obstacle_detection none in area(s) %s: no automatic no-go zones there '
                    '(and det_range sends no detection marks while mowing them)'
                    % ', '.join('%d:%s' % (i, n) for i, n, _ in self._areas
                                if i in self._area_blocked))
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
    def _send_promote(self, track_id, resize=False):
        t = self.core.find_track(track_id)
        if t is None or t.area_index < 0:
            return
        if not self._promote_cli.service_is_ready():
            if resize:
                self.core.resize_sent(track_id, False)
            else:
                self.core.reject_promotion(track_id)   # retry once map_server is up
            return
        # No polygon -> map_server uses the tracker-snapshot lookup path, which
        # stamps source=SOURCE_TRACKER (needed so ~/remove_obstacle may undo it).
        req = PromoteObstacle.Request()
        req.area_index = int(t.area_index)
        req.obstacle_id = int(t.id)
        # '#<id>' lets a restart re-adopt the keepout under the same id (its snapshot)
        if t.kind == 'det' and t.label():
            req.name = 'auto: %s #%d' % (t.label(), t.id)
        elif t.kind == 'bumper':
            req.name = 'auto: bumper #%d' % t.id
        else:
            req.name = '%s #%d' % (self.p('obstacle_name'), t.id)
        # the polygon map_server copies is the one in our latest snapshot: publish first
        self._publish_tracks(self._clk.now())
        fut = self._promote_cli.call_async(req)
        fut.add_done_callback(lambda f, tid=track_id, rs=resize: self._on_promote(f, tid, rs))

    def _on_promote(self, fut, track_id, resize=False):
        t = self.core.find_track(track_id)
        try:
            resp = fut.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('promote_obstacle(#%d) failed: %s' % (track_id, exc))
            self.core.reject_promotion(track_id)
            if resize:
                self.core.resize_sent(track_id, False)
            return
        if resize:
            self.core.resize_sent(track_id, bool(resp is not None and resp.success))
            return
        if resp is not None and resp.success:
            self.core.confirm_promoted(track_id, self._clk.now())
            if t is not None and t.kind != 'cluster':
                self.get_logger().warn(
                    'no-go zone #%d proposed (pending review): %s r=%.2f m at (%.2f, %.2f), '
                    'score %.2f, on path: %s, area %d'
                    % (t.id, t.label() or t.kind, t.footprint_radius(self.core.params),
                       t.cx, t.cy, t.score_max, t.on_path, t.area_index))
            else:
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

        if self._det_buf:
            self.core.add_detections(self._det_buf, now)
            self._det_buf = []
        if self._bumper_hits:
            self.core.add_bumper_hits(self._bumper_hits, now, self._areas)
            self._bumper_hits = []
        result = self.core.step(obstacle_pts, now, self._robot_pose, clear_pts, self._areas,
                                obstacle_fresh=obstacle_fresh, clear_fresh=clear_fresh,
                                allow_promote=self._promotion_allowed(now),
                                dynamic_points=self._dynamic_points(now),
                                plans=self._plans_ahead(now),
                                robot_area=self._robot_area(),
                                area_blocked=self._area_blocked)

        self._publish_tracks(now)
        self._publish_zones(now)
        self._update_snapshots(now)
        for tid in result['promote']:
            self._send_promote(tid)
        for tid in result['remove']:
            self._send_remove(tid)
        for tid in result.get('resize', []):
            self._send_resize(tid)

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
