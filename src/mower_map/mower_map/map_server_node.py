# SPDX-License-Identifier: GPL-3.0-or-later
"""map_server_node: zones, keepout mask, mow progress and dock pose.

Python port of the MowgliNext ``mowgli_map`` map_server_node service/topic
contract (see README.md). Geometry and persistence live in
``mower_map.areas`` (ROS-free, unit tested).
"""

import array
import math
import os
import time
from collections import deque

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Point32, Polygon as PolygonMsg, PoseStamped
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from mower_interfaces.msg import MowerBaseDevStatus
from mowgli_interfaces.msg import GnssStatus, MapArea, MapObstacleInfo, ObstacleArray
from mowgli_interfaces.srv import (AddMowingArea, ClearObstacle, GetMowingArea,
                                   GetRecoveryPoint, PromoteObstacle, SetDockingPoint)

from mower_map import areas as core

try:
    import tf2_ros
except ImportError:  # pragma: no cover
    tf2_ros = None

CAP_HORIZONTAL_ACCURACY = 8

PARAMS = {
    'maps_dir': '/ros2_ws/maps',
    'areas_file': '',                  # '' -> <maps_dir>/areas.dat
    'dock_file': '',                   # '' -> <maps_dir>/dock_pose.yaml
    'robot_yaml_path': '',             # '' -> do not touch mowgli_robot.yaml
    'map_frame': 'map',
    'base_frame': 'base_footprint',
    'blade_frame': 'blade_link',
    'resolution': 0.1,
    'mask_margin': 2.0,                # lethal ring around the areas' bbox (m)
    'obstacle_margin': 0.0,            # grow obstacles in the mask (m)
    'lethal_outside_areas': True,
    'dock_keepout': True,              # dock outline lethal in the mask
    # default dock outline (dock-local, m) used when dock_pose.yaml has none;
    # the vendor's type-5 outline (charger plate around the rear axle)
    'dock_outline_x_min': -0.2,
    'dock_outline_x_max': 0.2,
    'dock_outline_half_width': 0.275,
    'datum_lat': 0.0,                  # areas.dat datum stamp only
    'datum_lon': 0.0,
    # mow progress
    'odom_topic': '/odometry/filtered_map',
    'status_topic': '/mower_base/status',
    'gps_status_topic': '/gps/status',
    'blade_radius': 0.15,
    'blade_offset_x': 0.3,             # fallback if TF base->blade is unavailable
    'blade_offset_y': 0.0,
    'status_max_age_s': 1.0,           # is_cutting older than this -> not cutting
    'mow_progress_publish_period_s': 2.0,
    # boundary
    'boundary_check_rate_hz': 5.0,
    'soft_boundary_margin_m': 0.3,
    'lethal_boundary_margin_m': 0.5,
    'boundary_debounce_samples': 3,
    'boundary_recovery_offset_m': 0.8,
    # set_docking_point gates
    'dock_gates_override': False,      # bench testing: skip every gate
    'require_charging': True,
    'require_rtk_accuracy': True,
    'require_yaw_converged': True,
    'dock_set_status_max_age_s': 2.0,
    'dock_set_gps_accuracy_max_m': 0.04,
    'dock_set_gps_max_age_s': 2.0,
    'yaw_convergence_threshold_rad': 0.00873,
    'yaw_convergence_window_s': 5.0,
    'yaw_convergence_min_samples': 20,
}


def _latched(depth=1):
    return QoSProfile(depth=depth, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=QoSReliabilityPolicy.RELIABLE)


def _poly_to_msg(poly):
    msg = PolygonMsg()
    msg.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in poly]
    return msg


def _poly_from_msg(msg):
    return [(float(p.x), float(p.y)) for p in msg.points]


class MapServerNode(Node):

    def __init__(self, **kwargs):
        super().__init__('map_server_node', **kwargs)
        for k, v in PARAMS.items():
            self.declare_parameter(k, v)
        maps_dir = self.p('maps_dir')
        self.areas_path = self.p('areas_file') or os.path.join(maps_dir, 'areas.dat')
        self.dock_path = self.p('dock_file') or os.path.join(maps_dir, 'dock_pose.yaml')
        self.map_frame = self.p('map_frame')
        self.resolution = float(self.p('resolution'))

        self.store = core.MapStore()
        self.dock = None                     # core.DockPose
        self.spec = core.grid_for_polygons([], self.resolution, 0.0)
        self.progress = core.ProgressGrid(self.spec)
        self.progress_dirty = True
        self.boundary = core.BoundaryClassifier(
            self.p('soft_boundary_margin_m'), self.p('lethal_boundary_margin_m'),
            self.p('boundary_debounce_samples'))
        self.tracker_snapshot = []

        # inputs
        self.status = None
        self.status_time = None
        self.gnss = None
        self.gnss_time = None
        self.pose = None                     # (x, y, yaw, child_frame)
        self.pose_time = None
        self.recent_poses = deque()          # (t, x, y, yaw)
        self.blade_offset = None
        self.last_boundary_check = 0.0

        # publishers
        self.mask_pub = self.create_publisher(OccupancyGrid, '/keepout_mask', _latched())
        self.info_pub = self.create_publisher(CostmapFilterInfo, '/costmap_filter_info',
                                              _latched())
        self.progress_pub = self.create_publisher(OccupancyGrid, '~/mow_progress', _latched())
        self.dock_pub = self.create_publisher(PoseStamped, '~/docking_pose', _latched())
        self.boundary_pub = self.create_publisher(Bool, '~/boundary_violation', 1)
        self.lethal_pub = self.create_publisher(Bool, '~/lethal_boundary_violation', 1)
        self.replan_pub = self.create_publisher(Bool, '~/replan_needed', 1)

        # subscriptions
        self.create_subscription(MowerBaseDevStatus, self.p('status_topic'), self.on_status, 10)
        self.create_subscription(GnssStatus, self.p('gps_status_topic'), self.on_gnss, 10)
        self.create_subscription(Odometry, self.p('odom_topic'), self.on_odom, 10)
        self.create_subscription(ObstacleArray, '/obstacle_tracker/obstacles',
                                 self.on_tracker, 1)

        # services
        self.create_service(AddMowingArea, '~/add_area', self.srv_add_area)
        self.create_service(GetMowingArea, '~/get_mowing_area', self.srv_get_mowing_area)
        self.create_service(Trigger, '~/clear_map', self.srv_clear_map)
        self.create_service(Trigger, '~/save_areas', self.srv_save_areas)
        self.create_service(Trigger, '~/load_areas', self.srv_load_areas)
        self.create_service(SetDockingPoint, '~/set_docking_point', self.srv_set_docking_point)
        self.create_service(PromoteObstacle, '~/promote_obstacle', self.srv_promote_obstacle)
        self.create_service(ClearObstacle, '~/discard_obstacle', self.srv_discard_obstacle)
        self.create_service(GetRecoveryPoint, '~/get_recovery_point',
                            self.srv_get_recovery_point)
        self.create_service(Trigger, '~/reset_mow_progress', self.srv_reset_mow_progress)

        self.tf_buffer = None
        if tf2_ros is not None:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.publish_filter_info()
        self.load_dock()
        self.load_areas(startup=True)
        self.create_timer(max(0.1, float(self.p('mow_progress_publish_period_s'))),
                          self.on_progress_timer)
        self.get_logger().info('map_server_node up: %d areas from %s, dock %s'
                               % (len(self.store.areas), self.areas_path,
                                  'set' if self.dock else 'unset'))

    def p(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------
    # state changes
    # ------------------------------------------------------------------
    def dock_outline_map(self):
        if self.dock is None or not self.p('dock_keepout'):
            return None
        return self.dock.outline_in_map()

    def default_dock_outline(self):
        x0, x1 = float(self.p('dock_outline_x_min')), float(self.p('dock_outline_x_max'))
        hw = float(self.p('dock_outline_half_width'))
        if x1 <= x0 or hw <= 0:
            return None
        return [(x1, hw), (x1, -hw), (x0, -hw), (x0, hw)]

    def rebuild(self, replan=True):
        """Recompute the grid, publish the keepout mask, resize progress."""
        polys = [a.polygon for a in self.store.areas]
        outline = self.dock_outline_map()
        if outline:
            polys.append(outline)
        try:
            spec = core.grid_for_polygons(polys, self.resolution, float(self.p('mask_margin')))
        except ValueError as exc:
            self.get_logger().error('keepout mask not rebuilt: %s' % exc)
            return
        t0 = time.monotonic()
        mask = core.build_keepout_mask(self.store.areas, spec, outline,
                                       float(self.p('obstacle_margin')),
                                       bool(self.p('lethal_outside_areas')))
        self.mask_pub.publish(self.grid_msg(spec, mask))
        if spec != self.spec:
            self.progress.resize(spec)
            self.spec = spec
        self.progress_dirty = True
        self.publish_progress()
        self.get_logger().info('keepout mask %dx%d @ %.2f m, origin (%.2f, %.2f), %.0f ms'
                               % (spec.width, spec.height, spec.resolution, spec.origin_x,
                                  spec.origin_y, (time.monotonic() - t0) * 1e3))
        if replan:
            self.replan_pub.publish(Bool(data=True))

    def grid_msg(self, spec, data):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = float(spec.resolution)
        msg.info.width = spec.width
        msg.info.height = spec.height
        msg.info.origin.position.x = float(spec.origin_x)
        msg.info.origin.position.y = float(spec.origin_y)
        msg.info.origin.orientation.w = 1.0
        msg.data = array.array('b', data.astype('int8').tobytes())
        return msg

    def publish_filter_info(self):
        info = CostmapFilterInfo()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = self.map_frame
        info.type = 0
        info.filter_mask_topic = '/keepout_mask'
        info.base = 0.0
        info.multiplier = 1.0
        self.info_pub.publish(info)

    def publish_progress(self):
        self.progress_pub.publish(self.grid_msg(self.spec, self.progress.data))
        self.progress_dirty = False

    def publish_dock(self):
        if self.dock is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = float(self.dock.x)
        msg.pose.position.y = float(self.dock.y)
        msg.pose.orientation.z = math.sin(self.dock.yaw / 2.0)
        msg.pose.orientation.w = math.cos(self.dock.yaw / 2.0)
        self.dock_pub.publish(msg)

    def save_areas(self):
        core.save_areas_file(self.areas_path, self.store.areas,
                             float(self.p('datum_lat')), float(self.p('datum_lon')))

    def persist_best_effort(self, context):
        try:
            self.save_areas()
        except OSError as exc:
            self.get_logger().warn('%s: applied live but save failed: %s' % (context, exc))

    def load_areas(self, startup=False):
        if not os.path.exists(self.areas_path):
            if startup:
                self.get_logger().info('no %s yet: starting with an empty map' % self.areas_path)
                self.rebuild(replan=False)
            return False, 'no file %s' % self.areas_path
        try:
            areas, datum = core.load_areas_file(self.areas_path)
        except (OSError, UnicodeDecodeError) as exc:
            self.get_logger().error('load %s failed: %s' % (self.areas_path, exc))
            if startup:
                self.rebuild(replan=False)
            return False, str(exc)
        self.store.load(areas)
        lat, lon = float(self.p('datum_lat')), float(self.p('datum_lon'))
        if datum and (abs(lat) > 1e-9 or abs(lon) > 1e-9) and \
                (abs(datum[0] - lat) > 1e-8 or abs(datum[1] - lon) > 1e-8):
            self.get_logger().warn(
                'areas.dat datum (%.9f, %.9f) differs from datum param (%.9f, %.9f); '
                'polygons are NOT re-projected' % (datum[0], datum[1], lat, lon))
        for a in self.store.areas:
            self.get_logger().info("loaded area '%s': %d vertices, %s, %d obstacles"
                                   % (a.name, len(a.polygon),
                                      'navigation' if a.is_navigation else 'mowing',
                                      len(a.obstacles)))
        self.rebuild(replan=not startup)
        return True, 'loaded %d areas from %s' % (len(self.store.areas), self.areas_path)

    def load_dock(self):
        dock = None
        if os.path.exists(self.dock_path):
            try:
                dock = core.load_dock_file(self.dock_path)
            except Exception as exc:  # yaml errors etc.
                self.get_logger().error('load %s failed: %s' % (self.dock_path, exc))
        if dock is None and self.p('robot_yaml_path'):
            d = core.read_robot_yaml_dock_pose(self.p('robot_yaml_path'))
            if d is not None and (d.x or d.y or d.yaw):
                dock = d
        if dock is not None:
            if dock.outline is None:
                dock.outline = self.default_dock_outline()
            self.dock = dock
            self.publish_dock()

    # ------------------------------------------------------------------
    # subscriptions
    # ------------------------------------------------------------------
    def on_status(self, msg):
        self.status = msg
        self.status_time = time.monotonic()

    def on_gnss(self, msg):
        self.gnss = msg
        self.gnss_time = time.monotonic()

    def on_tracker(self, msg):
        self.tracker_snapshot = list(msg.obstacles)

    def is_cutting(self):
        return (self.status is not None and self.status_time is not None
                and time.monotonic() - self.status_time <= float(self.p('status_max_age_s'))
                and bool(self.status.is_cutting))

    def get_blade_offset(self, child_frame):
        if self.blade_offset is not None:
            return self.blade_offset
        if self.tf_buffer is not None and child_frame:
            try:
                tf = self.tf_buffer.lookup_transform(child_frame, self.p('blade_frame'), Time())
                self.blade_offset = (tf.transform.translation.x, tf.transform.translation.y)
                self.get_logger().info('blade offset from TF %s->%s: (%.3f, %.3f)'
                                       % (child_frame, self.p('blade_frame'),
                                          *self.blade_offset))
                return self.blade_offset
            except Exception:
                pass
        return (float(self.p('blade_offset_x')), float(self.p('blade_offset_y')))

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = core.yaw_from_quaternion(q.x, q.y, q.z, q.w)
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        now = time.monotonic()
        self.pose = (x, y, yaw, msg.child_frame_id)
        self.pose_time = now
        self.recent_poses.append((now, x, y, yaw))
        window = float(self.p('yaw_convergence_window_s'))
        while self.recent_poses and now - self.recent_poses[0][0] > window:
            self.recent_poses.popleft()

        if self.is_cutting():
            ox, oy = self.get_blade_offset(msg.child_frame_id)
            c, s = math.cos(yaw), math.sin(yaw)
            if self.progress.stamp_disc(x + c * ox - s * oy, y + s * ox + c * oy,
                                        float(self.p('blade_radius'))):
                self.progress_dirty = True

        rate = float(self.p('boundary_check_rate_hz'))
        if self.store.areas and (rate <= 0 or now - self.last_boundary_check >= 1.0 / rate):
            self.last_boundary_check = now
            self.boundary.soft_margin = float(self.p('soft_boundary_margin_m'))
            self.boundary.lethal_margin = float(self.p('lethal_boundary_margin_m'))
            self.boundary.debounce_samples = int(self.p('boundary_debounce_samples'))
            inside, dist = core.robot_area_status(x, y, self.store.areas)
            soft, lethal = self.boundary.update(inside, dist)
            self.boundary_pub.publish(Bool(data=soft))
            self.lethal_pub.publish(Bool(data=lethal))
            if lethal and self.is_cutting():
                self.get_logger().error('LETHAL BOUNDARY VIOLATION at (%.2f, %.2f), %.2f m '
                                        'outside' % (x, y, dist),
                                        throttle_duration_sec=2.0)

    def on_progress_timer(self):
        if self.progress_dirty:
            self.publish_progress()

    # ------------------------------------------------------------------
    # services
    # ------------------------------------------------------------------
    def srv_add_area(self, req, res):
        a = req.area
        obstacles = []
        for j, obs in enumerate(a.obstacles):
            info = a.obstacle_info[j] if j < len(a.obstacle_info) else None
            obstacles.append((_poly_from_msg(obs), info.name if info else '',
                              int(info.source) if info else core.SOURCE_USER))
        is_nav = bool(req.is_navigation_area or a.is_navigation_area)
        try:
            core.grid_for_polygons([_poly_from_msg(a.area)], self.resolution,
                                   float(self.p('mask_margin')))
            self.store.add_area(a.name, _poly_from_msg(a.area), is_nav, obstacles)
        except ValueError as exc:
            self.get_logger().warn('add_area rejected: %s' % exc)
            res.success = False
            return res
        self.get_logger().info("added area '%s' (%s), %d vertices, %d obstacles"
                               % (a.name, 'navigation' if is_nav else 'mowing',
                                  len(a.area.points), len(a.obstacles)))
        self.rebuild()
        self.persist_best_effort('add_area')
        res.success = True
        return res

    def srv_get_mowing_area(self, req, res):
        idx = int(req.index)
        if idx >= len(self.store.areas):
            res.success = False
            return res
        area = self.store.areas[idx]
        out = MapArea()
        out.name = area.name
        out.area = _poly_to_msg(area.polygon)
        out.is_navigation_area = area.is_navigation
        for obs in area.obstacles:
            out.obstacles.append(_poly_to_msg(obs.polygon))
            out.obstacle_info.append(MapObstacleInfo(name=obs.name, source=obs.source,
                                                     pending=obs.pending, id=obs.id))
        res.area = out
        res.success = True
        return res

    def srv_clear_map(self, req, res):
        self.store.clear()
        self.progress.reset()
        self.rebuild()
        self.persist_best_effort('clear_map')
        res.success = True
        res.message = 'All map layers and areas cleared.'
        self.get_logger().info(res.message)
        return res

    def srv_save_areas(self, req, res):
        try:
            self.save_areas()
            res.success = True
            res.message = 'saved %d areas to %s' % (len(self.store.areas), self.areas_path)
        except OSError as exc:
            res.success = False
            res.message = 'save failed: %s' % exc
        self.get_logger().info(res.message)
        return res

    def srv_load_areas(self, req, res):
        res.success, res.message = self.load_areas()
        self.get_logger().info(res.message)
        return res

    def srv_reset_mow_progress(self, req, res):
        self.progress.reset()
        self.publish_progress()
        res.success = True
        res.message = 'mow progress cleared'
        return res

    def dock_gate_failure(self):
        """None when set_docking_point may proceed, else the reason."""
        if self.p('dock_gates_override'):
            return None
        now = time.monotonic()
        if self.p('require_charging'):
            max_age = float(self.p('dock_set_status_max_age_s'))
            if self.status is None or now - self.status_time > max_age:
                return 'no fresh %s (robot not detected on dock)' % self.p('status_topic')
            if not (self.status.is_charging or self.status.is_docking_done):
                return 'robot not on the dock (is_charging and is_docking_done both false)'
        if self.p('require_rtk_accuracy'):
            max_age = float(self.p('dock_set_gps_max_age_s'))
            if self.gnss is None or now - self.gnss_time > max_age:
                return 'no fresh %s' % self.p('gps_status_topic')
            acc = float(self.gnss.horizontal_accuracy_m)
            if not (self.gnss.value_flags & CAP_HORIZONTAL_ACCURACY) or not math.isfinite(acc) \
                    or acc > float(self.p('dock_set_gps_accuracy_max_m')):
                return ('GPS not accurate enough (horizontal_accuracy_m=%.3f > %.3f, need '
                        'RTK fixed)' % (acc, float(self.p('dock_set_gps_accuracy_max_m'))))
        if self.p('require_yaw_converged'):
            yaws = [p[3] for p in self.recent_poses]
            need = int(self.p('yaw_convergence_min_samples'))
            if len(yaws) < need:
                return 'only %d yaw samples in the window (need %d)' % (len(yaws), need)
            std = core.yaw_circular_std(yaws)
            if std > float(self.p('yaw_convergence_threshold_rad')):
                return 'EKF yaw not converged (std %.3f deg)' % math.degrees(std)
        return None

    def srv_set_docking_point(self, req, res):
        why = self.dock_gate_failure()
        if why:
            self.get_logger().warn('set_docking_point rejected: ' + why)
            res.success = False
            return res
        old = self.dock
        if req.use_gps_position:
            if not self.recent_poses:
                self.get_logger().warn('set_docking_point rejected: use_gps_position but no '
                                       '%s pose yet' % self.p('odom_topic'))
                res.success = False
                return res
            x = sum(p[1] for p in self.recent_poses) / len(self.recent_poses)
            y = sum(p[2] for p in self.recent_poses) / len(self.recent_poses)
        else:
            x, y = req.docking_pose.position.x, req.docking_pose.position.y
        if req.yaw_source == SetDockingPoint.Request.MOTION:
            yaw = float(req.yaw_rad)
        elif req.yaw_source == SetDockingPoint.Request.REQUEST:
            q = req.docking_pose.orientation
            yaw = core.yaw_from_quaternion(q.x, q.y, q.z, q.w)
        else:  # PRESERVE
            yaw = old.yaw if old is not None else 0.0
        outline = old.outline if old is not None and old.outline else self.default_dock_outline()
        self.dock = core.DockPose(float(x), float(y), float(yaw), outline)
        self.publish_dock()
        try:
            core.save_dock_file(self.dock_path, self.dock)
        except OSError as exc:
            self.get_logger().warn('dock pose applied but not saved: %s' % exc)
        if self.p('robot_yaml_path'):
            if not core.update_robot_yaml_dock_pose(self.p('robot_yaml_path'), self.dock.x,
                                                    self.dock.y, self.dock.yaw):
                self.get_logger().warn('could not update %s' % self.p('robot_yaml_path'))
        self.get_logger().info('docking point set: (%.3f, %.3f) yaw %.3f rad'
                               % (self.dock.x, self.dock.y, self.dock.yaw))
        self.rebuild()
        res.success = True
        return res

    def srv_promote_obstacle(self, req, res):
        if req.pending_id != 0:
            idx = self.store.accept_pending(int(req.pending_id), req.name)
            if idx is None:
                res.success = False
                res.message = ('no pending obstacle with id %d (already accepted, discarded, '
                               'or lost to a restart)' % req.pending_id)
                return res
            self.persist_best_effort('promote_obstacle')
            res.success = True
            res.message = ('pending obstacle %d accepted as a permanent keepout for area %d'
                           % (req.pending_id, idx))
            return res
        poly = _poly_from_msg(req.polygon)
        source = core.SOURCE_USER
        if len(poly) < 3:
            source = core.SOURCE_TRACKER
            match = [o for o in self.tracker_snapshot if o.id == req.obstacle_id]
            if not match:
                res.success = False
                res.message = ('obstacle_id not found in last tracker snapshot and no polygon '
                               'supplied')
                return res
            poly = _poly_from_msg(match[0].polygon)
        ok, msg = self.store.add_obstacle(int(req.area_index), poly, req.name, source)
        if not ok:
            res.success = False
            res.message = 'promotion rejected: ' + msg
            return res
        self.rebuild()
        self.persist_best_effort('promote_obstacle')
        res.success = True
        res.message = 'obstacle promoted to permanent keepout for area %d' % req.area_index
        self.get_logger().info(res.message)
        return res

    def srv_discard_obstacle(self, req, res):
        if not self.store.discard_pending(int(req.obstacle_id)):
            res.success = False
            res.message = ('no pending obstacle with id %d (accepted keepouts are part of the '
                           'saved map - edit the area to remove one)' % req.obstacle_id)
            return res
        self.rebuild()
        res.success = True
        res.message = 'pending obstacle %d discarded' % req.obstacle_id
        return res

    def robot_xy(self):
        if self.pose is not None and time.monotonic() - self.pose_time < 2.0:
            return self.pose[0], self.pose[1], None
        if self.tf_buffer is None:
            return None, None, 'no pose (odometry stale, tf unavailable)'
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.p('base_frame'), Time())
            return tf.transform.translation.x, tf.transform.translation.y, None
        except Exception as exc:
            return None, None, 'tf lookup failed: %s' % exc

    def srv_get_recovery_point(self, req, res):
        x, y, err = self.robot_xy()
        if err:
            res.success = False
            res.message = err
            return res
        r = core.recovery_point(x, y, self.store.areas,
                                float(self.p('boundary_recovery_offset_m')))
        res.success = r['success']
        res.message = r['message']
        res.recovery_pose.position.x = float(r['x'])
        res.recovery_pose.position.y = float(r['y'])
        res.recovery_pose.orientation.z = math.sin(r['yaw'] / 2.0)
        res.recovery_pose.orientation.w = math.cos(r['yaw'] / 2.0)
        res.distance_outside = float(r['distance_outside'])
        return res


def main(args=None):
    rclpy.init(args=args)
    node = MapServerNode()
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
