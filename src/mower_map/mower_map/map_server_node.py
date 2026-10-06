# SPDX-License-Identifier: GPL-3.0-or-later
"""map_server_node: zones, keepout mask, mow progress and dock pose.

Python port of the MowgliNext ``mowgli_map`` map_server_node service/topic
contract (see README.md). Geometry and persistence live in
``mower_map.areas`` (ROS-free, unit tested).
"""

import array
import math
import functools
import json
import os
import threading
import time
from collections import deque

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Point32, Polygon as PolygonMsg, PolygonStamped, PoseStamped
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from mower_interfaces.msg import MowerBaseDevStatus
from mower_interfaces.srv import GetAreaSettings, SetAreaSettings
from mowgli_interfaces.msg import GnssStatus, MapArea, MapObstacleInfo, ObstacleArray
from mowgli_interfaces.srv import (AddMowingArea, ClearObstacle, GetMowingArea,
                                   GetRecoveryPoint, PromoteObstacle, SetDockingPoint)

from mower_map import area_settings as aset
from mower_map import areas as core
from mower_map import terrain as terr
from mower_map.sub_pump import SubscriptionPump, flat_parser, parse_odometry

try:
    import tf2_ros
    from tf2_msgs.msg import TFMessage
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
    'dock_keepout': True,              # dock outline lethal in the MOWING mask only
    # navigation mask (/nav_keepout_mask; Nav2 global costmap + keepout filter):
    # (mowing U navigation areas) grown by nav_margin_m, minus obstacles grown
    # by nav_obstacle_margin_m; the dock outline is never lethal here.
    'nav_margin_m': 0.35,              # half footprint width (0.27) + 0.08
    'nav_obstacle_margin_m': 0.10,
    # dock corridor (nav mask only): capsule of half-width nav_margin_m from the
    # dock through its approach pose to the nearest area boundary, so docking
    # still plans when the dock lies outside every drawn area.
    'dock_corridor_enabled': True,
    'dock_corridor_max_m': 5.0,        # approach pose -> area gap beyond this: dock capsule only
    'approach_distance': 0.8,          # same name/default as mower_docking
    # soft band (nav mask only): cells outside the free set but within this
    # distance of it cost SOFT_COST (90 of 100; 100 = lethal) instead of lethal
    'nav_soft_band_m': 1.5,
    # return corridor (nav mask only): while the robot pose is outside the free
    # set (areas + nav_margin_m + dock corridor), free a capsule of half-width
    # nav_margin_m from the robot to the closest free cell so Nav2 can plan back.
    'return_corridor_enabled': True,
    'return_corridor_max_m': 15.0,     # longer gap: warn, no corridor
    'return_corridor_rebuild_m': 0.5,  # rebuild after the robot moved this far
    'return_corridor_check_period_s': 1.0,
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
    # dock yaw auto-correct: a dock yaw captured from the fused heading while the
    # heading_aligner quality was low is rotated by (COG offset - set-time offset)
    # once the aligner reaches a good COG alignment (once per set, same session).
    'heading_status_topic': '/heading_aligner/status',
    'dock_yaw_autocorrect': True,
    # per-area mowing settings (area_settings.yaml next to areas.dat)
    'area_settings_file': '',          # '' -> <dir of areas_file>/area_settings.yaml
    'area_settings_prune_delay_s': 30.0,   # after clear_map: drop entries of areas not re-added
    # terrain memory (docs/terrain_aware_planning.md): per-area slope + traction raster
    # and incident log in <maps_dir>/terrain_<area>.npz/.json
    'terrain_enabled': True,
    'terrain_dir': '',                 # '' -> dir of areas_file
    'terrain_imu_topic': '/imu/data_aligned',
    'terrain_imu_max_age_s': 0.5,
    'terrain_tilt_max_deg': 35.0,      # tilt samples beyond this are dropped (lift, bump)
    'terrain_tilt_max_yaw_rate': 0.6,  # rad/s; turning -> centripetal accel pollutes tilt
    'terrain_sample_min_move_m': 0.05,  # one tilt sample per this much travel
    'terrain_roll_offset_deg': 0.0,    # IMU mounting bias (cancels anyway over headings)
    'terrain_pitch_offset_deg': 0.0,
    'terrain_dig_topic': '/dig_stall',
    'terrain_incident_topic': '/mission/incident',
    'terrain_bad_half_life_days': 21.0,
    'terrain_slope_half_life_days': 365.0,
    'terrain_cluster_radius_m': 1.0,
    'terrain_keepout_margin_m': 0.4,
    'terrain_save_period_s': 60.0,
    'terrain_publish_period_s': 10.0,
    'terrain_cost_threshold': 20.0,    # traction score below this costs nothing
    'terrain_cost_max': 60,            # nav cost at score 100 (< 65: never lethal)
    'terrain_slope_min_deg': 3.0,      # slope_mode: flatter -> keep the planner's angle
}


def _latched(depth=1):
    return QoSProfile(depth=depth, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=QoSReliabilityPolicy.RELIABLE)


def _poly_to_msg(poly):
    msg = PolygonMsg()
    msg.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in poly]
    return msg


def _poly_from_msg(msg):
    # The GUI sends GeoJSON rings (first vertex repeated at the end); the
    # stored ring is implicitly closed, so drop the duplicate closing vertex.
    return core.normalise_polygon((p.x, p.y) for p in msg.points)


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
        self.settings_path = self.p('area_settings_file') or \
            aset.settings_path_for(self.areas_path)
        self.settings = aset.AreaSettingsStore()
        self._cleared_areas = []             # (name, polygon) before the last clear_map
        self._prune_timer = None
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
        self.heading = None                  # last /heading_aligner/status dict
        # True only for a dock set in THIS session with a low-quality heading: the
        # offset delta is meaningless across an IMU/aligner restart (yaw zero moves).
        self._dock_autocorrect_armed = False
        self.pose_time = None
        self.recent_poses = deque()          # (t, x, y, yaw)
        self.blade_offset = None
        self.last_boundary_check = 0.0
        self.base_nav = None                 # (spec, nav mask without return corridor)
        self.return_anchor = None            # robot (x, y) the return corridor was built for
        self.return_active = False
        self.last_return_check = 0.0

        # publishers
        self.mask_pub = self.create_publisher(OccupancyGrid, '/keepout_mask', _latched())
        self.nav_mask_pub = self.create_publisher(OccupancyGrid, '/nav_keepout_mask',
                                                  _latched())
        self.info_pub = self.create_publisher(CostmapFilterInfo, '/costmap_filter_info',
                                              _latched())
        self.progress_pub = self.create_publisher(OccupancyGrid, '~/mow_progress', _latched())
        self.dock_pub = self.create_publisher(PoseStamped, '~/docking_pose', _latched())
        # latched: True only when the dock pose was recorded by set_docking_point
        self.dock_measured_pub = self.create_publisher(Bool, '~/docking_pose_measured',
                                                       _latched())
        self.corridor_pub = self.create_publisher(PolygonStamped, '~/dock_corridor', _latched())
        self.return_corridor_pub = self.create_publisher(PolygonStamped, '~/return_corridor',
                                                         _latched())
        self.boundary_pub = self.create_publisher(Bool, '~/boundary_violation', 1)
        self.lethal_pub = self.create_publisher(Bool, '~/lethal_boundary_violation', 1)
        self.replan_pub = self.create_publisher(Bool, '~/replan_needed', 1)
        self.settings_pub = self.create_publisher(String, '~/area_settings', _latched())
        # terrain memory outputs (latched): traction score 0..100 (-1 = never driven),
        # the Nav2 cost layer (0..terrain_cost_max) and a JSON summary per area.
        self.terrain_grid_pub = self.create_publisher(OccupancyGrid, '~/terrain_grid', _latched())
        self.terrain_cost_pub = self.create_publisher(OccupancyGrid, '~/terrain_cost', _latched())
        self.terrain_summary_pub = self.create_publisher(String, '~/terrain_summary', _latched())
        self.terrain = {}                    # area name -> terr.AreaTerrain
        self.terrain_dirty = False           # published layers out of date
        self._terrain_last_xy = None
        self._terrain_last_t = None
        self._dig_latched = False
        self._lethal_prev = False
        self._soft_prev = False

        # Inputs bypass rclpy.spin (see sub_pump.py): 100 Hz status + 30 Hz odometry + 30 Hz
        # /tf through the executor cost ~40 % of a core while idle. Odometry is handled per
        # message (progress stamping, yaw window, boundary checks); the latest-value inputs
        # are sampled (depth 1) where they are read, keeping their receipt times. State used
        # to be touched by the single spin thread only, so pump callbacks, services and the
        # timer all run under self._lock now.
        self._lock = threading.RLock()
        self._pump = SubscriptionPump(self, 'map_server_inputs')
        sub = self._pump.subscribe
        lock = self._lock
        self._status_sub = sub(MowerBaseDevStatus, self.p('status_topic'), self.on_status, 1,
                               parser=flat_parser(MowerBaseDevStatus), lock=lock,
                               sampled=True, with_receipt=True)
        self._gnss_sub = sub(GnssStatus, self.p('gps_status_topic'), self.on_gnss, 1, lock=lock,
                             sampled=True, with_receipt=True)
        sub(Odometry, self.p('odom_topic'), self.on_odom, 10, parser=parse_odometry, lock=lock)
        sub(String, self.p('heading_status_topic'), self.on_heading_status,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       history=QoSHistoryPolicy.KEEP_LAST), lock=lock)
        self._tracker_sub = sub(ObstacleArray, '/obstacle_tracker/obstacles', self.on_tracker,
                                1, lock=lock, sampled=True)
        self._imu_sub = None
        if self.p('terrain_enabled'):
            from sensor_msgs.msg import Imu
            self._imu_sub = sub(Imu, self.p('terrain_imu_topic'), self.on_imu, 1, lock=lock,
                                sampled=True, with_receipt=True)
            sub(Bool, self.p('terrain_dig_topic'), self.on_dig_stall,
                QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                           history=QoSHistoryPolicy.KEEP_LAST), lock=lock)
            sub(String, self.p('terrain_incident_topic'), self.on_incident, 20, lock=lock)
        self.imu = None
        self.imu_time = None

        # services
        srv = self._locked_service
        srv(AddMowingArea, '~/add_area', self.srv_add_area)
        srv(GetMowingArea, '~/get_mowing_area', self.srv_get_mowing_area)
        srv(Trigger, '~/clear_map', self.srv_clear_map)
        srv(Trigger, '~/save_areas', self.srv_save_areas)
        srv(Trigger, '~/load_areas', self.srv_load_areas)
        srv(SetDockingPoint, '~/set_docking_point', self.srv_set_docking_point)
        srv(PromoteObstacle, '~/promote_obstacle', self.srv_promote_obstacle)
        srv(ClearObstacle, '~/discard_obstacle', self.srv_discard_obstacle)
        srv(GetRecoveryPoint, '~/get_recovery_point', self.srv_get_recovery_point)
        srv(Trigger, '~/reset_mow_progress', self.srv_reset_mow_progress)
        srv(SetAreaSettings, '~/set_area_settings', self.srv_set_area_settings)
        srv(GetAreaSettings, '~/get_area_settings', self.srv_get_area_settings)
        # Path (channel) metadata of navigation areas, JSON {points, width_m}
        # in map metres (reuses the area-settings service types).
        srv(SetAreaSettings, '~/set_area_channel', self.srv_set_area_channel)
        srv(GetAreaSettings, '~/get_area_channel', self.srv_get_area_channel)
        # Terrain memory actions, JSON in settings_json (area_index = area, 255 = all):
        # {"action": "keepout"|"confirm"|"dismiss"|"clear", "cluster_id": n}
        srv(SetAreaSettings, '~/terrain_action', self.srv_terrain_action)

        self.tf_buffer = None
        if tf2_ros is not None:
            self.tf_buffer = tf2_ros.Buffer()
            # Same subscriptions/QoS/callbacks as tf2_ros.TransformListener(buffer, self).
            # /tf is only read by the two lookups below, so its queue (100 deep, ~3 s) is
            # drained into the buffer right before them instead of on every message.
            self._tf_sub = sub(TFMessage, '/tf', self._on_tf,
                               QoSProfile(depth=100, durability=QoSDurabilityPolicy.VOLATILE,
                                          history=QoSHistoryPolicy.KEEP_LAST),
                               sampled=True, deliver_all=True)
            sub(TFMessage, '/tf_static', self._on_tf_static,
                QoSProfile(depth=100, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                           history=QoSHistoryPolicy.KEEP_LAST))

        self.publish_filter_info()
        self.load_dock()
        self.load_settings()
        self.load_areas(startup=True)
        self.publish_settings()
        self.create_timer(max(0.1, float(self.p('mow_progress_publish_period_s'))),
                          self._locked(self.on_progress_timer))
        if self.p('terrain_enabled'):
            self.load_terrain()
            self.publish_terrain()
            self.create_timer(max(1.0, float(self.p('terrain_publish_period_s'))),
                              self._locked(self.on_terrain_publish_timer))
            self.create_timer(max(5.0, float(self.p('terrain_save_period_s'))),
                              self._locked(self.on_terrain_save_timer))
        self._pump.start()
        self.get_logger().info('map_server_node up: %d areas from %s, dock %s'
                               % (len(self.store.areas), self.areas_path,
                                  'set' if self.dock else 'unset'))

    def p(self, name):
        return self.get_parameter(name).value

    def _locked(self, fn):
        @functools.wraps(fn)
        def wrapper(*args):
            with self._lock:
                return fn(*args)
        return wrapper

    def _locked_service(self, srv_type, name, fn):
        return self.create_service(srv_type, name, self._locked(fn))

    def destroy_node(self):
        self._pump.stop()
        return super().destroy_node()

    def _on_tf(self, msg):
        for transform in msg.transforms:
            self.tf_buffer.set_transform(transform, 'default_authority')

    def _on_tf_static(self, msg):
        for transform in msg.transforms:
            self.tf_buffer.set_transform_static(transform, 'default_authority')

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
        corridor = self.dock_corridor()
        if corridor is not None:
            polys.append(corridor.polygon())
        robot = self.return_corridor_robot_xy(polys)
        if robot is not None:
            polys.append([robot])
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
        nav_mask = core.build_nav_mask(self.store.areas, spec,
                                       float(self.p('nav_margin_m')),
                                       float(self.p('nav_obstacle_margin_m')),
                                       corridor, float(self.p('nav_soft_band_m')))
        self.base_nav = (spec, nav_mask)
        ret = None
        self.return_anchor = None
        if robot is not None and not core.is_free_at(nav_mask, spec, robot[0], robot[1]):
            self.return_anchor = robot
            ret = core.return_corridor(robot[0], robot[1], nav_mask, spec,
                                       float(self.p('nav_margin_m')),
                                       float(self.p('return_corridor_max_m')))
            if ret is not None and not ret.connected:
                self.get_logger().warn(
                    'return corridor: robot (%.2f, %.2f) is %.2f m from the navigable map '
                    '(> return_corridor_max_m %.1f): no corridor, Nav2 cannot plan back'
                    % (robot[0], robot[1], ret.gap_m, float(self.p('return_corridor_max_m'))),
                    throttle_duration_sec=30.0)
                ret = None
            if ret is not None:
                nav_mask = core.build_nav_mask(self.store.areas, spec,
                                               float(self.p('nav_margin_m')),
                                               float(self.p('nav_obstacle_margin_m')),
                                               corridor, float(self.p('nav_soft_band_m')),
                                               ret)
        if (ret is not None) != self.return_active:
            self.get_logger().info(
                'return corridor %s' % ('freed: (%.2f, %.2f) -> (%.2f, %.2f), %.2f m'
                                        % (ret.path[0] + ret.path[-1] + (ret.gap_m,))
                                        if ret is not None else 'cleared'))
        self.return_active = ret is not None
        self.publish_return_corridor(ret)
        self.nav_mask_pub.publish(self.grid_msg(spec, nav_mask))
        if spec != self.spec:
            self.progress.resize(spec)
            self.spec = spec
        self.progress_dirty = True
        self.publish_progress()
        self.get_logger().info('keepout mask %dx%d @ %.2f m, origin (%.2f, %.2f), %.0f ms; '
                               'free cells: mowing %d, nav %d'
                               % (spec.width, spec.height, spec.resolution, spec.origin_x,
                                  spec.origin_y, (time.monotonic() - t0) * 1e3,
                                  int((mask == 0).sum()), int((nav_mask == 0).sum())))
        if self.p('terrain_enabled') and getattr(self, 'terrain', None) is not None \
                and set(self.terrain) != {a.name for a in self.store.areas if not a.is_navigation}:
            self.load_terrain()
            self.terrain_dirty = True
        if replan:
            self.replan_pub.publish(Bool(data=True))

    def return_corridor_robot_xy(self, polys):
        """Fresh robot (x, y) for the return corridor, or None (disabled, no
        areas, stale pose, or implausibly far from the map so the grid would
        explode)."""
        if not self.p('return_corridor_enabled') or not self.store.areas:
            return None
        if self.pose is None or time.monotonic() - self.pose_time > 2.0:
            return None
        x, y = self.pose[0], self.pose[1]
        bb = core.bounding_box(polys)
        if bb is None:
            return None
        dx = max(bb[0] - x, 0.0, x - bb[2])
        dy = max(bb[1] - y, 0.0, y - bb[3])
        if math.hypot(dx, dy) > float(self.p('return_corridor_max_m')) + 5.0:
            return None
        return (x, y)

    def publish_return_corridor(self, ret):
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        if ret is not None:
            msg.polygon.points = [Point32(x=float(x), y=float(y), z=0.0)
                                  for x, y in ret.polygon()]
        self._return_corridor_poly = ret.polygon() if ret is not None else None
        self._refresh_corridor_polys()
        self.return_corridor_pub.publish(msg)

    def check_return_corridor(self, x, y, now):
        """Rate-limited from on_odom: rebuild the nav mask when the robot left
        the free set (or moved > return_corridor_rebuild_m while off it), and
        once more when it is back on a free cell so the corridor is cleared."""
        if not self.p('return_corridor_enabled') or self.base_nav is None:
            return
        if now - self.last_return_check < float(self.p('return_corridor_check_period_s')):
            return
        self.last_return_check = now
        spec, base = self.base_nav
        if core.is_free_at(base, spec, x, y):
            if self.return_active:
                self.rebuild(replan=False)
            return
        a = self.return_anchor
        if a is None or math.hypot(x - a[0], y - a[1]) > \
                float(self.p('return_corridor_rebuild_m')):
            self.rebuild(replan=False)

    def dock_corridor(self):
        """Compute the dock corridor and publish its outline (empty polygon when
        disabled / no dock)."""
        corridor = None
        if self.dock is not None and self.p('dock_corridor_enabled'):
            corridor = core.dock_corridor(self.dock, self.store.areas,
                                          float(self.p('approach_distance')),
                                          float(self.p('nav_margin_m')),
                                          float(self.p('dock_corridor_max_m')))
            if not corridor.connected and self.store.areas:
                self.get_logger().warn(
                    'dock corridor: nearest area is %.2f m from the approach pose (> '
                    'dock_corridor_max_m %.2f); only the dock->approach capsule is free'
                    % (corridor.gap_m, float(self.p('dock_corridor_max_m'))))
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        if corridor is not None:
            msg.polygon.points = [Point32(x=float(x), y=float(y), z=0.0)
                                  for x, y in corridor.polygon()]
        self._dock_corridor_poly = corridor.polygon() if corridor is not None else None
        self._refresh_corridor_polys()
        self.corridor_pub.publish(msg)
        return corridor

    def _refresh_corridor_polys(self):
        """Polygons that count as 'inside' for the boundary check besides the areas."""
        polys = []
        for poly in (getattr(self, '_dock_corridor_poly', None),
                     getattr(self, '_return_corridor_poly', None)):
            if poly:
                polys.append(poly)
        self._corridor_polys = polys

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
        info.filter_mask_topic = '/nav_keepout_mask'  # Nav2 uses the nav mask
        info.base = 0.0
        info.multiplier = 1.0
        self.info_pub.publish(info)

    def publish_progress(self):
        self.progress_pub.publish(self.grid_msg(self.spec, self.progress.data))
        self.progress_dirty = False

    def publish_dock(self):
        self.dock_measured_pub.publish(
            Bool(data=bool(self.dock is not None and self.dock.measured)))
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
        before = [(a.name, list(a.polygon)) for a in self.store.areas]
        self.store.load(areas)
        self.sync_settings(before, startup=startup)
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
        if dock is not None and not dock.measured:
            self.get_logger().warn(
                'dock pose (%.2f, %.2f, yaw %.2f) is NOT measured (placeholder): docking will '
                'not reverse blind; drive onto the dock and press "Set docking point"'
                % (dock.x, dock.y, dock.yaw))
        if dock is not None and dock.heading_quality_at_set in core.HEADING_QUALITY_CORRECTABLE:
            self.get_logger().warn(
                'dock yaw was captured with a %s-quality heading (offset %s deg) and was not '
                'auto-corrected before this restart; the correction is not applied across a '
                'restart - re-set the docking point after a 2 m straight drive'
                % (dock.heading_quality_at_set, dock.heading_offset_deg_at_set))
        self.publish_dock()

    # ------------------------------------------------------------------
    # subscriptions
    # ------------------------------------------------------------------
    def on_status(self, msg, receipt=None):
        self.status = msg
        self.status_time = time.monotonic() if receipt is None else receipt

    def on_gnss(self, msg, receipt=None):
        self.gnss = msg
        self.gnss_time = time.monotonic() if receipt is None else receipt

    def on_heading_status(self, msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(st, dict):
            return
        self.heading = st
        self.maybe_autocorrect_dock_yaw()

    def maybe_autocorrect_dock_yaw(self):
        """Rotate a dock yaw captured under a low-quality heading offset once the aligner
        has a good COG offset (see core.dock_yaw_autocorrect). Once per set."""
        if not self._dock_autocorrect_armed or not self.p('dock_yaw_autocorrect'):
            return
        st = self.heading or {}
        new_yaw = core.dock_yaw_autocorrect(self.dock, st.get('offset_deg'), st.get('quality'),
                                            st.get('source'))
        if new_yaw is None:
            return
        self._dock_autocorrect_armed = False
        old_yaw, old_off = self.dock.yaw, self.dock.heading_offset_deg_at_set
        self.dock.yaw = float(new_yaw)
        self.dock.heading_quality_at_set = core.HEADING_QUALITY_CORRECTED
        self.dock.heading_offset_deg_at_set = float(st['offset_deg'])
        self.get_logger().warn(
            'dock yaw auto-corrected: %.1f -> %.1f deg (heading offset at set %.1f deg, '
            'quality low; COG offset now %.1f deg, quality %s)'
            % (math.degrees(old_yaw), math.degrees(new_yaw), old_off,
               float(st['offset_deg']), st.get('quality')))
        self.store_dock()
        self.rebuild()

    def store_dock(self):
        """Publish + persist self.dock (dock_pose.yaml and robot yaml)."""
        self.publish_dock()
        try:
            core.save_dock_file(self.dock_path, self.dock)
        except OSError as exc:
            self.get_logger().warn('dock pose applied but not saved: %s' % exc)
        if self.p('robot_yaml_path'):
            if not core.update_robot_yaml_dock_pose(self.p('robot_yaml_path'), self.dock.x,
                                                    self.dock.y, self.dock.yaw):
                self.get_logger().warn('could not update %s' % self.p('robot_yaml_path'))

    def on_tracker(self, msg):
        self.tracker_snapshot = list(msg.obstacles)

    def is_cutting(self):
        self._pump.poll((self._status_sub,))
        return (self.status is not None and self.status_time is not None
                and time.monotonic() - self.status_time <= float(self.p('status_max_age_s'))
                and bool(self.status.is_cutting))

    def get_blade_offset(self, child_frame):
        if self.blade_offset is not None:
            return self.blade_offset
        if self.tf_buffer is not None and child_frame:
            self._pump.poll((self._tf_sub,))
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

        self.check_return_corridor(x, y, now)
        if self._imu_sub is not None:
            self.terrain_sample(x, y, yaw, msg, now)

        rate = float(self.p('boundary_check_rate_hz'))
        if self.store.areas and (rate <= 0 or now - self.last_boundary_check >= 1.0 / rate):
            self.last_boundary_check = now
            self.boundary.soft_margin = float(self.p('soft_boundary_margin_m'))
            self.boundary.lethal_margin = float(self.p('lethal_boundary_margin_m'))
            self.boundary.debounce_samples = int(self.p('boundary_debounce_samples'))
            inside, dist = core.robot_area_status(x, y, self.store.areas)
            if not inside and getattr(self, '_corridor_polys', None):
                # The dock corridor and a return corridor are legitimate places
                # to be (docked, undocking, coming home): never a violation.
                if any(core.point_in_polygon(x, y, poly) for poly in self._corridor_polys):
                    inside, dist = True, 0.0
            soft, lethal = self.boundary.update(inside, dist)
            self.boundary_pub.publish(Bool(data=soft))
            self.lethal_pub.publish(Bool(data=lethal))
            if lethal and not self._lethal_prev:
                self.terrain_incident('boundary_lethal', x, y, '%.2f m outside' % dist)
            elif soft and not self._soft_prev and not lethal:
                self.terrain_incident('boundary', x, y, '%.2f m outside' % dist)
            self._soft_prev, self._lethal_prev = soft, lethal
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
        self.settings_area_added(a.name, _poly_from_msg(a.area))
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
        # Keep the area settings: the GUI edits areas by clear_map + add_area
        # of every area. An area re-added under a new name with the same
        # polygon takes its settings along (rename); entries whose area is not
        # back after area_settings_prune_delay_s are dropped (delete).
        self._cleared_areas = [(a.name, list(a.polygon)) for a in self.store.areas]
        self.schedule_settings_prune()
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

    # ------------------------------------------------------------------
    # per-area mowing settings
    # ------------------------------------------------------------------
    def area_names(self):
        return [a.name for a in self.store.areas]

    def load_settings(self):
        try:
            self.settings, warnings = aset.load_file(self.settings_path)
        except (OSError, UnicodeDecodeError) as exc:
            self.get_logger().error('load %s failed: %s' % (self.settings_path, exc))
            self.settings, warnings = aset.AreaSettingsStore(), []
        for w in warnings:
            self.get_logger().warn('%s: %s' % (self.settings_path, w))
        self.get_logger().info('area settings: %d area(s) with own values, defaults %s (%s)'
                               % (len(self.settings.areas), self.settings.defaults or 'built-in',
                                  self.settings_path))

    def save_settings(self):
        try:
            aset.save_file(self.settings_path, self.settings)
        except OSError as exc:
            self.get_logger().warn('area settings applied live but save failed: %s' % exc)

    def publish_settings(self):
        self.settings_pub.publish(String(data=self.settings.snapshot_json(self.area_names())))

    def sync_settings(self, before, startup=False):
        """After the area list was replaced (load_areas): carry settings of
        renamed areas over (same polygon), drop entries of deleted areas."""
        after = [(a.name, list(a.polygon)) for a in self.store.areas]
        changed = False
        for old, new in self.settings.carry_over(before, after):
            self.get_logger().info("area settings moved '%s' -> '%s' (renamed)" % (old, new))
            changed = True
        for name in self.settings.prune([n for n, _ in after]):
            self.get_logger().info("area settings of '%s' dropped (area deleted)" % name)
            changed = True
        if changed:
            self.save_settings()
        if not startup:
            self.publish_settings()

    def settings_area_added(self, name, polygon):
        if name not in self.settings.areas:
            current = set(self.area_names())
            for old, old_poly in self._cleared_areas:
                if old != name and old in self.settings.areas and old not in current \
                        and aset.polygons_match(old_poly, polygon):
                    self.settings.rename(old, name)
                    self.get_logger().info("area settings moved '%s' -> '%s' (renamed)"
                                           % (old, name))
                    self.save_settings()
                    break
        self.publish_settings()

    def schedule_settings_prune(self):
        if self._prune_timer is not None:
            self._prune_timer.cancel()
            self.destroy_timer(self._prune_timer)
        self._prune_timer = self.create_timer(
            max(0.1, float(self.p('area_settings_prune_delay_s'))),
            self._locked(self.on_settings_prune))

    def on_settings_prune(self):
        if self._prune_timer is not None:
            self._prune_timer.cancel()
            self.destroy_timer(self._prune_timer)
            self._prune_timer = None
        self._cleared_areas = []
        dropped = self.settings.prune(self.area_names())
        for name in dropped:
            self.get_logger().info("area settings of '%s' dropped (area deleted)" % name)
        if dropped:
            self.save_settings()
            self.publish_settings()

    def srv_set_area_settings(self, req, res):
        idx = int(req.area_index)
        ok, msg, update = aset.parse_json(req.settings_json)
        if not ok:
            res.success, res.message = False, msg
            return res
        if idx == aset.DEFAULTS_INDEX:
            self.settings.set_defaults(update)
            target = 'defaults'
        elif idx < len(self.store.areas):
            name = self.store.areas[idx].name
            self.settings.set_area(name, update)
            target = "area %d '%s'" % (idx, name)
        else:
            res.success = False
            res.message = 'no area %d (%d areas; 255 = defaults)' % (idx, len(self.store.areas))
            return res
        self.save_settings()
        self.publish_settings()
        eff = self.settings.effective_defaults() if idx == aset.DEFAULTS_INDEX \
            else self.settings.effective(self.store.areas[idx].name)
        res.success = True
        res.message = '%s: %s' % (target, json.dumps(eff, sort_keys=True))
        self.get_logger().info('set_area_settings %s' % res.message)
        return res

    def srv_set_area_channel(self, req, res):
        idx = int(req.area_index)
        ok, msg, points, width = core.parse_channel_json(req.settings_json)
        if ok:
            ok, msg = self.store.set_channel(idx, points, width)
        res.success, res.message = ok, msg
        if ok:
            self.persist_best_effort('set_area_channel')
            self.get_logger().info('set_area_channel: %s' % msg)
        else:
            self.get_logger().warn('set_area_channel rejected: %s' % msg)
        return res

    def srv_get_area_channel(self, req, res):
        idx = int(req.area_index)
        if idx >= len(self.store.areas):
            res.success, res.settings_json = False, '{}'
            return res
        res.success = True
        res.settings_json = core.channel_to_json(self.store.areas[idx])
        return res

    def srv_get_area_settings(self, req, res):
        idx = int(req.area_index)
        if idx == aset.DEFAULTS_INDEX:
            eff = self.settings.effective_defaults()
        elif idx < len(self.store.areas):
            eff = self.settings.effective(self.store.areas[idx].name)
            # Derived, read-only: the slope-aware swath angle of this area (null =
            # keep the planner's own). The mission uses it only while
            # mow_angle_deg is auto (-1) and slope_mode != off.
            eff = dict(eff)
            eff['slope_mow_angle_deg'], eff['slope_angle_why'] = self.terrain_mow_angle(
                self.store.areas[idx].name, eff)
        else:
            res.success = False
            res.settings_json = '{}'
            return res
        res.success = True
        res.settings_json = json.dumps(eff, sort_keys=True)
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
        self._pump.poll((self._status_sub, self._gnss_sub))
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
        # The on-dock / RTK gates protect a pose taken FROM THE ROBOT. An
        # operator-supplied pose (map drag, typed heading: yaw_source REQUEST
        # without use_gps_position) needs neither: the robot may be anywhere.
        from_robot = bool(req.use_gps_position) or \
            req.yaw_source != SetDockingPoint.Request.REQUEST
        why = self.dock_gate_failure() if from_robot else None
        if why:
            self.get_logger().warn('set_docking_point rejected: ' + why)
            res.success = False
            return res
        old = self.dock
        heading_at_set = None                # (offset_deg, quality) when yaw = fused heading
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
            measured = True
        elif req.yaw_source == SetDockingPoint.Request.REQUEST:
            q = req.docking_pose.orientation
            yaw = core.yaw_from_quaternion(q.x, q.y, q.z, q.w)
            measured = True                  # operator-set heading (map drag)
        elif old is not None and old.measured:  # PRESERVE a measured yaw
            yaw, measured = old.yaw, True
        elif req.use_gps_position and self.recent_poses:
            # PRESERVE but the stored yaw is the never-measured placeholder: the robot sits
            # on the dock, so its fused heading (heading_aligner COG/file, never seeded from
            # an untrusted dock yaw) IS the dock yaw. Not circular while the old pose is
            # unmeasured, because heading_aligner only seeds from a measured dock.
            yaw = core.yaw_circular_mean([p[3] for p in self.recent_poses])
            measured = True
            st = self.heading or {}
            heading_at_set = (st.get('offset_deg'), str(st.get('quality') or 'none'))
            if core.heading_quality_low(heading_at_set[1]):
                # Accepted anyway (the operator is on the dock now); the yaw is rotated
                # automatically once the aligner gets a good COG offset.
                self.get_logger().warn(
                    'set_docking_point: heading quality is %s (offset %s deg, source %s): '
                    'the captured dock yaw may be wrong - drive 2 m straight first; it '
                    'will be auto-corrected after the next good COG alignment'
                    % (heading_at_set[1], heading_at_set[0], st.get('source')))
            self.get_logger().info('set_docking_point: stored dock yaw was a placeholder; '
                                   'captured the docked heading %.1f deg from %s'
                                   % (math.degrees(yaw), self.p('odom_topic')))
        else:  # PRESERVE, nothing measured to preserve
            yaw = old.yaw if old is not None else 0.0
            measured = False
        outline = old.outline if old is not None and old.outline else self.default_dock_outline()
        self.dock = core.DockPose(float(x), float(y), float(yaw), outline, measured=measured)
        if heading_at_set is not None:
            off, qual = heading_at_set
            self.dock.heading_offset_deg_at_set = None if off is None else float(off)
            self.dock.heading_quality_at_set = qual
        elif (req.yaw_source == SetDockingPoint.Request.PRESERVE and old is not None
              and old.measured):
            # yaw preserved: keep its provenance (and a pending auto-correct)
            self.dock.heading_offset_deg_at_set = old.heading_offset_deg_at_set
            self.dock.heading_quality_at_set = old.heading_quality_at_set
        if heading_at_set is not None or req.yaw_source != SetDockingPoint.Request.PRESERVE:
            # a new yaw: arm the auto-correct only for a fused-heading capture at low quality
            self._dock_autocorrect_armed = (
                heading_at_set is not None and self.dock.heading_offset_deg_at_set is not None
                and self.dock.heading_quality_at_set in core.HEADING_QUALITY_CORRECTABLE)
        self.store_dock()
        self.get_logger().info('docking point set: (%.3f, %.3f) yaw %.3f rad measured=%s'
                               % (self.dock.x, self.dock.y, self.dock.yaw, self.dock.measured))
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
            self._pump.poll((self._tracker_sub,))
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
        self._pump.poll((self._tf_sub,))
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

    # ------------------------------------------------------------------
    # terrain memory (mower_map.terrain; docs/terrain_aware_planning.md)
    # ------------------------------------------------------------------
    def terrain_dir(self):
        return self.p('terrain_dir') or os.path.dirname(os.path.abspath(self.areas_path))

    def terrain_params(self):
        return {'resolution': self.resolution,
                'bad_half_life_days': float(self.p('terrain_bad_half_life_days')),
                'slope_half_life_days': float(self.p('terrain_slope_half_life_days')),
                'cluster_radius_m': float(self.p('terrain_cluster_radius_m')),
                'keepout_margin_m': float(self.p('terrain_keepout_margin_m'))}

    def load_terrain(self):
        """(Re)load the memory of every mowing area; areas that vanished are
        saved and dropped, new ones loaded from disk (or started empty)."""
        names = {a.name: a for a in self.store.areas if not a.is_navigation}
        for name in list(self.terrain):
            if name not in names:
                self.save_terrain_area(self.terrain.pop(name))
        now = time.time()
        for name, area in names.items():
            cur = self.terrain.get(name)
            if cur is not None and cur.polygon == [tuple(p) for p in area.polygon]:
                continue
            if cur is not None:
                self.save_terrain_area(cur)
            t = terr.AreaTerrain.load(self.terrain_dir(), name, area.polygon,
                                      self.terrain_params(), now)
            if getattr(t, 'load_error', None):
                self.get_logger().warn('terrain %s: unreadable (%s), starting empty'
                                       % (name, t.load_error))
            t.decay(now)
            self.terrain[name] = t
        self.terrain_dirty = True

    def save_terrain_area(self, t):
        try:
            t.save(self.terrain_dir())
        except OSError as exc:
            self.get_logger().warn('terrain %s: save failed: %s' % (t.name, exc),
                                   throttle_duration_sec=60.0)

    def on_terrain_save_timer(self):
        now = time.time()
        for t in self.terrain.values():
            t.decay(now)
            if t.dirty:
                self.save_terrain_area(t)

    def on_terrain_publish_timer(self):
        if self.terrain_dirty:
            self.publish_terrain()

    def terrain_area_at(self, x, y):
        """AreaTerrain of the mowing area containing (x, y), else the one whose
        raster (area + margin) contains it, else None."""
        best = None
        for a in self.store.areas:
            if a.is_navigation or a.name not in self.terrain:
                continue
            if core.point_in_polygon(x, y, a.polygon):
                return self.terrain[a.name]
            if best is None and self.terrain[a.name].raster.contains(x, y):
                best = self.terrain[a.name]
        return best

    def on_imu(self, msg, receipt=None):
        q = msg.orientation
        self.imu = terr.rpy_from_quaternion(q.x, q.y, q.z, q.w)[:2] + (msg.angular_velocity.z,)
        self.imu_time = time.monotonic() if receipt is None else receipt

    def terrain_sample(self, x, y, yaw, odom, now):
        """One tilt sample per terrain_sample_min_move_m of travel, while moving
        straight-ish on the ground (not docked / charging, not lifted)."""
        last = self._terrain_last_xy
        if last is not None and math.hypot(x - last[0], y - last[1]) < \
                float(self.p('terrain_sample_min_move_m')):
            return
        dt = 0.0 if self._terrain_last_t is None else min(2.0, now - self._terrain_last_t)
        self._terrain_last_xy, self._terrain_last_t = (x, y), now
        if last is None:
            return
        st = self.status
        if st is not None and (getattr(st, 'is_charging', False) or
                               getattr(st, 'lift_triggered', False)):
            return
        self._pump.poll((self._imu_sub,))
        if self.imu is None or now - self.imu_time > float(self.p('terrain_imu_max_age_s')):
            return
        roll, pitch, wz = self.imu
        if abs(wz) > float(self.p('terrain_tilt_max_yaw_rate')):
            return
        roll -= math.radians(float(self.p('terrain_roll_offset_deg')))
        pitch -= math.radians(float(self.p('terrain_pitch_offset_deg')))
        lim = math.radians(float(self.p('terrain_tilt_max_deg')))
        if abs(roll) > lim or abs(pitch) > lim:
            return
        t = self.terrain_area_at(x, y)
        if t is None:
            return
        gx, gy = terr.tilt_to_gradient(roll, pitch, yaw)
        if t.raster.add_tilt(x, y, gx, gy):
            t.raster.add_seen(x, y, dt)
            t.dirty = True
            self.terrain_dirty = True

    def on_dig_stall(self, msg):
        latched = bool(msg.data)
        if latched and not self._dig_latched and self.pose is not None:
            self.terrain_incident('dig_stall', self.pose[0], self.pose[1], '/dig_stall latched')
        self._dig_latched = latched

    def on_incident(self, msg):
        """/mission/incident JSON {kind, x?, y?, detail?}; pose defaults to the robot."""
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(d, dict):
            return
        x, y = d.get('x'), d.get('y')
        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
            if self.pose is None:
                return
            x, y = self.pose[0], self.pose[1]
        self.terrain_incident(str(d.get('kind', 'unknown')), float(x), float(y),
                              str(d.get('detail', '')))

    def terrain_incident(self, kind, x, y, detail=''):
        if not self.p('terrain_enabled'):
            return None
        t = self.terrain_area_at(x, y)
        if t is None:
            return None
        inc = t.add_incident(kind, x, y, time.time(), detail)
        self.terrain_dirty = True
        self.get_logger().info('terrain incident %s #%d in %s at (%.2f, %.2f): %s'
                               % (kind, inc['id'], t.name, x, y, detail))
        return inc

    def terrain_mow_angle(self, name, eff):
        t = self.terrain.get(name) if self.p('terrain_enabled') else None
        mode = str(eff.get('slope_mode', 'off'))
        if mode == 'off':
            return None, 'slope_mode off'
        if t is None:
            return None, 'no terrain memory'
        inside = self._area_inside_mask(name, t)
        axis = terr.slope_axis(t.raster, inside, min_cells=t.p['min_slope_cells'],
                               min_coverage=t.p['min_slope_coverage'])
        return terr.choose_mow_angle(axis, mode, float(eff.get('slope_contour_above_deg', 10.0)),
                                     float(self.p('terrain_slope_min_deg')))

    def _area_inside_mask(self, name, t):
        area = next((a for a in self.store.areas if a.name == name), None)
        if area is None:
            return None
        r = t.raster
        spec = core.GridSpec(r.origin_x, r.origin_y, r.width, r.height, r.resolution)
        cx, cy = spec.cell_centres()
        return core.points_in_polygon(cx.ravel(), cy.ravel(),
                                      area.polygon).reshape(r.height, r.width)

    def terrain_summary(self):
        out = {'version': terr.SCHEMA_VERSION, 'stamp': time.time(), 'areas': []}
        for idx, a in enumerate(self.store.areas):
            t = self.terrain.get(a.name)
            if a.is_navigation or t is None:
                continue
            eff = self.settings.effective(a.name)
            s = t.summary(str(eff.get('slope_mode', 'off')),
                          float(eff.get('slope_contour_above_deg', 10.0)),
                          self._area_inside_mask(a.name, t))
            s['area_index'] = idx
            out['areas'].append(s)
        return out

    def publish_terrain(self):
        """Traction grid (score 0..100, -1 never driven and no incident), the nav
        cost grid and the JSON summary, all on the keepout-mask grid."""
        import numpy as np
        spec = self.spec
        score = np.full((spec.height, spec.width), -1, np.int16)
        cost = np.zeros((spec.height, spec.width), np.int16)
        for t in self.terrain.values():
            sc = t.raster.traction_score()
            known = np.where((t.raster.seen > 0) | (sc >= 1.0), np.round(sc), -1)
            terr.stamp_into(score, (spec.origin_x, spec.origin_y), spec.resolution, t.raster,
                            known)
            terr.stamp_into(cost, (spec.origin_x, spec.origin_y), spec.resolution, t.raster,
                            terr.traction_cost(sc, float(self.p('terrain_cost_threshold')),
                                               int(self.p('terrain_cost_max'))))
        self.terrain_grid_pub.publish(self.grid_msg(spec, score))
        self.terrain_cost_pub.publish(self.grid_msg(spec, cost))
        self.terrain_summary_pub.publish(String(data=json.dumps(self.terrain_summary(),
                                                               sort_keys=True)))
        self.terrain_dirty = False

    def srv_terrain_action(self, req, res):
        try:
            d = json.loads(req.settings_json or '{}')
        except ValueError as exc:
            res.success, res.message = False, 'invalid JSON: %s' % exc
            return res
        action = d.get('action')
        cid = d.get('cluster_id')
        idx = int(req.area_index)
        areas = [(i, a) for i, a in enumerate(self.store.areas)
                 if not a.is_navigation and a.name in self.terrain
                 and (idx == aset.DEFAULTS_INDEX or i == idx)]
        if not areas:
            res.success, res.message = False, 'no terrain memory for area %d' % idx
            return res
        if action == 'clear':
            for _, a in areas:
                self.terrain[a.name].clear()
            self.terrain_dirty = True
            self.on_terrain_save_timer()
            self.publish_terrain()
            res.success, res.message = True, 'terrain traction memory cleared'
            return res
        if action not in ('keepout', 'confirm', 'dismiss') or not isinstance(cid, int):
            res.success = False
            res.message = 'action must be keepout|confirm|dismiss with an int cluster_id, or clear'
            return res
        for i, a in areas:
            t = self.terrain[a.name]
            cl = next((c for c in t.clusters() if c['id'] == cid), None)
            if cl is None:
                continue
            if action == 'keepout':
                name = 'terrain #%d (%s)' % (cid, ','.join(sorted(cl['kinds'])))
                ok, msg = self.store.add_obstacle(i, cl['hull'], name, core.SOURCE_DIG)
                if not ok:
                    res.success, res.message = False, 'keep-out rejected: ' + msg
                    return res
                t.set_status(cl['ids'], 'keepout')
                self.rebuild()
                self.persist_best_effort('terrain keepout')
                msg = 'cluster %d -> keep-out in area %d (%d points)' % (cid, i, len(cl['hull']))
            else:
                t.set_status(cl['ids'], 'confirmed' if action == 'confirm' else 'dismissed')
                msg = 'cluster %d %sed' % (cid, action.rstrip('e'))
            self.save_terrain_area(t)
            self.publish_terrain()
            self.get_logger().info('terrain: ' + msg)
            res.success, res.message = True, msg
            return res
        res.success, res.message = False, 'no cluster %s' % cid
        return res


def main(args=None):
    try:  # crash records -> /userdata/ros2/crashes (docs/crash_recovery.md)
        from mower_control.crash_record import install as _install_crash_record
        _install_crash_record('map_server_node')
    except ImportError:
        pass
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
