#!/usr/bin/env python3
# Copyright 2026 Airseekers Tron ROS 2 port contributors
# SPDX-License-Identifier: GPL-3.0-or-later
"""rclpy glue for :mod:`mower_mission.mission_fsm`.

Runs as node ``behavior_tree_node`` so its private names match what the
MowgliNext GUI hard-codes (``/behavior_tree_node/high_level_control`` ...).
All decisions are made by the pure-Python FSM; this module converts ROS
messages into its input snapshot and executes the effects it returns:

* subscriptions only update :class:`Inputs` under the lock;
* a 10 Hz periodic thread (sub_pump.PeriodicRunner) calls :meth:`MissionFSM.tick`;
* service callbacks call :meth:`command` / :meth:`start_in_area` / :meth:`clear_resume`;
* actions and services are started from short-lived worker threads that
  ``wait_for_server`` with a timeout (so a missing server becomes an
  ``unavailable`` result instead of a hang) and feed their results back
  into the FSM from the executor threads (reentrant callback group on a
  MultiThreadedExecutor).
"""

import functools
import json
import math
import os
import signal
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point32, Polygon, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import BackUp, FollowPath, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from sensor_msgs.msg import BatteryState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.msg import Log
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, SetBool, Trigger

from mower_interfaces.action import Dock, Undock
from mower_interfaces.msg import MowerBaseDevStatus
from mower_interfaces.srv import ChargingControl, CutterControl, GetAreaSettings, SetAreaSettings
from mowgli_interfaces.action import PlanCoverage
from mowgli_interfaces.msg import Emergency, GnssStatus, HighLevelStatus, MapArea, Status
from mowgli_interfaces.srv import AddMowingArea, GetMowingArea, HighLevelControl, StartInArea

from mower_mission import mission_fsm as fsm_mod
from mower_mission import mow_progress
from mower_mission.resume import ResumeCursor
from mower_mission.quiet_action import QuietActionClient
from mower_mission.sub_pump import (PeriodicRunner, SubscriptionPump, flat_parser,
                                    parse_odometry)

TOPIC_DEFAULTS = {
    # inputs
    'emergency_topic': '/hardware_bridge/emergency',
    'hw_status_topic': '/hardware_bridge/status',
    'battery_topic': '/battery',
    'mower_status_topic': '/mower_base/status',
    'gnss_status_topic': '/gps/status',
    'heading_status_topic': '/heading_aligner/status',   # mower_localization heading_aligner
    'obstacle_policy_topic': '/obstacle_policy',   # mower_vision obstacle_guard (JSON, 5 Hz)
    # mower_control supervisor (latched JSON): critical_down -> EMERGENCY (docs/crash_recovery.md)
    'supervisor_status_topic': '/supervisor/status',
    'odom_topic': '/odometry/filtered_map',
    'boundary_violation_topic': '/map_server_node/boundary_violation',
    'lethal_boundary_violation_topic': '/map_server_node/lethal_boundary_violation',
    # outputs (relative names resolve under /behavior_tree_node)
    'high_level_status_topic': '~/high_level_status',
    'coverage_resume_topic': '~/coverage_resume_available',
    'recording_trajectory_topic': '~/recording_trajectory',
    'full_plan_topic': '/coverage/full_plan',
    'emergency_twist_topic': '/cmd_vel_emergency',
    'high_level_control_service': '~/high_level_control',
    'start_in_area_service': '~/start_in_area',
    'clear_coverage_resume_service': '~/clear_coverage_resume',
    'manual_blade_service': '~/manual_blade',          # SetBool, MANUAL_MOWING only
    'active_area_settings_topic': '~/active_area_settings',
    # plan preview: SetAreaSettings request, area_index = area (255 = all, 254 = clear)
    'preview_plan_service': '~/preview_plan',
    'preview_summary_topic': '~/preview_summary',     # latched String JSON
    # live mow progress for the GUI map (mow_progress.py): latched String JSON
    'mow_plan_topic': '~/mow_plan',                   # sub-path geometry, on plan change
    'mow_progress_topic': '~/mow_progress',           # cursor / percent / ETA, 1 Hz
    'mow_progress_rate_hz': 1.0,
    # clients
    'get_mowing_area_service': '/map_server_node/get_mowing_area',
    'get_area_settings_service': '/map_server_node/get_area_settings',
    'coverage_server_node': '/coverage_server',        # set_parameters target (bridge)
    'controller_server_node': '/controller_server',    # set_parameters target (Nav2)
    'obstacle_guard_node': '/obstacle_guard',          # set_parameters: obstacle_detection
    'det_range_node': '/det_range',                    # set_parameters: publish_obstacle_points
    # Nav2 "observation buffer has not been updated" WARNs -> 'sensor stale' sub_state
    # ('' = off). Taken raw and byte-searched on the 10 Hz tick (no deserialisation).
    'rosout_topic': '/rosout',
    'nav_cmd_topic': '/cmd_vel_nav',      # '' = no slow-pivot sub_state
    'add_area_service': '/map_server_node/add_area',
    'set_area_channel_service': '/map_server_node/set_area_channel',  # recorded PATHs
    'cutter_control_service': '/cutter_control',
    'cutter_off_service': '/cutter_off',
    'clear_estop_service': '/clear_estop',
    'charging_service': '/charging',
    'clear_global_costmap_service': '/global_costmap/clear_entirely_global_costmap',
    'clear_local_costmap_service': '/local_costmap/clear_entirely_local_costmap',
    'plan_coverage_action': '/plan_coverage',
    'follow_path_action': '/follow_path',
    'navigate_to_pose_action': '/navigate_to_pose',
    'dock_action': '/mower_docking/dock',
    'undock_action': '/mower_docking/undock',
    'backup_action': '/backup',
}

NODE_DEFAULTS = {
    'map_frame': 'map',
    'coverage_resume_path': '~/.ros/mower_mission/coverage_resume.txt',
    'alternate_state_path': '~/.ros/mower_mission/alternate_counts.json',
    'recording_fallback_dir': '~/.ros/mower_mission/recordings',
    'server_wait_timeout_s': 3.0,
    'status_rate_hz': 1.0,
    'zero_burst_s': 0.5,
    'zero_burst_rate_hz': 20.0,
    'use_dock_pose': False,
    'dock_pose_x': 0.0,
    'dock_pose_y': 0.0,
    'dock_pose_yaw': 0.0,
}

_STATUS_TO_OUTCOME = {
    GoalStatus.STATUS_SUCCEEDED: fsm_mod.SUCCEEDED,
    GoalStatus.STATUS_CANCELED: fsm_mod.CANCELED,
    GoalStatus.STATUS_ABORTED: fsm_mod.ABORTED,
}


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _battery_percent(pct):
    if pct is None or (isinstance(pct, float) and math.isnan(pct)):
        return None
    p = float(pct)
    if p <= 1.0:
        p *= 100.0
    return max(0.0, min(100.0, p))


def _expand(path):
    return os.path.abspath(os.path.expanduser(os.path.expandvars(path)))


class MissionNode(Node):

    def __init__(self, **kwargs):
        super().__init__('behavior_tree_node', **kwargs)
        fsm_defaults = fsm_mod.Params()
        fsm_names = [n for n in fsm_defaults.__dataclass_fields__ if n != 'dock_pose']
        for name in fsm_names:
            self.declare_parameter(name, getattr(fsm_defaults, name))
        for name, default in {**TOPIC_DEFAULTS, **NODE_DEFAULTS}.items():
            self.declare_parameter(name, default)
        gp = self.get_parameter
        self._p = {n: gp(n).value for n in list(TOPIC_DEFAULTS) + list(NODE_DEFAULTS)}
        fsm_params = {n: gp(n).value for n in fsm_names}
        if self._p['use_dock_pose']:
            fsm_params['dock_pose'] = (self._p['dock_pose_x'], self._p['dock_pose_y'],
                                       self._p['dock_pose_yaw'])
        self._resume_path = _expand(self._p['coverage_resume_path'])
        self._fallback_dir = _expand(self._p['recording_fallback_dir'])

        self._lock = threading.RLock()
        cursor = self._load_cursor()
        self._alternate_path = _expand(self._p['alternate_state_path'])
        self.fsm = fsm_mod.MissionFSM(fsm_mod.Params.from_dict(fsm_params), cursor=cursor,
                                      now=time.monotonic(),
                                      alternate_counts=self._load_alternate())
        # Battery settings are applied live (GUI Settings -> Battery pushes them).
        self.add_on_set_parameters_callback(self._on_set_params)
        self._height_percent = None     # last per-area blade height, re-sent with blade ON
        self._inflight = set()          # action tokens sent or being sent
        self._cancelled = set()
        self._handles = {}              # token -> ClientGoalHandle
        self._burst_until = 0.0
        self._shutdown_futures = []

        self._cb = ReentrantCallbackGroup()
        p = self._p

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self._hl_pub = self.create_publisher(HighLevelStatus, p['high_level_status_topic'], 10)
        self._resume_pub = self.create_publisher(Bool, p['coverage_resume_topic'], latched)
        self._traj_pub = self.create_publisher(Path, p['recording_trajectory_topic'], 10)
        self._plan_pub = self.create_publisher(Path, p['full_plan_topic'], latched)
        self._twist_pub = self.create_publisher(Twist, p['emergency_twist_topic'], 10)
        # Motion-enable latch (consumed by mower_control/cmd_vel_slew's motion gate).
        self._motion_pub = self.create_publisher(Bool, '/motion_enabled', latched)
        self._motion_enabled = None
        self._motion_pub_t = 0.0
        self._settings_pub = self.create_publisher(String, p['active_area_settings_topic'],
                                                   latched)
        self._preview_pub = self.create_publisher(String, p['preview_summary_topic'], latched)
        self._mow_plan_pub = self.create_publisher(String, p['mow_plan_topic'], latched)
        self._mow_progress_pub = self.create_publisher(String, p['mow_progress_topic'], latched)
        # terrain-memory incidents (mower_map map_server_node, docs/terrain_aware_planning.md)
        self._incident_pub = self.create_publisher(String, '/mission/incident', 20)
        # path-corridor decision on the current obstacle policy (GUI "Why stopped")
        self._decision_pub = self.create_publisher(String, '/mission/obstacle_decision', latched)
        self._decision_last = None
        self._mp_plan_id = None       # plan id last published on ~/mow_plan
        self._mp_mission = None       # identity of the mission being timed
        self._mp_t0 = None            # monotonic start of that mission
        self._mp_last = (None, 0.0)   # (plan_id, mowed_m) of the previous sample
        self._mp_session_m = 0.0      # metres mowed since the mission started
        self._mp_policy = {}          # last non-'none' obstacle policy + its time

        # Inputs bypass the executor (see sub_pump.py). Every handler only stores the latest
        # value for the FSM, which reads them on the 10 Hz tick (and in status/services), so
        # they are sampled (depth 1, newest message) right where they are consumed instead of
        # being taken at their publish rates: /mower_base/status (100 Hz) and odometry
        # (30 Hz) through the 6-thread MultiThreadedExecutor cost ~55 % of a core idle.
        self._pump = SubscriptionPump(self, 'mission_inputs')
        sub = functools.partial(self._pump.subscribe, sampled=True)
        sub(Emergency, p['emergency_topic'], self._on_emergency, 1)
        sub(Status, p['hw_status_topic'], self._on_hw_status, 1)
        sub(BatteryState, p['battery_topic'], self._on_battery, 1)
        sub(MowerBaseDevStatus, p['mower_status_topic'], self._on_base, 1,
            parser=flat_parser(MowerBaseDevStatus))
        sub(GnssStatus, p['gnss_status_topic'], self._on_gnss, 1)
        sub(Odometry, p['odom_topic'], self._on_odom, 1, parser=parse_odometry)
        sub(Bool, p['boundary_violation_topic'], self._on_boundary, 1, parser=flat_parser(Bool))
        sub(Bool, p['lethal_boundary_violation_topic'], self._on_lethal, 1,
            parser=flat_parser(Bool))
        sub(String, p['heading_status_topic'], self._on_heading, latched)
        sub(String, p['obstacle_policy_topic'], self._on_obstacle_policy, latched)
        sub(Bool, '/mower_docking/marker_in_view', self._on_marker_in_view, latched,
            parser=flat_parser(Bool))
        sub(PoseStamped, '/map_server_node/docking_pose', self._on_dock_pose, latched)
        if p['rosout_topic']:
            sub(Log, p['rosout_topic'], self._on_rosout,
                QoSProfile(depth=50, reliability=QoSReliabilityPolicy.RELIABLE),
                raw=True, deliver_all=True)
        if p['nav_cmd_topic']:
            sub(Twist, p['nav_cmd_topic'], self._on_nav_cmd, 1, sampled=True, with_receipt=True)
        if p['supervisor_status_topic']:
            sub(String, p['supervisor_status_topic'], self._on_supervisor, latched)
        self._hw_rain = self._base_rain = False
        self._hw_charging = self._base_charging = False

        cli = self.create_client
        self._srv_clients = {
            fsm_mod.SRV_GET_AREA: (cli(GetMowingArea, p['get_mowing_area_service'],
                                       callback_group=self._cb), p['get_mowing_area_service']),
            fsm_mod.SRV_ADD_AREA: (cli(AddMowingArea, p['add_area_service'],
                                       callback_group=self._cb), p['add_area_service']),
            fsm_mod.SRV_SET_CHANNEL: (cli(SetAreaSettings, p['set_area_channel_service'],
                                          callback_group=self._cb),
                                      p['set_area_channel_service']),
            fsm_mod.SRV_CLEAR_ESTOP: (cli(Empty, p['clear_estop_service'],
                                          callback_group=self._cb), p['clear_estop_service']),
            fsm_mod.SRV_CHARGING: (cli(ChargingControl, p['charging_service'],
                                       callback_group=self._cb), p['charging_service']),
            fsm_mod.SRV_GET_AREA_SETTINGS: (
                cli(GetAreaSettings, p['get_area_settings_service'], callback_group=self._cb),
                p['get_area_settings_service']),
        }
        self._param_clients = {}
        for key, pname in ((fsm_mod.PARAM_NODE_COVERAGE, 'coverage_server_node'),
                           (fsm_mod.PARAM_NODE_CONTROLLER, 'controller_server_node'),
                           (fsm_mod.PARAM_NODE_OBSTACLE_GUARD, 'obstacle_guard_node'),
                           (fsm_mod.PARAM_NODE_DET_RANGE, 'det_range_node')):
            srv_name = p[pname].rstrip('/') + '/set_parameters'
            self._param_clients[key] = (cli(SetParameters, srv_name, callback_group=self._cb),
                                        srv_name)
        self._cutter_cli = cli(CutterControl, p['cutter_control_service'], callback_group=self._cb)
        self._cutter_off_cli = cli(Trigger, p['cutter_off_service'], callback_group=self._cb)
        self._clear_costmap_clis = [
            (cli(ClearEntireCostmap, p[k], callback_group=self._cb), p[k])
            for k in ('clear_global_costmap_service', 'clear_local_costmap_service')]

        # QuietActionClient: other goals' feedback (bt_navigator ~100 Hz, controller 20 Hz)
        # is dropped without an executor dispatch (quiet_action.py).
        act = lambda t, n: (QuietActionClient(self, t, p[n], callback_group=self._cb), p[n])  # noqa
        self._actions = {
            fsm_mod.ACT_PLAN: act(PlanCoverage, 'plan_coverage_action'),
            fsm_mod.ACT_FOLLOW: act(FollowPath, 'follow_path_action'),
            fsm_mod.ACT_NAV: act(NavigateToPose, 'navigate_to_pose_action'),
            fsm_mod.ACT_DOCK: act(Dock, 'dock_action'),
            fsm_mod.ACT_UNDOCK: act(Undock, 'undock_action'),
            fsm_mod.ACT_BACKUP: act(BackUp, 'backup_action'),
        }

        srv = self.create_service
        srv(HighLevelControl, p['high_level_control_service'], self._srv_hlc,
            callback_group=self._cb)
        srv(StartInArea, p['start_in_area_service'], self._srv_start_in_area,
            callback_group=self._cb)
        srv(Trigger, p['clear_coverage_resume_service'], self._srv_clear_resume,
            callback_group=self._cb)
        srv(SetBool, p['manual_blade_service'], self._srv_manual_blade,
            callback_group=self._cb)
        srv(SetAreaSettings, p['preview_plan_service'], self._srv_preview_plan,
            callback_group=self._cb)

        # tick / status / zero-burst on one plain thread (sub_pump.PeriodicRunner) instead of
        # rclpy timers in one mutually exclusive group: same sequential semantics, but no
        # MultiThreadedExecutor wake (wait set of ~60 entities) 31 times a second while idle.
        self._periodic = PeriodicRunner(self, [
            (1.0 / max(1.0, float(self.fsm.p.tick_hz)), self._tick),
            (1.0 / max(0.1, float(p['status_rate_hz'])), self._publish_status_now),
            (1.0 / max(1.0, float(p['zero_burst_rate_hz'])), self._burst_tick),
            (1.0 / max(0.1, float(p['mow_progress_rate_hz'])), self._publish_mow_progress),
        ], 'mission_periodic')

        with self._lock:
            self._execute(self.fsm.initial_effects(time.monotonic()))
        self._pump.start()
        self._periodic.start()
        self.get_logger().info(
            'mission node up: resume file %s (%s), docking server %s' % (
                self._resume_path, 'resume available' if cursor.available else 'no resume',
                'on' if self.fsm.p.use_docking_server else 'off'))

    # ------------------------------------------------------------------
    # resume file
    # ------------------------------------------------------------------
    def _load_cursor(self):
        try:
            with open(self._resume_path, 'r', encoding='utf-8') as fh:
                cur = ResumeCursor.loads(fh.read())
        except OSError:
            cur = None
        if cur is None:
            return ResumeCursor()
        return cur

    def _save_cursor(self, text):
        try:
            os.makedirs(os.path.dirname(self._resume_path), exist_ok=True)
            tmp = self._resume_path + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as fh:
                fh.write(text)
                fh.flush()
                os.fsync(fh.fileno())
            os.replace(tmp, self._resume_path)
        except OSError as exc:
            self.get_logger().error('cannot write %s: %s' % (self._resume_path, exc))

    def _delete_cursor(self):
        for path in (self._resume_path, self._resume_path + '.tmp'):
            try:
                os.remove(path)
            except FileNotFoundError:
                pass
            except OSError as exc:
                self.get_logger().error('cannot remove %s: %s' % (path, exc))

    def _load_alternate(self):
        try:
            with open(self._alternate_path, 'r', encoding='utf-8') as fh:
                data = json.load(fh)
            return {str(k): int(v) for k, v in data.items()} if isinstance(data, dict) else {}
        except (OSError, ValueError, TypeError):
            return {}

    def _save_alternate(self, counts):
        try:
            os.makedirs(os.path.dirname(self._alternate_path), exist_ok=True)
            tmp = self._alternate_path + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as fh:
                json.dump(counts, fh, sort_keys=True)
            os.replace(tmp, self._alternate_path)
        except OSError as exc:
            self.get_logger().error('cannot write %s: %s' % (self._alternate_path, exc))

    def _save_fallback(self, points, name):
        try:
            os.makedirs(self._fallback_dir, exist_ok=True)
            path = os.path.join(self._fallback_dir, '%s_%s.txt' % (
                name.replace(' ', '_'), time.strftime('%Y%m%d_%H%M%S')))
            with open(path, 'w', encoding='utf-8') as fh:
                fh.write('# %s, map frame x y (m), implicitly closed polygon\n' % name)
                for x, y in points:
                    fh.write('%.3f %.3f\n' % (x, y))
            self.get_logger().warn('recorded polygon saved to %s' % path)
        except OSError as exc:
            self.get_logger().error('cannot save recorded polygon: %s' % exc)

    # ------------------------------------------------------------------
    # inputs
    # ------------------------------------------------------------------
    def _on_emergency(self, msg):
        with self._lock:
            i = self.fsm.inputs
            i.emergency_active = bool(msg.active_emergency)
            i.emergency_reason = msg.reason
            i.emergency_stamp = time.monotonic()

    def _on_hw_status(self, msg):
        with self._lock:
            self._hw_rain = bool(msg.rain_detected)
            self._hw_charging = bool(msg.is_charging)
            self.fsm.inputs.rain = self._hw_rain or self._base_rain
            self.fsm.inputs.is_charging = self._hw_charging or self._base_charging

    LIVE_PARAMS = ('battery_low_percent', 'battery_full_percent', 'battery_low_action',
                   'battery_max_charge_percent', 'battery_charge_hysteresis_percent')

    def _on_set_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for prm in params:
            if prm.name == 'battery_low_action' and str(prm.value).strip().lower() not in ('stop', 'dock'):
                return SetParametersResult(successful=False, reason="battery_low_action: stop|dock")
        with self._lock:
            for prm in params:
                if prm.name in self.LIVE_PARAMS:
                    setattr(self.fsm.p, prm.name, prm.value)
                    self.get_logger().info('%s -> %s (live)' % (prm.name, prm.value))
        return SetParametersResult(successful=True)

    def _on_battery(self, msg):
        with self._lock:
            self.fsm.inputs.battery_percent = _battery_percent(msg.percentage)

    def _on_base(self, msg):
        with self._lock:
            i = self.fsm.inputs
            i.docked = bool(msg.is_docking_done)
            i.is_cutting = bool(msg.is_cutting)
            i.lift = bool(msg.lift_triggered)
            i.stop_button = bool(msg.stop_triggered)
            self._base_rain = bool(msg.rain_triggered)
            self._base_charging = bool(msg.is_charging)
            i.rain = self._hw_rain or self._base_rain
            i.is_charging = self._hw_charging or self._base_charging

    def _on_heading(self, msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            st = {}
        with self._lock:
            i = self.fsm.inputs
            i.heading_aligned = bool(st.get('aligned', False))
            i.heading_source = str(st.get('source', 'none'))
            i.heading_stamp = time.monotonic()

    def _on_supervisor(self, msg):
        """/supervisor/status: a safety-critical dependency (MCU driver, cmd_vel_slew,
        twist_mux) that vanished from the graph is an emergency cause (safe stop)."""
        try:
            st = json.loads(msg.data)
            down = [str(n) for n in st.get('critical_down', [])
                    if str(n).strip('/') != self.get_name()]
        except (ValueError, AttributeError, TypeError):
            return
        with self._lock:
            self.fsm.inputs.critical_nodes_down = ', '.join(sorted(down))

    def _on_obstacle_policy(self, msg):
        """{kind: none|dynamic|static, class, distance_m, bearing_deg}; the receipt time is
        the freshness stamp (obstacle_guard republishes it at a fixed rate)."""
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(st, dict):
            return
        d = st.get('distance_m')
        if st.get('kind', 'none') != 'none':
            self._mp_policy = dict(st, wall_time=time.time())
        with self._lock:
            i = self.fsm.inputs
            i.obstacle_kind = str(st.get('kind', 'none'))
            i.obstacle_class = str(st.get('class', '') or '')
            i.obstacle_distance = float(d) if isinstance(d, (int, float)) else None
            b = st.get('bearing_deg')
            i.obstacle_bearing = float(b) if isinstance(b, (int, float)) else None
            i.obstacle_stamp = time.monotonic()

    def _on_marker_in_view(self, msg):
        with self._lock:
            self.fsm.inputs.dock_marker_in_view = bool(msg.data)

    def _on_dock_pose(self, msg):
        with self._lock:
            self.fsm.inputs.dock_pose = (msg.pose.position.x, msg.pose.position.y,
                                         _yaw_from_quat(msg.pose.orientation))

    def _on_nav_cmd(self, msg, receipt):
        with self._lock:
            self.fsm.inputs.nav_cmd = (float(msg.linear.x), float(msg.angular.z))
            self.fsm.inputs.nav_cmd_stamp = receipt

    def _on_rosout(self, data):
        if fsm_mod.parse_collision_abort(data):
            with self._lock:
                self.fsm.inputs.collision_stamp = time.monotonic()
        n = fsm_mod.parse_stereo_stale(data)
        if n is None:
            return
        with self._lock:
            self.fsm.inputs.stereo_stale_s = n
            self.fsm.inputs.stereo_stale_stamp = time.monotonic()

    def _on_gnss(self, msg):
        with self._lock:
            self.fsm.inputs.fix_type = int(msg.fix_type)
            self.fsm.inputs.gps_quality = max(0.0, min(1.0, float(msg.quality_percent) / 100.0))

    def _on_odom(self, msg):
        pose = msg.pose.pose
        with self._lock:
            self.fsm.inputs.pose = (pose.position.x, pose.position.y,
                                    _yaw_from_quat(pose.orientation))

    def _on_boundary(self, msg):
        with self._lock:
            self.fsm.inputs.boundary_violation = bool(msg.data)

    def _on_lethal(self, msg):
        with self._lock:
            self.fsm.inputs.lethal_boundary_violation = bool(msg.data)

    # ------------------------------------------------------------------
    # timers / services
    # ------------------------------------------------------------------
    def _tick(self):
        with self._lock:
            self._pump.poll()
            self._execute(self.fsm.tick(time.monotonic()))
            self._publish_motion_enabled()
            dec = json.dumps(self.fsm.obstacle_decision, sort_keys=True)
            if dec != self._decision_last:
                self._decision_last = dec
                self._decision_pub.publish(String(data=dec))

    def _publish_motion_enabled(self):
        """Latched /motion_enabled: on change, and re-asserted every 1 s."""
        en = bool(self.fsm.motion_enabled())
        now = time.monotonic()
        if en != self._motion_enabled or now - self._motion_pub_t >= 1.0:
            if en != self._motion_enabled:
                self.get_logger().info('motion_enabled -> %s (%s)' % (en, self.fsm.phase))
            self._motion_enabled = en
            self._motion_pub_t = now
            self._motion_pub.publish(Bool(data=en))

    def _burst_tick(self):
        if time.monotonic() < self._burst_until:
            self._twist_pub.publish(Twist())

    def _srv_hlc(self, req, resp):
        with self._lock:
            self._pump.poll()
            ok, fx = self.fsm.command(int(req.command), time.monotonic())
            self.get_logger().info('high_level_control(%d) -> %s [%s]' % (
                req.command, ok, self.fsm.phase))
            self._execute(fx)
            self._publish_motion_enabled()
        resp.success = bool(ok)
        return resp

    def _srv_start_in_area(self, req, resp):
        with self._lock:
            self._pump.poll()
            ok, fx = self.fsm.start_in_area(int(req.area), time.monotonic())
            self.get_logger().info('start_in_area(%d) -> %s' % (req.area, ok))
            self._execute(fx)
            self._publish_motion_enabled()
        resp.success = bool(ok)
        return resp

    def _srv_manual_blade(self, req, resp):
        with self._lock:
            self._pump.poll()
            ok, msg, fx = self.fsm.manual_blade(bool(req.data), time.monotonic())
            self._execute(fx)
        resp.success, resp.message = bool(ok), msg
        return resp

    def _srv_preview_plan(self, req, resp):
        # Planning only: no state change, no motion. The plan arrives later on
        # /coverage/full_plan + ~/preview_summary (settings_json is reserved).
        with self._lock:
            self._pump.poll()
            ok, msg, fx = self.fsm.preview_plan(int(req.area_index), time.monotonic())
            self._execute(fx)
        resp.success, resp.message = bool(ok), msg
        return resp

    def _srv_clear_resume(self, req, resp):
        with self._lock:
            self._pump.poll()
            ok, msg, fx = self.fsm.clear_resume(time.monotonic())
            self._execute(fx)
        resp.success, resp.message = bool(ok), msg
        return resp

    # ------------------------------------------------------------------
    # effects
    # ------------------------------------------------------------------
    def _execute(self, effects):
        # One failing effect must never stop the ones after it (a BladeOff may follow).
        for e in effects:
            try:
                self._execute_one(e)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error('effect %s failed: %r' % (type(e).__name__, exc))

    def _log_effect(self, e):
        # rclpy refuses one call site logging at different severities: one call per level.
        log = self.get_logger()
        if e.level == 'error':
            log.error(e.text)
        elif e.level == 'warn':
            log.warning(e.text)
        else:
            log.info(e.text)

    def _execute_one(self, e):
        f = fsm_mod
        if isinstance(e, f.Log):
            self._log_effect(e)
        elif isinstance(e, f.PublishStatus):
            self._publish_status(e.status)
        elif isinstance(e, f.BladeOn):
            self._blade(True)
        elif isinstance(e, f.BladeOff):
            self._blade(False, e.reason)
        elif isinstance(e, f.ZeroBurst):
            self._twist_pub.publish(Twist())
            self._burst_until = time.monotonic() + float(self._p['zero_burst_s'])
        elif isinstance(e, f.CancelActions):
            self._cancel_all(e.reason)
        elif isinstance(e, f.StartAction):
            self._inflight.add(e.token)
            threading.Thread(target=self._send_goal, args=(e,), daemon=True).start()
        elif isinstance(e, f.CallService):
            threading.Thread(target=self._call_service, args=(e,), daemon=True).start()
        elif isinstance(e, f.PublishPlan):
            self._plan_pub.publish(self._path(e.poses))
        elif isinstance(e, f.PublishTrajectory):
            self._traj_pub.publish(self._path([(x, y, 0.0) for x, y in e.points]))
        elif isinstance(e, f.PublishResumeAvailable):
            self._resume_pub.publish(Bool(data=bool(e.available)))
        elif isinstance(e, f.SaveResume):
            self._save_cursor(e.text)
        elif isinstance(e, f.DeleteResume):
            self._delete_cursor()
        elif isinstance(e, f.SaveRecordingFallback):
            self._save_fallback(e.points, e.name)
        elif isinstance(e, f.PublishAreaSettings):
            self._settings_pub.publish(String(data=json.dumps(e.settings, sort_keys=True)))
        elif isinstance(e, f.PublishPreviewSummary):
            self._preview_pub.publish(String(data=json.dumps(e.summary, sort_keys=True)))
        elif isinstance(e, f.SaveAlternateCounts):
            self._save_alternate(e.counts)
        elif isinstance(e, f.RecordIncident):
            self._incident_pub.publish(String(data=json.dumps(
                {'kind': e.kind, 'x': e.x, 'y': e.y, 'detail': e.detail,
                 'state': self.fsm.phase}, sort_keys=True)))

    def _publish_status_now(self):
        with self._lock:
            self._pump.poll()
            st = self.fsm.status()
        self._publish_status(st)

    def _publish_mow_progress(self):
        """~/mow_plan + ~/mow_progress for the GUI map (read-only view of the FSM)."""
        try:
            now = time.monotonic()
            with self._lock:
                m = self.fsm.mission
                if m is not self._mp_mission:
                    self._mp_mission, self._mp_session_m = m, 0.0
                    self._mp_t0 = now if m is not None else None
                    self._mp_last = (None, 0.0)
                geo = mow_progress.plan_geometry(self.fsm)
                pid = geo[0] if geo else None
                i = self.fsm.inputs
                why = {'obstacle': self._mp_policy or None,
                       'stereo_stale_s': i.stereo_stale_s if (
                           i.stereo_stale_stamp is not None
                           and now - i.stereo_stale_stamp <= 15.0) else None,
                       'boundary_violation': bool(i.boundary_violation),
                       'emergency': bool(self.fsm.status().get('emergency')),
                       'lift': bool(i.lift), 'stop_button': bool(i.stop_button),
                       'rain': bool(i.rain), 'docked': bool(i.docked),
                       'critical_nodes_down': i.critical_nodes_down or ''}
                elapsed = (now - self._mp_t0) if self._mp_t0 is not None else None
                st = mow_progress.progress(self.fsm, pid, bool(i.is_cutting), elapsed,
                                           self._mp_session_m, why)
            last_pid, last_m = self._mp_last
            if pid is not None and pid == last_pid and st['mowed_m'] > last_m:
                self._mp_session_m += st['mowed_m'] - last_m
            self._mp_last = (pid, st['mowed_m'])
            if geo is not None and pid != self._mp_plan_id:
                self._mp_plan_id = pid
                self._mow_plan_pub.publish(String(data=json.dumps(
                    {'plan_id': pid, 'area': geo[1], 'subpaths': geo[2]})))
            self._mow_progress_pub.publish(String(data=json.dumps(st, default=str)))
        except Exception as ex:  # noqa: BLE001 - GUI telemetry must never hurt the mission
            self.get_logger().warn('mow_progress: %s' % ex, throttle_duration_sec=30.0)

    def _publish_status(self, st):
        msg = HighLevelStatus()
        msg.state = int(st['state'])
        msg.state_name = st['state_name']
        msg.sub_state_name = st['sub_state_name']
        msg.current_area = int(st['current_area'])
        msg.current_path = int(st['current_path'])
        msg.current_path_index = int(st['current_path_index'])
        msg.total_swaths = int(st['total_swaths'])
        msg.completed_swaths = int(st['completed_swaths'])
        msg.skipped_swaths = int(st['skipped_swaths'])
        msg.coverage_percent = float(st['coverage_percent'])
        msg.gps_quality_percent = float(st['gps_quality_percent'])
        msg.battery_percent = float(st['battery_percent'])
        msg.is_charging = bool(st['is_charging'])
        msg.emergency = bool(st['emergency'])
        self._hl_pub.publish(msg)

    # ---- blade -----------------------------------------------------------
    def _blade(self, on, reason=''):
        if on:
            req = CutterControl.Request()
            req.cutter.enable = True
            req.cutter.direction = False
            req.cutter.speed = 0          # 0 -> driver default speed
            req.cutter.position = 0
            # Keep the per-area blade height in every blade-ON command (the
            # MCU's handling of height.enable=false is unverified).
            req.height.enable = self._height_percent is not None
            req.height.position = int(self._height_percent or 0)
            if not self._cutter_cli.service_is_ready():
                self.get_logger().error('blade ON: %s unavailable' %
                                        self._p['cutter_control_service'])
                return
            self.get_logger().info('blade ON')
            fut = self._cutter_cli.call_async(req)
            fut.add_done_callback(self._on_blade_on_done)
            return
        self.get_logger().info('blade OFF (%s)' % reason)
        if self._cutter_off_cli.service_is_ready():
            self._shutdown_futures.append(self._cutter_off_cli.call_async(Trigger.Request()))
        elif self._cutter_cli.service_is_ready():
            req = CutterControl.Request()
            req.cutter.enable = False
            req.height.enable = False
            self._shutdown_futures.append(self._cutter_cli.call_async(req))
        else:
            self.get_logger().warn('blade OFF: neither %s nor %s available' % (
                self._p['cutter_off_service'], self._p['cutter_control_service']))
        self._shutdown_futures = [f for f in self._shutdown_futures if not f.done()]

    def _on_blade_on_done(self, fut):
        try:
            ok = bool(fut.result().result)
        except Exception:  # noqa: BLE001
            ok = False
        if not ok:
            self.get_logger().warn('cutter_control refused blade ON (MCU interlock?)')

    # ---- actions ---------------------------------------------------------
    def _cancel_all(self, reason):
        with self._lock:
            tokens = list(self._inflight)
            self._cancelled.update(tokens)
            handles = [self._handles.pop(t) for t in tokens if t in self._handles]
        if tokens:
            self.get_logger().info('cancelling %d action goal(s): %s' % (len(tokens), reason))
        for h in handles:
            try:
                h.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn('cancel failed: %s' % exc)

    def _feed_action(self, token, outcome, result):
        with self._lock:
            self._inflight.discard(token)
            self._handles.pop(token, None)
            self._cancelled.discard(token)
            self._execute(self.fsm.on_action_result(token, outcome, result, time.monotonic()))

    def _send_goal(self, e):
        client, ros_name = self._actions[e.name]
        if not client.wait_for_server(timeout_sec=float(self._p['server_wait_timeout_s'])):
            self.get_logger().warn('action server %s not available after %.1fs' % (
                ros_name, float(self._p['server_wait_timeout_s'])))
            self._feed_action(e.token, fsm_mod.UNAVAILABLE,
                              {'message': '%s unavailable' % ros_name})
            return
        with self._lock:
            if e.token in self._cancelled:
                self._inflight.discard(e.token)
                self._cancelled.discard(e.token)
                return
        try:
            goal = self._make_goal(e)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('bad %s goal: %s' % (e.name, exc))
            self._feed_action(e.token, fsm_mod.REJECTED, {'message': str(exc)})
            return
        self.get_logger().info('sending %s goal' % ros_name)
        if e.name == fsm_mod.ACT_DOCK:   # relay the docking status detail into sub_state
            fut = client.send_goal_async(
                goal, feedback_callback=lambda fb, e=e: self._on_dock_feedback(e, fb))
        else:
            fut = client.send_goal_async(goal)
        fut.add_done_callback(lambda f, e=e: self._on_goal_response(e, f))

    def _on_dock_feedback(self, e, fb):
        text = str(fb.feedback.state)
        detail = text.split(': ', 1)[1] if ': ' in text else ''
        if not detail:
            return
        with self._lock:
            self._execute(self.fsm.on_action_feedback(e.token, detail, time.monotonic()))

    def _on_goal_response(self, e, fut):
        try:
            handle = fut.result()
        except Exception as exc:  # noqa: BLE001
            self._feed_action(e.token, fsm_mod.REJECTED, {'message': str(exc)})
            return
        if not handle.accepted:
            self._feed_action(e.token, fsm_mod.REJECTED, {'message': 'goal rejected'})
            return
        with self._lock:
            cancelled = e.token in self._cancelled
            if not cancelled:
                self._handles[e.token] = handle
        if cancelled:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(
            lambda f, e=e: self._on_goal_result(e, f))

    def _on_goal_result(self, e, fut):
        try:
            wrapped = fut.result()
            outcome = _STATUS_TO_OUTCOME.get(wrapped.status, fsm_mod.ABORTED)
            result = self._result_dict(e.name, wrapped.result)
        except Exception as exc:  # noqa: BLE001
            outcome, result = fsm_mod.ABORTED, {'message': str(exc)}
        self._feed_action(e.token, outcome, result)

    def _pose_stamped(self, x, y, yaw):
        ps = PoseStamped()
        ps.header.frame_id = self._p['map_frame']
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.z = math.sin(float(yaw) / 2.0)
        ps.pose.orientation.w = math.cos(float(yaw) / 2.0)
        return ps

    def _path(self, poses):
        path = Path()
        path.header.frame_id = self._p['map_frame']
        path.header.stamp = self.get_clock().now().to_msg()
        for p in poses:
            ps = self._pose_stamped(p[0], p[1], p[2] if len(p) > 2 else 0.0)
            ps.header.stamp = path.header.stamp
            path.poses.append(ps)
        return path

    @staticmethod
    def _polygon(points):
        return Polygon(points=[Point32(x=float(p[0]), y=float(p[1]), z=0.0) for p in points])

    def _make_goal(self, e):
        g = e.goal
        if e.name == fsm_mod.ACT_PLAN:
            goal = PlanCoverage.Goal()
            goal.outer_boundary = self._polygon(g['outer_boundary'])
            goal.obstacles = [self._polygon(o) for o in g['obstacles']]
            goal.mow_angle_deg = float(g['mow_angle_deg'])
            goal.perpendicular = bool(g['perpendicular'])
        elif e.name == fsm_mod.ACT_FOLLOW:
            goal = FollowPath.Goal()
            goal.path = self._path(g['poses'])
            goal.controller_id = g['controller_id']
            goal.goal_checker_id = g['goal_checker_id']
        elif e.name == fsm_mod.ACT_NAV:
            goal = NavigateToPose.Goal()
            goal.pose = self._pose_stamped(*g['pose'])
        elif e.name == fsm_mod.ACT_DOCK:
            goal = Dock.Goal()
            goal.use_vision = bool(g['use_vision'])
            goal.timeout_s = float(g['timeout_s'])
        elif e.name == fsm_mod.ACT_UNDOCK:
            goal = Undock.Goal()
            goal.distance_m = float(g['distance_m'])
            goal.speed_mps = float(g['speed_mps'])
            goal.wait_for_rtk = bool(g['wait_for_rtk'])
            goal.rtk_timeout_s = float(g['rtk_timeout_s'])
        elif e.name == fsm_mod.ACT_BACKUP:
            goal = BackUp.Goal()
            goal.target.x = float(g['distance_m'])
            goal.speed = float(g['speed_mps'])
            secs = float(g['distance_m']) / max(0.01, float(g['speed_mps'])) + 10.0
            goal.time_allowance = Duration(sec=int(secs))
        else:
            raise ValueError('unknown action %s' % e.name)
        return goal

    @staticmethod
    def _poses(path):
        return [(ps.pose.position.x, ps.pose.position.y, _yaw_from_quat(ps.pose.orientation))
                for ps in path.poses]

    def _result_dict(self, name, res):
        if name == fsm_mod.ACT_PLAN:
            return {'success': bool(res.success), 'message': res.message,
                    'drivable_subpaths': [self._poses(p) for p in res.drivable_subpaths],
                    'segments': [self._poses(p) for p in res.segments],
                    'full_path': self._poses(res.full_path),
                    'segment_types': [int(t) for t in res.segment_types],
                    'ring_count': int(res.ring_count), 'swath_count': int(res.swath_count),
                    'total_distance': float(res.total_distance)}
        if name in (fsm_mod.ACT_DOCK, fsm_mod.ACT_UNDOCK):
            return {'success': bool(res.success), 'message': res.message}
        return {}

    # ---- services ----------------------------------------------------------
    @staticmethod
    def _param_msg(name, value):
        v = ParameterValue()
        if isinstance(value, bool):
            v.type, v.bool_value = ParameterType.PARAMETER_BOOL, value
        elif isinstance(value, int):
            v.type, v.integer_value = ParameterType.PARAMETER_INTEGER, value
        elif isinstance(value, float):
            v.type, v.double_value = ParameterType.PARAMETER_DOUBLE, value
        else:
            v.type, v.string_value = ParameterType.PARAMETER_STRING, str(value)
        return Parameter(name=name, value=v)

    def _cutter_height(self, e):
        """Per-area blade height: /cutter_control with the cutter OFF and
        height.enable=true (only issued while the blade is off, in PLANNING)."""
        pct = int(e.request['percent'])
        self._height_percent = pct
        if not self._cutter_cli.wait_for_service(
                timeout_sec=float(self._p['server_wait_timeout_s'])):
            self.get_logger().warn('blade height: %s unavailable' %
                                   self._p['cutter_control_service'])
            return
        req = CutterControl.Request()
        req.cutter.enable = False
        req.height.enable = True
        req.height.position = pct
        self.get_logger().info('blade height %d mm -> %d %% (/cutter_control height)'
                               % (int(e.request['height_mm']), pct))
        fut = self._cutter_cli.call_async(req)
        fut.add_done_callback(lambda f: None)

    def _call_service(self, e):
        if e.name == fsm_mod.SRV_CUTTER_HEIGHT:
            self._cutter_height(e)
            return
        if e.name == fsm_mod.SRV_CLEAR_COSTMAPS:
            # fire-and-forget (untracked by the FSM): let the costmaps refresh
            # before a transit / follow retry or a boundary recovery.
            for client, ros_name in self._clear_costmap_clis:
                if client.wait_for_service(timeout_sec=float(self._p['server_wait_timeout_s'])):
                    client.call_async(ClearEntireCostmap.Request())
                else:
                    self.get_logger().warn('service %s not available' % ros_name)
            return
        if e.name == fsm_mod.SRV_SET_PARAMS:
            client, ros_name = self._param_clients[e.request['node']]
        else:
            client, ros_name = self._srv_clients[e.name]
        if not client.wait_for_service(timeout_sec=float(self._p['server_wait_timeout_s'])):
            self.get_logger().warn('service %s not available' % ros_name)
            self._feed_service(e.token, False, {})
            return
        try:
            req = self._make_request(e)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('bad %s request: %s' % (e.name, exc))
            self._feed_service(e.token, False, {})
            return
        fut = client.call_async(req)
        fut.add_done_callback(lambda f, e=e: self._on_service_done(e, f))

    def _make_request(self, e):
        r = e.request
        if e.name == fsm_mod.SRV_GET_AREA:
            return GetMowingArea.Request(index=int(r['index']))
        if e.name == fsm_mod.SRV_ADD_AREA:
            req = AddMowingArea.Request()
            req.area = MapArea()
            req.area.name = r['name']
            req.area.area = self._polygon(r['polygon'])
            req.area.is_navigation_area = bool(r['is_navigation_area'])
            req.is_navigation_area = bool(r['is_navigation_area'])
            return req
        if e.name == fsm_mod.SRV_SET_CHANNEL:
            return SetAreaSettings.Request(area_index=int(r['index']),
                                           settings_json=str(r['settings_json']))
        if e.name == fsm_mod.SRV_CHARGING:
            return ChargingControl.Request(enable_charging=bool(r['enable']))
        if e.name == fsm_mod.SRV_CLEAR_ESTOP:
            return Empty.Request()
        if e.name == fsm_mod.SRV_GET_AREA_SETTINGS:
            return GetAreaSettings.Request(area_index=int(r['index']))
        if e.name == fsm_mod.SRV_SET_PARAMS:
            req = SetParameters.Request()
            req.parameters = [self._param_msg(k, v) for k, v in r['params'].items()]
            return req
        raise ValueError('unknown service %s' % e.name)

    def _on_service_done(self, e, fut):
        try:
            res = fut.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('%s failed: %s' % (e.name, exc))
            self._feed_service(e.token, False, {})
            return
        resp = {}
        if e.name == fsm_mod.SRV_GET_AREA:
            a = res.area
            resp = {'success': bool(res.success), 'area': {
                'name': a.name,
                'outer': [(p.x, p.y) for p in a.area.points],
                'obstacles': [[(p.x, p.y) for p in o.points] for o in a.obstacles],
                'is_navigation_area': bool(a.is_navigation_area)}}
        elif e.name == fsm_mod.SRV_ADD_AREA:
            resp = {'success': bool(res.success)}
        elif e.name == fsm_mod.SRV_SET_CHANNEL:
            resp = {'success': bool(res.success), 'message': res.message}
        elif e.name == fsm_mod.SRV_GET_AREA_SETTINGS:
            resp = {'success': bool(res.success), 'settings_json': res.settings_json}
        elif e.name == fsm_mod.SRV_SET_PARAMS:
            bad = [(prm.name, r.reason) for prm, r in zip(
                self._make_request(e).parameters, res.results) if not r.successful]
            if bad:
                msg = '; '.join('%s: %s' % b for b in bad)
                self.get_logger().warn('%s refused: %s' % (ros_name_of(e, self), msg))
                self._feed_service(e.token, False, {'message': msg})
                return
            self.get_logger().info('%s: %s' % (ros_name_of(e, self), ', '.join(
                '%s=%r' % kv for kv in e.request['params'].items())))
        self._feed_service(e.token, True, resp)

    def _feed_service(self, token, ok, resp):
        with self._lock:
            self._execute(self.fsm.on_service_result(token, ok, resp, time.monotonic()))

    # ------------------------------------------------------------------
    def destroy_node(self):
        self._periodic.stop()
        self._pump.stop()
        return super().destroy_node()

    def shutdown_effects(self):
        """Blade off, cancel goals, zero twist; returns futures to wait for."""
        with self._lock:
            self._execute(self.fsm.shutdown(time.monotonic()))
        return [f for f in self._shutdown_futures if not f.done()]


def ros_name_of(e, node):
    return node._param_clients[e.request['node']][1]


def main(args=None):
    try:  # crash records (docs/crash_recovery.md); soft dependency on mower_control
        from mower_control.crash_record import install as _install_crash_record
        _install_crash_record('behavior_tree_node')
    except ImportError:
        pass
    # Keep the context alive on SIGINT/SIGTERM so the blade-off call can still go out.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def _term(_signum, _frame):
        raise KeyboardInterrupt
    # Explicit handlers: a process started from a non-interactive shell inherits
    # SIGINT = SIG_IGN, and then Python would never raise KeyboardInterrupt.
    signal.signal(signal.SIGINT, _term)
    signal.signal(signal.SIGTERM, _term)

    node = MissionNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        node._periodic.stop()            # no tick may race the shutdown effects
        try:
            node.get_logger().info('shutting down: blade off, cancel goals, zero velocity')
            for fut in node.shutdown_effects():
                executor.spin_until_future_complete(fut, timeout_sec=1.0)
        except Exception as exc:  # noqa: BLE001
            print('mower_mission: shutdown blade-off failed: %s' % exc)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
