# Copyright 2026 Airseekers Tron ROS 2 port contributors
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Mission logic ported from MowgliNext mowgli_behavior (GPL-3.0):
# trees/main_tree.xml (guards, start/mow/home/record/manual sequences),
# src/coverage_nodes.cpp (FollowStrip: per-sub-path FollowPath, blade-off
# transits; ours retries failures and ends in MOWING_INCOMPLETE instead of
# silently skipping), src/coverage_persistence.cpp (resume cursor),
# src/recording_nodes.cpp (10 Hz / 5 cm sampling, Douglas-Peucker, add_area),
# plus the Airseekers vendor cutter interlock (OpenCutterCheck retries).
"""Pure-Python mission state machine of the Tron ``behavior_tree_node``.

No ROS imports.  The node feeds it an input snapshot (:class:`Inputs`, kept
up to date by subscriptions), commands, a 10 Hz :meth:`MissionFSM.tick` and
the results of the actions/services it asked for.  Every call returns a
list of *effects* (:class:`BladeOn`, :class:`StartAction`, ...) which the
node executes in list order.  Upstream's ``ReactiveSequence`` of guards is
:meth:`MissionFSM._guards`, evaluated at the start of every tick.

Blade invariant (enforced here, asserted by the tests): a :class:`BladeOn`
is only ever emitted in MANUAL_MOWING, or in MOWING right before a
FollowPath goal; a :class:`BladeOff` precedes every transit, dock, undock,
stop, emergency and shutdown.
"""

import json
import re
import bisect
import math
from dataclasses import dataclass, field
from typing import Any, Optional

from mower_common import geometry as rgeo
from mower_common.phases import DOCK_PHASES, DOCKED_MOTION_PHASES, MOTION_PHASES  # noqa: F401
from mower_mission import body_boundary as bbody
from mower_mission import geometry as geo
from mower_mission import path_conditioning as pcond
from mower_mission.resume import AreaCursor, ResumeCursor, absolute_to_local, subpath_offsets

# ---------------------------------------------------------------------------
# mowgli_interfaces constants (duplicated so this module stays ROS-free)
# ---------------------------------------------------------------------------
STATE_NULL = 0          # also EMERGENCY
STATE_IDLE = 1
STATE_AUTONOMOUS = 2
STATE_RECORDING = 3
STATE_MANUAL_MOWING = 4

CMD_START = 1
CMD_HOME = 2
CMD_RECORD_AREA = 3
CMD_S2 = 4
CMD_RECORD_FINISH = 5
CMD_RECORD_CANCEL = 6
CMD_MANUAL_MOW = 7
CMD_STOP = 8
# Airseekers extension (no MowgliNext equivalent): leave the dock by a short straight
# drive and stay idle off the dock. 9 is unused by HighLevelControl.srv.
CMD_UNDOCK = 9
# Airseekers extension: like RECORD_AREA but the driven trajectory is saved as an open
# PATH (navigation band with centreline + width, as the GUI "Draw path" tool stores it).
CMD_RECORD_PATH = 10
# Airseekers extension: drive around an obstacle / no-go zone; the ring is saved as a
# permanent keepout via /map_server_node/promote_obstacle instead of a mowing area.
# 11 is unused by HighLevelControl.srv.
CMD_RECORD_OBSTACLE = 11
CMD_RESET_EMERGENCY = 254
CMD_DELETE_MAPS = 255

FIX_RTK_FIXED = 3

# heading_aligner (mower_localization) sources that count as an absolute heading
HEADING_ALIGNED_SOURCES = ('dock', 'cog', 'file_verified')
# Operator text (GUI sub_state); mower_gui_bridge diagnostics.HEADING_HINT says the same.
# 0.1 m/s leaves margin over heading_aligner min_speed (0.04 m/s, 2026-10-13: a 0.05 m/s
# joystick drive never aligned under the old 0.06 and the message did not say why).
HEADING_NOT_ALIGNED = ('heading not aligned: drive straight 1 m at 0.1 m/s or more in manual, '
                       'or start from the dock')

# MANUAL_MOWING sub_state (the GUI reads the blade state from it)
SUB_MANUAL_BLADE_ON = 'joystick, blade on'
SUB_MANUAL_BLADE_OFF = 'joystick, blade off'

# action / service names used in effects (the node maps them to ROS names)
ACT_UNDOCK = 'undock'
ACT_DOCK = 'dock'
ACT_BACKUP = 'backup'
ACT_DRIVE = 'drive_on_heading'      # Nav2 behavior_server DriveOnHeading (heading calibration)
ACT_SPIN = 'spin'                   # Nav2 behavior_server Spin (route cusp about-turn, fix 11)


def spin_target_yaw(angle, lead):
    """Relative Spin target for a pivot of ``angle`` rad that coasts ``lead`` rad after the
    Spin stops commanding (pivot_stop_lead_rad): short by the lead, never by more than half
    the angle, never past zero; ``lead`` <= 0 = ``angle`` unchanged."""
    angle = float(angle)
    lead = min(max(0.0, float(lead)), 0.5 * abs(angle))
    return math.copysign(abs(angle) - lead, angle) if lead > 0.0 else angle
ACT_PLAN = 'plan_coverage'
ACT_NAV = 'navigate_to_pose'
ACT_FOLLOW = 'follow_path'

SRV_GET_AREA = 'get_mowing_area'
SRV_ADD_AREA = 'add_area'
SRV_SET_CHANNEL = 'set_area_channel'
SRV_PROMOTE_OBSTACLE = 'promote_obstacle'   # /map_server_node/promote_obstacle
SRV_CLEAR_ESTOP = 'clear_estop'
SRV_CHARGING = 'charging'
SRV_GET_AREA_SETTINGS = 'get_area_settings'   # /map_server_node/get_area_settings
SRV_SET_PARAMS = 'set_parameters'             # request {'node': ..., 'params': {...}}
CUTTER_HEIGHT_MIN_MM = 30                    # vendor clamp, MCU height position is mm
CUTTER_HEIGHT_MAX_MM = 90
SRV_CUTTER_HEIGHT = 'cutter_height'           # /cutter_control height only, blade off
SRV_CLEAR_COSTMAPS = 'clear_costmaps'         # Nav2 clear_entirely_{global,local}_costmap
SRV_PLAN_ROUTE = 'plan_route'                 # /map_server_node/plan_route (route graph)

# set_parameters targets (the node maps them to <node>/set_parameters)
PARAM_NODE_COVERAGE = 'coverage_server'
PARAM_NODE_CONTROLLER = 'controller_server'
PARAM_NODE_OBSTACLE_GUARD = 'obstacle_guard'   # mower_vision: obstacle_detection level
PARAM_NODE_DET_RANGE = 'det_range'             # publish_obstacle_points (costmap marks)

# Per-area mowing settings (mower_map area_settings.py BUILTIN_DEFAULTS; used
# when /map_server_node/get_area_settings is unavailable or leaves a key out).
AREA_SETTINGS_DEFAULTS = {
    'cutter_height_mm': 50,
    'perimeter_laps': 2,
    'path_mode': 'zigzag',
    'mow_angle_deg': -1.0,
    'cut_speed_mps': 0.3,
    'swath_overlap_m': 0.02,
    'edge_margin_m': 0.05,      # planner boundary_inset_m (blade edge inside the line)
    # path spacing = planner operation_width. Fallback only: map_server's defaults carry the
    # robot-wide disc-derived standard width (blade_disc_mm/1000 - swath_overlap_m, 220 mm -> 0.18)
    'swath_width_m': 0.18,
    'edge_first': True,
    # soft | hard (owner 2026-10-21: every area soft for now): the planner's edge_layout
    'edge_mode': 'soft',
    # edge layout: number of edge laps (3 = stock); the rows move with them (planner edge_laps)
    'edge_laps': 3,
    'repeat': 1,
    'alternate_angle_offset_deg': 90.0,
    'route_order': 'racetrack',
    'route_spiral_size': 6,
    'min_turn_radius_m': 0.5,
    'turn_type': 'auto',
    'obstacle_detection': 'standard',   # none | standard | sensitive (obstacle_guard)
    # continuous: blade stays on through waits / retries / in-area transits and detours;
    # conservative: blade off on each of them (pre-2026-10-07 behaviour)
    'blade_policy': 'continuous',
    'slope_mode': 'off',                # off | auto | contour | updown (terrain memory)
    'slope_contour_above_deg': 10.0,
}
PATH_MODES = ('zigzag', 'cross', 'alternate', 'spiral', 'contour_only')
ROUTE_ORDERS = ('boustrophedon', 'snake', 'spiral', 'racetrack')
TURN_TYPES = ('auto', 'loop', 'reverse', 'pivot')
EDGE_MODES = ('soft', 'hard')
OBSTACLE_DETECTION_LEVELS = ('none', 'standard', 'sensitive')
BLADE_POLICIES = ('continuous', 'conservative')
STEREO_STALE_SUB = 'sensor stale: stereo depth (%d s)'
PIVOT_SUB = 'turning (slow pivot)'
_STEREO_STALE_RE = re.compile(
    rb'/stereo_depth/points observation buffer has not been updated for ([0-9]+(?:\.[0-9]*)?) s')


def parse_stereo_stale(data, logger_name=b'local_costmap'):
    """Seconds N from a serialized rcl_interfaces/Log (raw /rosout CDR bytes, or text) that
    carries Nav2's "<topic> observation buffer has not been updated for N seconds" for the
    stereo cloud from ``logger_name``; None otherwise. Byte search only (no deserialize)."""
    if isinstance(data, str):
        data = data.encode('utf-8', 'replace')
    data = bytes(data)
    if b'observation buffer' not in data or logger_name not in data:
        return None
    mo = _STEREO_STALE_RE.search(data)
    return float(mo.group(1)) if mo else None


# Controller aborts caused by lethal costmap cells on the path ahead (Jazzy RPP
# "RegulatedPurePursuitController detected collision ahead!", FTC "collision detected ...",
# "lethal footprint collision"). A plain "Failed to make progress" is NOT one (grass stall).
# 2026-10-09: MPPI (transit FollowPath) aborts with "Optimizer fail to compute path" only when
# its cost critic flags EVERY sampled trajectory as colliding (data.fail_flag), i.e. blocked.
_COLLISION_MARKERS = (b'collision ahead', b'collision detected', b'lethal footprint collision',
                      b'Optimizer fail to compute path')


def parse_collision_abort(data, logger_name=b'controller_server'):
    """True when a serialized rcl_interfaces/Log (raw /rosout bytes, or text) from
    ``logger_name`` reports a costmap collision on the path ahead. Byte search only."""
    if isinstance(data, str):
        data = data.encode('utf-8', 'replace')
    data = bytes(data)
    if logger_name not in data:
        return False
    return any(mk in data for mk in _COLLISION_MARKERS)


def coverage_route_params(st):
    """Swath order / turn settings forwarded to /coverage_server."""
    return {'boundary_inset_m': max(0.0, float(st['edge_margin_m'])),
            'route_order': str(st['route_order']),
            'route_spiral_size': int(st['route_spiral_size']),
            'min_turn_radius_m': float(st['min_turn_radius_m']),
            'turn_type': str(st['turn_type']),
            # stock-style edge layout per area edge_mode
            'edge_layout': str(st.get('edge_layout', st.get('edge_mode', 'soft'))),
            'edge_laps': int(st.get('edge_laps', AREA_SETTINGS_DEFAULTS['edge_laps']))}

# action outcomes reported by the node
SUCCEEDED = 'succeeded'
ABORTED = 'aborted'
CANCELED = 'canceled'
REJECTED = 'rejected'
UNAVAILABLE = 'unavailable'   # server not up within the wait timeout
TIMEOUT = 'timeout'           # FSM watchdog

# A nav2-backed goal the node sent while the Nav2 lifecycle stack was still not active
# after nav2_ready_timeout_s carries result['nav2_inactive'] (e.g. 'Nav2 stack not active
# after 20s') when it then failed without ever being accepted (REJECTED / UNAVAILABLE;
# any failure for the dock action, whose docking server navigates with Nav2). The failure
# reason then names the real cause instead of a generic docking / transit failure
# (2026-10-13: a hung nav2 lifecycle manager showed as "Docking failed (NAV_TO_DOCK_FAILED)").
NAV2_INACTIVE_KEY = 'nav2_inactive'
# a /plan_coverage result whose path conditioning the node already ran off the FSM lock
# (MissionFSM.plan_conditioning_job + condition_plan): {'subpaths', 'cond', 'cond_errors',
# 'logs'}; without it _on_plan conditions inline (unit tests, sim without the node split)
CONDITIONED_KEY = 'conditioned'
NAV2_DOWN = 'navigation stack not active (Nav2 lifecycle not running)'


def nav2_down_why(why):
    """Failure text led by NAV2_DOWN (``why``: the original detail, may be empty)."""
    why = str(why or '')
    if why.startswith(NAV2_DOWN):
        return why
    return '%s: %s' % (NAV2_DOWN, why) if why else NAV2_DOWN

# state_name -> HighLevelStatus.state (codes as in upstream main_tree.xml)
STATE_CODES = {
    'EMERGENCY': STATE_NULL,
    'BOUNDARY_EMERGENCY_STOP': STATE_NULL,
    'NAV_TO_DOCK_FAILED': STATE_NULL,
    'IDLE': STATE_IDLE,
    'IDLE_DOCKED': STATE_IDLE,
    'CHARGING': STATE_IDLE,
    'RAIN_WAITING': STATE_IDLE,
    'RECORDING_COMPLETE': STATE_IDLE,
    'UNDOCK_FAILED': STATE_IDLE,
    # latched result: some sub-paths could not be mowed; robot stopped in place,
    # resume cursor kept (START resumes them). Left by START / HOME / STOP.
    'MOWING_INCOMPLETE': STATE_IDLE,
    'PREFLIGHT_CHECK': STATE_AUTONOMOUS,
    'UNDOCKING': STATE_AUTONOMOUS,
    'WAITING_FOR_RTK': STATE_AUTONOMOUS,
    'PLANNING': STATE_AUTONOMOUS,
    'MOWING': STATE_AUTONOMOUS,
    'TRANSIT': STATE_AUTONOMOUS,
    'AREA_UNREACHABLE': STATE_AUTONOMOUS,
    'BOUNDARY_PAUSED': STATE_AUTONOMOUS,
    'MOWING_COMPLETE': STATE_AUTONOMOUS,
    'RETURNING_HOME': STATE_AUTONOMOUS,
    'LOW_BATTERY_DOCKING': STATE_AUTONOMOUS,
    'RAIN_DETECTED_DOCKING': STATE_AUTONOMOUS,
    'COVERAGE_FAILED_DOCKING': STATE_AUTONOMOUS,
    'RECORDING': STATE_RECORDING,
    'MANUAL_MOWING': STATE_MANUAL_MOWING,
    # stuck guard gave up: latched fault, wheels/blade stopped; STOP / START / MANUAL clear
    'STUCK_NEEDS_HELP': STATE_IDLE,
}
# Phases the stuck guard escapes from (mission transits / mowing, docking).
STUCK_PHASES = ('TRANSIT', 'MOWING') + ('RETURNING_HOME', 'LOW_BATTERY_DOCKING',
                                        'RAIN_DETECTED_DOCKING', 'COVERAGE_FAILED_DOCKING')
STUCK_NEEDS_HELP_SUB = 'stuck: needs help at (%.2f, %.2f)'
# Task 0.4 (a): autonomous driving phases the hard boundary applies in (the heading
# calibration drive in WAITING_FOR_RTK too, see _hard_boundary_tick).
HARD_BOUNDARY_PHASES = ('TRANSIT', 'MOWING') + ('RETURNING_HOME', 'LOW_BATTERY_DOCKING',
                                                'RAIN_DETECTED_DOCKING', 'COVERAGE_FAILED_DOCKING')

IDLE_NAMES = ('IDLE', 'IDLE_DOCKED', 'CHARGING')
# ~/preview_plan is accepted only here (no mission session, robot at rest)
PREVIEW_PHASES = IDLE_NAMES + ('MOWING_COMPLETE', 'MOWING_INCOMPLETE')
PREVIEW_ALL = 255       # preview_plan area_index: every mowing area
PREVIEW_CLEAR = 254     # preview_plan area_index: clear the drawn preview
SEGMENT_RING = 0        # PlanCoverage.Result.SEGMENT_*
SEGMENT_SWATH = 1
# transient result states, shown for Params.result_display_s then -> idle
DISPLAY_NAMES = ('RECORDING_COMPLETE', 'UNDOCK_FAILED', 'NAV_TO_DOCK_FAILED')
# a mowing session is in progress (rain / battery / boundary guards apply)
MISSION_PHASES = ('UNDOCKING', 'WAITING_FOR_RTK', 'PLANNING', 'MOWING', 'TRANSIT',
                  'AREA_UNREACHABLE', 'BOUNDARY_PAUSED')
# DOCK_PHASES, MOTION_PHASES (the wheels may move: published as latched /motion_enabled and
# enforced by mower_control/cmd_vel_slew's motion gate; everything else -- IDLE*, CHARGING,
# EMERGENCY, BOUNDARY_EMERGENCY_STOP, MOWING_COMPLETE/INCOMPLETE, NAV_TO_DOCK_FAILED,
# PREFLIGHT_CHECK, RAIN_WAITING, ... -- forces zero; RECORDING is joystick driving like
# MANUAL_MOWING) and DOCKED_MOTION_PHASES are imported from mower_common.phases, the one
# source both nodes use.


# ---------------------------------------------------------------------------
# effects
# ---------------------------------------------------------------------------
@dataclass
class BladeOn:
    pass


@dataclass
class BladeOff:
    reason: str = ''


@dataclass
class ZeroBurst:
    pass


@dataclass
class StartAction:
    name: str
    goal: dict
    token: int


@dataclass
class CancelActions:
    reason: str = ''


@dataclass
class CallService:
    name: str
    request: dict
    token: int


@dataclass
class PublishStatus:
    status: dict


@dataclass
class PublishPlan:
    poses: list


@dataclass
class PublishRoute:
    """The conditioned transit / dock route being driven (path conditioning)."""
    what: str
    poses: list
    types: list


@dataclass
class PublishTrajectory:
    points: list


@dataclass
class PublishAreaSettings:
    settings: dict          # active settings of the area being planned / mowed (JSON-able)


@dataclass
class PublishPreviewSummary:
    summary: dict           # plan preview summary (latched ~/preview_summary JSON)


@dataclass
class SaveAlternateCounts:
    counts: dict            # area name -> completed alternate runs (angle rotation step)


@dataclass
class PublishResumeAvailable:
    available: bool


@dataclass
class SaveResume:
    text: str


@dataclass
class DeleteResume:
    pass


@dataclass
class SaveRecordingFallback:
    points: list
    name: str


@dataclass
class RecordIncident:
    """Terrain-memory incident (published on /mission/incident, JSON; mower_map
    map_server_node files it into the per-area terrain raster)."""
    kind: str       # follow_abort | transit_abort | subpath_failed | detour | boundary ...
    x: Optional[float] = None
    y: Optional[float] = None
    detail: str = ''


@dataclass
class DriveCmd:
    """Stuck escape: one Twist on /cmd_vel_emergency (highest twist_mux lane)."""
    linear: float = 0.0
    angular: float = 0.0


@dataclass
class SuppressStuckGuard:
    """Latched /stuck_guard/suppress: the slip_detector stuck guard is paused (and its
    /stuck latch cleared) while the mission drives its own escape."""
    active: bool


@dataclass
class ResetStuckGuard:
    """/stuck_guard/reset: clear a /stuck latch (operator STOP / START / MANUAL)."""
    pass


@dataclass
class Log:
    level: str      # 'info' | 'warn' | 'error'
    text: str


# ---------------------------------------------------------------------------
# inputs and parameters
# ---------------------------------------------------------------------------
@dataclass
class Inputs:
    emergency_active: bool = False      # Emergency.active_emergency
    emergency_stamp: Optional[float] = None   # receipt time of the last Emergency msg
    emergency_reason: str = ''
    lift: bool = False                  # MowerBaseDevStatus.lift_triggered
    stop_button: bool = False           # MowerBaseDevStatus.stop_triggered
    rain: bool = False
    is_charging: bool = False
    docked: bool = False                # MowerBaseDevStatus.is_docking_done
    is_cutting: bool = False
    battery_percent: Optional[float] = None
    fix_type: Optional[int] = None
    gps_quality: float = 0.0            # 0..1 (upstream publishes a fraction)
    pose: Optional[tuple] = None        # (x, y, yaw) in map
    boundary_violation: bool = False
    lethal_boundary_violation: bool = False
    # /map_server_node/outside_m (Float32, task 0.4 a): distance of the robot centre outside
    # (mowing areas U drawn paths U dock corridor), 0 inside; outside_stamp = receipt time
    outside_m: Optional[float] = None
    outside_stamp: Optional[float] = None
    # bumper_controller routing status rising edges (/mower_base/bumper_routing_status 0 -> 1)
    bumper_hits: int = 0
    # /heading_aligner/status (latched JSON): aligned + source (none|file|dock|cog)
    heading_aligned: bool = False
    heading_source: str = 'none'
    heading_stamp: Optional[float] = None     # receipt time of the last status
    # status file_check (source file): pending | yaw_ok (IMU kept running: the saved offset
    # is a usable prior) | yaw_failed | unverifiable | '' (see heading_logic._try_verify)
    heading_prior: str = ''
    # status quality (good|fair|poor|low|none) and the age of the COG heading (s: the older of
    # last_cog_age_s and age_s, see mission_node._on_heading); None/'' = not reported
    heading_quality: str = ''
    heading_cog_age: Optional[float] = None
    # /obstacle_policy (mower_vision obstacle_guard, JSON): closest close detection
    obstacle_kind: str = 'none'         # none | dynamic | static
    obstacle_class: str = ''
    obstacle_distance: Optional[float] = None
    obstacle_bearing: Optional[float] = None  # deg, base_link, left +
    obstacle_stamp: Optional[float] = None    # receipt time of the last policy message
    # /mower_docking/marker_in_view (latched Bool): owner rule "dock marker in view ->
    # ignore all other camera detections" while docking
    dock_marker_in_view: bool = False
    # /map_server_node/docking_pose (latched, map frame): (x, y, yaw) for the dock zone
    dock_pose: Optional[tuple] = None
    # /supervisor/status critical_down (comma-joined node names; '' = all alive)
    critical_nodes_down: str = ''
    # /rosout local_costmap "/stereo_depth/points observation buffer has not been updated
    # for N seconds": last receipt time + N (stale stereo -> controller aborts FollowPath)
    stereo_stale_s: Optional[float] = None
    stereo_stale_stamp: Optional[float] = None
    # /rosout controller_server collision-ahead abort (parse_collision_abort): receipt time
    collision_stamp: Optional[float] = None
    # /cmd_vel_nav (the controller's command): (linear.x, angular.z) + receipt time
    nav_cmd: Optional[tuple] = None
    nav_cmd_stamp: Optional[float] = None
    # /cmd_vel (applied, after twist_mux: Nav2, docking, emergency): (linear.x, angular.z) +
    # receipt time; drives the obstacle in-path prediction (2026-10-13)
    cmd_vel: Optional[tuple] = None
    cmd_vel_stamp: Optional[float] = None
    # /stuck (slip_detector stuck guard, latched Bool)
    stuck: bool = False
    # /collision_monitor_state: monotonic time the current STOP began (None = not stopping)
    cm_stop_since: Optional[float] = None
    # /tilt/status (mower_control tilt_monitor, latched JSON): band ok|caution|limit|critical|
    # unknown, filtered roll / pitch (deg, REP-103), axis that set the band, receipt time
    tilt_band: str = 'unknown'
    tilt_roll_deg: Optional[float] = None
    tilt_pitch_deg: Optional[float] = None
    tilt_axis: str = ''
    tilt_stamp: Optional[float] = None
    # 2026-10-09 /localization/status (mower_localization localization_monitor, latched
    # JSON): state ok|degraded|lost + reason; loc_stamp = receipt time (None = never seen)
    loc_state: str = ''
    loc_reason: str = ''
    loc_stamp: Optional[float] = None
    # 2026-10-23 option C: /obstacle_tracker/nogo_zones (nogo.ZoneSet) + receipt time
    nogo_zones: Any = None
    nogo_stamp: Optional[float] = None


@dataclass
class Params:
    tick_hz: float = 10.0
    emergency_timeout_s: float = 2.0
    battery_low_percent: float = 20.0
    battery_full_percent: float = 95.0
    # What a running mission does below battery_low_percent:
    #   'stop' = blade off, zero velocity, stay where it is (IDLE); 'dock' = go charge.
    # 2026-10-22: default 'dock' (owner: "when the battery is low it should automatically go
    # back and charge"; 'stop' left the robot idle mid-lawn at 14 %).
    battery_low_action: str = 'dock'
    # Charge limit: while docked, charging is disabled at >= this and re-enabled at
    # <= this - battery_charge_hysteresis_percent. 100 = no limit. A charge-and-resume
    # waits for min(battery_full_percent, battery_max_charge_percent).
    battery_max_charge_percent: float = 100.0
    battery_charge_hysteresis_percent: float = 5.0
    preflight_min_fix_type_docked: int = 1
    rtk_fix_type: int = FIX_RTK_FIXED
    rtk_timeout_s: float = 120.0
    require_heading_alignment: bool = True    # dock|cog heading before any Nav2 motion
    heading_status_timeout_s: float = 5.0     # older /heading_aligner/status = unaligned
    heading_wait_s: float = 30.0              # after undock + RTK, wait this long for COG
    # 2026-10-13 heading calibration drive (_hcal_*): START off the dock with only a saved
    # 'file' heading whose IMU continuity held (status file_check yaw_ok) drives a short
    # straight line (blade off, Nav2 DriveOnHeading / BackUp on the nav lane) so the aligner
    # gets a COG fix, instead of refusing. The line (distance + footprint, +-cone deg, robot
    # width) must lie inside one mowing area and clear of its keepouts by the PRIOR heading;
    # else the reverse line; else refused.
    heading_calibration_drive: bool = True
    heading_calibration_distance_m: float = 1.0
    heading_calibration_speed_mps: float = 0.15
    heading_calibration_timeout_s: float = 45.0     # drive incl. the Nav2 resume wait
    heading_calibration_cone_deg: float = 10.0      # prior heading error the map check allows
    heading_calibration_settle_s: float = 3.0       # RTK fixed this long first (the saved
                                                    # heading may still verify without a drive)
    heading_calibration_status_wait_s: float = 3.0  # after the drive: aligner status latency
    # 2026-10-14: the second START of the day planned on "aligned (cog)" with quality poor and
    # a 1200 s old COG fix and drove its transit 65 deg off. A START off the dock with a cog
    # heading older than heading_max_cog_age_s (0 = no age limit) or with a quality in
    # heading_reject_quality is treated like the 'file' prior: calibration drive first.
    heading_max_cog_age_s: float = 300.0
    heading_reject_quality: list = field(default_factory=lambda: ['poor'])
    rain_mode: int = 1                  # 0 ignore, 1 dock and wait
    rain_debounce_s: float = 2.0
    rain_delay_minutes: float = 30.0
    use_docking_server: bool = True
    undock_distance_m: float = 0.8
    manual_undock_distance_m: float = 0.5    # standalone UNDOCK command (GUI button)
    undock_speed_mps: float = 0.15
    dock_use_vision: bool = True
    dock_timeout_s: float = 180.0
    dock_pose: tuple = ()               # (x, y, yaw); only used without docking server
    mow_angle_deg: float = -1.0
    transit_gap_m: float = 0.6
    # A /navigate_to_pose transit that ABORTs / times out while the robot is
    # already this close to its target counts as arrived (Nav2 goal checker
    # tolerances / yaw settling can abort a goal the robot effectively reached).
    transit_arrived_radius_m: float = 0.3
    # A failed (aborted / timed-out) transit is re-sent up to transit_retries times,
    # transit_retry_delay_s apart (costmaps are cleared first so they can refresh);
    # Nav2 re-plans from the robot's current pose.  Same delay for follow retries.
    transit_retries: int = 3
    transit_retry_delay_s: float = 3.0
    # 2026-10-07: genuine (no collision) transit / follow retries back off: retry k waits
    # transit_retry_backoff_s[k-1] (last entry repeats); empty = transit_retry_delay_s.
    transit_retry_backoff_s: list = field(default_factory=lambda: [3.0, 10.0, 30.0])
    # 2026-10-07 live: a person in front aborted the transit 4x in 30 s (RPP "collision
    # ahead" -> controller patience exceeded) and the sub-path was dropped.  An abort that
    # coincides with a "collision ahead" (collision_abort_window_s) or a fresh dynamic /
    # static policy is a BLOCKED wait: hold up to blocked_wait_s (blocked_wait_home_s while
    # RETURNING_HOME), re-send every blocked_resend_s once no "collision ahead" was logged
    # for blocked_clear_s; no retry is consumed.
    blocked_wait_s: float = 60.0
    blocked_wait_home_s: float = 120.0
    blocked_resend_s: float = 5.0
    blocked_clear_s: float = 3.0
    # a sub-path whose transit still fails is re-tried once in the end-of-area pass
    requeue_failed_transits: bool = True
    # Owner 2026-10-07: "if it is not in the way of the path it should not stop the mow".
    # A ranged detection (distance + bearing) counts only inside the corridor along the
    # next obstacle_path_lookahead_m of the current follow / transit path, half-width =
    # robot_width_m / 2 + obstacle_path_margin_m; off-corridor = advisory only.
    obstacle_path_filter: bool = True
    obstacle_path_lookahead_m: float = 2.0
    obstacle_path_margin_m: float = 0.25
    robot_width_m: float = rgeo.FOOTPRINT_WIDTH_M   # 0.47 with wheel trim (2026-10-20, was 0.54)
    # 2026-10-13 (person stopped the mower while the robot pivoted toward a detour target:
    # the straight line to the target is not where the robot goes). Where the mission does
    # not know the path the controller tracks (/navigate_to_pose transits: detours, gaps,
    # retries; return-to-dock and docking) the corridor is the robot footprint swept along
    # its PREDICTED motion: constant applied /cmd_vel (else /cmd_vel_nav) twist, younger than
    # obstacle_twist_max_age_s, for obstacle_sweep_horizon_s (arc v/w, straight, or pivot).
    # Footprint: robot_rear_edge_m behind to robot_front_m ahead of base_link, +-robot_width_m/2,
    # grown by obstacle_path_margin_m. Anything within the circumscribed footprint radius +
    # margin of the robot is always in the path (standing still / pivoting next to it).
    obstacle_sweep_horizon_s: float = 2.0
    obstacle_twist_max_age_s: float = 0.5
    robot_front_m: float = rgeo.FOOTPRINT_FRONT_M   # bumper 0.60 (owner-measured; was 0.52)
    # = obstacle_guard rear_edge_x_m: the rear 0.10 plus a deliberate 0.02 margin (renamed from
    # robot_rear_m on 2026-10-23 so it is not read as the body's rear, 0.10; was 0.25)
    robot_rear_edge_m: float = 0.12
    # 2026-10-09 route graph (docs/route_graph.md, owner: "a specific path for on paths, a
    # different navigation ... then path again"): a transit (sub-path start, end-of-area
    # retry, blocked / dynamic / localization re-send) first asks map_server's plan_route
    # and drives the route (drawn paths along their centreline, in-area legs inside the
    # area) with FollowPath; no route (off graph / no connection) or a route follow that
    # aborts (obstacle, MPPI failure) falls back to /navigate_to_pose (costmap planner inside
    # the hard nav mask) with the existing blocked / retry semantics. Detours and boundary
    # recoveries always use /navigate_to_pose. route_dock: return-to-dock first drives the
    # route to the dock approach pose (facing the dock), then the docking server (which
    # skips its own Nav2 leg within approach_skip_radius_m).
    route_transits: bool = True
    route_dock: bool = True
    # Task 0.4 (e), run 3: the dock route aborted at (-0.69, 0.04) and the docking server then
    # drove its own navigate_to_pose to the approach pose across the grass. The docking
    # server is now started only within dock_server_nav_max_m of the approach pose (dock +
    # dock_approach_distance_m along the dock yaw, = the dock route's end; mirrors map_server
    # corridor_half_width_m); farther away the dock route is planned and driven again from
    # the current pose, at most dock_route_retries more times, then NAV_TO_DOCK_FAILED (the
    # robot stops where it is). An obstacle abort of the dock route waits (blocked wait)
    # first. 0 = the old hand-over (the docking server navigates from anywhere).
    dock_server_nav_max_m: float = 0.35
    dock_approach_distance_m: float = 0.8     # = map_server / mower_docking approach_distance
    dock_route_retries: int = 2
    route_controller_id: str = 'FollowPath'          # MPPI transit controller
    route_goal_checker_id: str = 'general_goal_checker'
    route_end_tolerance_m: float = 0.5   # route "success" farther from the target = failed
    # path_conditioning: a transit that arrived more than this from the sub-path start drives
    # the last straight bit onto it (pivot to face it if needed, along-path end) before the
    # sub-path's own start pivot (_route_final_approach). 0 = off (revert).
    route_final_approach_m: float = 0.05
    route_final_approach_speed_mps: float = 0.1
    # Sim 2026-10-09: the scripted person crossed Path 1, the route aborted (collision ahead)
    # and the /navigate_to_pose fallback after the wait cut across the lawn into a drawn
    # obstacle corner. An obstacle abort now waits (blocked wait) and re-sends the ROUTE
    # this many times per transit; only a further obstacle abort falls back to Nav2 (a
    # static unknown obstacle on the route: Nav2 plans round its costmap marks).
    route_blocked_retries: int = 1
    # field 2026-10-20: plan_route failed once (stale deadline) and the transit fell back to
    # the costmap planner with a goal 34 m away, outside the 50 x 50 m rolling global costmap
    # ("Goal outside bounds"). A failed plan_route to a goal farther than this re-asks the
    # route (route_plan_retries times, after route_plan_retry_backoff_s[k-1]) instead; only a
    # goal within it may fall back to /navigate_to_pose. 0 = always fall back (old).
    transit_fallback_max_goal_m: float = 24.0
    route_plan_retries: int = 3
    route_plan_retry_backoff_s: list = field(default_factory=lambda: [2.0, 5.0, 10.0])
    # Sim 2026-10-09 (v4 map, end-to-end paths): MPPI stalled at the 128 deg hairpin from a
    # path's end into the lawn hop and the route aborted. The route is sent as separate
    # FollowPath legs split where it turns more than this (0 = one goal); each next leg
    # starts with the controller's own turn toward the path.
    route_split_turn_deg: float = 100.0
    # 2026-10-14 run 2 (fix 11): MPPI stalled at the dock-path -> area-path hairpin facing the
    # wrong way, the progress checker aborted, and the /navigate_to_pose fallback drove 0.56 m
    # off the drawn path. A route cusp is where the route reverses by more than
    # route_cusp_angle_rad (turn between the 0.3 m chords before / after a pose; a route-leg
    # junction counts too). A route follow abort / timeout within route_cusp_radius_m of a
    # cusp: cancel, pivot in place (Nav2 Spin) to the route direction after the cusp, re-send
    # the route from the cusp; at most route_cusp_retries about-turns per cusp (per sub-path
    # transit), then a transit abort (transit_retries; never the costmap planner).
    # 0 retries = the old behaviour (costmap fallback).
    # Task 0.4 (d), run 3: the start abort happened 0.75 m past the junction, outside the old
    # 0.5 m radius, so the costmap fallback ran again: 0.5 -> 1.0.
    route_cusp_radius_m: float = 1.0
    route_cusp_angle_rad: float = 2.0
    route_cusp_retries: int = 2
    # Spin time allowance = |angle| / this + 10 s. 0.05 (was 0.2, 2026-10-16 field run 6): on
    # the slope beside the dock path a pivot turned 27 deg in 8.5 s (0.055 rad/s, sliding
    # 0.38 m): a 112 deg corner needs ~37 s there, past the old 20 s allowance (Spin TIMEOUT
    # -> "aborted"). Flat turf pivots at ~0.25 rad/s, so this only matters where it is slow.
    route_cusp_spin_rps: float = 0.05
    # 2026-10-16 field run 6: a route corner Spin ended "aborted" (slope) and the mission
    # followed the leg anyway, 100+ deg off it; MPPI left the area at once (HARD BOUNDARY).
    # A pivot Spin (route corner, cusp about-turn, coverage pivot, final approach) that ends
    # without SUCCEEDED and leaves the robot at or beyond the pivot threshold off its heading
    # is re-sent for the remaining angle up to this many times; then STUCK_NEEDS_HELP (motion
    # stopped). Within the threshold the leg is followed, as after a successful pivot.
    # Revert: pivot_retries -1 (follow the leg after any pivot outcome, the old behaviour).
    pivot_retries: int = 2
    stretch_min_retry_m: float = 0.5      # shorter skipped stretches are not re-tried
    stretch_block_radius_m: float = 1.0   # ... nor ones this near a stretch that stayed blocked
    # A transit goal off the route graph (an edge / K-turn start on the line) is re-asked moved
    # inside by these insets; never the costmap planner to it (field 2026-10-23: that fallback
    # cut the area / dock-path junction, 6 trips). None fits: the piece is skipped.
    offgraph_goal_insets_m: list = field(default_factory=lambda: [0.3, 0.6])
    route_pivot_backout_m: float = 0.30   # refused route corner pivot: back off, re-plan
    pivot_retry_pause_s: float = 2.0    # wait between a refused pivot and its re-send
    # 2026-10-21 owner rule (stock firmware): a COVERAGE pivot Spin (leg start / pivot vertex)
    # that ends without SUCCEEDED ("Collision Ahead", costmap blocked, Spin abort / timeout)
    # with the robot still >= the pivot threshold off the leg is NOT re-sent: the leg (start
    # -> next pivot vertex, or the sub-path end) is skipped ("blocked pivot: skip"), recorded
    # and the next leg / sub-path is driven. blocked_pivot_requeue (and requeue_skipped):
    # skipped legs are re-tried once in the end-of-area pass, else reported uncut at once.
    # Above blocked_pivot_max_skips skips per mission (START / resume) the old re-send ->
    # STUCK_NEEDS_HELP path (pivot_retries) applies. Revert: blocked_pivot_policy 'retry'.
    # Route / cusp / final-approach pivots are unchanged (pivot_retries).
    blocked_pivot_policy: str = 'skip'      # skip | retry
    blocked_pivot_max_skips: int = 20
    blocked_pivot_requeue: bool = True
    # 2026-10-21 field (Area 6, stone / tree 0.15-0.35 m outside the line): edge laps against
    # real obstacles end the blocked segment like stock ("edge end occupied -> arrived"): a
    # follow abort with a collision, collision_monitor STOP held edge_blocked_hold_s, or a
    # refused pivot on an edge lap skips ahead to the next pivot vertex, at most
    # edge_blocked_skip_m ("edge blocked: skip X m"), recorded like a blocked-pivot skip; the
    # mission total is logged at the end. Edge lap: a sub-path >= 2 m whose median distance
    # to the area outline / an obstacle ring is <= edge_lap_max_inset_m. Revert: false.
    edge_blocked_skip: bool = True
    edge_blocked_hold_s: float = 3.0
    edge_blocked_skip_m: float = 2.0
    edge_lap_max_inset_m: float = 0.6
    # refused coverage pivot (policy skip): back straight up this far (Nav2 BackUp at
    # pivot_backout_speed_mps, both wheels above the 0.06 m/s floor) and pivot ONCE more
    # before skipping. 0 = off.
    pivot_backout_m: float = 0.35
    pivot_backout_speed_mps: float = 0.1
    # STUCK_NEEDS_HELP while mowing: the failing piece (leg start -> next pivot vertex / at
    # most edge_blocked_skip_m) is recorded as skipped and the resume cursor moved past it,
    # so START never re-runs the identical failing pivot first. Revert: false.
    resume_skip_failing: bool = True
    # 2026-10-20 error budget (docs/analysis/2026-10-20_error_budget.md): a Spin stops
    # commanding when its relative yaw reaches the target, but the robot is still turning at
    # the pivot floor (behavior_server min_rotational_vel 0.3 rad/s since 2026-10-20; was 0.25)
    # and keeps turning through
    # the stop latency (twist_mux -> cmd_vel_slew angular decel -> MCU / wheel lag): every
    # pivot turned too far (sim ideal 6-7 deg with the old 1.0 rad/s Spin, field 2-7 deg at
    # the next leg's start). The Spin is sent this much short of the target (rad, never more
    # than half the angle) so the robot comes to rest on it: 0.04 (~0.25 rad/s x 0.15 s)
    # left a 1.3 deg median overshoot in the ideal sim (b4784b0), so 0.06 (calibrated at 0.25;
    # the pivot now ends at 0.3 rad/s, ~0.05 rad in 0.15-0.2 s: re-check in the field).
    # 0 = the old behaviour. Calibrate on the robot (error budget doc, field check).
    pivot_stop_lead_rad: float = 0.06
    # Task 0.4 (d), run 3 (owner: "in sharp corners it could reverse and turn rather than
    # overshoot the corner, the turn itself looks good"): the route is also split at every
    # corner sharper than this (rad), and before any route leg (transit or dock route) whose
    # start is within route_cusp_radius_m and whose heading is more than this off the robot's
    # yaw, the robot stops and pivots in place (Nav2 Spin) to the leg heading, then the leg is
    # sent. One about-turn per leg; a failed Spin sends the leg anyway (the follow judges).
    # 0 = off (the controller turns onto the leg itself, splits at route_split_turn_deg only).
    # 2026-10-15 sim (real map): the dock path's drawn 90 deg corners were not split (the
    # test was "> pi/2" on single 3-pose chords: exactly 90 failed) and cost 0.20 m each.
    # 1.38 rad (79 deg) is where an arc of the 0.5 m minimum turn radius cuts the corner by
    # the 0.15 m pass limit (0.5 (1 / cos(theta / 2) - 1) = 0.15 -> theta = 79.5 deg), so a
    # drawn ~90 deg corner (and an 80 deg one) stops and pivots; the test is inclusive with
    # a 1 deg tolerance. Revert to the run 4 threshold: 1.5708.
    route_corner_turn_rad: float = 1.38
    # FollowPath goal checker of a leg that ends on a pivot vertex: a route leg ending at a
    # split corner (not the last leg; the transit checker, xy 0.25 m, ended them ~0.2 m short
    # and the pivot happened there) and, with path_conditioning, every conditioned forward
    # coverage leg. coverage_corner_goal_checker (2026-10-16): along-path, done at the plane
    # through the vertex, no yaw test, no 0.08 m disc. '' = route_goal_checker_id.
    route_corner_goal_checker_id: str = 'coverage_corner_goal_checker'
    # 2026-10-15 undock incident: the start route began with a 0.17 m connector from the robot
    # to the dock path, then turned 90 deg onto it. That stub was the first leg (pivot 111 deg
    # to it, then a FollowPath of 0.17 m ending at the 90 deg corner) and MPPI circled its end
    # for 82 s. A corner (route_corner_turn_rad) within route_stub_trim_m of either route end
    # now cuts the stub off: the route starts (ends) at that corner, so the pre-leg pivot turns
    # to the heading AFTER the corner. A trimmed end is <= this from the target (inside
    # route_end_tolerance_m / dock_server_nav_max_m). 0 = off (revert).
    route_stub_trim_m: float = 0.3
    # ... and the first leg joins the route where the robot projects onto its first
    # route_start_join_m (robot within that distance of the route), not at a start behind
    # it (the robot can be ~0.15 m off the route start after undocking or a pivot).
    # 0 = off (revert).
    route_start_join_m: float = 0.5
    # 2026-10-15 undock incident: MPPI turned in place / circled for 82 s on a route leg and
    # the PoseProgressChecker (rotation counts as progress) never aborted it. Route legs
    # (transit and dock route) now name a translation-only progress checker
    # (nav2_params.yaml transit_progress_checker: SimpleProgressChecker 0.25 m / 20 s); the
    # coverage follows keep the rotation-counting one (follow_progress_checker_id: an RPP
    # rotate-to-heading at a swath end is progress). With more than one progress checker
    # loaded, Jazzy's controller_server rejects a FollowPath goal without an id, so both
    # are always sent. Revert: route_progress_checker_id: progress_checker.
    route_progress_checker_id: str = 'transit_progress_checker'
    follow_progress_checker_id: str = 'progress_checker'
    # ... and a mission-side watch on route legs (a circle of 0.3 m diameter can keep
    # resetting a radius progress checker): the arc length of the robot's projection onto
    # the leg must grow by route_progress_min_m within route_progress_window_s, else the
    # leg is cancelled and handled as a FollowPath abort (a corner leg within
    # route_leg_end_accept_m of its end counts as reached, below; otherwise the usual route
    # abort: blocked wait / cusp about-turn / bounded costmap fallback). Route corner pivots
    # are Nav2 Spin, not FollowPath, so they never count. A /navigate_to_pose transit
    # (costmap fallback) gets the same window on its distance to the target (no
    # route_progress_min_m closer -> transit abort). 0 window = off (revert, both).
    route_progress_window_s: float = 20.0
    route_progress_min_m: float = 0.15
    # A route leg into a split corner (not the last leg) that aborts (Nav2 progress checker
    # or the watch above, no obstacle) within this of its end counts as reached: the next
    # leg's pivot is done from there, instead of a route abort beside the corner (the leg
    # goal checker wants 0.08 m; the translation-only checker no longer counts the final
    # turning-in as progress). 0 = off (revert).
    route_leg_end_accept_m: float = 0.25
    # (2026-10-15 run mow-040433 STUCK1 / STUCK2, a route leg into a split corner never
    # ended on the 0.08 m + yaw leg checker, MPPI dithered: the mission-side corner-reached
    # rule route_corner_xy_m / reach / stall was removed 2026-10-16. Corner legs end on
    # route_corner_goal_checker_id = coverage_corner_goal_checker, along-path, no yaw test;
    # a leg that still stops short is ended by the route progress watch and
    # route_leg_end_accept_m.)
    # ... and before a route leg after a split corner (not the first leg: a route start keeps
    # route_corner_turn_rad) the robot pivots (Nav2 Spin) when the leg heading is more than
    # this off its yaw (was route_corner_turn_rad, 79 deg): after STUCK1's corner MPPI would
    # have had a ~51 deg turn-in, the slow-turn dither regime. 0 = route_corner_turn_rad
    # (revert).
    route_leg_pivot_min_rad: float = 0.6
    # Path conditioning (docs/DESIGN_path_conditioning_2026-10.md steps 2-3; 2026-10-15 run 5:
    # RPP drove the outer ring's 100 deg V at (-4.74, -1.17) as an arc 0.26 m wide, out of
    # the area twice -> STUCK). Every forward coverage sub-path (2026-10-16: with each
    # reverse-curve swath-end turn first replaced by its swath -> connector -> swath corners,
    # _join_turn_chains) and every transit / dock route goes through ONE pure-geometry pass
    # (path_conditioning.condition) against the allowed region (mowing areas U drawn path
    # channels U the dock corridor): each sharp corner becomes the largest fillet arc of
    # r in [r_min, r_max] that fits the space (clearance >= min(margin + half width, the
    # drawn path's) - slack), else a pivot at the vertex; on the outermost ring convex
    # corners >= edge_pivot_rad pivot (edge cut); resampled at 0.05 m with segment types.
    # Pivots run as a leg end on the vertex (route_corner_goal_checker_id: along-path, the
    # plane through the vertex) plus an in-place Spin to the next leg, blade on. A sub-path
    # that fails the validation is NOT mowed (never driven). false = raw paths as before
    # (revert). path_conditioning_pivot_min_rad is also the leg-start pivot threshold
    # (_cov_pivot: the robot -> leg junction is a vertex no fillet fits).
    # Stock-style edge layout (owner 2026-10-21): the planner lays laps / rows by the area's
    # edge_mode (soft: laps 0 / 0.15 / 0.30, row ends 0.45 in; hard: body inside + 0.10).
    # The edge_layout_enabled false revert (headland rings) was deleted on 2026-10-23.
    path_conditioning: bool = True
    path_conditioning_r_min_m: float = 0.5
    path_conditioning_r_max_m: float = 1.0
    path_conditioning_margin_m: float = 0.05
    path_conditioning_half_width_m: float = rgeo.FOOTPRINT_HALF_WIDTH_M   # 0.235 (was 0.24)
    path_conditioning_slack_m: float = 0.05
    path_conditioning_pivot_min_rad: float = 0.35
    # 0 since the edge layout (2026-10-21): lap-0 runs ON the line, a pivot at a convex
    # vertex would swing the nose ~0.6 m past it; fillets keep it within ~0.1-0.25 m
    path_conditioning_edge_pivot_rad: float = 0.0
    path_conditioning_edge_clearance_m: float = 0.20
    # Edge layout round 2 (owner 2026-10-21): body planning limit by area edge_mode (m of the
    # body outline outside the region at a corner's fillet / pivot sweep; < 0 = off), wider
    # convex fillets, then K-turns (path_conditioning.py header; config/mission.yaml).
    path_conditioning_body_limit_soft_m: float = 0.26
    path_conditioning_body_limit_hard_m: float = 0.0
    # Field 2026-10-23 (a K-turn planned 0.54 m out next to a bank): a pivot / K-turn / sharp
    # fillet whose planned body exceeds the cap is cut inside the corner (tip left uncut);
    # cap = limit + 0.09 leaves ~0.15 m pivot drift under the 0.55 / 0.40 soft trips. <0 = the
    # conditioner default (limit + 0.09).
    path_conditioning_body_cap_soft_m: float = 0.35
    path_conditioning_body_cap_hard_m: float = 0.09
    path_conditioning_convex_r_max_m: float = 2.0
    path_conditioning_kturn: bool = True
    path_conditioning_kturn_min_rad: float = 1.05
    path_conditioning_kturn_gentle_m: float = 0.33   # gentler corners: K-turn when over this
    # convex fillets: limit = max(body limit, the arc's own front-corner envelope + this)
    path_conditioning_fillet_envelope_margin_m: float = 0.03
    # Area / path outlines for the planner and the conditioner region, cleaned in memory
    # (geometry.clean_area_ring): drawing jogs within area_outline_jog_m of their neighbours'
    # chord dropped, then Douglas-Peucker at area_outline_simplify_m. 0 = that step off.
    area_outline_simplify_m: float = 0.05
    area_outline_jog_m: float = 0.10
    path_conditioning_kturn_back_max_m: float = 1.5
    # transit / dock routes: smallest fillet radius (tighter corners pivot at the vertex)
    path_conditioning_route_r_min_m: float = 0.8
    # The costmap-planner fallback (no route: start off the route graph; a route abort away
    # from any cusp) is bounded: once the robot's own distance from the drawn network (the
    # planned route if there was one; 0 inside a mowing area) exceeds this, the fallback is
    # cancelled and counted as a transit abort. Starting farther off (an off-graph start)
    # arms it only once the robot got within the bound (or moved this much farther away).
    # Mirrors map_server corridor_half_width_m. 0 = unbounded.
    transit_fallback_max_offpath_m: float = 0.35
    # 2026-10-09 transit variation (owner: tyre tracks from driving the same line): every
    # mission start bumps a persisted run counter (alternate_counts['__transit_run__']) that
    # plan_route uses with each area's transit_variation setting. False = always run 0.
    route_variation: bool = True
    # in-area blade-on transit / detour: straight segment sampled this finely must stay
    # inside the area polygon (minus its obstacle holes)
    blade_transit_sample_m: float = 0.2
    # FollowPath abort / timeout mid-swath: resume from the last reached pose.
    follow_retries: int = 3
    clear_costmaps_on_retry: bool = True
    clear_global_costmap_on_retry: bool = False
    # After a sub-path still fails, the session ends in MOWING_INCOMPLETE; dock only
    # when this is true (default: stop in place so the operator can inspect).
    return_home_on_incomplete: bool = False
    # 2026-10-15 MPPI study (docs/analysis/2026-10-15_mppi_study.md): coverage sub-paths are
    # driven by the feasibility-aware MPPI (nav2_params.yaml): mppi_follow_controller_id, and
    # mppi_reverse_controller_id for sub-paths whose poses face against their travel (reverse
    # turn legs from the coverage bridge); the cut speed goes to mppi_controller_speed_param
    # capped at mppi_speed_max_mps. It needs path_conditioning (on raw paths it overshoots the
    # sharp ring vertices and the short swath-end turn legs, study §8). The 'rpp' revert
    # (coverage_controller, follow_/reverse_controller_id, controller_speed_param) was deleted
    # on 2026-10-23 (code review stage 0.1).
    mppi_follow_controller_id: str = 'FollowCoveragePathMPPI'
    mppi_reverse_controller_id: str = 'FollowCoveragePathReverseMPPI'
    mppi_controller_speed_param: str = 'FollowCoveragePathMPPI.vx_max'
    mppi_speed_max_mps: float = 0.3      # MPPI must not sample above the MCU clamp
    follow_goal_checker_id: str = 'coverage_goal_checker'
    # Turn legs (reverse sub-paths, and any sub-path chunk no longer than
    # turn_leg_max_m: the forward pieces of a Reeds-Shepp turn) use this checker:
    # coverage_goal_checker (xy 0.25 m) finishes a 0.3 m leg at its start.
    # '' = use follow_goal_checker_id for everything.
    turn_leg_goal_checker_id: str = 'coverage_leg_goal_checker'
    turn_leg_max_m: float = 1.2
    # 2026-10-13 (swath-end pivots, 11:25 log): a follow_path retry ("aborted at pose 14,
    # retry 1/3") or a stuck-escape resume of a nearly finished turn leg re-sent a 1-3 pose
    # goal, and RPP's first move toward a carrot a few cm away was a rotation in place (often
    # the opposite way to the last one). Turn leg here = a reverse sub-path no longer than
    # turn_leg_max_m, or the forward piece that ends at the cusp of one. When the robot is within turn_leg_skip_m
    # of that leg's end and faces within turn_leg_skip_yaw (rad) of the end pose's yaw, the leg
    # counts as driven and the next leg (which starts at that cusp) is sent instead. Mirrors
    # coverage_leg_goal_checker overshoot_xy_tolerance / yaw_goal_tolerance (nav2_params.yaml),
    # which would have ended the leg there anyway. 0 = off.
    turn_leg_skip_m: float = 0.20
    turn_leg_skip_yaw: float = 0.5
    # 2026-10-14 run 2 (fix 10): a 0.8 m Reeds-Shepp turn piece reached its last pose but
    # neither the leg goal checker (xy 0.08 + yaw 0.5) nor the skip above (0.2 m + 0.5 rad)
    # was met with the heading ~10 deg off and the pivots slipping: MPPI looped for 7 min
    # (~4000 deg of rotation). Once a turn leg's end is reached in xy (tracked pose index at
    # its last pose, or within turn_leg_skip_m of the end pose) the leg is done ("next leg
    # from here") after turn_leg_done_after_s on the leg OR turn_leg_done_after_rot_rad of
    # accumulated |yaw change| since, whatever the yaw. Turn leg = reverse leg, the forward
    # piece ending at a reverse leg's cusp, or a sub-path no longer than turn_leg_max_m. An end
    # yaw within tolerance stays with the leg goal checker / the skip above. 0 = that cap off.
    turn_leg_done_after_s: float = 20.0
    turn_leg_done_after_rot_rad: float = 6.28
    # Task 0.4 (f), 2026-10-14 run 3: the 49 m outer ring ends at the cusp of a reverse leg,
    # so it counted as a turn leg; progress_local jumped to its last pose (the nearest-index
    # search matched the ring's own end) 4.6 m before the end, the cap fired and 4.6 m of
    # ring were skipped. True: "end reached" is the distance to the leg end only
    # (turn_leg_skip_m, never progress_local) and a leg is capped only if its OWN length is
    # <= turn_leg_max_m (turn_leg_skip keeps its rule). False = the fix 10 rule (revert flag).
    turn_leg_end_by_distance: bool = True
    # Any sub-path: a follow_path that has not advanced the tracked pose index for this long
    # (Nav2's progress checker is satisfied by looping) is cancelled and handled as a
    # follow_path abort (follow_retries). Waits / holds / escapes cancel the goal, so they
    # never count. 0 = off.
    leg_loop_abort_s: float = 90.0
    blade_confirm_timeout_s: float = 5.0
    blade_start_attempts: int = 3
    blade_retry_pause_s: float = 2.0
    require_blade_confirmation: bool = True
    # MANUAL_MOWING starts blade OFF; the blade runs only after an explicit
    # manual_blade(True) request (GUI mow_enabled=1). False = upstream behaviour
    # (blade on as soon as MANUAL_MOW is entered).
    manual_blade_requires_enable: bool = False
    blade_spinup_s: float = 2.0         # used when confirmation is disabled
    # --- hard boundary (task 0.4 a; 2026-10-14 run 3: seven transit excursions up to 0.85 m
    # outside the areas and paths; owner: "going outside the area is an immediate fail") ---
    # The robot centre more than hard_boundary_m outside (mowing areas U drawn paths U the
    # dock corridor; map_server ~/outside_m, fresher than hard_boundary_stale_s) in TRANSIT,
    # MOWING or a dock phase: cancel the goal, zero, blade off, then retrace the driven EKF
    # track (the last hard_boundary_trail_m, newest first) in reverse at
    # hard_boundary_retrace_speed_mps on /cmd_vel_emergency until back inside plus
    # hard_boundary_retrace_extra_m (at most hard_boundary_retrace_max_m, never beyond the
    # recorded track: no blind reverse; inside = a fresh ~/outside_m <= 0, the track / max
    # running out while still outside or without a fresh reading -> STUCK_NEEDS_HELP). Steering keeps
    # both wheels rolling backwards (inner wheel >= hard_boundary_min_inner_mps); a heading
    # error above hard_boundary_pivot_err_rad is turned on the spot first (counter-rotating
    # pivot at hard_boundary_pivot_rps). Counts as a transit abort (TRANSIT: the transit
    # retry path; MOWING: the sub-path is resumed; dock phases: the dock is re-run); the
    # hard_boundary_max_aborts-th excursion of a mission ends in STUCK_NEEDS_HELP (after the
    # retrace). Still outside after the retrace, no progress, or no pose -> STUCK_NEEDS_HELP.
    # The heading calibration drive (unverified heading) is only stopped, not retraced.
    # hard_boundary false = off (revert flag; the MOWING-only soft pause below still runs).
    hard_boundary: bool = True
    hard_boundary_m: float = 0.10
    hard_boundary_stale_s: float = 1.0
    hard_boundary_max_aborts: int = 6
    # ... and the hard_boundary_spot_max-th excursion within hard_boundary_spot_m of earlier
    # ones (step 3 round 2: the mission cap rose from 2 to 6 with the stock-style recovery;
    # the same spot twice still ends in STUCK_NEEDS_HELP)
    hard_boundary_spot_max: int = 2
    hard_boundary_spot_m: float = 1.0
    # base_link check (drawn paths) from the mission's own area snapshot in TRANSIT / MOWING
    # (immune to a GUI map reload); ~/outside_m in dock phases / without a snapshot
    hard_boundary_snapshot: bool = True
    hard_boundary_trail_m: float = 1.5
    hard_boundary_trail_step_m: float = 0.05
    hard_boundary_pause_s: float = 0.5
    hard_boundary_retrace_speed_mps: float = 0.10
    hard_boundary_retrace_extra_m: float = 0.10
    hard_boundary_retrace_max_m: float = 1.0
    hard_boundary_lookahead_m: float = 0.25
    hard_boundary_min_inner_mps: float = 0.06
    hard_boundary_track_width_m: float = rgeo.TRACK_SPLIT_M   # MCU wheel split 0.40 (was 0.48)
    hard_boundary_pivot_err_rad: float = 0.6
    hard_boundary_pivot_rps: float = 0.3
    hard_boundary_progress_s: float = 3.0
    hard_boundary_progress_m: float = 0.05        # driven + 0.20 m x |yaw| turned
    # --- body-aware boundary-out check (step 3 round 2, stock CheckRegion / onBoundaryOut,
    # docs/analysis/2026-10-21_stock_firmware_findings.md). Where base_link is in (or < 1 m outside) a mowing area
    # (not on a drawn path only / the dock corridor / a dock phase: those keep the base_link
    # hard_boundary_m check) the 4 footprint corners + mid-front + mid-rear
    # (mower_mission.body_boundary) are checked against the allowed region (mowing areas U
    # drawn paths U dock corridor, the mission's area snapshot: a GUI map reload cannot reach
    # it). Trip when a point is further out than its limit: edge_mode hard = every point
    # hard_boundary_body_hard_m; soft = rear corners _soft_side_m, front corners
    # _soft_front_m (on a convex bend the front corners sit out by geometry alone), nose
    # _soft_nose_m, tail
    # _soft_tail_m (soft lap 0 puts the sides 0.235 out, a row end the nose 0.15 / the front
    # corners <= 0.195 out; + tracking p99 0.07..0.13). Recovery (stock): stop, blade off,
    # move hard_boundary_back_m straight (reverse; forward when the worst point is behind
    # base_link) at <= 0.10 m/s, hard_boundary_back_timeout_s; resume (the leg-start pivot
    # turns to the path heading at the nearest point). Still beyond the limit after the move
    # -> the track retrace above (fallback). The retry cap / STUCK escalation is unchanged.
    hard_boundary_body: bool = True
    hard_boundary_body_hard_m: float = 0.10
    hard_boundary_body_soft_side_m: float = 0.35
    hard_boundary_body_soft_front_m: float = 0.55   # front corners (convex-bend allowance)
    hard_boundary_body_soft_nose_m: float = 0.40
    hard_boundary_body_soft_tail_m: float = 0.30
    hard_boundary_plan_radius_m: float = 1.0      # plan-relative soft limit near a corner
    hard_boundary_plan_margin_m: float = 0.20     # ... its planned body out + this (p99 0.13 + lap-join transient)
    hard_boundary_body_clear_m: float = 0.02      # back-move success: excess <= -this
    hard_boundary_back_m: float = 0.30
    hard_boundary_back_speed_mps: float = 0.10
    hard_boundary_back_timeout_s: float = 5.0
    # A boundary reading (outside_m / body worst point) that jumps by more than
    # hard_boundary_jump_m since the previous fresh one is ignored for up to
    # hard_boundary_jump_hold_s (logged, not counted): field 2026-10-21 07:0x, a GUI map
    # reload published 23.37 m "outside" mid-reload and used a budget slot. 0 = off.
    # A recovery (straight move / retrace) that ends still outside: up to
    # hard_boundary_inward_tries bounded inward moves (<= _max_m each, _timeout_s) toward the
    # nearest boundary point, forward (nose into the area) unless the inside lies behind;
    # then STUCK_NEEDS_HELP. START / resume while outside (base_link > _entry_m, or a body
    # point within hard_boundary_body_clear_m of its limit): the inward move FIRST, no pivot,
    # not counted (field 2026-10-22 morning).
    hard_boundary_inward_tries: int = 3
    hard_boundary_inward_max_m: float = 0.6
    hard_boundary_inward_timeout_s: float = 10.0
    hard_boundary_entry_m: float = 0.02
    hard_boundary_jump_m: float = 1.0
    hard_boundary_jump_hold_s: float = 2.0
    # --- Spin drift (field 2026-10-21 07:05:32: a 64 deg route-corner Spin slid 0.40 m on the
    # west-edge slope, then the drive out tripped the boundary). base_link more than
    # spin_drift_m from where a pivot Spin started: cancel it, log, re-plan the approach
    # (coverage: transit back to the leg start; mission transit: transit retry; else the
    # Spin's own unfinished-pivot handling). The 2nd drift on the same coverage leg skips the
    # leg (blocked pivot). 0 = off.
    spin_drift_m: float = 0.15
    spin_drift_body_margin_m: float = 0.05   # ... stop only with the body this near its limit
    spin_drift_hard_m: float = 0.30          # ... or drifting more than this
    # Owner 2026-10-23: obstacle avoidance (detours, skips, retries, blocked / dynamic waits,
    # bumper skips, pivot back-outs) keeps the blade on while the robot - and a detour's
    # straight path to its rejoin point - stays inside the area by the boundary guard's body
    # rules. Off still for: leaving the area, lift / tilt / e-stop / emergency, localisation
    # hold, STOP / manual, docking / return outside, battery, STUCK, mission end. false = the
    # old blade-off behaviour.
    blade_keep_on_obstacles: bool = True
    # In-place coverage pivots without a planned way round turn the way whose swept body
    # stays further inside (toward the interior, even the longer way) when that is better by
    # more than this; < 0 = always the shorter way (sim 2026-10-22: a -116 deg leg-start pivot
    # on lap 0 swung the nose 0.40 m out where +244 deg stays inside).
    pivot_way_gain_m: float = 0.05
    # A transit to a sub-path start within edge_approach_near_m of the line (an edge-lap start
    # / resume point) ends edge_approach_inset_m inside, beside it; 0 = at the start.
    edge_approach_inset_m: float = 0.30
    edge_approach_near_m: float = 0.25
    edge_join_m: float = 1.5     # ... then the leg's first metres fade from that inset onto the
    #                              lap (tangent join, round 4); 0 = join across from the inset
    # --- repeated bumper hits (field 2026-10-21 07:29: a real object just outside the line,
    # the bumper fired 6 and 9 times in turn). bumper_skip_hits hits (rising edges of the
    # bumper_controller routing status) within bumper_skip_radius_m of each other (and within
    # bumper_skip_window_s) while following a coverage leg: cancel, skip that stretch (edge
    # lap: edge_blocked_skip_m; row: to the next pivot vertex), log + incident. In a mission
    # transit: a transit abort (re-plan). 0 = off.
    bumper_skip_hits: int = 3
    bumper_skip_radius_m: float = 1.0
    bumper_skip_window_s: float = 180.0
    boundary_recover_after_s: float = 3.0     # pause this long, then a recovery transit
    boundary_max_recoveries: int = 2          # recovery transits per sub-path before latching
    boundary_recovery_advance_m: float = 1.0  # recovery target: this far ahead on the sub-path
    boundary_recovery_retries: int = 3        # failed recovery transit re-sends, then stop in place
    progress_window_m: float = 3.0      # progress tracking searches this far ahead on the path
    follow_end_tolerance_m: float = 1.0  # FollowPath "success" with more path left = premature
    follow_chunk_m: float = 25.0        # max FollowPath goal length (0 = whole sub-path)
    follow_chunk_clearance_m: float = 0.5   # goal end kept this far from earlier chunk poses
    follow_premature_retries: int = 3
    record_rate_hz: float = 10.0
    record_min_spacing_m: float = 0.05
    record_simplify_tolerance_m: float = 0.05
    record_close_radius_m: float = 0.5
    record_min_area_m2: float = 1.0
    record_min_obstacle_area_m2: float = 0.5   # CMD_RECORD_OBSTACLE (11): no-go zones are small
    record_max_points: int = 50000
    record_path_width_m: float = 0.7      # default band width of a recorded PATH
    record_path_min_length_m: float = 1.0
    max_areas: int = 200
    result_display_s: float = 5.0
    service_timeout_s: float = 10.0
    plan_timeout_s: float = 120.0
    undock_timeout_s: float = 180.0
    transit_timeout_s: float = 300.0
    follow_timeout_s: float = 3600.0
    dock_action_timeout_s: float = 400.0
    # Contacts reported while the docking action still runs: wait for its result (the
    # server stops and confirms on contact); only after this long cancel it and treat the
    # robot as docked (2026-10-07: the mission flipped while the server was in RETRY).
    dock_contact_grace_s: float = 10.0
    # --- per-area mowing settings ---
    use_area_settings: bool = True      # read /map_server_node/get_area_settings per area
    cut_width_m: float = 0.22           # blade cut width (display only; spacing = area swath_width_m)
    cut_speed_max_mps: float = 0.5
    set_cut_speed: bool = True          # set_parameters on /controller_server per area
    set_cutter_height: bool = True      # /cutter_control height per area
    # --- obstacle avoidance (/obstacle_policy from obstacle_guard) ---
    obstacle_avoidance: bool = True     # dynamic stop-and-wait + static swath detours
    obstacle_policy_timeout_s: float = 1.0    # older policy = 'none'
    # Docking phases: within this of the dock pose, camera obstacle detections are ignored
    # (the operator stands by the dock, 2026-10-07: the goal was cancelled every ~1 s).
    # Bumper + Nav2 costmap stay active. 0 disables.
    dock_zone_radius_m: float = 2.0
    obstacle_static_memory_s: float = 3.0     # a 'static' seen this recently explains an abort
    dynamic_wait_s: float = 30.0        # person / pet: wait this long, then treat as static
    # transit / docking (blade off): shorter wait. The operator often stands near the
    # robot when pressing Home; Nav2's costmap + the controller's collision check still stop it.
    dynamic_wait_transit_s: float = 15.0
    # after a wait timed out: ignore further dynamic detections this long (rely on Nav2 /
    # the costmap marks) unless the robot moved > dynamic_ignore_move_m or the class changed.
    # Without this the same person re-triggers the wait 0.1 s after the goal is re-sent.
    dynamic_ignore_after_timeout_s: float = 60.0
    dynamic_ignore_move_m: float = 1.0
    # per-area obstacle_detection: pushed to /obstacle_guard (+ /det_range) on area start
    set_obstacle_detection: bool = True
    dynamic_wait_sensitive_factor: float = 2.0   # 'sensitive' areas wait this much longer
    # a stale-stereo costmap warning seen within this window overrides the sub_state
    stereo_stale_window_s: float = 2.0
    dynamic_clear_hold_s: float = 2.0   # policy clear this long before resuming
    detour_skip_m: float = 2.0          # detour target: this far along the path past the robot
    detour_skip_step_m: float = 0.5     # ... extended by this when the detour transit fails
    detour_max_skip_m: float = 3.5      # ... up to this
    max_detours_per_subpath: int = 5
    detour_max_leave_m: float = 0.0     # detour pose may lie this far outside the area ring
    # follow_path abort with no classified obstacle (stereo cloud / bumper marks only):
    # detour after this many plain retries at the sub-path chunk (-1 = never detour).
    # 2026-10-07: an unclassified abort detours only when it coincides with a costmap
    # collision abort (collision_abort_window_s); an unranged class alone never detours.
    detour_unclassified_after: int = 0
    collision_abort_window_s: float = 5.0
    # sub_state 'turning (slow pivot)' after the controller commands linear 0 / angular != 0
    # this long (the Tron pivots at 0.02-0.08 rad/s on turf; the GUI explains the wait)
    pivot_sub_after_s: float = 3.0
    requeue_skipped: bool = True        # re-try detour-skipped stretches at the area end
    # --- stuck guard (2026-10-07: the robot dug a hole for 45 s on a return transit) ---
    # /stuck in STUCK_PHASES: cancel, blade off, wait stuck_pause_s, then escape steps on
    # /cmd_vel_emergency (reverse / forward arc / reverse opposite arc, stuck_step_m each);
    # a step aborts without stuck_step_progress_m of EKF progress per stuck_step_progress_s.
    # Escaped -> clear costmaps, 'stuck' terrain incident, resume. stuck_max_recoveries
    # within stuck_recovery_period_s, a failed escape, or a new stuck within
    # stuck_same_spot_m of an escaped spot -> STUCK_NEEDS_HELP.
    # map_server lethal flag held off while the mission's own pose is inside the current
    # area's outer ring (+ margin), for at most lethal_disagree_max_s (see _lethal_agrees)
    lethal_agree_margin_m: float = 0.5
    lethal_disagree_max_s: float = 3.0
    stuck_guard: bool = True
    stuck_max_recoveries: int = 3
    stuck_recovery_period_s: float = 120.0
    stuck_pause_s: float = 1.0
    stuck_step_m: float = 0.3
    stuck_step_speed: float = 0.1
    stuck_arc_deg: float = 30.0
    stuck_step_progress_s: float = 3.0
    stuck_step_progress_m: float = 0.05
    stuck_escape_m: float = 0.3
    stuck_same_spot_m: float = 0.5
    # 2026-10-08: a new stuck near an escaped spot only means "still stuck there" if the
    # robot never got further than this from that spot since the escape; after it drove on
    # (mowing lanes bring it back within 0.5 m) it is a fresh event and gets its own escape.
    stuck_same_spot_left_m: float = 1.0
    stuck_ignore_after_s: float = 1.0   # /stuck ignored this long after an escape
    # --- tilt guard (2026-10-09, mower_control tilt_monitor /tilt/status; see _tilt_tick) ---
    # caution: cap the controller speed (blade stays on). limit: cancel, blade off, 'steep'
    # terrain incident, back out along the driven track on /cmd_vel_emergency, then resume
    # (mowing: detour past the spot). critical: an emergency cause (tilt_monitor also stops
    # on /cmd_vel_emergency and calls /cutter_off itself).
    tilt_guard: bool = True
    tilt_status_timeout_s: float = 2.0  # older /tilt/status -> band unknown (no action)
    tilt_caution_speed_mps: float = 0.15
    tilt_transit_speed_param: str = 'FollowPath.vx_max'  # MPPI transit
    tilt_transit_speed_mps: float = 0.3     # restored after caution (nav2_params FollowPath)
    tilt_trail_m: float = 2.0           # driven track kept for the back-out
    tilt_trail_step_m: float = 0.05
    tilt_backout_pause_s: float = 0.5
    tilt_backout_speed: float = 0.1
    tilt_backout_min_m: float = 0.3      # back-out at least this far, until the band < limit
    tilt_backout_max_m: float = 1.0     # still 'limit' after this -> STUCK_NEEDS_HELP
    tilt_backout_lookahead_m: float = 0.3
    tilt_backout_max_w: float = 0.3
    tilt_backout_progress_s: float = 3.0
    tilt_backout_progress_m: float = 0.05
    tilt_max_backouts: int = 3          # within tilt_backout_period_s -> STUCK_NEEDS_HELP
    tilt_backout_period_s: float = 300.0
    tilt_same_spot_m: float = 0.75      # limit again this close to a backed-out spot -> help
    # 2026-10-09 localization hold (_loc_hold_tick): a mow follow / transit stops with the
    # blade off while /localization/status is not 'ok' (RTK float/dropout drift budget
    # spent) and resumes after loc_resume_s of 'ok'; after loc_hold_max_s it stops in
    # place (IDLE, resume cursor kept). No status ever received / older than
    # loc_status_timeout_s = no new hold (localization_monitor off or restarting).
    loc_hold: bool = True
    loc_status_timeout_s: float = 5.0
    loc_resume_s: float = 3.0
    loc_hold_max_s: float = 600.0
    # ---- 2026-10-23 option C: no-go zones in the mission (fsm/phases/nogo.py) ----
    nogo_enabled: bool = True
    nogo_zones_max_age_s: float = 10.0       # /obstacle_tracker/nogo_zones receipt age
    # mid-mow re-plan around a zone (pending / accepted) on the remaining path of the area
    nogo_replan: bool = True
    nogo_replan_states: list = field(default_factory=lambda: ['pending', 'accepted'])
    nogo_replan_pad_m: float = 0.10          # footprint growth for the on-path test
    nogo_replan_min_interval_s: float = 60.0
    nogo_replan_max_per_area: int = 3
    nogo_replan_check_s: float = 1.0         # on-path test rate
    nogo_progress_tol_m: float = 0.02        # + swath/2: a re-plan pose that counts as mowed
    # pre-pivot sweep check (coverage pivots, K-turn about-turns, route corners)
    nogo_pivot_check: bool = True
    nogo_pivot_backup_m: float = 0.30        # (b) / (c) back-up, capped by the body limits
    nogo_pivot_shifts_m: list = field(default_factory=lambda: [0.3, 0.15, -0.15])   # (d)
    nogo_pivot_max_maneuvers: int = 2        # per pivot, then (e) re-plan / skip
    nogo_move_speed_mps: float = 0.10        # maneuver back-up / drive (both wheels >= 0.06)
    # edge laps (B6, owner): detour inward around a zone / static mark on the lap, rejoin
    nogo_edge_detour: bool = True
    nogo_edge_lookahead_m: float = 2.5
    nogo_edge_clearance_m: float = 0.10
    # bumper / blocked retries clear only the clearable 5 s obstacle layer (Nav2
    # clear_around_*_costmap; static_det_layer and bumper_layer are not clearable), not the
    # whole costmaps (the 60 s det marks and bumper marks were wiped just when they mattered)
    clear_costmaps_selective: bool = True

    @classmethod
    def from_dict(cls, d):
        p = cls()
        for k, v in (d or {}).items():
            if hasattr(p, k):
                setattr(p, k, v)
        return p


@dataclass
class Mission:
    """In-memory state of a running mowing session (rebuilt from the cursor
    on every (re)start)."""
    single_area_target: Optional[int] = None
    resume: bool = False
    areas: dict = field(default_factory=dict)   # idx -> area dict (non-navigation)
    all_area_count: int = 0
    queue: list = field(default_factory=list)
    area_idx: Optional[int] = None
    subpaths: list = field(default_factory=list)
    lengths: list = field(default_factory=list)
    cum: list = field(default_factory=list)     # per sub-path cumulative lengths
    progress_local: int = 0                     # tracked pose index while following
    sub_i: int = 0
    start_local: int = 0
    resume_sub: Optional[tuple] = None          # (sub, local) from the cursor
    retry_used: bool = False
    skipped: int = 0                            # sub-paths of the current area NOT mowed
    transit_fails: int = 0                      # per sub-path
    follow_fails: int = 0                       # per sub-path / chunk
    recovering: bool = False                    # the in-flight transit is a boundary recovery
    recovery_fails: int = 0
    retry_at: float = 0.0                       # step == 'retry_wait' until this time
    failed: list = field(default_factory=list)  # [(area, sub|None, reason, abs pose index)]
    planned_total: int = 0                      # sub-paths planned over the session
    completed_count: int = 0
    step: Optional[str] = None                  # transit | spinup | spin_pause | follow
    transit_target: Any = None                  # (x, y, yaw) of the in-flight transit
    boundary_recoveries: int = 0
    premature: int = 0
    chunk_end: int = 0
    left_dock: bool = False
    settings: Optional[dict] = None             # effective settings of the current area
    height_override_mm: Optional[int] = None    # live deck height change (this mow only)
    runs: list = field(default_factory=list)    # [{'angle': deg|-1, 'perpendicular': bool}]
    run_i: int = 0
    detours: int = 0                            # per sub-path
    detour: Optional[dict] = None               # in-flight detour {'from', 'to', 'skip', 'why'}
    stretch: Optional[tuple] = None             # re-queued (abs_from, abs_to) being re-tried
    stretch_end: Optional[int] = None           # local end index of that stretch
    requeue: Optional[list] = None              # stretches left in the end-of-area pass
    blocked: list = field(default_factory=list)  # [(area, abs_from, abs_to)] still blocked
    transit_failed: list = field(default_factory=list)  # [(area, sub)] dropped on transit failure
    requeue_subs: Optional[list] = None         # failed entries left in the end-of-area pass
    retry_entry: Optional[tuple] = None         # the m.failed entry being re-tried
    blade_transit: bool = False                 # in-flight transit keeps the blade on
    route_fallback: bool = False                # this transit uses /navigate_to_pose (no route)
    route_aborts: int = 0                       # route follow aborts of this transit
    route_plan_fails: int = 0                   # far-goal plan_route failures (re-asked)
    route_poses: Optional[list] = None          # route of the in-flight route transit
    route_full: Optional[list] = None           # the planned route (route_cusps index into it)
    route_cusps: list = field(default_factory=list)   # [(i, x, y, heading_after)] of route_full
    final_approach: bool = False                # this transit's final approach leg was sent
    join: Any = None
    route_backouts: set = field(default_factory=set)   # route corners backed off from
    unbypassable: list = field(default_factory=list)   # [(x, y)] stretches that stayed blocked
    offgraph: Any = None                        # (original goal, tries) off-graph goal re-asks                            # (sub, local, side, inset) tangent join (_join_leg)
    cusp_turns: dict = field(default_factory=dict)    # cusp (x, y) -> about-turns, per sub-path
    cusp_resume: Optional[list] = None          # route remainder sent after the about-turn
    cusp_heading: Optional[float] = None        # heading the cusp about-turn turns to
    fallback_ref: Optional[list] = None         # route of a costmap fallback (corridor bound)
    fallback_armed: Optional[bool] = None       # corridor bound active (None = not started)
    fallback_off0: float = 0.0                  # off-path distance when the fallback started
    leg_watch: Optional[dict] = None            # fix 10 leg-end cap / loop guard state
    cond: dict = field(default_factory=dict)    # sub -> {'types', 'pivots', 'corners'}
    cond_errors: dict = field(default_factory=dict)   # sub -> validation error (not mowed)
    corner_body: list = field(default_factory=list)   # [(x, y, planned body out m)] corners
    corner_end: Optional[int] = None            # the in-flight chunk ends on this pivot
    pivot_done: Optional[tuple] = None          # (area, sub, local) just pivoted for
    pivot_target: Optional[tuple] = None        # (heading, threshold) of that coverage pivot
    pivot_skips: int = 0                        # blocked coverage pivots skipped (this mission)
    pivot_backout_key: Optional[tuple] = None   # (area, sub, local) already backed out for
    edge_skipped_m: float = 0.0                 # edge-lap metres skipped (this mission)
    edge_laps: dict = field(default_factory=dict)   # (area, sub) -> is an edge lap
    # 2026-10-23 option C (fsm/phases/nogo.py)
    nogo_extra: dict = field(default_factory=dict)      # area -> zone polygons added mid-mow
    nogo_replans: dict = field(default_factory=dict)    # area -> re-plans
    nogo_last_replan: float = -1e9
    nogo_replan: Any = None                     # in-flight re-plan context
    nogo_check_at: float = -1e9
    nogo_seq: Any = None                        # pre-pivot maneuver in flight
    nogo_tries: dict = field(default_factory=dict)      # pivot key -> maneuvers
    nogo_route_aborts: set = field(default_factory=set)
    nogo_turns: int = 0                         # pivots re-routed by the sweep check
    nogo_detour: Any = None                     # edge detour in flight
    nogo_detoured: set = field(default_factory=set)
    nogo_edge_detours: int = 0


@dataclass
class _Pending:
    name: str
    token: int
    deadline: float
    purpose: Any = None
    timeout: float = 0.0        # s; mark_sent() re-stamps deadline = send time + timeout


# route_corner_turn_rad is inclusive with this tolerance (a drawn 90 deg corner measures 89.9)
ROUTE_CORNER_TOL_RAD = math.radians(1.0)

# below these the robot counts as standing still for the in-path prediction
_STILL_V = 0.02     # m/s
_STILL_W = 0.05     # rad/s


def _along(pts, poses, look):
    """``pts`` (starting at the robot) extended along ``poses`` for ``look`` metres."""
    acc, prev = 0.0, pts[-1]
    for q in poses:
        seg = geo.dist(prev, q)
        if acc + seg > look and seg > 1e-9:
            f = (look - acc) / seg
            pts.append((prev[0] + (q[0] - prev[0]) * f, prev[1] + (q[1] - prev[1]) * f))
            break
        pts.append((q[0], q[1]))
        acc += seg
        prev = q
    return pts


def _seg_dist(p, a, b):
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    t = 0.0 if L < 1e-12 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def blocked_text(n):
    """Sub_state for stretches the end-of-area pass could not mow (vendor IotNotice
    900027 'obstacle cannot be bypassed')."""
    return '%d stretch%s blocked by obstacles' % (n, '' if n == 1 else 'es')


def blade_allowed(fsm):
    """Invariant used by the tests: may the blade be commanded on right now?"""
    if fsm.phase == 'MANUAL_MOWING':
        return True
    m = fsm.mission
    if m is None:
        return False
    if fsm.phase == 'MOWING' and m.step in ('spinup', 'follow', 'corner_pivot', 'nogo_detour'):
        return True
    # continuous blade policy: waits / retries / in-area transits inside the area
    return fsm._blade_keep_ok()


def condition_plan(job):
    """Path conditioning of one coverage plan (pure; the node runs it off the FSM lock).
    ``job`` from MissionFSM._conditioning_job. Returns {'n_in', 'subpaths', 'cond',
    'cond_errors', 'logs': [(level, text)]}."""
    area, region, prm = job['area'], job['region'], job['prm']
    n_in = len(job['subpaths'])
    out, cond, cond_errors, logs = [], {}, {}, []
    # swath-end Reeds-Shepp turns (forward / reverse legs of 0.3-0.9 m) -> the swath ->
    # connector -> swath geometry they connect (roadmap 3.2: one tracker path per swath
    # pair, the turn included; conditioned below: fillets where they fit, else pivots)
    subpaths, joins = pcond.join_turn_chains(job['subpaths'], region, prm,
                                             leg_max_m=float(job['leg_max_m']))
    kept = [j for j in joins if not j.ok]
    if joins:
        logs.append(('info', 'area %d: %d swath-end reverse turn(s) replaced by their '
                             'connector corners, %d kept as planned%s' % (
                                 area, len(joins) - len(kept), len(kept),
                                 ''.join(' [%s]' % j.why for j in kept))))
    npiv = nfil = nkt = 0
    why, why_deg = {}, {}
    corner_body = []
    for k, sp in enumerate(subpaths):
        if geo.is_reverse_subpath(sp):
            out.append(sp)                 # a reverse leg of a turn kept as planned
            continue
        r = pcond.condition(sp, region, prm)
        if not r.valid:
            cond_errors[len(out)] = r.errors[0]
            logs.append(('error', 'area %d sub-path %d: path conditioning failed (%s): it '
                                  'will not be driven' % (area, len(out), r.errors[0])))
            out.append(sp)
            continue
        npiv += len(r.pivots) - 2 * len(r.kturns)
        nfil += sum(1 for c in r.corners if c.choice == 'fillet')
        for c in r.corners:
            if c.choice == 'pivot':
                why[c.why] = why.get(c.why, 0) + 1
                why_deg[c.why] = why_deg.get(c.why, 0.0) + abs(math.degrees(c.turn))
            elif c.choice == 'kturn':
                nkt += 1
                logs.append(('info', 'area %d sub-path %d: K-turn at (%.2f, %.2f) %+.0f deg: %s'
                             % (area, len(out), c.x, c.y, math.degrees(c.turn), c.why)))
            elif c.choice == 'cut':
                logs.append(('warn', 'area %d sub-path %d: corner cut at (%.2f, %.2f) %+.0f deg '
                                     '(body %s m out, tip left uncut): %s'
                             % (area, len(out), c.x, c.y, math.degrees(c.turn),
                                '%.2f' % c.body_m if c.body_m is not None else '?', c.why)))
            if c.body_m is not None and c.body_m > 0.0:
                corner_body.append((float(c.x), float(c.y), float(c.body_m)))
            if prm.body_limit_m is not None and c.body_m is not None and \
                    c.choice in ('fillet', 'pivot', 'kturn', 'cut') and \
                    c.body_m > (c.limit_m if c.limit_m is not None else prm.body_limit_m) + 1e-3:
                # nothing fits the planning limit here: the least-overhang option is driven
                logs.append(('warn', 'area %d sub-path %d: corner at (%.2f, %.2f) %+.0f deg: '
                                     '%s, body %.2f m out > planning limit %.2f (least '
                                     'overhang of fillet / pivot / K-turn)%s' % (
                                         area, len(out), c.x, c.y, math.degrees(c.turn),
                                         c.choice, c.body_m, c.limit_m or prm.body_limit_m,
                                         ' [%s]' % c.why if c.why else '')))
        # K-turns (edge layout round 2): forward to P1 / reverse onto V / forward legs, each
        # its own sub-path; the planned pivot at a leg's start (P1 about-turn, V pivot) rides
        # along as start_turn (_cov_pivot keeps its direction: the nose through the area)
        for lg in pcond.split_kturns(r):
            c = {'types': lg['types'], 'pivots': lg['pivots']}
            if lg['start_turn'] is not None:
                c['start_turn'] = float(lg['start_turn'])
            if lg['reverse']:
                c['kturn'] = True
            cond[len(out)] = c
            out.append(lg['poses'])
    logs.append(('info', 'area %d: path conditioning: %d fillets, %d pivots (%s), %d K-turns, '
                         '%d sub-path(s) failed validation' % (
                             area, nfil, npiv, ', '.join(
                                 '%s %d / %.0f deg' % (k, n, why_deg[k])
                                 for k, n in sorted(why.items())) or 'none', nkt,
                             len(cond_errors))))
    return {'n_in': n_in, 'subpaths': out, 'cond': cond, 'cond_errors': cond_errors,
            'logs': logs, 'corner_body': corner_body}
