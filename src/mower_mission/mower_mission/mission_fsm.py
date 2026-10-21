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
import math
from dataclasses import dataclass, field
from typing import Any, Optional

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
            'turn_type': str(st['turn_type'])}

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
DOCK_PHASES = ('RETURNING_HOME', 'LOW_BATTERY_DOCKING', 'RAIN_DETECTED_DOCKING',
               'COVERAGE_FAILED_DOCKING')
# Phases in which the wheels may move (published as latched /motion_enabled and
# enforced by mower_control/cmd_vel_slew's motion gate). Everything else -- IDLE*,
# CHARGING, EMERGENCY, BOUNDARY_EMERGENCY_STOP, MOWING_COMPLETE/INCOMPLETE,
# NAV_TO_DOCK_FAILED, PREFLIGHT_CHECK, RAIN_WAITING, ... -- forces zero.
# RECORDING is joystick driving (boundary recording) like MANUAL_MOWING.
MOTION_PHASES = ('MANUAL_MOWING', 'RECORDING', 'UNDOCKING', 'WAITING_FOR_RTK',
                 'PLANNING', 'TRANSIT', 'MOWING', 'AREA_UNREACHABLE',
                 'BOUNDARY_PAUSED') + DOCK_PHASES
# Phases allowed to move while the base reports docked/charging (explicit undock;
# docking itself ends on the contacts).
DOCKED_MOTION_PHASES = ('UNDOCKING', 'MANUAL_MOWING') + DOCK_PHASES   # manual: the operator may drive off the dock


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


@dataclass
class Params:
    tick_hz: float = 10.0
    emergency_timeout_s: float = 2.0
    battery_low_percent: float = 20.0
    battery_full_percent: float = 95.0
    # What a running mission does below battery_low_percent:
    #   'stop' = blade off, zero velocity, stay where it is (IDLE); 'dock' = go charge.
    battery_low_action: str = 'stop'
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
    robot_width_m: float = 0.47        # owner-measured with wheel trim (2026-10-20, was 0.54)
    # 2026-10-13 (person stopped the mower while the robot pivoted toward a detour target:
    # the straight line to the target is not where the robot goes). Where the mission does
    # not know the path the controller tracks (/navigate_to_pose transits: detours, gaps,
    # retries; return-to-dock and docking) the corridor is the robot footprint swept along
    # its PREDICTED motion: constant applied /cmd_vel (else /cmd_vel_nav) twist, younger than
    # obstacle_twist_max_age_s, for obstacle_sweep_horizon_s (arc v/w, straight, or pivot).
    # Footprint: robot_rear_m behind to robot_front_m ahead of base_link, +-robot_width_m/2,
    # grown by obstacle_path_margin_m. Anything within the circumscribed footprint radius +
    # margin of the robot is always in the path (standing still / pivoting next to it).
    obstacle_sweep_horizon_s: float = 2.0
    obstacle_twist_max_age_s: float = 0.5
    robot_front_m: float = 0.60       # nav2 footprint front (bumper, owner-measured; was 0.52)
    robot_rear_m: float = 0.12        # obstacle_guard rear_edge_x_m (rear 0.10 + 0.02; was 0.25)
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
    path_conditioning: bool = True
    path_conditioning_r_min_m: float = 0.5
    path_conditioning_r_max_m: float = 1.0
    path_conditioning_margin_m: float = 0.05
    path_conditioning_half_width_m: float = 0.235   # body half width with wheel trim (was 0.24)
    path_conditioning_slack_m: float = 0.05
    path_conditioning_pivot_min_rad: float = 0.35
    path_conditioning_edge_pivot_rad: float = 1.38
    path_conditioning_edge_clearance_m: float = 0.20
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
    # 2026-10-15 MPPI study (docs/analysis/2026-10-15_mppi_study.md): which tracker drives the
    # coverage sub-paths. 'mppi' = mppi_follow_controller_id / mppi_reverse_controller_id
    # (feasibility-aware MPPI, nav2_params.yaml), the cut speed goes to
    # mppi_controller_speed_param capped at mppi_speed_max_mps; 'rpp' =
    # follow_controller_id / reverse_controller_id / controller_speed_param, as before.
    # Default 'mppi' (2026-10-16, owner: MPPI as the single tracker; it needs
    # path_conditioning: on raw paths it overshoots the sharp ring vertices and the short
    # swath-end turn legs, study §8). 'rpp' = the revert.
    coverage_controller: str = 'mppi'
    mppi_follow_controller_id: str = 'FollowCoveragePathMPPI'
    mppi_reverse_controller_id: str = 'FollowCoveragePathReverseMPPI'
    mppi_controller_speed_param: str = 'FollowCoveragePathMPPI.vx_max'
    mppi_speed_max_mps: float = 0.3      # MPPI must not sample above the MCU clamp
    follow_controller_id: str = 'FollowCoveragePath'
    # Sub-paths whose poses face against their travel (reverse turn legs from
    # the coverage bridge) go to this controller (RPP with allow_reversing).
    reverse_controller_id: str = 'FollowCoveragePathReverse'
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
    hard_boundary_max_aborts: int = 2
    hard_boundary_trail_m: float = 1.5
    hard_boundary_trail_step_m: float = 0.05
    hard_boundary_pause_s: float = 0.5
    hard_boundary_retrace_speed_mps: float = 0.10
    hard_boundary_retrace_extra_m: float = 0.10
    hard_boundary_retrace_max_m: float = 1.0
    hard_boundary_lookahead_m: float = 0.25
    hard_boundary_min_inner_mps: float = 0.06
    hard_boundary_track_width_m: float = 0.40     # MCU wheel split (cmd_vel_slew TRACK_WIDTH_M; was 0.48)
    hard_boundary_pivot_err_rad: float = 0.6
    hard_boundary_pivot_rps: float = 0.3
    hard_boundary_progress_s: float = 3.0
    hard_boundary_progress_m: float = 0.05        # driven + 0.20 m x |yaw| turned
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
    cut_width_m: float = 0.20           # blade cut width (display only; spacing = area swath_width_m)
    cut_speed_max_mps: float = 0.5
    controller_speed_param: str = 'FollowCoveragePath.desired_linear_vel'
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
    tilt_transit_speed_param: str = 'FollowPath.vx_max'  # MPPI transit (rpp: .desired_linear_vel)
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
    cusp_turns: dict = field(default_factory=dict)    # cusp (x, y) -> about-turns, per sub-path
    cusp_resume: Optional[list] = None          # route remainder sent after the about-turn
    cusp_heading: Optional[float] = None        # heading the cusp about-turn turns to
    fallback_ref: Optional[list] = None         # route of a costmap fallback (corridor bound)
    fallback_armed: Optional[bool] = None       # corridor bound active (None = not started)
    fallback_off0: float = 0.0                  # off-path distance when the fallback started
    leg_watch: Optional[dict] = None            # fix 10 leg-end cap / loop guard state
    cond: dict = field(default_factory=dict)    # sub -> {'types', 'pivots', 'corners'}
    cond_errors: dict = field(default_factory=dict)   # sub -> validation error (not mowed)
    corner_end: Optional[int] = None            # the in-flight chunk ends on this pivot
    pivot_done: Optional[tuple] = None          # (area, sub, local) just pivoted for
    pivot_target: Optional[tuple] = None        # (heading, threshold) of that coverage pivot
    pivot_skips: int = 0                        # blocked coverage pivots skipped (this mission)


@dataclass
class _Pending:
    name: str
    token: int
    deadline: float
    purpose: Any = None
    timeout: float = 0.0        # s; mark_sent() re-stamps deadline = send time + timeout


class MissionFSM:
    """See module docstring."""

    def __init__(self, params=None, cursor=None, now=0.0, alternate_counts=None):
        self.p = params if isinstance(params, Params) else Params.from_dict(params)
        self.alternate_counts = dict(alternate_counts or {})
        self.inputs = Inputs()
        self.cursor = cursor if cursor is not None else ResumeCursor()
        self.phase = 'IDLE'
        self._charge_req = None   # last charge enable sent while docked (None = not sent)
        self._charge_req_t = 0.0
        self.sub_state = ''
        self.blade_on = False
        self._plan_shown = False
        self.mission = None
        self._now = now
        # optional wall clock (the node sets time.monotonic): request deadlines are stamped
        # from it, not from the event start, so a slow event (78 s of path conditioning on
        # the RK3588, field 2026-10-20) cannot hand a request an already-expired deadline
        self.clock = None
        self._created = now
        self._phase_since = now
        self._fx = []
        self._token = 0
        self._action = None
        self._service = None
        self._emergency = False
        self._dock_purpose = None
        self._dock_routed = None            # (phase, purpose) whose dock route already ran
        self._dock_route_tries = 0          # dock route re-sends of this dock (task 0.4 e)
        self._dock_route_end = None         # (x, y) end of the last dock route (approach pose)
        self._route_legs = None             # {'legs': [poses...], 'i': n} of a route follow
        self._route_watch = None            # route progress watch {'token', 's', 't'}
        self._pivot_tries = {}              # unfinished-pivot re-sends per pivot (key -> n)
        self._dock_contact_since = None
        self._manual_undocking = False           # standalone UNDOCK (CMD_UNDOCK) in flight
        self._resume_after = None           # None | 'rain' | 'charge'
        self._rain_since = None
        self._rain_clear_since = None
        self._boundary_since = None
        self._heading_wait_since = None
        self._hcal_wanted = False           # preflight let START through for a calibration drive
        self._hcal = None                   # heading calibration drive in progress
        self._hcal_rtk_since = None
        self._spin_attempt = 0
        self._spin_deadline = 0.0
        self._track = []
        self._last_sample = -1e9
        self._record_poly = None
        self._enum = None                   # {'purpose', 'index', 'areas', 'count'}
        self._region_areas = {}             # last area enumeration (path conditioning)
        self._record_name = ''
        self._record_kind = 'area'          # 'area' (closed boundary) | 'path' (open band)
        self._record_line = None            # simplified centreline of a PATH being saved
        self._record_index = -1             # area index the PATH gets in the map server
        self._last_status = None
        self._incomplete_text = ''
        self._dyn = None                    # dynamic-obstacle wait context
        self._path_eval = None              # last in-path judgement (_obstacle_on_path)
        self._loc = None                    # localization hold context (_loc_hold_tick)
        self._blk = None                    # path-blocked wait context (see _blocked)
        self.obstacle_decision = {'kind': 'none'}   # /mission/obstacle_decision (GUI)
        self._dyn_ignore = None             # post-timeout ignore window (see _dyn_ignored)
        self._last_static_t = -1e9
        self._pivot_since = None      # start of the current in-place pivot command
        self._preview = None                # running plan preview (see preview_plan)
        self._preview_shown = False         # a preview is drawn on /coverage/full_plan
        self._preview_id = 0
        self._stk = None                    # stuck escape in progress (see _stuck_tick)
        self._stuck_hist = []               # [[t, x, y, far]] stuck spots escaped from
        self._stuck_ignore_until = -1e9
        self._tob = None                    # tilt back-out in progress (see _tilt_tick)
        self._tilt_hist = []                # [[t, x, y]] steep spots backed out of
        self._tilt_trail = []               # [(x, y)] recent driven track, oldest first
        self._tilt_capped = False           # caution speed cap sent to the controller
        self._hb = None                     # hard-boundary retrace in progress (task 0.4 a)
        self._hb_count = 0                  # hard-boundary excursions of this mission
        self._hb_trail = []                 # [(x, y)] recent driven EKF track, oldest first

    # ==================================================================
    # presentation
    # ==================================================================
    @property
    def state(self):
        return STATE_CODES.get(self.phase, STATE_IDLE)

    def status(self):
        m = self.mission
        total = len(m.subpaths) if m and m.area_idx is not None else 0
        cov = 0.0
        if m and m.lengths and m.area_idx is not None:
            ac = self.cursor.areas.get(m.area_idx)
            tot_len = sum(m.lengths)
            if ac and tot_len > 0:
                done = sum(m.lengths[i] for i in ac.completed if i < len(m.lengths))
                if m.step == 'follow' and m.sub_i < len(m.cum) and m.sub_i not in ac.completed:
                    done += m.cum[m.sub_i][min(m.progress_local, len(m.cum[m.sub_i]) - 1)]
                cov = min(100.0, 100.0 * done / tot_len)
        ac = self.cursor.areas.get(m.area_idx) if m and m.area_idx is not None else None
        i = self.inputs
        return {
            'state': self.state,
            'state_name': self.phase,
            'sub_state_name': self.display_sub_state(),
            'current_area': m.area_idx if m and m.area_idx is not None else -1,
            'current_path': m.sub_i if m and m.area_idx is not None and m.subpaths else -1,
            'current_path_index': (m.progress_local if m.step == 'follow' else m.start_local)
            if m and m.subpaths else -1,
            'total_swaths': total,
            'completed_swaths': len(ac.completed) if ac else 0,
            'skipped_swaths': m.skipped if m else 0,
            'coverage_percent': cov,
            'gps_quality_percent': float(i.gps_quality),
            'battery_percent': float(i.battery_percent) if i.battery_percent is not None else 0.0,
            'is_charging': bool(i.is_charging),
            'emergency': self.phase == 'EMERGENCY' or self._emergency,
        }

    def motion_enabled(self):
        """True only when the wheels may move (see MOTION_PHASES)."""
        if self._emergency or self.phase not in MOTION_PHASES:
            return False
        i = self.inputs
        if (i.docked or i.is_charging) and self.phase not in DOCKED_MOTION_PHASES:
            return False
        return True

    # ==================================================================
    # effect helpers
    # ==================================================================
    def _begin(self, now):
        self._now = float(now)
        self._fx = []

    def _end(self):
        fx, self._fx = self._fx, []
        return fx

    def _log(self, level, text):
        self._fx.append(Log(level, text))

    def _incident(self, kind, detail=''):
        """Terrain-memory incident at the current pose (no-op without a pose)."""
        pose = self.inputs.pose
        if pose is None:
            return
        self._fx.append(RecordIncident(kind, float(pose[0]), float(pose[1]), str(detail)))

    def _go(self, name, sub=None):
        if self.blade_on and name not in ('MOWING', 'MANUAL_MOWING') and \
                not (name == 'TRANSIT' and self._blade_keep_ok()):
            # Safety net only: every path turns the blade off explicitly first.
            self._log('error', 'safety: blade forced off on entering %s' % name)
            self._blade_off('safety net (%s)' % name)
        if sub is not None:
            self.sub_state = sub
        if name != self.phase:
            self._log('info', 'state %s -> %s%s' % (
                self.phase, name, (' (%s)' % self.sub_state) if self.sub_state else ''))
            self.phase = name
            self._phase_since = self._now
            self._heading_wait_since = None
            self._hcal = None
            self._hcal_rtk_since = None
            if name not in ('PREFLIGHT_CHECK', 'WAITING_FOR_RTK'):
                self._hcal_wanted = False
        st = self.status()
        if st != self._last_status:
            self._last_status = st
            self._fx.append(PublishStatus(dict(st)))

    def _idle_name(self):
        if self.inputs.docked:
            return 'CHARGING' if self.inputs.is_charging else 'IDLE_DOCKED'
        return 'IDLE'

    def _resume_charge_percent(self):
        return min(float(self.p.battery_full_percent), float(self.p.battery_max_charge_percent))

    def _charge_limit_tick(self):
        """Keep the MCU charge enable in line with battery_max_charge_percent while
        docked (also re-enables charging after a stack restart on the dock)."""
        i = self.inputs
        if not i.docked:
            self._charge_req = None
            return
        b = i.battery_percent
        if b is None:
            return
        limit = float(self.p.battery_max_charge_percent)
        # Re-assert "off" every 60 s, and within 2 s whenever the MCU MEASURES charging
        # (is_charging = dock contacts + current into the battery, mcu_node 2026-10-09): the
        # docking server enables charging itself at the end of every dock (live 2026-10-09: 99 %,
        # limit 95 %, enabled 40 ms after our "off"), which the 60 s re-assert let run for a minute.
        if b >= limit and (self._charge_req is not False
                           or self._now - self._charge_req_t >= 60.0
                           or (i.is_charging and self._now - self._charge_req_t >= 2.0)):
            self._charge_req, self._charge_req_t = False, self._now
            self._log('info', 'battery %.0f%% >= max charge %.0f%%: charging disabled' % (b, limit))
            self._call(SRV_CHARGING, {'enable': False}, track=False)
        elif b <= limit - float(self.p.battery_charge_hysteresis_percent) \
                and (self._charge_req is not True
                     or (not i.is_charging and self.phase in IDLE_NAMES
                         and self._now - self._charge_req_t >= 10.0)):
            # Re-assert "on" when the MCU reports not charging 10 s after our
            # request: an undock (stopped on the dock) or the docking server
            # turns the charger off behind our back.
            self._charge_req, self._charge_req_t = True, self._now
            self._log('info', 'docked at %.0f%% (max charge %.0f%%): charging enabled' % (b, limit))
            self._call(SRV_CHARGING, {'enable': True}, track=False)

    def _go_idle(self, sub=None):
        self._resume_after = None
        if self._plan_shown:
            # clear the latched /coverage/full_plan so the GUI stops drawing the route
            self._plan_shown = False
            self._fx.append(PublishPlan([]))
        self._go(self._idle_name(), sub)

    def _blade_off(self, reason):
        if self.mission is not None:
            self.mission.blade_transit = False
        self.blade_on = False
        self._fx.append(BladeOff(reason))

    # ---- blade policy (2026-10-07 owner: "run blades unless e-stop / lift") ----
    def _continuous(self):
        m = self.mission
        return m is not None and m.settings is not None and \
            m.settings.get('blade_policy', 'continuous') == 'continuous'

    def _blade_keep_ok(self):
        """May a running blade stay on in the current mission step (continuous policy,
        inside the area)?  Used by the _go safety net and blade_allowed()."""
        m = self.mission
        if not self._continuous() or self.inputs.boundary_violation or \
                self.phase not in ('MOWING', 'TRANSIT') or m.recovering:
            return False
        if m.step in ('dyn_wait', 'retry_wait', 'blocked_wait'):
            return True
        return m.step == 'transit' and m.blade_transit

    def _soft_blade_off(self, reason):
        """Blade off unless the continuous policy keeps it running here."""
        if self.blade_on and self._continuous() and not self.inputs.boundary_violation \
                and self.mission is not None and not self.mission.recovering \
                and self.phase in ('MOWING', 'TRANSIT'):
            self._log('info', 'blade kept on (%s, blade_policy continuous)' % reason)
            return
        self._blade_off(reason)

    def _segment_in_area(self, a, b):
        m = self.mission
        area = m.areas.get(m.area_idx) if m is not None else None
        if area is None or a is None or b is None:
            return False
        d = geo.dist(a, b)
        n = max(1, int(math.ceil(d / max(0.05, float(self.p.blade_transit_sample_m)))))
        for k in range(n + 1):
            pt = (a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
            if not geo.inside_area(pt, area):
                return False
        return True

    def _transit_blade(self, target, reason, force_off=False):
        """Before a mission transit / detour: keep the blade on only for a continuous
        area whose straight robot->target segment stays inside the area polygon."""
        m = self.mission
        keep = (not force_off and self.blade_on and self._continuous()
                and not self.inputs.boundary_violation and not m.recovering
                and self._segment_in_area(self.inputs.pose, target))
        if keep:
            m.blade_transit = True
            self._log('info', 'blade kept on: %s stays inside area %d' % (reason, m.area_idx))
        else:
            self._blade_off(reason)

    def _blade_on(self):
        self.blade_on = True
        self._fx.append(BladeOn())

    def _next_token(self):
        self._token += 1
        return self._token

    def _fresh_now(self):
        """The clock a request deadline starts from: the injected clock when set (never
        earlier than the event time), else the event time."""
        c = self.clock
        if c is None:
            return self._now
        try:
            return max(self._now, float(c()))
        except Exception:  # noqa: BLE001
            return self._now

    def _start_action(self, name, goal, timeout, purpose=None):
        if self._action is not None:
            self._fx.append(CancelActions('superseded by %s' % name))
        tok = self._next_token()
        self._action = _Pending(name, tok, self._fresh_now() + float(timeout), purpose,
                                float(timeout))
        self._fx.append(StartAction(name, goal, tok))
        return tok

    def _call(self, name, request, track=True, purpose=None):
        tok = self._next_token()
        if track:
            self._service = _Pending(name, tok, self._fresh_now() + self.p.service_timeout_s,
                                     purpose, float(self.p.service_timeout_s))
        self._fx.append(CallService(name, request, tok))
        return tok

    def _cancel_all(self, reason):
        if self._preview is not None:
            self._preview_finish('failed', 'cancelled: %s' % reason)
        self._dyn = None
        self._blk = None
        self._dyn_ignore = None
        self._action = None
        self._service = None
        self._enum = None
        self._manual_undocking = False
        self._dock_routed = None
        self._fx.append(CancelActions(reason))

    def _persist(self):
        self._fx.append(SaveResume(self.cursor.dumps()))
        self._fx.append(PublishResumeAvailable(self.cursor.available))

    # ==================================================================
    # public API
    # ==================================================================
    def tick(self, now):
        self._begin(now)
        self._charge_limit_tick()
        if not self._guards():
            self._run_phase()
        return self._end()

    def command(self, cmd, now):
        """HighLevelControl. Returns ``(success, effects)``."""
        self._begin(now)
        ok = self._command(int(cmd))
        return ok, self._end()

    def manual_blade(self, enable, now):
        """Blade on/off request while MANUAL_MOWING (``~/manual_blade``).
        Returns ``(success, message, effects)``."""
        self._begin(now)
        ok, msg = self._manual_blade(bool(enable))
        self._log('info' if ok else 'warn', 'manual_blade(%s): %s' % (bool(enable), msg))
        return ok, msg, self._end()

    def start_in_area(self, area, now):
        self._begin(now)
        if self._emergency or self.phase in ('EMERGENCY', 'BOUNDARY_EMERGENCY_STOP'):
            self._log('warn', 'start_in_area(%d) refused: emergency' % area)
            return False, self._end()
        self._stuck_operator(CMD_START)
        ok = self._start(single=int(area))
        return ok, self._end()

    def clear_resume(self, now):
        """``~/clear_coverage_resume``. Returns ``(success, message, effects)``."""
        self._begin(now)
        if self.mission is not None:
            return False, 'mowing session running; stop it first', self._end()
        self.cursor = ResumeCursor()
        self._fx.append(DeleteResume())
        self._fx.append(PublishResumeAvailable(False))
        self._log('info', 'coverage resume cleared: next START mows from scratch')
        return True, 'coverage resume cleared', self._end()

    def preview_plan(self, area, now):
        """``~/preview_plan``: plan area ``area`` (PREVIEW_ALL = every mowing area,
        PREVIEW_CLEAR = clear the drawn preview) with its current settings, exactly
        as PLANNING would, WITHOUT starting a mission or changing state. The result
        goes to /coverage/full_plan (latched) and ~/preview_summary.
        Returns ``(success, message, effects)``."""
        self._begin(now)
        ok, msg = self._preview_request(int(area))
        self._log('info' if ok else 'warn', 'preview_plan(%d): %s' % (int(area), msg))
        return ok, msg, self._end()

    def mark_sent(self, token, now):
        """The node actually sent request ``token`` at ``now`` (after waiting for the
        server / the Nav2 lifecycle): its watchdog runs from here. Never shortens a
        deadline. No effects."""
        for pend in (self._action, self._service):
            if pend is not None and pend.token == token and pend.timeout > 0:
                pend.deadline = max(pend.deadline, float(now) + pend.timeout)

    def plan_conditioning_job(self, token, outcome, result, now):
        """A /plan_coverage result ``token`` arrived: the inputs of its path conditioning
        (seconds to minutes of pure Python on the RK3588) so the node can run
        ``condition_plan(job)`` WITHOUT holding the FSM lock, then feed the result back with
        ``result[CONDITIONED_KEY]``. None when nothing is to be conditioned. Re-arms the
        plan watchdog for the conditioning time. No effects."""
        a, m = self._action, self.mission
        if (a is None or a.token != token or a.name != ACT_PLAN or a.purpose == 'preview'
                or m is None or not self.p.path_conditioning or outcome != SUCCEEDED):
            return None
        subpaths = self._plan_subpaths(outcome, result or {})
        if not subpaths:
            return None
        self._now = max(self._now, float(now))
        a.deadline = max(a.deadline, float(now) + max(a.timeout, float(self.p.plan_timeout_s)))
        return self._conditioning_job(subpaths)

    def on_action_result(self, token, outcome, result=None, now=None):
        self._begin(self._now if now is None else now)
        a = self._action
        if a is None or a.token != token:
            return self._end()          # stale (cancelled / superseded)
        self._action = None
        self._handle_action(a, outcome, result or {})
        return self._end()

    def on_action_feedback(self, token, detail, now=None):
        """Docking-server feedback detail (e.g. 'searching marker: seen 0.86 m, 3.5 deg')
        -> sub_state, so the GUI shows the marker visibility while docking."""
        self._begin(self._now if now is None else now)
        a = self._action
        if (a is not None and a.token == token and a.name == ACT_DOCK and detail
                and self.phase in DOCK_PHASES and self._dyn is None):
            self.sub_state = str(detail)
        return self._end()

    def on_cutter_height(self, mm, now=None):
        """Commanded deck height from the MCU driver (latched /cutter/height_mm).

        While a mow is running (any phase with a Mission), a height that differs from
        the current area's effective height is a live change (GUI /cutter/set_height):
        it overrides ``cutter_height_mm`` for the rest of THIS mow (later sub-path /
        area plans and blade re-requests) and is NOT persisted to the saved area
        settings. The echo of the mission's own per-area height equals the effective
        height and is ignored. Returns the effects (re-latched active settings).

        2026-10-13: the override is also stored in the resume cursor (``live_height``
        row), so a resumed mow (rain / charge / stop / stuck / e-stop interrupt, which
        rebuilds the Mission) keeps it instead of re-sending the area's saved height at
        the resumed area start. A fresh start (new cursor) drops it: the saved area
        setting is the default again. The GUI's "Save to area" is the only way the area
        setting itself changes.
        """
        self._begin(self._now if now is None else now)
        m = self.mission
        try:
            hmm = self.height_mm(int(mm))
        except (TypeError, ValueError):
            return self._end()
        if m is None:
            return self._end()
        cur = (m.settings or {}).get('cutter_height_mm')
        if m.settings is not None and cur is not None and self.height_mm(cur) == hmm:
            return self._end()
        m.height_override_mm = hmm
        self.cursor.live_height_mm = hmm
        self._persist()
        self._log('info', 'live blade height %d mm (this mow only)' % hmm)
        if m.settings is not None:
            m.settings = dict(m.settings, cutter_height_mm=hmm)
            if m.runs:
                self._fx.append(PublishAreaSettings(self.active_settings()))
        return self._end()

    def on_service_result(self, token, ok, response=None, now=None):
        self._begin(self._now if now is None else now)
        s = self._service
        if s is None or s.token != token:
            return self._end()
        self._service = None
        self._handle_service(s, bool(ok), response or {})
        return self._end()

    def shutdown(self, now):
        self._begin(now)
        if self.mission is not None:
            self._interrupt_mission('shutdown')
        self._cancel_all('shutdown')
        self._blade_off('shutdown')
        self._fx.append(ZeroBurst())
        return self._end()

    def initial_effects(self, now):
        """Effects to run once at node start (latched resume flag, status)."""
        self._begin(now)
        self._fx.append(PublishResumeAvailable(self.cursor.available))
        self._go(self.phase)
        return self._end()

    # ==================================================================
    # plan preview (~/preview_plan): PLANNING without a mission
    # ==================================================================
    def _preview_refusal(self):
        if self._emergency or self.phase in ('EMERGENCY', 'BOUNDARY_EMERGENCY_STOP'):
            return 'emergency active'
        if self.mission is not None:
            return 'mowing session running'
        if self.phase not in PREVIEW_PHASES:
            return 'not idle (%s)' % self.phase
        if self._enum is not None:
            return 'busy (loading areas)'
        if (self._action is not None and self._action.purpose != 'preview') or \
                (self._service is not None and not str(self._service.purpose).startswith(
                    'preview_')):
            return 'busy'
        return ''

    def _preview_request(self, area):
        why = self._preview_refusal()
        if why:
            return False, 'refused: %s' % why
        if self._preview is not None:
            self._preview_drop('superseded by a new preview')
        if area == PREVIEW_CLEAR:
            self._preview_drop('cleared', clear=True)
            return True, 'preview cleared'
        if area != PREVIEW_ALL and not 0 <= area < int(self.p.max_areas):
            return False, 'refused: bad area index %d' % area
        self._preview_id += 1
        self._preview = {'id': self._preview_id, 'area': -1 if area == PREVIEW_ALL else area,
                         'index': 0, 'areas': {}, 'queue': [], 'cur': None, 'results': []}
        self._fx.append(PublishPreviewSummary({
            'id': self._preview_id, 'status': 'planning', 'area': self._preview['area']}))
        self._call(SRV_GET_AREA, {'index': 0}, purpose='preview_enum')
        return True, 'planning preview of %s' % (
            'all areas' if area == PREVIEW_ALL else 'area %d' % area)

    def _preview_drop(self, why, clear=False):
        """Forget a running preview (its pending calls become stale) and, when
        ``clear`` or a preview is drawn, clear /coverage/full_plan + the summary."""
        pv = self._preview
        if pv is not None:
            self._preview = None
            if self._action is not None and self._action.purpose == 'preview':
                self._action = None
                self._fx.append(CancelActions('preview %s' % why))
            if self._service is not None and str(self._service.purpose).startswith('preview_'):
                self._service = None
        if pv is not None or self._preview_shown or clear:
            if self._preview_shown or clear:
                self._fx.append(PublishPlan([]))
            self._preview_shown = False
            self._fx.append(PublishPreviewSummary({
                'id': self._preview_id, 'status': 'cleared', 'message': why}))

    def _preview_on_service(self, purpose, ok, resp):
        pv = self._preview
        if pv is None:
            return
        if purpose == 'preview_enum':
            if ok and resp.get('success'):
                a = resp.get('area', {})
                if not a.get('is_navigation_area', False):
                    pv['areas'][pv['index']] = a
                pv['index'] += 1
                if pv['index'] < self.p.max_areas:
                    self._call(SRV_GET_AREA, {'index': pv['index']}, purpose='preview_enum')
                    return
            if not ok and pv['index'] == 0:
                self._preview_finish('failed', '/map_server_node/get_mowing_area unavailable')
                return
            if pv['area'] >= 0:
                if pv['area'] not in pv['areas']:
                    self._preview_finish('failed', 'area %d does not exist or is a navigation '
                                                   'area' % pv['area'])
                    return
                pv['queue'] = [pv['area']]
            else:
                pv['queue'] = sorted(pv['areas'])
            if not pv['queue']:
                self._preview_finish('failed', 'no mowing areas defined')
                return
            self._preview_next()
        elif purpose == 'preview_settings':
            cur = pv['cur']
            st, src = self._merge_settings(ok, resp, area_idx=cur['index'])
            cur['settings'], cur['source'] = st, src
            cur['runs'] = self._build_runs(st, area_name=cur['name'])
            run = cur['runs'][0]
            self._call(SRV_SET_PARAMS, {'node': PARAM_NODE_COVERAGE, 'params': {
                'operation_width': self._swath_spacing(st),
                'headland_rings': int(st['perimeter_laps']),
                'path_mode': str(st['path_mode']),
                'mow_angle_deg': float(run['angle']),
                'edge_first': bool(st['edge_first']),
                **coverage_route_params(st)}}, purpose='preview_params')
        elif purpose == 'preview_params':
            cur = pv['cur']
            if not ok:
                self._log('warn', 'preview area %d: /coverage_server set_parameters failed (%s): '
                                  'planning with its current parameters'
                          % (cur['index'], resp.get('message') or 'unavailable'))
            area, run = pv['areas'][cur['index']], cur['runs'][0]
            self._start_action(ACT_PLAN, {
                'outer_boundary': list(area.get('outer', [])),
                'obstacles': [list(o) for o in area.get('obstacles', [])],
                'mow_angle_deg': float(run['angle']),
                'perpendicular': bool(run['perpendicular'])}, self.p.plan_timeout_s,
                purpose='preview')

    def _preview_next(self):
        pv = self._preview
        if not pv['queue']:
            self._preview_finish('ok', '')
            return
        idx = pv['queue'].pop(0)
        pv['cur'] = {'index': idx, 'name': str(pv['areas'][idx].get('name', ''))}
        if self.p.use_area_settings:
            self._call(SRV_GET_AREA_SETTINGS, {'index': idx}, purpose='preview_settings')
        else:
            self._preview_on_service('preview_settings', False, {})

    def _preview_on_plan(self, outcome, result):
        pv = self._preview
        if pv is None:
            return
        cur = pv['cur']
        area = pv['areas'][cur['index']]
        st = cur['settings']
        subpaths = []
        if outcome == SUCCEEDED and result.get('success'):
            subpaths = [list(sp) for sp in result.get('drivable_subpaths', []) if len(sp) >= 2]
            if not subpaths:
                subpaths = [list(sp) for sp in result.get('segments', []) if len(sp) >= 2]
            if not subpaths and len(result.get('full_path', [])) >= 2:
                subpaths = [list(result['full_path'])]
        info = {'index': cur['index'], 'name': cur['name'], 'path_mode': st['path_mode'],
                'headland_rings': int(st['perimeter_laps']), 'edge_first': bool(st['edge_first']),
                'mow_angle_deg': float(cur['runs'][0]['angle']),
                'perpendicular': bool(cur['runs'][0]['perpendicular']),
                'runs': len(cur['runs']), 'settings_source': cur['source'],
                'operation_width_m': round(self._swath_spacing(st), 4),
                'swath_width_m': round(self._swath_spacing(st), 4),
                'edge_margin_m': round(max(0.0, float(st['edge_margin_m'])), 4)}
        if not subpaths:
            info['error'] = str(result.get('message') or outcome)
            info.update({'rings': 0, 'swaths': 0, 'length_m': 0.0, 'sub_paths': 0,
                         'inset_m': None})
            pv['results'].append((info, [], [], []))
            self._preview_next()
            return
        segs = [list(sp) for sp in result.get('segments', []) if len(sp) >= 2]
        types = list(result.get('segment_types', []))
        if len(types) != len(segs):
            types = []
        rings = int(result.get('ring_count', types.count(SEGMENT_RING) if types else 0))
        swaths = int(result.get('swath_count', types.count(SEGMENT_SWATH) if types else 0))
        length = float(result.get('total_distance') or 0.0) or \
            sum(geo.path_length(sp) for sp in subpaths)
        outer = list(area.get('outer', []))
        probe = [p for s, t in zip(segs, types) if t == SEGMENT_RING for p in s] or \
            [p for sp in subpaths for p in sp]
        step = max(1, len(probe) // 400)
        inset = min((geo.distance_to_ring(p, outer) for p in probe[::step]), default=None) \
            if len(outer) >= 3 else None
        info.update({'rings': rings, 'swaths': swaths, 'length_m': round(length, 2),
                     'sub_paths': len(subpaths),
                     'inset_m': None if inset is None else round(inset, 3)})
        full = result.get('full_path') or [p for sp in subpaths for p in sp]
        pv['results'].append((info, list(full), list(zip(types, segs)), subpaths))
        self._preview_next()

    def _preview_finish(self, status, message):
        pv, self._preview = self._preview, None
        if self._action is not None and self._action.purpose == 'preview':
            self._action = None
        if self._service is not None and str(self._service.purpose).startswith('preview_'):
            self._service = None
        results = pv['results'] if pv else []
        infos = [r[0] for r in results]
        planned = [r for r in results if r[1]]
        if status == 'ok' and not planned:
            status = 'failed'
            message = '; '.join('area %d: %s' % (i['index'], i.get('error', '?'))
                                for i in infos) or 'nothing planned'

        def pts(seq):
            return [[round(float(p[0]), 2), round(float(p[1]), 2)] for p in seq]

        full, segments, transits, prev_end = [], [], [], None
        for _info, path, typed, subpaths in planned:
            full.extend(path)
            for t, s in typed:
                segments.append({'type': 'ring' if t == SEGMENT_RING else 'swath',
                                 'points': pts(s)})
            for sp in subpaths:
                if prev_end is not None and geo.dist(prev_end, sp[0]) > 1e-3:
                    transits.append(pts([prev_end, sp[0]]))
                prev_end = sp[-1]
            if not typed:
                segments.extend({'type': 'swath', 'points': pts(sp)} for sp in subpaths)
        insets = [i['inset_m'] for i in infos if i.get('inset_m') is not None]
        summary = {
            'id': pv['id'] if pv else self._preview_id, 'status': status,
            'area': pv['area'] if pv else -1, 'message': message,
            'areas': infos,
            'rings': sum(i['rings'] for i in infos), 'swaths': sum(i['swaths'] for i in infos),
            'length_m': round(sum(i['length_m'] for i in infos), 2),
            'sub_paths': sum(i['sub_paths'] for i in infos),
            'inset_m': min(insets) if insets else None,
            'segments': segments, 'transits': transits}
        if planned:
            self._fx.append(PublishPlan(list(full)))
            self._preview_shown = True
            self._plan_shown = False    # the mission's _go_idle must not clear the preview
        self._fx.append(PublishPreviewSummary(summary))
        self._log('info' if status == 'ok' else 'warn',
                  'plan preview %s: %d area(s), %d rings, %d swaths, %.1f m%s' % (
                      status, len(infos), summary['rings'], summary['swaths'],
                      summary['length_m'], (' (%s)' % message) if message else ''))

    # ==================================================================
    # guards (upstream ReactiveSequence), evaluated every tick
    # ==================================================================
    def _emergency_cause(self):
        i = self.inputs
        causes = []
        if i.emergency_active:
            causes.append(i.emergency_reason or 'emergency')
        if i.lift:
            causes.append('lift')
        if i.stop_button:
            causes.append('stop button')
        if i.critical_nodes_down:
            causes.append('node down: %s' % i.critical_nodes_down)
        if self._tilt_band() == 'critical':
            causes.append('tilt critical (%s)' % self._tilt_text())
        stamp = i.emergency_stamp if i.emergency_stamp is not None else self._created
        if self._now - stamp > self.p.emergency_timeout_s:
            causes.append('/hardware_bridge/emergency silent > %.1fs' % self.p.emergency_timeout_s)
        return ', '.join(causes)

    def _guards(self):
        """Returns True when a guard owns this tick (phase logic skipped)."""
        self._stuck_watch()
        self._tilt_watch()
        self._hard_boundary_watch()
        cause = self._emergency_cause()
        if cause:
            if not self._emergency:
                self._emergency = True
                self._enter_emergency(cause)
            elif self.sub_state != cause and self.phase == 'EMERGENCY':
                self._go('EMERGENCY', cause)
            return True
        if self._emergency:
            self._emergency = False
            if self.phase == 'EMERGENCY':
                self._log('info', 'emergency cleared (mowing is not auto-resumed)')
                self._go_idle('emergency cleared')
            return True

        if self._stuck_tick():
            return True
        if self._tilt_tick():
            return True
        if self._hard_boundary_tick():
            return True

        i = self.inputs
        # Soft / lethal boundary rules (map_server flags, mowing areas only) apply only where
        # the blade may be on; the hard boundary above (task 0.4 a, areas U paths U dock
        # corridor at 0.10 m) covers every autonomous driving phase. PLANNING right
        # after an undock and blade-off TRANSITs (to/from the dock, along drawn
        # paths, the return corridor) are legitimately outside the mowing areas:
        # the Nav2 keepout mask bounds them (2026-10-07: a lethal stop fired in
        # PLANNING on the dock, 3.7 m from the area, before the robot moved).
        if self.phase in ('MOWING', 'BOUNDARY_PAUSED') and self.mission is not None:
            if self._lethal_agrees():          # flag + own-pose cross-check
                self._boundary_stop('lethal boundary violation')
                return True
            # A soft violation pauses only while the blade may be on (MOWING); a
            # blade-off TRANSIT is exactly how the robot gets back inside.
            if i.boundary_violation and self.phase == 'MOWING':
                self._boundary_pause()
                return True

        if self.p.rain_mode == 1 and self.phase in MISSION_PHASES and self.mission is not None:
            if i.rain:
                if self._rain_since is None:
                    self._rain_since = self._now
                if self._now - self._rain_since >= self.p.rain_debounce_s:
                    self._rain_since = None
                    self._log('warn', 'rain detected: docking, then waiting %.0f min'
                              % self.p.rain_delay_minutes)
                    self._interrupt_mission('rain')
                    self._dock('RAIN_DETECTED_DOCKING', 'rain')
                    return True
            else:
                self._rain_since = None

        b = i.battery_percent
        if b is not None and b < self.p.battery_low_percent and self.phase in MISSION_PHASES \
                and self.mission is not None:
            if str(self.p.battery_low_action).strip().lower() == 'dock':
                self._log('warn', 'battery %.0f%% < %.0f%%: docking to charge'
                          % (b, self.p.battery_low_percent))
                self._interrupt_mission('low battery')
                self._dock('LOW_BATTERY_DOCKING', 'battery')
                return True
            self._log('warn', 'battery %.0f%% < %.0f%%: stopping in place (battery_low_action=stop)'
                      % (b, self.p.battery_low_percent))
            self._interrupt_mission('low battery')
            self._cancel_all('low battery')
            self._blade_off('low battery')
            self._fx.append(ZeroBurst())
            self._dock_purpose = None
            self._go_idle('low battery %.0f%%: stopped' % b)
            return True
        return False

    def _boundary_pause(self):
        self._cancel_all('boundary violation')
        self._blade_off('boundary violation')
        self._fx.append(ZeroBurst())
        self._boundary_since = self._now
        # the robot is usually just outside the nav mask edge: let the costmaps
        # refresh during boundary_recover_after_s before any recovery transit.
        self._clear_costmaps()
        pose = self.inputs.pose
        self._log('warn', 'boundary violation at %s: mowing paused, blade off' % (
            '(%.2f, %.2f)' % (pose[0], pose[1]) if pose is not None else 'unknown pose'))
        self.mission.step = None
        self._go('BOUNDARY_PAUSED', 'boundary violation')

    def _enter_emergency(self, cause):
        self._log('error', 'EMERGENCY: %s -> cancel actions, blade off, zero velocity' % cause)
        if self.mission is not None:
            self._interrupt_mission('emergency')
        self._cancel_all('emergency')
        self._blade_off('emergency')
        self._fx.append(ZeroBurst())
        self._track = []
        self._record_poly = None
        self._resume_after = None
        self._dock_purpose = None
        self._go('EMERGENCY', cause)

    def _lethal_agrees(self):
        """map_server's lethal flag cross-checked with the mission's own pose: while that
        pose lies inside the current area's outer ring (+ lethal_agree_margin_m) the flag
        is held off, for at most lethal_disagree_max_s (an area edited smaller mid-mow must
        still stop the robot). No pose / no area: trust the flag (2026-10-07 incident: one
        injected (27, 0) pose latched BOUNDARY_EMERGENCY_STOP)."""
        if not self.inputs.lethal_boundary_violation:
            self._lethal_disagree_since = None
            return False
        m = self.mission
        area = m.areas.get(m.area_idx) if m is not None else None
        pose = self.inputs.pose
        if area is None or pose is None or not (area.get('outer') or []):
            return True
        outer_only = {'outer': area.get('outer')}
        if not geo.inside_area(pose[:2], outer_only, float(self.p.lethal_agree_margin_m)):
            self._lethal_disagree_since = None
            return True
        since = getattr(self, '_lethal_disagree_since', None)
        if since is None:
            self._lethal_disagree_since = since = self._now
            self._log('warn', 'lethal boundary flag ignored: mission pose (%.2f, %.2f) is inside '
                      'area %s' % (pose[0], pose[1], m.area_idx))
        return self._now - since >= float(self.p.lethal_disagree_max_s)

    def _boundary_stop(self, why):
        self._log('error', 'BOUNDARY_EMERGENCY_STOP: %s (STOP or RESET_EMERGENCY to clear)' % why)
        if self.mission is not None:
            self._interrupt_mission(why)
        self._cancel_all(why)
        self._blade_off(why)
        self._fx.append(ZeroBurst())
        self._go('BOUNDARY_EMERGENCY_STOP', why)

    # ==================================================================
    # per-phase tick logic
    # ==================================================================
    def _run_phase(self):
        ph, now, i = self.phase, self._now, self.inputs
        a = self._action
        if a is not None and now > a.deadline:
            self._log('warn', '%s timed out after %.0fs: cancelling'
                      % (a.name, self._timeout_of(a.name)))
            self._fx.append(CancelActions('%s watchdog' % a.name))
            self._action = None
            self._handle_action(a, TIMEOUT, {})
            return
        s = self._service
        if s is not None and now > s.deadline:
            self._service = None
            self._log('warn', '%s timed out' % s.name)
            self._handle_service(s, False, {})
            return
        if self._ranged_static():
            self._last_static_t = now
        self._pivot_track(now)
        if self._dynamic_tick():
            return
        if self._loc_hold_tick():
            return
        if self._blocked_tick():
            return
        if self._route_progress_tick():
            return

        if ph == 'MANUAL_MOWING':
            self._manual_tick()
            return

        if ph in IDLE_NAMES:
            if self._resume_after == 'charge':
                b = i.battery_percent
                full = self._resume_charge_percent()
                if b is not None and b >= full:
                    self._log('info', 'battery %.0f%% >= %.0f%%: resuming mowing' % (b, full))
                    self._resume('charge')
                elif ph != 'CHARGING':
                    self._go('CHARGING')
            elif ph != self._idle_name():
                self._go(self._idle_name())
        elif ph in DISPLAY_NAMES:
            if now - self._phase_since >= self.p.result_display_s:
                self._go_idle()
        elif ph in DOCK_PHASES and a is not None and a.purpose == 'dock':
            if self._at_dock():
                if self._dock_contact_since is None:
                    self._dock_contact_since = now
                    self._log('info', 'dock contacts reported: waiting for the docking result')
                elif now - self._dock_contact_since >= self.p.dock_contact_grace_s:
                    self._log('warn', 'docked for %.0f s but the docking action is still '
                              'running: cancelling it' % self.p.dock_contact_grace_s)
                    self._fx.append(CancelActions('docked (contacts)'))
                    self._action = None
                    self._dock_contact_since = None
                    self._dock_done()
            else:
                self._dock_contact_since = None
        elif ph == 'RAIN_WAITING':
            if i.rain and self.p.rain_mode == 1:
                self._rain_clear_since = None
            elif self._rain_clear_since is None:
                self._rain_clear_since = now
            elif now - self._rain_clear_since >= self.p.rain_delay_minutes * 60.0:
                self._log('info', 'dry for %.0f min: resuming mowing' % self.p.rain_delay_minutes)
                self._resume('rain')
        elif ph == 'WAITING_FOR_RTK':
            if self._hcal is not None:
                self._hcal_tick()
                return
            if i.fix_type is not None and i.fix_type >= self.p.rtk_fix_type:
                # a START that came through preflight for a calibration drive plans only on a
                # fresh heading; the dock path (undock seeds 'dock') is unchanged
                if self._heading_plan_ok() if self._hcal_wanted else self.heading_ok():
                    self._log('info', 'RTK fixed, heading aligned (%s): planning'
                              % i.heading_source)
                    self._next_area()
                    return
                if self._hcal_wanted:
                    self._hcal_maybe_start()
                    return
                if self._heading_wait_since is None:
                    self._heading_wait_since = now
                    self._log('info', 'RTK fixed: waiting for heading alignment')
                    self._go('WAITING_FOR_RTK', 'waiting for heading alignment')
                elif now - self._heading_wait_since > self.p.heading_wait_s:
                    self._heading_failed()
            elif now - self._phase_since > self.p.rtk_timeout_s:
                if self._hcal_wanted and not self._heading_plan_ok():
                    # no dock / Nav2 transit with an unverified heading
                    self._hcal_fail('no RTK fixed within %.0fs' % self.p.rtk_timeout_s)
                else:
                    self._mission_failed('no RTK fixed within %.0fs' % self.p.rtk_timeout_s)
            else:
                self._hcal_rtk_since = None
        elif ph in ('MOWING', 'TRANSIT') and self.mission is not None \
                and self.mission.step == 'retry_wait':
            if now >= self.mission.retry_at:
                self._retry_fire()
        elif ph == 'MOWING' and self.mission is not None:
            if self.mission.step == 'follow':
                self._track_progress()
                if self._leg_watch():
                    return
            self._tick_blade()
        elif ph == 'TRANSIT' and self.mission is not None and self.mission.step == 'transit':
            if self._transit_nav_progress_tick():
                return
            self._fallback_corridor_tick()
        elif ph == 'BOUNDARY_PAUSED' and self.mission is not None:
            m = self.mission
            if not i.boundary_violation:
                self._log('info', 'boundary violation cleared: resuming the sub-path')
                m.start_local = self._track_progress()
                self._dispatch()
            elif now - self._boundary_since >= self.p.boundary_recover_after_s:
                if m.boundary_recoveries >= self.p.boundary_max_recoveries:
                    self._boundary_stop('still outside the boundary after %d recovery transits'
                                        % m.boundary_recoveries)
                    return
                m.boundary_recoveries += 1
                m.recovering, m.recovery_fails = True, 0
                sp = m.subpaths[m.sub_i]
                near = self._track_progress()
                m.start_local = geo.advance_index(sp, near, self.p.boundary_recovery_advance_m)
                self._log('warn', 'still outside the boundary: blade-off recovery transit %d/%d '
                                  'to sub-path pose %d' % (m.boundary_recoveries,
                                                           self.p.boundary_max_recoveries,
                                                           m.start_local))
                self._dispatch(force_transit=True)
        elif ph == 'RECORDING':
            self._tick_record()

    def _timeout_of(self, name):
        if self._hcal is not None and name in (ACT_DRIVE, ACT_BACKUP):
            return self.p.heading_calibration_timeout_s
        return {ACT_PLAN: self.p.plan_timeout_s, ACT_UNDOCK: self.p.undock_timeout_s,
                ACT_BACKUP: self.p.undock_timeout_s, ACT_NAV: self.p.transit_timeout_s,
                ACT_FOLLOW: self.p.follow_timeout_s,
                ACT_DOCK: self.p.dock_action_timeout_s}.get(name, 60.0)   # ACT_SPIN: per goal

    # ==================================================================
    # commands
    # ==================================================================
    def _command(self, cmd):
        self._stuck_operator(cmd)
        if cmd == CMD_RESET_EMERGENCY:
            self._call(SRV_CLEAR_ESTOP, {}, track=False)
            self._log('info', 'reset emergency: /clear_estop requested')
            if self.phase == 'BOUNDARY_EMERGENCY_STOP':
                self._go_idle('boundary stop reset')
            return True
        if cmd == CMD_STOP:
            self._stop()
            return True
        if self._emergency or self.phase == 'EMERGENCY':
            self._log('warn', 'command %d refused: emergency active' % cmd)
            return False
        if self.phase == 'BOUNDARY_EMERGENCY_STOP':
            self._log('warn', 'command %d refused: boundary emergency stop latched '
                              '(send STOP or RESET_EMERGENCY)' % cmd)
            return False
        if cmd == CMD_START:
            return self._start(single=None)
        if cmd == CMD_HOME:
            return self._home()
        if cmd == CMD_RECORD_AREA:
            return self._record_start()
        if cmd == CMD_RECORD_PATH:
            return self._record_start(kind='path')
        if cmd == CMD_RECORD_OBSTACLE:
            return self._record_start(kind='obstacle')
        if cmd == CMD_RECORD_FINISH:
            return self._record_finish()
        if cmd == CMD_RECORD_CANCEL:
            if self.phase != 'RECORDING':
                self._log('warn', 'RECORD_CANCEL refused: not recording')
                return False
            self._track = []
            self._record_poly = None
            self._enum = None
            self._service = None
            self._log('info', 'recording cancelled, trajectory discarded')
            self._fx.append(PublishTrajectory([]))
            self._go_idle('recording cancelled')
            return True
        if cmd == CMD_MANUAL_MOW:
            return self._manual_mow()
        if cmd == CMD_UNDOCK:
            return self._manual_undock()
        self._log('warn', 'command %d not supported' % cmd)
        return False

    def _leave_current_mode(self, why):
        """Blade off and drop recording / holds before entering another mode."""
        if self.phase == 'MANUAL_MOWING' or self.blade_on:
            self._blade_off(why)
        if self.phase == 'RECORDING':
            self._track = []
            self._record_poly = None
            self._enum = None
            self._service = None
            self._fx.append(PublishTrajectory([]))
        self._resume_after = None

    def _stop(self):
        self._log('info', 'STOP: cancel actions, blade off, zero velocity')
        if self.mission is not None:
            self._interrupt_mission('stop')
        self._cancel_all('stop')
        self._blade_off('stop')
        self._fx.append(ZeroBurst())
        if self.phase == 'RECORDING':
            self._track = []
            self._record_poly = None
            self._fx.append(PublishTrajectory([]))
        self._resume_after = None
        self._dock_purpose = None
        self._manual_undocking = False
        if self.phase != 'EMERGENCY':
            self._go_idle('stopped')

    def _home(self):
        if self.phase in DOCK_PHASES:
            return True
        self._hb_count = 0
        self._leave_current_mode('home')
        if self.mission is not None:
            self._interrupt_mission('home')
        self._dock('RETURNING_HOME', 'home')
        return True

    def _manual_mow(self):
        if self.state == STATE_AUTONOMOUS:
            self._log('warn', 'MANUAL_MOW refused: autonomous run active (STOP first)')
            return False
        if self.phase == 'RECORDING':
            self._leave_current_mode('manual mow')
        self._resume_after = None
        if self.p.manual_blade_requires_enable:
            # Two-step manual: drive first; the blade needs manual_blade(True).
            if self.blade_on:
                self._blade_off('manual mow (re-entered)')
            self._go('MANUAL_MOWING', SUB_MANUAL_BLADE_OFF)
            return True
        # Upstream order: publish state 4 first, then enable the blade.
        self._go('MANUAL_MOWING', SUB_MANUAL_BLADE_ON)
        self._blade_on()
        return True

    def _manual_blade_veto(self):
        """Why the blade may not run in MANUAL_MOWING right now ('' = allowed)."""
        cause = self._emergency_cause()
        if cause:
            return 'emergency: %s' % cause
        if self.inputs.is_charging:
            return 'charging'
        if self.inputs.docked:
            return 'docked'
        if self._tilt_band() in ('limit', 'critical'):
            return 'steep slope (%s)' % self._tilt_text()
        return ''

    def _manual_blade(self, enable):
        if self.phase != 'MANUAL_MOWING':
            return False, 'not in MANUAL_MOWING (%s)' % self.phase
        if not enable:
            if self.blade_on:
                self._blade_off('manual: blades stop requested')
            self._go('MANUAL_MOWING', SUB_MANUAL_BLADE_OFF)
            return True, 'blade off'
        veto = self._manual_blade_veto()
        if veto:
            return False, 'refused: %s' % veto
        if not self.blade_on:
            self._blade_on()
        self._go('MANUAL_MOWING', SUB_MANUAL_BLADE_ON)
        return True, 'blade on'

    def _manual_tick(self):
        """Two-step manual: a docked / charging robot never keeps the blade on
        (emergency, lift and stop button are handled by the guards)."""
        if self.p.manual_blade_requires_enable and self.blade_on:
            veto = self._manual_blade_veto()
            if veto:
                self._log('warn', 'manual: blade off (%s)' % veto)
                self._blade_off('manual: %s' % veto)
                self._go('MANUAL_MOWING', SUB_MANUAL_BLADE_OFF)

    # ==================================================================
    # start / resume
    # ==================================================================
    def _preflight(self):
        i = self.inputs
        self._hcal_wanted = False
        if self._emergency_cause():
            return 'emergency active'
        if i.battery_percent is None:
            return 'battery level unknown'
        if i.battery_percent <= self.p.battery_low_percent:
            return 'battery %.0f%% <= %.0f%%' % (i.battery_percent, self.p.battery_low_percent)
        if self.p.rain_mode == 1 and i.rain:
            return 'rain detected'
        if i.docked and (i.fix_type is None or i.fix_type < self.p.preflight_min_fix_type_docked):
            return 'no GNSS fix (fix_type %s < %d)' % (
                i.fix_type, self.p.preflight_min_fix_type_docked)
        stale = '' if self._at_dock() else self._heading_cog_stale()
        if not self._at_dock() and (not self.heading_ok() or stale):
            # Undocked: Nav2 would steer with an arbitrary IMU heading (or a stale COG one,
            # 2026-10-14). At the dock the undock (straight drive out along the dock yaw)
            # seeds it instead.
            if self._hcal_prior_ok(('pending', 'yaw_ok')):
                # a usable saved heading: calibrate by a short straight drive once RTK is
                # fixed (WAITING_FOR_RTK, _hcal_maybe_start)
                if stale:
                    self._log('info', 'heading cog is %s: calibration drive first' % stale)
                self._hcal_wanted = True
                return ''
            if stale:
                self._log('warn', 'heading cog is %s and no calibration drive: not aligned'
                          % stale)
            return HEADING_NOT_ALIGNED
        return ''

    def _at_dock(self):
        return bool(self.inputs.docked or self.inputs.is_charging)

    def heading_ok(self):
        """Absolute heading available (heading_aligner source dock|cog, status fresh)."""
        if not self.p.require_heading_alignment:
            return True
        i = self.inputs
        if i.heading_stamp is None or self._now - i.heading_stamp > \
                self.p.heading_status_timeout_s:
            return False
        return bool(i.heading_aligned) and i.heading_source in HEADING_ALIGNED_SOURCES

    def _heading_cog_stale(self):
        """Why an aligned 'cog' heading is too old / too poor to plan a START off the dock on
        ('' = fine, or not a cog heading): last COG fix older than heading_max_cog_age_s, or
        status quality in heading_reject_quality (2026-10-14). Dock / rtk / file_verified
        headings are never stale here."""
        p, i = self.p, self.inputs
        if not p.require_heading_alignment or i.heading_source != 'cog' \
                or not i.heading_aligned:
            return ''
        why = []
        age, max_age = i.heading_cog_age, float(p.heading_max_cog_age_s)
        if max_age > 0.0 and age is not None and age > max_age:
            why.append('%.0f s old' % age)
        reject = [str(q) for q in (p.heading_reject_quality or []) if str(q)]
        if i.heading_quality and i.heading_quality in reject:
            why.append('quality %s' % i.heading_quality)
        return ' / '.join(why)

    def _heading_plan_ok(self):
        """heading_ok() and, for a cog heading, not stale (_heading_cog_stale)."""
        return self.heading_ok() and not self._heading_cog_stale()

    def _start(self, single=None, auto=False):
        if self.phase == 'RECORDING':
            self._log('warn', 'START refused: finish or cancel the recording first')
            return False
        if self.mission is not None:
            if single is None or single == self.mission.single_area_target:
                return True             # already mowing: idempotent
            self._log('warn', 'start_in_area(%d) refused: mowing session running' % single)
            return False
        self._preview_drop('mission started')
        if not auto:
            self._hb_count = 0              # task 0.4 (a): excursions are counted per mission
        if self.phase in DOCK_PHASES:
            self._cancel_all('start')
        self._leave_current_mode('start')
        self._go('PREFLIGHT_CHECK', '')
        reason = self._preflight()
        if reason:
            self._log('warn', 'START refused: preflight failed: %s' % reason)
            self._go_idle('preflight failed: %s' % reason)
            return False

        cur = self.cursor
        if single is not None:
            resume = cur.available and cur.single_area_target == single
            if not resume and cur.available:
                self._log('info', 'start_in_area(%d): discarding the previous resume cursor'
                          % single)
        else:
            resume = cur.available
            # an automatic resume (rain / charge) keeps a start_in_area target
            single = cur.single_area_target if (resume or auto) else None
        if not resume:
            self.cursor = ResumeCursor()
            self.cursor.single_area_target = single
        self.mission = Mission(single_area_target=single, resume=resume)
        self._pivot_tries = {}              # pivot re-send counters start fresh on resume
        # a resumed mow keeps its live GUI deck height (on_cutter_height)
        self.mission.height_override_mm = cur.live_height_mm if resume else None
        self._bump_transit_run()            # each drive varies its transit lines
        self._log('info', '%s%s: enumerating mowing areas' % (
            'resuming' if resume else 'starting',
            (' (area %d only)' % single) if single is not None else ''))
        self._go('PREFLIGHT_CHECK', 'loading areas')
        self._enum = {'purpose': 'start', 'index': 0, 'areas': {}, 'count': 0}
        self._call(SRV_GET_AREA, {'index': 0}, purpose='enum')
        return True

    def _resume(self, why):
        self._resume_after = None
        if not self._start(single=None, auto=True):
            self._log('warn', 'automatic resume after %s failed' % why)

    def _areas_loaded(self, areas, count):
        m = self.mission
        m.areas = areas
        m.all_area_count = count
        if m.single_area_target is not None:
            if m.single_area_target not in areas:
                self._abort_start('area %d does not exist or is a navigation area'
                                  % m.single_area_target)
                return
            queue = [m.single_area_target]
        else:
            queue = sorted(areas)
        cur = self.cursor
        queue = [a for a in queue if a not in cur.completed_areas]
        if cur.current_area in queue:   # resume the interrupted area first
            queue.remove(cur.current_area)
            queue.insert(0, cur.current_area)
        if not queue:
            if m.resume:
                self._log('info', 'resume cursor has no areas left: starting fresh')
                self.cursor = ResumeCursor()
                m.height_override_mm = None     # fresh mow: area heights again
                self.cursor.single_area_target = m.single_area_target
                queue = [m.single_area_target] if m.single_area_target is not None \
                    else sorted(areas)
            if not queue:
                self._abort_start('no mowing areas defined')
                return
        m.queue = queue
        self.cursor.current_command = CMD_START
        self._persist()
        self._log('info', 'areas to mow: %s' % queue)
        if self._at_dock():
            # charging but is_docking_done false still means "in the dock": always leave
            # it with the straight undock (it also yields the course-over-ground heading).
            self._undock()
        else:
            self._go('WAITING_FOR_RTK', '')

    def _abort_start(self, reason):
        """Failure before the robot moved: back to idle."""
        self._log('warn', 'START failed: %s' % reason)
        self.mission = None
        self._go_idle('start failed: %s' % reason)

    def _manual_undock(self):
        """UNDOCK command: drive straight off the dock, then idle off the dock."""
        if self.phase not in ('IDLE_DOCKED', 'CHARGING') or not self._at_dock() \
                or self.mission is not None:
            self._log('warn', 'UNDOCK refused: not docked (phase %s)' % self.phase)
            return False
        self._resume_after = None
        self._manual_undocking = True
        self._log('info', 'UNDOCK: driving %.2f m off the dock' % self.p.manual_undock_distance_m)
        self._undock(self.p.manual_undock_distance_m)
        return True

    def _undock(self, distance=None):
        distance = self.p.undock_distance_m if distance is None else float(distance)
        self._blade_off('undock')
        self._go('UNDOCKING', '')
        if self.p.use_docking_server:
            self._start_action(ACT_UNDOCK, {
                'distance_m': distance,
                'speed_mps': self.p.undock_speed_mps,
                'wait_for_rtk': False,
                'rtk_timeout_s': self.p.rtk_timeout_s}, self.p.undock_timeout_s)
        else:
            self._call(SRV_CHARGING, {'enable': False}, track=False)
            self._start_action(ACT_BACKUP, {'distance_m': distance,
                                            'speed_mps': self.p.undock_speed_mps},
                               self.p.undock_timeout_s)

    # ==================================================================
    # area / sub-path progression (FollowStrip port)
    # ==================================================================
    def _next_area(self):
        m = self.mission
        if self.blade_on:
            self._blade_off('area finished')
        while m.queue and m.queue[0] in self.cursor.completed_areas:
            m.queue.pop(0)
        if not m.queue:
            if m.failed or m.blocked:
                self._mission_incomplete()
            else:
                self._mission_complete()     # status still shows the last area's counts
            return
        m.area_idx = None
        m.subpaths, m.lengths, m.cum = [], [], []
        idx = m.queue.pop(0)
        area = m.areas[idx]
        m.area_idx = idx
        m.sub_i, m.start_local, m.skipped, m.step = 0, 0, 0, None
        m.requeue, m.stretch, m.stretch_end, m.detour = None, None, None, None
        m.requeue_subs, m.retry_entry = None, None
        self.cursor.current_area = idx
        m.settings, m.runs = None, []
        m.run_i = int(self.cursor.area_runs.get(idx, 0))
        if self.p.use_area_settings:
            self._go('PLANNING', 'area %d %s: loading settings' % (idx, area.get('name', '')))
            self._call(SRV_GET_AREA_SETTINGS, {'index': idx}, purpose='area_settings')
        else:
            self._on_area_settings(False, {})

    # ------------------------------------------------------------------
    # per-area mowing settings
    # ------------------------------------------------------------------
    @staticmethod
    def height_mm(mm):
        """cutter_height_mm -> MCU height position: absolute deck height in mm,
        clamped to the vendor range 30-90 (mower_bt_nodes CutterControl::onRunning)."""
        return int(round(max(float(CUTTER_HEIGHT_MIN_MM),
                             min(float(CUTTER_HEIGHT_MAX_MM), float(mm)))))

    def _merge_settings(self, ok, resp, area_idx=None):
        st = dict(AREA_SETTINGS_DEFAULTS)
        src = 'built-in defaults'
        got = {}
        if ok and resp.get('success'):
            try:
                got = json.loads(resp.get('settings_json') or '{}')
                if isinstance(got, dict):
                    st.update({k: v for k, v in got.items() if k in st and v is not None})
                    src = 'map_server'
            except ValueError:
                self._log('warn', 'area settings: bad JSON from get_area_settings, using defaults')
        elif self.p.use_area_settings:
            self._log('warn', 'area %d: get_area_settings unavailable: using built-in defaults'
                      % (self.mission.area_idx if area_idx is None else area_idx))
        # Slope-aware angle (terrain memory): the map server derives
        # slope_mow_angle_deg for areas with slope_mode != off; used only while
        # the area's own angle is auto, ahead of the global parameter.
        if float(st['mow_angle_deg']) < 0.0 and st.get('slope_mode', 'off') != 'off':
            sa = got.get('slope_mow_angle_deg') if src == 'map_server' else None
            if isinstance(sa, (int, float)) and not isinstance(sa, bool) and sa >= 0.0:
                st['mow_angle_deg'] = math.fmod(float(sa), 180.0)
                self._log('info', 'slope_mode %s: swath angle %.1f deg (%s)'
                          % (st['slope_mode'], st['mow_angle_deg'],
                             got.get('slope_angle_why', '')))
            else:
                self._log('info', 'slope_mode %s: planner angle kept (%s)'
                          % (st['slope_mode'], got.get('slope_angle_why', 'no slope angle')
                             if src == 'map_server' else 'map_server unavailable'))
        # Global mow_angle_deg parameter: fallback for areas left on auto.
        if float(st['mow_angle_deg']) < 0.0 and self.p.mow_angle_deg >= 0.0:
            st['mow_angle_deg'] = float(self.p.mow_angle_deg)
        if st['route_order'] not in ROUTE_ORDERS:
            self._log('warn', 'unknown route_order %r: boustrophedon' % st['route_order'])
            st['route_order'] = 'boustrophedon'
        if st['turn_type'] not in TURN_TYPES:
            self._log('warn', 'unknown turn_type %r: auto' % st['turn_type'])
            st['turn_type'] = 'auto'
        st['route_spiral_size'] = max(2, int(st['route_spiral_size']))
        st['min_turn_radius_m'] = max(0.0, float(st['min_turn_radius_m']))
        if st['path_mode'] not in PATH_MODES:
            self._log('warn', 'unknown path_mode %r: zigzag' % st['path_mode'])
            st['path_mode'] = 'zigzag'
        st['repeat'] = max(1, int(st['repeat']))
        if st['obstacle_detection'] not in OBSTACLE_DETECTION_LEVELS:
            self._log('warn', 'unknown obstacle_detection %r: standard' % st['obstacle_detection'])
            st['obstacle_detection'] = 'standard'
        if st.get('blade_policy') not in BLADE_POLICIES:
            self._log('warn', 'unknown blade_policy %r: continuous' % st.get('blade_policy'))
            st['blade_policy'] = 'continuous'
        st['perimeter_laps'] = max(0, int(st['perimeter_laps']))
        return st, src

    def _swath_spacing(self, st):
        """Path spacing (m) = the area's swath_width_m, used as-is (cut_width_m plays no part)."""
        try:
            w = float(st.get('swath_width_m') or 0.0)
        except (TypeError, ValueError):
            w = 0.0
        return w if w > 0.0 else AREA_SETTINGS_DEFAULTS['swath_width_m']

    def _area_name(self):
        m = self.mission
        return str(m.areas.get(m.area_idx, {}).get('name', ''))

    def _build_runs(self, st, area_name=None):
        """One entry per PlanCoverage goal of this area, in mowing order.

        cross: two plans per repeat, the second at +90 deg. alternate: the
        angle turns by alternate_angle_offset_deg on every run, continuing
        across sessions (alternate_counts). On an AUTO angle a +90 deg turn is
        the goal's ``perpendicular`` flag (the bridge learns the auto angle);
        any other turn is taken from 0 deg (the map x axis).
        """
        base = float(st['mow_angle_deg'])
        mode = st['path_mode']
        n = int(st['repeat'])

        def run(rot):
            rot = math.fmod(rot, 180.0)
            if base >= 0.0:
                return {'angle': math.fmod(base + rot, 180.0), 'perpendicular': False}
            if abs(rot) < 1e-6:
                return {'angle': -1.0, 'perpendicular': False}
            if abs(rot - 90.0) < 1e-6:
                return {'angle': -1.0, 'perpendicular': True}
            return {'angle': rot, 'perpendicular': False}

        if mode == 'cross':
            return [r for _ in range(n) for r in (run(0.0), run(90.0))]
        if mode == 'alternate':
            off = float(st['alternate_angle_offset_deg'])
            done = int(self.alternate_counts.get(
                self._area_name() if area_name is None else area_name, 0))
            return [run((done + k) * off) for k in range(n)]
        return [run(0.0) for _ in range(n)]

    def display_sub_state(self):
        """sub_state as reported: a fresh stale-stereo costmap warning wins while driving
        (Nav2 aborts every FollowPath then; the generic retry text hides the cause)."""
        i = self.inputs
        if (self.phase in ('TRANSIT', 'MOWING') and i.stereo_stale_stamp is not None
                and self._now - i.stereo_stale_stamp <= self.p.stereo_stale_window_s
                and not self.sub_state.startswith('waiting for ')):
            return STEREO_STALE_SUB % int(round(i.stereo_stale_s or 0.0))
        if (self._tilt_capped and self._tob is None and self._tilt_band() == 'caution'
                and self.phase in MOTION_PHASES):
            return 'slope caution (%s): slowed; %s' % (self._tilt_text(), self.sub_state)
        if self.pivoting_slowly() and not self.sub_state.startswith('waiting for '):
            return PIVOT_SUB + (': ' + self.sub_state if self.sub_state else '')
        return self.sub_state

    def obstacle_level(self):
        m = self.mission
        lvl = (m.settings or {}).get('obstacle_detection') if m is not None else None
        return lvl if lvl in OBSTACLE_DETECTION_LEVELS else 'standard'

    def _push_obstacle_detection(self, level):
        if not self.p.set_obstacle_detection:
            return
        self._call(SRV_SET_PARAMS, {'node': PARAM_NODE_OBSTACLE_GUARD,
                                    'params': {'obstacle_detection': str(level)}}, track=False)
        self._call(SRV_SET_PARAMS, {'node': PARAM_NODE_DET_RANGE,
                                    'params': {'publish_obstacle_points': level != 'none'}},
                   track=False)

    def _on_area_settings(self, ok, resp):
        m = self.mission
        st, src = self._merge_settings(ok, resp)
        if m.height_override_mm is not None:
            # live height change during this mow wins over the saved area height
            st = dict(st, cutter_height_mm=m.height_override_mm)
            src = src + ', live height'
        m.settings = st
        self._push_obstacle_detection(st['obstacle_detection'])
        m.runs = self._build_runs(st)
        if m.run_i >= len(m.runs):
            m.run_i = 0
        hmm = self.height_mm(st['cutter_height_mm'])
        speed = min(float(st['cut_speed_mps']), float(self.p.cut_speed_max_mps))
        if self._tilt_capped:               # area starts on a slope: keep the caution cap
            speed = min(speed, float(self.p.tilt_caution_speed_mps))
        self._log('info', 'area %d %s: settings (%s) %s; blade height %d mm, '
                          'cut speed %.2f m/s, %d run(s)'
                  % (m.area_idx, self._area_name(), src, json.dumps(st, sort_keys=True),
                     hmm, speed, len(m.runs)))
        if self.p.set_cutter_height:
            # Blade is off here (PLANNING); height only, before any mowing.
            self._call(SRV_CUTTER_HEIGHT, {'height_mm': hmm}, track=False)
        if self.p.set_cut_speed:
            self._call(SRV_SET_PARAMS, {'node': PARAM_NODE_CONTROLLER,
                                        'params': self.coverage_speed_params(speed)},
                       track=False)
        self._plan_run()

    def active_settings(self):
        """JSON-able active settings (latched ~/active_area_settings)."""
        m = self.mission
        if m is None or m.settings is None or not m.runs:
            return {}
        run = m.runs[m.run_i]
        out = dict(m.settings)
        out.update({
            'area_index': m.area_idx, 'area_name': self._area_name(),
            'run': m.run_i + 1, 'runs': len(m.runs),
            'run_mow_angle_deg': run['angle'], 'run_perpendicular': run['perpendicular'],
            'operation_width_m': round(self._swath_spacing(m.settings), 4)})
        return out

    def _plan_run(self):
        m = self.mission
        st, run = m.settings, m.runs[m.run_i]
        self._fx.append(PublishAreaSettings(self.active_settings()))
        self._go('PLANNING', 'area %d %s [%s, run %d/%d]' % (
            m.area_idx, self._area_name(), st['path_mode'], m.run_i + 1, len(m.runs)))
        self._call(SRV_SET_PARAMS, {'node': PARAM_NODE_COVERAGE, 'params': {
            'operation_width': self._swath_spacing(st),
            'headland_rings': int(st['perimeter_laps']),
            'path_mode': str(st['path_mode']),
            'mow_angle_deg': float(run['angle']),
            'edge_first': bool(st['edge_first']),
            **coverage_route_params(st)}}, purpose='coverage_params')

    def _start_plan(self):
        m = self.mission
        area, run = m.areas[m.area_idx], m.runs[m.run_i]
        self._start_action(ACT_PLAN, {
            'outer_boundary': list(area.get('outer', [])),
            'obstacles': [list(o) for o in area.get('obstacles', [])],
            'mow_angle_deg': float(run['angle']),
            'perpendicular': bool(run['perpendicular'])}, self.p.plan_timeout_s)

    def _on_plan(self, outcome, result):
        m = self.mission
        if outcome == UNAVAILABLE:
            self._mission_failed('/plan_coverage action server unavailable')
            return
        subpaths = self._plan_subpaths(outcome, result)
        if not subpaths:
            why = result.get('message') or outcome
            self._log('warn', 'area %d: coverage planning failed (%s): skipping it'
                      % (m.area_idx, why))
            m.failed.append((m.area_idx, None, 'area %d planning failed: %s' % (m.area_idx, why),
                             -1))
            self._go('AREA_UNREACHABLE', 'area %d: planning failed: %s' % (m.area_idx, why))
            self._next_area()
            return
        m.cond, m.cond_errors = {}, {}
        if self.p.path_conditioning:
            subpaths = self._condition_subpaths(subpaths, result.get(CONDITIONED_KEY))
        m.subpaths = subpaths
        m.planned_total += len(subpaths)
        m.cum = [geo.cumulative_lengths(sp) for sp in subpaths]
        m.lengths = [c[-1] for c in m.cum]
        full = result.get('full_path') or [p for sp in subpaths for p in sp]
        self._fx.append(PublishPlan(list(full)))
        self._plan_shown = True
        self._preview_shown = False
        fp = geo.plan_fingerprint(subpaths)
        _offs, total = subpath_offsets(subpaths)
        ac = self.cursor.areas.get(m.area_idx)
        m.resume_sub = None
        if ac is not None and ac.fingerprint == fp and ac.pose_count == total:
            if ac.resume_index >= 0:
                m.resume_sub = absolute_to_local(subpaths, ac.resume_index)
            self._log('info', 'area %d: resuming (%d/%d sub-paths done, cursor %s)' % (
                m.area_idx, len(ac.completed), len(subpaths), m.resume_sub))
        else:
            if ac is not None and (ac.completed or ac.resume_index >= 0):
                self._log('warn', 'area %d: plan geometry changed, stale resume cursor '
                                  'discarded' % m.area_idx)
            self.cursor.areas[m.area_idx] = AreaCursor(total, fp, -1, set())
        self._persist()
        self._log('info', 'area %d: %d drivable sub-paths, %.1f m' % (
            m.area_idx, len(subpaths), sum(m.lengths)))
        m.sub_i = 0
        self._next_subpath()

    @staticmethod
    def _plan_subpaths(outcome, result):
        subpaths = []
        if outcome == SUCCEEDED and result.get('success'):
            subpaths = [list(sp) for sp in result.get('drivable_subpaths', []) if len(sp) >= 2]
            if not subpaths:
                subpaths = [list(sp) for sp in result.get('segments', []) if len(sp) >= 2]
            if not subpaths and len(result.get('full_path', [])) >= 2:
                subpaths = [list(result['full_path'])]
        return subpaths

    def _next_subpath(self):
        m = self.mission
        ac = self.cursor.area(m.area_idx)
        while m.sub_i < len(m.subpaths) and m.sub_i in ac.completed:
            m.sub_i += 1
        if m.sub_i >= len(m.subpaths):
            self._area_done()
            return
        m.start_local = 0
        m.boundary_recoveries = 0
        m.premature = 0
        if m.resume_sub is not None and m.resume_sub[0] == m.sub_i:
            m.start_local = m.resume_sub[1]
            m.resume_sub = None
        m.retry_used = False
        m.transit_fails = m.follow_fails = m.recovery_fails = 0
        m.route_plan_fails = 0
        m.recovering = False
        m.detours, m.detour = 0, None
        m.cusp_turns = {}
        m.leg_watch = None
        m.corner_end, m.pivot_done = None, None
        self._blk = None
        self._dispatch()

    def _dispatch(self, force_transit=False):
        """Send the current sub-path from ``start_local``: blade-off transit
        first when its start is farther than ``transit_gap_m``."""
        m = self.mission
        sp = m.subpaths[m.sub_i]
        if m.sub_i in m.cond_errors:
            self._subpath_failed('path conditioning: %s' % m.cond_errors[m.sub_i])
            return
        if self._seg_last() - m.start_local < 1:
            self._segment_done()
            return
        if self._turn_leg_driven():
            return
        target = sp[m.start_local]
        if force_transit or geo.needs_transit(self.inputs.pose, target, self.p.transit_gap_m):
            # SAFETY (FollowStrip): blade off before any dispatch across the gap, unless
            # the continuous policy keeps it on for a transit inside the area.
            self._transit_blade(target, 'transit', force_off=force_transit)
            m.step = 'transit'
            self._go('TRANSIT', 'area %d sub-path %d/%d' % (
                m.area_idx, m.sub_i + 1, len(m.subpaths)))
            yaw = geo.heading_of(sp, m.start_local)
            m.transit_target = (target[0], target[1], yaw)
            m.route_fallback, m.route_aborts = False, 0
            self._send_transit()
        else:
            self._begin_blade()

    def _is_turn_leg(self):
        """The current sub-path is a turn leg of the turn_leg_skip rule: a (short) reverse
        leg of the coverage bridge, or the forward piece that ends at the cusp of one."""
        m = self.mission
        sp = m.subpaths[m.sub_i]
        leg_max = float(self.p.turn_leg_max_m)

        def reverse_leg(poses):     # a (short) reverse turn leg of the coverage bridge
            return geo.is_reverse_subpath(poses) and geo.path_length(poses) <= leg_max

        nxt = m.subpaths[m.sub_i + 1] if m.sub_i + 1 < len(m.subpaths) else None
        cusp_next = nxt is not None and reverse_leg(nxt) and geo.dist(nxt[0], sp[-1]) <= 0.01
        # Task 0.4 (f): this rule stays for turn_leg_skip (distance + yaw, safe on a long
        # swath ending at a cusp: the 2026-10-13 retry case); the _leg_watch cap no longer
        # uses it (a 49 m ring ending at a cusp was capped in run 3).
        return reverse_leg(sp) or cusp_next

    def _turn_leg_driven(self):
        """Skip the rest of a turn leg the robot has already all but driven (see
        ``turn_leg_skip_m``): done -> True (the next sub-path has been dispatched)."""
        m, pose = self.mission, self.inputs.pose
        skip_m = float(self.p.turn_leg_skip_m)
        if skip_m <= 0.0 or pose is None or len(pose) < 3 or m.stretch is not None:
            return False
        sp = m.subpaths[m.sub_i]
        end = sp[-1]
        if len(end) < 3:
            return False
        if not self._is_turn_leg():
            return False
        d = geo.dist(pose, end)
        dyaw = abs(math.atan2(math.sin(pose[2] - end[2]), math.cos(pose[2] - end[2])))
        if d > skip_m or dyaw > float(self.p.turn_leg_skip_yaw):
            return False
        self._log('info', 'area %d sub-path %d: turn leg end %.2f m / %.0f deg away: leg '
                          'done, next leg from here (no re-send, no turn in place)'
                  % (m.area_idx, m.sub_i, d, math.degrees(dyaw)))
        m.step = None
        self._subpath_done()
        return True

    def _leg_watch(self):
        """Fix 10 (2026-10-14 run 2), called on every MOWING tick while a follow_path runs.

        * Turn-leg end cap: once the leg's end is reached in xy (within turn_leg_skip_m of
          the end pose; task 0.4 f: never the tracked pose index, which can jump to the last
          pose early, and only for legs no longer than turn_leg_max_m) the leg is done after
          turn_leg_done_after_s OR turn_leg_done_after_rot_rad of accumulated |yaw change|
          of the map pose, whatever the yaw. A leg whose end yaw is met is left to the
          controller's leg goal checker (and the turn_leg_skip rule on a re-send), as before.
        * Loop guard: a follow_path whose tracked pose index has not advanced for
          leg_loop_abort_s is cancelled and handled as a follow_path abort.

        Returns True when it ended / aborted the follow (the tick is done)."""
        m, a, pose, p = self.mission, self._action, self.inputs.pose, self.p
        if a is None or a.name != ACT_FOLLOW or m.step != 'follow' or not m.subpaths:
            return False
        sp = m.subpaths[m.sub_i]
        key = (m.area_idx, m.sub_i, m.stretch)
        w = m.leg_watch
        by_dist = bool(p.turn_leg_end_by_distance)
        if w is None or w['key'] != key:
            short = geo.path_length(sp) <= float(p.turn_leg_max_m)
            w = m.leg_watch = {
                'key': key, 'reached': None, 'rot': 0.0, 'yaw': None, 'token': None,
                'idx': None, 'since': self._now,
                # task 0.4 (f): capped only by the leg's own length
                'capped': short if by_dist else (self._is_turn_leg() or short)}
        if w['token'] != a.token:            # a new goal (retry / resume): fresh loop timer
            w['token'], w['idx'], w['since'], w['yaw'] = a.token, m.progress_local, self._now, None
        elif m.progress_local != w['idx']:
            w['idx'], w['since'] = m.progress_local, self._now
        # ---- turn-leg end cap ----
        cap_s, cap_rot = float(p.turn_leg_done_after_s), float(p.turn_leg_done_after_rot_rad)
        if w['capped'] and m.stretch is None and pose is not None and len(pose) >= 3:
            end = sp[-1]
            skip_m = float(p.turn_leg_skip_m)
            d = geo.dist(pose, end)
            if w['reached'] is not None and w['yaw'] is not None:
                w['rot'] += abs(geo.wrap_angle(float(pose[2]) - w['yaw']))
            w['yaw'] = float(pose[2])
            # task 0.4 (f): distance only; progress_local can jump to the last pose early
            at_end = (skip_m > 0.0 and d <= skip_m) or \
                (not by_dist and m.progress_local >= len(sp) - 1)
            if w['reached'] is None and at_end:
                w['reached'], w['rot'] = self._now, 0.0
            if w['reached'] is not None:
                dyaw = abs(geo.wrap_angle(float(pose[2]) - float(end[2]))) if len(end) >= 3 \
                    else math.inf
                why = None
                if cap_s > 0.0 and self._now - w['reached'] >= cap_s:
                    why = 'time cap: %.0f s since the end was reached' % (self._now - w['reached'])
                elif cap_rot > 0.0 and w['rot'] >= cap_rot:
                    why = 'rotation cap: %.0f deg turned since the end was reached' \
                        % math.degrees(w['rot'])
                if why is not None:
                    self._log('warn', 'area %d sub-path %d: turn leg end %.2f m / %.0f deg away (%s): '
                              'leg done, next leg from here (no re-send, no turn in place)'
                              % (m.area_idx, m.sub_i, d, math.degrees(min(dyaw, math.pi)), why))
                    self._fx.append(CancelActions('turn leg done'))
                    self._action = None
                    m.step = None
                    m.leg_watch = None
                    self._subpath_done()
                    return True
        # ---- loop guard ----
        loop_s = float(p.leg_loop_abort_s)
        if loop_s > 0.0 and self._now - w['since'] >= loop_s:
            msg = 'pose %d not advanced for %.0f s (leg loop guard)' % (
                m.progress_local, self._now - w['since'])
            self._log('warn', 'area %d sub-path %d: %s: cancelling follow_path, handled as an '
                              'abort' % (m.area_idx, m.sub_i, msg))
            self._fx.append(CancelActions('leg loop guard'))
            self._action = None
            w['token'] = None
            self._handle_action(a, ABORTED, {'message': msg})
            return True
        return False

    def _begin_blade(self):
        m = self.mission
        if self.inputs.boundary_violation:
            self._boundary_pause()       # never spin the blade up outside the boundary
            return
        self._go('MOWING', 'area %d sub-path %d/%d' % (m.area_idx, m.sub_i + 1, len(m.subpaths)))
        if self.blade_on and (self.inputs.is_cutting or not self.p.require_blade_confirmation):
            self._follow()
            return
        self._spin_attempt = 1
        self._spin_up()

    def _spin_up(self):
        m = self.mission
        m.step = 'spinup'
        self._blade_on()
        wait = self.p.blade_confirm_timeout_s if self.p.require_blade_confirmation \
            else self.p.blade_spinup_s
        self._spin_deadline = self._now + wait

    def _track_progress(self):
        """Advance ``progress_local`` to the pose nearest the robot, searching
        only ``progress_window_m`` ahead (never backwards, never onto a later lap)."""
        m, pose = self.mission, self.inputs.pose
        if pose is None or not m.subpaths:
            return m.progress_local
        m.progress_local = geo.nearest_index(m.subpaths[m.sub_i], pose, m.progress_local,
                                             self.p.progress_window_m)
        return m.progress_local

    def _tick_blade(self):
        m = self.mission
        if m.step == 'spinup':
            if self.p.require_blade_confirmation:
                if self.inputs.is_cutting:
                    self._follow()
                elif self._now >= self._spin_deadline:
                    self._blade_off('blade start attempt %d not confirmed' % self._spin_attempt)
                    if self._spin_attempt >= self.p.blade_start_attempts:
                        self._mission_failed('blade did not start (is_cutting false after %d '
                                             'attempts)' % self._spin_attempt)
                        return
                    self._log('warn', 'blade not cutting after %.0fs (attempt %d/%d): retrying'
                              % (self.p.blade_confirm_timeout_s, self._spin_attempt,
                                 self.p.blade_start_attempts))
                    m.step = 'spin_pause'
                    self._spin_deadline = self._now + self.p.blade_retry_pause_s
            elif self._now >= self._spin_deadline:
                self._follow()
        elif m.step == 'spin_pause' and self._now >= self._spin_deadline:
            self._spin_attempt += 1
            self._spin_up()

    def _follow(self):
        m = self.mission
        if self._cov_pivot():
            return
        m.step = 'follow'
        m.progress_local = m.start_local
        sp = m.subpaths[m.sub_i]
        m.chunk_end = min(geo.chunk_end(sp, m.start_local, self.p.follow_chunk_m,
                                        self.p.follow_chunk_clearance_m), self._seg_last())
        m.corner_end = None
        for c in self._cov_pivots():
            if m.start_local < c <= m.chunk_end and c < self._seg_last():
                m.chunk_end = m.corner_end = c       # this chunk ends on the pivot vertex
                break
        poses = list(sp[m.start_local:m.chunk_end + 1])
        reverse = geo.is_reverse_subpath(sp)
        checker = self._follow_goal_checker(poses, reverse)
        if (m.corner_end is not None or m.sub_i in m.cond) and not reverse and \
                self.p.route_corner_goal_checker_id:
            # a conditioned forward leg (into a pivot vertex, to a chunk end or to the
            # sub-path end) ends when the robot has come up to its end along the path
            # (coverage_corner_goal_checker, along_path), not on a position disc
            checker = self.p.route_corner_goal_checker_id
        self._start_action(ACT_FOLLOW, {
            'poses': poses,
            'controller_id': self.coverage_controller_id(reverse),
            'goal_checker_id': checker,
            'progress_checker_id': self.p.follow_progress_checker_id},
            self.p.follow_timeout_s)

    # ---- path conditioning (design 2026-10-15 steps 2-3) --------------------
    def _cond_params(self):
        p = self.p
        return pcond.Params(
            r_min=float(p.path_conditioning_r_min_m), r_max=float(p.path_conditioning_r_max_m),
            margin_m=float(p.path_conditioning_margin_m),
            half_width_m=float(p.path_conditioning_half_width_m),
            slack_m=float(p.path_conditioning_slack_m),
            pivot_min_rad=float(p.path_conditioning_pivot_min_rad),
            edge_pivot_rad=float(p.path_conditioning_edge_pivot_rad),
            edge_clearance_m=float(p.path_conditioning_edge_clearance_m))

    def _region(self):
        """Allowed region polygons: mowing areas U drawn path channels U the dock corridor
        (the dock -> approach capsule as a rectangle)."""
        polys = [list(a['outer']) for a in (self._region_areas or {}).values()
                 if len(a.get('outer') or []) >= 3]
        dock = self.inputs.dock_pose
        if dock is not None and len(dock) >= 3:
            x, y, yaw = float(dock[0]), float(dock[1]), float(dock[2])
            L, w = float(self.p.dock_approach_distance_m), 0.35
            c, sn = math.cos(yaw), math.sin(yaw)
            polys.append([(x - w * c + w * sn, y - w * sn - w * c),
                          (x + (L + w) * c + w * sn, y + (L + w) * sn - w * c),
                          (x + (L + w) * c - w * sn, y + (L + w) * sn + w * c),
                          (x - w * c - w * sn, y - w * sn + w * c)])
        return polys

    def _conditioning_job(self, subpaths):
        return {'subpaths': [list(sp) for sp in subpaths], 'region': self._region(),
                'prm': self._cond_params(), 'leg_max_m': float(self.p.turn_leg_max_m),
                'area': self.mission.area_idx}

    def _condition_subpaths(self, subpaths, pre=None):
        """Apply a coverage plan's path conditioning: ``pre`` (condition_plan output the
        node computed off the lock for these sub-paths) or computed inline."""
        m = self.mission
        if pre is None or pre.get('n_in') != len(subpaths):
            pre = condition_plan(self._conditioning_job(subpaths))
        m.cond, m.cond_errors = dict(pre['cond']), dict(pre['cond_errors'])
        for level, text in pre['logs']:
            self._log(level, text)
        return [list(sp) for sp in pre['subpaths']]

    def _condition_route(self, poses, what):
        """A transit / dock route -> FollowPath legs split at the conditioner's pivots."""
        prm = self._cond_params()
        # routes are tracked at transit speed by the route controller: a fillet tighter than
        # path_conditioning_route_r_min_m is not drivable there (sim gate 2026-10-16: MPPI ran
        # 0.4 m wide of an r 0.5 m dock-path fillet and left the area) -> pivot at the vertex
        prm.r_min = max(prm.r_min, float(self.p.path_conditioning_route_r_min_m))
        prm.r_max = max(prm.r_max, prm.r_min)
        r = pcond.condition(poses, self._region(), prm, ring=False)
        if not r.valid:
            self._log('error', '%s: path conditioning failed (%s): raw route split as before'
                      % (what, r.errors[0]))
            return None
        legs, s0 = [], 0
        for c in r.pivots:
            if 0 < c < len(r.poses) - 1:
                legs.append(list(r.poses[s0:c + 1]))
                s0 = c
        legs.append(list(r.poses[s0:]))
        legs = [lg for lg in legs if len(lg) >= 2] or [list(r.poses)]
        self._fx.append(PublishRoute(what, list(r.poses), list(r.types)))
        self._log('info', '%s: conditioned: %s' % (what, ', '.join(
            '%s %+.0f deg%s' % (c.choice, math.degrees(c.turn), ' r %.1f' % c.r if c.r else '')
            for c in r.corners) or 'no corners'))
        return legs

    def _cov_turn_leg(self, k):
        """Sub-path ``k`` is a reverse leg (owner turn_type reverse, or a turn chain that
        could not be conditioned): driven as planned, never pivoted onto."""
        return geo.is_reverse_subpath(self.mission.subpaths[k])

    def _cov_pivots(self, k=None):
        m = self.mission
        k = m.sub_i if k is None else k
        c = m.cond.get(k)
        return list(c['pivots']) if c else []

    def _cov_pivot(self):
        """Before a coverage follow from ``start_local``: the conditioner's pivot rule at the
        leg's start. The start is a vertex of the path actually driven (robot -> leg): a
        conditioner pivot marker, the sub-path start after a transit, or a resume after a
        retry / retrace / hold. The robot is on the line, so no fillet fits there: when the
        join heading is path_conditioning_pivot_min_rad or more off the robot's yaw it turns
        in place (Nav2 Spin, both wheels) first, as at any pivot vertex; a smaller bend is
        left to the tracker. One mechanism for both coverage controllers (it replaces the
        separate leg-start rules path_conditioning_start_pivot_rad and MPPI's
        coverage_leg_pivot_rad). Not on reverse legs. True = started."""
        m, p, pose = self.mission, self.p, self.inputs.pose
        key = (m.area_idx, m.sub_i, m.start_local)
        if m.pivot_done == key:               # just pivoted for this leg: follow it
            m.pivot_done = None
            return False
        m.pivot_done = None
        lim = float(p.path_conditioning_pivot_min_rad)
        if not p.path_conditioning or lim <= 0.0 or pose is None or len(pose) < 3 or \
                self._cov_turn_leg(m.sub_i):
            return False
        sp, pivots = m.subpaths[m.sub_i], self._cov_pivots()
        corner = m.start_local in pivots
        end = min([c for c in pivots if c > m.start_local] + [self._seg_last()])
        heading = self._join_heading(sp[m.start_local:end + 1], pose)
        if heading is None:
            return False
        delta = geo.wrap_angle(heading - float(pose[2]))
        if abs(delta) < lim:
            return False
        q = sp[m.start_local]
        self._log('info', 'area %d sub-path %d: %s at (%.2f, %.2f): pivot %.0f deg in place, '
                          'blade on, then follow' % (m.area_idx, m.sub_i,
                                                     'pivot vertex' if corner else 'leg start',
                                                     q[0], q[1], math.degrees(delta)))
        m.step = 'corner_pivot'
        m.pivot_done = key
        m.pivot_target = (heading, lim)
        self._go('MOWING', 'area %d sub-path %d/%d: pivot %.0f deg' % (
            m.area_idx, m.sub_i + 1, len(m.subpaths), math.degrees(delta)))
        allowance = abs(delta) / max(0.05, float(p.route_cusp_spin_rps)) + 10.0
        self._start_action(ACT_SPIN, {'angle': delta, 'time_allowance_s': allowance},
                           allowance + 5.0, purpose='cov_pivot')
        return True

    @staticmethod
    def _join_heading(leg, pose, look=0.4):
        """Heading from the robot to the point ``look`` m past its projection onto ``leg``
        (where the tracker joins it; on the leg's start: the leg's own heading)."""
        if len(leg) < 2:
            return None
        cum = geo.cumulative_lengths(leg)
        s = geo.path_projection(leg, pose)[0]
        tx, ty = geo._point_at(leg, cum, min(cum[-1], s + look))
        if math.hypot(tx - pose[0], ty - pose[1]) < 0.1:
            return MissionFSM._leg_heading(leg)
        return math.atan2(ty - pose[1], tx - pose[0])

    def _pivot_unfinished(self, outcome, purpose, key, heading, lim, what):
        """A pivot Spin ended without SUCCEEDED. False: the robot is within ``lim`` of
        ``heading`` (the caller follows the leg, as after a successful pivot). True: handled -
        the Spin is re-sent for the remaining angle (pivot_retries per pivot ``key``), or,
        when those are used up, the robot stops (STUCK_NEEDS_HELP). Following a leg from a
        large heading error is what made MPPI leave the area in field run 6."""
        retries = int(self.p.pivot_retries)
        if retries < 0 or heading is None:
            self._log('warn', '%s pivot %s: following the leg anyway' % (what, outcome))
            return False
        pose = self.inputs.pose
        if pose is None or len(pose) < 3:
            self._log('error', '%s pivot %s and no pose: stopping' % (what, outcome))
            self._stuck_needs_help('%s pivot %s (no pose)' % (what, outcome), (0.0, 0.0))
            return True
        delta = geo.wrap_angle(heading - float(pose[2]))
        if abs(delta) < lim:
            self._log('warn', '%s pivot %s %.0f deg off the leg (< %.0f): following it'
                      % (what, outcome, math.degrees(delta), math.degrees(lim)))
            return False
        n = self._pivot_tries.get(key, 0)
        if n >= retries:
            self._stuck_needs_help('%s pivot %s %d time%s, still %.0f deg off the leg (>= %.0f)'
                                   % (what, outcome, n + 1, '' if n == 0 else 's',
                                      math.degrees(delta), math.degrees(lim)), pose)
            return True
        self._pivot_tries[key] = n + 1
        self._log('warn', '%s pivot %s with %.0f deg left (>= %.0f): pivot again %d/%d'
                  % (what, outcome, math.degrees(delta), math.degrees(lim), n + 1, retries))
        allowance = abs(delta) / max(0.01, float(self.p.route_cusp_spin_rps)) + 10.0
        self._start_action(ACT_SPIN, {'angle': delta, 'time_allowance_s': allowance},
                           allowance + 5.0, purpose=purpose)
        return True

    def _cov_pivot_done(self, outcome):
        """Coverage pivot ended: follow the leg (an unfinished pivot first goes through
        _pivot_unfinished: pivot again or stop)."""
        m = self.mission
        if self.phase != 'MOWING' or m.step != 'corner_pivot':
            return
        if outcome == SUCCEEDED:
            self._pivot_tries = {}
        else:
            heading, lim = m.pivot_target if m.pivot_target else (None, 0.0)
            if self._cov_pivot_skip(outcome, heading, lim):
                return
            if self._pivot_unfinished(outcome, 'cov_pivot', ('cov',) + tuple(m.pivot_done or ()),
                                      heading, lim, 'coverage'):
                return
        self._go('MOWING', 'area %d sub-path %d/%d' % (m.area_idx, m.sub_i + 1, len(m.subpaths)))
        self._follow()

    def _cov_pivot_skip(self, outcome, heading, lim):
        """blocked_pivot_policy 'skip' (stock firmware): an unfinished coverage pivot is not
        re-sent; the leg it turned onto (start_local -> next pivot vertex / sub-path end) is
        skipped and recorded, the next leg / sub-path is driven. True = handled. False (the
        robot is within ``lim`` of the leg, the per-mission cap is used up, policy 'retry',
        no pose): the caller's _pivot_unfinished path (follow / re-send / STUCK)."""
        m, p, pose = self.mission, self.p, self.inputs.pose
        if str(p.blocked_pivot_policy).strip().lower() != 'skip' or heading is None or \
                pose is None or len(pose) < 3 or outcome == CANCELED:
            return False
        delta = geo.wrap_angle(heading - float(pose[2]))
        if abs(delta) < lim:
            return False
        cap = int(p.blocked_pivot_max_skips)
        if m.pivot_skips >= cap:
            self._log('warn', 'blocked pivot: %d skips this mission (cap %d): pivot again / '
                      'stop' % (m.pivot_skips, cap))
            return False
        m.pivot_skips += 1
        self._pivot_tries = {}
        last = self._seg_last()
        frm = min(m.start_local, last)
        end = min([c for c in self._cov_pivots() if c > frm] + [last])
        why = 'coverage pivot %s, %.0f deg off the leg' % (outcome, math.degrees(delta))
        self._log('warn', 'blocked pivot: skip area %d sub-path %d poses %d -> %d at (%.2f, %.2f) '
                          '(%s), skip %d/%d' % (m.area_idx, m.sub_i, frm, end, pose[0], pose[1],
                                                why, m.pivot_skips, cap))
        self._incident('blocked_pivot', why)
        m.pivot_done, m.pivot_target, m.corner_end = None, None, None
        self._soft_blade_off('blocked pivot')
        if m.stretch is not None:            # the end-of-area re-try of a skipped piece
            self._stretch_blocked('blocked pivot (%s)' % why)
            return True
        if end > frm:
            offs, _ = subpath_offsets(m.subpaths)
            if p.blocked_pivot_requeue and p.requeue_skipped:
                self._record_skip(frm, end)
            else:
                m.blocked.append((m.area_idx, offs[m.sub_i] + frm, offs[m.sub_i] + end))
        if end >= last:
            m.step = None
            self._segment_done()
            return True
        m.start_local = end
        m.transit_fails = m.follow_fails = 0
        m.route_plan_fails = 0
        self._dispatch(force_transit=True)
        return True

    def coverage_mppi(self):
        """True when coverage sub-paths are driven by MPPI (``coverage_controller``)."""
        return str(self.p.coverage_controller).strip().lower() == 'mppi'

    def coverage_controller_id(self, reverse):
        """Nav2 controller id for a coverage sub-path (forward or reverse leg)."""
        if self.coverage_mppi():
            return (self.p.mppi_reverse_controller_id if reverse
                    else self.p.mppi_follow_controller_id)
        return self.p.reverse_controller_id if reverse else self.p.follow_controller_id

    def coverage_speed_params(self, speed):
        """{controller_server param: value} for a coverage cut speed."""
        if self.coverage_mppi():
            return {self.p.mppi_controller_speed_param:
                    min(float(speed), float(self.p.mppi_speed_max_mps))}
        return {self.p.controller_speed_param: float(speed)}

    def _follow_goal_checker(self, poses, reverse):
        """Goal checker for a FollowPath chunk: the leg checker for turn legs."""
        if self.p.turn_leg_goal_checker_id and (
                reverse or geo.path_length(poses) <= self.p.turn_leg_max_m):
            return self.p.turn_leg_goal_checker_id
        return self.p.follow_goal_checker_id

    def _subpath_done(self):
        m = self.mission
        ac = self.cursor.area(m.area_idx)
        ac.completed.add(m.sub_i)
        m.completed_count += 1
        offs, _total = subpath_offsets(m.subpaths)
        nxt = m.sub_i + 1
        ac.resume_index = offs[nxt] if nxt < len(offs) else -1
        self._persist()
        m.sub_i = nxt
        self._next_subpath()
        if self.mission is not None and self.phase in ('MOWING', 'TRANSIT'):
            self._go(self.phase)   # refresh completed_swaths / coverage

    def _subpath_failed(self, why, transit=False):
        """Retries exhausted: remember the sub-path as NOT mowed (it stays out of
        ``completed``, so a resume retries it) and go on with the next one.
        ``transit``: it never got to its start; re-tried once in the end-of-area pass."""
        m = self.mission
        self._blk = None
        if m.stretch is not None:
            self._stretch_blocked(why)
            return
        if transit:
            m.transit_failed.append((m.area_idx, m.sub_i))
        m.skipped += 1
        sp = m.subpaths[m.sub_i]
        local = self._track_progress() if m.step == 'follow' else m.start_local
        offs, _ = subpath_offsets(m.subpaths)
        m.failed.append((m.area_idx, m.sub_i, why, offs[m.sub_i] + min(local, len(sp) - 1)))
        self._incident('subpath_failed', why)
        self._log('warn', 'area %d sub-path %d NOT mowed: %s' % (m.area_idx, m.sub_i, why))
        m.step = None
        m.sub_i += 1
        self._next_subpath()

    # ---- retries ---------------------------------------------------------
    def _clear_costmaps(self):
        # global only when clear_global_costmap_on_retry (review 2026-10-09): the global costmap
        # now holds the sensor obstacle marks the detour / replan must route around; wiping it
        # on every retry sent the robot straight back through marks it could no longer see.
        if self.p.clear_costmaps_on_retry:
            self._call(SRV_CLEAR_COSTMAPS, {'global': bool(self.p.clear_global_costmap_on_retry)},
                       track=False)

    def _retry_delay(self, n):
        """Back-off before retry ``n`` (1-based): transit_retry_backoff_s[n-1], the
        last entry repeating; transit_retry_delay_s when the list is empty."""
        b = list(self.p.transit_retry_backoff_s or [])
        if not b or n is None:
            return float(self.p.transit_retry_delay_s)
        return float(b[min(max(int(n), 1), len(b)) - 1])

    def _schedule_retry(self, phase, sub, delay=None):
        """Blade off, clear the costmaps, wait ``delay`` (default transit_retry_delay_s),
        then re-send."""
        m = self.mission
        self._blk = None
        self._soft_blade_off('retry')
        self._clear_costmaps()
        m.step = 'retry_wait'
        m.retry_at = self._now + float(self.p.transit_retry_delay_s if delay is None else delay)
        self._log('warn', sub)
        self._go(phase, sub)

    def _retry_fire(self):
        m = self.mission
        m.step = None
        if m.transit_target is not None and self._transit_arrived() and \
                (m.recovering or m.transit_fails):
            self._log('info', 'retry: robot already within %.2f m of the transit target'
                      % self.p.transit_arrived_radius_m)
            m.recovering = False
            self._begin_blade()
            return
        self._dispatch(force_transit=m.recovering)

    def _not_mowed_text(self, extra_remaining=0):
        m = self.mission
        n = sum(1 for x in m.failed if x[1] is not None) + extra_remaining
        reason = m.failed[-1][2] if m.failed else ''
        unplanned = sum(1 for x in m.failed if x[1] is None)
        txt = ''
        if n or unplanned or not m.blocked:
            txt = '%d of %d sub-paths not mowed' % (n, m.planned_total)
            if unplanned:
                txt += ', %d area(s) not planned' % unplanned
        if m.blocked:
            b = blocked_text(len(m.blocked))
            txt = '%s, %s' % (txt, b) if txt else b
        return txt + (': %s' % reason if reason else '')

    def _finish_incomplete(self, text):
        """Session over with sub-paths left: latch MOWING_INCOMPLETE (resume kept)."""
        self._cancel_all('mowing incomplete')
        self._blade_off('mowing incomplete')
        self._fx.append(ZeroBurst())
        self.cursor.current_command = CMD_START
        self._persist()
        self._log('warn', 'MOWING_INCOMPLETE: %s (START resumes the missing sub-paths)' % text)
        self._go('MOWING_INCOMPLETE', text)
        if self.p.return_home_on_incomplete:
            self._incomplete_text = text
            self._dock('RETURNING_HOME', 'incomplete', sub=text)

    def _mission_incomplete(self):
        m = self.mission
        text = self._not_mowed_text()
        first = next((x for x in m.failed if x[1] is not None),
                     m.failed[0] if m.failed else m.blocked[0])
        self.cursor.current_area = first[0]
        self.mission = None
        self._fx.append(PublishAreaSettings({}))
        self._finish_incomplete(text)

    def _boundary_recovery_failed(self, why):
        """Recovery transits keep failing: stop in place, never dock from here."""
        m = self.mission
        ac = self.cursor.area(m.area_idx)
        left = sum(1 for i in range(m.sub_i, len(m.subpaths)) if i not in ac.completed)
        m.failed.append((m.area_idx, m.sub_i, why, -1))
        text = self._not_mowed_text(extra_remaining=left - 1)
        unvisited = len([a for a in m.queue if a not in self.cursor.completed_areas])
        if unvisited:
            text += ' (%d more area(s) not started)' % unvisited
        self._interrupt_mission(why)
        self._finish_incomplete(text)

    def _area_done(self):
        m = self.mission
        ac = self.cursor.area(m.area_idx)
        if m.requeue is None and ac.skipped and self.p.requeue_skipped:
            m.requeue = sorted(ac.skipped)
            self._log('info', 'area %d: re-trying %d stretch(es) skipped by obstacle detours'
                      % (m.area_idx, len(m.requeue)))
        if m.requeue:
            self._next_stretch()
            return
        if m.requeue_subs is None:
            m.requeue_subs = [x for x in m.failed if x[0] == m.area_idx and x[1] is not None
                              and (x[0], x[1]) in m.transit_failed] \
                if self.p.requeue_failed_transits else []
            if m.requeue_subs:
                self._log('info', 'area %d: re-trying %d sub-path(s) whose transit failed'
                          % (m.area_idx, len(m.requeue_subs)))
        if m.requeue_subs:
            self._next_failed_retry()
            return
        blocked = [b for b in m.blocked if b[0] == m.area_idx]
        failed = [x for x in m.failed if x[0] == m.area_idx and x[1] is not None]
        self._log('info' if not failed else 'warn',
                  'area %d done: %d/%d sub-paths mowed, %d NOT mowed' % (
                      m.area_idx, len(ac.completed), len(m.subpaths), len(failed))
                  + ('' if not blocked else ', %s' % blocked_text(len(blocked))))
        if failed or blocked:
            # not complete: keep it out of completed_areas, keep its run index, and
            # point the cursor at the first missing pose so START resumes there.
            if self.blade_on:
                self._blade_off('area finished with failures')
            ac.resume_index = min(x[3] for x in failed if x[3] >= 0) \
                if any(x[3] >= 0 for x in failed) else -1
            self._persist()
            self._next_area()
            return
        if m.runs and m.run_i + 1 < len(m.runs):
            # repeat / cross: plan and mow the same area again.
            if self.blade_on:
                self._blade_off('area run finished')
            m.run_i += 1
            self.cursor.area_runs[m.area_idx] = m.run_i
            self.cursor.areas[m.area_idx] = AreaCursor()
            self._persist()
            m.subpaths, m.lengths, m.cum = [], [], []
            m.sub_i, m.start_local, m.skipped, m.step = 0, 0, 0, None
            m.requeue, m.stretch, m.stretch_end = None, None, None
            m.requeue_subs, m.retry_entry = None, None
            self._log('info', 'area %d: run %d/%d (%s)' % (
                m.area_idx, m.run_i + 1, len(m.runs), m.settings['path_mode']))
            self._plan_run()
            return
        if m.settings is not None and m.settings['path_mode'] == 'alternate':
            name = self._area_name()
            self.alternate_counts[name] = int(self.alternate_counts.get(name, 0)) + len(m.runs)
            self._fx.append(SaveAlternateCounts(dict(self.alternate_counts)))
        self.cursor.area_runs.pop(m.area_idx, None)
        self.cursor.completed_areas.add(m.area_idx)
        ac.resume_index = -1
        self._persist()
        self._next_area()

    def _mission_complete(self):
        self._blade_off('mowing complete')
        self._go('MOWING_COMPLETE', '')
        self.mission = None
        self._fx.append(PublishAreaSettings({}))
        self._log('info', 'all areas mowed: returning home')
        self.cursor = ResumeCursor()
        self._fx.append(DeleteResume())
        self._fx.append(PublishResumeAvailable(False))
        self._dock('RETURNING_HOME', 'home')

    def _heading_failed(self):
        """No absolute heading: stop where we are. Never dock/navigate with Nav2 here, it
        would steer with the wrong heading."""
        self._log('error', 'mowing aborted: %s' % HEADING_NOT_ALIGNED)
        self._interrupt_mission(HEADING_NOT_ALIGNED)
        self._cancel_all(HEADING_NOT_ALIGNED)
        self._blade_off(HEADING_NOT_ALIGNED)
        self._go_idle(HEADING_NOT_ALIGNED)
        self._last_status = None

    # ------------------------------------------------------------------
    # heading calibration drive (2026-10-13; Params.heading_calibration_*)
    # ------------------------------------------------------------------
    def _hcal_prior_ok(self, checks=('yaw_ok',)):
        """A calibration drive may use the aligner's saved 'file' heading as its prior:
        enabled, status fresh, source file, continuity check state in ``checks`` (yaw_ok =
        the IMU kept integrating since the save, only the position is unproven); or an
        aligned 'cog' heading that is stale for planning (_heading_cog_stale, 2026-10-14)."""
        p, i = self.p, self.inputs
        if not (p.heading_calibration_drive and p.require_heading_alignment):
            return False
        if i.heading_stamp is None or self._now - i.heading_stamp > p.heading_status_timeout_s:
            return False
        if i.heading_source == 'cog':
            return bool(self._heading_cog_stale())
        return i.heading_source == 'file' and i.heading_prior in checks

    def _hcal_prior_text(self):
        i = self.inputs
        if i.heading_source == 'cog':
            return 'cog %s' % (self._heading_cog_stale() or 'fresh')
        return i.heading_prior or 'unchecked'

    def _hcal_reach(self, direction):
        """Length of the straight the robot body sweeps: drive + footprint ahead of it."""
        p = self.p
        return float(p.heading_calibration_distance_m) + float(
            p.robot_front_m if direction > 0 else p.robot_rear_m)

    def _hcal_clear(self, pose, direction):
        """The straight ``direction`` (+1 ahead, -1 behind) from ``pose`` by the PRIOR
        heading, widened by the robot width and +-heading_calibration_cone_deg, lies inside
        one mowing area (outside its keepout holes). Returns the area index or None."""
        m = self.mission
        if m is None or not m.areas or pose is None:
            return None
        p = self.p
        base = pose[2] + (0.0 if direction > 0 else math.pi)
        reach = self._hcal_reach(direction)
        hw = float(p.robot_width_m) / 2.0
        cone = math.radians(float(p.heading_calibration_cone_deg))
        n = max(2, int(math.ceil(reach / 0.1)))
        pts = []
        for da in (-cone, 0.0, cone):
            h = base + da
            c, s_ = math.cos(h), math.sin(h)
            for off in (-hw, 0.0, hw):
                x0, y0 = pose[0] - s_ * off, pose[1] + c * off
                pts.extend((x0 + c * reach * k / n, y0 + s_ * reach * k / n)
                           for k in range(n + 1))
        for idx in sorted(m.areas):
            if all(geo.inside_area(pt, m.areas[idx]) for pt in pts):
                return idx
        return None

    def _hcal_line(self, pose):
        """Obstacle corridor of the calibration drive: the rest of its straight."""
        h = self._hcal
        base = h['yaw'] + (0.0 if h['dir'] > 0 else math.pi)
        left = max(0.0, self._hcal_reach(h['dir']) - geo.dist(pose, h['start']))
        return [(pose[0], pose[1]),
                (pose[0] + left * math.cos(base), pose[1] + left * math.sin(base))]

    def _hcal_maybe_start(self):
        """RTK fixed, heading not aligned, START came off the dock with a saved heading:
        give the continuity check heading_calibration_settle_s, then drive."""
        now, p, i = self._now, self.p, self.inputs
        if self._hcal_rtk_since is None:
            self._hcal_rtk_since = now
            self._go('WAITING_FOR_RTK', 'RTK fixed: checking the saved heading')
            return
        if now - self._hcal_rtk_since < float(p.heading_calibration_settle_s):
            return
        if not self._hcal_prior_ok():
            self._hcal_fail('saved heading not usable (%s %s)'
                            % (i.heading_source, self._hcal_prior_text()), drove=False)
            return
        pose = i.pose
        for direction in (1, -1):
            area = self._hcal_clear(pose, direction)
            if area is not None:
                break
        else:
            self._hcal_fail('no clear %.1f m straight ahead or behind inside a mowing area'
                            % float(p.heading_calibration_distance_m), drove=False)
            return
        d, v = float(p.heading_calibration_distance_m), float(p.heading_calibration_speed_mps)
        word = 'forward' if direction > 0 else 'backward'
        if self.blade_on:
            self._blade_off('heading calibration')
        self._hcal = {'dir': direction, 'start': (pose[0], pose[1]), 'yaw': pose[2],
                      'done_at': None, 'outcome': None}
        self._log('info', 'heading calibration: saved heading %s, driving %.1f m %s at %.2f m/s '
                          'inside area %d (blade off)' % (self._hcal_prior_text(), d, word, v,
                                                          area))
        self._go('WAITING_FOR_RTK', 'calibrating heading: driving %.1f m %s' % (d, word))
        self._start_action(ACT_DRIVE if direction > 0 else ACT_BACKUP,
                           {'distance_m': d, 'speed_mps': v},
                           p.heading_calibration_timeout_s, purpose='heading_cal')

    def _hcal_tick(self):
        h, i = self._hcal, self.inputs
        if self._heading_plan_ok():
            if self._action is not None:
                self._fx.append(CancelActions('heading aligned'))
                self._action = None
            pose = i.pose
            moved = geo.dist(pose, h['start']) if pose is not None else float('nan')
            self._hcal = None
            self._log('info', 'heading calibrated (%s) after %.2f m' % (i.heading_source, moved))
            reason = self._preflight()
            if reason:
                self._hcal_fail('preflight after the drive: %s' % reason)
                return
            if i.fix_type is not None and i.fix_type >= self.p.rtk_fix_type:
                self._log('info', 'RTK fixed, heading aligned (%s): planning' % i.heading_source)
                self._next_area()
            else:
                self._go('WAITING_FOR_RTK', '')
            return
        if h['done_at'] is None:
            kind = self._policy()
            if kind != 'none':
                self._hcal_fail('stopped: %s %s in the way' % (
                    kind, i.obstacle_class or 'obstacle'))
            return
        if self._now - h['done_at'] > float(self.p.heading_calibration_status_wait_s):
            out = h['outcome']
            why = 'no COG alignment after the drive' if out == SUCCEEDED else \
                'drive %s%s' % (out, ' (obstacle / costmap)' if out == ABORTED else '')
            self._hcal_fail(why)

    def _hcal_result(self, outcome):
        h = self._hcal
        if h is None:
            return
        if outcome in (REJECTED, UNAVAILABLE, CANCELED) and not self._heading_plan_ok():
            self._hcal_fail('drive %s' % outcome)
            return
        h['done_at'], h['outcome'] = self._now, outcome
        self._go('WAITING_FOR_RTK', 'calibrating heading: waiting for the COG fix')

    def _hcal_fail(self, why, drove=True):
        """Refused or failed: stop (blade off, zero velocity) and stay idle where it is.
        Never dock / navigate here: Nav2 would steer with the unverified heading."""
        text = '%s (heading calibration: %s)' % (HEADING_NOT_ALIGNED, why)
        self._log('error' if drove else 'warn', 'START: %s' % text)
        self._interrupt_mission(text)
        self._cancel_all(text)
        self._blade_off(text)
        if drove:
            self._fx.append(ZeroBurst())
        self._hcal = None
        self._hcal_wanted = False
        self._go_idle(text)
        self._last_status = None

    def _mission_failed(self, reason):
        self._log('error', 'mowing aborted: %s' % reason)
        self._interrupt_mission(reason)
        self._cancel_all(reason)
        self._blade_off(reason)
        if self.inputs.docked:
            self._go_idle('mowing aborted: %s' % reason)
        else:
            self._dock('COVERAGE_FAILED_DOCKING', 'failed', sub=reason)

    def _interrupt_mission(self, why):
        """Store the precise resume cursor and drop the in-memory session."""
        m = self.mission
        if m is None:
            return
        self._blade_off(why)
        if m.area_idx is not None and m.subpaths and m.sub_i < len(m.subpaths):
            sp = m.subpaths[m.sub_i]
            local = m.start_local
            if m.step == 'follow':
                local = self._track_progress()
            elif m.detour is not None:
                local = m.detour['from']     # the skip is recorded only once around
            offs, _ = subpath_offsets(m.subpaths)
            ac = self.cursor.area(m.area_idx)
            ac.resume_index = offs[m.sub_i] + min(local, len(sp) - 1)
            self.cursor.current_area = m.area_idx
        self.cursor.current_command = CMD_START
        self._persist()
        self._log('info', 'mowing interrupted (%s): resume cursor saved' % why)
        if m.settings is not None:
            self._fx.append(PublishAreaSettings({}))
        self.mission = None

    # ==================================================================
    # docking
    # ==================================================================
    def _dock(self, name, purpose, sub=''):
        self._dyn = None
        self._blade_off('dock')
        if self._action is not None:
            self._fx.append(CancelActions('dock'))
            self._action = None
        self._dock_purpose = purpose
        self._go(name, sub)
        if self.inputs.docked:
            self._dock_done()
            return
        if self.p.use_docking_server:
            if self._dock_route_on() and self._dock_routed != (name, purpose):
                self._dock_route_tries = 0
                self._plan_dock_route()
                return
            self._dock_handover()
        elif len(self.p.dock_pose) == 3:
            self._start_action(ACT_NAV, {'pose': tuple(self.p.dock_pose)},
                               self.p.transit_timeout_s, purpose='dock')
        else:
            self._dock_failed('no docking server and no dock_pose configured')

    def _dock_route_on(self):
        return bool(self.p.route_transits and self.p.route_dock and self.inputs.pose is not None)

    def _plan_dock_route(self):
        self._call(SRV_PLAN_ROUTE, {'start': tuple(self.inputs.pose), 'to_dock': True,
                                    'variation': self._transit_run()}, purpose='dock_route')

    def _dock_approach(self):
        """The docking server's approach pose (dock + dock_approach_distance_m along the dock
        yaw, as map_server / mower_docking compute it); else the last dock route's end."""
        d = self.inputs.dock_pose
        if d is not None and len(d) >= 3:
            ad = float(self.p.dock_approach_distance_m)
            return (d[0] + ad * math.cos(d[2]), d[1] + ad * math.sin(d[2]))
        return getattr(self, '_dock_route_end', None)

    def _dock_handover(self):
        """Task 0.4 (e): the docking server (whose own navigate_to_pose drives the approach
        leg across the grass) only within dock_server_nav_max_m of the approach pose. Farther
        away: plan and drive the dock route again (dock_route_retries), then fail in place."""
        lim = float(self.p.dock_server_nav_max_m)
        ap, pose = self._dock_approach(), self.inputs.pose
        if lim > 0.0 and ap is not None and pose is not None and self._dock_route_on():
            d = geo.dist(pose, ap)
            if d > lim:
                tries = getattr(self, '_dock_route_tries', 0)
                if tries >= int(self.p.dock_route_retries):
                    self._dock_failed('dock route did not reach the approach pose (%.2f m away '
                                      'after %d re-sends); not crossing the lawn: the docking '
                                      'server navigates only within %.2f m' % (d, tries, lim))
                    return
                self._dock_route_tries = tries + 1
                self._log('warn', 'dock: %.2f m from the approach pose (> %.2f m): driving the '
                                  'dock route again (%d/%d), not the docking server\'s own '
                                  'navigation' % (d, lim, tries + 1,
                                                  int(self.p.dock_route_retries)))
                self._plan_dock_route()
                return
        self._dock_server()

    def _dock_server(self):
        self._start_action(ACT_DOCK, {'use_vision': self.p.dock_use_vision,
                                      'timeout_s': self.p.dock_timeout_s},
                           self.p.dock_action_timeout_s, purpose='dock')

    def _dock_route_planned(self, ok, resp):
        """plan_route(to_dock) answered: drive the route to the approach pose, or (no route)
        let the docking server navigate there itself."""
        if self.phase not in DOCK_PHASES or self._action is not None or \
                self._dock_purpose is None or self.inputs.docked:
            return
        poses = resp.get('poses') or []
        if not (ok and resp.get('success') and len(poses) >= 2):
            self._dock_routed = (self.phase, self._dock_purpose)
            self._log('info', 'dock route: %s: handing over to the docking server'
                      % (resp.get('message') or 'plan_route unavailable'))
            self._dock_handover()
            return
        self._dock_route_end = (float(poses[-1][0]), float(poses[-1][1]))
        prepared = self._route_prepare(poses)
        legs = (self._condition_route(prepared, 'dock route') if self.p.path_conditioning
                else None) or self._split_route(prepared)
        self._log('info', 'dock route %.1f m (%d leg%s): %s' % (
            float(resp.get('length_m', 0.0)), len(legs), '' if len(legs) == 1 else 's',
            resp.get('message', '')))
        self._route_legs = {'legs': legs, 'i': 0}
        self._send_route_leg('dock_route')

    def _dock_route_result(self, outcome, nav2_down=False):
        """The dock route ended (any outcome): the docking server takes over at the approach
        pose (task 0.4 e: only within dock_server_nav_max_m of it; farther the route is driven
        again, see _dock_handover). An obstacle abort waits first (blocked wait, then a fresh
        dock route). Nav2 down: straight to the docking server (it cannot drive the lawn
        without Nav2 either, and its failure names the cause)."""
        if self._route_next_leg(outcome, 'dock_route'):
            return
        self._route_legs = None
        self._dock_routed = (self.phase, self._dock_purpose)
        if self.inputs.docked:
            self._dock_done()
            return
        if nav2_down:
            self._log('warn', 'dock route follow_path %s: Nav2 down, handing over to the '
                              'docking server' % outcome)
            self._dock_server()
            return
        if outcome != SUCCEEDED:
            self._log('warn', 'dock route follow_path %s' % outcome)
            if outcome in (ABORTED, TIMEOUT) and float(self.p.dock_server_nav_max_m) > 0.0 \
                    and self._blocked_cause():
                self._dock_routed = None      # the re-send after the wait plans a fresh route
                if self._blocked('dock', 'dock route %s' % outcome):
                    return
                self._dock_routed = (self.phase, self._dock_purpose)
        self._dock_handover()

    def _dock_done(self):
        self._dock_routed = None
        self._dock_route_end = None
        self._blk = None
        purpose, self._dock_purpose = self._dock_purpose, None
        if not self.p.use_docking_server:
            self._call(SRV_CHARGING, {'enable': True}, track=False)
        if purpose == 'rain':
            self._rain_clear_since = None
            self._resume_after = 'rain'
            self._go('RAIN_WAITING', 'waiting %.0f min after rain' % self.p.rain_delay_minutes)
        elif purpose == 'battery':
            self._go('CHARGING', 'charging to %.0f%%, then resuming' % self._resume_charge_percent())
            self._resume_after = 'charge'
        elif purpose == 'incomplete':
            self._go_idle('docked; %s' % self._incomplete_text or 'mowing incomplete')
        else:
            self._go_idle('docked')

    def _dock_failed(self, why):
        self._dock_routed = None
        self._blk = None
        self._dock_purpose = None
        self._resume_after = None
        self._log('error', 'docking failed: %s' % why)
        self._go('NAV_TO_DOCK_FAILED', why)

    # ==================================================================
    # recording (RecordArea port)
    # ==================================================================
    def _record_start(self, kind='area'):
        if self.state == STATE_AUTONOMOUS:
            self._log('warn', 'RECORD_%s refused: autonomous run active (STOP first)'
                      % kind.upper())
            return False
        if self.phase == 'RECORDING':
            return True
        self._leave_current_mode('record %s' % kind)
        self._blade_off('recording')
        self._track = []
        self._record_poly = None
        self._record_line = None
        self._record_kind = kind
        self._last_sample = -1e9
        hint = {'path': 'drive the path with the joystick',
                'obstacle': 'drive around the obstacle with the joystick'}.get(
                    kind, 'drive the boundary with the joystick')
        self._go('RECORDING', hint)
        self._sample(force=True)
        return True

    def _sample(self, force=False):
        pose = self.inputs.pose
        if pose is None or len(self._track) >= self.p.record_max_points:
            return
        pt = (float(pose[0]), float(pose[1]))
        if self._track and geo.dist(self._track[-1], pt) < self.p.record_min_spacing_m:
            return
        self._track.append(pt)
        self._fx.append(PublishTrajectory(list(self._track)))

    def _tick_record(self):
        if self._record_poly is not None:
            return                       # saving
        if self._now - self._last_sample >= 1.0 / max(0.1, self.p.record_rate_hz) - 1e-6:
            self._last_sample = self._now
            self._sample()

    def _record_finish(self):
        if self.phase != 'RECORDING':
            self._log('warn', 'RECORD_FINISH refused: not recording')
            return False
        if self._record_poly is not None:
            return True
        if self._record_kind == 'path':
            return self._record_finish_path()
        n = len(self._track)
        poly = geo.simplify_ring(self._track, self.p.record_simplify_tolerance_m,
                                 close_radius=self.p.record_close_radius_m)
        if len(poly) < 3 and n >= 3:
            poly = list(self._track)     # upstream: keep raw if DP too aggressive
        area = geo.polygon_area(poly)
        # a recorded no-go zone is much smaller than a mowing area: its own minimum
        min_area = (self.p.record_min_obstacle_area_m2 if self._record_kind == 'obstacle'
                    else self.p.record_min_area_m2)
        if len(poly) < 3 or area < min_area:
            why = 'polygon has %d vertices / %.2f m^2 (need 3 and %.1f m^2)' % (
                len(poly), area, min_area)
            self._log('warn', 'recording rejected: %s' % why)
            self._track = []
            self._fx.append(PublishTrajectory([]))
            self._go_idle('recording rejected: %s' % why)
            return False
        self._record_poly = poly
        self._log('info', 'recording finished: %d samples -> %d vertices, %.1f m^2' % (
            n, len(poly), area))
        self._go('RECORDING', 'saving obstacle' if self._record_kind == 'obstacle'
                 else 'saving area')
        self._enum = {'purpose': 'record', 'index': 0, 'areas': {}, 'count': 0}
        self._call(SRV_GET_AREA, {'index': 0}, purpose='enum')
        return True

    def _record_finish_path(self):
        n = len(self._track)
        line = geo.simplify_open_path(self._track, self.p.record_simplify_tolerance_m)
        length = geo.path_length(line) if len(line) >= 2 else 0.0
        if len(line) < 2 or length < self.p.record_path_min_length_m:
            why = 'path is %.2f m long (need %.1f m)' % (length, self.p.record_path_min_length_m)
            self._log('warn', 'recording rejected: %s' % why)
            self._track = []
            self._fx.append(PublishTrajectory([]))
            self._go_idle('recording rejected: %s' % why)
            return False
        self._record_line = line
        self._record_poly = geo.buffer_polyline(line, self.p.record_path_width_m / 2.0)
        self._log('info', 'path recording finished: %d samples -> %d points, %.1f m' % (
            n, len(line), length))
        self._go('RECORDING', 'saving path')
        self._enum = {'purpose': 'record', 'index': 0, 'areas': {}, 'count': 0}
        self._call(SRV_GET_AREA, {'index': 0}, purpose='enum')
        return True

    def _record_save(self, mowing_count, total_count=None, areas=None):
        if self._record_kind == 'obstacle':
            areas = areas or {}
            idx = self._record_obstacle_area(areas)
            if idx is None:
                # no mowing area to store the keepout under (the server rejects
                # obstacles on navigation areas) -> keep the ring like add_area failures
                self._record_name = 'No-go %d' % (mowing_count + 1)
                self._log('warn', 'cannot save no-go zone: no mowing area to attach it to')
                self._record_saved(False, failed_service='promote_obstacle')
                return
            obstacles = sum(len(a.get('obstacles') or []) for a in areas.values())
            self._record_index = idx
            self._record_name = 'No-go %d' % (obstacles + 1)
            self._call(SRV_PROMOTE_OBSTACLE, {'area_index': idx,
                                              'polygon': list(self._record_poly),
                                              'name': self._record_name},
                       purpose='record_obstacle')
            return
        if self._record_kind == 'path':
            total = mowing_count if total_count is None else total_count
            self._record_index = total            # add_area appends at the end
            self._record_name = 'Path %d' % (total - mowing_count + 1)
            self._call(SRV_ADD_AREA, {'name': self._record_name,
                                      'polygon': list(self._record_poly),
                                      'is_navigation_area': True}, purpose='add_area')
            return
        name = 'Area %d' % (mowing_count + 1)
        self._record_name = name
        self._call(SRV_ADD_AREA, {'name': name, 'polygon': list(self._record_poly),
                                  'is_navigation_area': False}, purpose='add_area')

    def _record_obstacle_area(self, areas):
        """Mowing area a recorded no-go zone is stored under: the one containing the
        ring centroid, else the first mowing area. None when there is none. The
        containment is soft on purpose — keepout and nav masks paint obstacle
        polygons by their own geometry, so a zone outside every work area (dock
        corridor, gap between areas) still holds once stored under any mowing area."""
        if not areas:
            return None
        ring = self._record_poly or []
        c = (sum(p[0] for p in ring) / len(ring),
             sum(p[1] for p in ring) / len(ring)) if ring else None
        for i in sorted(areas):
            if c is None or geo.point_in_polygon(c, areas[i].get('outer') or []):
                return i
        return sorted(areas)[0]

    def _record_saved(self, ok, failed_service='add_area'):
        poly, self._record_poly = self._record_poly, None
        self._track = []
        self._fx.append(PublishTrajectory([]))
        if ok:
            self._log('info', '%s "%s" saved (%d vertices)' % (
                self._record_kind, self._record_name, len(poly)))
            self._go('RECORDING_COMPLETE', self._record_name)
        else:
            self._log('error', '%s failed: polygon kept in a fallback file' % failed_service)
            self._fx.append(SaveRecordingFallback(list(poly), self._record_name))
            self._go_idle('%s failed; polygon saved locally' % failed_service)

    # ==================================================================
    # results
    # ==================================================================
    # ---- route-graph transits (2026-10-09, see Params.route_transits) ---------
    TRANSIT_RUN_KEY = '__transit_run__'

    def _transit_run(self):
        """Run index for plan_route's transit variation (0 = base route)."""
        if not self.p.route_variation:
            return 0
        return int(self.alternate_counts.get(self.TRANSIT_RUN_KEY, 0))

    def _bump_transit_run(self):
        if self.p.route_transits and self.p.route_variation:
            self.alternate_counts[self.TRANSIT_RUN_KEY] = self._transit_run() + 1
            self._fx.append(SaveAlternateCounts(dict(self.alternate_counts)))

    def _send_transit(self):
        """(Re-)send the in-flight transit to ``m.transit_target``: the route first, the
        costmap planner (/navigate_to_pose) once this transit fell back."""
        m = self.mission
        m.route_poses = None
        m.route_full, m.route_cusps, m.cusp_resume = None, [], None
        m.final_approach = False
        if self.p.route_transits and not m.route_fallback and m.detour is None \
                and not m.recovering and self.inputs.pose is not None:
            m.fallback_ref = None
            self._call(SRV_PLAN_ROUTE, {'start': tuple(self.inputs.pose),
                                        'goal': tuple(m.transit_target),
                                        'variation': self._transit_run()}, purpose='route')
            return
        self._start_fallback_nav()

    def _start_fallback_nav(self, ref=None):
        """/navigate_to_pose to the transit target. A route fallback gets the corridor bound
        (transit_fallback_max_offpath_m) against ``ref`` (the route it replaces, if any; a
        re-send keeps the previous one) and the mowing areas."""
        m = self.mission
        if ref is not None:
            m.fallback_ref = [tuple(q) for q in ref]
        m.fallback_armed = None
        self._start_action(ACT_NAV, {'pose': m.transit_target}, self.p.transit_timeout_s)

    def _offpath(self, pose):
        """Distance of ``pose`` from the drawn network the fallback must stay on: the route
        it replaces (cross-track), 0 inside a mowing area (outer ring); inf when there is
        nothing to compare with."""
        m = self.mission
        best = math.inf
        if m.fallback_ref:
            best = geo.polyline_distance(pose, m.fallback_ref)
        for area in (m.areas or {}).values():
            outer = list(area.get('outer') or [])
            if len(outer) < 3:
                continue
            if geo.point_in_polygon(pose, outer):
                return 0.0
            best = min(best, geo.distance_to_ring(pose, outer))
        return best

    def _fallback_corridor_tick(self):
        """Fix 11: cancel a costmap-planner route fallback that leaves the drawn network by
        more than transit_fallback_max_offpath_m (live cross-track of the robot's own pose:
        the FSM never sees Nav2's planned path) and count it as a transit abort."""
        m, a, pose = self.mission, self._action, self.inputs.pose
        lim = float(self.p.transit_fallback_max_offpath_m)
        if lim <= 0.0 or a is None or a.name != ACT_NAV or not m.route_fallback or \
                m.detour is not None or m.recovering or pose is None:
            return
        off = self._offpath(pose)
        if math.isinf(off):
            return
        if m.fallback_armed is None:          # first tick of this fallback goal
            m.fallback_armed, m.fallback_off0 = off <= lim, off
            if not m.fallback_armed:
                self._log('info', 'transit fallback starts %.2f m off the drawn network: '
                                  'corridor bound armed once within %.2f m' % (off, lim))
            return
        if not m.fallback_armed:
            if off <= lim:
                m.fallback_armed = True
            elif off <= m.fallback_off0 + lim:
                m.fallback_off0 = min(m.fallback_off0, off)
                return
        if off <= lim:
            return
        self._log('warn', 'transit fallback left the corridor by %.2f m: abort' % off)
        self._fx.append(CancelActions('transit fallback left the corridor'))
        self._action = None
        self._transit_abort('transit fallback left the corridor (%.2f m)' % off)

    def _transit_nav_progress_tick(self):
        """2026-10-15 sim (forced spin): after a route abort the /navigate_to_pose transit
        (costmap planner; its FollowPath keeps the rotation-counting progress_checker) spun in
        place for minutes inside the corridor. The robot's distance to the transit target must
        shrink by route_progress_min_m within route_progress_window_s, else the goal is
        cancelled and counted as a transit abort. True when it fired."""
        win, need = float(self.p.route_progress_window_s), float(self.p.route_progress_min_m)
        m, a, pose = self.mission, self._action, self.inputs.pose
        if win <= 0.0 or a is None or a.name != ACT_NAV or a.purpose is not None or \
                m.detour is not None or pose is None or m.transit_target is None:
            return False
        d = geo.dist(pose, m.transit_target)
        w = self._route_watch
        if w is None or w['token'] != a.token:
            self._route_watch = {'token': a.token, 'd': d, 't': self._now}
            return False
        if d <= w['d'] - need:
            w['d'], w['t'] = d, self._now
            return False
        if self._now - w['t'] < win:
            return False
        self._log('warn', 'transit navigate_to_pose: %.2f m from the target, no %.2f m closer '
                          'for %.0f s (progress watch): abort' % (d, need, self._now - w['t']))
        self._fx.append(CancelActions('transit progress watch'))
        self._action = None
        self._route_watch = None
        self._transit_abort('transit made no progress for %.0f s' % win)
        return True

    def _transit_abort(self, why):
        """A failed mission transit: transit_fails, 'transit_abort' incident, then a retry
        after the back-off or (retries exhausted) the sub-path is dropped."""
        m = self.mission
        m.transit_fails += 1
        self._incident('transit_abort', why)
        if m.transit_fails > self.p.transit_retries:
            self._subpath_failed('%s %d times' % (why, m.transit_fails), transit=True)
            return
        delay = self._retry_delay(m.transit_fails)
        self._schedule_retry('TRANSIT', 'area %d sub-path %d/%d: %s, retry %d/%d in %.0f s'
                             % (m.area_idx, m.sub_i + 1, len(m.subpaths), why,
                                m.transit_fails, self.p.transit_retries, delay), delay=delay)

    def _route_planned(self, ok, resp):
        m = self.mission
        if m is None or self.phase != 'TRANSIT' or m.step != 'transit' or \
                self._action is not None:
            return                       # superseded (stop, guard, wait) while planning
        poses = resp.get('poses') or []
        if not (ok and resp.get('success') and len(poses) >= 2):
            why = resp.get('message') or 'plan_route unavailable'
            if self._route_replan_far(why):
                return
            m.route_fallback = True
            self._log('info', 'no route (%s): transit via the costmap planner' % why)
            m.fallback_ref = None
            self._start_fallback_nav()
            return
        m.route_plan_fails = 0
        if m.blade_transit and int(resp.get('single_area', -1)) != m.area_idx:
            # the straight segment stays in the area but the route does not (path legs)
            self._blade_off('route leaves area %d' % m.area_idx)
        m.route_poses = self._route_prepare(poses)
        legs = (self._condition_route(m.route_poses, 'route') if self.p.path_conditioning
                else None) or self._split_route(m.route_poses)
        m.route_full = m.route_poses
        m.route_cusps = self._route_cusps(m.route_poses, legs)
        self._log('info', 'route %.1f m (%.1f m on paths, %d leg%s): %s' % (
            float(resp.get('length_m', 0.0)), float(resp.get('path_length_m', 0.0)),
            len(legs), '' if len(legs) == 1 else 's', resp.get('message', '')))
        self._route_legs = {'legs': legs, 'i': 0}
        self._send_route_leg('route')

    def _route_replan_far(self, why):
        """plan_route failed for a transit goal beyond transit_fallback_max_goal_m (outside
        the costmap planner's rolling window): re-ask the route after a back-off
        (route_plan_retries), then a transit abort (its own bounded retries). True = handled
        (no costmap-planner fallback)."""
        m, pose, lim = self.mission, self.inputs.pose, float(self.p.transit_fallback_max_goal_m)
        if lim <= 0.0 or pose is None or m.transit_target is None:
            return False
        d = geo.dist(pose, m.transit_target)
        if d <= lim:
            return False
        m.route_plan_fails += 1
        if m.route_plan_fails > int(self.p.route_plan_retries):
            self._transit_abort('no route (%s) to a goal %.0f m away (beyond the costmap '
                                'planner)' % (why, d))
            return True
        b = list(self.p.route_plan_retry_backoff_s or []) or [float(self.p.transit_retry_delay_s)]
        delay = float(b[min(m.route_plan_fails, len(b)) - 1])
        self._schedule_retry('TRANSIT', 'no route (%s): goal %.1f m away is beyond the costmap '
                                        'planner (%.0f m): re-asking the route %d/%d in %.0f s'
                             % (why, d, lim, m.route_plan_fails, int(self.p.route_plan_retries),
                                delay), delay=delay)
        return True

    def _route_corner_th(self):
        """Corner threshold of the route split (rad; 0 = the legacy split)."""
        th = float(self.p.route_corner_turn_rad)
        if th > 0.0 and float(self.p.route_split_turn_deg) > 0.0:
            th = min(th, math.radians(float(self.p.route_split_turn_deg)))
        return th

    def _route_prepare(self, poses):
        """Planned route -> the poses to drive (2026-10-15 undock incident): stubs shorter
        than route_stub_trim_m before the first / after the last corner are cut off, then the
        route starts where the robot projects onto its first route_start_join_m."""
        poses = [tuple(p) for p in poses]
        th, stub = self._route_corner_th(), float(self.p.route_stub_trim_m)
        # path conditioning: no stub trimming (design 3.3) - a corner near a route end is a
        # pivot marker like any other, and the stub after it a leg that ends along-path on
        # the target (the trim ended transits up to 0.3 m short of the sub-path start)
        if th > 0.0 and stub > 0.0 and not self.p.path_conditioning:
            out = geo.trim_route_stubs(poses, th, stub, tol_rad=ROUTE_CORNER_TOL_RAD)
            if len(out) != len(poses):
                cut0 = poses.index(out[0])
                self._log('info', 'route: %.2f m stub before the corner at (%.2f, %.2f) and '
                                  '%.2f m after (%.2f, %.2f) dropped' % (
                                      geo.path_length(poses[:cut0 + 1]), out[0][0], out[0][1],
                                      geo.path_length(poses[cut0 + len(out) - 1:]),
                                      out[-1][0], out[-1][1]))
                poses = out
        join, pose = float(self.p.route_start_join_m), self.inputs.pose
        if join > 0.0 and pose is not None:
            out = geo.join_route_from(poses, pose, join)
            if out[0] != poses[0]:
                self._log('info', 'route: first leg joins at (%.2f, %.2f), the robot\'s '
                                  'projection, not the start (%.2f, %.2f) behind it'
                          % (out[0][0], out[0][1], poses[0][0], poses[0][1]))
                poses = out
        return poses

    def _split_route(self, poses):
        """Route poses -> FollowPath legs. Task 0.4 (d), route_corner_turn_rad > 0: split at
        every corner of at least that angle (geometry.route_corners: the bend between the
        straight runs either side, robust to pose spacing and drawing wobble; neighbouring
        sharp corners each get their own split), the leg ending exactly on the corner pose
        with its yaw along the run into the corner. 0 = the old split (turns sharper than
        route_split_turn_deg over 3 poses either side, ~0.3 m)."""
        th = self._route_corner_th()
        if th <= 0.0:
            return self._split_route_legacy(poses)
        poses = list(poses)
        cuts = [i for i, _t in geo.route_corners(poses, th, tol_rad=ROUTE_CORNER_TOL_RAD)]
        if not cuts:
            return [poses]
        legs, s0 = [], 0
        for c in cuts:
            leg = list(poses[s0:c + 1])
            end = leg[-1]
            if len(end) >= 3:                 # arrive along the run into the corner
                a = poses[c - 1]
                leg[-1] = (end[0], end[1], math.atan2(end[1] - a[1], end[0] - a[0])) + \
                    tuple(end[3:])
            legs.append(leg)
            s0 = c
        legs.append(list(poses[s0:]))
        return [lg for lg in legs if len(lg) >= 2] or [poses]

    def _split_route_legacy(self, poses):
        """Before task 0.4 (d) (route_corner_turn_rad 0): split at turns sharper than
        route_split_turn_deg (measured over 3 poses either side, ~0.3 m)."""
        th = math.radians(float(self.p.route_split_turn_deg))
        k = 3
        if th <= 0 or len(poses) < 2 * k + 2:
            return [list(poses)]

        def turn(j):
            a, b, c = poses[j - k], poses[j], poses[j + k]
            h1 = math.atan2(b[1] - a[1], b[0] - a[0])
            h2 = math.atan2(c[1] - b[1], c[0] - b[0])
            return abs(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1)))

        cuts, i = [], k
        while i < len(poses) - k:
            if turn(i) >= th:
                j = max(range(i, min(len(poses) - k, i + 2 * k)), key=turn)
                cuts.append(j)
                i = j + 2 * k
            else:
                i += 1
        legs, s0 = [], 0
        for c in cuts:
            legs.append(list(poses[s0:c + 1]))
            s0 = c
        legs.append(list(poses[s0:]))
        return [lg for lg in legs if len(lg) >= 2] or [list(poses)]

    def _route_cusps(self, poses, legs):
        """Cusps of a planned route (geometry.route_cusps at route_cusp_angle_rad) plus the
        junctions between its FollowPath legs that turn more than that angle."""
        ang = float(self.p.route_cusp_angle_rad)
        if ang <= 0.0 or int(self.p.route_cusp_retries) <= 0:
            return []
        cusps = list(geo.route_cusps(poses, ang))
        k = 3
        for a, b in zip(legs, legs[1:]):
            if len(a) < 2 or len(b) < 2:
                continue
            p0, j, p2 = a[max(0, len(a) - 1 - k)], a[-1], b[min(len(b) - 1, k)]
            h1 = math.atan2(j[1] - p0[1], j[0] - p0[0])
            h2 = math.atan2(p2[1] - j[1], p2[0] - j[0])
            if abs(geo.wrap_angle(h2 - h1)) < ang or \
                    any(geo.dist(j, (c[1], c[2])) <= 0.3 for c in cusps):
                continue
            i = min(range(len(poses)), key=lambda q: geo.dist(poses[q], j))
            cusps.append((i, float(j[0]), float(j[1]), h2))
        return sorted(cusps)

    def _route_cusp_turn(self, why):
        """Fix 11: a route follow abort near a route cusp. None = no cusp within
        route_cusp_radius_m (the caller goes on as before); True = about-turn started;
        False = this cusp's about-turns are used up (the caller aborts the transit)."""
        m, pose, route = self.mission, self.inputs.pose, self.mission.route_full
        r = float(self.p.route_cusp_radius_m)
        if not route or not m.route_cusps or pose is None or len(pose) < 3 or r <= 0.0:
            return None
        near = [c for c in m.route_cusps if geo.dist(pose, (c[1], c[2])) <= r]
        if not near:
            return None
        i, cx, cy, heading = min(near, key=lambda c: geo.dist(pose, (c[1], c[2])))
        key = (round(cx, 2), round(cy, 2))
        n = m.cusp_turns.get(key, 0)
        limit = int(self.p.route_cusp_retries)
        if n >= limit:
            self._log('warn', 'route cusp at (%.2f, %.2f): %s after %d about-turn%s: transit '
                              'abort (no costmap fallback at a cusp)'
                      % (cx, cy, why, n, '' if n == 1 else 's'))
            return False
        m.cusp_turns[key] = n + 1
        rest = [tuple(q) for q in route[i:]]
        if len(rest) < 2:
            rest = [(cx, cy, heading), tuple(route[-1])]
        m.cusp_resume = rest
        m.route_poses = rest
        m.cusp_heading = heading
        delta = geo.wrap_angle(heading - float(pose[2]))
        self._log('warn', 'route cusp at (%.2f, %.2f): about-turn %d/%d then resume route (%s; '
                          'turn %.0f deg)' % (cx, cy, n + 1, limit, why, math.degrees(delta)))
        self._go(self.phase, 'route cusp: about-turn %d/%d' % (n + 1, limit))
        if abs(delta) < 0.3:                  # already facing the route after the cusp
            self._resume_after_cusp()
            return True
        allowance = abs(delta) / max(0.05, float(self.p.route_cusp_spin_rps)) + 10.0
        self._start_action(ACT_SPIN, {'angle': delta, 'time_allowance_s': allowance},
                           allowance + 5.0, purpose='route_cusp')
        return True

    def _resume_after_cusp(self):
        """Send the route remainder after a cusp about-turn as FollowPath legs."""
        m = self.mission
        rest = m.cusp_resume
        m.cusp_resume = None
        m.route_poses = rest
        # the cusp about-turn was this leg's pivot (task 0.4 d: no second one, even after a
        # failed Spin; the route follow is the judge)
        self._route_legs = {'legs': self._split_route(rest), 'i': 0, 'turned': 0}
        self._send_route_leg('route')

    def _route_cusp_spun(self, outcome):
        """About-turn ended: resume the route (an unfinished about-turn first goes through
        _pivot_unfinished: pivot again or stop)."""
        m = self.mission
        if m is None or self.phase != 'TRANSIT' or m.step != 'transit' or not m.cusp_resume:
            return
        if outcome == SUCCEEDED:
            self._pivot_tries = {}
        else:
            q = m.cusp_resume[0]
            if self._pivot_unfinished(outcome, 'route_cusp', ('cusp', round(q[0], 2),
                                                             round(q[1], 2)),
                                      m.cusp_heading,
                                      self._pivot_follow_rad(), 'route cusp about-turn'):
                return
        self._resume_after_cusp()

    def _pivot_follow_rad(self):
        """Largest heading error a leg is followed from after an unfinished pivot (cusp,
        final approach): the path conditioning pivot threshold, else route_leg_pivot_min_rad."""
        lim = float(self.p.path_conditioning_pivot_min_rad)
        if self.p.path_conditioning and lim > 0.0:
            return lim
        return max(0.3, float(self.p.route_leg_pivot_min_rad))

    @staticmethod
    def _leg_heading(leg, look=0.3):
        """Heading of a route leg's start: chord to the first pose ``look`` m along it."""
        a = leg[0]
        for q in leg[1:]:
            if geo.dist(a, q) >= look:
                return math.atan2(q[1] - a[1], q[0] - a[0])
        b = leg[-1]
        if geo.dist(a, b) < 1e-6:
            return None
        return math.atan2(b[1] - a[1], b[0] - a[0])

    def _route_corner_turn(self, purpose):
        """Task 0.4 (d): before route leg ``i`` (transit or dock route), pivot in place to
        its heading when that is more than route_corner_turn_rad off the robot's yaw and its
        start is within route_cusp_radius_m. True = the about-turn was started."""
        lim, rl, pose = float(self.p.route_corner_turn_rad), self._route_legs, self.inputs.pose
        if lim <= 0.0 or rl is None or pose is None or len(pose) < 3 or \
                rl.get('turned') == rl['i']:
            return False
        leg = rl['legs'][rl['i']]
        if len(leg) < 2 or geo.dist(pose, leg[0]) > float(self.p.route_cusp_radius_m):
            return False
        heading = self._leg_heading(leg)
        if heading is None:
            return False
        delta = geo.wrap_angle(heading - float(pose[2]))
        piv = float(self.p.route_leg_pivot_min_rad)
        if piv > 0.0 and rl['i'] > 0:          # stopped at a split corner
            lim = min(lim, piv)
        if self.p.path_conditioning and float(self.p.path_conditioning_pivot_min_rad) > 0.0:
            # one pivot rule (as _cov_pivot): every leg start, the first one too, is a vertex
            # of the driven path no fillet fits (sim gate 2026-10-16: MPPI turned onto a
            # transit 33 deg off and ran 0.17 m beside it)
            lim = float(self.p.path_conditioning_pivot_min_rad)
        if abs(delta) < lim - ROUTE_CORNER_TOL_RAD:     # inclusive, as the split
            return False
        rl['turned'], rl['purpose'] = rl['i'], purpose
        rl['pivot'] = (heading, lim - ROUTE_CORNER_TOL_RAD)
        self._log('info', 'route corner at (%.2f, %.2f): stop and pivot %.0f deg to the next '
                          'leg (%d/%d), then follow it'
                  % (leg[0][0], leg[0][1], math.degrees(delta), rl['i'] + 1, len(rl['legs'])))
        self._go(self.phase, 'route corner: pivot %.0f deg' % math.degrees(delta))
        allowance = abs(delta) / max(0.05, float(self.p.route_cusp_spin_rps)) + 10.0
        self._start_action(ACT_SPIN, {'angle': delta, 'time_allowance_s': allowance},
                           allowance + 5.0, purpose='route_corner')
        return True

    def _route_corner_spun(self, outcome):
        """Corner pivot ended (any outcome: the route follow is the judge): send the leg."""
        rl, m = self._route_legs, self.mission
        if rl is None:
            return
        purpose = rl.get('purpose', 'route')
        if purpose == 'route':
            if m is None or self.phase != 'TRANSIT' or m.step != 'transit':
                return
        elif self.phase not in DOCK_PHASES or self._dock_purpose is None:
            return
        if outcome == SUCCEEDED:
            self._pivot_tries = {}
        else:
            leg = rl['legs'][rl['i']]
            heading, lim = rl.get('pivot') or (None, 0.0)
            if self._pivot_unfinished(outcome, 'route_corner',
                                      ('corner', round(leg[0][0], 2), round(leg[0][1], 2)),
                                      heading, lim, 'route corner'):
                return
        self._send_route_leg(purpose)

    def _send_route_leg(self, purpose):
        rl = self._route_legs
        if self._route_corner_turn(purpose):
            return
        checker = self.p.route_goal_checker_id
        if (rl['i'] + 1 < len(rl['legs']) and float(self.p.route_corner_turn_rad) > 0.0 or
                self.p.path_conditioning) and self.p.route_corner_goal_checker_id:
            # task 0.4 (d): end on the corner; path conditioning: every leg (the last one
            # too) ends along-path on its end, not 0.25 m short on the transit disc
            checker = self.p.route_corner_goal_checker_id
        self._start_action(ACT_FOLLOW, {'poses': rl['legs'][rl['i']],
                                        'controller_id': self.p.route_controller_id,
                                        'goal_checker_id': checker,
                                        'progress_checker_id': self.p.route_progress_checker_id},
                           self.p.transit_timeout_s, purpose=purpose)
        self._route_watch = None

    def _route_progress_tick(self):
        """2026-10-15 undock incident: a route leg (transit or dock route) whose robot
        projection has not advanced route_progress_min_m along the leg for
        route_progress_window_s (turning / circling in place) is cancelled and handled as a
        FollowPath abort. True when it fired."""
        win, need = float(self.p.route_progress_window_s), float(self.p.route_progress_min_m)
        a, rl, pose = self._action, self._route_legs, self.inputs.pose
        if a is None or a.name != ACT_FOLLOW or rl is None or pose is None or \
                a.purpose not in ('route', 'dock_route'):
            return False
        leg = rl['legs'][rl['i']]
        s = geo.path_projection(leg, pose)[0]
        w = self._route_watch
        if w is None or w['token'] != a.token:
            w = self._route_watch = {'token': a.token, 's': s, 't': self._now}
        if w['t'] == self._now:
            return False
        if s >= w['s'] + need:
            w['s'], w['t'] = s, self._now
            return False
        if win <= 0.0 or self._now - w['t'] < win:
            return False
        msg = 'no progress along route leg %d/%d for %.0f s (%.2f of %.2f m; route progress ' \
              'watch)' % (rl['i'] + 1, len(rl['legs']), self._now - w['t'], s,
                          geo.path_length(leg))
        self._log('warn', '%s: cancelling follow_path, handled as an abort' % msg)
        self._fx.append(CancelActions('route progress watch'))
        self._action = None
        self._route_watch = None
        self._handle_action(a, ABORTED, {'message': msg})
        return True

    def _route_next_leg(self, outcome, purpose):
        """A route leg succeeded at its end and another follows: send it. True if sent."""
        rl, pose = self._route_legs, self.inputs.pose
        if rl is None or rl['i'] + 1 >= len(rl['legs']) or pose is None:
            return False
        d = geo.dist(pose, rl['legs'][rl['i']][-1])
        if outcome != SUCCEEDED:
            acc = float(self.p.route_leg_end_accept_m)
            if outcome not in (ABORTED, TIMEOUT) or acc <= 0.0 or d > acc or \
                    self._blocked_cause():
                return False
            self._log('info', 'route leg %d/%d %s %.2f m from its corner: counted as reached'
                      % (rl['i'] + 1, len(rl['legs']), outcome, d))
        elif d > self.p.route_end_tolerance_m:
            return False
        rl['i'] += 1
        self._send_route_leg(purpose)
        return True

    def _route_result(self, outcome):
        """Route follow ended. Arrived -> mow. An obstacle abort holds first (blocked wait)
        and re-sends the route up to route_blocked_retries times, then through
        /navigate_to_pose. An abort / timeout at a route cusp turns about and re-sends the
        rest of the route (route_cusp_retries), then counts as a transit abort (fix 11).
        Any other failure re-sends via /navigate_to_pose at once without consuming a
        transit retry (those stay for the costmap planner, bounded to the corridor)."""
        m = self.mission
        if self._route_next_leg(outcome, 'route'):
            return
        self._route_legs = None
        route = m.route_poses
        m.route_poses = None
        pose = self.inputs.pose
        d = geo.dist(pose, m.transit_target) if pose is not None else math.inf
        if (outcome == SUCCEEDED and d <= self.p.route_end_tolerance_m) or \
                (outcome != UNAVAILABLE and self._transit_arrived()):
            if self._route_final_approach(d):
                return
            m.recovering = False
            m.transit_fails = 0
            m.route_plan_fails = 0
            self._blk = None
            self._begin_blade()
            return
        why = ('route follow_path %s' % outcome) if outcome != SUCCEEDED else \
            'route follow_path succeeded %.2f m from the target' % d
        m.route_aborts += 1
        if outcome in (ABORTED, TIMEOUT) and self._blocked_cause():
            # obstacle: wait, then re-send the route (route_blocked_retries), later Nav2
            m.route_fallback = m.route_aborts > int(self.p.route_blocked_retries)
            if m.route_fallback:
                m.fallback_ref = route
            if self._blocked('transit', why):
                return
        if outcome in (ABORTED, TIMEOUT):
            turned = self._route_cusp_turn(why)
            if turned:
                return
            if turned is False:
                self._transit_abort('%s at a route cusp' % why)
                return
        m.route_fallback = True
        self._log('warn', '%s: transit via the costmap planner' % why)
        self._start_fallback_nav(ref=route)

    def _route_final_approach(self, d):
        """Path conditioning: a transit that ended (arrived) more than route_final_approach_m
        from the sub-path start drives onto it first. The junction robot -> start is part of
        the driven path: a straight hop as explicit steps, like a pivot (Nav2 Spin to face
        the start when that is path_conditioning_pivot_min_rad or more off the robot's yaw,
        then Nav2 DriveOnHeading the remaining distance at route_final_approach_speed_mps),
        not a tracker leg (sim gate 2026-10-16: transits ended 0.1-0.23 m beside the start
        and the start pivot / rejoin ran that far off the line; as a 0.1 m FollowPath leg
        MPPI reversed 0.4 m out of the area). Once per transit; the outcome is not judged
        (the transit had arrived). True = started."""
        m, pose, tol = self.mission, self.inputs.pose, float(self.p.route_final_approach_m)
        if not self.p.path_conditioning or tol <= 0.0 or d <= tol or pose is None or \
                len(pose) < 3 or m.transit_target is None or m.final_approach:
            return False
        m.final_approach = True
        b = m.transit_target
        delta = geo.wrap_angle(math.atan2(b[1] - pose[1], b[0] - pose[0]) - float(pose[2]))
        self._log('info', 'transit ended %.2f m from the sub-path start: final approach onto '
                          'it (turn %.0f deg, then straight)' % (d, math.degrees(delta)))
        if abs(delta) >= float(self.p.path_conditioning_pivot_min_rad):
            allowance = abs(delta) / max(0.05, float(self.p.route_cusp_spin_rps)) + 10.0
            self._start_action(ACT_SPIN, {'angle': delta, 'time_allowance_s': allowance},
                               allowance + 5.0, purpose='final_approach_spin')
            return True
        self._final_approach_drive()
        return True

    def _final_approach_drive(self):
        """The straight hop of the final approach: the start's distance along the heading."""
        m, pose = self.mission, self.inputs.pose
        b = m.transit_target
        dist = 0.0
        if pose is not None and len(pose) >= 3:
            dist = (b[0] - pose[0]) * math.cos(pose[2]) + (b[1] - pose[1]) * math.sin(pose[2])
        if dist < 0.02:
            self._final_approach_done(SUCCEEDED)
            return
        speed = float(self.p.route_final_approach_speed_mps)
        self._start_action(ACT_DRIVE, {'distance_m': dist, 'speed_mps': speed,
                                       'wait_for_rtk': False, 'rtk_timeout_s': 0.0},
                           dist / max(0.01, speed) + 15.0, purpose='final_approach')

    def _final_approach_done(self, outcome):
        m = self.mission
        if m is None or self.phase != 'TRANSIT' or m.step != 'transit':
            return
        if outcome != SUCCEEDED:
            self._log('warn', 'final approach %s: mowing from here' % outcome)
        m.recovering = False
        m.transit_fails = 0
        m.route_plan_fails = 0
        self._blk = None
        self._begin_blade()

    def _transit_arrived(self):
        """True when the last transit target is within transit_arrived_radius_m."""
        m = self.mission
        tgt = getattr(m, 'transit_target', None) if m is not None else None
        if tgt is None or self.inputs.pose is None or self.p.transit_arrived_radius_m <= 0:
            return False
        return geo.dist(self.inputs.pose, tgt) <= self.p.transit_arrived_radius_m

    def _handle_action(self, a, outcome, result):
        name = a.name
        if a.purpose == 'preview':
            self._preview_on_plan(outcome, result)
            return
        if outcome != SUCCEEDED:
            self._log('warn', '%s finished: %s %s' % (name, outcome, result.get('message', '')))
        nav2_down = outcome != SUCCEEDED and bool(result.get(NAV2_INACTIVE_KEY))
        if nav2_down:
            self._log('error', '%s %s: %s (%s)' % (name, outcome, NAV2_DOWN,
                                                  result.get(NAV2_INACTIVE_KEY)))
        if a.purpose == 'heading_cal':
            self._hcal_result(outcome)
            return
        if a.purpose == 'dock_route':
            self._dock_route_result(outcome, nav2_down)
            return
        if a.purpose == 'route_corner':
            self._route_corner_spun(outcome)
            return
        if a.purpose == 'final_approach_spin':
            m = self.mission
            if m is not None and self.phase == 'TRANSIT' and m.step == 'transit':
                if outcome == SUCCEEDED:
                    self._pivot_tries = {}
                else:
                    b, pose = m.transit_target, self.inputs.pose
                    heading = None if pose is None else \
                        math.atan2(b[1] - pose[1], b[0] - pose[0])
                    if self._pivot_unfinished(outcome, 'final_approach_spin',
                                              ('final', round(b[0], 2), round(b[1], 2)),
                                              heading, self._pivot_follow_rad(),
                                              'final approach'):
                        return
                self._final_approach_drive()
            return
        if a.purpose == 'final_approach':
            self._final_approach_done(outcome)
            return
        if a.purpose == 'dock':
            self._dock_contact_since = None
            if outcome == SUCCEEDED and (result.get('success', True)):
                self._dock_done()
            elif self._at_dock() and outcome != CANCELED:
                self._log('warn', 'docking action ended %s (%s) but the contacts report '
                          'docked: treating as docked' % (outcome, result.get('message', '')))
                self._dock_done()
            elif outcome in (ABORTED, TIMEOUT) and self._blocked_cause() and \
                    self._blocked('dock', 'docking %s' % outcome):
                return
            else:
                why = result.get('message') or outcome
                if outcome == UNAVAILABLE:
                    why = 'docking action server unavailable'
                if nav2_down:
                    # a navigation failure says nothing beyond "Nav2 down"; anything
                    # else the docking server reported is kept as the detail
                    nav = outcome in (REJECTED, UNAVAILABLE) or \
                        str(why).startswith('NAV_TO_DOCK_FAILED')
                    why = NAV2_DOWN if nav else nav2_down_why(why)
                self._dock_failed(why)
            return
        if self._manual_undocking and name in (ACT_UNDOCK, ACT_BACKUP):
            self._manual_undocking = False
            if outcome == SUCCEEDED and result.get('success', True):
                self._log('info', 'UNDOCK done: idle off the dock')
                self._go_idle('undocked')
            else:
                why = result.get('message') or outcome
                if outcome == UNAVAILABLE:
                    why = '/mower_docking/undock action server unavailable' \
                        if name == ACT_UNDOCK else '/backup action server unavailable'
                if nav2_down:
                    why = nav2_down_why(why)
                self._log('error', 'undock failed: %s' % why)
                self._go('UNDOCK_FAILED', why)
            return
        m = self.mission
        if m is None:
            return
        if nav2_down and name in (ACT_FOLLOW, ACT_NAV, ACT_SPIN):
            # Retrying / falling back to the costmap planner cannot help while the stack
            # is down (each attempt waits out nav2_ready_timeout_s again, then every
            # sub-path is skipped as "transit rejected"): abort with the real cause.
            self._mission_failed(nav2_down_why('%s %s' % (name, outcome)))
            return
        if a.purpose == 'route':
            self._route_result(outcome)
            return
        if a.purpose == 'route_cusp':
            self._route_cusp_spun(outcome)
            return
        if a.purpose == 'cov_pivot':
            self._cov_pivot_done(outcome)
            return
        if name in (ACT_UNDOCK, ACT_BACKUP):
            if outcome == SUCCEEDED and result.get('success', True):
                m.left_dock = True
                self._go('WAITING_FOR_RTK', '')
            else:
                why = result.get('message') or outcome
                if outcome == UNAVAILABLE:
                    why = '/mower_docking/undock action server unavailable' \
                        if name == ACT_UNDOCK else '/backup action server unavailable'
                if nav2_down:
                    why = nav2_down_why(why)
                self._log('error', 'undock failed: %s' % why)
                self.mission = None
                self._go('UNDOCK_FAILED', why)
        elif name == ACT_PLAN:
            self._on_plan(outcome, result)
        elif name == ACT_NAV:
            if m.detour is not None and outcome != UNAVAILABLE:
                if outcome == SUCCEEDED or self._transit_arrived():
                    self._detour_done()
                else:
                    self._detour_failed(outcome)
                return
            if outcome == SUCCEEDED:
                m.recovering = False
                m.transit_fails = 0
                m.route_plan_fails = 0
                self._blk = None
                self._begin_blade()
            elif outcome == UNAVAILABLE:
                self._mission_failed('/navigate_to_pose action server unavailable')
            elif self._transit_arrived():
                d = geo.dist(self.inputs.pose, m.transit_target)
                self._log('warn', 'transit %s but robot is %.2f m from the target '
                                  '(<= %.2f m): treating as arrived'
                          % (outcome, d, self.p.transit_arrived_radius_m))
                m.recovering = False
                self._blk = None
                self._begin_blade()
            elif m.recovering:
                m.recovery_fails += 1
                why = 'boundary recovery transit %s' % outcome
                if m.recovery_fails > self.p.boundary_recovery_retries:
                    self._boundary_recovery_failed('%s %d times' % (why, m.recovery_fails))
                    return
                self._schedule_retry('TRANSIT', 'area %d sub-path %d/%d: %s, retry %d/%d in %.0f s'
                                     % (m.area_idx, m.sub_i + 1, len(m.subpaths), why,
                                        m.recovery_fails, self.p.boundary_recovery_retries,
                                        self.p.transit_retry_delay_s))
            else:
                if self._blocked_cause() and self._blocked('transit', 'transit %s' % outcome):
                    return
                self._transit_abort('transit %s' % outcome)
        elif name == ACT_FOLLOW:
            if outcome == SUCCEEDED:
                self._blk = None
                cum = m.cum[m.sub_i]
                prog = self._track_progress()
                last = self._seg_last()
                end = min(m.chunk_end, last)
                left = cum[end] - cum[min(prog, end)]
                if left > self.p.follow_end_tolerance_m:
                    # Stock Jazzy goal checkers only look at the final pose; a coverage
                    # path that passes near its own end "succeeds" early.
                    if m.premature >= self.p.follow_premature_retries:
                        self._subpath_failed('follow_path keeps reporting success with '
                                             '%.1f m left' % left)
                        return
                    m.premature += 1
                    m.start_local = prog
                    self._log('warn', 'follow_path succeeded with %.1f m of sub-path %d left '
                                      '(goal checker fired early?): continuing from pose %d '
                                      '(%d/%d)' % (left, m.sub_i, prog, m.premature,
                                                   self.p.follow_premature_retries))
                    self._dispatch()
                    return
                if end < last:
                    m.start_local = end           # next chunk of the same sub-path
                    m.retry_used = False
                    m.follow_fails = 0
                    self._dispatch()
                    return
                if m.stretch is None:
                    self._log('info', 'area %d sub-path %d done (%.1f m)' % (
                        m.area_idx, m.sub_i, cum[-1]))
                self._segment_done()
            elif outcome == UNAVAILABLE:
                self._mission_failed('/follow_path action server unavailable')
            else:
                prog = self._track_progress()
                if self._policy() == 'dynamic':
                    m.start_local = prog
                    m.step = None
                    self._dyn_start('follow')
                    return
                if m.stretch is not None:
                    self._stretch_blocked('follow_path %s at pose %d' % (outcome, prog))
                    return
                pf = int(self.p.detour_unclassified_after)
                static = self._static_recent()
                collided = self._collision_recent()
                # aborts inside a blocked wait count toward the unclassified-detour threshold
                nb = self._blk['n'] if self._blk is not None and \
                    self._blk['ctx'] == 'follow' else 0
                if (static or (pf >= 0 and m.follow_fails + nb >= pf and collided)) and \
                        self._try_detour(
                            prog, '%s obstacle' % (self.inputs.obstacle_class or 'static')
                            if static else 'follow_path %s (costmap collision ahead)' % outcome):
                    self._blk = None
                    return
                if collided:
                    m.start_local = prog
                    if self._blocked('follow', 'follow_path %s at pose %d' % (outcome, prog)):
                        return
                m.follow_fails += 1
                m.start_local = prog
                self._incident('follow_abort', 'follow_path %s at pose %d' % (outcome, prog))
                if m.follow_fails > self.p.follow_retries:
                    self._subpath_failed('follow_path %s %d times (at pose %d)'
                                         % (outcome, m.follow_fails, m.start_local))
                    return
                delay = self._retry_delay(m.follow_fails)
                self._schedule_retry('MOWING', 'area %d sub-path %d/%d: follow_path %s at pose %d, '
                                     'retry %d/%d in %.0f s' % (
                                         m.area_idx, m.sub_i + 1, len(m.subpaths), outcome,
                                         m.start_local, m.follow_fails, self.p.follow_retries,
                                         delay), delay=delay)

    # ==================================================================
    # obstacle avoidance
    # ==================================================================
    def _policy(self):
        """Fresh /obstacle_policy kind: 'none' | 'dynamic' | 'static'."""
        i = self.inputs
        if not self.p.obstacle_avoidance or i.obstacle_stamp is None or \
                self._now - i.obstacle_stamp > self.p.obstacle_policy_timeout_s:
            return 'none'
        kind = i.obstacle_kind if i.obstacle_kind in ('dynamic', 'static') else 'none'
        if i.dock_marker_in_view and self.phase in DOCK_PHASES:
            # Owner: "if the rear camera has the dock marker in vision, all other camera
            # detections should be ignored" (2026-10-07: the owner by the dock cancelled
            # DOCKING twice as 'dynamic obstacle: person'). Bumper/costmap stay active.
            if kind != 'none' and not getattr(self, '_marker_ignore_logged', False):
                self._marker_ignore_logged = True
                self._log('info', 'marker in view: obstacle detections ignored while docking')
            return 'none'
        self._marker_ignore_logged = False
        if self.phase in DOCK_PHASES and self._in_dock_zone():
            if kind != 'none' and not getattr(self, '_zone_ignore_logged', False):
                self._zone_ignore_logged = True
                self._log('info', 'within %.1f m of the dock: obstacle detections ignored '
                          'while docking' % self.p.dock_zone_radius_m)
            return 'none'
        self._zone_ignore_logged = False
        self._path_eval = None
        if kind != 'none' and self.p.obstacle_path_filter:
            on = self._obstacle_on_path()
            self._decide(kind, on)
            if on is False:
                return 'none'
        elif kind == 'none' and self.obstacle_decision.get('kind') != 'none':
            self.obstacle_decision = {'kind': 'none'}
        return kind

    def _in_dock_zone(self):
        """Robot within dock_zone_radius_m of the dock pose (topic, else params.dock_pose)."""
        pose = self.inputs.pose
        dock = self.inputs.dock_pose or (tuple(self.p.dock_pose) if len(self.p.dock_pose) >= 2
                                         else None)
        r = float(self.p.dock_zone_radius_m)
        if pose is None or dock is None or r <= 0.0:
            return False
        return math.hypot(pose[0] - dock[0], pose[1] - dock[1]) <= r

    def _fresh_motion(self):
        """(v, w) of the applied /cmd_vel, else /cmd_vel_nav, younger than
        obstacle_twist_max_age_s; None when neither is fresh."""
        i, age = self.inputs, float(self.p.obstacle_twist_max_age_s)
        for cmd, t in ((i.cmd_vel, i.cmd_vel_stamp), (i.nav_cmd, i.nav_cmd_stamp)):
            if cmd is not None and t is not None and self._now - t <= age:
                return float(cmd[0]), float(cmd[1])
        return None

    def _path_motion(self):
        """Twist the in-path prediction uses: while a dynamic stop holds, the one captured
        when it started (the robot is stopped now; judging the wait against "standing
        still" would call the same person advisory and resume into them), else fresh."""
        if self._dyn is not None:
            return self._dyn.get('motion')
        return self._fresh_motion()

    def _upcoming_path(self):
        """How a detection is judged right now, or None (no path to judge against: it
        counts as before):

        * ``('corridor', pts)``: the path the controller tracks -- the next
          obstacle_path_lookahead_m of the MOWING sub-path or of a route transit. Kept for
          these: it is the true path (a pivot at a swath end follows it too).
        * ``('swept', poses)``: robot poses along the predicted motion (_path_motion) for
          obstacle_sweep_horizon_s -- /navigate_to_pose transits (detours, gaps, retries)
          and dock phases while the robot moves (Nav2 plans its own path; the straight line
          to the target is not where a pivoting robot goes, 2026-10-13).
        * ``('line', pts)``: straight to the transit target, for a TRANSIT standing still
          with no prediction (waits, goal just sent): the intended direction.

        While a dynamic stop holds, the kind chosen when it started is kept."""
        m, pose = self.mission, self.inputs.pose
        if pose is None:
            return None
        frozen = self._dyn.get('path') if self._dyn is not None else None
        motion = self._path_motion()
        moving = motion is not None and (abs(motion[0]) >= _STILL_V or abs(motion[1]) >= _STILL_W)
        if frozen == 'swept' and motion is not None:
            moving = True
        elif frozen in ('line', 'corridor'):
            moving = False

        def swept():
            return 'swept', geo.predict_poses(pose, motion[0], motion[1],
                                              float(self.p.obstacle_sweep_horizon_s))

        if self.phase in DOCK_PHASES:
            return swept() if moving else None
        if self._hcal is not None and self.phase == 'WAITING_FOR_RTK':
            return 'line', self._hcal_line(pose)      # heading calibration drive
        if m is None or self.phase not in ('MOWING', 'TRANSIT'):
            return None
        look = float(self.p.obstacle_path_lookahead_m)
        pts = [(pose[0], pose[1])]
        rp = m.route_poses if (m.step == 'transit' or frozen == 'corridor') else None
        if rp:
            # route transit: the corridor follows the route, not the straight line
            k = min(range(len(rp)), key=lambda j: geo.dist(pose, rp[j]))
            return 'corridor', _along(pts, rp[k:], look)
        if m.step == 'transit' or (self.phase == 'TRANSIT' and m.step in (
                'dyn_wait', 'blocked_wait', 'retry_wait')):
            if moving:
                return swept()
            t = m.transit_target
            if t is None:
                return None
            d = geo.dist(pose, t)
            f = 1.0 if d <= look or d < 1e-6 else look / d
            pts.append((pose[0] + (t[0] - pose[0]) * f, pose[1] + (t[1] - pose[1]) * f))
            return 'line', pts
        if not m.subpaths or m.sub_i >= len(m.subpaths):
            return None
        sp = m.subpaths[m.sub_i]
        i = m.progress_local if m.step == 'follow' else m.start_local
        return 'corridor', _along(pts, sp[max(0, min(i, len(sp) - 1)):], look)

    def _obstacle_on_path(self):
        """True / False: the ranged detection is / is not in the path (see _upcoming_path;
        anything within the circumscribed footprint + margin always is); None: cannot tell
        (unranged, no bearing, no path) -> counts as before. Records how it was judged in
        ``self._path_eval`` (for /mission/obstacle_decision and the dynamic stop)."""
        self._path_eval = None
        i = self.inputs
        d, b, pose = i.obstacle_distance, i.obstacle_bearing, i.pose
        if d is None or b is None or pose is None:
            return None
        yaw = pose[2] if len(pose) > 2 else 0.0
        a = yaw + math.radians(float(b))
        o = (pose[0] + d * math.cos(a), pose[1] + d * math.sin(a))
        p = self.p
        hw, margin = float(p.robot_width_m) / 2.0, float(p.obstacle_path_margin_m)
        front, rear = float(p.robot_front_m), float(p.robot_rear_m)
        model = self._upcoming_path()
        self._path_eval = {'path': model[0] if model else None,
                           'motion': self._path_motion()}
        if geo.dist(o, pose) <= math.hypot(max(front, rear), hw) + margin:
            self._path_eval['by'] = 'footprint'
            return True
        if model is None:
            return None
        kind, pts = model
        self._path_eval['by'] = kind
        if kind == 'swept':
            return geo.swept_hit(o, pts, front, rear, hw, margin)
        best = float('inf')
        for p0, p1 in zip(pts, pts[1:] if len(pts) > 1 else pts):
            best = min(best, _seg_dist(o, p0, p1))
        return best <= hw + margin

    def _decide(self, kind, on):
        i = self.inputs
        ev = getattr(self, '_path_eval', None) or {}
        mo = ev.get('motion')
        dec = {'kind': kind, 'class': i.obstacle_class, 'distance_m': i.obstacle_distance,
               'bearing_deg': i.obstacle_bearing,
               'on_path': on, 'action': 'advisory' if on is False else 'act',
               'lookahead_m': float(self.p.obstacle_path_lookahead_m),
               'half_width_m': float(self.p.robot_width_m) / 2.0
               + float(self.p.obstacle_path_margin_m),
               'judged_by': ev.get('by', ev.get('path')),
               'motion': None if mo is None else [round(mo[0], 2), round(mo[1], 2)],
               'phase': self.phase}
        if on is False and (self.obstacle_decision.get('action') != 'advisory'
                            or self.obstacle_decision.get('class') != dec['class']):
            how = {'swept': 'predicted motion v %.2f w %.2f' % tuple(mo) if mo else 'motion',
                   'line': 'line to the target'}.get(dec['judged_by'], 'path ahead')
            self._log('info', '%s %s at %.1f m / %.0f deg is off the path corridor (%s): '
                              'advisory only' % (kind, dec['class'] or 'obstacle',
                                                 i.obstacle_distance, i.obstacle_bearing, how))
        self.obstacle_decision = dec

    def _ranged_static(self):
        """A fresh 'static' policy that may explain an abort: it must carry a range
        (obstacle_guard only emits ranged statics; an unranged one is accepted only at the
        'sensitive' level, where the owner can opt in to unranged statics in the guard)."""
        if self._policy() != 'static':
            return False
        return self.inputs.obstacle_distance is not None or self.obstacle_level() == 'sensitive'

    def _static_recent(self):
        return self._ranged_static() or \
            self._now - self._last_static_t <= self.p.obstacle_static_memory_s

    def _collision_recent(self):
        c = self.inputs.collision_stamp
        return c is not None and self._now - c <= self.p.collision_abort_window_s

    # ---- path blocked (costmap collision / obstacle policy): wait, no retry ----
    def _blocked_cause(self):
        return self._collision_recent() or self._policy() in ('dynamic', 'static')

    def _blocked_limit(self, b):
        return float(self.p.blocked_wait_home_s if b['phase'] == 'RETURNING_HOME'
                     else self.p.blocked_wait_s)

    def _blocked(self, ctx, why):
        """An abort explained by an obstacle: hold (blade off, no goal) instead of
        consuming a retry.  False once this blocked episode has used up its wait."""
        b, now, m = self._blk, self._now, self.mission
        if b is None or b['ctx'] != ctx:
            b = {'ctx': ctx, 'since': now, 'n': 0, 'phase': self.phase,
                 'purpose': self._dock_purpose, 'sub': self.sub_state}
            if self._blocked_limit(b) <= 0:
                return False
            self._blk = b
            self._log('warn', 'path blocked (%s): waiting up to %.0f s, re-sending every %.0f s '
                              'once no collision is reported (no retry used)'
                      % (why, self._blocked_limit(b), self.p.blocked_resend_s))
        elif now - b['since'] >= self._blocked_limit(b):
            self._blk = None
            self._log('warn', 'path still blocked after %.0f s (%s)'
                      % (self._blocked_limit(b), why))
            return False
        b['n'] += 1
        b['sent_at'] = now
        b['waiting'] = True
        b['why'] = why
        if m is not None and ctx in ('transit', 'follow'):
            m.step = 'blocked_wait'
            self._soft_blade_off('path blocked')
        else:
            self._blade_off('path blocked')
        self._go(self.phase, 'path blocked: waiting (%d s)' % int(now - b['since']))
        return True

    def _blocked_tick(self):
        """Returns True when the blocked wait owns this tick."""
        b, now, m = self._blk, self._now, self.mission
        if b is None or not b.get('waiting'):
            return False
        if b['ctx'] == 'dock':
            valid = self.phase == b['phase'] and self._action is None
        else:
            valid = m is not None and m.step == 'blocked_wait' and \
                self.phase in ('TRANSIT', 'MOWING')
        if not valid:
            self._blk = None
            return False
        waited = now - b['since']
        if waited >= self._blocked_limit(b):
            self._blk = None
            self._blocked_timeout(b)
            return True
        c = self.inputs.collision_stamp
        clear = (c is None or now - c >= float(self.p.blocked_clear_s)) and \
            self._policy() != 'dynamic'
        if clear and now - b['sent_at'] >= float(self.p.blocked_resend_s):
            b['waiting'] = False
            self._log('info', 'path blocked %.0f s: no collision for %.0f s, re-sending'
                      % (waited, float(self.p.blocked_clear_s)))
            if b['ctx'] == 'dock':
                self._dock(b['phase'], b['purpose'], sub=b['sub'])
            elif b['ctx'] == 'transit':
                m.step = 'transit'
                self._go('TRANSIT', b['sub'])
                self._send_transit()
            else:
                m.step = None
                self._dispatch()
            return True
        sub = 'path blocked: waiting (%d s)' % int(waited)
        if sub != self.sub_state:
            self._go(self.phase, sub)
        return True

    def _blocked_timeout(self, b):
        m = self.mission
        why = 'path blocked for %.0f s (%s)' % (self._blocked_limit(b), b.get('why', ''))
        self._log('warn', why)
        if b['ctx'] == 'dock':
            self._dock_failed(why)
        elif b['ctx'] == 'transit':
            m.step = None
            self._subpath_failed(why, transit=True)
        else:
            m.step = None
            if m.stretch is not None:
                self._stretch_blocked(why)
            elif not self._try_detour(m.start_local, why):
                self._subpath_failed(why)

    def _pivot_track(self, now):
        """Start time of the current in-place pivot command (fresh /cmd_vel_nav with
        |linear| < 0.01 and |angular| > 0.01), else None."""
        i = self.inputs
        cmd = i.nav_cmd
        fresh = cmd is not None and i.nav_cmd_stamp is not None and now - i.nav_cmd_stamp <= 1.0
        if fresh and abs(cmd[0]) < 0.01 and abs(cmd[1]) > 0.01:
            if getattr(self, '_pivot_since', None) is None:
                self._pivot_since = now
        else:
            self._pivot_since = None

    def pivoting_slowly(self):
        since = getattr(self, '_pivot_since', None)
        return (self.phase in ('TRANSIT', 'MOWING') and since is not None
                and self._now - since > self.p.pivot_sub_after_s)

    def _seg_last(self):
        """Last local index of what is being mowed: the sub-path, or a re-queued stretch."""
        m = self.mission
        last = len(m.subpaths[m.sub_i]) - 1
        return last if m.stretch_end is None else min(last, m.stretch_end)

    def _segment_done(self):
        if self.mission.stretch is not None:
            self._stretch_done()
        else:
            self._subpath_done()

    # ---- dynamic obstacles (person / pet): stop, blade off, wait ----------
    def _dyn_context(self):
        m, a = self.mission, self._action
        if self.phase == 'MOWING' and m is not None and m.step in ('spinup', 'spin_pause',
                                                                    'follow', 'corner_pivot'):
            return 'follow'
        if self.phase == 'TRANSIT' and m is not None and m.step == 'transit' and a is not None:
            return 'transit'
        if self.phase in DOCK_PHASES and a is not None and a.purpose in ('dock', 'dock_route',
                                                                         'route_corner'):
            return 'dock'
        return None

    def _dyn_start(self, ctx):
        m = self.mission
        cls = self.inputs.obstacle_class or 'obstacle'
        if m is not None and ctx == 'follow' and m.step == 'follow':
            m.start_local = self._track_progress()
        self._dyn_ignore = None
        # the in-path judgement that started the stop (same kind, same twist) keeps judging
        # the wait: the clear-hold sees the same evaluation the entry did (2026-10-13)
        ev = getattr(self, '_path_eval', None) or {}
        self._dyn = {'ctx': ctx, 'since': self._now, 'clear_since': None, 'cls': cls,
                     'sub': self.sub_state, 'phase': self.phase, 'purpose': self._dock_purpose,
                     'path': ev.get('path'), 'motion': ev.get('motion')}
        if self._action is not None:
            self._fx.append(CancelActions('dynamic obstacle: %s' % cls))
            self._action = None
        self._fx.append(ZeroBurst())
        if m is not None and ctx in ('follow', 'transit'):
            m.step = 'dyn_wait'
            self._soft_blade_off('dynamic obstacle: %s' % cls)
        else:
            self._blade_off('dynamic obstacle: %s' % cls)
        d = self.inputs.obstacle_distance
        self._log('warn', '%s%s ahead: stopped, blade %s, waiting up to %.0f s' % (
            cls, (' at %.1f m' % d) if d is not None else '',
            'kept on (blade_policy continuous)' if self.blade_on else 'off',
            self._dyn_wait_s(ctx)))
        self._go(self.phase, 'waiting for %s to move (0 s)' % cls)

    def _dynamic_tick(self):
        """Returns True when the dynamic-obstacle logic owns this tick."""
        d, kind = self._dyn, self._policy()
        if d is None:
            if kind != 'dynamic':
                return False
            ctx = self._dyn_context()
            if ctx is None or self._dyn_ignored():
                return False
            self._dyn_start(ctx)
            return True
        if kind == 'dynamic':
            d['clear_since'] = None
            d['cls'] = self.inputs.obstacle_class or d['cls']
        elif d['clear_since'] is None:
            d['clear_since'] = self._now
        waited = self._now - d['since']
        if d['clear_since'] is not None and \
                self._now - d['clear_since'] >= self.p.dynamic_clear_hold_s:
            self._dyn = None
            self._log('info', '%s gone for %.1f s: resuming' % (d['cls'],
                                                                 self.p.dynamic_clear_hold_s))
            self._dyn_resume(d)
            return True
        if waited >= self._dyn_wait_s(d['ctx']):
            self._dyn = None
            self._dyn_timeout(d)
            return True
        sub = 'waiting for %s to move (%d s)' % (d['cls'], int(waited))
        if sub != self.sub_state:
            self._go(self.phase, sub)
        return True

    def _dyn_wait_s(self, ctx):
        w = float(self.p.dynamic_wait_s if ctx == 'follow' else self.p.dynamic_wait_transit_s)
        if self.obstacle_level() == 'sensitive':
            w *= max(1.0, float(self.p.dynamic_wait_sensitive_factor))
        return w

    def _dyn_ignored(self):
        """True while a fresh dynamic detection falls in the post-timeout ignore
        window (same class, robot moved <= dynamic_ignore_move_m). Logs once."""
        g = self._dyn_ignore
        if g is None:
            return False
        cls = self.inputs.obstacle_class or 'obstacle'
        pose = self.inputs.pose
        moved = 0.0
        if pose is not None and g['pose'] is not None:
            moved = math.hypot(pose[0] - g['pose'][0], pose[1] - g['pose'][1])
        if self._now >= g['until'] or cls != g['cls'] or \
                moved > float(self.p.dynamic_ignore_move_m):
            self._dyn_ignore = None
            return False
        if not g['logged']:
            g['logged'] = True
            self._log('info', 'ignoring %s detections for %.0f s more (wait already timed out; '
                              'Nav2 / costmap handle it) unless the robot moves > %.1f m'
                      % (cls, g['until'] - self._now, float(self.p.dynamic_ignore_move_m)))
        return True

    def _dyn_resume(self, d):
        m = self.mission
        if d['ctx'] == 'dock':
            self._dock(d['phase'], d['purpose'], sub=d['sub'])
        elif m is None:
            return
        elif d['ctx'] == 'transit':
            m.step = 'transit'
            self._go(self.phase, d['sub'])
            self._send_transit()
        else:
            m.step = None
            self._dispatch()

    def _dyn_timeout(self, d):
        m = self.mission
        why = '%s did not move within %.0f s' % (d['cls'], self._dyn_wait_s(d['ctx']))
        pose = self.inputs.pose
        self._dyn_ignore = {'until': self._now + float(self.p.dynamic_ignore_after_timeout_s),
                            'cls': d['cls'], 'logged': False,
                            'pose': (pose[0], pose[1]) if pose is not None else None}
        if d['ctx'] != 'follow' or m is None:
            # transits / docking: Nav2 plans around the marked obstacle
            self._log('warn', '%s: continuing (Nav2 plans around it)' % why)
            self._dyn_resume(d)
            return
        self._log('warn', '%s: treating it as a static obstacle' % why)
        m.step = None
        if m.stretch is not None:
            self._stretch_blocked(why)
        elif not self._try_detour(m.start_local, why):
            self._subpath_failed(why)

    # ---- localization hold (2026-10-09): drift budget spent -> stop, blade off ----
    def _loc_fresh(self):
        i = self.inputs
        return i.loc_stamp is not None and \
            self._now - i.loc_stamp <= float(self.p.loc_status_timeout_s)

    def _loc_hold_tick(self):
        """Returns True while the localization hold owns this tick. Reuses the dynamic-
        obstacle resume path (_dyn_resume), so a held follow re-dispatches from its
        progress and a held transit re-sends its goal."""
        m, i, d = self.mission, self.inputs, self._loc
        if d is not None and (m is None or self.phase != d['phase'] or m.step != 'loc_hold'):
            self._loc = None                 # another guard / command took over
            return False
        if d is None:
            if not self.p.loc_hold or not self._loc_fresh() or i.loc_state in ('', 'ok'):
                return False
            ctx = self._dyn_context()
            if ctx not in ('follow', 'transit'):
                return False
            if ctx == 'follow' and m.step == 'follow':
                m.start_local = self._track_progress()
            self._loc = {'ctx': ctx, 'since': self._now, 'ok_since': None, 'sub': self.sub_state,
                         'phase': self.phase, 'purpose': self._dock_purpose}
            if self._action is not None:
                self._fx.append(CancelActions('localization %s' % i.loc_state))
                self._action = None
            self._fx.append(ZeroBurst())
            m.step = 'loc_hold'
            self._blade_off('localization %s' % i.loc_state)
            self._log('warn', 'localization %s (%s): stopped, blade off, waiting for RTK fixed'
                      % (i.loc_state, i.loc_reason or 'no reason'))
            self._go(self.phase, 'localization %s: holding' % i.loc_state)
            return True
        if self._loc_fresh() and i.loc_state == 'ok':
            if d['ok_since'] is None:
                d['ok_since'] = self._now
            if self._now - d['ok_since'] >= float(self.p.loc_resume_s):
                self._loc = None
                self._log('info', 'localization ok again after %.0f s: resuming'
                          % (self._now - d['since']))
                self._dyn_resume(d)
                return True
        else:
            d['ok_since'] = None
        held = self._now - d['since']
        if held >= float(self.p.loc_hold_max_s):
            self._loc = None
            why = 'localization not ok for %.0f s (%s)' % (held, i.loc_reason or i.loc_state)
            self._log('error', '%s: stopping in place' % why)
            self._interrupt_mission(why)
            self._cancel_all(why)
            self._blade_off(why)
            self._fx.append(ZeroBurst())
            self._dock_purpose = None
            self._go_idle(why)
            return True
        sub = 'localization %s: holding (%d s)' % (i.loc_state or 'unknown', int(held))
        if sub != self.sub_state:
            self._go(self.phase, sub)
        return True

    # ---- static obstacles: blade-off detour to the swath beyond ----------
    def _try_detour(self, cur, why):
        m, p = self.mission, self.p
        if not p.obstacle_avoidance or m.stretch is not None:
            return False
        if m.detours >= p.max_detours_per_subpath:
            self._log('warn', 'area %d sub-path %d: %d detours used (max_detours_per_subpath): '
                              'no detour' % (m.area_idx, m.sub_i, m.detours))
            return False
        return self._detour_send(cur, float(p.detour_skip_m), why)

    def _detour_send(self, cur, skip, why):
        m, p = self.mission, self.p
        sp = m.subpaths[m.sub_i]
        last = self._seg_last()
        idx = geo.advance_index(sp, cur, skip)
        if idx >= last:
            # obstacle on the sub-path end: skip the rest, re-queued at the area end
            m.detours += 1
            self._blade_off('obstacle at the sub-path end')
            self._log('warn', 'area %d sub-path %d: %s within %.1f m of the end: skipping '
                              'poses %d -> %d' % (m.area_idx, m.sub_i, why, skip, cur, last))
            self._record_skip(cur, last)
            m.step = None
            self._subpath_done()
            return True
        if not geo.inside_area(sp[idx], m.areas.get(m.area_idx, {}), p.detour_max_leave_m):
            self._log('warn', 'area %d sub-path %d: detour pose %d (%.2f, %.2f) outside the '
                              'area: no detour' % (m.area_idx, m.sub_i, idx, sp[idx][0],
                                                   sp[idx][1]))
            return False
        # Blade always OFF on a detour (review 2026-10-09): with detour_unclassified_after 0 the
        # first "collision ahead" goes straight here, and the obstacle may be a child, pet or
        # hose that YOLO did not classify; the Nav2 path (not the straight segment
        # _transit_blade checks) passes within 0.25-0.6 m of it.
        self._transit_blade(sp[idx], 'detour', force_off=True)
        if not m.detour:
            self._incident('detour', why)
        m.detour = {'from': cur, 'to': idx, 'skip': skip, 'why': why}
        m.start_local = idx
        m.step = 'transit'
        m.transit_target = (sp[idx][0], sp[idx][1], geo.heading_of(sp, idx))
        self._log('warn', 'area %d sub-path %d: %s: blade-%s detour %d/%d to pose %d '
                          '(%.1f m ahead)' % (m.area_idx, m.sub_i, why,
                                              'on' if m.blade_transit else 'off', m.detours + 1,
                                              p.max_detours_per_subpath, idx, skip))
        # costmaps are NOT cleared: the global planner must see the obstacle marks
        self._go('TRANSIT', 'detour around obstacle: area %d sub-path %d/%d' % (
            m.area_idx, m.sub_i + 1, len(m.subpaths)))
        self._start_action(ACT_NAV, {'pose': m.transit_target}, p.transit_timeout_s)
        return True

    def _record_skip(self, lfrom, lto):
        m = self.mission
        offs, _ = subpath_offsets(m.subpaths)
        ac = self.cursor.area(m.area_idx)
        ac.skipped.append((offs[m.sub_i] + int(lfrom), offs[m.sub_i] + int(lto)))
        self._persist()

    def _detour_done(self):
        m = self.mission
        d, m.detour = m.detour, None
        m.detours += 1
        m.transit_fails = m.follow_fails = 0
        m.route_plan_fails = 0
        m.recovering = False
        self._record_skip(d['from'], d['to'])
        self._log('info', 'detour done: sub-path %d poses %d -> %d skipped (re-tried at the '
                          'area end)' % (m.sub_i, d['from'], d['to']))
        self._begin_blade()

    def _detour_failed(self, outcome):
        m, p = self.mission, self.p
        d, m.detour = m.detour, None
        skip = d['skip'] + float(p.detour_skip_step_m)
        if p.detour_skip_step_m > 0 and skip <= float(p.detour_max_skip_m) + 1e-6:
            self._log('warn', 'detour transit %s: extending the skip to %.1f m'
                      % (outcome, skip))
            if self._detour_send(d['from'], skip, d['why']):
                return
        m.start_local = d['from']
        m.follow_fails += 1
        if m.follow_fails > p.follow_retries:
            self._subpath_failed('obstacle could not be bypassed (%s)' % d['why'])
            return
        self._schedule_retry('MOWING', 'area %d sub-path %d/%d: detour failed (%s), retry %d/%d '
                             'in %.0f s' % (m.area_idx, m.sub_i + 1, len(m.subpaths), outcome,
                                            m.follow_fails, p.follow_retries,
                                            p.transit_retry_delay_s))

    # ---- end-of-area pass over the skipped stretches ----------------------
    def _next_stretch(self):
        m = self.mission
        ac = self.cursor.area(m.area_idx)
        fr, to = m.requeue.pop(0)
        a = absolute_to_local(m.subpaths, fr)
        b = absolute_to_local(m.subpaths, to)
        if a is None or b is None or a[0] != b[0] or b[1] <= a[1]:
            if (fr, to) in ac.skipped:
                ac.skipped.remove((fr, to))
                self._persist()
            self._area_done()
            return
        m.stretch, m.stretch_end = (fr, to), b[1]
        m.sub_i, m.start_local = a
        m.retry_used, m.premature, m.boundary_recoveries = False, 0, 0
        m.transit_fails = m.follow_fails = m.recovery_fails = 0
        m.route_plan_fails = 0
        m.recovering, m.detour = False, None
        self._log('info', 'area %d: re-trying skipped stretch sub-path %d poses %d -> %d'
                  % (m.area_idx, a[0], a[1], b[1]))
        self._dispatch()

    def _next_failed_retry(self):
        """End-of-area pass: re-try one sub-path dropped on transit failure, as a
        stretch from its first missing pose (success marks it completed)."""
        m = self.mission
        e = m.requeue_subs.pop(0)
        if e in m.failed:
            m.failed.remove(e)
        sub = e[1]
        offs, _ = subpath_offsets(m.subpaths)
        last = len(m.subpaths[sub]) - 1
        start = max(0, min(e[3] - offs[sub], last)) if e[3] >= 0 else 0
        m.retry_entry = e
        m.stretch, m.stretch_end = (offs[sub] + start, offs[sub] + last), last
        m.sub_i, m.start_local = sub, start
        m.retry_used, m.premature, m.boundary_recoveries = False, 0, 0
        m.transit_fails = m.follow_fails = m.recovery_fails = 0
        m.route_plan_fails = 0
        m.recovering, m.detour = False, None
        self._blk = None
        self._log('info', 'area %d: end-of-area retry of sub-path %d (%s)'
                  % (m.area_idx, sub, e[2]))
        self._dispatch()

    def _stretch_done(self):
        m = self.mission
        ac = self.cursor.area(m.area_idx)
        if m.retry_entry is not None:
            ac.completed.add(m.retry_entry[1])
            m.completed_count += 1
            self._log('info', 'area %d: sub-path %d mowed on the end-of-area retry'
                      % (m.area_idx, m.retry_entry[1]))
            m.retry_entry = None
        elif m.stretch in ac.skipped:
            ac.skipped.remove(m.stretch)
            self._log('info', 'area %d: skipped stretch %s mowed' % (m.area_idx, m.stretch))
        m.stretch, m.stretch_end, m.step = None, None, None
        self._persist()
        self._area_done()

    def _stretch_blocked(self, why):
        m = self.mission
        if m.retry_entry is not None:
            e, m.retry_entry = m.retry_entry, None
            m.failed.append(e)               # NOT mowed after all (original reason kept)
            self._log('warn', 'area %d sub-path %d still NOT mowed on the end-of-area retry '
                      '(%s)' % (m.area_idx, e[1], why))
            m.stretch, m.stretch_end, m.step = None, None, None
            self._blade_off('end-of-area retry failed')
            self._area_done()
            return
        m.blocked.append((m.area_idx,) + tuple(m.stretch))
        self._log('warn', 'area %d: stretch %s still blocked (%s): obstacle cannot be bypassed'
                  % (m.area_idx, m.stretch, why))
        m.stretch, m.stretch_end, m.step = None, None, None
        self._blade_off('stretch blocked')
        self._area_done()

    def _handle_service(self, s, ok, resp):
        if isinstance(s.purpose, str) and s.purpose.startswith('preview_'):
            self._preview_on_service(s.purpose, ok, resp)
            return
        if s.purpose == 'route':
            self._route_planned(ok, resp)
            return
        if s.purpose == 'dock_route':
            self._dock_route_planned(ok, resp)
            return
        if s.purpose == 'enum':
            e = self._enum
            if e is None:
                return
            if ok and resp.get('success'):
                area = resp.get('area', {})
                e['count'] += 1
                e.setdefault('all', {})[e['index']] = area
                if not area.get('is_navigation_area', False):
                    e['areas'][e['index']] = area
                e['index'] += 1
                if e['index'] < self.p.max_areas:
                    self._call(SRV_GET_AREA, {'index': e['index']}, purpose='enum')
                    return
            self._enum = None
            if e.get('all'):
                self._region_areas = dict(e['all'])     # path conditioning's region
            if e['purpose'] == 'start':
                if self.mission is None:
                    return
                if not ok and e['index'] == 0:
                    self._abort_start('/map_server_node/get_mowing_area unavailable')
                    return
                self._areas_loaded(e['areas'], e['count'])
            else:
                if self._record_poly is not None:
                    self._record_save(len(e['areas']), e['count'], e['areas'])
        elif s.purpose == 'record_obstacle':
            if self._record_poly is not None:
                added = ok and bool(resp.get('success', False))
                if not added:
                    self._log('warn', 'promote_obstacle refused: %s'
                              % (resp.get('message') or 'unavailable'))
                self._record_saved(added, failed_service='promote_obstacle')
        elif s.purpose == 'add_area':
            if self._record_poly is not None:
                added = ok and bool(resp.get('success', False))
                if added and self._record_kind == 'path' and self._record_line:
                    self._call(SRV_SET_CHANNEL, {
                        'index': self._record_index,
                        'settings_json': json.dumps({
                            'points': [[round(x, 3), round(y, 3)] for x, y in self._record_line],
                            'width_m': float(self.p.record_path_width_m)})},
                        purpose='set_channel')
                    return
                self._record_saved(added)
        elif s.purpose == 'set_channel':
            if self._record_poly is not None:
                if not (ok and resp.get('success', False)):
                    # the band polygon is saved; only the centreline metadata is missing
                    self._log('warn', 'set_area_channel failed for "%s": %s' % (
                        self._record_name, resp.get('message', 'unavailable')))
                self._record_saved(True)
        elif s.purpose in ('area_settings', 'coverage_params'):
            m = self.mission
            if m is None or self.phase != 'PLANNING' or m.area_idx is None:
                return
            if s.purpose == 'area_settings':
                self._on_area_settings(ok, resp)
                return
            if not ok:
                self._log('warn', 'area %d: /coverage_server set_parameters failed (%s): planning '
                                  'with its current parameters'
                          % (m.area_idx, resp.get('message') or 'unavailable'))
            self._start_plan()

    # ==================================================================
    # stuck guard (mower_control slip_detector /stuck -> escape / STUCK_NEEDS_HELP)
    # ==================================================================
    def _stuck_steps(self):
        v = abs(float(self.p.stuck_step_speed))
        w = v * math.radians(float(self.p.stuck_arc_deg)) / max(0.05, float(self.p.stuck_step_m))
        return [(-v, 0.0, 'reverse'), (v, w, 'forward arc'), (-v, -w, 'reverse opposite arc')]

    def _stuck_operator(self, cmd):
        """Operator STOP / START / MANUAL / HOME / UNDOCK: clear STUCK_NEEDS_HELP's
        history and the slip_detector latch so the next run starts clean."""
        if self.phase == 'STUCK_NEEDS_HELP' and cmd in (CMD_STOP, CMD_START, CMD_MANUAL_MOW,
                                                         CMD_HOME, CMD_UNDOCK,
                                                         CMD_RESET_EMERGENCY):
            self._log('info', 'stuck: cleared by the operator (command %d)' % cmd)
            self._stuck_hist = []
            self._tilt_hist = []
            self._hb_count = 0
            self._stuck_ignore_until = self._now + float(self.p.stuck_ignore_after_s)
            self._fx.append(ResetStuckGuard())
            if cmd != CMD_STOP and cmd != CMD_RESET_EMERGENCY:
                self._go_idle('stuck cleared')

    def _stuck_watch(self):
        """Abort an escape when something else took over (emergency, STOP, phase change)."""
        k = self._stk
        if k is None:
            return
        if self._emergency_cause() or self.phase != k['phase']:
            self._log('warn', 'stuck escape aborted (%s)' % (
                self._emergency_cause() or 'state %s' % self.phase))
            self._stk = None
            self._fx.append(SuppressStuckGuard(False))
            self._fx.append(DriveCmd(0.0, 0.0))

    def _stuck_tick(self):
        """Returns True while the stuck guard owns the tick."""
        if self._stk is not None:
            self._stuck_escape_tick()
            return True
        i = self.inputs
        if i.pose is not None:              # how far the robot got from each escaped spot
            for h in self._stuck_hist:
                h[3] = max(h[3], math.hypot(h[1] - i.pose[0], h[2] - i.pose[1]))
        if not (self.p.stuck_guard and i.stuck and self.phase in STUCK_PHASES) \
                or self._now < self._stuck_ignore_until:
            return False
        if self.phase in ('TRANSIT', 'MOWING') and self.mission is None:
            return False
        pose = i.pose
        if pose is None:
            self._stuck_needs_help('stuck, no pose', (0.0, 0.0))
            return True
        horizon = self._now - float(self.p.stuck_recovery_period_s)
        self._stuck_hist = [h for h in self._stuck_hist if h[0] >= horizon]
        same = [h for h in self._stuck_hist
                if math.hypot(h[1] - pose[0], h[2] - pose[1]) <= self.p.stuck_same_spot_m
                and h[3] < float(self.p.stuck_same_spot_left_m)]
        if same:
            self._stuck_needs_help('stuck again at the same spot', pose)
            return True
        if len(self._stuck_hist) >= int(self.p.stuck_max_recoveries):
            self._stuck_needs_help('stuck %d times within %.0f s' % (
                len(self._stuck_hist) + 1, self.p.stuck_recovery_period_s), pose)
            return True
        self._log('error', 'STUCK at (%.2f, %.2f) in %s: cancel, blade off, escape %d/%d'
                  % (pose[0], pose[1], self.phase, len(self._stuck_hist) + 1,
                     int(self.p.stuck_max_recoveries)))
        m = self.mission
        self._stk = {'phase': self.phase, 'origin': (float(pose[0]), float(pose[1])),
                     'stage': 'pause', 't': self._now, 'step': -1,
                     'dock_purpose': self._dock_purpose}
        self._cancel_all('stuck')
        self._blade_off('stuck')
        self._fx.append(ZeroBurst())
        self._fx.append(SuppressStuckGuard(True))
        if m is not None:
            if m.step == 'follow':
                m.start_local = self._track_progress()
            m.step = None
        self.sub_state = 'stuck: stopping'
        return True

    def _stuck_escape_tick(self):
        k, pose, now = self._stk, self.inputs.pose, self._now
        if pose is None:
            self._stk = None
            self._fx.append(SuppressStuckGuard(False))
            self._stuck_needs_help('stuck, pose lost during the escape', k['origin'])
            return
        if k['stage'] == 'pause':
            if now - k['t'] < self.p.stuck_pause_s:
                return
            self._stuck_next_step(pose)
            return
        v, w, name = self._stuck_steps()[k['step']]
        d = math.hypot(pose[0] - k['start'][0], pose[1] - k['start'][1])
        if d >= k['best'] + self.p.stuck_step_progress_m:
            k['best'], k['best_t'] = d, now
        done = d >= self.p.stuck_step_m
        stalled = now - k['best_t'] > self.p.stuck_step_progress_s
        if not done and not stalled:
            self._fx.append(DriveCmd(v, w))
            return
        self._fx.append(DriveCmd(0.0, 0.0))
        moved = math.hypot(pose[0] - k['origin'][0], pose[1] - k['origin'][1])
        self._log('info' if done else 'warn', 'stuck escape step %d (%s): %s, %.2f m from the '
                  'stuck spot' % (k['step'] + 1, name, 'done' if done else
                                  'no progress in %.0f s' % self.p.stuck_step_progress_s, moved))
        if done or moved > self.p.stuck_escape_m:
            self._stuck_escaped(moved)
        elif k['step'] + 1 < len(self._stuck_steps()):
            self._stuck_next_step(pose)
        else:
            self._stk = None
            self._fx.append(SuppressStuckGuard(False))
            self._stuck_needs_help('escape failed', k['origin'])

    def _stuck_next_step(self, pose):
        k = self._stk
        k['stage'] = 'drive'
        k['step'] += 1
        k['start'] = (float(pose[0]), float(pose[1]))
        k['best'], k['best_t'] = 0.0, self._now
        v, w, name = self._stuck_steps()[k['step']]
        self.sub_state = 'stuck: escape %d/3 (%s)' % (k['step'] + 1, name)
        self._log('info', 'stuck escape step %d: %s %.2f m at %.2f m/s'
                  % (k['step'] + 1, name, self.p.stuck_step_m, abs(v)))
        self._fx.append(DriveCmd(v, w))

    def _stuck_escaped(self, moved):
        k, self._stk = self._stk, None
        ox, oy = k['origin']
        self._stuck_hist.append([self._now, ox, oy, float(moved)])
        self._stuck_ignore_until = self._now + float(self.p.stuck_ignore_after_s)
        self._fx.append(SuppressStuckGuard(False))
        self._fx.append(RecordIncident('stuck', ox, oy, 'stuck guard escape (%s, %.2f m)'
                                       % (k['phase'], moved)))
        self._call(SRV_CLEAR_COSTMAPS, {'global': True}, track=False)
        self._log('info', 'stuck: escaped (%.2f m): costmaps cleared, terrain incident at '
                  '(%.2f, %.2f), resuming %s' % (moved, ox, oy, k['phase']))
        if k['phase'] in DOCK_PHASES:
            self._dock(k['phase'], k['dock_purpose'], 'resuming after stuck escape')
            return
        m = self.mission
        if m is None or m.area_idx is None or not m.subpaths:
            self._go_idle('stuck escape: nothing to resume')
            return
        m.detour = None
        m.start_local = self._track_progress()
        self._dispatch()

    def _stuck_needs_help(self, why, pose):
        x, y = float(pose[0]), float(pose[1])
        self._log('error', 'STUCK_NEEDS_HELP: %s at (%.2f, %.2f): motion stopped, blade off '
                  '(STOP / START / MANUAL to clear)' % (why, x, y))
        if self.mission is not None:
            self._interrupt_mission('stuck')
        self._cancel_all('stuck')
        self._blade_off('stuck')
        self._fx.append(ZeroBurst())
        self._dock_purpose = None
        self._resume_after = None
        self._stk = None
        self._go('STUCK_NEEDS_HELP', STUCK_NEEDS_HELP_SUB % (x, y))

    # ==================================================================
    # tilt guard (mower_control tilt_monitor /tilt/status), 2026-10-09
    # ==================================================================
    # The monitor owns filtering, debounce and hysteresis; this only acts on its band.
    # critical -> _emergency_cause; caution -> controller speed cap; limit -> back out
    # along the track just driven (pure pursuit on /cmd_vel_emergency, either direction),
    # 'steep' terrain incident, then resume like the stuck escape (mowing: detour past it).
    def _tilt_band(self):
        """The monitor's band, or 'unknown' when the guard is off or the status is stale."""
        i = self.inputs
        if not self.p.tilt_guard or i.tilt_stamp is None \
                or self._now - i.tilt_stamp > float(self.p.tilt_status_timeout_s):
            return 'unknown'
        return i.tilt_band

    def _tilt_text(self):
        i = self.inputs
        r, p = i.tilt_roll_deg, i.tilt_pitch_deg
        if i.tilt_axis == 'pitch' and p is not None:
            return 'pitch %.0f deg' % p
        if r is not None:
            return 'roll %.0f deg' % r
        return 'tilt'

    def _tilt_watch(self):
        """Abort a back-out when something else took over (emergency, STOP, phase change)."""
        k = self._tob
        if k is None:
            return
        if self._emergency_cause() or self.phase != k['phase']:
            self._log('warn', 'steep-slope back-out aborted (%s)' % (
                self._emergency_cause() or 'state %s' % self.phase))
            self._tob = None
            self._fx.append(SuppressStuckGuard(False))
            self._fx.append(DriveCmd(0.0, 0.0))

    def _tilt_trail_add(self):
        pose = self.inputs.pose
        if pose is None or self._tob is not None:
            return
        tr = self._tilt_trail
        if tr and math.hypot(pose[0] - tr[-1][0], pose[1] - tr[-1][1]) \
                < float(self.p.tilt_trail_step_m):
            return
        tr.append((float(pose[0]), float(pose[1])))
        n = int(float(self.p.tilt_trail_m) / max(0.01, float(self.p.tilt_trail_step_m))) + 1
        del tr[:-n]

    def _tilt_speed(self, band):
        """Caution cap on the coverage + transit controllers; restored when back to ok.
        'unknown' keeps the current state (no flapping on a lost status)."""
        if band == 'unknown':
            return
        want = band in ('caution', 'limit', 'critical')
        if want == self._tilt_capped:
            return
        self._tilt_capped = want
        cap = float(self.p.tilt_caution_speed_mps)
        m = self.mission
        params = {}
        if want:
            params = dict(self.coverage_speed_params(cap))
            params[self.p.tilt_transit_speed_param] = cap
            self._log('warn', 'slope caution (%s): controller speed capped at %.2f m/s'
                      % (self._tilt_text(), cap))
        else:
            params = {self.p.tilt_transit_speed_param: float(self.p.tilt_transit_speed_mps)}
            if m is not None and m.settings:
                params.update(self.coverage_speed_params(min(
                    float(m.settings.get('cut_speed_mps', AREA_SETTINGS_DEFAULTS['cut_speed_mps'])),
                    float(self.p.cut_speed_max_mps))))
            self._log('info', 'slope ok: controller speed restored')
        if self.p.set_cut_speed:
            self._call(SRV_SET_PARAMS, {'node': PARAM_NODE_CONTROLLER, 'params': params},
                       track=False)

    def _tilt_tick(self):
        """Returns True while the tilt guard owns the tick."""
        if not self.p.tilt_guard:
            return False
        self._tilt_trail_add()
        band = self._tilt_band()
        self._tilt_speed(band)
        if self._tob is not None:
            self._tilt_backout_tick(band)
            return True
        if band != 'limit':
            return False
        if self.phase == 'MANUAL_MOWING':
            if self.blade_on:               # the operator drives; only the blade is ours
                self._log('warn', 'manual: blade off (steep slope, %s)' % self._tilt_text())
                self._blade_off('manual: steep slope')
                self._go('MANUAL_MOWING', SUB_MANUAL_BLADE_OFF)
            return False
        if self.phase not in STUCK_PHASES:
            return False
        if self.phase in ('TRANSIT', 'MOWING') and self.mission is None:
            return False
        pose = self.inputs.pose
        if pose is None:
            self._tilt_needs_help('steep slope, no pose', (0.0, 0.0))
            return True
        horizon = self._now - float(self.p.tilt_backout_period_s)
        self._tilt_hist = [h for h in self._tilt_hist if h[0] >= horizon]
        if any(math.hypot(h[1] - pose[0], h[2] - pose[1]) <= float(self.p.tilt_same_spot_m)
               for h in self._tilt_hist):
            self._tilt_needs_help('steep again at the same spot', pose)
            return True
        if len(self._tilt_hist) >= int(self.p.tilt_max_backouts):
            self._tilt_needs_help('steep %d times within %.0f s' % (
                len(self._tilt_hist) + 1, self.p.tilt_backout_period_s), pose)
            return True
        ox, oy = float(pose[0]), float(pose[1])
        self._log('error', 'STEEP SLOPE (%s) at (%.2f, %.2f) in %s: cancel, blade off, '
                  'back out' % (self._tilt_text(), ox, oy, self.phase))
        m = self.mission
        follow = m is not None and m.step == 'follow'
        # back-out path: the driven track, newest first, without the spot itself
        path = [q for q in reversed(self._tilt_trail)]
        self._tob = {'phase': self.phase, 'origin': (ox, oy), 'stage': 'pause',
                     't': self._now, 'path': path, 'i': 0, 'follow': follow,
                     'dock_purpose': self._dock_purpose, 'best': 0.0, 'best_t': self._now}
        self._cancel_all('steep slope')
        self._blade_off('steep slope')
        self._fx.append(ZeroBurst())
        self._fx.append(SuppressStuckGuard(True))
        self._fx.append(RecordIncident('steep', ox, oy, '%s (limit) in %s'
                                       % (self._tilt_text(), self.phase)))
        if m is not None:
            if follow:
                m.start_local = self._track_progress()
            m.step = None
        self.sub_state = 'steep slope (%s): stopping' % self._tilt_text()
        return True

    def _tilt_backout_cmd(self, pose):
        """(v, w) toward the next back-out path point: pure pursuit, forward or reverse
        (whichever side the track lies on), straight reverse once the track is used up."""
        k, p = self._tob, self.p
        v0, la = abs(float(p.tilt_backout_speed)), float(p.tilt_backout_lookahead_m)
        path = k['path']
        while k['i'] < len(path) and math.hypot(path[k['i']][0] - pose[0],
                                                path[k['i']][1] - pose[1]) < la:
            k['i'] += 1
        if k['i'] >= len(path):
            return (k.get('v', -v0), 0.0)
        tx, ty = path[k['i']]
        dx, dy = tx - pose[0], ty - pose[1]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        xr, yr = c * dx + s * dy, -s * dx + c * dy
        v = v0 if xr > 0 else -v0
        k['v'] = v
        wmax = float(p.tilt_backout_max_w)
        w = max(-wmax, min(wmax, v * 2.0 * yr / max(1e-6, xr * xr + yr * yr)))
        return v, w

    def _tilt_backout_tick(self, band):
        k, pose, now, p = self._tob, self.inputs.pose, self._now, self.p
        if pose is None:
            self._tob = None
            self._fx.append(SuppressStuckGuard(False))
            self._tilt_needs_help('steep slope, pose lost during the back-out', k['origin'])
            return
        if k['stage'] == 'pause':
            if now - k['t'] < float(p.tilt_backout_pause_s):
                return
            k['stage'], k['best_t'] = 'drive', now
            self.sub_state = 'steep slope (%s): backing out' % self._tilt_text()
            self._log('info', 'steep slope: backing out along %d track points at %.2f m/s'
                      % (len(k['path']), abs(float(p.tilt_backout_speed))))
        d = math.hypot(pose[0] - k['origin'][0], pose[1] - k['origin'][1])
        if d >= k['best'] + float(p.tilt_backout_progress_m):
            k['best'], k['best_t'] = d, now
        if band not in ('limit', 'critical') and d >= float(p.tilt_backout_min_m):
            self._tilt_backed_out(d)
            return
        if d >= float(p.tilt_backout_max_m) or now - k['best_t'] > float(p.tilt_backout_progress_s):
            why = ('still steep after backing out %.2f m' % d if d >= float(p.tilt_backout_max_m)
                   else 'back-out made no progress in %.0f s' % float(p.tilt_backout_progress_s))
            self._fx.append(DriveCmd(0.0, 0.0))
            self._tob = None
            self._fx.append(SuppressStuckGuard(False))
            self._tilt_needs_help(why, k['origin'])
            return
        self._fx.append(DriveCmd(*self._tilt_backout_cmd(pose)))

    def _tilt_backed_out(self, moved):
        k, self._tob = self._tob, None
        ox, oy = k['origin']
        self._tilt_hist.append([self._now, ox, oy])
        self._tilt_trail = []
        self._fx.append(DriveCmd(0.0, 0.0))
        self._fx.append(SuppressStuckGuard(False))
        self._log('info', 'steep slope: backed out %.2f m from (%.2f, %.2f), resuming %s'
                  % (moved, ox, oy, k['phase']))
        if k['phase'] in DOCK_PHASES:
            self._dock(k['phase'], k['dock_purpose'], 'resuming after steep-slope back-out')
            return
        m = self.mission
        if m is None or m.area_idx is None or not m.subpaths:
            self._go_idle('steep-slope back-out: nothing to resume')
            return
        if k['phase'] == 'MOWING' and k['follow']:
            # skip past the spot along the swath (blade-off detour); the 'steep' incident
            # raises the terrain cost so Nav2's detour / transit plans route around it
            if not self._try_detour(m.start_local, 'steep slope'):
                self._subpath_failed('steep slope')
            return
        m.detour = None
        self._dispatch()

    def _tilt_needs_help(self, why, pose):
        """Like the stuck guard's give-up (same latched phase, same operator clear)."""
        x, y = float(pose[0]), float(pose[1])
        self._log('error', 'STUCK_NEEDS_HELP: %s at (%.2f, %.2f): motion stopped, blade off '
                  '(STOP / START / MANUAL to clear)' % (why, x, y))
        if self.mission is not None:
            self._interrupt_mission('steep slope')
        self._cancel_all('steep slope')
        self._blade_off('steep slope')
        self._fx.append(ZeroBurst())
        self._dock_purpose = None
        self._resume_after = None
        self._tob = None
        self._go('STUCK_NEEDS_HELP', 'steep slope: needs help at (%.2f, %.2f)' % (x, y))


    # ==================================================================
    # hard boundary (task 0.4 a, 2026-10-14 run 3)
    # ==================================================================
    # map_server publishes ~/outside_m: the robot centre's distance outside the mowing
    # areas U drawn paths U dock corridor (0 inside; the return corridor does NOT count).
    # Beyond hard_boundary_m in an autonomous driving phase: stop, blade off, reverse along
    # the driven track until back inside, count it, resume or give up (see Params).
    def _outside(self):
        """The fresh ~/outside_m, else None (no check without a recent value)."""
        i = self.inputs
        if i.outside_m is None or i.outside_stamp is None or \
                self._now - i.outside_stamp > float(self.p.hard_boundary_stale_s):
            return None
        return float(i.outside_m)

    def _hard_boundary_watch(self):
        """Abort a retrace when something else took over (emergency, STOP, phase change)."""
        k = self._hb
        if k is None:
            return
        if self._emergency_cause() or self.phase != k['phase']:
            self._log('warn', 'boundary retrace aborted (%s)' % (
                self._emergency_cause() or 'state %s' % self.phase))
            self._hb = None
            self._fx.append(SuppressStuckGuard(False))
            self._fx.append(DriveCmd(0.0, 0.0))

    def _hb_trail_add(self):
        pose = self.inputs.pose
        if pose is None:
            return
        tr, step = self._hb_trail, max(0.01, float(self.p.hard_boundary_trail_step_m))
        if tr and math.hypot(pose[0] - tr[-1][0], pose[1] - tr[-1][1]) < step:
            return
        tr.append((float(pose[0]), float(pose[1])))
        del tr[:-(int(float(self.p.hard_boundary_trail_m) / step) + 1)]

    def _hard_boundary_tick(self):
        """Returns True while the hard boundary owns the tick."""
        if not self.p.hard_boundary:
            return False
        if self._hb is not None:
            self._hb_retrace_tick()
            return True
        self._hb_trail_add()
        hcal = self.phase == 'WAITING_FOR_RTK' and self._hcal is not None
        if self.phase not in HARD_BOUNDARY_PHASES and not hcal:
            return False
        if self.phase in ('TRANSIT', 'MOWING') and self.mission is None:
            return False
        out = self._outside()
        if out is None or out <= float(self.p.hard_boundary_m):
            return False
        pose = self.inputs.pose
        where = '(%.2f, %.2f)' % (pose[0], pose[1]) if pose is not None else 'unknown pose'
        if hcal:
            # unverified heading: no retrace on the EKF track, just stop where it is
            self._hcal_fail('left the mowing area by %.2f m at %s' % (out, where))
            return True
        self._hb_count += 1
        self._log('error', 'HARD BOUNDARY: %.2f m outside the areas / paths at %s in %s (> %.2f '
                           'm): cancel, zero, blade off, retrace the driven track (%d/%d this '
                           'mission)' % (out, where, self.phase, float(self.p.hard_boundary_m),
                                         self._hb_count, int(self.p.hard_boundary_max_aborts)))
        if pose is None:
            self._hb_needs_help('outside the boundary, no pose', (0.0, 0.0))
            return True
        m = self.mission
        follow = m is not None and m.step == 'follow'
        ox, oy = float(pose[0]), float(pose[1])
        path = [q for q in reversed(self._hb_trail) if math.hypot(q[0] - ox, q[1] - oy) > 0.02]
        track = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip([(ox, oy)] + path, path))
        self._hb = {'track_m': track, 'phase': self.phase, 'origin': (ox, oy), 'stage': 'pause', 't': self._now,
                    'path': path, 'i': 0, 'follow': follow, 'out0': out,
                    'dock_purpose': self._dock_purpose, 'driven': 0.0, 'last': tuple(pose),
                    'prog': 0.0, 'prog_best': 0.0, 'prog_t': self._now, 'inside_at': None}
        self._cancel_all('hard boundary')
        self._blade_off('hard boundary')
        self._fx.append(ZeroBurst())
        self._fx.append(SuppressStuckGuard(True))
        self._fx.append(RecordIncident('boundary', ox, oy, 'hard boundary: %.2f m outside in %s'
                                       % (out, self.phase)))
        self._loc = None
        if m is not None:
            if follow:
                m.start_local = self._track_progress()
            m.step = None
        self._go(self.phase, 'boundary: %.2f m outside, stopping' % out)
        return True

    def _hb_cmd(self, pose):
        """(v, w) of the retrace: reverse toward the next track point (pure pursuit on the
        rear), both wheels rolling backwards (inner >= hard_boundary_min_inner_mps); a large
        heading error is turned on the spot first (counter-rotating pivot >= 0.3 rad/s);
        straight reverse over the last lookahead once the track points are used up (the
        caller stops at the track's length: never a blind reverse beyond the driven track)."""
        k, p = self._hb, self.p
        v = -min(abs(float(p.hard_boundary_retrace_speed_mps)), 0.10)   # owner: slow reverse
        path, la = k['path'], float(p.hard_boundary_lookahead_m)
        while k['i'] < len(path) and math.hypot(path[k['i']][0] - pose[0],
                                                path[k['i']][1] - pose[1]) < la:
            k['i'] += 1
        if k['i'] >= len(path):
            return v, 0.0
        tx, ty = path[k['i']]
        err = geo.wrap_angle(math.atan2(ty - pose[1], tx - pose[0]) - (float(pose[2]) + math.pi))
        if abs(err) > float(p.hard_boundary_pivot_err_rad):
            return 0.0, math.copysign(max(0.25, abs(float(p.hard_boundary_pivot_rps))), err)
        half = 0.5 * float(p.hard_boundary_track_width_m)
        wmax = max(0.0, (abs(v) - float(p.hard_boundary_min_inner_mps)) / max(1e-3, half))
        return v, max(-wmax, min(wmax, 1.5 * err))

    def _hb_stop(self):
        self._hb = None
        self._fx.append(DriveCmd(0.0, 0.0))
        self._fx.append(SuppressStuckGuard(False))

    def _hb_retrace_tick(self):
        k, pose, now, p = self._hb, self.inputs.pose, self._now, self.p
        if pose is None:
            self._hb_stop()
            self._hb_needs_help('pose lost during the boundary retrace', k['origin'])
            return
        if k['stage'] == 'pause':
            if now - k['t'] < float(p.hard_boundary_pause_s):
                return
            k['stage'], k['prog_t'], k['last'] = 'drive', now, tuple(pose)
            self._go(self.phase, 'boundary: reversing along the driven track')
            self._log('info', 'boundary: reversing along %d track points at %.2f m/s'
                      % (len(k['path']), min(abs(float(p.hard_boundary_retrace_speed_mps)),
                                             0.10)))
        last = k['last']
        step = math.hypot(pose[0] - last[0], pose[1] - last[1])
        turn = abs(geo.wrap_angle(float(pose[2]) - float(last[2])))
        k['driven'] += step
        k['prog'] += step + 0.5 * float(p.hard_boundary_track_width_m) * turn
        k['last'] = tuple(pose)
        if k['prog'] >= k['prog_best'] + float(p.hard_boundary_progress_m):
            k['prog_best'], k['prog_t'] = k['prog'], now
        # Success = a fresh ~/outside_m reading inside (<= 1 mm) after reversing
        # hard_boundary_retrace_extra_m past the re-entry point, or inside when the recorded
        # track / hard_boundary_retrace_max_m ends first. Never "back inside" on a reading
        # that is still outside (sim 2026-10-15: the track ran out 0.066 m outside and the
        # old <= hard_boundary_m test resumed from there), stale or missing.
        out = self._outside()
        inside = out is not None and out <= 1e-3
        if inside:
            if k['inside_at'] is None:
                k['inside_at'] = k['driven']
            if k['driven'] - k['inside_at'] >= float(p.hard_boundary_retrace_extra_m):
                self._hb_retraced()
                return
        else:
            k['inside_at'] = None             # outside, or no fresh reading: re-entry unproven
        if k['driven'] >= min(float(p.hard_boundary_retrace_max_m), k['track_m'] + 0.02):
            if inside:
                self._hb_retraced()
                return
            self._hb_stop()
            self._hb_needs_help('retrace ended still %s after reversing %.2f m (driven track '
                                '%.2f m, max %.2f m)'
                                % ('outside by an unknown distance (no fresh outside_m)'
                                   if out is None else '%.3f m outside' % out, k['driven'],
                                   k['track_m'], float(p.hard_boundary_retrace_max_m)),
                                k['origin'])
            return
        if now - k['prog_t'] > float(p.hard_boundary_progress_s):
            self._hb_stop()
            self._hb_needs_help('boundary retrace made no progress in %.0f s'
                                % float(p.hard_boundary_progress_s), k['origin'])
            return
        self._fx.append(DriveCmd(*self._hb_cmd(pose)))

    def _hb_retraced(self):
        k = self._hb
        self._hb_stop()
        self._hb_trail = []
        ox, oy = k['origin']
        n, lim = self._hb_count, int(self.p.hard_boundary_max_aborts)
        past = k['driven'] - k['inside_at'] if k['inside_at'] is not None else 0.0
        self._log('info', 'boundary: back inside after reversing %.2f m (%.2f m past the '
                          're-entry%s, excursion %d/%d)'
                  % (k['driven'], past, '' if past >= float(self.p.hard_boundary_retrace_extra_m)
                     else ', track / max reached', n, lim))
        if n >= lim:
            self._hb_needs_help('left the boundary %d times in this mission' % n, (ox, oy))
            return
        why = 'left the boundary (%.2f m outside)' % k['out0']
        if k['phase'] in DOCK_PHASES:
            self._dock(k['phase'], k['dock_purpose'], 'resuming after the boundary retrace')
            return
        m = self.mission
        if m is None or m.area_idx is None or not m.subpaths:
            self._go_idle('boundary retrace: nothing to resume')
            return
        if k['phase'] == 'TRANSIT':
            if m.detour is not None:
                self._detour_failed(why)
                return
            m.route_fallback = False          # the retry asks for the drawn route again
            self._transit_abort(why)
            return
        m.detour = None
        self._dispatch()

    def _hb_needs_help(self, why, pose):
        """Like the stuck guard's give-up (same latched phase, same operator clear)."""
        x, y = float(pose[0]), float(pose[1])
        self._log('error', 'STUCK_NEEDS_HELP: %s at (%.2f, %.2f): motion stopped, blade off '
                  '(STOP / START / MANUAL to clear)' % (why, x, y))
        if self.mission is not None:
            self._interrupt_mission('boundary')
        self._cancel_all('boundary')
        self._blade_off('boundary')
        self._fx.append(ZeroBurst())
        self._dock_purpose = None
        self._resume_after = None
        self._hb = None
        self._go('STUCK_NEEDS_HELP', 'boundary: needs help at (%.2f, %.2f)' % (x, y))


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
    if fsm.phase == 'MOWING' and m.step in ('spinup', 'follow', 'corner_pivot'):
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
    npiv = nfil = 0
    why, why_deg = {}, {}
    for k, sp in enumerate(subpaths):
        if geo.is_reverse_subpath(sp):
            out.append(sp)                 # a reverse leg of a turn kept as planned
            continue
        r = pcond.condition(sp, region, prm)
        if not r.valid:
            cond_errors[k] = r.errors[0]
            logs.append(('error', 'area %d sub-path %d: path conditioning failed (%s): it '
                                  'will not be driven' % (area, k, r.errors[0])))
            out.append(sp)
            continue
        cond[k] = {'types': r.types, 'pivots': r.pivots}
        npiv += len(r.pivots)
        nfil += sum(1 for c in r.corners if c.choice == 'fillet')
        for c in r.corners:
            if c.choice == 'pivot':
                why[c.why] = why.get(c.why, 0) + 1
                why_deg[c.why] = why_deg.get(c.why, 0.0) + abs(math.degrees(c.turn))
        out.append(r.poses)
    logs.append(('info', 'area %d: path conditioning: %d fillets, %d pivots (%s), %d '
                         'sub-path(s) failed validation' % (
                             area, nfil, npiv, ', '.join(
                                 '%s %d / %.0f deg' % (k, n, why_deg[k])
                                 for k, n in sorted(why.items())) or 'none',
                             len(cond_errors))))
    return {'n_in': n_in, 'subpaths': out, 'cond': cond, 'cond_errors': cond_errors,
            'logs': logs}
