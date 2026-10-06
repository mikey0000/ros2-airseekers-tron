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
import math
from dataclasses import dataclass, field
from typing import Any, Optional

from mower_mission import geometry as geo
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
CMD_RESET_EMERGENCY = 254
CMD_DELETE_MAPS = 255

FIX_RTK_FIXED = 3

# heading_aligner (mower_localization) sources that count as an absolute heading
HEADING_ALIGNED_SOURCES = ('dock', 'cog', 'file_verified')
HEADING_NOT_ALIGNED = 'heading not aligned: drive straight 1 m in manual'

# MANUAL_MOWING sub_state (the GUI reads the blade state from it)
SUB_MANUAL_BLADE_ON = 'joystick, blade on'
SUB_MANUAL_BLADE_OFF = 'joystick, blade off'

# action / service names used in effects (the node maps them to ROS names)
ACT_UNDOCK = 'undock'
ACT_DOCK = 'dock'
ACT_BACKUP = 'backup'
ACT_PLAN = 'plan_coverage'
ACT_NAV = 'navigate_to_pose'
ACT_FOLLOW = 'follow_path'

SRV_GET_AREA = 'get_mowing_area'
SRV_ADD_AREA = 'add_area'
SRV_CLEAR_ESTOP = 'clear_estop'
SRV_CHARGING = 'charging'
SRV_GET_AREA_SETTINGS = 'get_area_settings'   # /map_server_node/get_area_settings
SRV_SET_PARAMS = 'set_parameters'             # request {'node': ..., 'params': {...}}
SRV_CUTTER_HEIGHT = 'cutter_height'           # /cutter_control height only, blade off
SRV_CLEAR_COSTMAPS = 'clear_costmaps'         # Nav2 clear_entirely_{global,local}_costmap

# set_parameters targets (the node maps them to <node>/set_parameters)
PARAM_NODE_COVERAGE = 'coverage_server'
PARAM_NODE_CONTROLLER = 'controller_server'

# Per-area mowing settings (mower_map area_settings.py BUILTIN_DEFAULTS; used
# when /map_server_node/get_area_settings is unavailable or leaves a key out).
AREA_SETTINGS_DEFAULTS = {
    'cutter_height_mm': 50,
    'perimeter_laps': 2,
    'path_mode': 'zigzag',
    'mow_angle_deg': -1.0,
    'cut_speed_mps': 0.3,
    'swath_overlap_m': 0.02,
    'edge_first': True,
    'repeat': 1,
    'alternate_angle_offset_deg': 90.0,
}
PATH_MODES = ('zigzag', 'cross', 'alternate', 'spiral', 'contour_only')

# action outcomes reported by the node
SUCCEEDED = 'succeeded'
ABORTED = 'aborted'
CANCELED = 'canceled'
REJECTED = 'rejected'
UNAVAILABLE = 'unavailable'   # server not up within the wait timeout
TIMEOUT = 'timeout'           # FSM watchdog

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
}

IDLE_NAMES = ('IDLE', 'IDLE_DOCKED', 'CHARGING')
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
class PublishTrajectory:
    points: list


@dataclass
class PublishAreaSettings:
    settings: dict          # active settings of the area being planned / mowed (JSON-able)


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
    # /heading_aligner/status (latched JSON): aligned + source (none|file|dock|cog)
    heading_aligned: bool = False
    heading_source: str = 'none'
    heading_stamp: Optional[float] = None     # receipt time of the last status
    # /obstacle_policy (mower_vision obstacle_guard, JSON): closest close detection
    obstacle_kind: str = 'none'         # none | dynamic | static
    obstacle_class: str = ''
    obstacle_distance: Optional[float] = None
    obstacle_stamp: Optional[float] = None    # receipt time of the last policy message


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
    rain_mode: int = 1                  # 0 ignore, 1 dock and wait
    rain_debounce_s: float = 2.0
    rain_delay_minutes: float = 30.0
    use_docking_server: bool = True
    undock_distance_m: float = 0.8
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
    # FollowPath abort / timeout mid-swath: resume from the last reached pose.
    follow_retries: int = 3
    clear_costmaps_on_retry: bool = True
    # After a sub-path still fails, the session ends in MOWING_INCOMPLETE; dock only
    # when this is true (default: stop in place so the operator can inspect).
    return_home_on_incomplete: bool = False
    follow_controller_id: str = 'FollowCoveragePath'
    follow_goal_checker_id: str = 'coverage_goal_checker'
    blade_confirm_timeout_s: float = 5.0
    blade_start_attempts: int = 3
    blade_retry_pause_s: float = 2.0
    require_blade_confirmation: bool = True
    # MANUAL_MOWING starts blade OFF; the blade runs only after an explicit
    # manual_blade(True) request (GUI mow_enabled=1). False = upstream behaviour
    # (blade on as soon as MANUAL_MOW is entered).
    manual_blade_requires_enable: bool = False
    blade_spinup_s: float = 2.0         # used when confirmation is disabled
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
    record_min_area_m2: float = 1.0
    record_max_points: int = 50000
    max_areas: int = 200
    result_display_s: float = 5.0
    service_timeout_s: float = 10.0
    plan_timeout_s: float = 120.0
    undock_timeout_s: float = 180.0
    transit_timeout_s: float = 300.0
    follow_timeout_s: float = 3600.0
    dock_action_timeout_s: float = 400.0
    # --- per-area mowing settings ---
    use_area_settings: bool = True      # read /map_server_node/get_area_settings per area
    cut_width_m: float = 0.20           # blade cut width; swath spacing = this - swath_overlap_m
    cut_speed_max_mps: float = 0.5
    controller_speed_param: str = 'FollowCoveragePath.desired_linear_vel'
    set_cut_speed: bool = True          # set_parameters on /controller_server per area
    set_cutter_height: bool = True      # /cutter_control height per area
    # mm -> MCU height percent, flattened (mm, percent) pairs, piecewise linear,
    # clamped. The MCU's 0-100 scale is UNVERIFIED on the Tron (see README).
    cutter_height_mm_to_percent: list = field(
        default_factory=lambda: [30.0, 0.0, 90.0, 100.0])
    # --- obstacle avoidance (/obstacle_policy from obstacle_guard) ---
    obstacle_avoidance: bool = True     # dynamic stop-and-wait + static swath detours
    obstacle_policy_timeout_s: float = 1.0    # older policy = 'none'
    obstacle_static_memory_s: float = 3.0     # a 'static' seen this recently explains an abort
    dynamic_wait_s: float = 30.0        # person / pet: wait this long, then treat as static
    dynamic_clear_hold_s: float = 2.0   # policy clear this long before resuming
    detour_skip_m: float = 1.0          # detour target: this far along the path past the robot
    detour_skip_step_m: float = 0.5     # ... extended by this when the detour transit fails
    detour_max_skip_m: float = 3.0      # ... up to this
    max_detours_per_subpath: int = 5
    detour_max_leave_m: float = 0.0     # detour pose may lie this far outside the area ring
    # follow_path abort with no classified obstacle (stereo cloud / bumper marks only):
    # detour after this many plain retries at the sub-path chunk (-1 = never detour).
    detour_unclassified_after: int = 1
    requeue_skipped: bool = True        # re-try detour-skipped stretches at the area end

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
    runs: list = field(default_factory=list)    # [{'angle': deg|-1, 'perpendicular': bool}]
    run_i: int = 0
    detours: int = 0                            # per sub-path
    detour: Optional[dict] = None               # in-flight detour {'from', 'to', 'skip', 'why'}
    stretch: Optional[tuple] = None             # re-queued (abs_from, abs_to) being re-tried
    stretch_end: Optional[int] = None           # local end index of that stretch
    requeue: Optional[list] = None              # stretches left in the end-of-area pass
    blocked: list = field(default_factory=list)  # [(area, abs_from, abs_to)] still blocked


@dataclass
class _Pending:
    name: str
    token: int
    deadline: float
    purpose: Any = None


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
        self._created = now
        self._phase_since = now
        self._fx = []
        self._token = 0
        self._action = None
        self._service = None
        self._emergency = False
        self._dock_purpose = None
        self._resume_after = None           # None | 'rain' | 'charge'
        self._rain_since = None
        self._rain_clear_since = None
        self._boundary_since = None
        self._heading_wait_since = None
        self._spin_attempt = 0
        self._spin_deadline = 0.0
        self._track = []
        self._last_sample = -1e9
        self._record_poly = None
        self._enum = None                   # {'purpose', 'index', 'areas', 'count'}
        self._record_name = ''
        self._last_status = None
        self._incomplete_text = ''
        self._dyn = None                    # dynamic-obstacle wait context
        self._last_static_t = -1e9

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
            'sub_state_name': self.sub_state,
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

    def _go(self, name, sub=None):
        if self.blade_on and name not in ('MOWING', 'MANUAL_MOWING'):
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
        # Re-assert "off" every 60 s: the docking server enables charging itself at
        # the end of a dock and could otherwise override the limit.
        if b >= limit and (self._charge_req is not False or self._now - self._charge_req_t >= 60.0):
            self._charge_req, self._charge_req_t = False, self._now
            self._log('info', 'battery %.0f%% >= max charge %.0f%%: charging disabled' % (b, limit))
            self._call(SRV_CHARGING, {'enable': False}, track=False)
        elif b <= limit - float(self.p.battery_charge_hysteresis_percent) \
                and self._charge_req is not True:
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
        self.blade_on = False
        self._fx.append(BladeOff(reason))

    def _blade_on(self):
        self.blade_on = True
        self._fx.append(BladeOn())

    def _next_token(self):
        self._token += 1
        return self._token

    def _start_action(self, name, goal, timeout, purpose=None):
        if self._action is not None:
            self._fx.append(CancelActions('superseded by %s' % name))
        tok = self._next_token()
        self._action = _Pending(name, tok, self._now + float(timeout), purpose)
        self._fx.append(StartAction(name, goal, tok))
        return tok

    def _call(self, name, request, track=True, purpose=None):
        tok = self._next_token()
        if track:
            self._service = _Pending(name, tok, self._now + self.p.service_timeout_s, purpose)
        self._fx.append(CallService(name, request, tok))
        return tok

    def _cancel_all(self, reason):
        self._dyn = None
        self._action = None
        self._service = None
        self._enum = None
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

    def on_action_result(self, token, outcome, result=None, now=None):
        self._begin(self._now if now is None else now)
        a = self._action
        if a is None or a.token != token:
            return self._end()          # stale (cancelled / superseded)
        self._action = None
        self._handle_action(a, outcome, result or {})
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
        stamp = i.emergency_stamp if i.emergency_stamp is not None else self._created
        if self._now - stamp > self.p.emergency_timeout_s:
            causes.append('/hardware_bridge/emergency silent > %.1fs' % self.p.emergency_timeout_s)
        return ', '.join(causes)

    def _guards(self):
        """Returns True when a guard owns this tick (phase logic skipped)."""
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

        i = self.inputs
        if self.phase in ('MOWING', 'TRANSIT', 'BOUNDARY_PAUSED', 'PLANNING') \
                and self.mission is not None:
            if i.lethal_boundary_violation:
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
        if self._policy() == 'static':
            self._last_static_t = now
        if self._dynamic_tick():
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
        elif ph == 'RAIN_WAITING':
            if i.rain and self.p.rain_mode == 1:
                self._rain_clear_since = None
            elif self._rain_clear_since is None:
                self._rain_clear_since = now
            elif now - self._rain_clear_since >= self.p.rain_delay_minutes * 60.0:
                self._log('info', 'dry for %.0f min: resuming mowing' % self.p.rain_delay_minutes)
                self._resume('rain')
        elif ph == 'WAITING_FOR_RTK':
            if i.fix_type is not None and i.fix_type >= self.p.rtk_fix_type:
                if self.heading_ok():
                    self._log('info', 'RTK fixed, heading aligned (%s): planning'
                              % i.heading_source)
                    self._next_area()
                    return
                if self._heading_wait_since is None:
                    self._heading_wait_since = now
                    self._log('info', 'RTK fixed: waiting for heading alignment')
                    self._go('WAITING_FOR_RTK', 'waiting for heading alignment')
                elif now - self._heading_wait_since > self.p.heading_wait_s:
                    self._heading_failed()
            elif now - self._phase_since > self.p.rtk_timeout_s:
                self._mission_failed('no RTK fixed within %.0fs' % self.p.rtk_timeout_s)
        elif ph in ('MOWING', 'TRANSIT') and self.mission is not None \
                and self.mission.step == 'retry_wait':
            if now >= self.mission.retry_at:
                self._retry_fire()
        elif ph == 'MOWING' and self.mission is not None:
            if self.mission.step == 'follow':
                self._track_progress()
            self._tick_blade()
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
        return {ACT_PLAN: self.p.plan_timeout_s, ACT_UNDOCK: self.p.undock_timeout_s,
                ACT_BACKUP: self.p.undock_timeout_s, ACT_NAV: self.p.transit_timeout_s,
                ACT_FOLLOW: self.p.follow_timeout_s,
                ACT_DOCK: self.p.dock_action_timeout_s}.get(name, 60.0)

    # ==================================================================
    # commands
    # ==================================================================
    def _command(self, cmd):
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
        if self.phase != 'EMERGENCY':
            self._go_idle('stopped')

    def _home(self):
        if self.phase in DOCK_PHASES:
            return True
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
        if not self._at_dock() and not self.heading_ok():
            # Undocked: Nav2 would steer with an arbitrary IMU heading. At the dock the
            # undock (straight drive out along the dock yaw) seeds it instead.
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

    def _start(self, single=None, auto=False):
        if self.phase == 'RECORDING':
            self._log('warn', 'START refused: finish or cancel the recording first')
            return False
        if self.mission is not None:
            if single is None or single == self.mission.single_area_target:
                return True             # already mowing: idempotent
            self._log('warn', 'start_in_area(%d) refused: mowing session running' % single)
            return False
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

    def _undock(self):
        self._blade_off('undock')
        self._go('UNDOCKING', '')
        if self.p.use_docking_server:
            self._start_action(ACT_UNDOCK, {
                'distance_m': self.p.undock_distance_m,
                'speed_mps': self.p.undock_speed_mps,
                'wait_for_rtk': False,
                'rtk_timeout_s': self.p.rtk_timeout_s}, self.p.undock_timeout_s)
        else:
            self._call(SRV_CHARGING, {'enable': False}, track=False)
            self._start_action(ACT_BACKUP, {'distance_m': self.p.undock_distance_m,
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
    def height_percent(self, mm):
        """cutter_height_mm -> MCU percent via cutter_height_mm_to_percent."""
        flat = [float(v) for v in (self.p.cutter_height_mm_to_percent or [])]
        pts = sorted(zip(flat[0::2], flat[1::2]))
        if not pts:
            return 0
        mm = float(mm)
        if mm <= pts[0][0]:
            pct = pts[0][1]
        elif mm >= pts[-1][0]:
            pct = pts[-1][1]
        else:
            pct = pts[-1][1]
            for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
                if x0 <= mm <= x1:
                    pct = y0 + (y1 - y0) * (mm - x0) / (x1 - x0) if x1 > x0 else y1
                    break
        return int(round(max(0.0, min(100.0, pct))))

    def _merge_settings(self, ok, resp):
        st = dict(AREA_SETTINGS_DEFAULTS)
        src = 'built-in defaults'
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
                      % self.mission.area_idx)
        # Global mow_angle_deg parameter: fallback for areas left on auto.
        if float(st['mow_angle_deg']) < 0.0 and self.p.mow_angle_deg >= 0.0:
            st['mow_angle_deg'] = float(self.p.mow_angle_deg)
        if st['path_mode'] not in PATH_MODES:
            self._log('warn', 'unknown path_mode %r: zigzag' % st['path_mode'])
            st['path_mode'] = 'zigzag'
        st['repeat'] = max(1, int(st['repeat']))
        st['perimeter_laps'] = max(0, int(st['perimeter_laps']))
        return st, src

    def _area_name(self):
        m = self.mission
        return str(m.areas.get(m.area_idx, {}).get('name', ''))

    def _build_runs(self, st):
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
            done = int(self.alternate_counts.get(self._area_name(), 0))
            return [run((done + k) * off) for k in range(n)]
        return [run(0.0) for _ in range(n)]

    def _on_area_settings(self, ok, resp):
        m = self.mission
        st, src = self._merge_settings(ok, resp)
        m.settings = st
        m.runs = self._build_runs(st)
        if m.run_i >= len(m.runs):
            m.run_i = 0
        pct = self.height_percent(st['cutter_height_mm'])
        speed = min(float(st['cut_speed_mps']), float(self.p.cut_speed_max_mps))
        self._log('info', 'area %d %s: settings (%s) %s; blade height %d mm -> %d %%, '
                          'cut speed %.2f m/s, %d run(s)'
                  % (m.area_idx, self._area_name(), src, json.dumps(st, sort_keys=True),
                     int(st['cutter_height_mm']), pct, speed, len(m.runs)))
        if self.p.set_cutter_height:
            # Blade is off here (PLANNING); height only, before any mowing.
            self._call(SRV_CUTTER_HEIGHT, {'height_mm': int(st['cutter_height_mm']),
                                           'percent': pct}, track=False)
        if self.p.set_cut_speed:
            self._call(SRV_SET_PARAMS, {'node': PARAM_NODE_CONTROLLER,
                                        'params': {self.p.controller_speed_param: speed}},
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
            'cutter_height_percent': self.height_percent(m.settings['cutter_height_mm']),
            'operation_width_m': round(self.p.cut_width_m - float(m.settings['swath_overlap_m']),
                                       4)})
        return out

    def _plan_run(self):
        m = self.mission
        st, run = m.settings, m.runs[m.run_i]
        self._fx.append(PublishAreaSettings(self.active_settings()))
        self._go('PLANNING', 'area %d %s [%s, run %d/%d]' % (
            m.area_idx, self._area_name(), st['path_mode'], m.run_i + 1, len(m.runs)))
        self._call(SRV_SET_PARAMS, {'node': PARAM_NODE_COVERAGE, 'params': {
            'operation_width': max(0.01, float(self.p.cut_width_m) - float(st['swath_overlap_m'])),
            'headland_rings': int(st['perimeter_laps']),
            'path_mode': str(st['path_mode']),
            'mow_angle_deg': float(run['angle']),
            'edge_first': bool(st['edge_first'])}}, purpose='coverage_params')

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
        subpaths = []
        if outcome == SUCCEEDED and result.get('success'):
            subpaths = [list(sp) for sp in result.get('drivable_subpaths', []) if len(sp) >= 2]
            if not subpaths:
                subpaths = [list(sp) for sp in result.get('segments', []) if len(sp) >= 2]
            if not subpaths and len(result.get('full_path', [])) >= 2:
                subpaths = [list(result['full_path'])]
        if not subpaths:
            why = result.get('message') or outcome
            self._log('warn', 'area %d: coverage planning failed (%s): skipping it'
                      % (m.area_idx, why))
            m.failed.append((m.area_idx, None, 'area %d planning failed: %s' % (m.area_idx, why),
                             -1))
            self._go('AREA_UNREACHABLE', 'area %d: planning failed: %s' % (m.area_idx, why))
            self._next_area()
            return
        m.subpaths = subpaths
        m.planned_total += len(subpaths)
        m.cum = [geo.cumulative_lengths(sp) for sp in subpaths]
        m.lengths = [c[-1] for c in m.cum]
        full = result.get('full_path') or [p for sp in subpaths for p in sp]
        self._fx.append(PublishPlan(list(full)))
        self._plan_shown = True
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
        m.recovering = False
        m.detours, m.detour = 0, None
        self._dispatch()

    def _dispatch(self, force_transit=False):
        """Send the current sub-path from ``start_local``: blade-off transit
        first when its start is farther than ``transit_gap_m``."""
        m = self.mission
        sp = m.subpaths[m.sub_i]
        if self._seg_last() - m.start_local < 1:
            self._segment_done()
            return
        target = sp[m.start_local]
        if force_transit or geo.needs_transit(self.inputs.pose, target, self.p.transit_gap_m):
            # SAFETY (FollowStrip): blade off before any dispatch across the gap.
            self._blade_off('transit')
            m.step = 'transit'
            self._go('TRANSIT', 'area %d sub-path %d/%d' % (
                m.area_idx, m.sub_i + 1, len(m.subpaths)))
            yaw = geo.heading_of(sp, m.start_local)
            m.transit_target = (target[0], target[1], yaw)
            self._start_action(ACT_NAV, {'pose': m.transit_target},
                               self.p.transit_timeout_s)
        else:
            self._begin_blade()

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
        m.step = 'follow'
        m.progress_local = m.start_local
        sp = m.subpaths[m.sub_i]
        m.chunk_end = min(geo.chunk_end(sp, m.start_local, self.p.follow_chunk_m,
                                        self.p.follow_chunk_clearance_m), self._seg_last())
        self._start_action(ACT_FOLLOW, {
            'poses': list(sp[m.start_local:m.chunk_end + 1]),
            'controller_id': self.p.follow_controller_id,
            'goal_checker_id': self.p.follow_goal_checker_id}, self.p.follow_timeout_s)

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

    def _subpath_failed(self, why):
        """Retries exhausted: remember the sub-path as NOT mowed (it stays out of
        ``completed``, so a resume retries it) and go on with the next one."""
        m = self.mission
        if m.stretch is not None:
            self._stretch_blocked(why)
            return
        m.skipped += 1
        sp = m.subpaths[m.sub_i]
        local = self._track_progress() if m.step == 'follow' else m.start_local
        offs, _ = subpath_offsets(m.subpaths)
        m.failed.append((m.area_idx, m.sub_i, why, offs[m.sub_i] + min(local, len(sp) - 1)))
        self._log('warn', 'area %d sub-path %d NOT mowed: %s' % (m.area_idx, m.sub_i, why))
        m.step = None
        m.sub_i += 1
        self._next_subpath()

    # ---- retries ---------------------------------------------------------
    def _clear_costmaps(self):
        if self.p.clear_costmaps_on_retry:
            self._call(SRV_CLEAR_COSTMAPS, {}, track=False)

    def _schedule_retry(self, phase, sub):
        """Blade off, clear the costmaps, wait transit_retry_delay_s, then re-send."""
        m = self.mission
        self._blade_off('retry')
        self._clear_costmaps()
        m.step = 'retry_wait'
        m.retry_at = self._now + float(self.p.transit_retry_delay_s)
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
            self._start_action(ACT_DOCK, {'use_vision': self.p.dock_use_vision,
                                          'timeout_s': self.p.dock_timeout_s},
                               self.p.dock_action_timeout_s, purpose='dock')
        elif len(self.p.dock_pose) == 3:
            self._start_action(ACT_NAV, {'pose': tuple(self.p.dock_pose)},
                               self.p.transit_timeout_s, purpose='dock')
        else:
            self._dock_failed('no docking server and no dock_pose configured')

    def _dock_done(self):
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
        self._dock_purpose = None
        self._resume_after = None
        self._log('error', 'docking failed: %s' % why)
        self._go('NAV_TO_DOCK_FAILED', why)

    # ==================================================================
    # recording (RecordArea port)
    # ==================================================================
    def _record_start(self):
        if self.state == STATE_AUTONOMOUS:
            self._log('warn', 'RECORD_AREA refused: autonomous run active (STOP first)')
            return False
        if self.phase == 'RECORDING':
            return True
        self._leave_current_mode('record area')
        self._blade_off('recording')
        self._track = []
        self._record_poly = None
        self._last_sample = -1e9
        self._go('RECORDING', 'drive the boundary with the joystick')
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
        n = len(self._track)
        poly = geo.simplify_ring(self._track, self.p.record_simplify_tolerance_m)
        if len(poly) < 3 and n >= 3:
            poly = list(self._track)     # upstream: keep raw if DP too aggressive
        area = geo.polygon_area(poly)
        if len(poly) < 3 or area < self.p.record_min_area_m2:
            why = 'polygon has %d vertices / %.2f m^2 (need 3 and %.1f m^2)' % (
                len(poly), area, self.p.record_min_area_m2)
            self._log('warn', 'recording rejected: %s' % why)
            self._track = []
            self._fx.append(PublishTrajectory([]))
            self._go_idle('recording rejected: %s' % why)
            return False
        self._record_poly = poly
        self._log('info', 'recording finished: %d samples -> %d vertices, %.1f m^2' % (
            n, len(poly), area))
        self._go('RECORDING', 'saving area')
        self._enum = {'purpose': 'record', 'index': 0, 'areas': {}, 'count': 0}
        self._call(SRV_GET_AREA, {'index': 0}, purpose='enum')
        return True

    def _record_save(self, mowing_count):
        name = 'Area %d' % (mowing_count + 1)
        self._record_name = name
        self._call(SRV_ADD_AREA, {'name': name, 'polygon': list(self._record_poly),
                                  'is_navigation_area': False}, purpose='add_area')

    def _record_saved(self, ok):
        poly, self._record_poly = self._record_poly, None
        self._track = []
        self._fx.append(PublishTrajectory([]))
        if ok:
            self._log('info', 'area "%s" saved (%d vertices)' % (self._record_name, len(poly)))
            self._go('RECORDING_COMPLETE', self._record_name)
        else:
            self._log('error', 'add_area failed: polygon kept in a fallback file')
            self._fx.append(SaveRecordingFallback(list(poly), self._record_name))
            self._go_idle('add_area failed; polygon saved locally')

    # ==================================================================
    # results
    # ==================================================================
    def _transit_arrived(self):
        """True when the last transit target is within transit_arrived_radius_m."""
        m = self.mission
        tgt = getattr(m, 'transit_target', None) if m is not None else None
        if tgt is None or self.inputs.pose is None or self.p.transit_arrived_radius_m <= 0:
            return False
        return geo.dist(self.inputs.pose, tgt) <= self.p.transit_arrived_radius_m

    def _handle_action(self, a, outcome, result):
        name = a.name
        if outcome != SUCCEEDED:
            self._log('warn', '%s finished: %s %s' % (name, outcome, result.get('message', '')))
        if a.purpose == 'dock':
            if outcome == SUCCEEDED and (result.get('success', True)):
                self._dock_done()
            else:
                why = result.get('message') or outcome
                if outcome == UNAVAILABLE:
                    why = 'docking action server unavailable'
                self._dock_failed(why)
            return
        m = self.mission
        if m is None:
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
                self._begin_blade()
            elif outcome == UNAVAILABLE:
                self._mission_failed('/navigate_to_pose action server unavailable')
            elif self._transit_arrived():
                d = geo.dist(self.inputs.pose, m.transit_target)
                self._log('warn', 'transit %s but robot is %.2f m from the target '
                                  '(<= %.2f m): treating as arrived'
                          % (outcome, d, self.p.transit_arrived_radius_m))
                m.recovering = False
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
                m.transit_fails += 1
                if m.transit_fails > self.p.transit_retries:
                    self._subpath_failed('transit %s %d times' % (outcome, m.transit_fails))
                    return
                self._schedule_retry('TRANSIT', 'area %d sub-path %d/%d: transit %s, retry %d/%d '
                                     'in %.0f s' % (m.area_idx, m.sub_i + 1, len(m.subpaths),
                                                    outcome, m.transit_fails,
                                                    self.p.transit_retries,
                                                    self.p.transit_retry_delay_s))
        elif name == ACT_FOLLOW:
            if outcome == SUCCEEDED:
                cum = m.cum[m.sub_i]
                prog = self._track_progress()
                last = self._seg_last()
                end = min(m.chunk_end, last)
                left = cum[end] - cum[min(prog, end)]
                if left > self.p.follow_end_tolerance_m:
                    # Stock Humble goal checkers only look at the final pose; a coverage
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
                if (static or (pf >= 0 and m.follow_fails >= pf)) and self._try_detour(
                        prog, '%s obstacle' % (self.inputs.obstacle_class or 'static')
                        if static else 'follow_path %s' % outcome):
                    return
                m.follow_fails += 1
                m.start_local = prog
                if m.follow_fails > self.p.follow_retries:
                    self._subpath_failed('follow_path %s %d times (at pose %d)'
                                         % (outcome, m.follow_fails, m.start_local))
                    return
                self._schedule_retry('MOWING', 'area %d sub-path %d/%d: follow_path %s at pose %d, '
                                     'retry %d/%d in %.0f s' % (
                                         m.area_idx, m.sub_i + 1, len(m.subpaths), outcome,
                                         m.start_local, m.follow_fails, self.p.follow_retries,
                                         self.p.transit_retry_delay_s))

    # ==================================================================
    # obstacle avoidance
    # ==================================================================
    def _policy(self):
        """Fresh /obstacle_policy kind: 'none' | 'dynamic' | 'static'."""
        i = self.inputs
        if not self.p.obstacle_avoidance or i.obstacle_stamp is None or \
                self._now - i.obstacle_stamp > self.p.obstacle_policy_timeout_s:
            return 'none'
        return i.obstacle_kind if i.obstacle_kind in ('dynamic', 'static') else 'none'

    def _static_recent(self):
        return self._policy() == 'static' or \
            self._now - self._last_static_t <= self.p.obstacle_static_memory_s

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
                                                                    'follow'):
            return 'follow'
        if self.phase == 'TRANSIT' and m is not None and m.step == 'transit' and a is not None:
            return 'transit'
        if self.phase in DOCK_PHASES and a is not None and a.purpose == 'dock':
            return 'dock'
        return None

    def _dyn_start(self, ctx):
        m = self.mission
        cls = self.inputs.obstacle_class or 'obstacle'
        if m is not None and ctx == 'follow' and m.step == 'follow':
            m.start_local = self._track_progress()
        self._dyn = {'ctx': ctx, 'since': self._now, 'clear_since': None, 'cls': cls,
                     'sub': self.sub_state, 'phase': self.phase, 'purpose': self._dock_purpose}
        if self._action is not None:
            self._fx.append(CancelActions('dynamic obstacle: %s' % cls))
            self._action = None
        self._blade_off('dynamic obstacle: %s' % cls)
        self._fx.append(ZeroBurst())
        if m is not None and ctx in ('follow', 'transit'):
            m.step = 'dyn_wait'
        d = self.inputs.obstacle_distance
        self._log('warn', '%s%s ahead: stopped, blade off, waiting up to %.0f s' % (
            cls, (' at %.1f m' % d) if d is not None else '', self.p.dynamic_wait_s))
        self._go(self.phase, 'waiting for %s to move (0 s)' % cls)

    def _dynamic_tick(self):
        """Returns True when the dynamic-obstacle logic owns this tick."""
        d, kind = self._dyn, self._policy()
        if d is None:
            if kind != 'dynamic':
                return False
            ctx = self._dyn_context()
            if ctx is None:
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
        if waited >= self.p.dynamic_wait_s:
            self._dyn = None
            self._dyn_timeout(d)
            return True
        sub = 'waiting for %s to move (%d s)' % (d['cls'], int(waited))
        if sub != self.sub_state:
            self._go(self.phase, sub)
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
            self._start_action(ACT_NAV, {'pose': m.transit_target}, self.p.transit_timeout_s)
        else:
            m.step = None
            self._dispatch()

    def _dyn_timeout(self, d):
        m = self.mission
        why = '%s did not move within %.0f s' % (d['cls'], self.p.dynamic_wait_s)
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
        self._blade_off('detour')
        m.detour = {'from': cur, 'to': idx, 'skip': skip, 'why': why}
        m.start_local = idx
        m.step = 'transit'
        m.transit_target = (sp[idx][0], sp[idx][1], geo.heading_of(sp, idx))
        self._log('warn', 'area %d sub-path %d: %s: blade-off detour %d/%d to pose %d '
                          '(%.1f m ahead)' % (m.area_idx, m.sub_i, why, m.detours + 1,
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
        m.recovering, m.detour = False, None
        self._log('info', 'area %d: re-trying skipped stretch sub-path %d poses %d -> %d'
                  % (m.area_idx, a[0], a[1], b[1]))
        self._dispatch()

    def _stretch_done(self):
        m = self.mission
        ac = self.cursor.area(m.area_idx)
        if m.stretch in ac.skipped:
            ac.skipped.remove(m.stretch)
        self._log('info', 'area %d: skipped stretch %s mowed' % (m.area_idx, m.stretch))
        m.stretch, m.stretch_end, m.step = None, None, None
        self._persist()
        self._area_done()

    def _stretch_blocked(self, why):
        m = self.mission
        m.blocked.append((m.area_idx,) + tuple(m.stretch))
        self._log('warn', 'area %d: stretch %s still blocked (%s): obstacle cannot be bypassed'
                  % (m.area_idx, m.stretch, why))
        m.stretch, m.stretch_end, m.step = None, None, None
        self._blade_off('stretch blocked')
        self._area_done()

    def _handle_service(self, s, ok, resp):
        if s.purpose == 'enum':
            e = self._enum
            if e is None:
                return
            if ok and resp.get('success'):
                area = resp.get('area', {})
                e['count'] += 1
                if not area.get('is_navigation_area', False):
                    e['areas'][e['index']] = area
                e['index'] += 1
                if e['index'] < self.p.max_areas:
                    self._call(SRV_GET_AREA, {'index': e['index']}, purpose='enum')
                    return
            self._enum = None
            if e['purpose'] == 'start':
                if self.mission is None:
                    return
                if not ok and e['index'] == 0:
                    self._abort_start('/map_server_node/get_mowing_area unavailable')
                    return
                self._areas_loaded(e['areas'], e['count'])
            else:
                if self._record_poly is not None:
                    self._record_save(len(e['areas']))
        elif s.purpose == 'add_area':
            if self._record_poly is not None:
                self._record_saved(ok and bool(resp.get('success', False)))
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


def blocked_text(n):
    """Sub_state for stretches the end-of-area pass could not mow (vendor IotNotice
    900027 'obstacle cannot be bypassed')."""
    return '%d stretch%s blocked by obstacles' % (n, '' if n == 1 else 'es')


def blade_allowed(fsm):
    """Invariant used by the tests: may the blade be commanded on right now?"""
    if fsm.phase == 'MANUAL_MOWING':
        return True
    m = fsm.mission
    return fsm.phase == 'MOWING' and m is not None and m.step in ('spinup', 'follow')
