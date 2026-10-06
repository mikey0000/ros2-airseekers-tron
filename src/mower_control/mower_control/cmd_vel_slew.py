#!/usr/bin/env python3
"""Slew-rate limiter for ``/cmd_vel`` with a command watchdog (ROS 2 Humble).

Port of the MowgliNext ``cmd_vel_slew.hpp`` pattern -- see
``ros2_stack/docs/mowglinext_baseline.md`` section 4.3, item 9: shape the
commanded twist *before* it reaches the firmware so the MCU never sees a step
change in linear/angular velocity, and echo the exact applied command for
rosbag analysis.

Subscribed topics
-----------------
``/cmd_vel_raw``   geometry_msgs/Twist          unshaped command in (twist_mux,
                                                     teleop, Nav2, ...)

Published topics
---------------
``/cmd_vel``       geometry_msgs/Twist          slew-limited command out; this
                                                     is what ``mower_mcu_driver``
                                                     subscribes to

Parameters
----------
``max_linear_accel``   ``0.5``    m/s^2 cap on linear.x acceleration
``max_angular_accel``  ``1.0``    rad/s^2 cap on angular.z acceleration
``cmd_timeout``        ``0.5``    s after the last ``/cmd_vel_raw`` the output is
                                  forced to exactly zero (immediate exact stop,
                                  MowgliNext style) and held there until a fresh
                                  command arrives
``rate``               ``20.0``   Hz output/update rate
``frame_id``           ``base_link``  frame stamped on the output
``pivot_assist``       ``true``   creeping pivot (see :class:`PivotAssist`): after
                                  ``pivot_assist_delay_s`` (1.5) of linear 0 / |angular|
                                  >= ``pivot_assist_min_angular_radps`` (0.08) in TRANSIT or
                                  MOWING, add ``pivot_assist_linear_mps`` (0.05) forward, at
                                  most ``pivot_assist_max_dist_m`` (0.3) per pivot
                                  DEFAULT NOW ``false``: subsumed by the turn shaper
``turn_shaper``        ``true``   both-wheel turn shaping (see :func:`shape_for_both_wheels`,
                                  :class:`TurnShaper`) in autonomous phases: inner wheel
                                  >= ``min_inner_wheel_mps`` (0.06) with ``track_width_m``
                                  (0.48, URDF), ``linear_max`` (0.3), ``angular_max``
                                  (0.3): inner wheels in the one-wheel zone snap to an
                                  arc or a proper pivot (|w| >= 0.25);
                                  ``shape_manual`` (false) leaves the joystick untouched.
                                  Decision latched on ``~/shape_status``
``applied_log_period`` ``1.0``    s between ``cmd_vel applied`` log lines
                                  (0 = every tick)

Straight-line shaping (all live-settable, defaults neutral = no behaviour change;
see :class:`DriveShaper`). Applied to the *target* angular.z before slewing:

``angular_trim_radps``     ``0.0``   rad/s added to angular.z while |linear.x| > 0.02,
                                     signed with linear.x (a wheel-torque imbalance
                                     pulls the other way when reversing)
``angular_deadband_radps`` ``0.0``   |angular.z| below this is zeroed while
                                     |linear.x| > 0.02 (joystick drift)
``heading_hold``           ``false`` lock the IMU yaw when the angular command goes to
                                     ~0 while driving (|linear.x| > 0.05) and steer
                                     back onto it
``heading_hold_kp``        ``1.0``   rad/s per rad of yaw error
``heading_hold_kd``        ``0.1``   rad/s per rad/s of gyro z (damping)
``heading_hold_max_radps`` ``0.15``  clamp on the heading-hold correction
``robot_settings_file``    ``$MOWER_ROBOT_SETTINGS_FILE`` or
                           ``/work/config/gui/mowgli_robot.yaml``: the GUI
                                     settings yaml; the keys above found there
                                     override the defaults at startup (the GUI
                                     pushes them live on save; the launch file does
                                     not pass parameters to this node)

Extra input: ``/imu/data`` (sensor_msgs/Imu, sampled once per tick; only used by
heading hold, which releases when the IMU is older than 0.3 s).

Behaviour
---------
* Fixed-rate loop.  Each tick moves the applied linear.x / angular.z at most
  ``accel * dt`` toward the target; every other twist component passes through
  unchanged.
* Every published command is echoed to the log at full float precision (the
  MowgliNext ``/cmd_vel_applied`` idea, as a log line instead of a topic),
  throttled to ``applied_log_period`` (default 1 s; 0 = every tick, the old
  20 Hz echo). Silence with ``--ros-args --log-level cmd_vel_slew:=warn``.
* Watchdog: when no ``/cmd_vel_raw`` has arrived within ``cmd_timeout``, the
  applied velocity is set to exactly 0.0 (not slewed) and zero is published
  every tick until a fresh command arrives; recovery then slews up from zero.

CPU (RK3588): ``/cmd_vel_raw`` is a sampled input of a ``SubscriptionPump``
read once per tick with its receive time (no executor wake per message), the
loop runs on a ``PeriodicRunner`` thread instead of an rclpy timer, and
``/cmd_vel`` is published as pre-serialized Twist bytes. Idle cost went from
~12 % of a core to ~1 %.
"""

import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu

from mower_control.fast_msgs import parse_twist, twist_bytes
from mower_control.sub_pump import PeriodicRunner, SubscriptionPump, parse_imu


def slew(current, target, max_accel, dt):
    """Move ``current`` toward ``target`` by at most ``max_accel * dt``."""
    delta = target - current
    max_delta = max_accel * dt
    if abs(delta) <= max_delta:
        return target
    return current + math.copysign(max_delta, delta)


def wrap_angle(a):
    """``a`` wrapped to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# Straight-line shaping parameters: name -> (default, type). The names double as
# the GUI settings keys (mowgli_robot.yaml; gui pkg/api/param_bindings.go).
SHAPER_PARAMS = {
    'angular_trim_radps': (0.0, float),
    'angular_deadband_radps': (0.0, float),
    'heading_hold': (False, bool),
    'heading_hold_kp': (1.0, float),
    'heading_hold_kd': (0.1, float),
    'heading_hold_max_radps': (0.15, float),
}

TRIM_MIN_LINEAR = 0.02      # m/s: trim / deadband only while actually driving
HOLD_MIN_LINEAR = 0.05      # m/s: heading hold only above this
HOLD_ZERO_ANGULAR = 1e-3    # rad/s: "zero" angular command when no deadband is set
IMU_MAX_AGE = 0.3           # s: heading hold releases on an older IMU sample


class DriveShaper:
    """Trim, deadband and heading hold on the commanded angular.z (pure logic)."""

    def __init__(self, **params):
        for name, (default, _kind) in SHAPER_PARAMS.items():
            setattr(self, name, params.get(name, default))
        self.yaw_ref = None     # locked yaw while heading hold is engaged

    @property
    def holding(self):
        return self.yaw_ref is not None

    def reset(self):
        self.yaw_ref = None

    def shape(self, lin, ang, yaw=None, gyro_z=0.0):
        """Shaped angular.z for target (lin, ang); ``yaw`` None = no fresh IMU."""
        driving = abs(lin) > TRIM_MIN_LINEAR
        deadband = max(0.0, self.angular_deadband_radps)
        straight = abs(ang) < (deadband if deadband > 0.0 else HOLD_ZERO_ANGULAR)
        if driving and deadband > 0.0 and abs(ang) < deadband:
            ang = 0.0
        if (self.heading_hold and straight and abs(lin) > HOLD_MIN_LINEAR
                and yaw is not None):
            if self.yaw_ref is None:
                self.yaw_ref = yaw
            limit = abs(self.heading_hold_max_radps)
            corr = (self.heading_hold_kp * wrap_angle(self.yaw_ref - yaw)
                    - self.heading_hold_kd * gyro_z)
            ang = max(-limit, min(limit, corr))
        else:
            self.yaw_ref = None
        if driving and self.angular_trim_radps:
            ang += math.copysign(self.angular_trim_radps, lin)
        return ang


# --------------------------------------------------------------------------- pivot assist
PIVOT_ASSIST_PHASES = frozenset(('TRANSIT', 'MOWING'))


class PivotAssist:
    """Vendor-like "creeping pivot" (pure logic).

    The Tron's casters on turf make in-place pivots unreliable (measured 0.02-0.08 rad/s
    yaw at 0.3 rad/s commanded; a forward arc reaches ~0.2, docs/analysis/
    2026-10-06_ring_drift.md). Humble RPP's rotate-to-heading commands linear 0, so after
    ``delay_s`` of a pure pivot command (|angular| >= ``min_angular``, linear == 0) in a
    TRANSIT/MOWING phase this adds ``linear`` m/s forward, at most ``max_dist_m`` of
    commanded travel per pivot (the pivot ends when the command stops being a pivot).
    Lawn-safe: 0.05 m/s x 6 s at most; the costmap / controller still checks collisions.
    """

    # min_angular 0.08 (2026-10-07 live): RPP's rotate-to-heading only ramped to 0.16 rad/s
    # (0.086 achieved) and the old 0.2 threshold never fired; 0.08 completed the turn.
    def __init__(self, enabled=True, linear=0.05, min_angular=0.08, delay_s=1.5,
                 max_dist_m=0.3):
        self.enabled = bool(enabled)
        self.linear = float(linear)
        self.min_angular = float(min_angular)
        self.delay_s = float(delay_s)
        self.max_dist_m = float(max_dist_m)
        self.reset()

    def reset(self):
        self.pivot_s = 0.0        # time the current pivot command has lasted
        self.dist_m = 0.0         # forward creep commanded during it
        self.active = False

    def apply(self, lin, ang, phase, dt):
        """Target linear.x for raw target (lin, ang) in mission ``phase``."""
        pivot = (self.enabled and phase in PIVOT_ASSIST_PHASES and abs(lin) < 1e-3
                 and abs(ang) >= self.min_angular)
        if not pivot:
            self.reset()
            return lin
        self.pivot_s += dt
        if self.pivot_s <= self.delay_s or self.dist_m >= self.max_dist_m:
            self.active = False
            return lin
        self.active = True
        self.dist_m += self.linear * dt
        return self.linear


PIVOT_PARAMS = {   # node parameter -> PivotAssist kwarg
    'pivot_assist': 'enabled',
    'pivot_assist_linear_mps': 'linear',
    'pivot_assist_min_angular_radps': 'min_angular',
    'pivot_assist_delay_s': 'delay_s',
    'pivot_assist_max_dist_m': 'max_dist_m',
}



# --------------------------------------------------------------------------- both-wheel turn shaper
# Owner rules (2026-10-07): "all turns should require both wheels turning at different
# speed" -- a pivot with counter-rotating wheels IS both wheels turning; what is forbidden
# is the one-wheel zone, an inner wheel crawling at 0 <= |v_inner| < min_inner_wheel_mps
# (the MCU deadbands it, so the robot pivots on one stalled wheel and scrubs the turf).
# The MCU mixes (linear, angular) itself (docs/wheel_control_semantics.md); we snap a
# command whose inner wheel would fall in (-min_inner, +min_inner) to the NEARER legal
# regime: a proper pivot (v = 0, inner <= -min_inner, |w| >= 2*min_inner/b = 0.25 rad/s)
# or an arc (inner rolling the same way at >= min_inner).
TRACK_WIDTH_M = 0.48          # 2 x drive_track_y 0.24 (config/urdf/mower.urdf.xacro, vendor)
# Autonomous DRIVING phases only. Docking/undocking are exempt: the marker-guided
# reverse (-0.05 m/s with small steering) sits in the one-wheel zone and was being
# snapped to v=0 pivots, so the robot pushed nowhere and the stall guard fired at
# the contacts (2026-10-07). MANUAL_MOWING/RECORDING untouched.
SHAPE_PHASES = frozenset(('TRANSIT', 'MOWING', 'BOUNDARY_PAUSED'))
MANUAL_PHASES = frozenset(('MANUAL_MOWING', 'RECORDING'))
SHAPE_DEFAULTS = {
    'track_width_m': TRACK_WIDTH_M,
    'min_inner_wheel_mps': 0.06,
    'linear_max': 0.3,
    'angular_max': 0.3,       # MCU clamp
}


def shape_for_both_wheels(v, w, params=None):
    """Keep the inner wheel out of the one-wheel zone (-min_inner, +min_inner).

    The signed inner wheel speed (in the direction of travel) is ``|v| - |w|*b/2``.
    Returns ``(v, w, info)``; ``info`` is None when the command passes unchanged, else a
    dict ``{'kind': 'pivot'|'arc', 'r': radius_m, 'inner': inner_mps, 'outer': outer_mps}``.

    * inner >= +min_inner (arc) or <= -min_inner (proper counter-rotating pivot): pass.
    * 0 < inner < min_inner: widen to the tightest arc keeping the inner wheel at
      min_inner (keeping |w| when possible, else lowering |w| so outer <= linear_max);
      reversing stays reversing.
    * -min_inner < inner <= 0 (incl. slow pure pivots): snap to a pure pivot, v = 0,
      |w| = max(|w|, 2*min_inner/b) capped at angular_max.
    """
    p = dict(SHAPE_DEFAULTS)
    if params:
        p.update(params)
    half = 0.5 * float(p['track_width_m'])
    v_min = float(p['min_inner_wheel_mps'])
    v_max = float(p['linear_max'])
    w_max = float(p['angular_max'])
    if abs(w) < 1e-6:
        return v, w, None                     # straight (or stop): both wheels equal
    inner = abs(v) - abs(w) * half
    if inner >= v_min - 1e-9 or inner <= -v_min + 1e-9:
        return v, w, None                     # proper arc or proper pivot
    if abs(v) < 1e-6:
        # A pure pivot: both wheels counter-rotate whatever the rate. Never bump it:
        # the controller's small heading corrections near a goal became 0.25 rad/s
        # pulses and the robot hunted around the heading (2026-10-07).
        return v, w, None
    if inner <= 0.0:                          # one-wheel zone with some forward speed
        aw = min(max(abs(w), v_min / half), w_max)
        info = {'kind': 'pivot', 'r': 0.0, 'inner': -aw * half, 'outer': aw * half}
        return 0.0, math.copysign(aw, w), info
    sign = -1.0 if v < 0.0 else 1.0
    aw = abs(w)
    av = v_min + aw * half
    if av + aw * half > v_max:                # outer wheel would exceed linear_max
        aw = max(0.0, (v_max - v_min) / (2.0 * half))
        av = 0.5 * (v_max + v_min)
    info = {'kind': 'arc', 'r': av / aw if aw > 0.0 else float('inf'),
            'inner': av - aw * half, 'outer': av + aw * half}
    return sign * av, math.copysign(aw, w), info


class TurnShaper:
    """Stateful wrapper: phase gating + latched status. Manual phases pass unless
    ``shape_manual``."""

    def __init__(self, enabled=True, shape_manual=False, **params):
        self.enabled = bool(enabled)
        self.shape_manual = bool(shape_manual)
        self.params = dict(SHAPE_DEFAULTS)
        self.params.update(params)
        self.reset()

    def reset(self):
        self.status = 'pass'

    def apply(self, v, w, phase, dt):
        """Return the shaped (v, w) for raw target (v, w) in mission ``phase``."""
        active = self.enabled and (phase in SHAPE_PHASES or
                                   (self.shape_manual and phase in MANUAL_PHASES))
        if not active:
            self.reset()
            return v, w
        sv, sw, info = shape_for_both_wheels(v, w, self.params)
        if info is None:
            self.status = 'pass'
            return v, w
        if info['kind'] == 'pivot':
            self.status = 'pivot w=%.2f rad/s, inner %.2f m/s' % (abs(sw), info['inner'])
        else:
            self.status = 'arc r=%.2f m, inner %.2f m/s' % (info['r'], info['inner'])
        return sv, sw


TURN_PARAMS = {   # node parameter -> (TurnShaper attribute / params key, type)
    'turn_shaper': ('enabled', bool),
    'shape_manual': ('shape_manual', bool),
    'track_width_m': ('track_width_m', float),
    'min_inner_wheel_mps': ('min_inner_wheel_mps', float),
    'linear_max': ('linear_max', float),
    'angular_max': ('angular_max', float),
}
TURN_DEFAULTS = {'turn_shaper': True, 'shape_manual': False, 'track_width_m': TRACK_WIDTH_M,
                 'min_inner_wheel_mps': 0.06, 'linear_max': 0.3, 'angular_max': 0.3}
SHAPE_LOG_PERIOD_S = 2.0      # turn-shaper INFO log: on state-kind change, at most every 2 s


def set_turn_param(shaper, name, value):
    attr, kind = TURN_PARAMS[name]
    if attr in SHAPE_DEFAULTS:
        shaper.params[attr] = kind(value)
    else:
        setattr(shaper, attr, kind(value))


# --------------------------------------------------------------------------- motion gate
# Mirrors mower_mission.mission_fsm.MOTION_PHASES / DOCKED_MOTION_PHASES (kept local so
# mower_control does not depend on mower_mission).
MOTION_PHASES = frozenset((
    'MANUAL_MOWING', 'RECORDING', 'UNDOCKING', 'WAITING_FOR_RTK', 'PLANNING', 'TRANSIT',
    'MOWING', 'AREA_UNREACHABLE', 'BOUNDARY_PAUSED', 'RETURNING_HOME',
    'LOW_BATTERY_DOCKING', 'RAIN_DETECTED_DOCKING', 'COVERAGE_FAILED_DOCKING'))
DOCKED_MOTION_PHASES = frozenset((
    'UNDOCKING', 'MANUAL_MOWING', 'RETURNING_HOME', 'LOW_BATTERY_DOCKING', 'RAIN_DETECTED_DOCKING',
    'COVERAGE_FAILED_DOCKING'))
STUCK_REASON = 'stuck: wheels spinning'
MOTION_ENABLE_MAX_AGE = 3.0   # s: the mission re-asserts /motion_enabled every 1 s


class MotionGate:
    """Last-line wheel interlock (pure logic; times are monotonic seconds).

    Motion is allowed only when ALL hold:
      * the mission's latched ``/motion_enabled`` is true and fresh (<= 3 s);
      * the mission state (``high_level_status.state_name``) is a motion phase and
        ``emergency`` is false;
      * ``/mower_base/status`` does not report stop_triggered / lift_triggered, and,
        when it reports docked/charging, the mission is undocking or docking.
    Unknown inputs (nothing received yet) mean "not allowed".

    ``enabled_since`` is the time the gate last opened: commands received before it
    are stale and must never be released (lanes have to re-assert after a reset).
    """

    def __init__(self):
        self.motion_enabled = None      # (bool, receipt)
        self.state_name = None
        self.emergency = True
        self.base = None                # dict of the MowerBaseDevStatus flags we use
        self.allowed = False
        self.enabled_since = None
        self.reason = 'no inputs'
        self.stuck = False              # /stuck (slip_detector stuck guard) latched true

    def set_stuck(self, value):
        self.stuck = bool(value)

    def set_motion_enabled(self, value, now):
        self.motion_enabled = (bool(value), now)

    def set_status(self, state_name, emergency):
        self.state_name = str(state_name)
        self.emergency = bool(emergency)

    def set_base(self, docked, charging, stop, lift):
        self.base = {'docked': bool(docked), 'charging': bool(charging),
                     'stop': bool(stop), 'lift': bool(lift)}

    def _why_not(self, now):
        if self.stuck:
            # Stuck guard (2026-10-07 hole-digging incident): stop the wheels whatever the
            # controller wants; the mission's escape runs with the guard suppressed.
            return STUCK_REASON
        me = self.motion_enabled
        if me is None:
            return '/motion_enabled not received'
        if not me[0]:
            return 'motion disabled by mission'
        if now - me[1] > MOTION_ENABLE_MAX_AGE:
            return '/motion_enabled stale'
        if self.state_name is None:
            return 'no mission status'
        if self.emergency:
            return 'mission emergency'
        if self.state_name not in MOTION_PHASES:
            return 'state %s' % self.state_name
        b = self.base
        if b is None:
            return 'no /mower_base/status'
        if b['stop']:
            return 'stop button / estop'
        if b['lift']:
            return 'lifted'
        if (b['docked'] or b['charging']) and self.state_name not in DOCKED_MOTION_PHASES:
            return 'docked'
        return ''

    def update(self, now):
        """Re-evaluate; returns (allowed, changed)."""
        why = self._why_not(now)
        allowed = not why
        changed = allowed != self.allowed
        if changed and allowed:
            self.enabled_since = now
        self.allowed = allowed
        self.reason = why or 'allowed'
        return allowed, changed


def default_settings_file():
    return os.environ.get('MOWER_ROBOT_SETTINGS_FILE', '/work/config/gui/mowgli_robot.yaml')


def settings_overrides(path):
    """{param: value} of the SHAPER_PARAMS keys found in the GUI settings yaml."""
    if not path:
        return {}
    try:
        import yaml
        with open(path, encoding='utf-8') as f:
            doc = yaml.safe_load(f) or {}
    except Exception:  # noqa: BLE001 - missing/unreadable file: defaults apply
        return {}
    flat = {}
    if isinstance(doc, dict):
        for section in doc.values():
            params = section.get('ros__parameters') if isinstance(section, dict) else None
            if isinstance(params, dict):
                flat.update(params)
    out = {}
    for name, (_default, kind) in SHAPER_PARAMS.items():
        value = flat.get(name)
        if kind is bool and isinstance(value, bool):
            out[name] = value
        elif kind is float and isinstance(value, (int, float)) and not isinstance(value, bool):
            out[name] = float(value)
    return out


class CmdVelSlewNode(Node):
    """Slew-limits ``/cmd_vel_raw`` -> ``/cmd_vel`` and enforces the watchdog."""

    def __init__(self):
        super().__init__('cmd_vel_slew')

        self.declare_parameter('max_linear_accel', 0.5)
        self.declare_parameter('max_angular_accel', 1.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('applied_log_period', 1.0)
        self.declare_parameter('robot_settings_file', default_settings_file())
        for name, (default, _kind) in SHAPER_PARAMS.items():
            self.declare_parameter(name, default)
        _pa = PivotAssist(enabled=False)   # subsumed by the TurnShaper (no double creep)
        for name, attr in PIVOT_PARAMS.items():
            self.declare_parameter(name, getattr(_pa, attr))
        for name, default in TURN_DEFAULTS.items():
            self.declare_parameter(name, default)
        self._turn = TurnShaper()
        for name in TURN_PARAMS:
            set_turn_param(self._turn, name, self.get_parameter(name).value)
        self._pivot = PivotAssist(**{attr: self.get_parameter(name).value
                                     for name, attr in PIVOT_PARAMS.items()})
        overrides = settings_overrides(str(self.get_parameter('robot_settings_file').value))
        if overrides:
            self.set_parameters([Parameter(k, value=v)
                                 for k, v in overrides.items()])
        self._shaper = DriveShaper(**{n: self.get_parameter(n).value for n in SHAPER_PARAMS})
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self._imu = None              # (yaw, gyro_z) of the newest /imu/data
        self._imu_time = None

        self._max_linear_accel = float(self.get_parameter('max_linear_accel').value)
        self._max_angular_accel = float(self.get_parameter('max_angular_accel').value)
        self._cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self._rate = float(self.get_parameter('rate').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._log_period = float(self.get_parameter('applied_log_period').value)

        # Applied (slewed) state, the last raw target, watchdog bookkeeping (monotonic s).
        self._cur_lin = 0.0
        self._cur_ang = 0.0
        self._tgt = None              # (lx, ly, lz, ax, ay, az) of the last /cmd_vel_raw
        self._last_cmd_time = None
        self._last_tick = None
        self._next_log = 0.0

        qos = QoSProfile(depth=10,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST)
        # Sampled latest-value input: only the newest command matters each tick, and its
        # receive time (not the poll time) feeds the watchdog.
        self._pump = SubscriptionPump(self, 'cmd_vel_slew_inputs')
        self._pump.subscribe(Twist, '/cmd_vel_raw', self._on_cmd_vel_raw,
                             QoSProfile(depth=1,
                                        reliability=QoSReliabilityPolicy.RELIABLE,
                                        history=QoSHistoryPolicy.KEEP_LAST),
                             parser=parse_twist, sampled=True, with_receipt=True)
        self._pump.subscribe(Imu, '/imu/data', self._on_imu,
                             QoSProfile(depth=1,
                                        reliability=QoSReliabilityPolicy.RELIABLE,
                                        history=QoSHistoryPolicy.KEEP_LAST),
                             parser=parse_imu, sampled=True, with_receipt=True)
        # Motion gate inputs (see MotionGate). /mower_base/status is 100 Hz: sampled.
        from std_msgs.msg import Bool
        from rclpy.qos import QoSDurabilityPolicy
        from mower_interfaces.msg import MowerBaseDevStatus
        from mowgli_interfaces.msg import HighLevelStatus
        from mower_control.sub_pump import flat_parser
        self._gate = MotionGate()
        self._pump.subscribe(MowerBaseDevStatus, '/mower_base/status', self._on_base,
                             QoSProfile(depth=1,
                                        reliability=QoSReliabilityPolicy.RELIABLE,
                                        history=QoSHistoryPolicy.KEEP_LAST),
                             parser=flat_parser(MowerBaseDevStatus), sampled=True)
        self._pump.subscribe(HighLevelStatus, '/behavior_tree_node/high_level_status',
                             self._on_hl, QoSProfile(depth=10,
                                                     reliability=QoSReliabilityPolicy.RELIABLE,
                                                     history=QoSHistoryPolicy.KEEP_LAST))
        self._pump.subscribe(Bool, '/motion_enabled', self._on_motion_enabled,
                             QoSProfile(depth=1,
                                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=QoSReliabilityPolicy.RELIABLE,
                                        history=QoSHistoryPolicy.KEEP_LAST),
                             sampled=True, with_receipt=True)
        self._pump.subscribe(Bool, '/stuck', self._on_stuck,
                             QoSProfile(depth=1,
                                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=QoSReliabilityPolicy.RELIABLE,
                                        history=QoSHistoryPolicy.KEEP_LAST),
                             parser=flat_parser(Bool), sampled=True)
        self._pump.start()
        self._pub = self.create_publisher(Twist, '/cmd_vel', qos)
        from std_msgs.msg import String
        self._gate_pub = self.create_publisher(
            String, '~/gate_status',
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       history=QoSHistoryPolicy.KEEP_LAST))

        self._shape_pub = self.create_publisher(
            String, '~/shape_status',
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       history=QoSHistoryPolicy.KEEP_LAST))

        self._runner = PeriodicRunner(self, [(1.0 / self._rate, self._tick)], 'cmd_vel_slew')
        self._runner.start()
        self.get_logger().info(
            'cmd_vel_slew ready: max_linear_accel=%.3f m/s^2, '
            'max_angular_accel=%.3f rad/s^2, cmd_timeout=%.3f s, rate=%.1f Hz'
            % (self._max_linear_accel, self._max_angular_accel,
               self._cmd_timeout, self._rate))
        self.get_logger().info('cmd_vel_slew shaping: %s%s' % (
            {n: getattr(self._shaper, n) for n in SHAPER_PARAMS},
            ' (from %s)' % sorted(overrides) if overrides else ''))

    def shutdown(self):
        self._runner.stop()
        self._pump.stop()

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def _on_cmd_vel_raw(self, msg, receipt):
        """Store the raw target; the periodic loop does the slewing/publishing."""
        a, b = msg.linear, msg.angular
        self._tgt = (a.x, a.y, a.z, b.x, b.y, b.z)
        self._last_cmd_time = receipt

    def _on_base(self, msg):
        self._gate.set_base(msg.is_docking_done, msg.is_charging,
                            msg.stop_triggered, msg.lift_triggered)

    def _on_hl(self, msg):
        self._gate.set_status(msg.state_name, msg.emergency)

    def _on_stuck(self, msg):
        self._gate.set_stuck(msg.data)

    def _on_motion_enabled(self, msg, receipt):
        self._gate.set_motion_enabled(msg.data, receipt)

    def _on_imu(self, msg, receipt):
        q = msg.orientation
        self._imu = (yaw_from_quaternion(q.x, q.y, q.z, q.w), msg.angular_velocity.z)
        self._imu_time = receipt

    def _on_set_parameters(self, params):
        for p in params:
            if p.name in TURN_PARAMS and getattr(self, '_turn', None) is not None:
                set_turn_param(self._turn, p.name, p.value)
                self.get_logger().info('cmd_vel_slew: %s = %r' % (p.name, p.value))
        for p in params:
            if p.name in PIVOT_PARAMS:
                setattr(self._pivot, PIVOT_PARAMS[p.name],
                        bool(p.value) if p.name == 'pivot_assist' else float(p.value))
                self.get_logger().info('cmd_vel_slew: %s = %r' % (p.name, p.value))
        for p in params:
            if p.name not in SHAPER_PARAMS:
                continue
            kind = SHAPER_PARAMS[p.name][1]
            value = p.value
            if kind is float and isinstance(value, int) and not isinstance(value, bool):
                value = float(value)
            if not isinstance(value, kind) or (kind is float and not math.isfinite(value)):
                return SetParametersResult(successful=False,
                                           reason='%s must be a %s' % (p.name, kind.__name__))
        for p in params:
            if p.name in SHAPER_PARAMS:
                kind = SHAPER_PARAMS[p.name][1]
                setattr(self._shaper, p.name, kind(p.value))
                if p.name == 'heading_hold' and not p.value:
                    self._shaper.reset()
                self.get_logger().info('cmd_vel_slew: %s = %r' % (p.name, p.value))
        return SetParametersResult(successful=True)

    def _tick(self):
        self._pump.poll()
        out = self.step(time.monotonic())
        if out is not None:
            self._pub.publish(twist_bytes(*out))

    def step(self, now):
        """One loop iteration at monotonic time ``now``; returns the Twist tuple to publish."""
        if self._last_tick is None:
            self._last_tick = now
            return None
        dt = now - self._last_tick
        self._last_tick = now
        if dt <= 0.0:
            return None

        stale = (self._last_cmd_time is None or
                 now - self._last_cmd_time > self._cmd_timeout)

        gate = getattr(self, '_gate', None)
        if gate is not None:
            allowed, changed = gate.update(now)
            if changed:
                self.get_logger().warning('motion gate %s (%s)' % (
                    'OPEN' if allowed else 'CLOSED', gate.reason))
            status = '%s: %s' % ('OPEN' if allowed else 'CLOSED', gate.reason)
            if status != getattr(self, '_gate_status', None):
                self._gate_status = status
                pub = getattr(self, '_gate_pub', None)
                if pub is not None:
                    from std_msgs.msg import String
                    pub.publish(String(data=status))
            if not allowed:
                tgt = self._tgt
                if (tgt is not None and (tgt[0] != 0.0 or tgt[5] != 0.0)
                        and now >= getattr(self, '_next_drop_log', 0.0)):
                    self._next_drop_log = now + 2.0
                    self.get_logger().warning(
                        'motion gate CLOSED (%s): dropping cmd linear.x=%r angular.z=%r'
                        % (gate.reason, tgt[0], tgt[5]))
                # Forced zero; drop the held command so nothing stale survives.
                self._tgt = None
                self._last_cmd_time = None
                stale = True
            elif self._last_cmd_time is not None and self._last_cmd_time < gate.enabled_since:
                # Command predates the gate opening: never release it.
                stale = True

        if stale:
            # Immediate exact stop (MowgliNext cmd_vel_slew): no slewing to zero.
            self._cur_lin = 0.0
            self._cur_ang = 0.0
            tgt_lin, tgt_ang = 0.0, 0.0
            if getattr(self, '_shaper', None) is not None:
                self._shaper.reset()
            if getattr(self, '_pivot', None) is not None:
                self._pivot.reset()
            if getattr(self, '_turn', None) is not None:
                self._turn.reset()
                CmdVelSlewNode._publish_shape_status(self)
        else:
            tgt_lin = self._tgt[0]
            tgt_ang = self._tgt[5]
            shaper = getattr(self, '_shaper', None)
            if shaper is not None:
                imu_time = getattr(self, '_imu_time', None)
                fresh = imu_time is not None and now - imu_time <= IMU_MAX_AGE
                yaw, gyro_z = self._imu if fresh else (None, 0.0)
                was = shaper.holding
                tgt_ang = shaper.shape(tgt_lin, tgt_ang, yaw, gyro_z)
                if shaper.holding != was:
                    self.get_logger().info('heading hold %s (yaw_ref=%s)' % (
                        'engaged' if shaper.holding else 'released', shaper.yaw_ref))
            pivot = getattr(self, '_pivot', None)
            if pivot is not None:
                gate = getattr(self, '_gate', None)
                was = pivot.active
                tgt_lin = pivot.apply(tgt_lin, tgt_ang,
                                      gate.state_name if gate is not None else None, dt)
                if pivot.active != was:
                    self.get_logger().info(
                        'pivot assist %s (%.2f m crept, pivot %.1f s)' % (
                            'ON: creeping forward' if pivot.active else 'off',
                            pivot.dist_m, pivot.pivot_s))
            tgt_lin, tgt_ang = CmdVelSlewNode._shape_turn(self, tgt_lin, tgt_ang, dt)
            self._cur_lin = slew(self._cur_lin, tgt_lin, self._max_linear_accel, dt)
            self._cur_ang = slew(self._cur_ang, tgt_ang, self._max_angular_accel, dt)

        # Pass through every component we do not shape.
        src = self._tgt if self._tgt is not None else (0.0,) * 6
        out = (self._cur_lin, src[1], src[2], src[3], src[4], self._cur_ang)

        # Exact echo: full float precision, throttled (see applied_log_period) and
        # only while something is commanded: an idle robot said "0.0 / 0.0" once a
        # second and that line was ~85 % of /rosout.
        if now >= self._next_log and (self._cur_lin or self._cur_ang or tgt_lin or tgt_ang):
            self._next_log = now + self._log_period
            self.get_logger().info(
                'cmd_vel applied: linear.x={!r} angular.z={!r} '
                '(target linear.x={!r} angular.z={!r})'.format(
                    self._cur_lin, self._cur_ang, tgt_lin, tgt_ang))
        return out

    def _shape_turn(self, lin, ang, dt):
        """Both-wheel turn shaping (see :class:`TurnShaper`); publishes shape_status."""
        turn = getattr(self, '_turn', None)
        if turn is None:
            return lin, ang
        gate = getattr(self, '_gate', None)
        lin, ang = turn.apply(lin, ang, gate.state_name if gate is not None else None, dt)
        CmdVelSlewNode._publish_shape_status(self)
        return lin, ang

    def _publish_shape_status(self):
        status = self._turn.status
        if status != getattr(self, '_shape_status', None):
            self._shape_status = status
            # Log only when the regime (pass/arc/pivot) changes, at most every 2 s: the
            # radius changes every 20 Hz tick and flooded the log.
            kind = status.split(' ', 1)[0]
            now = time.monotonic()
            if (kind != getattr(self, '_shape_log_kind', 'pass')
                    and now >= getattr(self, '_shape_log_next', 0.0)):
                self._shape_log_kind = kind
                self._shape_log_next = now + SHAPE_LOG_PERIOD_S
                self.get_logger().info('turn shaper: %s' % status)
            pub = getattr(self, '_shape_pub', None)
            if pub is not None:
                from std_msgs.msg import String
                pub.publish(String(data=status))

    _slew = staticmethod(slew)


def main(args=None):
    try:  # crash records -> /userdata/ros2/crashes (docs/crash_recovery.md)
        from mower_control.crash_record import install as _install_crash_record
        _install_crash_record('cmd_vel_slew')
    except ImportError:
        pass
    rclpy.init(args=args)
    node = CmdVelSlewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
