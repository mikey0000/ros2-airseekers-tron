# Copyright 2026 The mower_docking authors
# SPDX-License-Identifier: Apache-2.0
"""Pure docking / undocking logic for the Airseekers Tron (no ROS, no cv2).

Everything in here is deterministic and driven by an injected sensor
snapshot so it can be unit-tested without rclpy:

* geometry helpers (yaw wrap, approach pose, rpy -> rotation matrix,
  Rodrigues, camera-frame ArUco pose -> base_link marker observation);
* :class:`ContactDebouncer` (vendor ``is_dock_done_num: 3``);
* :class:`DistanceTracker` (odom-integrated travel with a time * speed
  fallback while odom is missing or stale);
* :func:`reverse_control` (heading/lateral PD for reversing onto the dock);
* :class:`DockStateMachine` and :class:`UndockStateMachine`.

Frame conventions (see docs/audit_2026-10-05/vendor_mission_layer.md s3/s5):

* ``map`` origin = rear-axle centre (``base_link``) of the robot sitting on
  the charger, X forward.  The docked robot faces AWAY from the charger, so it
  reverses (negative ``linear.x``) onto the contacts and leaves forwards.
* A marker observation is expressed in ``base_link``: ``x, y`` is the marker
  centre and ``yaw`` is the heading of the marker's outward normal (its +Z
  axis) projected into the ground plane.  With the robot perfectly aligned in
  front of the dock the marker is behind it (x < 0) and its normal points
  back at the robot, i.e. ``yaw == 0``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

# --------------------------------------------------------------------------
# small maths helpers (pure python; numpy only where a 3x3 matrix is handy)
# --------------------------------------------------------------------------


def wrap_angle(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def approach_pose(dock: Pose2D, distance: float) -> Pose2D:
    """Approach pose ``distance`` metres in front of the dock.

    ``dock`` is the base_link pose of a robot sitting on the charger (it
    faces away from the charger).  The approach pose lies along that same
    heading and keeps the same yaw, so the robot then only has to reverse
    straight back onto the contacts.  Vendor: dock at origin ->
    ``undock_point`` (0.8, 0) heading +X.
    """
    return Pose2D(dock.x + distance * math.cos(dock.yaw),
                  dock.y + distance * math.sin(dock.yaw),
                  wrap_angle(dock.yaw))


def rpy_to_matrix(roll: float, pitch: float, yaw: float):
    """URDF/ROS fixed-axis rpy -> 3x3 rotation (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def rodrigues(rvec: Sequence[float]):
    """Axis-angle vector -> 3x3 rotation (same convention as cv2.Rodrigues)."""
    rx, ry, rz = (float(v) for v in rvec)
    th = math.sqrt(rx * rx + ry * ry + rz * rz)
    if th < 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    kx, ky, kz = rx / th, ry / th, rz / th
    c, s, v = math.cos(th), math.sin(th), 1.0 - math.cos(th)
    return [
        [c + kx * kx * v, kx * ky * v - kz * s, kx * kz * v + ky * s],
        [ky * kx * v + kz * s, c + ky * ky * v, ky * kz * v - kx * s],
        [kz * kx * v - ky * s, kz * ky * v + kx * s, c + kz * kz * v],
    ]


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _matvec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


@dataclass(frozen=True)
class MarkerObs:
    """Marker observation in base_link (see module docstring)."""
    x: float
    y: float
    yaw: float
    stamp: float = 0.0
    marker_id: int = -1


def marker_in_base(rvec: Sequence[float], tvec: Sequence[float],
                   cam_xyz: Sequence[float], cam_rpy: Sequence[float],
                   stamp: float = 0.0, marker_id: int = -1) -> MarkerObs:
    """Convert an OpenCV marker pose (marker in the camera OPTICAL frame,
    i.e. solvePnP ``rvec``/``tvec``) into a base_link :class:`MarkerObs`.

    ``cam_xyz``/``cam_rpy`` is the pose of the camera optical frame in
    base_link, exactly the URDF ``rear_camera_joint`` origin
    (``-0.201 0 0.25`` / ``-1.5708 0 1.5708``).
    """
    r_bc = rpy_to_matrix(*cam_rpy)
    r_cm = rodrigues(rvec)
    p = _matvec(r_bc, [float(t) for t in tvec])
    p = [p[i] + float(cam_xyz[i]) for i in range(3)]
    r_bm = _matmul(r_bc, r_cm)
    nx, ny = r_bm[0][2], r_bm[1][2]  # marker +Z axis (outward normal)
    return MarkerObs(p[0], p[1], math.atan2(ny, nx), stamp, marker_id)


@dataclass(frozen=True)
class DockErrors:
    remaining: float  # metres still to reverse along the dock axis (>0 = not there)
    lateral: float    # robot offset from the dock axis (left positive)
    heading: float    # robot heading relative to the docked heading


def dock_errors(marker: MarkerObs, docked_offset: float) -> DockErrors:
    """Errors of the robot (base_link origin) w.r.t. the docked pose.

    The docked base_link origin lies ``docked_offset`` metres from the marker
    along its outward normal, with the docked heading equal to the normal's
    heading.
    """
    tx = marker.x + docked_offset * math.cos(marker.yaw)
    ty = marker.y + docked_offset * math.sin(marker.yaw)
    c, s = math.cos(-marker.yaw), math.sin(-marker.yaw)
    rx, ry = -tx, -ty
    return DockErrors(remaining=c * rx - s * ry,
                      lateral=s * rx + c * ry,
                      heading=wrap_angle(-marker.yaw))


def marker_gate_ok(err: DockErrors, max_lateral: float, max_yaw_rad: float) -> bool:
    """Vendor: reject a marker estimate worse than 0.5 m / 25 deg."""
    return abs(err.lateral) <= max_lateral and abs(err.heading) <= max_yaw_rad


@dataclass
class ControllerGains:
    max_speed: float = 0.15
    final_speed: float = 0.05
    slow_distance: float = 0.3      # remaining <= this -> final_speed
    slow_zone: float = 0.3          # ramp max_speed -> final_speed over this, beyond slow_distance
    final_zone: float = 0.2         # remaining <= this: realign check; a marker lost in here ->
    #                                 straight reverse on the last heading (vendor 0.2)
    close_speed: float = 0.08       # m/s max reverse speed within slow_distance (steering on)
    lateral_slow: float = 0.20      # m lateral error at which the close-zone speed hits the floor
    k_lateral: float = 7.0          # rad of desired heading per metre of lateral error
    max_approach_angle: float = 0.5
    kp_heading: float = 1.5
    kd_heading: float = 0.0
    max_angular: float = 0.3
    # Turn-first: scale the reverse speed down with the heading error so a robot that
    # enters DOCKING at the marker gate (25 deg / 0.5 m) turns onto the axis within the
    # short run to the contacts instead of driving further off it. A creep floor keeps
    # the wheels rolling (an in-place pivot does not turn on turf, ring-drift analysis).
    turn_slow_err: float = 0.5      # rad of heading-tracking error at which v hits the floor
    min_turn_speed: float = 0.035   # m/s reverse speed floor (> stall_min_cmd 0.03, so the
    #                                 stall guard stays armed while creeping)


def reverse_speed(remaining: float, g: ControllerGains, near: Optional[float] = None) -> float:
    """Reverse speed magnitude: close_speed within ``slow_distance`` of the
    contacts, ramping up to max_speed over ``slow_zone`` beyond it."""
    frac = clamp((remaining - g.slow_distance) / max(g.slow_zone, 1e-6), 0.0, 1.0)
    near = g.close_speed if near is None else near
    return near + (g.max_speed - near) * frac


def reverse_control(err: DockErrors, g: ControllerGains,
                    prev_heading_err: Optional[float] = None,
                    dt: float = 0.05) -> Tuple[float, float, float]:
    """Heading/lateral PD for reversing along the dock axis.

    Kinematics in the dock frame with ``v < 0``: ``y' = v sin(h)``.  Choosing a
    desired heading ``h_d = k_lat * y`` gives ``y' = -|v| k_lat y`` (stable)
    and ``w = kp (h_d - h) + kd d/dt(h_d - h)`` tracks it.

    Returns ``(linear_x, angular_z, heading_error_term)``; feed the last value
    back as ``prev_heading_err`` for the D term.
    """
    h_d = clamp(g.k_lateral * err.lateral, -g.max_approach_angle, g.max_approach_angle)
    e = wrap_angle(h_d - err.heading)
    de = 0.0 if prev_heading_err is None or dt <= 0 else wrap_angle(e - prev_heading_err) / dt
    w = clamp(g.kp_heading * e + g.kd_heading * de, -g.max_angular, g.max_angular)
    vmax = reverse_speed(err.remaining, g)
    frac = 1.0
    if g.turn_slow_err > 0:
        frac = clamp(1.0 - abs(e) / g.turn_slow_err, 0.0, 1.0)
    if err.remaining <= g.slow_distance and g.lateral_slow > 0:
        # slow and steady near the contacts: the larger the residual offset, the slower,
        # so the steering gets more travel-per-correction to null it before contact
        frac = min(frac, clamp(1.0 - abs(err.lateral) / g.lateral_slow, 0.0, 1.0))
    vmax = min(vmax, max(g.min_turn_speed, vmax * frac))
    return -vmax, w, e


def heading_hold(target_yaw: float, yaw: Optional[float], kp: float, max_w: float) -> float:
    if yaw is None:
        return 0.0
    return clamp(kp * wrap_angle(target_yaw - yaw), -max_w, max_w)


# --------------------------------------------------------------------------
# contact debounce / distance tracking
# --------------------------------------------------------------------------


class ContactDebouncer:
    """``is_docking_done`` must be true on ``n`` consecutive samples."""

    def __init__(self, n: int = 3):
        self.n = max(1, int(n))
        self._count = 0

    def update(self, sample: bool) -> bool:
        self._count = self._count + 1 if sample else 0
        return self.value

    @property
    def value(self) -> bool:
        return self._count >= self.n

    def reset(self) -> None:
        self._count = 0


class DistanceTracker:
    """Travelled distance from ``/odom`` with a time * |cmd| fallback.

    Each update adds the odom displacement since the previous update when odom
    is fresh, otherwise ``|last commanded speed| * dt``.
    """

    def __init__(self):
        self.distance = 0.0
        self._last_pose: Optional[Pose2D] = None
        self._last_t: Optional[float] = None
        self.used_fallback = False

    def start(self, t: float, odom: Optional[Pose2D]) -> None:
        self.distance = 0.0
        self._last_pose = odom
        self._last_t = t
        self.used_fallback = False

    def update(self, t: float, odom: Optional[Pose2D], cmd_speed: float) -> float:
        if self._last_t is None:
            self.start(t, odom)
            return 0.0
        dt = max(0.0, t - self._last_t)
        if odom is not None and self._last_pose is not None:
            self.distance += math.hypot(odom.x - self._last_pose.x, odom.y - self._last_pose.y)
        else:
            self.distance += abs(cmd_speed) * dt
            self.used_fallback = True
        self._last_pose = odom
        self._last_t = t
        return self.distance


# --------------------------------------------------------------------------
# state machines
# --------------------------------------------------------------------------


@dataclass
class Snapshot:
    """Sensor state injected into the state machines each control tick."""
    t: float
    odom: Optional[Pose2D] = None            # None = no fresh odom
    marker: Optional[MarkerObs] = None       # latest valid detection (any age)
    contact: bool = False                    # debounced is_docking_done
    status_fresh: bool = True                # /mower_base/status recently received
    stop_triggered: bool = False
    lift_triggered: bool = False
    nav_status: Optional[str] = None         # None|'running'|'succeeded'|'failed'
    charging_result: Optional[bool] = None   # None = pending/not requested
    rtk_fixed: bool = False
    progress_pose: Optional[Pose2D] = None   # fused pose (/odometry/filtered) for the stall guard
    dig_stall: bool = False                  # slip_detector /dig_stall (latched Bool)
    map_pose: Optional[Pose2D] = None        # robot pose in the map frame (approach skip / align)
    raw_contact: bool = False                # latest undebounced is_docking_done
    is_charging: bool = False                # MowerBaseDevStatus.is_charging


class StallGuard:
    """Wheel stall / dig detector: commanded speed vs. measured (EKF) progress.

    Over a sliding window of ``timeout_s`` with every sample commanding
    ``|v| > min_cmd``, stall = measured displacement < ``ratio`` * expected
    (integral of |v| dt).  A missing ``progress_pose`` restarts the window.
    """

    def __init__(self, min_cmd: float = 0.03, ratio: float = 0.2, timeout_s: float = 1.0):
        self.min_cmd = min_cmd
        self.ratio = ratio
        self.timeout_s = timeout_s
        self.reset()

    def reset(self) -> None:
        self._t0: Optional[float] = None
        self._p0: Optional[Pose2D] = None
        self._expected = 0.0
        self._last_t: Optional[float] = None
        self.measured = 0.0

    def update(self, t: float, pose: Optional[Pose2D], cmd: float) -> bool:
        if pose is None or abs(cmd) <= self.min_cmd or self.timeout_s <= 0:
            self.reset()
            return False
        if self._t0 is None:
            self._t0, self._p0, self._last_t, self._expected = t, pose, t, 0.0
            return False
        self._expected += abs(cmd) * max(0.0, t - self._last_t)
        self._last_t = t
        self.measured = math.hypot(pose.x - self._p0.x, pose.y - self._p0.y)
        if t - self._t0 < self.timeout_s:
            return False
        if self.measured < self.ratio * self._expected:
            return True
        # still moving: restart the window from here
        self._t0, self._p0, self._expected = t, pose, 0.0
        return False


@dataclass
class Output:
    linear: float = 0.0
    angular: float = 0.0
    state: str = ''
    done: bool = False
    success: bool = False
    message: str = ''
    remaining: float = float('nan')
    retries: int = 0
    travelled: float = 0.0
    requests: List[tuple] = field(default_factory=list)  # ('start_nav', Pose2D) ...
    detail: str = ''                          # human status (GUI sub_state), incl. marker visibility
    notes: List[str] = field(default_factory=list)       # one-shot log lines for the node
    calibrated_offset: Optional[float] = None  # measured docked_marker_offset (on success)
    marker_in_view: bool = False               # valid dock marker fresh (or FINAL/creep phase)


@dataclass
class DockParams:
    approach_distance: float = 0.8
    skip_nav_to_approach: bool = False
    nav_timeout_s: float = 120.0
    use_vision: bool = True
    search_timeout_s: float = 15.0
    max_lateral_error: float = 0.5
    max_yaw_error_deg: float = 25.0
    marker_timeout_s: float = 0.5
    max_lost_frames: int = 20
    docked_marker_offset: float = 0.45
    docking_timeout_s: float = 60.0
    final_timeout_s: float = 10.0
    final_max_distance: float = 0.30
    blind_extra_distance: float = 0.1
    blind_timeout_margin_s: float = 10.0
    max_retries: int = 3
    retry_forward_distance: float = 0.3
    retry_speed: float = 0.1
    retry_timeout_s: float = 15.0
    heading_hold_kp: float = 1.0
    charging_timeout_s: float = 5.0
    # Blind (dead-reckoned) final docking is only allowed when the dock pose was
    # measured (set_docking_point) AND a marker was seen within
    # blind_marker_max_age_s during this attempt; allow_blind_docking restores the
    # vendor behaviour (blind reverse after any search timeout).
    allow_blind_docking: bool = False
    blind_marker_max_age_s: float = 30.0
    # stall / dig guard (DOCKING / FINAL_DOCKING / RETRY)
    stall_min_cmd: float = 0.03
    stall_progress_ratio: float = 0.2
    stall_timeout_s: float = 1.0
    use_dig_stall: bool = True
    # Approach-pose shortcuts (2026-10-07: Nav2 "Failed to make progress" 0.25 m from the
    # approach pose because the remaining in-place pivot does not turn on turf).
    approach_skip_radius_m: float = 0.6       # already this close: skip Nav2, align + search
    nav_fail_arrived_radius_m: float = 1.0    # Nav2 failed within this: treat as arrived
    relaxed_nav_retry: bool = True            # beyond it: one Nav2 retry with the current yaw
    # ALIGNING: slow turn toward the dock yaw (robot creeps forward, pivots do not turn on turf)
    align_angular: float = 0.25
    align_creep: float = 0.03
    align_max_travel: float = 0.4
    align_timeout_s: float = 20.0
    align_tolerance_deg: float = 10.0
    # DOCKING: once tracking, the marker gate is widened by this factor (hysteresis), so a
    # robot that entered at the gate edge is not dropped while it turns onto the axis.
    docking_gate_scale: float = 1.4
    # Reaching final_zone BADLY off the axis -> drive forward along the dock heading and
    # re-approach (does not use a retry), at most max_realigns times per goal. Smaller
    # errors are corrected while reversing (the controller steers all the way to contact).
    final_max_lateral: float = 0.15
    final_max_heading_deg: float = 20.0
    max_realigns: int = 2
    marker_median_n: int = 3         # gates/decisions use the median of the last N markers
    realign_forward_distance: float = 0.8
    # FINAL_DOCKING budget exhausted without contact (2026-10-07 Home: the retry drove
    # forward off the pins 7 s before the contacts reported): hold still, then creep a bit
    # further, only then retry.
    contact_settle_s: float = 3.0
    final_extra_creep_m: float = 0.15
    # docked_marker_offset auto-calibration from the last marker seen before contact
    calib_max_marker_age_s: float = 15.0
    calib_min_offset: float = 0.2
    calib_max_offset: float = 1.2
    gains: ControllerGains = field(default_factory=ControllerGains)


class DockState:
    NAV_TO_APPROACH = 'NAV_TO_APPROACH'
    ALIGNING = 'ALIGNING'
    SEARCHING = 'SEARCHING'
    DOCKING = 'DOCKING'
    FINAL_DOCKING = 'FINAL_DOCKING'
    RETRY = 'RETRY'
    CHARGING = 'CHARGING'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'


class DockMsg:
    NAV_TO_DOCK_FAILED = 'NAV_TO_DOCK_FAILED'
    DOCK_MAXOUT = 'DOCK_MAXOUT'
    DOCK_TIMEOUT = 'DOCK_TIMEOUT'
    EMERGENCY_STOP = 'EMERGENCY_STOP'
    BASE_STATUS_STALE = 'BASE_STATUS_STALE'
    CHARGER_ENABLE_FAILED = 'CHARGER_ENABLE_FAILED'
    CANCELED = 'CANCELED'
    DOCKED = 'DOCKED'
    ALREADY_DOCKED = 'ALREADY_DOCKED'
    DOCK_NOT_FOUND = 'DOCK_NOT_FOUND'
    DOCK_STALLED = 'DOCK_STALLED'


class DockStateMachine:
    """Vendor-like docking FSM (INIT/SEARCHING/DOCKING/FINAL_DOCKING/RETRY).

    Call :meth:`start` once, then :meth:`step` at the control rate.  The
    returned :class:`Output` carries the velocity command and side-effect
    requests for the node: ``('start_nav', Pose2D)``, ``('cancel_nav',)``,
    ``('enable_charging',)``.  Results come back through the snapshot
    (``nav_status`` / ``charging_result``).
    """

    def __init__(self, params: DockParams, dock_pose: Pose2D,
                 goal_timeout_s: float = 180.0, use_vision: Optional[bool] = None,
                 dock_pose_measured: bool = False):
        self.p = params
        self.dock_pose_measured = bool(dock_pose_measured)
        self._marker_seen_t: Optional[float] = None
        self._stall = StallGuard(params.stall_min_cmd, params.stall_progress_ratio,
                                 params.stall_timeout_s)
        self._dig_at_start = False
        self.use_vision = params.use_vision if use_vision is None else bool(use_vision)
        self.dock_pose = dock_pose
        self.approach = approach_pose(dock_pose, params.approach_distance)
        self.goal_timeout_s = goal_timeout_s
        self.state = ''
        self.retries = 0
        self.message = ''
        self._t_start = 0.0
        self._t_state = 0.0
        self._lost = 0
        self._last_marker_stamp: Optional[float] = None
        self._prev_e: Optional[float] = None
        self._prev_t: Optional[float] = None
        self._tracker = DistanceTracker()
        self._blind = False
        self._final_limit = params.final_max_distance
        self._final_timeout = params.final_timeout_s
        self._hold_yaw: Optional[float] = None
        self._retry_distance = params.retry_forward_distance
        self._last_cmd = 0.0
        self._remaining = float('nan')
        self._final_entry_remaining = 0.0
        self._pending: List[tuple] = []
        self._notes: List[str] = []
        self._relaxed_tried = False
        self._realigns = 0
        self._realign = False
        self._align_target: Optional[float] = None   # odom-frame yaw to turn to
        self._marker_text = 'not visible'
        self._heading_left: Optional[float] = None
        self._final_phase = 'reverse'           # FINAL_DOCKING: reverse | settle | creep
        self._final_phase_t = 0.0
        self._final_phase_d = 0.0
        self._cal_obs: Optional[Tuple[float, Pose2D, float]] = None  # (rem0, odom, stamp)
        self._now = 0.0
        self._valid_marker_t: Optional[float] = None   # stamp of the last in-gate marker
        self._obs: List[Tuple[float, DockErrors]] = []   # (stamp, errors) for the median
        self._last_err: Optional[DockErrors] = None      # latest DOCKING observation
        self._last_err_odom: Optional[Pose2D] = None
        self.calibrated_offset: Optional[float] = None

    # -- helpers ---------------------------------------------------------
    def _enter(self, state: str, snap: Snapshot) -> None:
        self.state = state
        self._t_state = snap.t
        self._tracker.start(snap.t, snap.odom)
        self._hold_yaw = snap.odom.yaw if snap.odom is not None else None
        self._prev_e = None
        self._prev_t = None
        self._stall.reset()

    def _fail(self, msg: str, snap: Snapshot) -> None:
        if self.state == DockState.NAV_TO_APPROACH:
            self._pending.append(('cancel_nav',))
        self.state = DockState.FAILED
        self.message = msg

    def _post_approach(self, snap: Snapshot) -> None:
        if self.use_vision:
            self._lost = 0
            self._enter(DockState.SEARCHING, snap)
        else:
            self._try_blind(snap)

    def blind_allowed(self, t: float) -> Tuple[bool, str]:
        if self.p.allow_blind_docking:
            return True, ''
        if not self.dock_pose_measured:
            return False, 'dock pose not measured (press "Set docking point" while docked)'
        if self._marker_seen_t is None:
            return False, 'marker not visible'
        if t - self._marker_seen_t > self.p.blind_marker_max_age_s:
            return False, 'marker not seen for %.0f s' % (t - self._marker_seen_t)
        return True, ''

    def _try_blind(self, snap: Snapshot) -> None:
        ok, why = self.blind_allowed(snap.t)
        if ok:
            self._enter_blind(snap)
        else:
            self._fail('%s: dock not found: %s' % (DockMsg.DOCK_NOT_FOUND, why), snap)

    def _enter_blind(self, snap: Snapshot) -> None:
        self._blind = True
        self._enter(DockState.FINAL_DOCKING, snap)
        self._final_phase = 'reverse'
        g = self.p.gains
        self._final_limit = self.p.approach_distance + self.p.blind_extra_distance
        fast = max(0.0, self.p.approach_distance - g.slow_zone)
        slow = self._final_limit - fast
        self._final_timeout = (fast / max(g.max_speed, 1e-3) + slow / max(g.final_speed, 1e-3)
                               + self.p.blind_timeout_margin_s)

    def _enter_vision_final(self, snap: Snapshot, phase: str = 'reverse') -> None:
        """Marker lost within final_zone: reverse straight on the last docked heading
        (dead-reckoned remaining), or (phase 'settle') overran the expected contact
        point with the marker still in view."""
        self._blind = False
        rem = self._remaining if math.isfinite(self._remaining) else 0.0
        if self._last_err is not None and self._last_err_odom is not None and \
                snap.odom is not None:
            rem = self._last_err.remaining - math.hypot(snap.odom.x - self._last_err_odom.x,
                                                        snap.odom.y - self._last_err_odom.y)
        self._final_entry_remaining = max(0.0, rem)
        hold = None
        if self._last_err is not None and self._last_err_odom is not None:
            hold = wrap_angle(self._last_err_odom.yaw - self._last_err.heading)
        self._enter(DockState.FINAL_DOCKING, snap)
        if hold is not None:
            self._hold_yaw = hold
        self._final_phase = phase
        self._final_phase_t = snap.t
        self._final_phase_d = 0.0
        g = self.p.gains
        # same overrun allowance as the vendor budget (0.30 m from 0.20 m out = 0.10 m)
        self._final_limit = max(0.0, rem) + max(0.0, self.p.final_max_distance - g.final_zone)
        self._final_timeout = self.p.final_timeout_s

    def _push_obs(self, m: MarkerObs, err: DockErrors) -> None:
        if self._obs and self._obs[-1][0] == m.stamp:
            return
        self._obs.append((m.stamp, err))
        n = max(1, int(self.p.marker_median_n))
        del self._obs[:-n]

    def _median_err(self) -> Optional[DockErrors]:
        if not self._obs:
            return None
        def med(vals):
            v = sorted(vals)
            k = len(v)
            return v[k // 2] if k % 2 else 0.5 * (v[k // 2 - 1] + v[k // 2])
        es = [e for _, e in self._obs]
        return DockErrors(med([e.remaining for e in es]), med([e.lateral for e in es]),
                          med([e.heading for e in es]))

    def _enter_retry(self, snap: Snapshot, travelled_reverse: float) -> None:
        if self.retries >= self.p.max_retries:
            self._fail(DockMsg.DOCK_MAXOUT, snap)
            return
        self.retries += 1
        self._obs = []
        self._last_err = None
        if self.p.skip_nav_to_approach and (self._blind or not self.use_vision):
            # no Nav2 to re-approach: drive back to where the attempt started
            self._retry_distance = min(max(travelled_reverse, self.p.retry_forward_distance),
                                       self.p.approach_distance + self.p.blind_extra_distance)
        else:
            self._retry_distance = self.p.retry_forward_distance
        self._enter(DockState.RETRY, snap)

    def _enter_realign(self, snap: Snapshot, err: DockErrors) -> None:
        """Forward along the dock heading, then search again (no retry consumed)."""
        self._realigns += 1
        self._realign = True
        self._retry_distance = self.p.realign_forward_distance
        self._notes.append('off the dock axis at the final zone (lateral %.2f m, heading %.1f deg):'
                           ' realign %d/%d' % (err.lateral, math.degrees(err.heading),
                                               self._realigns, self.p.max_realigns))
        self._enter(DockState.RETRY, snap)
        self._obs = []
        self._last_err = None
        if snap.odom is not None:
            self._hold_yaw = wrap_angle(snap.odom.yaw - err.heading)   # docked heading in odom

    def _calibrate(self, snap: Snapshot) -> None:
        """docked_marker_offset = marker distance (at offset 0) at the last observation
        minus the distance driven since then."""
        c = self._cal_obs
        if c is None or snap.odom is None or snap.t - c[2] > self.p.calib_max_marker_age_s:
            return
        off = c[0] - math.hypot(snap.odom.x - c[1].x, snap.odom.y - c[1].y)
        if self.p.calib_min_offset <= off <= self.p.calib_max_offset:
            self.calibrated_offset = off
            self._notes.append('measured docked_marker_offset %.3f m (configured %.3f m)'
                               % (off, self.p.docked_marker_offset))
        else:
            self._notes.append('docked_marker_offset estimate %.3f m out of range: ignored' % off)

    def _dist_to_approach(self, snap: Snapshot) -> Optional[float]:
        if snap.map_pose is None:
            return None
        return math.hypot(snap.map_pose.x - self.approach.x, snap.map_pose.y - self.approach.y)

    def _arrive(self, snap: Snapshot) -> None:
        """At (or near) the approach pose: turn toward the dock yaw when the heading is
        outside the marker gate, then SEARCHING lets the marker decide."""
        if self.use_vision and snap.map_pose is not None and snap.odom is not None:
            err = wrap_angle(self.approach.yaw - snap.map_pose.yaw)
            if abs(err) > math.radians(self.p.max_yaw_error_deg):
                self._enter(DockState.ALIGNING, snap)
                self._align_target = wrap_angle(snap.odom.yaw + err)
                self._heading_left = err
                self._notes.append('heading %.0f deg off the dock yaw: aligning' % math.degrees(err))
                return
        self._post_approach(snap)

    def _nav_failed(self, snap: Snapshot, timed_out: bool) -> List[tuple]:
        d = self._dist_to_approach(snap)
        reqs: List[tuple] = [('cancel_nav',)] if timed_out else []
        why = 'timed out' if timed_out else 'failed'
        if d is not None and d <= self.p.nav_fail_arrived_radius_m:
            self._notes.append('Nav2 to the approach pose %s %.2f m from it: treating as arrived'
                               % (why, d))
            self._arrive(snap)
            return reqs
        if d is not None and self.p.relaxed_nav_retry and not self._relaxed_tried:
            self._relaxed_tried = True
            goal = Pose2D(self.approach.x, self.approach.y, snap.map_pose.yaw)
            self._notes.append('Nav2 to the approach pose %s %.2f m from it: retrying with the '
                               'current yaw (%.0f deg)' % (why, d, math.degrees(goal.yaw)))
            self._enter(DockState.NAV_TO_APPROACH, snap)
            return reqs + [('start_nav', goal)]
        if timed_out:
            self._fail(DockMsg.NAV_TO_DOCK_FAILED, snap)
        else:
            self.state = DockState.FAILED
            self.message = DockMsg.NAV_TO_DOCK_FAILED
        return []

    def _go_to_approach(self, snap: Snapshot) -> List[tuple]:
        """Nav2 to the approach pose unless already within approach_skip_radius_m."""
        d = self._dist_to_approach(snap)
        if d is not None and self.use_vision and d <= self.p.approach_skip_radius_m:
            self._notes.append('%.2f m from the approach pose: skipping Nav2' % d)
            self._arrive(snap)
            return []
        self._relaxed_tried = False
        self._enter(DockState.NAV_TO_APPROACH, snap)
        return [('start_nav', self.approach)]

    def _fresh_marker(self, snap: Snapshot) -> Optional[MarkerObs]:
        m = snap.marker
        if m is None or snap.t - m.stamp > self.p.marker_timeout_s:
            return None
        if m.stamp >= self._t_start:
            self._marker_seen_t = m.stamp if self._marker_seen_t is None \
                else max(self._marker_seen_t, m.stamp)
        return m

    # -- API -------------------------------------------------------------
    def start(self, snap: Snapshot) -> Output:
        self._t_start = snap.t
        self._now = snap.t
        self.retries = 0
        self._obs = []
        self._last_err = None
        self._valid_marker_t = None
        self._marker_seen_t = None
        self._dig_at_start = bool(snap.dig_stall)
        if snap.contact:
            self.state = DockState.CHARGING
            self._t_state = snap.t
            self.message = DockMsg.ALREADY_DOCKED
            return self._out(requests=[('enable_charging',)])
        if self.p.skip_nav_to_approach:
            self._post_approach(snap)
            return self._out()
        return self._out(requests=self._go_to_approach(snap))

    def cancel(self) -> Output:
        reqs = [('cancel_nav',)] if self.state == DockState.NAV_TO_APPROACH else []
        self.state = DockState.FAILED
        self.message = DockMsg.CANCELED
        return self._out(requests=reqs)

    def _out(self, v: float = 0.0, w: float = 0.0, requests=None) -> Output:
        self._last_cmd = v
        reqs = self._pending + list(requests or [])
        self._pending = []
        notes, self._notes = self._notes, []
        done = self.state in (DockState.SUCCEEDED, DockState.FAILED)
        if done:
            v = w = 0.0
        return Output(linear=v, angular=w, state=self.state, done=done,
                      success=self.state == DockState.SUCCEEDED, message=self.message,
                      remaining=self._remaining, retries=self.retries,
                      travelled=self._tracker.distance, requests=reqs,
                      detail=self.detail(), notes=notes,
                      calibrated_offset=self.calibrated_offset if self.state ==
                      DockState.SUCCEEDED else None,
                      marker_in_view=self._marker_in_view_now())

    def _marker_in_view_now(self) -> bool:
        """Owner rule: with the dock marker in view the mission ignores other camera
        (obstacle) detections. True while an in-gate marker is younger than
        marker_timeout_s, and throughout vision FINAL_DOCKING (straight / settle / creep)."""
        if self.state in (DockState.SUCCEEDED, DockState.FAILED, DockState.CHARGING, ''):
            return False
        if self.state == DockState.FINAL_DOCKING and not self._blind:
            return True
        return self._valid_marker_t is not None and \
            self._now - self._valid_marker_t <= self.p.marker_timeout_s

    def detail(self) -> str:
        """Human-readable status incl. marker visibility (GUI sub_state)."""
        st, mk = self.state, self._marker_text
        rem = ('%.2f m to go' % self._remaining) if math.isfinite(self._remaining) else ''
        if st == DockState.NAV_TO_APPROACH:
            return 'navigating to approach pose%s; marker %s' % (
                ' (current yaw)' if self._relaxed_tried else '', mk)
        if st == DockState.ALIGNING:
            left = '' if self._heading_left is None else \
                ' %.0f\N{DEGREE SIGN} to go' % abs(math.degrees(self._heading_left))
            return 'aligning to dock heading:%s; marker %s' % (left, mk)
        if st == DockState.SEARCHING:
            return 'searching marker: %s' % mk
        if st == DockState.DOCKING:
            e = self._last_err
            if e is not None and math.isfinite(self._remaining):
                return 'docking: %.2f m, lateral %.0f cm, %.0f\N{DEGREE SIGN}; marker %s' % (
                    max(0.0, self._remaining), abs(e.lateral) * 100.0,
                    abs(math.degrees(e.heading)), mk)
            return 'docking: %s; marker %s' % (rem, mk)
        if st == DockState.FINAL_DOCKING:
            if self._final_phase == 'settle':
                return 'final reverse: budget used, waiting for the contacts'
            if self._final_phase == 'creep':
                return 'final reverse: creeping further for the contacts'
            return 'final reverse%s: %s' % (' (blind)' if self._blind else
                                             ' (marker lost, last heading)', rem)
        if st == DockState.RETRY:
            return ('realigning %d/%d: forward along dock heading' % (
                self._realigns, self.p.max_realigns)) if self._realign else \
                'retry %d/%d: driving forward' % (self.retries, self.p.max_retries)
        if st == DockState.CHARGING:
            return 'on contacts: enabling charging'
        if st == DockState.FAILED:
            return 'docking failed: %s' % self.message
        if st == DockState.SUCCEEDED:
            return 'docked'
        return ''

    def _update_marker_text(self, snap: Snapshot) -> None:
        m = self._fresh_marker(snap)
        if m is not None and snap.odom is not None and \
                (self._cal_obs is None or self._cal_obs[2] != m.stamp):
            rem0 = -(math.cos(m.yaw) * m.x + math.sin(m.yaw) * m.y)   # remaining at offset 0
            self._cal_obs = (rem0, snap.odom, m.stamp)
        if m is None:
            self._marker_text = 'not visible'
            return
        err = dock_errors(m, self.p.docked_marker_offset)
        kg = max(1.0, self.p.docking_gate_scale)
        if marker_gate_ok(err, kg * self.p.max_lateral_error,
                          kg * math.radians(self.p.max_yaw_error_deg)):
            self._valid_marker_t = m.stamp
        txt = 'seen %.2f m, %.1f\N{DEGREE SIGN}' % (math.hypot(m.x, m.y),
                                                    math.degrees(err.heading))
        if not marker_gate_ok(err, self.p.max_lateral_error,
                              math.radians(self.p.max_yaw_error_deg)):
            txt += ' (outside gate: lateral %.2f m)' % err.lateral
        self._marker_text = txt

    def step(self, snap: Snapshot) -> Output:
        if self.state in (DockState.SUCCEEDED, DockState.FAILED):
            return self._out()
        self._now = snap.t
        travelled = self._tracker.update(snap.t, snap.odom, self._last_cmd)
        self._update_marker_text(snap)

        # ---- global guards ------------------------------------------------
        if snap.stop_triggered or snap.lift_triggered:
            self._fail(DockMsg.EMERGENCY_STOP, snap)
            return self._out()
        if not snap.status_fresh and self.state in (DockState.DOCKING, DockState.FINAL_DOCKING,
                                                    DockState.RETRY):
            self._fail(DockMsg.BASE_STATUS_STALE, snap)
            return self._out()
        if snap.t - self._t_start > self.goal_timeout_s and self.state != DockState.CHARGING:
            self._fail(DockMsg.DOCK_TIMEOUT, snap)
            return self._out()
        moving = (DockState.SEARCHING, DockState.DOCKING, DockState.FINAL_DOCKING,
                  DockState.ALIGNING, DockState.RETRY, DockState.NAV_TO_APPROACH)
        if (snap.contact or snap.is_charging) and self.state in moving:
            if self.state == DockState.NAV_TO_APPROACH:
                self._pending.append(('cancel_nav',))
            self._notes.append('contact in %s (debounced=%s charging=%s): stopping, confirming'
                               % (self.state, snap.contact, snap.is_charging))
            self._calibrate(snap)
            self.state = DockState.CHARGING
            self._t_state = snap.t
            self._remaining = 0.0
            return self._out(requests=[('enable_charging',)])
        if snap.raw_contact and self.state in moving and self.state != DockState.NAV_TO_APPROACH:
            return self._out()   # raw contact: hold still until the debounce confirms it

        if self.state in (DockState.DOCKING, DockState.FINAL_DOCKING, DockState.RETRY):
            dig = self.p.use_dig_stall and snap.dig_stall and not self._dig_at_start
            if dig or self._stall.update(snap.t, snap.progress_pose, self._last_cmd):
                why = 'slip detector /dig_stall' if dig else \
                    'moved %.2f m while commanding %.2f m/s' % (self._stall.measured,
                                                                 abs(self._last_cmd))
                self._fail('%s: wheels stalled in %s (%s)' % (DockMsg.DOCK_STALLED,
                                                             self.state, why), snap)
                return self._out()

        st = self.state
        el = snap.t - self._t_state
        g = self.p.gains

        if st == DockState.NAV_TO_APPROACH:
            if snap.nav_status == 'succeeded':
                self._arrive(snap)
            elif snap.nav_status == 'failed':
                return self._out(requests=self._nav_failed(snap, False))
            elif el > self.p.nav_timeout_s:
                return self._out(requests=self._nav_failed(snap, True))
            return self._out()

        if st == DockState.ALIGNING:
            m = self._fresh_marker(snap)
            if m is not None and marker_gate_ok(dock_errors(m, self.p.docked_marker_offset),
                                                self.p.max_lateral_error,
                                                math.radians(self.p.max_yaw_error_deg)):
                self._notes.append('marker in gate while aligning: searching')
                self._post_approach(snap)
                return self._out()
            if snap.odom is None or self._align_target is None:
                self._post_approach(snap)
                return self._out()
            err = wrap_angle(self._align_target - snap.odom.yaw)
            self._heading_left = err
            if abs(err) <= math.radians(self.p.align_tolerance_deg):
                self._post_approach(snap)
                return self._out()
            if travelled >= self.p.align_max_travel or el > self.p.align_timeout_s:
                self._notes.append('alignment stopped %.0f deg short (%.2f m, %.0f s): '
                                   'searching' % (math.degrees(abs(err)), travelled, el))
                self._post_approach(snap)
                return self._out()
            return self._out(self.p.align_creep, math.copysign(self.p.align_angular, err))

        if st == DockState.SEARCHING:
            m = self._fresh_marker(snap)
            if m is not None:
                err = dock_errors(m, self.p.docked_marker_offset)
                self._push_obs(m, err)
                self._remaining = err.remaining
                if marker_gate_ok(self._median_err(), self.p.max_lateral_error,
                                  math.radians(self.p.max_yaw_error_deg)):
                    self._lost = 0
                    self._last_err, self._last_err_odom = err, snap.odom
                    self._enter(DockState.DOCKING, snap)
                    return self._out()
            if el > self.p.search_timeout_s:
                self._try_blind(snap)
            return self._out()

        if st == DockState.DOCKING:
            if el > self.p.docking_timeout_s:
                self._enter_retry(snap, travelled)
                return self._out()
            m = self._fresh_marker(snap)
            new = m is not None and m.stamp != self._last_marker_stamp
            err = dock_errors(m, self.p.docked_marker_offset) if m is not None else None
            if err is not None:
                self._push_obs(m, err)
            gerr = self._median_err() if err is not None else None
            k = max(1.0, self.p.docking_gate_scale)
            if gerr is None or not marker_gate_ok(gerr, k * self.p.max_lateral_error,
                                                  k * math.radians(self.p.max_yaw_error_deg)):
                if self._last_err is not None and self._last_err.remaining <= g.final_zone:
                    # lost in the last stretch (too close / occluded): straight on the
                    # last docked heading, dead-reckoned
                    self._notes.append('marker lost %.2f m from the contacts: reversing straight'
                                       ' on the last heading' % self._last_err.remaining)
                    self._enter_vision_final(snap)
                    return self._out(-g.final_speed, 0.0)
                self._lost += 1
                if self._lost > self.p.max_lost_frames:
                    self._enter_retry(snap, travelled)
                return self._out()  # hold still while the marker is lost
            if new:
                self._lost = 0
                self._last_marker_stamp = m.stamp
            self._last_err, self._last_err_odom = err, snap.odom
            self._remaining = err.remaining
            if gerr.remaining <= g.final_zone:
                off = abs(gerr.lateral) > self.p.final_max_lateral or \
                    abs(gerr.heading) > math.radians(self.p.final_max_heading_deg)
                if off and self._realigns < self.p.max_realigns:
                    self._enter_realign(snap, gerr)
                    return self._out()
            overrun = max(0.0, self.p.final_max_distance - g.final_zone)
            if gerr.remaining <= -overrun:
                self._notes.append('passed the expected contact point by %.2f m with the marker '
                                   'in view: holding for the contacts' % -gerr.remaining)
                self._enter_vision_final(snap, phase='settle')
                return self._out()
            dt = snap.t - self._prev_t if self._prev_t is not None else 0.05
            self._prev_t = snap.t
            v, w, self._prev_e = reverse_control(err, g, self._prev_e, dt)
            return self._out(v, w)

        if st == DockState.FINAL_DOCKING:
            if self._final_phase == 'settle':
                if snap.t - self._final_phase_t < self.p.contact_settle_s:
                    return self._out()
                if self.p.final_extra_creep_m <= 0:
                    self._enter_retry(snap, travelled)
                    return self._out()
                self._final_phase, self._final_phase_t = 'creep', snap.t
                self._final_phase_d = travelled
                self._notes.append('no contact after %.0f s: creeping %.2f m further'
                                   % (self.p.contact_settle_s, self.p.final_extra_creep_m))
            if self._final_phase == 'creep':
                limit_t = self.p.final_extra_creep_m / max(g.final_speed, 1e-3) * 2.0 + 2.0
                if travelled - self._final_phase_d >= self.p.final_extra_creep_m or \
                        snap.t - self._final_phase_t > limit_t:
                    self._notes.append('no contact after the extra creep: retry')
                    self._enter_retry(snap, travelled)
                    return self._out()
                w = heading_hold(self._hold_yaw, snap.odom.yaw if snap.odom else None,
                                 self.p.heading_hold_kp, g.max_angular) \
                    if self._hold_yaw is not None else 0.0
                return self._out(-g.final_speed, w)
            if travelled >= self._final_limit or el > self._final_timeout:
                self._final_phase, self._final_phase_t = 'settle', snap.t
                self._notes.append('final budget used (%.2f m, %.0f s) without contact: '
                                   'holding %.0f s' % (travelled, el, self.p.contact_settle_s))
                return self._out()
            if self._blind:
                left = self._final_limit - travelled
                self._remaining = max(0.0, left - self.p.blind_extra_distance)
                speed = reverse_speed(self._remaining, g, near=g.final_speed)   # straight: 0.05
            else:
                self._remaining = max(0.0, self._final_entry_remaining - travelled)
                speed = g.final_speed
            w = heading_hold(self._hold_yaw, snap.odom.yaw if snap.odom else None,
                             self.p.heading_hold_kp, g.max_angular) \
                if self._hold_yaw is not None else 0.0
            return self._out(-speed, w)

        if st == DockState.RETRY:
            if travelled >= self._retry_distance:
                if self._realign:
                    self._realign = False
                    self._post_approach(snap)
                    return self._out()
                if self.p.skip_nav_to_approach:
                    self._post_approach(snap)
                    return self._out()
                return self._out(requests=self._go_to_approach(snap))
            if el > self.p.retry_timeout_s:
                self._fail(DockMsg.DOCK_MAXOUT, snap)
                return self._out()
            w = heading_hold(self._hold_yaw, snap.odom.yaw if snap.odom else None,
                             self.p.heading_hold_kp, g.max_angular) \
                if self._hold_yaw is not None else 0.0
            return self._out(self.p.retry_speed, w)

        if st == DockState.CHARGING:
            if snap.charging_result is True:
                self.state = DockState.SUCCEEDED
                if not self.message:
                    self.message = DockMsg.DOCKED
            elif snap.charging_result is False or el > self.p.charging_timeout_s:
                self.state = DockState.FAILED
                self.message = DockMsg.CHARGER_ENABLE_FAILED
            return self._out()

        return self._out()


# --------------------------------------------------------------------------
# undock
# --------------------------------------------------------------------------


@dataclass
class UndockParams:
    direction: float = 1.0           # +1: leave forwards (robot reversed onto the dock)
    heading_hold_kp: float = 1.0
    max_angular: float = 0.3
    max_speed: float = 0.3           # vendor Undocking linear_max
    charging_timeout_s: float = 5.0
    drive_timeout_margin_s: float = 5.0


class UndockState:
    CHARGER_OFF = 'CHARGER_OFF'
    BACKING_UP = 'BACKING_UP'
    WAITING_FOR_RTK = 'WAITING_FOR_RTK'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'


class UndockMsg:
    UNDOCKED = 'UNDOCKED'
    CHARGER_OFF_FAILED = 'CHARGER_OFF_FAILED'
    EMERGENCY_STOP = 'EMERGENCY_STOP'
    BASE_STATUS_STALE = 'BASE_STATUS_STALE'
    DRIVE_TIMEOUT = 'DRIVE_TIMEOUT'
    RTK_TIMEOUT = 'RTK_TIMEOUT'
    CANCELED = 'CANCELED'


class UndockStateMachine:
    """CHARGER_OFF -> BACKING_UP (drive ``distance`` away from the charger)
    -> WAITING_FOR_RTK (optional) -> SUCCEEDED."""

    def __init__(self, params: UndockParams, distance: float, speed: float,
                 wait_for_rtk: bool, rtk_timeout_s: float):
        self.p = params
        self.distance = max(0.0, float(distance))
        self.speed = clamp(abs(float(speed)), 0.01, params.max_speed)
        self.wait_for_rtk = bool(wait_for_rtk)
        self.rtk_timeout_s = float(rtk_timeout_s)
        self.state = ''
        self.message = ''
        self._t_state = 0.0
        self._tracker = DistanceTracker()
        self._hold_yaw: Optional[float] = None
        self._last_cmd = 0.0

    def _enter(self, st: str, snap: Snapshot) -> None:
        self.state = st
        self._t_state = snap.t
        self._tracker.start(snap.t, snap.odom)
        self._hold_yaw = snap.odom.yaw if snap.odom is not None else None

    def _out(self, v: float = 0.0, w: float = 0.0, requests=None) -> Output:
        done = self.state in (UndockState.SUCCEEDED, UndockState.FAILED)
        if done:
            v = w = 0.0
        self._last_cmd = v
        return Output(linear=v, angular=w, state=self.state, done=done,
                      success=self.state == UndockState.SUCCEEDED, message=self.message,
                      travelled=self._tracker.distance, requests=list(requests or []))

    def start(self, snap: Snapshot) -> Output:
        self._enter(UndockState.CHARGER_OFF, snap)
        return self._out(requests=[('disable_charging',)])

    def cancel(self) -> Output:
        self.state = UndockState.FAILED
        self.message = UndockMsg.CANCELED
        return self._out()

    def _finish(self, ok: bool, msg: str) -> None:
        self.state = UndockState.SUCCEEDED if ok else UndockState.FAILED
        self.message = msg

    def step(self, snap: Snapshot) -> Output:
        if self.state in (UndockState.SUCCEEDED, UndockState.FAILED):
            return self._out()
        travelled = self._tracker.update(snap.t, snap.odom, self._last_cmd)
        el = snap.t - self._t_state
        if snap.stop_triggered or snap.lift_triggered:
            self._finish(False, UndockMsg.EMERGENCY_STOP)
            return self._out()
        if self.state == UndockState.CHARGER_OFF:
            if snap.charging_result is True:
                self._enter(UndockState.BACKING_UP, snap)
            elif snap.charging_result is False or el > self.p.charging_timeout_s:
                self._finish(False, UndockMsg.CHARGER_OFF_FAILED)
            return self._out()
        if self.state == UndockState.BACKING_UP:
            if not snap.status_fresh:
                self._finish(False, UndockMsg.BASE_STATUS_STALE)
                return self._out()
            if travelled >= self.distance:
                if self.wait_for_rtk:
                    self._enter(UndockState.WAITING_FOR_RTK, snap)
                    self._tracker.distance = travelled
                else:
                    self._finish(True, UndockMsg.UNDOCKED)
                return self._out()
            if el > self.distance / self.speed * 2.0 + self.p.drive_timeout_margin_s:
                self._finish(False, UndockMsg.DRIVE_TIMEOUT)
                return self._out()
            w = heading_hold(self._hold_yaw, snap.odom.yaw if snap.odom else None,
                             self.p.heading_hold_kp, self.p.max_angular) \
                if self._hold_yaw is not None else 0.0
            # slow to half speed over the last 5 cm so we do not overshoot much
            v = self.speed if self.distance - travelled > 0.05 else max(0.05, self.speed / 2)
            return self._out(self.p.direction * v, w)
        if self.state == UndockState.WAITING_FOR_RTK:
            if snap.rtk_fixed:
                self._finish(True, UndockMsg.UNDOCKED)
            elif el > self.rtk_timeout_s:
                self._finish(False, UndockMsg.RTK_TIMEOUT)
            return self._out()
        return self._out()
