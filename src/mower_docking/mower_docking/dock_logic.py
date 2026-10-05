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
    final_zone: float = 0.2         # remaining <= this -> FINAL_DOCKING (straight); vendor
    #                                 final_distance_to_dock_ = 0.2
    k_lateral: float = 1.5          # rad of desired heading per metre of lateral error
    max_approach_angle: float = 0.35
    kp_heading: float = 0.8
    kd_heading: float = 0.0
    max_angular: float = 0.3


def reverse_speed(remaining: float, g: ControllerGains) -> float:
    """Reverse speed magnitude: final_speed within ``slow_distance`` of the
    contacts, ramping up to max_speed over ``slow_zone`` beyond it."""
    frac = clamp((remaining - g.slow_distance) / max(g.slow_zone, 1e-6), 0.0, 1.0)
    return g.final_speed + (g.max_speed - g.final_speed) * frac


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
    v = -reverse_speed(err.remaining, g)
    return v, w, e


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
    gains: ControllerGains = field(default_factory=ControllerGains)


class DockState:
    NAV_TO_APPROACH = 'NAV_TO_APPROACH'
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


class DockStateMachine:
    """Vendor-like docking FSM (INIT/SEARCHING/DOCKING/FINAL_DOCKING/RETRY).

    Call :meth:`start` once, then :meth:`step` at the control rate.  The
    returned :class:`Output` carries the velocity command and side-effect
    requests for the node: ``('start_nav', Pose2D)``, ``('cancel_nav',)``,
    ``('enable_charging',)``.  Results come back through the snapshot
    (``nav_status`` / ``charging_result``).
    """

    def __init__(self, params: DockParams, dock_pose: Pose2D,
                 goal_timeout_s: float = 180.0, use_vision: Optional[bool] = None):
        self.p = params
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

    # -- helpers ---------------------------------------------------------
    def _enter(self, state: str, snap: Snapshot) -> None:
        self.state = state
        self._t_state = snap.t
        self._tracker.start(snap.t, snap.odom)
        self._hold_yaw = snap.odom.yaw if snap.odom is not None else None
        self._prev_e = None
        self._prev_t = None

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
            self._enter_blind(snap)

    def _enter_blind(self, snap: Snapshot) -> None:
        self._blind = True
        self._enter(DockState.FINAL_DOCKING, snap)
        g = self.p.gains
        self._final_limit = self.p.approach_distance + self.p.blind_extra_distance
        fast = max(0.0, self.p.approach_distance - g.slow_zone)
        slow = self._final_limit - fast
        self._final_timeout = (fast / max(g.max_speed, 1e-3) + slow / max(g.final_speed, 1e-3)
                               + self.p.blind_timeout_margin_s)

    def _enter_vision_final(self, snap: Snapshot) -> None:
        self._blind = False
        self._final_entry_remaining = max(0.0, self._remaining)
        self._enter(DockState.FINAL_DOCKING, snap)
        self._final_limit = self.p.final_max_distance
        self._final_timeout = self.p.final_timeout_s

    def _enter_retry(self, snap: Snapshot, travelled_reverse: float) -> None:
        if self.retries >= self.p.max_retries:
            self._fail(DockMsg.DOCK_MAXOUT, snap)
            return
        self.retries += 1
        if self.p.skip_nav_to_approach and (self._blind or not self.use_vision):
            # no Nav2 to re-approach: drive back to where the attempt started
            self._retry_distance = min(max(travelled_reverse, self.p.retry_forward_distance),
                                       self.p.approach_distance + self.p.blind_extra_distance)
        else:
            self._retry_distance = self.p.retry_forward_distance
        self._enter(DockState.RETRY, snap)

    def _fresh_marker(self, snap: Snapshot) -> Optional[MarkerObs]:
        m = snap.marker
        if m is None or snap.t - m.stamp > self.p.marker_timeout_s:
            return None
        return m

    # -- API -------------------------------------------------------------
    def start(self, snap: Snapshot) -> Output:
        self._t_start = snap.t
        self.retries = 0
        if snap.contact:
            self.state = DockState.CHARGING
            self._t_state = snap.t
            self.message = DockMsg.ALREADY_DOCKED
            return self._out(requests=[('enable_charging',)])
        if self.p.skip_nav_to_approach:
            self._post_approach(snap)
            return self._out()
        self._enter(DockState.NAV_TO_APPROACH, snap)
        return self._out(requests=[('start_nav', self.approach)])

    def cancel(self) -> Output:
        reqs = [('cancel_nav',)] if self.state == DockState.NAV_TO_APPROACH else []
        self.state = DockState.FAILED
        self.message = DockMsg.CANCELED
        return self._out(requests=reqs)

    def _out(self, v: float = 0.0, w: float = 0.0, requests=None) -> Output:
        self._last_cmd = v
        reqs = self._pending + list(requests or [])
        self._pending = []
        done = self.state in (DockState.SUCCEEDED, DockState.FAILED)
        if done:
            v = w = 0.0
        return Output(linear=v, angular=w, state=self.state, done=done,
                      success=self.state == DockState.SUCCEEDED, message=self.message,
                      remaining=self._remaining, retries=self.retries,
                      travelled=self._tracker.distance, requests=reqs)

    def step(self, snap: Snapshot) -> Output:
        if self.state in (DockState.SUCCEEDED, DockState.FAILED):
            return self._out()
        travelled = self._tracker.update(snap.t, snap.odom, self._last_cmd)

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
        if snap.contact and self.state in (DockState.SEARCHING, DockState.DOCKING,
                                           DockState.FINAL_DOCKING):
            self.state = DockState.CHARGING
            self._t_state = snap.t
            self._remaining = 0.0
            return self._out(requests=[('enable_charging',)])

        st = self.state
        el = snap.t - self._t_state
        g = self.p.gains

        if st == DockState.NAV_TO_APPROACH:
            if snap.nav_status == 'succeeded':
                self._post_approach(snap)
            elif snap.nav_status == 'failed':
                self.state = DockState.FAILED
                self.message = DockMsg.NAV_TO_DOCK_FAILED
            elif el > self.p.nav_timeout_s:
                self._fail(DockMsg.NAV_TO_DOCK_FAILED, snap)
            return self._out()

        if st == DockState.SEARCHING:
            m = self._fresh_marker(snap)
            if m is not None:
                err = dock_errors(m, self.p.docked_marker_offset)
                self._remaining = err.remaining
                if marker_gate_ok(err, self.p.max_lateral_error,
                                  math.radians(self.p.max_yaw_error_deg)):
                    self._lost = 0
                    self._enter(DockState.DOCKING, snap)
                    return self._out()
            if el > self.p.search_timeout_s:
                self._enter_blind(snap)
            return self._out()

        if st == DockState.DOCKING:
            if el > self.p.docking_timeout_s:
                self._enter_retry(snap, travelled)
                return self._out()
            m = self._fresh_marker(snap)
            new = m is not None and m.stamp != self._last_marker_stamp
            err = dock_errors(m, self.p.docked_marker_offset) if m is not None else None
            if err is None or not marker_gate_ok(err, self.p.max_lateral_error,
                                                 math.radians(self.p.max_yaw_error_deg)):
                self._lost += 1
                if self._lost > self.p.max_lost_frames:
                    self._enter_retry(snap, travelled)
                return self._out()  # hold still while the marker is lost
            if new:
                self._lost = 0
                self._last_marker_stamp = m.stamp
            self._remaining = err.remaining
            if err.remaining <= g.final_zone:
                self._enter_vision_final(snap)
                return self._out(-g.final_speed, 0.0)
            dt = snap.t - self._prev_t if self._prev_t is not None else 0.05
            self._prev_t = snap.t
            v, w, self._prev_e = reverse_control(err, g, self._prev_e, dt)
            return self._out(v, w)

        if st == DockState.FINAL_DOCKING:
            if travelled >= self._final_limit or el > self._final_timeout:
                self._enter_retry(snap, travelled)
                return self._out()
            if self._blind:
                left = self._final_limit - travelled
                self._remaining = max(0.0, left - self.p.blind_extra_distance)
                speed = reverse_speed(self._remaining, g)
            else:
                self._remaining = max(0.0, self._final_entry_remaining - travelled)
                speed = g.final_speed
            w = heading_hold(self._hold_yaw, snap.odom.yaw if snap.odom else None,
                             self.p.heading_hold_kp, g.max_angular) \
                if self._hold_yaw is not None else 0.0
            return self._out(-speed, w)

        if st == DockState.RETRY:
            if travelled >= self._retry_distance:
                if self.p.skip_nav_to_approach:
                    self._post_approach(snap)
                    return self._out()
                self._enter(DockState.NAV_TO_APPROACH, snap)
                return self._out(requests=[('start_nav', self.approach)])
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
