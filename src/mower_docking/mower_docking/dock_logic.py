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
from dataclasses import dataclass, field, replace
from typing import List, Optional, Sequence, Tuple

from mower_common import geometry as rgeo
from mower_common.angles import wrap_angle, yaw_from_quaternion  # noqa: F401 (re-exported)

# --------------------------------------------------------------------------
# small maths helpers (pure python; numpy only where a 3x3 matrix is handy)
# --------------------------------------------------------------------------


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v



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
    z: float = 0.0        # marker centre height in base_link (obstacle_guard dock foot)


def marker_in_base(rvec: Sequence[float], tvec: Sequence[float],
                   cam_xyz: Sequence[float], cam_rpy: Sequence[float],
                   stamp: float = 0.0, marker_id: int = -1,
                   yaw_correction: float = 0.0) -> MarkerObs:
    """Convert an OpenCV marker pose (marker in the camera OPTICAL frame,
    i.e. solvePnP ``rvec``/``tvec``) into a base_link :class:`MarkerObs`.

    ``cam_xyz``/``cam_rpy`` is the pose of the camera optical frame in
    base_link, exactly the URDF ``rear_camera_joint`` origin
    (``-0.201 0 0.25`` / ``-1.5708 0 1.5708``).

    ``yaw_correction`` (rad) rotates the camera about the base_link vertical axis through the
    camera centre (R_bc' = Rz(yaw_correction) R_bc): the measured mounting-yaw error of the
    rear camera (2026-10-21 night, -6.5 deg, docking.yaml ``rear_camera_yaw_correction_deg``).
    """
    r_bc = rpy_to_matrix(*cam_rpy)
    if yaw_correction:
        cz, sz = math.cos(yaw_correction), math.sin(yaw_correction)
        r_bc = _matmul([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]], r_bc)
    r_cm = rodrigues(rvec)
    p = _matvec(r_bc, [float(t) for t in tvec])
    p = [p[i] + float(cam_xyz[i]) for i in range(3)]
    r_bm = _matmul(r_bc, r_cm)
    nx, ny = r_bm[0][2], r_bm[1][2]  # marker +Z axis (outward normal)
    return MarkerObs(p[0], p[1], math.atan2(ny, nx), stamp, marker_id, p[2])


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


def dock_errors_with_heading(marker: MarkerObs, heading: float,
                             docked_offset: float) -> DockErrors:
    """Errors of the robot w.r.t. the docked pose from the marker POSITION only, with the
    robot's docked-frame ``heading`` supplied from elsewhere (the filter, i.e. odometry).

    At 30-50 px the marker's normal (yaw) is the ambiguous part of the planar solve (IPPE
    flips 22 deg on 29 % of the 2026-10-21 night detections) while its centre is well
    determined. :func:`dock_errors` derives the lateral THROUGH the marker yaw, so each flip
    also moved the lateral 0.35 m; here a flip cannot touch the position.

    Docked frame: the marker sits at (-docked_offset, 0); the robot at P with heading h sees
    it at m = R(-h) (M - P), so P = M - R(h) m. With ``heading = -marker.yaw`` this equals
    :func:`dock_errors`."""
    c, s = math.cos(heading), math.sin(heading)
    return DockErrors(remaining=-docked_offset - (c * marker.x - s * marker.y),
                      lateral=-(s * marker.x + c * marker.y),
                      heading=wrap_angle(heading))


def propagate_errors(e: DockErrors, odom_from: Optional['Pose2D'],
                     odom_to: Optional['Pose2D']) -> DockErrors:
    """Move a docked-frame estimate by the odometry displacement between two odom poses
    (robot frame, so any odom frame works)."""
    if odom_from is None or odom_to is None:
        return e
    dx, dy = odom_to.x - odom_from.x, odom_to.y - odom_from.y
    c, s = math.cos(odom_from.yaw), math.sin(odom_from.yaw)
    df, dl = c * dx + s * dy, -s * dx + c * dy
    ch, sh = math.cos(e.heading), math.sin(e.heading)
    return DockErrors(e.remaining + df * ch - dl * sh, e.lateral + df * sh + dl * ch,
                      wrap_angle(e.heading + wrap_angle(odom_to.yaw - odom_from.yaw)))


def median_errors(es: Sequence[DockErrors]) -> Optional[DockErrors]:
    """Component-wise median (heading as a median of offsets from the first, so it is
    safe across +-pi)."""
    if not es:
        return None

    def med(vals):
        v = sorted(vals)
        k = len(v)
        return v[k // 2] if k % 2 else 0.5 * (v[k // 2 - 1] + v[k // 2])
    h0 = es[0].heading
    return DockErrors(med([e.remaining for e in es]), med([e.lateral for e in es]),
                      wrap_angle(h0 + med([wrap_angle(e.heading - h0) for e in es])))


def marker_gate_ok(err: DockErrors, max_lateral: float, max_yaw_rad: float) -> bool:
    """Vendor: reject a marker estimate worse than 0.5 m / 25 deg."""
    return abs(err.lateral) <= max_lateral and abs(err.heading) <= max_yaw_rad


def tracking_gate_ok(err: DockErrors, g: 'ControllerGains', max_lateral: float,
                     max_yaw_rad: float) -> bool:
    """Marker gate relative to what the reverse controller WANTS: the heading is compared
    with the controller's desired approach heading h_d = clamp(k_lateral * lateral), not with
    the dock axis. 2026-10-08 dusk: at the approach pose the robot sat 0.27 m off the axis at
    -29 deg, i.e. already ON the controller's -28.6 deg approach line, yet the raw 25 deg gate
    rejected it for 15 s (SEARCHING), and in DOCKING the controller's own approach angle
    (+ tracking error) pushed the raw heading past the 35 deg widened gate -> 'lost' ->
    RETRY. With the blind fallback gone this ended in DOCK_MAXOUT / DOCK_NOT_FOUND."""
    h_d = clamp(g.k_lateral * err.lateral, -g.max_approach_angle, g.max_approach_angle)
    return abs(err.lateral) <= max_lateral and \
        abs(wrap_angle(err.heading - h_d)) <= max_yaw_rad


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
    # 2026-10-21 night: k 7 saturated h_d at 28.6 deg from 0.07 m of lateral, w changed sign
    # 0.9-1.8 times per second and the heading swung 0 -> 40 deg; 2.5 / 15 deg tracks the
    # same lateral without the swing (field replay, mower_docking README).
    k_lateral: float = 2.5          # rad of desired heading per metre of lateral error
    max_approach_angle: float = 0.26    # rad (15 deg)
    max_angular_accel: float = 0.6  # rad/s^2 rate limit on the reverse w (0 = off)
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
    # Both wheels (2026-10-16 sim gate: 80 one-wheel ticks on /cmd_vel_docking per dock): the
    # creep floor with a large heading error asked for e.g. v -0.035, w 0.3, an inner wheel
    # of -0.037 m/s: under the MCU's 0.06 m/s floor it does not turn, the robot pivots on it
    # (a one-wheel turn). A command whose inner wheel would fall in (-wheel_floor, 0) is a
    # proper reversing arc instead (|v| raised to wheel_floor + |w| track_width / 2, both
    # wheels rolling backwards; |w| lowered when that would exceed max_speed); within
    # final_zone of the contacts |w| is lowered to keep the inner wheel at 0 instead. The [0, wheel_floor) band
    # (small steering on the reverse, 2026-10-07) is unchanged. wheel_floor 0 = off (revert).
    track_width: float = rgeo.TRACK_SPLIT_M   # the MCU's wheel split 0.40 (was 0.48)
    wheel_floor: float = 0.06


def reverse_speed(remaining: float, g: ControllerGains, near: Optional[float] = None) -> float:
    """Reverse speed magnitude: close_speed within ``slow_distance`` of the
    contacts, ramping up to max_speed over ``slow_zone`` beyond it."""
    frac = clamp((remaining - g.slow_distance) / max(g.slow_zone, 1e-6), 0.0, 1.0)
    near = g.close_speed if near is None else near
    return near + (g.max_speed - near) * frac


def reverse_control(err: DockErrors, g: ControllerGains,
                    prev_heading_err: Optional[float] = None,
                    dt: float = 0.05, prev_w: Optional[float] = None) -> Tuple[float, float, float]:
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
    if prev_w is not None and g.max_angular_accel > 0 and dt > 0:
        # angular rate limit BEFORE the wheel-floor shaping below (which may only raise |v|
        # or lower |w| further, keeping both wheels at/above the floor)
        dw = g.max_angular_accel * dt
        w = clamp(w, prev_w - dw, prev_w + dw)
    vmax = reverse_speed(err.remaining, g)
    frac = 1.0
    if g.turn_slow_err > 0:
        frac = clamp(1.0 - abs(e) / g.turn_slow_err, 0.0, 1.0)
    if err.remaining <= g.slow_distance and g.lateral_slow > 0:
        # slow and steady near the contacts: the larger the residual offset, the slower,
        # so the steering gets more travel-per-correction to null it before contact
        frac = min(frac, clamp(1.0 - abs(err.lateral) / g.lateral_slow, 0.0, 1.0))
    vmax = min(vmax, max(g.min_turn_speed, vmax * frac))
    if g.wheel_floor > 0.0 and abs(w) > 1e-6:
        inner = vmax - abs(w) * 0.5 * g.track_width
        if -g.wheel_floor < inner < 0.0 and err.remaining <= g.final_zone:
            return -vmax, math.copysign(vmax / (0.5 * g.track_width) * (1.0 - 1e-6), w), e
        if -g.wheel_floor < inner < 0.0:
            # a proper reversing arc: both wheels rolling backwards, the inner one at the
            # floor (2026-10-16 sim gate: an in-place pivot here dithered with the marker's
            # heading noise and tripped the stuck guard twice -> STUCK_NEEDS_HELP)
            half = 0.5 * g.track_width
            v_arc = g.wheel_floor + abs(w) * half
            if v_arc <= g.max_speed:
                return -v_arc, w, e
            return -g.max_speed, math.copysign((g.max_speed - g.wheel_floor) / half, w), e
    return -vmax, w, e


def heading_hold(target_yaw: float, yaw: Optional[float], kp: float, max_w: float) -> float:
    if yaw is None:
        return 0.0
    return clamp(kp * wrap_angle(target_yaw - yaw), -max_w, max_w)


def rtk_dock_errors(pose: Pose2D, dock: Pose2D) -> DockErrors:
    """Robot (map pose) in the docked frame, same convention as :func:`dock_errors`:
    ``remaining`` = distance out along the dock axis, ``lateral`` left positive,
    ``heading`` relative to the docked heading (0 = facing away from the charger)."""
    dx, dy = pose.x - dock.x, pose.y - dock.y
    c, s = math.cos(dock.yaw), math.sin(dock.yaw)
    return DockErrors(c * dx + s * dy, -s * dx + c * dy, wrap_angle(pose.yaw - dock.yaw))


class MarkerFilter:
    """Kalman filter of the robot pose in the docked (marker) frame.

    State = :class:`DockErrors` (remaining, lateral, heading), diagonal covariance.
    ``predict`` integrates the odometry displacement since the previous call (robot frame,
    so it does not matter which odom frame it is); ``correct`` fuses one measurement.

    Two modes:

    * ``gate_chi2 <= 0`` (stock ``LineTrackingDocking``): process noise ``q`` per predict
      call, innovations CLAMPED at ``max_innov_m`` / ``max_innov_rad``. 2026-10-21 night: at
      q 0.01 per 20 Hz tick and markers at ~4 Hz the gain was ~0.5, so half of every clamped
      20 deg / 0.5 m flip went into the estimate.
    * ``gate_chi2 > 0`` (default since 2026-10-21): ``q`` / ``q_heading`` are per SECOND
      (predict gets ``t``), and the position (2-DOF, threshold ``gate_chi2``) and heading
      (1-DOF, ``gate_chi2_heading``) innovations are gated by their Mahalanobis distance:
      a flip is rejected, not half-trusted. most of the last ``reinit_after`` rejected but
      mutually consistent measurements (3/4 of the last ``reinit_after``) re-initialise the filter on them (a wrong start, or
      odometry that really slipped) instead of rejecting forever.
    """

    def __init__(self, q: float = 0.01, r: float = 0.1, max_innov_m: float = 0.5,
                 max_innov_rad: float = math.radians(20.0), gate_chi2: float = 0.0,
                 gate_chi2_heading: float = 3.84, q_heading: Optional[float] = None,
                 r_heading: Optional[float] = None, reinit_after: int = 8):
        self.q, self.r = float(q), float(r)
        self.q_h = self.q if q_heading is None else float(q_heading)
        self.r_h = self.r if r_heading is None else float(r_heading)
        self.max_innov_m, self.max_innov_rad = float(max_innov_m), float(max_innov_rad)
        self.gate_chi2, self.gate_chi2_h = float(gate_chi2), float(gate_chi2_heading)
        self.reinit_after = int(reinit_after)
        self._odom: Optional[Pose2D] = None
        self._t: Optional[float] = None
        self.reset()

    def reset(self) -> None:
        self.x: Optional[DockErrors] = None
        self.P = [0.0, 0.0, 0.0]
        self.clamped = 0
        self.rejected_pos = 0
        self.rejected_heading = 0
        self.updates = 0
        self.reinits = 0
        self._hist: List[Tuple[bool, DockErrors]] = []   # (rejected, z), last reinit_after

    def init(self, z: DockErrors) -> None:
        self.x = z
        self.P = [self.r, self.r, self.r_h]
        self._hist = []

    def predict(self, odom: Optional[Pose2D], t: Optional[float] = None) -> None:
        prev, self._odom = self._odom, odom
        tp, self._t = self._t, t
        if self.x is None or odom is None or prev is None:
            return
        self.x = propagate_errors(self.x, prev, odom)
        self._hist = [(r, propagate_errors(z, prev, odom)) for r, z in self._hist]
        if self.gate_chi2 > 0 and t is not None:
            dt = clamp(t - tp, 0.0, 1.0) if tp is not None else 0.0
            self.P = [self.P[0] + self.q * dt, self.P[1] + self.q * dt,
                      self.P[2] + self.q_h * dt]
        else:
            self.P = [p + self.q for p in self.P]

    def correct(self, z: DockErrors) -> DockErrors:
        if self.x is None:
            self.init(z)
            return self.x
        inn = [z.remaining - self.x.remaining, z.lateral - self.x.lateral,
               wrap_angle(z.heading - self.x.heading)]
        R = [self.r, self.r, self.r_h]
        if self.gate_chi2 > 0:
            d_pos = inn[0] ** 2 / (self.P[0] + R[0]) + inn[1] ** 2 / (self.P[1] + R[1])
            d_h = inn[2] ** 2 / (self.P[2] + R[2])
            use_pos, use_h = d_pos <= self.gate_chi2, d_h <= self.gate_chi2_h
            if not use_pos:
                self.rejected_pos += 1
            if not use_h:
                self.rejected_heading += 1
            n = self.reinit_after
            if n > 0:
                self._hist.append((not (use_pos and use_h), z))
                del self._hist[:-n]
                rej = [e for r, e in self._hist if r]
                if len(rej) * 4 >= 3 * n:
                    # most of the recent measurements disagree with the estimate: if they
                    # agree with EACH OTHER (a cluster of >= 3/4 n), the estimate is the
                    # outlier (a flipped start, or real slip) -> restart on that cluster
                    m = median_errors(rej)
                    near = [e for e in rej
                            if abs(wrap_angle(e.heading - m.heading)) <= math.radians(8.0) and
                            math.hypot(e.remaining - m.remaining, e.lateral - m.lateral) <= 0.15]
                    if len(near) * 4 >= 3 * n:
                        self.init(median_errors(near))
                        self.reinits += 1
                        return self.x
            k = [self.P[i] / (self.P[i] + R[i]) for i in range(3)]
            if not use_pos:
                k[0] = k[1] = 0.0
            if not use_h:
                k[2] = 0.0
        else:
            lim = [self.max_innov_m, self.max_innov_m, self.max_innov_rad]
            if any(abs(i) > l for i, l in zip(inn, lim)):
                self.clamped += 1
            inn = [clamp(i, -l, l) for i, l in zip(inn, lim)]
            k = [self.P[i] / (self.P[i] + R[i]) for i in range(3)]
        self.x = DockErrors(self.x.remaining + k[0] * inn[0], self.x.lateral + k[1] * inn[1],
                            wrap_angle(self.x.heading + k[2] * inn[2]))
        self.P = [(1.0 - kk) * p for kk, p in zip(k, self.P)]
        self.updates += 1
        return self.x


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
    nav_status: Optional[str] = None         # None|'running'|'succeeded'|'failed'|'unavailable'
    charging_result: Optional[bool] = None   # None = pending/not requested
    rtk_fixed: bool = False
    progress_pose: Optional[Pose2D] = None   # fused pose (/odometry/filtered) for the stall guard
    dig_stall: bool = False                  # slip_detector /dig_stall (latched Bool)
    map_pose: Optional[Pose2D] = None        # robot pose in the map frame (approach skip / align)
    raw_contact: bool = False                # latest undebounced is_docking_done
    is_charging: bool = False                # MowerBaseDevStatus.is_charging
    bumper: bool = False                     # bumper pressed OR bumper_controller manoeuvring
    rear_blocked: bool = False               # obstacle_guard /vision/rear_blocked (fresh)
    teleop: bool = False                     # operator joystick (/cmd_vel_teleop) active now


class StallGuard:
    """Wheel stall / dig detector: commanded speed vs. measured (EKF) progress.

    Over a sliding window of ``timeout_s`` with every sample commanding
    ``|v| > min_cmd``, stall = measured displacement < ``ratio`` * expected
    (integral of |v| dt).  A missing ``progress_pose`` restarts the window.

    Pivot tolerance: wheel travel from turning (``|d yaw| * turn_radius``, summed per
    sample) counts as progress, so a downstream shaper that turns a slow steered
    command into an in-place pivot (v=0 at the wheels, 2026-10-07) is not a "stall".
    """

    def __init__(self, min_cmd: float = 0.03, ratio: float = 0.2, timeout_s: float = 1.0,
                 turn_radius: float = 0.2):
        self.min_cmd = min_cmd
        self.ratio = ratio
        self.timeout_s = timeout_s
        self.turn_radius = turn_radius
        self.reset()

    def reset(self) -> None:
        self._t0: Optional[float] = None
        self._p0: Optional[Pose2D] = None
        self._expected = 0.0
        self._last_t: Optional[float] = None
        self._last_yaw: Optional[float] = None
        self._turned = 0.0
        self.measured = 0.0

    def update(self, t: float, pose: Optional[Pose2D], cmd: float) -> bool:
        if pose is None or abs(cmd) <= self.min_cmd or self.timeout_s <= 0:
            self.reset()
            return False
        if self._t0 is None:
            self._t0, self._p0, self._last_t, self._expected = t, pose, t, 0.0
            self._last_yaw, self._turned = pose.yaw, 0.0
            return False
        self._expected += abs(cmd) * max(0.0, t - self._last_t)
        self._last_t = t
        if self._last_yaw is not None:
            self._turned += abs(wrap_angle(pose.yaw - self._last_yaw)) * self.turn_radius
        self._last_yaw = pose.yaw
        self.measured = math.hypot(pose.x - self._p0.x, pose.y - self._p0.y)
        if t - self._t0 < self.timeout_s:
            return False
        if self.measured + self._turned < self.ratio * self._expected:
            return True
        # still moving: restart the window from here
        self._t0, self._p0, self._expected, self._turned = t, pose, 0.0, 0.0
        return False


class PostFailWatch:
    """After a failed dock goal: did the robot end on the contacts anyway?

    ``step`` returns ``'enable'`` (is_docking_done seen, not charging: enable charging),
    ``'charging'`` (already charging), ``'abort'`` (a new goal runs, or the window
    expired) or ``None`` (keep watching)."""

    def __init__(self, t0: float, window_s: float):
        self.t0 = t0
        self.window_s = window_s
        self._contact = False

    def step(self, t: float, docking_done: bool, is_charging: bool, busy: bool) -> Optional[str]:
        if busy:
            return 'abort'
        self._contact = self._contact or bool(docking_done)
        if self._contact or is_charging:
            return 'charging' if is_charging else 'enable'
        if t - self.t0 > self.window_s:
            return 'abort'
        return None


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
    stall_turn_radius_m: float = 0.2          # pivot wheel travel = |d yaw| * this (progress)
    # A stall in DOCKING / FINAL_DOCKING is usually the robot pushing against the dock
    # before the 3-sample contact debounce confirms (2026-10-07: DOCK_STALLED, then
    # is_docking_done 8 s later). Hold still this long for ANY contact signal (raw,
    # debounced, is_charging) before failing DOCK_STALLED.
    stall_contact_grace_s: float = 3.0
    # Approach-pose shortcuts (2026-10-07: Nav2 "Failed to make progress" 0.25 m from the
    # approach pose because the remaining in-place pivot does not turn on turf).
    approach_skip_radius_m: float = 0.6       # already this close: skip Nav2, align + search
    nav_fail_arrived_radius_m: float = 1.0    # Nav2 failed within this: treat as arrived
    relaxed_nav_retry: bool = True            # beyond it: one Nav2 retry with the current yaw
    # Owner rule (2026-10-07): "go in front of dock, then do an about turn to have the rear
    # camera face the dock". Nav2 is sent to the approach point FACING the dock (dock yaw +
    # 180 deg, the natural arrival direction: no end-of-path spin), then ALIGNING pivots
    # in place (the turn shaper passes proper pivots, |w| >= 0.3) the shorter way round.
    approach_facing_dock: bool = True
    align_angular: float = 0.3                # rad/s, true pivot at the 0.3 floor (b 0.40; was 0.25)
    align_creep: float = 0.0                  # m/s forward while turning (0 = pure pivot)
    align_max_travel: float = 0.4
    align_timeout_s: float = 25.0
    align_tolerance_deg: float = 10.0
    # DOCKING: once tracking, the marker gate is widened by this factor (hysteresis), so a
    # robot that entered at the gate edge is not dropped while it turns onto the axis.
    docking_gate_scale: float = 1.4
    # Reaching final_zone BADLY off the axis -> drive forward along the dock heading and
    # re-approach (does not use a retry), at most max_realigns times per goal. Smaller
    # errors are corrected while reversing (the controller steers all the way to contact).
    final_max_lateral: float = 0.08
    final_max_heading_deg: float = 12.0
    max_realigns: int = 2
    marker_median_n: int = 3         # gates/decisions use the median of the last N markers
    # DOCKING: hold still (never reverse) while the marker is lost; RETRY only after it has
    # been lost for max_lost_frames ticks AND this long (dusk: intermittent detections).
    marker_lost_hold_s: float = 3.0
    # Bumper during docking (2026-10-08: a bike pressed the front bumper mid-ALIGNING; the
    # bumper_controller backed off + rotated over the docking lane while the FSM's timers ran
    # on, the about-turn ended off the approach line, SEARCHING timed out and the robot
    # reversed BLIND onto the dock). Now: stop, wait until the bumper is released and the
    # bumper manoeuvre is over for bumper_clear_s (at most bumper_wait_max_s, then
    # DOCK_BLOCKED), then resume from the current pose (ALIGNING re-plans the about-turn).
    bumper_clear_s: float = 2.0
    bumper_wait_max_s: float = 30.0
    # Rear obstacle in the dock corridor (2026-10-09, obstacle_guard /vision/rear_blocked:
    # a living thing BETWEEN the robot and the dock; people at/beside/behind the charger are
    # already filtered out there). SEARCHING / DOCKING / FINAL_DOCKING hold still (no abort,
    # state timers and the goal timeout paused) until it has been clear for rear_clear_s,
    # then continue where they were; blocked longer than rear_wait_max_s -> DOCK_BLOCKED.
    rear_hold: bool = True
    rear_clear_s: float = 1.5
    rear_wait_max_s: float = 60.0
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
    # ---- 2026-10-21 stock-structure port (docs/analysis/2026-10-21_stock_firmware_findings.md)
    # Search gate: the stock 0.5 m / 25 deg CheckInitJump has no call site (dead code). Off:
    # any detection starts DOCKING (after search_min_detections), ALIGNING keeps the gate.
    use_marker_gate: bool = False
    search_min_detections: int = 1
    # Retries: re-approach with a lateral offset from the dock axis, alternating and growing
    # (retry 1: +a0 * side, 2: -a0 * side, 3: +a1 * side ... then 0 again); side = the side
    # of the axis the marker was last seen from (+1 when never seen).
    retry_lateral_offsets: Tuple[float, ...] = (0.1, 0.2, 0.3)
    # RTK lateral pre-alignment (stock adjust step 2): before SEARCHING, more than
    # lateral_align_tolerance off the (offset) dock axis -> drive forward on an arc/line back
    # onto it (both wheels >= wheel_floor), then reverse on RTK to approach_distance.
    lateral_align: bool = True
    lateral_align_require_rtk: bool = True
    lateral_align_tolerance: float = 0.15
    lateral_align_done: float = 0.04
    lateral_align_heading_deg: float = 8.0
    lateral_align_speed: float = 0.12
    lateral_align_k: float = 2.0
    lateral_align_max_angle: float = 0.5
    lateral_align_max_travel: float = 1.5
    lateral_align_timeout_s: float = 40.0
    # Marker Kalman filter + reverse on the prediction while the marker is briefly lost
    marker_filter: bool = True
    kf_q: float = 0.01
    kf_r: float = 0.05               # m^2 (stock 0.1 with clamping; docking.yaml)
    kf_max_innov_m: float = 0.5
    kf_max_innov_deg: float = 20.0
    lost_predict_s: float = 2.0
    lost_speed: float = 0.08
    # Pose check (stock): at this MARKER distance (remaining + docked_marker_offset) check
    # final_max_lateral / final_max_heading_deg once per approach; 0 = old final_zone check.
    pose_check_distance: float = 0.65
    # ---- 2026-10-21 night field fixes (findings doc "Field docking 2026-10-21 night")
    # Marker yaw is used for the HEADING only through the filter: the position is taken
    # from the marker centre with the filter's (odometry-propagated) heading, and the
    # heading/position innovations are chi-square gated (MarkerFilter gate mode).
    marker_position_only: bool = True
    kf_gate_chi2: float = 9.21          # 2-DOF 99 %; 0 = stock clamping (kf_max_innov_*)
    kf_gate_chi2_heading: float = 3.84  # 1-DOF 95 % (a 22 deg flip must not squeeze through)
    kf_q_heading: float = 0.0005        # rad^2/s (odometry yaw over a 10 s reverse: < 1 deg)
    kf_r_heading: float = 0.006         # rad^2 (4.4 deg 1-sigma; field inliers 3.9 deg)
    kf_reinit_after: int = 8
    # SEARCHING: the accepted detections must agree with each other ...
    search_consistency_deg: float = 10.0
    search_consistency_m: float = 0.15
    # ... and (dock pose measured, RTK fixed) with RTK; 0 = no RTK check. The night's first
    # solve was -167 deg (RTK 11 deg).
    rtk_check_max_deg: float = 30.0
    rtk_check_max_m: float = 0.3
    pose_check_median_n: int = 5        # pose check on the median of the last N detections
    # Forward manoeuvres (realign, retry, LATERAL_ALIGN) never take the robot further than
    # approach_distance + this from the dock (RTK; the "Dock path" corridor ends ~1.15 m out).
    # 0 = no cap.
    forward_cap_margin: float = 0.25
    # Operator joystick (/cmd_vel_teleop overrides /cmd_vel_docking in twist_mux): pause,
    # resume from the current pose once released for teleop_clear_s; fail after
    # teleop_wait_max_s. A joystick dock still ends DOCKED (contacts are checked first).
    teleop_hold: bool = True
    teleop_clear_s: float = 1.5
    teleop_wait_max_s: float = 120.0
    gains: ControllerGains = field(default_factory=ControllerGains)


class DockState:
    NAV_TO_APPROACH = 'NAV_TO_APPROACH'
    ALIGNING = 'ALIGNING'
    LATERAL_ALIGN = 'LATERAL_ALIGN'
    SEARCHING = 'SEARCHING'
    DOCKING = 'DOCKING'
    FINAL_DOCKING = 'FINAL_DOCKING'
    RETRY = 'RETRY'
    CHARGING = 'CHARGING'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'


# NAV_TO_DOCK_FAILED detail when the navigate_to_pose server is missing (nav_status
# 'unavailable'): Nav2 is down, not the dock (2026-10-13, a hung nav2 lifecycle manager).
NAV2_UNAVAILABLE = 'Nav2 navigate_to_pose action server unavailable (navigation stack not running)'


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
    DOCK_BLOCKED = 'DOCK_BLOCKED'


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
                                 params.stall_timeout_s, params.stall_turn_radius_m)
        self._stall_hold: Optional[Tuple[float, str]] = None   # (t, failure message)
        self._dig_at_start = False
        self.use_vision = params.use_vision if use_vision is None else bool(use_vision)
        self.dock_pose = dock_pose
        self.approach = approach_pose(dock_pose, params.approach_distance)
        # Nav2 goal at the approach point: facing the dock (about-turn done by ALIGNING)
        self.approach_goal = Pose2D(
            self.approach.x, self.approach.y,
            wrap_angle(self.approach.yaw + (math.pi if params.approach_facing_dock else 0.0)))
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
        self._obs: List[Tuple[float, DockErrors, Optional[Pose2D]]] = []  # (stamp, err, odom)
        self._last_err: Optional[DockErrors] = None      # latest DOCKING observation
        self._last_err_odom: Optional[Pose2D] = None
        self.calibrated_offset: Optional[float] = None
        self._bump: Optional[dict] = None
        self._lost_t = 0.0   # bumper hold: {'t0', 'clear_t', 'state'}
        self._rear: Optional[dict] = None    # rear obstacle hold: {'t0', 'clear_t', 'state'}
        self._paused_s = 0.0                 # time spent in rear holds (goal timeout)
        self._kf = MarkerFilter(params.kf_q, params.kf_r, params.kf_max_innov_m,
                                math.radians(params.kf_max_innov_deg),
                                gate_chi2=params.kf_gate_chi2,
                                gate_chi2_heading=params.kf_gate_chi2_heading,
                                q_heading=params.kf_q_heading, r_heading=params.kf_r_heading,
                                reinit_after=params.kf_reinit_after)
        self._teleop: Optional[dict] = None  # joystick hold: {'t0', 'clear_t', 'state'}
        self._prev_w: Optional[float] = None
        self._kf_notes = (0, 0)
        self._offset = 0.0                   # lateral offset of this attempt's approach line
        self._side = 1.0                     # side of the axis the marker was last seen from
        self._lat_done = False               # lateral pre-alignment done for this approach
        self._lat_phase = 'forward'
        self._pose_checked = False
        self._search_n = 0                   # distinct marker detections this SEARCHING

    # -- helpers ---------------------------------------------------------
    def _enter(self, state: str, snap: Snapshot) -> None:
        self.state = state
        self._t_state = snap.t
        self._tracker.start(snap.t, snap.odom)
        self._hold_yaw = snap.odom.yaw if snap.odom is not None else None
        self._prev_e = None
        self._prev_w = None
        self._prev_t = None
        self._stall.reset()

    def _fail(self, msg: str, snap: Snapshot) -> None:
        if self.state == DockState.NAV_TO_APPROACH:
            self._pending.append(('cancel_nav',))
        self.state = DockState.FAILED
        self.message = msg

    def _gate(self, err: Optional[DockErrors], scale: float = 1.0, force: bool = False) -> bool:
        if err is None:
            return False
        if not self.p.use_marker_gate and not force:
            return True      # stock: the 0.5 m / 25 deg check is dead code
        k = max(1.0, scale)
        lat, yaw = k * self.p.max_lateral_error, k * math.radians(self.p.max_yaw_error_deg)
        # raw dock-axis gate (vendor) OR on the controller's approach line
        return marker_gate_ok(err, lat, yaw) or tracking_gate_ok(err, self.p.gains, lat, yaw)

    def _post_approach(self, snap: Snapshot) -> None:
        if self.use_vision:
            if self._want_lateral_align(snap):
                return
            self._lost = 0
            self._obs = []
            self._kf.reset()
            self._search_n = 0
            self._enter(DockState.SEARCHING, snap)
        else:
            self._try_blind(snap)

    def _want_lateral_align(self, snap: Snapshot) -> bool:
        p = self.p
        if not p.lateral_align or self._lat_done or not self.dock_pose_measured or \
                snap.map_pose is None or (p.lateral_align_require_rtk and not snap.rtk_fixed):
            return False
        e = rtk_dock_errors(snap.map_pose, self.dock_pose)
        off = e.lateral - self._offset
        if abs(off) <= p.lateral_align_tolerance or abs(e.heading) > math.radians(45.0):
            return False
        self._lat_phase = 'forward'
        self._notes.append('RTK: %.2f m off the approach line (lateral %.2f, offset %.2f, '
                           'heading %.0f deg): lateral pre-alignment' % (
                               off, e.lateral, self._offset, math.degrees(e.heading)))
        self._enter(DockState.LATERAL_ALIGN, snap)
        return True

    def _set_offset(self, off: float) -> None:
        self._offset = off
        base = approach_pose(self.dock_pose, self.p.approach_distance)
        y = self.dock_pose.yaw
        self.approach = Pose2D(base.x - off * math.sin(y), base.y + off * math.cos(y), base.yaw)
        self.approach_goal = Pose2D(
            self.approach.x, self.approach.y,
            wrap_angle(self.approach.yaw + (math.pi if self.p.approach_facing_dock else 0.0)))

    def _offset_for(self, retry: int) -> float:
        mags = [abs(float(a)) for a in self.p.retry_lateral_offsets if abs(float(a)) > 0]
        seq = [0.0]
        for a in mags:
            seq += [a, -a]
        return self._side * seq[retry % len(seq)]

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

    def _push_obs(self, m: MarkerObs, err: DockErrors, odom: Optional[Pose2D] = None) -> None:
        if self._obs and self._obs[-1][0] == m.stamp:
            return
        self._obs.append((m.stamp, err, odom))
        n = max(1, int(self.p.marker_median_n), int(self.p.pose_check_median_n),
                int(self.p.search_min_detections))
        del self._obs[:-n]

    def _recent_obs(self, n: Optional[int] = None,
                    odom: Optional[Pose2D] = None) -> List[DockErrors]:
        """The last ``n`` observations, each moved to ``odom`` (now) by the odometry since."""
        n = max(1, int(self.p.marker_median_n if n is None else n))
        return [propagate_errors(e, o, odom) if odom is not None else e
                for _, e, o in self._obs[-n:]]

    def _median_err(self, n: Optional[int] = None,
                    odom: Optional[Pose2D] = None) -> Optional[DockErrors]:
        return median_errors(self._recent_obs(n, odom))

    def _measure(self, m: MarkerObs) -> DockErrors:
        """One marker measurement in the docked frame. Once the filter has a heading, the
        position comes from the marker centre with that heading (a pose flip cannot move
        it); the heading component stays the marker's own (the filter gates it)."""
        raw = dock_errors(m, self.p.docked_marker_offset)
        if self.p.marker_position_only and self.p.marker_filter and self._kf.x is not None:
            pos = dock_errors_with_heading(m, self._kf.x.heading, self.p.docked_marker_offset)
            return DockErrors(pos.remaining, pos.lateral, raw.heading)
        return raw

    def _rtk_remaining(self, snap: Snapshot) -> Optional[float]:
        if snap.map_pose is None or not self.dock_pose_measured:
            return None
        return rtk_dock_errors(snap.map_pose, self.dock_pose).remaining

    def _forward_cap(self) -> Optional[float]:
        m = self.p.forward_cap_margin
        return self.p.approach_distance + m if m > 0 else None

    def _rtk_disagrees(self, snap: Snapshot, med: DockErrors) -> str:
        p = self.p
        if (p.rtk_check_max_deg <= 0 and p.rtk_check_max_m <= 0) or snap.map_pose is None \
                or not self.dock_pose_measured or not snap.rtk_fixed:
            return ''
        r = rtk_dock_errors(snap.map_pose, self.dock_pose)
        dh = abs(wrap_angle(r.heading - med.heading))
        dlat = abs(r.lateral - med.lateral)
        if (p.rtk_check_max_deg > 0 and dh > math.radians(p.rtk_check_max_deg)) or \
                (p.rtk_check_max_m > 0 and dlat > p.rtk_check_max_m):
            return 'marker disagrees with RTK by %.0f deg / %.2f m lateral (max %.0f / %.2f)' % (
                math.degrees(dh), dlat, p.rtk_check_max_deg, p.rtk_check_max_m)
        return ''

    def _enter_retry(self, snap: Snapshot, travelled_reverse: float,
                     renav: bool = False) -> None:
        if self.retries >= self.p.max_retries:
            self._fail(DockMsg.DOCK_MAXOUT, snap)
            return
        self.retries += 1
        self._obs = []
        self._last_err = None
        self._kf.reset()
        self._lat_done = False
        self._set_offset(self._offset_for(self.retries))
        self._notes.append('retry %d/%d: approach line offset %+.2f m from the dock axis'
                           % (self.retries, self.p.max_retries, self._offset))
        if renav:
            # searching (not reversing): nothing to drive forward off, Nav2 re-approach now
            self._pending.extend(self._go_to_approach(snap, force=True))
            return
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
        cap = self._forward_cap()
        if cap is not None:
            rem = self._rtk_remaining(snap)
            if rem is None:
                rem = err.remaining
            self._retry_distance = clamp(cap - rem, 0.0, self.p.realign_forward_distance)
        self._notes.append('pose check: off the dock axis (lateral %.2f m, heading %.1f deg):'
                           ' realign %d/%d' % (err.lateral, math.degrees(err.heading),
                                               self._realigns, self.p.max_realigns))
        self._enter(DockState.RETRY, snap)
        self._obs = []
        self._last_err = None
        self._kf.reset()
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

    def _nav_failed(self, snap: Snapshot, timed_out: bool,
                    unavailable: bool = False) -> List[tuple]:
        d = self._dist_to_approach(snap)
        reqs: List[tuple] = [('cancel_nav',)] if timed_out else []
        why = 'timed out' if timed_out else 'unavailable' if unavailable else 'failed'
        if d is not None and d <= self.p.nav_fail_arrived_radius_m:
            self._notes.append('Nav2 to the approach pose %s %.2f m from it: treating as arrived'
                               % (why, d))
            self._arrive(snap)
            return reqs
        if unavailable:
            # No navigate_to_pose server (Nav2 stack not configured / not running): the
            # relaxed retry would only wait for the same missing server again. Fail now,
            # naming the cause (the mission shows it as the NAV_TO_DOCK_FAILED reason).
            self._notes.append('Nav2 navigate_to_pose action server unavailable: '
                               'not retrying')
            self.state = DockState.FAILED
            self.message = '%s: %s' % (DockMsg.NAV_TO_DOCK_FAILED, NAV2_UNAVAILABLE)
            return []
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

    def _go_to_approach(self, snap: Snapshot, force: bool = False) -> List[tuple]:
        """Nav2 to the approach pose unless already within approach_skip_radius_m (and not
        ``force``: a re-approach after a failed search must actually re-position)."""
        d = self._dist_to_approach(snap)
        if self.p.skip_nav_to_approach:
            self._arrive(snap)
            return []
        if not force and d is not None and self.use_vision and \
                d <= self.p.approach_skip_radius_m:
            self._notes.append('%.2f m from the approach pose: skipping Nav2' % d)
            self._arrive(snap)
            return []
        self._relaxed_tried = False
        self._lat_done = False
        self._enter(DockState.NAV_TO_APPROACH, snap)
        return [('start_nav', self.approach_goal)]

    # -- bumper hold -------------------------------------------------------
    def _bumper_hold(self, snap: Snapshot) -> Optional[Output]:
        """Stop while the bumper is pressed / the bumper manoeuvre runs; resume from the
        current pose once clear. Returns an Output while holding, None otherwise."""
        held = (DockState.ALIGNING, DockState.LATERAL_ALIGN, DockState.SEARCHING,
                DockState.DOCKING, DockState.FINAL_DOCKING, DockState.RETRY)
        if self._bump is not None and self.state not in held:
            self._bump = None        # left the held states (contact -> CHARGING): done
        if self._bump is None:
            if not snap.bumper or self.state not in held:
                return None
            self._bump = {'t0': snap.t, 'clear_t': None, 'state': self.state}
            self._notes.append('bumper in %s: stopped, waiting for it to clear (max %.0f s)'
                               % (self.state, self.p.bumper_wait_max_s))
            self._stall.reset()
            return self._out()
        b = self._bump
        if snap.bumper:
            b['clear_t'] = None
            if snap.t - b['t0'] > self.p.bumper_wait_max_s:
                self._bump = None
                self._fail('%s: bumper pressed for %.0f s in %s' % (
                    DockMsg.DOCK_BLOCKED, snap.t - b['t0'], b['state']), snap)
            return self._out()
        if b['clear_t'] is None:
            b['clear_t'] = snap.t
        if snap.t - b['clear_t'] < self.p.bumper_clear_s:
            return self._out()
        self._bump = None
        self._resume_after_bumper(snap, b['state'], snap.t - b['t0'])
        return self._out()

    # -- rear obstacle hold -------------------------------------------------
    def _rear_hold(self, snap: Snapshot) -> Optional[Output]:
        """Hold still while obstacle_guard reports a rear obstacle in the dock corridor.
        Returns an Output while holding, None otherwise (also on the resuming tick, which
        then runs the state normally with every state timer shifted by the hold)."""
        held = (DockState.LATERAL_ALIGN, DockState.SEARCHING, DockState.DOCKING,
                DockState.FINAL_DOCKING)
        if self._rear is None:
            if not self.p.rear_hold or not snap.rear_blocked or self.state not in held:
                return None
            self._rear = {'t0': snap.t, 'clear_t': None, 'state': self.state}
            self._notes.append('rear obstacle between robot and dock in %s: holding (max %.0f s)'
                               % (self.state, self.p.rear_wait_max_s))
            self._stall.reset()
            return self._out()
        r = self._rear
        if snap.rear_blocked:
            r['clear_t'] = None
            if snap.t - r['t0'] > self.p.rear_wait_max_s:
                self._rear = None
                self._fail('%s: rear obstacle in the dock corridor for %.0f s in %s' % (
                    DockMsg.DOCK_BLOCKED, snap.t - r['t0'], r['state']), snap)
            return self._out()
        if r['clear_t'] is None:
            r['clear_t'] = snap.t
        if snap.t - r['clear_t'] < self.p.rear_clear_s:
            return self._out()
        held_s = snap.t - r['t0']
        self._rear = None
        self._paused_s += held_s
        # the robot stood still: carry on as if the hold had not happened
        self._t_state += held_s
        self._final_phase_t += held_s
        self._lost_t += held_s
        self._lost = 0
        self._prev_t = None
        self._prev_e = None
        self._stall.reset()
        self._notes.append('rear corridor clear after %.1f s: resuming %s' % (held_s, r['state']))
        return None

    def _resume_after_bumper(self, snap: Snapshot, st: str, waited: float,
                             what: str = 'bumper') -> None:
        self._notes.append('%s clear after %.1f s: resuming %s from the current pose'
                           % (what, waited, st))
        self._obs = []
        self._lost = 0
        self._kf.reset()
        self._prev_w = None
        if st == DockState.LATERAL_ALIGN:
            # 2026-10-21 night: re-running the pre-alignment after each bumper hit looped 7
            # times (13 hits ~1.4 m out) and walked the lateral 0.28 -> 0.63 m: end it here
            self._lat_done = True
            self._notes.append('lateral pre-alignment ended by the %s: searching from here'
                               % what)
            self._arrive(snap)
            return
        if st in (DockState.ALIGNING, DockState.SEARCHING):
            # re-plan the about-turn from where the back-off left the robot
            self._arrive(snap)
            if self.state == DockState.ALIGNING:
                return
            if st == DockState.ALIGNING and self._align_target is not None and \
                    snap.map_pose is None and snap.odom is not None:
                err = wrap_angle(self._align_target - snap.odom.yaw)
                if abs(err) > math.radians(self.p.align_tolerance_deg):
                    self._enter(DockState.ALIGNING, snap)   # no map pose: keep the odom target
                    self._heading_left = err
            return
        if st == DockState.DOCKING:
            # the reverse continues only on a fresh in-gate marker; a stale pre-bump
            # observation must not trigger the dead-reckoned final stretch
            self._last_err = None
            self._enter(DockState.DOCKING, snap)
            return
        if st == DockState.FINAL_DOCKING:
            # dead-reckoned stretch invalidated by the back-off: re-approach
            self._enter_retry(snap, 0.0)
            return
        self._enter(st, snap)   # RETRY: forward drive again

    def _teleop_hold(self, snap: Snapshot) -> Optional[Output]:
        """Operator joystick: /cmd_vel_teleop beats /cmd_vel_docking in twist_mux, so the
        robot is not ours while it is active (2026-10-21 08:28:57: the joystick reversed the
        robot out of the Dock path polygon mid-goal). Pause (zero output, goal timeout
        paused), then resume from wherever the operator left it."""
        if not self.p.teleop_hold:
            return None
        held = (DockState.ALIGNING, DockState.LATERAL_ALIGN, DockState.SEARCHING,
                DockState.DOCKING, DockState.FINAL_DOCKING, DockState.RETRY)
        if self._teleop is not None and self.state not in held:
            self._teleop = None      # joystick dock: contact -> CHARGING
        if self._teleop is None:
            if not snap.teleop or self.state not in held:
                return None
            self._teleop = {'t0': snap.t, 'clear_t': None, 'state': self.state}
            self._notes.append('joystick (cmd_vel_teleop) active in %s: docking paused until '
                               'it is released for %.1f s' % (self.state, self.p.teleop_clear_s))
            self._stall.reset()
            return self._out()
        h = self._teleop
        if snap.teleop:
            h['clear_t'] = None
            if snap.t - h['t0'] > self.p.teleop_wait_max_s:
                self._teleop = None
                self._fail('%s: operator joystick active for %.0f s in %s' % (
                    DockMsg.CANCELED, snap.t - h['t0'], h['state']), snap)
            return self._out()
        if h['clear_t'] is None:
            h['clear_t'] = snap.t
        if snap.t - h['clear_t'] < self.p.teleop_clear_s:
            return self._out()
        self._teleop = None
        self._paused_s += snap.t - h['t0']
        self._resume_after_bumper(snap, h['state'], snap.t - h['t0'], what='joystick')
        return self._out()

    def _rear_held_s(self, snap: Snapshot) -> float:
        return snap.t - self._rear['t0'] if self._rear is not None else 0.0

    def _fresh_marker(self, snap: Snapshot) -> Optional[MarkerObs]:
        m = snap.marker
        if m is None or snap.t - m.stamp > self.p.marker_timeout_s:
            return None
        if m.stamp >= self._t_start:
            self._marker_seen_t = m.stamp if self._marker_seen_t is None \
                else max(self._marker_seen_t, m.stamp)
        return m

    def _rtk_check(self, snap: Snapshot, med: DockErrors) -> str:
        """Stock's intended (unused) cross-check: marker pose vs RTK pose (log only)."""
        if snap.map_pose is None or not self.dock_pose_measured:
            return ''
        r = rtk_dock_errors(snap.map_pose, self.dock_pose)
        return '; RTK says %.2f m, lateral %.2f m, heading %.1f deg (diff %.2f m / %.1f deg)' % (
            r.remaining, r.lateral, math.degrees(r.heading), r.lateral - med.lateral,
            math.degrees(wrap_angle(r.heading - med.heading)))

    def _step_lateral_align(self, snap: Snapshot, el: float, travelled: float) -> Output:
        p, g = self.p, self.p.gains
        if snap.map_pose is None or el > p.lateral_align_timeout_s or \
                (self._lat_phase == 'forward' and travelled > p.lateral_align_max_travel):
            self._notes.append('lateral pre-alignment stopped (%s, %.1f m, %.0f s): searching'
                               % ('no map pose' if snap.map_pose is None else self._lat_phase,
                                  travelled, el))
            self._lat_done = True
            self._post_approach(snap)
            return self._out()
        e = rtk_dock_errors(snap.map_pose, self.dock_pose)
        le = e.lateral - self._offset
        self._remaining = e.remaining
        dt = snap.t - self._prev_t if self._prev_t is not None else 0.05
        self._prev_t = snap.t
        cap = self._forward_cap()
        if self._lat_phase == 'forward' and cap is not None and e.remaining >= cap:
            self._notes.append('lateral pre-alignment stopped at the corridor end (%.2f m out, '
                               'cap %.2f m, %.2f m off the line): searching'
                               % (e.remaining, cap, le))
            self._lat_done = True
            self._post_approach(snap)
            return self._out()
        if self._lat_phase == 'forward':
            if abs(le) <= p.lateral_align_done and \
                    abs(e.heading) <= math.radians(p.lateral_align_heading_deg):
                self._lat_phase = 'reverse'
                self._prev_e = None
                self._notes.append('back on the approach line (%.2f m, %.1f deg) after %.2f m: '
                                   'reversing to %.2f m' % (le, math.degrees(e.heading),
                                                            travelled, p.approach_distance))
            else:
                v = p.lateral_align_speed
                half = 0.5 * g.track_width
                w_lim = g.max_angular if g.wheel_floor <= 0 else \
                    min(g.max_angular, max(0.0, v - g.wheel_floor) / half)
                h_d = clamp(-p.lateral_align_k * le, -p.lateral_align_max_angle,
                            p.lateral_align_max_angle)
                w = clamp(g.kp_heading * wrap_angle(h_d - e.heading), -w_lim, w_lim)
                return self._out(v, w)
        if e.remaining <= p.approach_distance:
            self._notes.append('lateral pre-alignment done: %.2f m off the approach line, '
                               '%.1f deg' % (le, math.degrees(e.heading)))
            self._lat_done = True
            self._post_approach(snap)
            return self._out()
        gl = replace(g, max_speed=min(g.max_speed, p.lateral_align_speed),
                     close_speed=min(g.close_speed, p.lateral_align_speed))
        v, w, self._prev_e = reverse_control(DockErrors(e.remaining, le, e.heading), gl,
                                             self._prev_e, dt)
        return self._out(v, w)

    def _step_docking(self, snap: Snapshot, el: float, travelled: float) -> Output:
        p, g = self.p, self.p.gains
        if el > p.docking_timeout_s:
            self._enter_retry(snap, travelled)
            return self._out()
        m = self._fresh_marker(snap)
        new = m is not None and m.stamp != self._last_marker_stamp
        err = self._measure(m) if m is not None else None
        if err is not None and new:
            self._push_obs(m, err, snap.odom)
        gerr = self._median_err(None, snap.odom) if err is not None else None
        k = max(1.0, p.docking_gate_scale)
        usable = gerr is not None and self._gate(gerr, k)
        use_kf = p.marker_filter
        if usable and new:
            self._lost = 0
            self._last_marker_stamp = m.stamp
            if use_kf:
                if self._kf.x is None:
                    self._kf.init(gerr)
                else:
                    c0 = self._kf.clamped
                    r0 = (self._kf.rejected_pos, self._kf.rejected_heading)
                    # gated filter: median position (of the last marker_median_n), RAW heading
                    # (a median mixes flips into the inliers and biases it; a raw flip is
                    # 22 deg out and simply fails the gate)
                    self._kf.correct(DockErrors(gerr.remaining, gerr.lateral, err.heading)
                                     if p.kf_gate_chi2 > 0 else err)
                    if self._kf.clamped != c0:
                        self._notes.append('marker jump clamped (measured lateral %.2f m, '
                                           'heading %.1f deg)' % (err.lateral,
                                                                  math.degrees(err.heading)))
                    r1 = (self._kf.rejected_pos, self._kf.rejected_heading)
                    if r1 != r0 and (r1[0] + r1[1]) % 5 == 1:
                        self._notes.append(
                            'marker rejected by the filter gate (%d position, %d heading so '
                            'far; measured lateral %.2f m, heading %.1f deg vs %.2f m, %.1f '
                            'deg)' % (r1[0], r1[1], err.lateral, math.degrees(err.heading),
                                      self._kf.x.lateral, math.degrees(self._kf.x.heading)))
        dt = snap.t - self._prev_t if self._prev_t is not None else 0.05
        self._prev_t = snap.t
        if not usable:
            last = self._kf.x if use_kf and self._kf.x is not None else self._last_err
            if use_kf and last is not None and snap.odom is not None:
                self._last_err, self._last_err_odom = last, snap.odom
            if last is not None and last.remaining <= g.final_zone:
                # lost in the last stretch (too close / occluded): straight on the
                # last docked heading, dead-reckoned
                self._notes.append('marker lost %.2f m from the contacts: reversing straight'
                                   ' on the last heading' % last.remaining)
                self._enter_vision_final(snap)
                return self._out(-g.final_speed, 0.0)
            self._lost += 1
            if self._lost == 1:
                self._lost_t = snap.t
                if use_kf and last is not None and p.lost_predict_s > 0:
                    self._notes.append('marker lost %.2f m out: reversing on the filter '
                                       'prediction for up to %.1f s' % (last.remaining,
                                                                       p.lost_predict_s))
            if use_kf and self._kf.x is not None and snap.odom is not None and \
                    snap.t - self._lost_t < p.lost_predict_s:
                est = self._kf.x
                self._remaining = est.remaining
                gl = replace(g, max_speed=min(g.max_speed, p.lost_speed),
                             close_speed=min(g.close_speed, p.lost_speed))
                v, w, self._prev_e = reverse_control(est, gl, self._prev_e, dt, self._prev_w)
                self._prev_w = w
                return self._out(v, w)
            if self._lost > p.max_lost_frames and \
                    snap.t - self._lost_t >= p.marker_lost_hold_s:
                self._enter_retry(snap, travelled)
            self._prev_e = None
            self._prev_w = None
            return self._out()  # hold still while the marker is lost
        if use_kf and self._kf.x is None:
            self._kf.init(gerr)
        est = self._kf.x if use_kf else err
        dec = self._kf.x if use_kf else gerr
        if p.pose_check_median_n > 1 and use_kf and len(self._obs) >= 3:
            # pose check: the median of the last N detections (bias-corrected camera), so a
            # single flip or a mid-swing filter value cannot spend a realign
            pm = self._median_err(p.pose_check_median_n, snap.odom)
            dec = DockErrors(est.remaining, pm.lateral, pm.heading)
        self._last_err, self._last_err_odom = est, snap.odom
        self._remaining = est.remaining
        if p.pose_check_distance > 0:
            check = not self._pose_checked and \
                dec.remaining + p.docked_marker_offset <= p.pose_check_distance
        else:
            check = dec.remaining <= g.final_zone
        if check:
            self._pose_checked = True
            off = abs(dec.lateral) > p.final_max_lateral or \
                abs(dec.heading) > math.radians(p.final_max_heading_deg)
            if off and self._realigns < p.max_realigns:
                self._enter_realign(snap, dec)
                return self._out()
            if off:
                self._notes.append('pose check at %.2f m: lateral %.2f m, heading %.1f deg off, '
                                   'no realign left: continuing' % (
                                       dec.remaining + p.docked_marker_offset, dec.lateral,
                                       math.degrees(dec.heading)))
        overrun = max(0.0, p.final_max_distance - g.final_zone)
        if dec.remaining <= -overrun:
            self._notes.append('passed the expected contact point by %.2f m with the marker '
                               'in view: holding for the contacts' % -dec.remaining)
            self._enter_vision_final(snap, phase='settle')
            return self._out()
        v, w, self._prev_e = reverse_control(est, g, self._prev_e, dt, self._prev_w)
        self._prev_w = w
        return self._out(v, w)

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
        if st == DockState.LATERAL_ALIGN:
            return 'lateral pre-align (%s, RTK) onto the approach line%s; marker %s' % (
                self._lat_phase, ' %+.2f m' % self._offset if self._offset else '', mk)
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
        self._side = 1.0 if err.lateral >= 0.0 else -1.0
        kg = max(1.0, self.p.docking_gate_scale)
        if self._gate(err, kg):
            self._valid_marker_t = m.stamp
        txt = 'seen %.2f m, %.1f\N{DEGREE SIGN}' % (math.hypot(m.x, m.y),
                                                    math.degrees(err.heading))
        if not self._gate(err, force=True):
            txt += ' (outside gate: lateral %.2f m)' % err.lateral
        self._marker_text = txt

    def step(self, snap: Snapshot) -> Output:
        if self.state in (DockState.SUCCEEDED, DockState.FAILED):
            return self._out()
        self._now = snap.t
        travelled = self._tracker.update(snap.t, snap.odom, self._last_cmd)
        self._kf.predict(snap.odom, snap.t)
        self._update_marker_text(snap)

        # ---- global guards ------------------------------------------------
        if snap.stop_triggered or snap.lift_triggered:
            self._fail(DockMsg.EMERGENCY_STOP, snap)
            return self._out()
        if not snap.status_fresh and self.state in (DockState.DOCKING, DockState.FINAL_DOCKING,
                                                    DockState.RETRY, DockState.LATERAL_ALIGN):
            self._fail(DockMsg.BASE_STATUS_STALE, snap)
            return self._out()
        if snap.t - self._t_start - self._paused_s - self._rear_held_s(snap) > \
                self.goal_timeout_s and self.state != DockState.CHARGING:
            self._fail(DockMsg.DOCK_TIMEOUT, snap)
            return self._out()
        moving = (DockState.SEARCHING, DockState.DOCKING, DockState.FINAL_DOCKING,
                  DockState.ALIGNING, DockState.RETRY, DockState.NAV_TO_APPROACH,
                  DockState.LATERAL_ALIGN)
        if (snap.contact or snap.is_charging) and self.state in moving:
            if self.state == DockState.NAV_TO_APPROACH:
                self._pending.append(('cancel_nav',))
            self._notes.append('contact in %s (debounced=%s charging=%s): stopping, confirming'
                               % (self.state, snap.contact, snap.is_charging))
            self._calibrate(snap)
            self._stall_hold = None
            self.state = DockState.CHARGING
            self._t_state = snap.t
            self._remaining = 0.0
            return self._out(requests=[('enable_charging',)])
        if self._stall_hold is not None and self.state in moving:
            t_hold, msg = self._stall_hold
            if snap.raw_contact:
                # stalled against the dock: the raw contact is enough here (the robot
                # cannot push further anyway); confirm through the charging path
                self._stall_hold = None
                self._notes.append('contact %.1f s after the stall (raw is_docking_done): '
                                   'docked' % (snap.t - t_hold))
                self._calibrate(snap)
                self.state = DockState.CHARGING
                self._t_state = snap.t
                self._remaining = 0.0
                return self._out(requests=[('enable_charging',)])
            if snap.t - t_hold >= self.p.stall_contact_grace_s:
                self._stall_hold = None
                self._fail(msg, snap)
            return self._out()   # hold still while waiting for the contacts
        if snap.raw_contact and self.state in moving and self.state != DockState.NAV_TO_APPROACH:
            return self._out()   # raw contact: hold still until the debounce confirms it
        held = self._bumper_hold(snap)
        if held is not None:
            return held
        held = self._rear_hold(snap)
        if held is not None:
            return held
        held = self._teleop_hold(snap)
        if held is not None:
            return held

        if self.state in (DockState.DOCKING, DockState.FINAL_DOCKING, DockState.RETRY):
            dig = self.p.use_dig_stall and snap.dig_stall and not self._dig_at_start
            if dig or self._stall.update(snap.t, snap.progress_pose, self._last_cmd):
                why = 'slip detector /dig_stall' if dig else \
                    'moved %.2f m while commanding %.2f m/s' % (self._stall.measured,
                                                                 abs(self._last_cmd))
                msg = '%s: wheels stalled in %s (%s)' % (DockMsg.DOCK_STALLED, self.state, why)
                self._stall.reset()
                if self.state != DockState.RETRY and self.p.stall_contact_grace_s > 0:
                    self._stall_hold = (snap.t, msg)
                    self._notes.append('%s: holding %.1f s for the contacts' % (
                        msg, self.p.stall_contact_grace_s))
                    return self._out()
                self._fail(msg, snap)
                return self._out()

        st = self.state
        el = snap.t - self._t_state
        g = self.p.gains

        if st == DockState.NAV_TO_APPROACH:
            if snap.nav_status == 'succeeded':
                self._arrive(snap)
            elif snap.nav_status == 'failed':
                return self._out(requests=self._nav_failed(snap, False))
            elif snap.nav_status == 'unavailable':
                return self._out(requests=self._nav_failed(snap, False, unavailable=True))
            elif el > self.p.nav_timeout_s:
                return self._out(requests=self._nav_failed(snap, True))
            return self._out()

        if st == DockState.ALIGNING:
            m = self._fresh_marker(snap)
            if m is not None and self._gate(dock_errors(m, self.p.docked_marker_offset),
                                            force=True):
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
            # wrap_angle(err) in (-pi, pi]: the sign is the shorter way round
            return self._out(self.p.align_creep, math.copysign(self.p.align_angular, err))

        if st == DockState.LATERAL_ALIGN:
            return self._step_lateral_align(snap, el, travelled)

        if st == DockState.SEARCHING:
            m = self._fresh_marker(snap)
            if m is not None:
                err = dock_errors(m, self.p.docked_marker_offset)
                fresh = not self._obs or self._obs[-1][0] != m.stamp
                if fresh:
                    self._search_n += 1
                self._push_obs(m, err, snap.odom)
                self._remaining = err.remaining
                need = max(1, int(self.p.search_min_detections))
                n_med = max(need, int(self.p.marker_median_n))
                med = self._median_err(n_med, snap.odom)
                ok = fresh and self._search_n >= need and self._gate(med)
                if ok and need > 1:
                    # the last `need` detections must agree with each other (a lone IPPE
                    # flip, or a flip pair against a good one, is not a dock pose)
                    rec = self._recent_obs(need, snap.odom)
                    mr = median_errors(rec)
                    ok = all(abs(wrap_angle(e.heading - mr.heading)) <=
                             math.radians(self.p.search_consistency_deg) and
                             abs(e.lateral - mr.lateral) <= self.p.search_consistency_m
                             for e in rec)
                if ok:
                    why = self._rtk_disagrees(snap, med)
                    if why:
                        self._notes.append('detections rejected: %s' % why)
                        self._obs = []
                        self._search_n = 0
                        ok = False
                if ok:
                    self._lost = 0
                    self._last_err, self._last_err_odom = med, snap.odom
                    self._notes.append(
                        'marker accepted after %d detection(s) in %.1f s: %.2f m to go, '
                        'lateral %.2f m, heading %.1f deg%s' % (
                            self._search_n, el, med.remaining, med.lateral,
                            math.degrees(med.heading), self._rtk_check(snap, med)))
                    self._kf.reset()
                    self._kf.init(med)
                    self._pose_checked = False
                    self._enter(DockState.DOCKING, snap)
                    return self._out()
            if el > self.p.search_timeout_s:
                if self.p.allow_blind_docking:
                    self._try_blind(snap)     # explicit vendor mode only
                else:
                    # Owner rule: docking is driven by the rear marker; never reverse blind.
                    # Re-approach (Nav2 to the approach pose, about-turn, search again).
                    self._notes.append('no usable marker after %.0f s of searching (%d '
                                       'detection(s)): re-approaching (no blind reverse)'
                                       % (el, self._search_n))
                    self._enter_retry(snap, travelled, renav=True)
                    if self.state == DockState.FAILED:
                        self.message = '%s: rear marker not found after %d re-approaches' % (
                            DockMsg.DOCK_NOT_FOUND, self.retries)
            return self._out()

        if st == DockState.DOCKING:
            return self._step_docking(snap, el, travelled)

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
            cap = self._forward_cap()
            rem = self._rtk_remaining(snap)
            at_cap = cap is not None and rem is not None and rem >= cap
            if at_cap and travelled < self._retry_distance:
                self._notes.append('forward drive stopped at the corridor end (%.2f m out, cap '
                                   '%.2f m) after %.2f m' % (rem, cap, travelled))
            if travelled >= self._retry_distance or at_cap:
                if self._realign:
                    self._realign = False
                    self._lat_done = True    # no LATERAL_ALIGN after a realign (left the corridor)
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
