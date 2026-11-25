# SPDX-License-Identifier: GPL-3.0-or-later
"""Localization quality / drift-budget estimate behind /localization/status (pure, no ROS).

2026-10-09 (localization item 6). The mower must keep mowing through short RTK float /
dropouts on wheel + IMU (+ stereo VIO) dead reckoning, and stop cutting before the
accumulated error can put the blade outside the area. This module turns the inputs into
one verdict the mission layer acts on:

``ok``        estimated position error <= ``drift_budget_m``
``degraded``  over budget but <= ``lost_drift_m``: mission holds (blade off, stop) and
              resumes on its own once RTK FIXED is back (the estimate resets)
``lost``      over ``lost_drift_m``, never had RTK FIXED, no map-frame pose at all, or no
              RTK (FIXED or FLOAT) for more than ``lost_after_s`` whatever the drift
              estimate says: the distance model stops growing once the mower stops, so
              a held mower would otherwise stay ``degraded`` through any outage

Drift model (deliberately explicit, not the EKF covariance): robot_localization's pose
covariance grows with the hand-set process noise, not with the real dead-reckoning error,
so it does not drive the drift estimate. It is a second gate on ``ok`` only (RTK-loss
policy 2026-11-03): ``ok`` also needs the fused ekf_map xy sigma (``ekf_xy_sigma_m``,
from /odometry/filtered_map) <= ``loc_sigma_ok_m``; above it, or with no finite
covariance, the verdict is ``degraded``. A receiver FIXED flag alone is not enough when
the filter has not (yet) pulled its estimate in.

The drift estimate (2026-11-25, :mod:`mower_localization.drift_model`, fitted in
docs/analysis/2026-11-13_dead_reckoning_drift.md) is a p90 bound since the last confirmed
RTK FIXED: 5 cm + 2 %/m on flat ground, rising to 10 cm + 10 %/m at 11 deg of tilt
(/tilt/status), + 1 cm per pivot in place, the per-metre rate halved while gated VIO is fused.
While RTK FLOAT is fresh (gps_gate fuses it with a >= 0.4 m sigma) the estimate is capped at
``float_bound_m``. Distance comes from the continuous odom-frame pose (/odometry/filtered of
ekf_odom), so a pivot costs only its pivot term. A FIXED epoch only resets the estimate after
it has held for ``fixed_confirm_s`` (a lone FIXED inside a float stretch is often a wrong
integer fix).

Coast mode (2026-11-25, docs/analysis/2026-11-25_coast_mode.md): ``coast_left_m`` is what the
mission may still drive on dead reckoning, the drift_model coast budget (5 m flat, 1.5 m from
8 deg) less the distance since the fix, 0 after ``coast_max_s`` or once dead reckoning is lost.
While it is above 0 the ``lost_after_s`` rule waits. Dead reckoning is lost (state ``lost``,
until the next confirmed FIXED) on

* wheel slip: the wheel odometry yaw (/mcu/measured_speed) and the gyro-led odom yaw part by
  ``slip_yaw_rad`` within ``slip_window_s``;
* an RTK position that disagrees: ``float_gate_n`` /odometry/gps samples in a row, under
  RTK FLOAT or a FIXED not yet confirmed, further than ``float_gate_m`` + the drift estimate
  from the dead-reckoned pose (the map pose at the last fix moved by the odom-frame motion
  since). Field 2026-11-13: the float that ended a single-fix dropout was 0.3 m off.

``coast_max_s: 0`` with base_flat = base_slope = 0.03, rate_flat = rate_slope = 0.03 and
pivot 0 restores the monitor before 2026-11-25.
"""

import math
from dataclasses import dataclass

from mower_localization import drift_model as dm
from mower_localization import rtk_quality as rq

STATE_OK = 'ok'
STATE_DEGRADED = 'degraded'
STATE_LOST = 'lost'


@dataclass
class StatusParams:
    fix_status_timeout_s: float = 2.0   # /fix_status older -> rtk 'stale'
    fixed_confirm_s: float = 1.0        # FIXED held this long before the estimate resets
    # drift model (drift_model.DriftParams; p90 since the last fix, 2026-11-13 drift study)
    drift_base_flat_m: float = 0.05
    drift_base_slope_m: float = 0.10
    drift_rate_flat: float = 0.02       # m per m driven, flat
    drift_rate_slope: float = 0.10      # ... at drift_tilt_slope_deg
    drift_tilt_flat_deg: float = 4.0
    drift_tilt_slope_deg: float = 11.0
    drift_pivot_m: float = 0.01         # per pivot in place (>= pivot_min_deg, |vx| < pivot_max_vx)
    drift_rate_scale_vio: float = 0.5   # per-metre rate x this while gated VIO is fused
    pivot_min_deg: float = 45.0
    pivot_max_vx: float = 0.05
    tilt_timeout_s: float = 2.0         # /tilt/status (2 Hz) older -> flat
    float_bound_m: float = 0.5          # fresh RTK FLOAT (fused, 0.4 m floor) caps the estimate
    drift_budget_m: float = 0.3         # est above this -> degraded (mission holds)
    lost_drift_m: float = 1.0           # est above this -> lost
    map_odom_timeout_s: float = 1.0      # /odometry/filtered_map older -> lost
    vio_timeout_s: float = 1.0          # /odometry/vio_gated older -> VIO not active
    max_step_m: float = 1.0             # odom pose step larger than this = jump, not travel
    loc_sigma_ok_m: float = 0.15        # ok also needs ekf_map xy sigma <= this (<= 0 = off)
    lost_after_s: float = 10.0          # no RTK FIXED/FLOAT this long -> lost (<= 0 = off); stock and OpenMower use 10 s
    # coast budget (drift_model.CoastParams); coast_max_s <= 0: no coast budget
    coast_max_flat_m: float = 5.0
    coast_max_slope_m: float = 1.5
    coast_tilt_slope_deg: float = 8.0
    coast_max_s: float = 60.0
    slip_window_s: float = 2.0          # wheel vs gyro yaw window
    slip_yaw_rad: float = 0.6           # ... parting this far = slip -> lost (<= 0 = off)
    wheel_timeout_s: float = 0.5        # /mcu/measured_speed older -> no slip check
    float_gate_m: float = 0.15          # RTK vs dead reckoning: reject beyond this + est (<= 0 = off)
    float_gate_n: int = 2               # ... for this many /odometry/gps samples in a row

    def drift(self):
        return dm.DriftParams(self.drift_base_flat_m, self.drift_base_slope_m,
                              self.drift_rate_flat, self.drift_rate_slope,
                              self.drift_tilt_flat_deg, self.drift_tilt_slope_deg,
                              self.drift_pivot_m, {'vio': self.drift_rate_scale_vio})

    def coast(self):
        return dm.CoastParams(self.coast_max_flat_m, self.coast_max_slope_m,
                              self.coast_tilt_slope_deg, self.coast_max_s)


class LocalizationStatus:
    """Feed timestamped inputs (one monotonic clock, seconds); read :meth:`status`."""

    def __init__(self, params=None):
        self.p = params or StatusParams()
        self.dp, self.cp = self.p.drift(), self.p.coast()
        self.rtk = None              # last rtk_quality class from /fix_status
        self.rtk_t = None
        self.rtk_good_t = None       # last /fix_status with an RTK class (FIXED or FLOAT)
        self.fixed_since = None      # start of the current FIXED run
        self.last_fixed_t = None     # last time the estimate was reset by a confirmed FIXED
        self.dist_since_fixed = 0.0
        self.rate_acc = None         # sum of rate x step since the fix; None = never had RTK FIXED
        self.tilt_sum, self.tilt_n = 0.0, 0
        self.tilt, self.tilt_t = None, None
        self.pivots = dm.PivotCounter(self.p.pivot_min_deg, self.p.pivot_max_vx)
        self.slip = dm.SlipDetector(self.p.slip_window_s, self.p.slip_yaw_rad)
        self.slipped = False         # latched until the next confirmed FIXED
        self.gyro_wz = None
        self.wheel_t = None
        self.anchor = None           # (map pose, odom pose) at the last confirmed FIXED
        self.map_pose = None
        self.odom_pose = None
        self.rtk_dr = None           # (distance, gate) of the last RTK position vs dead reckoning
        self.disagree_n = 0
        self.rejected = None         # (distance, gate) latched until the next confirmed FIXED
        self.vio_t = None
        self.map_odom_t = None
        self.ekf_sigma = None
        self.heading = {}
        self._last_xy = None
        self._prev_state = None
        self.state_since = None

    # ------------------------------------------------------------------ inputs
    def on_fix_status(self, now, text):
        cls = rq.classify_fix_status(text)
        if cls is None:
            return
        self.rtk, self.rtk_t = cls, now
        if cls in (rq.RTK_FIXED, rq.RTK_FLOAT):
            self.rtk_good_t = now
        if cls == rq.RTK_FIXED:
            if self.fixed_since is None:
                self.fixed_since = now
            if now - self.fixed_since >= self.p.fixed_confirm_s:
                self._reset(now)
        else:
            self.fixed_since = None

    def _reset(self, now):
        """A confirmed FIXED epoch: dead reckoning starts again from here."""
        self.last_fixed_t = now
        self.dist_since_fixed = 0.0
        self.rate_acc = 0.0
        self.tilt_sum, self.tilt_n = 0.0, 0
        self.pivots.reset()
        self.slip.reset()
        self.slipped = False
        self.rejected, self.rtk_dr, self.disagree_n = None, None, 0
        if self.map_pose is not None and self.odom_pose is not None:
            self.anchor = (self.map_pose, self.odom_pose)

    def on_odom_pose(self, now, x, y, yaw=0.0, vx=0.0, gyro_wz=None):
        """Continuous odom-frame pose and twist (ekf_odom /odometry/filtered)."""
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        self.odom_pose = (x, y, yaw)
        self.gyro_wz = gyro_wz
        if self._last_xy is not None:
            step = math.hypot(x - self._last_xy[0], y - self._last_xy[1])
            if step <= self.p.max_step_m:
                self.dist_since_fixed += step
                if self.rate_acc is not None and not self._fixed_now(now):
                    aid = 'vio' if self.vio_active(now) else None
                    self.rate_acc += step * dm.drift_rate(self.tilt_now(now), self.dp, aid)
        self.pivots.update(yaw, vx)
        self._last_xy = (x, y)

    def on_tilt(self, now, roll_deg, pitch_deg):
        if roll_deg is None or pitch_deg is None:
            return
        self.tilt, self.tilt_t = math.hypot(roll_deg, pitch_deg), now
        self.tilt_sum += self.tilt
        self.tilt_n += 1

    def on_wheel_twist(self, now, wheel_wz):
        """Wheel odometry yaw rate (/mcu/measured_speed) against the gyro-led odom yaw rate."""
        self.wheel_t = now
        if self.gyro_wz is None or self.rate_acc is None or self._fixed_now(now):
            return
        self.slip.update(now, wheel_wz, self.gyro_wz)
        if self.slip.slipped():
            self.slipped = True

    def on_gps_odom(self, now, x, y):
        """An RTK position in the map frame (/odometry/gps): checked against dead reckoning
        while RTK is FLOAT or a FIXED not yet confirmed."""
        dr = self.dead_reckoned_pose()
        if dr is None or self._fixed_now(now) or \
                self.rtk_class(now) not in (rq.RTK_FIXED, rq.RTK_FLOAT):
            return
        d = math.hypot(x - dr[0], y - dr[1])
        est = self.dead_reckoning_estimate()
        self.rtk_dr = (d, self.p.float_gate_m + est)
        self.disagree_n = self.disagree_n + 1 if dm.float_disagrees(
            d, est, self.p.float_gate_m) else 0
        if self.rejected is None and self.disagree_n >= max(1, self.p.float_gate_n):
            self.rejected = self.rtk_dr

    def on_vio(self, now):
        self.vio_t = now

    def on_map_odom(self, now, cov_xx, cov_yy, pose=None):
        self.map_odom_t = now
        if pose is not None:
            self.map_pose = pose
        v = max(cov_xx, cov_yy)
        self.ekf_sigma = math.sqrt(v) if math.isfinite(v) and v >= 0.0 else None

    def on_heading_status(self, status):
        self.heading = status if isinstance(status, dict) else {}

    # ------------------------------------------------------------------ queries
    def rtk_class(self, now):
        if self.rtk is None or self.rtk_t is None or now - self.rtk_t > self.p.fix_status_timeout_s:
            return rq.RTK_STALE
        return self.rtk

    def _fixed_now(self, now):
        return (self.rtk_class(now) == rq.RTK_FIXED and self.fixed_since is not None
                and now - self.fixed_since >= self.p.fixed_confirm_s)

    def no_rtk_s(self, now):
        """Seconds without a fresh RTK FIXED/FLOAT class, None while one is current."""
        if self.rtk_class(now) in (rq.RTK_FIXED, rq.RTK_FLOAT) or self.rtk_good_t is None:
            return None
        return now - self.rtk_good_t

    def vio_active(self, now):
        return self.vio_t is not None and now - self.vio_t <= self.p.vio_timeout_s

    def tilt_now(self, now):
        """Fresh |tilt| (deg), None without one (flat)."""
        if self.tilt_t is None or now - self.tilt_t > self.p.tilt_timeout_s:
            return None
        return self.tilt

    def tilt_mean(self):
        """Mean |tilt| since the last fix (deg), None without samples."""
        return self.tilt_sum / self.tilt_n if self.tilt_n else None

    def dead_reckoning_estimate(self):
        """The drift model's estimate since the last fix (no float cap), None before the first."""
        if self.rate_acc is None:
            return None
        return dm.drift_base_m(self.tilt_mean(), self.dp) + self.rate_acc + \
            self.dp.pivot_m * self.pivots.count

    def estimate(self, now):
        """Estimated horizontal position error (m), or None before the first FIXED."""
        est = self.dead_reckoning_estimate()
        if est is not None and self.rtk_class(now) == rq.RTK_FLOAT:
            est = min(est, self.p.float_bound_m)
        return est

    def dead_reckoned_pose(self):
        """The map pose at the last fix moved by the odom-frame motion since, or None."""
        if self.anchor is None or self.odom_pose is None:
            return None
        map0, odom0 = self.anchor
        return dm.compose(map0, dm.relative(odom0, self.odom_pose))

    def dead_reckoning_lost(self):
        """Why dead reckoning cannot be trusted any more (until the next fix), else ''."""
        if self.slipped:
            return 'wheel slip (wheel and gyro yaw parted by %.1f rad within %.0f s)' % (
                self.p.slip_yaw_rad, self.p.slip_window_s)
        if self.rejected is not None:
            return 'RTK position %.2f m from dead reckoning (gate %.2f m)' % self.rejected
        return ''

    def coast_budget(self):
        return dm.coast_budget_m(self.tilt_mean(), self.dp, self.cp)

    def coast_left_m(self, now):
        """Distance a coast may still drive on dead reckoning (0: none)."""
        if self.cp.max_s <= 0.0 or self.rate_acc is None or self.dead_reckoning_lost() or \
                now - self.last_fixed_t > self.cp.max_s:
            return 0.0
        return max(0.0, self.coast_budget() - self.dist_since_fixed)

    def verdict(self, now):
        """(state, reason)."""
        p = self.p
        if self.map_odom_t is None or now - self.map_odom_t > p.map_odom_timeout_s:
            return STATE_LOST, 'no map-frame pose (/odometry/filtered_map)'
        est = self.estimate(now)
        rtk = self.rtk_class(now)
        if est is None:
            return STATE_LOST, 'no RTK FIXED since start (rtk %s)' % rtk
        why = '' if self._fixed_now(now) else self.dead_reckoning_lost()
        if why:
            return STATE_LOST, '%s (rtk %s)' % (why, rtk)
        no_rtk = self.no_rtk_s(now)
        if no_rtk is not None and p.lost_after_s > 0.0 and no_rtk > p.lost_after_s and \
                self.coast_left_m(now) <= 0.0:
            return STATE_LOST, 'no RTK for %.0f s (rtk %s, drift %.2f m)' % (no_rtk, rtk, est)
        if est > p.lost_drift_m:
            return STATE_LOST, 'estimated drift %.2f m > %.2f m (rtk %s)' % (
                est, p.lost_drift_m, rtk)
        if est > p.drift_budget_m:
            return STATE_DEGRADED, 'estimated drift %.2f m > budget %.2f m (rtk %s)' % (
                est, p.drift_budget_m, rtk)
        sig = p.loc_sigma_ok_m
        if sig > 0.0:
            if self.ekf_sigma is None:
                return STATE_DEGRADED, 'ekf_map covariance unavailable (rtk %s)' % rtk
            if self.ekf_sigma > sig:
                return STATE_DEGRADED, 'ekf_map sigma %.2f m > %.2f m (rtk %s)' % (
                    self.ekf_sigma, sig, rtk)
        if rtk == rq.RTK_FIXED:
            return STATE_OK, ''
        return STATE_OK, 'rtk %s, drift %.2f m within budget' % (rtk, est)

    def status(self, now):
        state, reason = self.verdict(now)
        if state != self._prev_state:
            self._prev_state, self.state_since = state, now
        est = self.estimate(now)
        h = self.heading
        r = lambda v, n=2: None if v is None else round(v, n)  # noqa: E731
        return {
            'state': state,
            'reason': reason,
            'state_age_s': r(now - self.state_since, 1),
            'rtk': self.rtk_class(now),
            'rtk_age_s': r(None if self.rtk_t is None else now - self.rtk_t, 1),
            'no_rtk_s': r(self.no_rtk_s(now), 1),
            'since_fixed_s': r(None if self.last_fixed_t is None else now - self.last_fixed_t, 1),
            'dist_since_fixed_m': r(self.dist_since_fixed),
            'est_drift_m': r(est, 3),
            'drift_budget_m': p_round(self.p.drift_budget_m),
            'lost_drift_m': p_round(self.p.lost_drift_m),
            'ekf_xy_sigma_m': r(self.ekf_sigma, 3),
            'sigma_ok_m': p_round(self.p.loc_sigma_ok_m),
            'map_odom_age_s': r(None if self.map_odom_t is None else now - self.map_odom_t),
            'vio_active': self.vio_active(now),
            'tilt_mean_deg': r(self.tilt_mean(), 1),
            'pivots': self.pivots.count,
            'coast_budget_m': r(self.coast_budget()),
            'coast_left_m': r(self.coast_left_m(now)),
            'rtk_dr_m': r(None if self.rtk_dr is None else self.rtk_dr[0], 3),
            'rtk_dr_gate_m': r(None if self.rtk_dr is None else self.rtk_dr[1], 3),
            'dr_lost': self.dead_reckoning_lost() or None,
            'heading_aligned': bool(h.get('aligned', False)),
            'heading_source': h.get('source'),
            'heading_sigma_deg': h.get('yaw_sigma_deg'),
        }


def p_round(v):
    return round(float(v), 3)
