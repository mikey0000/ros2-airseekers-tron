# SPDX-License-Identifier: GPL-3.0-or-later
"""Localization quality / drift-budget estimate behind /localization/status (pure, no ROS).

2026-10-09 (localization item 6). The mower must keep mowing through short RTK float /
dropouts on wheel + IMU (+ stereo VIO) dead reckoning, and stop cutting before the
accumulated error can put the blade outside the area. This module turns the inputs into
one verdict the mission layer acts on:

``ok``        estimated position error <= ``drift_budget_m``
``degraded``  over budget but <= ``lost_drift_m``: mission holds (blade off, stop) and
              resumes on its own once RTK FIXED is back (the estimate resets)
``lost``      over ``lost_drift_m``, never had RTK FIXED, or no map-frame pose at all

Drift model (deliberately explicit, not the EKF covariance): robot_localization's pose
covariance grows with the hand-set process noise, not with the real dead-reckoning error,
so it is reported (``ekf_xy_sigma_m``) but not gated on. Instead::

    est = fixed_sigma_m + sum(distance travelled since the last confirmed RTK FIXED
                              x drift_rate_{vio|wheel} at the time)
    while RTK FLOAT is fresh (gps_gate fuses it with a >= 0.4 m sigma):
        est = min(est, float_bound_m)

Distance comes from the continuous odom-frame pose (/odometry/filtered of ekf_odom), so a
pivot in place costs nothing and a slipping wheel that VIO corrects costs what VIO says.
``drift_rate_wheel`` 3 % / ``drift_rate_vio`` 1.5 % of distance are starting values (wheel
+ JY61P/stereo-gyro yaw on grass; heading error dominates) to be replaced by measured
values from a float/dropout drive (docs/localization.md). A FIXED epoch only resets the
estimate after it has held for ``fixed_confirm_s`` (a lone FIXED inside a float stretch is
often a wrong integer fix).
"""

import math
from dataclasses import dataclass

from mower_localization import rtk_quality as rq

STATE_OK = 'ok'
STATE_DEGRADED = 'degraded'
STATE_LOST = 'lost'


@dataclass
class StatusParams:
    fix_status_timeout_s: float = 2.0   # /fix_status older -> rtk 'stale'
    fixed_confirm_s: float = 1.0        # FIXED held this long before the estimate resets
    fixed_sigma_m: float = 0.03         # error right after a confirmed FIXED
    drift_rate_wheel: float = 0.03      # m per m travelled, wheel + IMU dead reckoning
    drift_rate_vio: float = 0.015       # m per m while gated VIO is being fused
    float_bound_m: float = 0.5          # fresh RTK FLOAT (fused, 0.4 m floor) caps the estimate
    drift_budget_m: float = 0.3         # est above this -> degraded (mission holds)
    lost_drift_m: float = 1.0           # est above this -> lost
    map_odom_timeout_s: float = 1.0      # /odometry/filtered_map older -> lost
    vio_timeout_s: float = 1.0          # /odometry/vio_gated older -> VIO not active
    max_step_m: float = 1.0             # odom pose step larger than this = jump, not travel


class LocalizationStatus:
    """Feed timestamped inputs (one monotonic clock, seconds); read :meth:`status`."""

    def __init__(self, params=None):
        self.p = params or StatusParams()
        self.rtk = None              # last rtk_quality class from /fix_status
        self.rtk_t = None
        self.fixed_since = None      # start of the current FIXED run
        self.last_fixed_t = None     # last time the estimate was reset by a confirmed FIXED
        self.dist_since_fixed = 0.0
        self.drift_dr = None         # dead-reckoning estimate; None = never had RTK FIXED
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
        if cls == rq.RTK_FIXED:
            if self.fixed_since is None:
                self.fixed_since = now
            if now - self.fixed_since >= self.p.fixed_confirm_s:
                self.last_fixed_t = now
                self.dist_since_fixed = 0.0
                self.drift_dr = self.p.fixed_sigma_m
        else:
            self.fixed_since = None

    def on_odom_pose(self, now, x, y):
        """Continuous odom-frame position (ekf_odom /odometry/filtered)."""
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        if self._last_xy is not None:
            step = math.hypot(x - self._last_xy[0], y - self._last_xy[1])
            if step <= self.p.max_step_m:
                self.dist_since_fixed += step
                if self.drift_dr is not None and not self._fixed_now(now):
                    self.drift_dr += step * (self.p.drift_rate_vio if self.vio_active(now)
                                             else self.p.drift_rate_wheel)
        self._last_xy = (x, y)

    def on_vio(self, now):
        self.vio_t = now

    def on_map_odom(self, now, cov_xx, cov_yy):
        self.map_odom_t = now
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

    def vio_active(self, now):
        return self.vio_t is not None and now - self.vio_t <= self.p.vio_timeout_s

    def estimate(self, now):
        """Estimated horizontal position error (m), or None before the first FIXED."""
        if self.drift_dr is None:
            return None
        est = self.drift_dr
        if self.rtk_class(now) == rq.RTK_FLOAT:
            est = min(est, self.p.float_bound_m)
        return est

    def verdict(self, now):
        """(state, reason)."""
        p = self.p
        if self.map_odom_t is None or now - self.map_odom_t > p.map_odom_timeout_s:
            return STATE_LOST, 'no map-frame pose (/odometry/filtered_map)'
        est = self.estimate(now)
        rtk = self.rtk_class(now)
        if est is None:
            return STATE_LOST, 'no RTK FIXED since start (rtk %s)' % rtk
        if est > p.lost_drift_m:
            return STATE_LOST, 'estimated drift %.2f m > %.2f m (rtk %s)' % (
                est, p.lost_drift_m, rtk)
        if est > p.drift_budget_m:
            return STATE_DEGRADED, 'estimated drift %.2f m > budget %.2f m (rtk %s)' % (
                est, p.drift_budget_m, rtk)
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
            'since_fixed_s': r(None if self.last_fixed_t is None else now - self.last_fixed_t, 1),
            'dist_since_fixed_m': r(self.dist_since_fixed),
            'est_drift_m': r(est, 3),
            'drift_budget_m': p_round(self.p.drift_budget_m),
            'lost_drift_m': p_round(self.p.lost_drift_m),
            'ekf_xy_sigma_m': r(self.ekf_sigma, 3),
            'map_odom_age_s': r(None if self.map_odom_t is None else now - self.map_odom_t),
            'vio_active': self.vio_active(now),
            'heading_aligned': bool(h.get('aligned', False)),
            'heading_source': h.get('source'),
            'heading_sigma_deg': h.get('yaw_sigma_deg'),
        }


def p_round(v):
    return round(float(v), 3)
