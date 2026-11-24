# SPDX-License-Identifier: GPL-3.0-or-later
"""Dead-reckoning drift model and coast budget (pure, no ROS; 2026-11-25 coast mode).

Fitted in docs/analysis/2026-11-13_dead_reckoning_drift.md (814 simulated RTK dropouts on the
owner's lawns, ekf_map with wheel vy fused) as a p90 bound on the position error since the
last RTK FIXED, and used by localization_status_logic for ``est_drift_m``::

    f   = clamp((tilt - tilt_flat) / (tilt_slope - tilt_flat), 0, 1)
    est = base(f) + sum over the driven steps of rate(f_step, aid) x step + pivot_m x pivots
    base(f) = base_flat + (base_slope - base_flat) f        5 cm flat, 10 cm at 11 deg
    rate(f) = (rate_flat + (rate_slope - rate_flat) f) x aid scale   2 %/m flat, 10 %/m at 11 deg

``f`` of the base term comes from the tilt averaged since the last fix, the rate from the tilt
of each step. Pivots in place cost ``pivot_m`` each (measured 9 mm p90), no distance.

**Pluggable aid.** ``rate_scale`` maps an aiding source to a factor on the per-metre rate
(``{'vio': 0.5}``): a source that is healthy at a step (VIO today, gated by vio_gate) lowers
that step's rate. A new source adds one entry and one freshness input in the status logic.

Setting base_flat = base_slope = 0.03, rate_flat = rate_slope = 0.03 and pivot_m = 0 gives the
model before 2026-11-25 (3 cm + 3 %/m, 1.5 %/m with VIO).

**Coast budget** (:func:`coast_budget_m`): the distance a coast may drive without RTK,
``max_flat_m`` (5 m) up to ``tilt_flat`` (4 deg), ``max_slope_m`` (1.5 m) from
``budget_tilt_slope`` (8 deg) on, linear in between. The time cap and the slip stop are the
status logic's.

**Slip** (:class:`SlipDetector`): the wheel odometry yaw and the gyro-led fused yaw part by
``yaw_rad`` (0.6 rad) within ``window_s`` (2 s). Wheel and gyro dead reckoning cannot follow a
slide (one field slide of 1.4 m with the wheels reading 0), so the status goes to lost.
"""

import math
from collections import deque
from dataclasses import dataclass, field


@dataclass
class DriftParams:
    base_flat_m: float = 0.05
    base_slope_m: float = 0.10
    rate_flat: float = 0.02         # m of error per m driven, flat
    rate_slope: float = 0.10        # ... at tilt_slope_deg and steeper
    tilt_flat_deg: float = 4.0      # at or below: flat
    tilt_slope_deg: float = 11.0    # at or above: the slope terms
    pivot_m: float = 0.01           # per pivot in place
    rate_scale: dict = field(default_factory=lambda: {'vio': 0.5})


@dataclass
class CoastParams:
    max_flat_m: float = 5.0         # coast distance on flat ground (tilt <= DriftParams.tilt_flat_deg)
    max_slope_m: float = 1.5        # ... from tilt_slope_deg on
    tilt_slope_deg: float = 8.0
    max_s: float = 60.0             # since the last RTK FIXED (<= 0: no coast budget)


def tilt_fraction(tilt_deg, p):
    """0 on flat ground, 1 from tilt_slope_deg, linear in between (None = flat)."""
    if tilt_deg is None or p.tilt_slope_deg <= p.tilt_flat_deg:
        return 0.0
    return min(1.0, max(0.0, (tilt_deg - p.tilt_flat_deg) / (p.tilt_slope_deg - p.tilt_flat_deg)))


def drift_base_m(tilt_mean_deg, p):
    return p.base_flat_m + (p.base_slope_m - p.base_flat_m) * tilt_fraction(tilt_mean_deg, p)


def drift_rate(tilt_deg, p, aid=None):
    """Error per metre driven at ``tilt_deg`` with aiding source ``aid`` (None = wheels + gyro)."""
    rate = p.rate_flat + (p.rate_slope - p.rate_flat) * tilt_fraction(tilt_deg, p)
    return rate * float(p.rate_scale.get(aid, 1.0)) if aid else rate


def drift_estimate(dist_m, tilt_deg, pivots, p, aid=None):
    """est_drift_m for ``dist_m`` driven on constant ``tilt_deg`` with ``pivots`` pivots."""
    return drift_base_m(tilt_deg, p) + drift_rate(tilt_deg, p, aid) * dist_m + p.pivot_m * pivots


def coast_budget_m(tilt_mean_deg, drift, coast):
    """Distance a coast may drive at the mean tilt since the last fix."""
    lo, hi = drift.tilt_flat_deg, coast.tilt_slope_deg
    t = 0.0 if tilt_mean_deg is None else tilt_mean_deg
    if t <= lo or hi <= lo:
        return coast.max_flat_m
    f = min(1.0, (t - lo) / (hi - lo))
    return coast.max_flat_m + (coast.max_slope_m - coast.max_flat_m) * f


def float_disagrees(distance_m, est_m, gate_m):
    """An RTK float / fresh fixed position ``distance_m`` from the dead-reckoned pose is
    rejected when it is further than ``gate_m`` + the drift estimate (gate_m <= 0: off)."""
    return gate_m > 0.0 and distance_m > gate_m + max(0.0, est_m)


def compose(a, b):
    """2D pose a (+) b."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def relative(a, b):
    """b expressed in a's frame: inverse(a) (+) b."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return (c * dx + s * dy, -s * dx + c * dy, b[2] - a[2])


class PivotCounter:
    """Counts turns in place of at least ``min_deg`` (|vx| under ``max_vx`` throughout)."""

    def __init__(self, min_deg=45.0, max_vx=0.05):
        self.min_rad, self.max_vx = math.radians(min_deg), max_vx
        self.count = 0
        self._turn = 0.0
        self._yaw = None

    def reset(self):
        self.count, self._turn = 0, 0.0

    def update(self, yaw, vx):
        prev, self._yaw = self._yaw, yaw
        if prev is None or abs(vx) >= self.max_vx:
            self._turn = 0.0
            return
        before = self._turn
        self._turn += abs(math.atan2(math.sin(yaw - prev), math.cos(yaw - prev)))
        if before < self.min_rad <= self._turn:
            self.count += 1


class SlipDetector:
    """Wheel yaw against gyro yaw (rates in rad/s, fed by :meth:`update`): :meth:`slipped` once
    they part by ``yaw_rad`` within ``window_s`` (``yaw_rad`` <= 0: off)."""

    def __init__(self, window_s=2.0, yaw_rad=0.6):
        self.window_s, self.yaw_rad = window_s, yaw_rad
        self._ring = deque()          # (t, integral of wheel - gyro yaw rate)
        self._acc = 0.0
        self._t = None

    def reset(self):
        self._ring.clear()
        self._acc, self._t = 0.0, None

    def update(self, t, wheel_wz, gyro_wz):
        if self._t is not None and 0.0 < t - self._t <= self.window_s:
            self._acc += (wheel_wz - gyro_wz) * (t - self._t)
        self._t = t
        ring = self._ring
        ring.append((t, self._acc))
        while ring and t - ring[0][0] > self.window_s:
            ring.popleft()

    def slipped(self):
        return self.yaw_rad > 0.0 and any(abs(self._acc - a) >= self.yaw_rad
                                          for _, a in self._ring)

