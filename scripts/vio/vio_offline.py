# SPDX-License-Identifier: GPL-3.0-or-later
"""Offline VIO evaluation helpers (2026-11-14, docs/vio.md section 8). No ROS imports.

The live chain is ov_msckf -> ``vio_odom_bridge`` -> ``vio_gate`` -> EKF. Offline the first
stage is ``ros2_serial_msckf`` (config/vio/serial/), which writes OpenVINS' odomimu as CSV;
the two Python stages run here through the SAME pure code the nodes use:

* bridge: ``stereo_vio_bridge.odom_convert`` (``base_twist``, ``body_twist_covariance``,
  ``rpy_matrix``) with the Metoak parameters of launch/vio.launch.py;
* gate: ``mower_localization.vio_gate_logic.VioGateLogic``, fed in time order with
  /fix_status, wheel vx and the VIO messages. A simulated RTK dropout (ekf_replay's withheld
  windows) replaces the status text inside the window by a non-FIXED one, so the gate opens
  after its hold-off exactly as it would on a real dropout.

Differences from the live nodes, all small: the gate's clock is the message time (live:
time.monotonic() at receipt); the bridge's stale-message drop (``max_age_s``) never fires
(no processing lag offline); the odomimu row is taken after the camera update its IMU sample
released (live: just before).

Also: RTK body velocity from map-frame fixes (truth for vx / vy), percentiles, a reason
classifier for the gate state. Used by ``eval_vio_bag.py`` and ``scripts/replay/ekf_replay.py``.
"""

from __future__ import annotations

import bisect
import csv
import math
import os
import re
import sys
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
STACK = os.path.dirname(os.path.dirname(HERE))
for _pkg in ('mower_cameras', 'mower_localization', 'stereo_vio_bridge'):
    _p = os.path.join(STACK, 'src', _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from mower_cameras.activity_gate import is_low_power  # noqa: E402
from mower_localization.vio_gate_logic import VioGateLogic, VioGateParams  # noqa: E402
from stereo_vio_bridge.odom_convert import (base_twist, body_twist_covariance,  # noqa: E402
                                            rpy_matrix)

# launch/vio.launch.py, vio_imu:=metoak (test_vio_offline checks they still match) and the
# vio_odom_bridge defaults for cov_scale / min_twist_var.
METOAK_BRIDGE = {'imu_to_base_rpy': (-1.586797, 0.0, 1.570796),
                 'imu_xyz_in_base': (0.451, -0.0514, 0.2352),
                 'cov_scale': 10.0, 'min_twist_var': 0.04}
WITHHELD_STATUS = 'WITHHELD simulated RTK dropout'
OV_COLUMNS = ('t', 'qx', 'qy', 'qz', 'qw', 'px', 'py', 'pz', 'vx', 'vy', 'vz',
              'wx', 'wy', 'wz', 'var_vx', 'var_vy', 'var_vz')
VIO_COLUMNS = ('t', 'vx', 'vy', 'vz', 'var_vx', 'var_vy', 'wz')


# ---------------------------------------------------------------------------- bridge
def read_csv_rows(path: str) -> List[Dict[str, float]]:
    """Numeric CSV with a header -> list of dicts."""
    with open(path, newline='') as fh:
        r = csv.DictReader(fh)
        return [{k: float(v) for k, v in row.items()} for row in r]


def bridge_rows(ov_rows: Iterable[Dict[str, float]], params: Optional[Dict] = None
                ) -> List[Tuple[float, ...]]:
    """OpenVINS odomimu rows -> /odometry/vio rows (t, vx, vy, vz, var_vx, var_vy, wz) in
    base_link, as ``vio_odom_bridge`` publishes them."""
    p = dict(METOAK_BRIDGE, **(params or {}))
    r = rpy_matrix(*p['imu_to_base_rpy'])
    out = []
    for o in ov_rows:
        v, w = base_twist(r, p['imu_xyz_in_base'], (o['vx'], o['vy'], o['vz']),
                          (o['wx'], o['wy'], o['wz']))
        cov = [0.0] * 36
        cov[0], cov[7], cov[14] = o['var_vx'], o['var_vy'], o['var_vz']
        bc = body_twist_covariance(cov, p['cov_scale'], p['min_twist_var'])
        out.append((o['t'], v[0], v[1], v[2], bc[0], bc[7], w[2]))
    return out


# ---------------------------------------------------------------------------- gate
def in_windows(windows: Sequence[Tuple[float, float]], t: float) -> bool:
    """Whether t lies in one of the sorted, disjoint [(start, end)]."""
    k = bisect.bisect_right(windows, (t, math.inf)) - 1
    return k >= 0 and t < windows[k][1]


def run_gate(vio: Sequence[Tuple[float, ...]], status: Sequence[Tuple[float, str]],
             wheel: Sequence[Tuple[float, float]],
             windows: Sequence[Tuple[float, float]] = (),
             params: Optional[VioGateParams] = None) -> List[Tuple[float, Optional[str]]]:
    """Gate verdict per VIO row: [(t, None if passed else reason)].

    ``vio`` rows (t, vx, vy, vz, var_vx, var_vy, ...), ``status`` [(t, /fix_status text)],
    ``wheel`` [(t, vx)]; inside ``windows`` the status reads as a simulated dropout."""
    logic = VioGateLogic(params)
    events = [(t, 0, s) for t, s in status]
    events += [(t, 1, vx) for t, vx in wheel]
    events += [(r[0], 2, r) for r in vio]
    events.sort(key=lambda e: (e[0], e[1]))
    wins = sorted(windows)
    out = []
    for t, kind, x in events:
        if kind == 0:
            logic.on_fix_status(WITHHELD_STATUS if in_windows(wins, t) else x, t)
        elif kind == 1:
            logic.on_wheel(x, t)
        else:
            out.append((t, logic.on_vio(t, x[1], x[2], x[4], x[5])))
    return out


_NUM = re.compile(r'[-+]?\d+(\.\d+)?(/\d+)?')


def reason_class(reason: Optional[str]) -> str:
    """Gate reason without its numbers ('VIO warming up (3/20)' -> 'VIO warming up ()')."""
    if reason is None:
        return 'PASSING'
    return _NUM.sub('', reason).replace('  ', ' ').strip()


def gate_summary(verdicts: Sequence[Tuple[float, Optional[str]]], max_dt: float = 0.5
                 ) -> Dict:
    """Time share per reason class, passing share, and 'diverged' latch episodes.

    Each verdict holds until the next one (capped at ``max_dt``: a VIO gap is no time)."""
    shares: Dict[str, float] = {}
    total, episodes, in_div, div_s = 0.0, 0, False, 0.0
    for (t, r), nxt in zip(verdicts, list(verdicts[1:]) + [None]):
        dt = min(nxt[0] - t, max_dt) if nxt is not None else 0.0
        c = reason_class(r)
        shares[c] = shares.get(c, 0.0) + dt
        total += dt
        div = r is not None and 'diverged' in r
        if div and not in_div:
            episodes += 1
        if div:
            div_s += dt
        in_div = div
    return {'seconds': total,
            'share': {k: v / total for k, v in sorted(shares.items())} if total else {},
            'passing_share': shares.get('PASSING', 0.0) / total if total else 0.0,
            'diverged_episodes': episodes, 'diverged_s': div_s}


# ---------------------------------------------------------------------------- start
def full_rate_activity(activity: Sequence[Tuple[float, str]]) -> float:
    """Time of the first /mission/activity [(t, text)] outside the camera nodes' idle duty
    cycle (mower_cameras.activity_gate: 1 Hz stereo pairs, 100 Hz stereo IMU while idle);
    0.0 when the bag does not start idle (no activity = full rate, as the nodes do)."""
    if not activity or not is_low_power(activity[0][1]):
        return 0.0
    return next((t for t, a in activity if not is_low_power(a)), activity[-1][0])


def imu_settled(stamps: Sequence[float], max_gap: float = 0.05, settle_s: float = 2.0
                ) -> Optional[float]:
    """First IMU stamp followed by ``settle_s`` without a gap over ``max_gap``: stereo_imu
    reopens the IIO buffers at each rate change and loses 0.4-1.2 s of samples."""
    start = stamps[0] if stamps else None
    for a, b in zip(stamps, stamps[1:]):
        if b - a > max_gap:
            start = b
        elif b - start >= settle_s:
            return start
    return None


# ---------------------------------------------------------------------------- truth, stats
def rtk_body_velocity(fixes: Sequence[Tuple[float, float, float]], ts: Sequence[float],
                      yaw_at, t: float, half: float = 0.5, tol: float = 0.15
                      ) -> Optional[Tuple[float, float]]:
    """Body (vx, vy) from map-frame fixes [(t, x, y)] (sorted, RTK FIXED, base_link point;
    ``ts`` their times): central difference over t +- ``half`` (fixes within ``tol`` of those
    times), rotated by ``yaw_at(t)`` (map yaw, or None when unknown)."""
    a = _nearest(fixes, ts, t - half, tol)
    b = _nearest(fixes, ts, t + half, tol)
    yaw = yaw_at(t)
    if a is None or b is None or yaw is None or b[0] - a[0] < half:
        return None
    vxm, vym = (b[1] - a[1]) / (b[0] - a[0]), (b[2] - a[2]) / (b[0] - a[0])
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * vxm + s * vym, -s * vxm + c * vym)


def _nearest(rows, ts, t, tol):
    k = bisect.bisect_left(ts, t)
    best = None
    for j in (k - 1, k):
        if 0 <= j < len(rows) and abs(ts[j] - t) <= tol:
            if best is None or abs(ts[j] - t) < abs(best[0] - t):
                best = rows[j]
    return best


def percentile(xs: Sequence[float], p: float) -> float:
    """Linear-interpolated percentile (numpy's default); nan for no data."""
    s = sorted(x for x in xs if x == x)
    if not s:
        return float('nan')
    k = (len(s) - 1) * p / 100.0
    lo = math.floor(k)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


class Interp:
    """Linear interpolation in sorted [(t, *values)]; None outside or across gaps."""

    def __init__(self, rows: Sequence[Tuple[float, ...]], max_gap: float = 0.5,
                 angle_cols: Sequence[int] = ()):
        self.rows = sorted(rows)
        self.times = [r[0] for r in self.rows]
        self.max_gap, self.angle_cols = max_gap, set(angle_cols)

    def at(self, t: float) -> Optional[Tuple[float, ...]]:
        k = bisect.bisect_left(self.times, t)
        if k <= 0 or k >= len(self.rows):
            if k < len(self.rows) and self.times[k] == t:
                return self.rows[k][1:]
            return None
        a, b = self.rows[k - 1], self.rows[k]
        if b[0] - a[0] > self.max_gap:
            return None
        f = (t - a[0]) / (b[0] - a[0])
        out = []
        for i in range(1, len(a)):
            d = b[i] - a[i]
            if i in self.angle_cols:
                d = math.atan2(math.sin(d), math.cos(d))
            out.append(a[i] + f * d)
        return tuple(out)
