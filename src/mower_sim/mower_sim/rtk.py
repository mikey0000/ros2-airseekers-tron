# SPDX-License-Identifier: Apache-2.0
"""ROS-free RTK degradation model: timed FLOAT / dropout segments and multipath (2026-11-03).

The sim's GNSS was always RTK FIXED, so the localization monitor's drift budget, the
mission's loc_hold (docs/localization.md, mission.yaml ``loc_*``) and the slip check's
"no RTK, no real reference" branch never ran. A world (or sim_gate.sh ``--rtk-loss``) can
declare::

    rtk:
      anchor: blade            # event times count from: blade (first cutter on), undock
                               # (first time off the dock), sim (sim start) or pivot (the
                               # first in-place rotation with the cutter on, held
                               # PIVOT_MIN_S: a coverage pivot; 2026-11-04 scenario ii)
      events:
        - {t0: 120, dur: 60, kind: float}     # FLOAT: /fix offset by a random walk
        - {t0: 400, dur: 30, kind: dropout}   # no /fix, no /fix_status at all
        - {t0: 500, dur: 14, kind: single}    # /fix + /fix_status SINGLE (gps_gate drops it;
                                              # field 2026-11-13: NTRIP corrections aged out)
        - {t0: 514, dur: 11, kind: float,     # optional per event (2026-11-13 dropout):
           offset: [0.0, -0.3],               #   fixed float offset (map m) instead of the walk
           map_gain: 1.0,                     #   share of it in the map estimate (loc_drift)
           ekf_sigma_m: 0.07}                 #   ekf_map sigma while it lasts (else the block's)
      float_min_m: 0.3         # float offset magnitude kept in [min, max]
      float_max_m: 1.0
      float_std_m: 0.5         # Ornstein-Uhlenbeck per axis (std, correlation time)
      float_tau_s: 20.0
      seed: 1
      pivot_min_s: 0.4         # anchor pivot: the rotation has lasted this long
      ekf_sigma_m: 0.0         # /odometry/filtered_map xy sigma while not FIXED (0 = off)
      multipath:               # optional: a "house" polygon biases the fix near it
        polygon: [[x, y], ...]
        range_m: 3.0           # bias fades from bias_m at the wall to 0 at range_m
        bias_m: 0.25           # pushed away from the wall
        float_within_m: 1.0    # closer than this the receiver also drops to FLOAT

``RtkModel.step`` returns the mode (``fixed`` / ``float`` / ``dropout``) and the (dx, dy) map
offset to add to the antenna position. The sim maps them onto /fix (status, covariance,
lat/lon), /fix_status (``solution=`` text the localization monitor classifies) and, with
loc_drift, the map estimate.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from .terrain import _edge_dist, _pip

FIXED, FLOAT, DROPOUT, SINGLE = 'fixed', 'float', 'dropout', 'single'
KINDS = (FLOAT, DROPOUT, SINGLE)
ANCHORS = ('blade', 'undock', 'sim', 'pivot')
PIVOT_MIN_S = 0.4        # anchor 'pivot': in-place rotation held this long
PIVOT_V, PIVOT_W = 0.02, 0.15   # |v| below, |w| above: rotating in place


@dataclass
class RtkEvent:
    t0: float
    dur: float
    kind: str = FLOAT
    offset: Optional[Tuple[float, float]] = None    # float: fixed offset (map m)
    map_gain: Optional[float] = None                # float: share in the map estimate
    ekf_sigma_m: Optional[float] = None             # ekf_map sigma while active


@dataclass
class Multipath:
    polygon: List[Tuple[float, float]]
    range_m: float = 3.0
    bias_m: float = 0.25
    float_within_m: float = 1.0

    def at(self, x: float, y: float) -> Tuple[float, float, bool]:
        """(bias dx, dy, float) for an antenna at (x, y)."""
        poly = self.polygon
        if _pip((x, y), poly):
            return 0.0, 0.0, True
        d = _edge_dist((x, y), poly)
        if d >= self.range_m:
            return 0.0, 0.0, False
        cx = sum(p[0] for p in poly) / len(poly)
        cy = sum(p[1] for p in poly) / len(poly)
        ux, uy = x - cx, y - cy
        n = math.hypot(ux, uy) or 1.0
        mag = self.bias_m * (1.0 - d / self.range_m)
        return mag * ux / n, mag * uy / n, d < self.float_within_m


@dataclass
class RtkParams:
    events: List[RtkEvent] = field(default_factory=list)
    anchor: str = 'blade'
    float_min_m: float = 0.3
    float_max_m: float = 1.0
    float_std_m: float = 0.5
    float_tau_s: float = 20.0
    seed: int = 1
    multipath: Optional[Multipath] = None
    # anchor 'pivot': the rotation must last this long first (a short pivot ends before a
    # hold could reach it)
    pivot_min_s: float = 0.4
    # /odometry/filtered_map xy sigma while not FIXED (0 = the odom covariance as before).
    # Field 2026-11-04: ekf_map sigma 0.17 m under RTK float -> the localization monitor's
    # loc_sigma_ok_m 0.15 check degraded at once, also standing / pivoting (no travel
    # for the drift budget)
    ekf_sigma_m: float = 0.0

    def active(self) -> bool:
        return bool(self.events) or self.multipath is not None


def parse_spec(spec: str) -> RtkEvent:
    """``t0:dur:kind`` (sim_gate.sh --rtk-loss) -> RtkEvent."""
    parts = spec.split(':')
    if len(parts) not in (2, 3):
        raise ValueError('rtk-loss %r: want t0:dur[:float|dropout]' % spec)
    kind = parts[2] if len(parts) == 3 else DROPOUT
    if kind not in KINDS:
        raise ValueError('rtk-loss %r: kind must be float, dropout or single' % spec)
    return RtkEvent(float(parts[0]), float(parts[1]), kind)


def rtk_from_dict(doc: Optional[dict]) -> Optional[RtkParams]:
    if not doc:
        return None
    ev = []
    for e in doc.get('events') or []:
        kind = str(e.get('kind', FLOAT))
        if kind not in KINDS:
            raise ValueError('rtk: event kind %r (float | dropout | single)' % kind)
        off = e.get('offset')
        ev.append(RtkEvent(float(e['t0']), float(e['dur']), kind,
                           None if off is None else (float(off[0]), float(off[1])),
                           None if e.get('map_gain') is None else float(e['map_gain']),
                           None if e.get('ekf_sigma_m') is None else float(e['ekf_sigma_m'])))
    anchor = str(doc.get('anchor', 'blade'))
    if anchor not in ANCHORS:
        raise ValueError('rtk: anchor %r not in %s' % (anchor, ANCHORS))
    mp = None
    if doc.get('multipath'):
        m = doc['multipath']
        poly = [(float(p[0]), float(p[1])) for p in m.get('polygon') or []]
        if len(poly) < 3:
            raise ValueError('rtk: multipath polygon needs >= 3 points')
        mp = Multipath(poly, float(m.get('range_m', 3.0)), float(m.get('bias_m', 0.25)),
                       float(m.get('float_within_m', 1.0)))
    p = RtkParams(events=ev, anchor=anchor, multipath=mp, seed=int(doc.get('seed', 1)),
                  **{k: float(doc[k]) for k in ('float_min_m', 'float_max_m', 'float_std_m',
                                                'float_tau_s', 'pivot_min_s',
                                                'ekf_sigma_m') if k in doc})
    return p if p.active() else None


class RtkModel:
    def __init__(self, p: RtkParams):
        self.p = p
        self.rng = random.Random(int(p.seed) * 104729 + 3)
        self.anchor_t: Optional[float] = 0.0 if p.anchor == 'sim' else None
        self._was_docked = False
        self._rot_since: Optional[float] = None
        self.err = [0.0, 0.0]
        self.mode = FIXED
        self.event: Optional[RtkEvent] = None    # the event that set the current mode

    def notify(self, t: float, docked: bool, cutting: bool, v: float = 0.0, w: float = 0.0):
        """Feed the anchor: first cutter on (blade) / first departure from the dock / first
        in-place rotation with the cutter on (pivot; body twist ``v``, ``w``)."""
        if self.anchor_t is not None:
            return
        if self.p.anchor == 'pivot':
            if cutting and not docked and abs(v) < PIVOT_V and abs(w) > PIVOT_W:
                if self._rot_since is None:
                    self._rot_since = t
                elif t - self._rot_since >= max(PIVOT_MIN_S, self.p.pivot_min_s):
                    self.anchor_t = t
            else:
                self._rot_since = None
        elif self.p.anchor == 'blade' and cutting:
            self.anchor_t = t
        elif self.p.anchor == 'undock':
            if docked:
                self._was_docked = True
            elif self._was_docked:
                self.anchor_t = t

    def event_mode(self, t: float) -> str:
        return self.active_event(t)[0]

    def active_event(self, t: float) -> Tuple[str, Optional[RtkEvent]]:
        """(mode, event) at sim time t: dropout over single over float, else fixed."""
        if self.anchor_t is None:
            return FIXED, None
        rel = t - self.anchor_t
        best = (FIXED, None)
        rank = {FIXED: 0, FLOAT: 1, SINGLE: 2, DROPOUT: 3}
        for e in self.p.events:
            if e.t0 <= rel < e.t0 + e.dur and rank[e.kind] > rank[best[0]]:
                best = (e.kind, e)
        return best

    def step(self, t: float, dt: float, ax: float, ay: float) -> Tuple[str, float, float]:
        """(mode, dx, dy) for the antenna at (ax, ay) at sim time t."""
        p = self.p
        mode, self.event = self.active_event(t)
        bx = by = 0.0
        if p.multipath is not None:
            bx, by, mp_float = p.multipath.at(ax, ay)
            if mp_float and mode == FIXED:
                mode = FLOAT
        if mode == FLOAT and self.event is not None and self.event.offset is not None:
            self.err = list(self.event.offset)
            bx += self.err[0]
            by += self.err[1]
        elif mode == FLOAT:
            if self.mode != FLOAT:          # entering float: a fresh offset of >= min
                a = self.rng.uniform(0.0, 2.0 * math.pi)
                self.err = [p.float_min_m * math.cos(a), p.float_min_m * math.sin(a)]
            a = min(1.0, dt / max(1e-3, p.float_tau_s))
            sig = p.float_std_m * math.sqrt(max(0.0, 2.0 * a - a * a))
            for k in (0, 1):
                self.err[k] += -a * self.err[k] + sig * self.rng.gauss(0.0, 1.0)
            r = math.hypot(*self.err)
            lo, hi = p.float_min_m, max(p.float_min_m, p.float_max_m)
            if r < 1e-9:
                self.err = [lo, 0.0]
            elif r < lo or r > hi:
                k = (lo if r < lo else hi) / r
                self.err = [self.err[0] * k, self.err[1] * k]
            bx += self.err[0]
            by += self.err[1]
        else:
            self.err = [0.0, 0.0]
        self.mode = mode
        return mode, bx, by
