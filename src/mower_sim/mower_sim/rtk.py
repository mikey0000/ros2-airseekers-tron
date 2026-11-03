# SPDX-License-Identifier: Apache-2.0
"""ROS-free RTK degradation model: timed FLOAT / dropout segments and multipath (2026-11-03).

The sim's GNSS was always RTK FIXED, so the localization monitor's drift budget, the
mission's loc_hold (docs/localization.md, mission.yaml ``loc_*``) and the slip check's
"no RTK, no real reference" branch never ran. A world (or sim_gate.sh ``--rtk-loss``) can
declare::

    rtk:
      anchor: blade            # event times count from: blade (first cutter on), undock
                               # (first time off the dock) or sim (sim start)
      events:
        - {t0: 120, dur: 60, kind: float}     # FLOAT: /fix offset by a random walk
        - {t0: 400, dur: 30, kind: dropout}   # no /fix, no /fix_status at all
      float_min_m: 0.3         # float offset magnitude kept in [min, max]
      float_max_m: 1.0
      float_std_m: 0.5         # Ornstein-Uhlenbeck per axis (std, correlation time)
      float_tau_s: 20.0
      seed: 1
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

FIXED, FLOAT, DROPOUT = 'fixed', 'float', 'dropout'
ANCHORS = ('blade', 'undock', 'sim')


@dataclass
class RtkEvent:
    t0: float
    dur: float
    kind: str = FLOAT


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

    def active(self) -> bool:
        return bool(self.events) or self.multipath is not None


def parse_spec(spec: str) -> RtkEvent:
    """``t0:dur:kind`` (sim_gate.sh --rtk-loss) -> RtkEvent."""
    parts = spec.split(':')
    if len(parts) not in (2, 3):
        raise ValueError('rtk-loss %r: want t0:dur[:float|dropout]' % spec)
    kind = parts[2] if len(parts) == 3 else DROPOUT
    if kind not in (FLOAT, DROPOUT):
        raise ValueError('rtk-loss %r: kind must be float or dropout' % spec)
    return RtkEvent(float(parts[0]), float(parts[1]), kind)


def rtk_from_dict(doc: Optional[dict]) -> Optional[RtkParams]:
    if not doc:
        return None
    ev = []
    for e in doc.get('events') or []:
        kind = str(e.get('kind', FLOAT))
        if kind not in (FLOAT, DROPOUT):
            raise ValueError('rtk: event kind %r (float | dropout)' % kind)
        ev.append(RtkEvent(float(e['t0']), float(e['dur']), kind))
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
                                                'float_tau_s') if k in doc})
    return p if p.active() else None


class RtkModel:
    def __init__(self, p: RtkParams):
        self.p = p
        self.rng = random.Random(int(p.seed) * 104729 + 3)
        self.anchor_t: Optional[float] = 0.0 if p.anchor == 'sim' else None
        self._was_docked = False
        self.err = [0.0, 0.0]
        self.mode = FIXED

    def notify(self, t: float, docked: bool, cutting: bool):
        """Feed the anchor: first cutter on (blade) / first departure from the dock."""
        if self.anchor_t is not None:
            return
        if self.p.anchor == 'blade' and cutting:
            self.anchor_t = t
        elif self.p.anchor == 'undock':
            if docked:
                self._was_docked = True
            elif self._was_docked:
                self.anchor_t = t

    def event_mode(self, t: float) -> str:
        if self.anchor_t is None:
            return FIXED
        rel = t - self.anchor_t
        mode = FIXED
        for e in self.p.events:
            if e.t0 <= rel < e.t0 + e.dur:
                if e.kind == DROPOUT:
                    return DROPOUT
                mode = FLOAT
        return mode

    def step(self, t: float, dt: float, ax: float, ay: float) -> Tuple[str, float, float]:
        """(mode, dx, dy) for the antenna at (ax, ay) at sim time t."""
        p = self.p
        mode = self.event_mode(t)
        bx = by = 0.0
        if p.multipath is not None:
            bx, by, mp_float = p.multipath.at(ax, ay)
            if mp_float and mode == FIXED:
                mode = FLOAT
        if mode == FLOAT:
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
