# SPDX-License-Identifier: Apache-2.0
"""ROS-free terrain model: side slopes, tilt and load-dependent wheel slip (2026-11-03).

Field evidence (docs/STATUS.md "Boundary recovery on a side slope", "spin_drift" incidents):
at the area 1 west edge (8-12 deg roll) a 0.1 m/s reverse turned the wheels 3.8 m while the
RTK pose moved 0.4 m and crept downhill, and pivots on slopes slide 0.15-0.30 m. The flat
kinematic drive could not fail that way. A world can now declare slope regions::

    terrain:
      regions:
        - name: west_edge
          polygon: [[x, y], ...]          # map metres
          slope_deg: 10.0                 # steepest-descent angle
          downhill_deg: 180.0             # map-frame direction of downhill (0 = +x / east)
          fade_m: 0.4                     # slope ramps 0 -> full over this inside the edge
      slip: {...}                         # SlipParams overrides (below)

Roll / pitch follow from the slope and the heading (REP-103: roll > 0 = right side down,
pitch > 0 = nose down). The overlap of regions takes the steepest.

Slip (``SlipParams``; f = slope factor 0 at ``onset_deg`` .. 1 at ``full_deg``, held above):

* longitudinal: ground speed = wheel speed x (1 - slip), slip = f x (``fwd_max`` driving
  forward, ``rev_max`` reversing: the castor leads and the drive wheels unload) x a speed
  factor sqrt(min(1, |v| / ``v_ref``)) (slow wheels grip better);
* pivots: yaw rate x (1 - f x ``pivot_yaw_loss``);
* lateral creep: the body moves DOWNHILL (map frame) at f x (``creep_drive`` while driving,
  ``creep_pivot`` while counter-rotating, blended by the pivot share), scaled by the wheel
  activity min(1, |wheel| / ``act_ref``); standing still does not creep.

Calibration (``SlipParams`` defaults, 10 deg): a 0.1 m/s reverse makes 0.005 m/s along the
track plus 0.010 m/s downhill (wheel odom / real ~ 9, field 3.8 / 0.4 = 9.5); a 90 deg pivot
at 0.3 rad/s (5.2 s) slides ~0.23 m (field 0.15-0.30 m); forward mowing at 0.2 m/s loses
25 % and creeps 1 cm/s. The wheel encoders (sim /odom) see the commanded wheel speeds.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, fields
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]


def _pip(p: Point, poly: Sequence[Point]) -> bool:
    x, y = p
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def _edge_dist(p: Point, poly: Sequence[Point]) -> float:
    best = math.inf
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 <= 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
        best = min(best, math.hypot(ax + t * dx - p[0], ay + t * dy - p[1]))
    return best


@dataclass
class SlopeRegion:
    polygon: List[Point]
    slope_deg: float = 10.0
    downhill_deg: float = 0.0
    fade_m: float = 0.4
    name: str = 'slope'

    def slope_at(self, p: Point) -> float:
        """Slope (rad) at p: 0 outside, faded in over ``fade_m`` inside the edge."""
        if not _pip(p, self.polygon):
            return 0.0
        s = math.radians(self.slope_deg)
        if self.fade_m > 0.0:
            s *= min(1.0, _edge_dist(p, self.polygon) / self.fade_m)
        return s


@dataclass
class SlipParams:
    onset_deg: float = 3.0
    full_deg: float = 10.0
    fwd_max: float = 0.25
    rev_max: float = 0.95
    v_ref: float = 0.1
    pivot_yaw_loss: float = 0.2
    creep_drive: float = 0.010     # m/s downhill at full slope while driving
    creep_pivot: float = 0.045     # m/s downhill at full slope while pivoting
    act_ref: float = 0.06          # m/s wheel speed for full creep activity

    def factor(self, slope_rad: float) -> float:
        d = math.degrees(abs(slope_rad))
        if d <= self.onset_deg:
            return 0.0
        if self.full_deg <= self.onset_deg:
            return 1.0
        return min(1.0, (d - self.onset_deg) / (self.full_deg - self.onset_deg))


@dataclass
class Terrain:
    regions: List[SlopeRegion] = field(default_factory=list)
    slip: SlipParams = field(default_factory=SlipParams)

    def gradient(self, x: float, y: float) -> Tuple[float, float, float]:
        """(slope rad, downhill unit x, y) at (x, y); steepest region wins."""
        best = (0.0, 1.0, 0.0)
        for r in self.regions:
            s = r.slope_at((x, y))
            if s > best[0]:
                a = math.radians(r.downhill_deg)
                best = (s, math.cos(a), math.sin(a))
        return best

    def roll_pitch(self, x: float, y: float, yaw: float) -> Tuple[float, float]:
        """(roll, pitch) rad of a body at (x, y, yaw) (REP-103 signs, see module doc)."""
        s, gx, gy = self.gradient(x, y)
        if s == 0.0:
            return 0.0, 0.0
        c, sn = math.cos(yaw), math.sin(yaw)
        along = gx * c + gy * sn          # downhill component along body +x
        left = -gx * sn + gy * c          # downhill component along body +y
        ss = math.sin(s)
        return (-math.asin(max(-1.0, min(1.0, ss * left))),
                math.asin(max(-1.0, min(1.0, ss * along))))

    def apply(self, x: float, y: float, yaw: float, v: float, w: float,
              left: float, right: float) -> Tuple[float, float, float, float]:
        """Ground twist after slope slip: (v, w, creep_x, creep_y map frame m/s).

        ``v``, ``w`` are the ground twist the flat model would produce, ``left`` / ``right``
        the wheel (encoder) speeds."""
        s, gx, gy = self.gradient(x, y)
        P = self.slip
        f = P.factor(s)
        if f <= 0.0:
            return v, w, 0.0, 0.0
        lin = 0.5 * (left + right)
        half = 0.5 * (right - left)
        act = max(abs(left), abs(right))
        pivot_share = abs(half) / (abs(lin) + abs(half)) if act > 1e-6 else 0.0
        if abs(lin) > 1e-6:
            spd = math.sqrt(min(1.0, abs(lin) / max(1e-6, P.v_ref)))
            slip = f * (P.rev_max if lin < 0.0 else P.fwd_max) * spd
            v *= max(0.0, 1.0 - slip)
        w *= max(0.0, 1.0 - f * P.pivot_yaw_loss * pivot_share)
        creep = f * (P.creep_drive * (1.0 - pivot_share) + P.creep_pivot * pivot_share) \
            * min(1.0, act / max(1e-6, P.act_ref))
        return v, w, creep * gx, creep * gy


def terrain_from_dict(doc: Optional[dict]) -> Optional[Terrain]:
    """World ``terrain:`` block -> Terrain, or None (flat world; behaviour unchanged)."""
    if not doc:
        return None
    regions = []
    for i, r in enumerate(doc.get('regions') or []):
        poly = [(float(p[0]), float(p[1])) for p in r.get('polygon') or []]
        if len(poly) < 3:
            raise ValueError('terrain: region %d needs a polygon of >= 3 points' % i)
        regions.append(SlopeRegion(
            polygon=poly, slope_deg=float(r.get('slope_deg', 10.0)),
            downhill_deg=float(r.get('downhill_deg', 0.0)),
            fade_m=float(r.get('fade_m', 0.4)), name=str(r.get('name', 'slope_%d' % i))))
    if not regions:
        return None
    names = {f.name for f in fields(SlipParams)}
    slip_doc = doc.get('slip') or {}
    bad = sorted(set(slip_doc) - names)
    if bad:
        raise ValueError('terrain: unknown slip params %s' % bad)
    return Terrain(regions=regions, slip=SlipParams(**{k: float(v) for k, v in slip_doc.items()}))
