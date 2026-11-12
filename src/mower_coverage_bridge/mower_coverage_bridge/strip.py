# Copyright 2026 michael
# SPDX-License-Identifier: Apache-2.0
"""Narrow-strip mode: detector and shuttle planner (pure geometry, numpy, no ROS).

Field 2026-11-13, Area 4: a strip 6.3 m long and 0.48-0.72 m wide. The Fields2Cover layout
(laps at 0 / 0.15 / 0.30 m in, rows 0.45 m in) needs a width over 0.90 m; on this strip it
collapsed to six fragments of lap 0 (plan coverage 0.315) with 0.1-0.2 m end turnarounds, and
the robot pivoted, stuck and left the area. docs/analysis/2026-11-13_narrow_strip.md.

**Detector** (:func:`measure`, :func:`detect`): an area is a narrow strip when its largest
inscribed circle has a radius under ``max_inscribed_r_m`` (0.385: a pivot sweeps a circle of
radius 0.645 about base_link, and 0.645 - 0.26 of soft overhang leaves 0.385), it is at least
``min_aspect`` (3) times as long as it is wide (minimum-area rectangle) and at least
``min_length_m`` (1.4) long. A shorter narrow area is no strip: its Fields2Cover plan meets the
plan-quality gate of the coverage server instead (refused under ``plan_coverage_min``).

**Planner** (:func:`plan_strip`): straight passes along the strip, driven as a shuttle,
without any pivot inside the strip.

* Edges. Every edge of the outline is an ordinary soft edge (owner 2026-11-13: the usual
  boundary mowing behaviour applies to both long sides); the short ends are lawn, which the
  same limits let the nose and tail run a little past. Holes: the footprint may not touch them.
* Body limits everywhere: the six body points of the boundary guard
  (``mower_mission.body_boundary``) may be outside the outline by at most the soft trip
  limits less a tracking margin (``out_limits``).
* Passes. For each candidate axis the valid stations of base_link are scanned on lines
  ``offset_step`` apart; each line's longest valid run, trimmed to where the blade still
  reaches the area, is a candidate pass. Passes are picked greedily by the new area their
  blade capsule cuts (raster), so the spacing follows the taper.
* Shuttle. Every pass is entered from the same end: forward to its far end, then reverse along
  the same line back to its start (the reverse retraces the track just driven; it may run on
  along the same line, inside the validated run, where the pass started earlier than its
  lane change ended). The next pass starts from there with a forward lane change: a cosine S
  from the current line onto the next one over the shortest of ``lane_change_m`` whose poses
  all pass the checks. A pass no lane change reaches is left out.
* Output: one point chain with reverse flags, cut by :func:`splitter.split_reverse` into
  forward / reverse sub-paths sharing their cusp poses (no transit between them).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from mower_coverage_bridge import splitter

Point = Tuple[float, float]

# Body outline from base_link (rear axle), mirrors mower_common.geometry FOOTPRINT_*.
FRONT_M, REAR_M, HALF_WIDTH_M = 0.60, 0.10, 0.235
# Blade disc centre ahead of base_link (URDF blade_joint) and its radius.
BLADE_X_M, BLADE_R_M = 0.23, 0.11

STRIP_MODES = ('auto', 'on', 'off')
# A successful strip plan's message starts with this (mower_mission fsm/common.py
# STRIP_PLAN_TAG mirrors it): the mission drives the plan as planned.
STRIP_TAG = 'strip plan:'


@dataclass
class StripParams:
    spacing_m: float = 0.18                 # nominal pass spacing (operation_width)
    front_m: float = FRONT_M
    rear_m: float = REAR_M
    half_width_m: float = HALF_WIDTH_M
    blade_x_m: float = BLADE_X_M
    blade_r_m: float = BLADE_R_M
    # body point kind -> how far it may be outside the outline: the soft guard limits
    # (body_boundary.limits_for: nose 0.40, tail 0.30, front corners 0.55, rear corners 0.35)
    # less a tracking margin
    out_limits: Dict[str, float] = field(default_factory=lambda: {
        'nose': 0.28, 'tail': 0.20, 'front': 0.40, 'side': 0.25,
        # base_link itself stays inside: the guard trips it at 0.10 out (BOUNDARY OUT),
        # and a pass start on a slanted end put it 0.11 out in the sim (2026-11-13)
        'base': 0.0})
    min_pass_m: float = 0.7                 # shorter passes are left out
    # every pass starts within this of the earliest pass start (the entry end), so each
    # retrace ends at the entry end and the plan ends there, never inside the strip
    entry_tol_m: float = 0.5
    entry_band_m: Tuple[float, float] = (0.3, 0.8)  # where the entry end's width is measured
    cut_tie: float = 0.03                   # cut fractions this close: the wider entry wins
    min_gain_m2: float = 0.03               # a pass must add at least this much cut area
    station_step_m: float = 0.02
    offset_step_m: float = 0.01
    lane_change_m: Tuple[float, ...] = (1.2, 1.5, 2.0, 1.0, 0.8)
    lane_change_step_m: float = 0.05        # pose spacing on a lane change
    lane_change_delay_m: Tuple[float, ...] = (0.0, 0.2, 0.4, 0.6, 0.8, 1.0)
    # the last lane change back onto the first pass's line (several pass widths), so the plan
    # ends where it began: the pose the inbound transit reached (sim 2026-11-13: the plan's
    # end on the far long side was off the route graph and the exit transit left the area)
    return_lane_change_m: Tuple[float, ...] = (2.0, 2.5, 3.0, 1.5)
    raster_m: float = 0.02
    end_edge_deg: float = 45.0              # steeper edges near an extreme are end edges
    # detector
    max_inscribed_r_m: float = 0.385
    min_aspect: float = 3.0
    min_length_m: float = 1.4


@dataclass
class StripShape:
    inscribed_r: float          # largest inscribed circle radius (m)
    length: float               # minimum-area rectangle, long side (m)
    width: float                # ... short side (m)
    axis_deg: float             # long side direction, [0, 180)


@dataclass
class StripPlan:
    ok: bool
    message: str
    axis_deg: float = 0.0
    passes: List[Tuple[Point, Point]] = field(default_factory=list)   # straight part per pass
    points: List[Point] = field(default_factory=list)                  # the whole chain
    reverse_flags: List[bool] = field(default_factory=list)
    cut_fraction: float = 0.0     # planned blade cut / area (raster)
    skipped_passes: int = 0

    def split(self) -> splitter.SplitResult:
        """Segments (each pass's straight part, SEGMENT_SWATH) and the drivable sub-paths
        (forward / reverse, :func:`splitter.split_reverse`)."""
        segs = [splitter.Segment(splitter.SEGMENT_SWATH, [a, b]) for a, b in self.passes]
        chain = splitter.SubPath(list(self.points), list(range(len(segs))),
                                 list(self.reverse_flags))
        return splitter.SplitResult(segs, splitter.split_reverse([chain]), MODE_STRIP)


MODE_STRIP = 'strip'


# ------------------------------------------------------------------ small geometry

def _ring(points: Sequence[Point]) -> np.ndarray:
    a = np.asarray([(float(p[0]), float(p[1])) for p in points], dtype=float)
    if len(a) > 1 and np.hypot(*(a[0] - a[-1])) < 1e-9:
        a = a[:-1]
    return a


def _edges(ring: np.ndarray) -> np.ndarray:
    """(M, 4) segments x0 y0 x1 y1 of a closed ring."""
    return np.hstack([ring, np.roll(ring, -1, axis=0)])


def _rot(pts: np.ndarray, ang: float) -> np.ndarray:
    """Rotate points by -ang (into the frame whose x axis is at ``ang``)."""
    c, s = math.cos(ang), math.sin(ang)
    return np.stack([c * pts[..., 0] + s * pts[..., 1], -s * pts[..., 0] + c * pts[..., 1]],
                    axis=-1)


def _unrot(pts: np.ndarray, ang: float) -> np.ndarray:
    return _rot(pts, -ang)


def _pip(px: np.ndarray, py: np.ndarray, ring: np.ndarray) -> np.ndarray:
    """Point in polygon, vectorised: (N,) bool."""
    x0, y0 = ring[:, 0][None, :], ring[:, 1][None, :]
    x1, y1 = np.roll(ring[:, 0], -1)[None, :], np.roll(ring[:, 1], -1)[None, :]
    X, Y = px[:, None], py[:, None]
    cond = (y0 > Y) != (y1 > Y)
    with np.errstate(divide='ignore', invalid='ignore'):
        xint = (x1 - x0) * (Y - y0) / (y1 - y0) + x0
    return np.count_nonzero(cond & (X < xint), axis=1) % 2 == 1


def _seg_dist(px: np.ndarray, py: np.ndarray, segs: np.ndarray) -> np.ndarray:
    """Distance of each point to the nearest segment: (N,)."""
    if len(segs) == 0:
        return np.full(len(px), np.inf)
    ax, ay, bx, by = (segs[:, k][None, :] for k in range(4))
    dx, dy = bx - ax, by - ay
    L2 = np.maximum(dx * dx + dy * dy, 1e-12)
    X, Y = px[:, None], py[:, None]
    t = np.clip(((X - ax) * dx + (Y - ay) * dy) / L2, 0.0, 1.0)
    return np.sqrt((ax + t * dx - X) ** 2 + (ay + t * dy - Y) ** 2).min(axis=1)


def _outside(px: np.ndarray, py: np.ndarray, ring: np.ndarray, segs: np.ndarray) -> np.ndarray:
    """How far each point is outside the ring (0 inside)."""
    d = _seg_dist(px, py, segs)
    return np.where(_pip(px, py, ring), 0.0, d)


def _rect_hits(x: np.ndarray, y: np.ndarray, yaw: np.ndarray, segs: np.ndarray,
               x0: float, x1: float, hw: float) -> np.ndarray:
    """(N,) True where any segment touches the body rectangle [x0, x1] x [-hw, hw] of the pose
    (x, y, yaw)."""
    if len(segs) == 0:
        return np.zeros(len(x), dtype=bool)
    c, s = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
    ax, ay = segs[:, 0][None, :] - x[:, None], segs[:, 1][None, :] - y[:, None]
    bx, by = segs[:, 2][None, :] - x[:, None], segs[:, 3][None, :] - y[:, None]
    u0, v0 = c * ax + s * ay, -s * ax + c * ay
    u1, v1 = c * bx + s * by, -s * bx + c * by
    du = u1 - u0
    eps = 1e-12
    with np.errstate(divide='ignore', invalid='ignore'):
        ta, tb = (x0 - u0) / du, (x1 - u0) / du
    inside0 = (u0 >= x0) & (u0 <= x1)
    tlo = np.where(du > eps, ta, np.where(du < -eps, tb, np.where(inside0, 0.0, np.inf)))
    thi = np.where(du > eps, tb, np.where(du < -eps, ta, np.where(inside0, 1.0, -np.inf)))
    tlo, thi = np.maximum(tlo, 0.0), np.minimum(thi, 1.0)
    ok = tlo <= thi
    tlo_f, thi_f = np.where(ok, tlo, 0.0), np.where(ok, thi, 0.0)
    va, vb = v0 + (v1 - v0) * tlo_f, v0 + (v1 - v0) * thi_f
    hit = ok & (np.maximum(va, vb) >= -hw) & (np.minimum(va, vb) <= hw)
    return hit.any(axis=1)


def _hull(pts: np.ndarray) -> np.ndarray:
    p = sorted(map(tuple, pts))
    if len(p) < 3:
        return np.asarray(p)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for q in p:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 0:
            lower.pop()
        lower.append(q)
    for q in reversed(p):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 0:
            upper.pop()
        upper.append(q)
    return np.asarray(lower[:-1] + upper[:-1])


def min_area_rect(points: Sequence[Point]) -> Tuple[float, float, float]:
    """(length, width, axis rad) of the minimum-area enclosing rectangle; axis along the long
    side, folded to [0, pi)."""
    h = _hull(_ring(points))
    best = None
    for i in range(len(h)):
        a, b = h[i], h[(i + 1) % len(h)]
        ang = math.atan2(b[1] - a[1], b[0] - a[0])
        r = _rot(h, ang)
        w, t = np.ptp(r[:, 0]), np.ptp(r[:, 1])
        if best is None or w * t < best[0]:
            best = (w * t, w, t, ang)
    _, w, t, ang = best
    if t > w:
        w, t, ang = t, w, ang + math.pi / 2.0
    return float(w), float(t), math.fmod(ang + 2.0 * math.pi, math.pi)


def inscribed_radius(outer: Sequence[Point], holes: Sequence[Sequence[Point]] = (),
                     step: float = 0.02, stop_above: Optional[float] = None) -> float:
    """Largest inscribed circle radius, from a grid of ``step`` (to within about step / 2).
    ``stop_above``: return as soon as a radius over it is found (the detector only needs to
    know that the area is wide)."""
    ring = _ring(outer)
    segs = np.vstack([_edges(ring)] + [_edges(_ring(h)) for h in holes if len(h) >= 3])
    hole_rings = [_ring(h) for h in holes if len(h) >= 3]
    lo, hi = ring.min(axis=0), ring.max(axis=0)
    xs = np.arange(lo[0] + step / 2, hi[0], step)
    ys = np.arange(lo[1] + step / 2, hi[1], step)
    best = 0.0
    chunk = max(1, 200000 // max(1, len(segs) * len(xs)))
    for i in range(0, len(ys), chunk):
        gx, gy = np.meshgrid(xs, ys[i:i + chunk])
        px, py = gx.ravel(), gy.ravel()
        inside = _pip(px, py, ring)
        for hr in hole_rings:
            inside &= ~_pip(px, py, hr)
        if not inside.any():
            continue
        d = _seg_dist(px[inside], py[inside], segs)
        best = max(best, float(d.max()))
        if stop_above is not None and best > stop_above:
            return best
    return best


def measure(outer: Sequence[Point], holes: Sequence[Sequence[Point]] = (),
            p: Optional[StripParams] = None) -> StripShape:
    p = p or StripParams()
    length, width, axis = min_area_rect(outer)
    r = inscribed_radius(outer, holes, stop_above=p.max_inscribed_r_m)
    return StripShape(r, length, width, math.degrees(axis))


def detect(shape: StripShape, p: Optional[StripParams] = None) -> Tuple[bool, str]:
    """(narrow strip?, why). A query: it only reports."""
    p = p or StripParams()
    if shape.inscribed_r >= p.max_inscribed_r_m:
        return False, 'inscribed radius %.2f m >= %.3f' % (shape.inscribed_r,
                                                            p.max_inscribed_r_m)
    if shape.length < p.min_aspect * shape.width:
        return False, 'not elongated: %.2f x %.2f m' % (shape.length, shape.width)
    if shape.length < p.min_length_m:
        return False, 'too short for passes: %.2f m < %.1f' % (shape.length, p.min_length_m)
    return True, 'narrow strip: %.2f x %.2f m, inscribed radius %.2f m' % (
        shape.length, shape.width, shape.inscribed_r)


# ------------------------------------------------------------------ the planner

class _Geom:
    """The area in the frame of one candidate axis (x along it, entry at low x)."""

    def __init__(self, outer, holes, ang, p: StripParams):
        self.p, self.ang = p, ang
        self.ring = _rot(_ring(outer), ang)
        self.segs = _edges(self.ring)
        self.holes = [_rot(_ring(h), ang) for h in holes if len(h) >= 3]
        self.hole_segs = np.vstack([_edges(h) for h in self.holes]) if self.holes \
            else np.zeros((0, 4))
        self.body = [('base', 0.0, 0.0), ('nose', p.front_m, 0.0), ('tail', -p.rear_m, 0.0),
                     ('front', p.front_m, p.half_width_m), ('front', p.front_m, -p.half_width_m),
                     ('side', -p.rear_m, p.half_width_m), ('side', -p.rear_m, -p.half_width_m)]
        # raster cells of the area (minus holes)
        lo, hi = self.ring.min(axis=0), self.ring.max(axis=0)
        st = p.raster_m
        gx, gy = np.meshgrid(np.arange(lo[0] + st / 2, hi[0], st),
                             np.arange(lo[1] + st / 2, hi[1], st))
        cx, cy = gx.ravel(), gy.ravel()
        ins = _pip(cx, cy, self.ring)
        for h in self.holes:
            ins &= ~_pip(cx, cy, h)
        self.cells = np.stack([cx[ins], cy[ins]], axis=1)
        self.cell_m2 = st * st

    def valid(self, x, y, yaw):
        """(N,) body checks of the poses (x, y, yaw) in this frame."""
        p = self.p
        x, y, yaw = (np.atleast_1d(np.asarray(v, dtype=float)) for v in (x, y, yaw))
        ok = ~_rect_hits(x, y, yaw, self.hole_segs, -p.rear_m, p.front_m, p.half_width_m)
        c, s = np.cos(yaw), np.sin(yaw)
        for kind, bx, by in self.body:
            px, py = x + c * bx - s * by, y + s * bx + c * by
            ok &= _outside(px, py, self.ring, self.segs) <= p.out_limits[kind] + 1e-9
        return ok

    def blade_reaches(self, x, y):
        """(N,) the blade disc (straight pose along +x) still reaches the area."""
        bx = x + self.p.blade_x_m
        return _outside(bx, y, self.ring, self.segs) < self.p.blade_r_m

    def capsule(self, xa, xb, y):
        """(K,) cells within the blade radius of the blade centre line of a straight pass of
        base_link from xa to xb on line y."""
        bx = self.p.blade_x_m
        cx, cy = self.cells[:, 0], self.cells[:, 1]
        dx = np.clip(cx, xa + bx, xb + bx) - cx
        return dx * dx + (cy - y) ** 2 <= self.p.blade_r_m ** 2

    def entry_width(self):
        """Mean width of the area over ``entry_band_m`` from its entry (low x) extreme: the
        room the robot has where it enters and leaves the strip."""
        a, b = self.p.entry_band_m
        x0 = float(self.ring[:, 0].min())
        n = np.count_nonzero((self.cells[:, 0] >= x0 + a) & (self.cells[:, 0] < x0 + b))
        return n * self.cell_m2 / (b - a)

    def cut_cells(self, pts):
        """(K,) cells under the blade disc along the base_link polyline ``pts``."""
        p = self.p
        hit = np.zeros(len(self.cells), dtype=bool)
        cx, cy = self.cells[:, 0], self.cells[:, 1]
        for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
            h = math.atan2(y1 - y0, x1 - x0)
            ax, ay = x0 + p.blade_x_m * math.cos(h), y0 + p.blade_x_m * math.sin(h)
            bx, by = x1 + p.blade_x_m * math.cos(h), y1 + p.blade_x_m * math.sin(h)
            dx, dy = bx - ax, by - ay
            L2 = max(dx * dx + dy * dy, 1e-12)
            t = np.clip(((cx - ax) * dx + (cy - ay) * dy) / L2, 0.0, 1.0)
            hit |= (ax + t * dx - cx) ** 2 + (ay + t * dy - cy) ** 2 <= p.blade_r_m ** 2
        return hit


def _longest_run(mask: np.ndarray) -> Optional[Tuple[int, int]]:
    best, start = None, None
    for i, v in enumerate(list(mask) + [False]):
        if v and start is None:
            start = i
        elif not v and start is not None:
            if best is None or i - start > best[1] - best[0] + 1:
                best = (start, i - 1)
            start = None
    return best


def _candidate_passes(g: _Geom):
    """[(y, xa, xb)]: per line ``offset_step`` apart, the longest run of valid straight poses
    along +x, trimmed to where the blade reaches the area; only lines whose run starts within
    ``entry_tol_m`` of the earliest start (the entry end)."""
    p = g.p
    lo, hi = g.ring.min(axis=0), g.ring.max(axis=0)
    xs = np.arange(lo[0] - p.front_m, hi[0] + p.rear_m + 1e-9, p.station_step_m)
    ys = np.arange(lo[1], hi[1] + 1e-9, p.offset_step_m)
    X, Y = np.meshgrid(xs, ys)
    ok = g.valid(X.ravel(), Y.ravel(), np.zeros(X.size)).reshape(X.shape)
    reach = g.blade_reaches(X.ravel(), Y.ravel()).reshape(X.shape)
    out = []
    for j, y in enumerate(ys):
        run = _longest_run(ok[j])
        if run is None:
            continue
        a, b = run
        while a <= b and not reach[j, a]:
            a += 1
        while b >= a and not reach[j, b]:
            b -= 1
        if b <= a or xs[b] - xs[a] < p.min_pass_m:
            continue
        out.append((float(y), float(xs[a]), float(xs[b])))
    if not out:
        return out
    first = min(xa for _y, xa, _xb in out)
    return [c for c in out if c[1] <= first + p.entry_tol_m]


def _pick_passes(g: _Geom, cands):
    """Greedy by new cut area; returned sorted by offset."""
    p = g.p
    if not cands:
        return []
    masks = np.stack([g.capsule(xa, xb, y) for y, xa, xb in cands])
    covered = np.zeros(len(g.cells), dtype=bool)
    picked = []
    while True:
        gain = np.count_nonzero(masks & ~covered, axis=1) * g.cell_m2
        k = int(np.argmax(gain))
        if gain[k] < p.min_gain_m2:
            break
        picked.append(cands[k])
        covered |= masks[k]
    return sorted(picked)


def _lane_change(x0, y0, y1, L, step):
    """Cosine S from (x0, y0) to (x0 + L, y1), poses (x, y, yaw) every ``step``."""
    n = max(2, int(math.ceil(L / step)))
    t = np.linspace(0.0, 1.0, n + 1)
    x = x0 + L * t
    y = y0 + (y1 - y0) * 0.5 * (1.0 - np.cos(math.pi * t))
    yaw = np.arctan2((y1 - y0) * 0.5 * math.pi * np.sin(math.pi * t), L)
    return x, y, yaw


def _straight_ok(g: _Geom, y, xa, xb):
    if xb < xa - 1e-9:
        return False
    n = max(2, int(math.ceil((xb - xa) / g.p.station_step_m)) + 1)
    xs = np.linspace(xa, xb, n)
    return bool(g.valid(xs, np.full(n, y), np.zeros(n)).all())


def _transition(g: _Geom, cur, nxt, lengths=None):
    """Lane change from line ``cur`` (y, x_start) onto pass ``nxt`` (y, xa, xb): returns
    (x where the S starts, L) or None. ``lengths``: the S lengths to try (lane_change_m)."""
    p = g.p
    y0, xs0, xb0 = cur
    y1, xa1, xb1 = nxt
    for delay in p.lane_change_delay_m:
        for L in (lengths or p.lane_change_m):
            sx = max(xs0, xa1 - L) + delay
            if sx + L > xb1 - p.min_pass_m or sx > xb0:
                continue
            if not _straight_ok(g, y0, xs0, sx):
                continue
            x, y, yaw = _lane_change(sx, y0, y1, L, p.lane_change_step_m)
            if g.valid(x, y, yaw).all() and _straight_ok(g, y1, sx + L, xb1):
                return sx, L
    return None


def _shuttle(g: _Geom, passes, start_left: bool):
    """Chain (points, reverse flags, straight parts, skipped, ends where it began) of the
    shuttle over ``passes`` (sorted by offset), starting at the left (high y) or the right
    long side. After the last pass a long lane change returns to the first pass's line and
    a reverse leg along it ends the chain at its start, where possible."""
    order = list(reversed(passes)) if start_left else list(passes)
    pts: List[Tuple[float, float]] = []
    flags: List[bool] = []
    straight = []
    skipped = 0
    cur = None              # (y, x where the line was last left, xb of that line)

    def add(xy, rev):
        if pts and math.hypot(xy[0] - pts[-1][0], xy[1] - pts[-1][1]) < 1e-6:
            return
        pts.append((float(xy[0]), float(xy[1])))
        flags.append(rev)

    for y, xa, xb in order:
        if cur is None:
            add((xa, y), False)
            add((xb, y), False)
            add((xa, y), True)
            straight.append(((xa, y), (xb, y)))
            cur = (y, xa, xb)
            continue
        tr = _transition(g, cur, (y, xa, xb))
        if tr is None:
            skipped += 1
            continue
        sx, L = tr
        add((sx, cur[0]), False)
        x, yy, _ = _lane_change(sx, cur[0], y, L, g.p.lane_change_step_m)
        for q in zip(x[1:], yy[1:]):
            add(q, False)
        add((xb, y), False)
        back = min(xa, sx + L)          # retrace, on along the line to its own start
        add((back, y), True)
        straight.append(((back, y), (xb, y)))
        cur = (y, back, xb)
    home = len(straight) <= 1
    if not home:
        # back onto the first pass's line, forward, then in reverse along it to its start
        y0, xa0, xb0 = order[0]
        tr = _transition(g, cur, (y0, xa0, xb0), g.p.return_lane_change_m)
        if tr is not None:
            sx, L = tr
            add((sx, cur[0]), False)
            x, yy, _ = _lane_change(sx, cur[0], y0, L, g.p.lane_change_step_m)
            for q in zip(x[1:], yy[1:]):
                add(q, False)
            add((xa0, y0), True)
            home = True
    return pts, flags, straight, skipped, home


def plan_strip(outer: Sequence[Point], holes: Sequence[Sequence[Point]] = (),
               p: Optional[StripParams] = None,
               extra_axes_deg: Sequence[float] = ()) -> StripPlan:
    """The best shuttle plan over the candidate axes (the minimum-area rectangle's long side,
    the directions of the two long sides, ``extra_axes_deg`` within 20 deg of the axis), each
    entered from either end. Best = most cut area (to within ``cut_tie``), then the wider entry
    end, then the shortest drive."""
    p = p or StripParams()
    ring = _ring(outer)
    if len(ring) < 3:
        return StripPlan(False, 'outline needs >= 3 points')
    _l, _w, axis = min_area_rect(ring)
    axes = [axis]
    # the long sides' own directions (end vertex to end vertex of each chain)
    r = _rot(ring, axis)
    i_lo, i_hi = int(np.argmin(r[:, 0])), int(np.argmax(r[:, 0]))
    for side in (1, -1):
        seq = []
        i = i_lo
        while True:
            seq.append(i)
            if i == i_hi:
                break
            i = (i + side) % len(ring)
        # the chain's own end points, past the end edges: the middle 80 % of it
        lo_k, hi_k = int(0.1 * (len(seq) - 1)), int(math.ceil(0.9 * (len(seq) - 1)))
        a, b = ring[seq[lo_k]], ring[seq[hi_k]]
        if np.hypot(*(b - a)) > 0.5:
            axes.append(math.atan2(b[1] - a[1], b[0] - a[0]))
    for d in extra_axes_deg:
        a = math.radians(float(d))
        if abs(math.sin(a - axis)) < math.sin(math.radians(20.0)):
            axes.append(a)
    area = float(len(_Geom(outer, holes, axis, p).cells)) * p.raster_m ** 2
    best = None
    tried = set()
    for a0 in axes:
        for ang in (a0, a0 + math.pi):
            key = round(math.fmod(ang + 4 * math.pi, 2 * math.pi), 3)
            if key in tried:
                continue
            tried.add(key)
            g = _Geom(outer, holes, ang, p)
            passes = _pick_passes(g, _candidate_passes(g))
            if not passes:
                continue
            for start_left in (True, False):
                pts, flags, straight, skipped, home = _shuttle(g, passes, start_left)
                if len(pts) < 3:
                    continue
                cut = float(np.count_nonzero(g.cut_cells(pts))) * g.cell_m2
                length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                             for a, b in zip(pts[:-1], pts[1:]))
                # ending where it began first, then cut (to within cut_tie), then the wider
                # entry end, then the shorter drive
                score = (home, round(cut / area / p.cut_tie) if area > 0 else 0,
                         round(g.entry_width(), 2), -length)
                if best is None or score > best[0]:
                    best = (score, ang, pts, flags, straight, skipped, cut)
    if best is None:
        return StripPlan(False, 'no pass of at least %.1f m fits inside the body limits '
                                '(%.2f x %.2f m)' % (p.min_pass_m, _l, _w))
    _score, ang, pts, flags, straight, skipped, cut = best
    world = _unrot(np.asarray(pts), ang)
    sw = [tuple(map(tuple, _unrot(np.asarray(s), ang))) for s in straight]
    frac = cut / area if area > 0 else 0.0
    return StripPlan(True, STRIP_TAG + ' %d pass(es) at %.1f deg, %d skipped, cut %.0f %% of %.2f m2'
                     % (len(straight), math.degrees(ang) % 360.0, skipped, 100.0 * frac, area),
                     math.degrees(ang) % 360.0,
                     [(tuple(map(float, a)), tuple(map(float, b))) for a, b in sw],
                     [(float(x), float(y)) for x, y in world], list(flags), frac, skipped)
