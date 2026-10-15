# Copyright 2026 Airseekers Tron ROS 2 port contributors
# SPDX-License-Identifier: GPL-3.0-or-later
"""Path conditioner: one pure-geometry pass that turns a path into one the robot can drive
(docs/DESIGN_path_conditioning_2026-10.md, steps 2-3). No ROS imports.

Input: a polyline (coverage sub-path, transit route or dock route) and the allowed region
(mowing areas U drawn path channels, as polygons). Output: poses resampled at ``step_m``,
each with a segment type (``line`` / ``arc`` / ``pivot``) and a curvature, plus the pose
indices where the robot must stop and turn in place.

1. Corners: wherever the path turns faster than an arc of ``r_min`` can (heading change over
   +-``detect_m`` larger than ``detect_factor`` x what an r_min arc turns there), the bends
   within ``merge_m`` of each other (a corner drawn in two clicks) are one corner, measured
   between the straight runs either side (Douglas-Peucker at ``simplify_m``, so drawing
   jitter does not count). Planned arcs of r >= r_min (Dubins / Reeds-Shepp swath turns) are
   no corners and stay as they are.
2. Per corner, from the space actually available: a fillet arc tangent to both runs, the
   largest r in [r_min, r_max] (0.1 m steps) whose tangent length fits both runs (half the
   run to a neighbouring corner) and whose every sample stays inside the region with a
   clearance of at least min(margin + half_width, the clearance the drawn path itself had
   there) - a fillet never takes the robot closer to the edge than the drawn path, nor under
   margin + half width where the drawing kept that much. Else a pivot at the vertex (the
   robot stays on the drawn line, stops and turns in place) when the corner turns at least
   ``pivot_min_rad``; a smaller unfittable bend is left as drawn (``kink``).
3. Outermost-ring edge policy: on a ring-like path (it turns through >= 180 deg) a convex
   corner (turning with the ring) of at least ``edge_pivot_rad`` whose vertex lies within
   ``edge_clearance_m`` of the region edge is a pivot, not a fillet (the fillet leaves an
   uncut patch r^2 (tan(theta/2) - theta/2) at the edge).
4. Resample at ``step_m``; type and curvature per pose (raw stretches: ``arc`` where the
   local radius is <= 2 m, else ``line``; a pivot pose carries the incoming heading).
5. Validate: every fillet pose keeps the clearance its construction required (min(margin +
   half_width, drawn clearance) less at most ``slack_m``, never under ``margin_m`` unless
   the drawing is), every other pose is inside the region (or no farther out than the drawn
   poses either side), every fillet r >= r_min. Errors are returned, never silently driven.

K-turns (Reeds-Shepp with reverse, where a pivot is not allowed) are not generated yet
(design Phase 4, slope-dependent): ``allow_pivot=False`` reports the corner as an error.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional

from . import geometry as geo

LINE, ARC, PIVOT = 'line', 'arc', 'pivot'


@dataclass
class Corner:
    index: int                   # raw pose index of the vertex
    x: float
    y: float
    turn: float                  # signed heading change (rad, left > 0)
    choice: str                  # fillet | pivot | kink
    r: float = 0.0               # fillet radius
    cut_m: float = 0.0           # how far the fillet cuts inside the vertex
    why: str = ''


@dataclass
class Conditioned:
    poses: list = field(default_factory=list)       # (x, y, yaw)
    types: list = field(default_factory=list)       # LINE | ARC | PIVOT per pose
    curvature: list = field(default_factory=list)   # 1/m, left > 0
    pivots: list = field(default_factory=list)      # pose indices to stop and turn at
    corners: list = field(default_factory=list)     # [Corner]
    errors: list = field(default_factory=list)
    _floors: list = field(default_factory=list, repr=False)

    @property
    def valid(self):
        return not self.errors


@dataclass
class Params:
    r_min: float = 0.5
    r_max: float = 1.0
    r_step: float = 0.1
    margin_m: float = 0.05
    half_width_m: float = 0.24
    step_m: float = 0.05
    simplify_m: float = 0.02
    detect_m: float = 0.06
    detect_factor: float = 1.5
    merge_m: float = 0.3
    run_m: float = 0.3             # length of the straight run measured either side
    pivot_min_rad: float = 0.35    # an unfittable bend under this stays as drawn
    edge_pivot_rad: float = 1.38   # outermost ring: convex corners >= this pivot (0 = off)
    edge_clearance_m: float = 0.20
    slack_m: float = 0.05          # a fillet may come this much closer to the edge than the
    #                                drawn path (never under margin_m unless the drawing is)
    cusp_rad: float = 2.6          # sharper turns within end_m of an end are turn cusps: kept
    end_m: float = 0.3
    allow_pivot: bool = True


# ---------------------------------------------------------------- region / clearance

def _pip(x, y, poly):
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i][0], poly[i][1]
        xj, yj = poly[j][0], poly[j][1]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def _edge_dist(x, y, poly):
    best = math.inf
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i - 1][0], poly[i - 1][1]
        bx, by = poly[i][0], poly[i][1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        best = min(best, math.hypot(ax + t * dx - x, ay + t * dy - y))
    return best


def clearance(pt, region):
    """Distance of ``pt`` inside the region (> 0) or outside it (< 0). Inside several
    overlapping polygons the largest single-polygon clearance counts (a conservative
    estimate of the union's). An empty region: +inf (no check)."""
    polys = [p for p in region if len(p) >= 3]
    if not polys:
        return math.inf
    x, y = float(pt[0]), float(pt[1])
    inside = [_edge_dist(x, y, p) for p in polys if _pip(x, y, p)]
    if inside:
        return max(inside)
    return -min(_edge_dist(x, y, p) for p in polys)


# ---------------------------------------------------------------- helpers

def _unit(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy)
    return (dx / L, dy / L) if L > 1e-9 else None


def _resample(pts, step):
    """Polyline -> points every ``step`` (first and last kept)."""
    out = [tuple(pts[0][:2])]
    carry = 0.0
    for a, b in zip(pts, pts[1:]):
        seg = geo.dist(a, b)
        if seg < 1e-9:
            continue
        s = step - carry
        while s < seg - 1e-9:
            f = s / seg
            out.append((a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f))
            s += step
        carry = seg - (s - step)
    if geo.dist(out[-1], pts[-1]) > 1e-6:
        if geo.dist(out[-1], pts[-1]) < 0.3 * step and len(out) > 1:
            out[-1] = tuple(pts[-1][:2])
        else:
            out.append(tuple(pts[-1][:2]))
    return out


def _turn_rate(pts, cum, i, w):
    """Heading change across raw pose ``i`` between the chords [s-w, s] and [s, s+w]."""
    s = cum[i]
    a = geo._point_at(pts, cum, max(0.0, s - w))
    b = geo._point_at(pts, cum, min(cum[-1], s + w))
    p = pts[i]
    if geo.dist(a, p) < 0.5 * w or geo.dist(p, b) < 0.5 * w:
        return 0.0
    h1 = math.atan2(p[1] - a[1], p[0] - a[0])
    h2 = math.atan2(b[1] - p[1], b[0] - p[0])
    return geo.wrap_angle(h2 - h1)


def _run_dir(pts, cum, s0, s1):
    a = geo._point_at(pts, cum, max(0.0, min(cum[-1], s0)))
    b = geo._point_at(pts, cum, max(0.0, min(cum[-1], s1)))
    return _unit(a, b)


def _intersect(p, u, q, v):
    """Intersection of the lines p + a u and q + b v (None when near parallel)."""
    den = u[0] * v[1] - u[1] * v[0]
    if abs(den) < 1e-6:
        return None
    a = ((q[0] - p[0]) * v[1] - (q[1] - p[1]) * v[0]) / den
    return (p[0] + a * u[0], p[1] + a * u[1])


def _arc(T1, u_in, turn, r, step):
    sgn = 1.0 if turn > 0 else -1.0
    n = (-u_in[1] * sgn, u_in[0] * sgn)
    C = (T1[0] + r * n[0], T1[1] + r * n[1])
    a0 = math.atan2(T1[1] - C[1], T1[0] - C[0])
    L = abs(turn) * r
    k = max(2, int(math.ceil(L / step)))
    return [(C[0] + r * math.cos(a0 + turn * j / k), C[1] + r * math.sin(a0 + turn * j / k))
            for j in range(k + 1)]


# ---------------------------------------------------------------- corners

def find_corners(pts, p):
    """[(vertex point, signed turn, s_lo, s_hi, raw index)] of the sharp corners."""
    n = len(pts)
    if n < 3:
        return []
    cum = geo.cumulative_lengths(pts)
    lim = p.detect_factor * 2.0 * p.detect_m / p.r_min
    hot = [i for i in range(1, n - 1) if abs(_turn_rate(pts, cum, i, p.detect_m)) > lim]
    groups = []
    for i in hot:
        if groups and cum[i] - cum[groups[-1][-1]] <= p.merge_m and \
                _turn_rate(pts, cum, i, p.detect_m) * _turn_rate(
                    pts, cum, groups[-1][-1], p.detect_m) > 0.0:
            groups[-1].append(i)
        else:
            groups.append([i])
    # straight runs either side, measured on the Douglas-Peucker simplification
    out = []
    for g in groups:
        s_lo, s_hi = cum[g[0]], cum[g[-1]]
        u_in = _run_dir(pts, cum, s_lo - p.run_m, s_lo)
        u_out = _run_dir(pts, cum, s_hi, s_hi + p.run_m)
        if u_in is None or u_out is None:
            continue
        turn = geo.wrap_angle(math.atan2(u_out[1], u_out[0]) - math.atan2(u_in[1], u_in[0]))
        if len(g) == 1:
            v, vi = tuple(pts[g[0]][:2]), g[0]
        else:
            vi = max(g, key=lambda i: abs(_turn_rate(pts, cum, i, p.detect_m)))
            v = _intersect(pts[g[0]], u_in, pts[g[-1]], u_out) or tuple(pts[vi][:2])
        out.append((v, turn, s_lo, s_hi, vi, u_in, u_out))
    return out


def condition(poses, region, params: Optional[Params] = None, ring=None):
    """Condition the polyline ``poses`` ((x, y[, yaw]) ...) against ``region`` (list of
    polygons). ``ring``: apply the outermost-ring edge policy (None = auto: the path turns
    through >= 180 deg in total). Returns :class:`Conditioned`."""
    p = params or Params()
    raw = [tuple(map(float, q[:2])) for q in poses]
    # drop repeated points
    pts = [raw[0]] if raw else []
    for q in raw[1:]:
        if geo.dist(q, pts[-1]) > 1e-6:
            pts.append(q)
    res = Conditioned()
    if len(pts) < 2:
        res.errors.append('path shorter than 2 poses')
        return res
    cum = geo.cumulative_lengths(pts)
    req = p.margin_m + p.half_width_m
    raw_clr = [clearance(q, region) for q in pts]

    def raw_ref(s0, s1):
        vals = [c for c, s in zip(raw_clr, cum) if s0 - 1e-9 <= s <= s1 + 1e-9]
        return min(vals) if vals else math.inf

    corners = find_corners(pts, p)
    total_turn = sum(_turn_rate(pts, cum, i, 0.5 * p.step_m + 1e-3) for i in range(1, len(pts) - 1))
    if ring is None:
        ring = abs(total_turn) >= math.pi
    ring_sign = 1.0 if total_turn > 0 else -1.0
    pieces = []          # ('raw', s0, s1) | ('arc', pts, r, turn) | ('pivot', vertex)
    s_prev = 0.0
    for k, (v, turn, s_lo, s_hi, vi, u_in, u_out) in enumerate(corners):
        # runs available for the tangents: back to where the previous corner's piece ended,
        # forward to the next corner minus the least that one needs for its own fillet
        # (nothing when it cannot fillet anyway)
        lim_in = max(0.0, s_lo - s_prev)
        if k + 1 < len(corners):
            run = corners[k + 1][2] - s_hi
            need = p.r_min * math.tan(min(abs(corners[k + 1][1]), math.radians(170)) / 2.0)
            lim_out = run - need if need <= 0.5 * run else run
        else:
            lim_out = cum[-1] - s_hi
        a_in = geo._point_at(pts, cum, s_lo)
        b_out = geo._point_at(pts, cum, s_hi)
        avail_in = lim_in + geo.dist(a_in, v)
        avail_out = lim_out + geo.dist(b_out, v)
        c = Corner(vi, v[0], v[1], turn, 'kink')
        if abs(turn) >= p.cusp_rad and (s_lo <= p.end_m or cum[-1] - s_hi <= p.end_m):
            c.why = 'turn cusp at the path end: kept'
            res.corners.append(c)
            continue
        edge = (p.edge_pivot_rad > 0.0 and ring and turn * ring_sign > 0.0 and
                abs(turn) >= p.edge_pivot_rad - math.radians(1.0) and
                clearance(v, region) <= p.edge_clearance_m)
        chosen = None
        if abs(turn) < math.radians(170.0) and not edge:
            r = p.r_max
            while r >= p.r_min - 1e-9:
                t = r * math.tan(abs(turn) / 2.0)
                if t <= avail_in + 1e-9 and t <= avail_out + 1e-9:
                    T1 = (v[0] - t * u_in[0], v[1] - t * u_in[1])
                    T2 = (v[0] + t * u_out[0], v[1] + t * u_out[1])
                    s1 = geo.path_projection(pts, T1)[0]
                    s2 = geo.path_projection(pts, T2)[0]
                    if s1 >= s_prev - 1e-6 and s2 > s1:
                        arc = _arc(T1, u_in, turn, r, p.step_m)
                        floor = _floor(raw_ref(s1, s2), req, p)
                        if all(clearance(q, region) >= floor - 1e-3 for q in arc):
                            chosen = (s1, s2, arc, r, floor)
                            break
                r = round(r - p.r_step, 6)
        if chosen is not None:
            s1, s2, arc, r, floor = chosen
            c.choice, c.r = 'fillet', r
            c.cut_m = r * (1.0 / math.cos(abs(turn) / 2.0) - 1.0)
            pieces.append(('raw', s_prev, s1))
            pieces.append(('arc', arc, r, turn, floor))
            s_prev = s2
        elif abs(turn) >= p.pivot_min_rad:
            if not p.allow_pivot:
                res.errors.append('corner at (%.2f, %.2f) %.0f deg: no fillet fits and pivots '
                                  'are not allowed (K-turn not implemented)'
                                  % (v[0], v[1], math.degrees(turn)))
            c.choice = 'pivot'
            c.why = 'edge policy' if edge else 'no fillet fits'
            if geo.dist(v, pts[vi]) > 0.02 and \
                    clearance(v, region) < min(req, raw_ref(s_lo, s_hi)) - 1e-3:
                # a merged corner pivots at the runs' intersection only when that is no
                # closer to the edge than the drawn path, else at the drawn vertex
                v = tuple(pts[vi])
                c.x, c.y = v
            pieces.append(('raw', s_prev, s_lo))
            pieces.append(('pivot', v))
            s_prev = s_hi
        res.corners.append(c)
    pieces.append(('raw', s_prev, cum[-1]))
    _assemble(res, pts, cum, pieces, p)
    _validate(res, pts, raw_clr, region, req, p)
    return res


def _floor(raw_ref, req, p):
    """Least clearance a conditioned pose may have where the drawn path had ``raw_ref``."""
    return min(raw_ref, max(p.margin_m, min(req, raw_ref) - p.slack_m))


def _raw_slice(pts, cum, s0, s1):
    if s1 <= s0 + 1e-9:
        return [geo._point_at(pts, cum, s0)]
    out = [geo._point_at(pts, cum, s0)]
    for q, s in zip(pts, cum):
        if s0 + 1e-9 < s < s1 - 1e-9:
            out.append(q)
    out.append(geo._point_at(pts, cum, s1))
    return out


def _assemble(res, pts, cum, pieces, p):
    xy, typ, curv, floors = [], [], [], []

    def add(q, t, k, fl=None):
        if xy and geo.dist(xy[-1], q) < 1e-3:
            if t == PIVOT:
                typ[-1], curv[-1] = PIVOT, 0.0
            return
        xy.append((float(q[0]), float(q[1])))
        typ.append(t)
        curv.append(k)
        floors.append(fl)

    for pc in pieces:
        if pc[0] == 'raw':
            sl = _raw_slice(pts, cum, pc[1], pc[2])
            if xy and geo.dist(xy[-1], sl[0]) > 1e-3:
                sl = [xy[-1]] + sl         # bridge from a pivot vertex / an arc end
            seg = _resample(sl, p.step_m)
            for q in seg:
                add(q, LINE, 0.0)
        elif pc[0] == 'arc':
            arc, r, turn, fl = pc[1], pc[2], pc[3], pc[4]
            k = (1.0 if turn > 0 else -1.0) / r
            for j, q in enumerate(arc):
                mid = 0 < j < len(arc) - 1
                add(q, ARC if mid else LINE, k if mid else 0.0, fl)
        else:
            add(pc[1], PIVOT, 0.0)
    # yaw: direction of travel (a pivot pose keeps the incoming heading); raw stretches get
    # their measured curvature (planned Dubins / Reeds-Shepp arcs stay arcs)
    n = len(xy)
    poses = []
    for i in range(n):
        if i + 1 < n and typ[i] != PIVOT:
            yaw = math.atan2(xy[i + 1][1] - xy[i][1], xy[i + 1][0] - xy[i][0])
        elif i > 0:
            yaw = math.atan2(xy[i][1] - xy[i - 1][1], xy[i][0] - xy[i - 1][0])
        else:
            yaw = 0.0
        poses.append((xy[i][0], xy[i][1], yaw))
    ccum = geo.cumulative_lengths(xy)
    w = 0.15
    for i in range(1, n - 1):
        if typ[i] != LINE or ccum[i] < w or ccum[-1] - ccum[i] < w:
            continue
        lo = next(j for j in range(i, -1, -1) if ccum[i] - ccum[j] >= w or j == 0)
        hi = next(j for j in range(i, n) if ccum[j] - ccum[i] >= w or j == n - 1)
        if any(t == PIVOT for t in typ[lo:hi + 1]):
            continue
        h1 = math.atan2(xy[i][1] - xy[lo][1], xy[i][0] - xy[lo][0])
        h2 = math.atan2(xy[hi][1] - xy[i][1], xy[hi][0] - xy[i][0])
        k = geo.wrap_angle(h2 - h1) / max(1e-6, 0.5 * (ccum[hi] - ccum[lo]))
        if abs(k) >= 0.5:                       # r <= 2 m: a planned arc (Dubins / RS)
            typ[i], curv[i] = ARC, k
    res.poses, res.types, res.curvature = poses, typ, curv
    res.pivots = [i for i, t in enumerate(typ) if t == PIVOT]
    res._floors = floors


def _validate(res, pts, raw_clr, region, req, p):
    if not region:
        return
    j = 0
    for i, q in enumerate(res.poses):
        j = geo.nearest_index(pts, q, max(0, j - 10), 3.0)
        fl = res._floors[i] if i < len(res._floors) else None
        if fl is None:
            # a pose on the drawn path itself (a raw stretch, a pivot vertex): it may not be
            # outside the region by more than the drawn poses either side of it are
            ref = min([0.0] + raw_clr[max(0, j - 1):j + 2])
        else:
            ref = fl
        if clearance(q, region) < ref - 0.01:
            res.errors.append('pose %d (%.2f, %.2f) closer to the edge than allowed '
                              '(%.3f < %.3f m)' % (i, q[0], q[1], clearance(q, region), ref))
            break
    for c in res.corners:
        if c.choice == 'fillet' and c.r < p.r_min - 1e-9:
            res.errors.append('fillet r %.2f < r_min at (%.2f, %.2f)' % (c.r, c.x, c.y))
