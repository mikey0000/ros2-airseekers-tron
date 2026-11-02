# Copyright 2026 Airseekers Tron ROS 2 port contributors
# SPDX-License-Identifier: GPL-3.0-or-later
"""Body-aware boundary-out check (step 3 round 2, 2026-10-21). Pure Python, no ROS.

Stock (``libmower_controller_check.so`` ``CheckRegion``, see
docs/analysis/2026-10-21_stock_firmware_findings.md) samples one line across the body 0.20 m
ahead of the axle and trips when >= 70 % of it is outside: with stock's half width 0.225 that
is the outer side about 0.315 m out. Our port checks six body points (the 4 footprint corners,
the mid-front "nose" and the mid-rear "tail", base_link = rear axle) against the allowed
region (mowing areas U drawn paths U dock corridor) and trips when any point is further out
than its limit:

* hard edge_mode: every point 0.10 (the body stays inside plus 0.10);
* soft edge_mode (stock layout: lap 0 has base_link ON the line): rear corners 0.35, front
  corners 0.55 (convex-bend allowance, see limits_for), nose 0.40 (a driven R 0.44 arc puts
  it sqrt(R^2 + 0.6^2) - R = 0.30 out; row ends 0.15), tail 0.30. Normal soft driving stays below: lap 0 puts the sides 0.235 out, a row end
  0.45 inside puts the nose 0.15 / the front corners <= 0.195 past (pivot sweep included);
  tracking error p99 0.07 (pivots) .. 0.13 (all samples, sim 2026-10-21) on top.
"""

import math

from mower_common import geometry as rgeo

# body frame (x forward, y left), metres from base_link (owner-measured 2026-10-20)
FRONT, REAR, HALF_WIDTH = (rgeo.FOOTPRINT_FRONT_M, rgeo.FOOTPRINT_REAR_M,
                           rgeo.FOOTPRINT_HALF_WIDTH_M)   # 0.60, 0.10, 0.235


def body_points(front=FRONT, rear=REAR, half=HALF_WIDTH):
    """name -> (bx, by, kind) with kind 'front' (front corners), 'side' (rear corners),
    'nose' or 'tail'."""
    # nose / tail first: an equal excess (square to the line) reports the nose / tail
    return {
        'nose': (front, 0.0, 'nose'), 'tail': (-rear, 0.0, 'tail'),
        'front_left': (front, half, 'front'), 'front_right': (front, -half, 'front'),
        'rear_left': (-rear, half, 'side'), 'rear_right': (-rear, -half, 'side'),
    }


def _pip(x, y, poly):
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i][0], poly[i][1]
        xj, yj = poly[j][0], poly[j][1]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def _edge_dist(x, y, poly):
    best = math.inf
    for i in range(len(poly)):
        ax, ay = poly[i - 1][0], poly[i - 1][1]
        bx, by = poly[i][0], poly[i][1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        best = min(best, math.hypot(ax + t * dx - x, ay + t * dy - y))
    return best


def nearest_edge_point(x, y, polys):
    """(px, py, d): the closest point on any polygon edge to (x, y); None without polygons."""
    best = None
    for poly in polys:
        if len(poly) < 3:
            continue
        for i in range(len(poly)):
            ax, ay = poly[i - 1][0], poly[i - 1][1]
            bx, by = poly[i][0], poly[i][1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
            px, py = ax + t * dx, ay + t * dy
            d = math.hypot(px - x, py - y)
            if best is None or d < best[2]:
                best = (px, py, d)
    return best


def point_outside(x, y, polys):
    """Distance of (x, y) outside the union of ``polys`` (0 inside; inf without polygons)."""
    best = math.inf
    for p in polys:
        if len(p) < 3:
            continue
        if _pip(x, y, p):
            return 0.0
        best = min(best, _edge_dist(x, y, p))
    return best


def limits_for(mode, soft_side=0.35, soft_nose=0.40, soft_tail=0.30, hard=0.10,
               soft_front=0.55):
    """kind -> limit (m) for edge_mode ``mode`` ('hard' | anything else = soft). Soft front
    corners get ``soft_front``: on a convex bend of radius R tracked by base_link they sit
    sqrt((R + 0.235)^2 + 0.6^2) - R out (owner option (c), 2026-10-22: lap 0 stays on the
    driven line; Area 6's driven arcs R 0.44 -> 0.46, R 0.72 -> 0.41) although nothing
    drifted; 0.55 = the R 0.44 envelope 0.46 + tracking p99 0.07 (+ margin). Stock's check
    line sits near the axle and is blind to that. Lateral drift is
    still caught by the rear corners (0.235 + drift, curvature-insensitive)."""
    if str(mode).strip().lower() == 'hard':
        return {'side': hard, 'front': hard, 'nose': hard, 'tail': hard}
    return {'side': soft_side, 'front': soft_front, 'nose': soft_nose, 'tail': soft_tail}


def evaluate(pose, polys, limits, points=None):
    """Body check at ``pose`` (x, y, yaw). Returns a dict: ``excess`` (max over the points of
    outside - limit; > 0 = trip), ``point`` (the worst point's name), ``out`` (its outside
    distance), ``rear`` (True when the worst point is behind base_link: recover forwards),
    ``outs`` {name: outside m}, ``side_only`` (the excess is a corner's while the nose and
    tail are within their limits: a lateral excess, which a straight move does not fix). None without polygons."""
    if not any(len(p) >= 3 for p in polys):
        return None
    pts = points or body_points()
    x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    c, s = math.cos(yaw), math.sin(yaw)
    best, outs = None, {}
    for name, (bx, by, kind) in pts.items():
        o = point_outside(x + c * bx - s * by, y + s * bx + c * by, polys)
        if not math.isfinite(o):
            return None
        outs[name] = o
        e = o - float(limits[kind])
        if best is None or e > best[0]:
            best = (e, name, o, bx < 0.0)
    side_only = pts[best[1]][2] in ('side', 'front') and all(
        outs[n] <= float(limits[k]) for n, (_x, _y, k) in pts.items()
        if k in ('nose', 'tail'))
    return {'excess': best[0], 'point': best[1], 'out': best[2], 'rear': best[3], 'outs': outs,
            'side_only': side_only}


def worst_outside(pose, polys, points=None, keepouts=()):
    """(name, m): the body point furthest outside the allowed ``polys`` (or furthest inside
    a keepout of ``keepouts``) at ``pose``; the raw worst-corner metric the recovery logs
    and plans with (no per-kind limits). None without polygons."""
    if not any(len(p) >= 3 for p in polys):
        return None
    pts = points or body_points()
    x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    c, s = math.cos(yaw), math.sin(yaw)
    best = None
    for name, (bx, by, _kind) in pts.items():
        px, py = x + c * bx - s * by, y + s * bx + c * by
        o = point_outside(px, py, polys)
        for k in keepouts:
            if len(k) >= 3 and _pip(px, py, k):
                o = max(o, _edge_dist(px, py, k))
        if best is None or o > best[1]:
            best = (name, o)
    return best


# recovery candidates (boundary_recover_footprint_plan, 2026-11-02 field side-slope failure)
CANDIDATE_ORDER = ('reverse_arc_left', 'reverse_arc_right', 'reverse', 'forward_arc_left',
                   'forward_arc_right', 'forward', 'pivot_left', 'pivot_right')


def candidate_motions(v, w_arc, w_pivot, dist_m, pivot_rad):
    """name -> (v, w, duration s): reverse / forward straight, reverse / forward arcs both
    ways (the scoring keeps the one toward the inside) and pivots both ways."""
    v, w_arc, w_pivot = abs(v), abs(w_arc), abs(w_pivot)
    t_lin = dist_m / max(1e-3, v)
    t_piv = pivot_rad / max(1e-3, w_pivot)
    return {
        'reverse_arc_left': (-v, w_arc, t_lin), 'reverse_arc_right': (-v, -w_arc, t_lin),
        'reverse': (-v, 0.0, t_lin),
        'forward_arc_left': (v, w_arc, t_lin), 'forward_arc_right': (v, -w_arc, t_lin),
        'forward': (v, 0.0, t_lin),
        'pivot_left': (0.0, w_pivot, t_piv), 'pivot_right': (0.0, -w_pivot, t_piv),
    }


def simulate(pose, v, w, duration, dt=0.1):
    """Unicycle poses (x, y, yaw) along the motion, the start excluded, the end included."""
    x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    out, t = [], 0.0
    n = max(1, int(math.ceil(duration / dt - 1e-9)))
    h = duration / n
    for _ in range(n):
        yaw += w * h
        x += v * h * math.cos(yaw)
        y += v * h * math.sin(yaw)
        t += h
        out.append((x, y, yaw))
    return out


def _seg_point_dist(x, y, e):
    """Distance from (x, y) to edge ``e`` = (ax, ay, bx, by, dx, dy, L2)."""
    ax, ay, _bx, _by, dx, dy, L2 = e
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
    return math.hypot(ax + t * dx - x, ay + t * dy - y)


class _LocalRegion:
    """``worst_outside`` restricted to the edges near one pose (review fix 3, 2026-11-02:
    plan_recovery on a 1000-vertex region took 0.75 s on amd64, under the node lock).

    Only edges within ``radius`` of ``pose`` are kept (``radius`` is raised to cover
    ``reach``, the furthest any queried body point gets from ``pose``). Inside / outside
    of a query point q is that of a reference point next to the pose (one full
    point-in-polygon test per polygon; nudged off any edge it sits on) flipped once per
    kept edge the segment ref -> q crosses: every such edge lies within reach of the pose.
    A point whose nearest kept edge is further than ``radius - reach`` (or that has none)
    falls back to the full edge scan; a polygon with no kept edge is at least that far
    away. So the result equals ``worst_outside`` (a query point exactly on an edge is 0
    out either way)."""

    def __init__(self, pose, polys, keepouts, points, reach, radius):
        x0, y0 = float(pose[0]), float(pose[1])
        self.points = points
        self.radius = max(float(radius), float(reach) + 0.1)
        self.far = self.radius - float(reach) - 0.01
        polys = [self._near(p, x0, y0) for p in polys if len(p) >= 3]
        keepouts = [self._near(k, x0, y0) for k in keepouts if len(k) >= 3]
        edges = [e for _p, es in polys + keepouts for e in es]
        self.rx, self.ry = x0, y0
        for k in range(1, 8):         # a reference point on no kept edge (parity is exact)
            if all(_seg_point_dist(self.rx, self.ry, e) > 1e-7 for e in edges):
                break
            self.rx, self.ry = x0 + 1e-4 * k * 0.6180339887, y0 + 1e-4 * k * 0.3819660113
        self.polys = [(p, _pip(self.rx, self.ry, p), es) for p, es in polys]
        self.keepouts = [(p, _pip(self.rx, self.ry, p), es) for p, es in keepouts]

    def _near(self, poly, x0, y0):
        edges = []
        for i in range(len(poly)):
            ax, ay = float(poly[i - 1][0]), float(poly[i - 1][1])
            bx, by = float(poly[i][0]), float(poly[i][1])
            e = (ax, ay, bx, by, bx - ax, by - ay, (bx - ax) ** 2 + (by - ay) ** 2)
            if _seg_point_dist(x0, y0, e) <= self.radius:
                edges.append(e)
        return poly, edges

    def _query(self, px, py, prepped):
        """(inside, edge distance over the kept edges) of (px, py) against one polygon."""
        _poly, inside, edges = prepped
        rx, ry = self.rx, self.ry
        ux, uy = px - rx, py - ry
        best = math.inf                  # squared
        for e in edges:
            ax, ay, bx, by, dx, dy, L2 = e
            t = 0.0 if L2 < 1e-12 else ((px - ax) * dx + (py - ay) * dy) / L2
            t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
            ex, ey = ax + t * dx - px, ay + t * dy - py
            d2 = ex * ex + ey * ey
            if d2 < best:
                best = d2
            # the edge crosses ref -> q: its ends on either side of that line (a point on
            # the line counts as the right side, like _pip's half-open rule) and the
            # crossing within the segment
            sa = ux * (ay - ry) - uy * (ax - rx) > 0.0
            sb = ux * (by - ry) - uy * (bx - rx) > 0.0
            if sa != sb:
                s_ = ((ax - rx) * dy - (ay - ry) * dx) / (ux * dy - uy * dx)
                if 0.0 <= s_ <= 1.0:
                    inside = not inside
        return inside, math.sqrt(best)

    def worst(self, pose):
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        c, s = math.cos(yaw), math.sin(yaw)
        best = None
        for name, (bx, by, _kind) in self.points.items():
            px, py = x + c * bx - s * by, y + s * bx + c * by
            o = math.inf
            for pp in self.polys:
                inside, d = self._query(px, py, pp)
                if inside:
                    o = 0.0
                    break
                o = min(o, d)
            if o > self.far:                 # nothing near: the full scan (exact, rare)
                o = point_outside(px, py, [pp[0] for pp in self.polys])
            for kp in self.keepouts:
                inside, d = self._query(px, py, kp)
                if inside:
                    o = max(o, d if d <= self.far else _edge_dist(px, py, kp[0]))
            if best is None or o > best[1]:
                best = (name, o)
        return best


def plan_recovery(pose, polys, motions, max_worsen=0.05, points=None, keepouts=(),
                  exclude=(), reverse_bias=0.02, radius=2.0):
    """Score the candidate ``motions`` (name -> (v, w, duration)) by the worst-corner outside
    distance along the simulated footprint. A candidate is admissible when it never exceeds
    the current worst by more than ``max_worsen`` (a pivot: never exceeds it at all) and ends
    lower than it started. Returns a list of (name, v, w, duration, end_m, peak_m), best
    first (lowest end; reverse moves get ``reverse_bias`` of credit: they back off the edge
    rather than driving along it), or [] when nothing admissible is left. Only region /
    keepout edges within ``radius`` of ``pose`` are scanned (_LocalRegion; same result)."""
    if not any(len(p) >= 3 for p in polys):
        return []
    pts = points or body_points()
    todo = [n for n in CANDIDATE_ORDER if n in motions and n not in exclude]
    travel = max([abs(motions[n][0]) * motions[n][2] for n in todo] or [0.0])
    reach = travel + max(math.hypot(bx, by) for bx, by, _k in pts.values())
    region = _LocalRegion(pose, polys, keepouts, pts, reach, radius)
    cur = region.worst(pose)[1]
    if not math.isfinite(cur):
        return []
    rows = []
    for name in todo:
        v, w, dur = motions[name]
        peak, end = 0.0, cur
        for q in simulate(pose, v, w, dur):
            end = region.worst(q)[1]
            peak = max(peak, end)
        cap = cur + 1e-6 if v == 0.0 else cur + max(0.0, max_worsen)
        if peak > cap or end >= cur - 1e-3:
            continue
        rows.append((name, v, w, dur, end, peak))
    rows.sort(key=lambda r: (r[4] - (reverse_bias if r[1] < 0.0 else 0.0), r[5]))
    return rows
