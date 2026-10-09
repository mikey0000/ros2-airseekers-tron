# SPDX-License-Identifier: GPL-3.0-or-later
"""Route-graph transit planner (2026-10-09, owner: "a specific path for on paths, a
different navigation e.g. follow perimeter / direct to the next path, then path again,
to keep the lines within the areas and paths").

Nav2's grid planners (NavFn, Smac) see the drawn paths only as cheaper cells of the
global costmap and still cut corners: in the sim the transit left "Path 1" at x ~ 6.8
and crossed the lawn. This module plans transits on a graph built from the map instead:

* **Path edges**: every drawn navigation path with a centreline (``Area.channel``) is
  followed exactly along that polyline, both ways. The dock -> approach pose leg (and,
  when the approach pose lies outside every area / path, the automatic approach ->
  nearest area leg, as ``areas.dock_corridor``) is a pseudo path ("dock").
* **Portals**: a path is entered / left only at its ends that lie in an area (or on its
  edge), at junctions with other paths, or, for a path that passes through an area
  without ending in it, where its centreline crosses the area boundary. The robot never
  leaves a path half way along it.
* **In-area legs** ("direct to the next path" / "follow perimeter"): inside one mowing
  area (or a plain navigation polygon) two points are joined by a straight segment when
  it keeps ``clearance_m`` from the area edge and from every obstacle polygon. Otherwise
  the route bends round the inset perimeter: a visibility graph over points
  ``clearance_m`` (a hair more) off every reflex area corner and every convex obstacle
  corner, which is exactly the shortest path inside the area shrunk by the clearance
  (it hugs the inset ring at concave corners and takes straight shortcuts elsewhere).
* **Links**: a portal / start / goal closer than ``clearance_m`` to the edge (a path end
  drawn onto the boundary, a coverage sub-path start on the outer ring) is joined to the
  nearest point that has the clearance by a short link whose every sample lies inside
  the area or a path polygon.

Costs: length x ``path_weight`` (1) on paths, x ``area_weight`` on in-area legs (3: a
drawn path running across the lawn is used unless the lawn hop is far shorter; with
1.5 the sim's dock -> area 1 transit still cut across the lawn, see test_route_graph),
x ``nav_area_weight`` inside navigation polygons without a centreline.

ROS-free, numpy only (like areas.py). map_server_node builds a RouteGraph whenever the
areas / paths / dock change and serves ``~/plan_route``.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from mower_map.areas import (Area, DockPose, capsule_outline, distance_to_polygon_edges,
                             points_in_polygon)

Point = Tuple[float, float]

PATH = 'path'      # along a drawn path / the dock corridor (blade off)
AREA = 'area'      # in-area leg inside one region (mowing area or navigation polygon)
LINK = 'link'      # short join between a path portal and an area (crosses the edge)

_MERGE_M = 0.01    # nodes closer than this are one node


@dataclass
class RouteParams:
    clearance_m: float = 0.35      # robot half width 0.29 + 0.06
    # 3 -> 8 (2026-10-09, mid-path portals): lawn metres expensive, so the route rides a
    # path to near the foot of the hop to the next path instead of cutting across early
    area_weight: float = 8.0
    nav_area_weight: float = 1.0
    path_weight: float = 1.0
    approach_distance: float = 0.8
    dock_corridor_half_width: float = 0.35
    dock_corridor_max_len: float = 5.0
    tolerance_m: float = 0.05      # a point this close outside a polygon counts as in it
    arc_step_deg: float = 30.0
    # Bend points sit clearance + this off a corner (falling back to the bare clearance
    # where that does not fit): sim 2026-10-09, MPPI cut the corner round the drawn
    # triangle by 0.25 m (robot centre 0.10 m from it with bend points at 0.36 m).
    corner_margin_m: float = 0.2
    simplify_m: float = 0.05       # corner candidates from the polygon simplified this much
    resample_m: float = 0.1
    anchor_max_m: float = 1.5      # a link reaches at most this far
    sample_m: float = 0.05         # links are checked at this spacing
    # bends next to a path leg get an arc of this radius (1.0 = 0.3 m/s at the 0.3 rad/s
    # yaw clamp), else the largest of 0.7 / 0.5 / 0.3 x that fits. OFF by default: in the
    # sim (2026-10-09, v2 map) MPPI cut the rounded bends as much again (58 samples / max
    # 0.27 m outside every polygon with 1.0 m fillets vs 14 / 0.09 m without).
    fillet_radius_m: float = 0.0
    fillet_min_turn_deg: float = 25.0
    # 0 = off since the owner's end-to-end rule (2026-10-09); kept for experiments
    portal_step_m: float = 0.0
    # transit variation (2026-10-09, owner: "driving the same line eventually creates tyre
    # tracks"): per-area mode none | lanes | perimeter | mixed, picked deterministically by
    # the run index the mission passes (see RouteGraph._vary)
    robot_half_width_m: float = 0.29
    lane_margin_m: float = 0.03        # lanes keep this inside band half width - robot half
    area_shift_max_m: float = 0.5      # in-area lane: mid waypoint shifted up to this
    perimeter_margin_m: float = 0.05   # perimeter ring = area inset by clearance + this
    perimeter_max_factor: float = 2.0  # perimeter variant at most this x the direct leg
    overlap_step_m: float = 0.5    # overlapping paths are joined this often along the overlap    # mid-path portal spacing (0 = ends / junctions only)

    @classmethod
    def from_dict(cls, d):
        p = cls()
        for k, v in (d or {}).items():
            if hasattr(p, k):
                setattr(p, k, type(getattr(p, k))(v))
        return p


@dataclass
class Route:
    poses: List[Tuple[float, float, float]]
    kinds: List[str]               # per pose: PATH | AREA | LINK
    areas: List[int]               # per pose: area index of an AREA pose, else -1
    legs: List[dict]
    length_m: float
    path_length_m: float
    cost: float
    single_area: int               # mowing area index when every leg stays in it, else -1
    variant: str = ''              # transit variation used ('' = none)

    def summary(self) -> str:
        return ', '.join('%s %.1f m' % (lg['name'], lg['length']) for lg in self.legs)


@dataclass
class _Edge:
    a: int
    b: int
    kind: str
    area: int                      # area index (AREA legs) or path's area index, -1 = none
    pts: List[Point]               # a -> b
    length: float
    name: str


@dataclass
class _Region:
    index: int
    name: str
    polygon: List[Point]
    navigation: bool
    weight: float
    obstacles: List[List[Point]]
    edges: np.ndarray              # (m, 4) boundary + obstacle edges
    nodes: List[int] = field(default_factory=list)


@dataclass
class _Path:
    index: int                     # areas index, -1 = dock corridor
    name: str
    line: List[Point]
    half_width: float
    polygon: List[Point]
    cum: List[float] = field(default_factory=list)
    splits: List[Tuple[float, int]] = field(default_factory=list)   # (s, node) sorted
    edge_ids: List[int] = field(default_factory=list)                # between splits

    @property
    def length(self):
        return self.cum[-1]


# ---------------------------------------------------------------------------
# geometry helpers
# ---------------------------------------------------------------------------

def _signed_area(poly):
    s = 0.0
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        s += x1 * y2 - x2 * y1
    return 0.5 * s


def _ccw(poly):
    poly = [(float(x), float(y)) for x, y in poly]
    return poly if _signed_area(poly) >= 0 else poly[::-1]


def _ring_edges(poly) -> np.ndarray:
    a = np.asarray(poly, dtype=float)
    return np.hstack([a, np.roll(a, -1, axis=0)])


def _simplify_ring(poly, tol):
    """Greedy ring simplification: drop a vertex within ``tol`` of the chord of its
    neighbours (corner candidates only; validation always uses the true polygon)."""
    pts = list(poly)
    changed = True
    while changed and len(pts) > 3:
        changed = False
        i = 0
        while i < len(pts) and len(pts) > 3:
            p, q, r = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
            if _pt_seg(q, p, r) < tol:
                pts.pop(i)
                changed = True
            else:
                i += 1
    return pts


def _pt_seg(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 <= 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _pts_seg_dist(px, py, ax, ay, bx, by):
    """Broadcast distance from points (px, py) to segments (a, b)."""
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    with np.errstate(invalid='ignore', divide='ignore'):
        t = np.where(l2 > 0, ((px - ax) * dx + (py - ay) * dy) / np.where(l2 > 0, l2, 1.0), 0.0)
    t = np.clip(t, 0.0, 1.0)
    return np.hypot(px - ax - t * dx, py - ay - t * dy)


def seg_seg_dist(S: np.ndarray, E: np.ndarray) -> np.ndarray:
    """(k, m) minimum distances between segments S (k, 4) and E (m, 4); 0 where they cross."""
    ax, ay, bx, by = (S[:, i:i + 1] for i in range(4))
    cx, cy, dx, dy = (E[None, :, i] for i in range(4))
    d = np.minimum(np.minimum(_pts_seg_dist(ax, ay, cx, cy, dx, dy),
                              _pts_seg_dist(bx, by, cx, cy, dx, dy)),
                   np.minimum(_pts_seg_dist(cx, cy, ax, ay, bx, by),
                              _pts_seg_dist(dx, dy, ax, ay, bx, by)))
    o1 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    o2 = (bx - ax) * (dy - ay) - (by - ay) * (dx - ax)
    o3 = (dx - cx) * (ay - cy) - (dy - cy) * (ax - cx)
    o4 = (dx - cx) * (by - cy) - (dy - cy) * (bx - cx)
    cross = (o1 * o2 < 0) & (o3 * o4 < 0)
    return np.where(cross, 0.0, d)


def _seg_intersections(a, b, poly_edges: np.ndarray):
    """Parameters t in [0, 1] along a->b where it crosses the edges."""
    out = []
    ax, ay = a
    rx, ry = b[0] - ax, b[1] - ay
    for cx, cy, dx, dy in poly_edges:
        sx, sy = dx - cx, dy - cy
        den = rx * sy - ry * sx
        if abs(den) < 1e-12:
            continue
        t = ((cx - ax) * sy - (cy - ay) * sx) / den
        u = ((cx - ax) * ry - (cy - ay) * rx) / den
        if -1e-9 <= t <= 1 + 1e-9 and -1e-9 <= u <= 1 + 1e-9:
            out.append(min(1.0, max(0.0, t)))
    return out


def _cumulative(line):
    cum = [0.0]
    for p, q in zip(line, line[1:]):
        cum.append(cum[-1] + math.hypot(q[0] - p[0], q[1] - p[1]))
    return cum


def _point_at(line, cum, s):
    s = max(0.0, min(cum[-1], s))
    for i in range(len(line) - 1):
        if s <= cum[i + 1] or i == len(line) - 2:
            seg = cum[i + 1] - cum[i]
            f = 0.0 if seg <= 0 else (s - cum[i]) / seg
            p, q = line[i], line[i + 1]
            return (p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1]))
    return line[-1]


def _sub_line(line, cum, s0, s1):
    """Polyline from arc length s0 to s1 (s0 <= s1), with the interior vertices."""
    pts = [_point_at(line, cum, s0)]
    for i in range(1, len(line) - 1):
        if s0 < cum[i] < s1:
            pts.append(line[i])
    pts.append(_point_at(line, cum, s1))
    return pts


def _project(line, cum, p):
    """(distance, s) of the closest point of the polyline to p."""
    best = (math.inf, 0.0)
    for i in range(len(line) - 1):
        a, b = line[i], line[i + 1]
        dx, dy = b[0] - a[0], b[1] - a[1]
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 <= 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
        d = math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)
        if d < best[0]:
            best = (d, cum[i] + t * math.sqrt(l2))
    return best


def _polyline_length(pts):
    return sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(pts, pts[1:]))


def _arc(v, a0, sweep, r_base, step):
    """Points round corner v from angle a0 through ``sweep`` (signed) at a radius whose
    chords keep ``r_base`` from v."""
    if abs(sweep) < 1e-3:
        return []
    if abs(sweep) < math.radians(20.0):
        r = r_base / math.cos(abs(sweep) / 2.0) + 0.01
        a = a0 + sweep / 2.0
        return [(v[0] + r * math.cos(a), v[1] + r * math.sin(a))]
    n = max(1, int(math.ceil(abs(sweep) / step)))
    da = sweep / n
    r = r_base / math.cos(abs(da) / 2.0) + 0.01
    return [(v[0] + r * math.cos(a0 + k * da), v[1] + r * math.sin(a0 + k * da))
            for k in range(n + 1)]


# ---------------------------------------------------------------------------
# graph
# ---------------------------------------------------------------------------

class RouteGraph:
    """Transit graph of one map (areas + dock). ``plan(start, goal)`` -> Route | reason."""

    def __init__(self, areas: Sequence[Area], dock: Optional[DockPose] = None,
                 params: Optional[RouteParams] = None):
        self.p = params or RouteParams()
        self.areas = list(areas)
        self.dock = dock
        self.xy: List[Point] = []
        self.adj: Dict[int, List[Tuple[int, float, int]]] = {}
        self.edges: List[_Edge] = []
        self.regions: List[_Region] = []
        self.paths: List[_Path] = []
        c = float(self.p.clearance_m)
        obstacles = [[(float(x), float(y)) for x, y in o.polygon]
                     for a in self.areas for o in a.obstacles if len(o.polygon) >= 3]
        self._obstacles = [_ccw(o) for o in obstacles]
        for i, a in enumerate(self.areas):
            if len(a.polygon) < 3:
                continue
            if a.is_navigation and a.channel and len(a.channel) >= 2 and a.channel_width_m > 0:
                line = [(float(x), float(y)) for x, y in a.channel]
                self.paths.append(_Path(i, a.name or 'path %d' % i, line,
                                        0.5 * float(a.channel_width_m),
                                        [(float(x), float(y)) for x, y in a.polygon]))
            else:
                poly = _ccw(a.polygon)
                bb = self._bbox(poly, c + 0.05)
                obs = [o for o in self._obstacles if self._bbox_hit(self._bbox(o, 0.0), bb)]
                edges = np.vstack([_ring_edges(poly)] + [_ring_edges(o) for o in obs])
                self.regions.append(_Region(
                    i, a.name or 'area %d' % i, poly, bool(a.is_navigation),
                    float(self.p.nav_area_weight if a.is_navigation else self.p.area_weight),
                    obs, edges))
        self._owner: Dict[int, int] = {}      # portal node -> id of its path
        self._add_dock_path()
        for pth in self.paths:
            pth.cum = _cumulative(pth.line)
        self._build_paths()
        for reg in self.regions:
            self._build_region(reg)
        self._link_overlaps()
        for pth in self.paths:
            self._attach_portals(pth)
        for reg in self.regions:
            self._connect_region(reg)

    # ---- basics ---------------------------------------------------------
    @staticmethod
    def _bbox(poly, m):
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        return (min(xs) - m, min(ys) - m, max(xs) + m, max(ys) + m)

    @staticmethod
    def _bbox_hit(a, b):
        return not (a[2] < b[0] or b[2] < a[0] or a[3] < b[1] or b[3] < a[1])

    def _node(self, p, merge=True):
        if merge:
            for i, q in enumerate(self.xy):
                if abs(q[0] - p[0]) < _MERGE_M and abs(q[1] - p[1]) < _MERGE_M:
                    return i
        self.xy.append((float(p[0]), float(p[1])))
        self.adj[len(self.xy) - 1] = []
        return len(self.xy) - 1

    def _edge(self, a, b, kind, area, pts, weight, name, adj=None, edges=None):
        adj = self.adj if adj is None else adj
        edges = self.edges if edges is None else edges
        if a == b:
            return None
        length = _polyline_length(pts)
        e = _Edge(a, b, kind, area, list(pts), length, name)
        edges.append(e)
        eid = len(edges) - 1
        cost = length * weight + 1e-6
        adj.setdefault(a, []).append((b, cost, eid))
        adj.setdefault(b, []).append((a, cost, eid))
        return eid

    def _in_poly(self, p, poly, tol=None):
        tol = self.p.tolerance_m if tol is None else tol
        if bool(points_in_polygon(np.array([p[0]]), np.array([p[1]]), poly)[0]):
            return True
        return float(distance_to_polygon_edges(np.array([p[0]]), np.array([p[1]]), poly)[0]) <= tol

    def _inside_union(self, px, py):
        """Vectorised: centre inside some area / path polygon and outside every obstacle."""
        ok = np.zeros(np.shape(px), dtype=bool)
        for reg in self.regions:
            ok |= points_in_polygon(px, py, reg.polygon)
        for pth in self.paths:
            ok |= points_in_polygon(px, py, pth.polygon)
        for o in self._obstacles:
            ok &= ~points_in_polygon(px, py, o)
        return ok

    # ---- clearance tests ----------------------------------------------------
    def _clearance(self, reg, px, py):
        """Vectorised distance to the region's edges; -1 where outside / in an obstacle."""
        px = np.atleast_1d(np.asarray(px, dtype=float))
        py = np.atleast_1d(np.asarray(py, dtype=float))
        E = reg.edges
        d = _pts_seg_dist(px[:, None], py[:, None], E[None, :, 0], E[None, :, 1],
                          E[None, :, 2], E[None, :, 3]).min(axis=1)
        inside = points_in_polygon(px, py, reg.polygon)
        for o in reg.obstacles:
            inside &= ~points_in_polygon(px, py, o)
        return np.where(inside, d, -1.0)

    def _seg_ok(self, reg, a, bs, r):
        """Mask over targets ``bs`` (list of points): straight a->b keeps ``r`` from every
        region / obstacle edge (endpoints are known to be inside with clearance)."""
        if not bs:
            return np.zeros(0, dtype=bool)
        S = np.array([[a[0], a[1], b[0], b[1]] for b in bs], dtype=float)
        out = np.zeros(len(bs), dtype=bool)
        for k0 in range(0, len(bs), 256):
            out[k0:k0 + 256] = seg_seg_dist(S[k0:k0 + 256], reg.edges).min(axis=1) >= r - 1e-6
        return out

    def _link_ok(self, a, b, reg=None, escape=False):
        """Every sample of a->b inside (the region, else) the area / path union.
        Returns None (bad), AREA (inside ``reg``) or LINK. ``escape``: a start pose on /
        in an obstacle polygon (Nav2 left the robot there, sim 2026-10-09) may link out
        through it (LINK, blade off)."""
        n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / self.p.sample_m)))
        f = np.linspace(0.0, 1.0, n + 1)
        px = a[0] + f * (b[0] - a[0])
        py = a[1] + f * (b[1] - a[1])
        if escape:
            ok = np.zeros(np.shape(px), dtype=bool)
            for poly in [r.polygon for r in self.regions] + [q.polygon for q in self.paths]:
                ok |= points_in_polygon(px, py, poly)
            return LINK if ok.all() else None
        if reg is not None:
            inr = points_in_polygon(px, py, reg.polygon)
            for o in reg.obstacles:
                inr &= ~points_in_polygon(px, py, o)
            if inr.all():
                return AREA
        return LINK if self._inside_union(px, py).all() else None

    def _anchor(self, reg, p, escape=False):
        """Nearest point (<= anchor_max_m) with full clearance in ``reg`` that ``p`` links to."""
        c = float(self.p.clearance_m)
        cands = []
        # first guess: straight off the nearest edge
        E = reg.edges
        d = _pts_seg_dist(p[0], p[1], E[:, 0], E[:, 1], E[:, 2], E[:, 3])
        k = int(np.argmin(d))
        ax, ay, bx, by = E[k]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 <= 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / l2))
        qx, qy = ax + t * dx, ay + t * dy
        if d[k] > 1e-3:
            nx, ny = (p[0] - qx) / d[k], (p[1] - qy) / d[k]
        else:
            ln = math.sqrt(l2) or 1.0
            nx, ny = -dy / ln, dx / ln          # left of a CCW ring edge = inside
            if k >= len(reg.polygon):           # obstacle edge (CCW): outside is the right
                nx, ny = -nx, -ny
        cands.append((qx + nx * (c + 0.03), qy + ny * (c + 0.03)))
        # then rings of candidates round p, nearest first
        for rr in np.arange(0.05, self.p.anchor_max_m + 1e-9, 0.05):
            for ang in np.linspace(0.0, 2 * math.pi, 48, endpoint=False):
                cands.append((p[0] + rr * math.cos(ang), p[1] + rr * math.sin(ang)))
        arr = np.array(cands)
        clr = self._clearance(reg, arr[:, 0], arr[:, 1])
        good = np.nonzero(clr >= c - 1e-9)[0]
        if good.size == 0:
            return None
        dist = np.hypot(arr[good, 0] - p[0], arr[good, 1] - p[1])
        order = good[np.argsort(dist, kind='stable')]
        if clr[0] >= c - 1e-9:
            order = np.concatenate([[0], order[order != 0]])
        for i in order[:64]:
            q = (float(arr[i, 0]), float(arr[i, 1]))
            kind = self._link_ok(p, q, reg, escape)
            if kind is not None:
                return q, kind
        return None

    # ---- dock -------------------------------------------------------------
    def _add_dock_path(self):
        d = self.dock
        if d is None:
            return
        ad = float(self.p.approach_distance)
        ax, ay = d.x + ad * math.cos(d.yaw), d.y + ad * math.sin(d.yaw)
        line = [(float(d.x), float(d.y)), (ax, ay)]
        polys = [r.polygon for r in self.regions] + [p.polygon for p in self.paths]
        if polys and not any(self._in_poly((ax, ay), q, 0.0) for q in polys):
            best = None
            for q in polys:
                E = _ring_edges(q)
                dd = _pts_seg_dist(ax, ay, E[:, 0], E[:, 1], E[:, 2], E[:, 3])
                k = int(np.argmin(dd))
                if best is None or dd[k] < best[0]:
                    x0, y0, x1, y1 = E[k]
                    dx, dy = x1 - x0, y1 - y0
                    l2 = dx * dx + dy * dy
                    t = 0.0 if l2 <= 0 else max(0.0, min(1.0, ((ax - x0) * dx + (ay - y0) * dy) / l2))
                    best = (float(dd[k]), x0 + t * dx, y0 + t * dy)
            if best is not None and best[0] <= self.p.dock_corridor_max_len:
                g, bx, by = best
                ext = 0.05 / g if g > 1e-9 else 0.0     # end just inside the polygon
                line.append((bx + (bx - ax) * ext, by + (by - ay) * ext))
        hw = float(self.p.dock_corridor_half_width)
        self.paths.append(_Path(-1, 'dock', line, hw, capsule_outline(line, hw)))

    # ---- paths --------------------------------------------------------------
    def _build_paths(self):
        tol = self.p.tolerance_m
        splits = {id(p): {0.0, p.length} for p in self.paths}
        self._crossings = {}       # (path id, region index) -> [s]
        for pth in self.paths:
            for reg in self.regions:
                E = _ring_edges(reg.polygon)
                ss = []
                for i in range(len(pth.line) - 1):
                    seg = pth.cum[i + 1] - pth.cum[i]
                    for t in _seg_intersections(pth.line[i], pth.line[i + 1], E):
                        ss.append(pth.cum[i] + t * seg)
                self._crossings[(id(pth), reg.index)] = ss
                splits[id(pth)].update(ss)
        # 2026-10-09 (owner GUI sim, redrawn map): mid-path portals. With portals only at
        # path ends the dock -> area route ignored Path 1 (using it meant its far end plus a
        # 6.9 m lawn hop back). Every route_portal_step_m along the parts of a centreline
        # that lie inside an area with full clearance is a portal into it, so the route
        # leaves a path near the foot of the hop to the next one: path -> direct -> path.
        self._mid_portals = {}     # (path id, region index) -> [s]
        step = float(self.p.portal_step_m)
        c = float(self.p.clearance_m)
        if step > 0:
            for pth in self.paths:
                ss = np.arange(0.0, pth.length + 1e-9, step)
                pts = np.array([_point_at(pth.line, pth.cum, x) for x in ss])
                for reg in self.regions:
                    clr = self._clearance(reg, pts[:, 0], pts[:, 1])
                    mid = [float(x) for x, ok in zip(ss, clr >= c - 1e-9) if ok]
                    self._mid_portals[(id(pth), reg.index)] = mid
                    splits[id(pth)].update(mid)
        self._junctions = {id(p): [] for p in self.paths}      # s of junction nodes
        for i, P in enumerate(self.paths):
            for Q in self.paths[i + 1:]:
                for a in range(len(P.line) - 1):
                    for b in range(len(Q.line) - 1):
                        E = np.array([[*Q.line[b], *Q.line[b + 1]]])
                        for t in _seg_intersections(P.line[a], P.line[a + 1], E):
                            x = _point_at(P.line, P.cum, P.cum[a] + t * (P.cum[a + 1] - P.cum[a]))
                            sp = _project(P.line, P.cum, x)[1]
                            sq = _project(Q.line, Q.cum, x)[1]
                            splits[id(P)].add(sp)
                            splits[id(Q)].add(sq)
                            # a real crossing is a junction (a lawn portal); a shallow one
                            # is an overlap (v2 map: Path 1 and Dock path cross at ~2 deg):
                            # leaving there would be leaving mid-path
                            hp = math.atan2(P.line[a + 1][1] - P.line[a][1],
                                            P.line[a + 1][0] - P.line[a][0])
                            hq = math.atan2(Q.line[b + 1][1] - Q.line[b][1],
                                            Q.line[b + 1][0] - Q.line[b][0])
                            ang = abs(math.atan2(math.sin(hp - hq), math.cos(hp - hq)))
                            if min(ang, math.pi - ang) >= math.radians(25.0):
                                self._junctions[id(P)].append(sp)
                                self._junctions[id(Q)].append(sq)
        # T-junctions: a path end inside another path's band joins its centreline.
        # Overlaps (2026-10-09, owner drew "Dock path" almost on top of Path 1): every
        # overlap_step_m along a path that runs inside another path's band is joined to
        # that centreline too, so overlapping / duplicated / shallow-crossing paths act as
        # one path along the overlap (both usable, switch anywhere), not only at their ends.
        self._tlinks = []
        step = max(0.1, float(self.p.overlap_step_m))
        for P in self.paths:
            n = max(1, int(math.ceil(P.length / step)))
            for k in range(n + 1):
                s_p = min(P.length, k * step) if k < n else P.length
                end = k == 0 or k == n
                e = _point_at(P.line, P.cum, s_p)
                for Q in self.paths:
                    if Q is P:
                        continue
                    d, sq = _project(Q.line, Q.cum, e)
                    inside = d <= Q.half_width + tol or (end and self._in_poly(e, Q.polygon, 0.0))
                    if not inside:
                        continue
                    if sq < 0.02:
                        sq = 0.0
                    elif sq > Q.length - 0.02:
                        sq = Q.length
                    splits[id(Q)].add(sq)
                    splits[id(P)].add(s_p)
                    self._tlinks.append((P, s_p, Q, sq))
        self._free_ends = {id(P): self._free_end_list(P) for P in self.paths}
        for pth in self.paths:
            ss = sorted(splits[id(pth)])
            merged = []
            for s in ss:
                if not merged or s - merged[-1] > 1e-3:
                    merged.append(s)
            pth.splits = [(s, self._node(_point_at(pth.line, pth.cum, s))) for s in merged]
            pth.edge_ids = []
            for (s0, n0), (s1, n1) in zip(pth.splits, pth.splits[1:]):
                pth.edge_ids.append(self._edge(n0, n1, PATH, pth.index,
                                               _sub_line(pth.line, pth.cum, s0, s1),
                                               self.p.path_weight, pth.name))
        for P, sp, Q, sq in self._tlinks:
            a = self._split_node(P, sp)
            b = self._split_node(Q, sq)
            if a != b:
                self._edge(a, b, PATH, Q.index, [self.xy[a], self.xy[b]], self.p.path_weight,
                           Q.name)

    def _free_end_list(self, P):
        """Ends of P that are lawn portals (2026-10-09 owner rule: paths are driven END TO
        END; one is left / entered only at its ends or at a junction). Not free:
        * the dock end of the dock pseudo path (never leave via the charger);
        * the dock approach end when it lies on a drawn path's band (leaving the dock uses
          that path), and a drawn path end inside the dock corridor (it starts at the dock);
        * a drawn path end inside another drawn path's band away from that path's ends
          (a spur joining it: a junction, not an exit).
        Coincident ends of duplicated / overlapping paths stay free (network ends)."""
        tol = self.p.tolerance_m
        out = []
        for s_end in (0.0, P.length):
            if P.index == -1 and s_end == 0.0:
                continue
            e = _point_at(P.line, P.cum, s_end)
            free = True
            for Q in self.paths:
                if Q is P:
                    continue
                d, sq = _project(Q.line, Q.cum, e)
                if not (d <= Q.half_width + tol or self._in_poly(e, Q.polygon, 0.0)):
                    continue
                if P.index == -1 or Q.index == -1:
                    free = False
                elif 0.3 < sq < Q.length - 0.3:
                    free = False
                else:
                    # chained paths (P ends where Q starts and Q carries on): the network
                    # continues, so this is a junction, not an exit; coincident ends of
                    # duplicated paths (Q goes back along P) stay free
                    sn = s_end - 0.3 if s_end > 0 else 0.3
                    inner = _point_at(P.line, P.cum, max(0.0, min(P.length, sn)))
                    ox, oy = e[0] - inner[0], e[1] - inner[1]
                    on = math.hypot(ox, oy)
                    if on > 1e-6:
                        ahead = (e[0] + 0.5 * ox / on, e[1] + 0.5 * oy / on)
                        if _project(Q.line, Q.cum, ahead)[0] <= Q.half_width:
                            free = False
            if free:
                out.append(s_end)
        return out

    def on_path_band(self, q):
        """Paths whose band (half width + tolerance, or polygon) contains q."""
        tol = self.p.tolerance_m
        return [p for p in self.paths
                if _project(p.line, p.cum, q)[0] <= p.half_width + tol
                or self._in_poly(q, p.polygon, 0.0)]

    @staticmethod
    def _split_node(pth, s):
        return min(pth.splits, key=lambda sn: abs(sn[0] - s))[1]

    # ---- regions ------------------------------------------------------------
    def _build_region(self, reg):
        """Corner candidates: clearance off every reflex area corner and every convex
        obstacle corner (the bends of shortest paths in the shrunk area)."""
        c = float(self.p.clearance_m)
        cm = c + max(0.0, float(self.p.corner_margin_m))
        step = math.radians(self.p.arc_step_deg)
        cands = []
        rings = [(_simplify_ring(reg.polygon, self.p.simplify_m), True)]
        rings += [(_simplify_ring(o, self.p.simplify_m), False) for o in reg.obstacles]
        for ring, outer in rings:
            n = len(ring)
            for i in range(n):
                u, v, w = ring[i - 1], ring[i], ring[(i + 1) % n]
                d1 = (v[0] - u[0], v[1] - u[1])
                d2 = (w[0] - v[0], w[1] - v[1])
                if math.hypot(*d1) < 1e-9 or math.hypot(*d2) < 1e-9:
                    continue
                turn = math.atan2(d1[0] * d2[1] - d1[1] * d2[0], d1[0] * d2[0] + d1[1] * d2[1])
                h1 = math.atan2(d1[1], d1[0])
                if outer and turn < 0:          # reflex corner of the area (CCW ring)
                    a0 = h1 + math.pi / 2
                elif not outer and turn > 0:    # convex obstacle corner: wrap outside
                    a0 = h1 - math.pi / 2
                else:
                    continue
                cands.append((_arc(v, a0, turn, cm, step), _arc(v, a0, turn, c, step)))
        for wide, tight in cands:
            for k, (pw, pt) in enumerate(zip(wide, tight)):
                for q in (pw, pt):              # the wide point, else the tight one
                    if self._clearance(reg, [q[0]], [q[1]])[0] >= c - 1e-9:
                        reg.nodes.append(self._node(q, merge=False))
                        break

    def _link_overlaps(self):
        """A corner node of one region that has the clearance in another overlapping
        region joins that region too (overlapping areas connect)."""
        c = float(self.p.clearance_m)
        for reg in self.regions:
            for other in self.regions:
                if other is reg or not reg.nodes:
                    continue
                pts = np.array([self.xy[n] for n in reg.nodes])
                clr = self._clearance(other, pts[:, 0], pts[:, 1])
                for n, ok in zip(list(reg.nodes), clr >= c - 1e-9):
                    if ok and n not in other.nodes:
                        other.nodes.append(n)

    def _attach_point(self, reg, node, adj=None, edges=None, nodes=None, escape=False):
        """Join ``node`` to region ``reg``: directly when it has the clearance, else by a
        link to an anchor. Returns the region-side node or None."""
        c = float(self.p.clearance_m)
        p = self.xy[node]
        clr = float(self._clearance(reg, [p[0]], [p[1]])[0])
        if clr >= c - 1e-9:
            return node
        got = self._anchor(reg, p) if not escape else None
        if got is None and escape:
            got = self._anchor(reg, p, escape=True)
        if got is None:
            return None
        q, kind = got
        a = self._node(q, merge=False)
        if nodes is not None:
            nodes.append(a)
        self._edge(node, a, kind, reg.index if kind == AREA else -1, [p, q],
                   reg.weight if kind == AREA else self.p.area_weight,
                   reg.name if kind == AREA else 'link', adj, edges)
        return a

    def _attach_portals(self, pth):
        tol = self.p.tolerance_m
        for reg in self.regions:
            ss = [s for s in self._free_ends[id(pth)]
                  if self._in_poly(_point_at(pth.line, pth.cum, s), reg.polygon, tol)]
            ss += [s for s in self._junctions[id(pth)]
                   if self._in_poly(_point_at(pth.line, pth.cum, s), reg.polygon, tol)]
            if pth.index != -1:
                # owner 2026-10-09: "a path is only left at its END or at a JUNCTION with an
                # AREA or another PATH": where the centreline crosses the area boundary is
                # such a junction (the link logic joins it to the area's inside)
                ss += list(self._crossings.get((id(pth), reg.index), []))
            ss += self._mid_portals.get((id(pth), reg.index), [])
            for s in sorted(set(round(x, 6) for x in ss)):
                n = self._split_node(pth, s)
                got = self._attach_point(reg, n)
                if got is not None and got not in reg.nodes:
                    reg.nodes.append(got)
                    self._owner[got] = id(pth)

    def _connect_region(self, reg):
        """All-pairs straight legs between the region's nodes, except between two portals
        of the same path (that path itself joins them; keeps the graph small)."""
        c = float(self.p.clearance_m)
        nodes = reg.nodes
        for i, a in enumerate(nodes):
            own = self._owner.get(a)
            rest = [b for b in nodes[i + 1:] if own is None or self._owner.get(b) != own]
            if not rest:
                break
            ok = self._seg_ok(reg, self.xy[a], [self.xy[b] for b in rest], c)
            for b, good in zip(rest, ok):
                if good:
                    self._edge(a, b, AREA, reg.index, [self.xy[a], self.xy[b]], reg.weight,
                               reg.name)

    # ---- queries ------------------------------------------------------------
    def dock_goal(self, facing_dock=True):
        """Approach pose (x, y, yaw) in front of the dock (mower_docking approach_pose;
        yaw faces the dock when ``facing_dock``, as its approach_facing_dock)."""
        d = self.dock
        if d is None:
            return None
        ad = float(self.p.approach_distance)
        yaw = d.yaw + (math.pi if facing_dock else 0.0)
        return (d.x + ad * math.cos(d.yaw), d.y + ad * math.sin(d.yaw),
                math.atan2(math.sin(yaw), math.cos(yaw)))

    def _attach_query(self, q, adj, edges, splits, escape=False, regions=True):
        """Temporary node for a start / goal and its joins. Returns the node id.
        ``regions`` False: path joins only (a start / goal on a path band is driven along
        the path, owner rule 2026-10-09)."""
        tol = self.p.tolerance_m
        n = self._node(q, merge=False)
        adj[n] = []
        joined = False
        for pth in self.paths:
            d, s = _project(pth.line, pth.cum, q)
            if not (d <= pth.half_width + tol or self._in_poly(q, pth.polygon, 0.0)):
                continue
            k = max(0, min(len(pth.splits) - 2,
                           next((i for i in range(len(pth.splits) - 1)
                                 if s <= pth.splits[i + 1][0]), len(pth.splits) - 2)))
            (s0, n0), (s1, n1) = pth.splits[k], pth.splits[k + 1]
            t = self._node(_point_at(pth.line, pth.cum, s), merge=False)
            adj[t] = []
            self._edge(t, n0, PATH, pth.index, _sub_line(pth.line, pth.cum, s0, s)[::-1],
                       self.p.path_weight, pth.name, adj, edges)
            self._edge(t, n1, PATH, pth.index, _sub_line(pth.line, pth.cum, s, s1),
                       self.p.path_weight, pth.name, adj, edges)
            for (pp, kk, ss, tt) in splits:       # the other query point on the same edge
                if pp is pth and kk == k:
                    lo, hi = sorted((s, ss))
                    pts = _sub_line(pth.line, pth.cum, lo, hi)
                    a, b = (t, tt) if s <= ss else (tt, t)
                    self._edge(a, b, PATH, pth.index, pts, self.p.path_weight, pth.name,
                               adj, edges)
            splits.append((pth, k, s, t))
            self._edge(n, t, PATH, pth.index, [q, self.xy[t]], self.p.path_weight, pth.name,
                       adj, edges)
            joined = True
        for reg in self.regions:
            if not regions or not self._in_poly(q, reg.polygon, tol):
                continue
            extra = []
            r = self._attach_point(reg, n, adj, edges, extra, escape)
            if r is None:
                continue
            joined = True
            targets = reg.nodes
            ok = self._seg_ok(reg, self.xy[r], [self.xy[b] for b in targets],
                              float(self.p.clearance_m))
            for b, good in zip(targets, ok):
                if good:
                    self._edge(r, b, AREA, reg.index, [self.xy[r], self.xy[b]], reg.weight,
                               reg.name, adj, edges)
            reg_q = getattr(self, '_q_region_nodes', None)
            if reg_q is not None:
                reg_q.setdefault(reg.index, []).append(r)
        return n if joined else None

    def plan(self, start, goal, variation=None, modes=None):
        """Route from ``start`` (x, y[, yaw]) to ``goal`` (x, y, yaw); returns a Route, or a
        str with the reason (off graph / no route)."""
        n0 = len(self.xy)
        adj = {k: list(v) for k, v in self.adj.items()}
        edges = list(self.edges)
        splits = []
        self._q_region_nodes = {}
        try:
            a, b = (float(start[0]), float(start[1])), (float(goal[0]), float(goal[1]))
            sb, gb = self.on_path_band(a), self.on_path_band(b)
            # Owner rule (2026-10-09): a start / goal on a path band uses that path (no lawn
            # hop straight off / onto it). Exception: start and goal in one area and the
            # start not at the dock (an in-area mowing transit that happens to begin on a
            # path band must not ride the path to its far end and back).
            share = any(self._in_poly(a, r.polygon, self.p.tolerance_m)
                        and self._in_poly(b, r.polygon, self.p.tolerance_m)
                        for r in self.regions)
            at_dock = any(p.index == -1 for p in sb)
            s_reg = not sb or (share and not at_dock)
            g_reg = not gb or (share and not at_dock)
            s = self._attach_query(a, adj, edges, splits, escape=True, regions=s_reg)
            if s is None:
                return 'start (%.2f, %.2f) is off the route graph' % (start[0], start[1])
            g = self._attach_query(b, adj, edges, splits, regions=g_reg)
            if g is None:
                return 'goal (%.2f, %.2f) is off the route graph' % (goal[0], goal[1])
            # the two query anchors of one region see each other
            for reg in self.regions:
                qn = self._q_region_nodes.get(reg.index, [])
                if len(qn) == 2 and qn[0] != qn[1]:
                    if self._seg_ok(reg, self.xy[qn[0]], [self.xy[qn[1]]],
                                    float(self.p.clearance_m))[0]:
                        self._edge(qn[0], qn[1], AREA, reg.index,
                                   [self.xy[qn[0]], self.xy[qn[1]]], reg.weight, reg.name,
                                   adj, edges)
                if s_reg and g_reg:
                    self._direct(reg, start, goal, s, g, adj, edges)
            seq = self._dijkstra(adj, s, g)
            if seq is None:
                return 'no route from (%.2f, %.2f) to (%.2f, %.2f)' % (
                    start[0], start[1], goal[0], goal[1])
            return self._route(seq, edges, goal, variation, modes)
        finally:
            del self.xy[n0:]
            for k in [k for k in self.adj if k >= n0]:
                del self.adj[k]
            self._q_region_nodes = None

    def _direct(self, reg, start, goal, s, g, adj, edges):
        """Start and goal in one region: straight hop when it keeps the smaller of their
        own edge clearances (a short in-area transit along the outer coverage ring)."""
        a, b = (float(start[0]), float(start[1])), (float(goal[0]), float(goal[1]))
        clr = self._clearance(reg, [a[0], b[0]], [a[1], b[1]])
        if clr.min() < 0.02:
            return
        r = min(float(self.p.clearance_m), float(clr.min()))
        if self._seg_ok(reg, a, [b], r)[0]:
            self._edge(s, g, AREA, reg.index, [a, b], reg.weight, reg.name, adj, edges)

    @staticmethod
    def _dijkstra(adj, s, g):
        dist = {s: 0.0}
        prev = {}
        pq = [(0.0, s)]
        while pq:
            d, u = heapq.heappop(pq)
            if u == g:
                break
            if d > dist.get(u, math.inf):
                continue
            for v, w, eid in adj.get(u, ()):
                nd = d + w
                if nd < dist.get(v, math.inf) - 1e-12:
                    dist[v] = nd
                    prev[v] = (u, eid)
                    heapq.heappush(pq, (nd, v))
        if g not in dist:
            return None
        seq = []
        v = g
        while v != s:
            u, eid = prev[v]
            seq.append((u, v, eid))
            v = u
        return seq[::-1], dist[g]

    def _fillet_ok(self, px, py):
        """Fillet samples: inside a path polygon, or with full clearance in a region."""
        px, py = np.asarray(px, dtype=float), np.asarray(py, dtype=float)
        ok = np.zeros(px.shape, dtype=bool)
        for pth in self.paths:
            ok |= points_in_polygon(px, py, pth.polygon)
        c = float(self.p.clearance_m)
        for reg in self.regions:
            if ok.all():
                break
            ok |= self._clearance(reg, px, py) >= c - 1e-6
        return bool(ok.all())

    def _fillet(self, verts):
        """Round acute bends next to a path leg with an arc of fillet_radius_m (smaller where
        the neighbouring segments are short), only where the arc stays inside the path
        polygon (or keeps the clearance in an area). Sim 2026-10-09: MPPI cut Path 2's 112
        deg bend and left the band by 0.13 m; the commanded route itself is now drivable."""
        R0 = float(self.p.fillet_radius_m)
        if R0 <= 0 or len(verts) < 3:
            return verts
        min_turn = math.radians(float(self.p.fillet_min_turn_deg))
        out = [verts[0]]
        for i in range(1, len(verts) - 1):
            a, v, b = verts[i - 1], verts[i], verts[i + 1]
            if PATH not in (v[2], b[2]):
                out.append(v)
                continue
            d1x, d1y = v[0] - a[0], v[1] - a[1]
            d2x, d2y = b[0] - v[0], b[1] - v[1]
            l1, l2 = math.hypot(d1x, d1y), math.hypot(d2x, d2y)
            if l1 < 1e-6 or l2 < 1e-6:
                out.append(v)
                continue
            turn = math.atan2(d1x * d2y - d1y * d2x, d1x * d2x + d1y * d2y)
            if abs(turn) < min_turn or abs(turn) > math.radians(170.0):
                out.append(v)
                continue
            half = (math.pi - abs(turn)) / 2.0           # half the interior angle
            u1 = (d1x / l1, d1y / l1)
            sgn = 1.0 if turn > 0 else -1.0                 # centre on the inner (turn) side
            n = max(2, int(math.ceil(abs(turn) / math.radians(10.0))))
            arc = None
            for f in (1.0, 0.7, 0.5, 0.3):
                t = min(R0 * f / math.tan(half), 0.45 * l1, 0.45 * l2)
                R = t * math.tan(half)
                if R < 0.1:
                    break
                p1 = (v[0] - u1[0] * t, v[1] - u1[1] * t)
                cx, cy = p1[0] - sgn * u1[1] * R, p1[1] + sgn * u1[0] * R
                a0 = math.atan2(p1[1] - cy, p1[0] - cx)
                cand = [(cx + R * math.cos(a0 + turn * k / n), cy + R * math.sin(a0 + turn * k / n))
                        for k in range(n + 1)]
                xs, ys = zip(*cand)
                if self._fillet_ok(xs, ys):
                    arc = cand
                    break
            if arc is None:
                out.append(v)
                continue
            for k, q in enumerate(arc):
                kind = v if k <= n // 2 else b            # first half: incoming leg
                out.append((q[0], q[1], kind[2], kind[3]))
        out.append(verts[-1])
        return out

    # ---- transit variation ------------------------------------------------
    LANES = (0.0, 1.0, -1.0, 0.5, -0.5)
    SHIFTS = (0.0, 1.0, -1.0)

    def _region_of(self, idx):
        return next((r for r in self.regions if r.index == idx), None)

    def _path_of(self, idx):
        return next((p for p in self.paths if p.index == idx), None)

    @staticmethod
    def _dense(pts, step=0.1):
        out = [tuple(pts[0])]
        for p, q in zip(pts, pts[1:]):
            L = math.hypot(q[0] - p[0], q[1] - p[1])
            n = max(1, int(math.ceil(L / step)))
            out += [(p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n)
                    for k in range(1, n + 1)]
        return out

    def _lane(self, pts, pth, frac):
        """Path leg shifted sideways by frac x the allowed lane offset (band half width -
        robot half width - lane_margin_m), ramped in / out over 1 m at the leg ends so it
        still starts / ends on the centreline (ends, junctions). None = no lane fits."""
        allowed = pth.half_width - float(self.p.robot_half_width_m) - float(self.p.lane_margin_m)
        L = _polyline_length(pts)
        if allowed < 0.05 or abs(frac) < 1e-6 or L < 1.0:
            return None, 0.0
        d = self._dense(pts)
        cum = _cumulative(d)
        taper = min(1.0, L / 3.0)
        for off in (allowed * frac, 0.5 * allowed * frac):
            out = []
            for i, p in enumerate(d):
                a, b = d[max(0, i - 1)], d[min(len(d) - 1, i + 1)]
                tl = math.hypot(b[0] - a[0], b[1] - a[1]) or 1.0
                nx, ny = -(b[1] - a[1]) / tl, (b[0] - a[0]) / tl
                f = min(1.0, cum[i] / taper, (L - cum[i]) / taper)
                out.append((p[0] + nx * off * f, p[1] + ny * off * f))
            xs, ys = zip(*out)
            dist = np.array([_project(pth.line, pth.cum, q)[0] for q in out])
            if (dist <= pth.half_width - float(self.p.robot_half_width_m) + 1e-6).all() and \
                    points_in_polygon(np.array(xs), np.array(ys), pth.polygon).all():
                return out, off
        return None, 0.0

    def _shift(self, pts, reg, frac):
        """In-area leg: each straight piece >= 2 m gets a mid waypoint shifted sideways
        by frac x min(area_shift_max_m, 15 % of its length), kept only if both halves keep
        the clearance (else half the shift, else none)."""
        if abs(frac) < 1e-6:
            return None
        c = float(self.p.clearance_m)
        out, changed = [tuple(pts[0])], False
        for a, b in zip(pts, pts[1:]):
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            if L >= 2.0:
                clr = self._clearance(reg, [a[0], b[0]], [a[1], b[1]])
                r = min(c, float(clr.min())) if clr.min() > 0 else 0.0
                nx, ny = -(b[1] - a[1]) / L, (b[0] - a[0]) / L
                for sh in (frac * min(float(self.p.area_shift_max_m), 0.15 * L),
                           0.5 * frac * min(float(self.p.area_shift_max_m), 0.15 * L)):
                    m = ((a[0] + b[0]) / 2 + nx * sh, (a[1] + b[1]) / 2 + ny * sh)
                    if r > 0 and self._clearance(reg, [m[0]], [m[1]])[0] >= c - 1e-6 and \
                            self._seg_ok(reg, a, [m], r)[0] and self._seg_ok(reg, m, [b], r)[0]:
                        out.append(m)
                        changed = True
                        break
            out.append(tuple(b))
        return out if changed else None

    def _inset_ring(self, reg):
        """The area boundary offset inward by clearance + perimeter_margin_m (CCW), or
        None when it is not a valid drivable ring (narrow parts, obstacles near the edge)."""
        cache = getattr(self, '_rings', None)
        if cache is None:
            cache = self._rings = {}
        if reg.index in cache:
            return cache[reg.index]
        c = float(self.p.clearance_m)
        dd = c + float(self.p.perimeter_margin_m)
        poly = _simplify_ring(reg.polygon, float(self.p.simplify_m))
        n = len(poly)
        ring = []
        for i in range(n):
            u, v, w = poly[i - 1], poly[i], poly[(i + 1) % n]
            l1 = math.hypot(v[0] - u[0], v[1] - u[1]) or 1.0
            l2 = math.hypot(w[0] - v[0], w[1] - v[1]) or 1.0
            n1 = (-(v[1] - u[1]) / l1, (v[0] - u[0]) / l1)
            n2 = (-(w[1] - v[1]) / l2, (w[0] - v[0]) / l2)
            bx, by = n1[0] + n2[0], n1[1] + n2[1]
            bl = math.hypot(bx, by)
            if bl < 1e-6:
                continue
            cosh = (n1[0] * bx + n1[1] * by) / bl
            k = dd / max(cosh, 0.3)
            ring.append((v[0] + bx / bl * k, v[1] + by / bl * k))
        ok = len(ring) >= 3
        if ok:
            clr = self._clearance(reg, [q[0] for q in ring], [q[1] for q in ring])
            ok = bool((clr >= c - 1e-6).all())
        if ok:
            for a, b in zip(ring, ring[1:] + ring[:1]):
                if not self._seg_ok(reg, a, [b], c)[0]:
                    ok = False
                    break
        cache[reg.index] = ring if ok else None
        return cache[reg.index]

    def _perimeter(self, pts, reg, ccw):
        """In-area leg A -> B round the inset ring (ccw / cw), or None."""
        ring = self._inset_ring(reg)
        if not ring:
            return None
        A, B = tuple(pts[0]), tuple(pts[-1])
        closed = ring + [ring[0]]
        cum = _cumulative(closed)
        P = cum[-1]
        sa, sb = _project(closed, cum, A)[1], _project(closed, cum, B)[1]
        pa, pb = _point_at(closed, cum, sa), _point_at(closed, cum, sb)
        span = (sb - sa) % P if ccw else (sa - sb) % P
        seq = [A, pa]
        for i in range(len(ring)):
            off = (cum[i] - sa) % P if ccw else (sa - cum[i]) % P
            if 1e-6 < off < span - 1e-6:
                seq.append((off, ring[i]))
        mids = sorted(seq[2:], key=lambda t: t[0])
        seq = [A, pa] + [q for _, q in mids] + [pb, B]
        c = float(self.p.clearance_m)
        clr = self._clearance(reg, [A[0], B[0]], [A[1], B[1]])
        r = min(c, float(clr.min()))
        if r <= 0:
            return None
        for a, b in ((A, pa), (pb, B)):
            if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-6 and not self._seg_ok(reg, a, [b], r)[0]:
                return None
        return seq

    def _vary(self, legs, k, modes):
        """Apply the per-area transit variation for run index ``k`` to the legs (in place);
        returns a description. mode per leg from ``modes`` (area index -> mode, default
        'lanes'): lanes = path legs offset sideways inside their band (cycle 0, +1, -1, +1/2,
        -1/2 of the allowed offset) and in-area pieces shifted sideways (0, +1, -1);
        perimeter = in-area legs cycle through direct / inset-ring cw / ccw (only variants
        <= perimeter_max_factor x the direct length); mixed = run k % 3: direct, lanes
        (index k // 3), perimeter (index k // 3)."""
        desc = []
        for lg in legs:
            mode = (modes or {}).get(lg['area'], 'lanes') if lg['area'] >= 0 else 'none'
            if mode not in ('lanes', 'perimeter', 'mixed'):
                continue
            kind, sub = mode, k
            if mode == 'mixed':
                kind = ('none', 'lanes', 'perimeter')[k % 3]
                sub = k // 3 + (1 if kind == 'lanes' else 0)
            if lg['kind'] == PATH and kind == 'lanes':
                pth = self._path_of(lg['area'])
                if pth is None:
                    continue
                pts, off = self._lane(lg['pts'], pth, self.LANES[sub % len(self.LANES)])
                if pts:
                    lg['pts'] = pts
                    desc.append('%s lane %+.2f m' % (lg['name'], off))
            elif lg['kind'] == AREA:
                reg = self._region_of(lg['area'])
                if reg is None:
                    continue
                # vary only the core between the first / last points with full clearance
                # (links to a boundary crossing, a near-edge start / goal stay as they are)
                full = lg['pts']
                clr = self._clearance(reg, [q[0] for q in full], [q[1] for q in full])
                good = [i for i, v in enumerate(clr) if v >= float(self.p.clearance_m) - 1e-6]
                if len(good) < 2 or good[-1] - good[0] < 1:
                    continue
                pre, post = full[:good[0]], full[good[-1] + 1:]
                lg['pts'] = full[good[0]:good[-1] + 1]
                if kind == 'lanes':
                    pts = self._shift(lg['pts'], reg, self.SHIFTS[sub % len(self.SHIFTS)])
                    if pts:
                        lg['pts'] = pts
                        desc.append('%s shift %+d' % (lg['name'], self.SHIFTS[sub % 3]))
                elif kind == 'perimeter':
                    direct = _polyline_length(lg['pts'])
                    cands = [] if mode == 'mixed' else [('direct', None)]
                    for name, ccw in (('cw', False), ('ccw', True)):
                        q = self._perimeter(lg['pts'], reg, ccw)
                        if q and _polyline_length(q) <= float(self.p.perimeter_max_factor) \
                                * max(direct, 1e-6):
                            cands.append((name, q))
                    if not cands:
                        lg['pts'] = list(pre) + list(lg['pts']) + list(post)
                        continue
                    name, q = cands[sub % len(cands)]
                    if q:
                        lg['pts'] = q
                        desc.append('%s perimeter %s' % (lg['name'], name))
                lg['pts'] = list(pre) + list(lg['pts']) + list(post)
            lg['length'] = _polyline_length(lg['pts'])
        return '; '.join(desc)

    def _route(self, found, edges, goal, variation=None, modes=None):
        seq, cost = found
        legs = []
        for u, v, eid in seq:
            e = edges[eid]
            pts = e.pts if e.a == u else e.pts[::-1]
            if e.length < 1e-9:
                continue
            if legs and legs[-1]['kind'] == e.kind and legs[-1]['area'] == e.area \
                    and legs[-1]['name'] == e.name:
                legs[-1]['pts'] += pts[1:]
                legs[-1]['length'] += e.length
            else:
                legs.append({'kind': e.kind, 'area': e.area, 'name': e.name,
                             'pts': list(pts), 'length': e.length})
        variant = ''
        if variation is not None and int(variation) > 0:      # 0 = the base route
            variant = self._vary(legs, int(variation), modes)
        # vertices; the segment ending at vertex i belongs to its (kind, area)
        verts = []
        for lg in legs:
            region_area = lg['area'] if lg['kind'] == AREA else -1
            for k, q in enumerate(lg['pts']):
                if verts and math.hypot(q[0] - verts[-1][0], q[1] - verts[-1][1]) < 1e-3:
                    continue
                verts.append((float(q[0]), float(q[1]), lg['kind'], region_area))
        verts = self._fillet(verts)
        poses, kinds, areas = [], [], []
        step = max(0.02, float(self.p.resample_m))
        for i, v in enumerate(verts):
            if i == 0:
                poses.append([v[0], v[1], 0.0])
                kinds.append(v[2])
                areas.append(v[3])
                continue
            p = verts[i - 1]
            L = math.hypot(v[0] - p[0], v[1] - p[1])
            n = max(1, int(math.ceil(L / step)))
            for k in range(1, n + 1):
                poses.append([p[0] + (v[0] - p[0]) * k / n, p[1] + (v[1] - p[1]) * k / n, 0.0])
                kinds.append(v[2])
                areas.append(v[3])
        if not poses:
            poses = [[float(goal[0]), float(goal[1]), 0.0]]
            kinds, areas = [AREA], [-1]
        if math.hypot(poses[-1][0] - goal[0], poses[-1][1] - goal[1]) < 0.05:
            poses[-1][0], poses[-1][1] = float(goal[0]), float(goal[1])   # snapped onto a path
        for i in range(len(poses) - 1):
            poses[i][2] = math.atan2(poses[i + 1][1] - poses[i][1], poses[i + 1][0] - poses[i][0])
        poses[-1][2] = float(goal[2]) if len(goal) > 2 else (poses[-2][2] if len(poses) > 1 else 0.0)
        mowing = {r.index for r in self.regions if not r.navigation}
        single = -1
        if legs and all(lg['kind'] == AREA for lg in legs) and \
                len({lg['area'] for lg in legs}) == 1 and legs[0]['area'] in mowing:
            single = legs[0]['area']
        for lg in legs:
            del lg['pts']
        return Route([tuple(p) for p in poses], kinds, areas, legs,
                     sum(lg['length'] for lg in legs),
                     sum(lg['length'] for lg in legs if lg['kind'] == PATH), cost, single,
                     variant)
