# SPDX-License-Identifier: GPL-3.0-or-later
"""Boundary lines ("virtual walls", 2026-11-02): polylines the mower body must never cross.

ROS-free. A boundary line is drawn in the GUI along a drop-off or a flower-bed edge, inside or
across areas. It is stored in areas.dat next to the areas (same map frame, same datum stamp, see
``areas.format_areas_dat``) and enforced in two ways:

* the TRUE line (plus ``width_m``) is a hard rule: a body footprint that touches it trips the
  mission's hard boundary guard (``body_depth`` / ``body_hits``; reason names the line id);
* planners see it as a thin keepout polygon (``keepout_polygon``: the line buffered to
  ``width_m + boundary_line_keepout_m``): the keepout / nav masks, the route graph and the
  coverage holes, the same way no-go polygons are handled.

Wire format (GUI <-> map_server <-> mission), map metres::

    {"version": 1, "lines": [{"id": "bl-1", "name": "drop-off",
                              "points": [[x, y], ...], "width_m": 0.0}, ...]}
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

Point = Tuple[float, float]

VERSION = 1
WIDTH_MAX_M = 2.0
NAME_MAX = 64
ID_MAX = 64
POINTS_MAX = 500
# consecutive vertices closer than this are one vertex
VERTEX_EPS_M = 0.01
# obstacle name of the planner keepout of a line (route graph / masks / coverage holes)
KEEPOUT_NAME_PREFIX = 'boundary line '
# Default boundary_line_keepout_m: the planner band is width_m + this, i.e. the body half
# width (0.235) + 0.05 on each side of the line. The coverage planner's hard edge layout does
# not grow holes (laps run 0.45 from them) and pivots at lap corners: with a 0.30 band the
# 0.645 m nose-corner sweep reached the line (test_coverage_planning BoundaryLineBody). The
# masks are robot-centre limits, so this band is also what keeps the body off the line there.
DEFAULT_KEEPOUT_M = 0.57


@dataclass
class BoundaryLine:
    id: str
    points: List[Point] = field(default_factory=list)
    name: str = ''
    width_m: float = 0.0

    def label(self) -> str:
        """'id "name"' (or just the id) for log lines and trip reasons."""
        return '%s "%s"' % (self.id, self.name) if self.name else self.id


def normalise_points(points: Iterable[Sequence[float]]) -> List[Point]:
    """x/y of each point, consecutive duplicates (within VERTEX_EPS_M) dropped. A line is
    open: a closing vertex equal to the first one is kept (a closed fence is allowed)."""
    out: List[Point] = []
    for p in points:
        q = (float(p[0]), float(p[1]))
        if out and math.hypot(q[0] - out[-1][0], q[1] - out[-1][1]) <= VERTEX_EPS_M:
            continue
        out.append(q)
    return out


def _clean_text(s, limit) -> str:
    return str(s if s is not None else '').replace('\r', ' ').replace('\n', ' ').strip()[:limit]


def validate(lines: Sequence[BoundaryLine]) -> Tuple[bool, str]:
    """(ok, message): every line has >= 2 distinct finite points and 0 <= width_m <= 2 m."""
    for k, ln in enumerate(lines):
        what = 'boundary line %d (%s)' % (k, ln.id or 'no id')
        if len(ln.points) < 2:
            return False, '%s needs at least 2 distinct points' % what
        if len(ln.points) > POINTS_MAX:
            return False, '%s has %d points (max %d)' % (what, len(ln.points), POINTS_MAX)
        if not all(math.isfinite(x) and math.isfinite(y) for x, y in ln.points):
            return False, '%s has a non-finite point' % what
        if not (math.isfinite(ln.width_m) and 0.0 <= ln.width_m <= WIDTH_MAX_M):
            return False, '%s width_m %r outside 0..%.1f' % (what, ln.width_m, WIDTH_MAX_M)
    return True, 'ok'


def assign_ids(lines: Sequence[BoundaryLine]) -> List[BoundaryLine]:
    """Copies with a unique non-empty id each: a missing or repeated id becomes 'line-N'."""
    seen, out, n = set(), [], 1
    for ln in lines:
        lid = ln.id
        if not lid or lid in seen:
            while 'line-%d' % n in seen or any(o.id == 'line-%d' % n for o in lines):
                n += 1
            lid = 'line-%d' % n
        seen.add(lid)
        out.append(BoundaryLine(lid, list(ln.points), ln.name, ln.width_m))
    return out


def parse_json(text: str) -> Tuple[bool, str, List[BoundaryLine]]:
    """Parse the wire format (or a bare list of lines). '' / '{}' = no lines.
    Returns (ok, message, lines); ids are made unique, the set is validated."""
    try:
        obj = json.loads(text) if text and text.strip() else {}
    except ValueError as exc:
        return False, 'bad JSON: %s' % exc, []
    if isinstance(obj, dict):
        version = obj.get('version', VERSION)
        if not isinstance(version, int) or version < 1:
            return False, 'bad version %r' % (version,), []
        if version > VERSION:
            return False, 'boundary lines version %d is newer than this map server (%d)' \
                % (version, VERSION), []
        raw = obj.get('lines') or []
    elif isinstance(obj, list):
        raw = obj
    else:
        return False, 'expected a JSON object or list', []
    if not isinstance(raw, list):
        return False, '"lines" must be a list', []
    lines = []
    for k, d in enumerate(raw):
        if not isinstance(d, dict):
            return False, 'boundary line %d: expected an object' % k, []
        try:
            pts = normalise_points(d.get('points') or [])
            width = float(d.get('width_m', 0.0) or 0.0)
        except (TypeError, ValueError, IndexError) as exc:
            return False, 'boundary line %d: bad points/width_m: %s' % (k, exc), []
        lines.append(BoundaryLine(_clean_text(d.get('id'), ID_MAX), pts,
                                  _clean_text(d.get('name'), NAME_MAX), width))
    lines = assign_ids(lines)
    ok, msg = validate(lines)
    return ok, msg, (lines if ok else [])


def to_dict(lines: Sequence[BoundaryLine]) -> dict:
    return {'version': VERSION, 'lines': [
        {'id': ln.id, 'name': ln.name,
         'points': [[round(x, 4), round(y, 4)] for x, y in ln.points],
         'width_m': round(float(ln.width_m), 4)} for ln in lines]}


def to_json(lines: Sequence[BoundaryLine]) -> str:
    return json.dumps(to_dict(lines))


# ---------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------

def segments(line: BoundaryLine):
    pts = line.points
    return list(zip(pts[:-1], pts[1:]))


def _pt_seg(px, py, a, b) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / L2))
    return math.hypot(px - (a[0] + t * dx), py - (a[1] + t * dy))


def nearest_point(line: BoundaryLine, x: float, y: float) -> Tuple[float, float, float]:
    """(qx, qy, d): the point of the centreline nearest to (x, y)."""
    best = (line.points[0][0], line.points[0][1], math.inf)
    for a, b in segments(line):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / L2))
        qx, qy = a[0] + t * dx, a[1] + t * dy
        d = math.hypot(x - qx, y - qy)
        if d < best[2]:
            best = (qx, qy, d)
    return best


def body_corners(x: float, y: float, yaw: float, front: float = 0.60, rear: float = 0.10,
                 half: float = 0.235) -> List[Point]:
    """The footprint box (x forward from base_link, y left) in the map frame, CCW."""
    c, s = math.cos(yaw), math.sin(yaw)
    local = [(front, -half), (front, half), (-rear, half), (-rear, -half)]
    return [(x + c * bx - s * by, y + s * bx + c * by) for bx, by in local]


def _proj(pts, ax, ay):
    v = [p[0] * ax + p[1] * ay for p in pts]
    return min(v), max(v)


def segment_polygon_depth(a: Point, b: Point, poly: Sequence[Point]) -> float:
    """Signed overlap of the segment a-b with the CONVEX polygon ``poly``: > 0 = they overlap
    and this is the smallest translation that separates them (separating axis theorem);
    <= 0 = disjoint, minus the distance between them."""
    axes = []
    n = len(poly)
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        ex, ey = q[0] - p[0], q[1] - p[1]
        L = math.hypot(ex, ey)
        if L > 1e-12:
            axes.append((-ey / L, ex / L))
    ex, ey = b[0] - a[0], b[1] - a[1]
    L = math.hypot(ex, ey)
    if L > 1e-12:
        axes.append((-ey / L, ex / L))
        axes.append((ex / L, ey / L))
    depth = math.inf
    for ax, ay in axes:
        s0, s1 = _proj((a, b), ax, ay)
        p0, p1 = _proj(poly, ax, ay)
        ov = min(s1 - p0, p1 - s0)
        if ov <= 0.0:
            # disjoint: exact distance between a segment and a convex polygon is reached at a
            # vertex of one and an edge of the other
            d = min([_pt_seg(c[0], c[1], a, b) for c in poly] +
                    [_pt_seg(e[0], e[1], poly[i], poly[(i + 1) % n])
                     for e in (a, b) for i in range(n)])
            return -d
        depth = min(depth, ov)
    return depth


def body_depth(line: BoundaryLine, corners: Sequence[Point]) -> float:
    """How far the body (convex footprint ``corners``) reaches into the line band (centreline
    +- width_m / 2): > 0 = crossing / touching (the separating distance), <= 0 = clear by that
    much. The worst segment of the polyline counts."""
    half = 0.5 * max(0.0, float(line.width_m))
    best = -math.inf
    for a, b in segments(line):
        best = max(best, segment_polygon_depth(a, b, corners) + half)
    return best


def body_hits(lines: Sequence[BoundaryLine], pose, front: float = 0.60, rear: float = 0.10,
              half: float = 0.235, min_depth: float = 0.0):
    """[(depth, line)] of every line the footprint at ``pose`` (x, y, yaw) reaches deeper than
    ``min_depth`` into, deepest first."""
    if pose is None or len(pose) < 3 or not lines:
        return []
    corners = body_corners(float(pose[0]), float(pose[1]), float(pose[2]), front, rear, half)
    out = []
    for ln in lines:
        if len(ln.points) < 2:
            continue
        d = body_depth(ln, corners)
        if d > min_depth:
            out.append((d, ln))
    out.sort(key=lambda t: -t[0])
    return out


def clearance(lines: Sequence[BoundaryLine], pose, front: float = 0.60, rear: float = 0.10,
              half: float = 0.235) -> float:
    """Smallest body-to-line-band distance at ``pose`` (negative = crossing by that much);
    inf without lines."""
    if pose is None or not lines:
        return math.inf
    corners = body_corners(float(pose[0]), float(pose[1]), float(pose[2]), front, rear, half)
    return min((-body_depth(ln, corners) for ln in lines if len(ln.points) >= 2),
               default=math.inf)


def keepout_polygon(line: BoundaryLine, keepout_m: float) -> List[Point]:
    """The planner's thin keepout polygon: the line buffered to a band of total width
    ``width_m + keepout_m`` (square caps, mitred joints; areas.buffer_polyline)."""
    from mower_map.areas import buffer_polyline
    w = max(0.0, float(line.width_m)) + max(0.0, float(keepout_m))
    if w <= 0.0 or len(line.points) < 2:
        return []
    return buffer_polyline(list(line.points), 0.5 * w)


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _segments_cross(a, b, c, d) -> bool:
    d1, d2 = _cross(c, d, a), _cross(c, d, b)
    d3, d4 = _cross(a, b, c), _cross(a, b, d)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def _pip(x, y, poly) -> bool:
    inside, j = False, len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def polygon_touches(poly_a: Sequence[Point], poly_b: Sequence[Point]) -> bool:
    """True when two simple polygons overlap (a vertex inside the other or edges crossing)."""
    if len(poly_a) < 3 or len(poly_b) < 3:
        return False
    if any(_pip(p[0], p[1], poly_b) for p in poly_a) or \
            any(_pip(p[0], p[1], poly_a) for p in poly_b):
        return True
    na, nb = len(poly_a), len(poly_b)
    return any(_segments_cross(poly_a[i], poly_a[(i + 1) % na], poly_b[j], poly_b[(j + 1) % nb])
               for i in range(na) for j in range(nb))


def keepouts_for_polygon(lines: Sequence[BoundaryLine], poly: Sequence[Point],
                         keepout_m: float) -> List[Tuple[BoundaryLine, List[Point]]]:
    """[(line, keepout polygon)] of the lines whose keepout overlaps ``poly`` (an area)."""
    out = []
    for ln in lines:
        k = keepout_polygon(ln, keepout_m)
        if len(k) >= 3 and polygon_touches(k, poly):
            out.append((ln, k))
    return out


def from_dicts(raw) -> Optional[List[BoundaryLine]]:
    """Lines from an already-decoded wire object (dict or list); None when malformed."""
    ok, _msg, lines = parse_json(json.dumps(raw))
    return lines if ok else None
