# SPDX-License-Identifier: GPL-3.0-or-later
"""Boundary smoothing for recorded areas: an inner polygon hull with a distance tolerance.

A recorded area boundary zigzags by 5-20 cm (RTK noise plus the operator's steering). Lap 0
runs on the line and the inner laps are offsets of it, so every zig is driven in every lap.
This removes the zigs that are smaller than ``tolerance`` and nothing else.

Algorithm: the inner polygon hull of JTS ``PolygonHullSimplifier`` (GEOS
``GEOSPolygonHullSimplifyMode``): "Inner hulls are contained by the input geometry", "the hull
vertices are a subset of the input vertices", and the result is valid. JTS stops at a
vertex-number fraction or an area-delta ratio. Here the stop is a distance, the one
Douglas-Peucker uses. The loop removes the convex vertex whose corner triangle has the
smallest area (the Visvalingam-Whyatt elimination order), only if the triangle holds no
other vertex (an "ear"). It removes a vertex only while every original vertex of the span
the new edge replaces lies within ``tolerance`` of that edge. GEOS has no distance mode and
shapely 2.0 does not expose the hull, hence this short port. Sources:
https://locationtech.github.io/jts/javadoc/org/locationtech/jts/simplify/PolygonHullSimplifier.html
https://libgeos.org/doxygen/geos__c_8h.html (GEOSPolygonHullSimplifyMode_r)

Guarantees, for a simple input ring (``smooth_ring`` returns the input unchanged otherwise):

- inward-or-on: removing a convex ear cuts off a triangle that lies inside the polygon, so the
  result is contained in the recorded polygon. The line never moves outward.
- bounded: every point of the recorded line is within ``tolerance`` of the result and every
  point of the result is within ``tolerance`` of the recorded line (the distance from a
  segment is convex, so the vertex check bounds whole edges).
- corners kept: the result's vertices are a subset of the recorded vertices, unmoved. A
  reflex vertex is never removed while it is reflex (only once removing its neighbours has
  already moved the line inward past it). A convex corner goes only if its apex is within
  ``tolerance`` of the chord of its neighbours, so a real corner is at most bevelled by the
  tolerance. Nothing is rounded into an arc.
- valid: removing an ear of a simple polygon leaves a simple polygon with the same winding.
- the area can only shrink; ``SmoothResult`` reports by how much.

Pure Python, no ROS and no shapely (``mower_map.areas`` is ROS-free too).
"""

import heapq
import math
from typing import List, NamedTuple, Sequence, Tuple

Point = Tuple[float, float]

# The default tolerance (m): mission.yaml record_smooth_tolerance_m and the smooth_areas CLI.
# Lawn-edge zigs measured on the owner's recordings are 5-20 cm; 8 cm removes most of them
# and stays under half a swath (op width 0.18 m). docs/analysis/2026-11-09_boundary_smoothing.md
DEFAULT_TOLERANCE_M = 0.08


class SmoothResult(NamedTuple):
    ring: List[Point]          # the smoothed ring (the input when nothing was removed)
    max_deviation_m: float     # largest distance of a recorded vertex from the result
    area_before_m2: float
    area_after_m2: float
    removed: int               # vertices removed
    reason: str                # '' when smoothed or nothing to remove; why it was skipped

    @property
    def area_change_m2(self) -> float:
        return self.area_after_m2 - self.area_before_m2


def _cross(o: Point, a: Point, b: Point) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def signed_area(ring: Sequence[Point]) -> float:
    """Shoelace area, positive for a counter-clockwise ring."""
    n = len(ring)
    return 0.5 * sum(ring[i][0] * ring[(i + 1) % n][1] - ring[(i + 1) % n][0] * ring[i][1]
                     for i in range(n))


def segment_distance(p: Point, a: Point, b: Point) -> float:
    """Distance from ``p`` to the segment ``a``-``b``."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    len_sq = dx * dx + dy * dy
    t = 0.0 if len_sq < 1e-18 else max(0.0, min(1.0, ((p[0] - a[0]) * dx
                                                      + (p[1] - a[1]) * dy) / len_sq))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    """Closed segments a-b and c-d share a point (touching counts)."""
    d1, d2 = _cross(c, d, a), _cross(c, d, b)
    d3, d4 = _cross(a, b, c), _cross(a, b, d)
    if ((d1 > 0) != (d2 > 0) and d1 != 0 and d2 != 0
            and (d3 > 0) != (d4 > 0) and d3 != 0 and d4 != 0):
        return True

    def on(p, q, r):  # r on segment p-q, given collinear
        return (min(p[0], q[0]) <= r[0] <= max(p[0], q[0])
                and min(p[1], q[1]) <= r[1] <= max(p[1], q[1]))
    return ((d1 == 0 and on(c, d, a)) or (d2 == 0 and on(c, d, b))
            or (d3 == 0 and on(a, b, c)) or (d4 == 0 and on(a, b, d)))


def ring_is_simple(ring: Sequence[Point]) -> bool:
    """No two non-adjacent edges of the closed ring touch, and no edge has zero length."""
    n = len(ring)
    if n < 3:
        return False
    for i in range(n):
        if ring[i] == ring[(i + 1) % n]:
            return False
    for i in range(n):
        a, b = ring[i], ring[(i + 1) % n]
        for j in range(i + 1, n):
            if j == i + 1 or (i == 0 and j == n - 1):
                continue                     # adjacent edges share a vertex by design
            if _segments_intersect(a, b, ring[j], ring[(j + 1) % n]):
                return False
    return True


def _in_triangle(p: Point, a: Point, b: Point, c: Point) -> bool:
    """``p`` inside or on the counter-clockwise triangle a, b, c. The bounding box keeps a
    flat (collinear) triangle from claiming every point of its line."""
    if not (min(a[0], b[0], c[0]) <= p[0] <= max(a[0], b[0], c[0])
            and min(a[1], b[1], c[1]) <= p[1] <= max(a[1], b[1], c[1])):
        return False
    return _cross(a, b, p) >= 0.0 and _cross(b, c, p) >= 0.0 and _cross(c, a, p) >= 0.0


def inner_hull_simplify(ring: Sequence[Point], tolerance: float) -> List[int]:
    """Indices (into ``ring``, in order) of the vertices the inner hull keeps.

    ``ring`` must be simple, without a repeated closing vertex. See the module docstring."""
    pts = [(float(p[0]), float(p[1])) for p in ring]
    n = len(pts)
    if n <= 3 or tolerance <= 0.0:
        return list(range(n))
    ccw = signed_area(pts) > 0.0
    if not ccw:                              # work counter-clockwise: interior on the left
        pts = pts[::-1]
    prv = [(i - 1) % n for i in range(n)]
    nxt = [(i + 1) % n for i in range(n)]
    alive = [True] * n
    count = n

    def deviation(i):
        a, b = prv[i], nxt[i]
        worst, j = 0.0, (a + 1) % n
        while j != b:
            worst = max(worst, segment_distance(pts[j], pts[a], pts[b]))
            j = (j + 1) % n
        return worst

    def removable(i):
        """Corner triangle area when vertex i may go, else None."""
        a, b = prv[i], nxt[i]
        turn = _cross(pts[a], pts[i], pts[b])
        if turn < 0.0:                       # reflex: removing it would push the line out
            return None
        if deviation(i) > tolerance:
            return None
        for j in range(n):                   # an ear: no other vertex in the triangle
            if alive[j] and j != a and j != i and j != b and \
                    _in_triangle(pts[j], pts[a], pts[i], pts[b]):
                return None
        return 0.5 * turn

    # Removing a vertex can only empty other triangles, so a vertex refused for holding a
    # vertex may become removable later: repeat passes until one removes nothing.
    changed = True
    while changed and count > 3:
        changed = False
        heap = []
        for i in range(n):
            if alive[i]:
                k = removable(i)
                if k is not None:
                    heapq.heappush(heap, (k, i, prv[i], nxt[i]))
        while heap and count > 3:
            _, i, a, b = heapq.heappop(heap)
            if not alive[i]:
                continue
            if prv[i] != a or nxt[i] != b:   # neighbours changed since it was queued
                k = removable(i)
                if k is not None:
                    heapq.heappush(heap, (k, i, prv[i], nxt[i]))
                continue
            if removable(i) is None:         # an ear test can fail after other removals
                continue
            alive[i] = False
            count -= 1
            changed = True
            nxt[a], prv[b] = b, a
            for j in (a, b):
                k = removable(j)
                if k is not None:
                    heapq.heappush(heap, (k, j, prv[j], nxt[j]))
    keep = [i for i in range(n) if alive[i]]
    if not ccw:
        keep = sorted(n - 1 - i for i in keep)
    return keep


def max_deviation(ring: Sequence[Point], keep: Sequence[int]) -> float:
    """Largest distance of a vertex of ``ring`` from the edge of the kept ring spanning it."""
    n, m = len(ring), len(keep)
    worst = 0.0
    for k in range(m):
        a, b = keep[k], keep[(k + 1) % m]
        j = (a + 1) % n
        while j != b:
            worst = max(worst, segment_distance(ring[j], ring[a], ring[b]))
            j = (j + 1) % n
    return worst


def _clean(ring: Sequence[Point]) -> List[Point]:
    pts = []
    for p in ring:
        q = (float(p[0]), float(p[1]))
        if not pts or q != pts[-1]:
            pts.append(q)
    if len(pts) >= 2 and pts[0] == pts[-1]:
        pts.pop()
    return pts


def smooth_ring(ring: Sequence[Point], tolerance: float = DEFAULT_TOLERANCE_M) -> SmoothResult:
    """Smooth one area boundary (implicitly closed ring). Pure: the input is not changed."""
    pts = _clean(ring)
    area = abs(signed_area(pts)) if len(pts) >= 3 else 0.0
    if tolerance <= 0.0:
        return SmoothResult(pts, 0.0, area, area, 0, 'tolerance is 0 (off)')
    if len(pts) <= 3:
        return SmoothResult(pts, 0.0, area, area, 0, '')
    if not ring_is_simple(pts):
        return SmoothResult(pts, 0.0, area, area, 0, 'the ring crosses itself: left as recorded')
    keep = inner_hull_simplify(pts, tolerance)
    out = [pts[i] for i in keep]
    return SmoothResult(out, max_deviation(pts, keep), area, abs(signed_area(out)),
                        len(pts) - len(out), '')


def zigzag_m(ring: Sequence[Point]) -> List[float]:
    """Per vertex: distance from the chord of its two neighbours (the wiggle metric)."""
    n = len(ring)
    out = []
    for i in range(n):
        a, b, c = ring[i - 1], ring[i], ring[(i + 1) % n]
        L = math.hypot(c[0] - a[0], c[1] - a[1])
        out.append(abs(_cross(a, c, b)) / L if L > 1e-9 else 0.0)
    return out


def resample_ring(ring: Sequence[Point], step: float) -> List[Point]:
    """Points every ~``step`` metres of arc length around the closed ring."""
    n = len(ring)
    lengths = [math.hypot(ring[(i + 1) % n][0] - ring[i][0], ring[(i + 1) % n][1] - ring[i][1])
               for i in range(n)]
    total = sum(lengths)
    k = max(3, int(round(total / step)))
    out, acc, t = [], 0.0, 0
    for i in range(n):
        a, b, seg = ring[i], ring[(i + 1) % n], lengths[i]
        while t < k and t * total / k <= acc + seg:
            f = (t * total / k - acc) / seg if seg > 0 else 0.0
            out.append((a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f))
            t += 1
        acc += seg
    return out


def wiggle_m(ring: Sequence[Point], step: float = 0.5) -> List[float]:
    """``zigzag_m`` of the ring resampled every ``step`` m: the same scale before and after
    smoothing (``zigzag_m`` on the vertices grows when vertices are removed, because the
    chords get longer)."""
    return zigzag_m(resample_ring(ring, step))


def excess_turning_deg(ring: Sequence[Point]) -> float:
    """Sum of |heading change| around the ring minus 360 deg: the back-and-forth turning a
    lap on this line drives (0 for a convex ring)."""
    n = len(ring)
    s = 0.0
    for i in range(n):
        a, b, c = ring[i - 1], ring[i], ring[(i + 1) % n]
        t = math.atan2(c[1] - b[1], c[0] - b[0]) - math.atan2(b[1] - a[1], b[0] - a[0])
        s += abs(math.atan2(math.sin(t), math.cos(t)))
    return max(0.0, math.degrees(s) - 360.0)
