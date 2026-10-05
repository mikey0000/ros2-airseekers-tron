# Copyright 2026 michael
# SPDX-License-Identifier: Apache-2.0
"""Pure geometry: split one mower_coverage path into segments and sub-paths.

No ROS imports, so it is unit-testable on any host with plain pytest.

Input is the pose sequence of the ``nav_msgs/Path`` returned by
``/coverage/plan`` (``mower_interfaces/srv/PlanCoverage``), reduced to (x, y).
How ``mower_coverage_node`` lays out that path (``mower_coverage_node.cpp``):

* NOT densified. Every pose is a polygon vertex or a swath end point.
* Headland rings first, outermost first. Each ring is a closed loop whose LAST
  pose repeats the FIRST (``rotateToLongestEdgeMid`` re-closes it), so a ring
  ends exactly where it started.
* Then the swaths in serpentine order, each EXACTLY two poses (start, end).
* No connector poses. A U-turn between swaths is just the straight edge from
  one swath's end to the next one's start (op_width long, 90 deg off the swath).
* Optional transit-in / transit-out poses only when has_start / has_goal are
  set; this bridge never sets them.

A "distance > 0.6 m" rule cannot find swath boundaries on such a path: every
swath edge is itself metres long. So the primary rule is STRUCTURAL and exact:

1. Rings: from the current pose, the first later pose that returns to it (within
   ``closure_tol_m``) closes one ring. Repeat for ``ring_count`` rings.
2. Swaths: if exactly ``2 * swath_count`` poses remain, pair them up.

If the counts do not match the path (a future planner that densifies, or an
older/newer server), a HEURISTIC fallback is used instead. It is meant for
densified paths: a boundary at any edge longer than ``transit_gap_m``, at any
pose where the heading turns by more than ``turn_split_deg``, and at any short
connector edge across which the heading reverses by more than
``turn_split_deg`` (a boustrophedon U-turn).

Segments are then joined into drivable sub-paths: consecutive segments go into
the same sub-path when the gap from one's end to the next one's start is
``<= transit_gap_m`` (``mowgli_interfaces::coverage_geometry::kSegmentTransitGapM``)
and the straight connector does not cross a keep-out. A larger gap starts a new
sub-path, and the mission layer bridges it with a blade-off Nav2 transit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]

# Mirrors mowgli_interfaces/action/PlanCoverage Result constants.
SEGMENT_RING = 0
SEGMENT_SWATH = 1

# Mirrors mowgli_interfaces::coverage_geometry::kSegmentTransitGapM.
DEFAULT_TRANSIT_GAP_M = 0.6
DEFAULT_TURN_SPLIT_DEG = 150.0
DEFAULT_CLOSURE_TOL_M = 1e-3
_DUP_TOL_M = 1e-6

MODE_STRUCTURAL = 'structural'
MODE_HEURISTIC = 'heuristic'


@dataclass
class Segment:
    kind: int
    points: List[Point]

    @property
    def length(self) -> float:
        return path_length(self.points)


@dataclass
class SubPath:
    points: List[Point]
    segment_indices: List[int] = field(default_factory=list)

    @property
    def length(self) -> float:
        return path_length(self.points)


@dataclass
class SplitResult:
    segments: List[Segment]
    subpaths: List[SubPath]
    mode: str

    @property
    def ring_count(self) -> int:
        return sum(1 for s in self.segments if s.kind == SEGMENT_RING)

    @property
    def swath_count(self) -> int:
        return sum(1 for s in self.segments if s.kind == SEGMENT_SWATH)


# --------------------------------------------------------------------------
# Small geometry helpers
# --------------------------------------------------------------------------

def dist(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def path_length(points: Sequence[Point]) -> float:
    return sum(dist(points[i - 1], points[i]) for i in range(1, len(points)))


def heading(a: Point, b: Point) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


def angle_diff_deg(h1: float, h2: float) -> float:
    """Absolute heading change in degrees, in [0, 180]."""
    d = (h2 - h1 + math.pi) % (2.0 * math.pi) - math.pi
    return abs(math.degrees(d))


def is_closed(points: Sequence[Point], tol: float = DEFAULT_CLOSURE_TOL_M) -> bool:
    return len(points) >= 4 and dist(points[0], points[-1]) < tol


def _cross(o: Point, a: Point, b: Point) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def segments_cross(p1: Point, p2: Point, q1: Point, q2: Point, eps: float = 1e-9) -> bool:
    """Proper crossing of segments p1-p2 and q1-q2.

    Touching at an end point or running collinear does not count: a swath
    that ends exactly on the boundary (headland_passes = -1) must still be
    joinable to its neighbour.
    """
    d1 = _cross(q1, q2, p1)
    d2 = _cross(q1, q2, p2)
    d3 = _cross(p1, p2, q1)
    d4 = _cross(p1, p2, q2)
    return ((d1 > eps and d2 < -eps) or (d1 < -eps and d2 > eps)) and \
        ((d3 > eps and d4 < -eps) or (d3 < -eps and d4 > eps))


def point_in_polygon(p: Point, poly: Sequence[Point]) -> bool:
    inside = False
    n = len(poly)
    for i in range(n):
        a = poly[i]
        b = poly[(i + 1) % n]
        if (a[1] > p[1]) != (b[1] > p[1]):
            x = a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
            if x > p[0]:
                inside = not inside
    return inside


def connector_crosses(a: Point, b: Point, fences: Sequence[Sequence[Point]],
                      holes: Sequence[Sequence[Point]]) -> bool:
    """True if the straight connector a-b crosses a fence edge or enters a hole."""
    for poly in list(fences) + list(holes):
        n = len(poly)
        if n < 2:
            continue
        for i in range(n):
            if segments_cross(a, b, poly[i], poly[(i + 1) % n]):
                return True
    mid = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
    return any(len(h) >= 3 and point_in_polygon(mid, h) for h in holes)


# --------------------------------------------------------------------------
# Segmentation
# --------------------------------------------------------------------------

def _dedup(points: Sequence[Point]) -> List[Point]:
    out: List[Point] = []
    for p in points:
        p = (float(p[0]), float(p[1]))
        if out and dist(out[-1], p) < _DUP_TOL_M:
            continue
        out.append(p)
    return out


def detect_rings(points: Sequence[Point], max_rings: Optional[int] = None,
                 closure_tol_m: float = DEFAULT_CLOSURE_TOL_M) -> Tuple[List[List[Point]], int]:
    """Peel closed loops off the front of ``points``.

    Returns (rings, index of the first pose after the last ring).
    """
    rings: List[List[Point]] = []
    i = 0
    n = len(points)
    while i < n and (max_rings is None or len(rings) < max_rings):
        close = None
        for j in range(i + 3, n):
            if dist(points[i], points[j]) < closure_tol_m:
                close = j
                break
        if close is None:
            break
        rings.append(list(points[i:close + 1]))
        i = close + 1
    return rings, i


def split_heuristic(points: Sequence[Point],
                    transit_gap_m: float = DEFAULT_TRANSIT_GAP_M,
                    turn_split_deg: float = DEFAULT_TURN_SPLIT_DEG) -> List[List[Point]]:
    """Gap / heading-reversal rule for DENSIFIED open paths (see module doc)."""
    pts = list(points)
    n = len(pts)
    if n < 2:
        return []
    # cut[k] True -> edge k (pts[k] -> pts[k+1]) is not driven as part of a
    # segment (a gap or a U-turn connector).
    cut = [False] * (n - 1)
    split_at = [False] * n  # split at pose k, pose shared by both segments
    hdg = [heading(pts[k], pts[k + 1]) for k in range(n - 1)]
    for k in range(n - 1):
        if dist(pts[k], pts[k + 1]) > transit_gap_m:
            cut[k] = True
    for k in range(1, n - 1):
        if not cut[k - 1] and not cut[k] and angle_diff_deg(hdg[k - 1], hdg[k]) > turn_split_deg:
            split_at[k] = True
    for k in range(1, n - 2):
        if cut[k] or cut[k - 1] or cut[k + 1]:
            continue
        if angle_diff_deg(hdg[k - 1], hdg[k + 1]) > turn_split_deg \
                and angle_diff_deg(hdg[k - 1], hdg[k]) <= turn_split_deg \
                and angle_diff_deg(hdg[k], hdg[k + 1]) <= turn_split_deg:
            cut[k] = True
    out: List[List[Point]] = []
    cur: List[Point] = [pts[0]]
    for k in range(n - 1):
        if cut[k]:
            out.append(cur)
            cur = [pts[k + 1]]
            continue
        cur.append(pts[k + 1])
        if split_at[k + 1]:
            out.append(cur)
            cur = [pts[k + 1]]
    out.append(cur)
    return [s for s in out if len(s) >= 2]


def looks_like_sparse_swaths(points: Sequence[Point], parallel_tol_deg: float = 1.0) -> bool:
    """True if ``points`` pair up into parallel 2-pose swaths (planner layout)."""
    n = len(points)
    if n < 2 or n % 2:
        return False
    ref = None
    for i in range(0, n, 2):
        a, b = points[i], points[i + 1]
        if dist(a, b) < _DUP_TOL_M:
            return False
        h = heading(a, b)
        if ref is None:
            ref = h
            continue
        d = angle_diff_deg(ref, h)
        if min(d, 180.0 - d) > parallel_tol_deg:
            return False
    # A single densified straight line also "pairs up"; require the pairs not
    # to chain end-to-start along the same line.
    for i in range(2, n, 2):
        a, b = points[i - 1], points[i]
        if dist(a, b) < _DUP_TOL_M:
            return False
    return n == 2 or not _is_collinear_chain(points)


def _is_collinear_chain(points: Sequence[Point], tol_deg: float = 1.0) -> bool:
    h0 = heading(points[0], points[1])
    for i in range(1, len(points) - 1):
        d = angle_diff_deg(h0, heading(points[i], points[i + 1]))
        if d > tol_deg:
            return False
    return True


def split_segments(points: Sequence[Point],
                   ring_count: Optional[int] = None,
                   swath_count: Optional[int] = None,
                   transit_gap_m: float = DEFAULT_TRANSIT_GAP_M,
                   turn_split_deg: float = DEFAULT_TURN_SPLIT_DEG,
                   closure_tol_m: float = DEFAULT_CLOSURE_TOL_M) -> Tuple[List[Segment], str]:
    """Split the planner path into ring and swath segments.

    Returns (segments, mode) where mode is MODE_STRUCTURAL when the planner's
    ring/swath counts matched the pose layout exactly, else MODE_HEURISTIC.
    """
    pts = _dedup(points)
    if len(pts) < 2:
        return [], MODE_STRUCTURAL
    rings, rest_i = detect_rings(pts, ring_count, closure_tol_m)
    rest = pts[rest_i:]
    rings_ok = ring_count is None or len(rings) == ring_count
    if rings_ok and swath_count is not None and len(rest) == 2 * swath_count:
        segs = [Segment(SEGMENT_RING, r) for r in rings]
        segs += [Segment(SEGMENT_SWATH, [rest[2 * i], rest[2 * i + 1]])
                 for i in range(swath_count)]
        return segs, MODE_STRUCTURAL
    # Counts unknown or inconsistent: closures are still rings.
    rings, rest_i = detect_rings(pts, None, closure_tol_m)
    segs = [Segment(SEGMENT_RING, r) for r in rings]
    rest = pts[rest_i:]
    # Sparse swath layout (two poses per swath, all swaths parallel): pair up.
    if looks_like_sparse_swaths(rest):
        segs += [Segment(SEGMENT_SWATH, [rest[i], rest[i + 1]])
                 for i in range(0, len(rest), 2)]
        return segs, MODE_STRUCTURAL
    # Otherwise assume a densified path and use the gap / turn rule.
    for piece in split_heuristic(pts[rest_i:], transit_gap_m, turn_split_deg):
        kind = SEGMENT_RING if is_closed(piece, closure_tol_m) else SEGMENT_SWATH
        segs.append(Segment(kind, piece))
    return segs, MODE_HEURISTIC


# --------------------------------------------------------------------------
# Joining into drivable sub-paths
# --------------------------------------------------------------------------

def join_subpaths(segments: Sequence[Segment],
                  transit_gap_m: float = DEFAULT_TRANSIT_GAP_M,
                  fences: Sequence[Sequence[Point]] = (),
                  holes: Sequence[Sequence[Point]] = ()) -> List[SubPath]:
    """Join consecutive segments whose end-to-start gap is <= transit_gap_m.

    The join is a straight connector (the mower is diff-drive and pivots).
    A connector that crosses a fence edge (e.g. the outer boundary) or a hole
    also starts a new sub-path, so the blade-off transit routes around it.
    """
    subs: List[SubPath] = []
    for idx, seg in enumerate(segments):
        if len(seg.points) < 2:
            continue
        if subs:
            last = subs[-1].points[-1]
            first = seg.points[0]
            gap = dist(last, first)
            if gap <= transit_gap_m and not (
                    gap > _DUP_TOL_M and connector_crosses(last, first, fences, holes)):
                pts = seg.points[1:] if gap < _DUP_TOL_M else seg.points
                subs[-1].points.extend(pts)
                subs[-1].segment_indices.append(idx)
                continue
        subs.append(SubPath(list(seg.points), [idx]))
    return subs


def transit_gaps(subpaths: Sequence[SubPath]) -> List[float]:
    """Straight-line gap between consecutive sub-paths (blade-off transits)."""
    return [dist(subpaths[i - 1].points[-1], subpaths[i].points[0])
            for i in range(1, len(subpaths))]


# --------------------------------------------------------------------------
# Output shaping
# --------------------------------------------------------------------------

def densify(points: Sequence[Point], step_m: float) -> List[Point]:
    """Insert poses so no edge is longer than step_m. step_m <= 0: no-op."""
    pts = list(points)
    if step_m <= 0.0 or len(pts) < 2:
        return pts
    out: List[Point] = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        d = dist(a, b)
        n = max(1, int(math.ceil(d / step_m - 1e-9)))
        for k in range(1, n + 1):
            t = k / n
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    return out


def yaws(points: Sequence[Point]) -> List[float]:
    """Yaw per pose: towards the next pose; the last pose keeps the previous."""
    n = len(points)
    out: List[float] = []
    for i in range(n):
        if i + 1 < n:
            out.append(heading(points[i], points[i + 1]))
        elif i > 0:
            out.append(heading(points[i - 1], points[i]))
        else:
            out.append(0.0)
    return out


def swath_angle_deg(segments: Sequence[Segment]) -> Optional[float]:
    """Heading of the first swath, folded to [0, 180)."""
    for s in segments:
        if s.kind == SEGMENT_SWATH and len(s.points) >= 2:
            return math.degrees(heading(s.points[0], s.points[-1])) % 180.0
    return None


def plan(points: Sequence[Point],
         ring_count: Optional[int] = None,
         swath_count: Optional[int] = None,
         transit_gap_m: float = DEFAULT_TRANSIT_GAP_M,
         turn_split_deg: float = DEFAULT_TURN_SPLIT_DEG,
         fences: Sequence[Sequence[Point]] = (),
         holes: Sequence[Sequence[Point]] = ()) -> SplitResult:
    """Full pipeline: segment, then join into drivable sub-paths."""
    segs, mode = split_segments(points, ring_count, swath_count, transit_gap_m, turn_split_deg)
    subs = join_subpaths(segs, transit_gap_m, fences, holes)
    return SplitResult(segs, subs, mode)


def summary(result: SplitResult) -> str:
    mow = sum(s.length for s in result.segments)
    drive = sum(s.length for s in result.subpaths)
    gaps = transit_gaps(result.subpaths)
    ang = swath_angle_deg(result.segments)
    return ('%d ring(s) + %d swath(s) = %d segment(s) -> %d drivable sub-path(s); '
            'segments %.2f m, sub-paths %.2f m (incl. %.2f m connectors), '
            '%d transit(s) %.2f m; swath angle %s; split=%s') % (
        result.ring_count, result.swath_count, len(result.segments), len(result.subpaths),
        mow, drive, drive - mow, len(gaps), sum(gaps),
        'n/a' if ang is None else '%.1f deg' % ang, result.mode)
