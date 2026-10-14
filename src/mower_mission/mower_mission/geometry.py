# Copyright 2026 Airseekers Tron ROS 2 port contributors
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Douglas-Peucker and polygon helpers ported from MowgliNext
# mowgli_behavior/src/recording_nodes.cpp (GPL-3.0).
"""Pure-Python 2-D geometry for the mission layer (no ROS imports).

Points are ``(x, y)`` tuples; path poses are ``(x, y, yaw)`` tuples.
"""

import math


def dist(a, b):
    """Euclidean distance between the x/y parts of two points or poses."""
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def path_length(poses):
    """Length of a polyline (sum of segment lengths)."""
    return sum(dist(poses[i], poses[i + 1]) for i in range(len(poses) - 1))


def perpendicular_distance(p, a, b):
    """Distance from ``p`` to the infinite line through ``a`` and ``b``.

    A degenerate line (``a == b``) returns the distance to ``a``, as upstream.
    """
    dx = float(b[0]) - float(a[0])
    dy = float(b[1]) - float(a[1])
    len_sq = dx * dx + dy * dy
    if len_sq < 1e-12:
        return dist(p, a)
    cross = abs(dx * (float(a[1]) - float(p[1])) - (float(a[0]) - float(p[0])) * dy)
    return cross / math.sqrt(len_sq)


def douglas_peucker(points, tolerance):
    """Douglas-Peucker simplification of an open polyline (endpoints kept).

    Iterative (explicit stack) so a long recording cannot hit Python's
    recursion limit; otherwise identical to upstream ``dp_recursive``.
    """
    points = list(points)
    n = len(points)
    if n < 3:
        return points
    keep = [False] * n
    keep[0] = keep[n - 1] = True
    stack = [(0, n - 1)]
    while stack:
        start, end = stack.pop()
        if end <= start + 1:
            continue
        max_d, max_i = 0.0, start
        for i in range(start + 1, end):
            d = perpendicular_distance(points[i], points[start], points[end])
            if d > max_d:
                max_d, max_i = d, i
        if max_d > tolerance:
            keep[max_i] = True
            stack.append((start, max_i))
            stack.append((max_i, end))
    return [p for p, k in zip(points, keep) if k]


def polygon_area(points):
    """Absolute shoelace area of an implicitly closed ring."""
    n = len(points)
    if n < 3:
        return 0.0
    s = 0.0
    for i in range(n):
        j = (i + 1) % n
        s += float(points[i][0]) * float(points[j][1]) - float(points[j][0]) * float(points[i][1])
    return abs(s) / 2.0


def _closest_on_segment(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    len_sq = dx * dx + dy * dy
    t = 0.0 if len_sq < 1e-12 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / len_sq))
    return (ax + t * dx, ay + t * dy)


def trim_closure_overshoot(points, radius, max_tail_fraction=0.25):
    """Cut a recorded loop where it comes back past its start.

    Looks at the segments in the last part of the drive (from half the track
    length on) for the point closest to ``points[0]``. When that point is within
    ``radius`` and the part of the drive after it (the overshoot) is at most
    ``max_tail_fraction`` of the track length, the drive ends there instead.
    Tracks that never return near the start are returned unchanged.
    """
    pts = [(float(p[0]), float(p[1])) for p in points]
    n = len(pts)
    if n < 4 or radius <= 0:
        return pts
    cum = cumulative_lengths(pts)
    total = cum[-1]
    if total <= 0:
        return pts
    start = pts[0]
    best = None
    for i in range(n - 1):
        if cum[i + 1] < 0.5 * total:
            continue
        c = _closest_on_segment(start, pts[i], pts[i + 1])
        d = dist(c, start)
        if best is None or d < best[0] - 1e-12:
            best = (d, i, c)
    if best is None or best[0] > radius:
        return pts
    _, i, c = best
    tail = dist(pts[i], c)
    tail = (total - cum[i]) - tail
    if tail <= 1e-9 or tail > max_tail_fraction * total:
        return pts
    out = pts[:i + 1]
    if dist(out[-1], c) > 1e-9:
        out.append(c)
    return out


def simplify_ring(points, tolerance, close_eps=None, close_radius=None):
    """Turn a recorded drive into a closed polygon ring.

    The track is simplified as a *closed* loop: the first point is appended
    so the closing edge takes part in the simplification, then the duplicate
    closing vertex is dropped again (``geometry_msgs/Polygon`` is implicitly
    closed; map_server_node does not want a repeated first vertex).  A track
    that already ends within ``close_eps`` of its start loses that last point
    first so it does not produce a zero-length closing edge.

    With ``close_radius`` the drive is first cut where it passes back by its
    start (``trim_closure_overshoot``): an operator who drives a little past
    the starting point before pressing finish would otherwise leave a sliver
    spike where the ring closes back over the already-driven track.
    """
    pts = [(float(p[0]), float(p[1])) for p in points]
    if close_eps is None:
        close_eps = tolerance
    if close_radius is not None:
        pts = trim_closure_overshoot(pts, close_radius)
    if len(pts) >= 2 and dist(pts[0], pts[-1]) <= close_eps:
        pts = pts[:-1]
    if len(pts) < 3:
        return pts
    ring = douglas_peucker(pts + [pts[0]], tolerance)
    if len(ring) >= 2 and dist(ring[0], ring[-1]) < 1e-9:
        ring = ring[:-1]
    return ring


def nearest_index(poses, point, start=0, max_arc=None):
    """Index (>= ``start``) of the pose closest to ``point``; ``start`` if empty.

    With ``max_arc`` only poses within that path length after ``start`` are
    considered, so progress along a path with adjacent laps (headland rings
    0.2 m apart) cannot jump onto a later lap that happens to be closer.
    """
    best_i, best_d = start, float('inf')
    arc = 0.0
    for i in range(start, len(poses)):
        if i > start:
            arc += dist(poses[i - 1], poses[i])
            if max_arc is not None and arc > max_arc:
                break
        d = dist(poses[i], point)
        if d < best_d:
            best_i, best_d = i, d
    return best_i


def cumulative_lengths(poses):
    """``out[i]`` = path length from pose 0 to pose ``i``."""
    out, acc = [0.0], 0.0
    for i in range(1, len(poses)):
        acc += dist(poses[i - 1], poses[i])
        out.append(acc)
    return out


def advance_index(poses, i, distance):
    """First index after ``i`` at least ``distance`` metres along the path
    (the last index when the remainder is shorter)."""
    acc = 0.0
    for k in range(i, len(poses) - 1):
        acc += dist(poses[k], poses[k + 1])
        if acc >= distance:
            return k + 1
    return max(i, len(poses) - 1)


def chunk_end(poses, start, max_len, clearance):
    """End index of a FollowPath chunk that starts at ``start``.

    A goal checker that only compares the robot with the goal's final pose
    (Jazzy's SimpleGoalChecker) declares success as soon as the robot passes
    near that pose, and a coverage path often passes near its own end (closed
    headland rings, a last swath ending where the ring began).  So the chunk
    end ``e`` is the furthest index within ``max_len`` metres of path whose pose
    stays more than ``clearance`` away from every pose of the chunk that is more
    than ``2 * clearance`` of path before it.  ``max_len <= 0`` disables chunking.
    """
    last = len(poses) - 1
    if max_len <= 0 or start >= last:
        return last
    cum = [0.0]
    best = None
    for e in range(start + 1, last + 1):
        cum.append(cum[-1] + dist(poses[e - 1], poses[e]))
        if cum[-1] > max_len and best is not None:
            break
        ok = True
        for j in range(start, e):
            if cum[e - start] - cum[j - start] < 2.0 * clearance:
                break                       # the poses right before e
            if dist(poses[j], poses[e]) <= clearance:
                ok = False
                break
        if ok:
            best = e
    return best if best is not None else min(start + 1, last)


def needs_transit(pose, target, transit_gap):
    """Blade-off transit decision of upstream ``FollowStrip::sendCurrentSwath``.

    No pose known -> transit (take the safe path).  Otherwise transit when
    the target is farther than ``transit_gap`` metres.
    """
    if pose is None:
        return True
    return dist(pose, target) > transit_gap


def heading_of(poses, i):
    """Yaw to use for pose ``i``: its own yaw, or the direction to the next pose
    when the planner left orientation empty (yaw exactly 0 and a next pose exists)."""
    yaw = float(poses[i][2]) if len(poses[i]) > 2 else 0.0
    if yaw == 0.0 and i + 1 < len(poses):
        dx = poses[i + 1][0] - poses[i][0]
        dy = poses[i + 1][1] - poses[i][1]
        if abs(dx) + abs(dy) > 1e-9:
            return math.atan2(dy, dx)
    return yaw


def is_reverse_subpath(poses):
    """True when the sub-path is driven BACKWARDS: the coverage bridge gives a
    reverse turn leg poses that face against the travel (yaw + pi)."""
    for i in range(len(poses) - 1):
        if len(poses[i]) < 3:
            return False
        dx = poses[i + 1][0] - poses[i][0]
        dy = poses[i + 1][1] - poses[i][1]
        if abs(dx) + abs(dy) < 1e-6:
            continue
        yaw = float(poses[i][2])
        return math.cos(yaw) * dx + math.sin(yaw) * dy < 0.0
    return False


def plan_fingerprint(subpaths):
    """64-bit FNV-1a over the millimetre-rounded plan geometry; never 0.

    Used, like upstream ``hashPlanGeometry``, as the staleness key of a
    persisted resume cursor: a re-plan with different geometry discards it.
    """
    h = 0xcbf29ce484222325
    for sp in subpaths:
        for p in sp:
            for v in (int(round(p[0] * 1000.0)), int(round(p[1] * 1000.0))):
                for byte in (v & 0xFFFFFFFFFFFFFFFF).to_bytes(8, 'little'):
                    h ^= byte
                    h = (h * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
        h ^= 0xFF
        h = (h * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
    return h or 1


def point_in_polygon(point, poly):
    """Even-odd rule; ``poly`` is a ring of (x, y) (closing vertex optional)."""
    x, y = float(point[0]), float(point[1])
    inside = False
    n = len(poly)
    for k in range(n):
        x1, y1 = float(poly[k][0]), float(poly[k][1])
        x2, y2 = float(poly[(k + 1) % n][0]), float(poly[(k + 1) % n][1])
        if (y1 > y) != (y2 > y):
            if x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
                inside = not inside
    return inside


def distance_to_ring(point, poly):
    """Shortest distance from ``point`` to the edges of ring ``poly``."""
    best = float('inf')
    n = len(poly)
    for k in range(n):
        a, b = poly[k], poly[(k + 1) % n]
        ax, ay, bx, by = float(a[0]), float(a[1]), float(b[0]), float(b[1])
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 <= 0 else max(0.0, min(1.0, ((point[0] - ax) * dx + (point[1] - ay) * dy)
                                             / l2))
        best = min(best, math.hypot(point[0] - (ax + t * dx), point[1] - (ay + t * dy)))
    return best


def inside_area(point, area, leave_m=0.0):
    """``point`` lies in the area (outer ring minus obstacle holes), allowing it to be up
    to ``leave_m`` outside the outer ring. An area without an outer ring accepts all."""
    outer = list(area.get('outer') or [])
    if len(outer) >= 3 and not point_in_polygon(point, outer) \
            and distance_to_ring(point, outer) > leave_m:
        return False
    for hole in area.get('obstacles') or []:
        if len(hole) >= 3 and point_in_polygon(point, hole):
            return False
    return True


def simplify_open_path(points, tolerance, min_spacing=1e-3):
    """Recorded PATH (open polyline): drop repeated points, then Douglas-Peucker.
    Unlike simplify_ring there is no closure and no overshoot trim: both
    endpoints are kept where they were driven."""
    pts = []
    for p in points:
        p = (float(p[0]), float(p[1]))
        if not pts or dist(pts[-1], p) > min_spacing:
            pts.append(p)
    return douglas_peucker(pts, tolerance)


def buffer_polyline(points, half_width, miter_limit=3.0):
    """Polygon (ring, not closed) of an open polyline buffered by ``half_width``
    on each side: left offsets forward, right offsets back, mitred joins
    (clamped to ``miter_limit`` x half_width) and flat ends. [] for < 2 points."""
    pts = [(float(p[0]), float(p[1])) for p in points]
    if len(pts) < 2:
        return []
    normals = []
    for a, b in zip(pts, pts[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        n = math.hypot(dx, dy) or 1.0
        normals.append((-dy / n, dx / n))
    left, right = [], []
    for i, p in enumerate(pts):
        if i == 0:
            nx, ny = normals[0]
            scale = 1.0
        elif i == len(pts) - 1:
            nx, ny = normals[-1]
            scale = 1.0
        else:
            ax, ay = normals[i - 1]
            bx, by = normals[i]
            nx, ny = ax + bx, ay + by
            ln = math.hypot(nx, ny)
            if ln < 1e-9:
                nx, ny, scale = ax, ay, 1.0
            else:
                nx, ny = nx / ln, ny / ln
                cos_half = nx * ax + ny * ay
                scale = min(miter_limit, 1.0 / max(cos_half, 1e-6))
        d = half_width * scale
        left.append((p[0] + nx * d, p[1] + ny * d))
        right.append((p[0] - nx * d, p[1] - ny * d))
    return left + right[::-1]


# ---------------------------------------------------------------------------
# Motion prediction for the obstacle in-path test (2026-10-13)
# ---------------------------------------------------------------------------
def predict_poses(pose, v, w, horizon_s, step_m=0.05, step_rad=0.05, max_samples=400):
    """Poses ``(x, y, yaw)`` of a robot at ``pose`` driving constant ``v`` (m/s) / ``w``
    (rad/s) for ``horizon_s``, sampled every <= ``step_m`` of travel and <= ``step_rad`` of
    turn (first = ``pose``). An arc of radius v / w; straight when |w| is ~0, a pure
    rotation when |v| is ~0. The turn is capped at one full revolution."""
    x0, y0 = float(pose[0]), float(pose[1])
    th0 = float(pose[2]) if len(pose) > 2 else 0.0
    t_end = max(0.0, float(horizon_s))
    v, w = float(v), float(w)
    if abs(w) * t_end > 2.0 * math.pi:
        t_end = 2.0 * math.pi / abs(w)
    n = max(1, int(math.ceil(max(abs(v) * t_end / step_m, abs(w) * t_end / step_rad))))
    n = min(n, int(max_samples))
    out = []
    for k in range(n + 1):
        t = t_end * k / n
        th = th0 + w * t
        if abs(w) < 1e-6:
            x, y = x0 + v * t * math.cos(th0), y0 + v * t * math.sin(th0)
        else:
            r = v / w
            x = x0 + r * (math.sin(th) - math.sin(th0))
            y = y0 - r * (math.cos(th) - math.cos(th0))
        out.append((x, y, th))
    return out


def in_footprint(point, pose, front_m, rear_m, half_width_m, margin_m=0.0):
    """Point within ``margin_m`` of the robot rectangle at ``pose`` (base_link frame: from
    ``rear_m`` behind to ``front_m`` ahead, +-``half_width_m``; rounded corners, i.e. the
    rectangle grown by a disc, so a pivot reaches at most the corner radius + margin)."""
    th = float(pose[2]) if len(pose) > 2 else 0.0
    dx, dy = float(point[0]) - float(pose[0]), float(point[1]) - float(pose[1])
    c, s = math.cos(th), math.sin(th)
    fx, fy = c * dx + s * dy, -s * dx + c * dy
    ex = max(-float(rear_m) - fx, 0.0, fx - float(front_m))
    ey = max(abs(fy) - float(half_width_m), 0.0)
    return math.hypot(ex, ey) <= float(margin_m)


def swept_hit(point, poses, front_m, rear_m, half_width_m, margin_m=0.0):
    """Point inside the footprint (+ margin) at any of ``poses`` (see predict_poses)."""
    return any(in_footprint(point, p, front_m, rear_m, half_width_m, margin_m) for p in poses)


def wrap_angle(a):
    """``a`` wrapped to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def polyline_distance(point, poses):
    """Distance from ``point`` to the polyline through ``poses`` (inf when empty)."""
    if not poses:
        return float('inf')
    if len(poses) == 1:
        return dist(point, poses[0])
    best = float('inf')
    px, py = float(point[0]), float(point[1])
    for k in range(len(poses) - 1):
        ax, ay = float(poses[k][0]), float(poses[k][1])
        bx, by = float(poses[k + 1][0]), float(poses[k + 1][1])
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
        best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best


def _point_at(poses, cum, s):
    """(x, y) at arc length ``s`` (clamped to the polyline)."""
    if s <= 0.0:
        return float(poses[0][0]), float(poses[0][1])
    for k in range(1, len(poses)):
        if cum[k] >= s:
            seg = cum[k] - cum[k - 1]
            f = 0.0 if seg < 1e-12 else (s - cum[k - 1]) / seg
            return (float(poses[k - 1][0]) + (float(poses[k][0]) - float(poses[k - 1][0])) * f,
                    float(poses[k - 1][1]) + (float(poses[k][1]) - float(poses[k - 1][1])) * f)
    return float(poses[-1][0]), float(poses[-1][1])


def route_cusps(poses, angle_rad, window_m=0.3):
    """Cusps of a route: points where the travel direction reverses by more than
    ``angle_rad``. The turn at pose ``i`` is the angle between the chord from the point
    ``window_m`` of path before it and the chord to the point ``window_m`` after it, so a
    filleted hairpin (a short arc instead of a vertex) counts by its total turn. A run of
    poses over the threshold is one cusp, at its sharpest pose.

    Returns ``[(i, x, y, heading_after)]``: route index, position and the direction of the
    route after the cusp (rad)."""
    if angle_rad <= 0.0 or len(poses) < 3:
        return []
    cum = cumulative_lengths(poses)
    w = max(1e-3, float(window_m))
    cand = []
    for i in range(1, len(poses) - 1):
        s = cum[i]
        if s < 0.5 * w or cum[-1] - s < 0.5 * w:
            continue                      # no chord before / after
        ax, ay = _point_at(poses, cum, s - w)
        cx, cy = _point_at(poses, cum, s + w)
        px, py = float(poses[i][0]), float(poses[i][1])
        if math.hypot(px - ax, py - ay) < 0.25 * w or math.hypot(cx - px, cy - py) < 0.25 * w:
            continue
        h1 = math.atan2(py - ay, px - ax)
        h2 = math.atan2(cy - py, cx - px)
        turn = abs(wrap_angle(h2 - h1))
        if turn >= angle_rad:
            cand.append((i, turn, h2))
    out, group = [], []
    for c in cand:
        if group and cum[c[0]] - cum[group[-1][0]] > w:
            out.append(max(group, key=lambda g: g[1]))
            group = []
        group.append(c)
    if group:
        out.append(max(group, key=lambda g: g[1]))
    return [(i, float(poses[i][0]), float(poses[i][1]), h2) for i, _t, h2 in out]


def route_corners(poses, angle_rad, window_m=0.3, simplify_m=0.02, min_arm_m=0.15,
                  tol_rad=math.radians(1.0)):
    """Corners of a route polyline that turn by at least ``angle_rad`` (inclusive, minus
    ``tol_rad``), for splitting it into stop-and-pivot legs (task 0.4 d).

    The bend angles are measured on the route simplified to its straight runs
    (Douglas-Peucker at ``simplify_m``): the heading in and out of a vertex is the chord of
    the whole run before / after it, so neither the pose spacing nor a hand-drawn wobble of
    a few centimetres / degrees changes them, and a vertex lands on the drawn vertex (a
    route pose). Bends that each stay under the threshold but turn the same way within a
    ``window_m`` stretch of route (a corner drawn as two or three clicks, a tight arc) add up
    to one corner, placed at the vertex where half of the sum is reached. A bend over the
    threshold is always its own corner, however close the next one is (two sharp corners
    0.3 m apart are each a stop-and-pivot: the robot drives the short run between them on
    the centreline instead of cutting both). Corners closer than ``min_arm_m`` to either
    route end are skipped (nothing to drive on that side).

    Returns ``[(i, turn)]``: route pose index and the signed turn (rad, left > 0)."""
    n = len(poses)
    if angle_rad <= 0.0 or n < 3:
        return []
    cum = cumulative_lengths(poses)
    pts = [(float(p[0]), float(p[1]), i) for i, p in enumerate(poses)]
    verts = [q[2] for q in douglas_peucker(pts, max(1e-4, float(simplify_m)))]
    if len(verts) < 3:
        return []

    def head(a, b):
        return math.atan2(float(poses[b][1]) - float(poses[a][1]),
                          float(poses[b][0]) - float(poses[a][0]))

    bends = []                            # (index, signed turn) at each simplified vertex
    for a, v, b in zip(verts, verts[1:], verts[2:]):
        bends.append((v, wrap_angle(head(v, b) - head(a, v))))
    lim = float(angle_rad) - float(tol_rad)
    out, run = [], []

    def flush():
        # within a run of same-way sub-threshold bends, every stretch shorter than window_m
        # whose bends add up to the threshold is one corner (the earliest such stretch first)
        j = 0
        while j < len(run):
            hit = None
            for a in range(j, len(run)):
                acc = 0.0
                for b in range(a, len(run)):
                    if cum[run[b][0]] - cum[run[a][0]] >= window_m:
                        break
                    acc += run[b][1]
                    if abs(acc) >= lim:
                        hit = (a, b, acc)
                        break
                if hit:
                    break
            if hit is None:
                break
            a, b, acc = hit
            half, pick = 0.0, run[b][0]
            for i, t in run[a:b + 1]:
                half += abs(t)
                if half >= 0.5 * abs(acc) - 1e-9:
                    pick = i
                    break
            out.append((pick, acc))
            j = b + 1
        del run[:]

    for i, t in bends:
        if abs(t) >= lim:
            flush()
            out.append((i, t))
            continue
        if run and (t * run[-1][1] <= 0.0 or cum[i] - cum[run[-1][0]] >= window_m):
            flush()
        run.append((i, t))
    flush()
    return [(i, t) for i, t in sorted(out)
            if cum[i] >= min_arm_m - 1e-9 and cum[-1] - cum[i] >= min_arm_m - 1e-9]
