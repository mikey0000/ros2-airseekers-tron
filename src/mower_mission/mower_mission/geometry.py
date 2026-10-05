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


def simplify_ring(points, tolerance, close_eps=None):
    """Turn a recorded drive into a closed polygon ring.

    The track is simplified as a *closed* loop: the first point is appended
    so the closing edge takes part in the simplification, then the duplicate
    closing vertex is dropped again (``geometry_msgs/Polygon`` is implicitly
    closed; map_server_node does not want a repeated first vertex).  A track
    that already ends within ``close_eps`` of its start loses that last point
    first so it does not produce a zero-length closing edge.
    """
    pts = [(float(p[0]), float(p[1])) for p in points]
    if close_eps is None:
        close_eps = tolerance
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
    (Humble's SimpleGoalChecker) declares success as soon as the robot passes
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
