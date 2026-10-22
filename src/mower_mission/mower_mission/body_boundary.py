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
