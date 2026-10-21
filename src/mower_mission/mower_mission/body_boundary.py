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
* soft edge_mode (stock layout: lap 0 has base_link ON the line): corners (sides) 0.35, nose
  0.35, tail 0.30. Normal soft driving stays below: lap 0 puts the sides 0.235 out, a row end
  0.45 inside puts the nose 0.15 / the front corners <= 0.195 past (pivot sweep included);
  tracking error p99 0.07 (pivots) .. 0.13 (all samples, sim 2026-10-21) on top.
"""

import math

# body frame (x forward, y left), metres from base_link (owner-measured 2026-10-20)
FRONT, REAR, HALF_WIDTH = 0.60, 0.10, 0.235


def body_points(front=FRONT, rear=REAR, half=HALF_WIDTH):
    """name -> (bx, by, kind) with kind 'side' (corners), 'nose' or 'tail'."""
    return {
        'front_left': (front, half, 'side'), 'front_right': (front, -half, 'side'),
        'rear_left': (-rear, half, 'side'), 'rear_right': (-rear, -half, 'side'),
        'nose': (front, 0.0, 'nose'), 'tail': (-rear, 0.0, 'tail'),
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


def limits_for(mode, soft_side=0.35, soft_nose=0.35, soft_tail=0.30, hard=0.10):
    """kind -> limit (m) for edge_mode ``mode`` ('hard' | anything else = soft)."""
    if str(mode).strip().lower() == 'hard':
        return {'side': hard, 'nose': hard, 'tail': hard}
    return {'side': soft_side, 'nose': soft_nose, 'tail': soft_tail}


def evaluate(pose, polys, limits, points=None):
    """Body check at ``pose`` (x, y, yaw). Returns a dict: ``excess`` (max over the points of
    outside - limit; > 0 = trip), ``point`` (the worst point's name), ``out`` (its outside
    distance), ``rear`` (True when the worst point is behind base_link: recover forwards),
    ``outs`` {name: outside m}. None without polygons."""
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
    return {'excess': best[0], 'point': best[1], 'out': best[2], 'rear': best[3], 'outs': outs}
