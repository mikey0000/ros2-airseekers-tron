# Copyright 2026 Airseekers Tron ROS 2 port contributors
# SPDX-License-Identifier: GPL-3.0-or-later
"""Robot body model for body-aware planning and measurement (docs/DESIGN_path_conditioning_2026-10.md
section 7). Pure Python, no ROS, no numpy.

The body is the Nav2 footprint polygon (robot frame: x forward, y left, origin base_link = the
drive axle centre and pivot centre) grown by the footprint padding exactly as Nav2 pads it
(``nav2_costmap_2d::padFootprint``: every vertex moves ``padding`` away from the origin along x
and along y). One source of truth: ``nav2_params.yaml`` ``local_costmap`` ``footprint`` /
``footprint_padding``; :func:`load_body` reads it when no explicit footprint is given.

``outside`` = how far the body sticks out of the allowed region (areas U drawn path channels U
dock corridor; a union of polygons): the largest distance outside the union over points sampled
along the body outline (0 when the whole outline is inside). The outline is enough: a part of
the outside that reached into the body without crossing its outline would be a hole of the
region, and the region polygons have none here (obstacle holes are not part of the hard rule).
"""

import ast
import math
import os

from mower_common import geometry as rgeo

# Tron, measured (owner 2026-10-21, base_link = rear axle / pivot centre): front 0.60, rear
# 0.10, half width 0.235 (= nav2_params.yaml footprint). Measurement uses NO padding (the
# padding is a Nav2 planning margin, not body): MEASURE_PADDING.
DEFAULT_FOOTPRINT = rgeo.FOOTPRINT   # ((0.60, 0.235), (0.60, -0.235), (-0.10, -0.235), (-0.10, 0.235))
DEFAULT_PADDING = 0.02
MEASURE_PADDING = 0.0


def parse_footprint(text):
    """Nav2 footprint string "[[x, y], ...]" -> [(x, y), ...] (>= 3 points), else None."""
    if not isinstance(text, str) or not text.strip():
        return None
    try:
        val = ast.literal_eval(text.strip())
        pts = [(float(p[0]), float(p[1])) for p in val]
    except (ValueError, SyntaxError, TypeError, IndexError):
        return None
    return pts if len(pts) >= 3 else None


def pad_footprint(poly, padding):
    """Nav2 padFootprint: x += sign(x) * padding, y += sign(y) * padding."""
    def sgn(v):
        return 1.0 if v > 0 else (-1.0 if v < 0 else 0.0)
    return [(round(x + sgn(x) * padding, 9), round(y + sgn(y) * padding, 9)) for x, y in poly]


def nav2_footprint(path, costmap='local_costmap'):
    """(footprint, padding) from a nav2 params YAML (``costmap``'s own section), or None."""
    try:
        import yaml
        with open(path) as fh:
            doc = yaml.safe_load(fh) or {}
        prm = doc[costmap][costmap]['ros__parameters']
        poly = parse_footprint(prm.get('footprint'))
        if poly is None:
            return None
        return poly, float(prm.get('footprint_padding', 0.0))
    except (OSError, KeyError, TypeError, ValueError, ImportError):
        return None
    except Exception:          # yaml.YAMLError without importing yaml at module level
        return None


def default_nav2_params_path():
    """The installed mower_navigation/config/nav2_params.yaml, else None."""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('mower_navigation'), 'config',
                         'nav2_params.yaml')
        return p if os.path.isfile(p) else None
    except Exception:
        return None


class Body:
    """Padded footprint polygon (robot frame) plus outline samples every ``step`` m."""

    def __init__(self, footprint=DEFAULT_FOOTPRINT, padding=DEFAULT_PADDING, step=0.08,
                 source='default'):
        self.raw = [tuple(map(float, p)) for p in footprint]
        self.padding = float(padding)
        self.poly = pad_footprint(self.raw, self.padding)
        self.source = source
        self.samples = []
        n = len(self.poly)
        for i in range(n):
            ax, ay = self.poly[i]
            bx, by = self.poly[(i + 1) % n]
            k = max(1, int(math.ceil(math.hypot(bx - ax, by - ay) / step)))
            for j in range(k):
                self.samples.append((ax + (bx - ax) * j / k, ay + (by - ay) * j / k))
        self.front = max(x for x, _ in self.poly)
        self.rear = -min(x for x, _ in self.poly)
        self.half_width = max(abs(y) for _, y in self.poly)
        self.radius = max(math.hypot(x, y) for x, y in self.poly)        # pivot swing
        self.rear_radius = max((math.hypot(x, y) for x, y in self.poly if x <= 0.0),
                               default=0.0)
        # arc length per sample of the farthest corner while pivoting
        self.sweep_step_rad = step / max(self.radius, 1e-3)
        # points that must stay inside during a pivot (hard: the whole outline)
        self.sweep_samples = self.samples
        self.mode = 'hard'

    def soft(self):
        """The soft-edge view (area edge_mode soft): only the centreline from the tail to the
        nose must stay inside (the sides may overhang), and in a pivot only its front half
        (base_link to the nose, sweeping on ``front``)."""
        v = Body.__new__(Body)
        v.__dict__.update(self.__dict__)
        k = max(2, int(math.ceil((self.front + self.rear) / 0.05)))
        v.samples = [(-self.rear + (self.front + self.rear) * j / k, 0.0) for j in range(k + 1)]
        v.sweep_samples = [(s, 0.0) for s in (self.front * j / 4 for j in range(5))]
        v.radius = max(self.front, self.rear)
        v.sweep_step_rad = 0.05 / max(self.front, 1e-3)
        v.mode = 'soft'
        return v

    def describe(self):
        return ('body (%s) %s padding %.2f (front %.2f, rear %.2f, half width %.2f, pivot swing '
                '%.2f / rear %.2f m, %s)' % (
                    self.mode, [list(p) for p in self.raw], self.padding, self.front, self.rear,
                    self.half_width, self.radius, self.rear_radius, self.source))

    def world(self, x, y, yaw, pts=None):
        c, s = math.cos(yaw), math.sin(yaw)
        return [(x + c * bx - s * by, y + s * bx + c * by) for bx, by in (pts or self.samples)]

    def sweep_yaws(self, yaw0, yaw1):
        """Headings from yaw0 to yaw1 (shortest way) so the outline moves <= step per sample."""
        d = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
        k = max(1, int(math.ceil(abs(d) / self.sweep_step_rad)))
        return [yaw0 + d * j / k for j in range(k + 1)]


def load_body(footprint='', padding=-1.0, nav2_params_file='', step=0.08):
    """The body from explicit params, else from nav2_params.yaml (``nav2_params_file`` or the
    installed mower_navigation one), else :data:`DEFAULT_FOOTPRINT` (``source`` says which).
    ``padding`` < 0 = the nav2 file's footprint_padding."""
    poly = parse_footprint(footprint) if footprint else None
    nav = None
    path = nav2_params_file or default_nav2_params_path()
    if path:
        nav = nav2_footprint(path)
    if poly is not None:
        pad = padding if padding >= 0.0 else (nav[1] if nav else DEFAULT_PADDING)
        return Body(poly, pad, step, 'param')
    if nav is not None:
        pad = padding if padding >= 0.0 else nav[1]
        return Body(nav[0], pad, step, path)
    return Body(DEFAULT_FOOTPRINT, padding if padding >= 0.0 else DEFAULT_PADDING, step,
                'built-in default')


# ---------------------------------------------------------------- region geometry

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
        d = math.hypot(ax + t * dx - x, ay + t * dy - y)
        if d < best:
            best = d
    return best


def clearance(x, y, polys):
    """Signed distance of a point to the region edge: > 0 inside (the largest single-polygon
    clearance, a conservative estimate of the union's), < 0 outside. No polygons: +inf."""
    best_in = None
    best_out = math.inf
    for p in polys:
        if len(p) < 3:
            continue
        d = _edge_dist(x, y, p)
        if _pip(x, y, p):
            best_in = d if best_in is None else max(best_in, d)
        else:
            best_out = min(best_out, d)
    if best_in is not None:
        return best_in
    return -best_out if math.isfinite(best_out) else math.inf


def body_clearance(body, x, y, yaw, polys, need=0.0):
    """Least clearance of the body outline at pose (x, y, yaw) (< 0: outside by that much).
    Returns early with the base point's clearance when that alone shows the body is inside
    with ``need`` to spare (the whole body lies within ``radius`` of base_link)."""
    c0 = clearance(x, y, polys)
    if c0 >= body.radius + need:
        return c0 - body.radius
    return min(clearance(px, py, polys) for px, py in body.world(x, y, yaw))


def body_outside(body, x, y, yaw, polys):
    """How far the body at (x, y, yaw) sticks out of the union of ``polys`` (0 inside)."""
    c = body_clearance(body, x, y, yaw, polys)
    return max(0.0, -c) if math.isfinite(c) else 0.0


def sweep_clearance(body, x, y, yaw0, yaw1, polys, need=0.0):
    """Least body clearance over a pivot in place at (x, y) from yaw0 to yaw1."""
    c0 = clearance(x, y, polys)
    if c0 >= body.radius + need:
        return c0 - body.radius
    v = min(min(clearance(px, py, polys)
                for px, py in body.world(x, y, a, body.sweep_samples))
            for a in body.sweep_yaws(yaw0, yaw1))
    if body.sweep_samples is not body.samples:     # soft: tail..nose at both ends too
        v = min(v, body_clearance(body, x, y, yaw0, polys, need),
                body_clearance(body, x, y, yaw1, polys, need))
    return v
