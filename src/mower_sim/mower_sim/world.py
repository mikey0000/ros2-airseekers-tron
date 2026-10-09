# SPDX-License-Identifier: Apache-2.0
"""ROS-free world model for the kinematic simulator.

A world is a YAML file (``worlds/*.yaml``)::

    datum: {lat: 48.0, lon: 9.0}          # GPS anchor of the map origin (ENU, x east)
    lawn: [[x, y], ...]                   # mowing area polygon, map metres
    areas: [{name, polygon, is_navigation}]   # optional extra areas (paths etc.)
    dock: {x: 0.0, y: 0.0, yaw: 0.0}      # base_link pose when on the charger
    start: {docked: true} | {x, y, yaw}   # initial robot pose
    obstacles:
      - {name: box, type: box, x, y, yaw, size_x, size_y, height}
      - {name: tree, type: cylinder, x, y, radius, height}
      - {name: person, type: cylinder, radius: 0.25, height: 1.7,
         path: [[x, y], ...], speed: 0.6, mode: pingpong|loop|once, start_delay: 0.0}
    battery: {voltage: 25.2, percentage: 0.8}

Obstacles are 2.5D: a 2D shape extruded from the ground to ``height``. Moving
obstacles follow ``path`` at ``speed`` (m/s) after ``start_delay`` seconds.
Map frame == the vendor frame: origin at the dock, x forward out of the dock
when ``dock.yaw`` is 0.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]
Polygon = List[Point]

# Footprint around base_link (rear axle), nav2_params.yaml / URDF.
FOOTPRINT: Polygon = [(0.52, 0.27), (0.52, -0.27), (-0.23, -0.27), (-0.23, 0.27)]


# ---------------------------------------------------------------------------
# 2D geometry
# ---------------------------------------------------------------------------

def transform(poly: Sequence[Point], x: float, y: float, yaw: float) -> Polygon:
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in poly]


def to_local(p: Point, x: float, y: float, yaw: float) -> Point:
    """Map point -> frame at (x, y, yaw)."""
    dx, dy = p[0] - x, p[1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * dx + s * dy, -s * dx + c * dy)


def point_in_polygon(p: Point, poly: Sequence[Point]) -> bool:
    x, y = p
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def _seg_point_dist(a: Point, b: Point, p: Point) -> float:
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 <= 0.0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(ax + t * dx - p[0], ay + t * dy - p[1])


def polygon_point_distance(poly: Sequence[Point], p: Point) -> float:
    """0 inside, else the distance to the nearest edge."""
    if point_in_polygon(p, poly):
        return 0.0
    n = len(poly)
    return min(_seg_point_dist(poly[i], poly[(i + 1) % n], p) for i in range(n))


def _axes(poly: Sequence[Point]):
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        yield (y1 - y2, x2 - x1)


def convex_overlap(a: Sequence[Point], b: Sequence[Point]) -> bool:
    """Separating-axis test for two convex polygons (touching counts as overlap)."""
    for axis in list(_axes(a)) + list(_axes(b)):
        pa = [axis[0] * x + axis[1] * y for x, y in a]
        pb = [axis[0] * x + axis[1] * y for x, y in b]
        if max(pa) < min(pb) or max(pb) < min(pa):
            return False
    return True


def polygon_distance(a: Sequence[Point], b: Sequence[Point]) -> float:
    """Distance between two polygons (0 when they overlap; convex assumed for overlap)."""
    if convex_overlap(a, b):
        return 0.0
    d = min(polygon_point_distance(b, p) for p in a)
    return min(d, min(polygon_point_distance(a, p) for p in b))


# ---------------------------------------------------------------------------
# Obstacles
# ---------------------------------------------------------------------------

@dataclass
class Obstacle:
    name: str
    type: str                      # 'box' | 'cylinder'
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    size_x: float = 0.5
    size_y: float = 0.5
    radius: float = 0.25
    height: float = 0.5
    path: Optional[List[Point]] = None
    speed: float = 0.5
    mode: str = 'pingpong'         # pingpong | loop | once
    start_delay: float = 0.0
    known: bool = False            # also written into areas.dat as a map obstacle

    @property
    def moving(self) -> bool:
        return bool(self.path) and len(self.path) >= 2 and self.speed > 0.0

    def position(self, t: float) -> Point:
        """Centre at sim time ``t`` (s since start)."""
        if not self.moving:
            return (self.x, self.y)
        pts = self.path
        if self.mode == 'loop':
            pts = list(pts) + [pts[0]]
        segs = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])]
        total = sum(segs)
        if total <= 0.0:
            return pts[0]
        s = max(0.0, t - self.start_delay) * self.speed
        if self.mode == 'once':
            s = min(s, total)
        elif self.mode == 'loop':
            s = s % total
        else:  # pingpong
            s = s % (2.0 * total)
            if s > total:
                s = 2.0 * total - s
        for (a, b), L in zip(zip(pts, pts[1:]), segs):
            if s <= L or L == segs[-1] and (a, b) == (pts[-2], pts[-1]):
                f = 0.0 if L <= 0.0 else min(1.0, s / L)
                return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))
            s -= L
        return pts[-1]

    def polygon(self, t: float = 0.0, segments: int = 16) -> Polygon:
        """Map-frame outline at time ``t`` (cylinders as a regular polygon)."""
        cx, cy = self.position(t)
        if self.type == 'box':
            hx, hy = self.size_x / 2.0, self.size_y / 2.0
            return transform([(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)], cx, cy, self.yaw)
        return [(cx + self.radius * math.cos(2 * math.pi * k / segments),
                 cy + self.radius * math.sin(2 * math.pi * k / segments))
                for k in range(segments)]

    def distance_to_polygon(self, poly: Sequence[Point], t: float = 0.0) -> float:
        if self.type == 'cylinder':
            c = self.position(t)
            return max(0.0, polygon_point_distance(poly, c) - self.radius)
        return polygon_distance(self.polygon(t), poly)


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class World:
    lawn: Polygon
    dock: Pose2D
    start: Pose2D
    start_docked: bool = False
    datum_lat: float = 48.0
    datum_lon: float = 9.0
    obstacles: List[Obstacle] = field(default_factory=list)
    extra_areas: List[dict] = field(default_factory=list)
    battery_voltage: float = 25.2
    battery_percentage: float = 0.8
    name: str = 'world'

    def footprint_at(self, x: float, y: float, yaw: float) -> Polygon:
        return transform(FOOTPRINT, x, y, yaw)

    def collisions(self, x: float, y: float, yaw: float, t: float,
                   margin: float = 0.0) -> List[Obstacle]:
        fp = self.footprint_at(x, y, yaw)
        return [o for o in self.obstacles if o.distance_to_polygon(fp, t) <= margin]

    def clearance(self, x: float, y: float, yaw: float, t: float) -> float:
        """Smallest footprint-to-obstacle distance (inf without obstacles)."""
        fp = self.footprint_at(x, y, yaw)
        return min((o.distance_to_polygon(fp, t) for o in self.obstacles), default=math.inf)


def _pose(d, default: Pose2D) -> Pose2D:
    if not d:
        return Pose2D(default.x, default.y, default.yaw)
    return Pose2D(float(d.get('x', default.x)), float(d.get('y', default.y)),
                  float(d.get('yaw', default.yaw)))


def world_from_dict(doc: dict, name: str = 'world') -> World:
    if 'world' in doc and isinstance(doc['world'], dict):
        doc = doc['world']
    lawn = [(float(p[0]), float(p[1])) for p in doc.get('lawn', [])]
    if len(lawn) < 3:
        raise ValueError('world: lawn polygon needs >= 3 points')
    dock = _pose(doc.get('dock'), Pose2D())
    start_doc = doc.get('start') or {'docked': True}
    docked = bool(start_doc.get('docked', False))
    start = Pose2D(dock.x, dock.y, dock.yaw) if docked else _pose(start_doc, dock)
    datum = doc.get('datum') or {}
    obstacles = []
    for i, o in enumerate(doc.get('obstacles') or []):
        typ = str(o.get('type', 'box'))
        if typ not in ('box', 'cylinder'):
            raise ValueError('world: obstacle %d has unknown type %r' % (i, typ))
        path = o.get('path')
        obstacles.append(Obstacle(
            name=str(o.get('name', '%s_%d' % (typ, i))), type=typ,
            x=float(o.get('x', path[0][0] if path else 0.0)),
            y=float(o.get('y', path[0][1] if path else 0.0)),
            yaw=float(o.get('yaw', 0.0)),
            size_x=float(o.get('size_x', 0.5)), size_y=float(o.get('size_y', 0.5)),
            radius=float(o.get('radius', 0.25)), height=float(o.get('height', 0.5)),
            path=[(float(p[0]), float(p[1])) for p in path] if path else None,
            speed=float(o.get('speed', 0.5)), mode=str(o.get('mode', 'pingpong')),
            start_delay=float(o.get('start_delay', 0.0)),
            known=bool(o.get('known', False))))
    bat = doc.get('battery') or {}
    return World(lawn=lawn, dock=dock, start=start, start_docked=docked,
                 datum_lat=float(datum.get('lat', 48.0)), datum_lon=float(datum.get('lon', 9.0)),
                 obstacles=obstacles, extra_areas=list(doc.get('areas') or []),
                 battery_voltage=float(bat.get('voltage', 25.2)),
                 battery_percentage=float(bat.get('percentage', 0.8)),
                 name=str(doc.get('name', name)))


def load_world(path: str) -> World:
    import os
    import yaml
    with open(path, encoding='utf-8') as f:
        doc = yaml.safe_load(f) or {}
    return world_from_dict(doc, os.path.splitext(os.path.basename(path))[0])


# ---------------------------------------------------------------------------
# GPS: map metres <-> WGS84 (local tangent plane at the datum; x east, y north)
# ---------------------------------------------------------------------------

_WGS84_A = 6378137.0
_WGS84_E2 = 6.69437999014e-3


def _radii(lat_deg: float) -> Tuple[float, float]:
    s = math.sin(math.radians(lat_deg))
    w = math.sqrt(1.0 - _WGS84_E2 * s * s)
    meridional = _WGS84_A * (1.0 - _WGS84_E2) / (w ** 3)
    normal = _WGS84_A / w
    return meridional, normal


def enu_to_latlon(x: float, y: float, lat0: float, lon0: float) -> Tuple[float, float]:
    m, n = _radii(lat0)
    lat = lat0 + math.degrees(y / m)
    lon = lon0 + math.degrees(x / (n * math.cos(math.radians(lat0))))
    return lat, lon


def latlon_to_enu(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    m, n = _radii(lat0)
    return (math.radians(lon - lon0) * n * math.cos(math.radians(lat0)),
            math.radians(lat - lat0) * m)
