# SPDX-License-Identifier: GPL-3.0-or-later
"""ROS-free core of the mower_map zone server.

Ported from MowgliNext ``mowgli_map`` (GPL-3.0): area/obstacle data model,
the ``areas.dat`` plain-text persistence format (byte-compatible with
``MapServerNode::save_areas_to_file`` / ``load_areas_from_file``), the keepout
rasteriser, boundary classification, the recovery-point search, the
mowgli_robot.yaml dock-pose splice, and the Airseekers vendor GeoJSON importer.

Only numpy is required (no shapely), so ``python3 -m pytest src/mower_map/test``
runs on a host without ROS.

Coordinates are map-frame metres. A polygon is a list of ``(x, y)`` tuples,
open (the closing vertex is not repeated).
"""

from __future__ import annotations

import json
import math
import os
import re
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

Point = Tuple[float, float]
Polygon = List[Point]

# MapObstacleInfo.SOURCE_*
SOURCE_USER = 0
SOURCE_TRACKER = 1
SOURCE_DIG = 2

# Upstream internal_helpers.hpp kObstacleDedupEpsilonM: two obstacles whose
# centroids are this close are the same obstacle.
OBSTACLE_DEDUP_EPS_M = 0.10

LETHAL = 100
FREE = 0


# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------

@dataclass
class Obstacle:
    polygon: Polygon
    name: str = ''
    source: int = SOURCE_USER
    pending: bool = False
    id: int = 0


@dataclass
class Area:
    name: str
    polygon: Polygon
    is_navigation: bool = False
    obstacles: List[Obstacle] = field(default_factory=list)


@dataclass
class DockPose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    # Dock keep-out outline in the DOCK-LOCAL frame (origin = dock pose,
    # +X = dock yaw). None = no outline known.
    outline: Optional[Polygon] = None

    def outline_in_map(self) -> Optional[Polygon]:
        if not self.outline or len(self.outline) < 3:
            return None
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return [(self.x + c * px - s * py, self.y + s * px + c * py) for px, py in self.outline]


def normalise_polygon(points: Iterable[Sequence[float]]) -> Polygon:
    """Take x/y of each point (ignoring z or vendor extras) and drop a
    repeated closing vertex."""
    poly = [(float(p[0]), float(p[1])) for p in points]
    if len(poly) >= 2 and poly[0] == poly[-1]:
        poly.pop()
    return poly


def polygon_centroid(poly: Polygon) -> Point:
    """Vertex-average centroid (same as upstream polygon_centroid)."""
    if not poly:
        return (0.0, 0.0)
    return (sum(p[0] for p in poly) / len(poly), sum(p[1] for p in poly) / len(poly))


def polygon_area(poly: Polygon) -> float:
    """Unsigned shoelace area."""
    n = len(poly)
    if n < 3:
        return 0.0
    s = 0.0
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return abs(s) * 0.5


def has_duplicate_obstacle(existing: Iterable[Polygon], candidate: Polygon,
                           eps: float = OBSTACLE_DEDUP_EPS_M) -> bool:
    cx, cy = polygon_centroid(candidate)
    for poly in existing:
        ex, ey = polygon_centroid(poly)
        if math.hypot(ex - cx, ey - cy) <= eps:
            return True
    return False


class MapStore:
    """The area list plus session-scoped obstacle ids (MapObstacleInfo.id)."""

    def __init__(self):
        self.areas: List[Area] = []
        self._next_obstacle_id = 1

    def _make_obstacle(self, polygon, name='', source=SOURCE_USER, pending=False) -> Obstacle:
        obs = Obstacle(list(polygon), name, int(source), bool(pending), self._next_obstacle_id)
        self._next_obstacle_id += 1
        return obs

    def clear(self):
        self.areas = []

    def add_area(self, name: str, polygon: Polygon, is_navigation: bool = False,
                 obstacles: Sequence[Tuple[Polygon, str, int]] = ()) -> Area:
        """Append an area. ``obstacles`` items are (polygon, name, source).
        Raises ValueError for a polygon with fewer than 3 vertices. Obstacles
        with fewer than 3 vertices are dropped (as upstream)."""
        polygon = normalise_polygon(polygon)
        if len(polygon) < 3:
            raise ValueError('area polygon needs at least 3 points')
        area = Area(name, polygon, bool(is_navigation), [])
        for obs_poly, obs_name, obs_source in obstacles:
            obs_poly = normalise_polygon(obs_poly)
            if len(obs_poly) >= 3:
                area.obstacles.append(self._make_obstacle(obs_poly, obs_name, obs_source))
        self.areas.append(area)
        return area

    def load(self, areas: List[Area]):
        """Replace the area list (from areas.dat), assigning fresh ids."""
        self.areas = []
        for a in areas:
            new = Area(a.name, list(a.polygon), a.is_navigation, [])
            for o in a.obstacles:
                new.obstacles.append(self._make_obstacle(o.polygon, o.name, o.source, False))
            self.areas.append(new)

    def add_obstacle(self, area_index: int, polygon: Polygon, name: str = '',
                     source: int = SOURCE_USER, pending: bool = False) -> Tuple[bool, str]:
        """Port of apply_promoted_obstacle. Returns (ok, message). A duplicate
        (centroid within 10 cm of an existing obstacle) is an accepted no-op."""
        polygon = normalise_polygon(polygon)
        if area_index < 0 or area_index >= len(self.areas):
            return False, 'bad area_index %d' % area_index
        area = self.areas[area_index]
        if area.is_navigation:
            return False, 'area %d is a navigation area' % area_index
        if len(polygon) < 3:
            return False, 'polygon needs at least 3 points'
        if has_duplicate_obstacle([o.polygon for o in area.obstacles], polygon):
            return True, 'duplicate keepout ignored (no-op)'
        area.obstacles.append(self._make_obstacle(polygon, name, source, pending))
        return True, 'obstacle added to area %d' % area_index

    def accept_pending(self, pending_id: int, name: str = '') -> Optional[int]:
        if pending_id == 0:
            return None
        for i, area in enumerate(self.areas):
            for obs in area.obstacles:
                if obs.id == pending_id and obs.pending:
                    obs.pending = False
                    if name:
                        obs.name = name
                    return i
        return None

    def discard_pending(self, pending_id: int) -> bool:
        if pending_id == 0:
            return False
        for area in self.areas:
            for k, obs in enumerate(area.obstacles):
                if obs.id == pending_id and obs.pending:
                    del area.obstacles[k]
                    return True
        return False


# ---------------------------------------------------------------------------
# areas.dat (exact upstream format)
# ---------------------------------------------------------------------------

def _fmt_coord(v: float) -> str:
    """C++ ``ostream << float`` with default precision 6: geometry_msgs
    Point32 holds float32, printed like printf %g."""
    return '%g' % float(np.float32(v))


def polygon_to_string(poly: Polygon) -> str:
    return ';'.join('%s,%s' % (_fmt_coord(x), _fmt_coord(y)) for x, y in poly)


def parse_polygon_string(s: str) -> Polygon:
    """Port of parse_polygon_string: "x,y;x,y;..." (float32 precision)."""
    poly: Polygon = []
    if not s:
        return poly
    for pt in s.split(';'):
        parts = pt.split(',')
        if len(parts) < 2:
            continue
        try:
            poly.append((float(np.float32(float(parts[0]))), float(np.float32(float(parts[1])))))
        except ValueError:
            continue
    return poly


def _one_line(s: str) -> str:
    return s.replace('\r', ' ').replace('\n', ' ')


def format_areas_dat(areas: List[Area], datum_lat: float = 0.0, datum_lon: float = 0.0) -> str:
    """Byte-for-byte port of MapServerNode::save_areas_to_file.

    The datum stamp is written only when a datum is set (|lat| or |lon| >
    1e-9), as upstream. PENDING obstacles are not persisted."""
    out = ['# Mowgli ROS2 — Persisted areas and docking point\n',
           '# Auto-generated by map_server_node. Do not edit manually.\n\n']
    if abs(datum_lat) > 1e-9 or abs(datum_lon) > 1e-9:
        out.append('datum_lat: %.9f\ndatum_lon: %.9f\n\n' % (datum_lat, datum_lon))
    out.append('area_count: %d\n\n' % len(areas))
    for i, area in enumerate(areas):
        out.append('area_%d_name: %s\n' % (i, _one_line(area.name)))
        out.append('area_%d_polygon: %s\n' % (i, polygon_to_string(area.polygon)))
        out.append('area_%d_is_navigation: %d\n' % (i, 1 if area.is_navigation else 0))
        persisted = [o for o in area.obstacles if not o.pending]
        out.append('area_%d_obstacle_count: %d\n' % (i, len(persisted)))
        for j, obs in enumerate(persisted):
            out.append('area_%d_obstacle_%d: %s\n' % (i, j, polygon_to_string(obs.polygon)))
            if obs.name:
                out.append('area_%d_obstacle_%d_name: %s\n' % (i, j, _one_line(obs.name)))
            if obs.source != SOURCE_USER:
                out.append('area_%d_obstacle_%d_source: %d\n' % (i, j, obs.source))
        out.append('\n')
    return ''.join(out)


def parse_areas_dat(text: str) -> Tuple[List[Area], Optional[Tuple[float, float]]]:
    """Port of MapServerNode::load_areas_from_file. Returns (areas, datum)
    where datum is (lat, lon) or None when the file carries no stamp.
    Areas with < 3 vertices are skipped; obstacles with < 3 vertices or a
    duplicate centroid are dropped; old dock_* keys are ignored."""
    kv: Dict[str, str] = {}
    for line in text.splitlines():
        if not line or line[0] == '#':
            continue
        colon = line.find(':')
        if colon < 0:
            continue
        kv[line[:colon]] = line[colon + 1:].lstrip(' \t')

    def get_int(key, default):
        try:
            return int(kv[key].strip()) if key in kv else default
        except ValueError:
            return default

    def get_float(key):
        try:
            return float(kv[key]) if key in kv else None
        except ValueError:
            return None

    areas: List[Area] = []
    for i in range(get_int('area_count', 0)):
        prefix = 'area_%d' % i
        area = Area(kv.get(prefix + '_name', ''),
                    parse_polygon_string(kv.get(prefix + '_polygon', '')),
                    get_int(prefix + '_is_navigation', 0) != 0, [])
        for j in range(get_int(prefix + '_obstacle_count', 0)):
            op = '%s_obstacle_%d' % (prefix, j)
            poly = parse_polygon_string(kv.get(op, ''))
            if len(poly) >= 3 and not has_duplicate_obstacle(
                    [o.polygon for o in area.obstacles], poly):
                area.obstacles.append(Obstacle(poly, kv.get(op + '_name', ''),
                                               get_int(op + '_source', SOURCE_USER), False, 0))
        if len(area.polygon) >= 3:
            areas.append(area)

    lat, lon = get_float('datum_lat'), get_float('datum_lon')
    datum = (lat, lon) if lat is not None and lon is not None else None
    return areas, datum


def write_text_atomic(path: str, text: str):
    d = os.path.dirname(os.path.abspath(path))
    os.makedirs(d, exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(text)
    os.replace(tmp, path)


def save_areas_file(path: str, areas: List[Area], datum_lat=0.0, datum_lon=0.0):
    write_text_atomic(path, format_areas_dat(areas, datum_lat, datum_lon))


def load_areas_file(path: str):
    with open(path, encoding='utf-8') as f:
        return parse_areas_dat(f.read())


# ---------------------------------------------------------------------------
# Dock pose persistence
# ---------------------------------------------------------------------------

def format_dock_yaml(dock: DockPose) -> str:
    lines = ['# mower_map dock pose: map frame, metres, ENU yaw (rad).',
             '# dock_outline is the dock keep-out polygon in the DOCK-LOCAL frame',
             '# (origin = dock pose, +X = dock yaw), areas.dat polygon syntax.',
             'dock_pose_x: %.6f' % dock.x,
             'dock_pose_y: %.6f' % dock.y,
             'dock_pose_yaw: %.6f' % dock.yaw]
    if dock.outline and len(dock.outline) >= 3:
        lines.append('dock_outline: "%s"' % polygon_to_string(dock.outline))
    return '\n'.join(lines) + '\n'


def parse_dock_yaml(text: str) -> Optional[DockPose]:
    import yaml
    data = yaml.safe_load(text) or {}
    if not isinstance(data, dict) or 'dock_pose_x' not in data:
        return None
    outline = parse_polygon_string(str(data.get('dock_outline') or '')) or None
    return DockPose(float(data.get('dock_pose_x', 0.0)), float(data.get('dock_pose_y', 0.0)),
                    float(data.get('dock_pose_yaw', 0.0)), outline)


def save_dock_file(path: str, dock: DockPose):
    write_text_atomic(path, format_dock_yaml(dock))


def load_dock_file(path: str) -> Optional[DockPose]:
    with open(path, encoding='utf-8') as f:
        return parse_dock_yaml(f.read())


_NUM_CHARS = set('0123456789.-+eE')


def splice_scalar(content: str, key: str, value: str) -> Tuple[str, bool]:
    """Port of robot_yaml_scalar::SpliceScalar: replace the numeric value of
    the first INDENTED ``key:`` line, keeping comments and layout."""
    pos = 0
    while pos < len(content):
        nl = content.find('\n', pos)
        end = len(content) if nl < 0 else nl
        line = content[pos:end]
        stripped = line.lstrip(' \t')
        indent = len(line) - len(stripped)
        if indent > 0 and stripped.startswith(key + ':'):
            c = indent + len(key) + 1
            while c < len(line) and line[c] in ' \t':
                c += 1
            v = c
            while v < len(line) and line[v] in _NUM_CHARS:
                v += 1
            if v > c:
                new_line = line[:c] + value + line[v:]
                return content[:pos] + new_line + content[end:], True
        if nl < 0:
            break
        pos = nl + 1
    return content, False


def update_robot_yaml_dock_pose(path: str, x: float, y: float, yaw: float) -> bool:
    """Port of robot_yaml_scalar::UpdateDockPose (missing keys are left
    missing, file must exist). Returns False when the file cannot be read."""
    try:
        with open(path, encoding='utf-8') as f:
            content = f.read()
    except OSError:
        return False
    for key, val in (('dock_pose_x', x), ('dock_pose_y', y), ('dock_pose_yaw', yaw)):
        content, _ = splice_scalar(content, key, '%.6f' % val)
    write_text_atomic(path, content)
    return True


def read_robot_yaml_dock_pose(path: str) -> Optional[DockPose]:
    """Read dock_pose_x/y/yaw from a mowgli_robot.yaml (any nesting)."""
    try:
        with open(path, encoding='utf-8') as f:
            text = f.read()
    except OSError:
        return None
    vals = {}
    for key in ('dock_pose_x', 'dock_pose_y', 'dock_pose_yaw'):
        m = re.search(r'^[ \t]*%s:[ \t]*([-+0-9.eE]+)' % key, text, re.M)
        if m:
            try:
                vals[key] = float(m.group(1))
            except ValueError:
                pass
    if 'dock_pose_x' not in vals or 'dock_pose_y' not in vals:
        return None
    return DockPose(vals['dock_pose_x'], vals['dock_pose_y'], vals.get('dock_pose_yaw', 0.0))


# ---------------------------------------------------------------------------
# Geometry (vectorised over query points)
# ---------------------------------------------------------------------------

def points_in_polygon(px: np.ndarray, py: np.ndarray, poly: Polygon) -> np.ndarray:
    """Even-odd ray cast, same rule as upstream point_in_polygon."""
    px = np.asarray(px, dtype=float)
    py = np.asarray(py, dtype=float)
    inside = np.zeros(px.shape, dtype=bool)
    n = len(poly)
    if n < 3:
        return inside
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if yi != yj:
            cond = (yi > py) != (yj > py)
            xint = (xj - xi) * (py - yi) / (yj - yi) + xi
            inside ^= cond & (px < xint)
        j = i
    return inside


def point_in_polygon(x: float, y: float, poly: Polygon) -> bool:
    return bool(points_in_polygon(np.array([x]), np.array([y]), poly)[0])


def distance_to_polygon_edges(px, py, poly: Polygon) -> np.ndarray:
    """Distance from each point to the nearest polygon edge (closed ring)."""
    px = np.asarray(px, dtype=float)
    py = np.asarray(py, dtype=float)
    best = np.full(px.shape, np.inf)
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        if l2 <= 0.0:
            d = np.hypot(px - ax, py - ay)
        else:
            t = np.clip(((px - ax) * dx + (py - ay) * dy) / l2, 0.0, 1.0)
            d = np.hypot(px - (ax + t * dx), py - (ay + t * dy))
        best = np.minimum(best, d)
    return best


def closest_edge_point(x: float, y: float, poly: Polygon) -> Tuple[float, float, float]:
    """(cx, cy, distance) of the closest point on the polygon boundary."""
    best = (0.0, 0.0, math.inf)
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 <= 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / l2))
        cx, cy = ax + t * dx, ay + t * dy
        d = math.hypot(x - cx, y - cy)
        if d < best[2]:
            best = (cx, cy, d)
    return best


# ---------------------------------------------------------------------------
# Raster grid
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class GridSpec:
    origin_x: float
    origin_y: float
    width: int
    height: int
    resolution: float

    def cell_centres(self):
        xs = self.origin_x + (np.arange(self.width) + 0.5) * self.resolution
        ys = self.origin_y + (np.arange(self.height) + 0.5) * self.resolution
        return np.meshgrid(xs, ys)  # each (height, width)

    def index_of(self, x: float, y: float) -> Tuple[int, int]:
        """(row, col) of the cell containing (x, y); may be out of range."""
        return (int(math.floor((y - self.origin_y) / self.resolution)),
                int(math.floor((x - self.origin_x) / self.resolution)))


def bounding_box(polys: Iterable[Polygon]):
    xs, ys = [], []
    for p in polys:
        for x, y in p:
            xs.append(x)
            ys.append(y)
    if not xs:
        return None
    return min(xs), min(ys), max(xs), max(ys)


def grid_for_polygons(polys: Iterable[Polygon], resolution: float, margin: float,
                      empty_half_size: float = 5.0, max_cells: int = 16_000_000) -> GridSpec:
    """Grid covering all polygons plus ``margin``, origin snapped to a
    multiple of the resolution so grids built at different times align
    cell-for-cell. With no polygons, a square of +-empty_half_size around
    the map origin."""
    bb = bounding_box(polys)
    if bb is None:
        bb = (-empty_half_size, -empty_half_size, empty_half_size, empty_half_size)
        margin = 0.0
    x0 = math.floor((bb[0] - margin) / resolution) * resolution
    y0 = math.floor((bb[1] - margin) / resolution) * resolution
    x1 = math.ceil((bb[2] + margin) / resolution) * resolution
    y1 = math.ceil((bb[3] + margin) / resolution) * resolution
    w = max(1, int(round((x1 - x0) / resolution)))
    h = max(1, int(round((y1 - y0) / resolution)))
    if w * h > max_cells:
        raise ValueError('map extent %.0f x %.0f m at %.2f m is too large (%d cells)'
                         % (x1 - x0, y1 - y0, resolution, w * h))
    return GridSpec(round(x0, 9), round(y0, 9), w, h, resolution)


def _mask_polygon(grid: np.ndarray, spec: GridSpec, poly: Polygon, value: int,
                  margin: float = 0.0):
    """Set cells whose centre is inside ``poly`` (or within ``margin`` of
    its boundary) to ``value``. Works on the polygon's bbox window only."""
    bb = bounding_box([poly])
    if bb is None or len(poly) < 3:
        return
    r0, c0 = spec.index_of(bb[0] - margin, bb[1] - margin)
    r1, c1 = spec.index_of(bb[2] + margin, bb[3] + margin)
    r0, c0 = max(r0, 0), max(c0, 0)
    r1, c1 = min(r1, spec.height - 1), min(c1, spec.width - 1)
    if r0 > r1 or c0 > c1:
        return
    xs = spec.origin_x + (np.arange(c0, c1 + 1) + 0.5) * spec.resolution
    ys = spec.origin_y + (np.arange(r0, r1 + 1) + 0.5) * spec.resolution
    gx, gy = np.meshgrid(xs, ys)
    hit = points_in_polygon(gx, gy, poly)
    if margin > 0.0:
        hit |= distance_to_polygon_edges(gx, gy, poly) <= margin
    grid[r0:r1 + 1, c0:c1 + 1][hit] = value


def build_keepout_mask(areas: List[Area], spec: GridSpec,
                       dock_outline: Optional[Polygon] = None,
                       obstacle_margin: float = 0.0,
                       lethal_outside_areas: bool = True) -> np.ndarray:
    """Keepout mask as int8 (height, width), row 0 = lowest y (OccupancyGrid
    order). Outside every area: 100 (when lethal_outside_areas, else 0);
    inside any mowing or navigation area: 0; inside an obstacle (grown by
    obstacle_margin) or the dock outline: 100. With no areas at all the mask
    is free apart from the dock outline."""
    outside = LETHAL if (lethal_outside_areas and areas) else FREE
    mask = np.full((spec.height, spec.width), outside, dtype=np.int8)
    for area in areas:
        _mask_polygon(mask, spec, area.polygon, FREE)
    for area in areas:
        for obs in area.obstacles:
            _mask_polygon(mask, spec, obs.polygon, LETHAL, obstacle_margin)
    if dock_outline:
        _mask_polygon(mask, spec, dock_outline, LETHAL)
    return mask


@dataclass
class DockCorridor:
    """Polyline the robot must be able to drive from the dock to the lawn:
    dock -> approach pose [-> closest point on the nearest area boundary]."""
    path: List[Point]
    half_width: float
    connected: bool          # path reaches an area (or the approach is inside one)
    gap_m: float             # approach pose -> nearest area boundary (inf: no areas)

    def polygon(self, arc_steps: int = 8) -> Polygon:
        return capsule_outline(self.path, self.half_width, arc_steps)


def dock_corridor(dock: Optional[DockPose], areas: List[Area], approach_distance: float = 0.8,
                  half_width: float = 0.35, max_len: float = 5.0) -> Optional[DockCorridor]:
    """Corridor from the dock through its approach pose (dock + approach_distance
    along the dock yaw, as mower_docking computes it) on to the closest point of
    the nearest mowing/navigation area boundary. If that gap exceeds ``max_len``
    the corridor stops at the approach pose (connected=False)."""
    if dock is None:
        return None
    ax = dock.x + approach_distance * math.cos(dock.yaw)
    ay = dock.y + approach_distance * math.sin(dock.yaw)
    path = [(dock.x, dock.y), (ax, ay)]
    best = (0.0, 0.0, math.inf)
    for area in areas:
        if len(area.polygon) < 3:
            continue
        if point_in_polygon(ax, ay, area.polygon):
            best = (ax, ay, 0.0)
            break
        c = closest_edge_point(ax, ay, area.polygon)
        if c[2] < best[2]:
            best = c
    connected = best[2] <= max(0.0, float(max_len))
    if connected and best[2] > 1e-6:
        path.append((best[0], best[1]))
    return DockCorridor(path, max(0.0, float(half_width)), connected, best[2])


def capsule_outline(path: List[Point], half_width: float, arc_steps: int = 8) -> Polygon:
    """Outline of the points within ``half_width`` of the polyline ``path``
    (round caps, round outer joins). Exact for straight paths; for a bend the
    inner side uses the offset-line intersection."""
    pts = [p for i, p in enumerate(path) if i == 0 or math.hypot(p[0] - path[i - 1][0],
                                                               p[1] - path[i - 1][1]) > 1e-9]
    r = half_width
    if len(pts) == 1:
        x, y = pts[0]
        n = 4 * arc_steps
        return [(x + r * math.cos(2 * math.pi * k / n), y + r * math.sin(2 * math.pi * k / n))
                for k in range(n)]

    def arc(cx, cy, a0, a1):
        # counter-clockwise from a0 to a1
        while a1 < a0:
            a1 += 2 * math.pi
        n = max(1, int(math.ceil((a1 - a0) / (math.pi / (2 * arc_steps)))))
        return [(cx + r * math.cos(a0 + (a1 - a0) * k / n), cy + r * math.sin(a0 + (a1 - a0) * k / n))
                for k in range(n + 1)]

    def side(seq):
        """Walk ``seq`` keeping the corridor on the right (offset to the left of travel)."""
        out = []
        hd = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(seq, seq[1:])]
        for i in range(len(seq) - 1):
            a, b = seq[i], seq[i + 1]
            nx, ny = -math.sin(hd[i]), math.cos(hd[i])
            if i == 0:
                out.append((a[0] + r * nx, a[1] + r * ny))
            if i + 1 < len(hd):
                turn = math.atan2(math.sin(hd[i + 1] - hd[i]), math.cos(hd[i + 1] - hd[i]))
                if turn < 0:   # right turn: left side is the outer side -> round join
                    out.extend(arc(b[0], b[1], hd[i + 1] + math.pi / 2, hd[i] + math.pi / 2)[::-1])
                else:          # left turn: inner side -> intersection of offsets
                    half = turn / 2.0
                    d = r / max(math.cos(half), 1e-3)
                    ang = hd[i] + math.pi / 2 + half
                    out.append((b[0] + d * math.cos(ang), b[1] + d * math.sin(ang)))
            else:
                out.append((b[0] + r * nx, b[1] + r * ny))
        return out, hd

    left, hd = side(pts)
    right, hdr = side(pts[::-1])
    end, start = pts[-1], pts[0]
    # clockwise ring: left side forward, front cap, right side back, rear cap
    ring = left[:-1]
    ring += arc(end[0], end[1], hd[-1] - math.pi / 2, hd[-1] + math.pi / 2)[::-1]
    ring += right[1:-1]
    ring += arc(start[0], start[1], hdr[-1] - math.pi / 2, hdr[-1] + math.pi / 2)[::-1]
    return ring


def _distance_to_segments(px, py, path: List[Point]) -> np.ndarray:
    best = np.full(np.shape(px), np.inf)
    for (ax, ay), (bx, by) in zip(path, path[1:] if len(path) > 1 else path):
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 <= 0 else np.clip(((px - ax) * dx + (py - ay) * dy) / l2, 0.0, 1.0)
        best = np.minimum(best, np.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best


def free_corridor(grid: np.ndarray, spec: GridSpec, corridor: DockCorridor):
    """Set every cell whose centre is within half_width of the corridor path FREE."""
    r = corridor.half_width
    bb = bounding_box([corridor.path])
    r0, c0 = spec.index_of(bb[0] - r, bb[1] - r)
    r1, c1 = spec.index_of(bb[2] + r, bb[3] + r)
    r0, c0 = max(r0, 0), max(c0, 0)
    r1, c1 = min(r1, spec.height - 1), min(c1, spec.width - 1)
    if r0 > r1 or c0 > c1:
        return
    xs = spec.origin_x + (np.arange(c0, c1 + 1) + 0.5) * spec.resolution
    ys = spec.origin_y + (np.arange(r0, r1 + 1) + 0.5) * spec.resolution
    gx, gy = np.meshgrid(xs, ys)
    hit = _distance_to_segments(gx, gy, corridor.path) <= r
    grid[r0:r1 + 1, c0:c1 + 1][hit] = FREE


def build_nav_mask(areas: List[Area], spec: GridSpec,
                   nav_margin: float = 0.35,
                   obstacle_margin: float = 0.10,
                   corridor: Optional[DockCorridor] = None) -> np.ndarray:
    """Navigation mask for Nav2's global costmap (static layer / keepout filter).

    Different semantics from the mowing mask (build_keepout_mask):
    free = (mowing areas U navigation areas) dilated outward by ``nav_margin``
    MINUS obstacles dilated by ``obstacle_margin``. Everything else is 100.
    The dock outline is deliberately NOT lethal here: it is a mowing
    exclusion, and the robot parks inside it, so making it lethal would make
    the robot's own start cell unplannable. ``nav_margin`` (default half the
    footprint width + 0.08 m) lets the robot centre sit on a coverage ring a
    few cm inside the boundary without the footprint touching lethal cells.
    ``corridor`` (see dock_corridor) is freed too, so the dock and its
    approach pose stay reachable when the dock sits outside every area;
    obstacles still win over it. With no areas at all the mask is free (apart from obstacles)."""
    nav_margin = max(0.0, float(nav_margin))
    if not areas:
        mask = np.full((spec.height, spec.width), FREE, dtype=np.int8)
    else:
        mask = np.full((spec.height, spec.width), LETHAL, dtype=np.int8)
        for area in areas:
            _mask_polygon(mask, spec, area.polygon, FREE, nav_margin)
    if corridor is not None:
        free_corridor(mask, spec, corridor)
    for area in areas:
        for obs in area.obstacles:
            _mask_polygon(mask, spec, obs.polygon, LETHAL, max(0.0, float(obstacle_margin)))
    return mask


class ProgressGrid:
    """Mow-progress raster: 0 = not cut, 100 = blade passed while cutting."""

    def __init__(self, spec: GridSpec):
        self.spec = spec
        self.data = np.zeros((spec.height, spec.width), dtype=np.int8)

    def reset(self):
        self.data[:] = 0

    def resize(self, spec: GridSpec):
        """Move to a new grid (same resolution), keeping overlapping cells."""
        new = np.zeros((spec.height, spec.width), dtype=np.int8)
        if abs(spec.resolution - self.spec.resolution) < 1e-9:
            dc = int(round((self.spec.origin_x - spec.origin_x) / spec.resolution))
            dr = int(round((self.spec.origin_y - spec.origin_y) / spec.resolution))
            sr0, sc0 = max(0, -dr), max(0, -dc)
            sr1 = min(self.spec.height, spec.height - dr)
            sc1 = min(self.spec.width, spec.width - dc)
            if sr1 > sr0 and sc1 > sc0:
                new[sr0 + dr:sr1 + dr, sc0 + dc:sc1 + dc] = self.data[sr0:sr1, sc0:sc1]
        self.spec = spec
        self.data = new

    def stamp_disc(self, x: float, y: float, radius: float) -> int:
        """Stamp 100 into cells whose centre lies within ``radius`` of (x, y).
        Returns the number of newly stamped cells."""
        s = self.spec
        r0, c0 = s.index_of(x - radius, y - radius)
        r1, c1 = s.index_of(x + radius, y + radius)
        r0, c0 = max(r0, 0), max(c0, 0)
        r1, c1 = min(r1, s.height - 1), min(c1, s.width - 1)
        if r0 > r1 or c0 > c1:
            return 0
        xs = s.origin_x + (np.arange(c0, c1 + 1) + 0.5) * s.resolution
        ys = s.origin_y + (np.arange(r0, r1 + 1) + 0.5) * s.resolution
        gx, gy = np.meshgrid(xs, ys)
        hit = np.hypot(gx - x, gy - y) <= radius
        window = self.data[r0:r1 + 1, c0:c1 + 1]
        newly = int(np.count_nonzero(hit & (window != LETHAL)))
        window[hit] = LETHAL
        return newly


# ---------------------------------------------------------------------------
# Boundary classification / recovery
# ---------------------------------------------------------------------------

def robot_area_status(x: float, y: float, areas: List[Area]) -> Tuple[bool, float]:
    """(inside any area polygon, min distance to an area edge when outside)."""
    if not areas:
        return False, math.inf
    best = math.inf
    for area in areas:
        if point_in_polygon(x, y, area.polygon):
            return True, 0.0
        best = min(best, closest_edge_point(x, y, area.polygon)[2])
    return False, best


class BoundaryClassifier:
    """Port of boundary_classifier.hpp ClassifyBoundary with its debounce
    counter held in the object."""

    def __init__(self, soft_margin=0.0, lethal_margin=0.5, debounce_samples=3):
        self.soft_margin = soft_margin
        self.lethal_margin = lethal_margin
        self.debounce_samples = debounce_samples
        self.consecutive_outside = 0

    def update(self, inside_any: bool, min_edge_dist: float) -> Tuple[bool, bool]:
        soft_outside = (not inside_any) and min_edge_dist > self.soft_margin
        self.consecutive_outside = self.consecutive_outside + 1 if soft_outside else 0
        soft = soft_outside and self.consecutive_outside >= self.debounce_samples
        lethal = (not inside_any) and min_edge_dist > self.lethal_margin
        return soft, lethal


def recovery_point(x: float, y: float, areas: List[Area], offset: float = 0.8):
    """Port of on_get_recovery_point. Returns dict(success, message, x, y,
    yaw, distance_outside)."""
    res = dict(success=False, message='', x=x, y=y, yaw=0.0, distance_outside=0.0)
    if not areas:
        res['message'] = 'no areas defined'
        return res
    for area in areas:
        if point_in_polygon(x, y, area.polygon):
            res['message'] = 'already inside a mowing area'
            return res
    best = (0.0, 0.0, math.inf)
    for area in areas:
        cand = closest_edge_point(x, y, area.polygon)
        if cand[2] < best[2]:
            best = cand
    if not math.isfinite(best[2]):
        res['message'] = 'no polygon edges found'
        return res
    vx, vy = best[0] - x, best[1] - y
    vlen = math.hypot(vx, vy)
    nx, ny = (vx / vlen, vy / vlen) if vlen > 1e-6 else (0.0, 0.0)
    res.update(success=True, message='recovery pose computed',
               x=best[0] + offset * nx, y=best[1] + offset * ny,
               yaw=math.atan2(ny, nx), distance_outside=best[2])
    return res


def yaw_circular_std(yaws: Sequence[float]) -> float:
    """Circular standard deviation (rad), as upstream's yaw-convergence gate."""
    if not yaws:
        return math.inf
    c = sum(math.cos(v) for v in yaws)
    s = sum(math.sin(v) for v in yaws)
    r = math.hypot(c, s) / len(yaws)
    return math.sqrt(max(0.0, -2.0 * math.log(min(max(r, 1e-12), 1.0))))


def yaw_from_quaternion(qx, qy, qz, qw) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


# ---------------------------------------------------------------------------
# Vendor GeoJSON importer
# ---------------------------------------------------------------------------

VENDOR_AREA = 1
VENDOR_CHANNEL = 3
VENDOR_OBSTACLE = 4
VENDOR_DOCK_OUTLINE = 5
VENDOR_CHARGE_POINT = 6
VENDOR_UNDOCK_POINT = 7
VENDOR_RTK_BASE = 8


def buffer_polyline(points: Polygon, half_width: float) -> Polygon:
    """Flat-sided buffer of an open polyline with square caps (ends extended
    by half_width) and mitred joints (miter clamped to 3x). Good for the
    gently curving vendor dock->area channels."""
    pts = [p for k, p in enumerate(points) if k == 0 or p != points[k - 1]]
    if len(pts) < 2 or half_width <= 0:
        return []
    # square caps
    def unit(ax, ay, bx, by):
        L = math.hypot(bx - ax, by - ay)
        return ((bx - ax) / L, (by - ay) / L)
    ux, uy = unit(*pts[0], *pts[1])
    pts[0] = (pts[0][0] - ux * half_width, pts[0][1] - uy * half_width)
    ux, uy = unit(*pts[-2], *pts[-1])
    pts[-1] = (pts[-1][0] + ux * half_width, pts[-1][1] + uy * half_width)

    left, right = [], []
    n = len(pts)
    for k in range(n):
        if k == 0:
            dx, dy = unit(*pts[0], *pts[1])
            nx, ny, scale = -dy, dx, 1.0
        elif k == n - 1:
            dx, dy = unit(*pts[-2], *pts[-1])
            nx, ny, scale = -dy, dx, 1.0
        else:
            d1 = unit(*pts[k - 1], *pts[k])
            d2 = unit(*pts[k], *pts[k + 1])
            n1 = (-d1[1], d1[0])
            n2 = (-d2[1], d2[0])
            mx, my = n1[0] + n2[0], n1[1] + n2[1]
            ml = math.hypot(mx, my)
            if ml < 1e-9:
                nx, ny, scale = n1[0], n1[1], 1.0
            else:
                nx, ny = mx / ml, my / ml
                cos_half = nx * n1[0] + ny * n1[1]
                scale = min(1.0 / max(cos_half, 1e-9), 3.0)
        x, y = pts[k]
        left.append((x + nx * half_width * scale, y + ny * half_width * scale))
        right.append((x - nx * half_width * scale, y - ny * half_width * scale))
    return left + right[::-1]


def _overlap_estimate(a: Polygon, b: Polygon, step: float = 0.05) -> float:
    """Approximate area of a ∩ b (m^2) by sampling a's bbox."""
    bb = bounding_box([a])
    if bb is None:
        return 0.0
    xs = np.arange(bb[0] + step / 2, bb[2], step)
    ys = np.arange(bb[1] + step / 2, bb[3], step)
    if xs.size == 0 or ys.size == 0:
        return 0.0
    gx, gy = np.meshgrid(xs, ys)
    hit = points_in_polygon(gx, gy, a) & points_in_polygon(gx, gy, b)
    return float(np.count_nonzero(hit)) * step * step


@dataclass
class VendorImport:
    areas: List[Area]
    dock: Optional[DockPose]
    warnings: List[str]


def import_vendor_geojson(data: dict, channel_half_width: float = 0.35,
                          import_channels: bool = True,
                          keep_dock_zone_obstacles: bool = False) -> VendorImport:
    """Convert an Airseekers map.geojson (dict) to areas + dock pose.

    * type 1 Polygon     -> mowing area (outer ring); inner rings -> obstacles
    * type 4 Polygon     -> obstacle of the area named by parent_id, else the
                            area containing its centroid, else the area it
                            overlaps most; dropped if none. An obstacle that
                            contains the charge point is the vendor's dock
                            no-mow zone and is dropped unless
                            keep_dock_zone_obstacles (it would put the dock
                            and the undock point inside a lethal keepout).
    * type 3 LineString  -> navigation area (channel buffered by
                            channel_half_width) when import_channels
    * type 5 Polygon     -> dock keep-out outline (stored dock-local)
    * type 6 / 7 Point   -> dock pose: position = charge_point, yaw = heading
                            from charge_point to undock_point
    Only x, y of each coordinate are used (z/roll/pitch/quality ignored).
    """
    warnings: List[str] = []
    feats = data.get('features') or []

    def ftype(f):
        try:
            return int((f.get('properties') or {}).get('type'))
        except (TypeError, ValueError):
            return None

    def props(f):
        return f.get('properties') or {}

    def coords(f):
        return (f.get('geometry') or {}).get('coordinates')

    # Dock first (needed for the dock-zone test).
    charge = undock = None
    outline_map = None
    for f in feats:
        t = ftype(f)
        c = coords(f)
        if t == VENDOR_CHARGE_POINT and c:
            charge = (float(c[0]), float(c[1]))
        elif t == VENDOR_UNDOCK_POINT and c:
            undock = (float(c[0]), float(c[1]))
        elif t == VENDOR_DOCK_OUTLINE and c:
            outline_map = normalise_polygon(c[0])
    dock = None
    if charge is not None:
        yaw = 0.0
        if undock is not None and undock != charge:
            yaw = math.atan2(undock[1] - charge[1], undock[0] - charge[0])
        else:
            warnings.append('no undock_point: dock yaw defaults to 0')
        outline_local = None
        if outline_map and len(outline_map) >= 3:
            cy, sy = math.cos(yaw), math.sin(yaw)
            outline_local = [((x - charge[0]) * cy + (y - charge[1]) * sy,
                              -(x - charge[0]) * sy + (y - charge[1]) * cy)
                             for x, y in outline_map]
        dock = DockPose(charge[0], charge[1], yaw, outline_local)
    else:
        warnings.append('no charge_point (type 6): dock pose not imported')

    areas: List[Area] = []
    area_ids: Dict[str, int] = {}
    for f in feats:
        if ftype(f) != VENDOR_AREA:
            continue
        c = coords(f)
        if not c:
            continue
        outer = normalise_polygon(c[0])
        if len(outer) < 3:
            warnings.append('area %r has < 3 points, skipped' % props(f).get('name'))
            continue
        name = str(props(f).get('name') or 'area_%d' % len(areas))
        area = Area(name, outer, False, [])
        for hole in c[1:]:
            hp = normalise_polygon(hole)
            if len(hp) >= 3:
                area.obstacles.append(Obstacle(hp, name + ' hole'))
        area_ids[str(props(f).get('id'))] = len(areas)
        areas.append(area)

    mowing_count = len(areas)
    for f in feats:
        if ftype(f) != VENDOR_OBSTACLE:
            continue
        c = coords(f)
        if not c:
            continue
        poly = normalise_polygon(c[0])
        if len(poly) < 3:
            continue
        name = str(props(f).get('name') or '')
        if charge is not None and point_in_polygon(charge[0], charge[1], poly) \
                and not keep_dock_zone_obstacles:
            warnings.append('obstacle %s contains the charge point (vendor dock no-mow zone): '
                            'dropped' % props(f).get('id'))
            continue
        target = area_ids.get(str(props(f).get('parent_id')))
        if target is None:
            cx, cy = polygon_centroid(poly)
            for k in range(mowing_count):
                if point_in_polygon(cx, cy, areas[k].polygon):
                    target = k
                    break
        if target is None:
            overlaps = [(_overlap_estimate(poly, areas[k].polygon), k) for k in range(mowing_count)]
            overlaps = [o for o in overlaps if o[0] > 0.0]
            if overlaps:
                target = max(overlaps)[1]
        if target is None:
            warnings.append('obstacle %s is outside every area: dropped' % props(f).get('id'))
            continue
        if not has_duplicate_obstacle([o.polygon for o in areas[target].obstacles], poly):
            areas[target].obstacles.append(Obstacle(poly, name))

    if import_channels:
        for f in feats:
            if ftype(f) != VENDOR_CHANNEL:
                continue
            c = coords(f)
            line = [(float(p[0]), float(p[1])) for p in (c or [])]
            poly = buffer_polyline(line, channel_half_width)
            if len(poly) >= 3:
                nm = str(props(f).get('name') or '')
                areas.append(Area('channel %s' % nm if nm else 'channel', poly, True, []))
    return VendorImport(areas, dock, warnings)


def import_vendor_geojson_file(path: str, **kw) -> VendorImport:
    with open(path, encoding='utf-8') as f:
        return import_vendor_geojson(json.load(f), **kw)
