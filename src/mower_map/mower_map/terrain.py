# SPDX-License-Identifier: GPL-3.0-or-later
"""Terrain memory: per-area slope + traction raster and incident log (no ROS).

See ``docs/terrain_aware_planning.md``. One :class:`TerrainRaster` per mowing
area (0.1 m cells, origin snapped to the resolution so it lines up cell for cell
with the keepout / nav masks of :func:`mower_map.areas.grid_for_polygons`).

Layers (float32, row = y, col = x, like the masks):

``gx``, ``gy``, ``gw``  decayed sums of the terrain gradient dz/dx, dz/dy (map
                        frame) from IMU tilt, and their weight. Slope of a cell
                        = atan(|(gx, gy)| / gw). Opposite headings over the same
                        cell cancel a constant IMU mounting bias.
``bad``                 decayed traction penalty (sum of splatted incident
                        weights). Traction score 0..100 = 100 * (1 - exp(-bad)).
``seen``                decayed seconds driven over the cell (confidence).

Incidents are point events ``{id, t, kind, x, y, w, status, detail}``; their
weight decays with the same half-life as ``bad``. Status: ``open`` (new),
``confirmed`` (owner says real), ``dismissed`` (owner says ignore; no longer
counts and its splat is removed), ``keepout`` (turned into a map obstacle).

Persistence: ``terrain_<area>.npz`` (layers + grid spec) and
``terrain_<area>.json`` (incidents + summary) next to ``areas.dat``.
"""

from __future__ import annotations

import json
import math
import os
import re
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

SCHEMA_VERSION = 1

# kind -> (traction weight, is a map marker). Traction kinds raise ``bad`` around
# the event; marker kinds (boundary, detour) are only listed / clustered.
KINDS = {
    'dig_stall': (1.0, True),        # slip_detector /dig_stall latch (cmd vs measured)
    'slip': (0.5, True),             # short slip burst (wheel vs EKF), offline import
    'pivot_stall': (0.4, True),      # pivot assist creeping, pivot did not turn
    'follow_abort': (0.6, True),     # FollowPath aborted (failed to make progress)
    'transit_abort': (0.3, True),    # NavigateToPose aborted / failed
    'subpath_failed': (0.8, True),   # retries exhausted, sub-path not mowed
    'stall_guard': (0.8, True),      # docking stall/dig guard
    'stuck': (1.0, True),            # stuck guard: commanded motion, EKF pose not moving
    # mission tilt guard: tilt_monitor 'limit' band here (2026-10-09). Full traction weight
    # so ~/terrain_cost steers Nav2 transits / detours around it and the cluster can be
    # made an owner-confirmed keep-out; the slope raster itself drops samples > 35 deg.
    'steep': (1.0, True),
    'boundary': (0.0, True),         # soft boundary excursion
    'boundary_lethal': (0.0, True),  # lethal boundary excursion
    'detour': (0.0, True),           # obstacle detour
}
STATUSES = ('open', 'confirmed', 'dismissed', 'keepout')

DEFAULTS = {
    'resolution': 0.1,
    'margin_m': 1.0,               # raster extent beyond the area polygon
    'bad_half_life_days': 21.0,    # traction penalty / incident weight half-life
    'slope_half_life_days': 365.0,  # slope evidence barely fades
    'splat_sigma_m': 0.25,         # incident splat (gaussian) sigma
    'incident_prune_w': 0.05,      # open/dismissed incidents below this weight are dropped
    'cluster_radius_m': 1.0,       # single-linkage clustering distance
    'keepout_margin_m': 0.4,       # hull growth for an incident keep-out polygon
    'min_slope_cells': 50,         # slope estimate needs this many observed cells
    'min_slope_coverage': 0.25,    # ... and this fraction of the area's cells
}

Point = Tuple[float, float]


# --------------------------------------------------------------------------- helpers
def safe_name(area_name: str) -> str:
    s = re.sub(r'[^A-Za-z0-9._-]+', '_', str(area_name)).strip('_')
    return s or 'area'


def rpy_from_quaternion(x, y, z, w):
    """(roll, pitch, yaw) radians, REP-103 (x fwd, y left, z up)."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def tilt_to_gradient(roll: float, pitch: float, yaw: float) -> Tuple[float, float]:
    """Terrain gradient (dz/dx, dz/dy) in the map frame from body roll/pitch and
    the map yaw. REP-103: positive pitch = nose DOWN, positive roll = left side UP.
    """
    dzdx_b = -math.tan(pitch)          # height change per metre forward
    dzdy_b = math.tan(roll)            # height change per metre to the left
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dzdx_b - s * dzdy_b, s * dzdx_b + c * dzdy_b


def convex_hull(points: Sequence[Point]) -> List[Point]:
    """Andrew monotone chain, counter-clockwise, no repeated closing vertex."""
    pts = sorted(set((float(x), float(y)) for x, y in points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def grown_hull(points: Sequence[Point], margin: float, steps: int = 8) -> List[Point]:
    """Convex hull of the points each replaced by a ``steps``-gon of radius
    ``margin`` (a cheap Minkowski buffer; circumscribed so it never shrinks)."""
    r = margin / math.cos(math.pi / steps) if margin > 0 else 0.0
    ring = []
    for x, y in points:
        if r <= 0:
            ring.append((x, y))
            continue
        for k in range(steps):
            a = 2 * math.pi * k / steps
            ring.append((x + r * math.cos(a), y + r * math.sin(a)))
    return [(round(x, 3), round(y, 3)) for x, y in convex_hull(ring)]


def score_from_bad(bad: np.ndarray) -> np.ndarray:
    """Traction score 0..100 (100 = worst) from the penalty layer."""
    return np.clip(100.0 * (1.0 - np.exp(-np.maximum(bad, 0.0))), 0.0, 100.0)


# --------------------------------------------------------------------------- raster
class TerrainRaster:
    """Slope + traction memory of one area."""

    LAYERS = ('gx', 'gy', 'gw', 'bad', 'seen')

    def __init__(self, origin_x, origin_y, width, height, resolution, t=None):
        self.origin_x, self.origin_y = float(origin_x), float(origin_y)
        self.width, self.height = int(width), int(height)
        self.resolution = float(resolution)
        for name in self.LAYERS:
            setattr(self, name, np.zeros((self.height, self.width), np.float32))
        self.decayed_at = float(time.time() if t is None else t)

    @classmethod
    def for_polygon(cls, polygon: Sequence[Point], resolution=0.1, margin=1.0, t=None):
        xs = [p[0] for p in polygon]
        ys = [p[1] for p in polygon]
        x0 = math.floor((min(xs) - margin) / resolution) * resolution
        y0 = math.floor((min(ys) - margin) / resolution) * resolution
        x1 = math.ceil((max(xs) + margin) / resolution) * resolution
        y1 = math.ceil((max(ys) + margin) / resolution) * resolution
        return cls(round(x0, 9), round(y0, 9), max(1, int(round((x1 - x0) / resolution))),
                   max(1, int(round((y1 - y0) / resolution))), resolution, t)

    # ---- geometry
    def index(self, x, y):
        return (int(math.floor((y - self.origin_y) / self.resolution)),
                int(math.floor((x - self.origin_x) / self.resolution)))

    def contains(self, x, y):
        r, c = self.index(x, y)
        return 0 <= r < self.height and 0 <= c < self.width

    def same_grid(self, other) -> bool:
        return (abs(self.origin_x - other.origin_x) < 1e-6 and
                abs(self.origin_y - other.origin_y) < 1e-6 and
                self.width == other.width and self.height == other.height and
                abs(self.resolution - other.resolution) < 1e-9)

    def adopt(self, other: 'TerrainRaster'):
        """Copy the overlapping cells of ``other`` (an older raster of a resized
        area, same resolution and snapped origin) into this one."""
        if abs(self.resolution - other.resolution) > 1e-9:
            return
        dc = int(round((other.origin_x - self.origin_x) / self.resolution))
        dr = int(round((other.origin_y - self.origin_y) / self.resolution))
        r0, c0 = max(0, dr), max(0, dc)
        r1, c1 = min(self.height, dr + other.height), min(self.width, dc + other.width)
        if r1 <= r0 or c1 <= c0:
            return
        for name in self.LAYERS:
            getattr(self, name)[r0:r1, c0:c1] = getattr(other, name)[r0 - dr:r1 - dr,
                                                                     c0 - dc:c1 - dc]
        self.decayed_at = other.decayed_at

    # ---- updates
    def decay(self, now, bad_half_life_days, slope_half_life_days):
        dt = float(now) - self.decayed_at
        if dt <= 0:
            return
        day = 86400.0
        kb = 0.5 ** (dt / (bad_half_life_days * day)) if bad_half_life_days > 0 else 1.0
        ks = 0.5 ** (dt / (slope_half_life_days * day)) if slope_half_life_days > 0 else 1.0
        self.bad *= kb
        self.seen *= kb
        self.gx *= ks
        self.gy *= ks
        self.gw *= ks
        self.decayed_at = float(now)

    def add_tilt(self, x, y, gx, gy, w=1.0) -> bool:
        r, c = self.index(x, y)
        if not (0 <= r < self.height and 0 <= c < self.width):
            return False
        self.gx[r, c] += gx * w
        self.gy[r, c] += gy * w
        self.gw[r, c] += w
        return True

    def add_seen(self, x, y, seconds) -> bool:
        r, c = self.index(x, y)
        if not (0 <= r < self.height and 0 <= c < self.width):
            return False
        self.seen[r, c] += seconds
        return True

    def splat(self, x, y, weight, sigma):
        """Add a gaussian of peak ``weight`` (negative removes) to ``bad``."""
        if weight == 0 or sigma <= 0:
            return
        rad = int(math.ceil(3 * sigma / self.resolution))
        r, c = self.index(x, y)
        r0, r1 = max(0, r - rad), min(self.height, r + rad + 1)
        c0, c1 = max(0, c - rad), min(self.width, c + rad + 1)
        if r1 <= r0 or c1 <= c0:
            return
        ys = self.origin_y + (np.arange(r0, r1) + 0.5) * self.resolution
        xs = self.origin_x + (np.arange(c0, c1) + 0.5) * self.resolution
        d2 = (xs[None, :] - x) ** 2 + (ys[:, None] - y) ** 2
        blk = self.bad[r0:r1, c0:c1]
        blk += (weight * np.exp(-d2 / (2 * sigma * sigma))).astype(np.float32)
        np.maximum(blk, 0.0, out=blk)

    # ---- queries
    def slope_deg(self, min_w=1.0) -> np.ndarray:
        """Per-cell slope (deg); NaN where fewer than ``min_w`` samples."""
        with np.errstate(invalid='ignore', divide='ignore'):
            g = np.hypot(self.gx, self.gy) / self.gw
        out = np.degrees(np.arctan(g))
        out[self.gw < min_w] = np.nan
        return out

    def traction_score(self) -> np.ndarray:
        return score_from_bad(self.bad)

    # ---- persistence
    def to_npz(self, path):
        tmp = path + '.tmp.npz'
        np.savez_compressed(
            tmp, version=np.int32(SCHEMA_VERSION),
            spec=np.array([self.origin_x, self.origin_y, self.width, self.height,
                           self.resolution, self.decayed_at], np.float64),
            **{n: getattr(self, n) for n in self.LAYERS})
        os.replace(tmp, path)

    @classmethod
    def from_npz(cls, path):
        with np.load(path) as z:
            ox, oy, w, h, res, t = [float(v) for v in z['spec']]
            r = cls(ox, oy, int(w), int(h), res, t)
            for n in cls.LAYERS:
                if n in z and z[n].shape == (r.height, r.width):
                    setattr(r, n, z[n].astype(np.float32))
        return r


# --------------------------------------------------------------------------- slope -> mow angle
SLOPE_MODES = ('off', 'auto', 'contour', 'updown')


def slope_axis(raster: TerrainRaster, inside: Optional[np.ndarray] = None, min_w=1.0,
               min_cells=50, min_coverage=0.25):
    """Dominant slope axis of an area from its gradient cells.

    Uses the structure tensor of the per-cell gradients (sign-free, so a ridge
    or a valley does not cancel out). Returns None when there is too little data,
    else ``{axis_deg, slope_deg, p90_slope_deg, anisotropy, cells, coverage}``
    with ``axis_deg`` the uphill/downhill direction folded to [0, 180).
    """
    ok = raster.gw >= min_w
    total = int(ok.size if inside is None else inside.sum())
    if inside is not None:
        ok &= inside
    n = int(ok.sum())
    if n < min_cells or total <= 0 or n < min_coverage * total:
        return None
    gx = raster.gx[ok] / raster.gw[ok]
    gy = raster.gy[ok] / raster.gw[ok]
    sxx, syy, sxy = float((gx * gx).mean()), float((gy * gy).mean()), float((gx * gy).mean())
    tr = sxx + syy
    if tr <= 0:
        return None
    disc = math.sqrt(max(0.0, (sxx - syy) ** 2 / 4.0 + sxy * sxy))
    l1, l2 = tr / 2 + disc, tr / 2 - disc
    axis = 0.5 * math.atan2(2 * sxy, sxx - syy)
    mag = np.hypot(gx, gy)
    return {
        'axis_deg': round(math.degrees(axis) % 180.0, 2),
        'slope_deg': round(math.degrees(math.atan(math.sqrt(l1))), 2),
        'p90_slope_deg': round(math.degrees(math.atan(float(np.percentile(mag, 90)))), 2),
        'anisotropy': round((l1 - l2) / (l1 + l2), 3) if l1 + l2 > 0 else 0.0,
        'cells': n,
        'coverage': round(n / float(total), 3),
    }


def choose_mow_angle(axis: Optional[dict], mode: str, contour_above_deg=10.0,
                     min_slope_deg=3.0, min_anisotropy=0.3):
    """Swath heading (deg, [0, 180), map frame) for ``slope_mode``, or
    (None, why) to keep the planner's own choice.

    updown  = swaths along the fall line (axis); contour = across it (axis+90);
    auto    = contour at/above ``contour_above_deg``, up/down below (owner rule).
    """
    if mode not in SLOPE_MODES or mode == 'off':
        return None, 'slope_mode off'
    if axis is None:
        return None, 'not enough slope data yet'
    if axis['slope_deg'] < min_slope_deg:
        return None, 'flat (%.1f deg < %.1f)' % (axis['slope_deg'], min_slope_deg)
    if axis['anisotropy'] < min_anisotropy:
        return None, 'no dominant slope direction (anisotropy %.2f)' % axis['anisotropy']
    if mode == 'auto':
        mode = 'contour' if axis['slope_deg'] >= contour_above_deg else 'updown'
    ang = axis['axis_deg'] + (90.0 if mode == 'contour' else 0.0)
    return round(ang % 180.0, 2), '%s: slope %.1f deg, fall line %.0f deg' % (
        mode, axis['slope_deg'], axis['axis_deg'])


# --------------------------------------------------------------------------- per-area memory
class AreaTerrain:
    """Raster + incidents of one area."""

    def __init__(self, name, polygon, params=None, t=None):
        self.p = dict(DEFAULTS)
        self.p.update(params or {})
        self.name = name
        self.polygon = [tuple(p) for p in polygon]
        self.raster = TerrainRaster.for_polygon(self.polygon, self.p['resolution'],
                                                self.p['margin_m'], t)
        self.incidents: List[dict] = []
        self.next_id = 1
        self.dirty = False

    # ---- decay
    def decay(self, now):
        dt = now - self.raster.decayed_at
        self.raster.decay(now, self.p['bad_half_life_days'], self.p['slope_half_life_days'])
        if dt > 0 and self.p['bad_half_life_days'] > 0:
            k = 0.5 ** (dt / (self.p['bad_half_life_days'] * 86400.0))
            for inc in self.incidents:
                inc['w'] = inc['w'] * k
        keep = []
        for inc in self.incidents:
            if inc['status'] in ('open', 'dismissed') and inc['w'] < self.p['incident_prune_w']:
                self.dirty = True
                continue
            keep.append(inc)
        self.incidents = keep

    # ---- events
    def add_incident(self, kind, x, y, t, detail='', weight=None):
        if kind not in KINDS:
            kind_w = 0.0
        else:
            kind_w = KINDS[kind][0]
        w = kind_w if weight is None else float(weight)
        inc = {'id': self.next_id, 't': float(t), 'kind': str(kind), 'x': round(float(x), 3),
               'y': round(float(y), 3), 'w': float(max(w, 0.0)), 'w0': float(max(w, 0.0)),
               'status': 'open', 'detail': str(detail)[:200]}
        self.next_id += 1
        self.incidents.append(inc)
        if w > 0:
            self.raster.splat(x, y, w, self.p['splat_sigma_m'])
        self.dirty = True
        return inc

    def set_status(self, ids, status) -> int:
        if status not in STATUSES:
            raise ValueError('bad status %r' % status)
        n = 0
        for inc in self.incidents:
            if inc['id'] in ids and inc['status'] != status:
                if status == 'dismissed' and inc['status'] != 'dismissed' and inc['w'] > 0:
                    self.raster.splat(inc['x'], inc['y'], -inc['w'], self.p['splat_sigma_m'])
                elif inc['status'] == 'dismissed' and inc['w'] > 0:
                    self.raster.splat(inc['x'], inc['y'], inc['w'], self.p['splat_sigma_m'])
                inc['status'] = status
                n += 1
        if n:
            self.dirty = True
        return n

    def clear(self):
        """Forget the traction memory (keeps slope)."""
        self.raster.bad[:] = 0
        self.raster.seen[:] = 0
        self.incidents = []
        self.dirty = True

    # ---- clusters
    def clusters(self):
        """Single-linkage clusters of the live incidents (open/confirmed).

        Returns [{id, ids, kinds, n, weight, x, y, hull, status}] sorted by weight,
        ``id`` = smallest incident id (stable while that incident lives).
        """
        live = [i for i in self.incidents if i['status'] in ('open', 'confirmed')]
        r2 = self.p['cluster_radius_m'] ** 2
        parent = list(range(len(live)))

        def find(a):
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a
        for a in range(len(live)):
            for b in range(a + 1, len(live)):
                if (live[a]['x'] - live[b]['x']) ** 2 + (live[a]['y'] - live[b]['y']) ** 2 <= r2:
                    parent[find(a)] = find(b)
        groups: Dict[int, List[dict]] = {}
        for k, inc in enumerate(live):
            groups.setdefault(find(k), []).append(inc)
        out = []
        for g in groups.values():
            pts = [(i['x'], i['y']) for i in g]
            kinds: Dict[str, int] = {}
            for i in g:
                kinds[i['kind']] = kinds.get(i['kind'], 0) + 1
            out.append({
                'id': min(i['id'] for i in g),
                'ids': sorted(i['id'] for i in g),
                'kinds': kinds,
                'n': len(g),
                'weight': round(sum(i['w'] for i in g), 3),
                'x': round(sum(p[0] for p in pts) / len(pts), 3),
                'y': round(sum(p[1] for p in pts) / len(pts), 3),
                'hull': grown_hull(pts, self.p['keepout_margin_m']),
                'status': 'confirmed' if any(i['status'] == 'confirmed' for i in g) else 'open',
                'last_t': max(i['t'] for i in g),
            })
        out.sort(key=lambda c: -c['weight'])
        return out

    # ---- summary
    def summary(self, slope_mode='off', contour_above_deg=10.0, inside=None):
        sl = self.raster.slope_deg()
        score = self.raster.traction_score()
        axis = slope_axis(self.raster, inside, min_cells=self.p['min_slope_cells'],
                          min_coverage=self.p['min_slope_coverage'])
        angle, why = choose_mow_angle(axis, slope_mode, contour_above_deg)
        finite = sl[np.isfinite(sl)]
        return {
            'area': self.name,
            'slope': axis,
            'max_slope_deg': round(float(finite.max()), 2) if finite.size else None,
            'slope_mode': slope_mode,
            'slope_mow_angle_deg': angle,
            'slope_angle_why': why,
            'traction_cells_bad': int((score >= 50).sum()),
            'traction_max': round(float(score.max()), 1) if score.size else 0.0,
            'incidents': len(self.incidents),
            'clusters': self.clusters(),
        }

    # ---- persistence
    def save(self, maps_dir):
        base = os.path.join(maps_dir, 'terrain_%s' % safe_name(self.name))
        os.makedirs(maps_dir, exist_ok=True)
        self.raster.to_npz(base + '.npz')
        doc = {'version': SCHEMA_VERSION, 'area': self.name, 'polygon': self.polygon,
               'next_id': self.next_id, 'incidents': self.incidents,
               'saved_at': time.time()}
        tmp = base + '.json.tmp'
        with open(tmp, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, indent=1, sort_keys=True)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, base + '.json')
        self.dirty = False

    @classmethod
    def load(cls, maps_dir, name, polygon, params=None, t=None):
        """Loads the stored memory of ``name`` (a fresh one when missing or
        unreadable). The raster is re-gridded to the current polygon."""
        a = cls(name, polygon, params, t)
        base = os.path.join(maps_dir, 'terrain_%s' % safe_name(name))
        try:
            if os.path.exists(base + '.npz'):
                old = TerrainRaster.from_npz(base + '.npz')
                if a.raster.same_grid(old):
                    a.raster = old
                else:
                    a.raster.adopt(old)
            if os.path.exists(base + '.json'):
                with open(base + '.json', 'r', encoding='utf-8') as fh:
                    doc = json.load(fh)
                a.incidents = [i for i in doc.get('incidents', []) if isinstance(i, dict)
                               and {'id', 'x', 'y', 'kind'} <= set(i)]
                for i in a.incidents:
                    i.setdefault('w', 0.0)
                    i.setdefault('status', 'open')
                    i.setdefault('t', 0.0)
                a.next_id = max([int(doc.get('next_id', 1))] +
                                [int(i['id']) + 1 for i in a.incidents])
        except (OSError, ValueError, KeyError) as exc:  # corrupt file: start over
            a = cls(name, polygon, params, t)
            a.load_error = str(exc)
        return a


def stamp_into(grid: np.ndarray, grid_origin, resolution, raster: TerrainRaster,
               values: np.ndarray, reduce=np.maximum):
    """Copy ``values`` (raster-shaped) into ``grid`` (map-mask shaped) with the
    cell offset of the two snapped origins, combining with ``reduce``."""
    gox, goy = grid_origin
    dc = int(round((raster.origin_x - gox) / resolution))
    dr = int(round((raster.origin_y - goy) / resolution))
    gh, gw = grid.shape
    r0, c0 = max(0, dr), max(0, dc)
    r1, c1 = min(gh, dr + raster.height), min(gw, dc + raster.width)
    if r1 <= r0 or c1 <= c0:
        return grid
    sub = values[r0 - dr:r1 - dr, c0 - dc:c1 - dc].astype(grid.dtype)
    grid[r0:r1, c0:c1] = reduce(grid[r0:r1, c0:c1], sub)
    return grid


def traction_cost(score: np.ndarray, threshold=20.0, cost_max=60) -> np.ndarray:
    """Nav cost 0..cost_max (OccupancyGrid units) from the traction score:
    0 below ``threshold``, linear to ``cost_max`` at score 100. ``cost_max`` < 65
    keeps it a preference (never lethal) for a non-trinary StaticLayer."""
    s = np.clip((score - threshold) / max(1e-6, 100.0 - threshold), 0.0, 1.0)
    out = np.round(s * cost_max).astype(np.int16)
    out[score < threshold] = 0
    return out
