# SPDX-License-Identifier: GPL-3.0-or-later
"""Non-grass memory: per-area evidence raster from the grass segmentation (no ROS).

2026-10-09, docs/grass_segmentation.md. ``seg_ros/nongrass_projector`` projects the seg_ros
class mask of the front camera onto the ground and sends one vote per 0.1 m map cell per
accepted frame (``/ai/seg/ground_cells``): non-grass (flower bed, gravel, paving, water,
toys) or grass (grass / soil). This module turns those votes into a SOFT, remembered cost:

* Evidence per cell, decayed with ``half_life_days``: ``hit`` (non-grass votes) and ``miss``
  (grass votes). A grass vote is as strong as a non-grass vote, so a toy that was picked up,
  or a shadow that moved, is cleared the next time the camera sees grass there.
* Temporal + viewpoint confirmation. A cell becomes non-grass only when ALL hold:
  ``hit >= confirm_hits`` (several frames), ``hit / (hit + miss) >= confirm_ratio`` (mostly
  non-grass when seen), and ``spread >= confirm_spread_m``: the camera positions that saw it
  as non-grass span at least that distance. One frame, one viewpoint (a parked robot, the
  robot's own shadow which moves with it and so never stays on one cell, glare at one angle)
  never confirms. Hysteresis: a confirmed cell is released only below ``release_hits`` or
  ``release_ratio``, so the cost does not flicker at the edge of a bed.
* Cost (OccupancyGrid units 0..100, read by a non-trinary Nav2 StaticLayer): confirmed cells
  ``cost_max`` (default 75 -> costmap 190 of 254; lethal is >= 100 in the layer, so never
  lethal), a linear ``halo_m`` ring around them (the planner plans a point; the 0.5 m body
  would brush the bed edge). Only inside the area polygon and only for areas whose
  ``avoid_non_grass`` setting is on. The mow area therefore always stays plannable.
* Clusters of confirmed cells (8-connected) are the GUI's keep-out SUGGESTIONS: the owner
  may turn one into a map obstacle (then coverage plans around it) or dismiss it (adds
  ``dismiss_weight`` of grass evidence so it needs a lot of new non-grass votes to return).

Layers (float32, rows = y, like the masks; same snapped grid as terrain.TerrainRaster):
``hit``, ``miss``, ``vx0``/``vy0`` (first non-grass viewpoint, NaN = none), ``spread``
(max distance of a later non-grass viewpoint from it) and ``confirmed`` (0/1).
Persistence: ``nongrass_<area>.npz`` next to ``areas.dat``.
"""

from __future__ import annotations

import math
import os
from typing import Dict, List, Optional, Sequence

import numpy as np

from mower_map import areas as core
from mower_map.terrain import TerrainRaster, grown_hull, safe_name

DEFAULTS = {
    'resolution': 0.1,
    'margin_m': 0.5,             # raster beyond the polygon (votes kept, never costed)
    'half_life_days': 14.0,
    'full_weight_samples': 2,    # a cell vote from fewer samples counts n / this
    'confirm_hits': 3.0,
    'confirm_ratio': 0.75,
    'confirm_spread_m': 0.5,
    'release_hits': 1.5,
    'release_ratio': 0.5,
    'min_evidence': 0.5,         # below: "never seen" (-1) in the GUI grid
    # 75 -> costmap 190 (non-trinary StaticLayer: v/100*254): above area_transit_cost 60
    # (151, the nav mask's cost of a mowing area when a drawn path exists; with use_maximum
    # a lower value would vanish under it) and below the soft band 90 (229) and the
    # inscribed 253. Never lethal (lethal is >= 100 in the layer).
    'cost_max': 75,
    'halo_m': 0.2,
    'min_cluster_m2': 0.25,      # smaller clusters are not suggested as keep-outs
    'keepout_margin_m': 0.1,
    'dismiss_weight': 20.0,
}


class NonGrassRaster(TerrainRaster):
    LAYERS = ('hit', 'miss', 'vx0', 'vy0', 'spread', 'confirmed')

    def __init__(self, origin_x, origin_y, width, height, resolution, t=None):
        super().__init__(origin_x, origin_y, width, height, resolution, t)
        self.vx0[:] = np.nan
        self.vy0[:] = np.nan

    def decay(self, now, half_life_days):  # noqa: D102 - different signature on purpose
        dt = float(now) - self.decayed_at
        if dt <= 0:
            return
        k = 0.5 ** (dt / (half_life_days * 86400.0)) if half_life_days > 0 else 1.0
        self.hit *= k
        self.miss *= k
        # evidence gone -> forget the viewpoints too (a new bed there starts from scratch)
        gone = (self.hit < 1e-3) & (self.miss < 1e-3)
        self.vx0[gone] = np.nan
        self.vy0[gone] = np.nan
        self.spread[gone] = 0.0
        self.decayed_at = float(now)

    def cells(self, xs, ys):
        """(rows, cols, ok) of map points."""
        xs = np.asarray(xs, dtype=float)
        ys = np.asarray(ys, dtype=float)
        c = np.floor((xs - self.origin_x) / self.resolution).astype(np.int64)
        r = np.floor((ys - self.origin_y) / self.resolution).astype(np.int64)
        ok = (r >= 0) & (r < self.height) & (c >= 0) & (c < self.width)
        return r, c, ok


def evidence_ratio(hit, miss):
    with np.errstate(invalid='ignore', divide='ignore'):
        return np.where(hit + miss > 0, hit / np.maximum(hit + miss, 1e-9), 0.0)


def halo_cost(confirmed: np.ndarray, cost_max: int, halo_m: float,
              resolution: float) -> np.ndarray:
    """``cost_max`` on confirmed cells, falling linearly to ~0 at ``halo_m`` beyond them
    (Euclidean over a small stencil; max-combined)."""
    out = np.where(confirmed, int(cost_max), 0).astype(np.int16)
    k = int(math.ceil(max(0.0, halo_m) / resolution))
    if k <= 0 or not confirmed.any():
        return out
    h, w = confirmed.shape
    span = halo_m + resolution
    for dr in range(-k, k + 1):
        for dc in range(-k, k + 1):
            d = math.hypot(dr, dc) * resolution
            if (dr == 0 and dc == 0) or d > halo_m + 1e-9:
                continue
            v = int(round(cost_max * (1.0 - d / span)))
            if v <= 0:
                continue
            src = confirmed[max(0, -dr):h - max(0, dr), max(0, -dc):w - max(0, dc)]
            dst = out[max(0, dr):h - max(0, -dr), max(0, dc):w - max(0, -dc)]
            np.maximum(dst, np.where(src, v, 0).astype(np.int16), out=dst)
    return out


def components(mask: np.ndarray) -> List[np.ndarray]:
    """8-connected components of a bool raster -> list of flat index arrays (sorted)."""
    h, w = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    out = []
    for start in np.flatnonzero(mask):
        r0, c0 = divmod(int(start), w)
        if seen[r0, c0]:
            continue
        stack = [(r0, c0)]
        seen[r0, c0] = True
        cells = []
        while stack:
            r, c = stack.pop()
            cells.append(r * w + c)
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] and not seen[rr, cc]:
                        seen[rr, cc] = True
                        stack.append((rr, cc))
        out.append(np.array(sorted(cells), dtype=np.int64))
    return out


class AreaNonGrass:
    """Non-grass memory of one mowing area."""

    def __init__(self, name, polygon, params=None, t=None):
        self.name = name
        self.polygon = [tuple(p) for p in polygon]
        self.p = dict(DEFAULTS)
        self.p.update(params or {})
        self.raster = NonGrassRaster.for_polygon(self.polygon, self.p['resolution'],
                                                 self.p['margin_m'], t)
        self.dirty = False
        self._inside = None

    # ---- geometry
    def inside(self) -> np.ndarray:
        if self._inside is None:
            r = self.raster
            spec = core.GridSpec(r.origin_x, r.origin_y, r.width, r.height, r.resolution)
            cx, cy = spec.cell_centres()
            self._inside = core.points_in_polygon(cx.ravel(), cy.ravel(),
                                                  self.polygon).reshape(r.height, r.width)
        return self._inside

    # ---- updates
    def observe(self, xs, ys, nongrass, samples=None, vx=None, vy=None) -> int:
        """One frame's cell votes (map metres). ``samples`` = pixel samples per cell (weight
        min(1, n / full_weight_samples)); ``vx, vy`` = camera position (scalar or per point).
        Returns the number of cells that fell in this raster."""
        r, c, ok = self.raster.cells(xs, ys)
        if not ok.any():
            return 0
        n = len(r)
        ng = np.asarray(nongrass, dtype=bool).reshape(-1)
        w = np.ones(n) if samples is None else np.minimum(
            1.0, np.asarray(samples, dtype=float) / max(1.0, float(self.p['full_weight_samples'])))
        vx = np.broadcast_to(np.asarray(np.nan if vx is None else vx, dtype=float), (n,))
        vy = np.broadcast_to(np.asarray(np.nan if vy is None else vy, dtype=float), (n,))
        rs = self.raster
        hit_i = ok & ng
        miss_i = ok & ~ng
        np.add.at(rs.miss, (r[miss_i], c[miss_i]), w[miss_i].astype(np.float32))
        if hit_i.any():
            rr, cc = r[hit_i], c[hit_i]
            np.add.at(rs.hit, (rr, cc), w[hit_i].astype(np.float32))
            hx, hy = vx[hit_i], vy[hit_i]
            have = np.isfinite(hx) & np.isfinite(hy)
            first = have & np.isnan(rs.vx0[rr, cc])
            rs.vx0[rr[first], cc[first]] = hx[first]
            rs.vy0[rr[first], cc[first]] = hy[first]
            d = np.hypot(hx - rs.vx0[rr, cc], hy - rs.vy0[rr, cc])
            d = np.where(have & np.isfinite(d), d, 0.0).astype(np.float32)
            np.maximum.at(rs.spread, (rr, cc), d)
        self.dirty = True
        return int(ok.sum())

    def decay(self, now):
        self.raster.decay(now, float(self.p['half_life_days']))

    def update_confirmed(self) -> bool:
        """Re-evaluate confirmation with hysteresis. Returns True if the set changed."""
        rs = self.raster
        ratio = evidence_ratio(rs.hit, rs.miss)
        confirm = (rs.hit >= self.p['confirm_hits']) & (ratio >= self.p['confirm_ratio']) & \
            (rs.spread >= self.p['confirm_spread_m'])
        release = (rs.hit < self.p['release_hits']) | (ratio < self.p['release_ratio'])
        was = rs.confirmed > 0.5
        now = (was & ~release) | confirm
        if np.array_equal(now, was):
            return False
        rs.confirmed[:] = now.astype(np.float32)
        self.dirty = True
        return True

    def clear(self):
        t = self.raster.decayed_at
        r = self.raster
        self.raster = NonGrassRaster(r.origin_x, r.origin_y, r.width, r.height, r.resolution, t)
        self.dirty = True

    # ---- outputs
    def confirmed_mask(self) -> np.ndarray:
        return (self.raster.confirmed > 0.5) & self.inside()

    def cost(self, enabled=True) -> np.ndarray:
        """Nav cost raster (0..cost_max); zeros when the area opted out."""
        if not enabled:
            return np.zeros((self.raster.height, self.raster.width), np.int16)
        c = halo_cost(self.confirmed_mask(), int(self.p['cost_max']), float(self.p['halo_m']),
                      self.raster.resolution)
        c[~self.inside()] = 0
        return c

    def grid(self) -> np.ndarray:
        """GUI raster: non-grass probability 0..100 where observed, -1 never seen;
        confirmed cells are forced to >= 50 so a heat map shows them."""
        rs = self.raster
        ev = rs.hit + rs.miss
        g = np.round(100.0 * evidence_ratio(rs.hit, rs.miss)).astype(np.int16)
        g[ev < self.p['min_evidence']] = -1
        conf = rs.confirmed > 0.5
        g[conf] = np.maximum(g[conf], 50)
        return g

    def clusters(self) -> List[dict]:
        rs = self.raster
        res = rs.resolution
        out = []
        for idx in components(self.confirmed_mask()):
            rows, cols = np.divmod(idx, rs.width)
            area = len(idx) * res * res
            xs = rs.origin_x + (cols + 0.5) * res
            ys = rs.origin_y + (rows + 0.5) * res
            # cell corners (+ margin) so a single cell still gives a real polygon
            m = float(self.p['keepout_margin_m']) + res * 0.5 * math.sqrt(2.0)
            hull = grown_hull(list(zip(xs.tolist(), ys.tolist())), m)
            out.append({'id': int(idx[0]), 'cells': int(len(idx)),
                        'area_m2': round(area, 3),
                        'x': round(float(xs.mean()), 3), 'y': round(float(ys.mean()), 3),
                        'suggest_keepout': bool(area >= float(self.p['min_cluster_m2'])),
                        'hull': [[x, y] for x, y in hull]})
        return out

    def dismiss(self, cluster_id: int) -> bool:
        """Owner: "this is lawn". Wipes the cluster's non-grass evidence and adds
        ``dismiss_weight`` of grass evidence to every cell of it."""
        rs = self.raster
        for idx in components(self.confirmed_mask()):
            if int(idx[0]) != int(cluster_id):
                continue
            rows, cols = np.divmod(idx, rs.width)
            rs.hit[rows, cols] = 0.0
            rs.miss[rows, cols] += float(self.p['dismiss_weight'])
            rs.vx0[rows, cols] = np.nan
            rs.vy0[rows, cols] = np.nan
            rs.spread[rows, cols] = 0.0
            rs.confirmed[rows, cols] = 0.0
            self.dirty = True
            return True
        return False

    def summary(self, enabled=True) -> dict:
        conf = self.confirmed_mask()
        res = self.raster.resolution
        cl = self.clusters()
        return {'area': self.name, 'enabled': bool(enabled),
                'confirmed_cells': int(conf.sum()),
                'confirmed_m2': round(float(conf.sum()) * res * res, 3),
                'observed_cells': int(((self.raster.hit + self.raster.miss)
                                       >= self.p['min_evidence']).sum()),
                'clusters': cl}

    # ---- persistence
    def path(self, maps_dir):
        return os.path.join(maps_dir, 'nongrass_%s.npz' % safe_name(self.name))

    def save(self, maps_dir):
        os.makedirs(maps_dir, exist_ok=True)
        self.raster.to_npz(self.path(maps_dir))
        self.dirty = False

    @classmethod
    def load(cls, maps_dir, name, polygon, params=None, t=None):
        """Stored memory of ``name`` re-gridded to the current polygon (fresh when missing;
        ``load_error`` set when unreadable)."""
        a = cls(name, polygon, params, t)
        path = a.path(maps_dir)
        try:
            if os.path.exists(path):
                old = NonGrassRaster.from_npz(path)
                if a.raster.same_grid(old):
                    a.raster = old
                else:
                    a.raster.adopt(old)
        except (OSError, ValueError, KeyError) as exc:
            a = cls(name, polygon, params, t)
            a.load_error = str(exc)
        return a


def cloud_fields(fields: Sequence, data: bytes, point_step: int, width: int,
                 names=('x', 'y', 'nongrass', 'weight', 'vx', 'vy')) -> Optional[Dict]:
    """float32 fields of a little-endian PointCloud2 -> {name: array} (None if a field
    is missing or not float32)."""
    off = {}
    for f in fields:
        off[f.name] = (int(f.offset), int(f.datatype))
    if any(n not in off or off[n][1] != 7 for n in names):   # 7 = PointField.FLOAT32
        return None
    if width <= 0:
        return {n: np.zeros(0, np.float32) for n in names}
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    if buf.size < point_step * width:
        return None
    rows = buf[:point_step * width].reshape(width, point_step)
    return {n: rows[:, off[n][0]:off[n][0] + 4].copy().view('<f4').reshape(-1) for n in names}
