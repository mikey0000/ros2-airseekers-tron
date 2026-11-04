# SPDX-License-Identifier: GPL-3.0-or-later
"""Mowed raster of one area: what the blade actually cut (A3 coverage map, 2026-11-05; no ROS).

The mission stamps the blade disc into it on every tick while the cutter REPORTS cutting
(MowerBaseDevStatus.is_cutting, not the command) and the localisation is trusted, at the blade
point (base_link -> blade_link from /tf_static) in the map frame. It is the record of what was
cut, used for

* the honest "% mowed": cut cells of the area's mowable polygon (outer minus accepted no-go
  zones) over all of them (``percent``);
* resume by position: a plan pose counts as mowed when the blade footprint across it is
  mostly cut (``plan_progress``); a mostly mowed sub-path is skipped and the mowed lead-in of
  the next one is where the mow goes on;
* the end-of-area missed-patch report (``missed_patches``).

Persisted as an ``area_raster`` row of the resume cursor (resume.py), keyed by the area index
plus ``area_key`` (a hash of the area's outer boundary): the map has no persistent area id, so
an index that now names another polygon, or an edited boundary, clears the raster.
"""

import base64
import math
import zlib

import numpy as np

RESOLUTION_M = 0.05     # 0.22 m cut width = 4.4 cells: the disc edge is placed to +-2.5 cm
MARGIN_M = 0.5          # grid margin round the area outer (the blade may cut just outside)
# a plan pose is mowed when this fraction of its footprint cross-section (the swath width
# across the path at the blade point) is cut: one cell row of the five a 0.18 m swath spans
# may be missing (a 5 cm tracking offset, a quantised disc edge), two (half the swath) not
COVER_FRAC = 0.75
# a sub-path at least this mowed (by length) is done; the mowed lead-in of another one is
# where its resume starts when it is at least MIN_PREFIX_M long (the rule of the mid-mow
# re-plan, nogo.mowed_progress before 2026-11-05)
DONE_FRAC = 0.9
MIN_PREFIX_M = 0.5
MAX_STEP_M = 1.0        # a blade jump longer than this between two ticks is not swept


def area_key(outer):
    """Stable 31-bit id of an area boundary (1 cm rounding)."""
    txt = ';'.join('%.2f,%.2f' % (float(q[0]), float(q[1])) for q in outer or ())
    return zlib.crc32(txt.encode()) & 0x7fffffff


def blade_point(x, y, yaw, offset):
    """Map position of the blade centre for base_link at (x, y, yaw)."""
    bx, by = offset
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * bx - s * by, y + s * bx + c * by


def blade_offset_from(static, base='base_link', blade='blade_link'):
    """(x, y) of ``blade`` in ``base`` from static transforms {child: (parent, x, y, yaw)}
    (the /tf_static tree, planar), chained through intermediate frames; None when the chain
    does not reach ``base``."""
    x, y, frame, seen = 0.0, 0.0, blade, set()
    while frame != base:
        if frame in seen or frame not in static:
            return None
        seen.add(frame)
        parent, tx, ty, tyaw = static[frame]
        c, s = math.cos(tyaw), math.sin(tyaw)
        x, y = tx + c * x - s * y, ty + s * x + c * y
        frame = parent
    return x, y


class MowedRaster:
    """Boolean grid of cut cells (row = y, column = x), origin at the lower-left corner."""

    __slots__ = ('key', 'res', 'ox', 'oy', 'data', 'stamps', '_mask')

    def __init__(self, key, ox, oy, width, height, res=RESOLUTION_M, data=None):
        self.key = int(key)
        self.res = float(res)
        self.ox, self.oy = float(ox), float(oy)
        self.data = np.zeros((int(height), int(width)), dtype=bool) if data is None else data
        self.stamps = 0                 # newly cut cells since creation (change counter)
        self._mask = None               # (holes signature, mowable mask) cache

    @classmethod
    def for_area(cls, outer, key=None, res=RESOLUTION_M):
        xs = [float(q[0]) for q in outer]
        ys = [float(q[1]) for q in outer]
        ox = math.floor((min(xs) - MARGIN_M) / res) * res
        oy = math.floor((min(ys) - MARGIN_M) / res) * res
        w = int(math.ceil((max(xs) + MARGIN_M - ox) / res))
        h = int(math.ceil((max(ys) + MARGIN_M - oy) / res))
        return cls(area_key(outer) if key is None else key, ox, oy, w, h, res)

    def __eq__(self, other):
        return (isinstance(other, MowedRaster) and self.key == other.key
                and (self.res, self.ox, self.oy) == (other.res, other.ox, other.oy)
                and self.data.shape == other.data.shape
                and bool(np.array_equal(self.data, other.data)))

    def __repr__(self):
        return 'MowedRaster(key %d, %dx%d, %d cut)' % (
            self.key, self.data.shape[1], self.data.shape[0], int(self.data.sum()))

    # ---- stamping --------------------------------------------------------
    def stamp_segment(self, x0, y0, x1, y1, radius):
        """Cut every cell whose centre lies within ``radius`` of the segment (x0, y0)-(x1,
        y1): the blade disc swept from one tick to the next. Returns the new cut cells."""
        res = self.res
        h, w = self.data.shape
        c0 = max(0, int(math.floor((min(x0, x1) - radius - self.ox) / res)))
        c1 = min(w - 1, int(math.floor((max(x0, x1) + radius - self.ox) / res)))
        r0 = max(0, int(math.floor((min(y0, y1) - radius - self.oy) / res)))
        r1 = min(h - 1, int(math.floor((max(y0, y1) + radius - self.oy) / res)))
        if c0 > c1 or r0 > r1:
            return 0
        gx = self.ox + (np.arange(c0, c1 + 1) + 0.5) * res
        gy = self.oy + (np.arange(r0, r1 + 1) + 0.5) * res
        px, py = np.meshgrid(gx - x0, gy - y0)
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        t = np.zeros_like(px) if L2 <= 1e-12 else np.clip((px * dx + py * dy) / L2, 0.0, 1.0)
        hit = (px - t * dx) ** 2 + (py - t * dy) ** 2 <= radius * radius
        win = self.data[r0:r1 + 1, c0:c1 + 1]
        new = int(np.count_nonzero(hit & ~win))
        if new:
            win |= hit
            self.stamps += new
        return new

    def cut_at(self, xs, ys):
        """Bool array: the cells holding the points (xs, ys) are cut (outside = False)."""
        xs, ys = np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
        c = np.floor((xs - self.ox) / self.res).astype(int)
        r = np.floor((ys - self.oy) / self.res).astype(int)
        h, w = self.data.shape
        ok = (c >= 0) & (c < w) & (r >= 0) & (r < h)
        out = np.zeros(xs.shape, dtype=bool)
        out[ok] = self.data[r[ok], c[ok]]
        return out

    # ---- area figures ----------------------------------------------------
    def mowable_mask(self, outer, holes=()):
        """Cells whose centre lies in ``outer`` and in none of ``holes`` (cached per holes)."""
        sig = (area_key(outer), tuple(area_key(hp) for hp in holes or ()))
        if self._mask is not None and self._mask[0] == sig:
            return self._mask[1]
        mask = self._fill(outer)
        for hp in holes or ():
            if len(hp) >= 3:
                mask &= ~self._fill(hp)
        self._mask = (sig, mask)
        return mask

    def _fill(self, poly):
        """Even-odd scanline fill of ``poly`` at the cell centres."""
        h, w = self.data.shape
        out = np.zeros((h, w), dtype=bool)
        pts = [(float(q[0]), float(q[1])) for q in poly]
        if len(pts) < 3:
            return out
        ax = np.array([p[0] for p in pts])
        ay = np.array([p[1] for p in pts])
        bx, by = np.roll(ax, -1), np.roll(ay, -1)
        cx = self.ox + (np.arange(w) + 0.5) * self.res
        for r in range(h):
            y = self.oy + (r + 0.5) * self.res
            cross = (ay <= y) != (by <= y)
            if not cross.any():
                continue
            xi = ax[cross] + (y - ay[cross]) * (bx[cross] - ax[cross]) / (by[cross] - ay[cross])
            xi.sort()
            for a, b in zip(xi[0::2], xi[1::2]):
                out[r] |= (cx >= a) & (cx < b)
        return out

    def percent(self, outer, holes=()):
        """Cut share (%) of the mowable polygon; None for an empty polygon."""
        mask = self.mowable_mask(outer, holes)
        n = int(np.count_nonzero(mask))
        if n == 0:
            return None
        return 100.0 * int(np.count_nonzero(mask & self.data)) / n

    def missed_patches(self, outer, holes, blade_width, min_m2):
        """[(area m2, x, y)] uncut parts of the mowable polygon the blade fits into (an
        opening by half the blade width drops the strip along the boundary it cannot reach
        and the seams between swaths) of at least ``min_m2``, largest first."""
        left = self.mowable_mask(outer, holes) & ~self.data
        k = max(1, int(math.ceil(0.5 * float(blade_width) / self.res)))
        core = left.copy()
        for _ in range(k):
            core = _erode(core)
        for _ in range(k):
            core = _dilate(core) & left
        cell = self.res * self.res
        out = []
        for rows, cols in _components(core):
            a = len(rows) * cell
            if a >= float(min_m2):
                out.append((a, self.ox + (float(np.mean(cols)) + 0.5) * self.res,
                            self.oy + (float(np.mean(rows)) + 0.5) * self.res))
        out.sort(reverse=True)
        return out

    # ---- resume by position ------------------------------------------------
    def plan_progress(self, subpaths, half_width, offset=(0.0, 0.0)):
        """``(done, prefix)`` of a plan: the sub-path indices at least DONE_FRAC mowed (by
        length) and {k: local index} of the mowed lead-in (>= MIN_PREFIX_M) of the others.
        A pose is mowed when COVER_FRAC of its footprint cross-section (``half_width`` each
        side, at the blade point of base_link on the pose) is cut."""
        done, prefix = set(), {}
        for k, sp in enumerate(subpaths):
            if len(sp) < 2:
                continue
            cov = self.poses_mowed(sp, half_width, offset)
            seg = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(sp[:-1], sp[1:])]
            total = sum(seg)
            got = sum(s for s, c0, c1 in zip(seg, cov[:-1], cov[1:]) if c0 and c1)
            if total <= 1e-6 or got >= DONE_FRAC * total:
                done.add(k)
                continue
            i, run = 0, 0.0
            while i + 1 < len(sp) and cov[i] and cov[i + 1]:
                run += seg[i]
                i += 1
            if run >= MIN_PREFIX_M:
                prefix[k] = i
        return done, prefix

    def poses_mowed(self, poses, half_width, offset=(0.0, 0.0)):
        """[bool] per pose (see plan_progress). The heading is the pose yaw when it has one
        (a reverse leg drives backwards), else the direction of the path there."""
        n = len(poses)
        if n == 0:
            return []
        xs = np.array([float(q[0]) for q in poses])
        ys = np.array([float(q[1]) for q in poses])
        nx, ny = np.roll(xs, -1), np.roll(ys, -1)
        px, py = np.roll(xs, 1), np.roll(ys, 1)
        nx[-1], ny[-1], px[0], py[0] = xs[-1], ys[-1], xs[0], ys[0]
        yaw = np.arctan2(ny - py, nx - px)
        if all(len(q) >= 3 for q in poses):
            yaw = np.array([float(q[2]) for q in poses])
        c, s = np.cos(yaw), np.sin(yaw)
        bx = xs + c * offset[0] - s * offset[1]
        by = ys + s * offset[0] + c * offset[1]
        m = max(3, int(round(2.0 * half_width / self.res)) + 1)
        off = np.linspace(-half_width, half_width, m)
        sx = bx[:, None] - s[:, None] * off[None, :]
        sy = by[:, None] + c[:, None] * off[None, :]
        frac = self.cut_at(sx, sy).mean(axis=1)
        return [bool(v) for v in frac >= COVER_FRAC - 1e-9]

    # ---- persistence -------------------------------------------------------
    def dumps(self):
        """One text row body: ``key res ox oy width height base64(zlib(packbits))``."""
        h, w = self.data.shape
        blob = base64.b64encode(zlib.compress(np.packbits(self.data, axis=None).tobytes(), 6))
        return '%d %.3f %.3f %.3f %d %d %s' % (self.key, self.res, self.ox, self.oy, w, h,
                                               blob.decode('ascii'))

    @classmethod
    def loads(cls, args):
        """Inverse of dumps (``args``: the row's tokens); ValueError when malformed."""
        if len(args) != 7:
            raise ValueError('area_raster: 7 fields expected')
        key, res, ox, oy, w, h = (int(args[0]), float(args[1]), float(args[2]),
                                  float(args[3]), int(args[4]), int(args[5]))
        if res <= 0.0 or w <= 0 or h <= 0:
            raise ValueError('area_raster: bad grid')
        try:
            raw = zlib.decompress(base64.b64decode(args[6]))
        except (zlib.error, ValueError) as ex:
            raise ValueError('area_raster: %s' % ex)
        bits = np.unpackbits(np.frombuffer(raw, dtype=np.uint8))
        if bits.size < w * h:
            raise ValueError('area_raster: short data')
        return cls(key, ox, oy, w, h, res, bits[:w * h].astype(bool).reshape(h, w))


def _erode(a):
    out = a.copy()
    out[1:, :] &= a[:-1, :]
    out[:-1, :] &= a[1:, :]
    out[:, 1:] &= a[:, :-1]
    out[:, :-1] &= a[:, 1:]
    out[0, :] = out[-1, :] = False
    out[:, 0] = out[:, -1] = False
    return out


def _dilate(a):
    out = a.copy()
    out[1:, :] |= a[:-1, :]
    out[:-1, :] |= a[1:, :]
    out[:, 1:] |= a[:, :-1]
    out[:, :-1] |= a[:, 1:]
    return out


def _components(a):
    """4-connected components of ``a``: [(rows, cols)] index lists."""
    h, w = a.shape
    seen = np.zeros_like(a)
    out = []
    for r0, c0 in zip(*np.nonzero(a)):
        if seen[r0, c0]:
            continue
        seen[r0, c0] = True
        stack, rows, cols = [(r0, c0)], [], []
        while stack:
            r, c = stack.pop()
            rows.append(r)
            cols.append(c)
            for rr, cc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                if 0 <= rr < h and 0 <= cc < w and a[rr, cc] and not seen[rr, cc]:
                    seen[rr, cc] = True
                    stack.append((rr, cc))
        out.append((rows, cols))
    return out
