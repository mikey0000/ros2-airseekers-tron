# SPDX-License-Identifier: GPL-3.0-or-later
"""No-go zones in the mission (option C of docs/analysis/2026-10-23_obstacle_nogo_options.md),
ROS-free helpers used by ``fsm/phases/nogo.py``:

* :func:`zone_polygon` / :func:`zone_in_plan` - a zone as a coverage hole;
* :func:`mowed_progress` - which sub-paths of a re-plan are already mowed (progress kept);
* :func:`plan_pivot` - the pre-pivot sweep check and its fallbacks (a)-(d);
* :func:`edge_detour` - an edge-lap stretch bent inward around a disc (B6, owner).

The sweep / footprint geometry is ``mower_map.nogo_zones`` (the tracker's on-path rule)."""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence

try:
    from mower_map import nogo_zones as nz
except ImportError:          # source tree without the mower_map install (unit tests)
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'mower_map'))
    from mower_map import nogo_zones as nz      # noqa: E402

Zone = nz.Zone
ZoneSet = nz.ZoneSet


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------------------
# zones as coverage holes
# ---------------------------------------------------------------------------

def zone_polygon(x: float, y: float, r: float, n: int = 16) -> List[tuple]:
    """Polygon with ``n`` vertices around the disc (circumscribed: the disc lies inside)."""
    R = float(r) / math.cos(math.pi / n)
    return [(x + R * math.cos(2.0 * math.pi * k / n), y + R * math.sin(2.0 * math.pi * k / n))
            for k in range(n)]


def _pip(x, y, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i][0], poly[i][1]
        x2, y2 = poly[(i + 1) % n][0], poly[(i + 1) % n][1]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def _disc_samples(x, y, r, n=64):
    """``n`` equal-area sample points of the disc (sunflower pattern)."""
    g = math.pi * (3.0 - math.sqrt(5.0))
    return [(x + r * math.sqrt((k + 0.5) / n) * math.cos(k * g),
             y + r * math.sqrt((k + 0.5) / n) * math.sin(k * g)) for k in range(n)]


def zone_in_plan(zone, obstacles, frac: float = 0.5) -> bool:
    """The zone is already a hole of the plan: at least ``frac`` of its disc lies inside
    one of ``obstacles`` (polygons). Field 2026-10-28: a zone stacked beside a hole
    re-planned again; a zone that outgrew its hole (re-sized) is planned again."""
    pts = _disc_samples(zone.x, zone.y, max(float(zone.r), 1e-3))
    for o in obstacles or []:
        if len(o) < 3:
            continue
        xs = [q[0] for q in o]
        ys = [q[1] for q in o]
        if zone.x + zone.r < min(xs) or zone.x - zone.r > max(xs) or \
                zone.y + zone.r < min(ys) or zone.y - zone.r > max(ys):
            continue
        inside = sum(1 for (px, py) in pts if _pip(px, py, o))
        if inside >= frac * len(pts):
            return True
    return False


def zone_in_area(zone, outer, margin: float = 0.3) -> bool:
    """Zone centre inside the area outline, or within ``margin`` outside its line."""
    if not outer or len(outer) < 3:
        return False
    if _pip(zone.x, zone.y, outer):
        return True
    best = math.inf
    n = len(outer)
    for i in range(n):
        a, b = outer[i], outer[(i + 1) % n]
        dx, dy = b[0] - a[0], b[1] - a[1]
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((zone.x - a[0]) * dx +
                                                       (zone.y - a[1]) * dy) / L2))
        best = min(best, math.hypot(a[0] + t * dx - zone.x, a[1] + t * dy - zone.y))
    return best <= margin


def path_hits_zone(paths, zone, pad: float = 0.0) -> str:
    """'corridor' / 'pivot' / '' for the first of ``paths`` the body would touch the zone on."""
    for pts in paths:
        if len(pts) < 2:
            continue
        hit = nz.path_hit([(q[0], q[1]) for q in pts], zone.x, zone.y, zone.r, pad=pad)
        if hit:
            return hit
    return ''


# ---------------------------------------------------------------------------
# progress kept across a re-plan
# ---------------------------------------------------------------------------

class _Grid:
    """Points hashed into cells of ``tol`` for 'is any point within tol' queries."""

    def __init__(self, pts, tol):
        self.tol = float(tol)
        self.cells = {}
        for x, y in pts:
            self.cells.setdefault((int(math.floor(x / tol)), int(math.floor(y / tol))),
                                  []).append((x, y))

    def near(self, x, y):
        cx, cy = int(math.floor(x / self.tol)), int(math.floor(y / self.tol))
        t2 = self.tol * self.tol
        for i in (cx - 1, cx, cx + 1):
            for j in (cy - 1, cy, cy + 1):
                for (px, py) in self.cells.get((i, j), ()):
                    if (px - x) ** 2 + (py - y) ** 2 <= t2:
                        return True
        return False


def _densify(poly, step):
    out = []
    for a, b in zip(poly[:-1], poly[1:]):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(math.ceil(L / step)))
        for k in range(n):
            out.append((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n))
    if poly:
        out.append((poly[-1][0], poly[-1][1]))
    return out


def mowed_progress(subpaths, mowed, tol: float, done_frac: float = 0.9,
                   min_prefix_m: float = 0.5):
    """Map mowed polylines (the old plan's done parts) onto a new plan's ``subpaths``.

    A pose is mowed when it lies within ``tol`` of a mowed polyline (the swath around it).
    Returns ``(done, prefix)``: the set of sub-path indices at least ``done_frac`` mowed
    (by length), and {k: local index} of the mowed lead-in (>= ``min_prefix_m``) of the
    others - where a resume of that sub-path starts."""
    pts = []
    for pl in mowed or []:
        if len(pl) >= 2:
            pts.extend(_densify([(q[0], q[1]) for q in pl], 0.5 * tol))
        elif len(pl) == 1:
            pts.append((pl[0][0], pl[0][1]))
    done, prefix = set(), {}
    if not pts:
        return done, prefix
    g = _Grid(pts, tol)
    for k, sp in enumerate(subpaths):
        if len(sp) < 2:
            continue
        cov = [g.near(q[0], q[1]) for q in sp]
        seg = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(sp[:-1], sp[1:])]
        total = sum(seg)
        got = sum(s for s, c0, c1 in zip(seg, cov[:-1], cov[1:]) if c0 and c1)
        if total <= 1e-6 or got >= done_frac * total:
            done.add(k)
            continue
        i, run = 0, 0.0
        while i + 1 < len(sp) and cov[i] and cov[i + 1]:
            run += seg[i]
            i += 1
        if run >= min_prefix_m:
            prefix[k] = i
    return done, prefix


# ---------------------------------------------------------------------------
# pre-pivot sweep check (fallbacks a-d)
# ---------------------------------------------------------------------------

@dataclass
class PivotPlan:
    kind: str                        # clear | other | backup | kturn | shift | blocked
    steps: list = field(default_factory=list)   # [('spin', rad) | ('backup', m) | ('drive', m)]
    why: str = ''
    hits: list = field(default_factory=list)    # zones the planned sweep touches


def _sweep(x, y, yaw, ang, discs):
    d = 1 if ang > 0 else -1
    return nz.pivot_sweep_hits(x, y, yaw, yaw + ang, d, discs)


# Living classes (obstacle_tracker never_classes): a candidate det track of one of these in a
# pivot sweep is a person / animal standing there, not an obstacle to route round.
LIVING_CLASSES = ('person', 'dog', 'cat', 'hedgehog', 'rabbit')


def living_sweep_hits(x: float, y: float, yaw: float, delta: float, discs: Sequence) -> list:
    """Labels of the living-class zones the pivot of ``delta`` at (x, y, yaw) sweeps."""
    live = [z for z in (discs or []) if z.state != 'mark' and z.label in LIVING_CLASSES]
    if abs(delta) < 1e-6 or not live:
        return []
    return sorted({h.zone.label for h in _sweep(x, y, yaw, delta, live)})


def _sweep_poses(x, y, yaw, ang, step_deg=5.0):
    n = max(1, int(math.ceil(abs(math.degrees(ang)) / step_deg)))
    return [(x, y, yaw + ang * k / n) for k in range(n + 1)]


def _line_poses(x, y, yaw, s, step=0.05):
    n = max(1, int(math.ceil(abs(s) / step)))
    return [(x + math.cos(yaw) * s * k / n, y + math.sin(yaw) * s * k / n, yaw)
            for k in range(n + 1)]


def _line_clear(x, y, yaw, s, discs):
    poses = _line_poses(x, y, yaw, s)
    return not any(bool(nz.poses_hit_disc(poses, z.x, z.y, z.r).any()) for z in discs)


def plan_pivot(x: float, y: float, yaw: float, delta: float, discs: Sequence,
               body: Optional[Callable] = None, backoff: Optional[Callable] = None,
               backup_m: float = 0.3, shifts: Sequence[float] = (0.3, 0.15, -0.15),
               kturn: bool = True) -> PivotPlan:
    """The mission's pre-pivot check: pivoting in place at (x, y) from ``yaw`` by ``delta``
    (signed; |delta| may exceed pi = the planned way round) against ``discs`` (zones and
    static marks). ``body(poses) -> worst excess`` (m beyond the body limit, <= 0 inside;
    None = no body check); ``backoff(pose, dist) -> dist`` = the longest straight reverse
    <= dist that keeps the body within its limits (0 = no room). In order:

    (a) the other turn direction, if its sweep is clear and the body stays within limits;
    (b) a capped back-up (<= ``backup_m``), then the pivot (either direction);
    (c) a K-turn: half the turn, a back-up, the other half;
    (d) the pivot point moved along the heading by ``shifts`` (+ later, - earlier);
    else ``blocked`` (the caller re-plans or skips, (e)).
    The body allowance is the planned pivot's own (a planned corner may already reach out)."""
    discs = list(discs or [])
    if abs(delta) < 1e-6 or not discs:
        return PivotPlan('clear')
    hits = _sweep(x, y, yaw, delta, discs)
    if not hits:
        return PivotPlan('clear')
    names = ', '.join('%s (%.2f, %.2f) r %.2f' % (h.zone.label or h.zone.state, h.zone.x,
                                                   h.zone.y, h.zone.r) for h in hits[:3])
    planned_body = body(_sweep_poses(x, y, yaw, delta)) if body else 0.0
    allow = max(0.0, planned_body if planned_body is not None else 0.0) + 1e-6

    def body_ok(poses):
        if body is None:
            return True
        w = body(poses)
        return w is None or w <= allow

    def turn_ok(px, py, pyaw, ang):
        return not _sweep(px, py, pyaw, ang, discs) and body_ok(_sweep_poses(px, py, pyaw, ang))

    other = delta - math.copysign(2.0 * math.pi, delta)
    # (a) the other way round
    if turn_ok(x, y, yaw, other):
        return PivotPlan('other', [('spin', other)],
                         'planned %+.0f deg sweeps %s: turn the other way (%+.0f deg)'
                         % (math.degrees(delta), names, math.degrees(other)), hits)
    # (b) back up, then pivot
    d = backoff((x, y, yaw), backup_m) if backoff else backup_m
    if d >= 0.1 - 1e-9:
        bx, by = x - d * math.cos(yaw), y - d * math.sin(yaw)
        if _line_clear(x, y, yaw, -d, discs):
            for ang in (delta, other):
                if turn_ok(bx, by, yaw, ang):
                    return PivotPlan('backup', [('backup', d), ('spin', ang)],
                                     'planned %+.0f deg sweeps %s: back up %.2f m, then pivot '
                                     '%+.0f deg' % (math.degrees(delta), names, d,
                                                    math.degrees(ang)), hits)
    # (c) K-turn: half, back up, the rest
    if kturn:
        for ang in (delta, other):
            h1 = 0.5 * ang
            if not turn_ok(x, y, yaw, h1):
                continue
            y1 = yaw + h1
            d2 = backoff((x, y, y1), backup_m) if backoff else backup_m
            if d2 < 0.1 - 1e-9 or not _line_clear(x, y, y1, -d2, discs):
                continue
            bx, by = x - d2 * math.cos(y1), y - d2 * math.sin(y1)
            if turn_ok(bx, by, y1, ang - h1):
                return PivotPlan('kturn', [('spin', h1), ('backup', d2), ('spin', ang - h1)],
                                 'planned %+.0f deg sweeps %s: K-turn %+.0f deg, back %.2f m, '
                                 '%+.0f deg' % (math.degrees(delta), names, math.degrees(h1),
                                                d2, math.degrees(ang - h1)), hits)
    # (d) pivot earlier / later along the heading
    for s in shifts:
        if s < 0.0:
            cap = backoff((x, y, yaw), -s) if backoff else -s
            if cap < -s - 1e-9:
                continue
        px, py = x + s * math.cos(yaw), y + s * math.sin(yaw)
        if not _line_clear(x, y, yaw, s, discs) or not body_ok(_line_poses(x, y, yaw, s)):
            continue
        for ang in (delta, other):
            if turn_ok(px, py, yaw, ang):
                return PivotPlan('shift', [('drive' if s > 0 else 'backup', abs(s)),
                                           ('spin', ang)],
                                 'planned %+.0f deg sweeps %s: pivot %.2f m %s (%+.0f deg)'
                                 % (math.degrees(delta), names, abs(s),
                                    'later' if s > 0 else 'earlier', math.degrees(ang)), hits)
    return PivotPlan('blocked', [], 'planned %+.0f deg sweeps %s: no clear alternative'
                     % (math.degrees(delta), names), hits)


# ---------------------------------------------------------------------------
# edge-lap detour (B6: detour around, rejoin)
# ---------------------------------------------------------------------------

def edge_detour(poses, i0: int, discs: Sequence, inside: Callable, *, half_width: float = 0.235,
                clearance: float = 0.10, lookahead_m: float = 3.0, ramp_m: float = 0.8,
                max_offset_m: float = 1.2):
    """Bend the edge lap ``poses`` from local ``i0`` inward around the ``discs`` within
    ``lookahead_m``: each pose beside a disc moves toward the area interior until the body
    side clears the disc by ``clearance``, ramped in and out (cosine) over ``ramp_m``.
    ``inside(x, y) -> bool`` tells the interior side. Returns ``(detour, end)``: the bent
    poses from i0 to local ``end`` (where the lap is rejoined), or None when no disc is
    beside the stretch or the needed offset exceeds ``max_offset_m``."""
    if i0 >= len(poses) - 1 or not discs:
        return None
    cum = [0.0]
    for a, b in zip(poses[i0:-1], poses[i0 + 1:]):
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    n = 0
    while n < len(cum) and cum[n] <= lookahead_m + ramp_m + 1.0:
        n += 1
    seg = [tuple(q) for q in poses[i0:i0 + n]]
    if len(seg) < 3:
        return None
    need = [0.0] * len(seg)
    sides = []
    for k, q in enumerate(seg):
        a, b = seg[max(0, k - 1)], seg[min(len(seg) - 1, k + 1)]
        h = math.atan2(b[1] - a[1], b[0] - a[0])
        nx, ny = -math.sin(h), math.cos(h)          # left normal
        sgn = 1.0 if inside(q[0] + 0.3 * nx, q[1] + 0.3 * ny) else -1.0
        sides.append((nx * sgn, ny * sgn, h))
        if cum[k] > lookahead_m:
            continue
        for z in discs:
            dx, dy = z.x - q[0], z.y - q[1]
            along = dx * math.cos(h) + dy * math.sin(h)
            if along < -0.15 or along > 0.65:        # beside the body box (-0.10 .. 0.60)
                continue
            lat_in = dx * nx * sgn + dy * ny * sgn   # disc offset toward the interior
            reach = z.r + half_width + clearance
            if abs(lat_in) >= reach:
                continue
            need[k] = max(need[k], lat_in + reach)
    if max(need) <= 1e-3:
        return None
    if max(need) > max_offset_m:
        return None
    # ramp: offset(s) = max over k of need[k] * cosine window around cum[k]
    off = [0.0] * len(seg)
    for k in range(len(seg)):
        for j in range(len(seg)):
            if need[j] <= 0.0:
                continue
            ds = abs(cum[k] - cum[j])
            if ds <= ramp_m:
                off[k] = max(off[k], need[j] * 0.5 * (1.0 + math.cos(math.pi * ds / ramp_m)))
    last = max(k for k in range(len(seg)) if off[k] > 1e-3)
    end = min(len(seg) - 1, last + 1)
    out = []
    for k in range(end + 1):
        q = seg[k]
        nx, ny, _h = sides[k]
        out.append((q[0] + off[k] * nx, q[1] + off[k] * ny))
    res = []
    for k, q in enumerate(out):
        a, b = out[max(0, k - 1)], out[min(len(out) - 1, k + 1)]
        res.append((q[0], q[1], math.atan2(b[1] - a[1], b[0] - a[0])))
    res[-1] = (res[-1][0], res[-1][1], sides[end][2])     # rejoin on the lap's heading
    return res, i0 + end
