# SPDX-License-Identifier: GPL-3.0-or-later
"""Swept-footprint geometry for automatic obstacle no-go zones (ROS-free, numpy only).

Option A of docs/analysis/2026-10-23_obstacle_nogo_options.md (the 2026-10-22 trunk 0.36 m
inside the Area 7 line, beside the lap-0 corner). Used by

* ``obstacle_tracker_core`` - the B6 on-path rule for class-voted zones: a detected static
  object only becomes a zone when the robot's body would touch it on the planned path ahead,
  along a straight stretch (the footprint corridor) OR while pivoting at a corner of the
  path (the pivot SWEEP: the 0.60 m nose corners reach 0.645 m from the rear axle);
* the mission (option C, next): ``pivot_sweep_hits`` answers "does this pivot's sweep, in
  this direction, intersect a no-go zone or a static mark?" against the zone set the tracker
  publishes on ``/obstacle_tracker/nogo_zones`` (``ZoneSet.from_json``).

Frames: map metres; poses are (x, y, yaw) of base_link (the rear axle, the pivot centre).
The footprint is the nav2 / URDF box: x -0.10..0.60, y -0.235..0.235.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

# (x_min, x_max, y_min, y_max) around base_link, nav2_params.yaml footprint (2026-10-20)
FOOTPRINT_BOX = (-0.10, 0.60, -0.235, 0.235)


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------------------
# footprint vs discs
# ---------------------------------------------------------------------------

def poses_hit_disc(poses, cx: float, cy: float, r: float,
                   box: Sequence[float] = FOOTPRINT_BOX, pad: float = 0.0) -> np.ndarray:
    """Per pose (N, 3): does the footprint box (grown by ``pad``) touch the disc
    (``cx``, ``cy``, ``r``)? Exact rectangle-to-point distance in each pose's frame."""
    q = np.asarray(poses, dtype=float).reshape(-1, 3)
    if len(q) == 0:
        return np.zeros(0, dtype=bool)
    c, s = np.cos(q[:, 2]), np.sin(q[:, 2])
    dx, dy = cx - q[:, 0], cy - q[:, 1]
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    x0, x1, y0, y1 = box
    ex = np.maximum(np.maximum(x0 - pad - lx, lx - (x1 + pad)), 0.0)
    ey = np.maximum(np.maximum(y0 - pad - ly, ly - (y1 + pad)), 0.0)
    return ex * ex + ey * ey <= float(r) * float(r) + 1e-12


def densify(points, step: float = 0.1) -> np.ndarray:
    """Polyline (N, 2) -> poses (M, 3) every ``step`` m, yaw = the segment heading
    (straight-line driving; turns are handled as pivots, see :func:`pivot_vertices`)."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    out = []
    for a, b in zip(pts[:-1], pts[1:]):
        d = b - a
        L = float(np.hypot(*d))
        if L < 1e-6:
            continue
        yaw = math.atan2(d[1], d[0])
        n = max(1, int(math.ceil(L / step)))
        for k in range(n):
            p = a + d * (k / n)
            out.append((p[0], p[1], yaw))
    if len(pts) >= 2 and out:
        out.append((pts[-1][0], pts[-1][1], out[-1][2]))
    return np.array(out, dtype=float).reshape(-1, 3)


def pivot_vertices(points, min_turn_deg: float = 30.0,
                   min_seg_m: float = 0.02) -> List[Tuple[float, float, float, float]]:
    """Corners of a polyline where the heading changes by >= ``min_turn_deg``:
    ``[(x, y, yaw_in, yaw_out)]`` (the robot pivots there; segments shorter than
    ``min_seg_m`` are merged)."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    keep = [pts[0]] if len(pts) else []
    for p in pts[1:]:
        if np.hypot(*(p - keep[-1])) >= min_seg_m:
            keep.append(p)
    out = []
    lim = math.radians(min_turn_deg)
    for a, b, c in zip(keep[:-2], keep[1:-1], keep[2:]):
        y_in = math.atan2(b[1] - a[1], b[0] - a[0])
        y_out = math.atan2(c[1] - b[1], c[0] - b[0])
        if abs(wrap(y_out - y_in)) >= lim:
            out.append((float(b[0]), float(b[1]), y_in, y_out))
    return out


def sweep_yaws(yaw_from: float, yaw_to: float, direction: int = 0,
               step_deg: float = 5.0) -> np.ndarray:
    """Headings passed while pivoting from ``yaw_from`` to ``yaw_to``: ``direction`` +1 =
    counter-clockwise, -1 = clockwise, 0 = the shorter way (what Nav2 Spin does)."""
    d = wrap(yaw_to - yaw_from)
    if direction > 0 and d < 0:
        d += 2.0 * math.pi
    elif direction < 0 and d > 0:
        d -= 2.0 * math.pi
    n = max(1, int(math.ceil(abs(math.degrees(d)) / step_deg)))
    return yaw_from + d * np.arange(n + 1) / n


def sweep_poses(x: float, y: float, yaw_from: float, yaw_to: float, direction: int = 0,
                step_deg: float = 5.0) -> np.ndarray:
    yaws = sweep_yaws(yaw_from, yaw_to, direction, step_deg)
    return np.stack([np.full(len(yaws), x), np.full(len(yaws), y), yaws], axis=1)


# ---------------------------------------------------------------------------
# B6 on-path test (planned path ahead: corridor + pivot sweeps)
# ---------------------------------------------------------------------------

def path_hit(path_xy, cx: float, cy: float, r: float, *, box=FOOTPRINT_BOX,
             pad: float = 0.0, step: float = 0.1, min_turn_deg: float = 30.0,
             include_pivots: bool = True) -> str:
    """Would the body touch the disc driving ``path_xy`` (N, 2)? Returns ``'corridor'``
    (along a stretch), ``'pivot'`` (only while turning at a corner, the shorter way) or
    ``''``. Cheap reject first: nothing within reach of any vertex / segment."""
    pts = np.asarray(path_xy, dtype=float).reshape(-1, 2)
    if len(pts) == 0:
        return ''
    reach = math.hypot(max(abs(box[0]), abs(box[1])), max(abs(box[2]), abs(box[3])))
    reach += float(r) + float(pad) + step
    if len(pts) == 1:
        near = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) <= reach
    else:
        near = _seg_dist(pts, cx, cy) <= reach
    if not np.any(near):
        return ''
    # only the segments near the disc (and their neighbours, for the pivots)
    idx = np.flatnonzero(near)
    lo, hi = max(0, idx.min() - 1), min(len(pts), idx.max() + 3)
    sub = pts[lo:hi]
    if np.any(poses_hit_disc(densify(sub, step), cx, cy, r, box, pad)):
        return 'corridor'
    if include_pivots:
        for (x, y, y_in, y_out) in pivot_vertices(sub, min_turn_deg):
            if np.any(poses_hit_disc(sweep_poses(x, y, y_in, y_out), cx, cy, r, box, pad)):
                return 'pivot'
    return ''


def _seg_dist(pts: np.ndarray, cx: float, cy: float) -> np.ndarray:
    """Distance from (cx, cy) to each segment pts[i]..pts[i+1] (N-1,), padded to N."""
    a, b = pts[:-1], pts[1:]
    d = b - a
    L2 = np.maximum((d * d).sum(axis=1), 1e-12)
    t = np.clip(((cx - a[:, 0]) * d[:, 0] + (cy - a[:, 1]) * d[:, 1]) / L2, 0.0, 1.0)
    px, py = a[:, 0] + t * d[:, 0], a[:, 1] + t * d[:, 1]
    out = np.hypot(px - cx, py - cy)
    return np.append(out, out[-1])


class PathCursor:
    """Where the robot is along a long plan (the latched coverage plan): the part from
    ``back_m`` behind the robot's projection onwards is "ahead". The cursor only moves
    forward while the robot stays near the plan, so a neighbouring row 0.2 m away does not
    pull it back."""

    def __init__(self, back_m: float = 2.0, near_m: float = 0.6):
        self.back_m, self.near_m = float(back_m), float(near_m)
        self.pts = np.empty((0, 2))
        self.cum = np.zeros(0)
        self.idx = 0

    def set_path(self, pts) -> None:
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        if len(pts) == len(self.pts) and np.array_equal(pts, self.pts):
            return
        self.pts = pts
        seg = np.hypot(*np.diff(pts, axis=0).T) if len(pts) > 1 else np.zeros(0)
        self.cum = np.concatenate([[0.0], np.cumsum(seg)])
        self.idx = 0

    def update(self, x: float, y: float) -> None:
        if len(self.pts) == 0:
            return
        d = np.hypot(self.pts[:, 0] - x, self.pts[:, 1] - y)
        ahead = np.flatnonzero((d <= self.near_m) & (np.arange(len(d)) >= self.idx))
        if len(ahead):
            self.idx = int(ahead[0])
        elif d.min() > self.near_m:
            pass   # off the plan (transit, recovery): keep the cursor
        else:
            self.idx = int(np.argmin(d))

    def ahead(self) -> np.ndarray:
        if len(self.pts) == 0:
            return self.pts
        s0 = self.cum[self.idx] - self.back_m
        i0 = int(np.searchsorted(self.cum, s0, side='left'))
        return self.pts[max(0, min(i0, self.idx)):]


# ---------------------------------------------------------------------------
# zone set (published by obstacle_tracker on /obstacle_tracker/nogo_zones, JSON)
# ---------------------------------------------------------------------------

@dataclass
class Zone:
    x: float
    y: float
    r: float
    id: int = 0
    label: str = ''
    state: str = 'pending'     # pending | accepted | candidate | mark
    score: float = 0.0


@dataclass
class ZoneSet:
    stamp: float = 0.0
    zones: List[Zone] = field(default_factory=list)
    marks: List[Zone] = field(default_factory=list)   # recent static-class det discs

    def to_json(self) -> str:
        def z(o):
            return {'id': o.id, 'x': round(o.x, 3), 'y': round(o.y, 3), 'r': round(o.r, 3),
                    'label': o.label, 'state': o.state, 'score': round(o.score, 2)}
        return json.dumps({'stamp': round(self.stamp, 3), 'frame': 'map',
                           'zones': [z(o) for o in self.zones],
                           'marks': [[round(o.x, 2), round(o.y, 2), round(o.r, 2)]
                                     for o in self.marks]})

    @classmethod
    def from_json(cls, text: str) -> 'ZoneSet':
        d = json.loads(text or '{}')
        zones = [Zone(float(o['x']), float(o['y']), float(o['r']), int(o.get('id', 0)),
                      str(o.get('label', '')), str(o.get('state', 'pending')),
                      float(o.get('score', 0.0))) for o in d.get('zones', [])]
        marks = [Zone(float(m[0]), float(m[1]), float(m[2]), state='mark')
                 for m in d.get('marks', [])]
        return cls(float(d.get('stamp', 0.0)), zones, marks)

    def discs(self, include_candidates: bool = True, include_marks: bool = True
              ) -> List[Zone]:
        out = [z for z in self.zones if include_candidates or z.state != 'candidate']
        if include_marks:
            out.extend(self.marks)
        return out


@dataclass
class SweepHit:
    zone: Zone
    yaw: float          # first heading of the sweep at which the body touches it


def pivot_sweep_hits(x: float, y: float, yaw_from: float, yaw_to: float, direction: int,
                     zones: Iterable[Zone], *, box=FOOTPRINT_BOX, pad: float = 0.0,
                     step_deg: float = 5.0) -> List[SweepHit]:
    """The mission's pre-pivot query: which zones / marks does the body sweep turning in
    place at (``x``, ``y``) from ``yaw_from`` to ``yaw_to`` in ``direction`` (+1 CCW,
    -1 CW, 0 the shorter way)? Empty list = the pivot is clear. ``pad`` grows the
    footprint (e.g. the 0.02 m costmap padding)."""
    poses = sweep_poses(x, y, yaw_from, yaw_to, direction, step_deg)
    hits = []
    reach = math.hypot(max(abs(box[0]), abs(box[1])), max(abs(box[2]), abs(box[3]))) + pad
    for z in zones:
        if math.hypot(z.x - x, z.y - y) > reach + z.r:
            continue
        m = poses_hit_disc(poses, z.x, z.y, z.r, box, pad)
        if np.any(m):
            hits.append(SweepHit(z, float(poses[int(np.argmax(m)), 2])))
    return hits


def pivot_clear_direction(x: float, y: float, yaw_from: float, yaw_to: float,
                          zones: Sequence[Zone], **kw) -> Optional[int]:
    """+1 / -1: a pivot direction whose sweep is clear (the shorter one first), 0 when
    the turn is (numerically) none, ``None`` when both directions hit something."""
    d = wrap(yaw_to - yaw_from)
    if abs(d) < 1e-6:
        return 0
    first = 1 if d > 0 else -1
    for direction in (first, -first):
        if not pivot_sweep_hits(x, y, yaw_from, yaw_to, direction, zones, **kw):
            return direction
    return None
