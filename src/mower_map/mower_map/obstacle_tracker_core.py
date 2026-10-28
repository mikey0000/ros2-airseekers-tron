# SPDX-License-Identifier: GPL-3.0-or-later
"""ROS-free core of the obstacle auto-marker (mower_map.obstacle_tracker_node).

Clusters obstacle points (the ground-removed stereo depth cloud +
YOLO-ranged detections, both already in the map frame), tracks the clusters
across frames and decides:

* **promote** - a cluster is a reliable, static obstacle (observed over
  ``persistence_age_s`` - minutes - and still in view, often enough, sane size,
  inside a mowing area, never near a ranged person / animal detection) -> the
  node calls ``/map_server_node/promote_obstacle``, which stamps a PENDING
  keepout (live in the mask for this session, not written to ``areas.dat``)
  for the operator to accept or discard. Off by default (``auto_promote``).

* **class-voted zones** (option A, 2026-10-23 trunk incident,
  docs/analysis/2026-10-23_obstacle_nogo_options.md) - ranged YOLO detections of a STATIC
  class (trunk, stone, wood, ...; never a person or an animal) skip the minutes-long
  persistence gate: ``det_min_frames`` frames of score >= ``det_min_score`` within
  ``det_window_s`` landing within ``det_assoc_m`` of each other, the voted class on the
  whitelist, the object ON the planned path ahead (footprint corridor or the sweep of a
  pivot at a corner, :mod:`mower_map.nogo_zones`; a bumper hit at the spot counts too) ->
  promoted as a PENDING keepout sized from the class-voted box width (median of the
  ranged widths, at least the class prior) + ``size_margin_m``. It is live for the rest of
  the mow (keepout mask: Nav2 routes round it), never written to ``areas.dat`` until the
  operator accepts it. A pending zone is re-sized as observations accumulate, merged with a
  duplicate, dropped when verified gone or not re-seen for ``pending_expire_s`` of mowing in
  its area, and Ignore suppresses its spot for the session.

* **remove** - a *promoted* obstacle that the sensors no longer find: either a
  ground ("clear") point appears in its footprint (the depth camera positively
  saw bare ground there), or the robot looked at the footprint from close
  range / ahead and found no obstacle point. After ``gone_confirmations``
  such frames the node calls ``/map_server_node/remove_obstacle``.

Only numpy is required, so ``python3 -m pytest src/mower_map/test`` runs on a
host without ROS. Coordinates are map-frame metres.

The clustering deliberately runs on the raw sensor clouds, NOT the Nav2 global
costmap, because a promoted keepout is rendered lethal in that costmap by the
keepout filter - the costmap can no longer tell "object still there" from "we
painted a keepout here", which would make removal impossible and re-detection
constant.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from mower_map.areas import point_in_polygon
from mower_map.nogo_zones import path_hit

Point = Tuple[float, float]
Polygon = List[Point]


# ---------------------------------------------------------------------------
# parameters
# ---------------------------------------------------------------------------

@dataclass
class TrackerParams:
    resolution: float = 0.1              # clustering grid (m)
    merge_cells: int = 2                 # cells joined across this Chebyshev gap
    association_dist_m: float = 0.5      # cluster <-> track association radius
    # B2 (review 2026-10-13): seen over this span (first..last sighting, still in view)
    # before promotion. Minutes, so a person / pet standing still is never marked.
    persistence_age_s: float = 120.0
    min_observations: int = 10           # ... and be seen this many updates
    min_obstacle_radius_m: float = 0.05
    max_obstacle_radius_m: float = 1.0   # bigger clusters (hedges, people groups) never promote
    det_radius_m: float = 0.15           # footprint assumed for a single-point detection
    polygon_inflation_m: float = 0.05    # safety growth of the promoted polygon
    polygon_steps: int = 12              # vertices of the promoted footprint ring
    require_inside_mowing_area: bool = True

    auto_promote: bool = False           # owner 2026-10-13: not ready, off by default
    # a track that ever had a ranged dynamic-class detection (person, dog, cat, ...) within
    # this distance of its footprint is never promoted
    dynamic_veto_radius_m: float = 1.0
    auto_remove: bool = True
    min_lifetime_s: float = 5.0          # never remove sooner than this after promoting
    gone_confirmations: int = 4          # consecutive "gone" frames before removal
    verify_distance_m: float = 3.0       # robot must be this close to "look"
    verify_fov_deg: float = 60.0         # ... and the footprint within this half-cone
    present_radius_m: float = 0.4        # an obstacle point this close = still there
    clear_radius_m: float = 0.4          # a ground point this close = positively gone

    transient_timeout_s: float = 30.0    # drop never-promoted tracks unseen this long
    # a candidate voted a living class (a static-class misdetection of a dog started it) is
    # dropped sooner: it only feeds the mission's pivot sweep check, which waits for it
    # (field 2026-10-28: a dog candidate blocked a route corner 25 s after it was last seen)
    living_timeout_s: float = 10.0
    max_tracks: int = 200
    max_track_drift_m: float = 0.4       # reject a track whose centroid wandered this far

    # --- class-voted zones (option A, 2026-10-23; owner defaults to review) ---
    class_zones: bool = True             # independent of auto_promote (cluster promotion)
    det_min_score: float = 0.6
    det_min_frames: int = 5
    det_window_s: float = 10.0
    det_assoc_m: float = 0.3             # detections within this of the zone centre
    det_vote_min_score: float = 0.3      # a lower score still votes for the class
    static_classes: Tuple[str, ...] = ('trunk', 'stone', 'wood', 'brick', 'trashbin',
                                       'potted plant', 'chair')
    never_classes: Tuple[str, ...] = ('person', 'dog', 'cat', 'hedgehog', 'rabbit')
    # class prior diameters (m): a zone is never smaller than its class
    class_prior_m: Tuple[Tuple[str, float], ...] = (
        ('trunk', 0.25), ('stone', 0.15), ('wood', 0.10), ('brick', 0.20),
        ('trashbin', 0.45), ('potted plant', 0.30), ('chair', 0.50))
    default_prior_m: float = 0.20
    size_margin_m: float = 0.10          # on the measured half width (planner adds 0.30)
    max_zone_radius_m: float = 1.0
    resize_step_m: float = 0.05          # re-send a pending zone whose radius moved this much
    require_on_path: bool = True         # B6: only objects the body would touch
    onpath_pivot_min_turn_deg: float = 30.0
    bumper_evidence_m: float = 0.5       # a bumper hit this close = on path + a vote
    bumper_zone_hits: int = 0            # >0: this many hits within bumper_evidence_m make a
                                         # bumper-only zone (0.25 m disc); 0 = off
    bumper_zone_radius_m: float = 0.25
    pending_expire_s: float = 3600.0     # mowing time in its area without a re-sighting
    det_gone_confirmations: int = 8      # det zones: "looked and absent" frames to remove
    det_verify_min_m: float = 0.8        # det zones are verified only from this far ...
    det_verify_fov_deg: float = 30.0     # ... and inside the camera's half field of view
    camera_x_m: float = 0.466            # front camera ahead of base_link (det zone checks)
    merge_dist_m: float = 0.3            # two det zones closer than this are one object
    ignore_radius_m: float = 0.5         # Ignore suppresses new zones this close


# ---------------------------------------------------------------------------
# geometry helpers
# ---------------------------------------------------------------------------

def cell_clusters(cells: Sequence[Tuple[int, int]], merge_cells: int) -> List[List[Tuple[int, int]]]:
    """Union-find over integer grid cells joined when their Chebyshev distance
    is <= ``merge_cells`` (so 1-cell gaps still group)."""
    if not cells:
        return []
    parent = {c: c for c in cells}

    def find(a):
        root = a
        while parent[root] != root:
            root = parent[root]
        while parent[a] != root:
            parent[a], a = root, parent[a]
        return root

    for (cx, cy) in cells:
        for dx in range(-merge_cells, merge_cells + 1):
            for dy in range(-merge_cells, merge_cells + 1):
                n = (cx + dx, cy + dy)
                if n in cells:
                    ra, rb = find((cx, cy)), find(n)
                    if ra != rb:
                        parent[rb] = ra
    groups: Dict[Tuple[int, int], List[Tuple[int, int]]] = {}
    for c in cells:
        groups.setdefault(find(c), []).append(c)
    return list(groups.values())


def cluster_points(points: np.ndarray, resolution: float,
                   merge_cells: int) -> List[np.ndarray]:
    """Group an (N, 2) map-frame point array into clusters of points."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) == 0:
        return []
    inv = 1.0 / float(resolution)
    cell_pts: Dict[Tuple[int, int], List[int]] = {}
    for i in range(len(pts)):
        key = (int(round(pts[i, 0] * inv)), int(round(pts[i, 1] * inv)))
        cell_pts.setdefault(key, []).append(i)
    out = []
    for group in cell_clusters(list(cell_pts), int(merge_cells)):
        idx: List[int] = []
        for c in group:
            idx.extend(cell_pts[c])
        out.append(pts[idx])
    return out


def ring_polygon(cx: float, cy: float, radius: float, steps: int) -> Polygon:
    steps = max(3, int(steps))
    return [(cx + radius * math.cos(2.0 * math.pi * i / steps),
             cy + radius * math.sin(2.0 * math.pi * i / steps)) for i in range(steps)]


def _cluster_centroid_radius(cluster: np.ndarray) -> Tuple[float, float, float]:
    cx = float(cluster[:, 0].mean())
    cy = float(cluster[:, 1].mean())
    if len(cluster) > 1:
        d = np.hypot(cluster[:, 0] - cx, cluster[:, 1] - cy)
        return cx, cy, float(d.max())
    return cx, cy, 0.0


def keepout_id(name) -> int:
    """Tracker id recorded in a promoted keepout's name ('auto: obstacle #17' -> 17,
    0 when there is none, e.g. the operator renamed it on accept)."""
    m = re.search(r'#(\d+)\s*$', str(name or ''))
    return int(m.group(1)) if m else 0


def keepout_entry(area_index, polygon, name=''):
    """A stored SOURCE_TRACKER keepout -> ``(area_index, cx, cy, radius, wanted_id)``
    for :meth:`TrackerCore.adopt_keepouts` (vertex centroid, farthest vertex), or
    ``None`` for a degenerate polygon."""
    pts = [(float(x), float(y)) for x, y in polygon]
    if len(pts) < 3:
        return None
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    r = max(math.hypot(p[0] - cx, p[1] - cy) for p in pts)
    return (int(area_index), cx, cy, r, keepout_id(name))


# ---------------------------------------------------------------------------
# track
# ---------------------------------------------------------------------------

@dataclass
class Track:
    id: int
    cx: float
    cy: float
    radius: float
    first_seen: float
    last_seen: float
    observation_count: int = 1
    origin_cx: float = 0.0           # first observed centroid, for the drift gate
    origin_cy: float = 0.0
    promoted: bool = False
    promoting: bool = False          # promotion request in flight
    removing: bool = False           # removal request in flight
    promoted_at: float = 0.0
    gone_votes: int = 0
    area_index: int = -1
    name: str = ''
    dynamic: bool = False            # a person / animal was detected here: never promote
    adopted: bool = False            # re-adopted from a stored SOURCE_TRACKER keepout
    # --- class-voted zones ---
    kind: str = 'cluster'            # cluster | det | bumper
    votes: Dict[str, float] = field(default_factory=dict)   # label -> summed score
    frames: List[Tuple[float, float, float]] = field(default_factory=list)  # (t, x, y) >= min
    widths: List[float] = field(default_factory=list)       # ranged widths (m), recent
    det_pts: List[Tuple[float, float]] = field(default_factory=list)  # recent det positions
    score_max: float = 0.0
    on_path: str = ''                # '' | corridor | pivot | bumper
    pending: bool = False            # promoted and not (yet) accepted by the operator
    promoted_radius: float = 0.0     # footprint radius the keepout was sent with
    unseen_s: float = 0.0            # mowing time in its area without a sighting
    bumper_hits: int = 0
    seen_det: bool = False           # a detection matched it this step
    resizing: bool = False

    def label(self) -> str:
        return max(self.votes, key=self.votes.get) if self.votes else ''

    def sized_radius(self, p: TrackerParams) -> float:
        """Class-voted zone radius: max(median ranged width, class prior) / 2 + margin; a
        promoted zone also covers the spread of the detections it absorbed (p90 distance
        from its fixed centre + margin: a long object seen in parts, field 2026-10-28
        ladder), capped at max_zone_radius_m."""
        prior = dict(p.class_prior_m).get(self.label(), p.default_prior_m)
        w = float(np.median(self.widths)) if self.widths else 0.0
        r = max(w, prior) / 2.0 + p.size_margin_m
        if self.promoted and len(self.det_pts) >= 3:
            arr = np.asarray(self.det_pts[-50:])
            d = np.hypot(arr[:, 0] - self.cx, arr[:, 1] - self.cy)
            r = max(r, float(np.percentile(d, 90)) + p.size_margin_m)
        return min(r, p.max_zone_radius_m)

    def footprint_radius(self, p: TrackerParams) -> float:
        if self.kind == 'det':
            return self.sized_radius(p)
        if self.kind == 'bumper':
            return p.bumper_zone_radius_m
        return max(self.radius, p.det_radius_m) + p.polygon_inflation_m

    def footprint(self, p: TrackerParams) -> Polygon:
        return ring_polygon(self.cx, self.cy, self.footprint_radius(p), p.polygon_steps)


# ---------------------------------------------------------------------------
# tracker
# ---------------------------------------------------------------------------

class TrackerCore:
    """Pure state machine: feed it sensor points, get promote/remove decisions."""

    def __init__(self, params: Optional[TrackerParams] = None):
        self.params = params or TrackerParams()
        self.tracks: List[Track] = []
        self._next_id = 1
        self.ignored: List[Tuple[float, float, float]] = []   # (x, y, r) Ignored this session
        self._last_step: Optional[float] = None

    # -- internal helpers ---------------------------------------------------
    def find_track(self, track_id: int) -> Optional[Track]:
        for t in self.tracks:
            if t.id == track_id:
                return t
        return None

    def _matching_track(self, cx: float, cy: float, used: set) -> Optional[Track]:
        best, best_d = None, float(self.params.association_dist_m)
        for t in self.tracks:
            if t.id in used:
                continue
            d = math.hypot(t.cx - cx, t.cy - cy)
            if d <= best_d:
                best, best_d = t, d
        return best

    def _area_index_for(self, cx: float, cy: float, areas) -> int:
        for (ai, _name, poly) in areas:
            if point_in_polygon(cx, cy, poly):
                return int(ai)
        return -1

    @staticmethod
    def _points_within(pts: np.ndarray, cx: float, cy: float, radius: float) -> bool:
        if pts is None or len(pts) == 0:
            return False
        dx = pts[:, 0] - cx
        dy = pts[:, 1] - cy
        return bool(np.any(dx * dx + dy * dy <= radius * radius))

    # -- class-voted zones (option A) ---------------------------------------
    def _is_ignored(self, cx: float, cy: float) -> bool:
        return any(math.hypot(cx - x, cy - y) <= r for (x, y, r) in self.ignored)

    def add_detections(self, dets, now: float) -> None:
        """Ranged YOLO detections in the map frame: ``[(x, y, label, score, width_m)]``
        (``width_m`` 0 = unknown). A static-class / any named detection votes for the
        class of the zone it lands in (within ``det_assoc_m``); a new one starts a ``det``
        zone. Living classes are left to the dynamic veto (they never start a zone)."""
        p = self.params
        static = {c.lower() for c in p.static_classes}
        never = {c.lower() for c in p.never_classes}
        for (x, y, label, score, width) in dets:
            label = str(label).strip().lower()
            score = float(score)
            if not label or score < p.det_vote_min_score:
                continue
            # a person / animal beside a zone is not part of it (only within det_assoc_m)
            t = self._det_track(x, y, wide=label not in never)
            if t is None:
                if label in never or label not in static or len(self.tracks) >= p.max_tracks:
                    continue
                t = Track(id=self._next_id, cx=float(x), cy=float(y), radius=0.0,
                          first_seen=now, last_seen=now, observation_count=0,
                          origin_cx=float(x), origin_cy=float(y), kind='det')
                self._next_id += 1
                self.tracks.append(t)
            elif t.kind != 'det':
                if label in never:
                    t.dynamic = True
                    continue
                if label not in static:
                    continue
                # a stereo cluster / adopted keepout at this spot is this object
                t.kind = 'det'
                if not t.promoted:
                    t.cx, t.cy = float(x), float(y)
            t.votes[label] = t.votes.get(label, 0.0) + score
            t.score_max = max(t.score_max, score)
            t.last_seen = now
            t.seen_det = True
            t.observation_count += 1
            t.det_pts.append((float(x), float(y)))
            del t.det_pts[:-100]
            if label in never:
                t.dynamic = True
            if score >= p.det_min_score and label in static:
                t.frames.append((now, float(x), float(y)))
                if width and width > 0.0:
                    t.widths.append(float(width))
                    del t.widths[:-50]
            if not t.promoted:
                # the zone centre is the mean of the recent detections
                arr = np.asarray(t.det_pts[-50:])
                t.cx, t.cy = float(arr[:, 0].mean()), float(arr[:, 1].mean())

    def _det_track(self, x: float, y: float, wide: bool = True) -> Optional[Track]:
        p = self.params
        best, best_d = None, float('inf')
        for t in self.tracks:
            # a promoted zone absorbs detections over its whole footprint (+ 0.10 m): a 1.5 m
            # ladder seen in parts stacked 8+ r 0.85 zones (field 2026-10-28) while the zone
            # only associated within det_assoc_m of its centre; it grows instead (resize)
            lim = max(p.det_assoc_m, t.footprint_radius(p) + 0.10) if t.promoted and wide \
                else p.det_assoc_m
            d = math.hypot(t.cx - x, t.cy - y)
            if d <= lim and d < best_d:
                best, best_d = t, d
        return best

    def add_bumper_hits(self, hits, now: float, areas=()) -> None:
        """Bumper contact positions (map frame). A hit within ``bumper_evidence_m`` of a
        det zone marks it on path (the body touched it) and adds a frame of evidence;
        with ``bumper_zone_hits`` > 0 repeated hits at one spot make a bumper-only zone."""
        p = self.params
        for (x, y) in hits:
            near = [t for t in self.tracks
                    if math.hypot(t.cx - x, t.cy - y) <= p.bumper_evidence_m
                    + (t.footprint_radius(p) if t.kind == 'det' else 0.0)]
            if near:
                for t in near:
                    t.bumper_hits += 1
                    if t.kind == 'det':
                        t.on_path = t.on_path or 'bumper'
                        t.frames.append((now, t.cx, t.cy))
                    if t.kind == 'bumper':
                        t.last_seen = now
                continue
            if p.bumper_zone_hits <= 0 or len(self.tracks) >= p.max_tracks:
                continue
            area_index = self._area_index_for(x, y, areas) if areas else -1
            t = Track(id=self._next_id, cx=float(x), cy=float(y), radius=0.0,
                      first_seen=now, last_seen=now, observation_count=1,
                      origin_cx=float(x), origin_cy=float(y), area_index=area_index,
                      kind='bumper', bumper_hits=1, on_path='bumper')
            self._next_id += 1
            self.tracks.append(t)

    def _qualifying_frames(self, t: Track, now: float) -> int:
        p = self.params
        t.frames = [f for f in t.frames if now - f[0] <= p.det_window_s]
        return sum(1 for (_, x, y) in t.frames
                   if math.hypot(x - t.cx, y - t.cy) <= p.det_assoc_m)

    def _merge_det_tracks(self) -> None:
        p = self.params
        dets = [t for t in self.tracks if t.kind == 'det']
        for i, a in enumerate(dets):
            for b in dets[i + 1:]:
                if a not in self.tracks or b not in self.tracks:
                    continue
                d = math.hypot(a.cx - b.cx, a.cy - b.cy)
                if a.promoted and b.promoted:
                    continue   # both are keepouts already: leave them to removal / dedup
                # a candidate overlapping a promoted zone is more of that object (field
                # 2026-10-28: the parts of a 1.5 m ladder promoted as stacked zones): the
                # promoted zone absorbs it and grows (resize) instead of a second zone
                never = {c.lower() for c in p.never_classes}
                one_promoted = ((a.promoted and not (b.promoted or b.promoting)
                                 and b.label() not in never) or
                                (b.promoted and not (a.promoted or a.promoting)
                                 and a.label() not in never))
                if d > p.merge_dist_m and not (
                        one_promoted and d <= a.footprint_radius(p) + b.footprint_radius(p)):
                    continue
                keep, drop = (b, a) if (b.promoted or b.promoting) or (
                    not (a.promoted or a.promoting) and b.first_seen < a.first_seen) else (a, b)
                if drop.promoting:
                    continue
                for k, v in drop.votes.items():
                    keep.votes[k] = keep.votes.get(k, 0.0) + v
                keep.frames.extend(drop.frames)
                keep.widths = (keep.widths + drop.widths)[-50:]
                keep.det_pts = (keep.det_pts + drop.det_pts)[-100:]
                keep.score_max = max(keep.score_max, drop.score_max)
                keep.on_path = keep.on_path or drop.on_path
                keep.bumper_hits += drop.bumper_hits
                keep.dynamic = keep.dynamic or drop.dynamic
                keep.first_seen = min(keep.first_seen, drop.first_seen)
                keep.last_seen = max(keep.last_seen, drop.last_seen)
                keep.observation_count += drop.observation_count
                keep.seen_det = keep.seen_det or drop.seen_det
                self.tracks.remove(drop)

    def _class_zone_ready(self, t: Track, now: float, allow_promote: bool,
                          area_blocked) -> bool:
        p = self.params
        if not (p.class_zones and allow_promote) or t.promoting or t.dynamic:
            return False
        if t.area_index < 0 or t.area_index in area_blocked:
            return False
        if self._is_ignored(t.cx, t.cy):
            return False
        if p.require_on_path and not t.on_path:
            return False
        if t.kind == 'bumper':
            return p.bumper_zone_hits > 0 and t.bumper_hits >= p.bumper_zone_hits
        label = t.label()
        if (label not in {c.lower() for c in p.static_classes}
                or label in {c.lower() for c in p.never_classes}):
            return False
        return self._qualifying_frames(t, now) >= p.det_min_frames

    def ignore(self, track_id: int) -> Optional[Track]:
        """Operator Ignore: suppress new zones at this spot for the session and forget the
        track. Returns the track (the caller removes its pending keepout) or None."""
        t = self.find_track(track_id)
        if t is None:
            return None
        self.ignored.append((t.cx, t.cy, max(self.params.ignore_radius_m,
                                             t.footprint_radius(self.params))))
        self.tracks.remove(t)
        return t

    def drop_pending(self) -> List[Track]:
        """End of a mowing session (option C, 2026-10-23): forget every PENDING zone (promoted
        or being promoted, not accepted by the operator). Pending zones are hard obstacles for
        the mow that found them only; no suppression, a next mow may propose them again.
        Returns the dropped tracks (the caller removes their pending keepouts)."""
        out = [t for t in self.tracks if t.pending and (t.promoted or t.promoting)]
        for t in out:
            self.tracks.remove(t)
        return out

    def _verify_cone(self, t: Track, robot_pose) -> bool:
        """The robot is looking at the spot (for a det zone: inside the camera's view)."""
        p = self.params
        if robot_pose is None:
            return False
        det = t.kind == 'det'
        ox, oy = robot_pose[0], robot_pose[1]
        if det:   # the camera looks from 0.47 m ahead of base_link
            ox += p.camera_x_m * math.cos(robot_pose[2])
            oy += p.camera_x_m * math.sin(robot_pose[2])
        dx, dy = t.cx - ox, t.cy - oy
        dist = math.hypot(dx, dy)
        if dist > p.verify_distance_m or (det and dist < p.det_verify_min_m):
            return False
        diff = abs(math.atan2(math.sin(math.atan2(dy, dx) - robot_pose[2]),
                              math.cos(math.atan2(dy, dx) - robot_pose[2])))
        return diff <= math.radians(p.det_verify_fov_deg if det else p.verify_fov_deg)

    # -- the main step ------------------------------------------------------
    def step(self, obstacle_points: np.ndarray, now: float,
             robot_pose: Optional[Tuple[float, float, float]] = None,
             clear_points: Optional[np.ndarray] = None,
             areas: Sequence[Tuple[int, str, Polygon]] = (),
             obstacle_fresh: bool = True, clear_fresh: bool = True,
             allow_promote: bool = True,
             dynamic_points: Optional[np.ndarray] = None,
             plans: Sequence[np.ndarray] = (),
             robot_area: int = -1,
             area_blocked: Sequence[int] = ()) -> Dict[str, list]:
        """Advance one tracker update.

        ``obstacle_points`` (N,2) map frame stereo + detection obstacle points.
        ``robot_pose`` (x, y, yaw) map frame, for the drive-by removal check.
        ``clear_points`` (M,2) map frame ground points from the clearing cloud.
        ``areas`` list of ``(area_index, name, polygon)`` mowing areas.
        ``obstacle_fresh`` / ``clear_fresh`` say whether those clouds are
        current; removal votes are only cast from fresh data, so a dead/stale
        sensor can never be read as "the object is gone". ``allow_promote``
        (False when the mower is not mowing, if that gate is enabled) blocks
        promotion without affecting tracking or removal. ``dynamic_points`` (K,2)
        map-frame positions of recent ranged dynamic-class detections (person,
        dog, cat, ...): every unpromoted track within ``dynamic_veto_radius_m``
        of one is flagged and never promoted.

        Class-voted zones (option A): ``plans`` are the planned paths ahead (map (N, 2)
        arrays: the coverage plan from the robot on, the current transit route) for the B6
        on-path test; ``robot_area`` the mowing area the robot is in (pending-zone expiry
        clock); ``area_blocked`` the area indices whose ``obstacle_detection`` is ``none``
        (no zone is made there).

        Returns ``{'promote': [track_id, ...], 'remove': [track_id, ...], 'resize':
        [track_id, ...]}``; the caller is responsible for the (async) service calls and
        must call :meth:`confirm_promoted` / :meth:`confirm_removed` accordingly. A
        ``resize`` is a pending zone whose keepout must be replaced by its new size.
        """
        p = self.params
        obstacle_points = np.asarray(obstacle_points, dtype=float).reshape(-1, 2) \
            if obstacle_points is not None else np.empty((0, 2))
        clear_points = np.asarray(clear_points, dtype=float).reshape(-1, 2) \
            if clear_points is not None else np.empty((0, 2))
        dynamic_points = np.asarray(dynamic_points, dtype=float).reshape(-1, 2) \
            if dynamic_points is not None else np.empty((0, 2))

        to_promote: List[int] = []
        to_remove: List[int] = []
        to_resize: List[int] = []
        dt = 0.0 if self._last_step is None else max(0.0, now - self._last_step)
        self._last_step = now
        area_blocked = set(int(a) for a in area_blocked)

        # 1. absorb clusters into tracks ------------------------------------
        # Tracking runs even when auto_promote is off, so the GUI / a bench
        # session can watch the tracker without writing anything to the map.
        matched: set = set()
        for cluster in cluster_points(obstacle_points, p.resolution, p.merge_cells):
            cx, cy, spread = _cluster_centroid_radius(cluster)
            radius = max(spread, p.det_radius_m)
            if radius < p.min_obstacle_radius_m or radius > p.max_obstacle_radius_m:
                continue
            area_index = self._area_index_for(cx, cy, areas) if areas else -1
            if p.require_inside_mowing_area and area_index < 0:
                continue
            t = self._matching_track(cx, cy, matched)
            if t is not None and t.kind != 'cluster':
                # a stereo blob at a class-voted zone: the same object, keep the det centre
                t.last_seen = now
                if area_index >= 0:
                    t.area_index = area_index
            elif t is not None:
                n = t.observation_count
                t.cx = (t.cx * n + cx) / (n + 1)
                t.cy = (t.cy * n + cy) / (n + 1)
                t.radius = max(t.radius, radius)
                t.observation_count += 1
                t.last_seen = now
                if area_index >= 0:
                    t.area_index = area_index
            else:
                if len(self.tracks) >= p.max_tracks:
                    continue
                t = Track(id=self._next_id, cx=cx, cy=cy, radius=radius,
                          first_seen=now, last_seen=now, observation_count=1,
                          origin_cx=cx, origin_cy=cy, area_index=area_index)
                self._next_id += 1
                self.tracks.append(t)
            matched.add(t.id)

        # 2. maintenance / promotion / removal ------------------------------
        self._merge_det_tracks()
        for t in list(self.tracks):
            if t.kind != 'cluster':
                if t.area_index < 0 and areas:
                    t.area_index = self._area_index_for(t.cx, t.cy, areas)
                if p.require_inside_mowing_area and areas and t.area_index < 0 \
                        and not t.promoted:
                    self.tracks.remove(t)
                    continue
                if not t.on_path and plans and not t.promoted:
                    r = t.footprint_radius(p)
                    for path in plans:
                        why = path_hit(path, t.cx, t.cy, r,
                                       min_turn_deg=p.onpath_pivot_min_turn_deg)
                        if why:
                            t.on_path = why
                            break
            if not t.promoted:
                unseen = now - t.last_seen
                if unseen > p.transient_timeout_s or (
                        t.kind == 'det' and unseen > p.living_timeout_s
                        and t.label() in {c.lower() for c in p.never_classes}):
                    self.tracks.remove(t)
                    continue
                if not t.dynamic and len(dynamic_points) and self._points_within(
                        dynamic_points, t.cx, t.cy,
                        t.footprint_radius(p) + p.dynamic_veto_radius_m):
                    t.dynamic = True
                if t.kind != 'cluster':
                    # option A: class-voted / bumper zone, no minutes-long gate
                    if self._class_zone_ready(t, now, allow_promote, area_blocked):
                        t.promoting = True
                        to_promote.append(t.id)
                    t.seen_det = False
                    continue
                # B2: promote only a track still in view (matched this step) whose own
                # sightings span persistence_age_s - a person who stood still for 10 s
                # and walked off must not be promoted two minutes later.
                if (p.auto_promote and allow_promote and not t.promoting
                        and not t.dynamic and t.id in matched
                        and t.observation_count >= p.min_observations
                        and t.last_seen - t.first_seen >= p.persistence_age_s
                        and t.radius <= p.max_obstacle_radius_m
                        and t.area_index >= 0
                        and math.hypot(t.cx - t.origin_cx, t.cy - t.origin_cy)
                        <= p.max_track_drift_m):
                    t.promoting = True
                    to_promote.append(t.id)
                continue

            # promoted obstacle: has the world changed under it?
            seen = t.seen_det or t.id in matched
            t.seen_det = False
            if t.pending and not t.removing:
                # pending zone: refine its size, expire it when not re-seen
                if (t.kind == 'det' and not t.resizing and not t.promoting
                        and abs(t.footprint_radius(p) - t.promoted_radius) >= p.resize_step_m):
                    t.resizing = True
                    to_resize.append(t.id)
                if seen:
                    t.unseen_s = 0.0
                elif allow_promote and robot_area >= 0 and robot_area == t.area_index:
                    t.unseen_s += dt
                if p.pending_expire_s > 0 and t.unseen_s >= p.pending_expire_s:
                    t.removing = True
                    to_remove.append(t.id)
                    continue
            if not p.auto_remove or now - t.promoted_at < p.min_lifetime_s:
                continue
            if t.kind == 'det':
                # YOLO zone (stereo may never see it, as the 2026-10-22 trunk): present =
                # detected this step; absent only when looked at inside the camera view
                if seen:
                    t.gone_votes = 0
                elif obstacle_fresh and self._verify_cone(t, robot_pose):
                    t.gone_votes += 1
                if t.gone_votes >= p.det_gone_confirmations and not t.removing:
                    t.removing = True
                    to_remove.append(t.id)
                continue
            if t.kind == 'bumper':
                continue   # nothing can see it gone; Ignore / expiry only
            present = obstacle_fresh and self._points_within(
                obstacle_points, t.cx, t.cy, p.present_radius_m)
            if present:
                t.gone_votes = 0
                continue
            clear = clear_fresh and self._points_within(
                clear_points, t.cx, t.cy, p.clear_radius_m)
            if clear:
                t.gone_votes += 1
            elif obstacle_fresh:
                # Fresh obstacle cloud, nothing there, and the robot is looking
                # at the spot: a drive-by confirmation.
                near = False
                if robot_pose is not None:
                    dx, dy = t.cx - robot_pose[0], t.cy - robot_pose[1]
                    dist = math.hypot(dx, dy)
                    if dist <= p.verify_distance_m:
                        bearing = math.atan2(dy, dx)
                        diff = abs(math.atan2(math.sin(bearing - robot_pose[2]),
                                              math.cos(bearing - robot_pose[2])))
                        near = diff <= math.radians(p.verify_fov_deg)
                if near:
                    t.gone_votes += 1
            # else: no fresh evidence either way -> hold the votes, don't inflate.
            if t.gone_votes >= p.gone_confirmations and not t.removing:
                t.removing = True
                to_remove.append(t.id)

        return {'promote': to_promote, 'remove': to_remove, 'resize': to_resize}

    # -- restart ------------------------------------------------------------
    def reserve_ids(self, next_id: int) -> None:
        """New tracks get ids >= ``next_id`` (the node passes one above the snapshot
        files kept from before a restart, so a new obstacle never reuses one)."""
        self._next_id = max(self._next_id, int(next_id))

    def adopt_keepouts(self, keepouts, now: float) -> List[int]:
        """Re-adopt stored ``SOURCE_TRACKER`` keepouts as promoted tracks, so one
        accepted before a restart can still be auto-removed once the object is gone.

        ``keepouts``: iterable of ``(area_index, cx, cy, footprint_radius, wanted_id)``
        (``wanted_id`` 0 = any; the node passes the tracker id the keepout was promoted
        under, so its snapshot file keeps matching). A keepout already covered by a
        promoted / promoting track within ``association_dist_m`` is skipped; an
        unpromoted track there is promoted in place. Idempotent: the node calls it on
        every area refresh. Returns the ids of the tracks adopted now.
        """
        p = self.params
        out: List[int] = []
        for entry in keepouts:
            area_index, cx, cy, fp_radius, wanted = entry[:5]
            pending = bool(entry[5]) if len(entry) > 5 else False
            near = [t for t in self.tracks
                    if math.hypot(t.cx - cx, t.cy - cy) <= p.association_dist_m]
            if any(t.promoted or t.promoting for t in near):
                continue
            radius = max(0.0, float(fp_radius) - p.polygon_inflation_m)
            if near:
                t = min(near, key=lambda q: math.hypot(q.cx - cx, q.cy - cy))
                t.cx, t.cy, t.radius = float(cx), float(cy), max(t.radius, radius)
                t.area_index = int(area_index)
            else:
                wanted = int(wanted or 0)
                tid = wanted if wanted > 0 and self.find_track(wanted) is None else self._next_id
                t = Track(id=tid, cx=float(cx), cy=float(cy), radius=radius,
                          first_seen=now, last_seen=now,
                          observation_count=p.min_observations,
                          origin_cx=float(cx), origin_cy=float(cy),
                          area_index=int(area_index))
                self._next_id = max(self._next_id, tid + 1)
                self.tracks.append(t)
            t.promoted, t.promoting, t.adopted = True, False, True
            t.promoted_at, t.gone_votes = now, 0
            t.pending = pending
            t.promoted_radius = t.footprint_radius(p)
            out.append(t.id)
        return out

    # -- service outcomes ---------------------------------------------------
    def confirm_promoted(self, track_id: int, now: float) -> None:
        t = self.find_track(track_id)
        if t is not None:
            t.promoted = True
            t.promoting = False
            t.promoted_at = now
            t.gone_votes = 0
            t.observation_count = max(t.observation_count, self.params.min_observations)
            t.last_seen = now
            t.pending = True        # every tracker promotion is a proposal (B2)
            t.promoted_radius = t.footprint_radius(self.params)
            t.unseen_s = 0.0
            t.resizing = False

    def resize_sent(self, track_id: int, ok: bool) -> None:
        """Outcome of re-sending a pending zone with its new size."""
        t = self.find_track(track_id)
        if t is not None:
            t.resizing = False
            if ok:
                t.promoted_radius = t.footprint_radius(self.params)

    def set_pending(self, track_id: int, pending: bool) -> None:
        """Pending state of a promoted track's keepout as stored by map_server."""
        t = self.find_track(track_id)
        if t is not None and t.promoted:
            t.pending = bool(pending)

    def reject_promotion(self, track_id: int) -> None:
        t = self.find_track(track_id)
        if t is not None:
            t.promoting = False

    def reject_removal(self, track_id: int) -> None:
        t = self.find_track(track_id)
        if t is not None:
            t.removing = False

    def confirm_removed(self, track_id: int) -> None:
        t = self.find_track(track_id)
        if t is not None:
            self.tracks.remove(t)
