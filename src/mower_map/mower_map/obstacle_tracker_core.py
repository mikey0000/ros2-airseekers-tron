# SPDX-License-Identifier: GPL-3.0-or-later
"""ROS-free core of the obstacle auto-marker (mower_map.obstacle_tracker_node).

Clusters obstacle points (the ground-removed stereo depth cloud +
YOLO-ranged detections, both already in the map frame), tracks the clusters
across frames and decides:

* **promote** - a cluster is a reliable, static obstacle (seen long enough,
  often enough, sane size, inside a mowing area) -> the node calls
  ``/map_server_node/promote_obstacle`` so it becomes a permanent keepout the
  planner routes around. Persisted to ``areas.dat``.

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
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from mower_map.areas import point_in_polygon

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
    persistence_age_s: float = 3.0       # track must live this long before promotion
    min_observations: int = 3            # ... and be seen this many updates
    min_obstacle_radius_m: float = 0.05
    max_obstacle_radius_m: float = 5.0
    det_radius_m: float = 0.15           # footprint assumed for a single-point detection
    polygon_inflation_m: float = 0.05    # safety growth of the promoted polygon
    polygon_steps: int = 12              # vertices of the promoted footprint ring
    require_inside_mowing_area: bool = True

    auto_promote: bool = True
    auto_remove: bool = True
    min_lifetime_s: float = 5.0          # never remove sooner than this after promoting
    gone_confirmations: int = 4          # consecutive "gone" frames before removal
    verify_distance_m: float = 3.0       # robot must be this close to "look"
    verify_fov_deg: float = 60.0         # ... and the footprint within this half-cone
    present_radius_m: float = 0.4        # an obstacle point this close = still there
    clear_radius_m: float = 0.4          # a ground point this close = positively gone

    transient_timeout_s: float = 30.0    # drop never-promoted tracks unseen this long
    max_tracks: int = 200
    max_track_drift_m: float = 0.4       # reject a track whose centroid wandered this far


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

    def footprint_radius(self, p: TrackerParams) -> float:
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

    # -- the main step ------------------------------------------------------
    def step(self, obstacle_points: np.ndarray, now: float,
             robot_pose: Optional[Tuple[float, float, float]] = None,
             clear_points: Optional[np.ndarray] = None,
             areas: Sequence[Tuple[int, str, Polygon]] = (),
             obstacle_fresh: bool = True, clear_fresh: bool = True,
             allow_promote: bool = True) -> Dict[str, list]:
        """Advance one tracker update.

        ``obstacle_points`` (N,2) map frame stereo + detection obstacle points.
        ``robot_pose`` (x, y, yaw) map frame, for the drive-by removal check.
        ``clear_points`` (M,2) map frame ground points from the clearing cloud.
        ``areas`` list of ``(area_index, name, polygon)`` mowing areas.
        ``obstacle_fresh`` / ``clear_fresh`` say whether those clouds are
        current; removal votes are only cast from fresh data, so a dead/stale
        sensor can never be read as "the object is gone". ``allow_promote``
        (False when the mower is not mowing, if that gate is enabled) blocks
        promotion without affecting tracking or removal.

        Returns ``{'promote': [track_id, ...], 'remove': [track_id, ...]}``;
        the caller is responsible for the (async) service calls and must call
        :meth:`confirm_promoted` / :meth:`confirm_removed` accordingly.
        """
        p = self.params
        obstacle_points = np.asarray(obstacle_points, dtype=float).reshape(-1, 2) \
            if obstacle_points is not None else np.empty((0, 2))
        clear_points = np.asarray(clear_points, dtype=float).reshape(-1, 2) \
            if clear_points is not None else np.empty((0, 2))

        to_promote: List[int] = []
        to_remove: List[int] = []

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
            if t is not None:
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
        for t in list(self.tracks):
            if not t.promoted:
                if now - t.last_seen > p.transient_timeout_s:
                    self.tracks.remove(t)
                    continue
                if (p.auto_promote and allow_promote and not t.promoting
                        and t.observation_count >= p.min_observations
                        and now - t.first_seen >= p.persistence_age_s
                        and t.area_index >= 0
                        and math.hypot(t.cx - t.origin_cx, t.cy - t.origin_cy)
                        <= p.max_track_drift_m):
                    t.promoting = True
                    to_promote.append(t.id)
                continue

            # promoted obstacle: has the world changed under it?
            if not p.auto_remove or now - t.promoted_at < p.min_lifetime_s:
                continue
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

        return {'promote': to_promote, 'remove': to_remove}

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
