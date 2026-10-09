"""Ground projection of the seg_ros class mask (pure numpy, no ROS; unit-tested on any host).

2026-10-09. Turns one segmentation frame into per-map-cell "non-grass" / "grass" votes for the
non-grass memory in ``mower_map`` (``mower_map/nongrass.py``; docs/grass_segmentation.md).

Geometry (per frame):

* A fixed grid of sample pixels (every ``stride`` px) is undistorted once per camera into
  normalised rays ``[x, y, 1]`` in the camera optical frame (plumb_bob, OpenCV fixed-point).
* The camera pose in ``base_link`` is TF(base_link <- ref_frame) composed with a fixed
  camera -> ref extrinsic. For the front stereo right colour eye (frame ``vio_camera``)
  ref_frame is ``stereo_camera_optical`` (= the depth reference eye cam0) and the extrinsic
  is the per-unit ``stereo_params.yaml`` (``X_src = R X_ref + T``, same convention and
  values as ``det_range.geometry``).
* Each ray is intersected with the ground plane ``n . p = d``: the live
  ``/stereo_depth/ground_plane`` fit (in stereo_camera_optical) when it is fresh and close to
  the prior, else the URDF flat ground (``z = 0`` in base_link: base_link sits on the ground,
  the stereo optical frame is 0.240 m above it, measured 2026-10-06).
* Optional depth check (the "prefer real depth" part): the ground point is projected into the
  hardware depth image (rectified reference eye, approximated by cam0 like det_range). If the
  measured depth there is clearly SHORTER than the ground point's depth, the pixel sees
  something standing on the ground (toy, shrub, post): it is dropped. Stereo + det_range
  already put 3D things in the costmap; this layer is for what stereo cannot see because it is
  flat (flower bed soil, gravel, paving, water). Flat ground projection of a tall object would
  also smear it far behind its true position.
* Pixels are labelled by class + confidence: a confident non-grass class is a hit, a confident
  grass/soil class a miss (it clears), anything unsure is ignored. ``soil`` (class 3) counts as
  GRASS: worn / dry patches in the lawn are mowable and must never become non-grass.
* Points within ``min_range_m..max_range_m`` (horizontal, from the camera) and
  ``|y| <= max_lateral_m`` survive, are moved to the map frame and binned per map cell; a cell
  is one vote per frame (non-grass only if hits > misses in that cell this frame).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

# Model classes (seg_ros/ppseg_postprocess.CLASSES): 0 other, 1 grass, 2 passway, 3 soil,
# 4 obstacle, 5 shrub.
NONGRASS_CLASSES = (0, 2, 4, 5)
GRASS_CLASSES = (1, 3)

# Front stereo right colour eye (cam1.yaml) and stereo extrinsics (stereo_params.yaml), the
# per-unit calibration of sn IC17SZ01011 (same values as det_range/geometry.py).
CAM1_K = (448.671997, 448.688690, 318.047577, 238.347107)        # fx fy cx cy @ 640x480
CAM1_D = (-0.409014881, 0.199768856, 0.0000428832, 0.000107274, -0.052232463)
CAM1_SIZE = (640, 480)
STEREO_RVEC = (-2.6304218918085098e-03, 4.5345658436417580e-03, -7.1752071380615234e-03)
STEREO_T_M = (-0.060048446655, 0.000586622357, -0.0000052343523)
# stereo_depth reference eye (rectified 640x360)
REF_K = (378.0, 378.0, 319.5, 179.5)

Rt = Tuple[np.ndarray, np.ndarray]   # (R 3x3, t 3): p_parent = R p_child + t


# --------------------------------------------------------------------------- small helpers
def rodrigues(r) -> np.ndarray:
    r = np.asarray(r, dtype=float)
    th = float(np.linalg.norm(r))
    if th < 1e-12:
        return np.eye(3)
    kx, ky, kz = r / th
    kk = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]])
    return np.eye(3) + math.sin(th) * kk + (1.0 - math.cos(th)) * (kk @ kk)


def quat_to_matrix(x, y, z, w) -> np.ndarray:
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 0 else 0.0
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


def compose(a: Rt, b: Rt) -> Rt:
    """a o b: p_a = R_a (R_b p + t_b) + t_a."""
    return a[0] @ b[0], a[0] @ b[1] + a[1]


def invert(a: Rt) -> Rt:
    return a[0].T, -(a[0].T @ a[1])


def src_in_ref(rvec=STEREO_RVEC, t=STEREO_T_M) -> Rt:
    """Pose of the source eye in the reference eye frame from ``X_src = R X_ref + T``:
    X_ref = R^T X_src - R^T T."""
    r = rodrigues(rvec)
    return invert((r, np.asarray(t, dtype=float)))


def undistort_points(uv, k, d, iters: int = 20) -> np.ndarray:
    """Pixels -> normalised undistorted coordinates (fixed-point iteration, like OpenCV;
    same as det_range.geometry.undistort_points)."""
    fx, fy, cx, cy = k
    uv = np.asarray(uv, dtype=float).reshape(-1, 2)
    xd = np.stack([(uv[:, 0] - cx) / fx, (uv[:, 1] - cy) / fy], axis=1)
    d = list(d or ())
    if not any(abs(c) > 0 for c in d):
        return xd
    k1, k2, p1, p2, k3 = (d + [0.0] * 5)[:5]
    xy = xd.copy()
    for _ in range(iters):
        x, y = xy[:, 0], xy[:, 1]
        r2 = x * x + y * y
        rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
        dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        xy = np.stack([(xd[:, 0] - dx) / rad, (xd[:, 1] - dy) / rad], axis=1)
    return xy


def scale_intrinsics(k, from_size, to_size):
    """K of a ``from_size`` (w, h) calibration for an image of ``to_size`` (pure scaling)."""
    sx = float(to_size[0]) / float(from_size[0])
    sy = float(to_size[1]) / float(from_size[1])
    fx, fy, cx, cy = k
    return (fx * sx, fy * sy, (cx + 0.5) * sx - 0.5, (cy + 0.5) * sy - 0.5)


# --------------------------------------------------------------------------- rays
@dataclass
class RayGrid:
    """Sample pixels (integer u, v) and their normalised rays in the camera optical frame."""
    u: np.ndarray
    v: np.ndarray
    rays: np.ndarray            # N x 3, [x, y, 1]
    size: Tuple[int, int]       # (w, h) of the mask this grid indexes


def make_ray_grid(size, k, d, stride: int = 8, row_min: int = 0) -> RayGrid:
    """Every ``stride`` px (pixel centres of the strided cells) from row ``row_min`` down."""
    w, h = int(size[0]), int(size[1])
    s = max(1, int(stride))
    us = np.arange(s // 2, w, s)
    vs = np.arange(max(0, int(row_min)) + s // 2, h, s)
    uu, vv = np.meshgrid(us, vs)
    u, v = uu.ravel().astype(np.int64), vv.ravel().astype(np.int64)
    n = undistort_points(np.stack([u, v], axis=1).astype(float), k, d)
    rays = np.hstack([n, np.ones((n.shape[0], 1))])
    return RayGrid(u=u, v=v, rays=rays, size=(w, h))


# --------------------------------------------------------------------------- ground plane
def transform_plane(n, d, a: Rt):
    """Plane ``n . p = d`` in frame A -> frame B with ``p_B = R p_A + t``."""
    nb = a[0] @ np.asarray(n, dtype=float)
    return nb, float(d) + float(nb @ a[1])


def flat_ground_base():
    """URDF flat ground in base_link: z = 0 (normal up, n . p = 0)."""
    return np.array([0.0, 0.0, 1.0]), 0.0


def plane_from_fit(msg_data) -> Optional[Tuple[np.ndarray, float]]:
    """``/stereo_depth/ground_plane`` [nx, ny, nz, h, inliers, fit_ok] (optical frame, n
    points down to the ground, h = camera height) -> (n, d) with n . p = d, or None."""
    if msg_data is None or len(msg_data) < 6 or not msg_data[5]:
        return None
    n = np.asarray(msg_data[:3], dtype=float)
    nn = float(np.linalg.norm(n))
    if not math.isfinite(nn) or nn < 1e-6 or not math.isfinite(float(msg_data[3])):
        return None
    return n / nn, float(msg_data[3]) / nn


def plane_agrees(live, prior, max_dh_m: float = 0.06, max_angle_deg: float = 6.0) -> bool:
    """A live plane (same frame as prior) is used only near the prior: a bad RANSAC fit (a
    wall, a hedge) must not tilt the whole projection. Both planes are (n, d), n unit."""
    (n1, d1), (n0, d0) = live, prior
    cosang = float(np.clip(abs(n1 @ n0), -1.0, 1.0))
    if math.degrees(math.acos(cosang)) > max_angle_deg:
        return False
    sign = 1.0 if float(n1 @ n0) >= 0 else -1.0
    return abs(sign * d1 - d0) <= max_dh_m


def intersect_plane(origin, dirs, n, d, min_grazing: float = 1e-3):
    """Rays ``origin + s * dirs`` (s > 0) against ``n . p = d``.

    Returns (points N x 3, valid N bool). Rays parallel to or pointing away from the plane
    (above the horizon) are invalid."""
    o = np.asarray(origin, dtype=float)
    dirs = np.asarray(dirs, dtype=float)
    n = np.asarray(n, dtype=float)
    nd = dirs @ n
    num = float(d) - float(n @ o)
    with np.errstate(divide='ignore', invalid='ignore'):
        s = num / nd
    # the camera is above the plane (num has the sign of the side the plane is on) and the ray
    # must head towards it: s > 0 and not grazing.
    valid = np.isfinite(s) & (s > 0) & (np.abs(nd) > min_grazing * np.linalg.norm(dirs, axis=1))
    pts = o[None, :] + np.where(valid, s, 0.0)[:, None] * dirs
    return pts, valid


# --------------------------------------------------------------------------- depth check
def depth_elevated(points_ref, depth, ref_k=REF_K, tol_m: float = 0.08,
                   tol_frac: float = 0.10, z_min: float = 0.2):
    """Ground points in the depth reference optical frame -> (elevated, checked) bool arrays.

    ``checked``: the point projects inside the depth image onto a valid depth pixel.
    ``elevated``: checked and the measured depth is shorter than the ground point's depth by
    more than max(tol_m, tol_frac * z): something stands between the camera and the ground."""
    p = np.asarray(points_ref, dtype=float)
    n = p.shape[0]
    elevated = np.zeros(n, bool)
    checked = np.zeros(n, bool)
    if depth is None or n == 0:
        return elevated, checked
    h, w = depth.shape[:2]
    fx, fy, cx, cy = ref_k
    z = p[:, 2]
    front = z > z_min
    with np.errstate(divide='ignore', invalid='ignore'):
        u = np.rint(fx * p[:, 0] / z + cx)
        v = np.rint(fy * p[:, 1] / z + cy)
    inside = front & np.isfinite(u) & np.isfinite(v) & (u >= 0) & (u < w) & (v >= 0) & (v < h)
    idx = np.nonzero(inside)[0]
    if idx.size == 0:
        return elevated, checked
    zm = depth[v[idx].astype(np.int64), u[idx].astype(np.int64)].astype(float)
    ok = np.isfinite(zm) & (zm > z_min)
    zg = z[idx]
    tol = np.maximum(tol_m, tol_frac * zg)
    checked[idx[ok]] = True
    elevated[idx[ok & (zm < zg - tol)]] = True
    return elevated, checked


# --------------------------------------------------------------------------- labels
def label_pixels(classes, conf, nongrass_classes=NONGRASS_CLASSES,
                 grass_classes=GRASS_CLASSES, min_conf_nongrass: float = 0.8,
                 min_conf_grass: float = 0.6):
    """Per-sample vote: 1 = confident non-grass (hit), 0 = confident grass (miss),
    -1 = ignore (unsure, or a class in neither list). ``conf`` is uint8 (x255) or float 0..1;
    None = fully trusted."""
    classes = np.asarray(classes)
    if conf is None:
        c = np.ones(classes.shape, float)
    else:
        c = np.asarray(conf)
        c = c.astype(float) / 255.0 if c.dtype == np.uint8 else c.astype(float)
    out = np.full(classes.shape, -1, np.int8)
    ng = np.isin(classes, np.asarray(nongrass_classes)) & (c >= min_conf_nongrass)
    gr = np.isin(classes, np.asarray(grass_classes)) & (c >= min_conf_grass)
    out[gr] = 0
    out[ng] = 1
    return out


# --------------------------------------------------------------------------- frame
@dataclass
class ProjectionParams:
    min_range_m: float = 0.35
    max_range_m: float = 2.0
    max_lateral_m: float = 1.5
    min_conf_nongrass: float = 0.8
    min_conf_grass: float = 0.6
    nongrass_classes: Sequence[int] = NONGRASS_CLASSES
    grass_classes: Sequence[int] = GRASS_CLASSES
    depth_tol_m: float = 0.08
    depth_tol_frac: float = 0.10


@dataclass
class FrameVotes:
    """Surviving samples in base_link: x, y (m) and vote (1 hit, 0 miss)."""
    x: np.ndarray
    y: np.ndarray
    vote: np.ndarray
    n_rays: int = 0
    n_ground: int = 0
    n_elevated: int = 0


def project_frame(classes: np.ndarray, conf: Optional[np.ndarray], grid: RayGrid,
                  cam_in_base: Rt, plane_base, params: ProjectionParams,
                  depth: Optional[np.ndarray] = None, base_to_ref: Optional[Rt] = None,
                  ref_k=REF_K) -> FrameVotes:
    """One mask (+ confidence) -> ground votes in base_link.

    ``cam_in_base``: camera optical frame -> base_link. ``plane_base``: (n, d) in base_link.
    ``depth`` + ``base_to_ref`` (base_link -> depth reference optical frame) enable the
    elevated-pixel rejection."""
    h, w = classes.shape[:2]
    if (w, h) != tuple(grid.size):
        raise ValueError('mask %dx%d does not match the ray grid %dx%d'
                         % (w, h, grid.size[0], grid.size[1]))
    cls = classes[grid.v, grid.u]
    cf = None if conf is None else conf[grid.v, grid.u]
    vote = label_pixels(cls, cf, params.nongrass_classes, params.grass_classes,
                        params.min_conf_nongrass, params.min_conf_grass)
    keep = vote >= 0
    r, o = cam_in_base
    dirs = grid.rays[keep] @ r.T
    pts, valid = intersect_plane(o, dirs, plane_base[0], plane_base[1])
    vote = vote[keep][valid]
    pts = pts[valid]
    n_ground = int(pts.shape[0])
    dx, dy = pts[:, 0] - o[0], pts[:, 1] - o[1]
    rng = np.hypot(dx, dy)
    inr = (rng >= params.min_range_m) & (rng <= params.max_range_m) & \
        (np.abs(pts[:, 1]) <= params.max_lateral_m)
    pts, vote = pts[inr], vote[inr]
    n_elev = 0
    if depth is not None and base_to_ref is not None and pts.shape[0]:
        p_ref = pts @ base_to_ref[0].T + base_to_ref[1]
        elev, _checked = depth_elevated(p_ref, depth, ref_k, params.depth_tol_m,
                                        params.depth_tol_frac)
        n_elev = int(elev.sum())
        pts, vote = pts[~elev], vote[~elev]
    return FrameVotes(x=pts[:, 0].copy(), y=pts[:, 1].copy(), vote=vote.astype(np.int8),
                      n_rays=int(grid.u.size), n_ground=n_ground, n_elevated=n_elev)


def to_map(votes: FrameVotes, base_in_map: Rt):
    """Base_link ground points -> map x, y (z of the ground point taken as 0 in base_link)."""
    r, t = base_in_map
    p = np.stack([votes.x, votes.y, np.zeros_like(votes.x)], axis=1)
    m = p @ r.T + t
    return m[:, 0], m[:, 1]


def bin_cells(mx, my, vote, resolution: float = 0.1):
    """Map points + votes -> one vote per cell: (cx, cy cell centres, nongrass bool, n samples).

    Cells are snapped to multiples of ``resolution`` (the mower_map mask grid has its origin
    snapped the same way, so the centres land in exactly one raster cell). A cell is non-grass
    for this frame only if its hits outnumber its misses (ties count as grass)."""
    mx = np.asarray(mx, dtype=float)
    if mx.size == 0:
        e = np.zeros(0)
        return e, e, np.zeros(0, bool), np.zeros(0, np.int32)
    ix = np.floor(mx / resolution).astype(np.int64)
    iy = np.floor(np.asarray(my, dtype=float) / resolution).astype(np.int64)
    key = np.stack([ix, iy], axis=1)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    hits = np.bincount(inv, weights=(np.asarray(vote) == 1).astype(float), minlength=len(uniq))
    tot = np.bincount(inv, minlength=len(uniq))
    cx = (uniq[:, 0] + 0.5) * resolution
    cy = (uniq[:, 1] + 0.5) * resolution
    return cx, cy, hits > (tot - hits), tot.astype(np.int32)


class MotionGate:
    """Forward a frame only after the camera moved ``min_move_m`` or turned ``min_turn_deg``
    since the last forwarded frame: a parked robot (docked, paused, waiting at an obstacle)
    must not pile up votes from one viewpoint, which is what turns a fixed shadow or glare
    into a confirmed mark."""

    def __init__(self, min_move_m: float = 0.15, min_turn_deg: float = 10.0):
        self.min_move = float(min_move_m)
        self.min_turn = math.radians(float(min_turn_deg))
        self.last = None

    def accept(self, x: float, y: float, yaw: float) -> bool:
        if self.last is not None:
            lx, ly, lyaw = self.last
            dyaw = abs((yaw - lyaw + math.pi) % (2 * math.pi) - math.pi)
            if math.hypot(x - lx, y - ly) < self.min_move and dyaw < self.min_turn:
                return False
        self.last = (x, y, yaw)
        return True
