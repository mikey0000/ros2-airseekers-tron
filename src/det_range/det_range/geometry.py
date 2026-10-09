# SPDX-License-Identifier: GPL-3.0-or-later
"""Pure numpy geometry for ``det_range`` (no ROS imports; unit-tested on any host).

Cameras (Metoak front stereo, per-unit calibration
``ros2_port_handoff/08_calibration_identity``):

* source eye = cam1 (RIGHT, colour, raw/distorted 640x480, frame ``vio_camera``): det_ros
  runs on ``/vio/right/image_raw``; intrinsics + plumb_bob distortion from
  ``/vio/right/camera_info`` (defaults = cam1.yaml).
* reference eye = cam0 (LEFT): the Simor hardware disparity (``stereo_depth``) is referenced
  to it, rectified, 640x360, fx = fy = bxf / base = 378 px, c = (319.5, 179.5).
* extrinsics (``stereo_params.yaml``): right relative to left, ``X_r = R X_l + T`` with
  ``R = Rodrigues(Rx, Ry, Rz)`` and ``T = (Tx, Ty, Tz)`` = (-60.05, 0.59, -0.005) mm.

The yaml gives no rectification rotations / projection matrices, so the rectified reference
frame is approximated by the cam0 frame itself (the stereo rotation is < 0.5 deg): a box
corner is undistorted in cam1, back-projected at depth Z, moved into cam0 with R/T and
projected with the depth intrinsics. Z is unknown until sampled, so ``range_box`` starts
from a guess, samples, re-maps at the measured depth and samples once more (the disparity
shift is bxf/Z: 23 px at 1 m, 9 px at 2.5 m), over a short sweep of depth hypotheses (see
``range_box``). Ignoring the rectification rotation costs ~1.7 px (~0.45 deg) of bearing.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

# cam1.yaml (right eye), plumb_bob order k1 k2 p1 p2 k3
CAM1_K = (448.671997, 448.688690, 318.047577, 238.347107)        # fx fy cx cy
CAM1_D = (-0.409014881, 0.199768856, 0.0000428832, 0.000107274, -0.052232463)
# stereo_params.yaml
STEREO_RVEC = (-2.6304218918085098e-03, 4.5345658436417580e-03, -7.1752071380615234e-03)
STEREO_T_M = (-0.060048446655, 0.000586622357, -0.0000052343523)
BXF_MM = 22698.404296875
# stereo_depth defaults (reference eye, rectified 640x360)
REF_K = (378.0, 378.0, 319.5, 179.5)


def rodrigues(r) -> np.ndarray:
    r = np.asarray(r, dtype=float)
    th = float(np.linalg.norm(r))
    if th < 1e-12:
        return np.eye(3)
    kx, ky, kz = r / th
    kk = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]])
    return np.eye(3) + np.sin(th) * kk + (1.0 - np.cos(th)) * (kk @ kk)


def distort(xy: np.ndarray, d: Sequence[float]) -> np.ndarray:
    """Normalised undistorted -> normalised distorted (plumb_bob)."""
    k1, k2, p1, p2, k3 = (list(d) + [0.0] * 5)[:5]
    x, y = xy[:, 0], xy[:, 1]
    r2 = x * x + y * y
    rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
    xd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return np.stack([xd, yd], axis=1)


def undistort_points(uv, k, d, iters: int = 20) -> np.ndarray:
    """Pixels -> normalised undistorted coordinates (fixed-point iteration, like OpenCV)."""
    fx, fy, cx, cy = k
    uv = np.asarray(uv, dtype=float).reshape(-1, 2)
    xd = np.stack([(uv[:, 0] - cx) / fx, (uv[:, 1] - cy) / fy], axis=1)
    if not any(abs(c) > 0 for c in d):
        return xd
    k1, k2, p1, p2, k3 = (list(d) + [0.0] * 5)[:5]
    xy = xd.copy()
    for _ in range(iters):
        x, y = xy[:, 0], xy[:, 1]
        r2 = x * x + y * y
        rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
        dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        xy = np.stack([(xd[:, 0] - dx) / rad, (xd[:, 1] - dy) / rad], axis=1)
    return xy


@dataclass
class StereoModel:
    src_k: Tuple[float, float, float, float] = CAM1_K
    src_d: Tuple[float, ...] = CAM1_D
    r_src_ref: np.ndarray = None          # X_src = R X_ref + T
    t_src_ref: np.ndarray = None
    ref_k: Tuple[float, float, float, float] = REF_K
    ref_size: Tuple[int, int] = (640, 360)
    bxf_m: float = BXF_MM / 1000.0        # baseline[m] * f[px] of the depth image

    def __post_init__(self):
        if self.r_src_ref is None:
            self.r_src_ref = rodrigues(STEREO_RVEC)
        if self.t_src_ref is None:
            self.t_src_ref = np.array(STEREO_T_M, dtype=float)

    def src_to_ref(self, uv, z: float) -> np.ndarray:
        """Source pixels at depth ``z`` (m) -> reference (depth image) pixels, Nx2."""
        return self.norm_to_ref(undistort_points(uv, self.src_k, self.src_d), z)

    def norm_to_ref(self, n, z: float) -> np.ndarray:
        """Undistorted normalised source coordinates at depth ``z`` -> reference pixels."""
        p_src = np.hstack([n, np.ones((n.shape[0], 1))]) * float(z)
        p_ref = (p_src - self.t_src_ref) @ self.r_src_ref      # = R^T (p - T), row-wise
        fx, fy, cx, cy = self.ref_k
        zr = np.maximum(p_ref[:, 2], 1e-6)
        return np.stack([fx * p_ref[:, 0] / zr + cx, fy * p_ref[:, 1] / zr + cy], axis=1)

    def ref_to_src(self, uv, z: float) -> np.ndarray:
        """Reference pixels at depth ``z`` -> source (distorted) pixels (tests / inverse)."""
        fx, fy, cx, cy = self.ref_k
        uv = np.asarray(uv, dtype=float).reshape(-1, 2)
        p_ref = np.stack([(uv[:, 0] - cx) / fx, (uv[:, 1] - cy) / fy,
                          np.ones(len(uv))], axis=1) * float(z)
        p_src = p_ref @ self.r_src_ref.T + self.t_src_ref
        xy = distort(p_src[:, :2] / p_src[:, 2:3], self.src_d)
        sfx, sfy, scx, scy = self.src_k
        return np.stack([sfx * xy[:, 0] + scx, sfy * xy[:, 1] + scy], axis=1)

    def ref_point(self, u: float, v: float, z: float) -> Tuple[float, float, float]:
        fx, fy, cx, cy = self.ref_k
        return ((u - cx) * z / fx, (v - cy) * z / fy, float(z))


def central_box(cx, cy, w, h, frac: float = 0.5):
    """(x1, y1, x2, y2) of the central ``frac`` (per side) of a box."""
    hw, hh = 0.5 * w * frac, 0.5 * h * frac
    return cx - hw, cy - hh, cx + hw, cy + hh


def box_corners(x1, y1, x2, y2) -> np.ndarray:
    xm, ym = 0.5 * (x1 + x2), 0.5 * (y1 + y2)
    return np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2],
                     [xm, y1], [xm, y2], [x1, ym], [x2, ym]], dtype=float)


def map_box(model: StereoModel, box, z: float, norm=None):
    """Map an (x1, y1, x2, y2) source box at depth z to an axis-aligned reference box
    (``norm``: the box corners already undistorted, see ``box_norm``)."""
    if norm is None:
        norm = box_norm(model, box)
    p = model.norm_to_ref(norm, z)
    return float(p[:, 0].min()), float(p[:, 1].min()), float(p[:, 0].max()), float(p[:, 1].max())


def box_norm(model: StereoModel, box) -> np.ndarray:
    return undistort_points(box_corners(*box), model.src_k, model.src_d)


def sample_depth(depth: np.ndarray, roi, min_valid: int = 20, z_min: float = 0.2,
                 z_max: float = 10.0) -> Optional[Tuple[float, int, float]]:
    """Median of valid depth pixels in ``roi`` (x1, y1, x2, y2; clipped to the image).

    Returns ``(median_m, n_valid, mad_m)`` or None when fewer than ``min_valid`` pixels are
    valid (0/NaN/inf/outside [z_min, z_max] are invalid).
    """
    h, w = depth.shape[:2]
    x1, y1, x2, y2 = roi
    c0, c1 = max(0, int(np.floor(x1))), min(w, int(np.ceil(x2)))
    r0, r1 = max(0, int(np.floor(y1))), min(h, int(np.ceil(y2)))
    if c1 <= c0 or r1 <= r0:
        return None
    patch = depth[r0:r1, c0:c1]
    vals = patch[np.isfinite(patch) & (patch >= z_min) & (patch <= z_max)]
    if vals.size < int(min_valid):
        return None
    med = float(np.median(vals))
    mad = float(np.median(np.abs(vals - med)))
    return med, int(vals.size), mad


def range_variance(z: float, mad: float, bxf_m: float = BXF_MM / 1000.0,
                   sigma_disp_px: float = 0.25) -> float:
    """Range variance (m^2): max of the stereo model (dZ = Z^2/bxf * sigma_d) and the
    robust spread of the sampled pixels (1.4826 * MAD)."""
    s_model = z * z / bxf_m * sigma_disp_px
    s_spread = 1.4826 * mad
    return float(max(s_model, s_spread) ** 2)


@dataclass
class RangeResult:
    z: float                  # median depth along the reference optical axis (m)
    n_valid: int
    variance: float           # m^2
    point_ref: Tuple[float, float, float]   # box centre, reference optical frame (m)
    roi_ref: Tuple[float, float, float, float]


DEFAULT_SWEEP_M = (0.4, 0.7, 1.0, 1.5, 2.5, 4.0, 7.0)


def _range_from(model, depth, inner, norm, z0, min_valid, z_min, z_max):
    """Map at z0, sample; re-map at the measured depth and sample again (iterate once).

    Returns (z, n, mad, roi, consistent) or None. ``consistent``: the second sample agrees
    with the depth the box was mapped at (within 10 %), i.e. the box landed on the surface
    it measured and not on a background seen through a wrong disparity shift."""
    roi = map_box(model, inner, z0, norm)
    s = sample_depth(depth, roi, min_valid, z_min, z_max)
    if s is None:
        return None
    roi2 = map_box(model, inner, s[0], norm)
    s2 = sample_depth(depth, roi2, min_valid, z_min, z_max)
    if s2 is None:
        return s[0], s[1], s[2], roi, False
    return s2[0], s2[1], s2[2], roi2, abs(s2[0] - s[0]) <= 0.1 * s[0]


def range_box(model: StereoModel, depth: np.ndarray, cx, cy, w, h, *, z_guess: float = 2.0,
              frac: float = 0.5, min_valid: int = 20, z_min: float = 0.2,
              z_max: float = 10.0, sigma_disp_px: float = 0.25,
              sweep: Sequence[float] = DEFAULT_SWEEP_M) -> Optional[RangeResult]:
    """Range a source-image bbox against the reference depth image.

    The central ``frac`` of the box is mapped into the depth image at a depth hypothesis,
    the median sampled, re-mapped at the measured depth and sampled once more. Because the
    mapping depends on the unknown depth (bxf/Z px: 38 px at 0.6 m), a narrow close object
    can be missed from a far hypothesis; so ``z_guess`` and then every ``sweep`` depth are
    tried, and the NEAREST self-consistent result wins (the safe choice; a far background
    seen past a narrow object is self-consistent too). Falls back to the first
    non-consistent sample when nothing is consistent.
    """
    inner = central_box(cx, cy, w, h, frac)
    norm = box_norm(model, inner)
    best = fallback = None
    tried = set()
    for z0 in (z_guess,) + tuple(sweep):
        if z0 in tried or not (z_min <= z0 <= z_max):
            continue
        tried.add(z0)
        r = _range_from(model, depth, inner, norm, z0, min_valid, z_min, z_max)
        if r is None:
            continue
        if r[4]:
            if best is None or r[0] < best[0]:
                best = r
        elif fallback is None:
            fallback = r
    res = best or fallback
    if res is None:
        return None
    z, n, mad, roi, _ok = res
    u, v = model.src_to_ref([[cx, cy]], z)[0]
    return RangeResult(z=z, n_valid=n,
                       variance=range_variance(z, mad, model.bxf_m, sigma_disp_px),
                       point_ref=model.ref_point(float(u), float(v), z),
                       roi_ref=roi)


def quat_to_matrix(x, y, z, w) -> np.ndarray:
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 0 else 0.0
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


def compose(a, b):
    """(R, t) of a∘b: p_a = R_a (R_b p + t_b) + t_a."""
    return a[0] @ b[0], a[0] @ b[1] + a[1]


def chain_to(static: dict, child: str, target: str, max_depth: int = 16):
    """Compose static transforms {child: (parent, R, t)} from ``child`` up to ``target``.

    Returns (R, t) mapping points in ``child`` to ``target`` or None."""
    acc = (np.eye(3), np.zeros(3))
    frame = child
    for _ in range(max_depth):
        if frame == target:
            return acc
        if frame not in static:
            return None
        parent, r, t = static[frame]
        acc = compose((r, t), acc)
        frame = parent
    return None


# ---------------------------------------------------------------------------
# Monocular ground-plane range (rear camera, 2026-10-09)
# ---------------------------------------------------------------------------
# The rear webcam has no stereo partner: a detection is ranged by intersecting the ray
# through its bbox BOTTOM-CENTRE pixel (where the object touches the lawn) with the ground
# plane z = ground_z in base_link (base_footprint == base_link; the front stereo ground fit
# put the ground at z ~ 0, 2026-10-06), using the camera intrinsics (camera_info K/D) and
# its URDF pose (rear_camera_joint, 0.25 m above base_link, optical axis level).
#
# Accuracy (h = camera height 0.25 m, R = ground range, theta = depression of the ray):
#   dR/dtheta = (R^2 + h^2) / h, so 1 deg of pitch error (the URDF pitch is unmeasured) is
#   ~7 % at 1 m and ~14 % at 2 m; +-3 px of bbox-bottom jitter (fy 684 at 640x480) adds
#   ~1.5 % / ~3 %. A box whose bottom is hidden (behind the dock, cut by the image edge) is
#   ranged on its lowest VISIBLE point: hidden behind something -> estimated FURTHER away
#   (correct side for "behind the dock"); cut by the bottom edge -> the estimate is the
#   closest visible ground (flag ``clipped``; the real object is nearer still). Lawn slope
#   acts like pitch error. Objects not standing on the ground (bird, ball in flight) range
#   too far. Variance published = (dR/dtheta * sigma_theta)^2 with
#   sigma_theta = hypot(sigma_px / fy, sigma_pitch).


@dataclass
class MonoResult:
    point: Tuple[float, float, float]   # ground point in the base frame (m)
    range_m: float                      # planar distance from the camera foot point
    variance: float                     # m^2
    clipped: bool                       # bbox bottom at the image edge (object nearer)


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def ground_ray_point(ray_cam, r_bc: np.ndarray, t_bc, ground_z: float = 0.0,
                     min_depression_deg: float = 2.0) -> Optional[np.ndarray]:
    """Intersect a camera-frame ray (optical frame, any length) with z = ground_z in the
    base frame (camera pose ``r_bc`` / ``t_bc`` in base). None when the ray is not at least
    ``min_depression_deg`` below the horizon (at or above it: no ground hit / unbounded)."""
    rb = r_bc @ np.asarray(ray_cam, dtype=float)
    n = float(np.linalg.norm(rb))
    t = np.asarray(t_bc, dtype=float)
    if n <= 0.0 or t[2] <= ground_z:
        return None
    if rb[2] > -np.sin(np.radians(min_depression_deg)) * n:
        return None
    s = (ground_z - t[2]) / rb[2]
    return t + s * rb


def mono_range_box(cx, cy, w, h, k, d, r_bc: np.ndarray, t_bc, image_height: float = 0.0,
                   *, ground_z: float = 0.0, min_depression_deg: float = 2.0,
                   max_range_m: float = 6.0, clip_px: float = 2.0, sigma_px: float = 3.0,
                   sigma_pitch_deg: float = 1.0) -> Optional[MonoResult]:
    """Range a bbox (source pixels, centre/size) on the ground plane from its bottom-centre
    pixel. ``k`` = (fx, fy, cx, cy), ``d`` = plumb_bob. None: ray above/near the horizon or
    beyond ``max_range_m``."""
    v = float(cy) + 0.5 * float(h)
    nrm = undistort_points([[float(cx), v]], k, d)[0]
    p = ground_ray_point((nrm[0], nrm[1], 1.0), r_bc, t_bc, ground_z, min_depression_deg)
    if p is None:
        return None
    t = np.asarray(t_bc, dtype=float)
    rng = float(np.hypot(p[0] - t[0], p[1] - t[1]))
    if rng > max_range_m:
        return None
    hc = float(t[2] - ground_z)
    sig_th = float(np.hypot(sigma_px / max(float(k[1]), 1e-6), np.radians(sigma_pitch_deg)))
    var = ((rng * rng + hc * hc) / hc * sig_th) ** 2
    clipped = image_height > 0 and v >= float(image_height) - clip_px
    return MonoResult(point=(float(p[0]), float(p[1]), float(p[2])), range_m=rng,
                      variance=float(var), clipped=bool(clipped))
