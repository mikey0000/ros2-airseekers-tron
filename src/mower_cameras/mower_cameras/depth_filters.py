"""Noise and ground filters for the Metoak hardware disparity (used by ``stereo_depth_node``).

Pure numpy (+ OpenCV when importable) so it is unit-testable without ROS. The pipeline turns
the raw disparity of one frame into an obstacle-only point cloud for the Nav2 local costmap:

1. ``speckle_filter``   — ``cv2.filterSpeckles`` on the fixed-point disparity: connected blobs
                          of similar disparity smaller than ``max_size`` px are invalidated
                          (classic SGBM speckle removal; isolated false matches on grass /
                          sky / textureless areas).
2. ``neighbour_filter`` — a pixel survives only if >= ``min_valid`` of its 3x3 neighbours
                          (itself included) are valid.
3. ``depth_to_points``  — decimated back-projection (in ``stereo_depth_node``).
4. ``fit_ground_plane`` — RANSAC + least-squares plane through the points near the prior
                          ground (from TF / URDF), gated to stay within ``max_tilt_deg`` /
                          ``max_dh_m`` of the prior; falls back to the prior.
   ``remove_ground``    — drops points less than ``margin`` above that plane (and below it).
5. ``Persistence``      — a point is published only if its voxel was also occupied in the
                          previous ``n - 1`` frames (kills single-frame flicker).

Plane convention: unit normal ``n`` points from the camera DOWN to the ground and
``h = n . p`` for ground points, so ``height(p) = h - n . p`` (metres above the ground).
"""
from __future__ import annotations

from collections import deque

import numpy as np

try:  # OpenCV is in the mower image; keep the module importable without it.
    import cv2
except ImportError:  # pragma: no cover - exercised only on dev boxes without cv2
    cv2 = None


def speckle_filter(raw, max_size=200, max_diff_px=1.0, scale=32.0):
    """Invalidate (set 0) disparity speckles. ``raw``: uint16 fixed-point disparity."""
    if max_size <= 0:
        return raw
    if cv2 is None:
        return raw
    d = raw.astype(np.int16)  # raw12 <= 4095 fits
    cv2.filterSpeckles(d, 0, int(max_size), float(max_diff_px) * float(scale))
    return d.astype(np.uint16)


def neighbour_filter(valid, min_valid=5):
    """Bool mask: valid pixels with >= ``min_valid`` valid pixels in their 3x3 window."""
    if min_valid <= 1:
        return valid
    v = valid.astype(np.uint8)
    if cv2 is not None:
        cnt = cv2.boxFilter(v, cv2.CV_16U, (3, 3), normalize=False,
                            borderType=cv2.BORDER_CONSTANT)
    else:
        p = np.pad(v, 1).astype(np.uint16)
        h, w = v.shape
        cnt = sum(p[i:i + h, j:j + w] for i in range(3) for j in range(3))
    return valid & (cnt >= int(min_valid))


def plane_from_pose(height, pitch_up_rad=0.0, roll_rad=0.0):
    """Ground plane (n, h) in the optical frame for a camera ``height`` above flat ground."""
    n = np.array([np.sin(roll_rad) * np.cos(pitch_up_rad),
                  np.cos(roll_rad) * np.cos(pitch_up_rad),
                  -np.sin(pitch_up_rad)])
    return n / np.linalg.norm(n), float(height)


def heights(pts, plane):
    n, h = plane
    return h - pts @ n


def fit_ground_plane(pts, prior, band=0.10, iters=20, inlier_tol=0.03, max_tilt_deg=8.0,
                     max_dh_m=0.08, min_inliers=150, max_samples=500, rng=None):
    """RANSAC ground plane near ``prior``. Returns ``(plane, n_inliers, ok)``.

    Only points within ``band`` of the prior ground are candidates, so a box or a person
    cannot capture the fit; the result must stay within ``max_tilt_deg`` / ``max_dh_m`` of
    the prior or the prior is returned with ``ok=False``.
    """
    n0, h0 = prior
    if len(pts) < 3:
        return prior, 0, False
    cand = pts[np.abs(heights(pts, prior)) < band]
    if len(cand) < min_inliers:
        return prior, len(cand), False
    rng = rng or np.random.default_rng(0)
    if len(cand) > max_samples:
        cand = cand[rng.choice(len(cand), max_samples, replace=False)]
    cos_max = np.cos(np.radians(max_tilt_deg))
    best = None
    for _ in range(int(iters)):
        a, b, c = cand[rng.choice(len(cand), 3, replace=False)]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n = n / nn
        if n @ n0 < 0:
            n = -n
        if n @ n0 < cos_max:
            continue
        inl = np.abs(cand @ n - n @ a) < inlier_tol
        k = int(inl.sum())
        if best is None or k > best[0]:
            best = (k, inl)
    if best is None or best[0] < min_inliers:
        return prior, 0 if best is None else best[0], False
    sel = cand[best[1]].astype(np.float64)
    c = sel.mean(axis=0)
    w, v = np.linalg.eigh(np.cov((sel - c).T))
    n = v[:, 0]
    if n @ n0 < 0:
        n = -n
    h = float(n @ c)
    if n @ n0 < cos_max or abs(h - h0) > max_dh_m:
        return prior, best[0], False
    return (n, h), best[0], True


def remove_ground(pts, plane, margin=0.12):
    """Keep only points more than ``margin`` above the ground plane."""
    return pts[heights(pts, plane) > margin]


class Persistence:
    """Temporal filter: keep points whose voxel was occupied in the last ``n`` frames."""

    def __init__(self, n=2, voxel=0.10):
        self.n = max(1, int(n))
        self.voxel = float(voxel)
        self.hist = deque(maxlen=self.n - 1)

    def _keys(self, pts):
        q = np.floor(pts / self.voxel).astype(np.int64)
        # pack 3 x 21-bit signed ints into one int64 key
        q += 1 << 20
        return (q[:, 0] << 42) | (q[:, 1] << 21) | q[:, 2]

    def __call__(self, pts):
        if self.n <= 1:
            return pts
        keys = self._keys(pts) if len(pts) else np.zeros(0, np.int64)
        keep = np.ones(len(pts), bool)
        if len(self.hist) < self.n - 1:
            keep[:] = False
        for prev in self.hist:
            keep &= np.isin(keys, prev)
        self.hist.append(np.unique(keys))
        return pts[keep]

    def reset(self):
        self.hist.clear()
