"""Synthetic-depth tests for mower_cameras.depth_filters (no ROS needed).

The scene is rendered as Metoak fixed-point disparity (raw12 = 32 * bxf / Z) for a camera
0.24 m above flat ground, pitched up 0.92 deg (the 2026-10-06 measurement), then run through
the same steps as stereo_depth_node.filtered_points.
"""
import numpy as np
import pytest

f = pytest.importorskip('mower_cameras.depth_filters')

W, H, FX, CX, CY, BXF, SCALE = 640, 360, 378.0, 319.5, 179.5, 22.6984, 32.0
CAM_H, PITCH = 0.24, np.radians(0.92)


def _rays():
    v, u = np.mgrid[0:H, 0:W].astype(np.float64)
    return np.stack([(u - CX) / FX, (v - CY) / FX, np.ones_like(u)], axis=-1)


def _ground_depth(rng, noise=0.01):
    n, h = f.plane_from_pose(CAM_H, PITCH)
    r = _rays()
    nr = r @ n
    z = np.zeros((H, W))
    ok = nr > 1e-3
    z[ok] = (h + rng.normal(0, noise, ok.sum())) / nr[ok]   # grass: +-1 cm height noise
    z[(z < 0.2) | (z > 6.0)] = 0
    return z


def _add_box(z, dist=1.0, half_w=0.15, height=0.30):
    """Vertical 0.3 m box face ``dist`` m ahead (camera frame), occluding the ground."""
    n, h = f.plane_from_pose(CAM_H, PITCH)
    r = _rays()
    x = r[..., 0] * dist
    y = r[..., 1] * dist
    p = np.stack([x, y, np.full_like(x, dist)], axis=-1)
    hgt = h - p @ n
    face = (np.abs(x) < half_w) & (hgt > 0) & (hgt < height)
    z = z.copy()
    z[face] = dist
    return z


def _to_raw(z):
    raw = np.zeros(z.shape, np.uint16)
    ok = z > 0
    raw[ok] = np.clip(np.round(SCALE * BXF / z[ok]), 1, 4095).astype(np.uint16)
    return raw


def _add_speckle(raw, rng, n=300):
    """Isolated false matches (large disparity = 'very close' points), 1-4 px blobs."""
    raw = raw.copy()
    for _ in range(n):
        v, u = rng.integers(0, H - 2), rng.integers(0, W - 2)
        raw[v:v + rng.integers(1, 3), u:u + rng.integers(1, 3)] = rng.integers(1500, 4000)
    return raw


def _pipeline(raw, persist=None, step=4, margin=0.12, max_z=4.0, rng=None, sub=1):
    """Mirror of stereo_depth_node.filtered_points (``sub`` = 2 is the node's half-res path)."""
    raw = f.speckle_filter(raw[::sub, ::sub], 200 // (sub * sub), 1.0, SCALE)
    valid = f.neighbour_filter(raw > 0, 5)
    d = np.where(valid, raw, 0).astype(np.float64) / SCALE
    z = np.zeros_like(d)
    z[d >= 1] = BXF / d[d >= 1]
    s = step // sub
    zs = z[::s, ::s]
    v, u = np.mgrid[0:z.shape[0]:s, 0:z.shape[1]:s]
    ok = (zs >= 0.2) & (zs <= max_z)
    zz = zs[ok]
    pts = np.stack([(u[ok] - CX / sub) * zz / (FX / sub), (v[ok] - CY / sub) * zz / (FX / sub),
                    zz], axis=1)
    prior = f.plane_from_pose(0.215, 0.0)    # deliberately the old (wrong) URDF pose
    plane, _, ok_fit = f.fit_ground_plane(pts, prior, rng=rng or np.random.default_rng(0))
    out = f.remove_ground(pts, plane, margin)
    if persist is not None:
        out = persist(out)
    return out, plane, ok_fit


@pytest.mark.parametrize('sub', [1, 2])
def test_noisy_ground_gives_no_obstacles(sub):
    rng = np.random.default_rng(1)
    raw = _add_speckle(_to_raw(_ground_depth(rng)), rng)
    pts, plane, ok = _pipeline(raw, sub=sub)
    assert ok
    assert plane[1] == pytest.approx(CAM_H, abs=0.01)
    assert np.degrees(np.arccos(plane[0][1])) == pytest.approx(0.92, abs=0.3)
    assert len(pts) == 0


def test_speckle_alone_would_mark_without_filters():
    rng = np.random.default_rng(2)
    raw = _add_speckle(_to_raw(_ground_depth(rng)), rng)
    assert np.count_nonzero(f.speckle_filter(raw, 200, 1.0, SCALE) != raw) > 100


@pytest.mark.parametrize('sub', [1, 2])
def test_box_cluster_is_retained(sub):
    rng = np.random.default_rng(3)
    raw = _add_speckle(_to_raw(_add_box(_ground_depth(rng))), rng)
    pts, _, ok = _pipeline(raw, sub=sub)
    assert ok
    assert len(pts) >= 20
    assert np.all(np.abs(pts[:, 2] - 1.0) < 0.05)        # all at the box face
    assert np.all(np.abs(pts[:, 0]) < 0.2)


def test_persistence_needs_consecutive_frames():
    rng = np.random.default_rng(4)
    per = f.Persistence(2, 0.10)
    frame = _to_raw(_add_box(_ground_depth(rng)))
    first, _, _ = _pipeline(frame, per)
    second, _, _ = _pipeline(_to_raw(_add_box(_ground_depth(rng))), per)
    assert len(first) == 0 and len(second) >= 20
    # one-frame flicker (box vanished) is not carried over
    third, _, _ = _pipeline(_to_raw(_ground_depth(rng)), per)
    fourth, _, _ = _pipeline(frame, per)
    assert len(third) == 0 and len(fourth) == 0


def test_box_cannot_capture_ground_fit():
    """A wall filling most of the view must not be accepted as the ground."""
    z = np.full((H, W), 1.0)
    pts_plane, _, ok = _pipeline(_to_raw(z))
    assert not ok


def test_neighbour_filter_counts():
    m = np.zeros((5, 5), bool)
    m[2, 2] = True
    assert not f.neighbour_filter(m, 2).any()
    m[1:4, 1:4] = True
    assert f.neighbour_filter(m, 5)[2, 2]
