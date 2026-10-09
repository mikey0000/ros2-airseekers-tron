# SPDX-License-Identifier: GPL-3.0-or-later
"""Monocular ground-plane ranging (rear camera): round trips, limits, accuracy note."""
import numpy as np
import pytest

from det_range.geometry import (MonoResult, distort, ground_ray_point, mono_range_box,
                                rpy_matrix)

K = (684.66634, 684.32908, 331.68625, 231.38923)
D = (-0.373942, 0.165317, -0.000099, -0.000519, 0.0)
T = np.array([-0.201, 0.0, 0.25])
RPY = (-1.5707963, 0.0, 1.5707963)
R = rpy_matrix(*RPY)
H_IMG = 480
POINTS = [(-1.0, 0.0), (-1.5, 0.3), (-2.0, -0.4), (-0.9, 0.2)]


def project(xy, d=D, r=R, t=T):
    """Ground point (z=0) in base -> source pixel (u, v)."""
    pc = r.T @ (np.array([xy[0], xy[1], 0.0]) - t)
    n = np.array([[pc[0] / pc[2], pc[1] / pc[2]]])
    nd = distort(n, d)[0]
    return K[0] * nd[0] + K[2], K[1] * nd[1] + K[3]


def range_at(xy, h=60.0, d=D, **kw):
    u, v = project(xy, d)
    return mono_range_box(u, v - h / 2, 40.0, h, K, d, R, T, image_height=H_IMG, **kw)


def test_pose_convention():
    """Optical z = -x_base, optical x = +y_base, optical y = down."""
    np.testing.assert_allclose(R @ [0, 0, 1], [-1, 0, 0], atol=1e-6)
    np.testing.assert_allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-6)
    np.testing.assert_allclose(R @ [0, 1, 0], [0, 0, -1], atol=1e-6)


@pytest.mark.parametrize('d', [D, (0.0,) * 5], ids=['distorted', 'no_distortion'])
@pytest.mark.parametrize('xy', POINTS)
def test_ground_truth_roundtrip(xy, d):
    res = range_at(xy, d=d)
    assert isinstance(res, MonoResult)
    assert np.hypot(res.point[0] - xy[0], res.point[1] - xy[1]) < 0.01
    assert res.point[2] == pytest.approx(0.0, abs=1e-6)
    assert res.range_m == pytest.approx(np.hypot(res.point[0] - T[0], res.point[1] - T[1]))


def test_horizon_returns_none():
    assert mono_range_box(K[2], K[3] - 50.0, 40.0, 100.0, K, D, R, T) is None  # bottom at cy
    assert mono_range_box(K[2], K[3] - 80.0, 40.0, 100.0, K, D, R, T) is None  # above


def test_beyond_max_range_returns_none():
    assert range_at((-1.5, 0.0), max_range_m=1.0) is None
    assert range_at((-1.5, 0.0), max_range_m=2.0) is not None


def test_clipped_flag():
    u, _ = project((-1.0, 0.0))
    h = 60.0
    assert mono_range_box(u, H_IMG - 1 - h / 2, 40.0, h, K, D, R, T,
                          image_height=H_IMG).clipped
    assert mono_range_box(u, H_IMG - 2 - h / 2, 40.0, h, K, D, R, T,
                          image_height=H_IMG).clipped
    assert not mono_range_box(u, H_IMG - 10 - h / 2, 40.0, h, K, D, R, T,
                              image_height=H_IMG).clipped
    # unknown image height never clips
    assert not mono_range_box(u, H_IMG - 1 - h / 2, 40.0, h, K, D, R, T).clipped


def test_variance_grows_with_range():
    v1 = range_at((-0.799, 0.0)).variance   # 1 m from the camera foot point
    v2 = range_at((-2.201, 0.0)).variance   # 2 m
    assert 0.0 < v1 < v2


def test_pitch_error_bias_and_monotonic():
    """A 1 deg pitch error biases a ~1 m range by 3-15 % (accuracy note) but stays monotonic."""
    r_true = rpy_matrix(RPY[0] + 0.01745, RPY[1], RPY[2])
    est = []
    for dist in (1.0, 1.5, 2.0):
        xy = (T[0] - dist, 0.0)
        u, v = project(xy, r=r_true)
        res = mono_range_box(u, v - 30.0, 40.0, 60.0, K, D, R, T, image_height=H_IMG)
        assert res is not None
        est.append(res.range_m)
    err1 = abs(est[0] - 1.0) / 1.0
    assert 0.03 < err1 < 0.15, est
    assert est[0] < est[1] < est[2]


def test_ground_ray_point_none_when_camera_not_above_ground():
    ray = (0.0, 1.0, 0.0)  # looks down-ish in the base frame via R
    assert ground_ray_point(ray, R, (T[0], 0.0, 0.25)) is not None
    assert ground_ray_point(ray, R, (T[0], 0.0, 0.0)) is None
    assert ground_ray_point(ray, R, (T[0], 0.0, -0.1)) is None
    assert ground_ray_point(ray, R, (T[0], 0.0, 0.25), ground_z=0.25) is None
