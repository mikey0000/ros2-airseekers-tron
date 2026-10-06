# SPDX-License-Identifier: GPL-3.0-or-later
import numpy as np
import pytest

from det_range.geometry import (StereoModel, chain_to, map_box, quat_to_matrix, range_box,
                                range_variance, sample_depth, undistort_points, distort)

M = StereoModel()


def test_undistort_roundtrip():
    xy = np.array([[0.0, 0.0], [0.3, -0.2], [-0.5, 0.4]])
    d = distort(xy, M.src_d)
    uv = np.stack([M.src_k[0] * d[:, 0] + M.src_k[2], M.src_k[1] * d[:, 1] + M.src_k[3]], 1)
    np.testing.assert_allclose(undistort_points(uv, M.src_k, M.src_d), xy, atol=1e-6)


@pytest.mark.parametrize('z', [0.6, 1.0, 2.5, 5.0])
def test_src_ref_roundtrip(z):
    ref = np.array([[320.0, 180.0], [100.0, 50.0], [500.0, 300.0]])
    src = M.ref_to_src(ref, z)
    # z is the source-eye depth on the way in and the reference-eye depth on the way back
    # (they differ by the < 0.5 deg stereo rotation): sub-pixel agreement.
    np.testing.assert_allclose(M.src_to_ref(src, z), ref, atol=0.2)


def test_disparity_shift_matches_baseline():
    # A right-eye pixel shifts right in the left (reference) eye by bxf/Z (+ a constant
    # offset from the small stereo rotation): the depth-dependent part must equal bxf/Z.
    c = [[M.src_k[2], M.src_k[3]]]
    u = {z: M.src_to_ref(c, z)[0] for z in (0.5, 1.0, 2.0, 8.0)}
    for z in (0.5, 1.0, 2.0):
        dz = u[z][0] - u[8.0][0]
        assert dz == pytest.approx(M.bxf_m * (1 / z - 1 / 8.0), rel=0.02)
        assert abs(u[z][1] - M.ref_k[3]) < 3.0


def test_sample_depth_median_and_min_valid():
    d = np.zeros((360, 640), np.float32)
    d[100:110, 200:202] = 1.5          # 20 valid px
    d[100, 200] = 9.0                  # one outlier
    med, n, mad = sample_depth(d, (190, 90, 220, 120))
    assert n == 20 and med == pytest.approx(1.5)
    d[100:110, 201] = 0.0              # 10 valid -> rejected
    assert sample_depth(d, (190, 90, 220, 120)) is None
    d[:] = np.nan
    assert sample_depth(d, (0, 0, 640, 360)) is None
    assert sample_depth(d, (700, 0, 800, 10)) is None


def _scene(z_obj=1.2, z_bg=4.0, ref_box=(260, 120, 380, 300)):
    d = np.full((360, 640), z_bg, np.float32)
    x1, y1, x2, y2 = ref_box
    d[y1:y2, x1:x2] = z_obj
    d[::7, ::5] = 0.0                  # sparse invalid holes
    return d


def test_range_box_synthetic_object():
    z_obj, ref_box = 1.2, (260, 120, 380, 300)
    depth = _scene(z_obj, ref_box=ref_box)
    # The object seen by the right eye: map the reference box into the source image.
    src = M.ref_to_src(np.array([[ref_box[0], ref_box[1]], [ref_box[2], ref_box[3]]]), z_obj)
    (sx1, sy1), (sx2, sy2) = src
    res = range_box(M, depth, (sx1 + sx2) / 2, (sy1 + sy2) / 2, sx2 - sx1, sy2 - sy1,
                    z_guess=3.0)
    assert res is not None
    assert res.z == pytest.approx(z_obj, abs=1e-4)
    assert res.n_valid >= 20
    assert res.point_ref[2] == pytest.approx(z_obj)
    # box centre in the reference eye
    assert res.point_ref[0] == pytest.approx((320 - M.ref_k[2]) * z_obj / M.ref_k[0], abs=0.02)
    assert 0 < res.variance < 0.01


def test_range_box_close_object_needs_the_baseline_shift():
    # A narrow object at 0.6 m: the depth-dependent shift (bxf/Z ~ 38 px) exceeds its width,
    # so mapping at the wrong depth samples the background; the iteration recovers it.
    z_obj, ref_box = 0.6, (300, 150, 330, 250)
    depth = _scene(z_obj, ref_box=ref_box)
    src = M.ref_to_src(np.array([[ref_box[0], ref_box[1]], [ref_box[2], ref_box[3]]]), z_obj)
    (sx1, sy1), (sx2, sy2) = src
    far = map_box(M, (sx1, sy1, sx2, sy2), 10.0)
    assert far[2] < ref_box[0] + 5          # at 10 m the box misses the object
    res = range_box(M, depth, (sx1 + sx2) / 2, (sy1 + sy2) / 2, sx2 - sx1, sy2 - sy1,
                    z_guess=2.0)
    assert res is not None and res.z == pytest.approx(z_obj, abs=1e-4)


def test_range_box_no_depth():
    assert range_box(M, np.zeros((360, 640), np.float32), 320, 240, 100, 100) is None


def test_map_box_ordered():
    x1, y1, x2, y2 = map_box(M, (100, 100, 200, 200), 2.0)
    assert x1 < x2 and y1 < y2


def test_range_variance_grows_with_range():
    assert range_variance(4.0, 0.0) > range_variance(1.0, 0.0)
    assert range_variance(1.0, 0.2) == pytest.approx((1.4826 * 0.2) ** 2)


def test_chain_to_base_link_matches_urdf():
    # URDF: stereo_camera_optical at xyz (0.466 0 0.215) rpy (-pi/2 0 -pi/2) in base_link.
    # quaternion of rpy(-pi/2, 0, -pi/2) = (-0.5, 0.5, -0.5, 0.5)
    r = quat_to_matrix(-0.5, 0.5, -0.5, 0.5)
    static = {'stereo_camera_optical': ('base_link', r, np.array([0.466, 0.0, 0.215]))}
    rot, t = chain_to(static, 'stereo_camera_optical', 'base_link')
    p = rot @ np.array([0.0, 0.0, 2.0]) + t        # 2 m along the optical axis
    np.testing.assert_allclose(p, [2.466, 0.0, 0.215], atol=1e-9)
    p = rot @ np.array([1.0, 0.0, 0.0]) + t        # optical +x (right) -> base -y
    np.testing.assert_allclose(p, [0.466, -1.0, 0.215], atol=1e-9)
    assert chain_to(static, 'nope', 'base_link') is None
