# Copyright 2026 The mower_docking authors
# SPDX-License-Identifier: Apache-2.0
"""Synthetic-image ArUco test (skipped when cv2.aruco is unavailable)."""

import math

import pytest

cv2 = pytest.importorskip('cv2')
np = pytest.importorskip('numpy')
if not hasattr(cv2, 'aruco'):
    pytest.skip('cv2.aruco not available', allow_module_level=True)

from mower_docking import aruco_detect  # noqa: E402
from mower_docking import dock_logic as dl  # noqa: E402

K = [800.0, 0.0, 640.0, 0.0, 800.0, 360.0, 0.0, 0.0, 1.0]
D = [0.0, 0.0, 0.0, 0.0, 0.0]


def render(marker_size, rvec, tvec, marker_id=7, w=1280, h=720):
    side = 400
    border = side // 6  # white quiet zone
    mk = aruco_detect.generate_marker_image('DICT_4X4_50', marker_id, side)
    canvas = np.full((side + 2 * border, side + 2 * border), 255, np.uint8)
    canvas[border:border + side, border:border + side] = mk
    s = marker_size / 2.0
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float64)
    img_pts, _ = cv2.projectPoints(obj, np.array(rvec, float), np.array(tvec, float),
                                   np.array(K).reshape(3, 3), np.array(D))
    src = np.array([[border, border], [border + side, border],
                    [border + side, border + side], [border, border + side]], np.float32)
    H = cv2.getPerspectiveTransform(src, img_pts.reshape(4, 2).astype(np.float32))
    # warp a white-padded canvas so the quiet zone is preserved
    out = cv2.warpPerspective(canvas, H, (w, h), borderValue=128)
    return out


def test_dictionary_and_size_defaults_match_vendor():
    det = aruco_detect.ArucoDetector()
    assert det.marker_size == pytest.approx(0.04)


def test_detect_and_pose_roundtrip():
    rvec = [math.pi, 0.0, 0.0]          # marker facing the camera
    tvec = [0.05, -0.02, 0.6]
    img = render(0.04, rvec, tvec)
    det = aruco_detect.ArucoDetector('DICT_4X4_50', 0.04)
    res = det.detect(img, K, D)
    assert len(res) == 1
    mid, rv, tv, _ = res[0]
    assert mid == 7
    assert tv == pytest.approx(tvec, abs=0.01)
    obs = dl.marker_in_base(rv, tv, (-0.201, 0, 0.25), (-math.pi / 2, 0, math.pi / 2))
    assert obs.x == pytest.approx(-0.801, abs=0.01)
    assert obs.y == pytest.approx(0.05, abs=0.01)
    assert abs(obs.yaw) < math.radians(5)


def test_marker_id_filter():
    img = render(0.04, [math.pi, 0, 0], [0, 0, 0.5], marker_id=3)
    det = aruco_detect.ArucoDetector('DICT_4X4_50', 0.04)
    assert det.detect(img, K, D, marker_id=5) == []
    assert det.detect(img, K, D, marker_id=3)[0][0] == 3


def test_to_gray_encodings():
    bgr = np.zeros((4, 6, 3), np.uint8)
    bgr[..., 2] = 255
    g = aruco_detect.to_gray('bgr8', bgr.tobytes(), 4, 6, 18)
    assert g.shape == (4, 6)
    mono = aruco_detect.to_gray('mono8', bytes(range(24)), 4, 6, 6)
    assert mono[1, 0] == 6
    assert aruco_detect.to_gray('weird', b'', 1, 1, 1) is None
