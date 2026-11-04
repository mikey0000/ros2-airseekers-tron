# SPDX-License-Identifier: Apache-2.0
"""Tests for mower_sim.marker (rear camera ArUco rendering); need cv2 with aruco."""

import math

import pytest

cv2 = pytest.importorskip('cv2')
if not hasattr(cv2, 'aruco'):
    pytest.skip('cv2 has no aruco module', allow_module_level=True)

from mower_sim.marker import (REAR_CAMERA_RPY, REAR_CAMERA_XYZ, RearCamera,  # noqa: E402
                              RearCameraParams)


def _detect_ids(img):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        _, ids, _ = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters()).detectMarkers(img)
    else:
        _, ids, _ = cv2.aruco.detectMarkers(img, d)
    return [] if ids is None else [int(i) for i in ids.reshape(-1)]


def test_render_from_front_detects_marker_id_0():
    cam = RearCamera(RearCameraParams(marker_size=0.3))
    img = cam.render(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert img.shape == (480, 640) and img.dtype.name == 'uint8'
    assert _detect_ids(img) == [0]


def test_render_default_marker_size_detected_at_close_range():
    cam = RearCamera()
    img = cam.render(0.7, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert _detect_ids(img) == [0]


def test_render_camera_looking_away_sees_nothing():
    cam = RearCamera(RearCameraParams(marker_size=0.3))
    img = cam.render(1.0, 0.0, 3.141592653589793, 0.0, 0.0, 0.0)
    assert _detect_ids(img) == []
    assert (img == cam.p.background).all()


def test_render_marker_beyond_dock_side_blank():
    """Robot behind the marker plane (marker faces +x, robot at x < -offset) sees its back."""
    cam = RearCamera(RearCameraParams(marker_size=0.3))
    img = cam.render(-2.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert _detect_ids(img) == []


def test_pinhole_matrix():
    """config/cameras/rear_camera_info_640x480.yaml intrinsics."""
    p = RearCameraParams()
    K = RearCamera().K
    assert K[0] == p.fx and K[4] == p.fy and K[2] == p.cx and K[5] == p.cy
    assert (p.width, p.height) == (640, 480)


def test_marker_pixels_move_with_lateral_offset():
    cam = RearCamera(RearCameraParams(marker_size=0.3))
    c0 = cam.project(cam.marker_corners_map(0.0, 0.0, 0.0, 0.15), 1.5, 0.0, 0.0)
    c1 = cam.project(cam.marker_corners_map(0.0, 0.0, 0.0, 0.15), 1.5, -0.3, 0.0)
    cx = cam.p.cx
    assert abs(sum(c[0] for c in c0) / 4 - cx) < 1.0
    assert abs(sum(c[0] for c in c1) / 4 - cx) > 20.0


def test_docking_pipeline_recovers_pose_errors():
    pytest.importorskip('mower_docking')
    from mower_docking.aruco_detect import ArucoDetector
    from mower_docking.dock_logic import dock_errors, marker_in_base
    p = RearCameraParams(marker_size=0.3)
    cam = RearCamera(p)
    # 640x480: PnP yaw of a 0.3 m marker at 1.5 m is good to ~2 deg, which the 0.45 m
    # docked offset turns into a few cm of lateral error
    img = cam.render(1.5, -0.2, -0.2, 0.0, 0.0, 0.0)
    det = ArucoDetector('DICT_4X4_50', p.marker_size).detect(img, cam.K, [0.0] * 5)
    assert det, 'marker not detected'
    mid, rvec, tvec, _ = det[0]
    assert mid == 0
    obs = marker_in_base(rvec, tvec, REAR_CAMERA_XYZ, REAR_CAMERA_RPY)
    err = dock_errors(obs, p.marker_offset)
    assert err.remaining == pytest.approx(1.5, abs=0.03)
    assert err.lateral == pytest.approx(-0.2, abs=0.07)
    assert err.heading == pytest.approx(-0.2, abs=0.05)



def test_real_marker_read_with_the_deployed_config_reads_its_docked_offset():
    """Sim default (2026-10-16): the real 0.055 m marker at 0.543 m, read by mower_docking
    with its deployed marker_size 0.04 and docked_marker_offset 0.45: at the docked pose the
    remaining distance reads ~0 (the deployed config docks in the field)."""
    import os
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'mower_docking'))
    from mower_docking.aruco_detect import ArucoDetector
    from mower_docking import dock_logic as dl
    cam = RearCamera(RearCameraParams(marker_size=0.055, marker_offset=0.543))
    det = ArucoDetector('DICT_4X4_50', 0.04)
    rem = []
    for d in (0.30, 0.50):
        img = cam.render(d, 0.0, 0.0, 0.0, 0.0, 0.0)
        res = det.detect(img, cam.K, [0.0] * 5)
        assert res
        _mid, rvec, tvec, _c = res[0]
        m = dl.marker_in_base(rvec, tvec, REAR_CAMERA_XYZ, REAR_CAMERA_RPY)
        rem.append(dl.dock_errors(m, 0.45).remaining)
    # under-read by 0.04 / 0.055 in the camera frame, zero at the docked pose
    assert rem[0] == pytest.approx(0.30 * 0.04 / 0.055, abs=0.03)
    assert rem[1] == pytest.approx(0.50 * 0.04 / 0.055, abs=0.04)


def _yaw_errors(cam, x, y, yaw, n):
    import os
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'mower_docking'))
    from mower_docking.aruco_detect import ArucoDetector
    from mower_docking import dock_logic as dl
    det = ArucoDetector('DICT_4X4_50', 0.04)
    errs = []
    for _ in range(n):
        img = cam.render(x, y, yaw, 0.0, 0.0, 0.0)
        res = det.detect(img, cam.K, [0.0] * 5)
        if res:
            m = dl.marker_in_base(res[0][1], res[0][2], REAR_CAMERA_XYZ, REAR_CAMERA_RPY)
            errs.append(math.degrees(dl.wrap_angle(-m.yaw - yaw)))
    return errs


def test_dusk_scene_is_dark_detected_and_swings_the_marker_yaw():
    """dock_dusk (2026-10-31 night dock): brightness ~10, detection 100 %, yaw +-15-20 deg."""
    clean = RearCamera(RearCameraParams(marker_size=0.055, marker_offset=0.543, dusk=True))
    img = clean.render(1.0, 0.05, 0.05, 0.0, 0.0, 0.0)
    assert img.mean() < 15.0
    e0 = _yaw_errors(clean, 1.0, 0.05, 0.05, 5)
    assert len(e0) == 5 and max(e0) - min(e0) < 1.0       # no jitter: the same solve each frame
    dusk = RearCamera(RearCameraParams(marker_size=0.055, marker_offset=0.543, dusk=True,
                                       corner_noise_px=0.5, seed=3))
    errs = _yaw_errors(dusk, 1.0, 0.05, 0.05, 40)
    assert len(errs) == 40
    assert max(errs) - min(errs) > 25.0
