# Copyright 2026 The mower_docking authors
# SPDX-License-Identifier: Apache-2.0
"""marker_check statistics + the 0.8 m rear-camera scenario at 640x480."""

import math

import pytest

from mower_docking import dock_logic as dl
from mower_docking import marker_check as mc

REAR_XYZ = (-0.201, 0.0, 0.25)
REAR_RPY = (-math.pi / 2, 0.0, math.pi / 2)


def _s(t, x=None, y=0.0, yaw=0.0, ms=5.0):
    return mc.Sample(t, None if x is None else dl.MarkerObs(x, y, yaw, t, 3), ms)


def test_summarize_empty():
    s = mc.summarize([_s(0.0), _s(0.1)], 1.0, 0.45, 0.5, 25.0)
    assert s.detections == 0 and s.frames == 2 and s.gate_ok is None
    assert 'NO MARKER' in mc.format_summary(s)


def test_summarize_marker_behind_passes_gate():
    samples = [_s(0.1 * i, x=-0.8, y=0.02, yaw=math.radians(3)) for i in range(10)]
    samples.append(_s(1.0))
    s = mc.summarize(samples, 1.1, 0.45, 0.5, 25.0)
    assert s.detections == 10 and s.detect_rate_hz == pytest.approx(10 / 1.1)
    assert s.x == pytest.approx(-0.8) and s.gate_ok and s.gate_pass_ratio == 1.0
    assert s.remaining == pytest.approx(0.8 - 0.45, abs=0.02)
    assert s.marker_ids == [3]


def test_summarize_gate_rejects_large_yaw():
    s = mc.summarize([_s(0.0, x=-0.8, yaw=math.radians(40))], 1.0, 0.45, 0.5, 25.0)
    assert s.gate_ok is False


cv2 = pytest.importorskip('cv2')
np = pytest.importorskip('numpy')


@pytest.mark.skipif(not hasattr(cv2, 'aruco'), reason='cv2.aruco not available')
@pytest.mark.parametrize('fx', [
    # full-width rescale of the 1080p calibration: the marker is ~26 px wide, IPPE flips the
    # yaw by 180 deg and range is ~6 % short -> the gate rejects it. Documented limit.
    pytest.param(513.5, marks=pytest.mark.xfail(reason='4 cm marker ~26 px at 0.8 m: '
                                                'pose ambiguity', strict=False)),
    684.0,          # 4:3 centre crop of the 1080p sensor (~35 px)
])
@pytest.mark.parametrize('lat', [0.0, 0.15])
def test_marker_08m_behind_rear_camera_640x480(fx, lat):
    """4 cm DICT_4X4_50 marker 0.8 m behind base_link, facing the robot, seen by the rear
    camera at 640x480: detected, pose in base_link within 3 cm / 12 deg, gate passes."""
    from mower_docking import aruco_detect
    from test_aruco_detect import render
    K = [fx, 0.0, 320.0, 0.0, fx, 240.0, 0.0, 0.0, 1.0]
    D = [0.0] * 5
    # marker centre in base_link (x=-0.8, y=lat, z=0.25 camera height) -> camera optical
    # frame: z = depth = 0.8 - 0.201, x = right = +y_base (looking backwards), y = down.
    tvec = [lat, 0.0, 0.8 - 0.201]
    rvec = [math.pi, 0.0, 0.0]   # marker +Z towards the camera
    import test_aruco_detect as tad
    old = tad.K
    tad.K = K
    try:
        img = render(0.04, rvec, tvec, marker_id=0, w=640, h=480)
    finally:
        tad.K = old
    det = aruco_detect.ArucoDetector('DICT_4X4_50', 0.04)
    res = det.detect(img, K, D)
    assert res, 'marker not detected at 0.8 m'
    mid, rv, tv, _ = res[0]
    obs = dl.marker_in_base(rv, tv, REAR_XYZ, REAR_RPY)
    assert obs.x == pytest.approx(-0.8, abs=0.03)
    assert obs.y == pytest.approx(lat, abs=0.03)
    # single 4 cm marker at ~27-36 px: yaw from IPPE is the noisy part (~7 deg seen)
    assert abs(math.degrees(obs.yaw)) < 12.0
    err = dl.dock_errors(obs, 0.45)
    assert dl.marker_gate_ok(err, 0.5, math.radians(25.0))
