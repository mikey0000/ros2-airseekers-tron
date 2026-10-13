# SPDX-License-Identifier: GPL-3.0-or-later
"""Class-size plausibility of stereo ranges (2026-10-13 09:45 false stop)."""
import pytest

from det_range.geometry import (StereoModel, box_clipped, check_class_size, size_range)

M = StereoModel()
IH = 480.0          # right eye 640x480
PERSON = 1.6
VAR = 0.01


def _box(h_frac, top_px):
    h = h_frac * IH
    return 320.0, top_px + 0.5 * h, h          # cx, cy, h


def test_size_range_pinhole_at_the_centre():
    # small box around the principal point: ~ fy * H / h_px
    cx, cy = M.src_k[2], M.src_k[3]
    zs = size_range(M, cx, cy, 100.0, PERSON)
    assert zs == pytest.approx(M.src_k[1] * PERSON / 100.0, rel=0.03)
    assert size_range(M, cx, cy, 0.0, PERSON) is None
    assert size_range(M, cx, cy, 100.0, 0.0) is None


def test_clipped_detection():
    assert box_clipped(*_box(0.74, 2.0)[1:], IH)               # touches the top
    assert box_clipped(IH - 50.0, 100.0, IH)                    # touches the bottom
    assert not box_clipped(*_box(0.74, 40.0)[1:], IH)


def test_regression_0945_unclipped_74pct_at_091m_is_rejected():
    # 09:45: 0.91 m / bbox_h_frac 0.74 with headroom -> a person there would be ~1.9 m away
    cx, cy, h = _box(0.74, 40.0)
    sc = check_class_size(M, 0.91, VAR, cx, cy, h, IH, PERSON)
    assert sc.verdict == 'implausible'
    assert sc.z == pytest.approx(sc.z_size) and sc.z > 1.5
    assert sc.variance == pytest.approx((0.3 * sc.z) ** 2)


def test_regression_0945_first_frame_63pct_at_097m_is_rejected():
    cx, cy, h = _box(0.63, 60.0)
    sc = check_class_size(M, 0.97, VAR, cx, cy, h, IH, PERSON)
    assert sc.verdict == 'implausible' and sc.z > 2.0


def test_genuine_close_person_clipped_keeps_stereo():
    # 09:45:16-21 genuine passes: 96 % of the frame, touching the top, 0.35 m
    cx, cy, h = _box(0.96, 0.0)
    sc = check_class_size(M, 0.35, VAR, cx, cy, h, IH, PERSON)
    assert sc.verdict == 'clipped' and sc.z == 0.35 and sc.variance == VAR


def test_tall_unclipped_box_keeps_stereo():
    cx, cy, h = _box(0.90, 10.0)          # 432 px, 10 px headroom, 38 px to the bottom
    assert not box_clipped(cy, h, IH)
    sc = check_class_size(M, 0.30, VAR, cx, cy, h, IH, PERSON)
    assert sc.verdict == 'tall' and sc.z == 0.30


def test_plausible_range_is_kept():
    cx, cy, h = _box(0.40, 100.0)
    zs = size_range(M, cx, cy, h, PERSON)
    for z in (0.55 * zs, zs, 3.0 * zs):    # nearer than the prior within the ratio, or far
        sc = check_class_size(M, z, VAR, cx, cy, h, IH, PERSON)
        assert sc.verdict == 'ok' and sc.z == z and sc.variance == VAR


def test_far_stereo_is_never_overruled():
    # the check only ever moves a range AWAY from the robot when stereo is implausibly near;
    # a stereo range farther than the prior (short subject, e.g. a child) is kept as is
    cx, cy, h = _box(0.30, 150.0)
    sc = check_class_size(M, 9.0, VAR, cx, cy, h, IH, PERSON)
    assert sc.verdict == 'ok' and sc.z == 9.0


def test_short_subject_close_and_clipped_keeps_stereo():
    # a 1.0 m child at 0.7 m: its head leaves the top of the frame -> clipped -> stereo kept
    cx, cy, h = _box(0.80, 0.0)
    sc = check_class_size(M, 0.7, VAR, cx, cy, h, IH, PERSON)
    assert sc.verdict == 'clipped' and sc.z == 0.7


def test_ratio_boundary_and_off_switches():
    cx, cy, h = _box(0.50, 80.0)
    zs = size_range(M, cx, cy, h, PERSON)
    assert check_class_size(M, 0.5 * zs + 1e-6, VAR, cx, cy, h, IH, PERSON).verdict == 'ok'
    assert check_class_size(M, 0.5 * zs - 1e-3, VAR, cx, cy, h, IH,
                            PERSON).verdict == 'implausible'
    assert check_class_size(M, 0.3, VAR, cx, cy, h, IH, None).verdict == 'no_prior'
    assert check_class_size(M, 0.3, VAR, cx, cy, h, IH, PERSON,
                            min_ratio=0.0).verdict == 'no_prior'


def test_pet_prior_is_small():
    # a cat (0.25 m) at 0.8 m spans ~140 px: its stereo range is plausible
    cx, cy = 320.0, 300.0
    h = M.src_k[1] * 0.25 / 0.8
    assert check_class_size(M, 0.8, VAR, cx, cy, h, IH, 0.25).verdict == 'ok'
