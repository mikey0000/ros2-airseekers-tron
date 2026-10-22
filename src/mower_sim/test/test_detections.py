# SPDX-License-Identifier: Apache-2.0
"""Scripted ranged detections (mower_sim/detections.py, 2026-10-23 no-go zones)."""
import math

import numpy as np
import pytest

from mower_sim.detections import DetectionSource, DetParams, disc_base
from mower_sim.sensors import STEREO_XYZ
from mower_sim.world import world_from_dict

WORLD = {'lawn': [[0, 0], [10, 0], [10, 10], [0, 10]], 'dock': {'x': 1, 'y': 1},
         'obstacles': [{'name': 'trunk', 'type': 'cylinder', 'x': 5.0, 'y': 5.0,
                        'radius': 0.12, 'height': 1.5,
                        'detect': {'label': 'trunk', 'score': 0.75}, 'stereo': False},
                       {'name': 'box', 'type': 'box', 'x': 8.0, 'y': 8.0}]}


def test_world_parses_detect_and_stereo_flags():
    w = world_from_dict(WORLD)
    t, b = w.obstacles
    assert t.detect_label == 'trunk' and t.detect_score == 0.75 and t.stereo is False
    assert b.detect_label == '' and b.stereo is True


def test_trunk_ahead_is_reported_with_front_face_and_width():
    w = world_from_dict(WORLD)
    src = DetectionSource(DetParams())
    # robot 2 m short of the trunk, facing it
    dets = src.detect(3.0, 5.0 - STEREO_XYZ[1], 0.0, w.obstacles, 0.0)
    assert len(dets) == 1
    d = dets[0]
    assert d.label == 'trunk' and d.score == 0.75
    assert d.width == pytest.approx(0.24)
    # front face 0.12 m short of the centre, in base_link
    assert d.face_base[0] == pytest.approx(2.0 - 0.12, abs=1e-6)
    assert d.bbox_w_px == pytest.approx(0.24 * 372.0 / d.range)
    disc = disc_base(d, src.cam[:2])
    assert disc.radius == pytest.approx(0.22)
    assert disc.centre[0] == pytest.approx(2.0, abs=1e-6)     # back to the trunk centre


def test_beside_or_behind_or_far_is_not_reported():
    w = world_from_dict(WORLD)
    src = DetectionSource(DetParams())
    assert src.detect(5.0, 4.4, 0.0, w.obstacles, 0.0) == []          # beside (90 deg)
    assert src.detect(6.0, 5.0, 0.0, w.obstacles, 0.0) == []          # behind
    assert src.detect(0.0, 5.0, 0.0, w.obstacles, 0.0) == []          # > 4 m
    assert len(src.detect(4.0, 5.0, math.radians(15), w.obstacles, 0.0)) == 1
