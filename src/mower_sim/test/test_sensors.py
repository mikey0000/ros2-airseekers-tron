# SPDX-License-Identifier: Apache-2.0
"""ROS-free tests for mower_sim.sensors (stereo raycaster, bumper sides)."""

import math

import numpy as np
import pytest

from mower_sim.sensors import StereoRaycaster, bumper_sides
from mower_sim.world import Obstacle


def _box(x, y, sx=0.6, sy=1.0, h=0.5):
    return Obstacle(name='b', type='box', x=x, y=y, size_x=sx, size_y=sy, height=h)


def _base(rc, pts):
    """Optical-frame points -> base_link."""
    return pts @ rc.r_cam.T + rc.t_cam


def test_no_obstacles_only_ground():
    rc = StereoRaycaster()
    obs, ground = rc.cast(0.0, 0.0, 0.0, [], 0.0)
    assert len(obs) == 0
    assert len(ground) > 0
    gb = _base(rc, ground.astype(float))
    assert np.all(np.abs(gb[:, 2]) < 1e-3)
    assert np.all(np.linalg.norm(ground, axis=1) <= 3.0 + 1e-4)
    assert obs.dtype == np.float32 and ground.dtype == np.float32


def test_ground_one_point_per_cell():
    rc = StereoRaycaster()
    _, ground = rc.cast(0.0, 0.0, 0.0, [], 0.0)
    cells = np.floor(_base(rc, ground.astype(float))[:, :2] / 0.10)
    assert len(np.unique(cells, axis=0)) == len(ground)


def test_box_ahead_gives_obstacle_points():
    rc = StereoRaycaster()
    obs, ground = rc.cast(0.0, 0.0, 0.0, [_box(2.5, 0.0)], 0.0)
    assert len(obs) > 10
    ob = _base(rc, obs.astype(float))
    assert ob[:, 0].min() == pytest.approx(2.2, abs=0.02)
    assert ob[:, 0].max() <= 2.8 + 0.02
    assert np.all(ob[:, 2] >= -1e-3) and np.all(ob[:, 2] <= 0.5 + 1e-3)
    assert np.all(np.abs(ob[:, 1]) <= 0.5 + 1e-3)
    assert len(ground) > 0


def test_obstacle_points_are_in_optical_frame_and_range():
    rc = StereoRaycaster()
    obs, _ = rc.cast(0.0, 0.0, 0.0, [_box(2.5, 0.0)], 0.0)
    assert np.all(obs[:, 2] > 0.0)                      # z forward
    rng = np.linalg.norm(obs, axis=1)
    assert rng.min() >= 0.2 and rng.max() <= 4.0


def test_box_beyond_range_not_seen():
    rc = StereoRaycaster()
    obs, _ = rc.cast(0.0, 0.0, 0.0, [_box(5.5, 0.0)], 0.0)   # face at 5.2 m
    assert len(obs) == 0


def test_box_behind_not_seen():
    rc = StereoRaycaster()
    obs, ground = rc.cast(0.0, 0.0, 0.0, [_box(-2.5, 0.0)], 0.0)
    assert len(obs) == 0
    assert len(ground) > 0


def test_low_box_gives_fewer_low_points():
    rc = StereoRaycaster()
    tall, _ = rc.cast(0.0, 0.0, 0.0, [_box(2.5, 0.0, h=0.5)], 0.0)
    low, _ = rc.cast(0.0, 0.0, 0.0, [_box(2.5, 0.0, h=0.05)], 0.0)
    assert 0 < len(low) < len(tall)
    assert np.all(_base(rc, low.astype(float))[:, 2] <= 0.05 + 1e-3)


def test_obstacle_hides_ground_behind_it():
    rc = StereoRaycaster()
    _, free = rc.cast(0.0, 0.0, 0.0, [], 0.0)
    _, blocked = rc.cast(0.0, 0.0, 0.0, [_box(1.5, 0.0, h=2.0)], 0.0)
    assert len(blocked) < len(free)
    gb = _base(rc, blocked.astype(float))
    shadow = np.abs(gb[:, 1]) < 0.3        # directly behind the box face (x = 1.2)
    assert np.all(gb[shadow, 0] < 1.2 + 0.05)


def test_rotated_robot_sees_box_at_plus_y():
    rc = StereoRaycaster()
    obs, _ = rc.cast(0.0, 0.0, math.pi / 2, [_box(0.0, 2.5, sx=1.0, sy=0.6)], 0.0)
    assert len(obs) > 10
    ob = _base(rc, obs.astype(float))      # base frame: forward is world +y
    assert ob[:, 0].min() == pytest.approx(2.2, abs=0.02)
    # and not when the robot faces +x
    none, _ = rc.cast(0.0, 0.0, 0.0, [_box(0.0, 2.5, sx=1.0, sy=0.6)], 0.0)
    assert len(none) == 0


def test_cylinder_ahead_seen():
    rc = StereoRaycaster()
    cyl = Obstacle(name='c', type='cylinder', x=2.5, y=0.0, radius=0.3, height=1.0)
    obs, _ = rc.cast(0.0, 0.0, 0.0, [cyl], 0.0)
    assert len(obs) > 5
    assert _base(rc, obs.astype(float))[:, 0].min() == pytest.approx(2.2, abs=0.02)


def test_to_optical_roundtrip():
    rc = StereoRaycaster()
    p = np.array([[2.0, 0.3, 0.1]])
    back = rc.to_optical(p) @ rc.r_cam.T + rc.t_cam
    assert back == pytest.approx(p)


# ---------------------------------------------------------------- bumper

def _bump(ox, oy):
    o = Obstacle(name='o', type='cylinder', x=ox, y=oy)
    return bumper_sides([o], 0.0, 0.0, 0.0, 0.0)


def test_bumper_ahead_left():
    assert _bump(0.8, 0.3) == (True, False)


def test_bumper_ahead_right():
    assert _bump(0.8, -0.3) == (False, True)


def test_bumper_centre_both():
    assert _bump(0.8, 0.0) == (True, True)
    assert _bump(0.8, 0.05) == (True, True)


def test_bumper_behind_none():
    assert _bump(-0.8, 0.3) == (False, False)
    assert _bump(-0.8, 0.0) == (False, False)


def test_bumper_no_contacts_and_rotated_pose():
    assert bumper_sides([], 0.0, 0.0, 0.0, 0.0) == (False, False)
    o = Obstacle(name='o', type='cylinder', x=0.0, y=0.8)
    # robot facing +y: obstacle ahead, centred
    assert bumper_sides([o], 0.0, 0.0, math.pi / 2, 0.0) == (True, True)
    # robot facing +x: obstacle on its left, not ahead of front_x
    assert bumper_sides([o], 0.0, 0.0, 0.0, 0.0) == (False, False)
