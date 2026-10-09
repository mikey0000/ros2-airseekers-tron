# SPDX-License-Identifier: Apache-2.0
"""ROS-free tests for mower_sim.kinematics (MCU drive behaviour)."""

import math

import pytest

from mower_sim import world as W
from mower_sim.kinematics import DiffDrive, DriveParams, wheel_speeds, wrap

DT = 0.02


def _world(obstacles=None, start=None):
    doc = {'lawn': [[-5, -5], [20, -5], [20, 5], [-5, 5]], 'obstacles': obstacles or []}
    if start:
        doc['start'] = start
        doc['dock'] = {'x': -50.0, 'y': 0.0, 'yaw': 0.0}   # keep the dock stop out of the way
    return W.world_from_dict(doc)


def _run(d, v, w, seconds, t0=0.0, recommand=True):
    """Step the drive for ``seconds``, re-sending the command every step."""
    t = t0
    for _ in range(int(round(seconds / DT))):
        if recommand:
            d.command(v, w)
        d.step(DT, t)
        t += DT
    return t


# ---------------------------------------------------------------- wheel_speeds

def test_wheel_speeds_clamp_linear():
    assert wheel_speeds(1.0, 0.0, DriveParams()) == pytest.approx((0.3, 0.3))
    assert wheel_speeds(-1.0, 0.0, DriveParams()) == pytest.approx((-0.3, -0.3))


def test_wheel_speeds_clamp_angular():
    left, right = wheel_speeds(0.0, 1.0, DriveParams())
    assert left == pytest.approx(-0.072)
    assert right == pytest.approx(0.072)


def test_wheel_speeds_clamp_is_separate():
    left, right = wheel_speeds(1.0, 1.0, DriveParams())
    assert left == pytest.approx(0.3 - 0.072)
    assert right == pytest.approx(0.3 + 0.072)


def test_wheel_floor():
    assert wheel_speeds(0.0, 0.2, DriveParams()) == (0.0, 0.0)   # 0.048 < 0.06
    left, right = wheel_speeds(0.0, 0.2, DriveParams(min_wheel_speed=0.0))
    assert left == pytest.approx(-0.048) and right == pytest.approx(0.048)


def test_wheel_floor_is_per_wheel():
    """Floor applies per wheel while turning; straight crawl (dock reverse 0.05) passes
    unless floor_straight."""
    left, right = wheel_speeds(0.06, 0.0, DriveParams())
    assert left == pytest.approx(0.06) and right == pytest.approx(0.06)
    assert wheel_speeds(-0.05, 0.0, DriveParams()) == pytest.approx((-0.05, -0.05))
    assert wheel_speeds(0.05, 0.0, DriveParams(floor_straight=True)) == (0.0, 0.0)
    # arc: inner wheel 0.1 - 0.24 * 0.2 = 0.052 < floor -> 0, outer keeps 0.148
    left, right = wheel_speeds(0.1, 0.2, DriveParams())
    assert left == 0.0 and right == pytest.approx(0.148)


def test_wrap():
    assert wrap(3 * math.pi) == pytest.approx(math.pi, abs=1e-9) or \
        wrap(3 * math.pi) == pytest.approx(-math.pi, abs=1e-9)
    assert wrap(0.5) == pytest.approx(0.5)


# ---------------------------------------------------------------- free driving

def test_straight_drive():
    d = DiffDrive(_world(start={'x': 0.0, 'y': 0.0, 'yaw': 0.0}))
    _run(d, 0.3, 0.0, 5.0)
    assert 1.4 < d.pose.x < 1.5
    assert d.pose.y == pytest.approx(0.0, abs=1e-9)
    assert d.pose.yaw == pytest.approx(0.0, abs=1e-9)


def test_turn_in_place_above_floor():
    d = DiffDrive(_world(start={'x': 0.0, 'y': 0.0, 'yaw': 0.0}))
    _run(d, 0.0, 0.3, 4.0)
    assert d.pose.yaw > 0.5
    assert math.hypot(d.pose.x, d.pose.y) < 1e-6


def test_slow_turn_below_floor_does_not_move():
    d = DiffDrive(_world(start={'x': 0.0, 'y': 0.0, 'yaw': 0.0}))
    _run(d, 0.0, 0.2, 3.0)
    assert d.pose.yaw == 0.0 and d.pose.x == 0.0


def test_command_timeout_stops_robot():
    d = DiffDrive(_world(start={'x': 0.0, 'y': 0.0, 'yaw': 0.0}))
    d.command(0.3, 0.0)
    t = 0.0
    for _ in range(int(0.45 / DT)):
        d.step(DT, t)
        t += DT
    assert d.v > 0.1                       # still driving inside the 0.5 s window
    for _ in range(int(2.5 / DT)):
        d.step(DT, t)
        t += DT
    assert d.v == 0.0 and d.w == 0.0
    x = d.pose.x
    assert 0.1 < x < 0.5                   # ~0.5 s of driving plus a short coast
    d.step(DT, t)
    assert d.pose.x == x


def test_disabled_zeros_motion():
    d = DiffDrive(_world(start={'x': 0.0, 'y': 0.0, 'yaw': 0.0}))
    d.enabled = False
    _run(d, 0.3, 0.0, 2.0)
    assert d.pose.x == 0.0 and d.v == 0.0


def test_disable_mid_drive_stops():
    d = DiffDrive(_world(start={'x': 0.0, 'y': 0.0, 'yaw': 0.0}))
    t = _run(d, 0.3, 0.0, 2.0)
    d.enabled = False
    _run(d, 0.3, 0.0, 3.0, t0=t)
    assert d.v == 0.0
    x = d.pose.x
    _run(d, 0.3, 0.0, 1.0, t0=t + 3.0)
    assert d.pose.x == x


# ---------------------------------------------------------------- collisions

BOX = [{'name': 'b', 'type': 'box', 'x': 2.0, 'y': 0.0, 'size_x': 1.0, 'size_y': 1.0,
        'height': 0.5}]


def test_collision_blocks_robot_at_box():
    w = _world(BOX, start={'x': 0.0, 'y': 0.0, 'yaw': 0.0})
    d = DiffDrive(w)
    t = _run(d, 0.3, 0.0, 8.0)
    clr = w.clearance(d.pose.x, d.pose.y, d.pose.yaw, t)
    assert 0.0 <= clr < 0.02
    assert not w.collisions(d.pose.x, d.pose.y, d.pose.yaw, t)
    assert d.blocked
    assert d.collisions_total >= 1
    x = d.pose.x
    n = d.collisions_total
    _run(d, 0.3, 0.0, 5.0, t0=t)           # keep pushing: no more motion, no more counts
    assert d.pose.x == pytest.approx(x, abs=1e-9)
    assert d.collisions_total == n
    assert d.pose.x < 1.0


def test_collision_counted_once_per_contact():
    """Creeping along the obstacle after a stall (wheel lag restarts from 0) is still the
    same contact: it ends only once the footprint is > 2 cm clear."""
    w = _world(BOX, start={'x': 0.0, 'y': 0.0, 'yaw': 0.0})
    d = DiffDrive(w)
    _run(d, 0.3, 0.0, 13.0)
    assert d.collisions_total == 1


def test_step_result_reports_contact():
    w = _world(BOX, start={'x': 0.0, 'y': 0.0, 'yaw': 0.0})
    d = DiffDrive(w)
    t = 0.0
    res = None
    for _ in range(int(8.0 / DT)):
        d.command(0.3, 0.0)
        res = d.step(DT, t)
        t += DT
    assert res.blocked and [o.name for o in res.contacts] == ['b']


def test_backing_away_from_box_allowed():
    w = _world(BOX, start={'x': 0.0, 'y': 0.0, 'yaw': 0.0})
    d = DiffDrive(w)
    t = _run(d, 0.3, 0.0, 8.0)
    x_stuck = d.pose.x
    _run(d, -0.3, 0.0, 3.0, t0=t)
    assert d.pose.x < x_stuck - 0.5
    assert not d.blocked


def test_moving_obstacle_hitting_robot_can_be_escaped():
    w = _world([{'type': 'cylinder', 'radius': 0.25, 'speed': 1.0, 'mode': 'once',
                 'path': [[3.0, 0.0], [0.7, 0.0]]}],
               start={'x': 0.0, 'y': 0.0, 'yaw': 0.0})
    d = DiffDrive(w)
    t = _run(d, -0.3, 0.0, 6.0)
    assert d.pose.x < -0.5
    assert w.clearance(d.pose.x, d.pose.y, d.pose.yaw, t) > 0.0


# ---------------------------------------------------------------- dock

def test_dock_start_on_dock():
    d = DiffDrive(_world())
    assert d.world.start_docked
    assert d.on_dock()
    assert d.dock_frame() == pytest.approx((0.0, 0.0, 0.0))


def test_dock_leave_and_return_hard_stop():
    d = DiffDrive(_world())
    t = 0.0
    while d.dock_frame()[0] < 0.5:
        d.command(0.3, 0.0)
        d.step(DT, t)
        t += DT
        assert t < 20.0
    assert not d.on_dock()
    # let it come to rest, then reverse far longer than needed
    for _ in range(int(2.0 / DT)):
        d.step(DT, t)
        t += DT
    min_xd = math.inf
    for _ in range(int(10.0 / DT)):
        d.command(-0.3, 0.0)
        d.step(DT, t)
        t += DT
        min_xd = min(min_xd, d.dock_frame()[0])
    assert min_xd >= 0.0
    assert d.dock_frame()[0] < 0.03
    assert d.on_dock()


def test_dock_stop_flag_reported():
    d = DiffDrive(_world())
    flagged = False
    t = 0.0
    for _ in range(int(2.0 / DT)):
        d.command(-0.3, 0.0)
        flagged |= d.step(DT, t).dock_stop
        t += DT
    assert flagged
    assert d.pose.x >= 0.0
    assert d.on_dock()


def test_dock_hard_stop_respects_dock_yaw():
    w = W.world_from_dict({'lawn': [[-5, -5], [5, -5], [5, 5], [-5, 5]],
                           'dock': {'x': 1.0, 'y': 1.0, 'yaw': math.pi / 2}})
    d = DiffDrive(w)
    assert d.on_dock()
    t = 0.0
    for _ in range(int(2.0 / DT)):
        d.command(-0.3, 0.0)
        d.step(DT, t)
        t += DT
    assert d.dock_frame()[0] >= 0.0
    assert d.pose.y == pytest.approx(1.0, abs=0.03)
