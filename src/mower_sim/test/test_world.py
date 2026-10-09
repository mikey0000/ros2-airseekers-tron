# SPDX-License-Identifier: Apache-2.0
"""ROS-free tests for mower_sim.world (run: python3 -m pytest src/mower_sim/test)."""

import math
import os

import pytest

from mower_sim import world as W

WORLDS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'worlds'))
LAWN = [[0, 0], [10, 0], [10, 10], [0, 10]]


def _world(**kw):
    doc = {'lawn': LAWN}
    doc.update(kw)
    return W.world_from_dict(doc)


def _square(cx, cy, h=0.5):
    return [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)]


# ---------------------------------------------------------------- parsing

def test_default_start_is_docked_at_dock_pose():
    w = _world(dock={'x': 1.0, 'y': 2.0, 'yaw': 0.5})
    assert w.start_docked
    assert (w.start.x, w.start.y, w.start.yaw) == (1.0, 2.0, 0.5)


def test_explicit_start_pose():
    w = _world(dock={'x': 1.0, 'y': 2.0, 'yaw': 0.5}, start={'x': 4.0, 'y': 5.0, 'yaw': -1.0})
    assert not w.start_docked
    assert (w.start.x, w.start.y, w.start.yaw) == (4.0, 5.0, -1.0)


def test_world_wrapper_key_and_name():
    w = W.world_from_dict({'world': {'lawn': LAWN, 'name': 'abc'}}, name='fallback')
    assert w.name == 'abc'
    assert W.world_from_dict({'lawn': LAWN}, name='fallback').name == 'fallback'


def test_unknown_obstacle_type_raises():
    with pytest.raises(ValueError):
        _world(obstacles=[{'type': 'sphere'}])


def test_short_lawn_raises():
    with pytest.raises(ValueError):
        W.world_from_dict({'lawn': [[0, 0], [1, 1]]})
    with pytest.raises(ValueError):
        W.world_from_dict({})


def test_obstacle_fields_parsed():
    w = _world(obstacles=[
        {'name': 'b', 'type': 'box', 'x': 3, 'y': 4, 'size_x': 1, 'size_y': 2, 'known': True},
        {'type': 'cylinder', 'radius': 0.3, 'path': [[1, 1], [2, 1]], 'speed': 1.0}])
    b, c = w.obstacles
    assert (b.x, b.y, b.size_x, b.size_y, b.known) == (3.0, 4.0, 1.0, 2.0, True)
    assert not b.moving
    assert c.name == 'cylinder_1'
    assert (c.x, c.y) == (1.0, 1.0)   # defaults to the first path point
    assert c.moving


@pytest.mark.parametrize('fname', ['garden.yaml', 'transit_box.yaml'])
def test_shipped_worlds_load(fname):
    w = W.load_world(os.path.join(WORLDS, fname))
    assert w.name == fname[:-5]
    assert len(w.lawn) >= 3
    assert w.obstacles


def test_shipped_world_contents():
    g = W.load_world(os.path.join(WORLDS, 'garden.yaml'))
    assert g.start_docked
    assert [o.name for o in g.obstacles if o.known] == ['flower_bed']
    assert [o.name for o in g.obstacles if o.moving] == ['person']
    t = W.load_world(os.path.join(WORLDS, 'transit_box.yaml'))
    assert not t.start_docked
    assert (t.start.x, t.start.y) == (1.5, 0.0)


# ---------------------------------------------------------------- geometry

def test_point_in_polygon():
    sq = _square(0, 0, 1.0)
    assert W.point_in_polygon((0.0, 0.0), sq)
    assert not W.point_in_polygon((1.5, 0.0), sq)
    assert not W.point_in_polygon((0.0, -2.0), sq)


def test_convex_overlap():
    a = _square(0, 0)
    assert W.convex_overlap(a, _square(0.5, 0.0))
    assert W.convex_overlap(a, _square(1.0, 0.0))        # touching counts
    assert not W.convex_overlap(a, _square(1.5, 0.0))


def test_polygon_distance():
    a = _square(0, 0)
    assert W.polygon_distance(a, _square(0.5, 0.0)) == 0.0
    assert W.polygon_distance(a, _square(2.0, 0.0)) == pytest.approx(1.0)
    assert W.polygon_distance(a, _square(2.0, 2.0)) == pytest.approx(math.sqrt(2.0))


def test_polygon_point_distance():
    sq = _square(0, 0, 1.0)
    assert W.polygon_point_distance(sq, (0.0, 0.0)) == 0.0
    assert W.polygon_point_distance(sq, (3.0, 0.0)) == pytest.approx(2.0)


def test_to_local_and_transform_roundtrip():
    p = W.transform([(1.0, 0.0)], 2.0, 3.0, math.pi / 2)[0]
    assert p == pytest.approx((2.0, 4.0))
    assert W.to_local(p, 2.0, 3.0, math.pi / 2) == pytest.approx((1.0, 0.0))


# ---------------------------------------------------------------- moving obstacles

def _mover(**kw):
    d = dict(name='m', type='cylinder', path=[(0.0, 0.0), (4.0, 0.0)], speed=1.0)
    d.update(kw)
    return W.Obstacle(**d)


def test_static_position():
    o = W.Obstacle(name='s', type='box', x=1.0, y=2.0)
    assert o.position(100.0) == (1.0, 2.0)


def test_pingpong_goes_out_and_back():
    o = _mover(mode='pingpong')
    assert o.position(0.0) == pytest.approx((0.0, 0.0))
    assert o.position(2.0) == pytest.approx((2.0, 0.0))
    assert o.position(4.0) == pytest.approx((4.0, 0.0))
    assert o.position(6.0) == pytest.approx((2.0, 0.0))
    assert o.position(8.0) == pytest.approx((0.0, 0.0))
    assert o.position(10.0) == pytest.approx((2.0, 0.0))


def test_loop_wraps():
    o = _mover(mode='loop', path=[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)])
    # perimeter 2 + 2 + hypot(2, 2) with the implicit closing segment back to start
    assert o.position(1.0) == pytest.approx((1.0, 0.0))
    assert o.position(3.0) == pytest.approx((2.0, 1.0))
    total = 4.0 + math.hypot(2.0, 2.0)
    assert o.position(total + 1.0) == pytest.approx((1.0, 0.0))


def test_once_stops_at_end():
    o = _mover(mode='once')
    assert o.position(3.0) == pytest.approx((3.0, 0.0))
    assert o.position(4.0) == pytest.approx((4.0, 0.0))
    assert o.position(50.0) == pytest.approx((4.0, 0.0))


def test_start_delay_holds_first_point():
    o = _mover(mode='once', start_delay=5.0)
    assert o.position(0.0) == pytest.approx((0.0, 0.0))
    assert o.position(5.0) == pytest.approx((0.0, 0.0))
    assert o.position(7.0) == pytest.approx((2.0, 0.0))


# ---------------------------------------------------------------- collisions / clearance

def _box_world(x, typ='box'):
    o = {'type': typ, 'x': x, 'y': 0.0, 'size_x': 1.0, 'size_y': 1.0, 'radius': 0.5}
    return _world(obstacles=[dict(o, name='o')])


@pytest.mark.parametrize('typ', ['box', 'cylinder'])
def test_collision_touching_vs_far(typ):
    # footprint front is at x = 0.52; obstacle near face at x - 0.5
    near = _box_world(1.02, typ)
    assert [o.name for o in near.collisions(0.0, 0.0, 0.0, 0.0)] == ['o']
    assert near.clearance(0.0, 0.0, 0.0, 0.0) == pytest.approx(0.0, abs=1e-6)
    far = _box_world(2.02, typ)
    assert far.collisions(0.0, 0.0, 0.0, 0.0) == []
    assert far.clearance(0.0, 0.0, 0.0, 0.0) == pytest.approx(1.0, abs=0.02)


def test_collision_margin_and_empty_clearance():
    w = _box_world(2.02)
    assert w.collisions(0.0, 0.0, 0.0, 0.0, margin=1.1)
    assert not w.collisions(0.0, 0.0, 0.0, 0.0, margin=0.9)
    assert _world().clearance(0.0, 0.0, 0.0, 0.0) == math.inf


def test_moving_obstacle_collides_only_when_arrived():
    w = _world(obstacles=[{'type': 'cylinder', 'radius': 0.25, 'speed': 1.0, 'mode': 'once',
                           'path': [[5.0, 0.0], [0.7, 0.0]]}])
    assert not w.collisions(0.0, 0.0, 0.0, 0.0)
    assert w.collisions(0.0, 0.0, 0.0, 10.0)


# ---------------------------------------------------------------- GPS

def test_enu_latlon_roundtrip_50m():
    for x, y in [(50.0, 0.0), (0.0, 50.0), (-35.0, 35.0), (30.0, -40.0)]:
        lat, lon = W.enu_to_latlon(x, y, 48.0, 9.0)
        bx, by = W.latlon_to_enu(lat, lon, 48.0, 9.0)
        assert abs(bx - x) < 1e-3 and abs(by - y) < 1e-3


def test_one_metre_north_is_degrees_lat():
    lat, lon = W.enu_to_latlon(0.0, 1.0, 48.0, 9.0)
    assert lat - 48.0 == pytest.approx(8.99e-6, rel=0.01)
    assert lon == 9.0


def test_one_metre_east_is_degrees_lon():
    lat, lon = W.enu_to_latlon(1.0, 0.0, 48.0, 9.0)
    assert lat == 48.0
    assert lon - 9.0 == pytest.approx(8.99e-6 / math.cos(math.radians(48.0)), rel=0.01)
