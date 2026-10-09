"""Helpers of alert_node (payloads, anchor persistence, mask conversion). Skipped without ROS."""

import json
import os

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('mower_interfaces.msg')
pytest.importorskip('mowgli_interfaces.msg')
pytest.importorskip('nav_msgs.msg')

from nav_msgs.msg import OccupancyGrid  # noqa: E402

from mower_alerts import alert_node as n  # noqa: E402
from mower_alerts import rules as r  # noqa: E402


def sample_alert(**params):
    return r.Alert('theft_lift', 'theft', 'theftLift', 5, 'THEFT', params)


def test_alert_payload_shape_and_json():
    a = sample_alert(lat=52.5, count=3, nested=None)
    p = n.alert_payload(a, 1700000000.123)
    assert p['type'] == 'alert'
    assert p['id'] == 'theft_lift-1700000000123'
    assert p['id'].startswith(a.incident)
    assert (p['incident'], p['kind'], p['message'], p['priority'], p['text']) == \
        ('theft_lift', 'theft', 'theftLift', 5, 'THEFT')
    assert p['params'] == {'lat': '52.5', 'count': '3', 'nested': 'None'}
    assert p['stamp'] == 1700000000.123
    assert json.loads(json.dumps(p)) == p


def test_alert_payload_priority_is_int():
    a = r.Alert('x', 'blocked', 'stuck', 4.0, 't')
    assert n.alert_payload(a, 1.0)['priority'] == 4
    assert isinstance(n.alert_payload(a, 1.0)['priority'], int)


def test_heartbeat_payload():
    p = n.heartbeat_payload(('a', 'b'), 12.5)
    assert p == {'type': 'heartbeat', 'active': ['a', 'b'], 'stamp': 12.5}
    assert json.loads(json.dumps(p)) == p
    assert n.heartbeat_payload([], 1.0)['active'] == []


def test_anchor_round_trip(tmp_path):
    path = str(tmp_path / 'sub' / 'anchor.json')       # parent dir is created
    n.save_anchor(path, r.Fix(52.123456, 13.654321, 0.8, 3))
    assert os.path.isfile(path)
    assert not os.path.exists(path + '.tmp')
    got = n.load_anchor(path)
    assert (got.lat, got.lon, got.accuracy_m) == (52.123456, 13.654321, 0.8)


def test_save_anchor_none_removes_file_and_is_idempotent(tmp_path):
    path = str(tmp_path / 'anchor.json')
    n.save_anchor(path, r.Fix(1.0, 2.0, 3.0))
    assert os.path.exists(path)
    n.save_anchor(path, None)
    assert not os.path.exists(path)
    n.save_anchor(path, None)                           # already gone: no error
    assert n.load_anchor(path) is None


def test_save_anchor_empty_path_is_noop():
    n.save_anchor('', r.Fix(1.0, 2.0, 3.0))
    n.save_anchor('', None)


def test_save_anchor_overwrites(tmp_path):
    path = str(tmp_path / 'anchor.json')
    n.save_anchor(path, r.Fix(1.0, 2.0, 3.0))
    n.save_anchor(path, r.Fix(4.0, 5.0, 6.0))
    assert n.load_anchor(path).lat == 4.0


def test_load_anchor_missing_file(tmp_path):
    assert n.load_anchor(str(tmp_path / 'nope.json')) is None


@pytest.mark.parametrize('content', ['not json', '', '{}', '[]', '{"lat": 1, "lon": 2}',
                                     '{"lat": "x", "lon": 2, "accuracy_m": 1}', 'null'])
def test_load_anchor_garbage(tmp_path, content):
    path = tmp_path / 'anchor.json'
    path.write_text(content)
    assert n.load_anchor(str(path)) is None


def test_mask_from_grid():
    msg = OccupancyGrid()
    msg.info.resolution = 0.5
    msg.info.width = 6
    msg.info.height = 4
    msg.info.origin.position.x = 10.0
    msg.info.origin.position.y = -2.0
    data = [100] * 24
    for row in (1, 2):
        for col in (2, 3):
            data[row * 6 + col] = 0
    msg.data = data
    m = n.mask_from_grid(msg)
    assert isinstance(m, r.MapMask)
    assert (m.ox, m.oy, m.res, m.w, m.h) == (10.0, -2.0, 0.5, 6, 4)
    assert m.has_free
    assert m.distance_outside(11.5, -0.75, 3.0) == 0.0           # inside the free block
    assert m.distance_outside(10.2, -1.8, 3.0) > 0.0             # corner cell, occupied


def test_mask_from_grid_all_occupied():
    msg = OccupancyGrid()
    msg.info.resolution = 0.1
    msg.info.width = 3
    msg.info.height = 3
    msg.data = [100] * 9
    assert not n.mask_from_grid(msg).has_free
