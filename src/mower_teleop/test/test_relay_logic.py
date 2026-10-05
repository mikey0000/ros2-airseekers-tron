import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from mower_teleop.relay_logic import LeaseTracker, clamp, twist_fields_from_json  # noqa: E402


def frame(lin=None, ang=None):
    return json.dumps({'twist': {'linear': lin or {}, 'angular': ang or {}}})


def test_clamp_positive():
    assert clamp(3.0, 0.5) == 0.5


def test_clamp_negative():
    assert clamp(-3.0, 0.5) == -0.5


def test_clamp_within_limit():
    assert clamp(0.2, 0.5) == 0.2


def test_fields_clamped_both_signs():
    f = twist_fields_from_json(frame({'x': 9, 'y': -9}, {'z': -7}), 0.5, 1.0)
    assert f['linear'] == (0.5, -0.5, 0.0)
    assert f['angular'] == (0.0, 0.0, -1.0)


def test_missing_fields_default_zero():
    assert twist_fields_from_json('{}', 0.5, 1.0) == dict(
        linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
    f = twist_fields_from_json(frame({'x': 0.3}), 0.5, 1.0)
    assert f['linear'] == (0.3, 0.0, 0.0)


def test_header_ignored():
    raw = json.dumps({'header': {'frame_id': 'x'}, 'twist': {'linear': {'x': 0.1}}})
    assert twist_fields_from_json(raw, 0.5, 1.0)['linear'][0] == 0.1


def test_non_numeric_raises():
    with pytest.raises(ValueError):
        twist_fields_from_json(frame({'x': 'fast'}), 0.5, 1.0)
    with pytest.raises(ValueError):
        twist_fields_from_json(frame({'x': None}), 0.5, 1.0)


def test_bad_json_raises():
    with pytest.raises(ValueError):
        twist_fields_from_json('{not json', 0.5, 1.0)
    with pytest.raises(ValueError):
        twist_fields_from_json('[1,2]', 0.5, 1.0)


def test_lease_replay_then_expiry():
    lt = LeaseTracker(0.25)
    assert not lt.should_replay(0.0)
    assert not lt.expired(0.0)
    lt.update(10.0)
    assert lt.should_replay(10.1)
    assert not lt.expired(10.1)
    assert lt.should_replay(10.25)
    assert not lt.should_replay(10.3)
    assert lt.expired(10.3)


def test_lease_refresh_and_clear():
    lt = LeaseTracker(0.25)
    lt.update(0.0)
    lt.update(0.2)
    assert lt.should_replay(0.4)
    lt.clear()
    assert not lt.expired(5.0)
    assert not lt.should_replay(5.0)
