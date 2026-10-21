"""Atomic map replace (field 2026-10-21): a GUI PUT /map is clear_map + add_area x N; the
boundary check ran against the half-built set and published outside_m = 23 m. With
begin_map_edit / commit_map_edit the edits are staged and swapped in in one step: no partial
area set is ever rebuilt, persisted or used by the boundary check, and the boundary check
publishes nothing while the edit is open (idle timeout drops it, the old map stays)."""
import importlib
import sys
import types
from unittest import mock

import pytest

from mower_map import areas as core

from test_dock_yaw_autocorrect import _Log, _stub_ros

A = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
B = [(20.0, 0.0), (30.0, 0.0), (30.0, 10.0), (20.0, 10.0)]
ROBOT_IN_B = (25.0, 5.0)


@pytest.fixture
def msn():
    try:
        import rclpy  # noqa: F401
        stubs = {}
    except ImportError:
        stubs = _stub_ros()
    with mock.patch.dict(sys.modules, stubs):
        sys.modules.pop('mower_map.map_server_node', None)
        sys.modules.pop('mower_map.sub_pump', None)
        mod = importlib.import_module('mower_map.map_server_node')
        yield mod
    sys.modules.pop('mower_map.map_server_node', None)
    sys.modules.pop('mower_map.sub_pump', None)


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m.data)


class _Clock:
    t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def node(msn, monkeypatch):
    clock = _Clock()
    monkeypatch.setattr(msn.time, 'monotonic', clock)
    monkeypatch.setattr(msn, 'Float32', lambda data: types.SimpleNamespace(data=data))
    monkeypatch.setattr(msn, 'Bool', lambda data: types.SimpleNamespace(data=data))
    n = msn.MapServerNode.__new__(msn.MapServerNode)
    params = {'map_edit_timeout_s': 10.0, 'mask_margin': 1.0, 'boundary_check_rate_hz': 0.0,
              'soft_boundary_margin_m': 0.1, 'lethal_boundary_margin_m': 0.5,
              'boundary_debounce_samples': 1, 'lethal_min_duration_s': 0.0,
              'pose_jump_max_m': 100.0, 'pose_jump_window_s': 1.0,
              'yaw_convergence_window_s': 5.0}
    n.p = params.get
    n.log = _Log()
    n.get_logger = lambda: n.log
    n.clock = clock
    n.resolution = 0.1
    n.store = core.MapStore()
    n.store.add_area('a', A)
    n.store.add_area('b', B)
    n._staged = None
    n._staged_deadline = 0.0
    n._cleared_areas = []
    n.progress = mock.MagicMock()
    n.rebuilt, n.persisted, n.synced = [], [], []
    n.rebuild = lambda replan=True: n.rebuilt.append([a.name for a in n.store.areas])
    n.persist_best_effort = lambda ctx: n.persisted.append(
        (ctx, [a.name for a in n.store.areas]))
    n.sync_settings = lambda before, startup=False: n.synced.append(before)
    n.schedule_settings_prune = lambda: None
    n.settings_area_added = lambda name, poly: None
    # on_odom collaborators
    n._pose_gate = types.SimpleNamespace(check=lambda now, x, y: (True, 0.0))
    n.recent_poses = __import__('collections').deque()
    n.is_cutting = lambda: False
    n.check_return_corridor = lambda x, y, now: None
    n._imu_sub = None
    n.last_boundary_check = 0.0
    n.boundary = core.BoundaryClassifier(0.1, 0.5, 1)
    n._lethal_filter = types.SimpleNamespace(min_duration_s=0.0, update=lambda now, v: v)
    n._soft_prev = n._lethal_prev = False
    n._measure_body = lambda x, y: None
    n.terrain_incident = lambda *a: None
    n.outside_pub, n.body_outside_pub = _Pub(), _Pub()
    n.boundary_pub, n.lethal_pub = _Pub(), _Pub()
    return n


def _res():
    return types.SimpleNamespace(success=None, message='')


def _pt(x, y):
    return types.SimpleNamespace(x=x, y=y)


def _add_req(name, poly):
    area = types.SimpleNamespace(name=name, area=types.SimpleNamespace(points=[_pt(*p) for p in poly]),
                                 obstacles=[], obstacle_info=[], is_navigation_area=False)
    return types.SimpleNamespace(area=area, is_navigation_area=False)


def _odom(n, x, y):
    q = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    pose = types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y), orientation=q)
    n.clock.t += 0.05
    n.on_odom(types.SimpleNamespace(pose=types.SimpleNamespace(pose=pose), child_frame_id='base_link'))


def _all_pubs(n):
    return n.outside_pub.msgs + n.body_outside_pub.msgs + n.boundary_pub.msgs + n.lethal_pub.msgs


def test_replace_is_atomic_and_boundary_check_silent(node):
    n = node
    _odom(n, *ROBOT_IN_B)
    assert n.outside_pub.msgs == [0.0]
    n.outside_pub.msgs.clear()
    n.body_outside_pub.msgs.clear(); n.boundary_pub.msgs.clear(); n.lethal_pub.msgs.clear()

    assert n.srv_begin_map_edit(None, _res()).success
    seen = []
    for step in (lambda: n.srv_clear_map(None, _res()),
                 lambda: n.srv_add_area(_add_req('a', A), _res()),
                 lambda: n.srv_add_area(_add_req('b2', B), _res())):
        assert step().success
        _odom(n, *ROBOT_IN_B)     # mid-replace: the robot is in B, B not (re)added yet
        seen.append([a.name for a in n.store.areas])
    # the live map never changed mid-edit, nothing rebuilt / persisted / published
    assert seen == [['a', 'b']] * 3
    assert n.rebuilt == [] and n.persisted == []
    assert _all_pubs(n) == []
    n.progress.reset.assert_not_called()

    assert n.srv_commit_map_edit(None, _res()).success
    assert n.rebuilt == [['a', 'b2']]              # one swap, one rebuild
    assert n.persisted == [('commit_map_edit', ['a', 'b2'])]
    assert n.synced == [[('a', A), ('b', B)]]      # rename b -> b2 carries settings
    _odom(n, *ROBOT_IN_B)
    assert n.outside_pub.msgs == [0.0]             # never the partial-set 23 m


def test_partial_set_never_used_for_outside_distance(node, msn):
    n = node
    calls = []
    real = core.outside_distance
    with mock.patch.object(msn.core, 'outside_distance',
                           side_effect=lambda x, y, areas, extra: calls.append(
                               [a.name for a in areas]) or real(x, y, areas, extra)):
        n.srv_begin_map_edit(None, _res())
        n.srv_clear_map(None, _res())
        n.srv_add_area(_add_req('a', A), _res())
        _odom(n, *ROBOT_IN_B)
        n.srv_add_area(_add_req('b', B), _res())
        n.srv_commit_map_edit(None, _res())
        _odom(n, *ROBOT_IN_B)
    assert calls == [['a', 'b']]


def test_idle_timeout_drops_staged_and_keeps_old_map(node):
    n = node
    n.srv_begin_map_edit(None, _res())
    n.srv_clear_map(None, _res())
    n.srv_add_area(_add_req('a', A), _res())
    n.clock.t += 5.0
    _odom(n, *ROBOT_IN_B)
    assert n.outside_pub.msgs == []               # still open (5 s idle < 10 s)
    n.clock.t += 11.0
    _odom(n, *ROBOT_IN_B)                         # expired: checks resume on the OLD map
    assert n.outside_pub.msgs == [0.0]
    assert [a.name for a in n.store.areas] == ['a', 'b']
    assert any('dropped' in w for w in n.log.warns)
    res = n.srv_commit_map_edit(None, _res())
    assert res.success is False
    assert n.rebuilt == [] and [a.name for a in n.store.areas] == ['a', 'b']


def test_edit_calls_extend_the_deadline(node):
    n = node
    n.srv_begin_map_edit(None, _res())
    n.srv_clear_map(None, _res())
    for i in range(4):                            # 4 x 8 s > 10 s total, but never idle 10 s
        n.clock.t += 8.0
        assert n.srv_add_area(_add_req('a%d' % i, A), _res()).success
    assert n.srv_commit_map_edit(None, _res()).success
    assert [a.name for a in n.store.areas] == ['a0', 'a1', 'a2', 'a3']


def test_abort_keeps_old_map(node):
    n = node
    n.srv_begin_map_edit(None, _res())
    n.srv_clear_map(None, _res())
    assert n.srv_abort_map_edit(None, _res()).success
    _odom(n, *ROBOT_IN_B)
    assert n.outside_pub.msgs == [0.0]
    assert [a.name for a in n.store.areas] == ['a', 'b'] and n.rebuilt == []


def test_staged_channel_targets_staged_index(node):
    n = node
    n.store.areas[1].is_navigation = True
    n.srv_begin_map_edit(None, _res())
    n.srv_clear_map(None, _res())
    req = _add_req('p', B)
    req.is_navigation_area = True
    n.srv_add_area(req, _res())
    ch = types.SimpleNamespace(area_index=0, settings_json='{"points": [[21, 5], [29, 5]], "width_m": 0.7}')
    assert n.srv_set_area_channel(ch, _res()).success
    assert n.store.areas[0].channel is None and n.persisted == []
    n.srv_commit_map_edit(None, _res())
    assert n.store.areas[0].name == 'p' and n.store.areas[0].channel


def test_legacy_clear_add_without_begin_unchanged(node):
    n = node
    assert n.srv_clear_map(None, _res()).success
    assert n.rebuilt == [[]]
    n.srv_add_area(_add_req('a', A), _res())
    assert n.rebuilt == [[], ['a']]
