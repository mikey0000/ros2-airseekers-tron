# SPDX-License-Identifier: GPL-3.0-or-later
"""map_server_node non-grass glue (nongrass_node.NonGrassMixin, 2026-10-09) on a ROS-free stub
node (same stubs as test_terrain_node / test_dock_yaw_autocorrect)."""
import json
import sys
import types

import numpy as np
import pytest

from mower_map import area_settings as aset
from mower_map import areas as core
from mower_map import nongrass as ngm

from test_dock_yaw_autocorrect import msn  # noqa: F401 (fixture)

SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class _Log:
    def __init__(self):
        self.warns, self.infos = [], []

    def warn(self, m, **kw):
        self.warns.append(m)

    def info(self, m, **kw):
        self.infos.append(m)

    error = warn


def _node(mod, tmp_path, monkeypatch, **params):
    n = mod.MapServerNode.__new__(mod.MapServerNode)
    p = dict(mod.PARAMS)
    p.update({'maps_dir': str(tmp_path), 'terrain_dir': str(tmp_path)})
    p.update(params)
    n.p = p.get
    log = _Log()
    n.get_logger = lambda: log
    n.log = log
    n.resolution = 0.1
    n.areas_path = str(tmp_path / 'areas.dat')
    n.store = core.MapStore()
    n.store.areas.append(core.Area('Lawn', list(SQUARE)))
    n.settings = aset.AreaSettingsStore()
    n.spec = core.grid_for_polygons([SQUARE], 0.1, 2.0)
    n.map_frame = 'map'
    n.status = None
    n.terrain = {}
    n.terrain_dirty = False
    n.nongrass = {}
    n.nongrass_dirty = True
    n._nongrass_stats = {'clouds': 0, 'cells': 0, 'dropped': 0}
    n.nongrass_cost_pub, n.nongrass_grid_pub = _Pub(), _Pub()
    n.nongrass_summary_pub = _Pub()
    n.rebuilt = 0

    def rebuild(replan=True):
        n.rebuilt += 1
    n.rebuild = rebuild
    n.persist_best_effort = lambda ctx: None
    n.grid_msg = lambda spec, data: types.SimpleNamespace(data=data.copy(), spec=spec)
    string = lambda data='': types.SimpleNamespace(data=data)  # noqa: E731
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', types.SimpleNamespace(String=string))
    return n


def _summary(n):
    return json.loads(n.nongrass_summary_pub.msgs[-1].data)


def _confirm(n, x=5.05, y=5.05, frames=4):
    for k in range(frames):
        n.nongrass_observe([x], [y], [True], [2.0], [0.2 * k], [0.0])
    for m in n.nongrass.values():
        m.update_confirmed()


def _req(d, i=0):
    return types.SimpleNamespace(area_index=i, settings_json=json.dumps(d))


def _res():
    return types.SimpleNamespace(success=None, message='')


def _cloud(n, xs, ys, frame='map', vx=0.0):
    import struct
    rows = b''.join(struct.pack('<7f', x, y, 0.0, 1.0, 2.0, vx, 0.0) for x, y in zip(xs, ys))
    names = ('x', 'y', 'z', 'nongrass', 'weight', 'vx', 'vy')
    return types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id=frame), point_step=28, width=len(xs), height=1,
        data=rows, fields=[types.SimpleNamespace(name=nm, offset=4 * i, datatype=7)
                           for i, nm in enumerate(names)])


def test_sync_areas_mowing_only_reload_and_drop(msn, tmp_path, monkeypatch):  # noqa: F811
    """Only mowing areas get memory; polygon change reloads; removal saves and drops."""
    n = _node(msn, tmp_path, monkeypatch)
    nav = core.Area('Path', [(20.0, 0.0), (22.0, 0.0), (22.0, 2.0), (20.0, 2.0)])
    nav.is_navigation = True
    n.store.areas.append(nav)
    n.nongrass_sync_areas()
    assert list(n.nongrass) == ['Lawn'] and n.nongrass_dirty
    m = n.nongrass['Lawn']
    n.nongrass_sync_areas()
    assert n.nongrass['Lawn'] is m                           # unchanged polygon: kept
    _confirm(n)
    n.store.areas[0].polygon = [(-2.0, 0.0), (10.0, 0.0), (10.0, 10.0), (-2.0, 10.0)]
    n.nongrass_sync_areas()
    m2 = n.nongrass['Lawn']
    assert m2 is not m and m2.raster.hit.sum() == pytest.approx(4.0)   # saved + adopted
    assert m2.confirmed_mask().sum() == 1
    assert (tmp_path / 'nongrass_Lawn.npz').exists()
    (tmp_path / 'nongrass_Lawn.npz').unlink()
    del n.store.areas[0]
    n.nongrass_sync_areas()
    assert n.nongrass == {} and (tmp_path / 'nongrass_Lawn.npz').exists()


def test_sync_areas_disabled_and_corrupt(msn, tmp_path, monkeypatch):  # noqa: F811
    """Disabled memory does nothing; an unreadable file warns and starts empty."""
    n = _node(msn, tmp_path, monkeypatch, nongrass_enabled=False)
    n.nongrass_sync_areas()
    assert n.nongrass == {}
    n = _node(msn, tmp_path, monkeypatch)
    (tmp_path / 'nongrass_Lawn.npz').write_bytes(b'garbage')
    n.nongrass_sync_areas()
    assert 'Lawn' in n.nongrass and n.log.warns and 'unreadable' in n.log.warns[0]


def test_observe_routes_and_ignores_outside(msn, tmp_path, monkeypatch):  # noqa: F811
    """Points land in the right area; points in no raster are not counted."""
    n = _node(msn, tmp_path, monkeypatch)
    n.store.areas.append(core.Area('Back', [(20.0, 0.0), (30.0, 0.0), (30.0, 10.0),
                                            (20.0, 10.0)]))
    n.nongrass_sync_areas()
    used = n.nongrass_observe([5.05, 25.05, 100.0, 15.0], [5.05, 5.05, 5.0, 5.0],
                              [True, True, True, True], [2.0] * 4, [0.0] * 4, [0.0] * 4)
    assert used == 2
    assert n.nongrass['Lawn'].raster.hit.sum() == 1 and n.nongrass['Back'].raster.hit.sum() == 1
    r, c, _ = n.nongrass['Back'].raster.cells([25.05], [5.05])
    assert n.nongrass['Back'].raster.hit[r[0], c[0]] == 1
    assert n._nongrass_stats == {'clouds': 1, 'cells': 2, 'dropped': 0}
    assert n.nongrass_observe([100.0], [100.0], [True], [2.0], [0.0], [0.0]) == 0


def test_publish_after_confirmation(msn, tmp_path, monkeypatch):  # noqa: F811
    """Confirmed cell -> 75 in the cost grid and the cluster in the JSON summary."""
    n = _node(msn, tmp_path, monkeypatch)
    n.nongrass_sync_areas()
    _confirm(n)
    n.on_nongrass_publish_timer()
    cost = n.nongrass_cost_pub.msgs[-1].data
    r, c = n.spec.index_of(5.05, 5.05)
    assert cost[r, c] == 75 and cost.max() == 75 and cost[r, c + 1] == 50
    grid = n.nongrass_grid_pub.msgs[-1].data
    assert grid[r, c] == 100 and grid[0, 0] == -1
    s = _summary(n)
    a = s['areas'][0]
    assert a['area'] == 'Lawn' and a['area_index'] == 0 and a['enabled'] is True
    assert a['confirmed_cells'] == 1 and len(a['clusters']) == 1
    assert a['clusters'][0]['cells'] == 1 and s['version'] == 1
    assert s['stats']['cells'] == 4 and not n.nongrass_dirty
    # nothing changed: the timer does not republish
    k = len(n.nongrass_cost_pub.msgs)
    n.on_nongrass_publish_timer()
    assert len(n.nongrass_cost_pub.msgs) == k


def test_avoid_non_grass_off_zero_cost_but_summary(msn, tmp_path, monkeypatch):  # noqa: F811
    """The switch gates the cost only; the memory and summary keep going."""
    n = _node(msn, tmp_path, monkeypatch)
    n.nongrass_sync_areas()
    n.settings.set_area('Lawn', {'avoid_non_grass': False})
    _confirm(n)
    n.on_nongrass_publish_timer()
    assert n.nongrass_cost_pub.msgs[-1].data.max() == 0
    a = _summary(n)['areas'][0]
    assert a['enabled'] is False and a['confirmed_cells'] == 1
    n.settings.set_area('Lawn', {'avoid_non_grass': True})
    n.publish_nongrass()
    assert n.nongrass_cost_pub.msgs[-1].data.max() == 75


def test_publish_without_memory_still_publishes_zero_grid(msn, tmp_path, monkeypatch):  # noqa: F811
    """Nav2 StaticLayer needs a first map even when nothing is learned."""
    n = _node(msn, tmp_path, monkeypatch)
    n.publish_nongrass()
    cost = n.nongrass_cost_pub.msgs[-1].data
    assert cost.shape == (n.spec.height, n.spec.width) and cost.max() == 0 and cost.min() == 0
    assert (n.nongrass_grid_pub.msgs[-1].data == -1).all()
    assert _summary(n)['areas'] == []
    n.nongrass = None                                        # memory off entirely
    n.publish_nongrass()
    assert len(n.nongrass_cost_pub.msgs) == 2


def test_cloud_dropped_while_charging_or_lifted(msn, tmp_path, monkeypatch):  # noqa: F811
    """No learning while docked or lifted; the drop is counted."""
    n = _node(msn, tmp_path, monkeypatch)
    n.nongrass_sync_areas()
    msg = _cloud(n, [5.05], [5.05])
    n.status = types.SimpleNamespace(is_charging=True, lift_triggered=False)
    n.on_nongrass_cloud(msg)
    n.status = types.SimpleNamespace(is_charging=False, lift_triggered=True)
    n.on_nongrass_cloud(msg)
    assert n._nongrass_stats == {'clouds': 0, 'cells': 0, 'dropped': 2}
    assert n.nongrass['Lawn'].raster.hit.sum() == 0
    n.status = types.SimpleNamespace(is_charging=False, lift_triggered=False)
    n.on_nongrass_cloud(msg)
    assert n.nongrass['Lawn'].raster.hit.sum() == 1 and n._nongrass_stats['cells'] == 1


def test_cloud_wrong_frame_or_fields_dropped(msn, tmp_path, monkeypatch):  # noqa: F811
    """A cloud in another frame, or without the float32 fields, is warned about and ignored."""
    n = _node(msn, tmp_path, monkeypatch)
    n.nongrass_sync_areas()
    n.on_nongrass_cloud(_cloud(n, [5.05], [5.05], frame='odom'))
    bad = _cloud(n, [5.05], [5.05])
    bad.fields = bad.fields[:-1]
    n.on_nongrass_cloud(bad)
    assert n.nongrass['Lawn'].raster.hit.sum() == 0 and len(n.log.warns) == 2
    assert n._nongrass_stats['clouds'] == 0
    n.on_nongrass_cloud(_cloud(n, [5.05], [5.05], frame=''))   # empty frame is accepted
    assert n.nongrass['Lawn'].raster.hit.sum() == 1


def test_cloud_frames_confirm_end_to_end(msn, tmp_path, monkeypatch):  # noqa: F811
    """Decoded PointCloud2 frames from several viewpoints confirm and publish."""
    n = _node(msn, tmp_path, monkeypatch)
    n.nongrass_sync_areas()
    for k in range(4):
        n.on_nongrass_cloud(_cloud(n, [5.05, 5.15], [5.05, 5.05], vx=0.2 * k))
    n.on_nongrass_publish_timer()
    assert _summary(n)['areas'][0]['confirmed_cells'] == 2


def test_terrain_action_nongrass_dismiss(msn, tmp_path, monkeypatch):  # noqa: F811
    """nongrass_dismiss clears the cluster, saves and republishes."""
    n = _node(msn, tmp_path, monkeypatch)
    n.nongrass_sync_areas()
    _confirm(n)
    n.publish_nongrass()
    cid = _summary(n)['areas'][0]['clusters'][0]['id']
    res = _res()
    n.srv_terrain_action(_req({'action': 'nongrass_dismiss', 'cluster_id': cid}), res)
    assert res.success, res.message
    assert n.nongrass['Lawn'].confirmed_mask().sum() == 0
    assert _summary(n)['areas'][0]['clusters'] == [] and n.rebuilt == 0
    assert (tmp_path / 'nongrass_Lawn.npz').exists()


def test_terrain_action_nongrass_keepout(msn, tmp_path, monkeypatch):  # noqa: F811
    """nongrass_keepout adds a dig obstacle around the cluster and rebuilds the map."""
    n = _node(msn, tmp_path, monkeypatch)
    n.nongrass_sync_areas()
    _confirm(n)
    cl = n.nongrass['Lawn'].clusters()[0]
    res = _res()
    n.srv_terrain_action(_req({'action': 'nongrass_keepout', 'cluster_id': cl['id']}), res)
    assert res.success, res.message
    obs = n.store.areas[0].obstacles
    assert len(obs) == 1 and obs[0].source == core.SOURCE_DIG and n.rebuilt == 1
    assert core.point_in_polygon(5.05, 5.05, obs[0].polygon)
    assert 'keep-out' in res.message


def test_terrain_action_nongrass_clear_and_errors(msn, tmp_path, monkeypatch):  # noqa: F811
    """clear wipes the memory; bad / missing / bool cluster ids and area indices fail."""
    n = _node(msn, tmp_path, monkeypatch)
    n.nongrass_sync_areas()
    _confirm(n)
    res = _res()
    n.srv_terrain_action(_req({'action': 'nongrass_dismiss', 'cluster_id': 999999}), res)
    assert res.success is False and 'no non-grass cluster' in res.message
    for bad in ('abc', None, True, 1.5):
        n.srv_terrain_action(_req({'action': 'nongrass_dismiss', 'cluster_id': bad}), res)
        assert res.success is False
    n.srv_terrain_action(_req({'action': 'nongrass_bogus', 'cluster_id': 1}), res)
    assert res.success is False
    n.srv_terrain_action(_req({'action': 'nongrass_clear'}, 7), res)    # no such area
    assert res.success is False and 'no non-grass memory' in res.message
    assert n.nongrass['Lawn'].confirmed_mask().sum() == 1
    n.srv_terrain_action(_req({'action': 'nongrass_clear'}, aset.DEFAULTS_INDEX), res)
    assert res.success and n.nongrass['Lawn'].raster.hit.sum() == 0
    assert n.nongrass_cost_pub.msgs[-1].data.max() == 0
    assert _summary(n)['areas'][0]['confirmed_cells'] == 0


def test_terrain_actions_still_work(msn, tmp_path, monkeypatch):  # noqa: F811
    """Non-nongrass actions keep going to the terrain memory, invalid JSON is rejected."""
    n = _node(msn, tmp_path, monkeypatch)
    n.terrain_grid_pub, n.terrain_cost_pub = _Pub(), _Pub()
    n.terrain_summary_pub = _Pub()
    n.terrain_dirty = False
    n.load_terrain()
    n.terrain_incident('dig_stall', 5.0, 5.0)
    res = _res()
    n.srv_terrain_action(_req({'action': 'bogus'}), res)
    assert res.success is False and 'keepout|confirm|dismiss' in res.message
    n.srv_terrain_action(_req({'action': 'keepout', 'cluster_id': 1}), res)
    assert res.success, res.message
    assert len(n.store.areas[0].obstacles) == 1 and n.rebuilt == 1
    n.srv_terrain_action(_req({'action': 'clear'}, aset.DEFAULTS_INDEX), res)
    assert res.success and n.terrain['Lawn'].incidents == []
    n.srv_terrain_action(types.SimpleNamespace(area_index=0, settings_json='{nope'), res)
    assert res.success is False and 'invalid JSON' in res.message


def test_save_timer_saves_dirty_only(msn, tmp_path, monkeypatch):  # noqa: F811
    """Periodic save writes only memories with unsaved evidence."""
    n = _node(msn, tmp_path, monkeypatch)
    n.nongrass_sync_areas()
    n.on_nongrass_save_timer()
    assert not (tmp_path / 'nongrass_Lawn.npz').exists()
    _confirm(n)
    n.on_nongrass_save_timer()
    assert (tmp_path / 'nongrass_Lawn.npz').exists() and not n.nongrass['Lawn'].dirty
    m = ngm.AreaNonGrass.load(str(tmp_path), 'Lawn', SQUARE, None, 0.0)
    assert m.raster.hit.sum() == pytest.approx(4.0)


def test_area_settings_avoid_non_grass_is_bool():
    """avoid_non_grass defaults true and only accepts real booleans."""
    assert aset.BUILTIN_DEFAULTS['avoid_non_grass'] is True
    assert aset.validate_value('avoid_non_grass', False) == (True, False)
    assert aset.validate_value('avoid_non_grass', True) == (True, True)
    for bad in (1, 0, 'true', 1.0, [True]):
        ok, msg = aset.validate_value('avoid_non_grass', bad)
        assert not ok and 'true or false' in msg
    ok, _m, cleaned = aset.validate({'avoid_non_grass': False})
    assert ok and cleaned == {'avoid_non_grass': False}
    assert not aset.validate({'avoid_non_grass': 'no'})[0]
    assert aset.validate({'avoid_non_grass': None})[2] == {'avoid_non_grass': None}
    st = aset.AreaSettingsStore()
    assert st.effective('Lawn')['avoid_non_grass'] is True
    st.set_area('Lawn', {'avoid_non_grass': False})
    assert st.effective('Lawn')['avoid_non_grass'] is False
