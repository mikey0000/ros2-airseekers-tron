# SPDX-License-Identifier: GPL-3.0-or-later
"""map_server_node terrain-memory glue (incidents, actions, tilt sampling, derived angle)
on a ROS-free stub node (same stubs as test_dock_yaw_autocorrect)."""
import json
import math
import time
import types

import pytest

from mower_map import area_settings as aset
from mower_map import areas as core
from mower_map import terrain as terr

from test_dock_yaw_autocorrect import _Log, msn  # noqa: F401 (fixture)

SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


def _node(mod, tmp_path, **params):
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
    n.terrain = {}
    n.terrain_dirty = False
    n.pose = (5.0, 5.0, 0.0, 'base_link')
    n._dig_latched = False
    n._terrain_last_xy = None
    n._terrain_last_t = None
    n.status = None
    n.imu = None
    n.imu_time = None
    n._imu_sub = object()
    n._pump = types.SimpleNamespace(poll=lambda subs=None: None)
    n.terrain_grid_pub, n.terrain_cost_pub, n.terrain_summary_pub = _Pub(), _Pub(), _Pub()
    n.rebuilt = 0

    def rebuild(replan=True):
        n.rebuilt += 1
    n.rebuild = rebuild
    n.persist_best_effort = lambda ctx: None
    n.grid_msg = lambda spec, data: types.SimpleNamespace(data=data.copy(), spec=spec)
    mod.String = lambda data='': types.SimpleNamespace(data=data)
    n.load_terrain()
    return n


def _str(d):
    return types.SimpleNamespace(data=json.dumps(d))


def test_incident_from_mission_topic_and_robot_pose(msn, tmp_path):  # noqa: F811
    n = _node(msn, tmp_path)
    n.on_incident(_str({'kind': 'follow_abort', 'x': 2.0, 'y': 3.0, 'detail': 'x'}))
    n.on_incident(_str({'kind': 'subpath_failed'}))               # robot pose (5, 5)
    n.on_incident(_str({'kind': 'detour', 'x': 50.0, 'y': 50.0}))  # off every area: dropped
    n.on_incident(types.SimpleNamespace(data='not json'))
    incs = n.terrain['Lawn'].incidents
    assert [(i['kind'], i['x'], i['y']) for i in incs] == [('follow_abort', 2.0, 3.0),
                                                           ('subpath_failed', 5.0, 5.0)]
    assert n.terrain_dirty


def test_dig_stall_rising_edge_only(msn, tmp_path):  # noqa: F811
    n = _node(msn, tmp_path)
    for v in (False, True, True, False, True):
        n.on_dig_stall(types.SimpleNamespace(data=v))
    assert [i['kind'] for i in n.terrain['Lawn'].incidents] == ['dig_stall', 'dig_stall']


def test_publish_terrain_grids_and_summary(msn, tmp_path):  # noqa: F811
    n = _node(msn, tmp_path)
    n.terrain_incident('dig_stall', 5.0, 5.0, 'x')
    n.publish_terrain()
    grid = n.terrain_grid_pub.msgs[-1].data
    cost = n.terrain_cost_pub.msgs[-1].data
    r, c = n.spec.index_of(5.0, 5.0)
    assert grid[r, c] >= 60 and grid[0, 0] == -1
    assert 0 < cost[r, c] <= 60 and cost.max() <= 60
    s = json.loads(n.terrain_summary_pub.msgs[-1].data)
    assert s['areas'][0]['area'] == 'Lawn' and s['areas'][0]['clusters'][0]['n'] == 1
    assert not n.terrain_dirty


def test_actions_keepout_dismiss_clear(msn, tmp_path):  # noqa: F811
    n = _node(msn, tmp_path)
    n.terrain_incident('dig_stall', 5.0, 5.0)
    n.terrain_incident('follow_abort', 5.4, 5.2)
    n.terrain_incident('dig_stall', 1.0, 1.0)
    res = types.SimpleNamespace(success=None, message='')
    req = lambda d, i=0: types.SimpleNamespace(area_index=i, settings_json=json.dumps(d))  # noqa
    n.srv_terrain_action(req({'action': 'bogus'}), res)
    assert res.success is False
    n.srv_terrain_action(req({'action': 'keepout', 'cluster_id': 1}), res)
    assert res.success, res.message
    obs = n.store.areas[0].obstacles
    assert len(obs) == 1 and obs[0].source == core.SOURCE_DIG and n.rebuilt == 1
    assert core.point_in_polygon(5.2, 5.1, obs[0].polygon)
    assert [c['id'] for c in n.terrain['Lawn'].clusters()] == [3]
    n.srv_terrain_action(req({'action': 'dismiss', 'cluster_id': 3}), res)
    assert res.success and n.terrain['Lawn'].clusters() == []
    assert (tmp_path / 'terrain_Lawn.json').exists()
    n.srv_terrain_action(req({'action': 'clear'}, 255), res)
    assert res.success and n.terrain['Lawn'].incidents == []


def _imu(roll, pitch, wz=0.0):
    return (roll, pitch, wz)


def test_tilt_sampling_moves_and_gates(msn, tmp_path):  # noqa: F811
    n = _node(msn, tmp_path)
    now = time.monotonic()
    n.imu, n.imu_time = _imu(0.0, math.radians(-8)), now     # nose up heading +x
    for k in range(20):
        n.terrain_sample(2.0 + 0.1 * k, 5.0, 0.0, None, now)
    r = n.terrain['Lawn'].raster
    assert r.gw.sum() == 19                                   # first sample only anchors
    sl = r.slope_deg()
    assert sl[r.index(3.0, 5.0)] == pytest.approx(8.0, abs=0.01)
    assert (r.gx[r.gw > 0] > 0).all()                         # uphill = +x
    # turning fast / stale IMU / lifted: no sample
    n.imu = _imu(0.0, 0.1, wz=1.0)
    n.terrain_sample(6.0, 5.0, 0.0, None, now)
    n.imu, n.imu_time = _imu(0.0, 0.1), now - 5.0
    n.terrain_sample(6.2, 5.0, 0.0, None, now)
    n.imu_time = now
    n.status = types.SimpleNamespace(is_charging=False, lift_triggered=True)
    n.terrain_sample(6.4, 5.0, 0.0, None, now)
    assert r.gw.sum() == 19


def test_get_area_settings_adds_derived_slope_angle(msn, tmp_path):  # noqa: F811
    n = _node(msn, tmp_path)
    n.settings.set_area('Lawn', {'slope_mode': 'contour'})
    r = n.terrain['Lawn'].raster
    g = math.tan(math.radians(12))
    for x in range(100):
        for y in range(100):
            for yaw in (0.0, math.pi):                       # gradient along +y
                r.add_tilt(0.05 + 0.1 * x, 0.05 + 0.1 * y, 0.0, g)
    res = types.SimpleNamespace(success=None, settings_json='')
    n.srv_get_area_settings(types.SimpleNamespace(area_index=0), res)
    eff = json.loads(res.settings_json)
    assert eff['slope_mow_angle_deg'] == pytest.approx(0.0, abs=0.5)   # contour = along x
    assert 'contour' in eff['slope_angle_why']
    n.settings.set_area('Lawn', {'slope_mode': None})
    n.srv_get_area_settings(types.SimpleNamespace(area_index=0), res)
    assert json.loads(res.settings_json)['slope_mow_angle_deg'] is None


def test_area_resize_keeps_memory(msn, tmp_path):  # noqa: F811
    n = _node(msn, tmp_path)
    n.terrain_incident('dig_stall', 5.0, 5.0)
    n.store.areas[0].polygon = [(-2.0, 0.0), (10.0, 0.0), (10.0, 10.0), (-2.0, 10.0)]
    n.load_terrain()                                         # saves old, reloads re-gridded
    t = n.terrain['Lawn']
    assert len(t.incidents) == 1
    assert t.raster.traction_score()[t.raster.index(5.0, 5.0)] > 60
