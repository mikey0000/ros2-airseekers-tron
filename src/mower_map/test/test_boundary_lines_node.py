# SPDX-License-Identifier: GPL-3.0-or-later
"""map_server_node boundary lines (2026-11-02) on the ROS-free stub node of
test_map_edit_txn: replace_areas carries them atomically, set/get_boundary_lines, the
footprint / blade disc hit report (~/boundary_line_hit) and the planner bands of both modes."""
import json
import types

from mower_map import areas as core

from test_map_edit_txn import (A, B, _Pub, _area, _odom, _replace_req, _res,  # noqa: F401
                               _settings, msn, node)

BLADE = {'blade_radius': -1.0, 'blade_disc_mm': 220, 'blade_offset_x': 0.23,
         'blade_offset_y': 0.0, 'boundary_line_keepout_m': 0.57,
         'boundary_line_blade_keepout_m': -1.0, 'boundary_line_check': True}

WALL = {'version': 1, 'lines': [{'id': 'bl-1', 'name': 'drop-off',
                                 'points': [[5.0, -1.0], [5.0, 8.0]], 'width_m': 0.0}]}


def _params(n, **over):
    params = {**BLADE, **over}
    base = n.p
    n.p = lambda k: params[k] if k in params else base(k)
    n.blade_offset = None


def _req(lines=None, areas=None):
    r = _replace_req(areas or [_area('a', A), _area('b', B)])
    r.boundary_lines_json = '' if lines is None else json.dumps(lines)
    return r


def test_replace_areas_swaps_the_lines_in_with_the_areas(node):
    n = node
    _settings(n)
    res = n.srv_replace_areas(_req(WALL), _res())
    assert res.success, res.message
    assert [ln.id for ln in n.store.boundary_lines] == ['bl-1']
    assert n.rebuilt == [['a', 'b']] and len(n.persisted) == 1     # one rebuild, one save
    assert '1 boundary lines' in res.message
    # '' keeps them (an older GUI, the OpenMower importer)
    assert n.srv_replace_areas(_req(None), _res()).success
    assert [ln.id for ln in n.store.boundary_lines] == ['bl-1']
    # an empty set clears them
    assert n.srv_replace_areas(_req({'version': 1, 'lines': []}), _res()).success
    assert n.store.boundary_lines == []


def test_replace_areas_with_a_bad_line_changes_nothing(node):
    n = node
    _settings(n)
    n.store.boundary_lines = [core.blines.BoundaryLine('old', [(1.0, 1.0), (2.0, 2.0)])]
    bad = {'lines': [{'id': 'x', 'points': [[0, 0]]}]}
    res = n.srv_replace_areas(_req(bad, [_area('new', A)]), _res())
    assert res.success is False and 'map unchanged' in res.message and 'at least 2' in res.message
    assert [a.name for a in n.store.areas] == ['a', 'b']
    assert [ln.id for ln in n.store.boundary_lines] == ['old']
    assert n.rebuilt == [] and n.persisted == []


def test_set_and_get_boundary_lines(node):
    n = node
    res = n.srv_set_boundary_lines(types.SimpleNamespace(area_index=255,
                                                         settings_json=json.dumps(WALL)), _res())
    assert res.success, res.message
    assert n.rebuilt and n.persisted[-1][0] == 'set_boundary_lines'
    got = n.srv_get_boundary_lines(types.SimpleNamespace(area_index=0),
                                   types.SimpleNamespace(success=None, settings_json=''))
    assert got.success
    assert json.loads(got.settings_json)['lines'][0]['id'] == 'bl-1'
    res = n.srv_set_boundary_lines(types.SimpleNamespace(area_index=255,
                                                         settings_json='{"version": 9}'), _res())
    assert res.success is False and 'newer' in res.message
    assert [ln.id for ln in n.store.boundary_lines] == ['bl-1']


def test_footprint_on_a_line_is_reported_with_its_id(node, msn, monkeypatch):
    n = node
    monkeypatch.setattr(msn, 'String', lambda data: types.SimpleNamespace(data=data))
    _params(n)
    n.line_hit_pub = _Pub()
    n.store.boundary_lines = core.blines.parse_json(json.dumps(WALL))[2]
    _odom(n, 4.5, 3.0)          # yaw 0: nose at x = 5.1, 0.10 across the wall
    _odom(n, 3.0, 3.0)          # nose at 3.6: clear
    hit, clear = (json.loads(m) for m in n.line_hit_pub.msgs)
    assert [h['id'] for h in hit['hits']] == ['bl-1'] and hit['hits'][0]['depth_m'] == 0.1
    assert clear['hits'] == []


BED = {'version': 2, 'lines': [{'id': 'bed', 'points': [[5.0, -1.0], [5.0, 8.0]],
                                'width_m': 0.0, 'mode': 'blade'}]}


def test_blade_line_hit_is_the_disc(node, msn, monkeypatch):
    n = node
    monkeypatch.setattr(msn, 'String', lambda data: types.SimpleNamespace(data=data))
    _params(n)
    n.line_hit_pub = _Pub()
    n.store.boundary_lines = core.blines.parse_json(json.dumps(BED))[2]
    _odom(n, 4.5, 3.0)          # nose 0.10 across, the disc edge (4.84) is 0.16 short: clear
    _odom(n, 4.7, 3.0)          # disc edge 5.04: 0.04 across
    clear, hit = (json.loads(m) for m in n.line_hit_pub.msgs)
    assert clear['hits'] == []
    assert hit['hits'] == [{'id': 'bed', 'name': '', 'mode': 'blade', 'depth_m': 0.04}]
    # the 330 mm disc reaches 0.055 further
    _params(n, blade_disc_mm=330)
    _odom(n, 4.7, 3.0)
    assert json.loads(n.line_hit_pub.msgs[-1])['hits'][0]['depth_m'] == 0.095


def _band_widths(n):
    out = {}
    for a in n._areas_with_lines(n.store.areas):
        for o in a.obstacles:
            xs = [p[0] for p in o.polygon]
            out[o.name] = round(max(xs) - min(xs), 4)
    return out


def test_planner_bands_of_both_modes(node):
    n = node
    _params(n)
    lines = {'version': 2, 'lines': [
        {'id': 'pond', 'points': [[2.0, -1.0], [2.0, 8.0]], 'mode': 'body'},
        {'id': 'bed', 'points': [[5.0, -1.0], [5.0, 8.0]], 'mode': 'blade', 'width_m': 0.1}]}
    n.store.boundary_lines = core.blines.parse_json(json.dumps(lines))[2]
    assert n.boundary_blade_keepout() == 0.32
    assert _band_widths(n) == {'boundary line pond': 0.57, 'boundary line bed': 0.42}
    _params(n, blade_disc_mm=330)                    # 2 x (0.165 + 0.05) + width 0.1
    assert _band_widths(n)['boundary line bed'] == 0.53
    _params(n, boundary_line_blade_keepout_m=0.5)    # explicit override
    assert _band_widths(n) == {'boundary line pond': 0.57, 'boundary line bed': 0.6}


def test_mode_survives_set_get_and_replace(node):
    n = node
    _settings(n)
    res = n.srv_set_boundary_lines(types.SimpleNamespace(area_index=255,
                                                         settings_json=json.dumps(BED)), _res())
    assert res.success, res.message
    got = json.loads(n.srv_get_boundary_lines(
        types.SimpleNamespace(area_index=0),
        types.SimpleNamespace(success=None, settings_json='')).settings_json)
    assert got['version'] == 2 and got['lines'][0]['mode'] == 'blade'
    # an older GUI's v1 set: body lines
    assert n.srv_replace_areas(_req(WALL), _res()).success
    assert [ln.mode for ln in n.store.boundary_lines] == ['body']
    assert n.srv_replace_areas(_req(BED), _res()).success
    assert [ln.mode for ln in n.store.boundary_lines] == ['blade']
