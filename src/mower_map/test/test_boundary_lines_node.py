# SPDX-License-Identifier: GPL-3.0-or-later
"""map_server_node boundary lines (2026-11-02) on the ROS-free stub node of
test_map_edit_txn: replace_areas carries them atomically, set/get_boundary_lines, and the
footprint hit report (~/boundary_line_hit)."""
import json
import types

from mower_map import areas as core

from test_map_edit_txn import (A, B, _Pub, _area, _odom, _replace_req, _res,  # noqa: F401
                               _settings, msn, node)

WALL = {'version': 1, 'lines': [{'id': 'bl-1', 'name': 'drop-off',
                                 'points': [[5.0, -1.0], [5.0, 8.0]], 'width_m': 0.0}]}


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
    params = {'boundary_line_check': True}
    base = n.p
    n.p = lambda k: params[k] if k in params else base(k)
    n.line_hit_pub = _Pub()
    n.store.boundary_lines = core.blines.parse_json(json.dumps(WALL))[2]
    _odom(n, 4.5, 3.0)          # yaw 0: nose at x = 5.1, 0.10 across the wall
    _odom(n, 3.0, 3.0)          # nose at 3.6: clear
    hit, clear = (json.loads(m) for m in n.line_hit_pub.msgs)
    assert [h['id'] for h in hit['hits']] == ['bl-1'] and hit['hits'][0]['depth_m'] == 0.1
    assert clear['hits'] == []
