# SPDX-License-Identifier: GPL-3.0-or-later
"""smooth_areas CLI: writes a proposal + report, never the input. Synthetic map only."""

import json
import os

from mower_map import areas as core
from mower_map import smooth_areas
from test_boundary_smoothing import SQUARE, bumpy_ring


def write_map(path):
    lawn = core.Area('Lawn', bumpy_ring(SQUARE, 0.06),
                     obstacles=[core.Obstacle([(1.0, 1.0), (2.0, 1.0), (2.0, 2.0)], name='tree')])
    other = core.Area('Back', bumpy_ring([(10.0, 0.0), (14.0, 0.0), (14.0, 3.0), (10.0, 3.0)],
                                         0.06))
    path_area = core.Area('Path 1', bumpy_ring([(6.0, 1.0), (10.0, 1.0), (10.0, 1.7),
                                                (6.0, 1.7)], 0.06), is_navigation=True)
    core.save_areas_file(str(path), [lawn, other, path_area])
    return path.read_text()


def test_proposal_changes_only_the_selected_area(tmp_path, capsys):
    src = tmp_path / 'areas.dat'
    before = write_map(src)
    assert smooth_areas.main([str(src), '--area', 'Lawn']) == 0
    assert src.read_text() == before                            # input untouched
    new, _ = core.load_areas_file(str(tmp_path / 'areas.smoothed.dat'))
    old, _ = core.load_areas_file(str(src))
    assert [a.name for a in new] == ['Lawn', 'Back', 'Path 1']
    assert sorted(new[0].polygon) == sorted(SQUARE)
    assert new[0].obstacles[0].name == 'tree'
    assert new[1].polygon == old[1].polygon and new[2].polygon == old[2].polygon
    rep = json.loads((tmp_path / 'areas.smoothed.json').read_text())
    assert rep['mode'] == 'smooth' and [r['name'] for r in rep['areas']] == ['Lawn']
    r = rep['areas'][0]
    assert r['vertices_after'] == 4 and r['max_deviation_m'] <= 0.08
    assert r['area_change_m2'] < 0
    assert 'Lawn' in capsys.readouterr().out


def test_default_smooths_every_mowing_area_but_no_navigation_area(tmp_path):
    src = tmp_path / 'areas.dat'
    write_map(src)
    out = tmp_path / 'p.dat'
    assert smooth_areas.main([str(src), '--out', str(out), '--tolerance', '0.1']) == 0
    new, _ = core.load_areas_file(str(out))
    old, _ = core.load_areas_file(str(src))
    assert len(new[0].polygon) == 4 and len(new[1].polygon) == 4
    assert new[2].polygon == old[2].polygon
    rep = json.loads((tmp_path / 'p.json').read_text())
    assert rep['tolerance_m'] == 0.1 and len(rep['areas']) == 2


def test_refuses_to_overwrite_the_input_and_unknown_names(tmp_path):
    src = tmp_path / 'areas.dat'
    before = write_map(src)
    assert smooth_areas.main([str(src), '--out', str(src)]) == 2
    assert smooth_areas.main([str(src), '--area', 'Nope']) == 2
    assert smooth_areas.main([str(src), '--area', 'Path 1']) == 2   # not a mowing area
    assert src.read_text() == before
    assert not os.path.exists(tmp_path / 'areas.smoothed.dat')


def test_restore_puts_back_the_recorded_ring(tmp_path):
    src = tmp_path / 'areas.dat'
    write_map(src)
    smooth_areas.main([str(src)])
    recorded = core.load_areas_file(str(src))[0][0].polygon
    smoothed = tmp_path / 'areas.smoothed.dat'
    qdir = tmp_path / 'record_quality'
    qdir.mkdir()
    (qdir / 'Lawn.json').write_text(json.dumps(
        {'name': 'Lawn', 'kind': 'area', 'vertices': [],
         'smoothing': {'recorded_polygon': [list(p) for p in recorded]}}))
    assert smooth_areas.main([str(smoothed), '--restore-from', str(qdir)]) == 0
    back, _ = core.load_areas_file(str(tmp_path / 'areas.restored.dat'))
    assert [tuple(p) for p in back[0].polygon] == [tuple(p) for p in recorded]
    rep = json.loads((tmp_path / 'areas.restored.json').read_text())
    assert rep['mode'] == 'restore'
    assert 'no recorded ring' in rep['areas'][1]['skipped']           # 'Back' has no sidecar
