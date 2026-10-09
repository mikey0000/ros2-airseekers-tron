# SPDX-License-Identifier: Apache-2.0
"""Tests for mower_sim.maps.write_maps (needs mower_map)."""

import os

import pytest

pytest.importorskip('mower_map')

from mower_map import areas as A  # noqa: E402

from mower_sim.maps import write_maps  # noqa: E402
from mower_sim.world import load_world  # noqa: E402

WORLDS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'worlds'))


def _write(name, tmp_path):
    w = load_world(os.path.join(WORLDS, name + '.yaml'))
    out = tmp_path / 'maps'
    write_maps(w, str(out))
    areas, datum = A.load_areas_file(str(out / 'areas.dat'))
    return w, out, areas, datum


def test_transit_box_one_area_no_obstacles(tmp_path):
    w, out, areas, datum = _write('transit_box', tmp_path)
    assert len(areas) == 1
    assert areas[0].polygon == pytest.approx(w.lawn)
    assert areas[0].obstacles == []
    assert datum == pytest.approx((48.0, 9.0))


def test_garden_writes_only_known_obstacle(tmp_path):
    w, out, areas, _ = _write('garden', tmp_path)
    assert len(areas) == 1
    obs = areas[0].obstacles
    assert [o.name for o in obs] == ['flower_bed']
    bed = next(o for o in w.obstacles if o.name == 'flower_bed')
    assert obs[0].polygon == pytest.approx(bed.polygon(0.0), abs=1e-3)


def test_dock_pose_yaml(tmp_path):
    w, out, _, _ = _write('garden', tmp_path)
    dock = A.load_dock_file(str(out / 'dock_pose.yaml'))
    assert dock.measured is True
    assert (dock.x, dock.y, dock.yaw) == pytest.approx((w.dock.x, w.dock.y, w.dock.yaw))
    assert dock.outline and len(dock.outline) == 4


def test_extra_areas_written_after_lawn(tmp_path):
    from mower_sim.world import world_from_dict
    w = world_from_dict({'lawn': [[0, 0], [5, 0], [5, 5], [0, 5]],
                         'areas': [{'name': 'path', 'polygon': [[5, 1], [9, 1], [9, 2], [5, 2]],
                                    'is_navigation': True},
                                   {'name': 'bad', 'polygon': [[0, 0], [1, 1]]}]})
    write_maps(w, str(tmp_path))
    areas, _ = A.load_areas_file(str(tmp_path / 'areas.dat'))
    assert [a.name for a in areas] == ['lawn', 'path']
    assert areas[1].is_navigation
