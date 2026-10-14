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


def _robot_map(path):
    """A small areas.dat as the robot writes it: two mowing areas, one drawn path."""
    areas = [
        A.Area(name='Back', polygon=[(-9, 0), (-6, 0), (-6, 3), (-9, 3)]),
        A.Area(name='Front', polygon=[(0, 0), (4, 0), (4, 4), (0, 4)],
               obstacles=[A.Obstacle(polygon=[(1, 1), (1.5, 1), (1.5, 1.5), (1, 1.5)])]),
        A.Area(name='Dock path', polygon=[(-1, -0.3), (0.5, -0.3), (0.5, 0.3), (-1, 0.3)],
               is_navigation=True, channel=[(-0.8, 0.0), (0.3, 0.0)], channel_width_m=0.6),
    ]
    A.save_areas_file(str(path), areas, -38.0, 175.3)


def test_areas_file_selects_lawn_and_paths(tmp_path):
    _robot_map(tmp_path / 'robot_areas.dat')
    (tmp_path / 'w.yaml').write_text(
        'name: real\n'
        'datum: {lat: 48.0, lon: 9.0}\n'
        'areas_file: robot_areas.dat\n'          # relative to the world file
        'areas_select: [Front, Dock path]\n'
        'dock: {x: 0.2, y: -0.1, yaw: 1.5}\n')
    w = load_world(str(tmp_path / 'w.yaml'))
    assert w.lawn_name == 'Front'
    assert w.lawn == pytest.approx([(0, 0), (4, 0), (4, 4), (0, 4)])
    assert w.obstacles == []                     # map obstacles are not physical
    write_maps(w, str(tmp_path / 'maps'))
    areas, datum = A.load_areas_file(str(tmp_path / 'maps' / 'areas.dat'))
    assert datum == pytest.approx((48.0, 9.0))   # the world's datum, not the file's
    assert [a.name for a in areas] == ['Front', 'Dock path']
    assert not areas[0].is_navigation and areas[1].is_navigation
    assert len(areas[0].obstacles) == 1
    assert [c for p in areas[1].channel for c in p] == pytest.approx([-0.8, 0.0, 0.3, 0.0])
    assert areas[1].channel_width_m == pytest.approx(0.6)
    dock = A.load_dock_file(str(tmp_path / 'maps' / 'dock_pose.yaml'))
    assert (dock.x, dock.y, dock.yaw) == pytest.approx((0.2, -0.1, 1.5))


def test_areas_file_default_lawn_is_first_mowing_area(tmp_path):
    from mower_sim.world import world_from_dict
    _robot_map(tmp_path / 'a.dat')
    w = world_from_dict({'areas_file': str(tmp_path / 'a.dat')})
    assert w.lawn_name == 'Back'
    assert [a['name'] for a in w.extra_areas] == ['Front', 'Dock path']


def test_areas_file_errors(tmp_path):
    from mower_sim.world import world_from_dict
    _robot_map(tmp_path / 'a.dat')
    f = str(tmp_path / 'a.dat')
    with pytest.raises(ValueError, match='not in'):
        world_from_dict({'areas_file': f, 'areas_select': ['Front', 'Nope']})
    with pytest.raises(ValueError, match='lawn_area'):
        world_from_dict({'areas_file': f, 'areas_select': ['Front'], 'lawn_area': 'Back'})
    with pytest.raises(ValueError, match='no mowing area'):
        world_from_dict({'areas_file': f, 'areas_select': ['Dock path']})
    with pytest.raises(ValueError, match='excludes'):
        world_from_dict({'areas_file': f, 'lawn': [[0, 0], [1, 0], [1, 1]]})
