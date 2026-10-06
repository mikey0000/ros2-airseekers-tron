# SPDX-License-Identifier: GPL-3.0-or-later
"""Per-area mowing settings store (no ROS)."""

import json

import pytest

from mower_map import area_settings as s

SQ = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0)]
TRI = [(10.0, 0.0), (15.0, 0.0), (12.0, 4.0)]


def test_builtin_defaults_match_the_contract():
    assert s.BUILTIN_DEFAULTS == {
        'cutter_height_mm': 50, 'perimeter_laps': 2, 'path_mode': 'zigzag',
        'mow_angle_deg': -1.0, 'cut_speed_mps': 0.3, 'swath_overlap_m': 0.02,
        'swath_width_m': 0.18, 'edge_margin_m': 0.05, 'edge_first': True, 'repeat': 1, 'alternate_angle_offset_deg': 90.0,
        'route_order': 'racetrack', 'route_spiral_size': 6, 'min_turn_radius_m': 0.5,
        'turn_type': 'auto', 'obstacle_detection': 'standard',
        'blade_policy': 'continuous',
        'slope_mode': 'off', 'slope_contour_above_deg': 10.0}
    assert s.SLOPE_MODES == ('off', 'auto', 'contour', 'updown')
    assert s.ROUTE_ORDERS == ('boustrophedon', 'snake', 'spiral', 'racetrack')
    assert s.TURN_TYPES == ('auto', 'loop', 'reverse', 'pivot')
    assert s.DEFAULTS_INDEX == 255
    assert s.PATH_MODES == ('zigzag', 'cross', 'alternate', 'spiral', 'contour_only')


@pytest.mark.parametrize('key,value,expect', [
    ('cutter_height_mm', 30, 30), ('cutter_height_mm', 90, 90), ('cutter_height_mm', 60.0, 60),
    ('perimeter_laps', 0, 0), ('perimeter_laps', 4, 4),
    ('path_mode', 'spiral', 'spiral'), ('path_mode', 'contour_only', 'contour_only'),
    ('mow_angle_deg', -1, -1.0), ('mow_angle_deg', -5.0, -1.0), ('mow_angle_deg', 45, 45.0),
    ('mow_angle_deg', 270.0, 90.0),
    ('cut_speed_mps', 0.5, 0.5), ('swath_overlap_m', 0.0, 0.0),
    ('edge_first', False, False), ('repeat', 3, 3),
    ('alternate_angle_offset_deg', 45, 45.0),
    ('route_order', 'boustrophedon', 'boustrophedon'), ('route_order', 'spiral', 'spiral'),
    ('route_spiral_size', 2, 2), ('route_spiral_size', 8.0, 8),
    ('min_turn_radius_m', 0, 0.0), ('min_turn_radius_m', 0.5, 0.5),
    ('turn_type', 'reverse', 'reverse'), ('turn_type', 'pivot', 'pivot'),
    ('blade_policy', 'continuous', 'continuous'), ('blade_policy', 'conservative', 'conservative'),
    ('slope_mode', 'auto', 'auto'), ('slope_mode', 'contour', 'contour'),
    ('slope_contour_above_deg', 12, 12.0),
    ('swath_width_m', 0.10, 0.10), ('swath_width_m', 0.4, 0.4),
    ('swath_width_m', 0.18, 0.18), ('edge_margin_m', 0, 0.0), ('edge_margin_m', 0.5, 0.5),
])
def test_valid_values_are_normalised(key, value, expect):
    ok, v = s.validate_value(key, value)
    assert ok and v == expect and type(v) is type(expect)


@pytest.mark.parametrize('key,value', [
    ('cutter_height_mm', 29), ('cutter_height_mm', 91), ('cutter_height_mm', 50.5),
    ('cutter_height_mm', '50'), ('cutter_height_mm', True),
    ('perimeter_laps', 5), ('perimeter_laps', -1),
    ('path_mode', 'diagonal'), ('path_mode', 3),
    ('cut_speed_mps', 0.51), ('cut_speed_mps', 0.0), ('swath_overlap_m', -0.01),
    ('edge_first', 1), ('repeat', 0), ('alternate_angle_offset_deg', 181.0),
    ('mow_angle_deg', float('nan')), ('bogus', 1),
    ('route_order', 'zigzag'), ('route_spiral_size', 1), ('route_spiral_size', 21),
    ('min_turn_radius_m', -0.1), ('min_turn_radius_m', 2.5), ('turn_type', 'omega'),
    ('blade_policy', 'always'), ('blade_policy', True),
    ('swath_width_m', 0.05), ('swath_width_m', 0.41), ('swath_width_m', 0),
    ('edge_margin_m', -0.01), ('edge_margin_m', 0.51),
])
def test_invalid_values_are_rejected(key, value):
    ok, msg = s.validate_value(key, value)
    assert not ok and isinstance(msg, str)


def test_parse_json():
    assert s.parse_json('{"path_mode": "spiral", "repeat": 2}') == \
        (True, 'ok', {'path_mode': 'spiral', 'repeat': 2})
    assert s.parse_json('')[0] and s.parse_json('')[2] == {}
    assert not s.parse_json('{"path_mode": ')[0]
    assert not s.parse_json('[1, 2]')[0]
    assert not s.parse_json('{"repeat": 99}')[0]
    assert s.parse_json('{"repeat": null}')[2] == {'repeat': None}


def test_effective_merges_area_over_defaults_over_builtin():
    st = s.AreaSettingsStore()
    st.set_defaults({'cutter_height_mm': 60, 'repeat': 2})
    st.set_area('Front', {'repeat': 3, 'path_mode': 'spiral'})
    eff = st.effective('Front')
    assert eff['cutter_height_mm'] == 60 and eff['repeat'] == 3
    assert eff['path_mode'] == 'spiral' and eff['perimeter_laps'] == 2
    assert st.effective('Unknown') == dict(s.BUILTIN_DEFAULTS, cutter_height_mm=60, repeat=2)
    assert set(eff) == set(s.BUILTIN_DEFAULTS)


def test_set_merges_null_removes_empty_object_resets():
    st = s.AreaSettingsStore()
    st.set_area('A', {'repeat': 3, 'edge_first': False})
    st.set_area('A', {'cutter_height_mm': 40})
    assert st.areas['A'] == {'repeat': 3, 'edge_first': False, 'cutter_height_mm': 40}
    st.set_area('A', {'repeat': None})
    assert st.areas['A'] == {'edge_first': False, 'cutter_height_mm': 40}
    st.set_area('A', {})
    assert 'A' not in st.areas
    st.set_defaults({'repeat': 2})
    st.set_defaults({})
    assert st.defaults == {}


def test_snapshot_lists_every_area_and_full_defaults():
    st = s.AreaSettingsStore()
    st.set_area('A', {'path_mode': 'cross'})
    snap = json.loads(st.snapshot_json(['A', 'B']))
    assert snap == {'defaults': s.BUILTIN_DEFAULTS,
                    'areas': {'A': {'path_mode': 'cross'}, 'B': {}}}


def test_rename_drop_prune():
    st = s.AreaSettingsStore()
    st.set_area('A', {'repeat': 2})
    st.set_area('B', {'repeat': 3})
    assert st.rename('A', 'A2') and st.areas == {'A2': {'repeat': 2}, 'B': {'repeat': 3}}
    assert not st.rename('missing', 'X')
    assert st.drop('B') and not st.drop('B')
    st.set_area('C', {'repeat': 4})
    assert st.prune(['A2']) == ['C'] and list(st.areas) == ['A2']


def test_carry_over_moves_a_renamed_area_and_leaves_deleted_ones():
    st = s.AreaSettingsStore()
    st.set_area('Front', {'path_mode': 'spiral'})
    st.set_area('Back', {'repeat': 2})
    before = [('Front', SQ), ('Back', TRI)]
    after = [('Lawn', [(x + 0.001, y) for x, y in SQ])]   # renamed, Back deleted
    moved = st.carry_over(before, after)
    assert moved == [('Front', 'Lawn')]
    assert st.areas['Lawn'] == {'path_mode': 'spiral'}
    assert st.prune(['Lawn']) == ['Back']


def test_carry_over_needs_the_same_polygon():
    st = s.AreaSettingsStore()
    st.set_area('Front', {'repeat': 2})
    moved = st.carry_over([('Front', SQ)], [('Other', TRI)])
    assert moved == [] and 'Other' not in st.areas


def test_yaml_round_trip_and_bad_entries(tmp_path):
    st = s.AreaSettingsStore()
    st.set_defaults({'cutter_height_mm': 70})
    st.set_area('Zone 1', {'path_mode': 'contour_only', 'perimeter_laps': 3})
    st.set_area('Zoné ü', {'edge_first': False})
    path = str(tmp_path / 'maps' / s.FILE_NAME)
    s.save_file(path, st)
    back, warnings = s.load_file(path)
    assert warnings == []
    assert back.defaults == st.defaults and back.areas == st.areas
    text = ('version: 1\ndefaults: {repeat: 0}\nareas:\n  ok: {repeat: 2}\n'
            '  bad: {path_mode: diagonal}\n')
    back, warnings = s.AreaSettingsStore.loads(text)
    assert back.defaults == {} and back.areas == {'ok': {'repeat': 2}}
    assert len(warnings) == 2
    missing, warnings = s.load_file(str(tmp_path / 'nope.yaml'))
    assert missing.areas == {} and warnings == []


def test_settings_path_is_next_to_areas_dat():
    assert s.settings_path_for('/ros2_ws/maps/areas.dat') == '/ros2_ws/maps/area_settings.yaml'


def test_old_file_with_only_overlap_gets_default_path_width():
    store, warnings = s.AreaSettingsStore.loads(
        "version: 1\nareas:\n  Old: {swath_overlap_m: 0.05}\n  New: {swath_width_m: 0.3}\n")
    assert warnings == []
    assert store.effective('Old')['swath_width_m'] == 0.18
    assert store.effective('Old')['swath_overlap_m'] == 0.05   # still accepted, unused
    assert store.effective('New')['swath_width_m'] == 0.3
