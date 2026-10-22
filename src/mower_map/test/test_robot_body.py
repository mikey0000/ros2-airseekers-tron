"""Robot body model (mower_map.robot_body; design DESIGN_path_conditioning_2026-10.md
section 7): one source of truth (nav2_params.yaml footprint + padding), body outside."""

import math
import os

import pytest

from mower_map import robot_body as rb

HERE = os.path.dirname(os.path.abspath(__file__))
NAV2 = os.path.join(HERE, '..', '..', 'mower_navigation', 'config', 'nav2_params.yaml')
SQUARE = [[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]]


def test_parse_and_pad_like_nav2():
    poly = rb.parse_footprint('[[0.60, 0.235], [0.60, -0.235], [-0.10, -0.235], [-0.10, 0.235]]')
    assert poly == list(rb.DEFAULT_FOOTPRINT)
    assert rb.pad_footprint(poly, 0.02) == [(0.62, 0.255), (0.62, -0.255), (-0.12, -0.255),
                                            (-0.12, 0.255)]
    assert rb.parse_footprint('') is None and rb.parse_footprint('[[1, 2]]') is None


@pytest.mark.skipif(not os.path.isfile(NAV2), reason='mower_navigation config not in tree')
def test_nav2_params_is_the_source_of_truth():
    fp, pad = rb.nav2_footprint(NAV2)
    assert fp == list(rb.DEFAULT_FOOTPRINT) and pad == pytest.approx(rb.DEFAULT_PADDING)
    # both costmaps carry the same footprint
    assert rb.nav2_footprint(NAV2, 'global_costmap') == (fp, pad)
    b = rb.load_body(nav2_params_file=NAV2)
    assert b.source == NAV2 and b.half_width == pytest.approx(0.255)
    assert rb.load_body('[[0.3, 0.2], [0.3, -0.2], [-0.1, -0.2], [-0.1, 0.2]]', 0.0,
                        NAV2).half_width == pytest.approx(0.2)


def test_body_outside_counts_the_nose_not_only_base_link():
    b = rb.Body(padding=rb.MEASURE_PADDING)
    # base_link 0.05 m inside the right edge, facing it: the nose (0.60 m) is 0.55 m out
    assert rb.body_outside(b, 3.95, 2.0, 0.0, SQUARE) == pytest.approx(0.55, abs=0.01)
    # facing along the edge: the side (0.235 m) is 0.185 m out
    assert rb.body_outside(b, 3.95, 2.0, math.pi / 2, SQUARE) == pytest.approx(0.185, abs=0.01)
    assert rb.body_outside(b, 2.0, 2.0, 0.3, SQUARE) == 0.0


def test_pivot_sweep_reaches_the_corner_radius():
    b = rb.Body(padding=rb.MEASURE_PADDING)
    assert b.radius == pytest.approx(0.645, abs=0.002) and b.rear_radius == pytest.approx(
        0.2556, abs=0.002)
    # 0.5 m from the right edge, heading up, pivot right by 90 deg: a front corner swings
    # through 0.645 m toward the edge -> 0.145 m out; a pivot left keeps the front inside
    assert rb.sweep_clearance(b, 3.5, 2.0, math.pi / 2, 0.0, SQUARE) == \
        pytest.approx(0.5 - b.radius, abs=0.01)
    assert rb.sweep_clearance(b, 3.5, 2.0, math.pi / 2, math.pi, SQUARE) > 0.0


def test_soft_view_is_the_centreline_and_front_sweep():
    b = rb.Body(padding=rb.MEASURE_PADDING).soft()
    # base_link 0.05 m inside the right edge facing along it: the sides overhang, but the
    # soft body (tail..nose on the centreline) is inside
    assert rb.body_outside(b, 3.95, 2.0, math.pi / 2, SQUARE) == 0.0
    # facing the edge: the nose is out
    assert rb.body_outside(b, 3.95, 2.0, 0.0, SQUARE) == pytest.approx(0.55, abs=0.01)
    # pivot sweep: only the front half (radius 0.60) counts
    assert rb.sweep_clearance(b, 3.5, 2.0, math.pi / 2, 0.0, SQUARE) == \
        pytest.approx(0.5 - 0.60, abs=0.01)


def test_edge_mode_is_an_area_setting():
    from mower_map import area_settings as aset
    assert aset.BUILTIN_DEFAULTS['edge_mode'] == 'soft'
    ok, _msg, d = aset.validate({'edge_mode': 'hard'})
    assert ok and d['edge_mode'] == 'hard'
    ok, _msg, d = aset.validate({'edge_mode': 'soft'})
    assert ok and d['edge_mode'] == 'soft'
    assert not aset.validate({'edge_mode': 'wobbly'})[0]
