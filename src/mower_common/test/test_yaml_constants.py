"""The YAML parameter files keep literal geometry values; they must equal mower_common.geometry
(code review 2026-10-23 stage 0.3: track 0.40 was in 8 places, the footprint in ~12)."""

import ast
import os

import pytest
import yaml

from mower_common import geometry as g

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')


def _load(pkg, name):
    with open(os.path.join(SRC, pkg, 'config', name)) as f:
        return yaml.safe_load(f)


def _ros(doc):
    """Every ros__parameters dict of a document (any nesting)."""
    out = []

    def walk(d):
        if isinstance(d, dict):
            for k, v in d.items():
                if k == 'ros__parameters' and isinstance(v, dict):
                    out.append(v)
                walk(v)
    walk(doc)
    return out


def _find(doc, key):
    return [p[key] for p in _ros(doc) if key in p]


def _approx_fp(fp):
    return [pytest.approx(list(c)) for c in fp]


def test_nav2_footprints_and_wheel_floor_track():
    doc = _load('mower_navigation', 'nav2_params.yaml')
    fps = []

    def walk(d):
        if isinstance(d, dict):
            for k, v in d.items():
                if k == 'footprint':
                    fps.append(ast.literal_eval(v))
                elif k == 'WheelFloorCritic':
                    assert v['track_width'] == pytest.approx(g.TRACK_SPLIT_M)
                walk(v)
    walk(doc)
    assert len(fps) >= 2
    for fp in fps:
        assert fp == _approx_fp(g.FOOTPRINT)


def test_mission_yaml():
    p = _ros(_load('mower_mission', 'mission.yaml'))[0]
    assert p['robot_front_m'] == pytest.approx(g.FOOTPRINT_FRONT_M)
    assert p['robot_width_m'] == pytest.approx(g.FOOTPRINT_WIDTH_M)
    # deliberate 0.02 margin behind the 0.10 rear (= obstacle_guard rear_edge_x_m)
    assert p['robot_rear_edge_m'] == pytest.approx(g.FOOTPRINT_REAR_M + 0.02)
    assert p['path_conditioning_half_width_m'] == pytest.approx(g.FOOTPRINT_HALF_WIDTH_M)
    assert p['hard_boundary_track_width_m'] == pytest.approx(g.TRACK_SPLIT_M)


def test_obstacle_guard_yaml():
    doc = _load('mower_vision', 'obstacle_guard.yaml')
    assert _find(doc, 'rear_half_width_m') == [pytest.approx(g.FOOTPRINT_HALF_WIDTH_M)]
    assert _find(doc, 'rear_edge_x_m') == [pytest.approx(-(g.FOOTPRINT_REAR_M + 0.02))]


def test_docking_yaml_marker():
    assert _find(_load('mower_docking', 'docking.yaml'), 'marker_size') == \
        [pytest.approx(g.MARKER_SIZE_M)]


def test_footprint_derived():
    assert g.FOOTPRINT[0] == (g.FOOTPRINT_FRONT_M, g.FOOTPRINT_HALF_WIDTH_M)
    assert g.FOOTPRINT[2] == (-g.FOOTPRINT_REAR_M, -g.FOOTPRINT_HALF_WIDTH_M)
    assert g.FOOTPRINT_WIDTH_M == 0.47
