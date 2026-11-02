"""Review 2026-11-02 H5: config/map_server.yaml is the single source of every map_server
parameter value; map_server_node.PARAMS must equal it. Fails on any future drift."""

import pytest

from mower_common.shipped_params import drift, shipped_params


def test_map_server_params_match_the_shipped_yaml():
    ms = pytest.importorskip('mower_map.map_server_node')
    shipped = shipped_params('mower_map', 'map_server.yaml', 'map_server_node')
    assert shipped
    assert drift(ms.PARAMS, shipped) == {}
    assert sorted(set(shipped) - set(ms.PARAMS) - {'use_sim_time'}) == []


def test_field_values_review_2026_11_02():
    ms = pytest.importorskip('mower_map.map_server_node')
    assert (ms.PARAMS['nav_margin_m'], ms.PARAMS['boundary_check_rate_hz']) == (0.05, 10.0)
