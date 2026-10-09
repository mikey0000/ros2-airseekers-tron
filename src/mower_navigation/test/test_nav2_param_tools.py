#!/usr/bin/env python3
# Copyright 2026 ROS 2 port team
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for launch/nav2_param_tools.py (transit_controller rewrite). Pure python."""
import copy
import os
import sys

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "launch"))
import nav2_param_tools  # noqa: E402

PARAMS = os.path.join(HERE, "..", "config", "nav2_params.yaml")


def load():
    with open(PARAMS) as f:
        return yaml.safe_load(f)


def _cp(doc):
    return doc["controller_server"]["ros__parameters"]


def test_mppi_without_speed_is_identity():
    doc = load()
    assert nav2_param_tools.apply_transit_controller(doc, "mppi") == doc


def test_mppi_maps_transit_speed_to_vx_max():
    doc = load()
    _cp(doc)["FollowPath"]["desired_linear_vel"] = 0.25
    fp = _cp(nav2_param_tools.apply_transit_controller(doc, "mppi"))["FollowPath"]
    assert fp["vx_max"] == 0.25
    assert "desired_linear_vel" not in fp


def test_mppi_speed_capped_at_mcu_limit():
    doc = load()
    _cp(doc)["FollowPath"]["desired_linear_vel"] = 0.5
    fp = _cp(nav2_param_tools.apply_transit_controller(doc, "mppi"))["FollowPath"]
    assert fp["vx_max"] == 0.3      # the MCU clamps linear at 0.3
    assert "desired_linear_vel" not in fp


def test_rpp_swaps_in_followpathrpp_block():
    doc = load()
    out = _cp(nav2_param_tools.apply_transit_controller(doc, "rpp"))
    assert out["FollowPath"] == _cp(doc)["FollowPathRPP"]
    assert out["FollowPath"]["plugin"].endswith("RegulatedPurePursuitController")


def test_rpp_keeps_transit_speed_override():
    doc = load()
    _cp(doc)["FollowPath"]["desired_linear_vel"] = 0.22
    fp = _cp(nav2_param_tools.apply_transit_controller(doc, "rpp"))["FollowPath"]
    assert fp["plugin"].endswith("RegulatedPurePursuitController")
    assert fp["desired_linear_vel"] == 0.22


def test_mode_is_case_and_space_insensitive():
    doc = load()
    out = nav2_param_tools.apply_transit_controller(doc, "RPP ")
    assert _cp(out)["FollowPath"] == _cp(doc)["FollowPathRPP"]


def test_unknown_mode_raises():
    with pytest.raises(ValueError):
        nav2_param_tools.apply_transit_controller(load(), "dwb")


def test_input_never_mutated():
    for mode in ("mppi", "rpp"):
        doc = load()
        _cp(doc)["FollowPath"]["desired_linear_vel"] = 0.22
        before = copy.deepcopy(doc)
        nav2_param_tools.apply_transit_controller(doc, mode)
        assert doc == before, mode


def test_doc_without_controller_server_unchanged():
    doc = {"planner_server": {"ros__parameters": {"x": 1}}}
    for mode in ("mppi", "rpp"):
        assert nav2_param_tools.apply_transit_controller(doc, mode) == doc
