# Copyright 2026 michael
# SPDX-License-Identifier: Apache-2.0
"""PlanCoverage blade_offset_m: the bridge forwards base_link -> blade_link from /tf_static."""

import pytest

from mower_coverage_bridge.coverage_server_node import planner_blade_offset


def test_blade_offset_is_the_urdf_blade_link_x():
    # config/urdf/mower.urdf.xacro blade_joint: cutter_xyz 0.23 0 -0.05
    assert planner_blade_offset({'blade_link': ('base_link', 0.23, 0.0, 0.0)}) == \
        pytest.approx(0.23)
    # chained through another static frame
    st = {'blade_link': ('deck', 0.03, 0.0, 0.0), 'deck': ('base_link', 0.20, 0.0, 0.0)}
    assert planner_blade_offset(st) == pytest.approx(0.23)


def test_unknown_blade_link_is_sent_as_negative():
    # the planner then keeps the base_link-referenced row ends (PlanCoverage.srv)
    assert planner_blade_offset({}) == -1.0
    assert planner_blade_offset({'blade_link': ('odom', 0.23, 0.0, 0.0)}) == -1.0
