#!/usr/bin/env python3
# Copyright 2026 ROS 2 port team
# SPDX-License-Identifier: Apache-2.0
"""Static checks on config/nav2_params.yaml. Runs with plain python3 + pyyaml:

    python3 src/mower_navigation/test/test_params_yaml.py

(also collected by pytest). Checks:
  * every *_plugins / *_plugin / plugins / filters entry has a sub-map with a
    `plugin` key (a missing `.plugin` is the "plugin param not defined" failure);
  * frame names match the stack contract;
  * the controller / goal-checker / planner ids the MowgliNext BT requests exist;
  * static_layer only on the keepout mask, no enable_stamped_cmd_vel (Humble);
  * progress/goal checker, transit RPP and local-costmap layer choices that
    the first real mow (2026-10-06) showed matter;
  * lifecycle_manager_navigation node order.
"""
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = os.path.join(HERE, "..", "config", "nav2_params.yaml")

PLUGIN_LIST_KEYS = ("controller_plugins", "goal_checker_plugins", "planner_plugins",
                    "behavior_plugins", "plugins", "filters", "progress_checker_plugins")
PLUGIN_SINGLE_KEYS = ("progress_checker_plugin",)
ALLOWED_FRAMES = {"map", "odom", "base_link", "base_footprint"}


def load():
    with open(PARAMS) as f:
        return yaml.safe_load(f)


def ros_param_blocks(doc, path=()):
    """Yield (path, params) for every ros__parameters mapping."""
    for key, val in doc.items():
        if not isinstance(val, dict):
            continue
        if "ros__parameters" in val:
            yield path + (key,), val["ros__parameters"]
        else:
            yield from ros_param_blocks(val, path + (key,))


def walk(d, path=()):
    for k, v in d.items():
        yield path + (k,), k, v
        if isinstance(v, dict):
            yield from walk(v, path + (k,))


def test_every_plugin_has_type():
    errors = []
    for node, params in ros_param_blocks(load()):
        for path, key, val in walk(params):
            parent = params
            for p in path[:-1]:
                parent = parent[p]
            names = []
            if key in PLUGIN_LIST_KEYS:
                names = val if isinstance(val, list) else [val]
            elif key in PLUGIN_SINGLE_KEYS:
                names = [val]
            for name in names:
                entry = parent.get(name)
                if not isinstance(entry, dict) or not entry.get("plugin"):
                    errors.append(f"{'/'.join(node + path)}: '{name}' has no .plugin")
    assert not errors, "\n".join(errors)


def test_frames():
    doc = load()
    errors = []
    for node, params in ros_param_blocks(doc):
        for path, key, val in walk(params):
            if key.endswith("_frame") or key.endswith("_frame_id"):
                if val not in ALLOWED_FRAMES:
                    errors.append(f"{'/'.join(node + path)} = {val!r}")
    assert not errors, "unexpected frames:\n" + "\n".join(errors)
    g = doc["global_costmap"]["global_costmap"]["ros__parameters"]
    lc = doc["local_costmap"]["local_costmap"]["ros__parameters"]
    assert g["global_frame"] == "map"
    assert lc["global_frame"] == "odom"
    assert doc["bt_navigator"]["ros__parameters"]["global_frame"] == "map"
    for n in ("bt_navigator", "controller_server", "velocity_smoother"):
        assert doc[n]["ros__parameters"]["odom_topic"] == "/odometry/filtered", n


def test_contract_ids():
    doc = load()
    c = doc["controller_server"]["ros__parameters"]
    assert set(c["controller_plugins"]) >= {"FollowPath", "FollowCoveragePath"}
    assert set(c["goal_checker_plugins"]) >= {"general_goal_checker", "coverage_goal_checker"}
    assert doc["planner_server"]["ros__parameters"]["planner_plugins"] == ["GridBased"]


def test_humble_constraints():
    doc = load()
    text = open(PARAMS).read()
    for _, params in ros_param_blocks(doc):
        assert "enable_stamped_cmd_vel" not in params
    # No /map: a StaticLayer is only allowed on the keepout mask (global costmap).
    g = doc["global_costmap"]["global_costmap"]["ros__parameters"]
    if "static_layer" in g.get("plugins", []):
        assert g["static_layer"]["map_topic"] in ("/nav_keepout_mask", "/keepout_mask")
    assert text.count("nav2_costmap_2d::StaticLayer") <= 1
    assert "map_server" not in doc and "amcl" not in doc


def test_progress_checker_counts_rotation():
    c = load()["controller_server"]["ros__parameters"]
    pc = c[c["progress_checker_plugin"]]
    assert pc["plugin"] == "nav2_controller::PoseProgressChecker"
    assert 0 < pc["required_movement_angle"] <= 0.6
    assert pc["required_movement_radius"] <= 0.3
    assert pc["movement_time_allowance"] >= 15.0


def test_goal_checkers():
    c = load()["controller_server"]["ros__parameters"]
    g = c["general_goal_checker"]
    assert g["plugin"] == "nav2_controller::SimpleGoalChecker"
    assert g["stateful"] is True
    assert g["xy_goal_tolerance"] == 0.25 and g["yaw_goal_tolerance"] == 0.5
    cov = c["coverage_goal_checker"]
    assert cov["stateful"] is True and cov["xy_goal_tolerance"] <= 0.15


def test_transit_rpp_rotation():
    rpp = load()["controller_server"]["ros__parameters"]["FollowPath"]
    assert rpp["use_rotate_to_heading"] is True
    assert rpp["rotate_to_heading_angular_vel"] == 0.5
    assert 0.5 <= rpp["max_angular_accel"] <= 1.5
    assert rpp["min_approach_linear_velocity"] == 0.05
    # RPP can only rotate to heading without reversing.
    assert rpp["allow_reversing"] is False


def test_local_costmap_has_no_inflation_layer():
    # Humble 1.1.x: InflationLayer stays !isCurrent() after a ClearEntireCostmap
    # when there are no obstacles, and controller_server then waits forever.
    lc = load()["local_costmap"]["local_costmap"]["ros__parameters"]
    assert "inflation_layer" not in lc["plugins"]
    for name in lc["plugins"]:
        assert lc[name]["plugin"] != "nav2_costmap_2d::InflationLayer"


def test_lifecycle_order():
    lm = load()["lifecycle_manager_navigation"]["ros__parameters"]
    assert lm["node_names"] == ["controller_server", "planner_server", "behavior_server",
                                "bt_navigator", "velocity_smoother"]


if __name__ == "__main__":
    failed = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except AssertionError as e:
                failed += 1
                print(f"FAIL {name}: {e}")
    sys.exit(1 if failed else 0)
