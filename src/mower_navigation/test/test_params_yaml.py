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
ALLOWED_FRAMES = {"map", "odom", "base_link", "base_footprint",
                  "stereo_camera_optical"}  # URDF sensor frame (obstacle source)


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
    # nav mask static_layer + terrain_layer (/map_server_node/terrain_cost), nothing else
    assert text.count("nav2_costmap_2d::StaticLayer") <= 2
    if "terrain_layer" in g.get("plugins", []):
        assert g["terrain_layer"]["map_topic"] == "/map_server_node/terrain_cost"
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
    cov = c["coverage_goal_checker"]        # for the RPP coverage controller (fallback)
    assert cov["plugin"] == "nav2_controller::SimpleGoalChecker"
    assert cov["stateful"] is True and cov["xy_goal_tolerance"] <= 0.25
    ftc = c["coverage_goal_checker_ftc"]    # MowgliNext PathProgressGoalChecker for FTC
    assert ftc["plugin"] == "mowgli_nav2_plugins/PathProgressGoalChecker"
    assert 0.9 <= ftc["progress_threshold"] <= 1.0
    assert ftc["plan_topic"] == "/controller_server/FollowCoveragePathFTC/global_plan"
    assert c["FollowCoveragePath"]["plugin"].endswith("RegulatedPurePursuitController")
    assert c["FollowCoveragePathFTC"]["plugin"] == "mowgli_nav2_plugins/FTCController"
    assert set(c["controller_plugins"]) >= {"FollowPath", "FollowCoveragePath", "FollowCoveragePathFTC"}


def test_transit_rpp_rotation():
    rpp = load()["controller_server"]["ros__parameters"]["FollowPath"]
    assert rpp["use_rotate_to_heading"] is True
    assert rpp["rotate_to_heading_angular_vel"] == 0.5
    assert 0.5 <= rpp["max_angular_accel"] <= 4.0   # 3.2 since 2026-10-06 (stalled pivots)
    assert rpp["min_approach_linear_velocity"] == 0.05
    # RPP can only rotate to heading without reversing.
    assert rpp["allow_reversing"] is False


def test_coverage_rpp_respects_drive_yaw_limit():
    # Ring drift 10-06: RPP must regulate speed so v / R stays within what the
    # drive delivers in turf (~0.2 rad/s; MCU clamps each axis at 0.3 separately).
    c = load()["controller_server"]["ros__parameters"]["FollowCoveragePath"]
    yaw_rate = c["desired_linear_vel"] / c["regulated_linear_scaling_min_radius"]
    assert c["use_regulated_linear_velocity_scaling"] is True
    assert yaw_rate <= 0.21
    assert c["rotate_to_heading_angular_vel"] <= 0.3
    assert c["min_lookahead_dist"] <= c["lookahead_dist"] <= c["max_lookahead_dist"]


def test_local_costmap_has_no_inflation_layer():
    # Humble 1.1.x: InflationLayer stays !isCurrent() after a ClearEntireCostmap
    # when there are no obstacles, and controller_server then waits forever.
    lc = load()["local_costmap"]["local_costmap"]["ros__parameters"]
    assert "inflation_layer" not in lc["plugins"]
    for name in lc["plugins"]:
        assert lc[name]["plugin"] != "nav2_costmap_2d::InflationLayer"


def test_det_range_obstacle_source():
    # det_range centroid cloud: marking-only PointCloud2 source, 3 m max range.
    ol = load()["local_costmap"]["local_costmap"]["ros__parameters"]["obstacle_layer"]
    assert "det_range" in ol["observation_sources"].split()
    src = ol["det_range"]
    assert src["topic"] == "/ai/det/obstacle_points"
    assert src["data_type"] == "PointCloud2"
    assert src["marking"] is True and src["clearing"] is False
    assert src["obstacle_max_range"] == 3.0
    for name in ol["observation_sources"].split():
        assert name in ol, f"observation source {name} has no sub-map"


def test_stereo_obstacle_source():
    # 2026-10-06: the stereo cloud marked the lawn lethal ("collision ahead" abort).
    ol = load()["local_costmap"]["local_costmap"]["ros__parameters"]["obstacle_layer"]
    assert "stereo" in ol["observation_sources"].split()
    src = ol["stereo"]
    assert src["topic"] == "/stereo_depth/points"
    assert src["sensor_frame"] == "stereo_camera_optical"
    assert src["min_obstacle_height"] == 0.08   # 0.08 since the 2026-10-06 box test
    assert src["max_obstacle_height"] == 1.2
    assert src["obstacle_max_range"] == 2.0
    assert src["raytrace_max_range"] >= src["obstacle_max_range"]
    assert src["observation_persistence"] == 0.7   # 0.7 since the 2026-10-07 stale-cloud fix
    assert src["expected_update_rate"] == 1.5   # 1.5 s since the 2026-10-07 stale-cloud fix
    assert src["inf_is_valid"] is False
    assert src["marking"] is True and src["clearing"] is True
    # the layer-wide cap must not cut the source's own band
    assert ol["max_obstacle_height"] >= src["max_obstacle_height"]


def _inscribed_radius(gp):
    pts = yaml.safe_load(gp["footprint"])
    pad = gp.get("footprint_padding", 0.0)
    return min(min(abs(x) for x, _ in pts), min(abs(y) for _, y in pts)) + pad


def test_global_inflation_covers_inscribed_radius():
    gp = load()["global_costmap"]["global_costmap"]["ros__parameters"]
    infl = gp["inflation_layer"]
    assert infl["inflation_radius"] == 0.26
    # strictly larger: Nav2 still warns at exactly the (float) inscribed radius
    assert infl["inflation_radius"] > _inscribed_radius(gp) + 1e-3
    # steep decay: keepout / nav-mask behaviour unchanged past the inscribed band
    assert infl["cost_scaling_factor"] >= 20.0


def test_stereo_costmap_off_is_bumper_only():
    sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "launch"))
    import robot_settings
    doc = robot_settings.without_stereo_sources(load())
    ol = doc["local_costmap"]["local_costmap"]["ros__parameters"]["obstacle_layer"]
    assert ol["observation_sources"].split() == ["bumper"]
    # original untouched
    assert "stereo" in load()["local_costmap"]["local_costmap"]["ros__parameters"][
        "obstacle_layer"]["observation_sources"].split()


def test_nav_mask_static_layer_keeps_soft_band():
    """mower_map soft band = 90: must not be trinary-collapsed to FREE."""
    g = load()["global_costmap"]["global_costmap"]["ros__parameters"]
    st = g["static_layer"]
    assert st["map_topic"] == "/nav_keepout_mask"
    assert st["trinary_costmap"] is False
    assert st.get("lethal_cost_threshold", 100) == 100
    assert st.get("unknown_cost_value", -1) == -1
    lethal = st.get("lethal_cost_threshold", 100)
    assert 0 < round(90 / lethal * 254) < 253          # soft band: plannable, not inscribed
    # the local costmap has no nav-mask layer (bumper/stereo obstacle_layer only)
    loc = load()["local_costmap"]["local_costmap"]["ros__parameters"]
    assert "static_layer" not in loc["plugins"]


def test_controller_failure_tolerance_survives_short_blockage():
    c = load()["controller_server"]["ros__parameters"]
    assert c["failure_tolerance"] >= 2.0


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
