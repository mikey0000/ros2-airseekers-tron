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
  * sensor obstacles in the global costmap + reroute-not-clear BTs (2026-10-09);
  * lifecycle_manager_navigation node order.
"""
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = os.path.join(HERE, "..", "config", "nav2_params.yaml")

LAUNCH = os.path.join(HERE, "..", "launch", "navigation.launch.py")
MISSION = os.path.join(HERE, "..", "..", "mower_mission", "config", "mission.yaml")
DOCKING = os.path.join(HERE, "..", "..", "mower_docking", "config", "docking.yaml")

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
    assert set(c["controller_plugins"]) >= {"FollowPath", "FollowPathRPP", "FollowCoveragePath",
                                            "FollowCoveragePathMPPI"}
    assert set(c["goal_checker_plugins"]) >= {"general_goal_checker", "coverage_goal_checker"}
    # 2026-10-09: GridBased = Smac Hybrid, GridBasedNavFn = the BT's fallback id
    p = doc["planner_server"]["ros__parameters"]
    assert p["planner_plugins"] == ["GridBased", "GridBasedNavFn"]
    assert p["GridBased"]["plugin"] == "nav2_smac_planner/SmacPlannerHybrid"
    assert p["GridBasedNavFn"]["plugin"] == "nav2_navfn_planner/NavfnPlanner"


def test_humble_constraints():
    doc = load()
    text = open(PARAMS).read()
    for _, params in ros_param_blocks(doc):
        assert "enable_stamped_cmd_vel" not in params
    # No /map: a StaticLayer is only allowed on the keepout mask (global costmap).
    g = doc["global_costmap"]["global_costmap"]["ros__parameters"]
    if "static_layer" in g.get("plugins", []):
        assert g["static_layer"]["map_topic"] in ("/nav_keepout_mask", "/keepout_mask")
    # nav mask static_layer + terrain_layer (/map_server_node/terrain_cost) + nongrass_layer
    # (/map_server_node/nongrass_cost, 2026-10-09), nothing else
    assert text.count("nav2_costmap_2d::StaticLayer") <= 3
    if "terrain_layer" in g.get("plugins", []):
        assert g["terrain_layer"]["map_topic"] == "/map_server_node/terrain_cost"
    if "nongrass_layer" in g.get("plugins", []):
        assert g["nongrass_layer"]["map_topic"] == "/map_server_node/nongrass_cost"
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
    assert g["xy_goal_tolerance"] == 0.25
    # xy-only: a yaw requirement makes RPP hunt between rotate-to-goal-yaw and
    # rotate-to-carrot at the xy boundary (2026-10-08 bag).
    assert g["yaw_goal_tolerance"] >= 3.14
    leg = c["coverage_leg_goal_checker"]
    # the at-goal reverse shuffle swings 0.02-0.03 m; the overshoot exit must see it
    assert leg["overshoot_hysteresis"] <= 0.02
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
    # 2026-10-09: FollowPath is MPPI now; the RPP transit block is FollowPathRPP.
    rpp = load()["controller_server"]["ros__parameters"]["FollowPathRPP"]
    assert rpp["use_rotate_to_heading"] is True
    # <= the MCU clamp (mcu_node angular_max 0.3): more only adds slew dead time.
    assert rpp["rotate_to_heading_angular_vel"] <= 0.3
    # RPP windows around the MEASURED rate: a stalled pivot must reach >= 0.25 rad/s in
    # one 20 Hz cycle (2026-10-07), so max_angular_accel * 0.05 >= 0.25.
    for name in ("FollowPathRPP", "FollowCoveragePath", "FollowCoveragePathReverse"):
        c = load()["controller_server"]["ros__parameters"][name]
        assert c["max_angular_accel"] * 0.05 >= 0.25, name
        assert c["rotate_to_heading_angular_vel"] <= 0.3, name
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


def test_local_costmap_inflation_with_bounds_keeper():
    # 2026-10-09: Humble 1.1.20 InflationLayer::reset() leaves current_=false until
    # updateCosts() runs, and LayeredCostmap skips it when the bounds are empty. bounds_keeper
    # (an ObstacleLayer without sources, footprint clearing on) forces non-empty bounds.
    import math
    lc = load()["local_costmap"]["local_costmap"]["ros__parameters"]
    assert lc["plugins"] == ["obstacle_layer", "bounds_keeper", "inflation_layer"]
    bk = lc["bounds_keeper"]
    assert bk["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert str(bk["observation_sources"]).split() == []
    assert bk["footprint_clearing_enabled"] is True
    assert bk["combination_method"] == 1          # max: leaves the master untouched
    # bumper points sit inside the footprint: this layer must not erase them
    assert lc["obstacle_layer"]["footprint_clearing_enabled"] is False
    infl = lc["inflation_layer"]
    assert infl["plugin"] == "nav2_costmap_2d::InflationLayer"
    r_c = _circumscribed_radius(lc)
    r_i = _inscribed_radius(lc)
    assert infl["inflation_radius"] > r_c
    # MPPI CostCritic only footprint-checks poses whose centre cost is non-zero
    cost = int(252 * math.exp(-infl["cost_scaling_factor"] * (r_c - r_i)))
    assert cost >= 1, cost


def test_det_range_obstacle_source():
    # det_range centroid cloud: marking-only PointCloud2 source, 3 m max range.
    ol = load()["local_costmap"]["local_costmap"]["ros__parameters"]["obstacle_layer"]
    assert "det_range" in ol["observation_sources"].split()
    src = ol["det_range"]
    # odom-frame copy from fixed_frame_relay (2026-10-09: never waits for TF)
    assert src["topic"] == "/ai/det/obstacle_points_odom"
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


def _circumscribed_radius(gp):
    pts = yaml.safe_load(gp["footprint"])
    pad = gp.get("footprint_padding", 0.0)
    return max(((abs(x) + pad) ** 2 + (abs(y) + pad) ** 2) ** 0.5 for x, y in pts)


def test_global_inflation_covers_inscribed_radius():
    gp = load()["global_costmap"]["global_costmap"]["ros__parameters"]
    infl = gp["inflation_layer"]
    # strictly larger: Nav2 still warns at exactly the (float) inscribed radius
    assert infl["inflation_radius"] > _inscribed_radius(gp) + 1e-3


def test_global_inflation_keeps_obstacle_clearance():
    # 2026-10-09: NavFn plans the centre point; a detour must keep ~circumscribed clearance
    # (0.52 m nose, ~0.61 m corners) or RPP aborts "collision ahead" on the obstacle again.
    gp = load()["global_costmap"]["global_costmap"]["ros__parameters"]
    infl = gp["inflation_layer"]
    assert abs(_circumscribed_radius(gp) - 0.61) < 0.01
    # Smac/MPPI only footprint-check when the centre cost >= computeCost(circumscribed):
    # the inflation must reach the circumscribed radius or the shortcut skips the check.
    assert infl["inflation_radius"] >= _circumscribed_radius(gp)
    assert 0.55 <= infl["inflation_radius"] <= 0.7
    assert 5.0 <= infl["cost_scaling_factor"] <= 15.0
    # the tail stays below the nav mask soft band (229) at the nav margin (0.35 m), so the
    # nav-mask edge behaviour is unchanged
    import math
    tail = 252 * math.exp(-infl["cost_scaling_factor"] * (0.35 - _inscribed_radius(gp)))
    assert tail < 229
    assert gp["plugins"][-1] == "inflation_layer"


def test_stereo_costmap_off_is_bumper_only():
    sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "launch"))
    import robot_settings
    doc = robot_settings.without_stereo_sources(load())
    # 2026-10-09: the collision_monitor drops its stereo source too (bumper only)
    assert doc["collision_monitor"]["ros__parameters"]["observation_sources"] == ["bumper"]
    assert "stereo" in load()["collision_monitor"]["ros__parameters"]["observation_sources"]
    for cm in ("local_costmap", "global_costmap"):
        ol = doc[cm][cm]["ros__parameters"]["obstacle_layer"]
        assert ol["observation_sources"].split() == ["bumper"], cm
        # original untouched
        orig = load()[cm][cm]["ros__parameters"]["obstacle_layer"]["observation_sources"]
        assert {"stereo", "stereo_clear", "det_range"} <= set(orig.split()), cm


def _check_stereo_clear(src, topic="/stereo_depth/clear_points"):
    assert src["topic"] == topic
    assert src["data_type"] == "PointCloud2"
    assert src["sensor_frame"] == "stereo_camera_optical"
    assert src["marking"] is False and src["clearing"] is True
    # ground points (z ~ 0) must survive the height filter
    assert src["min_obstacle_height"] <= -0.1 < 0.1 <= src["max_obstacle_height"]
    assert src["raytrace_max_range"] == 3.0
    # published only when the ground fit succeeds: must never gate isCurrent()
    assert src["expected_update_rate"] == 0.0
    # review 2026-10-09: 0 keeps the last cloud forever (stale wedge keeps clearing after a
    # failed ground fit); must expire, but outlive the <= 0.3 s stamp lag
    assert 0.3 < src["observation_persistence"] <= 1.0


def test_det_range_marks_expire():
    """review 2026-10-09: persistence 0 re-marks the last detection forever (a wall)."""
    d = load()
    for cm in ("local_costmap", "global_costmap"):
        ol = d[cm][cm]["ros__parameters"]["obstacle_layer"]
        assert 0.0 < ol["det_range"]["observation_persistence"] <= 2.0, cm


def test_local_stereo_clear_source():
    lc = load()["local_costmap"]["local_costmap"]["ros__parameters"]
    ol = lc["obstacle_layer"]
    assert "stereo_clear" in ol["observation_sources"].split()
    _check_stereo_clear(ol["stereo_clear"])
    assert lc["width"] == 6 and lc["height"] == 6 and lc["resolution"] == 0.05


def test_global_obstacle_layer():
    # 2026-10-09: sensor obstacles in the GLOBAL costmap so the planner reroutes around them.
    g = load()["global_costmap"]["global_costmap"]["ros__parameters"]
    lc = load()["local_costmap"]["local_costmap"]["ros__parameters"]["obstacle_layer"]
    # 2026-10-09: the nav mask is NOT a layer (it would be inflated): keepout_filter only
    assert g["plugins"] == ["terrain_layer", "nongrass_layer", "obstacle_layer", "inflation_layer"]
    assert g["global_frame"] == "map"
    assert g["transform_tolerance"] == 0.5
    assert 2.0 <= g["update_frequency"] <= 5.0
    ol = g["obstacle_layer"]
    assert ol["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert ol["footprint_clearing_enabled"] is False     # bumper points sit in the footprint
    sources = ol["observation_sources"].split()
    assert set(sources) == {"stereo", "stereo_clear", "det_range", "bumper"}
    for name in sources:
        assert name in ol, f"observation source {name} has no sub-map"
        # Humble planner_server waits for isCurrent(): never gate it on a sensor
        assert ol[name]["expected_update_rate"] == 0.0, name
    st = ol["stereo"]
    assert st["marking"] is True and st["clearing"] is True
    for k in ("sensor_frame", "min_obstacle_height", "max_obstacle_height",
              "obstacle_max_range", "raytrace_max_range", "observation_persistence"):
        assert st[k] == lc["stereo"][k], k
    # 2026-10-09: the global costmap takes the MAP-frame copies (never waits for map->odom)
    assert st["topic"] == "/stereo_depth/points_map"
    assert lc["stereo"]["topic"] == "/stereo_depth/points"
    _check_stereo_clear(ol["stereo_clear"], "/stereo_depth/clear_points_map")
    for name in ("det_range", "bumper"):
        assert ol[name]["marking"] is True and ol[name]["clearing"] is False, name
        # global: map-frame copies, local: odom-frame copies (fixed_frame_relay, 2026-10-09)
        assert ol[name]["topic"] == lc[name]["topic"].replace("_odom", "_map"), name
        assert ol[name]["topic"].endswith("_map") and lc[name]["topic"].endswith("_odom"), name
    assert ol["max_obstacle_height"] >= st["max_obstacle_height"]


BT_DIR = os.path.join(HERE, "..", "behavior_trees")
BT_FILES = ("navigate_to_pose_w_replanning_and_recovery.xml",
            "navigate_through_poses_w_replanning_and_recovery.xml")


def _bt(name):
    import xml.etree.ElementTree as ET
    return ET.parse(os.path.join(BT_DIR, name)).getroot()


def test_bt_reroutes_instead_of_clearing():
    # 2026-10-09: clearing the local costmap after a FollowPath failure erased the obstacle
    # mark that caused it; Wait instead so the 1 Hz replanner routes around it.
    for name in BT_FILES:
        root = _bt(name)
        fp = [n for n in root.iter("RecoveryNode") if n.get("name") == "FollowPath"]
        assert len(fp) == 1, name
        kids = list(fp[0])
        assert [k.tag for k in kids] == ["FollowPath", "Wait"], name
        assert kids[0].get("goal_checker_id") == "general_goal_checker", name
        assert 0 < float(kids[1].get("wait_duration")) <= 2.0, name
        assert not list(fp[0].iter("ClearEntireCostmap")), name
        rr = [n for n in root.iter("RoundRobin") if n.get("name") == "RecoveryActions"]
        assert len(rr) == 1, name
        kids = list(rr[0])
        assert [k.tag for k in kids] == ["Wait", "Sequence"], name     # Wait FIRST
        assert float(kids[0].get("wait_duration")) == 2.0, name
        assert [c.tag for c in kids[1]] == ["ClearEntireCostmap", "ClearEntireCostmap"], name
        # conservative recoveries: no Spin / BackUp; fail fast
        for bad in ("Spin", "BackUp", "DriveOnHeading"):
            assert not list(root.iter(bad)), (name, bad)
        nav = [n for n in root.iter("RecoveryNode") if n.get("name") == "NavigateRecovery"]
        assert nav and nav[0].get("number_of_retries") == "2", name
        # Humble 1.1.x Wait port is wait_duration, InputPort<int> ("1.5" would not parse)
        for w in root.iter("Wait"):
            assert set(w.attrib) <= {"name", "wait_duration", "server_name",
                                     "server_timeout"}, (name, w.attrib)
            assert w.get("wait_duration", "1").isdigit(), (name, w.attrib)


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


def _ctl(name):
    return load()["controller_server"]["ros__parameters"][name]


def _footprint_outline_hits(pts, G, pad=0.02, res=0.1):
    """True if the padded footprint, driven along y=0 through a wall gap of width G, ever has an
    outline sample in a lethal cell (Smac's footprintCost walks the polygon outline)."""
    import math
    eps = 1e-9
    padded = [(x + math.copysign(pad, x), y + math.copysign(pad, y)) for x, y in pts]
    samples = []
    for i in range(len(padded)):
        (x0, y0), (x1, y1) = padded[i], padded[(i + 1) % len(padded)]
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.02))
        samples += [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(n)]

    def lethal(ix, iy):
        # wall: x in [1.0, 1.2], |y| >= G/2; a cell is lethal when it overlaps the wall
        x_ok = ix * res < 1.2 - eps and (ix + 1) * res > 1.0 + eps
        far_y = max(abs(iy * res), abs((iy + 1) * res))
        return x_ok and far_y > G / 2 + eps

    for k in range(51):                                   # x = 0.0 .. 2.5, step 0.05
        px = k * 0.05
        for sx, sy in samples:
            if lethal(math.floor((px + sx) / res + eps), math.floor(sy / res + eps)):
                return True
    return False


def test_smac_hybrid_footprint_checks():
    doc = load()
    gb = doc["planner_server"]["ros__parameters"]["GridBased"]
    assert gb["motion_model_for_search"] == "DUBIN"          # forward only, like the robot
    assert 0.44 <= gb["minimum_turning_radius"] <= 0.5       # cmd_vel_slew / MCU r_min 0.44
    assert gb["allow_unknown"] is True                       # no map: unknown == free
    assert gb["downsample_costmap"] is False
    assert 0 < gb["max_planning_time"] <= 2.0                # BT replans at 1 Hz
    gp = doc["global_costmap"]["global_costmap"]["ros__parameters"]
    # polygon footprint and no robot_radius -> Costmap2DROS::getUseRadius() false -> Smac checks
    # the oriented footprint instead of a circle
    assert isinstance(gp["footprint"], str) and len(yaml.safe_load(gp["footprint"])) >= 3
    assert "robot_radius" not in gp
    # Why gaps are rejected: NavFn (centre point, wall only at cost >= 253 = 0.25 m inscribed
    # band) would have accepted a 0.55 m gap (0.275 > 0.25); the padded 0.58 m body cannot pass.
    pts = yaml.safe_load(gp["footprint"])
    assert _footprint_outline_hits(pts, 0.55) is True
    assert _footprint_outline_hits(pts, 0.80) is False


def test_mppi_transit():
    c = _ctl("FollowPath")
    assert c["plugin"] == "nav2_mppi_controller::MPPIController"
    assert c["motion_model"] == "DiffDrive"
    assert 0 < c["vx_max"] <= 0.3 and 0 < c["wz_max"] <= 0.3     # MCU clamps
    assert c["vx_min"] == 0.0                                    # no blind reversing
    critics = c["critics"]
    assert {"CostCritic", "GoalCritic", "PathAlignCritic", "PathFollowCritic",
            "PreferForwardCritic"} <= set(critics)
    # transit ends on xy only (general_goal_checker yaw tolerance 3.14)
    assert "GoalAngleCritic" not in critics
    assert _ctl("general_goal_checker")["yaw_goal_tolerance"] >= 3.14
    for name in critics:
        assert isinstance(c.get(name), dict), f"critic {name} has no sub-block"
    cc = c["CostCritic"]
    assert cc["consider_footprint"] is True
    assert cc["inflation_layer_name"] == "inflation_layer"
    lc = load()["local_costmap"]["local_costmap"]["ros__parameters"]
    assert cc["inflation_layer_name"] in lc["plugins"]
    # Humble Optimizer::setOffset throws when the controller period exceeds model_dt
    assert c["model_dt"] >= 1.0 / _ctl("controller_frequency") - 1e-9
    assert c["batch_size"] * c["time_steps"] <= 1000 * 56         # RK3588 CPU budget
    assert _ctl("FollowPathRPP")["plugin"].endswith("RegulatedPurePursuitController")


def test_mppi_coverage_alternative():
    c = _ctl("FollowCoveragePathMPPI")
    base = _ctl("FollowPath")
    assert c["plugin"] == "nav2_mppi_controller::MPPIController"
    assert c["motion_model"] == "DiffDrive"
    assert c["vx_max"] <= 0.2 and c["wz_max"] <= 0.3 and c["vx_min"] == 0.0
    # tight swath tracking: at least as stiff as the transit tuning
    assert c["PathFollowCritic"]["cost_weight"] >= base["PathFollowCritic"]["cost_weight"]
    assert c["PathAlignCritic"]["cost_weight"] >= base["PathAlignCritic"]["cost_weight"]
    assert c["model_dt"] >= 1.0 / _ctl("controller_frequency") - 1e-9
    # the default coverage controller stays RPP (MPPI is unvalidated on turf)
    with open(MISSION) as f:
        mdoc = yaml.safe_load(f)
    blocks = [p for _, p in ros_param_blocks(mdoc) if "follow_controller_id" in p]
    assert len(blocks) == 1
    assert blocks[0]["follow_controller_id"] == "FollowCoveragePath"
    assert _ctl("FollowCoveragePath")["plugin"].endswith("RegulatedPurePursuitController")


def test_collision_monitor():
    doc = load()
    cm = doc["collision_monitor"]["ros__parameters"]
    assert cm["cmd_vel_in_topic"] == "/cmd_vel_nav_smoothed"
    assert cm["cmd_vel_out_topic"] == "/cmd_vel_nav"
    assert cm["base_frame_id"] == "base_link" and cm["odom_frame_id"] == "odom"
    assert cm["polygons"] == ["stop_zone", "slowdown_zone", "approach_footprint"]
    for name in cm["polygons"]:
        assert cm[name]["type"] == "polygon", name
        assert cm[name]["action_type"] in {"stop", "slowdown", "approach"}, name
    assert cm["stop_zone"]["action_type"] == "stop"
    sd = cm["slowdown_zone"]
    assert sd["action_type"] == "slowdown" and 0 < sd["slowdown_ratio"] < 1
    ap = cm["approach_footprint"]
    assert ap["action_type"] == "approach"
    assert ap["footprint_topic"] == "/local_costmap/published_footprint"
    assert ap["time_before_collision"] >= 1.0
    xmax = {}
    for name in ("stop_zone", "slowdown_zone"):
        pts = cm[name]["points"]
        assert len(pts) % 2 == 0 and len(pts) >= 8, name
        xs = pts[0::2]
        # zones lie strictly ahead of the footprint front edge (0.52), never over the body
        assert min(xs) >= 0.52, name
        xmax[name] = max(xs)
    assert xmax["stop_zone"] < xmax["slowdown_zone"]
    assert cm["observation_sources"] == ["stereo", "bumper"]
    assert cm["stereo"]["type"] == "pointcloud" and cm["bumper"]["type"] == "pointcloud"
    assert cm["stereo"]["topic"] == "/stereo_depth/points"
    assert cm["bumper"]["topic"] == "/bumper_cloud"
    lc = doc["local_costmap"]["local_costmap"]["ros__parameters"]
    assert cm["stereo"]["min_height"] == lc["obstacle_layer"]["stereo"]["min_obstacle_height"]
    gate = doc["collision_zone_gate"]["ros__parameters"]
    # never the approach polygon: it is direction-aware already
    assert gate["zones"] == ["stop_zone", "slowdown_zone"]
    assert gate["cmd_topic"] == cm["cmd_vel_in_topic"]
    assert gate["forward_off_mps"] <= gate["forward_on_mps"]


def test_velocity_chain_and_docking_lane():
    src = open(LAUNCH).read()
    assert '("cmd_vel_smoothed", "/cmd_vel_nav_smoothed")' in src
    assert '("cmd_vel", "/cmd_vel_nav_raw")' in src
    assert 'package="nav2_collision_monitor"' in src
    with open(DOCKING) as f:
        ddoc = yaml.safe_load(f)
    dp = [p for _, p in ros_param_blocks(ddoc) if "cmd_vel_topic" in p]
    assert len(dp) == 1
    assert dp[0]["cmd_vel_topic"] == "/cmd_vel_docking"
    # docking bypasses the collision monitor (reverse dock / undock must not be stopped)
    out = load()["collision_monitor"]["ros__parameters"]["cmd_vel_out_topic"]
    assert dp[0]["cmd_vel_topic"] != out


def test_bt_planner_fallback():
    # 2026-10-09: NavFn (centre vs the uninflated keepout mask) first, Smac as the fallback
    ids = ["GridBasedNavFn", "GridBased"]
    plugins = load()["planner_server"]["ros__parameters"]["planner_plugins"]
    assert set(ids) <= set(plugins)
    for name, tag, key in (BT_FILES[0], "ComputePathToPose", "goal"), \
                          (BT_FILES[1], "ComputePathThroughPoses", "goals"):
        fbs = [n for n in _bt(name).iter("Fallback") if n.get("name") == "PlannerWithFallback"]
        assert len(fbs) == 1, name
        kids = list(fbs[0])
        assert [k.tag for k in kids] == [tag, tag], name
        assert [k.get("planner_id") for k in kids] == ids, name


def test_nav_mask_is_an_uninflated_centre_limit():
    """2026-10-09 owner: routing stays inside the drawn areas / paths. The nav mask must reach
    the global costmap only via the keepout filter (applied after inflation), never as an
    (inflated) layer, and the map server must not grow areas / paths outward."""
    g = load()["global_costmap"]["global_costmap"]["ros__parameters"]
    assert "static_layer" not in g["plugins"]
    assert g["filters"] == ["keepout_filter"]
    assert g["keepout_filter"]["filter_info_topic"] == "/costmap_filter_info"
    ms_path = os.path.join(HERE, "..", "..", "mower_map", "config", "map_server.yaml")
    if os.path.exists(ms_path):
        with open(ms_path) as f:
            ms = yaml.safe_load(f)["map_server_node"]["ros__parameters"]
        assert ms["nav_margin_m"] <= 0.1 and ms["path_margin_m"] == 0.0
        assert ms["nav_soft_band_m"] == 0.0 and ms["nav_soft_band_with_paths_m"] == 0.0
        assert ms["corridor_half_width_m"] >= 0.3


def test_lifecycle_order():
    lm = load()["lifecycle_manager_navigation"]["ros__parameters"]
    assert lm["node_names"] == ["controller_server", "planner_server", "behavior_server",
                                "bt_navigator", "velocity_smoother", "collision_monitor"]
    # navigation.launch.py LIFECYCLE_NODES (what the launch really starts) must agree
    import ast
    tree = ast.parse(open(LAUNCH).read())
    vals = [n.value for n in tree.body if isinstance(n, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == "LIFECYCLE_NODES" for t in n.targets)]
    assert len(vals) == 1
    assert ast.literal_eval(vals[0]) == lm["node_names"]



def test_global_static_layer_options_at_costmap_level():
    # 2026-10-09: Humble StaticLayer reads use_maximum / trinary_costmap / lethal_cost_threshold /
    # unknown_cost_value from the costmap namespace, not <layer>.<key>. Per-layer only, the mask
    # ran trinary (soft band -> FREE) and terrain_layer OVERWROTE it (dev-image smoke check 3).
    g = load()["global_costmap"]["global_costmap"]["ros__parameters"]
    assert g["use_maximum"] is True
    assert g["trinary_costmap"] is False
    assert g["lethal_cost_threshold"] == 100
    assert g["unknown_cost_value"] == -1

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


def test_velocity_smoother_angular_matches_mcu_clamp():
    vs = load()["velocity_smoother"]["ros__parameters"]
    assert vs["max_velocity"][2] <= 0.3 and vs["min_velocity"][2] >= -0.3


def test_nongrass_layer_is_soft_and_below_inflation():
    """2026-10-09 grass segmentation: the non-grass memory is a SOFT preference layer.
    Non-trinary (trinary would read 75 as FREE or, above the threshold, LETHAL), use_maximum
    (only ever raises the nav-mask cost), before inflation (never inflated: it is not lethal),
    global costmap only (coverage swaths follow the local costmap)."""
    doc = load()
    g = doc["global_costmap"]["global_costmap"]["ros__parameters"]
    plugins = g["plugins"]
    assert "nongrass_layer" in plugins
    assert plugins.index("nongrass_layer") > plugins.index("terrain_layer")
    assert plugins.index("nongrass_layer") < plugins.index("inflation_layer")
    ng = g["nongrass_layer"]
    assert ng["plugin"] == "nav2_costmap_2d::StaticLayer"
    assert ng["trinary_costmap"] is False
    assert ng["use_maximum"] is True
    assert ng["map_subscribe_transient_local"] is True
    loc = doc["local_costmap"]["local_costmap"]["ros__parameters"]
    assert "nongrass_layer" not in loc.get("plugins", [])
