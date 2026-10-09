# Nav2: collision monitor, Smac Hybrid, MPPI (2026-10-09)

Date: 2026-10-09. Dev-image only (Nav2 1.1.20); nothing below has run on the mower yet.
Config and the reasoning behind each value: `src/mower_navigation/config/nav2_params.yaml`.

## Changes

- **Velocity chain.** controller -> velocity_smoother (`/cmd_vel_nav_smoothed`) ->
  `collision_monitor` -> `/cmd_vel_nav` (lane 10). The docking, teleop, bumper and emergency
  lanes do not go through it. `collision_monitor` is last in the lifecycle list.
- **Collision monitor.** Sources are stereo and bumper; the stereo source is dropped with
  `stereo_costmap:=false`. It has three polygons:
  - `approach_footprint`: the costmap footprint, 2.0 s, direction-aware.
  - `stop_zone`: x 0.52..0.75.
  - `slowdown_zone`: x 0.52..1.3, ratio 0.5.

  Humble STOP/SLOWDOWN polygons ignore the direction of travel. So `collision_zone_gate`
  (a new plain node) enables those two zones only while linear.x >= 0.08 m/s. Reverse legs,
  pivots and creeping are checked only by the approach polygon.

  `bumper_controller` now publishes an empty `/bumper_cloud` at 5 Hz between contacts.
  Without it, the collision monitor would log a stale-source WARN on every command.
- **Planner.** `GridBased` is now SmacPlannerHybrid: DUBIN, minimum turning radius 0.45 m,
  no downsampling, 1.5 s planning limit, 10 m lookup table. It checks the full oriented
  footprint. NavFn is kept as `GridBasedNavFn`, and both BTs fall back to it when Smac fails.
- **Transit controller.** `FollowPath` is now MPPI (DiffDrive, 1000 x 40 x 0.1 s, vx 0..0.3,
  wz 0.3, CostCritic with a footprint check). The old RPP transit is `FollowPathRPP`.
  `navigation.launch.py transit_controller:=rpp`, also on `mower.launch.py`, swaps it back.
  The GUI `transit_speed` setting is mapped onto MPPI `vx_max` at launch.
- **Coverage.** `FollowCoveragePathMPPI` is a new MPPI coverage controller: vx 0.2, tight
  PathAlign/PathFollow. Select it with mission.yaml `follow_controller_id`. The default
  stays RPP `FollowCoveragePath`.
- **Local costmap inflation.** The Humble `InflationLayer::reset()` / isCurrent bug is
  **still in 1.1.20**. The workaround is `bounds_keeper`, a source-less ObstacleLayer with
  footprint clearing, which keeps the update bounds non-empty. With it in place, the local
  costmap now has inflation (1.0 m / 4). MPPI's footprint check depends on it.
- **Global costmap static-layer options.** `use_maximum`, `trinary_costmap`,
  `lethal_cost_threshold` and `unknown_cost_value` are read at **costmap** level in Humble;
  per-layer settings are ignored. Before this fix:
  - the mask was trinary;
  - `terrain_layer` overwrote the mask;
  - the mask reached the planner only through the keepout filter, which is not inflated.

  They are now set at costmap level.
- **Global inflation.** Raised from 0.6 to 0.65 so it is at least the circumscribed radius
  (0.61). Smac and MPPI only run the footprint check where the centre cost is at least the
  circumscribed cost.
- **Mission.** MPPI's "Optimizer fail to compute path" (every sampled trajectory collides)
  now counts as a collision abort in `parse_collision_abort`.
- **FTC.** Fixed the heading-unwrap 2*pi error on PRE_ROTATE -> FOLLOWING, which drove a
  full loop off the path; `test_ftc_angle_wrap` covers it. `goal_timeout` raised from 10 to
  30 s. The 10-06 veer itself matches the old FTC tuning: a replay with the old settings
  gives 0.70 m cross-track; the current settings give 0.04 m.

## Dev-image results (`src/mower_navigation/test/smoke/run_smoke.sh`)

- **Lifecycle:** all 6 nodes active in about 2 s.
- **Free transit 3 m:** succeeded in 12.7 s. |v| peaked at 0.24 m/s and |w| at 0.11 rad/s.
- **Box on the path:** MPPI went around it (max lateral 0.66 m) and the goal succeeded.
  The slowdown zone fired.
- **Mask gaps:** Smac rejects 0.4 and 0.5 m gaps and passes 0.9 m. NavFn still plans
  through a 0.5 m gap.
- **Costmap clear:** after clearing the local costmap, the robot drives again within 0.19 s.
  The control run without `bounds_keeper` (bumper-only sources) stalls forever, which
  confirms the bug.
- **Stop zone:** forward into the stop zone is stopped. Reverse at -0.15 m/s passes after
  0.07 s, and a 0.3 rad/s pivot passes.
- **CPU (x86):** controller_server 7-18 % of a core, planner_server 9-12 %.

## Live checks needed

1. **CPU at 20 Hz on the RK3588.** Check controller_server with MPPI (`top -H`) and look for
   "Control loop missed its desired rate". Lower `batch_size` first if needed.
2. **Smac reachability.** Run dock -> area and dock -> (-1.80, -1.29). Also check goals at
   swath starts on the boundary: with the mask now inflated, these may need the NavFn
   fallback. Look for "Planning algorithm GridBased failed" in the planner_server log.
3. **Stop zone near hedges.** Watch for forward stops against hedges at swath ends (stop
   band 0.23 m). Check that coverage reverse legs and pivots are never blocked; the gate
   logs at debug level.
4. **Stereo blind band.** Measure the real stereo blind band, nominally x < ~0.67 m in
   base_link (min range 0.2 m). Retune `stop_zone` and `time_before_collision` from it.
5. **Turf behaviour.** Check MPPI turn behaviour through the cmd_vel_slew turn shaper on
   turf. Live GUI changes of `transit_speed` do not reach MPPI until a restart.
6. **Inflated local costmap with FTC.** The local costmap is now inflated, so FTC sees the
   0.25 m inscribed band around marks as blocked when FTC is selected.
