# Roadmap, 2026-10-14: from patching to a mower that follows its map

Written after two field runs on 2026-10-14, the strategy review
(`analysis/2026-10-14_strategy_review.md`) and fixes 1–11 + heading A/B. Rule for everything
below: **one change-set per field run, measured with `scripts/wheel_bag_metrics.py` and the
aligner status, with a written pass/fail before the next change-set.** Anything that cannot be
measured from a bag gets a by-eye criterion written down first.

## 1. Where we are

Verified in the field (keep):
- Emergency / bumper stops: 24–51 ms to zero, 1–5 cm rolled (was 0.6 s / 9 cm).
- Sub-deadband pivots: 200 s → 67 s per mow; no sustained ones left.
- Pivot assist runs on the snapped pivots (127 creeps in run 2).
- Stereo yaw gaps are bridged and counted; the stale-heading START gate exists.
- Black box + metrics give a full picture of every run.

Deployed, unit-tested only, field test pending:
- Fix 10: turn-leg cap (20 s or one rotation at the leg end), per-leg loop guard (90 s).
- Fix 11: hairpin about-turn (Nav2 Spin) instead of the costmap fallback; fallback bounded to
  0.35 m from the route.

Open, structural (the reason the runs still fail):
- A. Heading: single antenna, corrections only on 0.6 m straights at 10 %, gyros ≈ 1 % scale
  error, so every long pivot session drifts the heading (run 2: −73° → −95°).
- B. Lane ends: 0.2–0.8 m Reeds-Shepp pieces as separate Nav2 goals with an 8 cm checker,
  driven by pivots that slip (run 2: 7 minutes on one 0.8 m piece).
- C. Transit: a free-space fallback that leaves the drawn paths, and a coverage start the
  route must reach across the grass (run 2: "Dock path 1.1 m, Area 1 1.5 m", the last 1.5 m
  inside the area are off any drawn path).
- D. Slope: only a speed cap (caution band) and a back-out (limit band). No slope-aware
  turning or swath direction; the vendor has SLOPE_BACK turns and slope-scaled velocity.
- E. Too many knobs: ≈ 550–600 nav/loc keys (vendor ≈ 146), six controllers, three limiters.

## 2. Method

1. **Hypothesis first.** Each task states the failure it removes, the bag metric that proves
   it, and the number that counts as pass.
2. **One change-set per run.** Params and code of one task only; everything else frozen.
3. **Measure, then decide.** Run the metrics, read the aligner series, write the result into
   `wheel_smoothness_field_tests.md` (run N), decide keep / revert / tune. Reverts are param
   flips; every task ships with its revert flag.
4. **Prefer removing to adding.** A task that deletes a behaviour and matches the vendor's
   running node (`navigation/param/cfg.yaml`) beats one that adds a guard.
5. **Field runs are short and scripted**: undock → transit → 3–4 swaths with their turns →
   return → dock, on the closest area, with the slope spot included once per phase.
6. **Order by leverage**: heading and transit remove failure causes; turns remove most of the
   shaper's work; pruning last, once behaviour is stable.

## 3. Task list

Legend: [P] parameters only (sync + restart), [C] code, [H] hardware, [D] decision. Each task:
goal → change → test → pass criterion → revert.

### Phase 0: close out the current change-set (this week)

- [x] 0.1 [P] (run 3: FAIL, left the area in transit; see field tests run 3) Field-test fixes 10 and 11 on the usual short mission.
      Pass: no leg longer than 20 s after reaching its end; at the dock hairpin a "route cusp …
      about-turn" line and no `navigate_to_pose` from a cusp; no fallback track > 0.35 m off
      the route; metrics: time in pivots < 25 % of moving time (run 2: 593 s of 2200).
      Watch: Spin refusing at the junction (corridor lethal in the local costmap).
- [ ] 0.2 [P] Raise `coverage_leg_goal_checker.xy_goal_tolerance` 0.08 → 0.12 if 0.1 shows
      the cap firing on every leg (the cap is a backstop, not the way legs should end).
- [ ] 0.4 [C] **Boundary is a hard rule in every autonomous state** (run 3: seven transit
      excursions up to 0.85 m). Change-set:
      (a) mission: robot centre outside (mowing areas ∪ drawn path polygons) by more than
          `hard_boundary_m` (0.10) in TRANSIT / RETURNING_HOME / MOWING → cancel, zero, then
          retrace the driven track back inside (reverse along the last 1 m of EKF poses),
          blade off; count as a transit abort; after 2 → STUCK_NEEDS_HELP. Not a log line.
      (b) shaper: when the controller asks for a turn tighter than the shaper's widest legal
          substitute allows, pivot instead of widening: if the requested radius
          r_req = |v|/|w| is below half the widened radius, snap to the pivot (both wheels,
          ≥ 0.25 rad/s), so a corner is turned on the spot, not overshot. Owner: "it could
          reverse and turn rather than overshoot the corner".
      (c) MPPI transit `vx_min` −0.10 (reverse allowed, `PreferForwardCritic` kept) so an
          overshot corner is backed out of, not looped around.
      (d) route-corner about-turn: fix 11's cusp handling for any route corner > 90° with the
          about-turn done before the corner (stop at the corner, pivot, continue), and
          `route_cusp_radius_m` 0.5 → 1.0.
      (e) docking server approach leg: drive the dock route to the approach pose with the
          route follower; the docking server's own `navigate_to_pose` only within
          `corridor_half_width_m` of the approach pose.
      (f) fix 10 bug: "end reached" by distance only (`turn_leg_skip_m`), never by
          `progress_local`; capped legs only if their own length ≤ `turn_leg_max_m`.
      Pass: zero samples outside areas ∪ paths by > 0.10 m in a full mow; no
      `navigate_to_pose` in the log; corners within 0.15 m of the route.
      **Implemented on `ws/hard-boundary` (2026-10-14), unit-tested, field test pending (run
      4; what to look for: `wheel_smoothness_field_tests.md` "Run 4 change-set"):**
      (a) `map_server` publishes `~/outside_m` (areas ∪ paths ∪ dock corridor, not the return
          corridor); `mission_fsm._hard_boundary_tick` in TRANSIT / MOWING / dock phases:
          cancel, zero, blade off, reverse ≤ 0.10 m/s along the last 1.5 m of EKF poses on
          `/cmd_vel_emergency` (both wheels backwards; > 0.6 rad heading error → pivot
          ≥ 0.25 rad/s first; never beyond the recorded track), until inside + 0.10 m; then a
          transit abort (MOWING: sub-path resumed; dock phases: dock re-run). The 2nd
          excursion of a mission, no progress, still outside → STUCK_NEEDS_HELP. Heading
          calibration drive: stopped only. Params `hard_boundary_m` 0.10,
          `hard_boundary_max_aborts` 2, `hard_boundary_retrace_*`, `hard_boundary_trail_m` 1.5.
          Revert: `hard_boundary: false`.
          2026-10-15 sim fix (`ws/hb-fixes`): "back inside" only on a fresh `outside_m` ≤ 0
          (inside + 0.10 m, or inside when the track / max ends); still outside (or no fresh
          reading) at the track end → STUCK_NEEDS_HELP, never a resume (was: resumed at
          ≤ `hard_boundary_m`, sim: 0.066 m outside).
      (b) `cmd_vel_slew`: `turn_shaper_pivot_instead` (true), `_ratio` 0.5, `_hysteresis` 0.1.
          Revert: `turn_shaper_pivot_instead: false`.
      (c) `nav2_params.yaml` FollowPath `vx_min` −0.10, PreferForwardCritic kept; nothing
          downstream clamps negative vx. Revert: `vx_min: 0.0`.
      (d) `route_corner_turn_rad` 1.38 (2026-10-15 sim fix, was 1.5708): split at corners
          ≥ 79° (inclusive, 1° tolerance), Spin to the leg heading before a leg ≥ 79° off the
          robot's yaw (transit and dock route); corners measured between the straight runs
          either side (`geometry.route_corners`: spacing / wobble independent, neighbouring
          sharp corners each split); the leg into a split corner ends on it
          (`route_corner_goal_checker_id` coverage_leg_goal_checker; it ended 0.2 m short
          with general_goal_checker); `route_cusp_radius_m` 1.0. Sim: every split corner
          ≤ 0.11–0.13 m (was 0.19–0.22); MPPI still cuts the dock path's 39° / 57° bends by
          0.20–0.34 m. Revert: `route_corner_turn_rad: 0.0` (run 4 threshold 1.5708,
          checker `''`).
      (e) `dock_server_nav_max_m` 0.35, `dock_route_retries` 2, `dock_approach_distance_m`
          0.8: the docking server only near the approach pose, else the dock route again,
          then NAV_TO_DOCK_FAILED in place. Revert: `dock_server_nav_max_m: 0.0`.
      (f) `turn_leg_end_by_distance` (true): leg end by distance only, cap by own length
          only (`_is_turn_leg` kept for `turn_leg_skip`). Revert: `false`.
- [ ] 0.3 [C] Docs: replace every "vendor" accel/decel citation of `adaptivePurePursuit.h`
      with the running node's values (0.6 / 1.0 m/s², 2.0 rad/s²; `cfg.yaml:8-9,69-72`).

### Phase 1: heading (open item A)

- [ ] 1.1 [P] Correct like the vendor: COG chord from 0.3 m (not 0.6), gain 0.5 (vendor 0.7),
      accept windows at ≥ 0.08 m/s on any gently curving track (IMU span ≤ 25°) instead of
      only straight ones. Params in `heading_aligner`: `min_distance`, `alpha`,
      `max_imu_yaw_change`, `min_speed`.
      Test: one mow. Pass: offset moves < 3° per 10 min of mowing (run 2: 6°), innovations
      centred on 0, no re-seed. Revert: old values.
- [ ] 1.2 [C] Per-source yaw-rate scale from COG turn pairs (gyro Δyaw / COG Δcourse between
      consecutive accepted windows, robust median over the mow, applied to the stereo and WIT
      rates separately; status shows both). Pass: Δ(stereo − WIT) per 360° of rotation < 1°
      (run 2: 2.3°), pivot sessions no longer move the offset.
- [ ] 1.3 [D] Dual antenna. The only approach the evidence calls robust on turf (ArduPilot
      mowers, moving baseline). The universal-gnss path already carries `rtk_dual_antenna`.
      Decide after 1.1/1.2: if the heading still drifts > 3° per mow, buy a UM982-class
      receiver and retire the COG machinery to a fallback. Until decided, no more aligner
      features.
- [ ] 1.4 [P] Keep the spinning bounded (fix 10 does this) so heading exposure stays small
      regardless of 1.1–1.3.

### Phase 2: transit and the mow start point (open item C)

- [ ] 2.1 [C] Coverage starts where the drawn path enters the area. The planner's first
      headland ring is "free to start anywhere" (`coverage_planning.cpp` `orderCells`,
      `start_loop`); give it the entry pose (the route's last on-path pose) and start the ring
      at the closest ring point, direction chosen so the ring continues away from the entry.
      Then the route is dock → path → ring start with no in-area leg.
      Pass: route log shows no "Area N x m" in-area leg beyond the ring inset; no grass crossed
      before the blade is on.
- [ ] 2.2 [C] Drawn paths must reach the area: the GUI / map check warns when a path ends more
      than `corridor_half_width_m` from the area boundary, and the route graph refuses a
      route whose last on-path pose is farther than that from the area (today it plans the
      in-area leg silently).
- [ ] 2.3 [C] Remove the free-space fallback entirely (vendor and MowgliNext have none): a
      start off the route graph gets a short straight "to nearest route point" leg bounded by
      the corridor, else the mission stops and reports where it is. Fix 11's 0.35 m bound
      becomes the only transit rule. Pass: no `navigate_to_pose` in a mission log.
- [ ] 2.4 [P] Return to dock the same way: RETURNING_HOME uses the route graph only; the
      return corridor stays as the costmap mask, not as a planner.

### Phase 3: turns and lane ends (open item B), the real redesign

- [ ] 3.1 [D] Choose the lane-end style per the vendor: arcs by default (0.3 m/s at
      0.5 rad/s), pivot only when the arc does not fit the drivable region. Decide the arc
      radius from the shaper floor: r = v / w = 0.6 m at 0.3 / 0.5 (vendor), 0.28 m at
      0.14 / 0.5 (slow). MowgliNext uses r 0.2 at low speed.
- [ ] 3.2 [C] One tracker goal per swath pair (swath → turn → swath), the turn included in
      the path, no per-piece goals; the `coverage_leg_goal_checker` applies only to the last
      pose of the pair. Keep `FollowCoveragePathReverse` for the K-turn case only.
      Pass: regime changes per swath < 6; time in pure pivots < 10 % of moving time; no leg
      cap firing.
- [ ] 3.3 [P] Angular limits to the vendor's: MCU clamp 0.3 → 0.5 rad/s (wheels-up check
      first), `velocity_smoother max_velocity[2]`, MPPI `wz_max`, RPP rotate and the shaper
      floor moved together (field-test doc item 5). Pass: pivot authority ratio > 0.7 on the
      flat, no turf scrub by eye.
- [ ] 3.4 [C] Shaper becomes a safety net: with 3.2/3.3 the controller should never emit a
      one-wheel command; log every shaper intervention as a counter and target zero per mow.
      Pivot assist stays (vendor TURNCRAWL analogue).

### Phase 4: slope (open item D)

Physical rule to validate first (by eye, wheels-up then on the known −16° roll spot):
a rear-drive mower with front casters loses the casters downhill when it pivots on a side
slope; the stable move is to turn with the drive wheels moving, nose rotating through
the downhill side, and never to stop or reverse pointing straight downhill on wet grass.

- [ ] 4.1 [C] Swath direction per area from the terrain raster (`mower_map/terrain.py` has the
      slope layer; `terrain_aware_planning.md` §1.1 has the F2C objective hook): above
      `slope_contour_deg` (start 8°) mow along the contour, so turns happen on the level ends;
      below it, keep the owner's angle.
- [ ] 4.2 [C] Slope turn mode (vendor SLOPE_BACK): when the tilt band at a swath end is
      caution, replace the pivot / K-turn with a reverse-uphill then forward-arc turn
      (reverse along the swath 0.3 m uphill, arc out with the nose through the downhill side),
      at the caution speed. Trigger from `/tilt/status` plus the raster's slope at the swath
      end. Pass: no stuck / slip incident at the slope spot over three runs.
- [ ] 4.3 [P] Velocity scaling by pitch/roll on the whole path (vendor `SlopeLimitVel`): a
      continuous cap instead of the two-band step; bands stay for blade-off / back-out.
- [ ] 4.4 [C] Swath ends never on a steep or low-traction cell: the planner trims or extends
      a swath by up to one width so its end (where the turn happens) sits on a cell below
      `slope_turn_max_deg`; else the turn is moved to the other end.

### Phase 5: consolidate (open item E)

- [ ] 5.1 [P] Remove unused controller configs (MPPI coverage, FTC) and the two widened
      limiters (MPPI model limits back to defaults, `velocity_smoother` off or pass-through);
      `cmd_vel_slew` is the single limiter at the vendor's 0.6 / 1.0 / 2.0.
- [ ] 5.2 [C] Fold the aligner down to what Phase 1 leaves: one COG estimator, dock seed,
      file continuity, scale; drop the rest.
- [ ] 5.3 Target: nav + localization keys under 300, controllers 2, goal checkers 2.

## 4. Order and cadence

Phase 0 → 1.1 → 2.1 + 2.2 (one run, they are independent of 1.x) → 1.2 → 2.3/2.4 → 1.3
decision → 3.x (one run each) → 4.x (needs 3.1/3.2 so slope turns are arcs) → 5.x.
Every run: `scp` the pin, `zstd -dkq`, metrics, aligner series, a "Run N" entry. Stop and
re-plan whenever a run shows a failure class not in this list.

## 5. Handover state (2026-10-14, end of the Fable session)

Deployed on the mower (master d084328 synced, container restarted): fixes 1–11, heading A/B,
all parameters confirmed live. **The fix 10/11 field test is running now** (task 0.1).

What to do with that run:
1. `ls /userdata/ros2/mows/` on the mower, copy the newest pin, decompress, run the metrics
   (`wheel_smoothness_field_tests.md` "Review loop"), then the aligner series:
   `virtual_minus_wit_deg`, `offset_deg`, `stereo_gaps` over time from
   `/heading_aligner/status` (the dump scripts of this session lived in the scratchpad; they
   are 30-line sqlite + rclpy dumps, trivial to rewrite: read `wheel_bag_metrics.bag_files` +
   `read_series` for the pattern).
2. In the container log look for: "route cusp at", "about-turn", "Spin", "turn leg end …
   cap", "leg loop", "transit fallback left the corridor", "navigate_to_pose" (should be
   absent from a cusp), "Failed to make progress".
3. Write "Run 3" into `wheel_smoothness_field_tests.md` with the pass criteria of task 0.1,
   decide keep / revert, then move to 1.1.

Known open risks for run 3: Spin may refuse at the dock junction if the corridor reads lethal
in the local costmap (the route is re-sent, bounded by `route_cusp_retries`); the 8 cm leg
tolerance is unchanged (task 0.2); heading still drifts ≈ 1 % of rotation (Phase 1).

Branches / worktrees left in place under `../ros2_stack-wt/`: `wheel-smooth`, `pure-pivot`,
`heading-gaps`, `mission-legs` (all merged with `--no-ff`). Nothing is pushed to origin.
Untracked docs in the tree from other sessions (`AIRSEEKERS_BACKEND_TASKS.md`,
`DEVELOPMENT.md`, `GUI_REBASE_*.md`) are not part of this work.
