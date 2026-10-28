# MPPI as the single tracker: study, feasibility-aware MPPI, sim A/B (2026-10-15)

Work package of `DESIGN_path_conditioning_2026-10.md` §3.2 (owner decision: MPPI replaces
Regulated Pure Pursuit for transit, dock routes and coverage). Branch `ws/mppi-study`.

**Result, plainly: MPPI-for-everything does not meet the target yet, so the shipped default
stays `coverage_controller: rpp`.**

- **Feasibility works.** A custom MPPI critic (`mppi::critics::WheelFloorCritic`,
  `mowgli_nav2_plugins`) makes undrivable samples expensive. The share of MPPI output the
  turn shaper must rewrite drops from 54 % to 9–10 % in coverage (sim, same config with and
  without the critic). Without it MPPI dithers at sharp corners: the field's undock donut and
  run 4 junction regime, reproduced. Transit with the critic tracks like master (p90 0.28 m,
  0 transit trips with the ground disturbance) with half the rewrites (2.4 vs 4.9 %).
- **Lines and arcs: MPPI is at least as good as RPP.** Coverage straights p90 0.017–0.034 m
  (RPP 0.011–0.032; field RPP 0.074); straights p99 0.09–0.27 m vs RPP 0.12–0.27 m;
  swath-end turn median 0.05–0.07 vs RPP 0.16–0.19 m.
- **Sharp vertices on raw paths: MPPI is worse than RPP.** It cuts, then overshoots outward,
  because it cannot turn tighter than r = 0.44 m without stopping. Result: 8 / 17
  hard-boundary trips (ideal / disturbed) vs RPP 2 / 2, and corner deviation up to 0.8 m.
  Without the critic it dithers at the vertex instead.
- **With the vertices pivoted** (a stand-in for the conditioner's pivot markers):
  - Corners close. Ring-corner deviations fall to 0.02–0.11 m, and corner p99 is 0.099 m
    vs RPP 0.18 m (ideal).
  - MPPI still leaves the area on the 0.3–0.6 m swath-end turn legs near the headland
    (5–16 trips; RPP 0 with the same stand-in). Run-to-run variance is high: the same
    config gave 5 and 16 trips (ideal).
- **What closes the gap:** the path conditioner (pivot at every vertex that cannot be
  filleted, r ≥ 0.5 m fillets elsewhere) plus the roadmap 3.2 lane-end redesign (one goal
  per swath + turn, so MPPI never has to stop on a 0.08 m checker at the end of a short
  curved leg). Then re-run this A/B. MPPI should not be switched on for coverage before that.


## 1. The robot MPPI has to drive

Diff drive, track b = 0.48 m (h = b/2 = 0.24 m). The MCU clamps |v| ≤ 0.3 m/s and
|w| ≤ 0.3 rad/s separately. A wheel asked for less than ~0.06 m/s while turning does not
move. With inner = |v| − |w|·h the drivable set is (design §2):

| command | condition | example |
|---|---|---|
| straight | w = 0 | any v |
| arc | inner ≥ +0.06 | r ≥ 0.44 m at v ≈ 0.13; r ≥ 1.0 m at 0.3 m/s |
| pivot | inner ≤ −0.06 | v = 0, \|w\| ≥ 0.25 rad/s |
| **not drivable** | −0.06 < inner < 0.06 | (0.03, 0.1), (0.05, 0.2), (0, 0.15) |

Nav2 MPPI's `DiffDrive` model assumes every (vx, wz) in its box is drivable. Everything in
the last row is rewritten downstream by `cmd_vel_slew`'s turn shaper (widened to an arc,
or snapped to a 0.25 rad/s pivot), so the next MPPI cycle starts from a state its rollout
did not predict.

## 2. Field evidence: what MPPI actually sent

`/cmd_vel_nav` (the controller output after `velocity_smoother`, i.e. what the shaper gets)
classified per tick by the table above (script: `field_feas.py` in the session scratchpad;
bags already extracted to JSON):

| bag | phase (controller) | ticks | not drivable | of which |
|---|---|---|---|---|
| 2026-10-15 undock donut | TRANSIT (MPPI) | 1684 | **84 %** | one-wheel 860, slow pivot 408, floor arc 157 |
| run 4 (mow-040433) | TRANSIT (MPPI) | 1869 | **34 %** | floor arc 434, one-wheel 165, slow pivot 45 |
| run 4 | RETURNING_HOME (MPPI) | 833 | **34 %** | |
| run 4 | MOWING (RPP) | 4649 | 2.1 % | |
| run 5 (full mow) | TRANSIT (MPPI) | 3479 | **24 %** | floor arc 638, one-wheel 155 |
| run 5 | RETURNING_HOME (MPPI) | 1490 | **28 %** | |
| run 5 | MOWING (RPP) | 13821 | 4.3 % | |

- **Undock donut** (82 s): MPPI |v| ≤ 0.03 with |w| 0.05–0.3, the shaper turned each tick
  into a 0.25 rad/s pivot, the robot circled. Cause: the cmd_vel_slew accel limits
  (ax 0.5 / −0.8, az 1.0) in MPPI's model (3196ffd, reverted in 1e19c15) + a 0.17 m stub leg.
- **Run 4 junction** (STUCK1/2): v −0.05..+0.07, w 0.05–0.15, shaped into 0.25 rad/s pivots,
  ~20 s twice. The model-mismatch loop: MPPI asks for a slow turn, gets a fast pivot, sees
  an overshoot, asks the other way.
- **Run 5**: in transit 40 % of MPPI ticks were slow turns (|v| < 0.06, |w| > 0.02) and 691
  ticks reversed (vx_min −0.10). MPPI transit cross-track vs `/plan` (EKF):
  median 0.079, p90 0.296, p99 0.572 m. RPP coverage on the same run: median 0.026,
  p90 0.074, p99 0.204, max 0.262 m, all large errors at sharp corners.

MPPI's failure mode on this robot is therefore not "MPPI is imprecise": it is that MPPI
lives in the band the drive cannot execute for a quarter to most of its ticks, and a
downstream rewrite then breaks its own prediction.

## 3. The configuration on master (e9e4d08), critic by critic

`FollowPath` (transit and dock routes; `FollowCoveragePathMPPI` was configured but unused):

| item | master | assessment |
|---|---|---|
| model_dt / time_steps / batch | 0.1 / 40 / 1000 at 20 Hz | model_dt ≠ control period: Jazzy only shifts the warm-start sequence when they match (README: "should be set to the duration of your control frequency … may also set it lower but not larger" [1]), so every cycle restarts from an unshifted sequence. 4 s horizon. RK3588: 17.5 Hz measured. |
| vx_std / wz_std | 0.1 / 0.2 | half the Nav2 defaults (0.2 / 0.4 [1]); fine for a 0.3 m/s robot |
| temperature / gamma | 0.3 / 0.015 | default [1]. 0.3 averages many samples: pivot samples and arc samples get blended into a slow turn, i.e. exactly the undrivable band |
| ax/az limits | defaults (3.0 / −3.0 / 3.5) | correct: the 0.5/−0.8/1.0 set livelocked (donut). Re-tested with the new critic (T3, §7): still a livelock, 80 % pivoting |
| ConstraintCritic 4 | default | only penalises leaving [vx_min, vx_max], not the wheel floor |
| CostCritic 3.81, footprint | default weight | needs the local inflation layer (it has one); the local costmap has **no boundary** (nav mask only in the global costmap's keepout filter), so MPPI cannot see the area edge (§7, B2) |
| GoalCritic 5, 1.0 m | | |
| PathAlignCritic 14, offset 20, step 3, 0.5 m | | offset 20 is the Nav2 default for 0.05 m paths at 0.5 m/s; README baseline "horizon·v_max/resolution/3" gives ≈ 4 here |
| PathFollowCritic 5, offset 5 | | pulls toward a point ahead: the corner-cutting critic (nav2#5925 [3]) |
| PathAngleCritic 2, `forward_preference: true` | | **the key is ignored in Jazzy 1.3** (it reads `mode`; checked in `libmppi_critics.so`, the string is not present). Same default (mode 0), so no behaviour change; now written as `mode: 0` |
| PreferForwardCritic 5 | | keeps the −0.10 reverse rare |
| TwirlingCritic | absent | correct: omni robots only [1] |
| VelocityDeadbandCritic | absent | exists in Jazzy (nav2#4256 [2]) but penalises \|vx\| and \|wz\| below a per-axis deadband independently: it would punish pivots (vx = 0) and straight driving (wz = 0), and cannot express "inner wheel = v − w·h". Not usable here. |

## 4. Nav2 Jazzy MPPI: documentation and known issues

Sources: [1] `nav2_mppi_controller/README.md` (jazzy branch) and
`src/optimizer.cpp`; installed package 1.3.13 in `mower:jazzy-dev-amd64`.

- **Critics are plugins; the motion model is not.** Critics are pluginlib classes of base
  `mppi::critics::CriticFunction`; the Jazzy `CriticManager` loads `"mppi::critics::" +
  <name in the critics list>` (prefix string checked in `libmppi_controller.so`), so a
  custom critic must be exported under that namespace. `Optimizer::setMotionModel` is a
  hard-coded if/else (DiffDrive / Omni / Ackermann); pluggable motion models landed on
  `main` in 2026 (nav2#6032 / PR #6076 [4]), not in Jazzy. There is no sampling hook:
  Gaussian noise around the warm start, accel clamps per step (`MotionModel::predict`),
  and `applyConstraints` on the *optimal* sequence only (Ackermann's min radius; DiffDrive:
  none). Steve Macenski on #6032: "the critics are what shape the function. Random
  sampling is just the exploration mechanism" [4].
- **Ackermann as a constraint?** `AckermannConstraints.min_turning_r` would enforce
  r ≥ r_min on the output, but forbids pivots and is a fixed radius: our floor needs a
  speed-dependent radius (r ≥ h·v/(v − 0.06)), so 0.44 m is only right at v = 0.13.
- **Output = weighted average.** The command is the softmax-weighted mean of the samples
  (temperature), then Savitzky–Golay smoothed. A critic can only make undrivable samples
  unlikely; the mean of a pivot sample and an arc sample is a slow turn. The arc band
  (inner ≥ 0.06, same signs) is convex, the pivot band is too, their union is not: low
  temperature and a strong penalty keep the mean inside one band.
- **Path tracking accuracy.** README: MPPI "can be an exact path tracker"; tuning order
  obstacles → PathFollow → PathAlign; PathAlign `offset_from_furthest` baseline
  horizon·v_max/resolution/3 [1]. nav2#5925 (Jazzy 1.3.8, coverage U-turns): MPPI cuts
  tight U-turns because "the path follow critic finds the end point and incentivizes …
  to go towards that point and shortcut, so it's a balancing act"; the arc-length fix
  (#6055) is on `main` [3]. #5652: corner cutting / pose skipping. #5303: no stop at cusps
  (`enforce_path_inversion`). #5214: the model assumes the motors reach the command
  instantly (our wheel lag + slew is the same mismatch).
- **Low speed / deadband.** #4256 added VelocityDeadbandCritic for "platforms [that]
  cannot move slowly" [2]. MowgliNext PR #230: after three tuning passes MPPI still output
  `linear.x = 0.012, angular.z = −0.0003` steady state, "an order of magnitude below the
  firmware deadband … the cost critics keep converging on near-zero velocity" [5].
- **CPU.** #6228 (open): "on our robot, MPPI consistently cannot achieve 20Hz"; #4513 (ARM
  Jetson Nano); #4621 Eigen port (+45–50 %, not in Jazzy 1.3.13, which is still xtensor).
  MowgliNext PR #226 on the same SoC (RK3588): 1500×40 ran at 5–19 Hz with "Control loop
  missed its desired rate" and TF extrapolation aborts; they went to 800×25, model_dt
  0.06 [5].

## 5. MowgliNext's MPPI attempt (github.com/mowglinext/mowglinext)

- 2026-05-17 PR #224: MPPI on transit, FTC kept on coverage because "MPPI's Euclidean
  nearest-point matching jumps between adjacent swaths".
- PR #226: RK3588 budget, 1500×40 → 800×25 (above).
- PR #230: transit back to RotationShim + RPP (near-zero output inside the deadband, above);
  they also added a closed-loop motor-deadband compensator in their hardware bridge.
- PR #294 (2026-06-15): RotationShim + MPPI for coverage, one segment at a time.
- PR #310: much of the circling / weaving was `controller_server.odom_topic` pointing at
  an unpublished topic (zero velocity feedback). **Checked here: ours is
  `/odometry/filtered`, published.** Same PR: "MPPI can't do the hard ~180° swath flips
  (Gaussian sampling never proposes a near-stationary pivot)".
- 2026-06-19: coverage reverted to FTC — "cut corners / omega-looped at U-turns, weaved on
  the straights"; their 2026-09 Lyrical controller review keeps FTC (8–16 mm on R 0.2 m
  arcs) and blames "l'échantillonnage et l'arbitrage path_align / wz_std" [5].

Lessons taken: (a) a wheel-floor robot needs MPPI to know the floor (they compensated in
firmware instead; we have no firmware PI); (b) explicit pivots for large heading changes
(their RotationShim, our Spin step); (c) watch CPU on the RK3588; (d) the swath-jumping
issue does not apply here (each FollowPath goal is one sub-path; the path handler searches
forward from the last closest pose within `max_robot_pose_search_dist`).

## 6. What was implemented

### 6.1 Mechanism: a feasibility critic (chosen) vs the alternatives

| option | verdict |
|---|---|
| custom critic plugin (`mppi::critics::WheelFloorCritic`) | **chosen**: the only supported extension point in Jazzy; scores exactly the coupled rule (inner wheel), allows pivots |
| VelocityDeadbandCritic (built-in) | per-axis, cannot express the inner wheel; would penalise pivots and straight driving (§3) |
| ConstraintCritic | only the [vx_min, vx_max] / wz box |
| Ackermann `min_turning_r` | fixed radius, no pivots, applied to the optimal sequence only (§4) |
| constrained sampling / custom motion model | not possible in Jazzy without forking nav2_mppi_controller (§4) |
| accel limits = the measured slew (0.5 / −0.8 / 1.0) | re-tested with the critic (T3): 80 % pivot time, leg loop guard; rejected again |
| boundary in the local costmap (keepout filter) | tried (B2): Spin and MPPI see the centre-limit mask under the footprint, "Optimizer fail to compute path", the transit never left the dock path; reverted |

### 6.2 Code, parameters, reverts

- `src/mowgli_nav2_plugins/include/mowgli_nav2_plugins/wheel_floor.hpp:39` `violation(v, w)`:
  distance in m/s of wheel speed from the drivable set of §1 (0 = drivable), the same rule
  as `cmd_vel_slew.both_wheels_legal(target=True)`; straight (`straight_wz`, 0) and stopped
  (`stop_speed`, 0.02 m/s) commands are free; `allow_pivots: false` makes counter-rotation
  cost too.
- `.../wheel_floor_critic.hpp:36` `scoreBatch`: `cost += weight · mean_t (violation/floor)^power`
  over the sampled controls (`state.cvx`, `state.cwz`, after MPPI's accel clamp).
- `src/mowgli_nav2_plugins/src/wheel_floor_critic.cpp`, `mppi_critics_plugin.xml`
  (`type="mppi::critics::WheelFloorCritic"`: Jazzy prefixes `mppi::critics::` to the name in
  the `critics` list). Library `mowgli_mppi_critics` (the FTC library does not link MPPI).
- `nav2_params.yaml`: `WheelFloorCritic` on `FollowPath`, `FollowCoveragePathMPPI`,
  `FollowCoveragePathReverseMPPI` (track 0.48, floor 0.06, pivots allowed; weight 20 transit,
  30 coverage). **Revert:** remove it from `critics` or `WheelFloorCritic.enabled: false`.
- `FollowPath` (transit / dock routes): the 2026-10-09 tuning is kept (model_dt 0.1 × 40, batch
  1000, wz_std 0.2, temperature 0.3) plus the critic (weight 20) and `PathAngleCritic.mode: 0`
  (the ignored `forward_preference` key removed). The study's 0.05 × 56 / temperature 0.15
  variant was better on one ideal bench and much worse with the disturbance (§8.2 G).
- `FollowCoveragePathMPPI` (rewritten; it was an unused alternative with the livelocking accel
  limits): model_dt 0.05, 40 steps (2 s), batch 800, vx 0..0.3 (vx_max set live by the
  mission), wz_std 0.3, temperature 0.1, PathAlign 30 (offset 4, step 2), PathFollow 2,
  PathAngle 4 (max angle 0.3, mode 0), PreferForward 5, Goal 5 (0.4 m), Cost 3.81 (footprint).
- `FollowCoveragePathReverseMPPI` (new): the same with vx −0.15..0 (backwards only), no
  PreferForwardCritic, PathAngle mode 1 (no direction preference).
- `mission_fsm.py:736` `coverage_controller` (`mppi` | `rpp`, **shipped `rpp`** until MPPI passes the
  gate), `mppi_follow_controller_id`,
  `mppi_reverse_controller_id`, `mppi_controller_speed_param` (`FollowCoveragePathMPPI.vx_max`),
  `mppi_speed_max_mps` 0.3: `coverage_controller_id()` (`:2925`) picks the controller per leg,
  `coverage_speed_params()` (`:2932`) routes the area cut speed and the slope caution cap to
  MPPI's vx_max. `rpp` is exactly the previous behaviour; `mppi` switches coverage to MPPI.
- `mission_fsm.py:746` `coverage_leg_pivot_rad` 1.2 / `coverage_leg_pivot_max_m` 0.5,
  `_coverage_leg_pivot()` (`:2890`): under MPPI a forward coverage leg that starts > 1.2 rad
  off the robot's yaw begins with a Nav2 Spin onto it (RPP's `rotate_to_heading_min_angle`
  1.2 did this inside the controller; MPPI does not, sim B1: it arced out of the area at the
  ring start). **Revert:** `coverage_leg_pivot_rad: 0`.
- Not changed: the turn shaper (it stays as the safety clamp; with the critic it has little
  to rewrite), `controller_frequency` 20 Hz, transit route splitting, the RPP blocks.

Tests: `test_wheel_floor_critic` (gtest, 8 cases), `test_params_yaml.py` (feasibility critic on
every MPPI controller with the shaper's track / floor, coverage model_dt = control period,
RK3588 rollout budget, no accel limits, coverage / reverse boxes, mission ids exist), `test_coverage_controller.py` (16 cases:
mppi / rpp selection forward and reverse, speed routing and cap, slope cap, leg-start pivot,
yaml).


## 7. Sim tuning log (real map, `real_run5` world, domain 176)

Tooling: the `ws/coverage-corners` sim gate (`scripts/sim_gate.sh` + `scripts/sim_gate/*.py`,
not yet committed there; used from a copy, with its `mower_sim` ground disturbance
`-d` = wheel bias +0.04, slip std 0.05, pivot efficiency 0.85) on a copy of run 5's
`areas.dat`; my metrics script adds per-class cross-track (straight / within 0.4 m of a
≥ 30° vertex / swath-end turn legs and Dubins arcs; in-place samples |v| < 0.02 excluded),
the shaper rewrite rate (every nav-lane command run through
`cmd_vel_slew.shape_for_both_wheels` with the deployed parameters), jerk, ω sign flips,
loop rate (`/cmd_vel_nav_raw` intervals), controller_server CPU (x86). Parameters were
frozen per run (`nav2_params_file` / `mission_params_file`). Short runs = the committed
gate (2 hard-boundary trips end the run in STUCK_NEEDS_HELP, i.e. the outer ring only).

| run | change | result |
|---|---|---|
| A1 | master | first ring corner (−2.02, 1.11): 2 trips, STUCK; transit −111° corner 0.193 m |
| B1 | + WheelFloorCritic, model_dt 0.05 × 56, temperature 0.15 (all MPPI) | raw rewrite 19 % → 3.8 %, one-wheel ticks 12 → 0; arcs out of the area at the **ring start** (arrives at 0.2 m/s, ring heads 146° away) |
| B2 | + nav mask as a local keepout filter | Spin aborted at once, "Optimizer fail to compute path", 10 min in TRANSIT: reverted |
| B3 | + mission leg-start pivot (1.2 rad) | ring start pivots −146°; trips at the ring **V corner** (−4.74, −1.17, −100°): cut inside, then overshot outward |
| T1 | PathAlign 40 with path orientations, PathFollow 2 | drove straight through the V corner, 0.63 m out: orientations rejected |
| T2 | PathAlign 30, PathFollow 2, PathAngle 4 / 0.3 rad, temperature 0.1, critic 30 | straights median 0.005 / p90 0.028 m; ring corners −100° / −115° still trip (overshoot) |
| T3 | T2 + accel limits 0.5 / −0.8 / 1.0 | livelock again: 80 % of the time pivoting, leg loop guard, raw rewrite 59 %: rejected |
| F | T2 with 800 × 40 everywhere (CPU) | coverage straights p99 0.038 m; **transit** with a 2 s horizon cut the area-entry leg (trip in TRANSIT) and a 0.8 m corner: transit keeps 56 steps |
| G | F, transit 700 × 56 | conditioned-path A/B (§8.2): corners good, transit overshoots with the disturbance |
| H | master transit + critic, coverage critic 100 | transit fixed; coverage too stiff with the disturbance (33 trips) |
| I | master transit + critic 20, coverage G (critic 30) | **committed** |

Transit-only bench (HOME 5–20 s into mowing, dock route through the −111° / 90° corners;
docking itself fails in this sim for every config: DOCK_MAXOUT / DOCK_STALLED, the known sim
approach issue, `wheel_smoothness_field_tests.md` "Sim fixes"): master FollowPath 0.316 m at
the −111° corner, master + critic only 0.319 m, B4 (critic, 0.05 × 56, temperature 0.15)
≤ 0.15 m, 0.05 × 56 with batch 700 but master temperature / wz_std 0.799 m. The low
temperature is what keeps the transit corner tight once the critic is on. One full run
with B4 transit (§8.1) also showed the opposite failure: after overshooting a split corner
the leg never ended and MPPI drove a 1.6 m omega loop for 25 s (no pivot sample chosen).


## 8. Sim A/B: master (MPPI transit + RPP coverage) vs MPPI for everything

Committed sim gate flow (START from the dock, full Area 1 mow, return, dock), measurement mode
for both arms: `hard_boundary_max_aborts: 100` (a trip is retraced and counted but does not
end the run in STUCK) and `return_home_on_incomplete: true`, so the whole mow is measured.
Disturbance `-d`: wheel bias +0.04, slip σ 0.05 (τ 1 s), pivot efficiency 0.85.

### 8.1 Raw paths (today's plans): A = master (MPPI transit + RPP coverage), B = MPPI everywhere

B = the first feasibility-aware set (critic, model_dt 0.05 × 56, batch 1000, temperature
0.15 / coverage 0.1). B-nocritic = B with the critic disabled (900 s).

| metric | AB-A-ideal | AB-B-ideal | AB-A-dist | AB-B-dist | AB-B-nocritic-dist |
|---|---|---|---|---|---|
| cross-track straights (med/p90/p99/max m) | 0.005 / 0.162 / 0.261 / 0.271 | 0.005 / 0.033 / 0.941 / 1.005 | 0.013 / 0.172 / 0.266 / 0.281 | 0.010 / 0.065 / 0.545 / 1.205 | 0.124 / 0.148 / 0.226 / 0.255 |
| cross-track near corners | 0.074 / 0.256 / 0.286 / 0.290 | 0.050 / 0.314 / 0.816 / 0.960 | 0.090 / 0.260 / 0.291 / 0.294 | 0.062 / 0.419 / 1.244 / 1.265 | 0.037 / 0.101 / 0.148 / 0.154 |
| cross-track swath-end turns | 0.111 / 0.214 / 0.722 / 0.748 | 0.040 / 0.165 / 0.238 / 0.239 | 0.135 / 0.227 / 0.233 / 0.243 | 0.073 / 0.586 / 1.028 / 1.131 | 0.028 / 0.122 / 0.292 / 0.325 |
| cross-track transit vs /plan | 0.031 / 0.153 / 0.215 / 0.238 | 0.090 / 1.671 / 2.412 / 2.513 | 0.078 / 0.291 / 0.629 / 0.700 | 0.185 / 1.278 / 1.853 / 1.932 | 0.050 / 0.149 / 0.188 / 0.192 |
| boundary: max outside, samples > 0.10 m | 0.106 m, 59 | 0.158 m, 279 | 0.11 m, 63 | 0.167 m, 1301 | 0.023 m, 0 |
| hard-boundary trips | 2 | 8 | 2 | 17 | 0 |
| stuck events / sub-paths NOT mowed | 0 / 2 | 0 / 2 | 0 / 2 | 0 / 2 | 0 / 2 |
| sharp corners > 0.15 m (gate e) / max dev | 9 / 0.325 m | 16 / 0.799 m | 10 / 0.402 m | 17 / 0.8 m | 1 / 0.154 m |
| shaper rewrite rate, /cmd_vel_nav MOWING | 3.3 % | 11.4 % | 2.8 % | 9.7 % | 53.9 % |
| shaper rewrite rate, /cmd_vel_nav TRANSIT | 2.4 % | 2.5 % | 1.9 % | 4.7 % | 8.8 % |
| shaper rewrite rate, raw controller out (all) | 18.5 % | 10.8 % | 18.2 % | 9.4 % | 54.5 % |
| one-wheel ticks (floor-stalled inner) | 11 (749) | 0 (318) | 145 (741) | 116 (518) | 0 (236) |
| pivot share (all / mowing) | 0.181 / 0.095 | 0.218 / 0.19 | 0.202 / 0.091 | 0.28 / 0.222 | 0.358 / 0.388 |
| jerk rms lin / ang, mowing (m/s3, rad/s3) | 0.76 / 2.49 | 0.96 / 2.31 | 0.82 / 2.96 | 1.10 / 2.83 | 0.43 / 1.97 |
| w sign flips / min (mowing / transit) | 12.2 / 5.5 | 10.2 / 5.2 | 20.0 / 10.9 | 15.2 / 8.9 | 11.4 / 5.6 |
| controller loop Hz (median / p10) | 20.0 / 19.9 | 20.0 / 18.8 | 20.0 / 19.9 | 20.0 / 18.8 | 20.0 / 19.4 |
| controller_server CPU % (median / p90, x86) | 3.0 / 10.0 | 18.2 / 27.3 | 9.1 / 13.9 | 18.2 / 24.0 | 10.0 / 20.0 |
| complete / docked / final state | False / False / MOWING_INCOMPLETE | False / False / IDLE | False / False / IDLE | False / False / IDLE | False / False / MOWING |
| run time (s) | 1262.3 | 970.3 | 661.3 | 1399.5 | 907.5 |

### 8.2 Sharp vertices pivoted (stand-in for the conditioner)

Stand-in: `ws/coverage-corners`' dropped corner-split patch (session scratchpad
`gate/dropped_corner_split.patch`), merged with this branch's controller selection:
coverage legs end on every ≥ 79° corner (mission corner rule, 0.08 m), then a Nav2 Spin onto
the next leg; it does not split turn legs ≤ 1.2 m or connector joins. Swapped in for these
runs only, not committed. A = RPP coverage (master nav2), G = MPPI everywhere (transit
0.05 × 56 × 700, temperature 0.15), H = master-tuned transit + critic, coverage critic weight
100, **I = the committed MPPI set** (master-tuned transit + critic 20, coverage G with critic 30).

| metric | CS-A-ideal | CS-G-ideal | CS-I-ideal | CS-A-dist | CS-G-dist | CS-H-dist | CS-I-dist |
|---|---|---|---|---|---|---|---|
| cross-track straights (med/p90/p99/max m) | 0.003 / 0.011 / 0.120 / 0.237 | 0.006 / 0.016 / 0.106 / 0.235 | 0.005 / 0.017 / 0.093 / 0.237 | 0.013 / 0.032 / 0.273 / 0.404 | 0.008 / 0.030 / 0.209 / 0.240 | 0.010 / 0.041 / 0.290 / 0.488 | 0.009 / 0.034 / 0.272 / 0.357 |
| cross-track near corners | 0.016 / 0.066 / 0.180 / 0.201 | 0.013 / 0.065 / 0.099 / 0.135 | 0.018 / 0.095 / 0.664 / 0.867 | 0.017 / 0.065 / 0.257 / 0.269 | 0.014 / 0.080 / 0.196 / 0.236 | 0.022 / 0.470 / 2.932 / 3.156 | 0.018 / 0.087 / 1.159 / 1.401 |
| cross-track swath-end turns | 0.186 / 0.250 / 0.771 / 0.810 | 0.050 / 0.371 / 0.502 / 0.527 | 0.547 / 2.057 / 3.005 / 3.284 | 0.161 / 0.279 / 0.598 / 0.660 | 0.065 / 0.341 / 0.470 / 0.503 | 0.689 / 2.486 / 3.243 / 3.412 | 0.073 / 1.196 / 2.360 / 2.481 |
| cross-track transit vs /plan | 0.068 / 0.215 / 0.398 / 0.433 | 0.092 / 1.164 / 2.079 / 5.194 | 0.088 / 0.360 / 0.638 / 3.502 | 0.081 / 0.279 / 0.630 / 0.689 | 0.256 / 1.535 / 2.411 / 2.536 | 0.058 / 0.240 / 0.323 / 0.342 | 0.068 / 0.277 / 0.646 / 1.235 |
| boundary: max outside, samples > 0.10 m | 0.0 m, 0 | 0.156 m, 177 | 0.393 m, 2406 | 0.0 m, 0 | 0.173 m, 338 | 0.173 m, 2231 | 0.16 m, 703 |
| hard-boundary trips | 0 | 5 | 16 | 0 | 9 | 33 | 5 |
| stuck events / sub-paths NOT mowed | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 3 | 1 / 1 | 0 / 1 | 0 / 1 |
| sharp corners > 0.15 m (gate e) / max dev | 6 / 0.402 m | 5 / 0.8 m | 11 / 0.483 m | 4 / 0.307 m | 12 / 0.8 m | 8 / 0.424 m | 4 / 0.425 m |
| shaper rewrite rate, /cmd_vel_nav MOWING | 2.1 % | 8.9 % | 9.1 % | 4.6 % | 9.1 % | 2.8 % | 7.7 % |
| shaper rewrite rate, /cmd_vel_nav TRANSIT | 3.0 % | 1.8 % | 2.5 % | 4.9 % | 3.2 % | 1.9 % | 2.4 % |
| shaper rewrite rate, raw controller out (all) | 5.5 % | 8.6 % | 9.4 % | 5.8 % | 8.4 % | 3.3 % | 8.2 % |
| one-wheel ticks (floor-stalled inner) | 74 (810) | 124 (440) | 121 (529) | 111 (2234) | 3 (436) | 86 (390) | 161 (444) |
| pivot share (all / mowing) | 0.254 / 0.23 | 0.232 / 0.209 | 0.283 / 0.231 | 0.241 / 0.203 | 0.253 / 0.217 | 0.265 / 0.194 | 0.285 / 0.25 |
| jerk rms lin / ang, mowing (m/s3, rad/s3) | 0.61 / 1.88 | 0.96 / 2.45 | 0.97 / 2.37 | 0.89 / 2.90 | 0.98 / 2.77 | 1.19 / 2.75 | 1.00 / 2.69 |
| w sign flips / min (mowing / transit) | 9.0 / 8.8 | 9.0 / 9.5 | 11.8 / 8.6 | 25.0 / 10.9 | 17.3 / 7.8 | 17.5 / 10.1 | 15.2 / 12.4 |
| controller loop Hz (median / p10) | 20.0 / 19.9 | 20.0 / 19.0 | 20.0 / 19.0 | 20.0 / 19.9 | 20.0 / 18.9 | 20.0 / 18.9 | 20.0 / 19.0 |
| controller_server CPU % (median / p90, x86) | 9.0 / 10.0 | 18.2 / 22.0 | 10.0 / 22.0 | 8.0 / 10.0 | 16.0 / 20.0 | 14.0 / 23.0 | 17.0 / 23.0 |
| complete / docked / final state | True / False / IDLE | True / False / IDLE | True / True / CHARGING | False / False / IDLE | True / False / STUCK_NEEDS_HELP | True / False / IDLE | True / False / IDLE |
| run time (s) | 965.5 | 1113.5 | 1495.7 | 1225.8 | 1319.1 | 1593.5 | 1142.5 |

Reading the tables:

- The **critic** is what removes the dither (rewrites 54 % → 9 %). A heavier weight (H, 100)
  takes coverage rewrites down to 2.8 %, but coverage tracking under the disturbance then
  collapses: 33 trips, MPPI will not make the small slow corrections. 30 is the compromise.
- The **transit** settings that helped corners on an ideal drive (G: low temperature,
  0.05 × 56) produce omega loops and junction overshoots under the disturbance: 9 trips,
  transit p90 1.5 m. Master's transit tuning + the critic (H, I) is as good as master
  (p90 0.24–0.28 m). That is what is committed.
- **Swath-end turn legs** are now MPPI's weak spot (cross-track p90 up to 1.2–2.1 m, the
  turn-leg trips): short Reeds–Shepp pieces near the boundary that end on a 0.08 m leg
  checker. RPP drives them at a constant 0.14 m/s minimum and arrives. MPPI decelerates into
  the floor band (47 % of its turn-leg ticks in G are undrivable), then overshoots.
- **Loop rate**: 20 Hz median, p10 18.6–19.0 Hz on x86 for MPPI. controller_server CPU
  9–18 % (median) vs RPP coverage 3–9 %. The RK3588 is 3–4× slower per core (§10).
- **One-wheel ticks** (cmd with inner wheel in (−0.06, 0)) mostly come from Spin / retrace
  ramps through the slew, in both arms. MPPI's own output had 0 in the ideal raw-path run.
- **Docking**: only CS-I-ideal docked (and was charging). Every other run ended in the sim's
  known docking-approach failure, whatever the tracker.


## 9. Raw paths, and what path conditioning has to add

On the raw paths of today's plans MPPI fails exactly where RPP fails, and worse:

- **Lines**: MPPI is better. Coverage straights median 0.005 m, p90 0.03–0.07 m, p99
  0.04 m on runs without corner bleed (F, T2), against RPP's 0.16 m p90 in the same sim and
  0.074 m p90 in field run 5.
- **Swath-end turns (Dubins / Reeds–Shepp legs, r ≥ 0.5 m)**: MPPI similar or better
  (ideal: p99 0.24 vs RPP 0.39–0.72 m), because the drivable arcs need no rewrite.
- **Sharp vertices** (outer ring −100° / −115° / −141° / +137°, the inner ring's +81/+99°
  notches, the dock path's −111° / 90°): MPPI cuts inside, then cannot turn tighter than
  r = 0.44 m without stopping, overshoots outward and trips the boundary (8 trips ideal,
  17 with the disturbance, corner deviations up to 0.8 m). Without the critic it does not
  overshoot: it dithers at the vertex instead (B-nocritic: 54 % of its commands rewritten
  by the shaper, the same V corner never passed, 4 aborts, sub-path NOT mowed, 341 s on it).
  Gaussian sampling around a moving warm start does not propose "decelerate straight, stop,
  pivot ≥ 0.25 rad/s, go" (MowgliNext #310 found the same), and every intermediate sample
  of that manoeuvre is undrivable.

What the path conditioner (design §3.1) has to deliver for MPPI, measured in §8.2 with a
stand-in (`ws/coverage-corners`' dropped corner split: legs end on every ≥ 79° coverage
corner, pivot in place, follow on):

1. **Pivot markers at every vertex MPPI cannot arc** — i.e. every corner whose fillet does not
   fit. MPPI must never be handed a vertex; it is handed a line that ends at the vertex, the
   mission pivots (Nav2 Spin, both wheels ≥ 0.25 rad/s), then the next line.
2. **Fillets with r ≥ 0.5 m** where they fit: MPPI tracks those as well as lines (the arc
   class above).
3. **Leg ends on vertices with a tight checker** (the corner rule's 0.08 m), so MPPI is never
   asked to come back to a goal it passed (the omega loop of §7).
4. **A speed profile** (v ≤ 0.3·r, lower before a pivot) only as an upper bound on vx_max per
   leg: MPPI has no curvature-dependent speed regulation of its own.


## 10. Open risks

- **Turf, slope, RTK.** The sim floors the wheel at 0.06 m/s and (with `-d`) adds bias, slip and
  85 % pivot efficiency; it has no slope, no caster drag, no grass-dependent floor, no RTK
  loss. The field rewrite rate of master MPPI transit was 24–84 %, the sim's 2–7 %: the sim
  under-reproduces the dither regime, so the critic's field benefit is likely larger than
  measured here, and its weight (20 / 30) may need a field check.
- **CPU on the RK3588.** Master transit (1000 × 40) already ran at 17.5 Hz. Transit keeps
  that budget (40 000 rollout points); coverage MPPI is 32 000 (800 × 40); the critic adds one
  pass over batch × steps (cheaper than any path critic).
  x86 controller_server CPU: master RPP coverage 3–9 % median, MPPI coverage 9–18 %. MPPI
  for coverage runs MPPI the whole mow instead of only in transit, so the RK3588 sees the
  controller's load continuously; watch "Control loop missed its desired rate" and the
  loop rate in the first field bag, lower `batch_size` first (MowgliNext had to go to
  800 × 25 on the same SoC).
- **Accel model.** MPPI's model accelerates at 3 m/s²; the real chain (velocity_smoother
  1.0 / −1.5, cmd_vel_slew 0.5 / −0.8, wheel lag 0.15 s) is far slower. Matching it
  livelocks (T3). The mismatch is a known Nav2 limitation (#5214); it costs a few cm at
  stops, not stability.
- **Merging with `ws/coverage-corners`.** My `coverage_leg_pivot_rad` (MPPI only, leg starts) and
  their dropped `coverage_start_pivot_min_rad` do the same thing; the conditioner's pivot
  markers should replace both.
  *Resolved on `ws/integrate` (2026-10-16): `coverage_leg_pivot_rad` / `_max_m` removed;
  coverage and route leg starts pivot by the conditioner's `path_conditioning_pivot_min_rad`
  for both controllers. Short swath-end turn legs no longer exist with conditioning (the
  Reeds-Shepp chain is replaced by conditioned swath -> connector -> swath geometry) and leg
  ends are judged along-path, which removes the §8.2 MPPI turn-leg failure; gate results in
  `simulation.md` "Gate runs on ws/integrate".*
- **Docking.** Every arm ended NAV_TO_DOCK_FAILED / DOCK_MAXOUT / DOCK_STALLED in this sim
  (known sim docking-approach issue), so "docked" is not discriminating here.
- **MOWING_INCOMPLETE.** Both arms lost the outer ring ("follow_path keeps reporting success
  with 10–18 m left") after the boundary retraces: a mission issue, not the tracker.


## 11. 2026-10-28 weave retune

Field bag 2026-10-28, straight swaths: cross-track p50 1.3 / p90 3.5 / max 9.9 cm, weave period
about 6.4 s (1.3 m), commanded w ~ -3.98 e - 0.57 psi (underdamped), 37 % of the w variance
MPPI sampling noise, and velocity_smoother's 0.01 rad/s angular deadband zeroing MPPI's small
corrections. Three tweaks were tried, cumulative, in the sim (`real_run5`, x3 unless noted).
Metrics on the 17 straight runs of the mow plan (56 m, >= 1.5 m long, 0.4 m trimmed at each
end), cross-track of `/sim/ground_truth` to the planned line; w = `/cmd_vel_nav_raw` on those
runs; band = rms of w in 0.1-0.3 Hz; e / arc = gate criterion e max and h arc p99.

| Run | Config | XTE p50 / p90 / max cm | w std rad/s | band rms | e max m | h arc p99 m | Gate |
|---|---|---|---|---|---|---|---|
| w0-base | master, ideal | 0.41 / 1.12 / 2.97 | 0.036 | 0.014 | 0.060 | 0.041 | only i |
| w1-6c-r2 | + deadband 0.002, ideal | 0.35 / 1.08 / 1.87 | 0.032 | 0.009 | 0.098 | 0.081 | only i |
| w2-6b | + wz_std 0.2, ideal | 0.39 / 0.76 / 2.65 | 0.023 | 0.007 | 0.072 | 0.084 | only i |
| w3-6a1 | + PathAlign orientations, ideal | 0.86 / 1.75 / 3.50 | 0.019 | 0.004 | 0.062 | 0.048 | only i (reverted) |
| w4-6a2 | + PathAngle 0.05 rad / w 8, ideal | 0.38 / 0.73 / 2.39 | 0.027 | 0.008 | 0.083 | 0.080 | only i (reverted) |
| w6-base-l | master, `-l` | 1.32 / 3.18 / 7.50 | 0.081 | 0.052 | 0.099 | 0.056 | only i |
| w7-head-l-r2 | 6c + 6b, `-l` | 1.26 / 2.93 / 5.66 | 0.070 | 0.044 | 0.101 | 0.095 | only i |
| w5-final | 6c + 6b, `-d -s 1 -x 1` | 1.24 / 3.79 / 10.79 | 0.090 | 0.064 | 0.133 | 0.091 | only i (a 0.087, b 0, docked) |

- **The ideal sim barely weaves** (p90 1.1 cm): the field weave needs localisation error.
  With `-l` / `-d` (loc_drift) the sim reproduces the field almost exactly (p90 3-4 cm, max
  8-11 cm, w peak at 6.4 s with `-d`), and the stack's own map pose shows p90 2.5-3.0 cm, so
  most of the field weave is the controller following the EKF map pose's 1-3 cm wander, not
  an intrinsic MPPI oscillation. No controller retune removes that part; RTK / EKF smoothing
  would.
- **6c (deadband 0.01 -> 0.002)**: zeroed commands on the straights 35 % -> 7 %, 0.1-0.3 Hz
  rms -35 %, max -37 %. Kept.
- **6b (wz_std 0.3 -> 0.2)**: w std -28 %, XTE p90 -30 % (ideal). Kept.
- **6a1 (PathAlignCritic use_path_orientations)**: removes the weave but the heading term
  weakens the lateral pull: standing per-swath offsets up to 2.3 cm, absolute p90 doubles.
  Reverted.
- **6a2 (PathAngleCritic 0.05 rad, weight 8)**: no reduction in the sim (w std up), more
  error on fillets. Reverted; its threshold is above the sim's heading error, so the sim
  cannot show a field benefit either way.
- **6c + 6b under `-l`**: p90 -8 %, max -25 %, per-swath max p90 6.2 -> 4.7 cm, w std -14 %.
- **Cost: the saturated fillet after sub-path 0's 94 deg corner** (about (-0.5, 4.5), R about
  0.5 m, w at the 0.3 limit). The head keeps more speed into it and runs wide: 0.084-0.116 m
  in all four runs with 6c (w1, w2, w5, w7-r2), <= 0.05-0.07 m in four runs without it
  (w0-base, w6-base-l and two other-branch ideal runs). h arc p99 0.08-0.095
  (gate 0.10) and e stay in limits; watch that fillet in the field.


## References

1. Nav2 MPPI README and source, jazzy branch:
   https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_mppi_controller
   (README "Notes to Users", `src/optimizer.cpp`, `src/critics/*.cpp`); docs page
   https://docs.nav2.org/configuration/packages/configuring-mppic.html.
2. VelocityDeadbandCritic, PR #4256: https://github.com/ros-navigation/navigation2/pull/4256
3. nav2 issues #5925 (U-turn tracking), #5652, #5303, #5214, #6055 (arc-length progress),
   #6228 / #4513 / #4621 (CPU): https://github.com/ros-navigation/navigation2/issues/
4. Pluggable motion models: issue #6032, PR #6076 (main, not Jazzy).
5. MowgliNext PRs #224, #226, #230, #294, #310, #626 and docs/NAV2_LYRICAL_CONTROLLER_REVIEW.md:
   https://github.com/mowglinext/mowglinext
