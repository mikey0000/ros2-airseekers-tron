# Row weave: heading estimate check and an FTC row controller, 2026-11-05

Branch `ws/ftc1105`. Follow-up to `2026-10-29_localisation_wander.md` (sections 1, 6 and 7).
About 2 cm p90 of weave is left on straight mow rows after the MPPI retune (in-band -32 %), the
look-ahead control point (-3 to -9 % in the field) and the GNSS smoothing A/B (no effect).

Two questions:

1. Does the heading estimate wander at the weave frequency (estimator side), or does the
   controller act on a good heading (controller side)?
2. If it is the controller: does OpenMower's FTC ("follow the carrot") law, used on the rows
   only, hold the line better than the coverage MPPI?

**Verdict.**

1. Controller side. The fused yaw is the gyro integral to within 0.3 deg in the band, while the
   heading itself swings 3.4-3.8 deg rms. The fused yaw follows the heading implied by the RTK
   track with coherence 0.96-0.98 at the weave peak. What separates the two is a 0.35 s lead of
   the body over its own ground track: the fix latency and the track lag. That is the in-loop
   lag found on 10-29, not an estimator error. The sim's heading error is 0.01 deg and it still
   weaves.
2. In the sim the FTC row law halves the in-band weave: band rms -46 % on `real_run5` and -51 %
   on `owner_far`, row XTE p90 -33 % / -29 %. MOWING time is 13-19 % shorter because the rows
   now run at the 0.3 m/s cut speed (MPPI held about 0.2). The cost: arc p99 +2 cm (bends
   entered faster), still inside h. One MPPI-only corner excursion on `real_run5` fails h-line
   there. Recommendation: field A/B.

## 1. Phase 1: heading estimate against the RTK track

### Data and method

| Data | Straights | What |
|---|---|---|
| Field 2026-10-28 (`bags/mower/bags/20261028-023252`, 101 of 107 splits readable) | 60 / 380 m, median 28 s | long swaths at 0.21 m/s, before B13-1 |
| Field 2026-11-04 (area 3, 72 splits) | 34 / 123 m, median 14 s | short cross-mode rows, master with B13-1 and B9 |
| Sim master `real_run5 -l -x 3 -s 1` | 12 | field localisation model |

Straights: RTK track pieces while the mission is MOWING and the command is forward, split by
Douglas-Peucker (8 cm), at least 3 m (11-04: 2 m), quadratic residual under 4 cm, 1 s trimmed at
the start and 0.5 s at the end, resampled to 10 Hz. Each signal is linearly detrended per
straight and band-passed to 0.05-0.4 Hz (2.5-20 s). Spectra are Hann-windowed per straight and
averaged over the straights.

| Signal | What it is |
|---|---|
| track heading | the body heading the RTK antenna track implies: `e_ant' = v psi + 0.098 psi'`, solved in the frequency domain (`Psi = jw E_ant / (v + jw 0.098)`); independent of the IMU |
| EKF yaw | `/odometry/filtered_map` yaw relative to the straight |
| raw IMU yaw | EKF yaw minus the heading_aligner offset (`/heading_aligner/status`, 1 Hz) |
| gyro integral | integral of `/odometry/filtered` wz (IMU rate fused with wheel rate) |
| wheel integral | integral of `/mcu/measured_speed` wz |
| lateral | RTK antenna lateral from the straight, and the axle lateral (antenna minus 0.098 m along the EKF yaw) |
| w | `/cmd_vel_nav` (controller output) |

`/imu/data` is not recorded in either bag, so "raw IMU" is rebuilt from the aligner offset, as
on 10-29. Scripts (session scratchpad `weave1105/scripts/`): `extract.py` (bag -> npz, dev
image), `phase1.py` (straights), `spec.py` (spectra, coherence, gain fits), `resid.py` (lag fit
and loop model), `simhead.py` (sim contrast).

### Results

| | Field 10-28 | Field 11-04 |
|---|---|---|
| Axle lateral, band rms (cm) | 1.74 | 1.85 |
| Lateral spectral peak | 6.0 s | 11.4 s (short rows) |
| Track heading, band rms (deg) | 3.99 | 3.83 |
| EKF yaw, band rms (deg) | 3.79 | 3.37 |
| EKF minus raw IMU yaw (deg rms) | 0.10 | 0.10 |
| EKF minus gyro integral (deg rms) | 0.28 | 0.32 |
| EKF minus wheel integral (deg rms) | 1.79 | 1.82 |
| EKF minus track heading (deg rms) | 2.34 | 2.29 |
| coherence EKF yaw / track heading, 0.125-0.25 Hz (4-8 s) | 0.96 | 0.94 |
| coherence, 0.05-0.125 Hz (8-20 s) | 0.84 | 0.92 |
| EKF / track transfer: gain, phase (0.125-0.25 Hz) | 0.93, +22.5 deg | 0.84, +23.3 deg |
| EKF minus track = tau x yaw rate: tau, R² | 0.36 s, 0.42 | 0.34 s, 0.30 |
| remainder after the lag (deg rms, upper bound) | 1.78 | 1.91 |
| lateral the remainder drives through the loop (band rms) | 0.52 cm (30 %) | 0.53 cm (29 %) |
| w fit on what the controller sees: Ky, Kpsi, R² | 4.68, 0.85, 0.89 | 3.55, 0.87, 0.79 |

Sim master (`branch-l3` data): lateral band rms 0.96 cm, heading 1.93 deg, heading the
controller sees minus truth 0.012 deg.

What the numbers say:

- **The heading estimate is the gyro.** Fused, raw and gyro-integrated yaw agree to 0.1-0.3 deg
  in the band against 3.4-4.0 deg of real heading weave. The heading aligner's COG steps are
  0.1 deg. The wheel heading differs by 1.8 deg (wheel slip and track width) and is not used for
  yaw.
- **The fused yaw tracks the body.** Coherence with the RTK track heading is 0.94-0.98 where the
  weave lives, at gain 0.84-0.93. Below 0.1 Hz coherence falls to 0.75-0.85 on 10-28, where
  RTK noise differentiated over short straights dominates the track heading.
- **The difference is a lead, not a wander.** The fused yaw leads the track heading by about
  23 deg of phase at the weave peak, a 0.34-0.36 s lead. That matches the 0.1 s fix latency plus
  the 0.055 m track lag at 0.21 m/s (0.26 s) measured on 10-29. A gyro cannot lead the true
  heading; the RTK track lags it. This is the in-loop lag of 10-29 §1 seen from the heading
  side.
- **The rest cannot drive the weave.** After the lag, 1.8-1.9 deg rms remain. That is an upper
  bound on any estimator error: it also holds the RTK noise differentiated into the track
  heading. Fed as a heading error into the loop with the fitted gains and a 0.2 s delay, it
  produces 0.52-0.53 cm band rms of lateral, 29-30 % of the observed amplitude (9 % of the
  power).
- **The sim weaves without any heading error.** Its controller heading is the truth to
  0.012 deg, and the lateral still has 0.96 cm band rms.
- **The controller holds the row loosely.** On what it sees, the coverage MPPI acts like
  `w = -Ky lat - Kpsi yaw` with Ky 3.5-4.7 and Kpsi 0.85-0.87, before and after the 0.25 m
  control point (B13-1). Ky barely moved, and so did Kpsi, which the control point was meant to
  raise. The lightly damped lateral mode of 10-29 is still there.

**Conclusion:** the weave is the controller acting on a good heading. No fusion change is
indicated: IMU yaw weighting and GNSS-course gating would act on a 0.1-0.3 deg term. Phase 2
goes ahead.

## 2. Phase 2: an FTC row controller

### 2.1 Source and what was ported

OpenMower's `ftc_local_planner` (`github.com/ClemensElflein/open_mower_ros`, commit `a69d144`,
`src/lib/ftc_local_planner/src/ftc_planner.cpp`, by Clemens Elflein; the repository is GPLv3).
It was read only, not built or run. The ported part is the FOLLOWING state:

- a carrot (control point) moves along the path at the target speed, ramping at
  `acceleration`;
- its pose in base_link gives `lon = x`, `lat = y`, `ang = yaw`;
- `v = kp_lon lon` (forward only), and
  `w = kp_ang ang + kd_ang d(ang) + kp_lat lat + ki_lat ∫lat + kd_lat d(lat)`, with the
  integral clamped to `±ki_lat_max`.

The law is in `mowgli_nav2_plugins/include/mowgli_nav2_plugins/ftc_row_law.hpp`, with the GPL
notice and the attribution. The repository already had a FTC port (MowgliNext's, deleted
2026-10-23). It drove whole coverage paths and left the ring at corners: its carrot turned at
45 deg/s, four times what the Tron can turn (README history before `b2be81f`). That is why this
port keeps FTC to the straight rows.

Not ported: PRE_ROTATE / POST_ROTATE / WAITING_FOR_GOAL_APPROACH, the `speed_angular` carrot
limit, `speed_fast` / `speed_slow`, `lateral_priority_*`, the oscillation recovery and the
crash detection at `max_follow_distance`. Corners, goals and blocked rows belong to MPPI here.

Port changes:

- The carrot stays between the robot's foot point and 1.5 × `vx_max / kp_lon` ahead of it.
  Upstream lets it run away and then declares a crash. Here it waits when the collision
  monitor's slowdown zone holds the robot back.
- Path tangents come from the path geometry, not from the pose orientations (MPPI uses
  `use_path_orientations: false` on the same paths).
- The gains are the Tron's own. A closed-loop row model (unicycle, 0.2 s command delay, 0.1 s
  wheel lag, 0.055 m track lag, 0.026 m turn centre, 0.1 s fix latency, 10 Hz pose, the sim's
  OU noise, 0.3 rad/s clamp; `weave1105/scripts/gainmodel.py`) gives:

  | Law | p90 lateral at 0.21 / 0.30 m/s (cm) |
  |---|---|
  | linear w = -3.5 lat - 0.87 yaw (field MPPI fit) | 2.41 / 3.39 |
  | **FTC kp_lat 2.5, kp_ang 1.2, lead 0.25 m (chosen)** | **1.43 / 1.58** |
  | FTC kp_lat 3.2, kp_ang 1.1 | 1.52 / 1.69 |
  | FTC kp_lat 3.2, kp_ang 0.6 | 1.74 / 1.92 |
  | FTC kp_lat 3.2, kp_ang 1.1, ki_lat 0.3 | 1.67 / 1.82 |
  | OpenMower stock (kp_lat 25, kp_ang 3, kd_lat 1.23, kp_lon 4) | 4.56 / 74 (unstable) |

  On a straight the carrot law is `w = -kp_lat e - (kp_ang + kp_lat × lead) psi`. The carrot
  lead (`v / kp_lon`, 0.25 m at 0.3 m/s) is the look-ahead control point, and `kp_ang` adds
  heading damping directly. MPPI's critics could not be made to give that (section 1: Kpsi
  0.85 with or without the 0.25 m control point). `kd` is 0: difference terms on a 10 Hz map
  pose are steps. `ki_lat` is 0 for now: the integral costs damping in the model. On a side
  slope the row would then sit about 2 cm downhill at 1 cm/s creep, where MPPI's path trim
  takes it out (see 2.5).

### 2.2 FTCRowController: who drives when

`mowgli_nav2_plugins::FTCRowController` (`ftc_row_controller.{hpp,cpp}`) is a Nav2 controller
for the coverage controller id. It is selected by one line: the `plugin:` of the
`FollowCoveragePathMPPI` block in `nav2_params.yaml`.

- **Inner MPPI, same name.** It loads `nav2_mppi_controller::MPPIController` through pluginlib
  and configures it under the same name, as Nav2's RotationShimController does with its primary
  controller. MPPI therefore reads every existing `FollowCoveragePathMPPI.*` parameter. The
  mission's live `FollowCoveragePathMPPI.vx_max` (cut speed) and `setSpeedLimit` reach it
  unchanged. The mission, its controller ids and its goal checkers are untouched.
- **MPPI runs every cycle.** Its warm start stays current. Its CostCritic and its collision
  and no-valid-control exceptions are unchanged, so a blocked row aborts the follow exactly as
  before and the mission's blocked-wait and detour logic applies. CPU cost is MPPI's plus a
  path transform and a projection.
- **The row law drives only on a straight row.** Every cycle the plan is put into the pose
  frame and the robot is projected onto it, searching forward only (1 m window from the last
  foot point, so the neighbouring row 0.18 m away is never picked).
  - **Entry:** the path is straight (all vertices within `straight_tolerance` 1 cm of the
    chord) for `handoff_distance + 2 × lead` (0.4 + 0.75 m at 0.3 m/s) past the foot point,
    the robot is within half of `max_lateral_error` (0.15 m) and `max_angle_error`
    (0.35 rad), and the footprint is below inscribed cost along that stretch of the local
    costmap. The carrot then starts at the steady lead for the current speed.
  - **Exit to MPPI:** a bend or the path end comes within `handoff_distance` of the carrot,
    the robot leaves the lateral or angle limit, or the footprint along the row ahead reaches
    inscribed cost (`row ahead blocked`).
- **The chain after the controller is unchanged:** velocity_smoother, collision_monitor (its
  stop and slowdown zones), twist_mux, cmd_vel_slew, mcu_node.

How the coverage path's corners are handled:

| Path feature | Who drives |
|---|---|
| Straight rows, straight edge-lap sides | row law, from entry to `handoff_distance` before the bend |
| Fillets, Dubins swath turns, arcs on edge laps | inner MPPI (not straight) |
| Pivot vertices of the conditioned path | the mission's Spin between legs; the leg into a vertex is MPPI for its last 0.4 m and ends on `coverage_corner_goal_checker` as before |
| Reverse legs of K-turns / Reeds-Shepp turns | `FollowCoveragePathReverseMPPI`, unchanged |
| Short forward turn legs (≤ 1.2 m) | MPPI: shorter than the entry length |
| Chunk ends mid-row, sub-path ends | MPPI for the last 0.4 m, then the goal checker |
| Obstacle on the row | MPPI from `row ahead blocked`; collision monitor and mission unchanged |
| Transit, detours, docking | `FollowPath` (MPPI), unchanged |

A/B: `plugin: "nav2_mppi_controller::MPPIController"` in that block gives master's behaviour
(the `FTC` sub-block is then never read).

**Parameters (A6).** Twelve new keys under `FollowCoveragePathMPPI.FTC`: seven gains,
`acceleration`, and four gate values. Nothing could be retired. MPPI still drives every bend, so
all of its parameters stay in use, and no existing key is a PD gain or a row gate. The row speed
and turn limit reuse MPPI's `vx_max` / `wz_max` instead of new keys, and the projection's first
search reuses `max_robot_pose_search_dist`.

### 2.3 Tests

`test_ftc_row_law` (gtest, no ROS graph):

- path point and tangent, duplicate poses;
- projection signs, and the forward window staying on its row next to the neighbouring row;
- straight-ahead test at the bend and at the path end;
- the PID terms against the upstream formula (derivatives, integral clamp, forward only,
  limits);
- carrot ramp and clamp;
- gate hysteresis and the hand-off before a bend;
- closed loop on a unicycle with the 0.2 s delay and the 0.1 s lag: from 8 cm / 0.15 rad off it
  settles to under 2 mm and holds 0.3 m/s at a 0.25 m lead.

`test_params_yaml`: the coverage block uses FTCRowController, transit and reverse stay plain
MPPI, the gains and the gate are in range. `scripts/ci_test.sh` passes (3861 cases).

### 2.4 Sim results

`-l -x 3 -s 1` (field localisation model), branch `ws/ftc1105` against the master baselines.
Weave columns are `weave.py` (truth against the sent plan, straights ≥ 1.5 m after a 0.4 m
trim). The evaluator criteria are from `verdict.txt`. Runs are in
`ros2_stack-wt/simout_ftc1105/` (bags pruned). The controller loop held 20 Hz sim, "desired
rate of 60" was in launch.log, and the power profile was `performance`.

| | real_run5 master | **real_run5 FTC** | owner_far master | **owner_far FTC** |
|---|---|---|---|---|
| row XTE p50 / p90 / max (cm) | 1.05 / 2.79 / 5.47 | **0.82 / 1.87 / 3.96** | 1.09 / 2.66 / 11.93 | **0.78 / 1.89 / 11.97** |
| detrended p50 / p90 (cm) | 0.66 / 1.82 | **0.39 / 0.89** | 0.90 / 2.40 | **0.51 / 1.31** |
| lateral band rms 0.08-0.4 Hz (cm) | 0.84 | **0.45** | 0.96 | **0.47** |
| w band rms 0.1-0.3 Hz (rad/s) | 0.0305 | **0.0159** | 0.0333 | **0.0133** |
| a max out / > 0.10 m | 0.089 / 0 | 0.074 / 0 | 0.079 / 0 | 0.070 / 0 |
| e corners > 0.15 m (max) | 0 of 21 (0.091) | 0 of 21 (0.110) | **1 of 84 (0.181) FAIL** | **0 of 84 (0.138)** |
| h p99 line / arc | 0.085 / 0.050 | **0.109 FAIL** / 0.073 | 0.068 / 0.048 | 0.074 / 0.067 |
| h line / arc max | 0.125 / 0.059 | 0.194 / 0.208 | 0.383 / 0.054 | 0.285 / 0.078 |
| i side / nose / sweep | 0.086 / 0.055 / 0.119 FAIL | 0.104 / 0.049 / 0.092 FAIL | 0.096 / 0.119 / 0.153 FAIL | 0.127 / 0.112 / 0.101 FAIL |
| b trips / d STUCK | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| coverage pivots; g pivot share | 28; 25 % | 29; 27 % | 94; 14 % | 93; 17 % |
| lawn cut (truth) | 97.0 % | 96.8 % | 99.2 % | **99.5 %** |
| MOWING time | 777 s | **676 s** | 3238 s | **2613 s** |
| row speed (truth, straight) | 0.20 m/s | 0.30 m/s | | 0.30 m/s |
| row law share of coverage cycles | - | 27-86 % per goal | - | 73 % (71 row episodes) |
| verdict | FAIL (i) | FAIL (h, i) | FAIL (e, i) | FAIL (i) |

What the runs show:

- **The weave halves.** In-band lateral -46 % / -51 %, w in the band -48 % / -60 %, detrended
  p90 -51 % / -45 %. The XTE p90 falls less (-33 % / -29 %) because it includes the hand-off
  stretches and the per-row standing offsets. The owner_far max (12 cm) is the same edge-start
  join in both runs, not a row.
- **The real_run5 h-line fail is not the row law.** The worst line samples (0.19 m) and the arc
  max (0.21 m) are on sub-path 8's 1.1 m leg into the +161 / -179 deg pivot vertices. That leg
  is shorter than the 1.15 m entry length, and the controller's log shows no row episode after
  sim 622 s; the excursion is at 638-644 s. It is MPPI on its own, on a corner master passes
  at 0.056 m in its run. It also weighs more in p99 here because the faster rows leave 20 %
  fewer line samples. The other line outliers (0.09-0.14 m at sub-path starts after pivots)
  are in both runs.
- **Bends are entered faster.** MPPI takes over 0.4 m before a bend at the row's 0.3 m/s
  instead of its own 0.2 m/s: arc p99 0.050 -> 0.073 and 0.048 -> 0.067, still inside the
  0.10 m limit. Upstream FTC slows its carrot to `speed_slow` before a bend; doing the same
  before the hand-off is the obvious next step if the field shows the arcs wider.
- **Corners, pivots, coverage.** e passes on owner_far (master fails it there), the pivot count
  is unchanged (29 / 93), there are no trips or STUCK, and cut is equal or better.
- **i still fails as on master.** The owner_far side worst (0.127 m at -68 deg) is at a pivot
  site the master run also flags (its nose / sweep worst).

What the sim cannot show: the field heading lag is modelled (track lag, fix latency), but slope,
slip and turf are not. B13-1 predicted -45 % in this sim and gave -3 to -9 % in the field, so
the sim gain is an upper bound. The row law is deterministic, though, so its gains are what the
robot gets; MPPI's effective gains were not.

### 2.5 Recommendation and field A/B

**Field A/B, not a straight merge.** The sim halves the weave and keeps the corners and
coverage. The open questions need turf:

- the faster rows (0.3 m/s is the cut speed the owner set; under MPPI the robot did 0.2);
- arc width after the faster hand-off;
- side slopes with `ki_lat` 0 (MPPI's path trim does not act on the rows any more).

Procedure:

1. Deploy `ws/ftc1105`. It needs `mowgli_nav2_plugins` and `mower_navigation` rebuilt.
2. Mow area 3 or 4 as in the 10-31 / 11-04 mows.
3. Run `scripts/wheel_cal.py` on the session. Weave p50 / p90 against 11-04 (0.72 / 1.91 cm,
   `--min-len 1.5`). Success: p90 -30 % or better, no new boundary events, row ends and arcs
   not visibly wider.
4. Check `controller_server`'s `FTCRowController ... plan done: rows N %` lines (row share)
   and `row -> MPPI (...)` reasons in the log.
5. Revert (one line): `plugin: "nav2_mppi_controller::MPPIController"` in the
   `FollowCoveragePathMPPI` block of `nav2_params.yaml`.

If the rows should keep MPPI's 0.2 m/s for the A/B, set the area's cut speed to 0.2 in the GUI:
the row speed is `vx_max`.
