# Coast mode: keep mowing through a short RTK loss on dead reckoning (2026-11-25)

Branch `ws/coast1125` (from `ws/drdrift1113` = master 2ca8d8a + the drift study). Unit-tested,
bag-replayed (gps_gate / ekf_map float handling) and run in the sim; not field tested.

## Summary

- When RTK drops (single, dropout, float that fails the sigma gate, or the ekf_map sigma gate
  on its own), a mow follow or a transit now **keeps going on dead reckoning** at the degraded
  speed cap (0.15 m/s) with the blade as it was, while the **coast budget** lasts and the
  path ahead clears the boundary. Then it holds exactly as before (fix wait, 5 s settled-fix
  resume, retrace at 120 s, give-up at 600 s). Before, the mower stopped within a second.
- **Budget** (localization_monitor, `coast_left_m`): 5 m of driving on flat ground, 1.5 m from
  8 deg of tilt, linear in between, at most 60 s since the last fix; none after wheel slip or an
  RTK position that disagrees with dead reckoning. Pivots in place spend no distance.
- **Drift model**: `est_drift_m` = 5 cm + 2 %/m flat, rising to 10 cm + 10 %/m at 11 deg,
  + 1 cm per pivot (the drift study's p90 fit), replacing 3 cm + 3 %/m. The VIO aid is a
  pluggable rate scale (0.5), not wired to anything new.
- **Clearance**: along the path ahead within the remaining budget, and at the robot, the body
  must stay est_drift + 0.10 m inside the boundary guard's own limit (soft edges: the body
  side may reach 0.35 m over the outline, hard edges 0.10 m) and off the keepouts. So a coast
  never drives the body to where the guard would trip, assuming the drift stays within the
  estimate; on a soft lap 0 (body side nominally 0.05 m inside) that allows est_drift up to
  0.25 m.
- **RTK vs dead reckoning gate**: float (or a fixed not yet confirmed) further than
  0.15 m + est_drift from the dead-reckoned pose for two samples is `lost` until the next
  confirmed fix: the mission holds and waits for settled fixed. That catches the 11-13 float
  (0.3 m off) and the 11-21 float onset (3.02 m off at a claimed 8 cm sigma).
- **gps_gate**: the 11-21 3 m jump came from a race: the first float /fix arrived 1 ms before
  its /fix_status, so it was fused as FIXED at its own 8 cm sigma. A "fixed" fix with a sigma
  over 5 cm is now fused as float, and a fix 0.5 m or more from the last one passed is held
  back until it has persisted 2 s.
- **ekf_map still follows a float that is a few decimetres off**, whatever its covariance:
  in the replay, a 0.3 m float offset after a 15 s dropout is 97 % in the pose after 0.6 s at
  today's 0.4 m floor, 81 % at 1.0 m and 54 % at 2.0 m, and fully in after 2-5 s in every case
  (the filter's own sigma has grown during the dropout). Inflating the float covariance does
  not keep the pose on dead reckoning, so it is not changed; the monitor's gate rejects such a
  float in 0.4 s and the mission holds until a settled fix, as on master.
- `coast_enabled: false` (mission) restores the 2026-11-03 degraded rule;
  `coast_max_s: 0` and the legacy drift parameters restore the monitor.
- Sim: on a flat lawn the mower coasted through a 15 s single dropout mid-row (1.7 m, blade on)
  and held 5 s where master's behaviour held 20 s; the dead-reckoning error (up to 0.06 m
  over the baseline, inside the estimate) tips the tracking criteria h and i. On the strip the
  edge pass holds at once and the 0.3 m float is rejected by the new gate, as on master
  (criterion k PASS). real_run5 PASS (master FAIL i). Section 6.

## 1. Inputs

- Drift study (`2026-11-13_dead_reckoning_drift.md`, 814 simulated dropouts, ekf_map wheel vy
  fused): p90 0.10 / 0.20 / 0.30 m after 2.5 / 7 / 13.75 m on flat ground, 0.5 / 1.5 / 2.75 m
  at >= 8 deg; pivots ~1 cm; slip invisible to dead reckoning; float within 0.02-0.22 m of
  dead reckoning normally, 0.3 m off after the 11-13 single dropout.
- RTK-dropout fixes on master (`2026-11-13_rtk_dropout.md`): settled-fix resume, untrusted-pose
  boundary trips join the hold, ekf_map wheel vy.
- Field review of the 2026-11-21..25 mows on master (coordinator): no RTK single, 21 float
  episodes (203 s, 416 s, 181 s, 113 s among them, about 15 min of waiting on 11-21), 12 stops
  on the ekf_map sigma limit (3 on RTK fixed at sigma 0.15 against the 0.15 limit), and a 3.02 m
  pose jump at a float onset (section 5).

## 2. Design as built

### 2.1 Localization monitor

`mower_localization/drift_model.py` (pure) and `localization_status_logic.py`:

| Piece | Rule | Parameters (`config/localization_monitor.yaml`) |
|---|---|---|
| drift estimate | base(mean tilt since the fix) + sum over steps of rate(tilt at the step, aid) x step + pivot_m x pivots; capped at 0.5 m while float is fresh (as before) | `drift_base_flat_m` 0.05, `drift_base_slope_m` 0.10, `drift_rate_flat` 0.02, `drift_rate_slope` 0.10, `drift_tilt_flat_deg` 4, `drift_tilt_slope_deg` 11, `drift_pivot_m` 0.01, `pivot_min_deg` 45, `pivot_max_vx` 0.05, `drift_rate_scale_vio` 0.5 |
| coast budget | `coast_left_m` = budget(mean tilt) - distance since the fix; 0 after `coast_max_s`, slip or a rejected RTK position | `coast_max_flat_m` 5.0, `coast_max_slope_m` 1.5, `coast_tilt_slope_deg` 8, `coast_max_s` 60 |
| no-RTK rule | `lost_after_s` (10 s) waits while `coast_left_m` > 0 | |
| slip | wheel yaw (`/mcu/measured_speed`) vs gyro-led odom yaw parting by 0.6 rad in 2 s, while not fixed: `lost` until the next confirmed FIXED | `slip_window_s` 2, `slip_yaw_rad` 0.6 |
| RTK vs dead reckoning | `/odometry/gps` under float or an unconfirmed fixed, `float_gate_n` samples in a row further than `float_gate_m` + est from the dead-reckoned pose: `lost` until the next confirmed FIXED | `float_gate_m` 0.15, `float_gate_n` 2 |

New status fields: `tilt_mean_deg`, `pivots`, `coast_budget_m`, `coast_left_m`, `rtk_dr_m`,
`rtk_dr_gate_m`, `dr_lost`. New inputs: `/tilt/status`, `/mcu/measured_speed`, `/odometry/gps`,
and the poses / twists of the two odometry topics it already read.

Why the gate is 0.15 m + est and not 0.3 m + est: the 11-13 float was 0.3 m off while the
standing robot's dead reckoning was 2 cm off; at 0.3 m + est (0.10 m on the slope) it would
have passed, and the ekf_map pose would then carry a 0.3 m error into the edge laps. Normal
float sat within 0.02-0.22 m of dead reckoning in the drift study, which includes the dead
reckoning's own error, and the est term covers that.

Why `lost` on a disagreement (and not just "end the coast"): `lost` is what makes the mission
hold and latch the fix wait, so only a settled fix resumes; the boundary guard then treats the
pose as untrusted.

### 2.2 Mission

`mower_mission/fsm/holds.py`:

- `_loc_coast_tick(state, ctx)`: called from `_loc_hold_tick` where the degraded rule was.
  Coasts while `coast_enabled`, the state is degraded, the context is a mow follow (incl.
  spin-up, spin pause, corner pivot) or a transit, no fix wait is pending, `coast_left_m` > 0,
  and the clearance check passes. Logs the start (`coasting on dead reckoning`) and the end
  (`coast ended after N s, M m from its start: <why>`).
- `_loc_coast_check` (query): the reason the coast may not go on, or ''.
- `_coast_path_ahead(horizon)`: the sub-path from the tracked progress (follow) or the step's
  start (spin-up, pivot), a route transit from its nearest pose, else the line to the transit
  target, sampled every 0.25 m, up to `coast_left_m`.
- `_coast_clearance(pt)`: how far base_link at the point may drift before the body reaches a
  limit: the largest edge distance of a region polygon holding it (a lower bound for the union
  of areas, paths and dock corridor the boundary guard enforces) plus the guard's side
  allowance for the edge mode there (`hard_boundary_body_soft_side_m` 0.35 /
  `hard_boundary_body_hard_m` 0.10), and the distance to the keepouts (no allowance), each
  less half the body width; memoised per coast. The first version required the body to stay
  0.10 m inside the outline itself; on the run5 lawn, whose rows end about 0.15 m from the
  outline, no interior row could coast (`coast-edge` / `coastoff-badparams` runs below), while
  the plan itself puts the body over the outline there.
- Speed cap and blade: unchanged code paths (the degraded cap applies, nothing turns the blade
  off).
- `fsm/guards/boundary.py` `_hb_pose_untrusted`: while coasting, degraded alone no longer makes
  a trip untrusted, nor does a class change to a non-RTK class (it does not move the pose). A
  pose jump against the wheels, a return of float / fixed within `loc_fixed_settle_s`, the fix
  wait and `lost` still do.

Parameters (`mission.yaml`): `coast_enabled` true (on by default, see below), `coast_edge_margin_m` 0.10. All `loc_*`
parameters are unchanged.

**Default `coast_enabled: true`** (owner 2026-11-25: merged off first, then turned on by
default the same day, before a field test; the sims below ran it on through a
`mission_params_file:=` copy with `coast_enabled: true`, which is now the default).
The case for turning it on: the budget ends at a p90 error of about 0.15 m on flat
ground and 0.25 m on the slope, below the 0.35 m soft side limit; the clearance check keeps
the body est_drift + 0.10 m inside the guard's limits; slip and disagreeing RTK end them; and every failure falls back to the
hold that is on master. The owner asked for this behaviour, and the sigma-gate stops (12 on
11-21, 3 of them on RTK fixed) become coasts where the path is clear. Set it false for the
first field mow if a hold-only baseline is wanted.

### 2.3 gps_gate

| Change | Why |
|---|---|
| `effective_class`: FIXED with a receiver sigma > `fixed_max_sigma` (0.05 m) is fused as float | 11-21 07:13:41: the float /fix (cov 0.0062 m^2) arrived 1 ms before its /fix_status (RTK_FLOAT), was classed with the previous FIXED and fused at sigma 0.08 m; fixed fixes report 3.5-4.2e-5 m^2 |
| `JumpGate`: a fix > `jump_gate_m` (0.5 m) from the last one passed is held back until the new position has held `jump_confirm_s` (2 s); after `jump_gap_s` (1 s) without fixes anything passes | the same event: ekf_map moved 3.02 m in 0.1 s. At 10 Hz the mower moves at most 5 cm per fix |

## 3. Tests

- `mower_localization/test/test_coast_drift.py` (29): the model (flat, 11 deg, linear tilt,
  pivots, VIO scale, legacy parameters), the budget by tilt, the float gate, the pivot
  counter, the slip detector; the status logic: a 5 m flat coast, the no-RTK rule waiting,
  the 60 s cap, the slope budget, pivots spending no distance, a fix resetting it, coast off,
  slip -> lost until fixed, float that disagrees / agrees, a single outlier, an unconfirmed
  fixed, the 11-21 float onset, the dead-reckoned pose ignoring the map pose jump, gate off.
- `test_localization_status.py`: the existing cases run on the legacy parameters (restorable).
- `test_rtk_quality.py` / `test_gps_gate.py`: the 11-21 class race, `JumpGate` (driving, a held 0.6 s jump, a persisting jump, a gap, off).
- `mower_mission/test/test_coast_mode.py` (13): a 15 s flat dropout coasts through with no
  stop, capped speed and the blade on; the budget running out on a slope holds and float does
  not resume it; float that disagrees holds until fixed settles; float that agrees carries on;
  the edge ahead holds at once; the clearance follows the drift and the budget horizon; a
  transit coasts; a pivot coasts; `coast_enabled: false` gives the swath rule; no budget field
  holds; a real drive-out while coasting trips (BOUNDARY OUT, counted); drifting toward the
  edge holds before the body is out; a pose jump while coasting is not trusted.
- `test_loc_policy.py`: the four swath-rule cases now run with `coast_enabled: false`.
- `scripts/sim_gate/test_evaluate.py`: criterion k with a coast (through, then held, resumed).

`scripts/ci_test.sh`: PASS. colcon 4017 test cases, 0 failed, 0 tolerated (colcon summary
4035 tests, 85 skipped); scripts 97 and replay 6 passed.

## 4. Replay: how far ekf_map follows an offset float

`scripts/replay/ekf_replay.py` (new variant options `fl_len`, `fl_dx`, `fl_dy`, `fl_floor`:
the first `fl_len` s of fixes after each withheld window become float offset by (dx, dy) at the
given sigma floor) and `scripts/replay/float_pull.py`. The 11-13 strip bag with the real
dropout and the 11-11 / 11-12 bags of the drift study were deleted for disk space; the 11-21
bag is gone too (only extracts without /fix positions remain). The replay used 25 minutes of
the 2026-10-28 field bag (`bags/mower/bags/20261028-023252`, splits 20-45, RTK fixed
throughout; ENU fit residual 0.8 cm rms), with ekf_map as on this branch (wheel vy 0.05), a 15 s
dropout every 60 s at two phases and then 12 s of float 0.3 m south of the truth.

Pull = the estimate's error along the offset as a share of it (0 = on the truth, 1 = on the
float); 57 windows per floor:

| float sigma floor | 0.6 s | 2 s | 5 s | 10 s | error p90 at the float start / end / 2 s after FIXED |
|---|---|---|---|---|---|
| 0.4 m (gps_gate today) | 0.97 / 1.00 | 1.00 / 1.03 | 1.00 / 1.03 | 1.00 / 1.04 | 0.26 / 0.31 / 0.01 m |
| 1.0 m | 0.82 / 0.93 | 0.96 / 1.02 | 0.99 / 1.05 | 0.99 / 1.07 | 0.26 / 0.32 / 0.01 m |
| 2.0 m | 0.54 / 0.82 | 0.81 / 0.97 | 0.93 / 1.07 | 0.95 / 1.10 | 0.26 / 0.34 / 0.01 m |

(p50 / p90 of the pull; the two phases agree within 0.05.) After a 15 s dropout ekf_map's
position sigma has grown well past any float floor that would still be useful, so the float
wins within one to two seconds. A covariance change cannot stop the pull; only not fusing the
float would, and the monitor needs the fused float (`/odometry/gps`) to check it. So gps_gate's
float covariance stays as it is, and the protection is downstream: the monitor rejects a
disagreeing float after two samples (0.4 s at its 5 Hz), the mission holds with the blade off,
the boundary guard treats the pose as untrusted, and only a settled fix resumes. Large jumps
(0.5 m and more) are held back by gps_gate's jump gate before they reach ekf_map.

Reproduce (dev image, worktree at /work, decompressed splits at /bag, writable):

```
V=""; for ph in 20 50; do for fl in 0.4 1.0 2.0; do
  V="$V --variant p${ph}f${fl/./}:vy_var=0.05,fuse_vy=1,wh_len=15,wh_every=60,wh_phase=$ph,fl_len=12,fl_dx=0,fl_dy=-0.3,fl_floor=$fl"
done; done
python3 -P scripts/replay/ekf_replay.py /bag /out/r1028 --t0 0 --t1 4e9 --speed 5 $V
python3 scripts/replay/float_pull.py /out/r1028
```

## 5. The 2026-11-21 float-onset jump

From the field review's extracts of the 11-21 bag (`/fix` status and covariance,
`/fix_status`, `/odometry/filtered_map`), times relative to the jump:

| t (s) | Message |
|---|---|
| -0.14 | /fix, cov 4.0e-5 m^2; /fix_status RTK_FIXED |
| -0.038 | **/fix, cov 6.2e-3 m^2** (the float solution, 3 m off) |
| -0.037 | /fix_status **RTK_FLOAT**, 1 ms later |
| 0.000 | ekf_map at (-19.74, -12.15), 3.02 m from (-17.07, -10.74), var still 0.006 m^2 |
| 0.07-0.46 | five more float fixes (cov 6.2e-3), ekf_map stays about 2.9 m off |
| 0.56 | /fix cov 4.2e-5, /fix_status RTK_FIXED again |

gps_gate classified the float fix with the class it had (RTK_FIXED, 0.1 s old) and gave it the
fixed treatment: its own 8 cm sigma, close to ekf_map's own, and the pose moved the whole 3 m
in one 0.1 s step; the next float fixes held it there. Nothing on master rejects that: the float
floor never applied, there is no innovation gate in ekf_map (deliberately: a Mahalanobis gate
locks the filter out after a boot or a long dropout, `ekf_dual.yaml`), and the monitor's sigma
gate saw nothing wrong.

Now:

1. `effective_class` turns that fix into float (6.2e-3 > 0.05^2), so it gets the 0.4 m floor
   (at most about 4 % of the 3 m per update against an 8 cm filter sigma).
2. `JumpGate` holds back every fix 0.5 m or more from the last one passed; the float episode
   lasted 0.6 s, so none of it would have reached ekf_map. Had the jumped position persisted
   2 s it would have been taken (the gate must not lock the filter out of a real solution).
3. If a jumped position does reach the filter, the monitor compares it with the
   dead-reckoned pose (3.02 m against a gate of about 0.2 m) and reports `lost` after two
   samples: the mission holds (as the untrusted-pose hold did on 11-21) and waits for a
   settled fix.

The raw bag is gone, so 1 and 2 are unit-tested with this sequence's covariances and timing
(`test_rtk_quality.py::TestEffectiveClass`, `test_gps_gate.py::TestJumpGate`), and 3 with a
3.02 m float onset without a dropout (`test_coast_drift.py`); none of it was replayed through
ekf_map.

**The other 11-21 findings.** No RTK single occurred; the 21 float episodes were ambiguity
losses with corrections flowing. Of the 12 stops on the ekf_map sigma gate (sigma just over
0.15 m, 3 of them on RTK fixed), each became a 5-6 s hold, blade off. With coast mode a sigma
gate `degraded` is a coast where the path ahead is clear (the budget is full on RTK fixed and
the estimate small), so those become slow-downs rather than stops; the gate itself is unchanged
(open item). The long float waits (203 s, 416 s, 181 s) started as holds with the fix wait
latched; a coast instead of the hold keeps the mission out of the fix wait, so float that
passes the dead-reckoning gate is mowed on as on master (`ok` on float). Beyond about 12 m of
driving the float-capped estimate (0.5 m) passes the 0.3 m budget, the coast budget is spent,
and the mower holds as before.

## 6. Sim gate

All runs `-l -x 3 -s 1 -t 4000`, one at a time under the shared lock; output in
`ros2_stack-wt/simout_coast1125/`. Master's results are from `simout_rtkdrop1113` (the 11-13
branch, now master). "Held" = time from the hold to the resume. Max true overhang = criterion a
(base_link outside by ground truth) and criterion i (body over the plan by ground truth).

| World / run | Verdict | Failing | RTK handling (k) | Holds / held | Cut (gt) | a: base_link out | i: body over plan side / nose / sweep |
|---|---|---|---|---|---|---|---|
| (a) `real_run5_coast` / coast-l-s1, coast on | FAIL | h, i | PASS: coasted 15.0 s (1.7 m) through the 15 s single, then one hold at the row end as RTK came back (clearance), resumed 5.0 s later | 1 / 5 s | 97.1 % | 0.091 m, 0 samples > 0.10 | 0.162 / 0.059 / 0.123 |
| (a) `real_run5_coast` / coastoff-l-s1, `coast_enabled: false` (master behaviour) | FAIL | a, i | PASS: held at once, resumed 5.3 s after FIXED | 1 / 20 s | 97.1 % | 0.112 m, 31 samples (at 152-155 s, before the dropout) | 0.103 / 0.089 / 0.125 |
| (b) `owner_area4_strip_rtkdrop` / rtkdrop-l-s1 | FAIL | c, i | PASS: no coast (south edge pass: clearance 0.13 m < 0.15 m); the float 0.29 m off was rejected (gate 0.20 m, `lost`); no recovery move; resumed on the settled fix | 1 / 31 s | 95.3 % | 0.029 m, 0 | 0.118 / 0.042 / 0.048 |
| (b) master (rtkdrop-l-s1, 11-13) | PASS | | held, resumed 5.3 s after FIXED | 1 / 31 s | 94.9 % | 0.015 m, 0 | 0.073 / 0.055 / 0.029 |
| (c) `real_run5` / run5-l-s1 | PASS | | n/a (no RTK injection; the monitor is not started) | 0 | 97.4 % | 0.076 m, 0 | 0.085 / 0.038 / 0.100 |
| (c) master (run5-l-s1, 11-13) | FAIL | i | n/a | 0 | 97.6 % | 0.080 m, 0 | 0.105 / 0.081 / 0.077 |

Reading the failures:

- **(a) coast on, h and i**: the dead-reckoning error of the coast. The map pose against the
  ground truth grew from 0.02 m (the run's baseline: fix latency and lever arm) to 0.076 m at
  the end of the 15 s coast, inside the 0.09 m estimate at that point; the sim's odom has a 1 %
  scale error, a yaw-bias walk and pivots about a point ahead of base_link that the wheels do
  not see. That shows as cross-track (h, line p99 0.111 against 0.10) and body over plan at
  the row end where the coast ended (i, side 0.162 against 0.10, planned 0.107, so the body was
  0.27 m over the outline against the 0.35 m soft limit). No boundary trip, base_link never
  more than 0.10 m outside. This is the price of mowing on: the criteria were set for RTK-fixed
  tracking and do not exempt a coast.
- **(a) coast off, a and i**: an excursion at an edge-lap corner 100 s before the dropout,
  the same spot that makes master's real_run5 fail i; run-to-run variation, not RTK related.
- **(b) c**: the run ended in IDLE_DOCKED instead of CHARGING ("battery 100 % >= max charge
  100 %: charging disabled" 39 s after docking; master's run ended 31 s after docking). The
  mow and the dock were complete. **(b) i**: side 0.118 at 340 s, before the dropout window.
- **(c)** PASS where master failed i: run-to-run variation (nothing of coast mode runs without
  an RTK injection).
- The first coast world run (`coast-edge-l-s1`) put the dropout on the edge laps (the sim's
  `blade` anchor is about 66 s before the mow's blade on): it held at once, as designed, and
  PASSed. `coastoff-badparams-l-s1` did not load its params file (not mounted) and ran with
  coast on and the first clearance rule: held at once on an interior row whose end is 0.15 m
  from the outline. That led to the clearance rule against the guard's limits (2.2).

The sim cannot show the slope budget (the strip world has no terrain, so the monitor's tilt is
0) or slip (no `/mcu/measured_speed`); both are unit-tested.

## 7. Open

- **Field test.** Watch the first RTK loss mid-row: the `coasting on dead reckoning` line, the
  coast end line and its reason, and `/localization/status` `coast_left_m`, `est_drift_m`,
  `rtk_dr_m`.
- **Slip detector** is the drift study's 2 s / 0.6 rad rule. It fires on hard pivots with
  normal scrub (5-10 % of dropout windows had one); each fire ends the coast into a hold. No
  `/mcu/measured_speed` in the sim, so it is unit-tested only.
- **Long float** (the 203 s / 416 s episodes): float that passes the gate keeps the status
  `ok`, so mowing goes on as on master; but the estimate (capped at 0.5 m under float) passes
  the 0.3 m budget after about 12 m, and with the coast budget spent that is a hold. Coasting
  does not lengthen long float stretches; using float that agrees with dead reckoning to
  re-anchor the estimate would.
- **Sigma-gate stops on RTK fixed** (sigma 0.15 against 0.15): they now coast instead of
  stopping where the path is clear; the gate itself is unchanged.
- **Side slope** is still not in the drift data; the model's tilt term uses |tilt|.
- **VIO.** `drift_rate_scale_vio` (0.5) already lowers the per-metre rate while
  `/odometry/vio_gated` is fresh; once VIO is validated, measure its p90 drift with
  `ekf_replay.py` (a variant fusing VIO) and set the scale, and raise `coast_max_*_m` by the
  same factor. A different aid source adds one entry to `DriftParams.rate_scale` and one
  freshness input in `LocalizationStatus`.
- **11-21 bag**: the raw bag is gone (only the field review's extracts remain, without /fix
  positions), so the jump could not be replayed through ekf_map; the class race and the jump
  gate are unit-tested with its timing and covariances.
