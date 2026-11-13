# RTK dropout on the Area 4 strip: resume on a settled fix, no boundary moves on a jumped pose, ekf_map vy (2026-11-13)

Branch `ws/rtkdrop1113` from master 3c3fdeb. Unit-tested, bag-replayed (EKF) and run in the
sim; not field tested.

## 1. The field mow

Area 4 strip, mowing pass 8 near the south edge, 2026-11-13 field bag. Times are from the
start of the RTK loss (t = 0); positions are `/odometry/filtered_map` in map metres.

| t (s) | What happened |
|---|---|
| 0 | `/fix_status` went from RTK_FIXED to GPS (single): the NTRIP stream had been reconnecting for about two minutes and `diff_age` reached 120 s. The robot was at (-13.96, 1.38). |
| 1 | The mission logged `localization degraded (ekf_map sigma 0.16 m > 0.15 m (rtk single)): stopped, blade off, waiting for RTK fixed`. Correct. |
| 1-14.6 | The robot stood still: wheel odometry and `/mcu/measured_speed` zero, odom yaw constant. The map pose ran north at about 3 cm/s, 0.41 m in total, to (-13.87, 1.81). The single fixes themselves moved about 3 cm, and gps_gate dropped them anyway, so the drift was not GNSS. |
| 14.6 | RTK came back as FLOAT (receiver variance about 0.005 m²). The float position was about 0.3 m south of the truth. The map pose jumped 0.72 m south to (-13.89, 1.10) in under a second. |
| 14.6 | `BOUNDARY OUT: body front_right 0.77 m outside (soft limit 0.55) ... in MOWING: cancel, zero, blade off, move 0.30 m back, then resume`. Against the true pose the corner was about 0.48 m out, under the limit: a false trip, from the pose jump with the wheels still. |
| 15-32 | The boundary recovery drove 3 arc moves (about 0.9 m) on the float pose, logged `back inside`, `blade ON` and `localization speed cap lifted`, and resumed mowing on float. There was no `localization ok again` line: the recovery resumed without going back through the hold. |
| 25.6 | Float to fixed: the pose jumped another 0.2 m while driving. |

Three faults:

1. **The resume gate was sigma only.** The hold waited for `ok`, and `ok` needs the ekf_map
   sigma at or under 0.15 m, which float passes (0.07 m). The log line said "waiting for RTK
   fixed", but a fix was never required. And the boundary recovery took over the robot
   (`self._loc = None`) and resumed through its own path.
2. **The boundary guard believed a jumped pose.** It acted on a 0.72 m pose step while the
   wheels had not moved and localization was degraded.
3. **ekf_map drifted standing still.** It fused wheel vx and yaw rate, not vy. The lateral
   velocity the RTK updates had left in its state was integrated while no fix was fused.

## 2. Fixes

### 2.1 Resume only on a settled RTK fix (`fsm/holds.py`)

- **Fix wait.** A localization hold that stops a mow or transit (`_loc_hold_begin`) latches
  `_loc_fix_wait`. The latch clears once the receiver has been RTK FIXED (fresh
  `/localization/status` with `rtk: fixed`) for `loc_fixed_settle_s` (5 s), or when the
  mission ends (`_loc_fix_track`, every tick).
- **The hold's own resume** needs `ok` for `loc_resume_s` (3 s, as before) and RTK FIXED
  settled. `ok` on float shows `localization ok (RTK float): waiting for RTK fixed (N s)` and
  logs once that it is still waiting. If fixed never comes back, the existing
  `loc_lost_dock_s` (600 s) give-up applies: MOWING_INCOMPLETE, staying put without a fix.
  Dock holds keep the old rule.
- **Other resume paths.** While the wait is pending, `_loc_hold_instead_of_resume` starts the
  hold (stop, blade off, keep the goal) instead of resuming. It is called by the boundary
  recovery (`_hb_retraced`), the stuck escape (`_stuck_escaped`) and BOUNDARY_PAUSED (soft
  boundary flag cleared, or its recovery transit). As a catch-all, `_loc_hold_tick` starts the
  hold again on the next tick if anything else resumed while the wait is pending.
- **During a recovery manoeuvre.** A degraded / lost status while the boundary recovery, the
  stuck escape or the tilt back-out owns the tick (the hold cannot start then) latches the
  wait too, so that manoeuvre's resume waits for the fix.
- `loc_resume_require_fixed: false` restores the old resume rule.

Resume paths checked (anything that can restart motion or the blade in MOWING / TRANSIT):

| Path | While the fix wait is pending |
|---|---|
| localization hold resume (`_loc_hold_tick`) | needs RTK FIXED settled |
| boundary recovery resume (`_hb_retraced`: rejoin, projection, transit retry, entry) | hold instead |
| new boundary trip (`_hard_boundary_tick`) | deferred into the hold (2.2), no recovery |
| soft boundary flag (`_boundary_pause`) | deferred into the hold, no BOUNDARY_PAUSED |
| BOUNDARY_PAUSED clear / recovery transit (`dispatch.py`) | hold instead |
| stuck escape resume (`_stuck_escaped`) | hold instead |
| tilt back-out resume (`_tilt_backed_out`: detour / skip) | not gated directly (its continuation is a detour or skip the hold could not reproduce); a pending wait re-holds on the next tick, before the blade-off detour transit gets far |
| dynamic-obstacle wait, path-blocked wait | cannot start during a hold (`_dyn_context` is None while the step is `loc_hold`); the hold owns the tick before the blocked wait |
| bumper skip | only fires while following |
| retry wait (`retry_wait`), leg watch, no-go backup | the catch-all re-holds on the next tick |
| operator START / STOP / MANUAL | operator commands; STOP ends the mission and the wait |

### 2.2 Boundary trips on an untrusted pose (`fsm/guards/boundary.py`)

When the guard trips in MOWING or TRANSIT, `_hb_pose_untrusted` asks whether the trip may
come from the pose rather than the robot:

- the fix wait is pending;
- `/localization/status` reads degraded or lost;
- the receiver class changed within `loc_fixed_settle_s` (5 s);
- RTK is not settled-fixed and the map pose moved `hard_boundary_jump_pose_m` (0.15 m) more
  than the wheel odometry within `hard_boundary_jump_window_s` (3 s) (`_hb_pose_jump`, a ring
  of (t, wheel xy, map xy) per guard tick).

If so, `_hb_defer_to_loc_hold` treats it as a localization event: the localization hold
starts (or keeps running), the robot stays stopped with the blade off, nothing is counted, no
recovery moves are driven, and the log says `boundary: <trip> while <why>: pose not
trusted, no recovery move; holding (blade off) for RTK fixed, then checking again`. When the
hold resumes on the settled fix, the guard checks the boundary again on that pose; a real
excursion then trips as usual.

What stays as before:

- With RTK fixed and settled the guard is unchanged, also when the wheels disagree with the
  pose (a slide on a slope is real motion).
- Without a `/localization/status` (bench) nothing is deferred.
- A drive-out on float with the wheels moving trips at once.
- Dock phases and the heading calibration are unchanged.
- `hard_boundary_loc_defer: false` restores the old guard.

### 2.3 ekf_map wheel vy (`mower_localization/config/ekf_dual.yaml`, `mcu_node`)

`ekf_map` now fuses the wheel odometry's vy = 0. `mcu_node` published vy as unobserved
(variance 1e6); `odom_vy_variance` is now 0.05 (m/s)², in mcu_node and in bringup. The
GNSS-smoothing switch still sets 1e-4. 0.05 is loose: it zeroes a stale vy within about a
second and does not pull the pose. The failed GNSS-smoothing A/B used 1e-4 together with a
1e-4 x/y process noise. `ekf_node` is unchanged: it has no GPS, so nothing biases its vy.
With `vio:=true` the map filter's VIO vy (variance floor 0.04) and the wheel vy now share the
weight about equally; VIO is off by default.

**Bag replay.** There was no replay tool in the repo; `scripts/replay/ekf_replay.py` is new.
It rebuilds the filter inputs the bags do not record:

- `/odom` from `/odometry/filtered` (vx, yaw rate; vy 0, per-variant variance);
- `/imu/data_aligned` from the fused yaw;
- `/odometry/gps` from `/fix`, classified by `/fix_status` with gps_gate's covariance
  (fixed 3 cm floor, float x4 / 0.4 m floor, single dropped).

The ENU to map offset and the 0.096 m antenna lever arm are fitted to `/odometry/filtered_map`
over the 7139 fixed fixes (residual 2.1 cm rms). Every variant then runs as a
robot_localization `ekf_node` with `ekf_map`'s parameters on sim time, 5x. The unchanged
filter reproduces the field pose to 2.0 / 4.7 cm (p50 / p90) over the whole bag.

| ekf_map | Drift standing through 14.5 s of single | Float-return jump (first 2 s) | RTK fixed, moving: pose - RTK p50 / p90 | lateral p90 | lateral step rms | vs the unchanged filter p90 / p99 |
|---|---|---|---|---|---|---|
| field (recorded) | 0.41 m north (2.9 cm/s) | 0.72 m | | | | |
| unchanged (vy not fused) | 0.53 m north (3.8 cm/s) | 0.82 m | 0.28 / 0.78 cm | 0.38 cm | 0.36 cm | |
| vy 0.01 | 0.016 m | 0.27 m | 0.26 / 0.72 cm | 0.36 cm | 0.36 cm | 0.25 / 0.69 cm |
| **vy 0.05 (chosen)** | **0.016 m** | **0.27 m** | **0.25 / 0.69 cm** | **0.35 cm** | **0.33 cm** | 0.24 / 0.66 cm |
| vy 1e-4 | 0.016 m | 0.27 m | 0.25 / 0.67 cm | 0.34 cm | 0.31 cm | 0.25 / 0.68 cm |
| vy 0.25 (other run) | 0.016 m | 0.28 m | 0.27 / 0.85 cm | 0.42 cm | 0.44 cm | |
| vy 1.0 (other run) | 0.088 m | 0.13 m | 0.25 / 0.80 cm | 0.39 cm | 0.39 cm | |

- The stationary drift is gone. What is left of the float-return jump is the float error
  itself (0.27 m, against about 0.3 m in the field). Fix 2.2 covers that jump.
- On the RTK-fixed moving parts (5302 samples) the pose stays as close to RTK as before and
  the lateral jitter does not grow. The 0.05 / 0.01 / 1e-4 rows ran in one replay with the
  unchanged filter; the 0.25 / 1.0 rows ran in a second one, whose unchanged filter read
  0.27 / 0.83 cm (run-to-run spread about 0.05 cm).
- 1.0 is too loose: the drift is 9 cm, and it wanders.

Command (dev image, worktree at /work, bag read-only at /s/bag):

```
python3 scripts/replay/ekf_replay.py /s/bag /s/out --t0 <bag start> --t1 <bag end> \
    --speed 5 --variant base: --variant vy05:vy_var=0.05,fuse_vy=1
```

## 3. Tests

`mower_mission/test/test_rtk_dropout.py` (13 cases):

- float does not resume a dropout hold, for 60 s;
- fixed resumes only after 5 s in one run, and a float blip restarts the settle;
- fixed never returning ends in the existing give-up;
- `loc_resume_require_fixed: false` restores the old rule;
- plain float mowing without a dropout is not held;
- the field sequence: single, then a float jump that puts the side corners about 0.5 m out with
  the wheels still, runs no recovery and logs the deferral once; fixed then resumes it with
  nothing counted;
- a pose jump on float with the wheels still is a localization event; when the settled fix
  confirms the robot is out, the trip is acted on;
- a genuine drive-out on float, on fixed without wheel odometry, and on fixed with the
  wheels disagreeing all still trip;
- a trip within 5 s of an RTK class change is deferred;
- `hard_boundary_loc_defer: false` restores the old guard;
- the boundary recovery cannot resume before the fix settles (single during the recovery,
  float at its end: it holds, then resumes on fixed).

Changed: the hold-resume timings in `test_loc_hold.py`, `test_loc_policy.py`,
`test_hold_pivot_resume.py` and `test_mission_legs.py` move to the 5 s settle (the `loc_resume_s` timing
test keeps the old rule with `loc_resume_require_fixed: false`). `mower_localization` config tests:
ekf_map fuses vy, ekf_node does not. `mower_sim`: the single kind and per-event options.
`scripts/sim_gate`: back-to-back loss windows.

`scripts/ci_test.sh`: PASS. colcon 3966 test cases, 0 failed, 0 tolerated (colcon summary
3984 tests, 85 skipped), mower_mission 931; scripts 96 and replay 6 passed.

## 4. Sim

All runs `-l -x 3 -s 1 -t 4000`, one at a time; output in
`ros2_stack-wt/simout_rtkdrop1113/`. Master's results are from the 2026-11-13 strip
gate (`simout_strip1113`). The sim has no EKF, so the vy change (2.3) is not exercised; the
mission changes are inert without an RTK injection (no `/localization/status`).

**Injection** (`mower_sim/rtk.py`, small): a new event kind `single` publishes `/fix` and
`/fix_status` SINGLE; gps_gate would drop it, so the map estimate dead-reckons as in a
dropout. Per event, a float can carry a fixed `offset` (map m), the share of it in the map
pose (`map_gain`), and the ekf_map sigma while it lasts (`ekf_sigma_m`). World
`owner_area4_strip_rtkdrop.yaml` is the strip world with the field sequence on the south-most
pass (forward on it 262-288 s after blade on in the strip gate):

- SINGLE for 14.5 s from 275 s after blade on, ekf_map sigma 0.5 m (degraded);
- FLOAT for 11 s with the fix 0.3 m south, all of it in the map pose, sigma 0.07 m (ok);
- then FIXED.

The evaluator's criterion k now treats back-to-back loss modes as one window
(`single+float`): the hold has to last until FIXED.

| World / run | a | b | c | d | e | f | g | h | i | j | k | Verdict | Master |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| owner_area4_strip / strip-l-s1 | 0.000 | 0 | yes | 0/0/0 | 0 of 5 | 0 | 5 % | - | 0.071 / 0.066 / 0.053 | n/a | n/a | **PASS** | PASS (strip5-l-s1) |
| owner_area4_strip_rtkdrop / rtkdrop-l-s1 (new) | 0.015 | 0 | yes | 0/0/0 | 0 of 6 | 0 | 6 % | - | 0.073 / 0.055 / 0.029 | n/a | held / blade off / resumed / inside | **PASS** | (no master run) |
| real_run5 / run5-l-s1 | 0.080 | 0 | yes | 0/0/0 | 0 of 19 | 0 | 16 % | 0.067 / 0.065 | 0.105 / 0.081 / 0.077, transit 0.374 | n/a | n/a | FAIL (i only) | FAIL (i only: 0.103 / 0.059 / 0.134, transit 0.354) |

Ground-truth cut: strip 95.4 % (master 94.2 %), strip with the dropout 94.9 %, real_run5
97.6 % (master 97.7 %).

The dropout run in the mission log (relative to START; RTK window 420.8-446.3 s):

- 421.0 s, `localization degraded (ekf_map sigma 0.50 m > 0.15 m (rtk single)): stopped,
  blade off, waiting for RTK fixed`.
- At the float return: `boundary: body rear_right 0.50 m outside (soft limit 0.35) while
  waiting for RTK fixed after a localization stop: pose not trusted, no recovery move;
  holding (blade off) for RTK fixed, then checking again`. This is the field's false trip,
  now not acted on: no BOUNDARY OUT, no recovery move, 0 trips counted (criterion b).
- `localization ok on RTK float: still waiting for RTK fixed`; the robot stayed put through
  the 11 s of float.
- 451.6 s (5.3 s after FIXED returned), `localization ok again after 31 s: resuming (RTK
  fixed for 5.0 s)`, then `blade ON`. The mow finished: MOWING_COMPLETE, docked, the body
  never more than 0.015 m outside by ground truth.

## 5. Open

- **Field test.** Nothing here has run on the mower. Watch the first RTK loss: the hold
  should last until the fix has settled (5 s of FIXED), and the map pose should stay put
  while the robot stands.
- **Corrections outage.** The trigger was an NTRIP outage (diff_age 120 s), not sky
  view. A corrections watchdog (alert on `corr=reconnecting` for more than a minute) would
  name the cause in the GUI.
- **Float for a long time.** A garden spot where RTK stays float now stops a mow that a
  dropout interrupted, until `loc_lost_dock_s` (600 s). Before, float resumed it. Plain
  float mowing without a dropout is unchanged.
- **Tilt back-out** is not gated directly (2.1 table); the catch-all re-holds a tick later.
- **VIO.** With `vio:=true`, the wheel vy = 0 and the VIO vy share the weight in ekf_map.
- **Single filter.** `localization_mode:=single` (`ekf.yaml`) still does not fuse vy.
