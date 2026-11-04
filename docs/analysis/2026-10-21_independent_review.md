# Independent review and work queue (2026-10-21)

This is a read-only review of master at ec8be43, done while `ws/edge-layout2`, `ws/fast-sim` and `ws/docking` (merged as 485c707) were in progress. The reviewer's report is condensed here, and the queue that follows from it is kept below.

**Rule from the owner:** NEW ideas or changes of direction need the owner's approval. Incremental improvements go on the queue and are validated by gates.

## Verdict

The numbers in this section are as of 2026-10-21. By 2026-11-04 the FSM is split into
`mower_mission/fsm/` (about 10.8k lines), the RPP configs are deleted, and `mission.yaml` has
359 keys (see `2026-11-04_open_work_audit.md` section 2.5).

The campaign is converging, but on a narrow front.

**Fixed against measured failures:**
- stop latency;
- pivots on the vertex;
- the GNSS lever arm;
- grass false stops (125 → 0);
- the first complete mows.

**What keeps the loop slow:**
- There is no faster-than-real-time gate yet.
- Every fix is a new mission-FSM rule: `mission_fsm.py` is about 6.3k lines and `mission.yaml` has 238 keys.
- `nav2_params.yaml` still carries 3 RPP and 3 MPPI controller configs, although MPPI is the single tracker.

**Against stock:**
- Not yet better on edges or obstacles.
- Already better on boundary safety, docking diagnostics, observability and resume.

## Overlooked or underweighted

- **The field-run log has lapsed.** `wheel_smoothness_field_tests.md` ends at run 7, and `STATUS.md` is dated 10-05.
- **No integral action in the tracking chain.** A constant actuator asymmetry (the 0.40 wheel split against the 0.39 track, tyre diameter, wheel bias) shows up as a steady cross-track offset. That is the likely cause of the 4 cm right bias.
  - Calibrate wheel scale per wheel from RTK on a long straight.
  - Check `gps_link` y with a reversed-direction straight: an antenna offset flips sign between directions, a wheel bias does not.
- **Per-wheel motor current** (`MotorInfo.current`) is on the bus but unused. It could detect slip, stall and blade load.
- **`cutter_size` is unused.** Row spacing is fixed at 0.18 (`area_settings.py:75`).
- **No coverage map.** Nothing tracks what was actually cut, so there is no missed-patch revisit and no honest "% mowed" after skips. Stock has none either, so this is a better-than-stock opportunity.
- **Slope turns.** A pivot on the west-edge slope slid 0.40 m with RTK fixed, at the same spot in 3 sessions. Phase 4 (slope-aware turning) is unscheduled.
- **No explicit edge-lap obstacle policy.** Stock turns AI avoidance off on edge laps (`OS_Bumper_AI_Except_Edge`).
- **Observability is degrading:** corrupt bag splits, corrupt docker json logs and a supervisor black-box error. Treat this as P1 reliability.
- **Deploy has no rollback** marker or health check.
- **No CI and no bag-replay regression tests.** Field incidents become prose instead of tests.

## Process notes

- **Converging and diverging at once.** Each field run removes one failure class, but the code grows guards on guards.
- **The gate cannot see** grass, slope, RTK loss or coverage. Its body check covered MOWING only; TRANSIT is being added.
- **Practices to add:**
  - bag-replay FSM regression tests;
  - per-run metrics JSON appended automatically to the field log;
  - a scenario library (west-edge slope, a bumper object, map reload mid-mission, RTK float) run in the fast sim;
  - CI running pytest plus the ideal gate;
  - an owner dashboard fed by the black box.

## Stock vs ours

The findings doc has no single comparison table. The per-topic stock behaviour is in
`2026-10-21_stock_firmware_findings.md`, and the comparison table is section 4 of
`2026-11-04_open_work_audit.md`. The gaps to stock as of 2026-10-21:
- K-turns;
- slope turn mode;
- labelled-class obstacle policy and edge-lap AI off;
- blade-aware spacing;
- schedule / allowed times.

## Owner decisions (bucket A): all six approved 2026-10-21

1. Fast-sim equivalence (1× against 3×) as a hard prerequisite for trusting fast verdicts.
2. Wheel-scale and track calibration from RTK (the 4 cm bias, pivot drift).
3. Coverage map, % mowed, and a missed-patch revisit pass.
4. Motor-current stall and blade-load sensing.
5. Phase 4 slope-aware turns, moved ahead of Phase 1.2.
6. Freeze new FSM guards: any addition must come with a removal. Delete the RPP configs and keep one limiter.

## Queue (bucket B: incremental, gate-validated)

| # | Item | Validation |
|---|---|---|
| B1 | Row spacing as a MowgliNext UI setting. The GUI already has `tool_width`, `blade_radius` and `blade_x_offset`/`blade_y_offset` (`useSettingsManager.ts:97`, `MowgliNextPage.tsx:104`). Bind them to the coverage swath spacing and the blade geometry (blade at 0.23 forward, disc radius 0.11 small / 0.135–0.15 big). Default the spacing from the MCU `cutter_size`: small 0.18 (UI range 0.13–0.18), big about 0.23 (UI range 0.16–0.23), as in the stock app. An owner value overrides the default, globally and optionally per area. | Changing the UI value changes the planned spacing; swapping the blade changes the default; plan spacing matches |
| B2 | Per-spot boundary budget (2 within 1 m), per-mission cap about 6 | 0 STUCK on real_run5 `-d` s1/s2 |
| B3 | `outside_m` from the mission's area snapshot in all phases, plus atomic map replace | A map PUT mid-mow uses 0 budget |
| B4 | Resume the field log: auto-append wheel_bag_metrics JSON per run | Each field run produces an entry |
| B5 | Bag-replay FSM tests for 06:31 (map reload) and 07:29 (bumper repeat); CI pytest on push | Tests fail on the old code, pass on the new |
| B6 | **Owner override, not stock:** keep camera avoidance ON during edge laps, but act only on detections directly on the planned path in the way. Then detour around the obstacle (no stop-and-wait), mark it on the map, and rejoin the path. | Gate with an on-path obstacle in an edge lap: detour plus map tag, no stop; off-path detections ignored |
| B7 | Log and bag integrity: split size, fsync, disk guard | 0 corrupt splits over 3 runs |
| B8 | Deploy script writes a rollback tag and checks container health | A bad deploy can be rolled back in one command |
| B9 | (2026-10-22 follow-up 3, dropped until 2026-10-29) `mcu_node` `odom_linear_scale`: scale the MCU-measured speed published on `/odom`, not the commands (`linear_scale`). Field 10-22: ground/odom 1.049; 10-28: 1.066 (varies with turf, re-check with `scripts/wheel_cal.py`). Prerequisite for GNSS smoothing in ekf_map. | Unit test; `wheel_cal.py` odom scale within 1 % of 1.0 on the next mow |
| B10 | (2026-10-22 follow-up 3) Confirm the left/right motor label swap wheels-up and fix `MOTOR_NAMES`; pivot reversing-wheel deficit (3.0 vs 5.1 rpm) | Wheels-up check by the owner |
| B11 | (2026-10-22 follow-up 4) Wire `DriveLoadDetector` (slip_detector feeds it, `/stuck_event` reason `drive_load`), log-only for 2 mows first | 2 field mows logged, no false events |
| B12 | (2026-10-22 follow-up 2, owner) Blade-off / blade-on calibration drive (procedure in `2026-10-22_wheel_scale_and_motor_current.md`) to settle the yaw trim (~+0.007 rad/s, `cmd_vel_slew` `angular_trim_radps`) | `wheel_cal.py` on both halves |
| B13 | (2026-10-29 localisation wander, `2026-10-29_localisation_wander.md`) Weave: (1) MPPI heading damping / control point ahead of the rotation centre, (2) GNSS smoothing in ekf_map (x/y process noise), (3) realistic sim localisation (fix latency ~0.1 s, effective antenna lever 0.072, no 2.5 cm jumps). Owner 2026-10-29: do (2) and (3) now, (1) after (3). (2) and (3) done on `ws/loc1029` (2 off by default, field A/B pending). (1) done on `ws/damp1029`: coverage MPPI `PathAlignAheadCritic`, control point 0.25 m ahead (section 6): field-model `-d` a 0.079 (was 0.13-0.16), e / h pass, weave band -45 %, i still fails as on every baseline; field check pending. | (3) sim reproduces field weave by the field mechanism; (2) field A/B; (1) sim + field |

### Queue status (2026-11-04)

"Merged" means the code is on master; "field" says whether a mow has confirmed it.

| # | Status | Evidence / next step |
|---|---|---|
| B1 | Partial | Per-area `swath_width_m` with a disc-derived default (`mower_map/area_settings.py`); `cutter_size` is parsed but not used as the default |
| B2 | Merged, seen in the field | `hard_boundary_spot_max: 2` within `hard_boundary_spot_m: 1.0` (`mission.yaml:478-479`) raised STUCK_NEEDS_HELP twice on 2026-11-04. Stock never escalates on boundary-outs; it counts contacts per place instead (findings doc, 2026-11-04 section 4) |
| B3 | Merged | The body check uses the mission's own area snapshot (`fsm/common.py:1104,1135`) |
| B4 | Open | Field log last entry 2026-10-23 |
| B5 | Merged | Incident replay tests in `mower_mission/test/replay/`; CI workflow exists but has not run remotely (73 commits unpushed on 2026-11-04) |
| B6 | Merged, field-unconfirmed | Detours for zones and static marks work in the field; costmap-only obstacles are on `ws/fix1104` |
| B7 | Merged, field-unconfirmed | Own-pass zstd with verify; needs a deploy with the `image` step, then 3 mows with 0 corrupt chunks |
| B8 | Open | `deploy_to_mower.sh` writes only `DEPLOYED_REV` |
| B9 | Merged, field-confirmed | `odom_linear_scale` 1.06 in bringup; ground / fused odom 1.004-1.013 on 2026-10-31 and 2026-11-04 |
| B10 | Open | Owner wheels-up check |
| B11 | Open | `motor_load.py` is not wired; waits for B10 |
| B12 | Open | `angular_trim_radps` is 0.0; field suggestions +0.004 to +0.010 rad/s vary by area |
| B13 | (1) merged, field -3 to -9 % p90; (2) failed the 2026-11-01 A/B; (3) merged | `2026-10-29_localisation_wander.md` sections 6 and 7 |

## Risks flagged in in-progress work

These were sent to the implementing agents.

**`ws/edge-layout2`:**
- The soft side limit has a thin margin; K-turns are not yet implemented.
- The 0.3 m reverse does not fix a side excess.
- `_hb_jumped` holds only 2 s; atomic map replace is missing.
- `emergency_stamp is None` blocks START in sim and on the bench.
- Body mode flips at path junctions.

**`ws/fast-sim`:** in Jazzy, some Nav2 loops may run on wall rates; about 184 wall-clock calls remain; `node_now` reads wall time before the first `/clock`.

**`ws/docking`:**
- `marker_size` is 0.04 against the real 0.055.
- The 8° pose check is noisy.
- `dock_timeout_s` is 480.
- Field 2026-10-21 night: the marker is found, but the motion is erratic.

## Schedule (2026-10-21)

| Slot | Work | Status |
|---|---|---|
| Running | A1: fast-sim equivalence, a hard prerequisite (`ws/fast-sim`) | Running |
| Running | Step 3 round 2 (`ws/edge-layout2`), which includes B2 and B3 | Running |
| Running | A2: wheel calibration; A4: motor-current analysis (`ws/wheel-cal`) | Running |
| Running | Docking motion diagnosis (field 2026-10-21 night) | Running |
| After `ws/edge-layout2` | A5: slope-aware turns | Queued |
| After `ws/edge-layout2` | A3: coverage map and revisit | Queued |
| After `ws/edge-layout2` | B6 (owner variant) | Queued |
| After `ws/edge-layout2` | A4 integration | Queued |
| After `ws/edge-layout2` | A6 cleanup: delete the RPP configs, one limiter | Queued |
| From now on | A6 rule: new FSM guards need a matching removal | Applies to all work |
| After `ws/edge-layout2` | Resume slider (owner 2026-10-22): the GUI resume shows a draggable progress slider, e.g. drag to 60 % to skip mowed ground or redo a stretch. The map previews the start point. It sets the mission resume cursor (`coverage_resume.txt`) through a new mission service, validated against the current plan, and START then resumes from there | Queued |
| After `ws/edge-layout2` | Stable area IDs in the stack (`MapArea` id persisted in `areas.dat`; mission, schedule and GUI bind by id; `GUI_REBASE_TASKS.md` 1.1/1.2). Prerequisite for re-forking the vendored GUI onto upstream `dev` (2–3 weeks). Owner approved 2026-10-22 | Queued |
