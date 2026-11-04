# Open-work audit (2026-11-04, master 41e61e3)

Read-only audit of what is unfinished, what needs tidying, and where stock firmware still does better. Nothing was changed except this file. Evidence is a file:line, a doc section, a branch or a commit. "Unverified" means I could not confirm it from the repo (usually because it needs the mower).

Sources: `analysis/2026-10-21_independent_review.md` (queue B1-B13, decisions A1-A6), `REVIEW_2026-11-02.md`, `STATUS.md`, `analysis/2026-10-23_code_review.md`, `analysis/2026-10-21_stock_firmware_findings.md`, the other `analysis/*.md`, git state, and a grep of `src/`.

Out of scope on purpose: the 2026-11-04 field fixes on `ws/fix1104` (listed as in progress only), the parallel stock-firmware review, and the owner policies (blade kept on through people and animal waits and in-area detours; camera avoidance on in edge laps, B6 owner variant).

## 1. Summary: the ten things that matter most

Goal: mows its map well, docks reliably, better than stock.

| # | What | Why it matters | Evidence |
|---|---|---|---|
| 1 | Recovery paths still fail in the field. The three 2026-11-04 problems (edge-lap branch skipped instead of detoured, RTK pause mid-pivot skipped the pivot, resume re-planned and restarted) are being fixed on `ws/fix1104`. | Every field failure of the last two weeks was a recovery failure, not a feature gap. | `REVIEW_2026-11-02.md` section 1; `ws/fix1104` (4cebcc9, one commit so far, worktree clean) |
| 2 | Night and dusk docking is unfixed: the SEARCH state re-seeds the heading from a flipping marker solution, so each approach starts 0.1-0.2 m off. 2026-10-31 night dock took 77 s and 5 approaches. | Dusk returns will repeat this. Seeding from RTK is a small change. | No docking commit since 2026-10-31 other than the charger-moved check (`git log --since=2026-10-31 -- src/mower_docking`); candidate fixes not built |
| 3 | Heading is single-antenna. About 1,350 lines exist to estimate it. Keyframe estimator and self-recovery are merged but neither has had a field A/B. UM982 is specced, not bought. | Root cause 1 of the review. | `REVIEW_2026-11-02.md` section 1 and step 4; `STATUS.md:594`, `:1094`; `HEADING_RECEIVER_2026-11.md` |
| 4 | Unattended recovery after reboot or crash is not started. The hard-boundary check still fails open when `outside_m` is stale. | "Never needs the owner" is the target; a stale value switches the safety check off. | `REVIEW_2026-11-02.md` section 4b (step 6 not started); `fsm/guards/boundary.py:25-31` (`_outside` returns None when stale; `mission.yaml:476`) |
| 5 | Slopes: pivots on 8-12 degree side slopes cannot be corrected after the fact. The planning-side fix (`slope_mode`, contour swaths, corners off the edge) is open and defaults to off. The sim slope world still fails. | The west-edge slope has caused three field incidents. | `REVIEW_2026-11-02.md` Round 2; `fsm/common.py:167`; `simulation.md` "Slope handling gate runs" |
| 6 | Sim gate criterion (i) fails on master, and run-to-run noise (+-0.05 m) is as large as the effect being gated. The lap-0 margin (about 1 cm over at convex bends) sits inside that noise. | A gate that cannot discriminate cannot protect the fixes. | `simulation.md:317`; `STATUS.md:1260` area (gate rows); `analysis/2026-10-23_code_review.md` stage-1 validation (0.62 m vs soft limit 0.61 at (-5.14, -1.00)) |
| 7 | A large block of features is "unit-tested only" and has never been confirmed on the mower, and the field log stopped on 2026-10-23. | Risk is hidden in the backlog; no per-run evidence trail. | `STATUS.md` section titles containing "unit-tested only" or "not yet run on the mower" (lines 194, 225, 239, 594, 709, 793, 929, 1132, 1157, 1183); `wheel_smoothness_field_tests.md:900` (last entry) |
| 8 | Guard sprawl is growing, not shrinking. `mission.yaml` has 359 leaf keys and `Params` about 343 fields, against 238 keys when the A6 freeze was approved and a target of under 200. | Each new key is another interaction to test. | yaml parse of `src/mower_mission/config/mission.yaml`; `independent_review.md` verdict; `REVIEW_2026-11-02.md` step 5 |
| 9 | Deploy has no rollback or health check, and the `image` step has been skipped on two deploys (Wi-Fi chip blank, zstd missing). | A bad or partial deploy is the one thing the owner cannot undo from the lawn. | `scripts/deploy_to_mower.sh:56-69` writes only `DEPLOYED_REV`; queue item B8 |
| 10 | No coverage map: "% mowed" comes from plan progress, there is no missed-patch revisit, and resume is a cursor, not a mowed raster. Stock resumes from position plus a mowed raster. | Directly behind the 2026-11-04 "resume re-planned and restarted" failure, and a place we can beat stock. | `mower_mission/mow_progress.py:18,68`; stock findings "3. Resume after STUCK"; A3 in the independent review |

## 2. Open work by area

Status words: open (not started), partial, in progress, merged-unverified (code on master, no field confirmation), parked.

### 2.1 Mowing and coverage

| Item | Status | Evidence | Next step | Blocked on |
|---|---|---|---|---|
| Lap-0 boundary margin, about 1 cm over the limit at convex bends | open | Owner report 2026-11-04. Same mechanism as the stage-1 soft-limit trip (`code_review.md`, stage-1 table) and owner option (c), lap 0 keeps its natural arc overhang (`stock_firmware_findings.md` "Decision 2026-10-22") | Decide whether 1 cm matters. If yes, raise the lap-0 corner allowance by the measured excess or slow the bend; add a gate scenario with an `owner_far`-type bulge | Owner policy call |
| Sim gate (i) fails on master | open | `simulation.md:317` (8 runs of identical code: i side 0.087-0.188); STATUS gate rows for param-diet, cpu-diet | Judge on the median of 3 seeds with a stated tolerance, or split (i) into side and sweep with different bands | None |
| A3 coverage map, true "% mowed", missed-patch revisit | open | No coverage raster in `src/` (grep); `mow_progress.py` percent is plan-based | Rasterise the blade footprint from the EKF track; compute % and unmowed patches; use it for resume and a final revisit pass | None. Do before slope work, it fixes resume too |
| B1 row spacing from the GUI and `cutter_size` | partial | `swath_width_m` per-area setting with disc-derived default (`mower_map/area_settings.py:85-87,132,158-171`); `cutter_size` is only parsed (`mcu_node.py:1399`), nothing uses it as the default | Default the swath from `cutter_size` (0.18 small, about 0.23 big); confirm the GUI blade-geometry bindings (unverified) | None |
| A5 slope-aware planning and turns | open | `slope_mode` default `off` (`fsm/common.py:167`); `REVIEW_2026-11-02.md` Round 2 "remains the open step 4"; `ws/slope-pivots` parked; `ws/slope-creep` merged (path trim, roll feed-forward) but the sim slope world still fails on west-edge pivots (`simulation.md` slope gate tables) | Contour swaths and corners kept off the slope edge, tested in the slope world; then a field check on the west edge | Owner field slope data for tuning |
| Pivot time share (25-27 % of moving time against an 18 % target) | parked | `STATUS.md:1211`; `analysis/2026-11-03_turns.md` (snake and tangent options off, net 11 s; 6 remaining transits unexplained) | Record the planner's row-change log line in the gate to find why 6 transits remain; check the 0.35 rad/s pivot rate against the MCU clamp on the robot (unverified) | Mower access for the clamp check |
| B9 `odom_linear_scale` and B12 yaw trim | merged-unverified | `mcu_node.py:555` (default 1.0); `cmd_vel_slew.py:227` (`angular_trim_radps` 0.0). Field suggestions were 1.049-1.067 and +0.006 to +0.010 rad/s (`independent_review.md` B9; wheel-scale analysis) | Run `scripts/wheel_cal.py` on the next mow; apply values as settings only if both halves agree | Next field mow |
| Weave (p90 about 2.0 cm) | parked | GNSS smoothing A/B failed on 2026-11-01 (weave unchanged, one boundary-out). The result is not written in `analysis/2026-10-29_localisation_wander.md` (the doc still presents the A/B as pending, section 5.3) | Revert smoothing (compose without the third file), record the result, remove the flag (see 3.2). Further weave work is controller-side only | Owner redeploy without the override |

### 2.2 Obstacles

| Item | Status | Evidence | Next step | Blocked on |
|---|---|---|---|---|
| Edge-lap branch skipped instead of detoured | in progress | Owner brief; stock detours and rejoins ahead (`stock_firmware_findings.md` "Lessons" item 1) | Fix is on `ws/fix1104` (not my finding) | `ws/fix1104` |
| B6 owner variant, on-path edge-lap detour with map tag | merged-unverified | Merged with `ws/nogo-c` (`project` notes 2026-10-23; `STATUS.md:725-760` for bounded skips). 2026-10-28 field run found stacked zones and a sliver re-plan; fixes merged later | One field mow with an on-path obstacle in an edge lap | Field mow |
| Same-place obstacle counter, bump-mark growth, end-of-mow confirmation pass | partial | `stock_firmware_findings.md` "Lessons" items 2 and 7. Bumper contacts already create per-place tracks with `bumper_hits` (`mower_map/obstacle_tracker_core.py:426-434`, `obstacle_tracker_stuck.py`); whether the radius grows with repeats, and whether STUCK_NEEDS_HELP counts contacts instead of boundary-outs, is unverified. No end-of-mow confirm pass found | Check radius growth and the escalation trigger; add the confirm pass | None |
| B11 motor-current `DriveLoadDetector` unwired | open | `mower_control/motor_load.py` is referenced only by `test_motor_load.py`; `code_review.md` stuck table row 6 | Wire it log-only for 2 mows as a `/stuck` reason `load`, after B10 | B10 wheels-up check |
| B10 left/right motor label swap, pivot reversing-wheel deficit | open | `mcu_node.py:201` `MOTOR_NAMES`; `independent_review.md` B10 | Wheels-up check, then fix the names | Owner on the robot |
| Slip handling vs stock (early speed caps, direction-aware step 1, wheel-yaw vs IMU rule) | open | `code_review.md` "Recommendation" items 1-2, marked Stage 2 | Cap speed on first slip evidence; make escape step 1 oppose the commanded direction | Owner approval (Stage 2) |
| Grass segmentation off by default; obstacle reroute | unverified | `STATUS.md:697-699` still say "not yet run on the mower", `nongrass:=false` | Confirm whether either ran since 2026-10-09; update the status text | Mower access |
| Auto no-go persistence (option D) | parked | `analysis/2026-10-23_obstacle_nogo_options.md` ordering: D only after clean mows with A | Leave until clean mows are logged | Clean mows |

### 2.3 Localisation

| Item | Status | Evidence | Next step | Blocked on |
|---|---|---|---|---|
| Dual-antenna heading | open | `HEADING_RECEIVER_2026-11.md`; magnetometer dropped (`STATUS.md:1116`) | Buy and bench a UM982-class receiver against COG, then fuse as absolute yaw, keep `heading_logic` as watchdog | Owner purchase; driver port question |
| Keyframe COG estimator A/B | merged-unverified | `cog_method` default `window`; replay shows keyframes p95 2.97 against 3.42 deg (`STATUS.md:1094-1116`) | Field A/B with `cog_method:=keyframes`; flip the default if it holds | Field mow |
| Heading self-recovery after a reboot off the dock | merged-unverified (field test pending) | `STATUS.md:594-631`; `fsm/phases/start.py`, `fsm/phases/hcal.py` | Reboot off the dock, press START, expect a blind drive then a mow; record in the field log | Field test |
| Graded RTK-loss policy and pivot gap | merged-unverified | `STATUS.md:1157-1181`; `localization_status_logic.py:63` (`lost_after_s` 10). Stock coasts on distance and a stationary pivot spends none (`stock_firmware_findings.md` "2. RTK float dropout", lesson 5) | Let a pivot finish on gyro through a float gap up to about 10 s; apply `lost` only to translation. Field-check a real float gap | Design choice, then field |
| Site calibration TODOs in localisation config | unverified | `mower_localization/config/navsat.yaml:53-55` (`yaw_offset`, declination); `docking.yaml:44` TODO(calib) | Confirm whether navsat is in the live chain; delete or fill | None |
| RTK base pairing from the web app | merged-unverified | Landed in 335ef53; plan in `analysis/2026-10-30_rtk_base_card_plan.md` | Field check of pair, reset and `lora_status` | Mower access |

### 2.4 Docking

| Item | Status | Evidence | Next step | Blocked on |
|---|---|---|---|---|
| Night and dusk dock: SEARCH re-seeds heading from a flipping marker solution (2026-10-31 night dock, 77 s, 5 approaches, daylight same day 27 s) | open | No `src/mower_docking` change since 2026-10-31 except 0cab7da and e416d46; project note lists the candidate fixes | Seed the SEARCH heading from RTK when the dock pose is measured; or `solvePnPGeneric` and keep the solution nearest the RTK heading; consider a darker night preset (20 or 10). Add a dusk marker to the sim | Dusk field test |
| `marker_size` 0.04 vs measured 0.055 | decided in code, undocumented in the review | `docking.yaml:33-41` keeps 0.04 on purpose (stock-equivalent thresholds); `code_review.md` stage 2 item 12 still says "set to measured 0.055" | Close item 12 as "kept"; remove the stale instruction from the review | None |
| Dock timeout 900 s and 8 retries | merged-unverified | `STATUS.md:1268+` param diet (dock timeout 900) | Watch one full retry cycle | Field |
| Charger-moved alert GUI hook | open | `STATUS.md:1189-1192` "the GUI side is not wired yet" | Wire the dock card offer | None |
| Dock-after-incomplete | merged-unverified | `return_home_on_incomplete` now true (`mission.yaml:397`, `fsm/common.py:1017`) | One mow that ends MOWING_INCOMPLETE with RTK fixed; confirm it docks. The 2026-11-01 mow idled 92 min before this default | Field |

### 2.5 Mission and FSM

| Item | Status | Evidence | Next step | Blocked on |
|---|---|---|---|---|
| RTK pause mid-pivot skipped the pivot; resume re-planned and restarted | in progress | Owner brief; `ws/fix1104` 4cebcc9 (hold that interrupts a coverage pivot redoes the rest on resume) | Not my finding. After merge, add the 2026-11-04 incidents to the replay fixtures (the commit adds one unit test, `test_hold_pivot_resume.py`) | `ws/fix1104` |
| A6 guard freeze | breached | 238 keys (2026-10-21) to 359 leaf keys now; `Params` about 343 fields (REVIEW said 326 on 2026-11-02; STATUS says 369 to 338 after the first diet). FSM is split into `fsm/` (10.8k lines, `mission_fsm.py` is a 36-line facade); `mower_mission` source is 15.8k lines | Second diet pass aiming at under 200 keys: delete the remaining "false = old behaviour" flags and one recovery-motion planner. Every addition needs a removal | None |
| Code review stage 2 (behaviour changes) | partial | Done: 2.9 (`/dig_stall`, pulled forward). Open: 2.10 stale `outside_m` fails open (`boundary.py:25-31`); 2.11 `velocity_smoother` still in the lane (`nav2_params.yaml:677-681`, `navigation.launch.py:29`); 2.12 `nav_lane_watchdog` still present (`mower_navigation/scripts/nav_lane_watchdog.py`) | Make stale `outside_m` mean stop (owner approval, Stage 2); decide on smoother and watchdog | Owner approval |
| One recovery-motion planner (retrace, inward, back, tilt back-out, pivot back-out) | open | `REVIEW_2026-11-02.md` step 5; `fsm/escape.py` shares a helper but the manoeuvres stay separate | After the diet pass | Diet pass |
| Unattended resume after a reboot or crash | open | `REVIEW_2026-11-02.md` step 6, section 4b "Not started" | Persist mission intent; on boot with RTK fixed and heading OK, resume; never after EMERGENCY or lift | None |
| `return_home_on_incomplete` | merged-unverified | See 2.4 | See 2.4 | Field |
| Heading self-recovery | merged-unverified | See 2.3 | See 2.3 | Field |
| Resume progress slider | open | `independent_review.md` schedule row (owner 2026-10-22) | After the coverage map: resume cursor set from position | A3 |
| Stable area IDs, then re-fork of the vendored GUI onto upstream `dev` | open | `independent_review.md` schedule last row (2-3 weeks, approved 2026-10-22); no id field found in `map_server_node.py` by grep | Add persistent `MapArea.id` first; it also stops wrong-area risks | None |

### 2.6 GUI

| Item | Status | Evidence | Next step | Blocked on |
|---|---|---|---|---|
| UX findings F12, F14 (new-zone dialog, path labels) | open | `STATUS.md:877` "Not done" | Fix with Playwright against the sim | None |
| Remaining audit items: obstacle/no-go/keepout naming, mow angle in four places, Diagnostics "unknown" group, Logs page empty before open, Schedule rules hard-coded ON | open | `REVIEW_2026-11-02.md` section 5 "Also:" | Batch into one UX pass | None |
| GUI hooks for dock-moved, float-fix vertex colouring, alert translations | open | `STATUS.md:1189-1192`, `:1207-1209` | Wire and test | None |
| Wi-Fi setup page: only tested against a fake | merged-unverified | `STATUS.md:1055-1067` | Check connect, forget and band on the mower | Mower access |
| BLE transport, secure origin, Tailscale card | merged-unverified | 335ef53; `analysis/2026-10-30_ble_link_measurements.md` | Field check of auto-switch and reconnect | Mower access |
| Rain: sensor verified on hardware, dock-until-dry | open | `REVIEW_2026-11-02.md` step 10 | Verify the sensor, then the mode | Hardware |

### 2.7 Sim and CI

| Item | Status | Evidence | Next step | Blocked on |
|---|---|---|---|---|
| Criterion (i) and noise | open | See 2.1 | See 2.1 | None |
| Gate validity depends on host load | known | `STATUS.md:1010` (rate race fixed, load-dependent INVALID runs remain: "-x 1 for verdicts") | Make the gate refuse to run above a load threshold (the script sets the power profile already) | None |
| CI has never run on the remote | unverified | Workflow `.github/workflows/ci.yml` exists; `git rev-list --left-right --count origin/master...master` gives `0 73` (73 commits unpushed) | Push (owner) and watch the first run | Owner push |
| CI tolerates a known failing test | open | `scripts/ci_test.sh:32` `CI_KNOWN_FAILURES` defaults to `test_scenario_transit_box`; the 2026-10-23 queue lists it | Fix or delete the test | None |
| Sim gaps: night marker, RTK-float pivot scenario, replay fixtures for the 2026-11-04 incidents | open | Review step 2 lists a night marker; the merged sim-fidelity work lists roll, slip, RTK injection, kills only (`STATUS`/`simulation.md`) | Add the three scenarios | None |
| Gate does not run in CI | by design | `ci.yml` header (45+ min) | Keep a nightly gate on the dev host | Host |

### 2.8 Ops and deploy

| Item | Status | Evidence | Next step | Blocked on |
|---|---|---|---|---|
| B8 rollback tag and health check | open | `deploy_to_mower.sh:56-69` | Tag the previous images, check container health after `up`, one-command rollback | Owner runs deploys |
| `image` step skipped on deploys | open | Project notes 2026-11-02 and 2026-11-03 (Wi-Fi chip blank, zstd missing) | Have the script warn when the Dockerfile or its inputs changed since the last image build | None |
| B4 field log | open | `wheel_smoothness_field_tests.md:900` last entry is 2026-10-23; later mows (10-28, 10-31, 11-01, 11-02, 11-03) are not in it | Auto-append `wheel_bag_metrics.py` JSON per mow | None |
| B7 bag recorder chunk corruption | merged-unverified | Two fixes: single compression thread (`STATUS.md:683`), then own-pass zstd with verify (`STATUS.md:1132`). The 2026-11-03 mow still lost 1 of 23 chunks before the second fix. Needs an image rebuild for `python3-zstandard` | After a deployed mow, `zstd -t` every chunk; 0 corrupt over 3 mows closes B7 | Deploy with `image` |
| H4 MCU heartbeat timeout | open | `STATUS.md:1085` and `MCU_HEARTBEAT_BENCH_TEST.md` ("not run") | Wheels-up bench test | Owner on the robot |
| H2 memory cost of standalone nodes | unverified | `STATUS.md:1075` "about +150 MB PSS, not measured" | Measure `MemAvailable` after deploy | Mower access |
| Deployed revision | unverified | A deploy of 41e61e3 was cleared but I cannot see whether it ran | Read `DEPLOYED_REV` | Mower access |
| Unpushed work | open | 73 commits ahead of `origin/master` | Owner pushes | Owner |

## 3. Tidy-up

### 3.1 Branches and worktrees (recommendations only; none were touched)

Seven local branches are not merged into master.

| Branch | Verdict | Evidence | Action |
|---|---|---|---|
| `ws/fix1104` | live | Worktree `ros2_stack-wt/fix1104` is clean; one commit 4cebcc9 on top of master; owner says more fixes are coming | Keep. Merge when the 2026-11-04 batch is done |
| `ws/ble-bridge`, `ws/ble-https`, `ws/ble-seam`, `ws/ble-web` | superseded | The four jobs landed as master 335ef53 ("RTK base card, secure origin, BLE transport and Tailscale on Tron"); `scripts/ble/ble_bridge.py`, the transport registry and the secure-origin code exist on master (the newer registry differs from `ws/ble-web`). `git cherry` shows them unmerged only because the patch ids differ; each also carries an unrelated stash-snapshot commit ("WIP on master", "index on master") | Delete all four |
| `ws/rtk-base-card` | superseded | `rtk_base.go`, `GetLoRa.srv` and the plan doc are on master through 335ef53; two stash-snapshot commits | Delete |
| `ws/slope-pivots` | dead (negative result) | Gate results worse, "never reverse up the slope" (0443ef3, a72d8d7, a1c656e, 17a2143); conclusion recorded in `REVIEW_2026-11-02.md` Round 2 and `analysis/2026-11-03_turns.md` | Cherry-pick the docs commit 0443ef3 if its gate tables should be kept (unverified that they are on master), tag the branch tip, then delete |

Other tidy-ups:

| Item | Evidence | Action |
|---|---|---|
| 44 merged `ws/*` branches | `git branch --merged master` (all dated 2026-10-29 to 2026-11-03) | Delete them all |
| Remote ref `origin/migration/jazzy` | Retired 2026-10-13 (project notes; `JAZZY_MIGRATION_CHECKLIST_2026-10-09.md`) | Delete the remote branch (owner push) |
| Tags `fanout-base-2026-10-13`, `fanout-base-2026-10-30-ble`, `fanout-base-2026-10-30-rtk` | `git tag` | Delete: fan-out snapshot markers whose branches are gone or superseded |
| Worktrees | Only `fix1104` exists; `ros2_stack-wt/` is 29 MB | Nothing to do. Earlier root-owned build directories from older worktrees are gone |
| Uncommitted: `docker/docker-compose.gnss-smoothing.yml` | Experiment that failed on 2026-11-01 | Delete, or keep only as an example inside the analysis doc |
| Uncommitted: edit to `analysis/2026-10-29_localisation_wander.md` | Documents the override file for the A/B | Rewrite as the A/B result (failed), then commit |
| Uncommitted: `analysis/2026-10-30_rtk_station_identity.md` | Reverse-engineering notes on the RTK base | Review for identifiers before committing (I did not read it in full; a grep finds a few lines mentioning serial, pairing or password words) |
| Three private working docs | Intentionally untracked | Leave as they are |

### 3.2 Dead code and config

| Item | Evidence | Action |
|---|---|---|
| RPP and FTC controllers are gone, but comments still describe them | `nav2_params.yaml:16-17, 84-86, 116, 141-177, 582, 881-882`; `slip_detector.py:103-106`; `cmd_vel_slew.py:325`; `nav2_param_tools.py:6-9`. A6 item "delete the RPP configs" is done (stage 0.1 merged: `9ac9cd7`) | Rewrite the comments; the file is 1,412 lines |
| `gnss_smoothing` switch and its config | `ekf_map_gnss_smoothing.yaml`, `mower_localization/setup.py`, `test_ekf_dual_config.py`, `mcu_node.py` | Remove after the owner reverts the override |
| `motor_load.py` | Unwired (see 2.2) | Wire (B11) or delete |
| Superseded packages and the off-by-default perception stack in the build | `src/` has no `COLCON_IGNORE`; `mower_proto` and `base_ble` remain (`base_ble_node.py:174,240,331` are TODO stubs); `REVIEW_2026-11-02.md` step 5 estimates about 16k lines | Move out of the build or delete, as step 5 says |
| `dig_stall` terrain kind | `mower_map/terrain.py:43` (kept for old saved incidents) | Fine; remove once old incident files are gone |
| Velocity smoother and `nav_lane_watchdog` | See 2.5 (stage 2) | Decide with the stage 2 approval |
| Remaining revert flags | Param diet retired 17 of 46; 50 tests still set a flag explicitly (`STATUS.md` param diet) | Delete flags older than two field runs |
| `ci_test.log` at the repo root | Ignored by `.gitignore:32` | None |

### 3.3 Doc drift

| Doc | Problem | Fix |
|---|---|---|
| `STATUS.md` | Header says "Last updated 2026-10-05". It is a 1,302-line dated log, so the opening sections (what exists, roadmap phases A-C, "Open risks" from 2026-10-09) describe a different project state. Several section titles say "not yet run on the mower" for features that may have run since | Split into a short current-state page and a dated log; refresh Open risks |
| `analysis/2026-10-21_independent_review.md` | Queue items B1-B13 carry no status; verdict numbers are stale (6.3k lines, 238 keys, RPP configs still present); points to a "Stock vs ours" table that does not exist in the findings doc | Add a status column; fix the pointer |
| `REVIEW_2026-11-02.md` | Scorecard in 4b and "Round 2" overlap; step 5 and 6 text predates the param diet | Fold into one scorecard |
| `analysis/2026-10-21_stock_firmware_findings.md` | The "Our port, before round 2" sections describe states that were replaced | Mark them historical |
| `analysis/2026-10-23_code_review.md` | Stage 2 item 12 (`marker_size` 0.055) contradicts `docking.yaml:33-41` | Mark as decided |
| `mcu_node.py:78-100` docstring | "Known gaps ... Heartbeat period unknown" predates H4 (`/mcu/driver_heartbeat`); the odometry note may also be stale (unverified) | Update |
| `ROADMAP_2026-10-06.md`, `ROADMAP_2026-10.md`, `JAZZY_MIGRATION_CHECKLIST_2026-10-09.md`, `REVIEW_2026-10-06.md`, `REVIEW_2026-10-13.md` | Historical | Move to `docs/archive/` |
| Evidence paths under `ros2_stack-wt/accuracy-simout/` cited in several docs | Outside the repo, not reproducible | Note which results are kept elsewhere |
| GNSS smoothing result | Only in the working notes | Add to `analysis/2026-10-29_localisation_wander.md` |

### 3.4 Test gaps

| Gap | Evidence | Action |
|---|---|---|
| Strict xfail on dock edge cases since 2026-10-23 | `test_dock_logic.py:924` `_EDGE_XFAIL` ("shorter realign / 0.90 m cap") | Fix the expectation or the code |
| Marker check xfail for 4 cm markers at 0.8 m | `test_marker_check.py:49` | Documented limit; revisit if `marker_size` ever changes |
| Known CI failure tolerated | `ci_test.sh:32` | See 2.7 |
| Environment skips (rclpy stubs, cv2.aruco, zstd, F2C v3) | `skipif` and `GTEST_SKIP` across `src/` | Confirm CI runs each at least once with the dependency present |
| Slip and slope paths cannot be exercised by unit tests alone | `REVIEW_2026-11-02.md` section 1 (cause 2); sim-fidelity added roll and slip | Keep slope and kill scenarios in the nightly gate |
| FSM tests now use the shipped flags (643 of 693) | `STATUS.md` param diet | Good; keep |
| Node classes untested | `code_review.md` "Testing" | Open |
| MCU heartbeat bench test never run | `MCU_HEARTBEAT_BENCH_TEST.md` | Owner bench run |
| GUI end-to-end in CI | Not found in `ci.yml` (unverified) | Add Playwright against the sim |

## 4. Where stock still does better

| Item | What stock does (citation) | Our gap | Suggested fix | Effort |
|---|---|---|---|---|
| Edge-lap obstacles | Detours and rejoins ahead on the original path, skips only when the edge end is occupied within 2 m (`stock_firmware_findings.md` "Lessons" 1) | Branch on the lap was skipped on 2026-11-04 | `ws/fix1104` batch, in progress | In progress |
| Resume | Position plus a mowed raster; re-plans only the unmowed area; clears only when area count or ids change (findings "3. Resume after STUCK") | Cursor file; resume re-planned and restarted on 2026-11-04 | A3 coverage raster, resume by nearest path and pose | L |
| RTK dropout | Coasts on a 50 m distance budget or 10 s of wheels; a stationary pivot spends none (findings "2. RTK float dropout"; `localization.md:296`) | `lost` hold at 10 s fires inside a pivot | Finish the pivot on gyro, hold only translation | S-M |
| Heading upkeep | Keyframe estimator, 0.2 s updates, weight 0.7, IMU-steadiness gates (`STATUS.md:1094-1116`) | Ported but off by default; single antenna | Field A/B then flip default; UM982 | S plus hardware |
| Turn modes | Arc, fish-tail, slope, adaptive (`turns.md:30`; findings "Edges, turns and coverage") | We pivot; 25-27 % of moving time. Research found no arc or fish-tail that fits the 0.60 m nose | Slope-aware planning (A5); fewer transits | L |
| Slip response | Severity 2 speed and rate caps, direction-aware escape, wheel-yaw vs IMU rule (`code_review.md` "Stock") | Detect late, no early caps, step 1 can repeat the failing direction; load detector unwired | Stage 2 items plus B11 | M |
| Repeated bumps | Mark radius grows with same-place count, exception at the third repeat (findings "4. Repeated boundary-outs") | Per-place bumper tracks exist (`obstacle_tracker_core.py:426`); radius growth and escalation trigger unverified | Confirm, then count contacts rather than boundary-outs | S |
| Suspect-spot revisit | `ObsCheckPath` confirms suspects after coverage (findings lesson 7) | Decide in flight | Cheap end-of-mow confirm pass | M |
| Pre-detour slowdown | Speed ramps from 0.6 m to 0.15 m/s near 0.3 m (findings line 674, lesson 1) | Unverified whether we ramp | Check, then match | S |
| Auto-found no-go zones with photos for the owner | Task report lists them with pictures (`REVIEW_2026-11-02.md` section 3) | Pending zones in the GUI, no photos | Attach a camera crop to pending zones | M |
| Day and night dock exposure | HSV brightness picks the preset (findings "Detection") | We step exposure; night SEARCH init still flips | See 2.4 | S |
| Rain and work window in the mission | `AutoChargeCheckEx` handles rain and allowed times (`audit_2026-10-05/vendor_mission_layer.md` 1.4) | Schedule rules hard-coded ON in the GUI; rain sensor unverified | Verify the sensor, expose the rules | S-M |
| Edge mapping, voice prompts, OTA incl. MCU firmware, camera-blocked photo | `REVIEW_2026-11-02.md` section 3 | Not built (camera-blocked monitor is) | Low value; skip unless asked | L each |

Not worth chasing (review decision): VIO through RTK loss; CPU is already near load 8-12 (`REVIEW_2026-11-02.md` section 3).

### Where we are already better

| Area | Evidence |
|---|---|
| Boundary safety: hard and soft margins, body-aware recovery, budgets. Stock's boundary-out never escalates | `stock_firmware_findings.md` "4."; `REVIEW_2026-11-02.md` section 1 |
| Stuck handling: closed-loop escape and `STUCK_NEEDS_HELP`. Stock retries forever with a 180 s sleep and its escape is open-loop | `code_review.md` comparison table; findings "4." |
| Interrupted pivots: ours redoes the rest, stock clears the turn and restarts tracking (the fix is `ws/fix1104`) | findings "2." lesson 4 |
| Docking diagnostics and cross-checks (marker against RTK, realign budget, charger-moved) | `STATUS.md:1183-1192`; findings "Fixes" |
| Observability: bag ring, black box, wheel metrics, incident replay tests | `REVIEW_2026-11-02.md` section 1; `ci_test.sh` |
| Resume UX and recovery ladder, graded RTK-loss policy with unified speed caps | `STATUS.md:1157` |
| Link security and access: HTTPS origin, BLE tunnel, Wi-Fi setup, Tailscale | 335ef53 |
| Planner features beyond stock: hard and soft edge modes, no-go zones with mid-mow re-plan, wall lines, per-area settings, map editor tools | `REVIEW_2026-11-02.md` section 1; `STATUS.md:543`, `:929` |
| A simulator with an 11-criterion gate (roll, slip, RTK loss, process kills) | `simulation.md` |

## 5. Suggested order, next two weeks

Rules: sim before field, one sim agent at a time, owner deploys.

| When | Work | Done when |
|---|---|---|
| Days 1-2 | Finish and merge `ws/fix1104`; add the three 2026-11-04 incidents as replay fixtures; one valid gate (host idle, `-x 1` verdict); owner deploys including the `image` step | Fixtures red before the fix, green after; deploy log shows the image rebuilt |
| Day 2 | Housekeeping: owner deletes the 4 superseded branches, merged `ws/*` branches, stale tags and `origin/migration/jazzy`; write the GNSS smoothing result into the localisation doc; revert the override; STATUS header and Open risks refresh | `git branch` shows master plus live branches only |
| Days 3-4 | Dusk dock: seed SEARCH heading from RTK; add a dusk marker to the sim; owner does one dusk dock | At most 2 approaches at dusk |
| Day 4 | Fail closed on a stale `outside_m` (stage 2.10, needs owner approval) and decide the smoother and watchdog items | Test: stale value stops the mower |
| Day 5 | One instrumented field mow with a checklist: keyframe heading A/B, self-recovery after reboot, dock-after-incomplete, recorder chunk integrity, graded RTK policy; auto-append the metrics to the field log (B4) | Field log has an entry for each; 0 corrupt chunks |
| Week 2, days 1-2 | Gate hygiene: median-of-3 verdict for (i); decide the lap-0 1 cm margin | Gate verdict stable across reruns |
| Week 2, days 2-4 | A3 first slice: coverage raster, honest % mowed, resume by position | % mowed matches the track on the sim; resume does not re-plan covered ground |
| Week 2, day 4 | B8 rollback tag and health check; warn when `image` is stale; wire `DriveLoadDetector` log-only (after the owner's B10 wheels-up check) | One-command rollback tested on a dev box |
| Week 2, day 5 | A6 pass: remove stale RPP comments, `gnss_smoothing`, dead packages from the build, a first batch of old revert flags | `mission.yaml` below 330 keys, tests unchanged |
| Open decisions | UM982 purchase; whether the 1 cm lap-0 overhang is acceptable; slope planning approach (A5) after the coverage raster | Owner call |
