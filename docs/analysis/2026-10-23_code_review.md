# Code review: nodes and rules (2026-10-23, master e93249d)

This was a read-only review. Its findings await the owner's approval stage by stage. The full report is in the session transcript; this file records the actionable parts.

## Verdict
The stack is **safe by accumulation, not safe by design**. Every field incident added a new rule, so the same questions get answered 3–5 times by different code, with different margins, clocks and budgets:
- "am I moving?"
- "am I inside?"
- "is something in front?"

`mission_fsm.py` is now 7,326 lines with 279 params. The A6 deletions (the RPP configs, one limiter) have not happened yet.

## Key findings
- **`/dig_stall` is unreachable** (`slip_detector.py:277`). Its threshold is 0.3, which equals the maximum command, so a full stall never exceeds it, and the latch never clears. Yet docking's `use_dig_stall` relies on it. **Safety.**
- **Two monotonic clocks under sim time:**
  - `cm_stop_since` (`mission_node.py:714`), so `edge_blocked_skip` can't be verified in the sim gate;
  - the watchdog timer (`nav_lane_watchdog.py:130`).
- **Collision "blocked" input is parsed from `/rosout` log text,** including RPP strings (`mission_fsm.py:160-197`). Use `/collision_monitor_state` and action result codes instead.
- **`cmd_vel_slew` copies `MOTION_PHASES` from the FSM by hand** (`:1106`), so the motion interlock can drift.
- **`cmd_vel_slew` parameter callbacks are not thread-safe** against `step()` (`:1475-1508`).
- **The hard boundary check silently switches off when `outside_m` is stale** (more than 1 s, e.g. during a map edit) (`mission_fsm.py:6585-6591`). Stale should mean stop. **Safety.**
- **Dead config and code:**
  - 3 RPP configs plus the `rpp` revert branches;
  - the FTC controller and its plugin (about 1.5k lines of C++);
  - the stale `static_layer` block;
  - `body_outside_m` (no consumer);
  - `_split_route_legacy`;
  - the legacy edge layout;
  - the slew revert flags;
  - `motor_load.py`, which has no consumer yet.
- **Duplicate machinery:**
  - three escape machines (`_stk`, `_tob`, `_hb`);
  - three hold machines (`_dyn`, `_blk`, `_loc`);
  - 17 separate retry/budget counters.
- **Five boundary layers,** plus `_lethal_agrees` and `BOUNDARY_PAUSED`.
- **No single source of truth for geometry:**
  - track 0.40 appears in 8 places;
  - the footprint appears in about 12;
  - `robot_rear_m` is 0.12, against 0.10 elsewhere;
  - `marker_size` is 0.04 in 4 files.
- **Copied helpers:** `sub_pump.py` exists in 11 identical copies (8.3k lines). `node_now`, `yaw_from_quaternion` and `wrap_angle` are duplicated too.
- **Swallowed exceptions** in docking (`:1247`, `:991`), the supervisor and fixed_frame_relay.
- **`time.time()` in runtime logic:** heading_logic, map_server and the supervisor.
- **Testing:** no CI, and the node classes are untested.

## Consolidation map
| Concern | Single owner | Delete |
|---|---|---|
| Stuck | Detect: slip_detector. React: the FSM `Escape`. The dock's `StallGuard` stays. | `/dig_stall`, `use_dig_stall`, the watchdog recover path |
| Boundary | One geometry function on the atomic area snapshot, and one reactor with a margin table (hard / soft / lethal) | `body_outside_m`, the map_server flag debounce, `_lethal_agrees`, `BOUNDARY_PAUSED`, `static_layer` |
| Obstacle | obstacle_guard: policy. collision_monitor: state. FSM: one `Hold` reactor. | `/rosout` parsing, the three hold dicts |
| Motion shaping | cmd_vel_slew: rates and the wheel floor. MPPI: limits. The WheelFloorCritic stays as defence in depth. | velocity_smoother from the lane, the slew revert flags |
| Retries | One `Budget` helper | per-site backoff code |

## Plan
**Stage 0: zero behaviour change.**
1. Delete RPP and FTC.
2. Delete the dead blocks and branches.
3. Create a `mower_common` package: `sub_pump`, clocks, angle helpers, `MOTION_PHASES`, geometry constants; fix `robot_rear_m`.
4. Fix the clocks and the swallowed exceptions.
5. CI (B5).

**Stage 1: behaviour-preserving, sim-gated.**
6. Split `mission_fsm.py` mechanically into `fsm/` modules as mixins, so the existing tests run unchanged.
7. Add `Escape`, `Hold` and `Budget` base helpers.
8. Take collision input from `/collision_monitor_state` instead of `/rosout`.

**Stage 2: behaviour changes; owner approval and field tests.**
9. Remove `/dig_stall`.
10. One boundary reactor; a stale `outside_m` means stop.
11. Remove velocity_smoother from the lane.
12. Move nav_lane_watchdog into the supervisor, or delete it; set `marker_size` to the measured 0.055.

**Do first:** 0.1, 0.3, 0.4, 1.6, 2.9.

## Dig stall and stuck detection (added 2026-10-23, read-only review)

### Ours today (master 00c041d)
| # | Mechanism | Condition (signals) | Window / threshold | Latch / clear | Who reacts | Field evidence |
|---|---|---|---|---|---|---|
| 1 | `/dig_stall` (`slip_detector.py:545-596`) | `abs(meas_lin - cmd_lin) > 0.3` **or** `abs(meas_ang - cmd_ang) > 0.3`, body twist, signed difference then absolute; `meas` = `/odom` twist (no `/wheel_vel` publisher in our stack, so it is the MCU `SpeedData` = **wheel encoder** speed, `mcu_node.py:1760`); `cmd` = last `/cmd_vel`, **not age-checked**. Gated on `/fix_status` containing `RTK_FIXED` (um960 driver). | 2.0 s continuous; any sub-threshold sample restarts | Latches forever; only a node restart clears it | docking (`use_dig_stall`, rising edge vs goal start, `dock_logic.py:1737`), map_server terrain incident. Mission and slew ignore it. | **Never latched.** Slip conditions started 125× (21 Oct dock log), 200× (22 Oct), all angular (d 0.300-0.447) at pivot reversals, all cleared within 0.1-0.3 s. |
| 2 | `/stuck` reason `stuck` (`StuckDetector`, `slip_detector.py:240`) | commanded (abs v > 0.03 or abs w > 0.1, cmd younger than 0.5 s) while the EKF map pose (`/odometry/filtered_map`, RTK-fused) moved < 0.05 m, turned < 10° (5° if ≥ half the window is pure pivots), path < 0.10 m, yaw path < 20° | 4.0 s, gaps ≤ 1.5 s tolerated | Clears when EKF moves > 0.3 m from the latch, on `/stuck_guard/suppress` or `/stuck_guard/reset`. Exempt: docking FINAL_DOCKING/CHARGING (fresh ≤ 1 s), collision_monitor STOP (window restarts), stale cmd | `cmd_vel_slew` forces zero; mission `_stuck_tick` escape | Fires: 21b (6 latches, wheels 0.01-0.05 m = blocked, not slipping), 22 Oct dock push (5 latches at 0.38,-1.32 with v=-0.05; fixed since by the dock exemption + `_at_dock()` ignore) |
| 3 | `/stuck` reason `spinning` (`slip_detector.py:243`) | wheel distance (integral of abs `/odom` linear) ≥ 0.2 m **and** > 3 × max(EKF chord, EKF path) | same 4.0 s window | as #2 | as #2 | The only real dig detection that ever fired: 21c 06:37:00.77, reverse arc v=-0.12 w=-0.28, wheels 0.34 m vs EKF 0.097 m. Blind to pivot spin (linear wheel speed ≈ 0 in a pivot) and to one-wheel spin that averages out. |
| 4 | Mission escape (`mission_fsm.py:6146-6300`, `mission.yaml:523-535`) | `/stuck` in STUCK_PHASES (TRANSIT, MOWING, RETURNING_HOME, *_DOCKING) | pause 1.0 s, then 3 steps of 0.3 m at 0.1 m/s: reverse, forward arc 30°, reverse opposite arc; a step aborts after 3 s without 0.05 m EKF progress | Escaped = step done or > 0.3 m from origin; then costmaps cleared, terrain incident, resume | STUCK_NEEDS_HELP on: all 3 steps failed, 4th stuck within 120 s, a stuck within 0.5 m of an escaped spot not left by 1 m, no pose | 21c 06:37: step 1 reverse 0.01 m in 3 s (failed), step 2 forward arc done. 21b: 4 escapes ~0.30 m. |
| 5 | Docking `StallGuard` (`dock_logic.py:551-607`, `docking.yaml:134-142`) | abs cmd > 0.03 and `/odometry/filtered` displacement + abs(dyaw)×0.2 < 0.2 × integral of abs cmd | 1.0 s sliding, restarts if moving | DOCKING/FINAL: hold 3.0 s for contacts, then FAILED `DOCK_STALLED`; RETRY: fail at once | docking FSM | (dock fail logs; not re-checked here) |
| 6 | `DriveLoadDetector` (`motor_load.py`) | abs(L)+abs(R) current > 20 A, mean abs v_cmd ≥ 0.08 over 1 s, ground speed < 0.3 × cmd; spin vs stall by rpm | 1 Hz motor frames, 1 frame confirm, 10 s holdoff | — | **Nobody (unwired)** | Offline: 1 hit in 73 min (the 06:37 spin, **2.0 s before** #3); 1 in 21b. Pivots out of scope (15-16 A). L/R labels swapped. |

### The "unreachable `/dig_stall`" claim: verified, with refinements
- **What is compared:** body twist, signed difference, then absolute value, per axis: `abs(meas.x - cmd.x) > 0.3` or `abs(meas.wz - cmd.wz) > 0.3` (strict `>`). It does not compare per wheel or against ground truth.
- **Command range:** `/cmd_vel` is limited to v ∈ [-0.10, 0.30] by MPPI (undock up to 0.30) and abs(w) ≤ 0.30 (`nav2_params.yaml:285-288`, the MCU clamp).
- **Full stall:** meas = 0 gives d = abs(cmd) ≤ 0.30. That is **never > 0.30**, so the claim holds on both axes.
- **Digging:** `meas` is the wheel-encoder speed. Spinning wheels track the command, so d ≈ 0. Even with a lower threshold, this rule measures how well the motors track the command, not slip. **This is the deeper reason it is useless.**
- **When it is reachable:** only when meas has the opposite sign to cmd, or overshoots it by more than 0.3:
  - linear: cmd -0.10 with meas > +0.20, or cmd 0.30 with meas < 0;
  - angular: pivot reversals, where d reaches 0.447 in the logs.
  
  It also needs 2.0 s without a single sub-threshold sample. The field never came close: the longest run was 0.3 s.
- **Stale command:** `step()` never ages `/cmd_vel`. A held last command of 0.30 combined with a small negative measured speed could latch it falsely. This is theoretical; it has not been seen.
- **RTK gate:** the RTK gate depends on the um960 `/fix_status` string. Replacing the GNSS driver would disable the rule silently.
- **Latch:** **it never clears**; only a respawn clears it. Docking reacts only to a rising edge after the goal starts, so after any latch the docking dig guard stays dead until the node restarts.

### Stock (v1.3.27 devel `.so`, Ghidra 10.4 headless)
Notation: @X is the Ghidra address; the file VA is X−0x100000. Confidence is high unless marked otherwise.

- **Distance slip, `check::CheckStuck`** (`libmower_controller_check.so`, getStuckStatus @2dfeb8; defaults from Config @2e1150 match `check_stuck.h:22-34`; no override found):
  - **Signals:** the commanded distance is the integral of the published cmd_vel hypot(vx, vy); angular is ignored. The measured distance is the chord of `/odom_fused` (the RTK-fused pose). **No IMU, wheel odometry or motor current is used.**
  - **Window:** 2.5 s. A tick is a hit when cmd ≥ 0.05 m, fused < 0.2 m and fused < cmd/3.0.
  - **Latch:** 3 hits latch it; more than 5 consecutive misses clear it.
  - **Severity:** 1 below 3 s, 2 from 3 s, and 3 (`is_stuck`) at **6 s** of latched slip.
- **Angle slip, `check::CheckSlide`** (SetNodeHandle @262674):
  - **Signals:** yaw integrated from `/wheel_vel` wz, compared with IMU yaw; both pass through a low-pass filter with α 0.15.
  - **Trigger:** a difference > **30°** over 2.5 s. The value is the double 0x3fe0c152382d7365 in Check::Init @26210c; it overrides the header's 25°.
  - **Latch:** the same 3 hits / 5 misses. The severity bands are medium confidence.
- **Motor current:** no current-based check exists in mower_controller.
- **Reaction in coverage** (NavCover, `libmower_controller_cover.so`):
  - **Severity 2:** a speed cap. Angular is capped at 0.3 rad/s (CheckSlip @1b96bc). Linear is scaled to a limit whose value was not found.
  - **Severity 3:**
    - The controller sends a `/controller/event` escape event and pauses with the blade off and zero speed for 2.0 s (@1b820c).
    - It then runs an **open-loop ladder of 8 × 3.0 s** (MotionInit @1ff340): v +0.3, −0.3, +0.5, −0.5, then (±0.5, ±0.3) in four combinations. The MCU clamps speed to ±0.3, so the 0.5 steps run at 0.3 in practice; this is an inference.
    - **Success:** the fused pose is more than 0.6 m from the escape start.
    - **Failure:** the controller stops, sends a fail event and goes to **EXCEPTION**, "waiting for manual intervention" (@1feee4).
    - **Repeated stucks are never counted or capped.**
- **Edge mode** (NavEdge):
  - **Direction-aware:** after a forward slip it backs up at −0.25, −0.35, −0.45 m/s, then turns at ±0.45 rad/s, each for 2.0 s; after a reverse slip it does the mirror image (@932360/@932778).
  - **Success:** more than 0.15 m or 25°; otherwise EXCEPTION.
  - It also runs a legacy `SlipDetection` (ratio 3, 0.2 m, **5.0 s**; header defaults, medium confidence).
- **Earlier note "5 s slip ratio, unstuck sequence, no escalation":**
  - **5 s: wrong for the main detector.** That figure comes from the legacy `mower_pnc` cfg and NavEdge.
  - **Unstuck sequence: confirmed.**
  - **No escalation: partly right.** There is no repeat counter, but a failed ladder does stop the robot in EXCEPTION.

### Comparison
| | Ours | Stock |
|---|---|---|
| Ground truth | RTK-fused EKF map pose | `/odom_fused` (RTK-fused) |
| Distance slip | wheel distance > 3× EKF path, ≥ 0.2 m, over 4 s → act at once | cmd distance > 3× fused, fused < 0.2 m, over 2.5 s → act at 6 s latched (≈ 6.5-8.5 s) |
| No-progress stall | yes (`stuck`: < 0.05 m / 10° / path limits over 4 s) | covered by the distance rule (uses cmd, not wheels) |
| Pivot / yaw slip | only via `stuck` (5° heading floor); `spinning` is blind to it | wheel yaw vs IMU yaw > 30° over 2.5 s |
| Motor current | `DriveLoadDetector` exists, unwired (2 s earlier on 06:37) | none |
| Early slow-down | none | severity 2: speed / ω caps |
| Escape | 3 closed-loop steps × 0.3 m at 0.1 m/s, 3 s no-progress abort, success at 0.3 m | 8 open-loop moves × 3 s at 0.3 (0.5) m/s, success at 0.6 m; edge mode is direction-aware |
| Escalation | STUCK_NEEDS_HELP: 3 failed steps, 4 stucks per 120 s, or the same spot twice | EXCEPTION after a failed ladder; no repeat counter |
| Docking | exempt in FINAL_DOCKING/CHARGING; `_at_dock()` ignore; own `StallGuard` 1 s / 20 % | (not traced) |

### Recommendation
1. **Detector.** `slip_detector`'s StuckDetector stays the single owner, with one `/stuck` latch and its existing exemptions. Its reasons:
   - `stuck`: unchanged.
   - `spinning`: compares wheel *travel* against the EKF path, measured as (abs(v) + abs(w)·b/2) per tick. Today it uses only the linear part, which leaves it blind to pivot spin. Pivot travel credits EKF yaw × b/2 on the EKF side, mirroring `StallGuard`'s turn credit. *(Stage 2.)*
   - `load`: `DriveLoadDetector` wired in, as a ≥ 2 s confirmation (sum > 20 A, ground speed < 0.3 × cmd) that shortens the latch for straight driving. First fix the L/R label swap. *(Stage 2.)*
   - Optional: stock's wheel-yaw vs IMU-yaw rule as a **speed cap only**, not a latch. *(Stage 2.)*
   - The ground truth stays the EKF map pose, with the existing freshness gate. No RTK string gate.
2. **Reaction.** Keep **our** closed-loop escape and `STUCK_NEEDS_HELP` escalation. They beat stock's open-loop 8 × 3 s ladder at up to 0.3 m/s on turf already being dug. Borrow two ideas from stock:
   - make step 1 *opposite to the commanded direction at the latch*. On 21c 06:37 the robot was reversing, and our step 1 reversed again and failed;
   - add a severity-2 style speed cap on the first slip evidence.
   
   *(Both are stage 2.)* Fold `_stk` into the Stage 1 `Escape` helper.
3. **Delete.** Remove these:
   - `/dig_stall` (`step()`, the `slip_*`/`require_rtk_fixed`/`fix_status` params, the `/wheel_vel` discovery);
   - docking `dig_stall_topic`/`use_dig_stall`/`_dig_at_start`;
   - map_server `terrain_dig_topic` (the mission already logs a `stuck` terrain incident);
   - the `/dig_stall` entries in bag_recorder and mow_recorder.
   
   It never fired, so removal changes no observed behaviour. The plan keeps it as **stage 2.9**; it is safe to pull forward as zero-behaviour cleanup if the owner agrees.
4. **Docking.** Docking owns its contact phases. Keep:
   - the FINAL_DOCKING/CHARGING exemption and the `_at_dock()` ignore;
   - `StallGuard` as docking's only stall signal (EKF progress, 1 s / 20 %, a 3 s contact grace).
   
   In DOCKING and NAV_TO_APPROACH, `/stuck` still drives the mission escape; that is correct.
