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
