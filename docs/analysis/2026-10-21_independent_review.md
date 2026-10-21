# Independent review and work queue (2026-10-21)

This is a read-only review of master at ec8be43, done while `ws/edge-layout2`, `ws/fast-sim` and `ws/docking` (merged as 485c707) were in progress. The reviewer's report is condensed here, and the queue that follows from it is kept below.

**Rule from the owner:** NEW ideas or changes of direction need the owner's approval. Incremental improvements go on the queue and are validated by gates.

## Verdict

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

See the table in `2026-10-21_stock_firmware_findings.md`. The gaps to stock:
- K-turns;
- slope turn mode;
- labelled-class obstacle policy and edge-lap AI off;
- blade-aware spacing;
- schedule / allowed times.

## Owner decisions pending (bucket A)

1. Fast-sim equivalence (1× against 3×) as a hard prerequisite for trusting fast verdicts.
2. Wheel-scale and track calibration from RTK (the 4 cm bias, pivot drift).
3. Coverage map, % mowed, and a missed-patch revisit pass.
4. Motor-current stall and blade-load sensing.
5. Phase 4 slope-aware turns, moved ahead of Phase 1.2.
6. Freeze new FSM guards: any addition must come with a removal. Delete the RPP configs and keep one limiter.

## Queue (bucket B: incremental, gate-validated)

| # | Item | Validation |
|---|---|---|
| B1 | Consume `cutter_size`: spacing defaults by blade | Plan spacing matches the blade; coverage unchanged |
| B2 | Per-spot boundary budget (2 within 1 m), per-mission cap about 6 | 0 STUCK on real_run5 `-d` s1/s2 |
| B3 | `outside_m` from the mission's area snapshot in all phases, plus atomic map replace | A map PUT mid-mow uses 0 budget |
| B4 | Resume the field log: auto-append wheel_bag_metrics JSON per run | Each field run produces an entry |
| B5 | Bag-replay FSM tests for 06:31 (map reload) and 07:29 (bumper repeat); CI pytest on push | Tests fail on the old code, pass on the new |
| B6 | Edge-lap AI avoidance off (stock), bumper only | Gate plus one field run |
| B7 | Log and bag integrity: split size, fsync, disk guard | 0 corrupt splits over 3 runs |
| B8 | Deploy script writes a rollback tag and checks container health | A bad deploy can be rolled back in one command |

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
