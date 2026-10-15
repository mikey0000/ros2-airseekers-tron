# Design: drivable paths first, then tight tracking (2026-10-15)

Status: proposed. It replaces the corner handling in roadmap task 0.4 (d, g) and the turn
shaper's rewriting. Evidence: runs 3–5 in `wheel_smoothness_field_tests.md`.

## 1. Problem

On straight lines the mower already follows tightly. Run 5 (2026-10-15, full Area 1)
measured the cross-track error against the planned coverage lines while MOWING:

| | median | p90 | p99 | max |
|---|---|---|---|---|
| cross-track | 0.026 m | 0.074 m | 0.204 m | 0.262 m |

All the large errors, and every boundary excursion in runs 3–5, happen at **sharp corners**:

- The ring V corner at (−4.74, −1.17) caused 2 hard-boundary trips (0.11 m, 0.12 m).
- The dock-path 111° corner on the slope: the Spin timed out, the leg was driven anyway,
  and the robot went 0.12 m out.
- At the dock-path 39° and 57° bends, MPPI cuts 0.20–0.34 m (sim).
- Run 3 had seven transit excursions of up to 0.85 m, where the shaper widened tight corners.

The cause is the paths themselves. Two kinds of path reach the controllers unchanged:

- **Drawn paths** (the channels "Dock path", "Path 2", "Path 3") are hand-drawn polylines
  with sharp vertices and drawing jitter.
- **Coverage rings** are the area polygon offset inward by `boundary_inset_m` 0.05 m
  (`mower_coverage/src/coverage_planning.cpp`, Fields2Cover headlands). The offset keeps
  every sharp vertex of the drawing. Only the swath ends get real Dubins / Reeds-Shepp turns.

The robot cannot drive a sharp vertex without stopping. A path with such a vertex asks for
something the robot cannot do. Each controller fails in its own way:

- RPP arcs wide and overshoots outside.
- MPPI cuts the corner, or dithers at near-zero speed.
- The shaper widens the arc or turns it into a pivot.

Since run 1 we have added one runtime fix per failure: shaper regimes, the leg cap, corner
splits, corner-reached rules, progress watches, the stuck guard, and the boundary retrace.
Each fixed one symptom and moved the failure to the next corner.

## 2. What the robot can physically drive

Diff drive, track b = 0.48 m. The MCU clamps |v| ≤ 0.3 m/s and |w| ≤ 0.3 rad/s. A wheel
asked for less than about 0.06 m/s while turning does not move (the turf/MCU floor).

- **Arc of radius r.** The robot has w = v/r, inner wheel speed v − w·b/2 ≥ 0.06 and
  w ≤ 0.3. Solving gives **r ≥ 0.44 m**, reached at v ≈ 0.13 m/s. Above that the speed
  limit is v ≤ 0.3·r, so a 1.0 m arc can be driven at full 0.3 m/s. We use
  **r_min = 0.5 m**: the existing `min_turn_radius_m`, which leaves margin.
- **Pivot.** Both wheels counter-rotate at ≥ 0.06 m/s, so |w| ≥ 0.25 rad/s. It sweeps only
  the footprint and needs grip. In the field it failed once on the slope (a 20 s Spin timeout).
- **Reeds-Shepp / K-turn.** Forward and reverse arcs at r ≥ r_min. Use it where a pivot
  isn't possible (slope) and there is room.

Anything else, such as a vertex at speed or an arc tighter than 0.44 m, is not drivable.

## 3. The method: condition every path once, then track it

### 3.1 Path conditioner (one function, used for every path)

The input is a polyline (a transit route, a dock route, or a coverage sub-path) plus the
**allowed region**: the areas ∪ drawn path channels, minus the margin.

1. **Simplify.** Run Douglas–Peucker at 0.02 m. This removes drawing jitter so that only
   real corners remain (`geometry.route_corners` already does this).
2. **Classify each corner.** Let θ be the heading change and r = r_min, or larger if the
   space allows. A fillet arc cuts the corner by `d = r·(1/cos(θ/2) − 1)` and needs a
   tangent length `t = r·tan(θ/2)` on both neighbouring segments.

   | θ | d at r = 0.5 m |
   |---|---|
   | 39° | 0.030 m |
   | 57° | 0.069 m |
   | 79° | 0.148 m |
   | 90° | 0.207 m |
   | 111° | 0.383 m |

   The choice per corner, in order:
   - **Fillet (arc)** if `t` fits both segments and the whole arc (with the robot's
     half-width and the margin) stays inside the allowed region. Use the largest r that fits,
     up to about 1.0 m, so it can be driven at full speed.
   - **Pivot at the vertex** otherwise. The vertex is on the drawn path, so the robot stays
     on the line.
   - **K-turn** (Reeds-Shepp with reverse) when a pivot is not allowed, for example when
     the tilt is above the slope limit (Phase 4), and there is room.

   This is "turn based on how much space it has". The space check is against the real
   boundary, not a fixed angle threshold. On a convex area corner (the ring turns inward),
   the fillet moves the path inward and away from the edge. On a reflex corner (the turn
   points outward), the fillet moves toward the edge, and the space check decides.
3. **Edge-quality policy for the outermost ring.** A fillet leaves a small uncut patch at
   each convex corner. Its area is r²·(tan(θ/2) − θ/2). For sharp corners on the outermost
   ring, prefer pivot-at-vertex, because the edge cut matters. Inner rings and transit use
   fillets.
4. **Resample and annotate.** Output poses at fixed spacing (0.05 m). Each pose carries a
   segment type (line / arc / pivot / reverse) and a curvature. The speed limit at each
   pose follows from §2: v ≤ min(v_max, 0.3·r), and lower before a pivot.
5. **Validate.** Every pose of the output is inside the allowed region with the margin,
   and every arc has r ≥ r_min. A failed check is a planning error. It is reported in the
   GUI and never driven.

### 3.2 Tracker: MPPI is the candidate to beat (owner, 2026-10-15)

The path conditioner (§3.1) does not depend on the controller: every controller does
better on a drivable path. The owner wants MPPI to be the controller we invest in, for all
of transit, dock routes and coverage, rather than going back to pure pursuit.

MPPI is in use today (transit `FollowPath`; `FollowCoveragePathMPPI` is configured but
unused). Its field failures are known, and each one points to a fix:

- **Dither at near-zero speed** (2026-10-15 undock spin; STUCK1/2 at the junction).
  - **Cause:** MPPI samples (v, w) pairs that the robot cannot drive (an inner wheel
    between 0 and 0.06 m/s). The turn shaper then rewrites them, so MPPI's motion model is
    wrong about what happens next.
  - **Fix:** make MPPI aware of what is feasible, so it never chooses those commands. For
    example, add a custom critic that penalises inner wheel speeds below the floor, or
    constrain the sampling. The shaper is then only a clamp.
- **Corner cutting** (0.20–0.34 m at the 39°/57° dock-path bends).
  - **Fix:** a conditioned path (§3.1), plus critic weights (PathFollow / PathAlign /
    PathAngle) tuned against cross-track in the sim gate.
- **Accel-limit livelock** (3196ffd, reverted in 1e19c15).
  - **Fix:** `ax`/`az` set from the measured wheel response, not from the slew constants,
    and verified in the sim.
- **CPU** (RK3588; the control loop missed 20 Hz at 17.5 Hz).
  - **Fix:** measure batch size / time steps against tracking error; the sim gate reports
    the loop rate.

Work package (`docs/analysis/2026-10-15_mppi_study.md`):

1. review the current MPPI config, the field bags and the Nav2 MPPI documentation and
   issues (and MowgliNext's MPPI attempt);
2. design a feasibility-aware MPPI (critic and/or sampling) and a tuning plan;
3. run the A/B in the sim gate: MPPI (tuned) vs RPP, on raw and on conditioned paths,
   with and without the disturbance.

The criteria are the same as §4, plus loop rate and smoothness (jerk, pivot share). MPPI replaces pure pursuit as the one tracker for transit, dock routes and coverage; RPP
stays only as a revert parameter, not as a second controller in use.
Pivots and reverse segments remain explicit steps whichever controller follows the
lines.

Because every commanded curvature is drivable, the turn shaper no longer needs to rewrite
anything. It becomes a **safety clamp** (wheel floor, slew) and no longer makes decisions.

### 3.3 What gets removed once it is measured to be unnecessary

Each item is removed only after the sim gate and one field run show it no longer fires:

- the turn shaper widening, pivot-instead and pivot-hold regimes (they become a clamp only);
- route corner splitting by angle threshold, the corner-reached / stall rules and the
  start-stub trimming (replaced by the conditioner's pivot markers);
- the turn-leg time cap (fix 10), as a decision path. It stays as a backstop log only;
- the costmap `navigate_to_pose` fallbacks for transit. A transit that can't follow the
  conditioned route stops and reports;
- pivot assist creep (to be checked: with drivable paths it should rarely fire).

These stay, because they are safety rules and not path handling:

- the hard boundary stop and retrace;
- the stuck guard;
- the motion gate;
- obstacle stops.

## 4. How we measure (every change, every time)

1. **Sim gate first.** `scripts/sim_gate.sh` (being built on `ws/coverage-corners`) runs a
   full mow from the dock on a copy of the current real `areas.dat`: undock, transit, all
   sub-paths, home, dock. It runs both with and without the slip/heading disturbance.
   Pass criteria:
   - zero samples more than 0.10 m outside, and zero hard-boundary trips;
   - MOWING_COMPLETE, docked and charging;
   - no stuck events, no fallback navigation;
   - no one-wheel ticks;
   - cross-track p99 ≤ 0.10 m on lines and arcs.

   A change that fails the gate does not go to the field.
2. **Field run.** One change-set per run. Pull the bag and report the same table as the gate,
   split by segment type: cross-track median / p90 / p99 / max on lines, on arcs and at
   pivots; boundary margin; pivot time share; corners per type.
3. **Keep or revert** in a "Run N" entry, as before.

The sim cannot show slope, real turf slip or RTK loss. Slope-dependent turn choice (pivot vs
K-turn) is Phase 4 and needs field data.

## 5. How this differs from what we do now

| | Now | Proposed |
|---|---|---|
| Where corners are handled | At runtime, in four places: controller (MPPI/RPP), turn shaper, mission corner rules, leg/stuck watchers | Once, before driving, in the path conditioner |
| Path given to the controller | Raw drawn polylines and polygon-offset rings with sharp vertices | A drivable path: lines, arcs r ≥ 0.5 m, explicit pivots / K-turns |
| Turn choice | A fixed angle threshold (79°) for transit routes only; coverage rings get none | Per corner, from the measured space to the boundary (fillet if it fits, else pivot, else K-turn) |
| Controllers | MPPI transit, RPP coverage, RPP reverse, docking controller; MPPI unaware of the wheel floor | One tracker for all paths, MPPI replaces pure pursuit (made aware of the wheel floor, tuned in the sim gate), RPP only as a revert param; explicit pivot / reverse steps; docking |
| Turn shaper | Rewrites commands (widen, pivot-instead, pivot snap), so the controller's model is wrong | A clamp only; nothing to rewrite because the curvature is drivable |
| Mission | Grows a rule per failure (cap, corner reached, stall, stub trim, progress watches) | Sends conditioned legs; keeps the safety rules; removes the corner rules |
| Validation | Unit tests, partial sim runs, then field | Full-mow sim gate with fixed pass criteria, then one field run |
| Outer ring | 0.05 m inset with sharp corners, 0.11–0.16 m tracking error at corners | Inset plus fillet / pivot per corner, validated inside the margin |

## 6. Steps

Each step passes the sim gate before the field.

1. **Sim gate** (in progress, `ws/coverage-corners`). Baseline the current master: it must
   reproduce the run 5 ring trip.
2. **Path conditioner** (pure geometry plus unit tests on the real map's rings and drawn
   paths), and a GUI/debug topic that shows the conditioned path.
3. **Use it for coverage sub-paths and transit / dock routes**, with the existing
   controllers. Run the gate, then a field run.
4. **MPPI as the single tracker** (§3.2 work package): feasibility-aware MPPI, tuned in the
   sim gate against RPP on conditioned paths; adopt it for transit and coverage if it passes.
   Gate, then field.
5. **Remove the superseded runtime rules** (§3.3) one at a time. Gate after each; field
   after the set.
6. **Phase 4: slope-aware turn choice** (pivot vs K-turn by tilt), with field data.

Revert flag: `path_conditioning: false` sends raw paths as today.
