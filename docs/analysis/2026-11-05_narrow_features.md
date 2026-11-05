# Narrow features: inner laps and rows dropped (2026-11-05)

Follow-up to `2026-11-05_corner_pockets.md`. In features narrower than about 2 m the soft edge
layout keeps lap 0 only, and a strip narrower than 0.9 m gets no rows. Seen offline on the
real_run5 SW lobe, the north and the sides of live Area 5 (63 % cut), and live Area 4 (42 %).

The live snapshot's `areas.dat` holds only names, outlines, obstacles and path channels. It has
no per-area mow settings (mow angle, edge laps). Every run below uses the mission defaults:
soft layout, 3 laps, auto angle.

## Where the laps and rows go

Soft layout: lap 0 on the line, laps 1 and 2 at 0.15 and 0.30, rows from 0.45 with row ends
0.45 in. The blade (0.22 m at 0.23 m ahead) cuts 0-0.11 from lap 0, 0.04-0.26 from lap 1,
0.19-0.41 from lap 2 and 0.34 inwards from the outermost row.

1. **Laps 1 and 2: the `lap_corner_radius` opening.** `planCoverage` builds each inner lap as
   the field inset by `o + 0.7`, grown back by 0.7 and clipped to the plain inset `o`. F2C
   buffers with mitres, so this does not round corners. It removes every part narrower than
   `2 (o + 0.7)`: 1.7 m for lap 1, 2.0 m for lap 2. When a lap comes out empty the loop stops,
   so lap 2 is never tried. That step alone drops the inner laps. The conditioner and the fill
   limits play no part.
2. **Rows in Area 4: the 0.45 erosion.** Area 4 is narrower than it looks: its largest
   inscribed circle has a radius of 0.35 m, so it is at most 0.70 m wide. The 0.45 erosion is
   empty, the planner logs "drivable region degenerate", and there are no rows. `fillGaps`
   returns early when the row region is empty, so nothing fills it either. The 1.2 m row
   minimum and the 1.0 m / 35 % fill limits are not involved.
3. **Lobes with a row region: the row region's edge.** In the real_run5 SW lobe and in Area 5
   the row region does reach in. Rows and fill passes keep base_link at least 0.45 in, so they
   cut from 0.34 inwards, and the band from 0.11 to 0.34 along the sides is left with no lap
   over it. The row minimum (three rows under 1.2 m dropped in real_run5) and the fill limits
   only matter inside the row region.

Stock fills the 0.45-eroded raster with rows of any length and runs its laps on the 0.15 and
0.30 erosions as rings, turning wherever a ring ends. Its body check runs near the axle on edge
laps only (`2026-10-21_stock_firmware_findings.md`). In Area 4 stock would also have no rows.
It would drive the inner rings and pivot at their ends.

## Why the opening cannot simply shrink

Offline runs with the opening radius changed (the same chain as the corner pockets note, with
the conditioner's warnings compared against master):

| Opening | Area 4 cut | Area 5 cut | real_run5 cut | new corners over the 0.26 limit |
|---|---|---|---|---|
| 0.7 (master) | 41.8 % | 63.4 % | 96.5 % | none |
| 0 (all inner laps) | 98.0 % | 82.1 % | 98.6 % | Area 4: a -53 deg pivot 0.47 out (over the 0.35 cap; under `kturn_min_rad`, so no K-turn or cut), K-turns 0.34, a pivot 0.32. Synthetic strips 0.5-1.5 m wide: lap 1 hairpin corners 0.49, lap 2 0.34 |
| lap 2 only at 0.5 | 41.8 % | 72.8 % | 96.4 % | Area 5: a -68 deg pivot 0.29 |

- An inner lap's corner either takes a fillet or a pivot at a vertex 0.15 or 0.30 from two
  lines. A pivot there sweeps the front corners (0.644 from base_link) 0.49 or 0.34 past the
  line.
- A fillet of radius r on lap `o` puts the outer front corner `hypot(r + 0.235, 0.6) - r - o`
  past the line. For lap 1 that is under 0.26 only from r 0.72, which is the existing 0.7. A
  hairpin end needs two such fillets, so it needs a feature at least `2 (o + r)` wide. That is
  what the opening keeps.
- The real lobes passed at radius 0 only because their tips happen to suit the conditioner.
  The opening stays as it is.

A centre-line pass along a strip too narrow for rows does not fit the rules either. Area 4's
centre is 0.35 from the line, and an about-turn there sweeps the body 0.644 - 0.35 = 0.29 past
it. That is over the planning limit and well over the 0.194 of a row-end pivot.

## Change: lap-band spurs (`mower_coverage` `fillGaps`)

A third tier after fill passes and pocket spurs. Where neither fits a gap, an in-and-out spur may
run into the inner laps' band:

- **Mouth in the row region.** The robot enters the spur where a row end could be: base_link
  at least 0.45 in. A transit arrival pivot there sweeps the body at most 0.194 past.
- **Spur line in the band.** The whole spur keeps base_link at least the first inner lap offset
  (0.15) in. Obstacles keep the rows' distance (0.40 + 0.45). Driving along it, the sides are at
  most 0.085 past the line.
- **End where the about-turn fits.** The end is pulled back in 5 cm steps until a 180 deg pivot
  there, one way round, keeps the body outline (corners and edge midpoints, as the conditioner
  samples it) within `hypot(0.60, 0.235) - row_end_inset` = 0.194 of the line. That is a
  row-end pivot's sweep. The nose swings across the feature, so the far side must be at least
  0.45 away. The about-turn's first sample is the arrival pose, so the nose at the end is
  covered too. Hard layouts get no lap-band spurs, because that limit is negative there.
- **Candidates.**
  - The inner lap lines (0.15 and 0.30) parallel to each edge (at least 0.3 m long) next to
    the gap.
  - Lines over the gap every half swath, at the row angle, its normal, the gap's long axis and
    12 deg to those edges, for a shallow entry from the row region.
  - Each line is clipped to the band. The mouth is the nearest row-region point back from its
    end, at least 0.5 m back.
  - The best by new area wins. A spur must be at least 0.5 m long and add 35 % of its strip,
    as for pocket spurs.
- Emission, drive order and the drops line are the pocket spurs' own. The drops line reads
  `gap fill: N pass(es) (K pocket spur(s), L along a lap band)`.
- **No new parameters.** The limit comes from the existing body dimensions and
  `row_end_inset`.

**Conditioner: the cusp's way round.**
- The conditioner took a vertex pivot's overhang with the sign of the turn. At a 180 deg cusp
  that sign is float noise, and the wrong way round reads 0.3-0.46 at a lap-band spur end.
- At run time the mission pivots the way whose sweep stays inside best (`_pivot_way`).
- Within 5 deg of 180 the conditioner now takes the better way's overhang. Without that it
  would log those ends over the limit, and could K-turn them or cut them over the 0.35 cap.

## Gap table (offline, master 37589d9 vs branch)

Idealised drive of the conditioned plan, blade raster. Missed = blade-fit uncut pieces of at
least 0.01 m2. Extra drive = conditioned path length, branch minus master.

| Area | master cut / missed | branch cut / missed | spurs along a lap band | extra drive |
|---|---|---|---|---|
| live Area 4 (= owner_far Area 4) | 41.8 % / 1.935 m2 | unchanged | 0 | 0 |
| live Area 5 (= owner_far Area 5) | 63.4 % / 2.348 m2 | 74.6 % / 2.168 m2 | 6 (and 6 pocket spurs; master had no fill) | +26.4 m |
| real_run5 Area 1 (SW lobe) | 96.5 % / 0.063 m2 | 96.8 % / 0.063 m2 | 2 | +2.2 m |
| owner_far Area 6 | 99.66 % / 0 | 99.68 % / 0 | 1 | +1.2 m |
| owner_far lawn section | 99.23 % / 0.268 m2 | unchanged | 0 | 0 |
| owner_far heart | 99.42 % / 0 | unchanged | 0 | 0 |
| live closer backyard lawn | 99.38 % / 0.273 m2 | 99.72 % / 0 | 3 | +14.0 m |
| live Clothesline | 91.16 % / 1.138 m2 | 91.96 % / 1.058 m2 | 2 | +3.7 m |
| live further backyard lawn, live heart | 99.75 %, 99.42 % | unchanged | 0 | 0 |

Synthetic shapes (no new over-limit corners on any):

| Shape | master | branch | extra drive |
|---|---|---|---|
| strip 8 m x 0.6 m | 35.9 % | unchanged | 0 |
| strips 8 m x 0.9 / 1.0 / 1.2 / 1.5 / 1.8 m | 44.8 / 54.7 / 54.4 / 67.6 / 80.4 % | 89.6 / 96.4 / 96.0 / 95.0 / 97.3 % | +55 to +70 m |
| lobes 0.9 / 1.2 / 1.6 m wide off a 6 m square | 92.2 / 93.1 / 93.4 % | 97.0 / 96.9 / 97.0 % | +25 to +30 m |
| 0.6 m lobe | 94.4 % | unchanged | 0 |
| long wedge 8 m x 1.6 m | 52.5 % | 90.0 % | +37 m |

The straight strips are reached by the 12 deg entries, from the thin row region down the
middle. That is many short spurs, each with an about-turn, so the extra drive is large: about
8 m of spur per metre of strip. The 0.6 m strip and lobe have no row region and stay as they
were.

Safety offline:
- No corner the conditioner logs over the planning limit is new, on any area.
- The inner-pose body maximum against the all-areas region is unchanged where a spur was
  added. The idealised drive pivots a cusp the better way round, as the mission does.
- Planner time on the closer backyard lawn: 0.44 to 0.51 s.

What is left in Area 5:
- 1.263 m2 between the obstacle and the north-west line. The obstacle sits within 0.40 of
  the line, so `holeKeepoutRegion` merges it with the outside and lap 0 detours around it. That
  is the obstacle rule, not this one.
- 0.630 m2 in the narrow north-east lobe. It has no row region and so no mouth.

## Sim gate (-l -x 3 -s 1)

Baselines are the corner-pocket branch runs, plan-identical to master 37589d9
(`simout_corner1105/branch-l3`, `branch-of-l3b`). The controller loop held a median and p10 of
20 Hz in every run below. The count of "missed its desired rate of 60" warnings is 88 against
28 for real_run5, and 144 against 147 for owner_far.

| Run | a | e | h p99 line / arc | i side / nose / sweep | gt lawn cut | MOWING s | b / STUCK |
|---|---|---|---|---|---|---|---|
| real_run5, baseline | 0.089 | 0.091 | 0.085 / 0.050 | FAIL 0.086 / 0.055 / 0.119 | 97.0 % | 777 | 0 / 0 |
| real_run5, branch | 0.082 | 0.112 | 0.076 / 0.061 | PASS 0.071 / 0.057 / 0.095 | 97.4 % | 793 | 0 / 0 |
| owner_far, baseline | 0.079 | FAIL 0.181 | 0.068 / 0.048 | FAIL 0.096 / 0.119 / 0.153 | 99.2 % | 3238 | 0 / 0 |
| owner_far, branch | 0.076 | PASS 0.144 | 0.065 / 0.055 | PASS 0.088 / 0.050 / 0.075 | 99.3 % | 3261 | 0 / 0 |
| owner_far Area 5 only, branch (no baseline) | 0.080 | 0.096 | FAIL 0.102 / 0.079 | FAIL 0.157 / 0.059 / 0.148 | 73.0 % | 563 | 0 / 0 |

Both baseline worlds pass every criterion on the branch. The (e) and (i) values that flip run
to run on master came out under their limits this time. real_run5 drives two lap-band spurs and
owner_far one, in Area 6. owner_far mows Area 6 only; its other areas are selected for transit.

None of the baseline worlds mows Area 5, which carries the most spurs: 6 lap-band and 6 pocket
spurs. A third run used a world with only owner_far Area 5, starting inside it: there is no
route from the dock with only the path channels selected, so (c) fails by construction
(final IDLE, not docked).
- Its ground truth cuts 73.0 %, where the offline chain gives 74.6 % (master offline 63.4 %).
- (i) side 0.157 over plan is on the west lap-band spur's in leg at (-9.59, -0.59), planned
  0.0, heading -85 deg. The 1.3 m leg between two about-turns does not converge. The corner
  pockets note saw the same on pocket spurs: line cross-track p90 0.17 there.
- The absolute side value there stays under 0.26. The run's worst absolute side, 0.428, is on
  lap 0 at the south end.
- (h) line p99 is 0.102. There is no master run of this world to compare with.

The world file is not committed: it starts the robot undocked and cannot return home.

## Still open

- **Area 4.** It is at most 0.70 m wide, so no point is 0.45 from the line and there can be
  no mouth. No pivot inside it stays within the row-end sweep. Only lap 0 can run there within
  the rules, at 42 %. The options are the owner's:
  - accept lap 0 only;
  - redraw or merge the strip;
  - allow inner-lap rings with K-turn ends. That needs conditioner work: today its K-turn
    search does not find the feasible double K-turn at a 0.35 m-wide lap 1 hairpin.
- **Drive cost in long straight strips.** The 12 deg entries reach a straight side's band one
  short spur at a time. A dog-leg spur would cut that cost: a shallow entry, then along the lap
  line, about-turn, and back.
- **Lobe ends** (the band across the end of a lobe). No spur can turn about there.
- **Obstacles within 0.40 of the line** (Area 5 north-west). Their pocket is outside the
  keep-out region by design.
