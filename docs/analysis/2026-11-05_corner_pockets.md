# Corner pockets inside the inner lap (2026-11-05)

Field report (soft layout, deployed b4437d0): "the third perimeter mow isn't great on that
corner ... a small V to cover the gap". The mowed raster of the live area 3 ("further
backyard lawn", the area the sim copies as owner_far Area 6) at 59 % cut shows an uncut
triangle of 0.275 m2 in the west lobe, 0.87 m in from the line, inside the narrow part where
the laps go round the tip.

## How the gaps were found

`planCoverage` alone cannot show these gaps. Its model is a base_link-centred 0.18 m strip
with sharp lap corners. The mission's conditioner changes the lap corners (fillets, K-turns,
corner cuts), and the blade runs 0.23 m ahead of base_link. So each area went through an
offline copy of the whole chain, with the mission's default area settings:

1. The planner: the node's `planCoverage` call and path assembly (soft, 3 laps, racetrack,
   r 0.5, op_width 0.18).
2. The bridge splitter, densified at 0.10 m.
3. `condition_plan` with the mission.yaml values. The outline goes through
   `clean_area_ring`, and the region is all the areas.
4. An idealised drive: every pose followed, pivots swept in place, blade off on reverse legs.
5. The blade disc stamped into `MowedRaster` (0.22 m at 0.23 m ahead), then
   `missed_patches` (the part the blade fits into, at least 0.01 m2).

Check against the sim gate ground truth on master: owner_far Area 6 gives 0.635 m2 at
(-32.85, -6.68), where ground truth has 0.633 m2 at (-32.85, -6.65). real_run5 gives
0.063 m2 at (-4.33, -0.57), where ground truth has 0.063 m2 at (-4.28, -0.53).

## Gap table (offline, master b4437d0 vs branch)

Cut % of the area in the idealised drive. Patches are blade-fit uncut pieces of at least
0.01 m2, as area @ (x, y).

| Area | master | branch | spurs / extra pass length |
|---|---|---|---|
| owner_far Area 6 (sim copy of area 3) | 98.95 %; 0.635 @ (-32.85, -6.68), west lobe | 99.68 %; none | 4 / +7.2 m |
| live area 3 further backyard lawn | 99.56 %; none (a 1.5 m fill pass crosses the lobe, driven last) | 99.75 %; none | 3 / +3.8 m |
| live heart | 97.33 %; 0.133 @ (-17.93, -6.68) | 99.46 %; none | 1 / +3.4 m |
| real_run5 Area 1 (= owner_far Area 1) | 95.94 %; 0.063 @ (-4.33, -0.57), SW lobe | 96.45 %; 0.063, same | 1 / +1.2 m |
| owner_far lawn section | 99.23 %; 0.140 @ (-13.53, -10.03), 0.128 @ (-24.52, -2.49) | unchanged | 0 |
| live closer backyard lawn | 99.38 %; 0.273 @ (-21.87, -13.44), south lobe tip | unchanged | 0 |
| live Clothesline (two obstacles) | 91.16 %; 0.675, 0.165, 0.118, 0.100, 0.080 | unchanged | 0 |
| live Area 4 (strip about 1 m wide) | 41.77 %; 1.873, 0.063 | unchanged | 0 |
| live Area 5 (obstacle near the line) | 63.35 %; 1.263, 0.630, 0.180, 0.160, 0.115 | unchanged | 0 |

The pockets come from three different causes:

1. **Pocket inside lap 2 that no row or fill reaches** (fixed). This is the owner's case.
   - In the owner_far west lobe the lobe is narrower than the rows need. Rows under 1.2 m are
     dropped (round 4), and fill passes must be at least 1.0 m long and add 35 % new area
     (round 3-4).
   - The fill search tried a few offsets and picked the best by gain before applying those
     limits. When that best pass failed the limits, the search gave up on the pocket, even
     where another pass would have fitted.
   - In the live polygon, a fill pass happens to cross the lobe. Fill passes are driven after
     every row, so at 59 % the mower had not reached it yet.
2. **Inner laps dropped in narrow features** (not changed).
   - Laps 1 and 2 are opened by `lap_corner_radius` 0.7. Where a feature is narrower than
     about 2 x (offset + 0.7), the lap is not drawn there.
   - The real_run5 SW lobe and the Area 5 north part have only lap 0. Area 4, a strip about
     1 m wide, has lap 0 and no rows at all (the 0.45 row region is degenerate), so it plans
     31 % and cuts 42 %.
   - The row region does not reach these places, so a spur cannot either.
3. **Acute lobe tips** (not changed). An example is the closer backyard south lobe. Lap 1
   runs to a 40 deg apex, and the conditioner rounds or cuts it, because a pivot there sweeps
   0.644 - 0.15 = 0.49 m past the line. The row region (0.45 in) ends about 1 m short of the
   apex.

PNGs (master | branch side by side) were made for the owner. They are not in the repo.

## Sim gate (ground truth)

| Run | a | e | h p99 line / arc | i side / nose / sweep | gt cut | MOWING s | b / STUCK |
|---|---|---|---|---|---|---|---|
| real_run5 -l -x 3 -s 1, master | 0.057 | 0.097 | PASS | FAIL 0.112 | 96.4 % | 740 | 0 / 0 |
| real_run5 -l -x 3 -s 1, branch | 0.089 | 0.091 | 0.085 / 0.050 | FAIL 0.086 / 0.055 / 0.119 | 97.0 % | 777 | 0 / 0 |
| owner_far -l -x 3 -s 1, master | 0.080 | 0.132 | 0.068 / 0.065 | FAIL 0.128 / 0.056 / 0.091 | 97.0 % | 3122 | 0 / 0 |
| owner_far, branch (first spur order) | FAIL 0.105 | FAIL 0.176 | FAIL 0.067 / 0.106 | FAIL 0.190 / 0.049 / 0.079 | 99.2 % | 3245 | 0 / 0 |
| owner_far, branch (chained spurs) | 0.079 | FAIL 0.181 | 0.068 / 0.048 | FAIL 0.096 / 0.119 / 0.153 | 99.2 % | 3238 | 0 / 0 |
| real_run5 -d -s 1 -x 1, master (brief) | 0.094-0.128 | 0.104 | flips | FAIL | | | |
| real_run5 -d -s 1 -x 1, branch | 0.087 | FAIL 0.155 (TRANSIT route corner (-3.00, 0.89), not a spur) | FAIL 0.095 / 0.108 | FAIL 0.150 / 0.067 / 0.087 | 96.0 % | 788 | 0 / 0 |

The ground-truth west lobe patch (0.633 m2 on master) is gone. No patch of at least
0.01 m2 is left in owner_far or in real_run5 (-l runs).

- **First branch run.** Its (a), (h) and the (i) side value came from sub-paths 0-5. Those
  were identical to master's (to 1 mm): the fill passes are appended after them. Re-evaluated
  over sub-paths 0-5 only, that run gives a 0.105, h arc 0.108 and i side 0.190, so those
  values are run-to-run drift noise. Its (e) 0.176 was at a spur mouth, which led to the
  chaining.
- **Chained run.** Its (e) 0.181 is at sub-path 2 idx 648 (-31.44, -11.26). That sub-path
  is the same as on master, where the corner had 0.125.
- **Spur tracking.** Over the spur sub-paths only, line cross-track was p90 0.17, max 0.31
  (first run): the short legs between about-turns do not converge, as round 4 found for short
  rows. The body stays in: in the lobe, nose / sweep over plan 0.12 / 0.15 and point outside
  0. The pass cuts anyway because the blade sweeps the about-turns.

## What stock does

The stock firmware plans on a 0.05 m raster with erosions of 0.15, 0.30 and 0.45. Its laps
run at 0, 0.15 and 0.30, and its rows fill the 0.45-eroded raster. TurningMode covers
in-place, arc, inner/outer fish-tail, slope and adaptive. Those are row-to-row U-turns. The
decompile shows no corner or edge-lap logic and no minimum row length; a blocked turn or
path end is skipped (`2026-10-21_stock_firmware_findings.md` "Edges, turns and coverage").
Stock rows therefore reach into any part of the 0.45 raster, narrow lobes included. Our
1.2 m minimum row (round 4) removed that.

## Technique

Three candidates:

- **Square the lap-2 corner with a pivot.** This is not possible within the soft limits. A
  pivot on lap k sweeps the body corners (radius 0.644) 0.644 - offset past the line: 0.34
  on lap 2 and 0.49 on lap 1, against a planning limit of 0.26. That is why the conditioner
  fillets those corners.
- **Fish-tail or three-point turn at the lap corner.** This already exists: the K-turn for
  corners over the limit. It keeps the lap on its line, but the pocket is inside the lap, so
  the K-turn does not reach it.
- **In-and-out spur (keyhole / "V"), chosen.** The spur is a straight pass along a line of the
  row region into the pocket. The robot drives forward to its end, pivots about there and
  drives back out on the same line.
  - Base_link stays in the row region, at least 0.45 from the line: the row-end rule.
  - Driving in, the nose is at most 0.15 past the line. Through the about-turn the body
    corners are at most 0.644 - 0.45 = 0.194 past, under the 0.26 planning limit and the 0.35
    side limit. Rows already do this at every row end.
  - Two spurs into one tip draw the "V" (owner_far: two spurs meet at (-33.26, -6.37)).
  - A reverse-out version (no pivot, so base_link could go deeper) is not used. The mission's
    `join_turn_chains` replaces planned reverse chains with a forward connector plus pivot, so
    the reverse leg would not survive as planned.

Implementation (`coverage_planning.cpp` `fillGaps`, edge layout only):

- **Candidate limits.** `bestFillPass` now takes the minimum length and efficiency and
  returns the best candidate among those that meet them.
- **Spur trigger.** A spur is planned for a gap only when no regular fill pass of at least
  1.0 m fits it.
- **Spur size.** The gap's blade-fit part (opening by half a swath) must be at least
  0.03 m2. The spur must be at least 0.5 m long and add 35 % of its strip.
- **Direction.** The spur runs towards the end nearer the line.
- **Path layout.** It is emitted as two swaths: the in leg, then the out leg starting 0.02 m
  short of the in leg's end, so that the bridge keeps two poses per swath and splits the plan
  structurally.
- **Drive order.** Spurs join the drive order like fill passes, entered at the mouth and
  never reversed. When the next spur's mouth is within 0.6 m and the line to it stays in the
  row region, the out leg runs straight to that mouth. That is one about-turn instead of
  pivot / short straight / pivot; the first owner_far gate failed (e) 0.176 m at such a
  0.18 m connector. The conditioner turns the cusp into a planned pivot. The drops line reads
  `gap fill: N pass(es) (K pocket spur(s))`.
- **No new parameters.** The limits are planner constants.

Like every fill pass, spurs are driven after the rows, at the end of the area.

## Seams in the field raster

The thin diagonal seams between rows in the live raster are mostly real at the raster's
resolution, not a sampling artefact:

- Rows are 0.18 apart with a 0.22 disc, so the nominal overlap is 0.04.
- The raster places the disc edge within 2.5 cm, and stamping is continuous between 10 Hz
  ticks, so it leaves no tick gaps.
- A cell stays uncut only where two neighbouring passes' disc edges are more than about
  0.18-0.2 m apart. That takes a combined lateral error of about 2-4 cm, about the sim's
  cross-track p90 of 0.032.
- Long continuous seams follow rows that were driven far apart in time: neighbouring
  racetrack blocks, a fill against a row. The localisation offset changes between those
  times; in the sim the `-l` drift reproduces the same seams.
- Isolated cells along rows come from the 2.5 cm quantisation at the 4 cm overlap margin.
  The physical uncut strip there is at most a few centimetres.
