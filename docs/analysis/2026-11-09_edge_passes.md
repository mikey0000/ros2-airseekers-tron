# Edge passes: one pass along an edge instead of a chain of fills (2026-11-09)

Follow-up to `2026-11-05_corner_pockets.md` and `2026-11-05_narrow_features.md`. Field mow
2026-11-08, live "closer backyard lawn": the rows run at 105 deg, almost along the long west
edge, and the edge was finished by a chain of short passes and "V"s. The owner is not worried
about coverage there, but fewer turns, a shorter drive and a cleaner look are worth having.

## What the chain is

The 2026-11-08 plan dump and the offline chain (planner, bridge splitter, mission conditioning,
idealised drive, blade raster; method in the corner pockets note) give the same fill passes for
this area. On master 6253c79:

- The recorded west line is not straight. It has three bends of 10-20 deg over its length, and
  the rows meet each straight stretch at 0-17 deg.
- Each stretch leaves one long thin pocket between the row ends (0.45 in) and lap 2's strip
  (to 0.39 in). The pockets are 0.39-0.54 m in and narrower than a swath. They have no
  blade-fit part (opening by half a swath).
- Each pocket gets its own fill pass: four passes of 1.2-3.6 m at about the rows' angle,
  0.4-0.8 m apart end to end, each driven separately with a dog-leg or transit between.
- The in-and-out spurs (the V's) of this area are in the south lobe, not along the edge.

Elsewhere, runs of spurs along one edge exist in live Area 5 (lap-band spurs at 12 deg into
the bands along both long sides) and in the real_run5 east corner (a fill pass and two spurs).

## Change (`mower_coverage` `fillGaps`, edge layout only)

`mergeEdgeRuns` runs on each fill round's passes before they are ordered:

1. **Runs.** Fill passes and spurs whose pockets lie within 1.0 m of the line are grouped when
   neighbours are within 20 deg of each other (undirected), their nearest ends at most 1.0 m
   apart, and at most 0.5 m apart side by side. Each group is sorted along its mean direction.
   A run is a stretch of the group with every consecutive pair such neighbours.
2. **Candidate.** The row region (the target inset by `row_end_inset`) is inset by 0.01, 0.03
   and 0.05 m. On each inset ring the candidate is the stretch that passes within 0.1, 0.5 or 1
   half swaths of the run's pockets. It is simplified by 1 cm, and corner arcs shorter than
   0.1 m are merged. The stretch is straight where the line is straight and bends where it
   bends. It must be at least 1.0 m long (the fill-pass minimum) and lie inside the row
   region, and every leg must be within 20 deg of the run's direction, so it never wraps round
   a corner.
3. **Acceptance.** The pockets are the run's pockets, less what their other passes cover. A
   candidate replaces the run only when all of these hold:
   - it leaves no more of the pockets uncovered than the run does (to 1 cm2);
   - it leaves no more of their blade-fit part uncovered;
   - it is no longer than the run's drive. The run's drive counts its passes (spurs twice) and
     the gaps between them. Length past that is allowed only where it covers area the run
     left, at a full swath's width.

   Among the candidates that pass, the one that leaves the least uncovered wins, the shorter
   on a tie. Longer runs are tried first.
4. **Emission.** The pass is an ordinary fill pass, driven either way. Each leg is a swath.
   Each leg after the first starts 0.02 m past its corner, as a spur's out leg does, so the
   bridge keeps two poses per swath. The drops line reads `gap fill: N pass(es) (K pocket
   spur(s), L along a lap band, E along an edge)`.

No new parameters (A6). The limits are planner constants.

**Safety.**
- Base_link stays in the row region, at least `row_end_inset` (0.45) in, as at every row end
  and fill pass.
- The body heading along it is within the rows' limits.
- A pivot anywhere on it, at a bend or at either end, sweeps the body at most
  hypot(0.60, 0.235) - 0.45 = 0.194 past the line, a row end's sweep.
- It is entered and left like any fill pass: from the transit, at either end, inside the row
  region.
- No corner the conditioner logs over the limit is new on any area below. The inner-pose
  body maximum is unchanged.

## What was tried and dropped

- **A strictly straight pass parallel to the edge** (the line's length-weighted direction near
  the pockets, +-2 and +-4 deg, every 2 cm across, clipped to the row region). On the closer
  backyard west edge the bends separate the pockets. A straight line covers one stretch's
  pocket and misses the next. For two to four of those pockets, the best straight pass leaves
  0.11-0.56 m2 more uncovered than their passes do. The ring stretch covers three of them
  (below).
- **Accepting on blade-fit area alone.** The pockets have no blade-fit part, so this test
  accepts almost any pass. On the closer backyard lawn it replaced four passes with a 2.2 m
  pass over one pocket. The next fill round re-planned the other three, for +7.8 m of drive.
  In Area 5 (with the band spur below) the raster fell from 74.63 % to 72.95 %. Hence the
  area test as well.
- **An edge spur along the lap band** (lap-band spur rules, with a 12 deg dog-leg entry where
  the band line never reaches the row region), for Area 5's V's. The bands there are wider
  than one swath. The best single spur left 0.07-0.20 m2 more than the 12 deg V's it would
  replace, and in no run did it pass the area test. It was removed.

## Offline table (master 6253c79 vs branch)

Idealised drive of the conditioned plan, blade raster (0.22 m disc 0.23 m ahead). Spurs are
the planner's in-and-out spurs. Fill stretches are continuous one-way pieces of the fill
passes; a spur counts two. About-turns are heading reversals of 150 deg or more at one spot
in the conditioned path, row ends included. Missed = blade-fit uncut patches of at least
0.01 m2.

| Area | spurs (V) | fill stretches | edge passes | about-turns | extra drive | missed blade-fit | cut |
|---|---|---|---|---|---|---|---|
| live closer backyard lawn | 3 / 3 | 11 / 9 | 1 | 70 / 70 | +0.0 m | 0 / 0 | 99.72 / 99.72 % |
| live further backyard lawn | 3 / 3 | 13 / 13 | 0 | 54 / 54 | 0 | 0 / 0 | 99.75 / 99.75 % |
| live Area 5 (= owner_far Area 5) | 6 / 6 | 12 / 12 | 0 | 17 / 17 | 0 | 2.168 / 2.168 m2 | 74.63 / 74.63 % |
| live Clothesline | 2 / 2 | 8 / 8 | 0 | 26 / 26 | 0 | 1.058 / 1.058 m2 | 91.96 / 91.96 % |
| real_run5 Area 1 (= owner_far Area 1) | 3 / 3 | 8 / 8 | 0 | 18 / 18 | 0 | 0.063 / 0.063 m2 | 96.84 / 96.84 % |
| owner_far Area 6 | 5 / 5 | 13 / 13 | 0 | 54 / 54 | 0 | 0 / 0 | 99.68 / 99.68 % |
| owner_far lawn section | 0 / 0 | 3 / 3 | 0 | 49 / 49 | 0 | 0.268 / 0.268 m2 | 99.23 / 99.23 % |
| owner_far heart | 1 / 1 | 6 / 6 | 0 | 20 / 20 | 0 | 0 / 0 | 99.42 / 99.42 % |
| owner_far Area 4 | 0 / 0 | 0 / 0 | 0 | 0 / 0 | 0 | 1.935 / 1.935 m2 | 41.71 / 41.71 % |
| synthetic: edge 3 deg off the rows | 0 / 0 | 3 / 2 | 1 | 43 / 43 | +0.2 m | 0.445 / 0.445 m2 | 98.62 / 98.62 % |
| synthetic: bent edge 5 / 15 / 5 deg off the rows | 0 / 0 | 5 / 2 | 1 | 54 / 54 | -1.3 m | 0.365 / 0.365 m2 | 98.36 / 98.35 % |

- **Closer backyard lawn.** Three of the four west-edge passes become one 8.1 m pass along
  the row region's edge (7 legs, bends under 20 deg). It leaves 0.003 m2 of those pockets
  against the passes' 0.042 m2. The north pass stays separate: the ring there turns round the
  north corner. The conditioned drive and the raster are unchanged to 0.1 m and 0.01 %.
- **Synthetic bent edge.** Four separate passes become one along the edge, and 3 fewer
  sub-paths. The raster changes by 0.01 %.
- **No change in every other area.** Their planner and conditioned output is identical to
  master's. Their runs fail the area test:
  - Area 5's 12 deg V's cover bands wider than a swath;
  - the further / owner_far Area 6 west lobe spurs fan into a tip, not along an edge;
  - in the real_run5 east corner, one pass would leave 0.03 m2 of blade-fit area that the
    spur covers.

The PNGs (master | branch) are not in the repo.

Planner time on the closer backyard lawn: 0.46 to 0.48 s.

## Tests

- `EdgeLayoutEdgePasses.ShallowEdgeGetsOneStraightPass`: an 8 x 10 m area whose west edge is
  3 deg off rows at 90 deg. One straight pass parallel to the edge, against two on master.
- `EdgeLayoutEdgePasses.BentEdgeGetsOnePassAlongIt`: the closer backyard edge's shape, three
  stretches 5 / 15 / 5 deg off rows at 105 deg. One pass along the edge, against four on
  master.
- Both fail on master. Every 5 cm along the pass, both check that base_link is at least 0.45
  in and the body within 0.26. They also check that plan coverage is not lower.

## Sim gate

Not run. The sim worlds' areas (real_run5 Area 1; owner_far Area 1, Area 4, Area 5, Area 6,
lawn section and heart) plan identically on the branch and on master, swaths, laps and
conditioned sub-paths alike. A gate run would repeat master's.

## Still open

- **Runs the area test rejects.** Area 5's bands would need two passes along each side, or a
  dog-leg spur that runs along the band and comes back on a second line.
- **The north end of the closer backyard west edge.** The row region's ring turns round the
  north corner there, so the pass stops short of the last pocket.
