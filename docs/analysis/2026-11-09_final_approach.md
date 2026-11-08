# Final approach onto an on-line start (2026-11-09, ws/approach1109)

The boundary smoothing gate (`2026-11-09_boundary_smoothing.md`, ws/smooth1109) failed b
(one hard-boundary trip) on `real_run5_smooth` in both seeds. That trip is mission
transit code and has nothing to do with smoothing. This note covers the cause and the fix.

## Reproduction

Master code (6253c79) with the world copied from ws/smooth1109
(`src/mower_sim/worlds/real_run5_smooth.yaml` and its areas file), `-d -x 3 -s 2`:

```
transit ended 0.35 m from the sub-path start: final approach onto it (turn 124 deg, then straight)
area 0 sub-path 0: leg start at (-1.96, 4.88): pivot -177 deg in place, blade on, then follow
BOUNDARY OUT: body nose 0.46 m outside (soft limit 0.43) at (-2.01, 4.92) in MOWING
BOUNDARY OUT: body rear_left 0.45 m outside (soft limit 0.43) at (-2.28, 5.13) ... (re-trip)
```

b = 2 (the smooth1109 runs had 1), a 0.357 m / 218 samples, e 1 of 19, h line p99 0.319.

## Mechanism

Pose trace of the smooth1109 seed 2 run (localised pose against Area 1's top edge, which
runs at 201.5 deg, interior to the south):

1. Lap 0 starts on the top edge at (-1.96, 4.88). Its first leg runs east along the line
   (21.5 deg).
2. **The transit target is the start itself, on the line.** `_transit_approach` should
   have moved it 0.30 m inside (`edge_approach_inset_m`; its purpose is to stop transits
   arriving square to the line with the nose 0.40 m out). It fell back to the start
   because it took the spot for a corner. The inset point (-1.85, 4.60) is 0.11 m from an
   edge, but that edge belongs to the drawn paths Path 2 / Path 3, which cross Area 1's
   top edge 0.15 m east of the start. Inside Area 1 those edges are not boundary at all.
   `bbody.nearest_edge_point` measures to every polygon's edges, not to the union's
   boundary.
3. The route's last leg runs square onto the line (heading about 112 deg). The tracker
   cuts the bend before it, keeps heading 72-76 deg and ends on the line 0.21 m (seed 2) /
   0.13 m (seed 1) / 0.35 m (master rerun) past the start along the lap. That is within
   `route_end_tolerance_m` 0.5, so the transit counts as arrived. The robot ended past
   the start, not short of it: it sits on the line, east of the start.
4. `_route_final_approach` turns to face the start (+79 to +124 deg) and drives straight
   onto it, i.e. west along the line, **against the lap heading**. The spin under-runs
   (pivot efficiency 0.85 in `-d`) and the base drifts, so the hop runs about 20 deg
   outward of the line. base_link ends 0.13-0.15 m outside with the nose about 0.38 m out.
5. The leg-start pivot then has to turn 156-177 deg on the line. `_pivot_way` takes the
   short way (clockwise, through north). That way looks clear because the drawn path
   strip lies north of the robot. Its first degrees swing the nose further out, west of the
   strip: 0.44-0.46 m against the 0.43 limit, 0.06-0.17 s into the Spin.

The body is 0.60 m ahead of base_link and 0.10 m behind it. Any nose-first approach onto
a point on the line at angle θ puts the nose 0.60·sin θ out. A forward hop against the lap
also leaves a near about-turn on the line, the worst place for one.

Why only on the smoothed map: smoothing moved lap 0's start from the west edge to the top
edge beside the drawn paths. The edge itself moved 4 cm. Any recorded map with a lap
start on the line beside a drawn path crossing it hits step 2.

## Fix (5b9a213, 8f112e7)

1. `body_boundary.nearest_boundary_point(x, y, polys)`: the nearest point on the boundary
   of the union. An edge stretch strictly inside another polygon is skipped (each edge is
   sampled at 2 cm plus its closest point, nearest edges first). `_transit_approach` uses
   it for both the "start near the line" test and the corner test. Here the inset point
   is 0.30 m from the real boundary, so the transit goes to the approach point 0.30 m
   inside. The leg-start pivot and the tangent join (`edge_join_m`) then merge onto the
   lap from inside, as designed. run5's (-1.83, 4.48), which the corner rule was added for
   on 2026-10-22, is 0.42 m from the real boundary. It was never an edge start, and it
   still gets the start itself as the target.
2. `_route_final_approach`: when the start lies behind the robot along the sub-path's first
   leg (the target yaw), the hop **reverses** onto it (Nav2 BackUp, purpose
   `final_approach`). The robot turns to face away from the start, backs onto it and
   arrives facing along the leg. The leg-start pivot is then small or skipped. This applies
   only when every pose of the reversed hop keeps the body within its hard-boundary limits
   (`_safe_backoff`, 0.05 m steps, with `hard_boundary_body_clear_m`). Otherwise the
   straight hop runs as before. Going backwards, the tail leads; it is 0.10 m behind
   base_link against the nose's 0.60 m.
   8f112e7 narrows it: only onto a start on the line (within `edge_approach_near_m` of the
   union's boundary; an approach point 0.30 m inside or an interior start pivots there
   safely), and never while the slope caution cap is on (`_tilt_capped`, the same state
   that sets the transit controller's `vx_min` 0). The first version reversed at every
   approach point (5 times per run, with 130-175 deg spins first). On the slope world
   (s3), one such reverse near the bottom corner slipped into STUCK, and the return home
   then ended in STUCK_NEEDS_HELP. With fix 1 in place, the reverse hop is a backstop for an
   on-line start that still gets no approach point (a real corner, or
   `edge_approach_inset_m` 0). It did not fire in the final gate runs, so only the unit
   tests exercise it.

No new parameters (A6). New Mission field `final_approach_reverse`.

Tests (fail on master):
- `test_transit_ending_past_an_on_line_start_reverses_onto_it_along_the_leg`: the sim
  geometry (on the line 0.21 m past the start, 55 deg into the line). Spin to face away,
  BackUp 0.21 m, MOWING without a leg-start pivot.
- `test_final_approach_reverse_only_when_the_reversed_body_stays_within_limits`: hard
  edge_mode, so the sides on the line exceed 0.10 and the hop stays forward.
- `test_final_approach_no_reverse_on_a_slope_or_off_the_line` (added with 8f112e7).
- `test_nearest_boundary_point_skips_edges_inside_another_polygon`.
- `test_edge_start_beside_a_drawn_path_crossing_the_line_gets_the_approach_point`.

`scripts/ci_test.sh`: PASS (3874 cases, 0 failed).

## Sim

SIM_GATE_OUT `ros2_stack-wt/simout_approach1109`, bags pruned.

Final code (8f112e7) unless noted. Columns: a = base_link max out / samples > 0.10, b =
hard-boundary trips, e = sharp corners > 0.15 m.

| Run | a | b | STUCK | e | i | cut % (gt) | verdict | baseline |
|---|---|---|---|---|---|---|---|---|
| master, real_run5_smooth `-d -s 2` (repro) | 0.357 / 218 | **2** | 0 | 1 of 19 | FAIL | 96.2 | FAIL | - |
| real_run5_smooth `-d -s 2` | 0.059 / 0 | **0** | 0 | 1 of 19 (0.247) | FAIL (side 0.102) | 94.1 | FAIL (e, i) | smooth1109: b 1, a 0.345 / 276 |
| real_run5_smooth `-d -s 1` | 0.109 / 6 | **0** | 0 | 1 of 19 (0.194) | FAIL (side 0.108) | 96.2 | FAIL (a, e, i) | smooth1109: b 1, a 0.084 / 0 |
| real_run5 `-l -s 1` | 0.072 / 0 | 0 | 0 | 0 of 21 | FAIL (side 0.110) | 97.7 | FAIL (i) | stopfirst2-l1: PASS, i side 0.069 |
| real_run5_slope `-d -s 3` | 0.143 / 161 | 0 | 0 | 5 of 22 | FAIL | 96.6 | FAIL (a, e, h, j as baseline) | stopfirst-slope-s3: 0.157 / 73, b 0, 6 of 22 |
| first version (5b9a213), slope `-d -s 3` | 0.222 / 837 | 1 | **1** (+ STUCK_NEEDS_HELP) | 5 of 22 | FAIL | 96.6 | FAIL | led to 8f112e7 |
| first version, smooth `-d -s 2` / `-s 1` | 0.044 / 0, 0.142 / 57 | 0 / 0 | 0 / 0 | 1 / 1 of 19 | FAIL / FAIL | 95.9 / 96.0 | FAIL | |
| first version, real_run5 `-l -s 1` | 0.085 / 0 | 0 | 0 | 0 of 21 | FAIL (side 0.113) | 97.2 | FAIL (i) | |

Reading:
- The smoothed world's bar is met in both seeds: b = 0 and no STUCK. The transit now goes
  to the approach point (-1.85, 4.60), and lap 0 joins from inside. The smoothed-world
  a / e / i fails are elsewhere. seed 1's a (0.109, 6 samples) is lap 0 tracking on the
  east edge at (-0.51, 3.36), and the e corner is lap 0's -93 deg vertex at (-0.80, 5.34), which smooth1109 seed 1 passed and the master repro failed. The
  smoothed world already failed i before this change, and e in the repro. seed 2's 94.1 % cut is
  1.5-2 points under the other smoothed runs (95.5-96.2). That is the spread seen in
  smooth1109, but it is not shown to be equal.
- On real_run5 and the slope world the final code makes the same decisions as master. The
  mission logs show the same edge start (-5.27, 2.52) -> approach point (-5.03, 2.35), no
  reverse hop and the same 10 transits. real_run5 `-l`'s i side 0.110 (at (-0.60, 4.76),
  mid-leg on the east edge, planned 0.001) is therefore run-to-run variation: the
  2026-11-06 notes already call i side marginal (0.101-0.102). Still, it is a FAIL against
  a baseline that passed, from one run.
- Slope s3: no STUCK, b 0, as the baseline. a has more samples (161 vs 73) at a lower max.

## Not changed

- `_route_tangent` (area setting `tangent_approach`, off by default) and the boundary
  guard's `_hb_inward_dir` / `_hb_inward_side` still use `nearest_edge_point`, so they see drawn-path edges inside
  an area too. They are not involved here.
- Why the route's last leg runs square onto the line, and why the tracker cuts the bend
  before it: with the approach point inside, the route no longer ends on the line.
