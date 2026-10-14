# Are we over-engineering? Vendor vs MowgliNext vs ours vs the field (2026-10-14)

Inputs: two bags from today (`wheel_smoothness_field_tests.md` runs 1 and 2), a web survey of
OpenMower / MowgliNext / ArduPilot Rover / commercial mowers, and a file-referenced comparison
of the vendor firmware (`ota_v1.3.27…/src`), the MowgliNext clone and this stack (both agent
reports are summarised here; file references are in the comparison tables below).

Correction first: our docs and `cmd_vel_slew.py` cite the vendor's `mower_controller` /
`adaptivePurePursuit.h` (accel 0.5 / decel 0.8). That package is built but not launched.
The running vendor node is `navigation` (`mower_pnc/mower_controller/navigation/param/cfg.yaml`):
pure-pursuit tracker `NrtPPCTrack`, one velocity smoother at 0.6 / 1.0 m/s² and 2.0 / 2.0 rad/s²
with sigmoid and slope scaling, coverage speed 0.38 m/s (min 0.12), lookahead 0.6 s + 0.05 m,
goal 0.05 m, arc U-turns at 0.3 m/s / 0.5 rad/s with an 8 s timeout per arc, and a TURNCRAWL
creeping-rotation state. Transit is A* on rasterised drawn channels tracked with Stanley.

## The three designs in one table

| Topic | Vendor (running) | MowgliNext | Ours |
|---|---|---|---|
| Heading source | dock yaw + drive-to-compute service; RTK keyframe chords 0.3–0.8 m, weight 0.7, every 0.2 s | continuous COG from 0.10 m baselines, σ 0.15 rad, in a GTSAM factor graph; dock anchor; undock calibration | COG windows 0.6 m of straight travel, low-pass α 0.1 (0.3 for the first 3), into a scalar offset the EKF fuses as absolute yaw; dock / file / continuity sources |
| Correction cadence | ≈ 5 Hz while straight | continuous while v ≥ 0.08 m/s | one update per 0.6 m straight, 10 % applied |
| Gyro scale | none | none | none (both IMUs ≈ 1 %, measured today) |
| Dual antenna | no | GUI only | gated off (single antenna) |
| Coverage tracker | pure pursuit over a whole lane group, lanes switched inside the tracker | FTC carrot (MPPI tried and reverted), one goal per sub-path | RPP per sub-path; each Reeds-Shepp piece (0.2–0.8 m) is its own goal with an 8 cm / 0.5 rad checker; MPPI + FTC also configured |
| U-turn | circular arc default (also fish-tail, in-place, self-adaptive) | r 0.2 m arcs, pivot corners ≤ 0.6 m | Dubins → K-turn → pivot, then re-shaped downstream |
| Rotate rate | arcs 0.5 rad/s; in-place not found | 0.75–0.8 rad/s | 0.3 clamp, 0.25 shaper floor (wheels at 0.06 m/s = the deadband) |
| Wheel feasibility | not needed: the tracker never asks for it | firmware per-wheel PI across the deadband | host-side shaper + latch + assist + coupled slew, after the controller |
| Velocity limiting | one smoother | one ramp in hardware_bridge | three stages (MPPI model, velocity_smoother, cmd_vel_slew), two of them widened to defer to the third |
| Transit | A* restricted to channels, Stanley, local replans, no free-space fallback | Smac restricted to areas + corridor, no unrestricted fallback | route graph + MPPI, free-space costmap fallback (left the path twice in run 2) |
| Stuck | 5 s slip ratio + unstuck sequence inside the tracker | dig detector + firmware anti-dig | slip_detector + mission escape |
| Config keys (nav + loc) | ≈ 146 in yaml | ≈ 380 | ≈ 550–600 + ≈ 40 nav-related mission params |

What nobody else has: a turn shaper, a regime latch, a pivot assist, a coupled slew, lane-aware
shaping, a COG-window aligner with file continuity, a stale-COG START gate, a stereo/WIT virtual
yaw with gap bridging, a leg goal checker with skip rules, a pivot-aware stuck floor, a route
graph with hairpin splitting, nav corridor masks, a drift budget. Each has a documented reason
and each fixed a real incident. Together they are a repair layer.

## Verdict

Both, in different places.

**Done and worth keeping (not fiddling).** The hard stops (emergency / bumper 24–51 ms, was
0.6 s), the slow-pivot removal (200 s → 67 s), the gap bridging, the stale-heading gate, the
black box and the metrics. These are safety or observability and they are measured.

**Fiddling.** The angular clamp (0.3 vs the vendor's 0.5 rad/s arcs and 2.0 rad/s² ramps),
the pivot floor at exactly the deadband, three velocity limiters tuned against each other,
six controller configs, and goal tolerances per leg type. Every one of these exists because the
layer above produces commands the turf cannot execute: 0.25 rad/s pivots with the wheels on the
deadband, sub-metre turn pieces that must be hit to 8 cm, rotate-to-heading on grass.

**Too much, structurally.** Three places where the vendor and MowgliNext avoid the problem by
design and we solve it by adding behaviour downstream:

1. **Lane ends.** The vendor tracks a whole lane group and drives arcs (0.3 m/s at 0.5 rad/s,
   8 s timeout). MowgliNext drives r 0.2 m arcs or pivot corners. We cut the turn into
   Reeds-Shepp pieces and give each to Nav2 as a goal; run 2 spent 7 minutes on one 0.8 m
   piece. Fix 10 (time / rotation cap) is the vendor's 8 s timeout bolted on; the right change
   is one goal per swath pair with the turn driven as an arc by the tracker, pivots only where
   an arc does not fit.
2. **Heading.** We correct once per 0.6 m straight at 10 %; the vendor applies 70 % every
   0.2 s on 0.3 m chords; MowgliNext corrects continuously. Our gyros are ≈ 1 % off in scale
   and every long pivot costs that. The software step is cheap: chords from 0.3 m, gain
   0.5–0.7, correct at ≥ 0.08 m/s instead of only on 0.6 m straights, and a per-source
   scale from turn pairs. The hardware step is the only one the evidence calls robust on turf:
   a dual-antenna receiver (UM982 class), which the universal-gnss path already exposes.
3. **Transit.** The vendor runs A* on the drawn channels with no free-space fallback;
   MowgliNext has no unrestricted fallback either. Our costmap fallback is what drove off the
   path. Fix 11 removes it at hairpins and bounds it elsewhere; removing it entirely matches
   both references.

## Recommended order

1. Land fixes 10 and 11 (in progress) as stopgaps; field-test once.
2. Heading gain and cadence to vendor levels (parameters: min chord 0.3 m, α 0.5, update at
   speed, not only on 0.6 m straights); add the turn-pair scale estimate. One field test.
3. Decide the hardware question: dual-antenna receiver or live with motion heading. Everything
   in the aligner is a workaround for the single antenna.
4. Lane-end redesign: swath + turn as one tracker goal, arcs by default, raise the arc
   angular rate toward 0.5 rad/s with v 0.3 (vendor), which also retires the 0.25 rad/s pivot
   floor and most of the shaper's work in mowing. Keep the shaper as the safety net for the
   owner's both-wheels rule.
5. Prune: unused controller configs (MPPI coverage, FTC), two of the three velocity limiters,
   and fix the docs that cite the unused vendor controller.
