# Error budget: leg-end and pivot accuracy (2026-10-20)

Step 2 of the coverage plan review. The question is whether manoeuvre-dense plans (many
pivots, short legs) are viable, and how far each edge mode has to inset the base_link path.
The go/no-go rule: **if the p99 leg-end or pivot error is over 0.10 m, manoeuvre-dense plans
are not viable.**

Tool: `scripts/sim_gate/accuracy.py`, which takes `field <series.json> <mission.log>` or
`sim <run>/data.json`. Each logged in-place pivot (pivot vertex, route corner, leg start,
final approach) is paired with its `/spin` goal and the next `/follow_path` goal. Pose `a`
is taken when the spin is sent and pose `b` when the next leg is sent.

- **Leg-end along / lateral:** `a` minus the vertex, in the heading frame at `a`.
  Positive along means an overshoot.
- **Pivot drift:** `|b - a|`. The excursion is the maximum displacement during the pivot.
- **Heading error:** the yaw turned minus the commanded angle.
- **Leg-start offset:** `b` minus the vertex, in the frame of the target heading.

Legs are classed by length: a swath is at least 1.5 m, a connector is shorter than
1.5 m, and a route corner counts as transit. All numbers are absolute values in m (heading
in deg). With n = 9-27 per class, **p99 is the max**.

## Data

- **Field:** run 7 (`20261016-054318`, master f257a15, the first complete mow), 46 pivots.
  The reference is the EKF (`/odometry/filtered_map`), so these are control errors as the
  stack saw them. There is no ground truth in the field.
- **Sim:** new world `src/mower_sim/worlds/accuracy.yaml`, an irregular 3.4 x 2.7 m hexagon
  run with the committed parameters and master's execution path.
  - 26-27 pivots per run, swath legs 1.5-3 m, connectors 0.2-1.2 m, about 6 min of mowing.
  - Two runs on domain 181: ideal, and `-d` (slip, wheel bias, pivot efficiency 0.85, loc
    drift, seed 1).
  - Reference: `/sim/ground_truth`.
  - The ideal run ended NAV_TO_DOCK_FAILED. The dock sits inside this small lawn; this does
    not affect the metrics.

## Results (p99 = max; median / p99)

| Metric | Class | Field run 7 (EKF) | Sim ideal (truth) | Sim `-d` (truth) |
|---|---|---|---|---|
| Leg-end along | swath | 0.008 / 0.023 | 0.011 / 0.017 | 0.051 / **0.120** |
| | connector | 0.037 / *0.350* | 0.008 / 0.018 | 0.008 / 0.055 |
| | transit | 0.046 / 0.096 | 0.008 / 0.029 | 0.021 / 0.058 |
| Leg-end lateral | swath | 0.052 / **0.106** | 0.018 / 0.064 | 0.025 / 0.064 |
| | connector | 0.029 / 0.058 | 0.025 / 0.064 | 0.011 / 0.059 |
| | transit | 0.022 / **0.330** | 0.000 / 0.033 | 0.024 / 0.058 |
| Pivot drift | all | 0.087 / **0.385** (p90 0.221) | 0.053 / 0.080 | 0.046 / 0.078 (EKF 0.113) |
| Pivot excursion | all | 0.094 / **0.402** | 0.053 / 0.080 | 0.048 / 0.080 (EKF 0.117) |
| Pivot heading error (deg) | all | 3.4 / 7.2 | 1.5 / 2.9 | 1.3 / 2.1 |
| Leg-start along | all | 0.103 / **0.456** (p90 0.250) | 0.024 / 0.091 | 0.049 / **0.121** |
| Leg-start lateral | all | 0.031 / **0.475** (p90 0.113) | 0.044 / 0.085 | 0.057 / **0.130** |
| Mowing cross-track (gate h, line) | | p99 0.089 (long swaths) | p99 **0.178** | p99 **0.207** |

The connector along 0.35 m and leg-start lateral 0.475 m come from run 7's double vertex
(-2.10, 1.63) / (-2.18, 1.68), which is 0.09 m apart. That geometry is manoeuvre-dense by
itself.

Findings:

1. **Pivots are not pure rotations, even in the ideal sim.** base_link moves 5 cm (median)
   in every pivot with a perfect drive. This is commanded motion: the spin, then the shaper
   or wheel floor. It is not slip.
   - In the field, the drift fits `2 L sin(dθ/2)` with L ≈ 0.06 m in most pivots, and two
     pivots drift 0.38 m.
   - The field drift is therefore either the same commanded creep or a GNSS lever-arm or
     heading error in the EKF. The latter matches run 7's open "4 cm right bias".
2. **Every pivot turns too far:** 46/46 in the field, 53/53 in the sim. The overshoot is
   1-3 deg in the sim and 2-7 deg in the field.
3. **Short legs do not settle.** The accuracy world has the same controller as run 7 but
   legs under 3 m. Its mowing cross-track p99 is 0.18-0.21 m, against 0.089 m on run 7's
   3.5-7 m swaths, and corner deviation fails gate e (max 0.23 m). The leg-start offset
   (field p90 0.25 m along / 0.11 m lateral, plus the heading error) is still being removed
   when a short leg ends.

## Verdict: NO-GO for manoeuvre-dense plans

The p99 exceeds 0.10 m in the field (leg-end lateral 0.106 swath / 0.33 transit, pivot drift
0.385) and in the sim with disturbance (leg-end along 0.120, leg-start 0.13, cross-track
0.21). Only the ideal sim stays under 0.10 m.

Plans should keep pivots few and legs long until the following are fixed and re-measured
with this tool:

- the pivot drift (pure-pivot regime, the lever arm);
- the spin overshoot.

## Recommended insets (base_link path to the boundary)

Footprint, from base_link: nose 0.52 m, rear 0.23 m, half-width 0.27 m. The pivot sweep
radius is 0.586 m (nose corner) and 0.355 m (rear corner).

Error terms, worst of field and sim `-d` p99:

- lateral 0.11 m;
- along 0.12 m;
- pivot excursion 0.22 m (field p90; the p99 is 0.40 m and is treated as a defect to fix,
  not budgeted).

| Edge mode | Driving along the edge (side) | Leg end facing the edge (nose) | Pivot at a vertex (sweep) |
|---|---|---|---|
| **Hard**: whole body inside + 0.10 m | 0.27 + 0.11 + 0.10 = **0.48 m** | 0.52 + 0.12 + 0.10 = **0.74 m** | 0.586 + 0.11 + 0.22 + 0.10 = **1.02 m** (1.20 m at the p99 drift) |
| **Soft**: sides may overhang 0.25 m, nose + rear inside | 0.27 - 0.25 + 0.11 = **0.13 m** | 0.52 + 0.12 = **0.64 m** | 0.586 + 0.11 + 0.22 = **0.92 m** (nose corners) |

So in both modes a pivot costs about 0.5 m more inset than a straight pass along the edge.
Edge passes should therefore turn on arcs or pivot only at the inner end, never on the
boundary.

**Minimum viable leg length: 1.5 m.**

- On legs of 1.5 m or more (run 7 swaths) the leg end holds 0.106 m lateral and 0.023 m
  along.
- Below that, the start offset (up to 0.25-0.33 m) and the 2-7 deg heading error are not
  removed before the leg ends. Cross-track p99 roughly doubles.
- Connectors under 1.5 m are acceptable only as non-cutting moves with the margins above.
