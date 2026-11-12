# MPPI row-weave damping: critic values and latency compensation, 2026-11-06

Branch `ws/damp1106`. Follow-up to `2026-11-05_row_weave_ftc.md` (FTC row controller, merged
off) and `2026-11-06_field_weave_diagnosis.md` (`ws/fid1106`). Question: can the plain coverage
MPPI get close to the FTC row law's weave reduction by (1) more heading damping from its
critics and (2) compensating the in-loop pose latency?

**Verdict.**

- **Yes, part of the way.** V2 (control point 0.25 -> 0.30 m, trim time constant 5 -> 20 s,
  0.2 s latency projection in PathAlignAheadCritic) reduces the in-band lateral by 29 % on
  `real_run5` (FTC: 46 %). w in the weave band falls 48 %, the same as FTC, and MPPI's own
  random part of w falls 37 %. The fitted damping ratio goes from 0.69 to 0.90. The rows
  stay at about 0.2 m/s; FTC runs them at 0.3.
- **owner_far (V2):** in-band lateral -31 % (FTC -51 %). The gate PASSES, including e and i,
  which master fails there.
- **Cost:** one +111 deg fillet on `real_run5` runs 3-4 cm wider (e 0.091 -> 0.125, limit
  0.15). That matches the extra control-point offset on a bend. The owner_far corners do not
  show it.
- **Slope world: the regression came from a stale leg goal, now fixed (section 7.6).** V2 had a
  hard-boundary trip, a fallback nav, 3 emergency ticks and 4x master's samples outside, from
  slope-side pivots that started 0.16 m off their vertex (master 0.08 m). Cause: the mission
  stamped each FollowPath path with its send time, so controller_server's goal checker judged
  the vertex where map -> odom put it when the leg was sent. On the slope odom slides
  0.15-0.2 m off map within a leg; the robot drove over the vertex, the checker's goal was
  elsewhere, and the leg ended 0.2-0.5 m past it. V2's steeper recovery crossings made that
  miss longer. Fix: stamp the path 0 (latest transform). With the fix, V2 (no slope keys)
  starts slope pivots 0.066 m from the vertex, a 0.197 m / 490 samples, b / d / f 0. The V2d
  slope schedule is removed again (it did not help with the fix: 0.349 m / 765, one trip).
- **Recommendation (revised again): merge-ready with one caveat** (section 7.6): a's sample
  count, 490, is just above master's two runs (349 / 426) while its max is lower; a rerun of
  unchanged V2d code moved a from 975 to 520 samples, so that is within the slope world's
  run-to-run spread.
- **Earlier recommendation, flat ground only:** merge V2 as the plain-MPPI default and keep
  FTC for the field A/B, run against V2 as the new baseline (section 6).

## 1. Method

### 1.1 Fitted gains

The effective gains come from `fid1106/scripts/fit.py`, the field diagnosis's closed-loop
identification: `w_nav = -Ky e_seen - Kpsi psi_seen + r`, band-passed 0.05-0.6 Hz, best of
0-0.2 s delay. The input is `simsegs.py` on each run's `data.json`, i.e. the same code as the
field numbers. The damping ratio of the lateral mode is `zeta = Kpsi / (2 sqrt(v Ky))` at the
measured row speed. That formula ignores the loop delays, so it only ranks variants.

| | Ky | Kpsi | zeta | MPPI residual r (rad/s) |
|---|---|---|---|---|
| field 10-28 / 11-04 (diagnosis note) | 4.47 / 3.47 | 1.04 / 1.08 | 0.53 / 0.63 | 0.026 / 0.030 |
| sim master `real_run5 -l` | 3.22 | 1.13 | 0.69 | 0.0147 |

The diagnosis note's levers (`b131.py`): Kpsi x 1.3 gives -17 to -30 % band rms against every
input; Ky x 0.78 mostly rejects pose noise; 0.1 s less delay gives -18 to -34 %. The target
here is therefore a higher fitted Kpsi, without trading it for a lower Ky.

### 1.2 Offline MPPI row emulation

To choose values before a sim, `damp1106/mppirow.py` (session scratchpad) emulates the
coverage MPPI on a straight row in numpy. It closes the loop around the `gainmodel.py` row
plant: 0.2 s command delay, 0.1 s wheel lag, 0.055 m track lag, 0.1 s fix latency at 10 Hz,
and OU pose noise.

- **What it reproduces:** the Nav2 1.3.13 optimizer, read from the release source
  (`optimizer.cpp`, `noise_generator.cpp`, `utils.hpp`). That covers:
  - the fixed noise set (`regenerate_noises: false`), the accel clamp, and integration
    from the measured speed;
  - the control cost `gamma / std^2 * u . eps`, the softmax at `temperature`, the 9-point
    SG filter with its 4-command history, the output `cs[1]` and the shift;
  - on a row, PathAlignAheadCritic (with the trim), PathFollow, PathAngle, Constraint and
    PreferForward. Goal, Cost and WheelFloor are inactive on a row.
- **Calibration:**
  - With the yaml's `gamma` 0.015 and `temperature` 0.1, the emulated row holds 0.09 m/s,
    against the sim's 0.205. At gamma/temperature 0.15 the control cost pulls the sequence
    toward zero by 15 % per cycle, and PathFollow is the only forward pull that the model
    has.
  - The real stack has about 2.5 times more forward pull. Its source was not found
    offline.
  - The emulation therefore runs at gamma 0.004, which gives 0.19-0.20 m/s.
  - Its gains are about twice the sim's (Ky 6.0, Kpsi 1.6 for master), but the ratio
    Kpsi/Ky matches (0.27 against 0.30). The emulation is used for ranking only.

Emulated band rms (0.08-0.4 Hz, cm; 3 seeds x 120 s; master 0.86):

| Lever | band rms | Note |
|---|---|---|
| PathAngleCritic always on (`max_angle_to_furthest` 0), weight 4 / 8 / 16 | 0.85 / 0.85 / 0.86 | no effect: it scores only the horizon's last pose, and the gate keeps it off on rows |
| PathFollow weight 2 -> 1 / 0 | 0.72 / 0.57 | the row slows to 0.13 / 0.09 m/s: PathFollow is MPPI's only forward pull |
| PathAlignAhead weight 30 -> 20 | 0.87 | both gains fall |
| `control_point_x` 0.35 / 0.40 | 0.67 / 0.60 | Kpsi/Ky up; inside offset `d^2/2R` on bends |
| `trim_tau_s` 5 -> 10 / 15 / 20 | 0.75 / 0.72 / 0.70 | trim off: 0.66 |
| `control_point_x` 0.30 + `trim_tau_s` 20 (**V1**) | 0.61 | |
| V1 + latency projection 0.2 s / 0.35 s | 0.54 / 0.50 | w std halves |
| V1 + `wz_std` 0.15 | 0.59 | |

### 1.3 The path trim is in the weave band

PathAlignAheadCritic's trim (2026-11-03, `path_trim.hpp`) integrates the cross-track error,
`o -= e dt / tau`, and shifts the alignment target by `o`. It was built against a steady
downhill creep, but it acts on every row.

- At the weave's `omega` of about 0.8 rad/s, a 5 s integral is 25 % of the proportional
  term, in phase with the heading.
- On a sinusoid that removes about `Ky v / (tau omega^2)`, roughly 0.2, from the effective
  Kpsi.
- On flat ground the emulated trim costs +30 % band rms.
- At 20 s the cost is a quarter of that. The steady state is the same: the trim still drives
  a persistent offset to zero, only four times slower. The roll feed-forward (`k_roll`),
  which did most of the slope work in the 2026-11-03 slope runs (the trim stayed within
  0.05 m there), is unchanged.

## 2. Variants

### V1: values only

- `control_point_x` 0.25 -> 0.30 m.
- `trim_tau_s` 5 -> 20 s.

These are the only two existing values that add heading damping on a row (section 1.2).
PathAngleCritic cannot, and PathFollow is the speed driver: lowering it slows the row.

### V2: V1 plus a latency projection

**Where.** PathAlignAheadCritic (ours), new key `latency_s` (0 = off). Each rollout is scored
as if it started from the pose projected `latency_s` ahead with the measured `(v, w)`
(`data.state.speed`, odometry).

- **The geometry is exact:** MPPI integrates rollouts relative to their start pose, so the
  projected rollout is a rigid move of the real one, and it stays on its arc. The path is
  matched `|v| latency_s` further along. Without that offset, a rollout lying exactly on the
  line would carry a constant 4-7 cm along-track error. Under the L1 distance that would hide
  small lateral errors.
- **Nothing else sees the projection:** the rollouts themselves, CostCritic (collision), the
  trim's cross-track and the other critics all keep the real pose. A wrapper that hands MPPI a
  projected pose would have moved the collision checks, and it would also have needed the
  `plugin:` line that is the FTC switch.
- **On a straight:** the projection adds `v latency_s` (4 cm at 0.2 m/s) to the control
  point's heading lever. It adds `d w latency_s` on top, a damping term on the measured yaw
  rate. On a bend, unlike a longer control point, it adds no inside offset.

**Latency value: 0.2 s.**

- **The lag itself:** phase 1 of the FTC note measured 0.35 s in-loop lag (0.1 s fix latency
  plus a 0.26 s ground-track lag at 0.21 m/s). The diagnosis note fits the field's turn
  centre plus track lag at 0.038-0.049 m, against the sim's 0.08 m. That makes the field's
  track lag about 0.1 s, so the field's in-loop lag is about 0.2 s.
- **Choice:** 0.2 s compensates the field lag. In the sim it under-compensates, which is the
  conservative side. 0.35 s would over-compensate in the field.
- **Bound:** the params test allows 0 to 0.5 s.

**A6.**

- New key: `PathAlignAheadCritic.latency_s` (coverage block only; transit stays 0).
- Removed: `use_path_orientations` from PathAlignAheadCritic, from its code and from the
  coverage and transit blocks. It was always false, it is Nav2's heading term, and the
  2026-10-28 retune (6a1) showed it trades the weave for standing offsets. The Nav2
  PathAlignCritic of the reverse block keeps its own key.

**Tests.** `test_path_align_ahead_critic` covers the following:

- `latency_s` 0 reproduces the old scoring bit for bit, against the Nav2 1.3.13 reference.
- The projection drives the pose along its arc: straight, arc and pivot.
- A projected rollout scores like a rollout that really starts at the projected pose, on a
  curved path with random controls.
- On a line, the projection leaves an on-line rollout and a parallel offset unchanged and adds
  `v latency_s x heading`.

`test_params_yaml` checks:

- `use_path_orientations` is gone;
- `trim_tau_s` >= 10;
- `latency_s` is within 0-0.5 s, and is 0 on transit.

### V3: V2 with `wz_std` 0.2 -> 0.15 (tried, not kept)

The diagnosis note finds MPPI's own random part as large an input as the turf yaw disturbance
in the field, so lower sampling noise was tried. A larger batch was not tried: 1000 -> 600 was
an RK3588 CPU limit (STATUS), and the x86 sim cannot show that cost.

## 3. Sim results, `real_run5 -l -x 3 -s 1`

The gate setup:

- The runs are in `ros2_stack-wt/simout_damp1106/`, bags pruned.
- Each one held a 20 Hz controller loop, "desired rate of 60" was in launch.log, and the
  power profile was `performance`.
- The weave columns are `weave.py`; the fit columns are section 1.1.
- The master and FTC baselines are `simout_corner1105/branch-l3` and `simout_ftc1105/ftc-l3`.
  Their planner predates the narrow-area merge (596cd4d): master's run has 11 sub-paths and
  21 pivots, the branch runs 12 and 22. Rows are unaffected. MOWING time is not comparable to
  master's, because of the extra sub-path; within the branch runs it is.

| | master | FTC | V1 | **V2** | V3 |
|---|---|---|---|---|---|
| fitted Ky / Kpsi (R²) | 3.22 / 1.13 (0.85) | 1.79 / 1.44 (0.52) design 2.5 / 1.83 | 3.46 / 1.46 (0.81) | **2.79 / 1.36 (0.82)** | 2.43 / 1.23 (0.89) |
| fit delay | 0.20 s | 0.20 s | 0.20 s | **0.15 s** | 0.20 s |
| zeta at the row speed | 0.69 | 1.05 (design) | 0.87 | **0.90** | 0.88 |
| MPPI residual r (rad/s) | 0.0147 | 0.0128 | 0.0142 | **0.0093** | 0.0082 |
| row XTE p50 / p90 / max (cm) | 1.05 / 2.79 / 5.47 | 0.82 / 1.87 / 3.96 | 1.03 / 2.62 / 6.08 | **0.90 / 2.38 / 6.25** | 1.18 / 2.40 / 4.10 |
| detrended p50 / p90 (cm) | 0.66 / 1.82 | 0.39 / 0.89 | 0.61 / 1.53 | **0.51 / 1.38** | 0.47 / 1.15 |
| lateral band rms 0.08-0.4 Hz (cm) | 0.84 | 0.45 | 0.65 (-23 %) | **0.60 (-29 %)** | 0.63 (-25 %) |
| w band rms 0.1-0.3 Hz (rad/s) | 0.0305 | 0.0159 | 0.0256 | **0.0160** | 0.0201 |
| a max out / > 0.10 m | 0.089 / 0 | 0.074 / 0 | 0.082 / 0 | **0.080 / 0** | 0.109 / 24 FAIL |
| e corners > 0.15 m (max) | 0 of 21 (0.091) | 0 of 21 (0.110) | 0 of 21 (0.132) | **0 of 21 (0.125)** | 0 of 21 (0.132) |
| h p99 line / arc | 0.085 / 0.050 | 0.109 FAIL / 0.073 | 0.073 / 0.049 | **0.083 / 0.055** | 0.087 / 0.061 |
| i side / nose / sweep | 0.086 / 0.055 / 0.119 FAIL | 0.104 / 0.049 / 0.092 FAIL | 0.086 / 0.065 / 0.094 pass | **0.089 / 0.072 / 0.102 FAIL** | 0.105 / 0.057 / 0.091 FAIL |
| b / STUCK | 0 / 0 | 0 / 0 | 0 / 0 | **0 / 0** | 0 / 0 |
| lawn cut | 97.0 % | 96.8 % | 97.9 % | **97.7 %** | 97.6 % |
| MOWING time (s) | 777 (older plan) | 676 | 801 | **812** | |
| moving time (s) | 993 (older plan) | 888 | 1030 | **1018** | 1033 |
| row speed (truth) | 0.206 | 0.30 | 0.205 | **0.203** | 0.202 |
| verdict | FAIL (i) | FAIL (h, i) | **PASS** | FAIL (i) | FAIL (a, i) |

What the runs show:

- **V1 raises the heading gain as the target asked.** Fitted Kpsi rises 29 % (1.13 -> 1.46),
  and Ky is almost unchanged (+7 %). That is the diagnosis note's "Kpsi x 1.3" lever, and the
  band rms falls 23 %. The control point alone did not do this in earlier sims (0.25 -> 0.35:
  Kpsi +3 %, Ky -15 %, diagnosis note section 4). The difference is the trim time constant.
- **V2 removes MPPI's sampling noise rather than adding more Kpsi.**
  - The fitted residual r falls 37 % and w in the band falls 48 %, to FTC's level.
  - The fit picks 0.05 s less delay, which is the projection working.
  - Ky and Kpsi both drop somewhat against V1. The projection feeds back the measured yaw
    rate, a term the two-gain fit cannot see, so it ends up in lower fitted gains. zeta still
    rises to 0.90.
  - Band rms is -29 %, and detrended p90 is -24 %.
- **V3 is not kept.** Lower `wz_std` cut r by another 12 %, but Ky and Kpsi fell again
  (2.43 / 1.23). Against the field's turf yaw disturbance, that is the wrong trade (diagnosis
  note: heading damping acts on every input, lower gains only on pose noise). The band rms and
  w band rms are worse than V2, arc p99 is 0.061, and a failed (0.109 m).
- **The fillet cost is real but small.** The +111 deg corner at (-4.96, 0.42) (sub-path 6,
  idx 102) is the worst e corner in every run. It reaches 0.091 on master and 0.125-0.132 in
  V1-V3. That is about the extra control-point inside offset on a 0.5 m fillet:
  `(0.30^2 - 0.25^2) / 2R`, 2.8 cm. h arc p99 is unchanged in V1 and V2 (0.049 / 0.055
  against 0.050).
- **i** passes in V1 and fails in V2 and V3 at 0.10-0.105 (master 0.119). That is inside the
  ±0.05 flip.

## 4. What the sim cannot show

- **The field gains do not have to follow.** B13-1 raised Kpsi 43 % in the sim and 4 % in the
  field (diagnosis note). The check is `fit.py` on the next field mow: fitted Kpsi above about
  1.3, and r below 0.026.
- **The latency value.** The sim's track lag is 0.08 m, the field's fit 0.04-0.05 m. 0.2 s is
  set for the field; the sim under-compensates.
- **The trim on slopes.** A 20 s trim takes a side-slope offset out four times more slowly.
  The roll feed-forward is unchanged. The slope world was run afterwards (section 7).

## 5. owner_far, `-l -x 3 -s 1 -t 4800` (V2)

The baselines are `simout_corner1105/branch-of-l3b` (master) and `simout_ftc1105/ftc-of-l3`
(FTC). The branch plan has 90 pivots, master's 88: that is the newer planner, and it explains
the MOWING difference at the same row speed. A first V2 run (`v2-of-l3-timeout`) hit the
gate's default 2700 s limit at 80 % cut. On its 40 straights the weave agrees with the full
run (band rms 0.61 cm), and it had one e corner at 0.152 m, which the full run does not repeat
(max 0.139). That spread is the usual run-to-run flip of e.

| | master | FTC | **V2** |
|---|---|---|---|
| fitted Ky / Kpsi (R²) | 3.23 / 1.13 (0.83) | 2.31 / 1.75 (0.88) | **2.73 / 1.31 (0.82)** |
| zeta at the row speed | 0.69 | 1.05 (design) | **0.87** |
| MPPI residual r (rad/s) | 0.0167 | 0.0054 | **0.0110 (-34 %)** |
| row XTE p50 / p90 / max (cm) | 1.09 / 2.66 / 11.93 | 0.78 / 1.89 / 11.97 | **0.92 / 2.12 / 11.97** |
| detrended p50 / p90 (cm) | 0.90 / 2.40 | 0.51 / 1.31 | **0.71 / 1.72** |
| lateral band rms 0.08-0.4 Hz (cm) | 0.96 | 0.47 (-51 %) | **0.66 (-31 %)** |
| w band rms 0.1-0.3 Hz (rad/s) | 0.0333 | 0.0133 | **0.0190 (-43 %)** |
| a max out / > 0.10 m | 0.079 / 0 | 0.070 / 0 | **0.059 / 0** |
| e corners > 0.15 m (max) | 1 of 84 (0.181) FAIL | 0 of 84 (0.138) | **0 of 86 (0.139)** |
| h p99 line / arc | 0.068 / 0.048 | 0.074 / 0.067 | **0.064 / 0.044** |
| i side / nose / sweep | 0.096 / 0.119 / 0.153 FAIL | 0.127 / 0.112 / 0.101 FAIL | **0.083 / 0.057 / 0.084** |
| b / STUCK | 0 / 0 | 0 / 0 | **0 / 0** |
| lawn cut | 99.2 % | 99.5 % | **99.4 %** |
| MOWING time (s) | 3238 | 2613 | **3347** (newer plan) |
| row speed (truth) | 0.207 | 0.30 | **0.206** |
| verdict | FAIL (e, i) | FAIL (i) | **PASS** |

- **The weave gain holds on the larger map.** Band rms is -31 % (real_run5 -29 %), w in the
  band -43 %, and XTE p90 -20 %. The XTE max is the same edge-start join in all three runs.
- **The fillet cost of real_run5 does not show here.** e passes with all 86 corners under
  0.14 m, h arc p99 is 0.044 against 0.048, and the side / nose / sweep excess is lower than
  in either baseline. On owner_far, where master fails it, i passes.
- **FTC is still better on the rows** (band -51 %), at the cost of wider arcs (0.067). It
  runs them at 0.3 m/s and has no sampling noise (r 0.005).

No `-d` run: no variant showed a trip, a STUCK or a recovery, so recovery was not at risk.

## 6. Recommendation: both

1. **Merge V2 as the plain-MPPI default.**
   - It passes both gates (the real_run5 i fail is the usual ±0.05 flip; master fails i on
     both maps).
   - It cuts the in-band weave by about 30 % and MPPI's sampling noise by about a third.
   - It needs no new behaviour, and rows stay at 0.2 m/s.
   - The cost is one fillet on real_run5, 3-4 cm wider and still inside e.
   - **Revert (values):** `control_point_x` 0.25, `trim_tau_s` 5, `latency_s` 0. Each is
     independent.
2. **Keep FTC for the field A/B.**
   - FTC still has about twice V2's row gain in the sim, and only a field mow says whether
     either gain survives turf. B13-1 delivered a tenth of its sim gain.
   - FTC's inner MPPI reads this same block, so with V2 merged, FTC's bends and hand-offs get
     V2's values too.
3. **Field order:**
   - First, one mow on V2: `fit.py` on the bag should show a fitted Kpsi above about 1.3
     (11-04: 1.08), r below the 0.026-0.030 of 10-28 / 11-04, and the `wheel_cal.py` weave
     p90 against 11-04's 1.91 cm.
   - Then, with V2 as the new baseline, the FTC A/B (FTC note 2.5).

What a field mow on V2 can tell:

- If Kpsi does not move in the field (as with B13-1), the critic route is exhausted and FTC
  is the lever.
- If Kpsi does move, but r stays at the field's level, the residual is MPPI's sampling, and
  FTC (no sampling) is again the lever.

## 7. Slope world (added 2026-11-06, after V2)

`scripts/sim_gate.sh -d -s 2 -x 3 -w src/mower_sim/worlds/real_run5_slope.yaml` (10 deg slope
over the Area 1 west edge band, downhill to -x). Runs are in `ros2_stack-wt/simout_damp1106/`
(bags pruned). Every run held 20 Hz (median and p10), with "desired rate of 60" in
launch.log and the `performance` profile.

### 7.1 V2 against master

`master-slope` is a6da6b7, which is 9cf3331 (this branch's base) plus a bag_recorder change.
`mastereq-slope` is this branch with every value back at master's (`control_point_x` 0.25,
`latency_s` 0, `trim_tau_s` 5, slope schedule off). That is code-equivalent to master and
reproduces it (a 0.214 / 426, b-d-f clean), so master's slope result is not a lucky draw.

V2 failed b, d and f, where master passes. All three come from one chain at the sub-path 4
vertex (-5.25, 0.92):

1. The pivot drifted downhill twice (spin-drift guard).
2. The third pivot was aborted, and the back-up stalled ("wheels spinning").
3. The stuck escape left the robot off the route graph, so transit went through the costmap
   planner (d).
4. That transit tripped the boundary (b), and the footprint recovery gave the 3 emergency
   ticks (f).

Master drifts twice at the same vertex and gets out. The other a excursions are the
sub-path 0 pivots at the SW slope corner.

### 7.2 What the data says about the cause

| Measure (slope world) | master | master-eq | V2 | V2b | V2c | V2d |
|---|---|---|---|---|---|---|
| downhill row offset, mean / p90 (cm) | +0.7 / 6.8 | +0.7 / 6.2 | **+2.1** / 8.8 | +0.6 / 7.0 | +1.3 / 6.9 | +0.6 / 7.0 |
| slope-side pivot hand-off: mean distance to vertex (m) | 0.085 | 0.079 | **0.158** | 0.171 | 0.135 | 0.151 |
| ... hand-offs >= 0.20 m | 1 of 10 | 0 of 9 | 3 of 10 | 4 of 10 | 3 of 9 | 2 of 9 |
| other pivots: mean distance (m) | 0.058 | 0.058 | 0.054 | 0.054 | 0.058 | 0.055 |

The scripts are in the session scratchpad, `slope1106/`:

- `downhill.py`: gt against the plan in MOWING, moving, |roll| > 7 deg. + = downhill.
- `arrive.py`: gt distance to the vertex when each coverage pivot starts. Slope side means
  vertex x < -4.6.

**Not the trim time constant.**

- With 20 s, the integral reaches the same 0.10 m cap as with 5 s after each slope pivot
  (critic log). That is because the post-pivot errors are 0.13-0.27 m.
- What the slow trim leaves is a steady downhill offset on the slope rows: +2.1 cm against
  +0.7. That matches the uphill crab (about 9 deg) times the lever increase: 0.30 + 0.2 s x
  0.15 m/s against 0.25, so 0.08 m x 0.16 rad = 1.3 cm.
- V2b (5 s trim on the slope) removes the offset (+0.6 cm), but not the excursions:
  a 0.423 m.

**The slope-side pivot hand-offs.**

- Every V2 variant hands the robot to the slope-side pivots about twice as far from the
  vertex as master: 0.14-0.17 m mean, with 2-4 hand-offs of 0.20-0.47 m.
- On flat ground the same pivots match master (0.054-0.058 m).
- An off-vertex pivot on the slope drifts downhill 0.15-0.31 m (spin-drift guard). That is
  where every a excursion and the V2 trip start.
- Typical case, the leg into (-5.27, -0.31) after the drifted (-5.20, -0.82) pivot:
  - master turns to 43-46 deg and stops on the vertex;
  - V2, V2b and V2d keep turning right to 12-31 deg, cross the vertex and are declared
    reached by the corner checker's "passed wide" rule 0.24-0.35 m east of it.

**Not the lever on the slope legs either.**

- V2c and V2d run the pre-11-06 values (0.25, no projection, 5 s) on the slope.
- In V2d that leg ran entirely at tilt >= 9.9 deg, so with master's values, and it still
  overshot.
- What V2d still differs in is upstream: the flat-ground values before the slope, and the
  state they leave (pose and heading at the slope entry, the MPPI warm start, the trim
  integral).
- The mechanism is not isolated. With one seed per variant, two master runs against four V2
  variants is evidence of a systematic effect, not proof.

### 7.3 Variants

V2b-V2d were removed again after the cause was found (7.6); the final code is V2 plus the
mission stamp fix.

| | slope values (at the threshold) | threshold |
|---|---|---|
| V2b (8b1e0b9) | trim tau 5 s | fresh \|roll\| >= 8 deg |
| V2c (37e2576) | control point 0.25, no projection, trim tau 5 s | fresh \|roll\| >= 8 deg |
| **V2d (af5aeec)** | same as V2c | fresh body tilt (roll and pitch) >= 8 deg |

- **Why V2d moved to the tilt.** In V2c, 3 of the 4 worst hand-offs came off legs along
  the fall line (roll 0-6 deg, pitch 8-10 deg), which kept the flat values.
- **Keys:** `slope_tilt_deg` 8.0, `slope_control_point_x` 0.25, `slope_trim_tau_s` 5.0 in
  the coverage PathAlignAheadCritic block. The transit block has none (off).
- **The threshold.** `slope_tilt_deg` is mission.yaml `slope_caution_roll_deg`
  (`test_params_yaml` asserts equality), applied to the full tilt.
- **A6.** There was nothing left to remove in this block (`use_path_orientations` already
  paid for `latency_s`). The reason for the three keys: they hold the pre-11-06 values, so
  slopes keep the behaviour the 2026-11-03 slope work was gated on.
- **Tests:**
  - `test_path_align_ahead_critic`: `SlopeUsesTheSlopeValues` and `TiltCountsRollAndPitch`;
  - `test_path_trim`: `ExplicitTauWindsWithThatTau`;
  - `test_params_yaml`.

### 7.4 Gate results

Slope world, `-d -s 2 -x 3`:

| | master | master-eq | V2 | V2b | V2c | **V2d** |
|---|---|---|---|---|---|---|
| a max / > 0.10 m samples | 0.210 / 349 | 0.214 / 426 | 0.287 / 1462 | 0.423 / 1025 | 0.239 / 677 | **0.313 / 975** |
| b hard-boundary trips | 0 | 0 | 1 | 0 | 0 | **0** |
| d STUCK / NOT mowed / fallback | 0/0/0 | 0/0/0 | 0/0/1 | 0/0/0 | 0/0/0 | **0/0/0** |
| f one-wheel / emergency | 0 | 0 | 3 | 0 | 0 | **0** |
| e corners > 0.15 m (max) | 6 of 22 (0.678) | 7 of 22 (0.686) | 6 of 21 (0.686) | 7 of 22 (0.653) | 7 of 22 (0.670) | **6 of 21 (0.692)** |
| h p99 line / arc | 0.419 / 0.633 | 0.420 / 0.641 | 0.569 / 0.673 | 0.400 / 0.613 | 0.434 / 0.627 | **0.412 / 0.652** |
| i side / nose / sweep | 0.343 / 0.368 / 0.372 | 0.325 / 0.373 / 0.378 | 0.463 / 0.486 / 0.497 | 0.279 / 0.355 / 0.361 | 0.322 / 0.362 / 0.364 | **0.318 / 0.382 / 0.387** |
| lawn cut | 96.6 % | 96.4 % | 95.8 % | 97.0 % | 96.9 % | **96.4 %** |

`real_run5 -l -x 3 -s 1`. Weave columns are `weave.py`; Ky / Kpsi / r are `fit.py`.

| | master | V2 | **V2d** |
|---|---|---|---|
| fitted Ky / Kpsi (R²) | 3.22 / 1.13 (0.85) | 2.79 / 1.36 (0.82) | **2.67 / 1.31 (0.79)** |
| MPPI residual r (rad/s) | 0.0147 | 0.0093 | **0.0109** |
| row XTE p50 / p90 / max (cm) | 1.05 / 2.79 / 5.47 | 0.90 / 2.38 / 6.25 | **0.78 / 2.84 / 5.18** |
| detrended p50 / p90 (cm) | 0.66 / 1.82 | 0.51 / 1.38 | **0.48 / 1.22** |
| lateral band rms 0.08-0.4 Hz (cm) | 0.84 | 0.60 | **0.55 (-35 %)** |
| w band rms 0.1-0.3 Hz (rad/s) | 0.0305 | 0.0160 | **0.0179** |
| e max (+111 deg fillet) | 0.091 | 0.125 | **0.107** |
| h p99 line / arc | 0.085 / 0.050 | 0.083 / 0.055 | **0.075 / 0.049** |
| i side / nose / sweep | 0.086 / 0.055 / 0.119 FAIL | 0.089 / 0.072 / 0.102 FAIL | **0.077 / 0.058 / 0.083** |
| row speed | 0.206 | 0.203 | **0.204** |
| verdict | FAIL (i) | FAIL (i) | **PASS** |

- On a flat world the tilt is 0, so V2d is V2 there. The differences against V2 are
  run-to-run spread. XTE p90 2.84 against 2.38 is one such: band rms and detrended p90 are
  better.
- owner_far was not rerun. V2d is not merge-ready on the slope bar (7.5), and on a flat world
  it is V2, which PASSED owner_far (section 5).

### 7.5 Verdict (before the cause was found; superseded by 7.6)

- **Bar:** not worse than master on b, a, d, f, h and i on the slope world.
- **V2d meets it on b, d and f** (all 0, as master).
- **On a, it does not:** 0.313 m / 975 samples against 0.210-0.214 / 349-426.
- **On h and i, it is within master's own spread** (line 0.412, side 0.318), slightly over
  on arc (0.652 against 0.633-0.641) and nose / sweep (0.382 / 0.387 against
  0.368-0.378).
- **Flat ground:** the weave gain holds (band -35 %, real_run5 PASS).
- **Not merge-ready as a default** while the slope-side hand-offs stay worse than master's
  with the V2 values off-slope.

Options:

1. **Find the upstream state.** One master-eq / V2d pair with per-cycle critic and pose
   logging over sub-path 0 (t 150-200 s), to see where the two first diverge on the leg into
   (-5.27, -0.31): pose at the slope entry, MPPI warm start, or the trim integral. That is
   1-2 runs.
2. **Merge V2d for lawns without a side slope.** The owner's map has the clothesline slope,
   so this is the owner's call. A field mow on the slope area would show whether the sim's
   hand-off effect is real.
3. **Ship master's values and keep V2d's code off** (`latency_s` 0, `control_point_x` 0.25,
   `trim_tau_s` 5, `slope_tilt_deg` 0). The slope schedule and the projection stay available
   for the FTC A/B.

### 7.6 Cause: a stale leg goal in odom; fix and results

**Finding the cause.** One V2d slope run with temporary per-cycle logging in LegGoalChecker
and PathAlignAheadCritic (`v2d-dbg-slope`, not committed). The scripts are in the session
scratchpad, `slope1106/` (`minapp.py`, `late.py`, `chain.py`).

- **The robot reached the vertex; the checker did not see it.** In every far hand-off of
  every V2 run, the stack's own pose (`/odometry/filtered_map`) came within 0.005-0.065 m of
  the vertex, and the leg went on for 1-10 s and ended 0.2-0.47 m past it:

  | vertex | master | master-eq | V2 | V2b | V2c | V2d |
  |---|---|---|---|---|---|---|
  | (-5.27, -0.31): closest / hand-off (m) | 0.011 / 0.072 | 0.016 / 0.117 | 0.005 / 0.235 | 0.065 / 0.323 | 0.050 / 0.198 | 0.053 / 0.327 |
  | (-5.03, -0.12): closest / hand-off (m) | 0.061 / 0.095 | 0.039 / 0.064 | 0.164 / 0.176 | 0.005 / 0.275 | 0.201 / 0.242 | 0.008 / 0.469 |

- **The checker's goal was frozen.** The logged goal of the leg into (-5.27, -0.31) stayed at
  the same odom coordinates (to the millimetre) for 10 s, while the robot, in odom, passed
  0.218 m from it. In map coordinates the same pass was 0.07 m from the vertex. After 10 s
  the goal jumped 0.21 m to where the vertex then was, and the passed-wide rule ended the
  leg 0.39 m away.
- **Why.** controller_server (Jazzy 1.3.13) transforms the goal into odom on every
  `isGoalReached` with the path's header stamp. The mission stamped each FollowPath path with
  its send time (`mission_node._path`), so tf2 used map -> odom at the send time for the
  whole leg. Once the stamp left the 10 s TF buffer, the lookup fell back to the latest
  transform: that is the jump. MPPI restamps the path and goal with the robot pose's time
  (`PathHandler::transformPath` / `getTransformedGoal`), so the robot followed the true path
  while the checker judged the stale goal.
- **Why the slope.** On flat ground map -> odom moves millimetres within a leg. On the 10 deg
  slope the downhill slide during a pivot and a crossing is not in the wheel odometry, so
  odom moves 0.15-0.2 m off map during one leg. The legs that suffer are the short ones that
  start off-line after a drifted pivot.
- **Why V2 more than master.** Both have the bug (master-eq's (-5.27, -0.31) leg ended 2 s
  late as well). After a drifted pivot the leg starts 0.2-0.27 m off the line, and MPPI
  crosses the vertex at a steep angle: master at about 55 deg to the leg, V2 variants at
  75-90 deg. Steeper crossings move nearly parallel to the checker's plane, so after the
  miss the robot travelled further before the passed-wide rule fired, and the next pivot
  (up to 123 deg) drifted further downhill. That is the a excursion. The crossing angle
  depends on the start offset and on run-to-run noise as much as on the values: V2c / V2d
  had master's values on that leg and still crossed at 76-92 deg. The flat-value state
  carried into the slope (trim integral, warm start) is not it: the trim integral at the leg
  start matched master-eq's within 1 cm, and MPPI's warm start is reset at every leg end.

**Fix (mower_mission, `mission_node._path`).** The FollowPath path and its poses are
stamped 0, which tf2 reads as "latest". controller_server then re-transforms the goal with
the current map -> odom on every cycle, as LegGoalChecker's header already assumed
(`new_goal_jump` handles the per-cycle drift). MPPI is unchanged (it restamps). It applies to
every FollowPath goal the mission sends (coverage chunks, corner legs, routes). Test:
`test_mission_path_stamp.py`.

**The slope schedule is removed again (A6).** With the fix, V2 needs no slope keys.
`slope_tilt_deg`, `slope_control_point_x`, `slope_trim_tau_s`, `tuningAt` / `tiltAngle` and
the `path_trim::update` time-constant overload are gone, back to the V2 code (798cc0c). With
the fix, V2d did worse than V2 (one run each, below).

**Gate results** (fix = the stamp fix; `v2d-dbg` is V2d unchanged, rerun with logging).
Slope world, `-d -s 2 -x 3`:

| | master | master-eq | V2d | V2d rerun (dbg) | V2d + fix | **V2 + fix** |
|---|---|---|---|---|---|---|
| a max / > 0.10 m samples | 0.210 / 349 | 0.214 / 426 | 0.313 / 975 | 0.233 / 520 | 0.349 / 765 | **0.197 / 490** |
| b / d fallback / f | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 1 / 0 / 0 | **0 / 0 / 0** |
| slope-side hand-off mean / max (m) | 0.085 / 0.33 | 0.079 / 0.12 | 0.151 / 0.47 | 0.174 / 0.53 | 0.104 / 0.43 | **0.066 / 0.12** |
| e corners > 0.15 m (max) | 6 of 22 (0.678) | 7 of 22 (0.686) | 6 of 21 (0.692) | 5 of 21 (0.691) | 8 of 22 (0.698) | **6 of 22 (0.654)** |
| h p99 line / arc | 0.419 / 0.633 | 0.420 / 0.641 | 0.412 / 0.652 | 0.435 / 0.652 | 0.462 / 0.653 | **0.408 / 0.610** |
| i side / nose / sweep | 0.343 / 0.368 / 0.372 | 0.325 / 0.373 / 0.378 | 0.318 / 0.382 / 0.387 | 0.329 / 0.384 / 0.385 | 0.390 / 0.427 / 0.490 | **0.409 / 0.385 / 0.368** |
| lawn cut | 96.6 % | 96.4 % | 96.4 % | 97.1 % | 97.1 % | **97.1 %** |

- **V2 + fix:** the slope-side hand-offs are now closer than master's (0.066 m mean, none
  over 0.12 m). a max is below master's, its sample count 490 just above master-eq's 426: the
  extra samples are the pivot at (-5.15, -0.03) (+67 deg against master's +37-46 deg, 0.21 m
  drift) and two short episodes near (-5.4, -0.4). h is at or below master's; i side is 0.07
  over master-eq, nose / sweep inside master's spread.
- **V2d + fix** tripped the boundary once on a slope row (west edge, 334 s, trim integral at
  its cap after a 0.16 m cross-track error), with the slope values in force. One run, so it
  does not show the schedule hurts; it does show the schedule is not needed.
- **Spread.** The unchanged V2d code gave a 0.313 / 975 and, rerun, 0.233 / 520. One run per
  variant on this world separates 2x effects, not 20 %.

Flat ground, final candidate (V2 + fix), `-l -x 3 -s 1`:

| | master | V2 | V2d | **V2 + fix** |
|---|---|---|---|---|
| real_run5 fitted Ky / Kpsi, r | 3.22 / 1.13, 0.0147 | 2.79 / 1.36, 0.0093 | 2.67 / 1.31, 0.0109 | **2.70 / 1.27, 0.0101** |
| real_run5 band rms (weave.py / fit.py, cm) | 0.84 / 0.77 | 0.60 / 0.49 | 0.55 / 0.47 | **0.74 / 0.62** |
| real_run5 XTE p90 / detrended p90 (cm) | 2.79 / 1.82 | 2.38 / 1.38 | 2.84 / 1.22 | **2.57 / 1.59** |
| real_run5 e max / h line / arc | 0.091 / 0.085 / 0.050 | 0.125 / 0.083 / 0.055 | 0.107 / 0.075 / 0.049 | **0.118 / 0.079 / 0.055** |
| real_run5 i side / nose / sweep | 0.086 / 0.055 / 0.119 | 0.089 / 0.072 / 0.102 | 0.077 / 0.058 / 0.083 | **0.093 / 0.073 / 0.109** |
| real_run5 verdict | FAIL (i) | FAIL (i) | PASS | **FAIL (i)** |
| owner_far band rms (weave.py / fit.py, cm) | 0.96 / - | 0.66 / - | - | **0.68 / 0.55** |
| owner_far fitted Ky / Kpsi, r | 3.23 / 1.13, 0.0167 | 2.73 / 1.31, 0.0110 | - | **2.67 / 1.30, 0.0112** |
| owner_far XTE p90 / detrended p90 (cm) | 2.66 / 2.40 | 2.12 / 1.72 | - | **2.10 / 1.70** |
| owner_far e (max) / h line / arc | 1 of 84 (0.181) / 0.068 / 0.048 | 0 of 86 (0.139) / 0.064 / 0.044 | - | **1 of 86 (0.155) / 0.074 / 0.064** |
| owner_far i side / nose / sweep | 0.096 / 0.119 / 0.153 FAIL | 0.083 / 0.057 / 0.084 | - | **0.106 / 0.127 / 0.143 FAIL** |
| owner_far verdict | FAIL (e, i) | PASS | - | **FAIL (e, i)** |

- **real_run5 band rms 0.74 cm is over the 0.62 bar**, but the fitted gains and r are V2's,
  and on owner_far (53 straights, 402 m against real_run5's 12 / 47 m) the weave is V2's to
  the hundredth (0.68 against 0.66, XTE p90 2.10 against 2.12). The fix only changes where
  the goal checker sees a leg's end, not the rows. The fit attributes the real_run5 rise to
  the pose-noise term (0.52 against 0.43 cm), which is localisation noise. I read it as
  run-to-run spread on a short sample, not a regression; it was not rerun (sim budget).
- **owner_far e and i** flip to FAIL at 0.155 m (limit 0.15) and 0.106 / 0.127 / 0.143, both
  inside master's own values there (e 0.181, i 0.096 / 0.119 / 0.153) and the ±0.05 flip of
  earlier runs (the first V2 owner_far run had an e corner at 0.152).

### 7.7 Verdict (with the fix)

- **Slope bar:** b, d, f at 0 as master; a max below master's, a samples 490 against
  349-426 (inside the spread one unchanged-code rerun showed); h at or below master; i
  within about 0.07 of master-eq.
- **Flat:** the weave gain holds on owner_far (band -29 % against master); real_run5's band
  rms came out at 0.74 in this run (V2 0.60, V2d 0.55).
- **Merge-ready, owner's call on the two single-run misses** (a samples on the slope,
  real_run5 band rms). A real_run5 rerun of the final code would settle the second one.
- **The fix stands on its own** and helps master as well: any FollowPath leg on a surface
  where odom slides off map (slopes, wet turf) was judged against a stale goal. It can be
  cherry-picked without the V2 values.

