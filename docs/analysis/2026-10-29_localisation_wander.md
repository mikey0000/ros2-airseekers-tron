# Localisation wander and the straight-line weave, 2026-10-29

Offline analysis of the 2026-10-28 field bag (`bags/mower/bags/20261028-023252`). No mower
access and no field test (rain). Follow-up to `2026-10-15_mppi_study.md` §11. That section
concluded that "most of the field weave is the controller following the EKF map pose's
1-3 cm wander". This note tests that claim.

**Verdict.** The robot really weaves. The localisation does not invent the weave: what the
controller sees is the RTK fix, almost unfiltered. The weave is the controller's own
lateral mode (0.15-0.16 Hz), which is lightly damped. Two things excite it or reduce its
damping:

- a pose error that moves *with* the weave, proportional to heading: equivalent to a
  measured point 0.10 m behind the rotation centre, or 0.5 s of position lag;
- RTK noise in the weave band, 0.66-0.82 cm rms at most.

The heading aligner, the map -> odom TF and the wheel odometry do not matter. No
localisation change is both clearly justified and testable in the sim today, so this
branch carries only this note.

## Data and method

The data are the 66 straight mowing swaths of the bag (rows from the 10-28 straight-line
analysis, 1 s trimmed at the start and 0.5 s at the end), about 440 m at 0.21 m/s,
resampled to 20 Hz. Lateral position is measured from the RTK-fitted swath line and
detrended per swath. "Band" means 0.08-0.4 Hz; the weave sits at 0.15 Hz.

| Signal | What it is |
|---|---|
| `ant` | RTK antenna (`/fix`) |
| `rtkb` | RTK base: antenna minus the 0.098 m lever along the fused yaw (what navsat does) |
| `map` | `/odometry/filtered_map` |
| `seen` | What Nav2 uses: the newest map -> odom (zero-order hold at 10 Hz) composed with odom at 20 Hz |
| `dr` | Dead reckoning: `/odometry/filtered` vx along the raw IMU heading. The raw heading is the fused yaw minus the heading_aligner offset, rebuilt from `/heading_aligner/status` (steps at the exact COG times, plus the rate ramp). Initial heading, yaw-rate bias, offset and lateral drift are fitted per swath. This is independent of GNSS inside the band, and is the short-term truth for the rotation centre. |

The scripts are in the session scratchpad (`straight1028/decomp.py`, `regress2.py`,
`lag.py`, `lag2.py`, `pivot.py`, `simcmp.py`, `loopsim*.py`, `ekfemu*.py`, `emuloop*.py`).
They use the `data.npz` extraction made by `extract.py`.

## 1. What the controller sees

| | All bands: abs p50 / p90 / p99 (cm) | Band rms (cm) | >0.4 Hz rms (cm) |
|---|---|---|---|
| RTK base lateral (the weave) | 1.27 / 3.26 / 5.80 | 1.81 | |
| DR lateral (wheels + IMU) | 1.28 / 3.26 / 5.74 | 1.48 | |
| map - rtkb | 0.14 / 0.38 / 0.74 | **0.08** | 0.24 |
| seen - map (TF hold, 10 vs 20 Hz) | 0.07 / 0.32 / 0.75 | **0.05** | 0.21 |
| seen - dr | 1.04 / 2.78 / 5.24 | **1.00** | 0.52 |

- **The robot weaves physically.**
  - The raw IMU heading oscillates 4.0 deg rms in the band (p90 7 deg).
  - DR reproduces the RTK lateral weave: coherence 0.96 at 0.15-0.2 Hz.
  - The weave is not a pose artefact.
- **ekf_map passes RTK straight through.** In the emulation below the steady Kalman gain
  is 0.86 per fix, because the x/y process noise is robot_localization's default 0.05 m²/s
  against the 3 cm fixed-RTK sigma.
- **(a) GNSS through the EKF: the only material term.** seen - dr (1.0 cm band rms)
  splits as follows:
  - **A deterministic part proportional to heading.** The fit is
    `e = -0.107 m · psi + 0.06 · lat`: seen lateral ≈ true lateral − 0.107 m × heading.
    That is equivalent to the controller tracking a point about 0.10 m *behind* the
    rotation centre, or a position lag of about 0.5 s. It decomposes into:
    - lever arm: pivots put the antenna 0.072 m ahead of the rotation centre (29 pivots
      > 60 deg, IQR 0.064-0.081, about 4 mm fit residual), but navsat removes 0.098 m,
      which accounts for 0.026 m;
    - fix latency of about 0.1 s (velocity-vector match against odom: minimum at +0.07 s
      relative to odom; speed xcorr: +0.12 s relative to the MCU), which accounts for
      0.02-0.025 m;
    - an unexplained remainder of about 0.055 m, probably the ground track lagging the
      heading (tyre/castor slip). That part is physical, not a pose error.
  - **A random part.** It is 0.66 cm band rms (band-limited fit) to 0.82 cm (per-swath
    fit), with abs p50 0.55 / p90 1.49 cm over 0.05-1 Hz. It is an upper bound: it also
    holds DR and slip errors.
  - **High-frequency RTK jitter** of 0.52 cm rms above 0.4 Hz. This matches the receiver's
    own sigma of 6.7 mm.
- **(b) Heading aligner: negligible.**
  - The controller's yaw minus the raw IMU heading is 0.08 deg rms in the band
    (p90 0.26 deg), against 4.0 deg of real heading weave.
  - COG updates come every 3.4 s. The step seen in the EKF yaw is p50 0.11 deg, p90 0.28.
  - Through a 0.1 m lever or a 1 m lookahead, that is 0.2-2 mm.
  - The steps correlate *positively* with the next w (r 0.58). The controller would
    respond with the opposite sign. So the steps are the weave aliased into the 0.6 m COG
    windows, not a cause of it.
- **(c) map -> odom TF: negligible.** seen - map is 0.05 cm band rms. The map -> odom yaw
  correction is p50 0.026 deg per 0.1 s.
- **(d) Wheel odom / IMU: no lateral contribution.**
  - At K 0.86 they barely enter the map pose.
  - Odometry reads **6.3 % short** of RTK on the straights (odom/RTK distance 0.937,
    p10-p90 0.929-0.947). The 10-22 wheel-scale note found 4.9 % on `/odom`; its
    `odom_linear_scale` 1.049 is still not implemented.
  - This matters for any smoothing (§3), not for the weave itself.

### Causality

Commanded w regressed on the band-passed signals the controller sees:

- **The fit:** `w ≈ -4.9 · lat_seen - 0.77 · yaw_seen` (rad/s, m, rad), R² 0.89. §11 had
  -3.98 / -0.57.
- **The lateral mode this implies:**
  - natural frequency √(v·Ky) = √(0.21 × 4.9) = 1.01 rad/s, which is **0.16 Hz**: the
    observed 6.4 s weave;
  - seen damping ratio 0.38.
- **On the true state the heading gain is much lower:** 0.77 − 0.107 × 4.9 ≈ 0.25. The
  regression on true lat / psi gives 0.26. The true damping ratio is therefore about
  **0.12**, before the 0.15-0.25 s command-to-wheel delay.
- **Shares of w variance:**

  | Source | Share of w variance |
  |---|---|
  | True lateral and heading | 86 % |
  | Random pose error | 7 % (band) to 13 % (0.05-1 Hz) |
  | Aligner yaw error | 0.5 % |

- **A lead/lag test cannot settle causality here.** `xcorr(seen - dr -> w)` peaks at 0.71
  with w 1.5 s later. But most of that error is itself proportional to the heading
  (in-loop), so the test is circular.
- **Linear closed-loop model** (unicycle at 0.21 m/s, the fitted gains, 0.25 s delay,
  driven by the field's own random error per swath):

  | Effective lag b (m) | Weave, band rms (cm) |
  |---|---|
  | 0 | 0.99 |
  | 0.05 | 1.34 |
  | 0.107 | 2.42 |

  The field value is 1.45 cm, so the model overshoots, but the ordering holds.
- **Halving the random error halves the weave. Removing the in-loop lag matters about as
  much again.** Raising the heading gain from 0.87 to 1.2 halves it too.

The field weave is the controller's lightly damped lateral mode. The pose error lowers its
damping (a lag that moves with the weave) and excites it (RTK noise). The localisation
does not drive it by itself: the robot does not weave *because* a free-running pose
wanders, and part of the "wander" exists *because* the robot weaves.

## 2. The sim's loc_drift against the field

The loc_drift model was driven along a straight at 0.21 m/s with the same detrending and
band, over 60 runs of 30 s.

| | Lateral abs p50 / p90 (cm) | Band rms (cm) | 0.1 s lateral step p50 / p90 (mm) |
|---|---|---|---|
| loc_drift defaults | 0.90 / 2.47 | 0.99 | 2.9 / 7.2 |
| loc_drift without RTK jumps | 0.64 / 1.57 | 0.62 | 2.8 / 6.8 |
| loc_drift × 0.5 | 0.45 / 1.23 | 0.50 | 1.4 / 3.6 |
| field random part (upper bound) | 0.55 / 1.49 (0.05-1 Hz) | 0.66-0.82 | map pose 2.2 / 5.8, including true motion |
| field total seen - dr | 1.04 / 2.78 | 1.00 | |

- **The noise is about 1.2-1.5× the field.** Most of the excess is the Poisson jumps
  (2.5 cm sigma every 4 s), which are not visible in this bag.
- **The total band power matches** (1.03 vs 1.00 cm). That is why `-l` / `-d` reproduce
  the field weave amplitude.
- **The mechanism differs.** The sim injects all of it as exogenous noise. In the field,
  about 0.75 cm of it is the in-loop heading-proportional lag, which loc_drift lacks
  entirely (no fix latency, the sim's antenna lever equals navsat's, no slip).
- **Consequence for tuning:**
  - In the sim, pose smoothing looks like the main lever. In the field, damping matters as
    much.
  - Heading-damping critics (6a2, PathAngle) cannot show their field benefit in the sim.
    §11 already noted this.

## 3. Candidate fixes, ranked

Each fix is ranked by expected effect on the weave, risk, and what it needs before it can
be tested.

1. **Controller heading damping or a lookahead point.** Not localisation, but the largest
   lever.
   - Raise the effective heading gain relative to the lateral gain on straights. The model
     halves the weave at Kpsi 0.87 -> 1.2.
   - Alternatively, track a point about 0.2 m ahead of the rotation centre, which cancels
     the 0.1 m behind.
   - Boundary risk is low: the localisation is unchanged.
   - It needs a sim that has the in-loop lag (item 3), or a field A/B. 6a1 (PathAlign
     orientations) showed that a heading term can trade into standing offsets; watch the
     per-swath bias.
2. **Smooth GNSS in ekf_map, after the odometry scale is fixed.**
   - **What:** lower the ekf_map x/y process noise (keep ekf_node's), so that wheels and
     IMU carry the in-band motion and RTK the low frequencies.
   - **Offline emulation:** ekf_map x/y fed with the field odom increments and RTK base,
     odom scale corrected, run through the loop model:

     | Process noise q (m²/s) | Time constant (s) | Lag b (m) | Random error, band rms (cm) | Model weave (cm) |
     |---|---|---|---|---|
     | 0.05 (current) | 0.02 | 0.100 | 0.82 | 2.17 |
     | 1e-4 | 0.9 | 0.065 | 0.57 | 1.12 (-48 %) |
     | 2e-5 | 2 | 0.027 | 0.49 | 0.76 (-65 %) |

   - **Costs:**
     - The pose departs from raw RTK by p50 0.9 / p90 2.0 cm at q 1e-4 and 1.4 / 3.3 cm
       at q 2e-5, part noise removal and part lag.
     - The along-track bias is scale error × v × time constant: 1.3 cm per second of time
       constant with today's 6.3 % odom error.
     - Convergence is slow from robot_localization's 1e-9 initial covariance; that needs
       `initial_estimate_covariance`.
     - Boundary and docking accuracy depend on that pose.
   - **Testing:** the sim cannot test it, because sim_node replaces ekf_map with loc_drift.
     Prerequisite: `odom_linear_scale` (wheel-cal note, 1.049-1.067). Then a field A/B at
     q 1e-4 with the boundary and docking metrics. Medium risk, potentially large effect.
3. **Sim realism.**
   - **What:**
     - give `/fix` about 0.1 s latency in the sim;
     - give the sim antenna an effective lever of 0.072 m while navsat keeps 0.098;
     - optionally a slip lag;
     - cut loc_drift's jumps (`jump_rate_hz` about 0.05 or `jump_std` about 0.01).
   - **Why:** this moves the sim's weave from noise-driven to damping-driven, as in the
     field, so item 1 can be tuned in the sim.
   - **Risk:** none on the robot. The gate baselines would have to be re-taken.
4. **GNSS latency compensation.** Stamp `/fix` about 0.1 s earlier and set ekf_map
   `smooth_lagged_data` / `history_length`. That removes about 0.02 of the 0.10 m lag. The
   effect is small, and the latency estimate is loose (0.07-0.15 s on straights; pivots
   suggest more). Low-medium risk; the sim cannot test it.
5. **Lever arm 0.098 -> 0.072?** It is worth 0.026 m of the lag. But the owner measured
   0.10, and a grass pivot centre need not be the axle. Do not change it without a static
   measurement. Pivoting on concrete and fitting the antenna circle would settle it.
6. **Heading aligner smoothing** (spread COG steps over time): no measurable benefit (0.5 %
   of w variance). Not worth the change. A longer COG window than 0.6 m would stop the
   weave aliasing into the offset (innovations of ±3.5 deg). That is hygiene, not a weave
   fix.
7. **Faster or smoothed map -> odom:** nothing to gain (0.05 cm band).

## 4. Why no change on this branch

- The one localisation change with a large modelled effect (item 2) trades pose fidelity
  for smoothness. Its accuracy cost is 1-2 cm p90, plus bias, unless the odometry scale is
  fixed first. The rule is never to trade boundary accuracy for smoothness.
- The sim cannot exercise it (no EKF in mower_sim).
- The changes that can be tested (items 1 and 3) are controller and sim-model work outside
  this localisation scope. They also change the gate's reference behaviour, so the owner
  should decide on them.
- No sim runs were made: there was no change to compare against master.
