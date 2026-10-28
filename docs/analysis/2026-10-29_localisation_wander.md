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

## 4. Why no change on this branch (at the time of the analysis; see §5)

- The one localisation change with a large modelled effect (item 2) trades pose fidelity
  for smoothness. Its accuracy cost is 1-2 cm p90, plus bias, unless the odometry scale is
  fixed first. The rule is never to trade boundary accuracy for smoothness.
- The sim cannot exercise it (no EKF in mower_sim).
- The changes that can be tested (items 1 and 3) are controller and sim-model work outside
  this localisation scope. They also change the gate's reference behaviour, so the owner
  should decide on them.
- No sim runs were made: there was no change to compare against master.

## 5. Follow-up on `ws/loc1029` (2026-10-29): B9, B13-3, B13-2

The owner approved B9 and B13 (2) and (3) the same day. What changed:

- **B9:** `mcu_node` `odom_linear_scale` 1.06.
- **B13-3:** the sim `loc_model` field model.
- **B13-2:** the `gnss_smoothing` switch, off by default.

### 5.1 B13-3: the sim's field localisation model

`loc_model:=field` is now the default whenever `loc_drift` is on (`-l` / `-d`).
`-M legacy` gives the 2026-10-16 model. Details are in docs/simulation.md
"Localisation drift". The model adds:

- **Fix latency.** The map position is the GNSS antenna's position 0.1 s ago, minus the
  0.098 m navsat lever along the current yaw.
- **Rotation centre.** The sim drive turns about a point 0.026 m ahead of base_link.
- **Track lag.** At speed the ground track lags the heading by 0.055 m.
- **No RTK jumps.** The OU error and jitter are unchanged.
- **Odometry.** Odom sees no lateral motion and has a 1 % scale error.

The robot's 0.098 m lever is unchanged.

Validation: master controller, `real_run5`, the 17 straight runs of `weave.py`. The
mechanism columns come from `simmech.py`, which applies the §1 regression to
/odometry/filtered_map against the ground truth and against DR.

| | Field 10-28 | Sim legacy `-l` (w7-head-l-r2) | Sim field `-l` x3 | Sim field, no track lag | Sim field `-d -s 1 -x 1` (2 runs) |
|---|---|---|---|---|---|
| w spectral peak (s) | 6.4 | 4.3 | **6.4** | 3.2 | 6.4 |
| XTE on truth p50 / p90 (cm) | 1.3 / 3.3-3.5 | 1.26 / 2.93 | 1.19 / 2.97 | 1.03 / 2.37 | 1.55-1.61 / 4.05-4.14 |
| w band rms 0.1-0.3 Hz (rad/s) | | 0.044 | 0.053 | 0.031 | 0.089 (run 1) |
| seen - DR band rms (cm) | 1.00 | 1.02 | 0.89 | 0.72 | 1.08-1.21 |
| random part, band rms (cm) | 0.66-0.82 | 0.73 | 0.50 | 0.48 | 0.53-0.54 |
| b, seen vs DR (m) | 0.107 | **0.121** | 0.166 | 0.138 | 0.111-0.122 |
| b, seen vs truth (m) | | 0.126 | 0.090 | 0.117 | 0.036-0.047 |

What the table shows:

- **The §1 regression is biased in closed loop.**
  - The legacy model has no in-loop lag by construction, yet the regression reads b about
    0.12 m on it (w6-base-l: 0.12 as well).
  - The controller turns in response to the pose noise, so the heading correlates with
    the error.
  - The field's 0.107 m is therefore not a clean measure of the in-loop lag. Only its
    independently measured parts are firm: the 0.026 m lever (pivots) and the
    0.02-0.025 m latency (velocity cross-correlation).
- **The weave's frequency decides the model.**
  - Only the model with the track lag weaves at the field's 6.4 s period. Without the
    lag the w peak sits at 3.2 s and the cross-track is too small.
  - So the 0.055 m stays, as a physical lag in the drive.
- **Band power.** The random part is 0.50 cm against the field's 0.66-0.82 cm upper
  bound. The total seen - DR band power is 0.89-1.08 cm against 1.00. The OU size was
  left at 1.2 cm, which keeps the open-loop p90 at 1.57 cm against the field's 1.49.
  Open-loop check of the field model: |lat| p50 0.65 / p90 1.57 cm, band 0.59 cm. The
  legacy model gives 0.96 / 2.63 / 1.03 cm.
- **Gate baselines.** These are in docs/simulation.md. With the field model the master
  controller fails:
  - e and h arc on the saturated fillet after sub-path 0's 94 deg corner (§11 of the MPPI
    study already flagged that fillet; the track lag widens it);
  - with `-d`, also **a**: 0.13-0.16 m outside at the SW K-turn, in both runs (legacy
    0.087).

  The field bag's own `outside_m` (map pose) exceeds 0.10 m on edge passes too (131
  samples, max 0.20 m), so the new model is showing a real margin problem.

### 5.2 B13-2: GNSS smoothing in ekf_map (field A/B switch)

**Replay.** The 10-28 bag was replayed through the real robot_localization `ekf_node`,
on sim time, x10, three filter instances at once:

- `/odom` is rebuilt from `/odometry/filtered` vx (× 1.06, as with B9) and the yaw rate;
- `/imu/data_aligned` is rebuilt from the fused yaw;
- `/odometry/gps` is the RTK base in the map frame, with the 3 cm floor;
- the `ekf_map` parameters are the committed ones, plus the overlay.

The scripts are in the session scratchpad (`loc1029/rl/`).

**Finding: the x/y process noise alone does nothing in robot_localization.** With
mcu_node's odom twist covariance (vx 0.02 m²/s², vy not observed), the velocity
uncertainty lets the position drift faster than the x/y process noise does. At q 1e-4 the
pose stayed within 0.9 cm p90 of RTK and the modelled weave went *up* 4 %. The §3
emulation had assumed exact odometry increments.

The switch therefore also:

- tightens `mcu_node` `odom_vx_variance` / `odom_vy_variance` to 1e-4;
- fuses vy (= 0) in ekf_map (`odom0_config`).

With those, robot_localization reproduces the emulation:

| ekf_map (tight odom covariance) | Pose - RTK on straights p50 / p90 (cm) | While moving p90 (cm) | Along-track bias on straights (cm) | Lateral p90 (cm) | In-loop b (m) | Random band rms (cm) | Model weave (cm) |
|---|---|---|---|---|---|---|---|
| q 0.05 (today) | 0.10 / 0.24 | 0.25 | -0.00 | 0.15 | 0.099 | 0.82 | 2.16 |
| **q 1e-4 (the switch)** | 0.83 / 1.70 | 2.03 | -0.10 | 1.40 | 0.067 | 0.57 | 1.15 (**-47 %**) |
| q 2e-5 | 1.24 / 2.50 | 3.12 | -0.23 | 2.15 | 0.033 | 0.50 | 0.80 (-63 %) |
| q 1e-4, odom NOT scaled (no B9) | 1.44 / 2.28 | 2.53 | **-1.18** | 1.44 | 0.073 | 0.56 | 1.19 (-45 %) |
| q 1e-4, loose odom covariance | 0.42 / 0.87 | 0.98 | -0.02 | 0.51 | 0.102 | 0.84 | 2.28 (+4 %) |

- **The lag.** Along-track bias divided by speed is 0.00 s with B9 and 0.06 s without it.
  The fixed 0.1 s fix latency is not compensated by either setting.
- **B9 is the prerequisite.** Without it the pose runs 1.2 cm behind RTK along the track.
- **Phase gating (smooth only on MOWING straights, raw RTK when docking, undocking or
  near the boundary): not done.** It is too complex for this step, for three reasons:
  - robot_localization reads `process_noise_covariance` once at start. There is no
    runtime switch.
  - The runtime equivalent would be to inflate the GNSS measurement covariance per phase.
    That also inflates ekf_map's reported pose covariance, which `localization_monitor`
    reads. It would also need a separate `/odometry/gps` copy, because heading_aligner
    reads that covariance.
  - Boundary proximity needs a signed inside-distance, which map_server does not publish
    (`outside_m` is 0 inside).

  So `gnss_smoothing` is one switch for the whole run. That is acceptable for an A/B, not
  for a default.
- **Other effects of the switch:**
  - ekf_node gets the tighter vx covariance as well;
  - after a GNSS gap, or in RTK float (0.4 m sigma floor), the map pose re-converges
    over seconds instead of at once;
  - `initial_estimate_covariance` x/y 1 m² lets the first fixes after boot snap the pose
    from (0, 0).

### 5.3 Field A/B procedure (owner)

1. **Before the A/B: deploy this branch as is.** That includes B9 (`odom_linear_scale`
   1.06) and keeps `gnss_smoothing` off.
2. **Mow A (off).** Do one normal full mow of Area 1 with the blade on and RTK fixed.
3. **Run `scripts/wheel_cal.py <session>` on mow A.** Check that "ground / fused odom
   speed" reads 1.00 ± 0.01. That is B9's acceptance; if it reads otherwise, adjust
   `odom_linear_scale` first. Note:
   - "weave: RTK lateral about the swath line" p50 / p90 (10-28: 0.84 / 2.09 cm);
   - "map pose - RTK, in-line rms" (10-28: 0.56 cm).
4. **Mow B (on).** Start the stack with `gnss_smoothing:=true` (`mower.launch.py`
   argument, e.g. `scripts/run_stack.sh gnss_smoothing:=true` or the service's launch
   arguments). Mow the same area on the same day, with similar turf and the blade on.
   - Watch the first undock and the dock return closely.
   - Stop the A/B if the docking misses, or if the GUI pose visibly lags in turns.
5. **Run `wheel_cal.py` on mow B and compare.**
   - **Success:** weave p90 down by 30 % or more, and no new boundary events.
   - **Expected cost:** "map pose - RTK" rises to about 1.5-2 cm.
   - **The sim gate cannot judge this.** Compare the boundary and docking items from the
     mission log and bag on both mows:
     - `/map_server_node/outside_m` maximum;
     - hard-boundary trips;
     - the docking attempts and the final docked offset.
   - The §1 decomposition (`straight1028/decomp.py` pipeline) gives the in-band split
     when needed.
6. **Revert.** Restart without the argument. Nothing is persisted.

## 6. B13-1: heading damping with a control point ahead (`ws/damp1029`, 2026-10-29)

The coverage MPPI (`FollowCoveragePathMPPI`) now scores path alignment at a point
0.25 m ahead of base_link. The new critic is `PathAlignAheadCritic` (mowgli_nav2_plugins),
and its parameter is `control_point_x: 0.25`. The transit and reverse controllers are
unchanged.

### 6.1 Why this approach

- **The lever is in the controller.** The weave is the lightly damped lateral mode (§1).
  PathAlignCritic matches each sampled pose to the path point at the same arc length.
  It gives `w ≈ -4.9 lat - 0.77 yaw`, i.e. it tracks a point only 0.157 m ahead.
- **The lag sits behind the rotation centre.** On the drive the track lag (0.055 m) and
  the turn centre (0.026 m ahead of base_link) are physical. The fix latency adds about
  0.02 m. All three put what the controller sees behind the rotation centre, which takes
  most of that 0.157 m.
- **The fix.** Scoring `(x + d cos yaw, y + d sin yaw)` against the path point `d` further
  along adds `d × heading` to every lateral error. That is the classic look-ahead control
  point. With d 0.25 the modelled true damping ratio goes from about 0.12 to about 0.7.
- **It is robust to the latency.** It does not depend on the latency value or on the
  stamps. It only adds heading damping.
- **How the critic is built.**
  - It is Nav2 1.3.13's PathAlignCritic loop with the control point.
  - It has the same parameters and the same gating (goal threshold, `offset_from_furthest`,
    occupancy ratio, invalid points).
  - With `d > 0` it uses the path up to d beyond the furthest reached point. Without that,
    the ahead points at the end of the horizon would be clamped to that point, which
    would bias speed.
  - A gtest checks that `control_point_x: 0` reproduces the upstream loop bit for bit, on
    random rollouts over a curved path with an invalid point.
- **The cost on arcs.** On an arc of radius R a control point d ahead holds base_link
  `d²/2R` inside: 3 cm at R 1 m, 6 cm on the 0.5 m fillets. That is towards the area,
  and it is what fixed the wide fillets.

### 6.2 What was not done

- **Latency compensation at the source (B13 candidate 1).**
  - On the robot `um960_node` stamps `/fix` with `now()` when its 10 Hz timer publishes
    the latest sentence. The stamp is therefore receive / publish time, not measurement
    time.
  - The sim cannot test a fix there. `loc_drift` replaces ekf_map and publishes the
    0.1 s-old antenna position, stamped now. Its latency is invisible to stamps, so
    re-stamping or `smooth_lagged_data` would show nothing in the gate.
  - It is also worth only about 0.02 m of the lag (§1), and the UM960 driver is due to
    be replaced by the RTCM gateway.
  - Left for later. It would need a field A/B.
- **Critic retunes (candidate 3) and slew rate damping (candidate 4).** Not tried. The
  control point passed a, e and h on the first setting, so a weaker or riskier lever was
  not needed.
- **control_point_x 0.35.** One `-l -x 3` run. Straights improved further (XTE p90 1.95 cm,
  max 3.45). But e failed with 0.28 m and a with 0.108. The run was INVALID: controller
  loop 7 Hz sim, because the host was loaded. So it is inconclusive, and 0.25 was kept.
  The params test bounds `control_point_x` to 0.1-0.4.

### 6.3 Results (sim, `real_run5`)

Weave columns are `weave.py` / `simmech.py` on the 17 straights (truth vs the sent plan):

- XTE p50 / p90 / max;
- detrended true lateral p50 / p90 and band rms (0.08-0.4 Hz);
- `/cmd_vel_nav_raw` w rms in 0.1-0.3 Hz and its spectral peak.

| Run | a max out / > 0.10 | e max | h p99 line / arc | i side excess | XTE p50 / p90 / max (cm) | detrended lat p50 / p90, band rms (cm) | w band rms, peak | Verdict |
|---|---|---|---|---|---|---|---|---|
| field `-l -x 3`, master (loc1029) | 0.096 / 0 | 0.228 (2 fail) | 0.095 / 0.236 | 0.261 | 1.19 / 2.97 / 5.35 | 0.87 / 2.12, 1.19 | 0.053, 6.4 s | FAIL e h i |
| **field `-l -x 3`, d 0.25** | **0.056 / 0** | **0.091** | **0.068 / 0.036** | 0.102 | 0.97 / 2.34 / 5.32 | 0.53 / 1.33, 0.66 | 0.028, 4.3 s | FAIL i only |
| field `-d -s 1 -x 1`, master (loc1029, 2 runs) | 0.155 / 153; 0.132 / 114 | 0.168 (2 fail); 0.144 | 0.116 / 0.179; 0.103 / 0.138 | 0.213; 0.177 | 1.61 / 4.14 / 10.27; 1.55 / 4.05 / 9.58 | 1.47 / 3.49, 1.90; 1.42 / 3.60, 1.98 | 0.089; 0.100, 6.4 s | FAIL a e h i; a h i |
| **field `-d -s 1 -x 1`, d 0.25 (r2)** | **0.079 / 0** | **0.133** | **0.089 / 0.063** | 0.108 | 1.06 / 3.01 / 5.15 | 0.85 / 1.93, 1.02 | 0.055, 4.3 s | FAIL i only |
| field `-d -s 1 -x 1`, d 0.25 (r1, INVALID: loop 17 Hz, host load) | 0.098 / 0 | 0.116 | 0.080 / 0.058 | 0.137 | 1.05 / 2.94 / 5.48 | 0.80 / 1.86, 1.01 | 0.054, 4.3 s | (INVALID) |
| legacy `-M legacy -d -s 1 -x 1`, master (w5-final) | 0.087 | 0.133 | 0.091 arc | fail | 1.24 / 3.79 / 10.79 | | 0.064 | FAIL i only |
| **legacy `-M legacy -d -s 1 -x 1`, d 0.25** | **0.060 / 0** | 0.104 | 0.057 / 0.095 | 0.122 | 1.24 / 2.88 / 5.25 | | 0.043, 4.3 s | FAIL i only |

What the runs show:

- **Criterion a passes with margin on the field model.**
  - `-d` gives 0.079 m against 0.13-0.16, and `-l` gives 0.056.
  - The SW K-turn edge arc (sub-path 0, poses 269-290, R about 0.6 m) is now tracked
    inside the area.
  - The remaining excursion is the pivot at vertex 292, 0.06-0.08 m beyond the vertex,
    which lies on the edge. That is the pivot-stop overshoot, not tracking.
- **e and h pass.** The 94 deg fillet after sub-path 0's corner went from 0.168-0.228 to
  0.034-0.09, and h arc p99 from 0.14-0.24 to 0.04-0.06.
- **The weave is damped.**
  - Truth band rms is down 45-48 % (1.19 -> 0.66 cm with `-l`, 1.9-2.0 -> 1.02 cm with `-d`).
  - Detrended p90 is down 37-46 %, and the straight XTE max halves with `-d`.
  - w in the weave band is down 40-47 %.
  - The 6.4 s peak is gone. The residual w spectrum peaks at 4.3 s, with half the power.
  - XTE p90 against the plan falls less (-21 % / -26 %), because it includes the
    per-swath standing offsets. Those are not what the fix addresses.
- **i still fails, as on every baseline.** That includes the legacy "only i".
  - Edge-lap side excess is now 0.10-0.12 m, down from 0.18-0.26.
  - The worst is on the west edge arc (-5.5..-5.75, 1.9-2.0), where the body runs about
    6 cm outside its plan on a R about 1.4 m right-hand arc.
  - That is a steady arc offset (the drive's crab with the track lag, plus the
    disturbance's wheel bias), not the weave. It is left for a separate item.
- **No regression on the legacy model.** The verdict is the same (only i). a, e, h and the
  straights are all better.
- **CPU.** The critic costs the same as PathAlignCritic plus a few path points per
  cycle. `-l -x 3` held 20 Hz sim. The x1 runs' 17-18 Hz medians coincided with host
  load: the 0.35 run fell to 7 Hz at the same time. The rollout count is unchanged.

### 6.4 Field check (owner)

1. Deploy `ws/damp1029`. It is the coverage MPPI change only, and needs a rebuild of
   `mowgli_nav2_plugins`.
2. Mow Area 1 as in §5.3 mow A, with `gnss_smoothing` off. This is a separate A/B from
   B13-2: don't combine the two on the first mow.
3. Run `wheel_cal.py` on the session. Compare against 10-28:
   - "weave: RTK lateral about the swath line" p50 / p90: 0.84 / 2.09 cm. Success is
     -30 % or more.
   - the mission log's boundary events and `/map_server_node/outside_m` max on the edge
     laps.
   - the pivot vertices on the edge: overshoot at pivots is unchanged by this.
4. Watch the fillets and the edge-lap arcs. They now run slightly inside (2-6 cm on tight
   arcs).
5. **Revert:** in `nav2_params.yaml`, put `PathAlignCritic` back in the
   `FollowCoveragePathMPPI` critics list and rename its block. Alternatively, set
   `control_point_x: 0.0`, which is equivalent.
