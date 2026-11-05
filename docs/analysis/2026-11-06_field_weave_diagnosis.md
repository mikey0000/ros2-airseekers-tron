# Field row weave: what it is made of, and why the sim over-predicts, 2026-11-06

Branch `ws/fid1106`. Follow-up to `2026-10-29_localisation_wander.md` and
`2026-11-05_row_weave_ftc.md`. Three controller levers have now fallen short of the sim's
prediction in the field:

| Lever | Sim | Field |
|---|---|---|
| B13-1 look-ahead control point | band rms -45 % | p90 -3 to -9 % |
| 6b/6c MPPI retune | p90 -8 % | in-band -32 %, max unchanged |
| GNSS smoothing | (not testable) | no effect |

Questions: is the field weave a limit cycle or a disturbance response? What does the actuator
chain do with small corrections? Where do the field and the sim loop differ, and what should
the sim add so it can judge controller changes?

**Verdict.**

1. **A disturbance response, not a limit cycle.** On straight rows the field lateral error
   has Gaussian amplitude statistics: kurtosis 2.9-3.3 (a sine gives 1.5) and envelope
   coefficient of variation 0.52-0.55 (a sine gives 0, narrowband noise 0.52). Its
   autocorrelation decays within one or two periods (damping ratio about 0.15-0.25). The sim
   has the same signature. There is no deadband, no minimum differential and no quantisation
   in the realised yaw rate, and the wheel floor and the turn shaper never act on a row.
2. **The actuator chain is already modelled.** The yaw the robot realises is linear in the
   command down to 0.005 rad/s. It has a constant trim of -0.005 rad/s and no left/right
   asymmetry beyond that trim. The apparent field "gain 0.88, delay 0.20 s" is a closed-loop
   identification artefact of the yaw disturbance. The sim's `-d` run, with the same ideal
   actuator, reads 0.92 / 0.25 s with the same fit.
3. **The sim's `-l` drive is missing the turf yaw disturbance.** The field body yaws
   0.026-0.028 rad/s rms (0.05-0.6 Hz) more than its commands explain. A `-l` run yaws
   0.002 rad/s more. The field weave is driven by that yaw disturbance, by measurement-like
   noise and by MPPI's own randomness. About three quarters of the `-l` weave is driven by
   its pose noise alone.
4. **Why the sim over-predicted B13-1: its baseline was worse than the field's.** MPPI's
   effective gains (the field fit `w = -Ky e - Kpsi psi`) after B13-1 agree between field and
   sim: field 11-04 Ky 3.47 / Kpsi 1.08; sim 3.22-3.39 / 1.13-1.24. Before B13-1 they do not.
   The field 10-28 robot (also before the 6b/6c retune) already acted with Kpsi 1.04. The sim
   with the control point set to 0 acts with Kpsi 0.87 and weaves 2.4 cm band rms, against the
   field's 1.6. In the sim the control point raised Kpsi by 43 %, in the field by 4 %. So the
   sim's large B13-1 gain mostly repaired a damping deficit that the field never had. Adding
   the field's yaw disturbance does not change that: the replay under the new model still
   gives -58 %. A candidate cause is the sim's track lag. The same fit recovers the sim's set
   0.08 m in closed loop, but reads 0.038-0.049 m in the field. The lag costs heading damping,
   which is what the control point puts back.
5. **Sim model change:** the field localisation model (`-l`) now turns on the existing
   per-wheel slip noise at the field-fitted level: std 0.05, correlation time 0.3 s
   (`field_slip_std`, `field_slip_tau_s`). This adds the missing turf yaw disturbance. The
   sim's yaw-rate spectra, its actuator fit and its MPPI residual move to the field's values,
   and the row weave rises from 0.84 to 1.00 cm band rms (field 1.5-1.6 on the measured
   position; section 5). The master gate now fails criterion a (0.116 m, 29 samples)
   besides i.

## 1. Data and method

| Data | Straights | Notes |
|---|---|---|
| Field 2026-10-28 (`bags/mower/bags/20261028-023252`) | 60, median 28 s, 0.217 m/s | before B13-1 |
| Field 2026-11-04 (bag `20261103-231722`, area 3) | 34, median 14 s, 0.215 m/s | master with B13-1 |
| Sim master `real_run5 -l -x 3 -s 1` (`simout_corner1105/branch-l3`) | 12, median 20 s | baseline |
| Sim master `owner_far -l -x 3 -s 1` (`branch-of-l3`) | 53 | baseline |
| Sim master `real_run5 -d -s 1` (`branch-d1`) | 12 | wheel bias 0.04, slip 0.05 / 1 s, pivot 0.85 |
| Sim `control_point_x` 0.35 (`simout_look1105/cp35-l3`) | 12 | lever contrast |

The field straights are the FTC note's (`weave1105/scripts/phase1.py`): RTK pieces while
MOWING and driving forward, split by Douglas-Peucker. The sim straights are `weave.py`'s,
judged on ground truth. Every statistic below uses the same code on both. The scripts are in
the session scratchpad `fid1106/scripts/`:

| Script | What it does |
|---|---|
| `extract2.py` | per-wheel motor telemetry, shaper status, MCU speed topics |
| `chain.py` | stage-to-stage gain and delay |
| `simsegs.py` | sim straights in the field format |
| `nature.py` | spectra, kurtosis, envelope, autocorrelation |
| `fit.py` | closed-loop identification and the residual loop model |
| `deadband.py` | small-w response |
| `dspec.py`, `dvy_src.py` | residual spectra and sources |
| `levers.py`, `b131.py`, `ftcpred.py` | lever sensitivity per input |

What the bags hold for the velocity chain:

| Stage | Topic | Recorded? |
|---|---|---|
| MPPI output | `/cmd_vel_nav_raw` | **no** (the sim records it) |
| velocity_smoother output | `/cmd_vel_nav_smoothed` | **no** |
| collision_monitor output (twist_mux lane) | `/cmd_vel_nav` | yes, 20 Hz |
| cmd_vel_slew output (what mcu_node gets) | `/cmd_vel` | yes, 20 Hz |
| mcu_node's command | `/mcu/commanded_speed` | yes, 5 Hz sample |
| wheel-measured body twist (MCU SpeedData) | `/mcu/measured_speed` | yes, 5 Hz sample of a float |
| per-wheel motor speed | `/mower_sensor_info` `left_motor` / `right_motor` `speed_rpm` | yes, but integer rpm (0.011 m/s), refreshed about 1 Hz and about 0.6 s stale: no use for dynamics |
| body yaw rate | `/odometry/filtered` twist wz (IMU rate fused with wheel rate) | yes, 20 Hz; `/imu/data` is not recorded |
| turn shaper decision | `/cmd_vel_slew/shape_status` | yes, on change |

## 2. Limit cycle or disturbance response

Lateral error of the axle on the straights, band-passed 0.08-0.4 Hz. The field uses RTK minus
the lever along the fused yaw; the sim uses truth.

| | Field 10-28 | Field 11-04 | Sim `-l` master |
|---|---|---|---|
| band rms (cm) | 1.64 | 1.55 | 0.77 |
| kurtosis (sine 1.5, Gaussian 3) | 2.90 | 3.29 | 3.47 |
| envelope CV (sine 0, narrowband Gaussian 0.52) | 0.52 | 0.55 | 0.56 |
| PSD peak | 0.161 Hz (6.2 s) | 0.083 Hz (window-limited, 14 s rows) | 0.078 Hz |
| autocorrelation at the first extremes | -0.61 at 3.0 s, +0.18 at 5.9 s | -0.42 at 3.3 s, -0.23 at 8 s | -0.54 at 4.4 s, +0.25 at 10.7 s |
| damping ratio (PSD width beyond the window's own, autocorrelation decay) | 0.15-0.17 | about 0.25 (short rows) | about 0.2 |
| per-straight band rms p10 / p50 / p90 (cm) | 1.06 / 1.61 / 2.03 | 0.99 / 1.45 / 2.16 | 0.42 / 0.70 / 1.00 |

A limit cycle would have a constant envelope, a kurtosis near 1.5 and an autocorrelation that
does not decay along a row. The field has none of these. Its weave is a lightly damped linear
mode (period 6-6.6 s) excited by broadband inputs. The amplitude varies from row to row by a
factor of two (CV 0.24-0.33), as a random excitation does.

The w spectra on the same straights (rad/s rms per band):

| Band (Hz) | Field 10-28 nav / gyro | Field 11-04 nav / gyro | Sim `-l` nav / gyro |
|---|---|---|---|
| 0.05-0.1 | 0.013 / 0.009 | 0.019 / 0.013 | 0.007 / 0.007 |
| 0.1-0.3 | 0.054 / 0.049 | 0.046 / 0.042 | 0.018 / 0.018 |
| 0.3-1 | 0.023 / 0.035 | 0.030 / 0.039 | 0.017 / 0.016 |
| 1-3 | 0.016 / **0.044** | 0.013 / **0.046** | 0.009 / 0.005 |
| 3-10 | 0.009 / 0.016 | 0.008 / 0.015 | 0.007 / 0.002 |

Above 0.3 Hz the field body yaws two to nine times more than it is commanded to. The sim
body yaws exactly as commanded. This is the first sign of a ground yaw disturbance. Above
1 Hz it is too fast to move the robot sideways (0.045 rad/s at 2 Hz is 0.2 deg). Its
low-frequency part is quantified in section 4.

## 3. Actuator chain

Stage pairs on the straights: gain and phase delay (0.08-0.5 Hz, cross-spectrum),
coherence. Field 10-28 / 11-04; sim `-l` / `-d`.

| Stage | Field gain | Field delay (s) | Field coh | Sim gain | Sim delay (s) |
|---|---|---|---|---|---|
| MPPI -> smoother -> collision_monitor (`_raw` -> `/cmd_vel_nav`) | not recorded | not recorded | | 1.00 / 1.00 | 0.038 / 0.028 |
| cmd_vel_slew (`/cmd_vel_nav` -> `/cmd_vel`) | 1.00 / 1.00 | 0.014 / 0.050 | 1.00 | 0.99 / 1.00 | 0.031 / 0.040 |
| `/cmd_vel` -> MCU measured w (5 Hz hold adds about 0.1 s) | 1.03 / 1.04 | 0.22 / 0.18 (xcorr 0.12 / 0.11) | 0.96-0.98 | (no MCU stage) | |
| MCU measured w -> gyro wz | 0.87 / 0.86 | 0.01 / 0.07 | 0.94-0.96 | | |
| `/cmd_vel` -> gyro wz | 0.91 / 0.90 | 0.22 / 0.24 | 0.91-0.96 | **1.00 / 0.94** | **0.13 / 0.27** |

Small corrections, realised yaw rate (gyro 0.2 s later, both low-passed at 0.6 Hz) against
the command the MCU got:

| Command bin (rad/s) | -0.10 | -0.04 | -0.015 | -0.005 | +0.005 | +0.015 | +0.04 | +0.10 |
|---|---|---|---|---|---|---|---|---|
| Field 10-28 realised | -0.087 | -0.040 | -0.021 | -0.013 | -0.001 | +0.006 | +0.027 | +0.079 |
| Field 11-04 realised | -0.086 | -0.036 | -0.018 | -0.009 | -0.002 | +0.011 | +0.027 | +0.075 |
| Sim `-l` realised | | -0.039 | -0.015 | -0.005 | +0.005 | +0.015 | +0.038 | +0.098 |

Findings, quantified:

| Candidate | Field | Verdict |
|---|---|---|
| Latency per stage | slew 0.01-0.05 s; MCU wheels about 0.1-0.12 s; cmd -> body 0.22-0.24 s phase | the sim's 0.15 s wheel lag plus its chain gives 0.13 s ideal and 0.27 s with the `-d` disturbance in the same fit; nothing unmodelled |
| Deadband / minimum realised differential | the response passes through the trim at every bin down to 0.005 rad/s; slope 0.82-0.86 | **none** |
| Quantisation | SpeedData is a float (20 862 distinct measured w values); the 1 rpm (0.011 m/s) steps exist only in the motor telemetry | **none in the drive** |
| Turn shaper / wheel floor | inner wheel on the rows at least 0.068 / 0.089 m/s (floor 0.06); shape status `pass` on 96-97 % of the straights (the rest is the entry arc) | **never acts on a row** |
| velocity_smoother deadband (0.002 rad/s) | `/cmd_vel_nav` exactly 0 on 2.0 % of 11-04 samples (8.4 % on 10-28, before 6c) | negligible |
| Left / right asymmetry | constant yaw trim -0.005 rad/s (wheel bias about 0.009); once that is removed, left and right slopes agree (0.84-0.86) | trim only; the controller removes it as a standing offset |
| Gain below 1 | gyro / command 0.82-0.90, MCU measured w / command 1.03; pivots 1.02-1.03 (10-20 note) | closed-loop artefact: the sim `-d` run, actuator gain 1.03, fits 0.92-0.94 |

The low fitted gain and the long fitted delay come from the yaw disturbance. The controller
answers each disturbance with a command of the opposite sign, so a regression of yaw rate on
command reads less gain and more delay than the drive has. The sim with the same ideal
actuator and a field-sized disturbance (`-d`) reproduces both numbers. There is no
actuator-side fix to make in cmd_vel_slew or in the MCU floor handling for the rows.

## 4. Closed-loop fit, field and sim

`fit.py`: each straight band-passed 0.05-0.6 Hz at 10 Hz. Three pieces are fitted, and each
one's residual is kept:

- **controller:** `w_nav = -Ky e_seen - Kpsi psi_seen` (best of 0-0.2 s delay), residual `r`;
- **actuator:** `gyro = G lag(tau) delay(d) [w_cmd]`, residual `d_w`, the yaw disturbance;
- **track:** `e' = v psi - Ls gyro + d_vy`, where `d_vy` is the lateral velocity no heading
  explains (the field's RTK is advanced by its 0.1 s latency).

On the sim the method recovers the model it was given: G 1.03 (0.40 / 0.39), lag 0.15 s,
Ls 0.080-0.085 (0.026 + 0.055).

| | Field 10-28 | Field 11-04 | Sim `-l` real_run5 | Sim `-l` owner_far | Sim `-d` real_run5 |
|---|---|---|---|---|---|
| Ky / Kpsi (R²) | 4.47 / 1.04 (0.91) | 3.47 / 1.08 (0.85) | 3.22 / 1.13 (0.85) | 3.24 / 1.12 (0.83) | 3.09 / 1.12 (0.87) |
| MPPI residual r (rad/s) | 0.026 | 0.030 | 0.015 | 0.017 | 0.023 |
| actuator G, delay, lag | 0.88, 0.20 s, 0 | 0.88, 0.20 s, 0 | 1.03, 0, 0.15 s | 1.03, 0, 0.15 s | 0.92, 0.25 s, 0 |
| **yaw disturbance d_w (rad/s)** | **0.026** | **0.028** | **0.002** | **0.003** | **0.026** |
| d_w per octave 0.05 / 0.1 / 0.2 / 0.4 Hz | 0.004 / 0.007 / 0.011 / 0.011 | 0.005 / 0.008 / 0.013 / 0.011 | 0.000 / 0.001 / 0.001 / 0.001 | | 0.007 / 0.009 / 0.011 / 0.007 |
| Ls (m), 0.1-0.3 Hz | 0.049 | 0.038 | 0.085 | 0.084 | 0.084 |
| d_vy (cm/s) from the measured position (RTK / sim map pose) | 1.23 | 1.32 | 0.97 | 1.01 | 1.01 |
| map pose - RTK (field) / - truth (sim), cm | 0.29 | 0.46 | 0.76 | 0.81 | 0.74 |
| lateral band rms 0.08-0.4, truth / measured (cm) | - / 1.64 | - / 1.58 | 0.77 / 1.07 | 0.86 / 1.18 | 1.28 / 1.51 |

The loop model closes the fitted controller, actuator and track around each data set's own
residuals: one input at a time, then all together. It gives the lateral band rms in cm. It
overshoots the field a little (as on 10-29) but ranks the inputs:

| Input | Field 10-28 | Field 11-04 | Sim `-l` | Sim `-d` |
|---|---|---|---|---|
| MPPI residual r | 1.07 | 0.65 | 0.31 | 0.68 |
| yaw disturbance d_w | 0.83 | 0.66 | **0.05** | 0.83 |
| lateral residual d_vy | 1.40 | 1.02 | 0.30 | 0.44 |
| pose error | 0.66 | 0.40 | **0.72** | 0.76 |
| all | 2.45 (measured 1.58) | 1.76 (1.50) | 0.88 (0.77) | 1.50 (1.28) |

- **The yaw disturbance is real and unmodelled in `-l`.** It is ten times the sim's. Its
  spectrum is flat to about 0.4 Hz. It does not come from the heading estimate, which
  follows the RTK track with coherence 0.94-0.98 (FTC note, phase 1). It is not the actuator
  either: the wheels report the commanded differential (MCU measured w / command 1.03), but
  the body does not follow it. That is turf slip. The `-d` run's per-wheel slip (std 0.05)
  gives the same level, but its 1 s correlation time puts too much of it below 0.1 Hz and too
  little above 0.4 Hz. With 0.3 s the slip has the field's shape (table above and section 5).
- **The lateral residual d_vy is measurement-like.** It is uncorrelated with d_w (|r| < 0.05
  at lag 0, at most 0.21 at any lag up to 1.5 s; d_w explains 9-10 % of it), weakly
  correlated with the receiver's sigma per straight (r 0.24-0.26), and its displacement
  spectrum is red. Computed from the sim's map pose the same way, it is 0.97-1.01 cm/s with
  the same octave shape against the field's 1.23-1.32. The sim's loc_drift noise already
  stands in for it. Whether it is RTK error or real side creep cannot be told without an
  independent lateral reference. For the measured weave the two are equivalent.
- **Track lag.** The fitted Ls (turn centre plus track lag, after the 0.1 s fix latency) is
  0.038-0.049 m in the field against the sim's 0.077-0.085 m. The fit recovers the sim's set
  0.081 m within 5 % in closed loop, with and without slip noise. That points to a field track
  lag of about 0.015-0.025 m rather than 0.055 m. The 0.055 m came from the 10-29 regression,
  which that note itself calls biased in closed loop. It also does not separate the latency
  term cleanly. It is not changed here: the run budget was spent on the slip validation and
  the B13-1 replay (sections 5 and 7).

### Which input each lever acts on

The effective-gain fit shows what a control point does to MPPI. Ky goes down, and Kpsi goes up
in the sim but hardly in the field:

| | Ky | Kpsi |
|---|---|---|
| field 10-28 (before B13-1 and 6b/6c) -> 11-04 | 4.47 -> 3.47 | 1.04 -> 1.08 |
| sim, new model, control point 0 -> 0.25 (section 5) | 4.18 -> 3.39 | 0.87 -> 1.24 |
| sim, old model, control point 0.25 -> 0.35 | 3.22 -> 2.74 | 1.13 -> 1.16 |

The fitted loop answers each input differently (`b131.py`; change in lateral band rms):

| Input (field 10-28 / 11-04 / sim `-l`) | Ky x 0.78 | Kpsi x 1.3 (heading damping) | both |
|---|---|---|---|
| pose error | -46 / -32 / -28 % | -39 / -24 / -29 % | -62 / -46 / -48 % |
| yaw disturbance d_w | -21 / -10 / -11 % | -30 / -19 / -17 % | -40 / -25 / -25 % |
| MPPI residual r | -29 / -6 / -10 % | -36 / -23 / -24 % | -47 / -26 / -28 % |

A lower lateral gain mostly follows less of the pose noise. Against the ground disturbance and
MPPI's own randomness it is worth a third to a half as much. Heading damping acts on every
input. In the field, B13-1 delivered mainly the Ky change (Kpsi +4 %), which fits the small
field result. In the sim it delivered the Kpsi change too.

Applied to each data set's own inputs, the sim's B13-1 gain change (4.18 / 0.87 -> 3.39 /
1.24) is worth -49 to -71 % (`lagtest.py`). The sim therefore predicts B13-1 correctly for
the gains it produced. What it got wrong was the gains: the field's MPPI never had the sim's
pre-B13-1 Kpsi of 0.87. In the loop, the field's fitted track lag (0.045 m instead of 0.08)
lowers the control-point-0 weave by 22-38 % on every input set. That is the direction needed,
but it is offline only (section 7).

## 5. Sim model change and validation

`mower_sim`:

- `kinematics.with_field_slip(p, std, tau_s, seed)` returns `p` with the existing
  `disturb_slip_*` per-wheel slip process turned on. A run that sets its own slip (`-d`) is
  left unchanged, and `std` 0 turns it off.
- `sim_node`: with `loc_model: field` it applies `field_slip_std` 0.05 and
  `field_slip_tau_s` 0.3, seeded with `loc_drift_seed`. The values are also arguments in
  `launch/sim.launch.py`; `sim_gate.sh ... -- field_slip_std:=0.0` restores the old ideal
  `-l` drive.
- Tests (`test_kinematics.py`): the field slip fills in only when the run has none; open
  loop at 0.21 m/s it adds 0.038 rad/s ± 20 % of yaw-rate noise with a 0.3 s correlation
  time, and the encoders do not see it.

Open loop at 0.21 m/s, cut into 20 s pieces and processed like the rows, the slip adds
0.006 / 0.008 / 0.011 / 0.009 rad/s of yaw rate in the 0.05 / 0.1 / 0.2 / 0.4-0.6 Hz octaves,
0.026 rad/s in all. The field residual is 0.004-0.005 / 0.007-0.008 / 0.011-0.013 / 0.011,
0.026-0.028 in all. The `-d` correlation time of 1 s gives 0.010 / 0.012 / 0.011 / 0.007:
too much of it is slow.

### Validation: one master run, `real_run5 -l -x 3 -s 1`

`simout_fid1106/fid-l3`; bags pruned, `data.json` and `verdict.txt` kept. The controller loop
held 20 Hz sim (p10 20), "desired rate of 60" was in launch.log, the power profile was
`performance`, and launch.log shows `ground disturbance ON: ... slip std 0.050 (tau 0.3 s)`.

| | Field 10-28 / 11-04 | Sim master before (`branch-l3`) | **Sim master, field slip (`fid-l3`)** |
|---|---|---|---|
| weave.py XTE p50 / p90 / max (cm) | | 1.05 / 2.79 / 5.47 | 1.21 / 3.02 / 4.67 |
| weave.py detrended p50 / p90 (cm) | wheel_cal 0.84 / 2.09; 0.72 / 1.91 | 0.66 / 1.82 | 0.88 / 2.00 |
| weave.py band rms truth (cm) | | 0.84 | **1.00** |
| lateral band rms, truth / measured position (cm) | - / 1.64; - / 1.58 | 0.77 / 1.07 | 0.91 / **1.22** |
| w band 0.1-0.3 Hz nav (rad/s) | 0.054; 0.046 | 0.018 | 0.034 |
| gyro / nav w, 1-3 Hz | 2.8; 3.5 | 0.6 | 1.5 |
| yaw disturbance d_w (rad/s) | 0.026; 0.028 | 0.002 | **0.023** |
| actuator fit G, delay | 0.88, 0.20 s | 1.03, lag 0.15 s | **0.95, 0.25 s** |
| MPPI residual r (rad/s) | 0.026; 0.030 | 0.015 | 0.018 |
| Ky / Kpsi | 4.47 / 1.04; 3.47 / 1.08 | 3.22 / 1.13 | 3.39 / 1.24 |
| kurtosis, envelope CV | 2.9-3.3, 0.52-0.55 | 3.47, 0.56 | 2.64, 0.48 |
| a max out / > 0.10 m | | 0.089 / 0 | **0.116 / 29 FAIL** |
| e corners > 0.15 m (max) | | 0 (0.091) | 0 (0.147) |
| h p99 line / arc | | 0.085 / 0.050 | 0.078 / 0.046 |
| i side / nose / sweep | | 0.086 / 0.055 / 0.119 FAIL | 0.116 / 0.058 / 0.065 FAIL |
| lawn cut | | 97.0 % | 97.7 % |
| verdict | | FAIL (i) | FAIL (a, i) |

- **The drive now has the field's disturbance.** The yaw residual, the actuator fit's
  artefact (gain below 1, longer delay) and the excess body yaw above 0.3 Hz all moved most
  of the way to the field. MPPI's in-band w doubled.
- **The weave rose 19 %** (weave.py band rms 0.84 -> 1.00; measured position 1.07 -> 1.22,
  field 1.58-1.64). The sim still sits about 25 % below the field. The fitted loop puts the
  rest in MPPI's larger field residual and the larger field d_vy. Neither of them has a
  model element yet.
- **Statistics stay those of a disturbance response** (kurtosis 2.6, CV 0.48).
- **Criterion a now fails.** It is a 1.1 s excursion of 0.116 m at (-0.52, 3.17) during
  MOWING, at the site i's side worst already flags. The field bag of 10-28 shows map-pose
  `outside_m` > 0.10 m in 131 samples (10-29 note), so a margin this size is realistic.
  Gate comparisons across this change need the same model on both sides: use
  `-- field_slip_std:=0.0` to compare against older `-l` baselines.

### B13-1 replay under the new model (second run)

`simout_fid1106/fid-cp0-l3`: the same run with the coverage `PathAlignAheadCritic`
`control_point_x` 0 (a temporary edit, not committed; this is the pre-B13-1 critic). Loop
20 Hz, "desired rate of 60" present.

| | cp 0 | cp 0.25 (master) |
|---|---|---|
| weave.py band rms / XTE p90 / max (cm) | 2.36 / 4.57 / 9.80 | 1.00 / 3.02 / 4.67 |
| w band rms (weave.py) | 0.099 | 0.043 |
| Ky / Kpsi | 4.18 / 0.87 | 3.39 / 1.24 |
| verdict | FAIL (e 0.186, h 0.108 / 0.113, i) | FAIL (a, i) |

B13-1 is still -58 % in the sim with the field's disturbance. Field 10-28, before B13-1,
weaved 1.64 cm band rms at Kpsi 1.04. The sim's pre-B13-1 controller is the outlier (see
section 4, "Which input each lever acts on").

## 6. What this means for the controller work

- **Damping, not bandwidth.** A lever that lowers MPPI's effective Ky mainly suppresses pose
  noise. The field weave is driven by ground yaw disturbance, by measurement-like noise and by
  MPPI's randomness. Raising the effective heading gain helps against all of them. Lowering
  Ky trades rejection of ground disturbances for noise rejection. Any MPPI critic change
  should be judged by the effective (Ky, Kpsi) that `fit.py` reads from its sim run, not by
  the `-l` band rms alone. A change that raises Kpsi in the sim may not do so in the field:
  B13-1 did in the sim (+43 %) and not in the field (+4 %). The field bag gives the check:
  run `fit.py` on the next mow and compare the fitted Kpsi.
- **MPPI's own randomness is a field-sized input.** It is 0.026-0.030 rad/s in the field and
  0.015 in the old `-l`. It grows with the disturbance (`-d` 0.023). Its loop share
  (0.65-1.07 cm) is as large as the yaw disturbance's. A deterministic row law (FTC) removes
  it. MPPI critic tuning (more samples, temporal smoothing) can only shrink it.
- **Latency compensation.** In the fitted field loop, 0.1 s less pose-to-command delay is
  worth 18-34 % of the band rms, across all inputs. The field fix latency (0.1 s) is real, and
  the sim models it as an unstamped delay (10-29 §6.2). A compensation that predicts the
  pose forward by the measured latency acts on the same delay in both, so the sim can judge
  it.
- **FTC.** With the FTC row gains (w = -2.5 e - (1.2 + 2.5 lead) psi, r = 0), the fitted
  loop on each data set's own inputs predicts -46 to -64 % band rms. The field and the sim
  predictions agree here, because FTC raises the heading gain (1.64-1.83 against MPPI's
  1.05-1.13) instead of lowering Ky, and it has no sampling residual. This is an offline
  estimate. The FTC sim gate was not rerun (owner).
- **Actuator side: no fix indicated.** No deadband, quantisation, shaper or floor action on
  the rows. The -0.005 rad/s trim is a standing offset the controller already removes. The
  MCU's wheel loop follows the command at gain 1.03 and about 0.1 s.

## 7. Open

- **Track lag.** The field fit reads 0.038-0.049 m for turn centre plus track lag; the sim
  has 0.081 m. Lowering `field_slip_lead_m` to about 0.02 m is the next fidelity step. It is
  untested, because the two-run budget went to the slip validation and the B13-1 replay. In
  the loop model it lowers the control-point-0 weave by 22-38 %, which is the direction of
  the field's better pre-B13-1 baseline. Check before changing it: the 6.4 s period that the
  0.055 m was chosen to match. The current period estimates are window-limited (12-20 s rows).
- **MPPI residual.** In the field it is 0.026-0.030 rad/s; in the sim it is 0.018 even with
  the slip. Part of it is MPPI answering the high-frequency yaw (1-3 Hz gyro 0.045 rad/s in
  the field, 0.015 in the sim now).

- `/cmd_vel_nav_raw` and `/cmd_vel_nav_smoothed` are not in the field bag ring, so the MPPI
  and smoother latencies are known from the sim only. Recording `/cmd_vel_nav_raw` at 20 Hz
  would let the next field bag separate MPPI's residual from the smoother.
- `/imu/data` is not recorded. The gyro used here is `/odometry/filtered` wz, which fuses the
  wheel rate. The analysis depends on it only through its coherence with the RTK track.
- d_vy's split between RTK error and physical side creep needs an independent lateral
  reference (a camera on the cut edge, or a second antenna).
