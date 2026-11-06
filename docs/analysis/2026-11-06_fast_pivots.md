# Stock-like fast pivots (2026-11-06, ws/pivot1106)

Question: does the stock pivot rate (`MoveUtils::Rotation`,
`2026-11-06_stock_slope_handling.md` Q3) reduce the downhill pivot drift on side slopes?
Stock turns at `clamp(1.2 e, 0.4, 0.7)` rad/s with a 2 rad/s^2 ramp, stops within 5 deg
and never slows for tilt. We pivoted at 0.35 rad/s on flat ground and 0.3 on a side slope.

## Where the pivot rate was capped or assumed

| Place | Before | After |
|---|---|---|
| behavior_server Spin `max_rotational_vel` / `min_rotational_vel` / `rotational_acc_lim` (nav2_params.yaml) | 0.35 / 0.35 / 1.0 | 0.7 / 0.4 / 1.0 |
| mcu_node `angular_max` (host clamp before SpeedData) | 0.35 | 0.7 |
| cmd_vel_slew turn shaper `angular_max` (caps a snapped pivot only; a proper pivot passes unchanged) | 0.35 | 0.7 |
| cmd_vel_slew `pivot_max_radps` (live cap on pure rotations) | set by the mission: 0.35 flat, 0.3 at roll > 6 deg | not set (0 = off); kept as a field revert lever |
| mission `pivot_rate_flat_rad_s` / `pivot_rate_slope_rad_s` and the cmd_vel_slew parameter client | 0.35 / 0.3 | removed |
| mission `pivot_creep_spin_rps` (sizes the upslope pre-drive) | 0.3 | 0.6 (mean Spin rate with ramps) |
| mission `pivot_stop_lead_rad` | 0.06 | 0.10 |
| mower_sim `DriveParams.angular_max` / sim_node `angular_max` | 0.35 | 0.7 |

Left unchanged:

- cmd_vel_slew `max_angular_accel` / `decel` 1.0. This is a global rate that every
  controller command also passes through, so raising it would change line tracking on every
  world. On a pivot, 2 rad/s^2 would save about 0.1 s (step model, below). Spin's
  `rotational_acc_lim` stays equal to the slew decel, so the slew follows Spin's slow-down
  exactly.
- The turn shaper's pivot floor `min_inner_wheel_mps / (b/2)` = 0.3 rad/s and
  `pivot_floor_step`. Spin now stops from 0.4 rad/s: the inner wheel runs at 0.08 m/s,
  above the 0.06 floor, so there is still no slow tail under the floor (the reason for the
  2026-10-20 min = max design).
- velocity_smoother `max_velocity` theta 0.3 and MPPI / RPP angular limits. Controller
  turns stay at 0.3 rad/s. Only Spin pivots get faster.
- Mission direct pivots (`hard_boundary_pivot_rps`, the spin-drift re-pivot) at 0.3, and
  the docking `align_angular` / `max_angular` at 0.3.
- Wheel speeds: at 0.7 rad/s with b 0.40 they are +-0.14 m/s, well under `linear_max`
  0.3. No wheel or MCU limit below that exists on the host. Stock sends the same 0.7
  through the same SpeedData path.

Side effects of the higher host clamp (not gated):

- The GUI joystick (`cmd_vel_ws_relay` `max_angular` 1.0) can now spin at 0.7 rad/s
  instead of 0.35.
- `bumper_controller` `pid_max_angular` 0.5 is no longer clipped to 0.35.
- map_server `terrain_tilt_max_yaw_rate` 0.6 now skips tilt samples during a full-speed
  pivot. That is intended: centripetal acceleration pollutes the tilt.
- Spin's `simulate_ahead_time` 0.5 s now looks 20 deg ahead at 0.7 rad/s (was 10 deg). No
  "Collision Ahead" Spin aborts occurred in the gates below.

## The sim is honest about the ramp

`mower_sim` has no acceleration limit of its own. The wheels follow a first-order lag
(`wheel_tau_s` 0.15). The ramps come from the stack: `pivot_floor_step` steps 0 -> 0.3 (as
on the robot), cmd_vel_slew then ramps 0.3 -> 0.7 at 1 rad/s^2, and the wheel lag smooths
both. Nothing spins up for free. Terrain slip (`terrain.py`): a pivot loses
`pivot_yaw_loss` 0.2 of its yaw rate at 10 deg, and the body creeps downhill at
`creep_pivot` 0.045 m/s while the wheels turn faster than `act_ref` 0.06 m/s. Both pivot
rates are above `act_ref`, so **the sim creep is per second of pivoting, not per degree.**

## New pivot profile

Spin commands `clamp(sqrt(2 x 1.0 x remaining), 0.4, 0.7)`. That is 0.7 until 14 deg from
the target, then a slow-down to 0.4 at 4.6 deg, then a stop from 0.4. It is sent short by
`pivot_stop_lead_rad` 0.10.

Step model (20 Hz Spin and slew, 0.15 s wheel lag, ideal ground; `pivot_stop_lead_rad`
0.10 for the new profile, 0.06 for the old):

| Turn | 0.35 flat (old) | 0.7 / 0.4, 1 rad/s^2 (new) | 0.7 / 0.4, 2 rad/s^2 |
|---|---|---|---|
| 45 deg | 2.2 s | 1.35 s | 1.25 s |
| 90 deg | 4.5 s | 2.45 s | 2.35 s |
| 180 deg | 9.0 s | 4.7 s | 4.6 s |
| end error (lead 0.10) | +1 deg (lead 0.06) | +0.4..+1.1 deg | +0.4..+0.9 deg |

Stock: 2.9 s for 90 deg and 5.1 s for 180 deg (its slow-down starts at 33 deg). On the
-d slope run the coast after the Spin's stop was about 0.05-0.1 rad. After the Spin,
FollowPath usually turns on in the same direction at 0.3 rad/s, so the pivot ends short of
the leg heading, not past it. No overshoot problem was seen.

## Gate results (all `-x 3`, controller loop median 20 Hz)

| Run | a max / n | b | d | f | e | g pivot share | h p99 line / arc | i side / nose / sweep; transit | cut % | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| slope `-d -s 2`, master (simout_damp1106/master-slope) | 0.210 / 349 | 0 | 0/0/0 | 0 | 6 of 22 (0.678) | 230 / 1178 s (20 %) | 0.419 / 0.633 | 0.343 / 0.368 / 0.372; 0.369 | 96.6 | FAIL |
| slope `-d -s 2`, fast pivots | **0.167 / 117** | 0 | 0/0/0 | 0 | 6 of 22 (**0.447**) | **133 / 1054 s (13 %)** | **0.263 / 0.242** | **0.251 / 0.198 / 0.232**; 0.406 | 96.9 | FAIL |
| real_run5 `-l -s 1`, baseline (simout_corner1105/branch-l3, older plan) | 0.089 / 0 | 0 | 0/0/0 | 0 | 0 of 21 (0.091) | 246 / 993 s (25 %) | 0.085 / 0.050 | 0.086 / 0.055 / 0.119; 0.373 | 97.0 | FAIL (i) |
| real_run5 `-l -s 1`, fast pivots | 0.095 / 0 | 0 | 0/0/0 | 0 | 0 of 21 (0.116) | **144 / 915 s (16 %)** | 0.087 / 0.056 | 0.101 / 0.072 / 0.118; 0.383 | 97.6 | FAIL (i) |
| owner_far `-l -s 1 -t 4800`, baseline (simout_corner1105/branch-of-l3b, older plan) | 0.079 / 0 | 0 | 0/0/0 | 0 | 1 of 84 (0.181) | 531 / 3778 s (14 %) | 0.068 / 0.048 | 0.096 / 0.119 / 0.153; 0.344 | 99.2 | FAIL (e, i) |
| owner_far `-l -s 1 -t 4800`, fast pivots | 0.066 / 0 | 0 | 0/0/0 | 0 | **0 of 86 (0.141)** | **328 / 3598 s (9 %)** | 0.070 / 0.066 | 0.102 / 0.078 / 0.111; 0.308 | 98.1 | FAIL (i) |

Slope world, per Spin (ground truth, from `data.json`): 31 Spins took 136 s on master, at
a mean of 0.23 rad/s on the slope. With fast pivots, 24 Spins took 69 s, at a mean of 0.46
rad/s. The spin-drift guard (0.15 m) fired 5 times on master (0.15-0.31 m) and never with fast pivots. real_run5 `-l`: 1 on the baseline, 0 here. The
excursions out of the west edge drop from three episodes (max 0.210 m) to two (max 0.167 m). The
worst corner deviation drops from 0.678 m to 0.447 m.

The fast run's transit body figure (i transit 0.406, against 0.369) is the only slope-world
figure that got worse. It sits at the area 0 start transit (-5.01, 2.48), not at a slope
pivot.

## Verdict

It meets the bar on the slope world and does not regress the flat worlds in any way that matters:

- Slope world: a 0.210 m / 349 samples -> 0.167 m / 117. b, d and f stay 0. No STUCK.
  Spin-drift trips go from 5 to 0. Pivot share drops from 20 % to 13 %, and every
  cross-track and body figure except transit improves. The world still FAILs (a, e, h, i,
  j), as master does. The remaining excursions are the row and edge creep (1 cm/s while
  driving), not the pivots.
- real_run5 and owner_far: no new e failure (owner_far goes from 1 of 84 to 0 of 86).
  Pivot share drops from 25 % to 16 % and from 14 % to 9 %. Both baselines use an older
  plan, so the comparison is approximate.
- One marginal item: the i side figure is 0.101 m (real_run5) and 0.102 m (owner_far)
  against the 0.10 limit (baselines 0.086 and 0.096). Both i verdicts already failed on
  the baselines (sweep or transit). The owner_far worst sample is at (-32.86, -7.02) with
  side = sweep, so it is a pivot sweep. Those 2 mm are worth a look if i becomes a gate.
- owner_far lawn cut 99.2 % -> 98.1 % (plan 100 % both, 84 -> 86 corners: a different plan
  from the older baseline). One spin-drift trip at exactly 0.15 m (baseline 0).

The real_run5 `-d -s 1` run was not taken: the stops looked safe (no Spin aborts, coast
consistent with the 0.10 lead, FollowPath finishing the turn the same way).

## Field check needed

1. **Does field pivot creep scale with time or with angle?** The sim assumes time (a
   per-second creep while the wheels turn), so the gain above follows from the model. If
   turf creep is per degree swept (each wheel scuffs a fixed arc), faster pivots gain
   nothing on drift, but they still cut pivot time. Measure it on the west edge: a set of
   90 deg and 180 deg pivots at 0.35 (`ros2 param set /cmd_vel_slew pivot_max_radps 0.35`)
   and at 0.7 (`pivot_max_radps 0.0`), RTK displacement per pivot.
2. **Wheel stick-slip at higher rates.** The 2026-10-20 field pivot test showed IMU |w|
   0.1-0.5 rad/s under a 0.3 command. Check that a 0.7 command does not bounce harder, and
   that the 0.4 rad/s stop lands within the 5 deg the mission tolerates (field overshoot
   was 1-6 deg at 0.3).
3. Joystick spins are faster (0.7 rad/s cap). Tell the owner.

Revert without a redeploy: `ros2 param set /cmd_vel_slew pivot_max_radps 0.35`. Spin then
runs at 0.35 (slew cap). `pivot_stop_lead_rad` 0.10 is then a little long, so the pivot
ends about 2 deg short, and FollowPath finishes the turn.
