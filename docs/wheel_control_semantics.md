# Wheel control semantics: what the vendor host did, and what our driver should do

Status: 2026-10-06. **Section 5 policy implemented** in `mower_mcu_driver/mcu_node.py` (`_speed_tick`): one frame per `/cmd_vel`, 3 zeros @ 100 ms on stop/timeout/interlock then silence, clamps ±0.3, brake and host PID removed, `speed_stream_enabled` = debug-only 20 Hz stream (default false). `mcu_protocol_spec.md` §9 corrected. Sources: `native_decompile/dec/mower_base_node/mower_base_node.c` (MBN),
`native_decompile/dec/libpid_controller/libpid_controller.so.c` (PID), `mcu_protocol_spec.md` 10b/10c,
`mcu_decompile/chassis/chassis.hex.c`. Confidence labels: **[decompile]** read directly from code,
**[tap]** measured on the wire, **[inferred]**, **[unknown]**.

## 1. Vendor host TX policy for SpeedData (module 4)

**The vendor host is a pure pass-through. One SpeedData frame goes out per received `/cmd_vel` message, with no timer, no PID, no timeout and no zero sent on its own.**

| Step | Where | What |
|---|---|---|
| Subscribe `/cmd_vel` (Twist, queue 10) and `/cmd_vel_stamped` | MBN 18943-18982 | → `MowerBase::twistCallbackROS` |
| `MowerBase::twistCallbackROS` | MBN 19688-19740 | forwards to `MowerOdom::twistCallbackROS`, then sets `is_cmd_moving` (`this+0x58b`, atomics `+0x5d8/+0x5d9`, `DevStatus::setBotMovingCmdStatus`) = `|lin.x| > 1e-6 || |ang.z| > 1e-6` |
| `MowerOdom::twistCallbackROS` | MBN 249660-249715 | builds `ProtocolData{module=4, 8 bytes = float32 lin.x, float32 ang.z}` and pushes it on the send queue. Logs "send_queue is full" if the push fails. There is no clamp, no scaling and no state |
| `PowerManager::wheelVelCB` | MBN 283422-283447 | **does not transmit.** It only sets `PowerManager+0x9c` = `|lin| >= 0.1 || |ang| >= 0.1` (a "moving" flag used for power and charging logic) |

The consequences:
- **Rate:** whatever rate the publisher of `/cmd_vel` uses. There is no `createTimer` on the command path, and none of the timers at MBN 18261, 20501, 246834 and 292832 call it [decompile]. The timer callbacks were not traced individually, but no other caller of the SpeedData send path exists (`grep SendSpeedData` finds only the SDK wrappers at MBN 395412 and 402584).
- **Idle:** if nobody publishes `/cmd_vel`, nothing goes out. This matches the tap: heartbeat every 100 ms, IMU at about 44 Hz, no SpeedData [tap].
- **Stop:** the host never sends a zero on its own. A zero reaches the MCU only if an upstream node publishes a zero Twist. Upstream nodes do this: `PidControllerROS::stop()` publishes exactly **one** zero Twist (PID 25658-25700). The vendor planner nodes are not in this binary [unknown].
- **`is_moving`** (measured): `MowerOdom+0x568` = `|measured lin| >= 0.05 || |imu yaw-rate| >= 0.05` (MBN 249566-249572).

**Where the PID sits.** `mower_controller::PidControllerROS` is **not** in the wheel loop. It is a position-level motion primitive with `rotate(angle)` and `moveStraight(dist)`. It subscribes to `/odom` and publishes `/cmd_vel` (PID 24993 and 25018). It loops at `ros::Rate(20.0)` (PID ~25351), and its exit tolerance is `|err| < 0.05`. The output is clamped to ±0.3 by `linear_*` / `angular_*` in `base.yaml`, with `rotate_timeout` / `linear_timeout` = 20 s. It ends with `stop()`, which publishes one zero Twist. In `mower_base_node` its only owner is `BumperController` (MBN 141516, member at `+0xc0`), which runs the bumper back-off manoeuvre. Its output re-enters through `/cmd_vel` and is passed through like any other command.

**Correction to `mcu_protocol_spec.md` §9.** The claim that "the host runs a wheel-level PID against wheel-velocity feedback and adjusts what it sends" is wrong. Both the spec and the current `mcu_node.py` docstrings repeat it, and both should be fixed.

## 2. MCU handling of SpeedData (0,0) and of silence

- **Cutter (gateway) board** [decompile, spec 10c]: it ignores SpeedData while `lift` or `stop` is set. After about 200 ticks without host traffic, the heartbeat path zeroes the stored setpoint.
- **Is a 0,0 frame "coast", "hold" or "integrator keeps running"?** [unknown]
  - The chassis image is stripped. The float-heavy function I inspected (`FUN_08012184`, chassis.hex.c 2615-2990) is a ramp/self-test state machine. Its `FUN_08012130` is a slew limiter that steps the setpoint toward a target. It is not the runtime speed loop.
  - I found no deadband constant and no explicit 0,0 special case.
  - The "0,0 = coast" statement in `mcu_node.py` comments is a hypothesis drawn from behaviour, not a decompile result.
- **What silence does** [inferred]: the setpoint is held. The last value received is kept until the cutter's heartbeat-timeout path zeroes it. Our heartbeat counts as host traffic, so in practice the last commanded value persists. The vendor got away with this only because its producers always end with a zero.

## 3. What "measured SpeedData" (module 4, MCU→host) is

- Format: `float32 linear, float32 angular` at about 100 Hz [tap, decompile MBN 249525-249528].
- The vendor host uses it as follows:
  - Republishes both fields as `/wheel_vel`.
  - Integrates **only `linear`** into the odom position with **IMU yaw** (`x += v·cos(yaw)·dt`, MBN 249556-249563).
  - Takes the odom twist angular from the IMU (`+0x1e0 = +0x4b8`), **not** from the measured angular.
- The vendor would not dead-reckon odom from a command echo, so measured linear is very likely encoder-derived [inferred, medium-high].
- The vendor never consumes measured angular for control or odom. Its sign, units and noise are **completely unvalidated** [unknown].
- The driver comment "MCU echoes our previous command with about one cycle of delay" was never demonstrated on wheels-up hardware. Treat it as unproven.

## 4. Root-cause hypotheses

**(a) Slow creep while streaming SpeedData(0,0) at 20 Hz**, from most to least likely:
1. **The chassis loop has no zero deadband or integrator reset, so "active 0" still drives the motors.** The chassis runs a speed or PWM loop. Each 0,0 frame keeps it engaged with setpoint 0. Then an integrator, encoder quantisation or a PWM offset at zero setpoint produces a small residual drive, and a lawn-mower gearbox turns that into a slow creep. The vendor never exercised the "stream 0" regime because its idle state was silence. *Evidence:* the vendor behaviour differs from ours exactly in this respect, and the creep went away when the regime changed. *Not proven:* no chassis code was found.
2. **Mismatch between float encoding and expected units.** A host-side `linear_scale` / `angular_scale` or a sign or offset error could make "0" non-zero on the wire. This is cheap to rule out: capture the TX bytes and confirm `00 00 00 00 00 00 00 00`.
3. **Ground slope or rolling under coast** if 0 really means coast. The creep would then be gravity, not drive. Check whether it also happens on level ground with the wheels up (it cannot happen wheels-up if this is the cause).

**(b) Circular crawl after lift and set-down, with the brake (`-0.5·measured`) active:**
1. **A closed loop on unvalidated measured *angular*.** The vendor never used measured angular. A sign inversion, a unit mismatch (deg/s versus rad/s) or a bias makes `-k·measured` into positive feedback, or into a constant non-zero command, about the yaw axis. That fits "circle" rather than "straight" and is the most likely cause.
2. **Lift transition.** While lifted, SpeedData is ignored but the brake keeps computing from measured, which may be stale or affected by free-spinning wheels. On set-down, the first accepted frame is a non-zero "brake" command. If the MCU holds that value (silence = hold, see section 2) and measured responds, the loop sustains itself.
3. **Measured is partly an echo of the command.** Then `u[n+1] = -0.5·u[n-d]` with a delay `d >= 2` can limit-cycle. A pure echo with `d = 1` would decay, so this alone explains (b) less well than item 1.

Whichever of these is true, the vendor never closed a loop on measured speed, so any closed-loop brake is outside known-good behaviour.

## 5. Recommended TX policy for `mower_mcu_driver`

Replicate the vendor's policy, plus one explicit stop rule that the vendor got for free from its producers.

| Parameter | Value | Note |
|---|---|---|
| `speed_stream_enabled` | **false**: event-driven | Send one SpeedData per `/cmd_vel` message, immediately, raw `float32(lin.x), float32(ang.z)` |
| `speed_cmd_rate` | unused (0) | No periodic resend |
| `brake_gain` | **0.0**, remove from the TX path | Vendor had no brake |
| Host PID in TX path | **none** | |
| Stop on zero cmd | When an incoming cmd is zero (`|v|, |w| <= 1e-6`) and the last cmd sent was non-zero: send the zero, then **2 more zeros at 100 ms**, then go silent | 3 zeros cover a dropped frame |
| `cmd_vel_timeout` | **0.5 s** | If no `/cmd_vel` arrives for 0.5 s after a non-zero cmd: send 3 zeros as above, then go silent. This replaces the vendor's reliance on its producers |
| Zero while already stopped | **do not send** | Keep idle as silence, matching the tap |
| Lift/stop set | do not send (MCU ignores it anyway); after the flag clears, send nothing until a fresh non-zero `/cmd_vel` arrives | Prevents a stale or brake command being accepted on set-down |
| Heartbeat / IMU forward | 100 ms / 44-45 Hz (unchanged) | |
| Velocity limit | clamp to ±0.3 m/s and ±0.3 rad/s on the host until characterised | Mirrors the PID clamps |

Run the vendor PID **only** as a position primitive (`rotate`/`moveStraight`, for the bumper escape). It should be a separate node that publishes `/cmd_vel` at 20 Hz:
- Gains: linear 0.9/0.3/1.0, angular 0.9/0.3/0.5.
- Output clamp ±0.3; timeout 20 s; tolerance 0.05.
- Measurement: `/odom`, where position is measured linear plus IMU yaw, and the angle is IMU yaw.
- It must never use measured angular.

If creep (a) persists under this policy, stopping becomes "3 zeros then silence". If it persists even with silence, the cause is mechanical or in the MCU, not the host.

## 6. Bench test plan (wheels up, chassis on stands, e-stop in hand)

1. **Wire check.** Use `mcu_tap` and publish one Twist(0,0). Confirm exactly 8 zero payload bytes. Publish Twist(0.1,0) and confirm `cd cc cc 3d 00 00 00 00`.
2. **Idle regimes, 60 s each, recording measured SpeedData plus a mark on each wheel:**
   - (i) silence;
   - (ii) a 0,0 stream at 20 Hz;
   - (iii) 3 zeros, then silence.

   Note any wheel rotation (regime ii is the test for hypothesis a1) and any measured offset or noise floor on both fields.
3. **Is measured an echo or encoder-derived?** Two checks:
   - With silence, turn one wheel by hand. If measured changes, it comes from the encoders.
   - Send 0.1 m/s for 3 s, then hold a wheel by hand (gloved, low speed). If measured drops while the command stays the same, it comes from the encoders.
   - Also measure the step-response latency.
4. **Scale and sign.**
   - Command 0.1 m/s for 10 s. Count revolutions of each wheel and compute expected = 1.0 m / (π·D). Compare with the integrated measured linear.
   - Command +0.3 rad/s. Check the left/right wheel directions (CCW positive), and compare measured angular against IMU gyro z (sign and units).
5. **Silence semantics.** Send 0.1 m/s once (a single frame), then go silent. Time how long the wheels keep turning (hold versus the heartbeat-timeout zeroing).
6. **Lift path.** Lift the mower while moving: the wheels should stop. Lower it with no `/cmd_vel` publisher running: the wheels must stay still.
7. **Closed loop.** Enable it only after steps 3-4 pass, starting with the linear axis. Any closed loop on angular must use IMU yaw rate, never measured angular, unless step 4 proves measured angular is good.

## Open items

- The chassis runtime speed loop (deadband or integrator at 0) was not located. The next RE target is the handler that consumes the module-4 setpoint arriving from the cutter link.
- The vendor planner's `/cmd_vel` rate and stop behaviour were not recovered (separate binaries).
- I did not verify which of the four `createTimer` sites drives the heartbeat at 100 ms.

## Both-wheel turn shaper (cmd_vel_slew, 2026-10-07)

Owner rule: "all turns should require both wheels turning". A pivot with counter-rotating
wheels IS both wheels turning; what is forbidden is the one-wheel zone, an inner wheel at
0 <= |v_inner| < `min_inner_wheel_mps` (0.06). `cmd_vel_slew.shape_for_both_wheels` (via
`TurnShaper`, autonomous phases only, latched `/cmd_vel_slew/shape_status`) computes the
signed inner wheel speed `|v| - |w|*b/2` and, if it lies in (-0.06, +0.06), snaps the
command to the NEARER legal regime:

- inner in (0, 0.06): the tightest arc keeping the inner wheel at +0.06 (same direction as
  the outer; reversing stays reversing), lowering `|w|` if the outer wheel would exceed
  `linear_max` 0.3. Implied minimum radius 0.36 m; 0.44 m with the MCU |w| <= 0.3 clamp.
- inner in (-0.06, 0] (including slow pure pivots): a pure pivot, v = 0,
  |w| = max(|w|, 2*0.06/b) = 0.25 rad/s, capped at `angular_max` 0.3 (MCU clamp).
- pure rotation (v = 0) with |w| below `turn_shaper_pivot_deadband_rps` (0.03): an exact
  stop (0, 0), status `pivot deadband: ... -> 0`.

Pure pivots at |w| >= 0.25 pass unchanged. Track width b = 0.48 m (URDF `drive_track_y`
0.24 x 2).

Where it applies (fix 9, 2026-10-14): only commands on the twist_mux navigation lane
(`/cmd_vel_nav`, the lane is re-derived from the lanes' receive times by `LaneTracker`) in
TRANSIT, MOWING, BOUNDARY_PAUSED and the dock phases RETURNING_HOME, LOW_BATTERY_DOCKING,
RAIN_DETECTED_DOCKING, COVERAGE_FAILED_DOCKING. The dock phases all start with a nav-driven
transit to the dock approach (`MissionFSM._dock()`), which drove 151 one-wheel ticks
unshaped on 2026-10-14; the marker-guided docking/undocking controller publishes on
`/cmd_vel_docking` in those same phases and is never shaped, nor are the bumper, emergency
and teleop lanes (teleop only with `shape_manual`). With no lane heard within its timeout
the phase-only rule applies (TRANSIT/MOWING/BOUNDARY_PAUSED).

Slow pure rotations (fix 8, 2026-10-14): from 2026-10-07 until fix 8 a pure rotation slower
than 0.25 rad/s passed unchanged, because snapping it had turned the controller's small
heading corrections near a goal into 0.25 rad/s pulses (no latch existed then). The
2026-10-14 mow showed what that costs: both wheels at |w| < 0.25 run under the MCU's
0.06 m/s deadband, so the robot does not turn (4564 such ticks, a stop-start that ended in
a transit abort), and MPPI dithering between v = 0.010 and v = 0.000 at the same w flipped
`/cmd_vel` between 0.25 and 0.10 rad/s every ~0.3 s (548 regime changes). Now a slow pure
rotation snaps to the 0.25 pivot like any other one-wheel command, and the regime latch
(below) is what keeps 0.25 pulses from hunting. `turn_shaper_pure_pivot: false` restores
the pass-through exactly (live-settable). `both_wheels_legal(..., target=True)` applies
the same rule to a command; the slew functions keep treating slow pure rotations as legal,
because a rate-limited ramp into or out of a pivot passes through them with both wheels
counter-rotating.

Regime latch (2026-10-14): the snap boundary is inner = 0, so at |w| = 0.3 a v of 0.070
snapped to a pivot (v = 0) and 0.075 to an arc (v = 0.132); a controller sitting in that
band flipped the target every tick. `TurnShaper` now latches the regime (pivot = inner
wheel backwards, arc = forwards). Changing it needs the raw inner wheel
`turn_shaper_hysteresis_mps` (0.02) past the boundary, or `turn_shaper_latch_ticks` (6)
ticks in a row asking for the other regime; meanwhile the command is shaped to the latched
regime (pivot snap or tightest legal arc). Straight/stop (including a pure rotation in
the deadband) or a sign change of v or w release the latch. Hysteresis 0 restores the
stateless snap. Since fix 8 an exact pure rotation (v = 0) no longer releases the latch: it
is the pivot regime (inner = -|w| b/2), and v = 0 is compatible with either sign of v, so
v dithering 0 / +0.01 is not a sign change. An arc latched when v drops to 0 is held as
the tightest arc for that w in its direction until the hysteresis releases it (at once
for |w| >= 0.083 rad/s, where inner = -|w| b/2 <= -0.02) or 6 ticks pass. When a pivot
assist creep stops (cap reached or held for an obstacle), the node resets the latch, so
the creep arc is never held past the assist.

History: the first version (same day) converted every pivot into a forward arc capped at
0.3 m of travel (then a 2 s true-pivot fallback). That turned the docking about-turn into
an r ~ 0.4 m arc that swung the robot past the dock, so the conversion, the 0.3 m cap and
the `shape_pivot_max_dist_m` / `shape_pivot_fallback_s` parameters were removed. The
"turn shaper: ..." INFO line is logged only when the regime (pass/arc/pivot) changes, at
most once per 2 s; the full decision stays on `~/shape_status`.

Pivot assist (`cmd_vel_slew` `PivotAssist`, re-enabled 2026-10-10, b97de97) is **on by
default**: after 0.8 s of a pure pivot (|w| >= 0.08) in TRANSIT/MOWING it adds 0.14 m/s
forward (inner wheel +0.068 at |w| = 0.3, a legal arc), at most 0.3 m per pivot. Gated
(2026-10-13) to FollowPath pivots on the twist_mux navigation lane (never the bumper,
emergency, teleop or docking lanes, Nav2 Spin/BackUp, or within 5 s of a bumper/emergency
message), and held while the local costmap has a lethal cell in the creep sweep or a
front/side vision obstacle is within 1.5 m: the slew is downstream of collision_monitor, so
nothing else collision-checks the creep. During an assist episode only (creep ramp-up,
hold and ramp-down) the slew keeps every tick out of the one-wheel zone
(`slew_both_wheels`): the pivot <-> creep-arc change passes through a stop (w to 0, then v),
instead of ramping v at full |w| through inner (-0.06, +0.06) for 5-9 ticks. Other
MPPI/RPP driving uses the coupled slew (see "Velocity chain limits" below). The assist runs
before the turn shaper, on the controller's own |w|: since fix 8 a 0.1 rad/s rotation is
shaped to a 0.25 pivot and, after 0.8 s, gets the creep like any other pivot; rotations
under 0.08 rad/s get the 0.25 pivot without a creep.

## Velocity chain limits (cmd_vel_slew, 2026-10-14)

`cmd_vel_slew` is the authoritative rate limiter of the chain (nav2 MPPI FollowPath ->
velocity_smoother -> collision_monitor -> twist_mux -> cmd_vel_slew -> mcu_node).

* **Decel:** `max_linear_accel` 0.5 m/s^2 applies while |v| grows, `max_linear_decel`
  0.8 m/s^2 while it shrinks toward zero (vendor pure pursuit: accel 0.5 / decel 0.8 /
  angular 1.5, `adaptivePurePursuit.h:23-25`). A sign change decelerates to zero, then
  accelerates. `max_angular_decel` defaults to `max_angular_accel` (1.0).
* **Hard stops:** when a `hard_stop_lanes` lane (default `emergency`, `bumper`) wins in
  twist_mux (re-derived by `LaneTracker`) and commands an exact zero, the output goes to
  zero on that tick, not ramped. The watchdog / motion-gate hard zero is unchanged; a
  nav/teleop/docking zero still ramps at the decel rate.
* **No double smoothing:** nav2 `velocity_smoother` is widened to max_accel
  [1.0, 0, 3.0] / max_decel [-1.5, 0, -3.0] (20 Hz, OPEN_LOOP and deadband unchanged) so
  it no longer stacks a second 0.5 m/s^2 ramp on the slew, and MPPI (`FollowPath`,
  `FollowCoveragePathMPPI`) plans with the slew's limits: `ax_max` 0.5, `ax_min` -0.8,
  `az_max` 1.0 (Jazzy defaults were ~3).
* **Coupled slew:** in turn-shaper phases (`slew_both_wheels_always`, default true)
  `slew_coupled` moves v and w together along the line to the target at the largest
  fraction both axis limits allow, so an arc from rest keeps the target's curvature and
  its inner wheel rolls with the outer (it is below 0.06 m/s only in the first ticks
  from rest, never backwards). Before, v ramped at 0.5 and w at 1.0 separately, which
  bent the arc during the ramp. Pivot <-> arc changes fall back to `slew_both_wheels`
  (through a stop). False = the old per-axis slew outside assist episodes; docking and
  manual phases keep the per-axis slew.

## Drive at rest (2026-10-07): no host-side release exists

Owner request "disable drive at rest" investigated; **nothing implemented in mcu_node** because
no host command can de-energise the drives:

* **Vendor host** (`mower_base_node.c`): the complete set of host->MCU sends is SpeedData
  (`twistCallbackROS` @0x003e6108, MBN 249701), CutterControl (`cutterControlROS` @0x003e38b8,
  `cutterOff` @0x003e3b34), ChargeControl (`chargeControlROS` @0x00404c98), SensorInfoControl
  (`poweroffROS` @0x00404b28, `clearEStopROS` @0x00336c50), module 0x12 (`motorCalibration`
  @0x003a960c), ImuData and Heartbeat (`sendDataToChassisInit` 10 ms timer @0x00337874).
  SDK wrappers: `SendSpeedData` @0x00460850 ... `SendSensorInfoControl` @0x00460868. There is
  no enable / brake / release / driver_enable message. The vendor idle state was silence too.
* **Cutter (gateway) fw** (re-decompile): re-sends SpeedData to the chassis each tick while the
  SpeedData/heartbeat age counters are <= 200 ticks (`FUN_08014a7c`, `FUN_08014078`); after
  that it zeroes the setpoint and stops forwarding. Lift/stop -> one zero then stop forwarding.
  The only other chassis-bound message is module 0x16 (1 B), tied to a latched flag (probably
  power_off). Nothing carries an enable/brake.
* **Chassis fw**: MotorStatus = fault code if any fault, else 1 if the motor mode byte != 0,
  else 0 (`FUN_08016274`, built into MotorsInfo by `FUN_080182ac`). Mode 2 = speed PI loop
  (kp 0.5, ki 9.0); a zero setpoint keeps mode 2, so status stays 1 and the PI holds zero
  speed with whatever current it needs. No host-command timeout and no zero-setpoint
  driver disable were found (decompile incomplete; medium confidence).
* **Observed 2026-10-07** with the stack idle, docked_idle/idle activity, no SpeedData TX
  for >600 s (silence already in effect): both drives status 1, left +0.30 A / 35 degC,
  right -0.94 A / 68 degC (62 degC earlier, still rising). Silence alone does **not**
  de-energise the drives.

### Docked zero-speed keepalive (2026-10-08, `mcu_node.py` `_docked_keepalive`)

> **Finding (2026-10-08, owner, live): OFF by default, experiments only.** With all-zero
> frames on `/mcu/sent_speed`, the owner saw the wheels turn at each keepalive ping: leaving
> position hold for speed mode releases the hold torque and the wheels creep. This matches the
> timeline: both dock-contact drops (…9582, …9609) happened under the keepalive, i.e. the
> keepalive walked the robot off the dock pads. `docked_zero_keepalive` now defaults to
> **false**; the hold current on the dock is accepted (position hold keeps the robot seated).
> The parameter is read every tick, so `ros2 param set /mower_mcu_driver
> docked_zero_keepalive true|false` takes effect live (builds before this change only read it
> at init). The text below describes the mechanism for experiments.

Because the chassis drops to mode-3 position hold 300 ticks after the last SpeedData (see the
2026-10-08 firmware follow-up below), a wheel displaced while docked (pushed by hand, settling
onto the dock rails) holds a steady current indefinitely. In mode 2 with a zero setpoint a
stationary displaced wheel has zero error and no holding current. The driver therefore sends a
zero SpeedData every `docked_zero_keepalive_period_s` (default 1.0 s) while docked and idle;
`docked_zero_keepalive` (default **false**, see finding above) enables it.

* Timeout assumption: the chassis tick period is not recorded in the decompile notes; assumed
  10 ms, so 300 ticks = 3 s and 1 s is a 3x margin. If the verification below fails (currents
  stay up), the tick may be shorter: lower the period (e.g. 0.2 s) before suspecting the theory.
* Gating (final, 2026-10-08, all must hold): `/mission/activity` is `idle` or `docked_idle`
  (no mission; dock contact is NOT required); no interlock / e-stop (that path stays silent
  at rest exactly as before); not moving, no stop sequence pending, and no non-zero command
  within the last period. Debug stream mode bypasses it. Every other activity (docking,
  undocking, mowing, ...) and an unknown/unpublished activity -> off; there the
  3-zeros-then-silence policy is unchanged. Rationale: `idle` means no mission; a zero speed
  setpoint still resists rolling on gentle ground, and a motor cooking in position hold is
  worse.
* History: the first version required `docked_idle` AND `dock_ok`; live, dock contacts
  dropped while parked (BT CHARGING -> IDLE, activity `idle`), the keepalive stopped and the
  right drive went back into hold (1220 counts, 37 -> 43 degC). A dock latch (ff5219c) did
  not help when the stack booted with contacts already dropped (activity `idle`, latch never
  set, right 1148 counts / 55 degC), so dock contact was dropped from the gate entirely.
* A non-zero `/cmd_vel` is sent immediately as before; the keepalive resumes one period after
  the stop. Start/stop are logged once each, frames are not logged (they do appear on
  `/mcu/sent_speed`).
* Original verification idea (superseded by the finding above), docked and untouched: left/right drive currents
  on `/mower_sensor_info` both fall to ~0 counts within a few seconds of the "keepalive
  started" log line, and the right motor temperature falls toward ambient over ~10 min.

So "status 1 at rest" is the vendor-normal state (the PI holding zero speed doubles as the
parking brake on the dock and on slopes). The right board's ~1 A hold current and heat are
an anomaly of that board/wheel, not of the host protocol: the right wheel was the inner
(reverse) wheel of the turn-shaper pivots and stuck-guard escapes just before the stop
(stuck incident #50 at (-3.36, 1.09)), so the PI is probably holding a wound-up or
mechanically pre-loaded wheel (gearbox / grass wrap / wheel against an obstacle). Check it:
lift the right wheel clear (with the robot off) and spin it by hand, then compare its
at-rest current after a fresh power cycle.

### Docked motor neutral search (2e63426, e5cf7b3): RETIRED 2026-10-13

`docked_motor_neutral` (default was **true**, live on the mower until this change) tried to
cut the dock hold current: while `docked_idle`, charger request off and either drive
> 1.5 A, it nudged one wheel at a time (`linear = v/2, angular = +-v/0.24`, 0.01 m/s for
0.5 s), sent ONE zero, averaged the current for `settle_s` 3.0 and hill-climbed the
direction, with a +-2 cm commanded roll budget and a 300 s retry. **Removed.** The
parameter is still declared (default **false**) so old overrides do not fail; it never
sends a frame, and `true` only logs one warning. The `docked_neutral_*` parameters are gone.

Why no safe version exists (firmware facts above):
* Every SpeedData, zeros included, puts the chassis in mode 2 for ~300 ticks; mode 3 then
  re-latches the hold at the encoder count of that moment. There is no host frame that
  releases or re-latches the hold without the mode-2 window. The current falls only if the
  wheel yields to its load during that window: that yield IS the creep that walked the
  robot off the pads on 10-08. A nudge-and-silence sequence cannot relieve the hold
  without entering the creep regime.
* The tick period is still unknown (assumed 10 ms, so 300 ticks = 3 s). `settle_s` 3.0
  started at the post-nudge zero, so the whole averaging window sat inside mode 2, where a
  displaced wheel yields and draws little. The baseline was taken silent (mode 3, high), so
  the first probe nearly always read "improved" (12 of 14 live) and the climb, which starts
  at `dir = +1` (1677), kept going forward = away from the contacts (the robot reverses
  onto the dock). Code: settle window armed at the nudge end (1811-1812), one zero (1821-1823).
* The MotionGate forbids motion while docked (`/motion_enabled` is false on the dock), so
  a search gated the way every other motion source is gated could never run.

Live record (driver logs in `/userdata/ros2/log`, the container's `ROS_LOG_DIR`): 12 starts
2026-10-11 16:03 to 2026-10-12 20:12 NZDT, 34 probes (16 "improved"). There were 14 first
probes after a silent baseline. All 14 were `dir +1` and 12 read "improved" (e.g. 4.18 ->
0.83 A, 11.93 -> 4.65 A). The relief did not last: 300 s after a "complete" run that
ended at 0.83 A, the same right wheel was back at 4.18 A. Later probes kept rising, past
anything seen with plain hold. Left went 11.93 -> 4.65 (mode-2 probe) -> 4.96 -> 6.92 ->
9.00 -> 11.01 A, and in the run before that 11.82 -> 13.65 -> 14.96 -> 13.90 -> 14.96 ->
16.97 A. The accepted hold cost is ~6 A. Logged commanded travel per wheel was +0.004 to
+0.010 m, always net forward. The 300 s retry restarted the budget each time, and one
rerun came 4 s after "search complete".

Review findings, verified in the removed code (line numbers are `mcu_node.py` @7e21777):
* budget: `_docked_neutral_begin_phase` (1693) reset `_docked_neutral_travel = 0` on each
  wheel phase and each (re)start. A gate flap (< 1.5 A for one tick, 1652) ran the stop
  branch (1786-1790) without setting the completion latch, so the next hot tick restarted
  with a fresh budget; the 300 s latch (1639) did the same after every complete run. Travel was commanded, not
  measured. Nothing aborted on measured wheel speed, odom or `dock_ok` loss.
* gating (1603-1654): it checked the param, interlock/e-stop, `docked_idle`, `_moving`
  and `_charging_enabled`, which is the host request flag. That flag is False after any
  driver restart and while the charge limit holds the charger off (live: "charging
  disabled" every 2 s with `dock_ok` true). It did not check `/motion_enabled`, mission
  EMERGENCY, `dock_ok` or the age of the latched `/mission/activity`. Its frames went
  straight to `_send_speed_frame`: no twist_mux, no MotionGate, no +-0.3 clamp
  (1806; `search_speed` was only floored, 679).
* stop policy: one zero per nudge (1821-1823) instead of the 3-zero stop.
  `_moving` and `_last_nonzero_t` were never set. A real `/cmd_vel` in the same tick got
  the search's "stopped" (0,0) appended: the real command sets `_moving`, the gate drops,
  and the off branch sends a zero (1790) in the same tick. One-wheel moves broke the
  both-wheels rule. 0.01 m/s sits inside the ~0.06 m/s one-wheel band, so a nudge may not
  move the wheel at all, but its frames still switch the chassis to mode 2.

Outcome: the dock hold current stays the accepted cost (see "Drive at rest"). Tests:
`test_node_smoke.py` `test_neutral_*` / `test_real_cmd_vel_on_hot_dock_*`. The param on
true, hot currents, gate flaps and 800 s of ticks must produce zero frames, and a real
`/cmd_vel` on the dock gets one frame plus exactly 3 zeros.

## Bumper during docking (bumper_controller, 2026-10-08)

`bumper_controller` (`src/bumper_controller`, priority lane `/cmd_vel_bumper`, status
`/mower_base/bumper_routing_status`) picks its manoeuvre from the docking FSM state that
`mower_docking` republishes on `/mower_docking/state` (std_msgs/String, every control tick
of a Dock goal, `''` when the goal ends; older than `dock_state_timeout_s` = not docking).
The mission's `high_level_status` only says "docking", not the sub-phase, so it is not used.
The decision is the pure function `decideManoeuvre()` in
`include/bumper_controller/bumper_decision.h`, latched when the manoeuvre starts.

| Docking state | Front bumper | Rear bumper only |
|---|---|---|
| none / stale / UNDOCK / mowing / manual | reverse `back_distance` + rotate clear (unchanged) | n/a (same) |
| NAV_TO_APPROACH, ALIGNING, SEARCHING | straight reverse `dock_backoff_distance` (0.10 m), no turn, then zero for `dock_hold_s` | hold zero `dock_hold_s` |
| DOCKING, FINAL_DOCKING, RETRY (reversing onto / creeping away from the dock) | **no motion**: zero for `dock_hold_s` | forward `dock_rear_backoff_distance` (0.15 m, away from the dock), then hold |

Front and rear together: hold. The Tron chassis only reports front bumper bits
(`bumper`, `bumper_l`, `bumper_r`, all on the front strip), so the rear column is never
taken on this hardware. Unchanged: the 2 s `max_duration_s` bound, the trailing zero
burst, suppression when docked / charging / stop / lift, rising-edge triggering. During
the hold, routing status stays 1, so the docking node's bumper pause (`bumper_clear_s`
after routing goes idle) resumes from the current pose. Parameters:
`src/bumper_controller/config/bumper_controller.yaml` (`dock_backoff_distance`,
`dock_rear_backoff_distance` clamped to 0..0.3 m, `dock_hold_s` 0..2 s,
`dock_state_topic`, `dock_state_timeout_s`).

### Follow-up 2026-10-08: chassis motor modes from the firmware (chassis 0.6.34)

* Per-motor mode byte setters: `FUN_08010d40` (raw, used with 0 = OFF), `FUN_08010d30` mode 1
  (current/torque ref), `FUN_08010d5a` mode 2 (speed PI, kp 0.5 ki 9.0), `FUN_08010d4a` mode 3
  (relative position hold: on entry `FUN_08012e56` latches the current encoder count as the
  target, gains reloaded, limit 300). Control step `FUN_08013d44`-ish switch @0x08013d74:
  mode 0 requests motor state 6 (`FUN_08016740(drv,6)`, driver stop), mode 1 has no outer loop.
* Group API `(*api+8)(cmd,data)` (table in RAM, init data not in the .bin, so the mapping is
  inferred from use): cmd 0 -> all mode 0 (`FUN_08013280`), cmd 2 -> speed, cmd 3 -> position
  hold, cmd 4 {1,1} -> motor state 3 (calibration/alignment, `FUN_08013220`).
* Callers: cmd 0 only while flag +0x102 is set = BMS firmware pass-through update
  (`FUN_080146c0`, "BMS_V"). Speed (cmd 2) while the SpeedData age counter (`FUN_08017596`,
  reset by module 8 in `FUN_080179cc` and module 0x52) is < 300 ticks; after that one cmd 3
  zero = **position hold at the current spot**, re-sent at ~5000 ticks. cmd 4 only from
  MCCalib (module 0x12) handling. No host frame reaches cmd 0 or cmd 1.
* Hence at rest on the dock the wheels sit in position hold; a pre-loaded wheel makes the
  integrator hold a steady current indefinitely (right wheel 2026-10-08: raw -6220 counts,
  44 degC vs left +2150 / 32 degC, cutter 33 degC ambient, speed 0, docked, charging 4.1 A).
  No de-energise path exists short of power-off / the BMS-update mode.
