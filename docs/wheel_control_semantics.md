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

Owner rule: no one-wheel turns, no counter-rotating pivots. `cmd_vel_slew.shape_for_both_wheels`
(via `TurnShaper`, autonomous phases only, latched `/cmd_vel_slew/shape_status`) widens any
command whose inner wheel `|v| - |w|*b/2` is below `min_inner_wheel_mps` 0.06 to the tightest
arc keeping the inner wheel at 0.06 (both wheels same direction; reversing stays reversing),
lowering `|w|` if the outer wheel would exceed 0.3. Track width b = 0.48 m (URDF
`drive_track_y` 0.24 x 2). Implied minimum radius r_min = b/2*(v_out+v_in)/(v_out-v_in) =
0.36 m (outer 0.3 / inner 0.06); with the MCU's |w| <= 0.3 clamp the tightest arc is 0.44 m.
Converted pivots are capped at 0.3 m of travel, then a true pivot passes for 2 s. The old
pivot assist is off by default (subsumed).
