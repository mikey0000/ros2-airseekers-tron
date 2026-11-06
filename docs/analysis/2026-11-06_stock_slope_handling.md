# Stock Airseekers firmware: slope handling (2026-11-06)

Reverse-engineered from stock OTA v1.3.27-rc.1, `devel/navigation/lib/libnavigation_lib.so`
(aarch64, built without optimisation, so the control flow reads straight from the decompile).
Question: what does stock do on slopes, and what should we copy for the downhill pivot drift
(0.15-0.31 m per pivot on the 10 deg `real_run5_slope` world) and the west-edge side-slope
incidents?

- **C**: read in the decompiled code or the disassembly, or in a launch/config file.
- **I**: inferred from names, structure or physical sense.

Sources, all outside this repo:

- `native_decompile/dec/libnavigation_lib/libnavigation_lib.so.c` (Ghidra 10.4, 29 k
  functions). Ghidra loaded the library at +0x100000, so the function at file address
  0x72ab54 appears as `@ 0x0082ab54`.
- `native_decompile/sym/libnavigation_lib.so.symbols.txt` (dynsym).
- `native_decompile/dw/navigation.structs.txt`: the `navigation` executable still carries
  DWARF. Its struct layouts confirm the member offsets used below (`NrtPPCTrack::m_slopePitch`
  at +0xb9c, `MainNode::m_sumPitchAng` at +0xbd8, `SensorInfo` at `MainNode`+0xbe0, the
  `MoveUtils` slip fields at +0x120..+0x148).
- Vendor headers in `ros2_port_handoff/04_full_src_tree/src/mower_pnc/mower_controller/navigation/src/`.

## Which controller is live (C, launch files)

`mower-controller.service` runs `roslaunch mower mower_controller.launch`, which includes
`$(find navigation)/launch/mower_controller.launch`. That starts the `navigation` node
(`libnavigation_lib.so`, classes `nav::MainNode`, `cover::NrtPPCTrack` for coverage rows,
`local::RtStanleyTrack` for edges and transits). No launch file in the v1.3.27 tree includes
`mower_controller/launch/mower_controller.launch` (`mower_controller_node`,
`libmower_controller*.so`, `nav_config.yaml`).

The 2026-10-21 findings (`NavCoverUturn`, `CheckStuck`, the `nav_config.yaml` thresholds) were
read from `libmower_controller*.so`. In this OTA that library looks like a newer controller
that has not been switched on yet (I). Where the two disagree, this document describes the
code the robot runs.

## Tilt input (C)

`MainNode::CallBackImu` (`@0x0086aebc`) subscribes to `/imu`. It takes `tf::Matrix3x3::getRPY`
of the orientation and keeps a running mean of the last 200 pitch and 200 roll samples
(`m_pitchAngs`, `m_sumPitchAng`, ...). The means go into `SensorInfo.imuAng.averagePitchAng` /
`averageRollAng` (`SensorInfo` +0x1e8 / +0x1ec), in radians. The window is 200 samples, about
2 s if `/imu` runs at 100 Hz (I).

Nothing uses `LocPose.roll` / `pitch`. `check::Check` (`CheckRpy`, `CheckSlid`, and the
`slip_*` keys in `param/cfg.yaml`) is compiled in but never constructed or called (no call
sites, no PLT entries). The `slip_*` values in the yaml are therefore dead.

**Pitch sign (I).** `getRPY` on the stock `/imu` frame gives pitch > 0 for one of nose-up or
nose-down. The JY61P mounting is not visible. The only behaviour that pins it down is the
fish-tail creep below: it drives backward when the latched pitch is < -3 deg. That drive goes
uphill only if negative pitch means nose down, so this document reads **stock pitch > 0 as
nose up (climbing)**. That is the opposite of REP-103 and of our sim. Every pitch-gated rule
below is one-sided (no `abs`), so the sign matters. Where a rule reads differently under the
other sign, the text says so.

## Q1. `SlopeLimitVel`: a linear speed cap only (C)

`motion::MoveUtils::SlopeLimitVel(pitch, roll, maxAng, maxVel, cmd)` (`@0x00754780`, 240 B):

```
if (pitch > maxAng || |roll| > maxAng)
    cmd.linear.x = sign(cmd.linear.x) * min(|cmd.linear.x|, maxVel);
// cmd.angular.z untouched
```

- **Angle:** pitch and roll tested separately, with no combined tilt. Roll uses `abs`. Pitch
  does not, so only climbing (stock sign, I) triggers on pitch.
- **Callers and constants:** `NrtPPCTrack::Tracking` (rows and turns) and
  `RtStanleyTrack::Tracking` (edges and transits). Both pass the averaged IMU pitch and roll,
  `maxAng = 0x3e123a13 = 0.14280 rad = 8.18 deg` (pi/22), and `maxVel = 0.3 m/s`. Both are
  hard-coded and do not come from config.
- **When:** after the state machine, before the smoothers, whenever the command is non-zero.
  ESCAPE (cover) and the equivalent state in the edge tracker are skipped.
- **Effect:** rows run at 0.38 m/s (`shaped_linear_vel`), edges at 0.45 (`edge_linear_vel`),
  transits at 0.5. All are capped to 0.3 m/s above 8.2 deg. Pivots are not touched.
- `nav::MainNode::SlopeLimitVel` (5 deg on |pitch| or |roll|, 0.3 m/s) exists but has no
  caller. It is dead code.

## Q2. `SLOPE_BACK` and `SELF_ADAPTIVE`: a 3-point turn with a straight reverse first (C)

**Selection is per area, not automatic (C).** `TaskUnit.truning_mode` in the app's area
settings is passed through `UpdateNavParams` / `SetTypePath` into `CentreTask.turnMode`
(`NrtPPCTrack` +0x741). `NrtPPCTrack::EventUturn` (`@0x00823600`) branches on it at the end of
each row:

| turn mode | controller state |
|---|---|
| IN_PLACE (1) | OBSROTATION: in-place pivot to the next row yaw |
| INNER_FISH_TAIL (3), SLOPE_BACK (5), SELF_ADAPTIVE (6) | TURNINNERFISHTAIL |
| OUTER_FISH_TAIL (4) | TURNOUTERFISHTAIL |
| DEFAULT (0), CIRCULAR_ARC (2) | TURNBACKARC: pivot plus reverse arc (`arc_*` in cfg.yaml) |

When the boundary check (`CheckBoudary` on 0.15 m circles at the row end and at the next row
start) fails at both ends, every mode goes to LOCALPATHTRACK (A*). Nothing switches the turn
mode on tilt.

**`TurnInnerFishTail` (`@0x0082ab54`).** It runs `InnerFishtailStatus` (enum from
`nrt-ppc-track.h`). Every segment is a local path followed by `FishPathTrack`.

1. **INIT, pre-reverse distance.** Row length L (distance from the row start to its end).
   The pre-reverse runs only if L > 0.4 m and the mode is 5 or 6:
   - SLOPE_BACK (5): `back = clamp(L - 0.3, 0.1, 0.9)` m.
   - SELF_ADAPTIVE (6): `back = min(L - 0.3, 0.3)` when averagePitch <= 8 deg, else
     `min(L - 0.3, 0.3 + (pitch - 8 deg) * 0.6 / 4 deg)`, then clamped to 0.1..0.9 m. That is
     0.3 m up to 8 deg, rising linearly to 0.9 m at 12 deg. Log:
     `turn mode backdist ---> %d, %f, %f`.
   - INNER_FISH_TAIL (3) and short rows skip the pre-reverse.

   The reverse target is the row point `back` m before the row end. The path is a cubic
   Bezier back along the mowed row (`m_backUpPath`, backward, 0.2 m/s).
2. **BACKUP ("斜坡直退", straight reverse on a slope).** Straight reverse along the row just
   mowed. After every 0.3 m it goes to **HEARSAFEPARK**: a 0.2 s stop, then BACKUP resumes from
   a new start pose. In the disassembly the test is `dist > 0.3` (`@0x72c100`); Ghidra printed
   it inverted. The segment ends when the path is done, or on a reverse slip, or after 12 s
   per 0.3 m piece.
3. **HEARBACKARC.** Reverse arc to `rowYaw + sign(turn) * 50 deg` (pi/3.6) at 0.2 m/s and
   0.2/0.6 rad/s, so r = 0.6 m. It ends on arrival, reverse slip or 20 s.
4. **FORWARDARC.** Forward arc for the remaining heading change, capped at 63 deg (pi/2.85),
   r = 0.6 m, 0.2 m/s. The turn ends early if less than 30 deg remains.
5. **TAILROTATION.** In-place rotation (`MoveUtils::Rotation`) to face a straight line that
   ends 0.25 m behind the next row start.
6. **TAILBACKARC.** Straight reverse along that line, with the same 0.3 m / 0.1 s park rule
   (TAILSAFEPARK), until arrival, reverse slip or 16 s.
7. **RETURNLINE.** If the robot projects past the row start (>= -0.02 m) it tracks the row.
   Otherwise it drives a forward Bezier onto the row start first.

So SLOPE_BACK and SELF_ADAPTIVE are the inner fish-tail with a straight reverse in front of
it. **Stock never pivots 180 deg on the spot at a row end in these modes.** Most of the
heading change is driven on two r = 0.6 m arcs (50 deg reverse, up to 63 deg forward). Only
the remainder is pivoted, roughly 60-70 deg (I, from the geometry), and the robot then
reverses into the next row. All segments run at <= 0.2 m/s.

**Direction relative to the fall line (I).** The pre-reverse always goes straight back along
the row. Its length depends on pitch only, not on roll, and only for nose-up (stock sign).
Read with stock pitch > 0 = climbing, SELF_ADAPTIVE backs up to 0.9 m downhill before turning
at the top end of a climbing row. That keeps the turn away from the top edge. Under the other
sign it backs up uphill at the bottom end of a descending row, away from the bottom edge.
Either way the reverse puts room between the turn and the row-end boundary in proportion to
the slope.

Nothing in the turn looks at roll. On a pure side slope (rows along the contour, pitch ~ 0)
SELF_ADAPTIVE reverses 0.3 m and SLOPE_BACK reverses up to 0.9 m. The arcs are then
identical to the flat case.

## Q3. Pivots on slopes (C, by absence)

- **No slope term in any rotation.** `MoveUtils::Rotation` (`@0x008567e4`):
  `w = sign(e) * clamp(1.2 |e|, 0.4, 0.7) rad/s`, done at |e| < 0.087 rad (5 deg), linear 0.
  The ramp is `VelZSmoother`, 2 rad/s^2 (`acc_vel_z` / `dacc_vel_z` 2.0, +1 when the sign
  flips). `CheckRotateDir` / `PredictRotationalCollision` pick the turn direction from
  obstacles and the boundary only. `TurnBackArc`, `TurnMultiPoint`, `TurnOuterFishTail`,
  `TurnCrawl`, `AvoidObsRotation`, `ExceptionBack`, `Escape` and both `Rotation` overloads
  never read the IMU means.
- **Stock pivots are fast.** A 90 deg pivot takes about 2.9 s and 180 deg about 5.1 s (ramp
  included, simulated from the formula above). Our slope pivots run at 0.25-0.35 rad/s: 5.2-6.3
  s for 90 deg, 10.5-12.6 s for 180 deg. Stock does not slow its pivots on a slope.
- **Downhill creep pre-compensation: one tiny rule (C).** When `FishPathTrack` reaches the end
  of a fish-tail segment it latches `m_slopePitch = averagePitch` (`@0x00826574`, line 0x7f8
  `arrived local path`). For the remaining heading alignment (|heading error| > 6 deg) it
  turns at +-0.5 rad/s while driving 0.01 m/s (0.1 m/s on the final RETURNLINE). It drives
  forward if `m_slopePitch` > +3 deg, backward if < -3 deg, and in the path's direction
  otherwise. Log: `slip action dir %d, %f, %f, %f, %f`. That is the only place pitch
  touches a rotation. 0.01 m/s does not cancel a 4.5 cm/s creep. It only fixes which way the
  wheels move.
- **`m_slopePitch`** is only that latch. **`averagePitchAng`** drives `SlopeLimitVel`, the
  row-end 0.2 m/s cap (Q6), the SELF_ADAPTIVE reverse distance and the latch.
  **`averageRollAng`** drives `SlopeLimitVel` and the side-slope speed scaling (Q4). Nothing
  else reads them.
- **Disabled drift detector (C).** `NrtPPCTrack::SideSlipRotation` and `SideSlipLine` detect
  that the robot has slid past the row line (projection < 0 or < -0.1 m with > 0.2 m offset)
  after a rotation, and switch to TURNMULTIPOINT (log `side slip rotation inner fish`). They
  have no callers. The vendor tried "detect pivot drift, fall back to a multi-point turn" and
  turned it off.

## Q4. Row tracking on side slopes (C)

`TrackPath2` (`@0x00824fb8`, the row tracker used by `TrackLine`) is pure pursuit:

- Lookahead `Ld = clamp((k - 0.1) v + 0.35 - 1.5 |e_lat|, 0.3, 1.0)` m (`k` is a member at
  +0xa58). A larger lateral error shortens the lookahead.
- Curvature from `calculateCurvatureToTarget`, `w = v * kappa`, clamped to +-1.2 rad/s.
- Heading-error speed `alphaVel = clamp((60 deg - |alpha|) / (60 deg - alpha_thre) * v,
  v_min, v)`, then multiplied by 1.0 at |e_lat| <= 0.01 m falling linearly to 0.5 at 0.1 m.
- **Roll term (C):** if |averageRoll| > 8 deg, `rollVel = clamp((0.35 - |e_lat|) / 0.3 * v,
  v_min, v)` and `v = min(alphaVel, rollVel)`. Log: `-------alphaVel %f, rollVel %f`. So on a
  side slope the row speed drops with the lateral error: full speed up to 5 cm off, `v_min`
  (`min_linear_vel` 0.12, I) at about 0.25 m and beyond.
- **No roll feed-forward or trim on the steering.** Roll changes only the speed. The fish-tail
  heading loop (`PIDBCController`, kp 2.0, ki 0, kd 0.1, back-calculation 0.25, +-1 rad/s) has
  no integral term, so it cannot absorb a steady crab either. Its heading target includes
  `atan(e_lat / 0.2)` while `isLine` is set.

## Q5. Slip detection, `ExceptionBack`, `TurnBackUp` (C)

`MoveUtils::RobotIsSlipped` (`@0x00754870`). The thresholds are hard-coded in the `MoveUtils`
constructor. The `slip_*` yaml keys are read only by the unused `check::Check`.

- It integrates the commanded twist into `predDist += v * 0.05` and `predAng += w * 0.05` over
  a window, and compares with the actual `Dist(pose, startPose)` and `|theta - theta0|`.
- After 2.0 s (`m_slipDetectionTime`) the window counts as slipping if actual dist < 0.2 m
  **and** (actual dist < |predDist| / 3 **or** (actual yaw < |predAng| / 3 **and** < 0.5
  rad)). While slipping, the command is divided by `0.5 * t_window`. If the condition fails,
  the window resets.
- Slip is declared at a 5.0 s window (`m_slipTimeout`). `SlipStatus` is FORWARD or BACKUP from
  the command sign. The `isSelfHelp` argument is unused.
- **Reverse slip inside a turn is not a fault.** In TURNBACKARC, TURNMULTIPOINT and both
  fish-tails, with linear < 0, a slip only sets a flag (`back is slip enter next state`). The
  fish-tail treats it as "segment done" and moves on. On a slope where the reverse does not
  bite, the turn continues with the next arc instead of escalating.
- **Any other slip, or a motor over-current flag:** `addEscapePoint`, then an escape
  direction from `DistToForbidden` at the pose and 0.3 m behind / ahead (move away from the
  boundary), then CUTTERPAUSE (2 s), then ESCAPE.
- **`ExceptionBack`** (bumper / slip / turn-obstacle reverse): `BackUpLine` for
  `m_bumperBackDist`. Over 8 s it sets STUCK. Repeated bumps give MULTIPLE_COLLISIONS. The
  next step is TURNBACKUP (perception line change: rotate, then point-track).
- **Slope effect:** none directly. No slip threshold scales with tilt. Slope matters only
  through the reverse-slip rule and the 0.3 m/s cap.

## Q6. Other slope behaviour

- **Row-end slowdown (C).** In `TrackPath2`, on a scan line with < 1.5 m left and
  averagePitch > 8.18 deg, `v = min(v, 0.2)` (log `-------5ctrl.linearVel`). Read as stock
  pitch > 0 = climbing (I): slow down before the top-end turn.
- **Blade (I).** No slope rule touches the cutter. CUTTERPAUSE (slip escape) and long A*
  detours set the controller's motion code to 3 instead of 1 (`NrtPPCTrack` +0x869). That is
  likely the "blade off" request to the task layer.
- **No slope-aware planning in the controller (C).** No row-direction choice, area refusal or
  maximum-angle check. The planner was not part of this pass. Hard tilt is the MCU's job
  (lift / tilt cut-off, see `mower_mcu_driver` notes).

## Comparison with our stack

| topic | stock (live `navigation` node) | ours |
|---|---|---|
| tilt input | 200-sample mean of IMU pitch/roll | `/tilt/status` bands from tilt_monitor |
| linear cap | 0.3 m/s at \|roll\| or pitch > 8.2 deg (rows 0.38, edges 0.45) | `slope_caution_roll_deg` 8 caution cap |
| row-end turn on slope | per-area mode: straight reverse 0.1-0.9 m, r 0.6 arcs, small residual pivot | in-place pivot at the vertex, upslope split (`pivot_creep_*`) |
| pivot rate | 0.4-0.7 rad/s, 2 rad/s^2, no slope term | 0.25-0.35 rad/s, slowed on slope |
| pivot creep compensation | none (0.01 m/s pitch-directed drive) | upslope drive of half the expected creep |
| side-slope steering | none; speed cut with lateral error above 8 deg roll | PathAlignAheadCritic trim + roll feed-forward |
| reverse slip | skip to the next turn segment | not handled (reverse legs mostly joined away) |
| drift-after-pivot detector | written, disabled | spin-drift guard 0.15 m (cancel / re-pivot) |
| slope-aware planning | none | `slope_mode` (off by default), `edge_slope` rule |

Stock is simpler than ours on steering and planning. It mainly avoids long pivots on slopes:
fish-tail turn modes replace the pivot, and the pivots it does make are short and fast.

## What to adopt

Ranked by expected effect on downhill pivot drift and the west-edge incidents.

1. **Faster pivots on slopes, not slower. Effort: low (params plus one gate run).** Stock
   pivots at `clamp(1.2 e, 0.4, 0.7)` rad/s and stops at 5 deg. Our slope rule lowers the rate
   to 0.3 rad/s. In our creep model the drift scales with time spent pivoting (4.5 cm/s at 10
   deg). A 90 deg pivot drops from about 5.2 s to 2.9 s, so the creep per pivot goes from
   about 0.23 m to about 0.13 m, and from about 0.47 m to 0.23 m at 180 deg. Reverse the
   roll-aware rate (raise `pivot_max_radps` / Spin on slope towards 0.6-0.7 with 2 rad/s^2)
   and gate it on `real_run5_slope`. Risk: the sim's `pivot_yaw_loss` and creep may not scale
   with rate the way turf does, so the field confirms it. This is the cheapest lever with the
   largest expected gain.
2. **Replace row-end 180 deg pivots on side slopes with an arc turn. Effort: high
   (planner turn type, reverse controller, sim slip model).** Stock's answer to slope turns
   is SLOPE_BACK / SELF_ADAPTIVE: straight reverse, 50 deg reverse arc r 0.6, forward arc up to
   63 deg, short pivot, reverse into the next row, all at <= 0.2 m/s. Driving wheels keep
   traction, and our model creeps 1 cm/s driving against 4.5 cm/s pivoting. Two blockers on
   our side: r 0.6 arcs do not fit the 0.45 m row-end inset (`2026-11-03_turns.md`), and our
   slope model slips 95 % in reverse. So this needs the reverse-slip rule (item 4) and a turn
   area beyond the inset. Prototype it as an area `turn_type` for `slope_mode` areas only, and
   keep it behind the sim gate.
3. **Speed cut with lateral error on side slopes. Effort: low.** Above 8 deg roll, stock
   scales row speed by `(0.35 - |e_lat|) / 0.3`, down to `v_min` (0.12) at about 0.25 m off. Our
   caution cap is a flat ceiling. Slowing in proportion to the error gives the trim and the
   critic more time per metre to pull the robot back before the west edge. It does not stop
   creep during a pivot, so its effect is on the straights and the edge laps (the incidents),
   not on vertex drift.
4. **Reverse slip skips the segment. Effort: low once reverse segments exist.** Stock treats
   a reverse that does not move (2 s window, < 1/3 of commanded, < 0.2 m) as "segment done"
   and continues. Any reverse leg we add on slopes (item 2, boundary-out recovery) should do
   the same instead of escalating to STUCK.
5. **Pre-turn reverse proportional to slope at a row end facing the boundary. Effort:
   medium.** SELF_ADAPTIVE reverses 0.3 m below 8 deg, rising linearly to 0.9 m at 12 deg,
   before turning. For our pivots: back off from the edge-side vertex by a slope-scaled
   distance (along the row, away from the boundary) before pivoting, so that the expected
   creep (0.15-0.3 m) lands inside the area. This targets the west-edge excursions directly
   and needs no new controller. Use roll and the downhill direction, not stock's one-sided
   pitch.
6. **Row-end slowdown when climbing. Effort: trivial.** 0.2 m/s in the last 1.5 m of a row
   above 8.2 deg pitch. This is a small help to the turn start position. Copy it only if item
   5 is built.

Not worth copying: stock's 0.01 m/s pitch-directed creep (too small to matter; our
`pivot_creep_*` upslope drive is stronger); the disabled `SideSlipRotation` detector (our
spin-drift guard does the same job, and its cancel / re-pivot adds creep, which is a reason
to make the pivots shorter, item 1); the `MainNode::SlopeLimitVel` 5 deg rule (dead code).
Stock has no roll feed-forward, integral trim, slope-aware row direction or blade stop on
slope. Our stack is already ahead there.
