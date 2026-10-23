# mower_docking

Docking / undocking action server for the Airseekers Tron (ROS 2 Jazzy, rclpy).
It re-implements the vendor `mower_charge` behaviour: the robot drives in
reverse onto the charger, guided by the rear camera and an ArUco marker. The
vendor reference is `docs/audit_2026-10-05/vendor_mission_layer.md` section 5.

| Interface | Type | Notes |
|---|---|---|
| `/mower_docking/dock` | `mower_interfaces/action/Dock` | `use_vision`, `timeout_s` |
| `/mower_docking/undock` | `mower_interfaces/action/Undock` | `distance_m`, `speed_mps`, `wait_for_rtk`, `rtk_timeout_s` |
| `/cmd_vel_docking` (pub) | `geometry_msgs/Twist` | twist_mux lane: priority 15, timeout 0.5 s. Published at 20 Hz while a goal drives the robot, with zeros for 0.5 s on every exit. Nothing is published during `NAV_TO_APPROACH`, so Nav2 (priority 10) can drive. |
| `/odom` (sub) | `nav_msgs/Odometry` | distance travelled and heading hold. If odom is stale (> 0.5 s), the server falls back to integrating time × \|cmd\|. |
| `/mower_base/status` (sub) | `MowerBaseDevStatus` | `is_docking_done` (debounced over 3 samples), `stop_triggered`, `lift_triggered` |
| `/map_server_node/docking_pose` (sub, latched) | `geometry_msgs/PoseStamped` | Pose of a docked robot in `map`. The fallback is param `dock_pose`. |
| `/map_server_node/docking_pose_measured` (sub, latched) | `std_msgs/Bool` | True only after "Set docking point" recorded the pose. Blind final docking needs it. |
| `/odometry/filtered` (sub, goal only) | `nav_msgs/Odometry` | fused pose for the stall guard (`progress_odom_topic`) |
| `/gps/status` (sub) | `mowgli_interfaces/GnssStatus` | `fix_type == 3` means RTK fixed |
| `/rear_camera/image_raw`, `/rear_camera/camera_info` (sub) | sensor_msgs | Subscribed only while a vision dock runs |
| `/charging` (client) | `ChargingControl` | enable after contact, disable before undocking |
| `/navigate_to_pose` (client) | `nav2_msgs/NavigateToPose` | approach pose, default goal checker |
| `~/marker_pose`, `~/approach_pose` (pub) | `PoseStamped` | debug: marker in `base_link`, Nav2 goal |

Only one goal (dock or undock) runs at a time. Any further goals are rejected.

## Dock state machine

```
start ──contact already?──► CHARGING
  │
  ▼
NAV_TO_APPROACH ──nav fail / nav_timeout──► FAILED NAV_TO_DOCK_FAILED
  │ (skipped if skip_nav_to_approach)
  ▼
use_vision? ──no──────────────────────────────┐
  │yes                                         │
  ▼                                            ▼
SEARCHING ──no usable marker in search_timeout_s──► blind allowed? (see below)
  │ marker passes the 0.5 m / 25° gate      no ──► FAILED "DOCK_NOT_FOUND: dock not found: <why>"
  │                                         yes ─► FINAL_DOCKING (blind: reverse
  │                                                approach_distance + 0.1 m on odom)
  ▼
DOCKING (reverse, heading/lateral PD, ≤ 0.15 m/s; 0.05 m/s within 0.3 m)
  │ remaining ≤ final_zone (0.2 m)
  ▼
FINAL_DOCKING (straight reverse at 0.05 m/s with an odom heading hold)
  │ is_docking_done on 3 consecutive samples  (checked in SEARCHING, DOCKING and FINAL_DOCKING)
  ▼
CHARGING (/charging enable_charging=true) ──► SUCCEEDED "DOCKED"

FINAL_DOCKING without contact after 10 s or 0.30 m (blind: the planned distance) ─┐
DOCKING with > max_lost_frames ticks lost, or 60 s ────────────────────────────────┤
                                                                                     ▼
RETRY: drive forward retry_forward_distance (0.3 m) at 0.1 m/s, then NAV_TO_APPROACH
       (with skip_nav_to_approach: drive back the distance just reversed, then retry).
       After max_retries (8 since 2026-10-21, was 3): FAILED "DOCK_MAXOUT".
```

**Blind final docking** (dead-reckoned reverse without a marker) is allowed only when
the dock pose is *measured* (`/map_server_node/docking_pose_measured`, see
"Dock calibration" below) **and** a marker was seen during this attempt within
`blind_marker_max_age_s` (30 s). Otherwise the goal fails with zero velocity and
`DOCK_NOT_FOUND: dock not found: marker not visible` (or `...: dock pose not measured`),
which the mission shows as the NAV_TO_DOCK_FAILED reason. `allow_blind_docking: true`
restores the vendor behaviour (blind reverse after any search timeout, also for
`use_vision: false`). Background: on 2026-10-06 the search timed out, the blind reverse
followed the never-measured (0, 0, yaw 0) placeholder dock pose and drove the robot's
rear into a log.

**Stall guard** (DOCKING, FINAL_DOCKING, RETRY): if `|cmd| > stall_min_cmd`
(0.03 m/s) and the fused pose (`/odometry/filtered`) moves less than
`stall_progress_ratio` (20 %) of the commanded distance over `stall_timeout_s` (1 s),
the goal fails with `DOCK_STALLED` and zero
velocity. There is no retry after a stall (no further reversing). The bumper is at the
front, so nothing else senses a reverse into an obstacle.

**Rear obstacle hold** (2026-10-09): `obstacle_guard` publishes `/vision/rear_blocked`
(latched Bool) when a person or animal stands in the reverse corridor BETWEEN the robot and
the dock (it uses `~/marker_pose`, which now carries the marker height in `z`, to end the
corridor `rear_dock_margin_m` before the dock; anyone at, beside or behind the charger is
ignored). In SEARCHING, DOCKING and FINAL_DOCKING the FSM then stands still (no abort, no
retry). State timers and the goal timeout are paused. It continues where it was once the
corridor has been clear for `rear_clear_s` (1.5 s). After `rear_wait_max_s` (60 s) it
fails with `DOCK_BLOCKED: rear obstacle in the dock corridor ...`. Set `rear_hold: false`
or `rear_blocked_topic: ''` to disable it.

The following guards apply in every state:

- `stop_triggered` or `lift_triggered` aborts with `EMERGENCY_STOP`.
- A stale `/mower_base/status` (> 1 s) while moving aborts with `BASE_STATUS_STALE`.
- Exceeding the goal `timeout_s` aborts with `DOCK_TIMEOUT`.
- Cancelling the action returns `CANCELED`, and the Nav2 goal is cancelled too.

The robot stays still while the marker is lost. It never reverses on a stale estimate.

Result messages: `DOCKED`, `ALREADY_DOCKED`, `NAV_TO_DOCK_FAILED`, `DOCK_MAXOUT`,
`DOCK_TIMEOUT`, `EMERGENCY_STOP`, `BASE_STATUS_STALE`, `CHARGER_ENABLE_FAILED`,
`CANCELED`, `DOCK_NOT_FOUND: ...`, `DOCK_STALLED: ...`,
`NAV_TO_DOCK_FAILED: Nav2 navigate_to_pose action server unavailable (...)` (no Nav2 server
within `nav_server_wait_s`: fails at once, without the relaxed current-yaw retry).

Feedback carries `state` and `retries`. `distance_m` is the remaining reverse
distance: from vision in DOCKING, or from the plan in blind mode. It is -1 when unknown.

### Approach geometry

The map origin is the rear axle of the docked robot, X forward (audit section 3). A
docked robot faces away from the charger. With `docking_pose = (x, y, yaw)`, the
approach pose is `(x + d·cos yaw, y + d·sin yaw, yaw)` with
`d = approach_distance = 0.8`. This matches the vendor `undock_point` of (0.8, 0) heading +X. From
there the robot only has to reverse straight back.

### Reverse controller

The marker pose comes from solvePnP on the rectification-free image with
`camera_info` K/D. It is transformed into `base_link` by `rear_camera_T_base`,
then converted into errors relative to the docked pose:

- the docked pose lies `docked_marker_offset` metres from the marker along the marker normal;
- `remaining` is the distance still to reverse;
- `lateral` is the offset from the dock axis;
- `heading` is the heading error.

With `v < 0`, the desired heading is `h_d = clamp(k_lateral·lateral, ±max_approach_angle)`
(2.5 and 15° since 2026-10-21) and `ω = kp_heading·(h_d − heading) + kd_heading·d/dt`,
clamped to ±0.3 rad/s and rate-limited to `max_angular_accel` (0.6 rad/s²). The
sign is derived for reversing: `ẏ = v·sin h`. A closed-loop unicycle test in
`test/test_dock_logic.py` confirms that it converges.

## Stock structure port (2026-10-21)

Ported from the decompiled stock `mower_charge` / `libmower_charge_core.so`
(`docs/analysis/2026-10-21_stock_firmware_findings.md`, Docking). The structure is stock's;
the speeds are ours (wheel floor 0.06 m/s, both wheels always turning, pivots at 0.3 rad/s).
The state machine above still applies, with these changes:

```
NAV_TO_APPROACH ─► ALIGNING ─► [LATERAL_ALIGN] ─► SEARCHING ─► DOCKING ─► FINAL_DOCKING
                                 RTK: > 0.25 m off      any         KF on the marker;
                                 this attempt's line:   detection   lost: reverse on the
                                 forward arc, then                  prediction <= 2 s,
                                 reverse to 0.8 m                   then hold (8 s) -> RETRY
                                                                    pose check at 0.65 m
                                                                    marker distance
```

1. **Exposure stepping** (`exposure.py`, `mower_cameras` `v4l2_ctrl.py`). The rear camera
   starts on auto exposure, or on manual `exposure_dark_start` (30) when the first frame's
   mean grey level is under `exposure_dark_brightness` (50). In SEARCHING, a detection
   ratio <= 0.2 over 3 s (>= 10 frames) steps to the next setting of stock's sequence:
   auto -> 100, 90, 80, 30, 20, 10, 5, 2, 1 -> auto. While the marker is lost in DOCKING
   it steps every 1 s. A setting that works is kept. Auto is restored at the end of the
   dock goal (if it was changed) and after every undock. The docking node publishes
   `std_msgs/Int32` on `/rear_camera/exposure_cmd` (-1 = auto); `mower_cameras`
   (`camera_node`, which owns `/dev/rear_camera`) applies it with V4L2 ioctls
   (`exposure_auto` 1/3, `exposure_absolute`) and latches the result as JSON on
   `/rear_camera/exposure_state`. A camera without the controls reports `ok: false` and
   nothing else happens. Untestable in the sim (the rendered marker ignores exposure).
2. **No search gate** (`use_marker_gate: false`). Stock's 0.5 m / 25° `CheckInitJump`
   has no call site. Any detection starts DOCKING; ALIGNING still ends early only on a
   marker inside the old gate. Each SEARCH window is logged ("SEARCH window 4.2 s ->
   DOCKING: 41 frames, 12 with a marker ...") and the accepting detection is compared with
   the RTK pose (stock's intended, unused check; log only).
3. **Offset retries.** `max_retries` 8 (was 3). Each retry re-approaches on a line offset
   from the dock axis: 0, then +0.1, -0.1, +0.2, -0.2, +0.3, -0.3 m times `side`, the side
   the marker was last seen from. The mission's `dock_timeout_s` is now 480 s.
4. **RTK lateral pre-alignment** (`LATERAL_ALIGN`). After ALIGNING, with a measured dock
   pose and RTK fixed: more than 0.15 m off this attempt's approach line -> drive forward
   at 0.12 m/s on an arc back onto it (|w| <= (0.12 - 0.06) / 0.20 = 0.3, so both wheels
   >= the floor), then reverse on RTK with the normal reverse controller to the approach
   distance. Max 1.5 m forward / 40 s, then SEARCHING anyway. Bumper and rear-obstacle
   holds cover it.
5. **Marker Kalman filter** (`MarkerFilter`). State = robot in the docked frame, predicted
   from `/odom`, corrected with every detection (Q 0.01, R 0.1), innovations clamped at
   0.5 m / 20° (IPPE pose flips, bad corners). The controller and the pose check use the
   filtered estimate. When the marker is lost outside `final_zone` the robot keeps reversing
   on the prediction for `lost_predict_s` (2 s) at <= `lost_speed` (0.08 m/s), then holds
   until `marker_lost_hold_s` (8 s), then RETRY. `marker_filter: false` restores the raw
   path.
6. **Pose check** at a marker distance of `pose_check_distance` 0.65 m (once per
   approach): lateral > 0.05 m or heading > 8° -> forward 0.65 m and restart the final
   approach (`max_realigns` 2, no retry used). The near-field straight reverse
   (marker lost within `final_zone`) is unchanged.
7. **Diagnostics** (recorded by `bag_recorder`, full and low-power rings):
   `/mower_docking/marker_pose`, `/mower_docking/diag` (1 Hz JSON: frames, detections,
   mean brightness, exposure setting / steps / ratio, filter state, clamps, retry offset)
   and `/mower_docking/debug_image/compressed` (1 Hz grey JPEG of the detector input, only
   during a dock goal), plus `/rear_camera/exposure_state`.

Wheel floor: the new forward arc keeps both wheels >= 0.06 m/s. Reversing goes through
`reverse_control`, unchanged: an inner wheel in (-floor, 0) becomes a reversing arc; its
existing same-direction band (inner wheel in [0, floor)) is kept as on master.

### marker_size

`marker_size` stays 0.04 (stock's value) although the black square on the charger is
about 0.055 m (not yet measured exactly). Every marker distance therefore reads about
0.73 x the true one, in the field and in the sim (`rear_marker_size` 0.055). Stock uses
0.04 on the same dock, so its marker-frame thresholds transfer as they are. If you change
`marker_size`:

- delete `calibration_file` and dock once in daylight, so `docked_marker_offset` is
  re-measured (it is calibrated at the 0.04 scale; 0.45 in the yaml);
- scale by new/0.04: `pose_check_distance`, `final_zone`, `slow_distance`, `slow_zone`,
  `final_max_lateral`, `max_lateral_error`, `kf_max_innov_m`, `lateral_slow`,
  `final_max_distance`, `calib_min_offset` / `calib_max_offset`;
- the sim's `rear_marker_offset` (0.543) is derived from 0.45 at the 0.04 scale.

## Night field fixes (2026-10-21, branch `ws/docking2`)

Field test 2026-10-21 night (findings doc, "Field docking 2026-10-21 night"): exposure and
detection worked, the motion did not. Fixes:

1. **Marker heading flips.** At 30–50 px, 29 % of the solves flip the marker normal by
   ~22°, and the old `dock_errors` derived the lateral through that yaw (0.35 m jumps).
   Now, once the filter has a heading, the position comes from the marker centre with the
   filter's heading (`dock_errors_with_heading`, `marker_position_only`); the filter's
   heading is propagated by odometry and corrected by the raw marker heading through a
   1-DOF χ² gate (`kf_gate_chi2_heading` 3.84); positions (median of the last
   `marker_median_n`, odometry-propagated) through a 2-DOF gate (`kf_gate_chi2` 9.21).
   `kf_q` / `kf_q_heading` are per second. When 3/4 of the last `kf_reinit_after`
   measurements are rejected but agree with each other, the filter restarts on them.
   Why not "heading from RTK": the docked EKF yaw is not observed (single antenna), the
   RTK heading during a slow reverse lags, and it depends on the stored dock yaw; why not
   "IPPE nearest the prediction": both IPPE solutions are near-equal at this size and the
   detector would need the prediction; the gated odometry heading gets the same result
   inside dock_logic and is testable offline.
2. **Camera yaw bias.** The consistent detections read the robot heading −7.5° off RTK.
   A fit over 232 detections (marker bearing + normal against the RTK track) puts it in
   the rear camera's mounting yaw (−5.5…−7.5°); the dock yaw agrees with the evening
   undock chord to ~3°. `rear_camera_yaw_correction_deg: -6.5` rotates the extrinsic about
   the vertical through the camera. mower_sim renders the same error
   (`rear_camera_mount_yaw_deg`).
3. **Search acceptance.** `search_min_detections: 3` mutually consistent detections
   (`search_consistency_deg` 10, `search_consistency_m` 0.15) that also agree with RTK
   (`rtk_check_max_deg` 30, `rtk_check_max_m` 0.3; dock measured and RTK fixed).
4. **Reverse controller.** `k_lateral` 7 → 2.5, `max_approach_angle` 28.6° → 15°,
   `max_angular_accel` 0.6 rad/s² (before the wheel-floor shaping, so both wheels stay at or
   above 0.06 m/s). Pose check 0.08 m / 12° on the median of the last
   `pose_check_median_n` (5) detections. Trade-off: from extreme starts (0.3 m off at 0.6 m)
   it reaches the contacts up to ~7 cm off instead of 2 cm; from the approach distance with
   ≤ 0.15 m lateral within 2.5 cm.
5. **Corridor.** Every forward manoeuvre (realign, retry drive, LATERAL_ALIGN arc) stops at
   `approach_distance + forward_cap_margin` (0.8 + 0.25 m) on RTK; the realign distance is
   also clipped up front. No LATERAL_ALIGN after a realign; a bumper hit ends LATERAL_ALIGN
   (it looped 7 times on the night).
6. **Joystick.** `/cmd_vel_teleop` (any |v| > 0.01 or |w| > 0.02 within 0.6 s) pauses the
   goal (zero output, goal timeout paused); released for `teleop_clear_s` it resumes from the
   current pose like the bumper hold; a joystick dock still ends DOCKED (contacts are checked
   first). Docking side only: no twist_mux or mission change.
7. The SEARCH-window log line counts frames from cumulative counters (it reported
   "0 with a marker" when the marker was accepted within the last diag second).

Validation: `test/test_dock_field_2026_10_21.py` (synthetic 29 % / 22° flips seen through
a 7.5° mis-mounted camera, the over-steer case, corridor, joystick) and the field replay
`scripts/replay_field_dock.py DIR --dock X Y YAW [--old]` on the ex082701 export.

### Dock yaw re-measurement

`set_docking_point` keeps a measured dock yaw unless it gets `yaw_source` MOTION, and the
docked EKF yaw is not a measurement. The forward undock is a straight 0.8 m drive under RTK:
the undock action records the fused map track and fits its direction
(`mower_docking/dock_yaw.py`: skips the docked point, RTK fixed throughout, ≥ 0.45 m span,
≤ 3° turn, ≤ 2 cm RMS). The result is logged and saved in the calibration file
(`dock_yaw_measurement`). `ros2 service call /mower_docking/apply_measured_dock_yaw
std_srvs/srv/Trigger` while the robot is on the dock sends it to
`/map_server_node/set_docking_point` (MOTION, GPS position), refused when it differs from the
stored yaw by more than 20°. `auto_apply_measured_dock_yaw: true` applies it after the next
successful dock instead (off by default). GUI: bind it next to "Set docking point" in the
dock settings card (mower_gui_bridge service passthrough); not wired yet.

## Less forward driving before the reverse (2026-10-23, branch `ws/dock-tweak`)

Owner request after the 2026-10-23 midday mow (`docs/wheel_smoothness_field_tests.md`).
In attempt 1, a marker sighting ended ALIGNING 42° off the dock yaw. LATERAL_ALIGN then
drove 0.43 m forward to the 1.05 m cap and fixed nothing (lateral −0.21 → −0.23 m), and it
lost the marker. SEARCHING saw it in 0 of 96 frames, and a forced Nav2 re-approach
followed (1.72 m forward, 0.46 m back). That cost 1 min 43 s.

| rule | before | now |
|---|---|---|
| ALIGNING ends on a marker sighting | yes (inside the old gate) | no, only at `align_tolerance_deg` (or the travel/time budget) |
| LATERAL_ALIGN heading limit | 45° | `lateral_align_max_heading_deg` 15° |
| LATERAL_ALIGN with the marker in view | yes | no (`lateral_align_skip_if_marker`); the KF + reverse controller correct the lateral |
| `lateral_align_tolerance` | 0.15 m | 0.25 m |
| `forward_cap_margin` (cap = 0.8 + margin) | 0.25 (1.05 m) | 0.10 (0.90 m) |
| `realign_forward_distance` | 0.65 m | 0.40 m; badly off: max(0.40, 1.5 × \|lateral\|) ≤ `realign_forward_max` 0.65 (and the cap) |
| realign forward leg | heading hold | `realign_steer`: arcs back toward the axis (gain/limit of LATERAL_ALIGN, both wheels ≥ floor) |
| ALIGNING target | dock yaw | rear camera on the marker's RTK bearing (`align_to_marker_bearing`, dock pose measured): the camera sees ±25° and reads 6.5° off, so at 0.25 m lateral the dock yaw put the marker out of frame |
| search timeout within `approach_skip_radius_m` (0.6 m) | forced Nav2 re-approach (offset retry) | once per approach: rotate in place to the dock yaw and search again (`inplace_search_retry`, no retry used); a second failed search gets the offset Nav2 retry |

These are unchanged: `approach_distance` 0.8, the both-wheels and 0.06 m/s floor rules, and
the corridor cap.

The trade-off: with 0.40 m realigns under a 0.90 m cap, starts that are extremely far off
no longer reach the 4 cm contact window in the unit closed loop. Those are the gate-edge
starts at 0.45–0.5 m lateral within 1 m, and the 2026-10-08 dusk start at 0.36 m lateral,
0.66 m out. They are marked `xfail(strict)` in `test_dock_logic.py`. Nav2 + ALIGNING have
not delivered starts like these in the field (≤ 0.25 m). From the approach pose,
LATERAL_ALIGN now has only 0.10 m of forward room, so in practice it runs only when the robot
is inside the approach distance with no marker in view.

Sim (`accuracy-simout/dock-tweak/`, `run_dock.sh`, dock-only `scenario dock`, start at
the approach pose facing the dock unless noted; forward = sim ground-truth forward travel):

| scenario | master time / forward | now time / forward |
|---|---|---|
| straight on (0.8, 0, facing the dock) | 20.8 s / 0.00 m | 20.7 s / 0.00 m |
| 0.25 m off axis | 104.7 s / 4.16 m (LATERAL_ALIGN to the cap, 2 Nav2 retries) | 33.1 s / 0.42 m (one pose-check realign) |
| 40° off heading, 0.21 m off (attempt 1) | 19.9 s / 0.47 m (LATERAL_ALIGN 0.43 m) | 24.9 s / 0.41 m (0 before DOCKING, one realign) |
| real_run5 ideal gate x1, dock phase | 14.1 s / 0 (e10ed95) | 15.8 s / 0, docked |

In the sim, the marker stays in view after master's 0.43 m LATERAL_ALIGN in the 40° case,
so master docks there. In the field it did not (0/96 frames).

Tests: `test/test_dock_tweak_2026_10_23.py` (one test per rule, plus a replay of attempt 1
with a 35° rear-camera field of view: no LATERAL_ALIGN, no Nav2, no forward drive before
DOCKING).

## Undock

The sequence is `CHARGER_OFF` (`/charging false`, waiting for the reply), then
`BACKING_UP`, then `WAITING_FOR_RTK` (optional), then `UNDOCKED`.

- **BACKING_UP**: drives `distance_m` at `speed_mps` (capped at 0.3 m/s, the vendor
  Undocking max). It holds heading on odom and measures distance on odom, with a
  time fallback. It talks to `/cmd_vel_docking` directly and does not need Nav2 or
  localisation. Note that "backing up" means leaving the dock. The robot reversed
  onto the dock, so it leaves forwards (+X, toward `undock_point`). Set
  `undock_direction: -1.0` to invert.
- **WAITING_FOR_RTK**: when `wait_for_rtk` is set, waits for `/gps/status`
  `fix_type == 3` until `rtk_timeout_s`. A timeout gives `success=false RTK_TIMEOUT`.

Lift or stop aborts with `EMERGENCY_STOP`. Feedback: `state`, `travelled_m`.

## ArUco marker

The vendor values were recovered from `libmower_charge_core.so`, by disassembling the
`docking::marker::marker()` constructor:

- **Dictionary**: `cv::aruco::getPredefinedDictionary(0)` = **`DICT_4X4_50`**.
- **Marker size**: `marker_size_ = 0.04f`, so **0.04 m** (the source comment says
  "方块的宽度4cm", "square width 4 cm"). Pose is estimated with
  `estimatePoseSingleMarkers`. The marker id is not checked (any id).
- **Vendor camera extrinsic params** (`camera_ext_{x,y,z,roll,pitch,yaw}`, defaults
  0.1, 0, 0, −π/2, 0, −π/2, which are inverted in the vendor code) and
  `filter_alpha` 0.2. These belong to the vendor's own marker-frame maths and are
  **not** used here. This package uses the URDF rear camera joint instead.

Both values are parameters (`aruco_dictionary`, `marker_size`, `marker_id`).
**Measure the marker on your charger**, but read "marker_size" above before changing
`marker_size`: distances scale linearly with it, and the docked offset and the
marker-frame thresholds were tuned at 0.04.

### Printing a replacement marker

```
ros2 run mower_docking print_marker --id 0 --size 0.04 --dpi 600 -o marker.png
```

1. Print at 100 % scale (no "fit to page"), on matte paper.
2. Check that the black square measures 40.0 mm.
3. Keep the white border: at least one module (1/6 of the square) is needed for detection.
4. Mount it flat and vertical on the charger, centred on the contact axis, facing the approaching robot.

### Calibration

1. **Intrinsics**: `/rear_camera/camera_info` must carry the real K/D. The
   `config/cameras/rear_camera_info.yaml` file holds 1920×1080 plumb_bob values.
   The stack runs the rear webcam at **640×480 @ 10 Hz** (`cameras.launch.py`); that
   4:3 mode is the centre 1440×1080 crop of the sensor scaled by 4/9 (measured
   2026-10-06), and `rear_camera_info_640x480.yaml` holds the K/D derived from the
   1080p calibration (fx ≈ 685 px, ~1 % accuracy). 16:9 modes are rescaled
   automatically; any other mode needs its own `rear_camera_info_<w>x<h>.yaml`.
2. **Extrinsics**: set `rear_camera_T_base` to `[x, y, z, roll, pitch, yaw]`. This is
   the pose of the camera **optical** frame in `base_link`, the same as the URDF
   `rear_camera_joint` (default `-0.201 0 0.25`, rpy `-π/2 0 π/2`: optical Z
   points to −X, image right is +Y). **TODO(calib)**: measure the real mount
   (height, and any downward tilt as extra pitch).
3. **`docked_marker_offset`** (placeholder 0.45 m):
   On the Tron the backlit dock marker is **cut off at the top of the rear image while
   the robot sits on the contacts** (checked 2026-10-06, 0 detections in 30 s), so it
   cannot be read there. Instead:
   1. Push the robot straight off the contacts by hand (forwards, motors idle) until
      `marker_check` (below) detects the marker, e.g. 0.3–0.5 m.
   2. Measure with a tape how far the robot moved from the docked position (`d`).
   3. The offset is `(-x) - d`, with `x` the median marker x from `marker_check`.

   If the robot stops short (FINAL_DOCKING timeouts), increase the offset. If it
   hits the dock while still in DOCKING, decrease it.
4. Check the sign conventions: with the marker straight behind, `marker_pose.y ≈ 0`.
   With the marker to the robot's left (+Y), `y > 0`. With the robot heading
   rotated left (CCW), the marker yaw becomes negative.

### Motion-free marker check

`docking_server` only looks at the camera during a goal. `marker_check` runs the same
detector / extrinsics / gates for a fixed time and commands nothing:

```
python3 -m mower_docking.marker_check --duration 30 [--save /tmp/marker.png]
```

It prints the processed and detection rates, the detector time per frame, the median
marker pose in `base_link` and whether the 0.5 m / 25° gates pass. Expected with the
marker 0.8 m behind the rear axle, centred and facing the robot: `x ≈ -0.80`,
`|y| < 0.05`, `|yaw| < 10°`, `gate_ok=True`. On the robot (RK3588, 640×480): ~11 ms per
frame for the detector. A 4 cm marker is ~35 px wide at 0.8 m, the yaw estimate is then
the noisy part (a few degrees; `test/test_marker_check.py`).

## Dock calibration (do this once, and after moving the charger)

The dock pose is not known until you record it. Until then `dock_pose.yaml` holds the
(0, 0, yaw 0) placeholder, `docking_pose_measured` is false, the docking server never
reverses blind, and heading_aligner does not seed the heading from the dock.

1. Make sure RTK is fixed (the GUI shows RTK FIX).
2. In manual mode, drive the robot onto the charger: reverse straight onto the contacts
   until the GUI shows it charging.
3. Press **Set docking point** in the GUI (map server `set_docking_point`). This records
   the docked position (and the yaw only if none was measured before: see "Dock yaw
   re-measurement" to correct a stale yaw), writes `dock_pose_measured: true` into
   `/userdata/ros2/maps/dock_pose.yaml`, and republishes the latched
   `/map_server_node/docking_pose` and `docking_pose_measured`.
4. Check the marker: from the approach pose (0.8 m straight out from the docked pose,
   same heading) the ArUco marker on the charger must be in the rear camera's view, with
   nothing between. Drive there in manual and run
   `ros2 topic echo /mower_docking/marker_pose` during a dock goal, or check that a dock
   reaches DOCKING instead of failing with `DOCK_NOT_FOUND: ... marker not visible`.

## Parameters

See `config/docking.yaml`. Every value is annotated with its vendor source. The
key ones are:

| Name | Default | Notes |
|---|---|---|
| `approach_distance` | 0.8 | vendor `undock_point` |
| `skip_nav_to_approach` | false | bench mode: assume the robot is at the approach pose |
| `max_speed` / `final_speed` | 0.15 / 0.05 | `max_speed` is hard-capped at 0.15 |
| `slow_distance` / `slow_zone` | 0.3 / 0.3 | 0.05 m/s within 0.3 m, linear ramp from 0.15 at 0.6 m |
| `final_zone` | 0.2 | switch to straight FINAL_DOCKING (vendor `final_distance_to_dock_` 0.2) |
| `final_timeout_s` / `final_max_distance` | 10 / 0.30 | vendor FINAL_DOCKING timeouts |
| `docking_timeout_s` | 60 | vendor "Docking timeout > 60s" |
| `max_lateral_error` / `max_yaw_error_deg` | 0.5 / 25 | vendor marker gate |
| `max_retries` | 3 | then `DOCK_MAXOUT` |
| `contact_debounce_samples` | 3 | vendor `is_dock_done_num` |
| `search_timeout_s` | 15 | then blind (if allowed) or `DOCK_NOT_FOUND` |
| `allow_blind_docking` | false | true = vendor blind reverse after any search timeout |
| `blind_marker_max_age_s` | 30 | marker must have been seen this recently (this attempt) for a blind final dock |
| `dock_pose_measured_override` | false | bench only: treat the dock pose as measured |
| `stall_min_cmd` / `stall_progress_ratio` / `stall_timeout_s` | 0.03 / 0.2 / 1.0 | stall guard on `/odometry/filtered` |

## Safety notes

- The node publishes on `/cmd_vel_docking` only. twist_mux arbitrates it: teleop (20),
  bumper (50) and emergency (100) override it, and the `/estop` lock blocks it.
  The MCU firmware still owns the hard e-stop and lift cut-off.
- The robot never moves faster than 0.15 m/s in reverse, or 0.05 m/s for the final
  0.3 m (vision mode; blind mode uses the planned distance). Every motion phase is bounded by both distance (odom) and time, so a
  missing contact ends in RETRY and then FAILED, not in pushing against the dock.
- If odom is missing, distance is integrated from time × commanded speed, which
  overestimates travel if the wheels slip. This errs on the side of stopping early.
- Lift or stop, a stale base status, a cancel, or a server shutdown all publish zero
  velocity. Every exit publishes zeros for 0.5 s.
- The blade is not touched here. The mission layer must close the cutter before
  docking (vendor `CloseCutterCheck`).
- `/charging true` is only sent after a debounced contact. `/charging false` is
  sent before any undock motion, and undocking aborts if that call fails.

## Bench test (no hardware)

`scripts/bench_fake_base.py` does four things:

- integrates `/cmd_vel_docking` into `/odom`;
- reports `is_docking_done` while x ≤ 1 cm, with the dock at x = 0 acting as a hard stop;
- serves `/charging`;
- latches the docking pose at the origin.

```
python3 install/mower_docking/share/mower_docking/scripts/bench_fake_base.py --start-docked &
ros2 launch mower_docking docking.launch.py skip_nav_to_approach:=true &
ros2 action send_goal /mower_docking/undock mower_interfaces/action/Undock \
  "{distance_m: 0.8, speed_mps: 0.15, wait_for_rtk: false}" --feedback
# blind docking is off by default: set allow_blind_docking: true in docking.yaml for the bench
ros2 action send_goal /mower_docking/dock mower_interfaces/action/Dock "{use_vision: false}" --feedback
```

## Tests

```
colcon test --packages-select mower_docking && colcon test-result --verbose
# or, without ROS:  cd src/mower_docking && python3 -m pytest test
```

`test_dock_logic.py` is pure Python. `test_aruco_detect.py` renders a synthetic
marker and checks detection and pose. It is skipped when `cv2.aruco` is missing.

License: Apache-2.0.
