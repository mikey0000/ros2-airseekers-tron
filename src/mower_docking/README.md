# mower_docking

Docking / undocking action server for the Airseekers Tron (ROS 2 Humble, rclpy).
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
SEARCHING ──no marker in search_timeout_s──► FINAL_DOCKING (blind: reverse
  │ marker passes the 0.5 m / 25° gate           approach_distance + 0.1 m on odom)
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
       After max_retries (3): FAILED "DOCK_MAXOUT".
```

The following guards apply in every state:

- `stop_triggered` or `lift_triggered` aborts with `EMERGENCY_STOP`.
- A stale `/mower_base/status` (> 1 s) while moving aborts with `BASE_STATUS_STALE`.
- Exceeding the goal `timeout_s` aborts with `DOCK_TIMEOUT`.
- Cancelling the action returns `CANCELED`, and the Nav2 goal is cancelled too.

The robot stays still while the marker is lost. It never reverses on a stale estimate.

Result messages: `DOCKED`, `ALREADY_DOCKED`, `NAV_TO_DOCK_FAILED`, `DOCK_MAXOUT`,
`DOCK_TIMEOUT`, `EMERGENCY_STOP`, `BASE_STATUS_STALE`, `CHARGER_ENABLE_FAILED`,
`CANCELED`.

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

With `v < 0`, the desired heading is `h_d = clamp(k_lateral·lateral, ±0.35)` and
`ω = kp_heading·(h_d − heading) + kd_heading·d/dt`, clamped to ±0.3 rad/s. The
sign is derived for reversing: `ẏ = v·sin h`. A closed-loop unicycle test in
`test/test_dock_logic.py` confirms that it converges.

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
**Measure the marker on your charger.** If its black square is not 40 mm, set
`marker_size` to the measured value. Distances scale linearly with it.

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
   If you change the resolution, re-calibrate with `camera_calibration`.
2. **Extrinsics**: set `rear_camera_T_base` to `[x, y, z, roll, pitch, yaw]`. This is
   the pose of the camera **optical** frame in `base_link`, the same as the URDF
   `rear_camera_joint` (default `-0.201 0 0.25`, rpy `-π/2 0 π/2`: optical Z
   points to −X, image right is +Y). **TODO(calib)**: measure the real mount
   (height, and any downward tilt as extra pitch).
3. **`docked_marker_offset`** (placeholder 0.45 m):
   1. Put the robot on the contacts by hand, so that `is_docking_done` is true.
   2. Start a vision dock goal, or just enable vision, and `ros2 topic echo /mower_docking/marker_pose`.
   3. Read `-x` of the marker (for a centred marker with yaw ≈ 0). That is the offset.

   If the robot stops short (FINAL_DOCKING timeouts), increase the offset. If it
   hits the dock while still in DOCKING, decrease it.
4. Check the sign conventions: with the marker straight behind, `marker_pose.y ≈ 0`.
   With the marker to the robot's left (+Y), `y > 0`. With the robot heading
   rotated left (CCW), the marker yaw becomes negative.

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
| `search_timeout_s` | 15 | then dead-reckon |

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
