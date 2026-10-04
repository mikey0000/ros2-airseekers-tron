# Localization & control plan

What we adopt, adapt, and reject from `mowglinext_baseline.md` for localization and control,
and where the stack actually stands.

| Area | Package | Status |
|---|---|---|
| GNSS gate | `mower_localization` (`gps_gate`) | implemented, 25 tests pass, never run on hardware |
| EKF parameters | `mower_localization` (`config/ekf.yaml`) | written, **blocked on the IMU issue in §3** |
| GPS fusion | `mower_localization` (`config/navsat.yaml`) | written, **blocked on the same issue** |
| Launch wiring | `launch/nav2.launch.py` | localization wired end to end; Nav2 controller block is scaffolding |
| `cmd_vel` shaping | `mower_control` (`cmd_vel_slew`) | implemented, not wired into bringup |
| Slip detection | `mower_control` (`slip_detector`) | implemented, not wired into bringup |
| IMU calibration | `mower_control` (`imu_cal`) | implemented, not wired into bringup |

Nothing here has touched the mower. `mower_localization` and `mower_control` are new since
the last commit and were verified with `python3 -m pytest` only — no `rclpy`, no `colcon`,
no serial port. References to `robot_localization` behaviour below were checked against the
upstream `humble-devel` source, not from memory.

---

## 1. The baseline's localization story, and where we diverge

MowgliNext replaced SLAM with a **GTSAM iSAM2 factor graph** (`fusion_graph_node`) that owned
*both* `map→odom` and `odom→base_footprint`. They explicitly removed `robot_localization`'s
dual-EKF, `navsat_transform_node` and Kinematic-ICP (baseline §3).

We go the other way: **`robot_localization` EKF + `navsat_transform_node`**, which is
baseline §7.2's own stated fallback ("`mower_localization` (or reuse `robot_localization` +
`navsat_transform` on Humble)"). Why:

- GTSAM has no Humble binary and needs an aarch64 source build (baseline §7.2) — a long
  build on a board where `docs/deployment.md` treats <10 GB free on `/userdata` as the
  go/no-go line.
- Our IMU is a magnetometer-noisy JY61P on a mower deck, not their MCU-on-same-link.
- A factor graph earns its complexity when fusing scan matches. We have **no 2D LiDAR at all**
  (baseline §7.2: "Airseekers has no 2D LiDAR"). Three sources — wheel odom, IMU, RTK — is
  the case an EKF handles well.
- Revisit if Metoak stereo depth becomes a second localization back-end, or if scan matching
  arrives.

### Invariants kept regardless of which filter we run

| Invariant | Why it matters here |
|---|---|
| **Sole TF ownership.** One publisher per transform; drivers publish no TF. | MowgliNext enforces it with a CI test that fails the build if any other file constructs a `TransformBroadcaster` (baseline §3). Worth adopting here. `wit_imu_driver` broadcasts `base_link→imu_link` — static hardware geometry, the acceptable exception, but it must be the only one. |
| **Map frame == GPS frame**, anchored by datum only. No SLAM back-end. | Why `slam_toolbox` is absent from the apt list in `nav2.launch.py`. Note this is currently an *intent*, not a fact — see §3. |
| **REP-105 chain**, `base_link` at the rear drive axle centre, not chassis centre (`chassis_center_x` ≈ 0.18 m). | A 0.18 m frame error is a 0.18 m swath error. Belongs in the URDF, which lands with `mower_bringup`. |
| **Gate on the receiver's own truth**, never on a fused covariance. | The `gps_gate` design (§2), and the same rule as their `dig_detector` (`DigTrustSigma` from `/gps/status`, baseline §4.3 item 8). |

### Topic flow as actually wired

```
  mower_mcu_driver ──/odom (50 Hz)─────────────┐
                                                │
  wit_imu_driver ──/imu (100 Hz)─► /imu/data ──┼──► ekf_node ──► TF odom → base_link   (sole publisher)
                                                │        │
  um960_gps_driver ──/fix ──► gps_gate ──/fix_gated        ├──► /odometry/filtered
                             │               │            └──► /odometry/filtered_map
                             │               └──► navsat_transform_node ──► /odometry/gps
                             └───► /gps_gate/{counters, warnings}                       │
                                                                                     │
                                                          ekf_node odom1 ◄─────────┘
```

Frames actually published: `odom → base_link` only. `map → odom` has **no publisher yet**
(§3 item 2). `base_link → {imu_link, gps}` come from the URDF that `mower_bringup` will
supply, except `imu_link` which `wit_imu_driver` broadcasts itself.

Verified against the upstream source, and worth writing down because it is easy to get wrong:

- `ekf_node` publishes `odometry/filtered` and `odometry/filtered_map`; those names are
  **relative**, so they resolve to `/odometry/filtered` etc. There is no `/odom/filtered`.
- `navsat_transform_node` subscribes to `gps/fix`, `imu` and `odometry/filtered` — the
  EKF's output, **not** the raw `/odom`. It derives `world_frame_id_` and
  `base_link_frame_id_` from that message's `header.frame_id` / `child_frame_id`, so it
  accepts **no frame parameters at all**. `nav2.launch.py` therefore passes none.
- It publishes `odometry/gps` as a **`nav_msgs/Odometry`**, not
  `PoseWithCovarianceStamped`. That is why `ekf.yaml` takes it as `odom1` (an
  odometry input) rather than `pose0`: `poseN` inputs are typed
  `PoseWithCovarianceStamped`, so a `pose0: /odometry/gps` would silently never connect.
  This matches upstream's own `dual_ekf_navsat_example.yaml`.

---

## 2. `gps_gate`

`src/mower_localization/mower_localization/gps_gate.py`. Republishes `/fix` as `/fix_gated`
only when the fix is worth fusing.

`navsat_transform_node` will fuse a single-point fix without complaint, and a 1-2 m error in
the projected pose becomes a metre-scale error in where Nav2 thinks the robot is. Gated
silence is better than a bad fix: the EKF coasts on wheel odometry, which drifts slowly and
recoverably, instead of jumping.

Decision inputs, in failure-first order:

1. `NavSatFix.status.status >= min_fix_status` (default `STATUS_FIX`) — keeps no-fix,
   estimated and simulated solutions out.
2. `status.status` ∈ `used_fixes` (default GPS / DGPS / RTK-fixed / RTK-float; `[4]` for
   RTK-fixed-only).
3. Largest diagonal of `position_covariance` ≤ `max_position_covariance` (default 100 m²,
   a 10 m sigma), checked only when the receiver reported a known covariance.
   `COVARIANCE_TYPE_UNKNOWN` means "no information", so the message passes.

All three read the message itself. Nothing consults a filter that already consumed the fix —
which is the baseline's dig-detector rule applied to a different input.

Rejected fixes are counted (`received`/`passed`/`rejected`) and logged, throttled to one
warning per 5 s, and not published. Those counters are the bring-up signal; a `/diagnostics`
hookup belongs with `mower_monitoring`.

QoS is reliable KEEP_LAST(10) on both ends, matching `um960_gps_driver`'s `/fix`. A
best-effort publisher here would silently drop every fix.

**Known limit:** `NavSatFix` carries no satellite count, so a "satellites used" gate cannot
live here. The UM960 parsers track `num_sats_used` and `/fix_status` exposes it
(`sats=18/21`), so a richer gate would have to read the status string instead of the fix
message.

---

## 3. EKF and GPS fusion — and what blocks them

`config/ekf.yaml`: 30 Hz, `two_d_mode: true`, `world_frame: odom`, `publish_tf: true`.

| Source | Fused state | Note |
|---|---|---|
| `odom0: /odom` (50 Hz) | x, y, vx, vy, yaw rate | Pose yaw deliberately **not** fused: `/odom`'s heading is the JY61P (via `use_imu_yaw`), the same sensor as `imu0`, so fusing both double-counts it. |
| `imu0: /imu/data` (100 Hz) | yaw, yaw rate, ax, ay | See the blocker below — the yaw it fuses is currently always zero. |
| `odom1: /odometry/gps` (~10 Hz) | x, y | GPS heading is worse than the IMU, so no yaw from it. |

`use_control: false`. `/cmd_vel` goes to the MCU, not the filter; feeding it in would fight
the firmware's own deadband.

`config/navsat.yaml`: 10 Hz (matching the receiver), `use_odometry_yaw: false`,
`broadcast_utm_transform: false`, `wait_for_datum: false` — the gate already drops
untrustworthy fixes, and waiting on datum would stall the whole localizer until the RTK
session converged.

### Blocker: `wit_imu_driver` does not publish an orientation

This is the single thing to fix before any of the above means anything.

`wit_imu_driver` parses the JY61P 0x53 angle frame (roll/pitch/yaw) and then
`fill_angle_from_tracked()` **only logs it at debug level** — `imu_msg.orientation` is never
assigned and stays all zeros. It also publishes all-zero covariances for orientation, angular
velocity and linear acceleration.

Three consequences, all quiet rather than loud:

- `navsat_transform_node` builds its GPS-to-world transform from that orientation. A zero
  quaternion yields a wrong heading for the entire GPS correction.
- `ekf_node` fuses `imu0` pose **yaw** (`ekf.yaml` line 60 enables it) — from zeros. It
  predicts yaw forward from `odom0`'s yaw rate, so the filter looks healthy while the absolute
  reference is fiction.
- Zero covariances mean "perfect measurement" to `robot_localization`, so the filter
  over-trusts every IMU-derived quantity.

The fix is in the driver, not the configs: build `Imu.orientation` as a quaternion from the
0x53 frame, and report realistic per-axis covariances. Until then, treat any localization
output as unvalidated.

Note also that `robot_localization` honours the `sensor_msgs` convention that a `-1` first
covariance element means "this IMU does not provide orientation" and skips it. Zero is the
worst possible value: it claims both availability and certainty.

### Other open issues

1. **No `map → odom` publisher.** `navsat_transform_node` only broadcasts a transform when
   `broadcast_utm_transform` is true, and `navsat.yaml` sets it false. `ekf_node` is
   configured `world_frame: odom`, so per its own documented two-mode behaviour it broadcasts
   `odom → base_link`, not `map → odom`. The consequence is real: the stack localizes in
   `odom` only, so it is continuous and GPS-corrected but anchored wherever the machine was
   switched on. Nav2's map-localized behaviour — and any map↔field-boundary correspondence —
   needs this transform. Options are the upstream dual-EKF pair (a second map-frame EKF) or
   turning the UTM broadcast on and treating `utm` as the map frame. Decision deferred until
   `mower_bringup` lands the URDF and `nav2_params.yaml`.
2. **`/imu` vs `/imu/data`.** `wit_imu_driver` publishes `/imu`; `ekf.yaml`, `imu_cal` and
   Nav2 all expect `/imu/data`. `nav2.launch.py` remaps it, but only within that file — any
   node started outside it sees `/imu`. Pick one canonical name and remap at the source.
3. **Datum is unset.** The first fix becomes the datum, which is fine for a first bring-up
   and wrong for repeatable field boundaries. Set it in `navsat.yaml` or call the `datum`
   service.
4. **`mower_mcu_driver` publishes no TF.** Correct by Invariant 2, but it means `/odom` is a
   message with no matching transform until `ekf_node` runs.
5. **Magnetic declination is 0.0**, which is wrong for any real site and biases the whole
   heading solution by that angle. Same for the antenna `yaw_offset`. Both are TODOs carried
   in `navsat.yaml`.
6. **No RTK corrections by default.** NTRIP is out of scope (`docs/um960.md`), so expect
   single-point fixes (~2.5 m). `gps_gate`'s covariance threshold is the guardrail; anything
   gating on fix *quality* — the dig detector, for one — should check RTK-fixed status in
   `/fix_status`, never the filter's own covariance.
7. **`ros-humble-robot-localization` is not in `docker/Dockerfile.humble`.** Without it the
   two parameter files have nothing to load them.

---

## 4. Control: what the baseline taught us

`mower_control` ports three baseline §4.3 patterns. Details in `docs/control.md`.

| Node | Baseline pattern | Adopted |
|---|---|---|
| `cmd_vel_slew` | §4.3 item 9 — shape before the wire, immediate exact stop, echo the exact encoded command | Rate-limited accel/decel, watchdog that zeroes rather than slews down, exact-echo logging. |
| `slip_detector` | §4.3 item 8 — `dig_detector.hpp`: commanded vs measured twist, gated on RTK-fixed, latched event | Commanded vs measured twist, RTK-gated, latched `/dig_stall` (`QoS(1).transient_local`). |
| `imu_cal` | §4.3 item 7 — at-rest bias cal with a persisted, validated file | Plausibility rules per window, atomic YAML write with a `# mower_control_imu_calibration_v1` header, validation on load. |

### Deliberately not ported

- **Host-side wheel PID.** The stock vendor node runs it from `libpid_controller.so`; the MCU
  accepts linear/angular only. `mower_mcu_driver` applies a plain scale+clamp and documents
  the gap. Baseline §4.3 item 3 sides with the vendor: the fast loop belongs in firmware.
  Closing this means reverse-engineering gains, not porting code.
- **Deadband/gain sanity gate** (§4.3 item 4 — a real 2026-09-15 stiction-lock failure).
  Worth building, but it needs a `0x54`-equivalent runtime-tuning path the vendor protocol may
  not offer. Deferred, not dismissed.
- **Escalation latch clearing.** Their `dig_escalation.hpp` clears the dig latch on charger
  contact or 2× escape displacement. `slip_detector` never clears its own latch; that needs a
  `dig_escalation` node.

### Controller choices carried from baseline §6

- **`odom_topic` must be set explicitly.** Nav2's default `/odom` has no publisher here and
  the robot silently refuses to move. `nav2.launch.py` points it at `/odometry/filtered`.
- **Transit** = RegulatedPurePursuit behind RotationShim, so a heading change costs a
  rotate-in-place rather than a wide loop.
- **`FTCController` is the piece that gives sub-centimetre swath accuracy**, and it is custom
  C++ in `mowgli_nav2_plugins`. Not in Humble; a from-source port into `mower_nav2_plugins`.
  Until it lands, coverage accuracy will not match the baseline's claims.
- **Velocity chain**: `Nav2 → cmd_vel_nav → collision_monitor → cmd_vel_monitored →
  cmd_vel_slew → mower_mcu_driver`. Tune mux timeouts so a dead `collision_monitor` cannot
  keep refreshing the firmware watchdog (§4.3 item 11).
- **Docking has no Humble `docking_server`.** Undock must be `nav2_msgs/BackUp`; docking
  needs a custom BT node or a small state machine in `mower_behavior`.
- **No 2D LiDAR.** Their `scan_deskew_node` / `costmap_scan_filter_node` chain has no sensor
  to condition here. Obstacle input will be Metoak stereo depth converted to
  `sensor_msgs/PointCloud2` for a costmap layer (`metoak_stereo_driver`, baseline §7.2) —
  new work with no upstream analogue, so it needs its own risk register.

---

## 5. Next steps, in dependency order

1. **Fix `wit_imu_driver` orientation + covariances** (§3 blocker). Everything downstream
   inherits this error; no filter tuning matters until it is right.
2. Decide the `map → odom` topology (§3 item 1) — dual EKF, or `utm` as the map frame.
3. Settle `/imu` vs `/imu/data` at the source (§3 item 2).
4. URDF for a Tron chassis variant: `base_link` at the rear axle, `gps` and `imu_link`
   extrinsics measured rather than guessed. Without a real `base_link → gps` transform,
   `navsat_transform_node` logs the lookup failure and assumes the receiver sits at the robot
   origin — a silent lever-arm error.
5. Measure site declination and the antenna `yaw_offset`; set the datum.
6. Land `mower_bringup`: URDF, `nav2_params.yaml` (Humble key names checked one by one — a
   wrong key is silently ignored), `twist_mux` lanes.
7. Wire `cmd_vel_slew` / `slip_detector` / `imu_cal` into bringup so `/cmd_vel` goes through
   the shaper.
8. Validate on a **recorded bag** before any live drive: replay and compare
   `/odometry/filtered` against ground truth, gate behaviour under simulated RTK loss, and
   slip detection from the field-recorded slip captures.
9. Only then on hardware: wheels up, blade off, RTK confirmed `RTK_FIXED` in `/fix_status`.

Characterise, don't assume (baseline takeaway 6): the heartbeat timeout, the MCU estop
passthrough, and the wheel PID gains are all still open unknowns, and all three matter more
than any filter tuning here.