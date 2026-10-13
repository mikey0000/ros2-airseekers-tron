# Localization — EKF fusion of `/odom`, `/fix` and `/imu/data`

`ros2_stack/src/mower_localization` — the localization layer of the
Airseekers Tron ROS 2 (Humble) stack. Three artefacts:

| Artefact | What it is |
|---|---|
| `config/ekf.yaml` | `robot_localization` `ekf_node` parameters: fuses `/odom`, `/imu/data` and the `navsat_transform_node` output |
| `config/navsat.yaml` | `navsat_transform_node` parameters: projects the gated `/fix` into an odom-frame 2D pose (`/odometry/gps`) |
| `mower_localization/gps_gate.py` | `gps_gate` node: republishes `/fix` as `/fix_gated` only when it is a usable fix |

This is the Humble-era answer to MowgliNext's `fusion_graph`
(GTSAM iSAM2 sole localizer): an EKF now, a factor graph later —
the baseline keeps that door open (`docs/mowglinext_baseline.md`
§7.2). The frame tree and the "drivers publish no TF" rule are
inherited from the baseline (§3, Invariant 2).

Topic names below are the *Humble* `robot_localization` defaults,
read from the `humble-devel` source (`navsat_transform.cpp`,
`ros_filter.cpp`) and the package's own
`dual_ekf_navsat_example.yaml` — the canonical GPS wiring.

## Data flow

```
Inputs                       ekf_node (30 Hz)                 navsat_transform_node (10 Hz)
────────                     ──────────────                   ──────────────────────────────
/odom (50 Hz, mcu) ────────▶ odom0
/imu/data (100 Hz, wit) ───▶ imu0                            imu = /imu/data (remap)
                             publishes /odometry/filtered ───▶ odometry/filtered (default
                             + TF odom -> base_link            topic, no remap needed)

/fix (10 Hz, um960)          odom1 = /odometry/gps ◀───────── /odometry/gps
  └─ gps_gate ─▶ /fix_gated ─(remap gps/fix:=/fix_gated)────▶ (nav_msgs/Odometry, ~10 Hz)
```

Startup order matters: `ekf_node` first (it produces
`/odometry/filtered`, which `navsat_transform_node` consumes),
then `navsat_transform_node`. The EKF runs without GPS — it just
has no absolute correction until the NavSat transform is computed
(one good fix + IMU + odometry).

The `gps_gate` node is the only Python in this package;
`ekf_node` and `navsat_transform_node` are C++ nodes from
`robot_localization`, configured by the two YAML files above.

## Frame tree

```
map
 └── odom            <- NOT published by this stack (yet): belongs to the
                        localization back-end (MowgliNext: fusion_graph owns
                        map->odom exclusively)
      └── base_link  <- ekf_node (publish_tf: true); sits at the centre of
                        the rear drive axle, not the chassis centre
                        (mowglinext_baseline.md §3)
           ├── imu_link   <- WIT JY61P; wit_imu_driver broadcasts it today
           │                (the one exception to Invariant 2 to revisit)
           └── gps        <- UM960 antenna (/fix header.frame_id)
```

- REP-105 chain `map → odom → base_link` (baseline §3).
- `world_frame: odom` in `ekf.yaml` means the EKF fuses in "odom
  world" and only owns `odom → base_link`. Setting
  `world_frame: map` would make it own `map → odom` too —
  deliberately not used: that transform belongs to the localizer
  back-end (baseline §3, Invariant 2: only the localizer may
  broadcast `map→odom` or `odom→base_link`).
- Until `map → odom` exists, the stack is localized in `odom`
  only: continuous, GPS-corrected dead reckoning. Nav2's
  map-localized behaviour needs the next step — anchor `map` to
  the GPS datum (baseline: "map frame == GPS frame, anchored only
  by `datum_lat`/`datum_lon`").
- `navsat_transform_node` also needs `base_link → gps` and
  `base_link → imu_link` to account for the antenna and IMU
  mounting offsets; without `base_link → gps` it logs "Unable to
  obtain base_link -> gps transform" and assumes the antenna is at
  the robot origin. The URDF (`mower_bringup`, later batch) is
  the right home for both.

## Topic contract and rates

| Topic | Type | Publisher | Rate | Consumer |
|---|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | `wit_imu_driver` (its `/imu` remapped) | **100 Hz** | `ekf_node` (`imu0`), `navsat_transform_node` (`imu`) |
| `/fix` | `sensor_msgs/NavSatFix` | `um960_gps_driver` | **10 Hz** | `gps_gate` |
| `/fix_gated` | `sensor_msgs/NavSatFix` | `gps_gate` | ≤ 10 Hz — only good fixes | `navsat_transform_node` (`gps/fix`) |
| `/odom` | `nav_msgs/Odometry` | `mower_mcu_driver` | **50 Hz** | `ekf_node` (`odom0`) |
| `/odometry/filtered` | `nav_msgs/Odometry` | `ekf_node` | 30 Hz | `navsat_transform_node` (default `odometry/filtered` input) |
| `/odometry/gps` | `nav_msgs/Odometry` | `navsat_transform_node` | ≈ 10 Hz | `ekf_node` (`odom1`) |
| TF `odom → base_link` | — | `ekf_node` | 30 Hz | everything that consumes odometry |

Two remaps are required by this wiring (all other topics are
absolute strings in source, per baseline §5.3):

1. `wit_imu_driver` publishes `/imu`; `navsat_transform_node`
   subscribes to `imu` — remap `imu:=/imu/data` at bringup.
   (`ekf_node` takes `/imu/data` as the `imu0` *parameter*, so
   no remap is needed on that side.)
2. `navsat_transform_node` subscribes to `gps/fix`, but it must
   consume the *gated* stream — remap `gps/fix:=/fix_gated`.

`/odometry/gps` is a `nav_msgs/Odometry` message, so it feeds the
EKF's **`odom1`** input — exactly the wiring of
robot_localization's own `dual_ekf_navsat_example.yaml`.
(The EKF's `pose0` input subscribes to
`geometry_msgs/PoseWithCovarianceStamped` and would not connect —
a classic silent wiring mistake.)

## `gps_gate` — the fix-quality gate

`navsat_transform_node` will happily fuse a single-point fix, and
a 1–2 m wander in the projected pose moves the robot's whole idea
of where it is — and, because the *first* fix becomes the datum, a
bad first fix anchors the world wrongly. `gps_gate` sits between
the UM960 driver and the NavSat transform so the localizer only
ever sees a fix worth localising from. A fix passes when **all**
of these hold:

| Rule | Parameter | Default |
|---|---|---|
| `NavSatFix.status.status >= min_fix_status` | `min_fix_status` | `1` (`NavSatStatus.STATUS_FIX`) |
| largest diagonal `position_covariance` ≤ threshold (m²), when the receiver reports one | `max_position_covariance` | `100.0` (10 m sigma; `0` disables) |
| `status.status` is one of the used fixes | `used_fixes` | `[STATUS_FIX, STATUS_DGPS_FIX, STATUS_RTK_FIX, STATUS_RTK_FLOAT]` (`[]` disables) |

- The decision uses the message's own `status` and
  `position_covariance` — the receiver's reported truth — never a
  filter that has already consumed the fix (the same rule the
  baseline's dig detector applies to `/gps/status`, §4.3.8).
- Rejected fixes are counted (`received`/`passed`/`rejected`
  attributes) and logged, throttled to one warning per 5 s.
  Nothing is published, so downstream nodes see a clean stop —
  the EKF then coasts on `/odom` + `/imu/data` until a good fix
  returns.
- `COVARIANCE_TYPE_UNKNOWN` (what `um960_gps_driver` reports when
  the receiver gave no accuracy numbers) cannot be checked and
  passes through; the UM960 driver is trusted to stop `/fix`
  entirely when the receiver has no solution.
- `NavSatFix` carries no satellite count, so a "satellites used"
  gate has to live in the driver (the UM960 parsers already track
  `num_sats_used`).
- QoS is reliable, KEEP_LAST(10) on both ends, matching the
  UM960 driver's `/fix` profile. `navsat_transform_node`
  subscribes with `SensorDataQoS` plus QoS overriding, so a
  reliable publisher connects to it fine.

```bash
ros2 run mower_localization gps_gate
# RTK-fixed-only operation:
ros2 run mower_localization gps_gate --ros-args \
  -p used_fixes:='[4]' -p max_position_covariance:=0.04
```

## `ekf.yaml` — `ekf_node` parameter reference

| Parameter | Value | Why |
|---|---|---|
| `frequency` | `30.0` | filter rate; faster than the slowest source |
| `sensor_timeout` | `0.2` | 2× the 10 Hz GPS period, so receiver jitter is not a timeout, but a dead IMU (100 Hz) is flagged in 0.2 s |
| `two_d_mode` | `true` | planar mower: z, roll, pitch are not estimated |
| `publish_tf` | `true` | this node owns `odom → base_link` |
| `map_frame` / `odom_frame` / `base_link_frame` | `map` / `odom` / `base_link` | REP-105 chain (must be unique — the node refuses non-unique frames) |
| `world_frame` | `odom` | fuse in the odom frame; no `map → odom` from this node |
| `odom0` `/odom` | twist vx + yaw rate | pose x,y are *not* fused (GPS `odom1` is the one absolute x/y source; see "ekf_node diagnostic warning" below). Pose yaw is *not* fused: `/odom`'s heading is the JY61P (`mcu_node use_imu_yaw`), the same sensor as `imu0`, so fusing it double-counts it. Twist vy is *not* fused: `mcu_node` marks it unobserved (variance 1e6) |
| `imu0` `/imu/data` | orientation yaw, yaw rate, gravity-removed accel x,y | `imu0_remove_gravitational_acceleration: true` |
| `odom1` `/odometry/gps` | pose x,y only | the NavSat transform output (`nav_msgs/Odometry`, hence an *odom* input, not `pose0`); GPS heading is far worse than the JY61P yaw |
| `use_control` | `false` | `/cmd_vel` goes to the MCU, not the filter |
| `process_noise_covariance` | 15×15 | robot_localization's documented template values |

Both `odom0`/`imu0`/`odom1` subscriptions use
`SensorDataQoS` (best-effort) inside `robot_localization`, so the
drivers' publishers (reliable `/odom`, best-effort `/imu`,
reliable `/odometry/gps`) all connect — the only incompatible pair
in ROS 2 is a *best-effort publisher into a reliable subscriber*.

### ekf_node diagnostic warning (fixed 2026-10-06)

`/diagnostics` showed `ekf_node: Filter diagnostic updater` at WARN with
"Potentially erroneous data or settings detected for a robot_localization state
estimation node" (the GUI shows it on the Diagnostics page). robot_localization
(`src/ros_filter.cpp`, 3.5.4) raises this summary whenever any static (config) or
dynamic (per-message) diagnostic is pending. The key/values of the status name the
causes. Reproduced in the dev container with fake `/odom` and `/imu/data` that carry
the drivers' real covariances (old `ekf.yaml`, `ros2 run robot_localization ekf_node`):

| Key | Kind | Cause |
|---|---|---|
| `X_configuration`, `Y_configuration` | static, raised at startup | "2 absolute pose inputs detected for X/Y". `odom0` fused wheel pose x,y *and* `odom1` fused GPS x,y, both non-differential. Two absolute sources that drift apart make the filter oscillate between them. |
| `odom0_twist_covariance` | dynamic, every `/odom` | "covariance at position (7) … Y_VELOCITY … extremely large (1e+06), but the update vector … is set to true". `odom0` fused vy, but `mcu_node` publishes vy as unobserved (`_TWIST_COVARIANCE_MEASURED`, 1e6). |

Fix (`src/mower_localization/config/ekf.yaml`): `odom0` now fuses twist vx and yaw
rate only. Wheel pose x,y add nothing over vx + heading, which the filter integrates
itself. Fusing vy at variance 1e6 constrained nothing. The old config also had the
odom yaw-rate flag in the last row (linear acceleration z, which robot_localization
ignores for Odometry inputs), so the MCU yaw rate was never fused. It is now in the
twist roll/pitch/yaw row.

After the fix, the same 20 s check through
`ros2 launch mower_bringup nav2.launch.py use_robot_state_publisher:=false` shows only
`Filter diagnostic updater: ... appears to be functioning properly` (level 0). The
`odometry/filtered topic status: Frequency too low` WARN seen in the first ~4 s of
both runs is the startup window of the frequency monitor, not a config problem.

Follow-up (not done here): `mcu_node` could publish a small vy variance (for example
0.001). `odom0` could then fuse vy = 0 as the diff-drive non-holonomic constraint.

## `navsat.yaml` — `navsat_transform_node` parameter reference

| Parameter | Value | Why |
|---|---|---|
| `frequency` | `10.0` | matches the UM960 `/fix` rate (`publish_rate_hz`) |
| `delay` | `0.0` | robot_localization's own example uses 3.0 s to let the EKF settle; raise it if the first transforms look wrong |
| `magnetic_declination_radians` | `0.0` | the JY61P yaw is already magnetometer-referenced; TODO(site): measure on the mower if the IMU is ever reconfigured for raw heading |
| `yaw_offset` | `0.0` | TODO(calib): antenna heading offset about Z |
| `use_odometry_yaw` | `false` | heading comes from the IMU; `true` would take it from `/odometry/filtered` (valid only if that yaw is world-referenced) and ignores the IMU entirely |
| `zero_altitude` | `true` | 2D filter; nothing consumes altitude |
| `broadcast_utm_transform` | `false` | no `utm` frame in our tree |
| `publish_filtered_gps` | `false` | diagnostics come from `gps_gate` and `/fix_status` instead |
| `wait_for_datum` | `false` | do not block bringup while the RTK session converges — `gps_gate` drops fixes until they are trustworthy anyway |
| `datum` | unset | **the first fix becomes the datum.** Pin it with `wait_for_datum: true` + `datum: [lat, lon, yaw]` (the third field is a *heading* in radians, ENU — 0 = east), or call the `/datum` service at bringup. `nav2.launch.py` sets both from the `datum_lat`/`datum_lon`/`datum_yaw` args. `mower.launch.py` fills unset args from `/userdata/ros2/datum.env`, which the GUI's "set datum from GPS" writes (`gui_bridge` `set_datum`, which also calls `/datum` live) |

Frames do **not** come from parameters on this node: it takes
`world_frame`/`base_link_frame` from the `header.frame_id` /
`child_frame_id` of the `/odometry/filtered` messages it consumes
— which is why the EKF must run first.

## Run it

```bash
cd ros2_stack && ./scripts/build.sh --packages-select mower_localization
# inside the container (./scripts/run_stack.sh):
ros2 run mower_localization gps_gate
ros2 run robot_localization ekf_node --ros-args \
  --params-file install/mower_localization/share/mower_localization/config/ekf.yaml
ros2 run robot_localization navsat_transform_node --ros-args \
  --params-file install/mower_localization/share/mower_localization/config/navsat.yaml \
  --remap gps/fix:=/fix_gated \
  --remap imu:=/imu/data
```

Check it:

```bash
ros2 topic hz /imu/data          # 100
ros2 topic hz /fix               # 10
ros2 topic hz /odom              # 50
ros2 topic hz /fix_gated         # <= 10, only while fixes are good
ros2 topic hz /odometry/filtered # 30
ros2 topic hz /odometry/gps      # ~10
ros2 topic info /fix_gated       # QoS reliable/10 on both ends
```

## Caveats and TODOs

- **`robot_localization` is not in the Docker image yet.** Add
  `ros-jazzy-robot_localization` to the apt list in
  `docker/Dockerfile.jazzy`; without it the two parameter files
  have nothing to load them.
- **IMU covariances.** `wit_imu_driver` currently publishes
  all-zero IMU covariances, which makes the EKF treat every fused
  IMU variable as a perfect measurement. Set realistic per-axis
  covariances there before trusting the heading estimate.
- **No RTK corrections by default.** NTRIP is out of scope for
  this package (`docs/um960.md`, "NTRIP corrections — read before
  expecting RTK"): expect single-point fixes (~2.5 m) and EKF
  drift between them. `gps_gate`'s covariance threshold is the
  guardrail; anything that must gate on fix *quality* (e.g. the
  dig detector, baseline §4.3.8) should gate on RTK-fixed status,
  never on the EKF's own covariance.
- **odom can jump.** Fusing absolute GPS into the odom world
  (this package's single-EKF design) means the odom frame
  discontinuously corrects itself at each good fix. The jump-free
  alternative is a second, map-frame EKF consuming the same
  `/odometry/gps` (robot_localization's `dual_ekf_navsat_example`)
  — which is what the future localizer back-end (the
  `fusion_graph` port) replaces.
- **Datum.** Unset means the first fix anchors the world — fine for
  a first bring-up, wrong for repeatable field boundaries.
- **`map → odom`.** Not published by this package. Landing the
  localizer back-end (the `fusion_graph` port) is what completes
  the frame tree; until then Nav2 runs on `odom`-framed output.
- **WIT yaw sign convention** is still a TODO in `mower_mcu_driver`
  (`_reckon`); validate on hardware before trusting heading
  anywhere downstream.

## Testing

The gate decision is a pure function and unit-tested:

```bash
cd ros2_stack/src/mower_localization
python3 -m pytest test -q     # 25 passed
```

`test/conftest.py` installs a small `rclpy`/`sensor_msgs` shim
when ROS is not importable, mirroring `um960_gps_driver`, so the
same tests run in a plain Python environment and inside the Humble
container.
