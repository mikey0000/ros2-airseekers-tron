# MowgliNext GUI on Airseekers Tron: audit and task list

Scope: the vendored GUI at `third_party/mowglinext/gui/` (upstream `faf658bc57b9`) running at
`http://192.168.1.105:4006` against our stack (`mower_gui_bridge`, `mower_mission`, `mower_map`, ...).
Paths below are relative to `third_party/mowglinext/gui/` unless they start with `src/`, `config/` or `docker/`.
Evidence was gathered on 2026-10-06 with read-only `curl` and websocket reads, plus a live Chrome walkthrough by the coordinator. No commands were sent to the mower.

## A. Upstreaming strategy

**1. Reuse the upstream model concept.** MowgliNext already has a robot-model key, `mower_model`
(schema `hardware_settings.mower_model`, default `YardForce500`). It has presets in
`web/src/constants/mowerModels.ts:34-139` and a picker in `components/settings/HardwareSection.tsx:44-104`
and `OnboardingPage.tsx:147-178`. The plan is to turn this "model" into a **robot profile**, not to add a
parallel key. The profile is a typed descriptor:

```ts
type RobotProfile = {
  value: string; label: string; image?: string;          // "AirseekersTron"
  defaults: Record<string, number>;                       // existing preset values
  features: Partial<Record<FeatureId, boolean>>;          // stm32_firmware, gnss_sidecar, docker_host,
                                                          // drive_tuning, lidar, cameras, dock_calibration,
                                                          // fusion_graph, imu_yaw_calibration, status_leds, lora_corrections
  perception: 'lidar' | 'camera' | 'none';
  docking: 'charger_contacts' | 'vision_marker';
  battery: { fullV: number; emptyV: number; criticalV: number; preferReportedPercent: boolean };
  cameras?: { id: string; label: string; topic: string; annotatedTopic?: string }[];
  hiddenSettingKeys?: string[];                           // e.g. wheel_pid_*, ticks_per_meter
  paramCatalogOverlay?: Record<string, ParamMeta>;
  teleop?: { maxLinear: number; maxAngular: number };
};
```

Existing YardForce profiles default to `features` = all true, so upstream behaviour stays the same.
`AirseekersTron` is one more entry.

**2. One gate function.** `tronFeatures.ts` becomes a compatibility shim. `isTronHidden(id)` maps to
`!useFeature(id)`, driven by the active profile. The build-time `VITE_TRON_FEATURES` flag goes away. A backend env
fallback `ROBOT_PROFILE` (beside `ONBOARDING_COMPLETED` in `pkg/providers/db.go`) gives the
profile before settings load. Every call site keeps the same one-line shape, so the existing Tron diffs
become the upstream feature checks.

**3. Small diffs, rebase-friendly.** Do not rename upstream files. Add new code in new files
(`constants/robotProfiles.ts`, `hooks/useRobotProfile.ts`, `pages/PerceptionPage.tsx`,
`pkg/api/cameras.go`). Fix generic bugs first, as standalone PRs, because upstream gains from them regardless.

**4. PR grouping (in merge order):**
- **PR-1 Generic bugs:** T09-T14, T38, T39. Maintainers can accept these with no profile concept.
- **PR-2 Profile foundation:** T01-T03, T08.
- **PR-3 Tron profile, imagery and hardware gating:** T04-T07, T25, T30.
- **PR-4 GNSS corrections presentation:** T15, T17.
- **PR-5 Camera perception:** T19-T21, T23.
- **PR-6 Settings-to-ROS parameter bindings:** T26, T29, T33.
- **PR-7 Docking and datum:** T32, T34, T35.
- **PR-8 Ops:** T36, T37, T43.

Stack-side tasks (bridge, mission, launch) stay in this repo and can ship ahead of their GUI PRs.

## B. Task table

Size: S ≤ ½ day, M 1-3 days, L > 3 days. "Kind" is GUI or Stack.

| id | title | area | files to touch | size | tier | deps | acceptance criteria | kind |
|---|---|---|---|---|---|---|---|---|
| T01 | Robot profile descriptor and `useRobotProfile()` | frontend/schema | new `web/src/constants/robotProfiles.ts` (wraps `mowerModels.ts`), new `hooks/useRobotProfile.ts`; schema `mower_model` enum description | L | opus | – | Profile resolved from `settings.mower_model`, else the `ROBOT_PROFILE` env fallback. Existing presets unchanged (`model_preset_parity_test.go` still passes). Unit tests cover feature lookup. | GUI |
| T02 | `ROBOT_PROFILE` env fallback and `GET /api/robot/profile` | backend | `pkg/providers/db.go` (EnvFallbacks), `pkg/api/setup.go` | S | sonnet | T01 | Endpoint returns the profile id before settings exist. Compose sets `ROBOT_PROFILE=AirseekersTron`. | GUI |
| T03 | Replace `TRON_HIDDEN_PAGES` with profile features | frontend | `web/src/tronFeatures.ts`, `main.tsx:74`, `AppShell.tsx:58,397,570`, `SettingsPage.tsx`, `DiagnosticsPage.tsx:198,750` | M | opus | T01 | Every current `isTronHidden` id maps to a `FeatureId`. With the YardForce profile the UI matches stock. With the Tron profile it matches today's trim. `VITE_TRON_FEATURES` is removed. | GUI |
| T04 | `AirseekersTron` preset values | frontend | `constants/mowerModels.ts` / `robotProfiles.ts`, i18n `mowerModels.AirseekersTron.*` | S | sonnet | T01 | Values match `config/urdf/mower.urdf.xacro` (chassis 0.70×0.50 @x=0.12 l.92-93, wheel r 0.10 l.96, track 2×0.24=0.48 l.99, cutter r 0.10 l.194) and the Nav2 footprint 0.75×0.54 (`src/mower_navigation/config/nav2_params.yaml:234`). **The 0.53 m track and 0.30 m cut width in the brief disagree with the URDF (0.48 / 0.20). Measure, then fix both.** Battery thresholds are left TBD until the BMS curve is known (21.6 V read at 52 %). | GUI |
| T05 | Robot imagery per profile | frontend | `HardwareSection.tsx:80-104` (add `<img>`), new `web/public/robots/*.svg`, `RobotAnatomy.tsx` | M | sonnet | T01 | Model cards show an image. Tron gets a placeholder top-down SVG: rounded 0.70×0.55 body, two rear drive wheels, two front omni wheels, one blade disc, three camera glyphs. The owner can drop `airseekers_tron.jpg` in `web/public/robots/`. Today there are no raster images at all; `web/public` only has logos. | GUI |
| T06 | URDF geometry parse follows the joint child link | frontend | `hooks/useRobotDescription.ts` (parse at ~l.115-140) | S | sonnet | – | The parser resolves `<joint name="left_wheel_joint"><child link=…>` instead of the hard-coded `left_wheel_link`. Our URDF names the link `left_wheel`, so the map silhouette today falls back to the default r=0.04475 and blade 0.09 (`DEFAULTS` l.19-32). | GUI |
| T07 | Stack: give `blade_link` a visual cylinder | stack | `config/urdf/mower.urdf.xacro:205-211` | S | sonnet | – | `blade_link` carries a cylinder of the real disc radius, so the GUI silhouette draws the true blade. | Stack |
| T08 | Configurable robot name and vendor-neutral strings | frontend | `i18n/locales/en.json` (39 "Mowgli", 7 "STM32", 10 "YardForce"; e.g. l.753-778 "Mowgli is idle", "Mowgli · {{area}}"), `fr.json` | M | sonnet | T01 | The strings use `{{robotName}}`, taken from the profile label or a new optional `robot_name` key. The firmware-debug text says "motor controller" unless the profile has `stm32_firmware`. | GUI |
| T09 | Bug: "AREA 0" shown when no area is active | frontend | `pages/MowgliNextPage.tsx:97-98,416` | S | sonnet | – | `current_area < 0` renders the "no zone" text. Live HL status has `current_area:-1`, but the UI shows "MOWGLI · AREA 0". | GUI |
| T10 | Bug: firmware version shows "vv0.6.36" | frontend+bridge | `MowgliNextPage.tsx:104`, `en.json:809` (`"v{{version}}"`); `src/mower_gui_bridge/.../gui_bridge_node.py` (firmware_version) | S | sonnet | – | The GUI strips a leading `v` before formatting. The bridge also publishes `0.6.36`. | both |
| T11 | Bug: router keeps the Parameters page body; no 404 route | frontend | `main.tsx:27-76,132` (Suspense above `RouterProvider`), `AppShell.tsx:280-305` (`AnimatePresence mode="wait"` + `useOutlet`) | M | opus | – | Repro: open `#/parameters`, then any route; the body stays on Parameters until reload. Likely cause is the lazy page suspending inside the keyed `motion.div` under `AnimatePresence wait`, with the only Suspense at the root. Try moving `<Suspense>` inside `AnimatedOutlet`. Confirm with `VITE_TRON_FEATURES=0`; our `.filter()` does not touch keys, so this is probably upstream. Also add `errorElement` plus a `*` route (today `#/stats` shows "Unexpected Application Error! 404"). | GUI |
| T12 | Bug: content is offset one viewport down on first load (1532×784) | frontend | `AppShell.tsx:140,177,209,255,290-301`, `IOSInstallBanner.tsx` | M | opus | – | Home, Map and Settings render at the top on first load. Find the stray spacer: the `minHeight:'100%'` motion.div inside the `overflow:auto` main, or a hidden banner. | GUI |
| T13 | Battery percent: unknown is not 0 %, and profile voltage thresholds | frontend | `utils/battery.ts:36-49`, `BatterySection.tsx:133-134`, `MowerStatus.tsx:63`, `MowgliNextPage.tsx:60` | S | sonnet | T01 | When HL status has not arrived, show "--" instead of the voltage fallback (today 21.6 V against 28/24 V gives 0 %; once HL arrives it shows 52 %). Voltage thresholds come from the profile. | GUI |
| T14 | State vocabulary: add Tron mission states | frontend+backend | `components/utils.tsx:19-55` (STATE_LABEL_KEYS, the map actually used), `components/dashboard/constants.ts:18-62` (MOWER_STATES, exported only via `dashboard/index.ts:4`, unused), `BTStateGraph.tsx:81-94`, i18n | S | sonnet | – | `WAITING_FOR_RTK` and `BOUNDARY_PAUSED` (from `src/mower_mission/mower_mission/mission_fsm.py:87,92`) get labels and tones. Either delete the dead MOWER_STATES or make it the single source. Our other names (`PLANNING`, `AREA_UNREACHABLE`, …) already have labels. | GUI |
| T15 | GNSS corrections: render transport, flow, source and age (see C) | frontend | `utils/gpsStatus.ts` (new `deriveCorrectionSummary`), `GnssLiveStatusSummaryCard.tsx:36,66-69`, `GnssLiveDiagnosticsCard.tsx:163-165,193-196,318-330`, `readinessChecks.ts:97-117`, `gnssPresentation.ts:29`, i18n | M | opus | – | When `CAP_CORRECTION_FLOW`/`CAP_CORRECTION_TRANSPORT`/`CAP_CORRECTIONS_ACTIVE` are set, the cards show "Corrections: LoRa · waiting for data" (amber), plus transport (or "radio link not observable" for LoRa) and age. `correction_stream_status` is used only when `CAP_CORRECTION_STREAM` is set. Test with the decoded live sample from D. | GUI |
| T16 | Bridge: also mirror `corr_flow` into `correction_stream_status` | bridge | `src/mower_gui_bridge/mower_gui_bridge/state_machine.py:475-520` + tests | S | sonnet | – | Stopgap until T15 lands. Map `idle→IDLE`, `waiting/held→WAITING`, `active→ACTIVE`, `stale→UNAVAILABLE`, `invalid→ERROR`, and set `CAP_CORRECTION_STREAM` in both flag words. The stock GUI then shows "Waiting" instead of "Unknown". | Stack |
| T17 | LoRa base pairing and status in Settings → NTRIP/Corrections | frontend+backend | `NtripSection.tsx`, new `pkg/api/corrections.go` → `/mower_gps_node/set_lora` (`docs/um960.md:110-116`) | M | opus | T01, T15 | With the `lora_corrections` feature, a "Correction source: LoRa base" card shows flow and age and a Pair button. NTRIP fields are hidden unless the source is NTRIP. | GUI |
| T18 | NTRIP settings reach the UM960 driver | stack | `src/um960_gps_driver/um960_gps_driver/um960_node.py:127-134` (own `ntrip_*` params and vendor `ntrip.yaml`), launch | M | opus | T26 | `ntrip_enabled/host/port/mountpoint/user/password` saved in the GUI are applied: live via a ROS param set, and persisted for the next boot. Today they only go to `mowgli_robot.yaml`, which "nothing in the Tron ROS stack reads" (`config/gui/mowgli_robot.yaml:12-13`). | Stack |
| T19 | Backend camera registry and MJPEG proxy | backend/schema | new `pkg/api/cameras.go` (`GET /api/cameras`, `GET /api/cameras/:id/stream` reverse-proxying `http://localhost:8080/stream?topic=…&type=mjpeg`), schema `perception_settings.camera_stream_base_url` | M | opus | T01 | Cameras come from the profile (`/left_oa_camera/image_raw`, `/right_oa_camera/image_raw`, `/rear_camera/image_raw`, annotated `/ai/det/image_annotated`). Everything is served from :4006, so there are no CORS or extra-port issues. If web_video_server is down, the endpoint returns 503 with a hint. | GUI |
| T20 | Perception page (camera profile) | frontend | new `pages/PerceptionPage.tsx`, `main.tsx` route, `AppShell.tsx:49-58` nav | M | sonnet | T19, T21 | A 3-tile grid (an `<img>` per MJPEG stream) with a raw/annotated toggle, detection count and classes, and an "obstacle close" badge. A FPS/quality selector keeps the 4 GB board safe. Shown only with `features.cameras`. The GUI has no image support today (no `<img>`, MJPEG or Image topic anywhere in `web/src`). | GUI |
| T21 | Topic map: vision topics | backend | `pkg/providers/ros.go:28-71` (add `visionObstacleClose` `/vision/obstacle_close` std_msgs/Bool, `detections` `/ai/det/detections` vision_msgs/Detection2DArray), `pkg/msgs/` generated types | M | sonnet | – | The websocket subscribe keys work. The Detection2DArray JSON is reduced to `{count, classes[], max_score}` server-side. | GUI |
| T22 | Stack: run web_video_server with the GUI | stack | camera launch (`docs/cameras_and_video.md`), `docker/docker-compose.gui.yml` | S | sonnet | – | `curl :8080/` answers (today: connection refused). Default quality is ≤ 50 at ≤ 5 fps. | Stack |
| T23 | Hide LiDAR UI per profile | frontend | `SensorsSection.tsx:17-35`, `useSettingsManager.ts` (sensors/localization keys: lidar_*, use_lidar_map_anchor), `useMapStreams.ts:191-240` (scan/lidarMap layers), `DiagnosticsPage.tsx:392-421,869` (lidar anchor, fusion_graph commands), `RobotAnatomy.tsx:22,52-55`, `ObstaclesSection.tsx` | M | sonnet | T03 | With `perception:'camera'`, no LiDAR fields or layers appear. Anatomy shows "Cameras (3)", fed by the image topics' freshness. | GUI |
| T24 | Stack: check the EKF warning | stack | `src/mower_localization/config/ekf.yaml` | S | sonnet | – | Find why `/diagnostics` shows ekf_node "Potentially erroneous data or settings detected…" (two_d_mode l.31, sensor config). Either fix it or document it. | Stack |
| T25 | Hardware section: hide firmware-only keys per profile | frontend | `HardwareSection.tsx` (Wheels & blade card l.110+), `useSettingsManager.ts:73-82` | S | sonnet | T01 | For Tron, `ticks_per_meter`, `deadband_pwm`, `wheel_pid_*`, `caster_*` and `chassis_mass_kg` (STM32/odometry-only on Mowgli; our MCU computes `/odom` itself) are hidden. Geometry keys become read-only from the URDF unless bound (T26). | GUI |
| T26 | Settings-to-ROS parameter bindings per profile | backend | new `pkg/api/param_bindings.go` (on `PUT /settings`, call `rosProvider.SetParameters`; `pkg/foxglove/parameters.go:62` already exists), profile binding table | L | opus | T01 | For Tron: `tool_width`→`/coverage_server.operation_width`, `mowing_speed`→`/controller_server.FollowCoveragePath.desired_linear_vel`, `transit_speed`→`…FollowPath.desired_linear_vel`, `undock_distance`/`undock_speed`→`/behavior_tree_node.undock_distance_m`/`undock_speed_mps`, `battery_low_percent`/`battery_full_percent`, `rain_mode`/`rain_delay_minutes`, `mow_angle_deg`→ mission. The UI flags unbound keys as "not used by this robot". The save result reports per-key apply status. | GUI |
| T27 | Stack: load bound keys from `mowgli_robot.yaml` at launch | stack | `src/mower_bringup/launch/*.py`, `src/mower_coverage_bridge/launch/coverage_bridge.launch.py`, mission launch | M | opus | T26 | Values saved in the GUI survive a reboot. A key missing from the yaml leaves the package default. | Stack |
| T28 | Stack: coverage swath width | stack | `src/mower_coverage/src/mower_coverage_node.cpp:28` (default 0.18 = Mowgli value), `coverage_server_node.py:55` (`operation_width` 0.0 → default) | S | sonnet | T04 | `operation_width` = measured cut width minus overlap, set in the launch. Planned swath spacing matches the blade. | Stack |
| T29 | Parameters page catalog overlay per profile | frontend | `components/settings/paramCatalog.ts:36-130` (keyed by short name), `ParametersPage.tsx` | S | sonnet | T01 | The Tron basic tier lists mission, nav and docking params (`undock_distance_m`, `battery_low_percent`, `rtk_timeout_s`, `desired_linear_vel`, `transit_gap_m`, …). Today only `mow_angle_deg` matches, so the page shows one parameter. | GUI |
| T30 | Teleop speed limits per profile | frontend | `pages/map/hooks/useManualMode.ts:15-16` (0.25 m/s, 0.6 rad/s hard-coded) | S | sonnet | T01 | Limits come from `profile.teleop`, capped by the relay's `max_linear`/`max_angular` read via `/api/params` (live: 0.5 / 1.0). | GUI |
| T31 | Coverage resume dialog verification | test | `components/MowerActions.tsx:41`, `hooks/useCoverageResumeAvailable.ts`; `src/mower_mission/mower_mission/mission_node.py:160-163,412` | S | sonnet | – | With a resume file present, Start shows the Resume / Start fresh dialog. Clear calls `clear_coverage_resume`. Done in sim, without driving the mower. | both |
| T32 | Gate dock calibration; add a vision-dock card | frontend | `components/settings/DockingSection.tsx:35` (`<DockCalibrationCard/>`), `DockCalibrationCard.tsx`, `hooks/useDockCalibration.ts` | M | sonnet | T03 | With `docking:'vision_marker'`, the CalibrateDock wizard (`/calibrate_imu_yaw_node/dock_calibration/*`) is hidden. A card shows `dock_pose_x/y/yaw`, a "Place dock on map" link (existing map dock-placement → `POST /mowglinext/map/docking` → `/map_server_node/set_docking_point`, `useMapFiles.ts:175-202`), and "Set dock = current robot pose" (`use_gps_position:true`). | GUI |
| T33 | Docking section keys for vision docking | frontend | `DockingSection.tsx:50-63`, `useSettingsManager.ts` | S | sonnet | T26 | `undock_distance`/`undock_speed` are bound (T26). `dock_approach_*`, `dock_charging_threshold` and `dock_use_charger_detection` are hidden or mapped to `src/mower_docking/config/docking.yaml` (`approach_distance` 0.8, `undock_max_speed` 0.3). | GUI |
| T34 | Stack: implement `set_datum` | bridge | `src/mower_gui_bridge/mower_gui_bridge/gui_bridge_node.py:563-566` | M | opus | – | With a valid fix, reply `success=true, message="lat,lon"` (the contract parsed in `web/src/utils/datumGps.ts:25-35` and `PositioningSection.tsx:70-86`). Optionally call robot_localization `SetDatum` on `/datum` and write `datum_lat/lon` to `mowgli_robot.yaml` and the launch `.env`. The reply warns that stored map-frame areas shift if the datum moves. | Stack |
| T35 | Datum UX outside onboarding | frontend | `MapPage.tsx:724-729` + `en.json:1733-1734` ("Configure it during onboarding…"), `PositioningSection.tsx:70-86` | S | sonnet | – | The Map no-datum panel links to Settings → Positioning (manual lat/lon already exists there, l.118-140). "Set from GPS" shows an error when the reply is unparseable (today it is silently ignored when `parts.length!==2`). The live snapshot shows `datum_lat=0, datum_lon=0`. | GUI |
| T36 | Logs page without docker: a `/rosout` source | backend+frontend | `pkg/api/containers.go:128`, `pkg/providers/ros.go` topicMap (`rosout` rcl_interfaces/Log), `pages/LogsPage.tsx` | M | opus | – | With `docker_host` off, Logs streams `/rosout` filtered by node and level. Today `/api/containers/` returns "Cannot connect to the Docker daemon". | GUI |
| T37 | Diagnostics and Settings: gate docker, STM32 and fusion-graph controls | frontend | `DiagnosticsPage.tsx` (containers card, firmware-debug card, IMU yaw calibration `useImuYawCalibration`, fusion_graph cmds l.869), `SettingsPage.tsx` restart-ROS2 (`utils/containers.ts:38-39`) | M | sonnet | T03 | No "No container data" card and no dead buttons calling `/fusion_graph_node/*`, `/calibrate_imu_yaw_node/*` or `/hardware_bridge/set_firmware_debug` on Tron. | GUI |
| T38 | Scheduler honours the schedule's area | backend | `pkg/providers/scheduler.go:170-182` (always `HighLevelControl{Command:1}`), `SchedulePage.tsx:239` (read-only notice) | S | sonnet | – | `sched.Area >= 0` calls `/behavior_tree_node/start_in_area` (served by `mission_node.py:351`). The UI area picker becomes editable. Generic upstream bug. | GUI |
| T39 | Session tracker state lists | backend | `pkg/providers/session_tracker.go:200-262` | S | sonnet | T14 | `wasMowing` includes `PLANNING`, `WAITING_FOR_RTK` and `BOUNDARY_PAUSED`, so a pause does not split sessions. Statistics endpoints work today and return 0 sessions. | GUI |
| T40 | Bridge README refresh | docs | `src/mower_gui_bridge/README.md` (still says set_datum fails and `start_in_area` is refused) | S | sonnet | T34 | The README matches the code and the `serve_high_level:=false` deployment. | Stack |
| T41 | Status LEDs, rain and MQTT sections per profile | frontend | `LedsSection.tsx`, `RainSection.tsx`, `MqttSection.tsx`, `mqtt_topic_prefix` default `mowgli` | S | sonnet | T03 | LED section hidden (Tron lights are `src/mower_lights`). The MQTT prefix defaults to the profile id. Rain stays (mission has rain guards). | GUI |
| T42 | Profile-aware onboarding instead of hiding the route | frontend | `pages/OnboardingPage.tsx` (firmware step l.1139, model step l.147-178, datum l.586-662), `onboarding/steps.ts` | M | opus | T03, T34 | For Tron the wizard skips the firmware and GNSS-config steps and keeps model, datum, dock and first area. `/onboarding` is re-enabled. | GUI |
| T43 | Robot Anatomy and Diagnostics health copy | frontend | `RobotAnatomy.tsx:45-120`, `en.json` robotAnatomy.* | S | sonnet | T07, T23 | Profile-driven part list. No STM32 wording. | GUI |

## C. GNSS corrections finding

**Live `/gps/status` sample** (decoded from `ws://…:4006/api/mowglinext/subscribe/gnssStatus`):
`backend=um960_gps_driver, receiver=Unicore UM960, fix_type=1 (GPS), rtk_mode=1 (NONE), sats 37/37,
hdop 0.42, h_acc 0.45 m, correction_source="lora", correction_flow_status=2 (WAITING),
correction_transport_status=0, corrections_active=false, differential_corrections=false,
correction_age_s=0, correction_stream_status=0, capability_flags=100670875` → bits
{0,1,3,4,7,8,10,11,12,25,26}, `value_flags=67112347` → bits {0,1,3,4,7,8,10,11,26}.

**What the bridge does** (`src/mower_gui_bridge/mower_gui_bridge/state_machine.py:412-520`). It sets
`CAP_CORRECTIONS_ACTIVE|CAP_CORRECTION_AGE|CAP_CORRECTION_TRANSPORT|CAP_CORRECTION_FLOW`. It sets value bits
only when a value is known: flow is known (WAITING); age is unknown (no `diff_age`/`corr_age`); transport is
unknown for LoRa (the radio link cannot be seen from the host, l.493-505). This matches the contract in
`src/mowgli_interfaces/msg/GnssStatus.msg` exactly: capability=1, value=0 means "supported, currently unknown".
The Go struct field order (`pkg/msgs/mowgli/types_generated.go:75-129`) matches our .msg. `adaptGnssStatus`
(`pkg/providers/transform.go:331-341`) passes our message through unchanged because it has a `header`.

**What the GUI reads.** The GUI reads only `correction_stream_status` / `CAP_CORRECTION_STREAM` (bit 23). This
is a "diagnostics-derived correction-stream summary mirrored from Universal GNSS", which our bridge never fills:
- `GnssLiveStatusSummaryCard.tsx:36,68` and `GnssLiveDiagnosticsCard.tsx:195` always show a "Correction stream"
  tag. Value 0 maps to "Unknown" (`gpsStatus.ts:117-133`).
- The corrections section is shown only if `CAP_CORRECTION_STREAM` or MSM is present
  (`GnssLiveDiagnosticsCard.tsx:163-165,321`), so it is hidden.
- The readiness check is `snap.gnss?.correction_stream_status` (`readinessChecks.ts:97-110`), so it stays "pending".
- Nothing in `web/src` (outside generated types and tests) reads `corrections_active`,
  `correction_flow_status`, `correction_transport_status`, `correction_source` or `correction_age_s`. The
  backend only maps Universal-GNSS flags (`transform.go:487`).

**The real bug** is on the GUI side. Upstream added the source-owned transport/flow/semantic fields to the message
but never presented them. Our bridge is correct, but invisible. One small consistency point to fix in T16: the
bridge could also fill the legacy stream summary, so stock GUIs show something meaningful.

**What the UI should show right now** (LoRa base not paired, `corr_flow=waiting`):
- Fix: "GPS fix (no RTK)", amber. RTK mode: "None".
- Corrections: "LoRa base · waiting for data", amber, with "radio link status not observable" and age "—".
- Readiness "Corrections": pending, with the call to action "pair / power the LoRa base station" (T17), not "fix NTRIP".
- When flow is ACTIVE: green, with age. STALE: red "LoRa data stale (age N s)". INVALID: red.
- NTRIP source: transport CONNECTING / STREAMING / FAILED shown next to flow.

The dashboard's "GPS 25 %" comes from `deriveGpsStatus` (`gpsStatus.ts:56-82`, GPS_FIX → 25), and is correct.
Note that HL `gps_quality_percent=0.4` is a fraction while GnssStatus `quality_percent=40` is a percentage,
following upstream's convention.

## D. Evidence gathered

- **`GET /api/settings/status`** returns `{"onboarding_completed":true}`.
- **`GET /api/diagnostics/snapshot`** returns `containers:[]`, `cross_checks.warnings:["GPS datum not configured (lat=0, lon=0)","Dock heading not configured (yaw=0)"]`, `dock_pose 0/0/0`.
- **`GET /api/containers/`** returns `"Cannot connect to the Docker daemon at unix:///var/run/docker.sock"`. This affects T36 and T37.
- **`GET /api/settings/schema`** has 14 sections. Defaults include `mower_model=YardForce500`, `tool_width 0.18`, `wheel_track 0.325`, `battery_full_voltage 28`, `lidar_*`, `led_*` and `wheel_pid_*` (all YardForce/STM32 values).
- **`GET /api/params`** returns live ROS params via foxglove, e.g. `/cmd_vel_ws_relay.max_linear=0.5`, `max_angular=1.0`.
- **Websocket reads:**
  - `highLevelStatus`: `{state:1, state_name:"IDLE", current_area:-1, battery_percent:52, gps_quality_percent:0.4}`
  - `power`: `{v_battery:21.6}`
  - `status`: `{firmware_version:"v0.6.36", mower_motor_temperature:16, mower_status:255}`
  - `gnssStatus`: see C.
- **`curl http://192.168.1.105:8080/`** fails to connect, so web_video_server is not running (T22).
- **Coordinator's Chrome walkthrough:**
  - Home shows "MOWGLI · AREA 0", "Mowgli is idle", "Firmware OK vv0.6.36", "GPS 25 % GPS fix" and "MOTOR 16 °C ESC 0 °C" (T08-T10).
  - Diagnostics shows "Battery 0 % · 21.6 V" on one load and 52 % on another (T13), an anatomy view with LiDAR (T23/T43), "Containers: No container data" (T37), and the ekf_node warning (T24).
  - The Map and Settings pages show "No GPS datum yet … during onboarding" (T35).
  - The Parameters page lists one parameter (T29).
  - Bugs seen: after visiting `#/parameters` the body sticks on later routes, and `#/stats` gives an unhandled 404 (T11). On first load the content sits one viewport down (T12).
  - The top bar shows only the FR/EN toggle.
