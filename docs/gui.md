# Web GUI (MowgliNext GUI on Tron)

The operator web UI is the MowgliNext GUI (Go/gin backend + React/Vite SPA),
vendored unmodified-except-for-a-few-flags at `third_party/mowglinext/gui/`
(upstream commit in `third_party/mowglinext/UPSTREAM_COMMIT`). It runs in its
own container, `mower_gui`, next to `mower_humble`.

| Piece | Path |
|-------|------|
| Image build | `gui/Dockerfile` (context `third_party/mowglinext/gui`), `gui/build.sh` |
| Compose overlay | `docker/docker-compose.gui.yml` |
| GUI settings file | `config/gui/mowgli_robot.yaml` |
| Profile gating | `third_party/mowglinext/gui/web/src/constants/profileGates.ts` (+ `robotProfiles.ts`) |
| ROS-side adapter | `src/mower_gui_bridge/` — see `src/mower_gui_bridge/README.md` |

## Build and run

```bash
# from ros2_stack/
./gui/build.sh                 # mower-gui:latest for the host arch (dev box)
ARCH=arm64 ./gui/build.sh      # mower-gui:arm64 for the mower
docker save mower-gui:arm64 | gzip > /tmp/mower-gui-arm64.tar.gz   # copy + `docker load` on the mower

docker compose -f docker/docker-compose.yml -f docker/docker-compose.gui.yml up -d
# GUI only:
docker compose -f docker/docker-compose.yml -f docker/docker-compose.gui.yml up -d mower_gui
```

Everything is compiled on the build host: every `RUN` (Go cross-compile with
`CGO_ENABLED=0`, `yarn install` + `yarn build` (= `tsc && vite build`), and the
ca-certificates/tzdata install in a build-platform `ubuntu:22.04` stage) runs
natively on x86. The only target-arch stage is a `debian:bookworm-slim` base
that just receives `COPY`s, so an arm64 build needs neither buildx nor QEMU and
nothing heavy ever runs on the 4 GB mower. `build.sh` uses BuildKit when the
buildx plugin is installed and the legacy builder otherwise (this dev box has
no buildx); for the legacy builder it pulls the runtime base per-arch into
`mower-gui-base:<arch>` and checks the resulting image's arch.

The runtime image is the base + CA bundle + zoneinfo + the static
binary (`/app/mowglinext`), the SPA (`/app/web`) and `asserts/` (`/app/asserts`;
the settings schema is opened relative to the CWD, which is `/app`). No
openocd, platformio, python or docker CLI. The compose service gets no
docker.sock, no `/dev`, no `privileged`, no `pid: host`.

Environment used by the compose service (all read through the DB-provider env
fallbacks in `pkg/providers/db.go`, so a value saved in the bitcask DB wins):

| Env | Value | Meaning |
|-----|-------|---------|
| `FOXGLOVE_URL` | `ws://localhost:8765` | foxglove_bridge endpoint |
| `MOWER_YAML_CONFIG_FILE` | `/mowgli_config/mowgli_robot.yaml` | `config/gui/mowgli_robot.yaml` |
| `DB_PATH` | `/db` | bitcask DB (`/userdata/ros2/gui_db` on the mower) |
| `ONBOARDING_COMPLETED` | `true` | Tron addition: `onboarding.completed` fallback, skips the wizard |
| `MQTT_ENABLED` / `HOMEKIT_ENABLED` | `false` | integrations off |
| `API_ADDR` / `WEB_DIR` | `:4006` / `/app/web` | image defaults |

## Ports

| Port | Who | What |
|------|-----|------|
| 4006/tcp | `mower_gui` | HTTP + WebSocket (SPA, `/api/...`, `/api/mowglinext/multiplex`). **No auth** — LAN only. |
| 8765/tcp | `mower_humble` (foxglove_bridge) | all topic subscriptions and service calls from the GUI backend |
| 8766/tcp | `mower_humble` (cmd_vel relay) | teleop/joystick `TwistStamped` JSON relay; falls back to publishing `/cmd_vel_teleop` via foxglove |

Everything is `network_mode: host`, so the GUI reaches both on `localhost`.

## What the GUI needs from ROS

The GUI never talks DDS; the backend subscribes through foxglove_bridge using
fixed topic names (`topicMap` in `third_party/mowglinext/gui/pkg/providers/ros.go`)
with `mowgli_interfaces` message types, and calls fixed service names
(`pkg/api/mowglinext.go`, e.g. `/behavior_tree_node/high_level_control`,
`/hardware_bridge/emergency_stop`, `/map_server_node/*`). Our stack provides
that surface via `src/mower_gui_bridge` (+ `src/mowgli_interfaces`) — the topic
/ service mapping and what is stubbed is documented in
`src/mower_gui_bridge/README.md`.

Minimum for a useful dashboard: `/hardware_bridge/status`,
`/hardware_bridge/power`, `/hardware_bridge/emergency`,
`/behavior_tree_node/high_level_status`, `/gps/fix`, `/odometry/filtered_map`.
Missing topics are not errors — the matching widget just stays empty.

The backend survives foxglove being down: it logs and retries the connection
(verified by the smoke test with nothing on 8765).

Known gaps / notes:

* **Foxglove subprotocol.** The GUI's foxglove client offers only
  `foxglove.sdk.v1` (`pkg/foxglove/client.go`). Our foxglove_bridge is 3.5.0,
  which speaks it, so this is fine — but an older bridge that only offers
  `foxglove.websocket.v1` will refuse the handshake and the GUI shows no data.
* **Joystick.** The map page only shows the joystick and streams
  `cmd_vel_teleop` while `high_level_status.state_name` is `RECORDING` or
  `MANUAL_MOWING` (or the GUI's own latched manual mode). The bridge must
  publish those state names for teleop to work.
* `foxglove.CallService` fails with "service not advertised" until the bridge
  has advertised services; channels are wiped on every reconnect.
* Backend routes for the hidden tools (rosbag, drive tuning, GNSS sidecar,
  updater, remote access, firmware flashing, container restarts) still exist
  and return errors when called — there is no docker socket or openocd.

## Profile gating (Tron trim)

What the GUI shows is decided at runtime by the active robot profile
(`web/src/constants/robotProfiles.ts`), not by a build flag. The profile is
`mower_model` from `mowgli_robot.yaml`, else the `ROBOT_PROFILE` env fallback
(`pkg/providers/db.go`; `docker/docker-compose.gui.yml` sets
`ROBOT_PROFILE=AirseekersTron`), else the schema default. `GET /api/robot/profile`
reports it. A stock MowgliNext robot has every feature, so it sees exactly the
upstream UI; Tron switches off the features it lacks and the matching UI
disappears. Nothing was deleted, only not rendered.

`web/src/constants/profileGates.ts` maps each gate id to the features it needs
(`useProfileGates()` / `useGate()` in the components, `navItems.ts` for the nav).
Gates hidden on Tron:

| Id | What | Needs |
|----|------|-------|
| `settings:updates` | Settings → Updates (host updater) | `docker_host` |
| `settings:remote_access` | Settings → Remote access (Tailscale sidecar) | `docker_host` |
| `settings:drive_motor` | Settings → Drive motor (FF / PID auto-tuning) | `drive_tuning` |
| `settings:leds` | Settings → LEDs | `status_leds` |
| `feature:firmware_flash` | dashboard "flash firmware" CTA | `stm32_firmware` |
| `feature:firmware_debug` | Diagnostics firmware-debug card | `stm32_firmware` |
| `feature:gnss_configurator` | GNSS receiver plan/apply/factory-reset card | `gnss_sidecar` |
| `feature:host_updater` | running-version footer / updates links | `docker_host` |
| `feature:rosbag`, `feature:containers`, `feature:restart_ros2` | Diagnostics rosbag, container table, Settings "Restart ROS2" | `docker_host` |
| `feature:lidar` | LiDAR settings, scan / LiDAR-map layers | `lidar` |
| `feature:fusion_graph`, `feature:imu_yaw_calibration`, `feature:dock_calibration` | their diagnostics / buttons / wizard | the same-named feature |

`/perception` is the reverse: additive, shown only with `cameras`. The onboarding
wizard is no longer gated: it is profile-aware and skips the firmware / GNSS
receiver steps on a robot without them. Hidden settings sections keep their
keys "claimed", so they do not leak into Settings → Advanced. Strings that name
the robot use `useRobotName()`; STM32-specific wording is chosen with
`useFeature('stm32_firmware')`.

The only build-time switch left is `VITE_SKIP_ONBOARDING=1` (default in
`gui/Dockerfile`), which disables the first-run redirect to the wizard.
Independently, the backend reports onboarding complete when
`ONBOARDING_COMPLETED=true` (added to `EnvFallbacks` in `pkg/providers/db.go`;
read by `GET /api/settings/status`).

Other upstream Tron edits: `third_party/mowglinext/gui/.dockerignore` excludes
the 40 MB `openmower-gui` artifact.

## Pinning the map origin

The map origin (GPS datum) is one value shared by `navsat_transform_node` and
the GUI. Set it with the launch args, using the dock position:

    ros2 launch mower_bringup mower.launch.py datum_lat:=-36.8485 datum_lon:=174.7633

`datum_yaw` (rad, ENU, default 0.0) is optional. The same lat/lon go in
`datum_lat`/`datum_lon` in `config/gui/mowgli_robot.yaml`; set both from the
same source (the `docker/docker-compose.yml` command or a `.env`). With the
args left at 0.0/0.0 the launch logs a WARN and the first GPS fix becomes the
origin, which moves every boot and misaligns the GUI map. With them set,
`navsat_transform_node` gets `wait_for_datum: true` and `datum: [lat, lon, yaw]`.

From the GUI: "set datum from GPS" (Settings > Positioning) calls
`/navsat_to_absolute_pose/set_datum` (`mower_gui_bridge`). With a valid fix it
replies `"<lat>,<lon>"`, writes `datum_lat`/`datum_lon` into
`config/gui/mowgli_robot.yaml` and `/userdata/ros2/datum.env`, and calls
robot_localization's `/datum`. When `datum_lat`/`datum_lon` are not passed,
`mower.launch.py` reads them from `datum_env_file` (default
`/userdata/ros2/datum.env`), so the datum survives restarts. The reply warns when the
datum moves by more than 1 m, because stored areas and the dock pose are in map-frame
metres. See `src/mower_gui_bridge/README.md#set_datum`.

## Datum pinning

The GUI converts between GPS and map coordinates with `datum_lat` /
`datum_lon` from `config/gui/mowgli_robot.yaml` and draws the dock from
`dock_pose_x/y/yaw`. Set the datum to the dock position, and keep it **equal
to the `datum_lat`/`datum_lon` launch args** (the Tron navsat transform), otherwise the GUI map and the robot's `map` frame disagree.

The file is sparse: only keys present override the schema defaults in
`asserts/mower_config.schema.json`; no other key is required for the GUI to
start (a missing file is tolerated too). The Settings page writes back only
the keys you change, preserving the file's owner/mode. Avoid editing the datum
from the GUI (Settings or the "set datum" button, which calls
`/navsat_to_absolute_pose/set_datum`) unless the navsat config is changed to
match — edit both files together and restart the stack.

## GUI changes for upstreaming

The GUI work is being prepared as upstream PRs to MowgliNext; the plan, the
task table (T01-T43) and the PR grouping are in `docs/gui_tasklist.md`. Done in
the vendored tree so far, by PR group: PR-1 generic bugs (route error element /
404, battery percent, scheduler and session fixes), PR-2 profile foundation
(`robotProfiles.ts`, `profileGates.ts`, `useRobotName`, vendor-neutral strings),
PR-3 Tron profile and hardware gating, PR-4 GNSS corrections presentation, PR-5
camera perception (`/perception`), PR-6 settings-to-ROS parameter bindings, PR-7
docking and datum (vision dock, dock-point rejection surfaced by the API), PR-8
ops (rosout logs page, diagnostics without Docker), plus the profile-aware
onboarding wizard (T42). Each group still has to be extracted onto its own
branch off the upstream commit.

## Licence

The GUI is GPLv3 (`third_party/mowglinext/LICENSE`). The image and our
modifications (the profile gating, the flag checks, the `db.go` env fallback,
`gui/Dockerfile`) are a derivative work: if the image or the mower firmware
containing it is **distributed** to anyone, the complete corresponding source
(including these modifications and build scripts) must be offered under GPLv3.
Running it privately carries no obligation. Keep the upstream copyright and
licence notices intact.
