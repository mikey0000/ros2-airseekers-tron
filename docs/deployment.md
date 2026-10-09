# Deployment — Humble container on the 20.04 (Noetic) mower

## Routine deploy: update a mower that is already set up (2026-10-09)

First-time setup (Docker on the device, udev rules, vendor services stopped) is further down
("Mower-side setup"). This section is the everyday loop. Everything runs from the dev box; the
device keeps all data on `/userdata` (its root fs is full).

**Before you start**
- The mower is **idle on the dock** (GUI: "At base"/"Charging"); `up` restarts every node.
- Dev box: `sshpass`, `rsync`, docker. Export the login once:
  `export SSHPASS=<password>`; `MOWER_HOST` (default 192.168.1.105) and `MOWER_USER`
  (default `root`; set `MOWER_USER=airseekers` if root login is disabled).
- Run tests first (`./scripts/dev_build.sh test --packages-select <pkgs>`); `sync` ships the
  working tree **as it is**, including uncommitted edits from any session.

**1. ROS 2 stack** (`scripts/deploy_to_mower.sh`)

```bash
cd ros2_stack
./scripts/deploy_to_mower.sh sync     # rsync -> /userdata/ros2_stack (--delete; no build/ install/ log/ .git)
./scripts/deploy_to_mower.sh image    # ONLY when docker/Dockerfile.humble changed (slow, on the device)
./scripts/deploy_to_mower.sh build    # colcon build in the container, 2 workers (4 GB RAM); log:
                                      #   /userdata/ros2/logs/colcon_build.log
./scripts/deploy_to_mower.sh up       # docker compose up -d (stack + GUI)
```

Python packages are symlink-installed: a Python-only change needs `sync` + a restart
(`docker restart mower_humble`); new packages, entry points, C++, `.srv`/`.msg` and launch
files need `build`. `sync` never overwrites the **device-owned** `config/gui/mowgli_robot.yaml`
(datum, GUI-saved settings); maps, bags and logs live outside the synced tree in `/userdata/ros2/`.

**2. GUI image** (built on the dev box, the device only loads it)

```bash
ARCH=arm64 ./gui/build.sh                      # -> mower-gui:arm64 (no QEMU needed)
docker save mower-gui:arm64 | gzip | sshpass -e ssh $MOWER_USER@$MOWER_HOST 'gunzip | docker load'
sshpass -e ssh $MOWER_USER@$MOWER_HOST 'cd /userdata/ros2_stack && docker compose \
  -f docker/docker-compose.yml -f docker/docker-compose.gui.yml up -d --force-recreate mower_gui'
```
Hard-refresh the browser (Ctrl+Shift+R) afterwards. `up -d --force-recreate` without a service
name also recreates `mower_humble`, which is what applies compose changes (stop signal, mounts).

**3. Check it came up**
- `./scripts/deploy_to_mower.sh logs` (Ctrl-C to leave): no node in a respawn loop, Nav2
  "Managed nodes are active", `map_server_node: route graph: ...`.
- GUI http://<mower>:4006: pose, battery, GNSS, mission state; Diagnostics page all green.
- Inside the container the `ros2` CLI daemon is unreliable: `ros2 daemon stop` first.
- `./scripts/crash_report.sh` summarises crashes since the deploy.

**4. Switching new behaviour off** (no code change). Launch arguments of `mower.launch.py` go
on the compose `command:` line in `docker/docker-compose.yml`, e.g.
`command: ["/work/scripts/stack_entry.sh", "localization_mode:=single", "transit_controller:=rpp"]`,
then `sync` + `up`. Mission/map options are yaml parameters (`sync` + restart):

| To go back to | Setting |
|---|---|
| single EKF + static map->odom | launch `localization_mode:=single` |
| RPP instead of MPPI for transits | launch `transit_controller:=rpp` |
| bumper-only obstacles (no stereo) | launch `stereo_costmap:=false` |
| no rolling bag recorder / alerts | launch `bag_recorder:=false` / `alerts:=false` |
| Nav2 transits instead of drawn-path routes | `src/mower_mission/config/mission.yaml` `route_transits: false` |
| no transit variation | mission.yaml `route_variation: false` (or area setting `transit_variation: none`) |
| no automatic dock no-go in areas | `src/mower_map/config/map_server.yaml` `dock_nogo_in_areas: false` |

**Other one-off installs** (root on the device, documented separately): power-button
shutdown `scripts/install_power_button.sh` (docs/buttons.md), stereo watchdog
`scripts/install_stereo_watchdog.sh`, udev rules `scripts/install_udev.sh`.

**Pull data back:** `./scripts/deploy_to_mower.sh bags [incidents|ring|all]` (rolling bag ring /
incident pins, docs/crash_recovery.md).

## Kernel / deployment decision (settled — Humble container now, native LTS later)

Two end-states were considered. **Plan: develop in the Humble container on the stock
5.10.209 kernel now; do the proper OS changeover + LTS update on the machine later.**
The container is a deliberate bridge, not the permanent deployment — when the machine is
upgraded to a native 22.04 LTS (Jammy, Humble's target) the same ROS 2 packages run bare-
metal and the `metoak_reimpl` drivers become the camera path.

| | **A: stock 5.10.209 + container** (recommended) | **B: Joshua-Riek 6.1 reflash** |
|---|---|---|---|
| Camera modules | mower already loads matching `inv_icm42600` / `mo_trig_flash` / `mo_tmp112` (5.10.209 builds, `09_platform/lsmod.txt`); `mo_init.sh` brings up `mo_xc9080`/`mo_simor`/`video_rkisp` on demand | need to build + load the clean-room `metoak_reimpl/kernel/*` drivers |
| OTA | **preserved** | **lost** (OS replaced) |
| Vendor Noetic stack | intact, can be stopped bridged or replaced incrementally | gone |
| VIO path | V4L2 read of the 1280x480 ISP stream (already implemented in `stereo_vio_bridge`) | same V4L2 path via re-impl subdevs |
| Risk | low; touches nothing on the mower | high; full OS swap on a safety-critical unit |
| Effort | docker install + `mo_init.sh` already present | full kernel/DT/OS bring-up |

Rationale:

- The stereo front **already works on the mower today** — the vendor `.ko` are
  loaded from the stock 5.10.209 rootfs (the `metoak/ko/*.ko` we pulled are
  **5.10.66 dev-board copies**, confirmed via `readelf -p .modinfo`, and will not
  load on 5.10.209; the mower's own matching modules are what actually run).
- OTA is the one thing a vendor update path depends on; dropping it is
  unacceptable without an explicit product decision.
- VSLAM (OpenVINS) only needs the raw **1280x480 stereo stream + IMU**, both of
  which the stock kernel already exposes (`/dev/video22` + IIO). The SDK's
  extra on-chip **Simor depth** is not used by VIO, so nothing is lost.
- The re-impl 6.1 drivers (`metoak_reimpl/`) are the **native-LTS target**: on the stock
  5.10 path the mower's own modules serve the V4L2 stream, but once the machine moves to
  a 22.04/6.x (Joshua-Riek BSP) image the clean-room drivers replace the vendor `.ko`.

### Path to native (LTS changeover)

| Stage | Kernel | Camera | ROS 2 |
|---|---|---|---|
| **now (dev)** | stock 5.10.209 + Humble **container** | vendor `.ko` (5.10.209) via V4L2 | Humble in Docker |
| **later (changeover)** | 22.04 LTS / 6.1 BSP (Joshua-Riek) | clean-room `metoak_reimpl` V4L2 subdevs | Humble native |

Between the two stages nothing consumer-facing changes: `stereo_vio_bridge` publishes the
same `/vio/*` topics + camera_info in both, and OpenVINS/det/seg consume them identically.

## The d-gap (why a container)

| Fact | Source |
|---|---|
| Device OS is Ubuntu **20.04.6 LTS (focal)** | `ros2_port_handoff/09_platform/os-release.txt` |
| Kernel **5.10.209**, arch **aarch64** (Rockchip BSP) | `ros2_port_handoff/09_platform/uname.txt` |
| Stock stack is ROS 1 **Noetic** (~297 `ros-noetic-*` debs, desktop-full) | `ros2_port_handoff/09_platform/ros_noetic_debs.txt`, `dpkg-l.txt` |
| **Humble needs 22.04 (jammy)**; Foxy/Galactic (the 20.04 ROS 2 releases) are EOL; BSP reflash + OTA would undo an OS upgrade | `mower_docs/02-ros2-migration.md` |

So Humble **cannot be apt-installed on the device**. The container
(`docker/Dockerfile.humble`, base `ros:humble-ros-base-jammy`, `linux/arm64`)
runs a Jammy userland on the stock 5.10 kernel while the host stays on 20.04.

## Coexistence with stock Noetic services

Pick one; bridging only the topics you need:

1. **Exclusive (recommended for bringup):** stop the stock services first so the
   container owns the serial ports (`serial_mower`/ttyS9, `serial_imu`/ttyS1,
   `serial_rtk`/ttyS4, `serial_ble`/ttyS3, ttyUSB0-2). Two masters on one UART
   garbles both.
2. **Bridged:** keep Noetic up and run a Noetic↔Humble `ros1_bridge` built
   **from source with `mower_msgs`** (prebuilt bridge does not know custom
   messages; camera topics cost CPU/RAM — board already busy per migration doc).

There is no `ros1_bridge` in this image on purpose — it must match the exact
message set and is a separate source build.

## Serial / device map (container gets all of these)

From `09_platform/dev_nodes.txt`, `udev-symlinks.txt`, `tty_settings_live.txt`:

| Container path | Host symlink / note | Baud (live) |
|---|---|---|
| `/dev/ttyS9` | `serial_mower` — mower MCU (raw mode) | 115200 raw |
| `/dev/ttyS1` | `serial_imu` | 115200 |
| `/dev/ttyS4` | `serial_rtk` | 115200 |
| `/dev/ttyS3` | `serial_ble` | 115200 |
| `/dev/ttyUSB0-2` | `serial_lte`→ttyUSB2 (Quectel EC200A) + AT/GNSS ports | — |
| `/dev/video44/53/62` | `right_oa_camera` / `left_oa_camera` / `rear_camera` | — |

Compose (`docker/docker-compose.yml`) uses `privileged: true`, `network_mode: host`
(DDS discovery, no port lists), `/dev` bind-mount plus explicit `devices:` pins,
and mounts `../src → /work/src` (plus `launch/`, `config/`).

## On-device prerequisites

- Docker is **not** preinstalled (migration doc §side-by-side). Install the
  `linux/arm64` Docker engine + compose plugin once (needs ~1 GB free; host has
  ~7.6 GB on `/` and ~24 GB on `/userdata` per `09_platform/df.txt`).
- aarch64 pulls the arm64 image variant automatically; `platform: linux/arm64`
  in compose pins it on mixed-arch build hosts.
- `foxglove-bridge` install is guarded (`|| skip`) in case an arm64 snapshot
  lacks it; everything else (`ros-base`, `nav2-bringup`, `control-toolbox`,
  `serial-driver`, `colcon`) is a hard dependency.

## Mower-side setup

The following scripts live in `ros2_stack/scripts/` and assist with on-device preparation:

- **`setup_mower.sh`** — Idempotent setup: detects Ubuntu 20.04 aarch64, warns if free space <10 GB on `/`, installs `docker.io` via `apt` if missing, adds the current user to the `docker` group, notes the iptables/nft caveat, and verifies Docker functionality with `docker info` (since `docker run hello-world` cannot be used off-image on this host). Set `PRE_PULL=1` to pre-pull `ros:humble-ros-base-jammy` before container bringup.
- **`preflight.sh`** — Preflight checks before bringing up the ROS 2 stack: verifies docker is present, confirms serial device node symlinks (`/dev/serial_mower` → `ttyS9`, `/dev/serial_imu` → `ttyS1`, `/dev/serial_rtk` → `ttyS4`), warns if vendor services (`mower-base`, `mower-logic`, `mower-controller`) are active (they may hold serial ports), warns if free disk on `/userdata` <10 GB, and prints a **go/no-go** summary.

Make both scripts executable after copying:

```bash
chmod +x ros2_stack/scripts/setup_mower.sh ros2_stack/scripts/preflight.sh
```

## Build and run (on a dev host or the mower; never touch 192.168.1.105 blindly)

```bash
cd ros2_stack
./scripts/build.sh [--packages-select <pkg>]   # docker build + colcon build --symlink-install
./scripts/run_stack.sh                         # shell in container (sources overlay)
./scripts/run_stack.sh <pkg> <launch.py> [args...]   # ros2 launch <pkg> <launch.py>
```

## Kernel caveats on the stock BSP
- `CONFIG_POSIX_MQUEUE is not set`: BuildKit's RUN steps fail (`error mounting mqueue`). `scripts/build.sh`/`run_stack.sh` set `DOCKER_BUILDKIT=0`; use the same for manual `docker compose build`. `docker run` is unaffected (verified).
- mqueue not needed at runtime for our Python drivers.
