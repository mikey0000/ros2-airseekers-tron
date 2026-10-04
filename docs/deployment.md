# Deployment — Humble container on the 20.04 (Noetic) mower

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
