# Airseekers Tron ROS 2

<p align="center">
  <img src="img/tron01_1440x.webp" alt="Airseekers Tron robot mower" width="480">
</p>

An open-source, self-hosted software stack for the **Airseekers Tron** robot lawn mower, built on
**ROS 2 Jazzy**. It replaces the vendor's ROS 1 software with a stack you can read, change and run
yourself, and gives you a web app to map your garden and control the mower from a phone or laptop.
No cloud account is needed to run it.

> **Status: experimental.** Docking, undocking and mowing have all been tested, but the stack is
> still young. See [What works today](#what-works-today) before you trust it with a running blade. This project is unofficial and not affiliated with or endorsed by Airseekers.

## What it does

- **Web control app.** Open `http://<mower-ip>:4006` to see the mower live, drive it with an
  on-screen joystick, start, pause and send it home.
- **Draw and edit your garden.** Record a mowing boundary by driving the edge, then add obstacles,
  no-go zones, transit paths and a docking corridor. The map can be edited from a desktop browser as
  well as a phone, so you can tidy boundaries and zones on a big screen. Import an existing vendor
  map if you have one.
- **Choose how each area is mowed.** See [Mowing patterns](#mowing-patterns) below. Settings are
  per area, and one run can mow several areas.
- **Dock and charge by itself.** Undocks to start a mow, returns to the charging station when
  finished or when the battery is low, and resumes where it left off.
- **Centimetre-level positioning.** Fuses RTK GPS, wheel odometry and the IMU. Optional visual
  odometry (OpenVINS) uses the front stereo camera.
- **Obstacle awareness.** Detects people, pets and objects with the front stereo camera and the
  on-board NPU, then stops or goes around them. Bumper, lift and stop-button handling are built in.
- **Safe by design.** Wheels only move in the states where motion is allowed; a lost connection,
  crashed node or stale command stops the mower; crashes are recorded for diagnosis.
- **Test without a mower.** A headless simulator runs the real navigation, mission and docking
  code, so changes can be checked before they go near a lawn.

## Mowing patterns

Each area has its own settings, chosen in the app:

| Setting | Options |
|---|---|
| **Pattern** | Zigzag (parallel stripes), cross-hatch (stripes in two directions), alternating zigzag (the angle changes each mow), spiral, or contour only (just the edge) |
| **Stripe direction** | Automatic, or a fixed angle. On slopes it can run across the slope, up and down it, or pick automatically by steepness |
| **Stripe order** | Race-track (the default, wide gentle turns that are kind to the lawn), snake, spiral groups, or neighbour-to-neighbour |
| **Edge laps** | 0 to 4 passes round the border, mowed first or last |
| **Turns** | Automatic (forward loop, else reverse-and-curve, else pivot), or force loop, reverse or pivot, with a minimum turn radius |
| **Also per area** | Cutting height (30-90 mm), speed, path width, edge margin, repeats per mow, obstacle sensitivity, and whether to avoid flower beds and gravel |

After a mow, gaps the mower missed can be detected and filled.

## What works today

| Area | State |
|---|---|
| Hardware drivers (MCU, RTK GPS, IMU, cameras) | Running on the mower |
| Web app, map recording and editing (phone and desktop), mission control | Working |
| Mowing: coverage planning, patterns and path following | Tested |
| Docking and undocking | Tested |
| Obstacle detection and stop/avoid policy | Working on the bench; more field testing wanted |
| Scheduling, weather skip, notifications | Planned |

The detailed, dated picture is in [docs/STATUS.md](docs/STATUS.md) and
[docs/ROADMAP_2026-10-06.md](docs/ROADMAP_2026-10-06.md).

## Getting started

You will need an Airseekers Tron, a computer on the same network, and comfort with SSH and Docker.
**Everything runs in containers on the mower; the stack never reflashes the mower's control boards
or changes the host OS.** Always begin with the wheels off the ground, and never start the blade
or drive without being ready to hit the stop button.

1. **Clone** (the repo uses a submodule):
   ```bash
   git clone --recurse-submodules https://github.com/mikey0000/ros2-airseekers-tron.git
   cd ros2-airseekers-tron
   ```
2. **Build the images** on your computer (the mower is too small to build on):
   ```bash
   ARCH=arm64 ./scripts/build_f2c.sh      # path-planning library
   ./gui/build.sh                         # web app (needs a Mapbox token, see docs/gui.md)
   ```
3. **Provide the AI models.** The vendor's object-detection models can't be redistributed. Copy them
   off your own mower and run `MODELS_SRC=<dir> ./scripts/install_models.sh`, or run without
   perception (`perception:=false det_range:=false`).
4. **Deploy** to the mower and start it:
   ```bash
   ./scripts/deploy_to_mower.sh all
   ```
5. **Open the app** at `http://<mower-ip>:4006`, record your boundary, and press start.

Full instructions, including first-time mower setup and starting the stack at boot, are in
[docs/deployment.md](docs/deployment.md). To try things with no hardware at all, see
[docs/simulation.md](docs/simulation.md).

## Using it

1. **Map the garden** from the Map page by driving the outline with the joystick; close the loop to
   create an area. Add obstacles and no-go zones as needed. Once recorded, open the app on a desktop
   browser to refine boundaries, zones and paths on a larger screen.
2. **Set the dock** position so the mower knows where to return.
3. **Pick a pattern** for each area, then **start a mow** from the dashboard, or from the schedule page once scheduling is set up.
4. **Watch and intervene.** Pause, send home, or stop at any time from the app or the mower's
   physical stop button.

More on the app in [docs/gui.md](docs/gui.md); phone Wi-Fi tips in [docs/wifi.md](docs/wifi.md).

## Documentation

| If you want to... | Read |
|---|---|
| Install and update the mower | [docs/deployment.md](docs/deployment.md) |
| Use the web app | [docs/gui.md](docs/gui.md) |
| Understand positioning | [docs/localization.md](docs/localization.md) |
| Understand cameras, detection and VIO | [docs/perception_vio.md](docs/perception_vio.md), [docs/cameras_and_video.md](docs/cameras_and_video.md) |
| See how crashes and failures are handled | [docs/crash_recovery.md](docs/crash_recovery.md) |
| Run the simulator | [docs/simulation.md](docs/simulation.md) |
| Hack on the code (architecture, packages, launch arguments, tests) | [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) |
| See what's planned | [docs/ROADMAP_2026-10-06.md](docs/ROADMAP_2026-10-06.md) |

## Based on MowgliNext

Parts of this project come from **[MowgliNext](https://github.com/mowglinext/mowglinext)**, an
open-source ROS 2 robot-mower stack. The web app, map server, mission/behaviour layer and several
Nav2 plugins are vendored from it (see `third_party/mowglinext/`), and the overall architecture
follows its design. Thank you to the MowgliNext contributors. We are working to contribute the
Tron support back upstream as a hardware backend; the plan is in
[docs/MOWGLINEXT_INTEGRATION.md](docs/MOWGLINEXT_INTEGRATION.md).

MowgliNext is dual-licensed: GPLv3 for personal and community use, with a separate commercial
licence for commercial use. The vendored parts stay under their upstream terms; check
`third_party/mowglinext/LICENSE` before using this in a product. OpenVINS (`third_party/open_vins`)
is from [rpng/open_vins](https://github.com/rpng/open_vins).

## Safety and disclaimer

A robot mower has a spinning blade. This software is provided as is, without warranty. You are
responsible for testing it safely, keeping people and pets clear, and complying with local rules.
Modifying your mower may void its warranty.

## Contributing

Issues and pull requests are welcome. Start with [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) for the
build, test and architecture notes.
