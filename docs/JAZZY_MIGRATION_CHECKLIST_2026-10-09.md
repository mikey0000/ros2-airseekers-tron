# Humble → Jazzy migration checklist (2026-10-09)

Companion to `docs/ROS2_UPGRADE_REVIEW_2026-10-09.md` (the review) and
`docs/mowglinext_baseline.md` §6 / `docs/mowglinext_integration.md` §4.4 (the
Lyrical↔Humble deltas this partly reverses).

**Target:** ROS 2 Jazzy Jalisco, Ubuntu 24.04 Noble userland, base image
`ros:jazzy-ros-base-noble`. Kernel stays `5.10.209`; glibc 2.39 runs on it (the
newer-userland-in-container model is already proven here). Nav2 goes 1.1.x → 1.3.x.

**Scope of change:** the stack is a Humble *backport* of a Lyrical-targeted
MowgliNext, so most of this is *de-backporting*. On Jazzy the `nav2_core` plugin
API is **byte-identical to Humble** — verified against upstream:

| interface | Humble | Jazzy |
|---|---|---|
| `Controller::setPlan` | `const nav_msgs::msg::Path&` | same |
| `Controller::computeVelocityCommands` | returns `TwistStamped` (3-arg) | same |
| `GoalChecker::isGoalReached` | `(Pose, Pose, Twist)` | same |
| `StaticLayer` reads `use_maximum`/`trinary_costmap`/`lethal_cost_threshold`/`unknown_cost_value` from the **costmap** namespace | yes | yes |

So **no C++ source change is required** for `mowgli_nav2_plugins` to compile on
Jazzy. The Kilted/Lyrical `setPlan`→`newPathReceived` break is *not* in Jazzy
(1.3) — it lands at 1.4+.

Legend: **[code]** = must change for the build/runtime to work; **[cleanup]** =
optional, drops a Humble workaround; **[verify]** = confirm availability/semantics
before relying on it; **[gain]** = newly available Jazzy capability worth wiring.

---

## 1. Docker layer (file-by-file, apt)

### 1.1 `docker/Dockerfile.humble` → `docker/Dockerfile.jazzy`

**[code]** base and distro:

- L21 `FROM --platform=linux/arm64 ros:humble-ros-base-jammy` → `ros:jazzy-ros-base-noble`
- L26 `ROS_DISTRO=humble` → `jazzy`
- L18 `ARG F2C_IMAGE=mower-f2c:v3-arm64` — keep, but the F2C layer must be rebuilt (§1.3)
- L115/L116 `.bashrc` `source /opt/ros/humble/setup.bash` → `/opt/ros/jazzy/...`

**[code]** every `ros-humble-*` apt line → `ros-jazzy-*` (L47–L64, L100, L125–L126):

| current | Jazzy |
|---|---|
| `ros-humble-ros-base` | `ros-jazzy-ros-base` |
| `ros-humble-nav2-bringup` | `ros-jazzy-nav2-bringup` |
| `ros-humble-robot-localization` | `ros-jazzy-robot-localization` |
| `ros-humble-ortools-vendor` | `ros-jazzy-ortools-vendor` **[verify]** |
| `ros-humble-grid-map-ros` / `-msgs` / `-cv` | `ros-jazzy-grid-map-*` |
| `ros-humble-behaviortree-cpp` | `ros-jazzy-behaviortree-cpp` **[verify]** (BT.CPP is v4 on Jazzy) |
| `ros-humble-twist-mux` | `ros-jazzy-twist-mux` |
| `ros-humble-foxglove-bridge` | `ros-jazzy-foxglove-bridge` |
| `ros-humble-xacro` / `-robot-state-publisher` / `-joint-state-publisher` | `ros-jazzy-*` |
| `ros-humble-v4l2-camera` | `ros-jazzy-v4l2-camera` |
| `ros-humble-cv-bridge` / `-image-transport` | `ros-jazzy-*` |
| `ros-humble-tf-transformations` | `ros-jazzy-tf-transformations` |
| `ros-humble-fields2cover` (F2C_V3=0 path, L100) | `ros-jazzy-fields2cover` |
| `ros-humble-vision-msgs` / `-web-video-server` | `ros-jazzy-*` |

**[code]** Python/apt tooling on Noble:

- L67 `python3-colcon-common-extensions` — **[verify]** this is not in Noble's
  own archive; use `ros-dev-tools` (official) or keep from the ROS apt repo. `colcon`
  itself is fine.
- L68 `python3-rosdep`, L69 `python3-vcstool` — present on Noble.
- L70–L78 `python3-serial`, `-pytest`, `-pip`, `-protobuf`, `-evdev`, `-aiohttp`,
  `-shapely`, `-pyproj`, `-transforms3d` — present on Noble.
- L73 `python3-protobuf` on Noble is 4.21.x (Jazzy's `rclpy`/`rosidl` are built
  against it) — fine.

**[code]** `pip3 install` — **Ubuntu 24.04 enforces PEP 668**
(`externally-managed-environment`). Both pip calls need
`--break-system-packages` (or a venv):

- L89 `pip3 install --no-cache-dir 'websockets>=12,<14'` — **row can be deleted
  entirely**: Noble's `python3-websockets` is 12.x, so Jammy's broken 9.1 (the
  reason for the pip line) is gone. If kept, add `--break-system-packages`. **[cleanup]**
- L134 rknn wheel install — add `--break-system-packages`.

**[code]** rknn wheel cp310 → cp312 (2.3.0 ships cp312, no cp314):

- L135 `...-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl`
  → `...-cp312-cp312-manylinux_2_17_aarch64.manylinux2014_aarch64.whl`

**[gain]** once on Noble/Jazzy, `ros-jazzy-rosbag2-storage-mcap` (and
`--snapshot-mode`, Iron+) become available — `mower_control/blackbox.py` documents
that Humble lacks snapshot mode, so its workaround can be simplified later.

### 1.2 `docker/Dockerfile.dev-amd64`

- L13 `ARG BASE=ros:humble-ros-base-jammy` → `ros:jazzy-ros-base-noble`
- L18 `ROS_DISTRO=humble` → `jazzy`
- Same `ros-humble-*` → `ros-jazzy-*` table as §1.1 (L21–L39)
- L59 `pip3 install ... websockets` — same delete/`--break-system-packages` note
- L70 `ros-humble-fields2cover` → `ros-jazzy-fields2cover`
- L76 `.bashrc` `/opt/ros/humble` → `/opt/ros/jazzy`

### 1.3 `docker/fields2cover/Dockerfile` (+ `scripts/build_f2c.sh`)

- L? `ARG BASE=ros:humble-ros-base-jammy` → `ros:jazzy-ros-base-noble`
- apt `ros-humble-ortools-vendor` → `ros-jazzy-ortools-vendor`
- `. /opt/ros/humble/setup.sh` → `. /opt/ros/jazzy/setup.sh`
- `libgdal-dev libgeos-dev libtbb-dev libtinyxml2-dev libeigen3-dev` — present on Noble
- F2C pin `F2C_REF=884d895b...` (v3.0 branch head) unchanged; rebuild once per arch
  (`ARCH=arm64 ./scripts/build_f2c.sh`) → image tag `mower-f2c:v3-<arch>`

### 1.4 `docker/Dockerfile.f2c-overlay`

- `ARG BASE_IMAGE=mower:humble` → `mower:jazzy`; `ARG F2C_IMAGE=mower-f2c:3.0.0-arm64` (verify tag)
- `ros-humble-fields2cover` → `ros-jazzy-fields2cover`
- `ros-humble-ortools-vendor` → `ros-jazzy-ortools-vendor`

### 1.5 `docker/docker-compose.yml`

- L? service `mower_humble:` → `mower_jazzy:`; `container_name: mower_humble` → `mower_jazzy`
- `image: mower:humble` → `mower:jazzy`
- env `ROS_DISTRO=humble` → `jazzy`
- `FASTRTPS_DEFAULT_PROFILES_FILE` path unchanged (`/work/install/...`)
- `command: ["/work/scripts/stack_entry.sh"]` unchanged

### 1.6 `docker/docker-compose.gui.yml`

- env `ROS_CONTAINER_NAME=mower_humble` → `mower_jazzy`
- `DOCKER_HOST`, `GUI_CONTAINER_NAME` unchanged

### 1.7 `docker/docker-compose.video.yml`, `gui/Dockerfile`, `config/cameras/fastdds_*.xml`

- No changes (GUI is Go/Node/debian:bookworm; Fast DDS profiles are distro-agnostic).

---

## 2. Scripts (file-by-file)

All of these carry `/opt/ros/humble` or the `mower:humble` tag / `mower_humble`
container name:

| file | lines to change |
|---|---|
| `scripts/run_stack.sh` | L26/L27 container name; L30 `/opt/ros/humble/setup.bash`→jazzy; L33/L37 `mower_humble`→`mower_jazzy` |
| `scripts/stack_entry.sh` | L6 `/opt/ros/humble/setup.bash`→jazzy (L2 comment too) |
| `scripts/build.sh` | L17 `mower_humble`→`mower_jazzy`; L18 `/opt/ros/humble`→jazzy (L2/L3 comments) |
| `scripts/dev_build.sh` | L10 `IMG=mower:humble-dev-amd64`→`mower:jazzy-dev-amd64`; L15 `ros:humble-ros-base-jammy`→`ros:jazzy-ros-base-noble`; L23 `/opt/ros/humble`→jazzy |
| `scripts/build_f2c.sh` | L19 `ros:humble-ros-base-jammy`→`ros:jazzy-ros-base-noble` (per-arch digest) |
| `scripts/build_openvins_mower.sh` | L27 `IMAGE=mower:humble`→`mower:jazzy`; L40/L41 `mower_humble`→`mower_jazzy`; L72 `/opt/ros/humble`→jazzy |
| `scripts/deploy_to_mower.sh` | L82 `/opt/ros/humble`→jazzy; L119 `docker exec -it mower_humble`→`mower_jazzy`; L2/L5 comments |
| `scripts/crash_report.sh` | L17 `CONTAINER=mower_humble`→`mower_jazzy` (L3/L12 comments) |
| `scripts/setup_mower.sh` | L85–L93 pre-pull `ros:humble-ros-base-jammy`→`ros:jazzy-ros-base-noble` |

`scripts/mower_services.sh` is the **Noetic** systemd teardown (`/opt/ros/noetic`) —
leave untouched.

---

## 3. `src/mower_navigation/config/nav2_params.yaml` — the Nav2 key deltas

This is the bulk of the real work. Every item below is verified against the Jazzy
1.3 upstream defaults/sources.

### 3.1 Plugin-class separator `pkg/Class` → `pkg::Class` **[code]**

Nav2 Iron+ changed the pluginlib `<class name=...>` attributes for the built-in
planners/behaviors. These six must be rewritten (everything else in the file
already uses `::`):

| line | now | Jazzy |
|---|---|---|
| 599 | `nav2_smac_planner/SmacPlannerHybrid` | `nav2_smac_planner::SmacPlannerHybrid` |
| 630 | `nav2_navfn_planner/NavfnPlanner` | `nav2_navfn_planner::NavfnPlanner` |
| 646 | `nav2_behaviors/Spin` | `nav2_behaviors::Spin` |
| 648 | `nav2_behaviors/BackUp` | `nav2_behaviors::BackUp` |
| 650 | `nav2_behaviors/DriveOnHeading` | `nav2_behaviors::DriveOnHeading` |
| 652 | `nav2_behaviors/Wait` | `nav2_behaviors::Wait` |

**Our own plugins are unaffected:** `mowgli_nav2_plugins/*.xml` keep
`name="mowgli_nav2_plugins/FTCController"` (etc.) while `type` is `...::...`, and
pluginlib resolves the config string against `name`. So `mowgli_nav2_plugins/...`
entries (L153, L165, L498) stay as-is.

### 3.2 `controller_server` progress checker **[code]**

- L98 (comment) and L99 `progress_checker_plugin: "progress_checker"` (singular —
  Humble) → `progress_checker_plugins: ["progress_checker"]` (plural list, Iron+).
- `controller_plugins` / `goal_checker_plugins` are already plural — no change.

### 3.3 `behavior_server` **[code]**

- L653 `global_frame: odom` → split into `local_frame: odom` + `global_frame: map`
  (Jazzy has both; `global_frame` is now the map for the global-frame behaviors).
- Plugin names per §3.1.
- The topic keys (`local_costmap_topic`, `global_costmap_topic`,
  `local_footprint_topic`, `global_footprint_topic`, L638–641) are **already the
  Jazzy names** — no change. (Humble's defaults were `costmap_topic`/`footprint_topic`,
  so these were forward-ported; they now become load-bearing rather than silently
  ignored.)
- `assisted_teleop` stays deliberately absent.

### 3.4 `collision_monitor` polygons **[code]**

Jazzy renamed the polygon threshold and inverted the sense:

- Humble `max_points: N` (stop when `>= N`; comment "> 3 points inside")
- Jazzy `min_points: N+1`, default 4; `max_points` still works as a **deprecated**
  alias (`min_points = max_points + 1`, logs a warning).

| line | now | Jazzy |
|---|---|---|
| 736 | `max_points: 3` (stop_zone) | `min_points: 4` |
| 747 | `max_points: 3` (slowdown_zone) | `min_points: 4` |
| 758 | `max_points: 1` (approach) | `min_points: 2` |

(Use `min_points: N+1` — that is exactly what Jazzy's own alias does, and it
preserves the author's stated "more than N" intent.)

- **[gain]** The Jazzy node adds `state_topic` (default empty). Set
  `state_topic: /collision_monitor_state` (`nav2_msgs/CollisionMonitorState`) to
  surface stop/slowdown state to the GUI (`mowglinext_baseline.md` §6 notes the newer
  `CollisionMonitorState`).
- Jazzy's SLOWDOWN polygon also has `linear_limit`/`angular_limit` (Jazzy defaults)
  alongside `slowdown_ratio`; `slowdown_ratio` (L746) still works, so no change
  required — but the newer limits give per-axis control if needed.
- Observation `type: "pointcloud"` (L765, L771) is unchanged in Jazzy.

### 3.5 Costmaps — the Humble StaticLayer workaround **stays valid**

`test_global_static_layer_options_at_costmap_level` asserts the mask reads
`use_maximum`/`trinary_costmap`/`lethal_cost_threshold`/`unknown_cost_value` from
the **costmap** namespace. Verified identical in Humble and Jazzy
`nav2_costmap_2d/plugins/static_layer.cpp` (`node->get_parameter("use_maximum")`
etc.). **No change** — the file's costmap-level placement is correct for Jazzy.

### 3.6 `bounds_keeper` inflation workaround **[cleanup]**

`test_local_costmap_inflation_with_bounds_keeper` documents a **Humble 1.1.20
`InflationLayer::reset()` bug** (leaves `current_ = false` until `updateCosts()`,
and `LayeredCostmap` skips it on empty bounds), worked around with an empty
`ObstacleLayer` (`bounds_keeper`). **[verify]** whether Jazzy 1.3 still needs it by
running the local costmap without `bounds_keeper`; if the bug is fixed, delete the
layer from `local_costmap.plugins` and the corresponding test assertions.

### 3.7 Generally compatible / no required change

- `velocity_smoother` L? — Jazzy adds `stamp_smoothed_velocity_with_smoothing_time`
  (default false) and `enable_stamped_cmd_vel`; current config stays valid.
- `bt_navigator` — Jazzy adds `navigators: ["navigate_to_pose",
  "navigate_through_poses"]` + per-navigator plugin entries, `error_code_names`,
  `plugin_lib_names`, `action_server_result_timeout`. Defaults exist, and injecting
  `default_nav_to_pose_bt_xml` / `default_nav_through_poses_bt_xml` from
  `navigation.launch.py` still works. Optionally add `error_code_names` (see §4).
- MPPI — Jazzy adds `ax_max/ax_min/ay_max/ay_min/az_max` and CostCritic
  `near_collision_cost`/`trajectory_point_step`. Existing keys valid; the
  `model_dt >= 1/controller_frequency` "throw" note becomes a warning, not a throw.
- `lifecycle_manager_navigation` `node_names` ordering unchanged; still
  `nav2_lifecycle_manager`.
- Costmap layer plugin names already use `::` (Obstacle/Inflation/Static/Keepout).

### 3.8 New Jazzy capabilities (optional, **[gain]**)

`docking_server` (opennav_docking), `route_server`, `waypoint_follower`,
`smoother_server`, `map_server`/`amcl`, and `enable_stamped_cmd_vel` (TwistStamped
end-to-end) all exist. The stack currently omits them deliberately
(`mowglinext_baseline.md` §6); adopting any is a follow-on, not a migration
prerequisite. **Recommendation: keep `enable_stamped_cmd_vel` false** for the first
cut so the teleop/twist_mux/collision_zone_gate/MCU path is untouched.

---

## 4. Behavior trees (`src/mower_navigation/behavior_trees/*.xml`)

- Node set used (`RecoveryNode`, `Sequence`, `RoundRobin`, `ComputePathToPose`,
  `ComputePathThroughPoses`, `FollowPath`, `Wait`, `ClearEntireCostmap`,
  `Fallback`) is stable across Humble→Jazzy.
- Jazzy BT nodes gained `error_code_id` output ports and
  `bt_navigator.error_code_names`. **[verify]** the custom trees still load without
  them (they are optional). If warnings appear, add the two names from §3.7.
- `Wait wait_duration` — Humble requires an integer (InputPort<int>, hence the test's
  `isdigit()` assertion); Jazzy's `Wait` accepts a double. Integer literals remain
  valid, so no change, but the test's integer-only assertion can be relaxed.

---

## 5. Source-package touch-ups beyond Docker

**[code]** `src/mower_control/mower_control/mow_recorder.py` L134 hardcodes
`os.environ.get('AMENT_PREFIX_PATH', '/opt/ros/humble')` → `'/opt/ros/jazzy'`
(or read `ROS_DISTRO`).

**[verify]** rclpy-internal shims documented as "private in Humble, but stable for
the whole distro":
- `src/*/*/sub_pump.py` L46 (`try:` around a private rclpy subscription attribute) —
  the private name may differ in Jazzy's rclpy.
- `src/um960_gps_driver/um960_gps_driver/um960_node.py` L129 ("Humble only knows
  `NO_FIX..GBAS_FIX`") — Jazzy adds more `NavSatStatus` constants; revisit the mapping.

**[cleanup]** `launch/nav2.launch.py` L16–L43 is a prose apt list of `ros-humble-*`
packages → update to `ros-jazzy-*`. Similar `(Humble)` comments across
`launch/*.launch.py`, `src/mower_teleop/config/twist_mux.yaml`,
`src/mower_navigation/scripts/collision_zone_gate.py`, `src/mower_control/*.py`,
and package descriptions are cosmetic but should move together.

**[verify]** `launch/cameras.launch.py` L17 claims `v4l2_camera` 0.6 (Humble) is
single-planar-only; Jazzy's newer `v4l2_camera` may add multi-planar — if so, a
camera workaround may be droppable.

**[verify]** Foxglove: the Go GUI speaks `foxglove.sdk.v1`
(`third_party/mowglinext/gui`); confirm the version in `ros-jazzy-foxglove-bridge`
(Rosbridge protocol version) matches before shipping.

---

## 6. Tests / CI

**[code]** `src/mower_navigation/test/test_params_yaml.py`:

- `test_progress_checker_counts_rotation` uses `c["progress_checker_plugin"]` → the
  plural list `c["progress_checker_plugins"][0]` (or keep the key lookup via the
  `PLUGIN_SINGLE_KEYS`/`PLUGIN_LIST_KEYS` helpers).
- `test_contract_ids` expects `nav2_smac_planner/SmacPlannerHybrid` and
  `nav2_navfn_planner/NavfnPlanner` → `::`.
- `test_humble_constraints` name/comments (still valid; `enable_stamped_cmd_vel`
  stays absent by choice).
- `test_local_costmap_inflation_with_bounds_keeper` — update/remove if §3.6 is
  cleaned up.
- `test_collision_monitor` — no `max_points` assertion, but add `min_points` once
  §3.4 lands.
- `mowgli_nav2_plugins/...` id assertions (L153/L165/L498) stay.

**[cleanup]** rename `test_humble_constraints` / comments; any test/script that
sources `/opt/ros/humble`.

---

## 7. Suggested execution order

1. `ARCH=arm64 ./scripts/build_f2c.sh` against `ros:jazzy-ros-base-noble` →
   `mower-f2c:v3-arm64` (§1.3). Do this on a host, never on the mower.
2. Bump Dockerfiles + compose + scripts (§1.1–1.2, §2). Filenames may stay
   `Dockerfile.humble`/tag `mower:humble` if you want a smaller diff, but renaming is
   clearer.
3. `docker compose ... build`; resolve the **[verify]** apt names (§1.1) — this is
   where unexpected missing packages surface.
4. Apply `nav2_params.yaml` deltas (§3.1–§3.4, §3.6) and BT tweaks (§4).
5. Update `mow_recorder.py` and the rclpy shims (§5).
6. Update `test_params_yaml.py` (§6); `colcon build` + run the static checks on
   `mower_jazzy`.
7. Smoke on the device: `ros2 launch mower_bringup mower.launch.py` with the smallest
   possible field run; validate the collision_monitor polygon thresholds first
   (`min_points` change is the only safety-behavior delta), then the transit
   controller and coverage.

---

## 8. What this buys (short)

- **LTS to 2029-05** on Jazzy/Noble (vs Humble EOL 2027-05).
- Nav2 1.3: better MPPI, `docking_server`/`route_server` if wanted,
  `CollisionMonitorState`, TwistStamped.
- Python 3.12 removes the Jammy `websockets` pin and other 3.10-era workarounds.
- Much closer to the Lyrical-targeted upstream — less backport drift to maintain.
- `v4l2_camera`, `foxglove_bridge`, rosbag2 snapshot mode all move forward.

**Not** in scope here: Lyrical/26.04 (needs a cp314 RKNN wheel — none exists yet),
and any OS reflash (still unnecessary).
