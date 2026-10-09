# Humble → Jazzy migration checklist (2026-10-09)

> **STATUS: EXECUTED + BUILDS GREEN (2026-10-09).** The migration below has
> been carried out on this branch (Docker, scripts, Nav2 config, plugin XML/config,
> package docs and the operational docs are all on Jazzy/Noble now; see
> `git status`/`git diff`). The amd64 dev workspace **builds clean on Jazzy/Noble**:
> the Fields2Cover v3 layer was rebuilt on Noble (`mower-f2c:v3-amd64`, label
> `com.airseekers.ros_distro=jazzy`) and `colcon build` finishes **33/33 packages**;
> `mower_coverage` links `libgdal.so.34`/`libtinyxml2.so.10` and `ldd` resolves every
> dependency. `mowgli_nav2_plugins` tests pass (**13/13** ctest targets, 170 tests,
> 0 failures). The arm64 layer (`mower-f2c:v3-arm64`, a real aarch64 ELF linking
> Noble arm64 libs) is also built, and `foxglove_bridge` **3.6.0** was verified live
> to negotiate `foxglove.sdk.v1` and advertise `clientPublish`/`parameters`/`services`.
> Still open: on-device image build + smoke test. **Caveat:** the execution included a
> repo-wide mechanical rename of `humble`/`jammy` tokens, which also rewrote the
> *left* ("current/before") column of the comparison tables here, so those tables
> are no longer a literal before/after diff.

Companion to `docs/ROS2_UPGRADE_REVIEW_2026-10-09.md` (the review) and
`docs/mowglinext_baseline.md` §6 / `docs/mowglinext_integration.md` §4.4 (the
Lyrical↔Humble deltas this partly reverses).

**Target:** ROS 2 Jazzy Jalisco, Ubuntu 24.04 Noble userland, base image
`ros:jazzy-ros-base-noble`. Kernel stays `5.10.209`; glibc 2.39 runs on it (the
newer-userland-in-container model is already proven here). Nav2 goes 1.1.x → 1.3.x.

**Scope of change:** the stack is a Humble *backport* of a Lyrical-targeted
MowgliNext, so most of this is *de-backporting*. On Jazzy the `nav2_core` plugin
API matches Humble for every interface we implement — verified against upstream —
**with one exception:** `nav2_core/exceptions.hpp` was split into per-type headers
(`planner_exceptions.hpp`, `controller_exceptions.hpp`, `smoother_exceptions.hpp`,
`route_exceptions.hpp`) and the controller server now catches `ControllerException`
instead of `PlannerException`. `mowgli_nav2_plugins/ftc_controller.cpp` was updated to
throw `nav2_core::ControllerException` (Humble's controller server used
`PlannerException` throughout, which is why the Humble port threw that type).

| interface | Humble | Jazzy |
|---|---|---|
| `Controller::setPlan` | `const nav_msgs::msg::Path&` | same |
| `Controller::computeVelocityCommands` | returns `TwistStamped` (3-arg) | same |
| `GoalChecker::isGoalReached` | `(Pose, Pose, Twist)` | same |
| `StaticLayer` reads `use_maximum`/`trinary_costmap`/`lethal_cost_threshold`/`unknown_cost_value` from the **costmap** namespace | yes | yes |
| `nav2_core/exceptions.hpp` | `PlannerException` + `ControllerException` | **split; controller must throw `ControllerException`** |

So the only production C++ source change required for `mowgli_nav2_plugins` on
Jazzy is the exception type/include above (confirmed by building the workspace).
Its **tests** additionally needed one API update: Jazzy's `Costmap2DROS` 3-arg
constructor gained a required 4th `use_sim_time` argument (Humble had no such
parameter), so `test_ftc_tracking_sim.cpp` / `test_ftc_angle_wrap.cpp` now pass
`false` to match `nav2_params.yaml`. The Kilted/Lyrical `setPlan`→`newPathReceived`
break is *not* in Jazzy (1.3) — it lands at 1.4+.

Legend: **[code]** = must change for the build/runtime to work; **[cleanup]** =
optional, drops a Humble workaround; **[verify]** = confirm availability/semantics
before relying on it; **[gain]** = newly available Jazzy capability worth wiring.

---

## 1. Docker layer (file-by-file, apt)

### 1.1 `docker/Dockerfile.jazzy` → `docker/Dockerfile.jazzy`

**[code]** base and distro:

- L21 `FROM --platform=linux/arm64 ros:jazzy-ros-base-noble` → `ros:jazzy-ros-base-noble`
- L26 `ROS_DISTRO=humble` → `jazzy`
- L18 `ARG F2C_IMAGE=mower-f2c:v3-arm64` — keep, but the F2C layer must be rebuilt (§1.3)
- L115/L116 `.bashrc` `source /opt/ros/jazzy/setup.bash` → `/opt/ros/jazzy/...`

**[code]** every `ros-jazzy-*` apt line → `ros-jazzy-*` (L47–L64, L100, L125–L126):

| current | Jazzy |
|---|---|
| `ros-jazzy-ros-base` | `ros-jazzy-ros-base` |
| `ros-jazzy-nav2-bringup` | `ros-jazzy-nav2-bringup` |
| `ros-jazzy-robot-localization` | `ros-jazzy-robot-localization` |
| `ros-jazzy-ortools-vendor` | `ros-jazzy-ortools-vendor` **[verify]** |
| `ros-jazzy-grid-map-ros` / `-msgs` / `-cv` | `ros-jazzy-grid-map-*` |
| `ros-jazzy-behaviortree-cpp` | `ros-jazzy-behaviortree-cpp` **[verify]** (BT.CPP is v4 on Jazzy) |
| `ros-jazzy-twist-mux` | `ros-jazzy-twist-mux` |
| `ros-jazzy-foxglove-bridge` | `ros-jazzy-foxglove-bridge` |
| `ros-jazzy-xacro` / `-robot-state-publisher` / `-joint-state-publisher` | `ros-jazzy-*` |
| `ros-jazzy-v4l2-camera` | `ros-jazzy-v4l2-camera` |
| `ros-jazzy-cv-bridge` / `-image-transport` | `ros-jazzy-*` |
| `ros-jazzy-tf-transformations` | `ros-jazzy-tf-transformations` |
| `ros-jazzy-fields2cover` (F2C_V3=0 path, L100) | `ros-jazzy-fields2cover` |
| `ros-jazzy-vision-msgs` / `-web-video-server` | `ros-jazzy-*` |

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

- L13 `ARG BASE=ros:jazzy-ros-base-noble` → `ros:jazzy-ros-base-noble`
- L18 `ROS_DISTRO=humble` → `jazzy`
- Same `ros-jazzy-*` → `ros-jazzy-*` table as §1.1 (L21–L39)
- L59 `pip3 install ... websockets` — same delete/`--break-system-packages` note
- L70 `ros-jazzy-fields2cover` → `ros-jazzy-fields2cover`
- L76 `.bashrc` `/opt/ros/jazzy` → `/opt/ros/jazzy`

### 1.3 `docker/fields2cover/Dockerfile` (+ `scripts/build_f2c.sh`)

- L? `ARG BASE=ros:jazzy-ros-base-noble` → `ros:jazzy-ros-base-noble`
- apt `ros-jazzy-ortools-vendor` → `ros-jazzy-ortools-vendor`
- `. /opt/ros/jazzy/setup.sh` → `. /opt/ros/jazzy/setup.sh`
- `libgdal-dev libgeos-dev libtbb-dev libtinyxml2-dev libeigen3-dev` — present on Noble
- F2C pin `F2C_REF=884d895b...` (v3.0 branch head) unchanged; **rebuild once per arch
  (confirmed required)**: a pre-migration (Jammy) v3 layer links `libgdal.so.30`/`libtinyxml2.so.9`
  and fails to link inside the Noble image (`libgdal.so.30 not found`). The layer now carries a
  `com.airseekers.ros_distro=jazzy` label and `dev_build.sh` rebuilds a stale amd64 layer automatically.
  → image tag `mower-f2c:v3-<arch>` (`build_f2c.sh`)
- The v3 pin needs a GCC-13 fix: `visualizer.cpp` uses `std::setw`/`std::setfill`
  without `#include <iomanip>`. The Dockerfile prepends the include **before**
  `cmake --build` (a post-build `|| true` variant silently masked the compile
  failure and produced a misleading `libFields2Cover.so not found` at install).
- Note: the final stage is `FROM scratch` built with the legacy builder
  (`DOCKER_BUILDKIT=0`), so the arm64 layer's reported image `Architecture` is
  `amd64` even though its contents are genuine aarch64. Harmless in the designed
  flow — the device builds with `DOCKER_BUILDKIT=0` too, and the legacy builder
  does not enforce platform matching on `FROM`/`COPY --from`. Transfer/`docker load`
  the layer onto the mower before `deploy_to_mower.sh image`.

### 1.4 `docker/Dockerfile.f2c-overlay`

- `ARG BASE_IMAGE=mower:jazzy` → `mower:jazzy`; `ARG F2C_IMAGE=mower-f2c:v3-arm64` (fixed from the stale `3.0.0-arm64`)
- `ros-jazzy-fields2cover` → `ros-jazzy-fields2cover`
- `ros-jazzy-ortools-vendor` → `ros-jazzy-ortools-vendor`

### 1.5 `docker/docker-compose.yml`

- L? service `mower_jazzy:` → `mower_jazzy:`; `container_name: mower_jazzy` → `mower_jazzy`
- `image: mower:jazzy` → `mower:jazzy`
- env `ROS_DISTRO=humble` → `jazzy`
- `FASTRTPS_DEFAULT_PROFILES_FILE` path unchanged (`/work/install/...`)
- `command: ["/work/scripts/stack_entry.sh"]` unchanged

### 1.6 `docker/docker-compose.gui.yml`

- env `ROS_CONTAINER_NAME=mower_jazzy` → `mower_jazzy`
- `DOCKER_HOST`, `GUI_CONTAINER_NAME` unchanged

### 1.7 `docker/docker-compose.video.yml`, `gui/Dockerfile`, `config/cameras/fastdds_*.xml`

- No changes (GUI is Go/Node/debian:bookworm; Fast DDS profiles are distro-agnostic).

---

## 2. Scripts (file-by-file)

All of these carry `/opt/ros/jazzy` or the `mower:jazzy` tag / `mower_jazzy`
container name:

| file | lines to change |
|---|---|
| `scripts/run_stack.sh` | L26/L27 container name; L30 `/opt/ros/jazzy/setup.bash`→jazzy; L33/L37 `mower_jazzy`→`mower_jazzy` |
| `scripts/stack_entry.sh` | L6 `/opt/ros/jazzy/setup.bash`→jazzy (L2 comment too) |
| `scripts/build.sh` | L17 `mower_jazzy`→`mower_jazzy`; L18 `/opt/ros/jazzy`→jazzy (L2/L3 comments) |
| `scripts/dev_build.sh` | L10 `IMG=mower:jazzy-dev-amd64`→`mower:jazzy-dev-amd64`; L15 `ros:jazzy-ros-base-noble`→`ros:jazzy-ros-base-noble`; L23 `/opt/ros/jazzy`→jazzy |
| `scripts/build_f2c.sh` | L19 `ros:jazzy-ros-base-noble`→`ros:jazzy-ros-base-noble` (per-arch digest) |
| `scripts/build_openvins_mower.sh` | L27 `IMAGE=mower:jazzy`→`mower:jazzy`; L40/L41 `mower_jazzy`→`mower_jazzy`; L72 `/opt/ros/jazzy`→jazzy |
| `scripts/deploy_to_mower.sh` | L82 `/opt/ros/jazzy`→jazzy; L119 `docker exec -it mower_jazzy`→`mower_jazzy`; L2/L5 comments |
| `scripts/crash_report.sh` | L17 `CONTAINER=mower_jazzy`→`mower_jazzy` (L3/L12 comments) |
| `scripts/setup_mower.sh` | L85–L93 pre-pull `ros:jazzy-ros-base-noble`→`ros:jazzy-ros-base-noble` |

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

**Our own plugins move to `::` as well.** pluginlib resolves a plugin's lookup
name to the `<class name=...>` attribute when present, **else** the `type`
(`pluginlib/class_loader_imp.hpp`: *"assuming lookup_name == real class name"*);
there is no `/`→`::` normalisation. So the two forms are mutually exclusive per
plugin. The official Jazzy tutorial confirms it: it declares
`<class type="polygon_plugins::Square" base_class_type="polygon_base::RegularPolygon">`
with **no** `name`, documents `name` as *optional* ("A lookup name (i.e. magic
name) used by the class loader"), and its class listing prints
`Plugin(name='polygon_plugins::Square', …)` — i.e. the lookup name defaults to
`type`. The migration drops the now-redundant `name="mowgli_nav2_plugins/…"`
attribute and uses the `type`-only convention, matching Jazzy's own Nav2 plugin
XMLs (which carry `type="pkg::Class"` with no `name`) and keeping
`nav2_params.yaml` uniform:

| file | line | id |
|---|---|---|
| `src/mowgli_nav2_plugins/ftc_controller_plugin.xml` | 3 | `mowgli_nav2_plugins::FTCController` |
| `src/mowgli_nav2_plugins/goal_checker_plugin.xml` | 3 / 16 | `mowgli_nav2_plugins::PathProgressGoalChecker` / `::LegGoalChecker` |
| `src/mower_navigation/config/nav2_params.yaml` | 153 / 165 / 498 | same ids |
| `src/mower_navigation/test/test_params_yaml.py` | 156 / 160 | same ids |

This intentionally differs from MowgliNext upstream (`third_party/mowglinext`,
pinned `faf658b`), which keeps `name="mowgli_nav2_plugins/…"` + `/` configs —
both resolve; `::` is the Nav2/Jazzy convention and removes a legacy optional
attribute. (Re-verify against `origin/master` if a future port re-syncs the XML.)

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
`os.environ.get('AMENT_PREFIX_PATH', '/opt/ros/jazzy')` → `'/opt/ros/jazzy'`
(or read `ROS_DISTRO`).

**[verify]** rclpy-internal shims documented as "private in Humble, but stable for
the whole distro":
- `src/*/*/sub_pump.py` L46 (`try:` around a private rclpy subscription attribute) —
  the private name may differ in Jazzy's rclpy.
- `src/um960_gps_driver/um960_gps_driver/um960_node.py` L129 ("Humble only knows
  `NO_FIX..GBAS_FIX`") — Jazzy adds more `NavSatStatus` constants; revisit the mapping.

**[cleanup]** `launch/nav2.launch.py` L16–L43 is a prose apt list of `ros-jazzy-*`
packages → update to `ros-jazzy-*`. Similar `(Humble)` comments across
`launch/*.launch.py`, `src/mower_teleop/config/twist_mux.yaml`,
`src/mower_navigation/scripts/collision_zone_gate.py`, `src/mower_control/*.py`,
and package descriptions are cosmetic but should move together.

**[verify]** `launch/cameras.launch.py` L17 claims `v4l2_camera` 0.6 (Humble) is
single-planar-only; Jazzy's newer `v4l2_camera` may add multi-planar — if so, a
camera workaround may be droppable.

**[done]** Foxglove: the Go GUI speaks `foxglove.sdk.v1`
(`third_party/mowglinext/gui`). `ros-jazzy-foxglove-bridge` is **3.6.0**; verified
live 2026-10-09 that it negotiates `foxglove.sdk.v1` (its `libfoxglove.so` defines
the token) and advertises `clientPublish`, `parameters`, `parametersSubscribe` and
`services` — everything the GUI client uses.

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
sources `/opt/ros/jazzy`.

---

## 7. Suggested execution order

1. `ARCH=arm64 ./scripts/build_f2c.sh` against `ros:jazzy-ros-base-noble` →
   `mower-f2c:v3-arm64` (§1.3). Do this on a host, never on the mower.
2. Bump Dockerfiles + compose + scripts (§1.1–1.2, §2). Filenames may stay
   `Dockerfile.jazzy`/tag `mower:jazzy` if you want a smaller diff, but renaming is
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
