# Coverage planning — `mower_coverage` on Fields2Cover

Plan for replacing the vendor's `polygon_coverage_planning` stack (ETH Zürich, ROS 1 Noetic,
shipped in `ros2_port_handoff/04_full_src_tree/src/mower_planner/coverage_planning/`) with a
minimal ROS 2 Jazzy package that runs Fields2Cover headland + swath directly.

Status: design only. Nothing built, nothing run on the mower. Every F2C API claim below was
checked against the actual `ros-jazzy-fields2cover` 2.1.x headers and the MowgliNext source,
not from memory.

| Question | Answer |
|---|---|
| Fields2Cover apt package | **`ros-jazzy-fields2cover`** (v2.1.x, arm64 + amd64 in `packages.ros.org/ros2/ubuntu` noble). `rosdep` has **no** `fields2cover` key, so `package.xml` must not `<depend>fields2cover</depend>` — apt-install it by name in the Dockerfile. |
| OR-Tools | Pulled in automatically as `ros-jazzy-ortools-vendor` (a `Depends:` of the F2C deb). Do **not** `find_package(ortools)` — the apt F2C config does not export `ortools::ortools`, unlike MowgliNext's pinned v3 build. |
| Entry point language | **C++17** (`ament_cmake`). Matches both references; the F2C 2.1.0 headers are `cxx_std_17`-clean. |
| Replaces | `polygon_coverage_geometry` + `polygon_coverage_planners` + `polygon_coverage_solvers` + `polygon_coverage_ros` (~5.9k LOC, GPL, CGAL + Mono/GkMa). |
| Licence win | F2C is BSD-3. `polygon_coverage_planning` is GPL-3 **and** its GTSP solver is non-commercial-only (see §1). |

---

## 1. What we are dropping, and why it is not a port candidate

The vendor tree has five relevant packages:

| Package | Contents | Verdict |
|---|---|---|
| `polygon_coverage_msgs` | `PolygonWithHoles`, `PolygonWithHolesStamped`, `PolygonService.srv`, `PlannerService.srv` | superseded (§4) |
| `polygon_coverage_geometry` | CGAL exact-kernel decomposition (boustrophedon + trapezoidal), polygon offsetting, visibility graph, weak monotonicity, TCD | drop |
| `polygon_coverage_planners` | `SweepPlanGraph`, `GTSPProductGraph`, sensor models (line/frustum), `PathCostFunction`, 3 `PolygonStripmapPlanner` variants | drop |
| `polygon_coverage_solvers` | `gk_ma.cc` — a **Mono JIT** host for the C# `GkMa.exe` memetic GTSP solver, downloaded at build time and patched with 19 `.patch` files; plus `combinatorics`, `boolean_lattice` | drop |
| `polygon_coverage_ros` | `PolygonPlannerBase` + 4 node variants, `coverage_planner.yaml` | drop |

Two blockers are structural, not effort-related:

1. **Licence.** `README.md` states the GTSP solver (`gk_ma`) "is free of charge for
   non-commercial purposes only", and the tree is GPL-3 with several GPL CGAL components
   (2D Triangulation, Regularized Boolean Set-Operations, Straight Skeleton, 2D Arrangement).
   Shipping that in a product is a legal decision, not an engineering one.
2. **The solver cannot be built on the target.** `polygon_coverage_solvers/CMakeLists.txt`
   runs `ExternalProject_Add` against a 2010 Nottingham URL
   (`cs.nott.ac.uk/~pszdk/gtsp_ma_source_codes.zip`, with an ETH polybox mirror), builds C#
   with `make -f MakefileCs` and C++ with `make -f MakefileCpp`, then links
   `${MONO_LIBRARIES}` and symlinks `/usr/lib/libmono-native.so`. That is a Mono + live network
   fetch on an aarch64 board. It has no place in `docker/Dockerfile.jazzy`.

The vendor's own driver (`mower_planning/src/cover_plan/coverage.h`) sits on top of this: it
maps `geometry_msgs/Polygon` in, calls `PolygonStripmapPlanner::setup()/solve()` with CGAL
`Point_2`, and returns a `nav_msgs/Path`. So the **only** thing we actually want from the old
stack is "polygon in, ordered coverage path out" — which is exactly what F2C's headland +
swath stages give, without a GTSP solve at all.

---

## 2. Fields2Cover: the facts we verified (against the Jammy/Humble debs; re-verify the Noble versions)

Downloaded and inspected `ros-humble-fields2cover_2.1.0-1jammy.20260909.*_{amd64,arm64}.deb`
and `ros-humble-ortools-vendor_9.9.1-3jammy.*.deb`; on Noble the same source ships as
`ros-jazzy-fields2cover` 2.1.x. Layout below is per-distro (replace `<distro>`):

* **Layout.** Headers → `/opt/ros/<distro>/include/fields2cover{.h,/…}`;
  `libFields2Cover.so` + bundled `libsteering_functions.so`, `libmatplot.so.1.2.0` →
  `/opt/ros/<distro>/lib/<triplet>/`; CMake config → `…/lib/<triplet>/cmake/Fields2Cover/`.
  Installed size ~16 MB. No `COPY --from` build stage needed.
* **Target name.** `Fields2Cover::Fields2Cover` (plus `::steering_functions`, `::matplot`).
  Not the bare `Fields2Cover` that `opennav_coverage` links — use the namespaced form.
* **`find_dependency` set.** `GDAL 3.0`, `Threads`, `Eigen3`, `GEOS CONFIG`, `TBB CONFIG`.
  All satisfied by the deb's own `Depends:` (`libgdal-dev`, `libgeos++-dev`, `libtbb-dev`,
  `libeigen3-dev`).
* **No `find_dependency(ortools)`.** `Fields2CoverTargets.cmake` sets
  `INTERFACE_LINK_LIBRARIES "GDAL::GDAL;GEOS::geos_c;/usr/lib/…/libm.so"` — no
  `ortools::ortools`. MowgliNext's CMakeLists documents the opposite for their hand-built v3
  ("v3's Fields2CoverConfig .cmake does NOT find_dependency(ortools) … the consumer must
  define that imported target itself"). **That workaround is v3-only. Do not port it.**
  `libortools.so.9` is a real `NEEDED` entry on `libFields2Cover.so`, satisfied at runtime
  because `ortools_vendor`'s `vendor_package.dsv` prepends
  `/opt/ros/<distro>/opt/ortools_vendor/lib` to `LD_LIBRARY_PATH`.
* **Version.** `Fields2CoverConfigVersion.cmake` reports `PACKAGE_VERSION "2.1.0"`, and the
  version file only accepts a matching **major**, so `find_package(Fields2Cover 2.1.0 CONFIG
  REQUIRED)` succeeds and a `3.x` pin would fail. Ask for `2.1.0`; do not ask for `3.0.0`.
* **`rosdep` has no `fields2cover` key.** Verified absent from `ros/rosdistro` `base.yaml`,
  `python.yaml`, `ruby.yaml`, `osx-homebrew.yaml`. `opennav_coverage/package.xml` declares
  `<depend>fields2cover</depend>`, which will make `rosdep install` fail on a fresh machine —
  do not copy that line.
* **Python bindings ship too** (`/usr/lib/python3/dist-packages/fields2cover/`) but we do not
  use them; C++ is the entry point (§3).

### v2.1.0 API surface we need (all present)

```cpp
#include "fields2cover.h"          // umbrella; also gives the f2c::* namespaces

f2c::hg::ConstHL     hl;                                        // headland_generator/constant_headland.h
f2c::types::Cells    mainland = hl.generateHeadlands(cells, w);  // F2CCells (Cells, double)
std::vector<f2c::types::Cells> passes =
    hl.generateHeadlandSwaths(cells, swath_width, n_swaths, /*dir_out2in=*/true);

f2c::sg::BruteForce  bf;  bf.setStepAngle(5°);
f2c::types::Swaths   sw = bf.generateSwaths(angle_rad, op_width, cell);
f2c::types::Swaths   sw = bf.generateBestSwaths(n_swath_obj, op_width, cell);  // f2c::obj::NSwath

f2c::rp::BoustrophedonOrder order;  // : f2c::rp::SingleCellSwathsOrderBase
f2c::types::Swaths   sorted = order.genSortedSwaths(sw);      // (variant = 0 default)
```

Type helpers that matter for ingestion, all verified present in `types/`:

* `f2c::types::LinearRing::addPoint(double, double, double z = 0)` and `addPoint(const Point&)`.
* `f2c::types::Cell(const f2c::types::LinearRing&)` (explicit) and `Cell::addRing(const LinearRing&)`.
* `Cells::addGeometry(const Cell&)`, `Cells::size()`, `Cells::getGeometry(i)`.
* `Cell::size()`, `Cell::getGeometry(i) → LinearRing`, `Geometries::area()` (on `Cell` **and** `Cells`).
* `LinearRing::getGeometry(i) → Point`, `Point::getX()/getY()`.
* `Swath::getPath() → LineString`; `LineString::getGeometry(i) → Point`.
* `Cells::buffer(double width)` / `Cells::buffer(double width, int side)` — the upstream
  GEOS buffer round-trip. Read the MowgliNext comments before calling it with `0.0`: their
  `#429` note is that `buffer(-0.0)` is a real buffer pass, not a no-op, and it can re-node
  the polygon and drop marginal parts.

Two upstream bug notes from MowgliNext's Dockerfile that **do not apply** to the apt package
(they patch a v3 commit; 2.1.0 ships its own fixes — v2.1.0's changelog lists
`generateBestSwaths` no longer returning an angle that covers nothing, and the `NSwathModified`
edge-cost bug). Keep them in mind only if we ever move to a v3 source build.

---

## 3. Shape of a minimal `mower_coverage`

```
ros2_stack/src/mower_coverage/
├── CMakeLists.txt              # ament_cmake, C++17, ~90 lines
├── package.xml                 # ament_cmake; NO <depend>fields2cover</depend>
├── include/mower_coverage/
│   ├── coverage_planning.hpp   # pure F2C, no ROS types in the signature
│   └── coverage_server.hpp     # nav2_util::LifecycleNode + SimpleActionServer
├── src/
│   ├── coverage_planning.cpp   # planBoustrophedon(): F2C → ring + swath geometry
│   ├── coverage_server.cpp     # action callback, param read, Path assembly
│   └── main.cpp                # rclcpp::init / spin / shutdown
├── action/PlanCoverage.action  # or reuse mower_interfaces (see §4)
├── config/coverage_planner.yaml
├── launch/coverage_planner.launch.py
└── test/test_coverage_planning.cpp
```

Deliberately *not* in scope: decomposition, route optimisation, turn planning, RViz polygon
tool, XML-RPC bridge. Three F2C calls do all the work.

### 3.1 The planner core (no ROS in it)

```cpp
struct CoveragePlan {
  std::vector<std::vector<std::pair<double,double>>> rings;   // densified closed loops, outermost first
  std::vector<std::pair<std::pair<double,double>,
                         std::pair<double,double>>>  swaths; // {start,end}, serpentine order
  double swath_angle_rad = 0.0;
  std::vector<std::string> drops;     // "dropped swath len=0.12<0.15"
  double planned_fraction = 0.0;      // strip areas / field area — visibility, not a guarantee
};

CoveragePlan planBoustrophedon(const f2c::types::Cell& field,
                               double op_width,          // swath spacing  → border width
                               int    headland_passes,   // ≥ 0
                               double border_inset,      // chassis pull-back from the recorded line
                               double mow_angle_rad,     // < 0 → auto (fewest swaths)
                               double min_swath_length);
```

Algorithm, in order:

1. **Ingest.** `dedupClosedRing()` per polygon: drop vertices within 1 cm of the previous kept
   vertex, drop near-collinear spikes, re-close (first == last), then repair self-touch via
   `OGRPolygon::Buffer(0.0)`. F2C/boost rejects a zero-length edge and *silently* drops the
   area — this is the single highest-value gate, and `mowgli_coverage` unit-tests it directly
   (`RingDedup.DropsDoubledLeadingVertex`).
2. **Ring count.** `headland_passes < 0` → 0 rings (swaths mow to the boundary);
   `== 0` → `max(1, ceil(border_width / op_width))`; `> 0` → that many.
3. **Boundary offset.** `hl.generateHeadlands(cells, border_inset - op_width/2)`. The
   `- op_width/2` is not cosmetic: F2C 2.1.0 places the outermost driven pass half a swath
   inside the planning cell in *both* generators, so without the term every pass sits
   `op_width/2` too deep and the "mow to the edge" mode undercuts by ~8 cm at shipped defaults.
4. **Headland rings.** `hl.generateHeadlandSwaths(safe_cells, op_width, n, dir_out2in=true)`,
   then per pass take each `Cell`'s exterior `LinearRing`. **v2.1.0 returns
   `std::vector<F2CCells>`** — one `Cells` per pass, rings as its cells. (MowgliNext's v3
   returns `vector<F2CMultiLineString>` of chained 2-point segments instead, and their
   `ringPassToLoops()` exists to re-stitch those. On 2.1.0 you get rings directly and can skip
   the stitching entirely.)
5. **Mainland.** `hl.generateHeadlands(safe_cells, n * op_width)` — skipped entirely when
   `n == 0` (never call `buffer(-0.0)`, see above).
6. **Swaths.** Per mainland cell: fixed angle → `bf.generateSwaths(angle, op_width, cell)`;
   auto → `bf.generateBestSwaths(f2c::obj::NSwath{}, op_width, cell)` after
   `bf.setStepAngle(5°)` (F2C's 1° default regenerates the whole swath set 360×; 5° cuts that
   ~5× and the `NSwath` objective is near-flat within a few degrees). Then
   `order.genSortedSwaths(...)`. Drop any swath shorter than `min_swath_length`.
7. **Report.** Emit `drops` and `planned_fraction`. Silent coverage loss is the failure mode
   worth designing against.

### 3.2 Geometry knobs → parameters

| Parameter | Default | Maps to |
|---|---|---|
| `operation_width` | 0.18 | F2C `op_width` / `cov_width` — swath spacing. **This is our "border width".** |
| `headland_width` | 0.20 | desired headland band; only feeds the auto ring count |
| `num_headland_passes` | 0 | `-1` none, `0` auto (`ceil(headland_width/op_width)`, min 1), `>0` forced |
| `border_inset` | 0.0 | chassis pull-back inside the recorded line |
| `mow_angle_rad` | −1 | goal field; `< 0` = auto angle |
| `min_swath_length` | 0.15 | drop sliver clips |

Declare each with `if (!has_parameter(n)) declare_parameter<T>(n, def)` — `declare_parameter`
**throws** on a second call, and `on_configure` runs again after any `on_cleanup`, which
otherwise leaves the node wedged unconfigured.

### 3.3 Server shape

`nav2_util::LifecycleNode` + `nav2_util::SimpleActionServer<PlanCoverage>` on `plan_coverage`,
`on_activate` → `action_server_->activate()` + `createBond()`. Lifecycle parity with the other
Nav2 servers means `lifecycle_manager_navigation` picks it up with no config change.

**Nav2 1.3 API deltas vs MowgliNext (Lyrical).** Their server will not compile here as-is:

| MowgliNext (Lyrical) | Jazzy |
|---|---|
| `nav2::LifecycleNode`, `nav2::SimpleActionServer`, `nav2::CallbackReturn` | `nav2_util::…` — `nav2_ros_common` **does not exist in Jazzy either (Lyrical+)** |
| `find_package(nav2_ros_common)` | `find_package(nav2_util)` (already implied by `ros-jazzy-nav2-bringup`) |
| `SimpleActionServer(node, name, cb, goal_received_cb, completion_cb, timeout, spin)` | 6-arg form: `(node, name, cb, completion_cb, timeout, spin)` — **no `GoalReceivedCallback`**, and the last arg is `rcl_action_server_options_t`, not `bool realtime` |
| `f2c::` v3 types | v2.1.0 `f2c::` types (same namespaces — see §2) |

`activate()`, `deactivate()`, `terminate_current()`, `terminate_all()`, `is_server_active()`,
`is_cancel_requested()`, `get_current_goal()` all exist on Jazzy's
`SimpleActionServer`; `createBond()`/`destroyBond()` on `LifecycleNode`. So the port is
mechanical.

---

## 4. Message compatibility

### 4.1 Input: polygon + border width

Neither reference carries swath/border width in the message — both put it in parameters.

| Reference | Polygon in | Holes | Width in message? |
|---|---|---|---|
| `polygon_coverage_msgs/PolygonWithHoles` | `geometry_msgs/Polygon hull` | `Polygon[] holes` | no (`lateral_footprint`/`wall_distance` are ROS params in `coverage_planner.yaml`) |
| `mowgli_interfaces/PlanCoverage` goal | `geometry_msgs/Polygon outer_boundary` | `geometry_msgs/Polygon[] obstacles` | no — `operation_width` / `default_headland_width` are `coverage_server` params |
| `opennav_coverage_msgs/ComputeCoveragePath` goal | `Coordinates[] polygons` (`axis1`/`axis2`, GPS-or-cartesian) | later entries in the same list | partly — `HeadlandMode.width`, `SwathMode.best_angle`/`step_angle` |

**Decision: adopt `PlanCoverage`'s shape and keep width in parameters.** Add to
`ros2_stack/src/mower_interfaces/action/PlanCoverage.action`:

```
# Goal
geometry_msgs/Polygon outer_boundary
geometry_msgs/Polygon[] obstacles
float64 mow_angle_deg     # swath heading in the map frame; < 0 = auto (fewest swaths)
---
# Result
bool success
string message
nav_msgs/Path[] segments        # rings outermost-first, then swaths
uint8[] segment_types           # parallel: SEGMENT_RING=0, SEGMENT_SWATH=1
nav_msgs/Path   full_path       # concatenation, for the GUI
float64 total_distance
uint32   ring_count
uint32   swath_count
float64  planning_time_s
---
# Feedback
string phase
```

`geometry_msgs/Polygon` everywhere (not `Coordinates`): our polygons are already metric in the
map frame from the map server, so the GPS-axis indirection buys nothing and costs a
`frame_id` + `use_gml_file` branch. Add `MowAngleDeg` semantics verbatim so a future
`mowgli_behavior` port drops in.

One compatibility trap: **`geometry_msgs/Point32`, not `Point`.** `Polygon.points` is
`Point32` (float32). Fine at map-frame magnitudes in metres, but do not feed raw WGS84 degrees
in — `opennav_coverage`'s `Coordinates` float32 axis is exactly why it needs its own type.

### 4.2 Result: what the caller actually gets

`opennav_coverage_msgs` returns `PathComponents{ Swath[] swaths; Path[] turns;
contains_turns; swaths_ordered }` plus a flat `nav_msgs/Path nav_path`, and its
`NavigateCompleteCoverage` BT walks swath/turn pairs. That model presumes F2C's Dubins /
Reeds-Shepp turn planning is driving the machine.

**We are not porting turn planning.** Airseekers is a diff-drive that pivots in place
(`RotationShimController`), so a coverage plan carries **no turn geometry** — turns are the
navigation stack's job. `mowgli_coverage` states this as the reason for its whole design:
every attempt to let F2C plan turns (Dubins Ω-loops, CC-Dubins, Reeds-Shepp cusps) produced
geometry the chassis could not track. So:

* Do **not** port `opennav_coverage_msgs/PathComponents`, `Swath.msg`, or any `*Mode.msg`.
  They exist only to carry turns and per-stage knobs we do not expose.
* The `segments` + `segment_types` list is the right shape for us: explicit, ordered,
  individually-drivable pieces, each one `FollowPath` goal, pivots between them.
* If we later want the continuous hole-free sub-paths MowgliNext drives (their
  `drivable_subpaths`, joined by forward turn-around arcs bounded to the outermost ring), that
  is **new geometry we would have to write** — roughly half of `coverage_planning.cpp` is that
  connector/fillet/ordering machinery, and none of it exists upstream. Phase it separately;
  it is not needed for a first working plan.

---

## 5. Build and test against our Docker image

### 5.1 `docker/Dockerfile.jazzy`

```dockerfile
# Coverage planning (F2C 2.1.0). Brings its own OR-Tools via
# ros-jazzy-ortools-vendor and its own GDAL/GEOS/TBB/Eigen dev headers.
# rosdep has NO `fields2cover` key, so this must be a literal apt name.
RUN apt-get update && apt-get install -y --no-install-releases \
      ros-jazzy-fields2cover \
    && rm -rf /var/lib/apt/lists/*
```

That is the whole addition. No builder stage, no `COPY --from`, no source build — this is the
single biggest win over the baseline's plan, which assumed a from-source arm64 F2C build
mirroring MowgliNext's `fields2cover-v3-builder`.

### 5.2 `CMakeLists.txt` essentials

```cmake
cmake_minimum_required(VERSION 3.8)
project(mower_coverage)

set(CMAKE_CXX_STANDARD 17)          # F2C 2.1.0 headers are cxx_std_17
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)    # NOT nav2_ros_common — absent in Humble
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mower_interfaces REQUIRED)
find_package(Fields2Cover 2.1.0 CONFIG REQUIRED)   # namespaced target, no ortools workaround

add_library(mower_coverage_core SHARED src/coverage_planning.cpp src/coverage_server.cpp)
target_link_libraries(mower_coverage_core
  Fields2Cover::Fields2Cover        # NOT bare `Fields2Cover`
  ${nav2_util_TARGETS} ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS}
  ${mowgli_interfaces_TARGETS} ${rclcpp_TARGETS} ${rclcpp_action_TARGETS})

ament_export_dependencies(Fields2Cover nav2_util ...)
```

Runtime RPATH: `/opt/ros/jazzy/lib/<triplet>` is already on `LD_LIBRARY_PATH` and
`ortools_vendor` prepends its own lib dir, so unlike MowgliNext's `/opt/fields2cover-300`
build **no `INSTALL_RPATH` juggling is needed**. (MowgliNext sets `INSTALL_RPATH
"/opt/fields2cover-300/lib"` on both the lib and the exe because their prefix is outside
`/opt/ros`; ours is inside.)

### 5.3 Tests

Follow `mowgli_coverage`'s pattern — pure-geometry gtests against the **real** F2C library, no
robot, no ROS node:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_coverage_planning test/test_coverage_planning.cpp)
  target_link_libraries(test_coverage_planning mower_coverage_core Fields2Cover::Fields2Cover)
endif()
```

`mowgli_coverage/test/test_coverage_planning.cpp` is **2 753 lines / 58 test cases** and is the
best available specification of the geometry we want. Its own header says the point is "to
catch a broken plan (empty / out-of-bounds / non-serpentine / hole-crossing) in CI instead of
on the robot." Directly liftable cases:

* `SquareCoversField`, `SquareSwathsAreSerpentine`, `FixedAngleIsHonouredAndDeterministic`
* `HoleIsNotCrossed`, `ConcaveFieldIsCovered`, `TooSmallFieldGivesEmptyPlan`
* `RingDedup.*` (doubled leading vertex, already-clean ring, degenerate ring still plans)
* `F2cPlacesOutermostSwathHalfOpWidthInsideThePlanningCell` — pins the `op_width/2` convention
* `HeadlandDisabledBorderNoWorseThanRingsOn`, `HeadlandDisabledClearanceRingIsExactlySafeBoundary`
* `LargeFieldUsesLongestEdgeAngleFallback`, `NegativeLongestEdgeNeverFallsBackToAutoSearch`
* `DiagnosticsReportPlannedFraction`

Two upstream thresholds worth copying as **ours**, because they were found in the field:

* Auto-angle search is O(area). A 100×100 m field takes ~24 s (past a 15 s action timeout), so
  gate it: above ~400 m² use the boundary's longest-edge angle instead of
  `generateBestSwaths`. ~3 orders of magnitude cheaper, deterministic, and on a rectangular
  lawn it is the angle the search converges to anyway.
* Ring-corner filleting: F2C closes every concentric ring at the *same* polygon corner, so the
  ring→ring junction demands a ~90–112° change across an `op_width` gap. Rotate each loop to
  start mid-longest-edge before filling. Filleting the *densified* loop silently caps the radius
  at ~0.03 m; sparsify to true corners first.

Run:

```bash
./scripts/build.sh --packages-select mower_coverage
docker compose -f docker/docker-compose.yml run --rm mower_jazzy \
  bash -c 'source /opt/ros/jazzy/setup.bash; cd /work; colcon test --packages-select mower_coverage; colcon test-result --verbose'
```

Expected first-run result: the F2C 2.1.x headers compile clean under the Noble toolchain (`g++ 13`). Two
upstream v3 issues MowgliNext had to `sed`-patch (`<iomanip>` in `visualizer.cpp`, `<algorithm>`
in `Graph.cpp`) are **already fixed in 2.1.0** — do not carry the patches.

---

## 6. Alternative: reuse `mowgli_coverage` directly

Instead of writing `mower_coverage`, vendor `mowgli_coverage` (7 files, 6.3k LOC) into
`ros2_stack/src/` and adapt it.

**What that buys.** The connector/fillet/sub-path-ordering machinery that took MowgliNext
months of field iteration (`buildContinuousSubPaths`, `buildConnector` Dubins search,
`roundSharpCorners`, `clampInsideRing`, `ConnectorStats`, the resume-by-index determinism
guarantees) arrives already written and already field-proven. Their `drivable_subpaths` output
is what their `FTCController` drives end-to-end.

**What it costs.**

1. **Fields2Cover v3 is unreleased.** Their `CMakeLists.txt` pins
   `find_package(Fields2Cover 3.0.0 CONFIG REQUIRED PATHS /opt/fields2cover-300 …)` against a
   hand-built v3 branch commit (`884d895`, 2026-03-02) plus two `sed` patches. The apt package
   is **2.1.0**. So vendoring means reproducing the entire `fields2cover-v3-builder` Docker
   stage on arm64 — the long source build the baseline already flagged as a schedule risk.
   The code then needs the v2→v3 `generateHeadlandSwaths` return-type adaptation
   (`vector<F2CCells>` → `vector<F2CMultiLineString>`, plus re-implementing `ringPassToLoops`).
2. **API drift against Humble.** `nav2_ros_common` → `nav2_util`; the 7-arg
   `SimpleActionServer` ctor → 6-arg. Mechanical but touches every translation unit.
3. **Coupling.** `coverage_server.cpp` reads 14 parameters, four of them live per plan, and
   `coverage_planning.cpp` is threaded with field-specific tuning narrative. It is one product's
   configuration expressed as code.
4. **Message dependency.** It hard-depends on `mowgli_interfaces` (16 msg / 15 srv / 3 action)
   for a single action definition.

**Recommendation.** Start with the minimal package in §3 on apt F2C 2.1.0 — it is a few hundred
lines, builds on the first try, and the geometry tests port directly. If the continuous
sub-path execution model turns out to matter, lift `buildContinuousSubPaths` +
`buildConnector` + `roundSharpCorners` from `mowgli_coverage` as a **second increment** (they
are self-contained, ROS-free functions over `BoustrophedonPlan`), rather than vendoring the
whole server and living with a v3 source build. Keep `mower_coverage` on apt F2C; that single
decision is worth more than the code reuse.

### Sequencing

1. Add `ros-jazzy-fields2cover` to `Dockerfile.jazzy`; confirm
   `find_package(Fields2Cover 2.1.0 CONFIG REQUIRED)` configures inside the container.
2. Add `PlanCoverage.action` to `mower_interfaces`.
3. Port the pure-geometry planner + `dedupClosedRing` / `bufferRingOutward`; land the ported
   tests **first** and watch them fail, then make them pass.
4. Add the lifecycle server + parameter file + launch.
5. Validate on a recorded polygon offline (no robot), then under Nav2 with the blade off.
6. Optionally lift the continuous-subpath connector machinery.