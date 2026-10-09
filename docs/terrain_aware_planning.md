# Terrain-aware planning: terrain memory, slope-aware swaths, traction-aware navigation

Status 2026-10-07: design, plus the first slice implemented (section 6). Nothing in this
document has run on the mower. The only on-robot check was an isolated smoke start of
`map_server_node` on `ROS_DOMAIN_ID=77` with a copy of the maps.

Owner request: "adjust the mow plan, and record issues for future planning: areas where
wheels get stuck, slope optimisation (e.g. turning downhill instead of uphill,
slope-optimised turns). Decide whether this belongs in behaviour (mission), the planner or
Nav2. Look for existing solutions first."

---

## 1. Research summary

### 1.1 Coverage planners
- **Fields2Cover** (F2C, v3 in `mower_coverage`) is strictly 2D. It has no elevation or slope
  input.
  - The swath generator searches angles (BruteForce, `setStepAngle`) against a pluggable
    `SGObjective`. `SGObjective` has overloaded `computeCost` for a swath, swaths, cells and
    angles. NSwath, NSwathModified, SwathLength, FieldCoverage and Overlaps are examples of
    subclasses. A custom slope-weighted objective is therefore possible: subclass it and score
    each swath's alignment with the local fall line from our raster.
    ([sg_objective.h](https://docs.ros.org/en/api/fields2cover/html/sg__objective_8h_source.html),
    [brute_force.h](https://docs.ros.org/en/api/fields2cover/html/brute__force_8h_source.html))
  - Route planners: boustrophedon, snake, spiral, custom order, and an OR-tools based route
    planner ([route planning](https://docs.ros.org/en/api/fields2cover/html/5__route__planning_8cpp_source.html)).
  - Turns: Dubins, DubinsCC and Reeds-Shepp
    ([path planning](https://docs.ros.org/en/api/fields2cover/html/6__path__planning_8py_source.html)).
  - Headlands have one constant width. I found no per-edge width. Not verified in source.
- **Academic work** splits the field into slope-homogeneous sub-fields and picks the driving
  angle per sub-field with an energy or slip model. Jin and Tang (3D terrain coverage)
  classify fields as flat or sloped and give each a different strategy. A review reports
  about 6.5 % energy saved by optimising the angle on 3D terrain versus a flat-terrain
  optimisation ([AAU review](https://vbn.aau.dk/da/publications/intelligent-coverage-path-planning-for-agricultural-robots-and-au);
  [side-to-side 3D CPP](https://vbn.aau.dk/en/publications/side-to-side-3d-coverage-path-planning-approach-for-agricultural-);
  [arXiv 2601.00614](https://arxiv.org/html/2601.00614v1)). Takeaway: the angle is the main
  lever, and it is chosen per region.
- **Patent**: "Slope compensation for autonomous lawn mower planner"
  ([US 2022/0369545](https://patents.justia.com/patent/20220369545)) covers the same topic
  for autonomous mowers. The full text was not retrievable (HTTP 403). The search abstract
  notes that steep grades cause loss of traction of differentially driven wheels, and that a
  mower that stops at a boundary while pointing downhill can get stuck. This motivates
  "turn while pointing downhill" and "avoid swath ends on low-traction or steep cells".

### 1.2 Nav2
- Humble has costmap filters for keepout and speed. The speed filter turns a mask into
  `speed_limit` = `base + multiplier * mask`, as percent (type 1) or absolute (type 2). The
  controller server forwards it via `setSpeedLimit`. RPP supports this; it is not verified
  for our FTC plugin
  ([docs](https://docs.nav2.org/configuration/packages/costmap-plugins/speed_filter.html),
  [humble source](https://api.nav2.org/nav2-humble/html/speed__filter_8cpp_source.html)).
- **Our finding (`nav2_params.yaml`)**: the global costmap's `KeepoutFilter` never touched a
  cell with only an inflation layer (verified 2026-10-06). The zone mask therefore reaches
  Nav2 through a `StaticLayer` on `/nav_keepout_mask`. Two consequences:
  - A second `StaticLayer` (non-trinary, `use_maximum`) is the lowest-risk way to add
    graded traction cost.
  - With the default `trinary_costmap: true` and `lethal_cost_threshold: 100`, the existing
    "soft band" value 90 in the nav mask is read as FREE, not as high cost. This is a
    separate finding and is not changed here.
- I did not research grid_map, elevation_mapping_cupy or MPPI slope critics in depth. They
  target 3D perception (LiDAR or depth elevation maps). We have no LiDAR, and the mower
  covers the same lawn repeatedly, so a learned 2D raster from proprioception (IMU tilt and
  slip events) is cheaper and fits better.

### 1.3 Mowers on the market and open source
- **Husqvarna**:
  - Standard Automowers are rated for 25-50 % slope; the AWD models for up to 70 %.
  - Do not lay the boundary across a slope steeper than 15 %. Where the edge slopes more
    than 15 %, keep the boundary 20-35 cm inside on flat ground, because the mower "struggles
    to turn there", especially on wet grass.
  ([Husqvarna slopes](https://husqvarna.com/us/support/husqvarna-self-service/how-to-optimize-an-automower-robotic-lawn-mower-installation-for-steep-slopes-ka-01431))
- **Mammotion Luba AWD**: rated for 65-75 % slope
  ([Mammotion](https://au.mammotion.com/pages/blog-why-mammotion-luba-awd-series)).
- **Eufy, Roborock and others** handle about 18°. Their documented "stuck in a hollow" fix is
  for the owner to draw a no-go zone in the app, and they raise a "steep slope / tilt too
  large" alert
  ([eufy](https://www.eufy.com/au/blogs/lawn-mower/robot-lawn-mower-slope-limits),
  [Roborock](https://garden-support.roborock.com/hc/en-150/articles/15315082544143-Q1-My-lawn-has-a-slight-depression-area-hole-low-lying-area-and-the-mower-is-stuck-in-it-while-passing-What-should-I-do-next)).
  This is the manual version of what we automate.
- **Human mowing guidance**: walk-behind and self-propelled mowers go across the slope;
  ride-ons go up and down on gentle grades
  ([Sunseeker](https://sunseekertech.com/us/blog/how-to-mow-a-steep-hill),
  [STIHL](https://www.stihl.com.au/en/work-technique-power-tool-maintenance/garden-guides-and-projects/lawn-care/mowing-slopes),
  [Lawn Love](https://lawnlove.com/blog/?p=2131)). For a light diff-drive robot with casters
  the trade-off is:
  - Contour swaths cause cross-slope drift (the caster and wheel slip sideways, so
    cross-track error grows) but keep both wheels equally loaded.
  - Up/down swaths keep tracking easy, but the downhill wheel loses traction at uphill turns.
  - Owner rule: contour above X°, up/down below. It is configurable and must be validated in
    the field.
- **MowgliNext / OpenMower**:
  - MowgliNext (`docs/mowglinext_baseline.md` §4.3/8) has a dig (wheel-slip) detector gated
    on RTK. It raises a latched `DigEvent`, runs an escape manoeuvre and creates a **pending
    keepout proposal** in the map server.
  - Our port has `/dig_stall` (`slip_detector`) and `promote_obstacle` with pending
    obstacles, but nothing connected them, and nothing was remembered across mows.
  - OpenMower itself: the search found no slope or stuck learning.
- **Stuck detection from proprioception**: the usual signals are commanded vs measured wheel
  twist (slip), wheel odometry vs GNSS/EKF track (MowgliNext's worst-wheel vs RTK), motor
  current, IMU tilt and vibration, and time-to-progress. We already have the first two
  (`slip_detector`), the RTK track (EKF), the IMU at 100-200 Hz and Nav2's progress checker.
  Motor current is in `/mower_base/motor_info`; its scale is unverified.

### 1.4 Implications
1. No off-the-shelf package builds a terrain memory from mow incidents. We build a small one.
   The parts that do exist are F2C's angle hook, Nav2 static or filter layers, and the GUI
   mask renderer.
2. The swath angle is the cheapest slope lever with the largest effect, and it fits the
   existing per-area settings.
3. Stuck spots: the market answer is a no-go zone that the owner confirms. Automate the
   proposal (incident cluster to hull) and keep the owner confirmation.
4. Transits should route around low-traction cells via a graded cost (not lethal), so they
   stay possible.

---

## 2. Architecture and ownership

```
 slip_detector --/dig_stall-------------.
 mission_fsm   --/mission/incident------+--> map_server_node: TerrainMemory (mower_map/terrain.py)
 map_server boundary check (internal)---'      per-area raster 0.1 m: slope (gx,gy,w), traction (bad), seen
 /imu/data_aligned + /odometry/filtered_map -->  incidents [{kind,x,y,t,w,status}] + clusters
                                                  |  persist terrain_<area>.npz + .json (maps dir)
          .---------------------------------------+------------------------------.
          v                                        v                              v
 ~/terrain_grid (traction 0..100)    ~/terrain_cost (0..60)            ~/terrain_summary (JSON)
 GUI heatmap + cluster markers       Nav2 global costmap               mission / GUI / planner
 -> ~/terrain_action (keepout|        terrain_layer (StaticLayer,      get_area_settings adds
    confirm|dismiss|clear)            non-trinary, use_maximum)        slope_mow_angle_deg
    keepout -> MapStore.add_obstacle  transits route around stuck       -> mission: plan angle
```

| Concern | Owner | Why |
|---|---|---|
| Terrain memory: raster, incidents, decay, persistence, keep-out creation | **mower_map** (map server) | It already owns the map grid, the masks, the per-area settings, obstacles and `promote_obstacle`. The raster aligns cell for cell with the masks. The memory stays per area, like the settings. Everything that turns into a map edit (keep-out) is here. |
| Incident logging (detours, follow/transit aborts, sub-path failures) | **mission** emits, map server files | Only the FSM knows why an action failed (static obstacle vs no progress). It emits `RecordIncident` effects (pure, testable), and the node publishes them. Boundary and dig incidents come straight to the map server from its own boundary classifier and from `/dig_stall`. |
| Swath angle from slope | **map server derives, mission applies, planner unchanged** (slice 1). Later: **planner** (F2C objective). | Slice 1 uses one angle per area, through the existing `mow_angle_deg` contract, with no C++ change. A per-cell slope objective (`SGObjective` subclass) and a per-region decomposition belong in `mower_coverage`, which needs the raster as input (section 4.2). |
| Turn placement (downhill turns, racetrack block order, Dubins loop side, no swath ends in low-traction cells) | **planner** (`mower_coverage`) | It is a property of the geometry the planner creates. It needs the slope raster passed with the plan goal (section 4.3). |
| Speed per segment, pivot-assist strength | **mission + control** | `desired_linear_vel` is already set by the mission per area. Per-segment speed belongs to a Nav2 SpeedFilter mask (generated by the map server) or to a speed attached to the path. Pivot assist lives in `cmd_vel_slew` and is tuned from the traction score at the robot (section 4.4). |
| Transit routing around stuck cells | **Nav2** (global costmap layer) | This is exactly what a costmap does. Coverage swaths are not re-routed by it: the controller follows the coverage path, and the local costmap does not see this layer. |

Why not put it all in Nav2: Nav2 cannot change the swath angle, the turn side or the route
order, and its costmaps do not persist knowledge across runs. Why not all in the mission: the
mission is a pure, very large FSM. Geometry and rasters there would bloat it and duplicate
the map grid.

---

## 3. Terrain memory: data model

### 3.1 Raster (`mower_map/terrain.py`, `TerrainRaster`)
- One raster per mowing area (navigation areas are excluded).
- Extent: the area's bounding box plus 1 m.
- Resolution 0.1 m. The origin is snapped to the resolution, so it aligns cell for cell with
  `grid_for_polygons` masks. When an area is redrawn, the overlap is re-gridded (`adopt`).

| Layer | Meaning | Decay |
|---|---|---|
| `gx`, `gy` | Sum of the terrain gradient dz/dx, dz/dy (map frame) from IMU tilt samples | slope half-life, 365 d |
| `gw` | Sample weight (count) | slope half-life |
| `bad` | Traction penalty: Gaussian splats (σ 0.25 m) of incident weights | traction half-life, 21 d |
| `seen` | Seconds driven over the cell (confidence; "never driven" vs "fine") | traction half-life |

Derived values:
- slope = `atan(|g| / w)`.
- traction score = `100·(1 − e^(−bad))`: one dig stall gives about 63, two give about 86.
- nav cost = 0 below score 20, then linear up to 60 at score 100.

**Slope estimate.** One tilt sample per 5 cm of travel, taken from `/imu/data_aligned`
roll/pitch and the EKF yaw (REP-103: positive pitch = nose down, positive roll = left side
up):
- dz/dx_body = −tan(pitch), dz/dy_body = tan(roll), rotated by yaw into the map frame.
- Dropped when |yaw rate| > 0.6 rad/s (centripetal acceleration tilts the attitude
  estimate), when |tilt| > 35°, when lifted, when charging, or when the IMU sample is older
  than 0.5 s.
- A constant IMU mounting bias rotates with heading. The racetrack and boustrophedon
  patterns drive each cell in opposite headings, so the bias averages out. This is tested:
  a 2° roll and −1.5° pitch bias recovers a 12° plane to within 0.6°. The
  `terrain_*_offset_deg` parameters remain as a manual trim.
- RTK altitude fusion (design, not in slice 1): along each straight segment, fit dz/ds from
  `/fix` altitude (RTK-fixed only, σ_z of about 2-3 cm) over at least 2 m windows. That gives
  the along-track gradient independently of IMU bias. Fuse it into `gx, gy` with weight
  `(σ_imu/σ_rtk)²`, and use the residual against the IMU to estimate the mounting bias
  online.

**Area slope axis** (`slope_axis`):
- Built from the structure tensor of per-cell gradients, so it has no sign: ridges and
  valleys do not cancel.
- Outputs: fall-line axis (deg, [0, 180)), RMS slope, p90 slope, and anisotropy (how
  directional the slope is).
- Needs at least 50 observed cells covering at least 25 % of the area.

### 3.2 Incidents

```json
{"id": 7, "t": 1791323518.0, "kind": "dig_stall", "x": -3.02, "y": 2.11,
 "w": 0.84, "w0": 1.0, "status": "open|confirmed|dismissed|keepout", "detail": "..."}
```

| Kind | Source | Traction weight |
|---|---|---|
| `dig_stall` | `/dig_stall` rising edge (slip_detector latch) | 1.0 |
| `subpath_failed` | mission: retries exhausted | 0.8 |
| `stall_guard` | docking stall/dig guard (design) | 0.8 |
| `follow_abort` | mission: FollowPath aborted, not a detour (typically "failed to make progress" in grass) | 0.6 |
| `slip` | offline bag import: short slip bursts (design) | 0.5 |
| `pivot_stall` | pivot assist engaged and yaw did not change (design) | 0.4 |
| `transit_abort` | mission: NavigateToPose aborted | 0.3 |
| `boundary`, `boundary_lethal` | map server boundary classifier, rising edge | 0, marker only |
| `detour` | mission: obstacle detour started | 0, marker only |

- Weights decay with the traction half-life.
- `open` and `dismissed` incidents below 0.05 are pruned. `confirmed` and `keepout`
  incidents are kept.
- Dismissing removes the incident's splat.

**Clusters**:
- Single linkage over open and confirmed incidents within 1 m.
- Cluster id = smallest incident id, which stays stable.
- Hull = convex hull of the points, each grown by 0.4 m (octagon buffer).
- Keep-out: `MapStore.add_obstacle(area, hull, "terrain #id (kinds)", SOURCE_DIG)`, then
  rebuild the masks and persist `areas.dat`. This is the MowgliNext "keepout proposal", made
  owner-confirmed.

### 3.3 Files (next to `areas.dat`, i.e. `/userdata/ros2/maps`)
- `terrain_<area>.npz`: `version`, `spec=[origin_x, origin_y, width, height, resolution,
  decayed_at]`, plus the layers `gx gy gw bad seen` (float32, rows = y). Compressed; about
  10-50 kB per area.
- `terrain_<area>.json`: `{version, area, polygon, next_id, incidents[], saved_at}`.
- `<area>` is the area name with `[^A-Za-z0-9._-]` replaced by `_`.
- Saved every 60 s when changed, and on area changes. A corrupt file is logged and the area
  starts empty.

### 3.4 Topics and services (all from `map_server_node`)

| Name | Type | QoS | Content |
|---|---|---|---|
| `~/terrain_grid` | nav_msgs/OccupancyGrid | latched | Traction score 0..100, −1 = no data. Same grid as the masks. Republished at most every 10 s while changing. |
| `~/terrain_cost` | nav_msgs/OccupancyGrid | latched | Nav cost 0..60 (`terrain_cost_max`) |
| `~/terrain_summary` | std_msgs/String | latched | JSON `{version, stamp, areas:[{area, area_index, slope:{axis_deg, slope_deg, p90_slope_deg, anisotropy, cells, coverage}, max_slope_deg, slope_mode, slope_mow_angle_deg, slope_angle_why, traction_cells_bad, traction_max, incidents, clusters:[{id, ids, kinds, n, weight, x, y, hull, status, last_t}]}]}` |
| `~/terrain_action` | mower_interfaces/SetAreaSettings | srv | `area_index` (255 = all), `settings_json` = `{"action":"keepout"\|"confirm"\|"dismiss","cluster_id":n}` or `{"action":"clear"}` |
| `/mission/incident` | std_msgs/String | in | JSON `{kind, x, y, detail, state}` (mission node, or any node) |
| `/dig_stall` | std_msgs/Bool | in, latched | slip_detector |
| `~/get_area_settings` | (existing) | srv | Now also returns the derived `slope_mow_angle_deg` (null = keep the planner's angle) and `slope_angle_why` |

The `SetAreaSettings` service type is reused so that no interface rebuild is needed.

---

## 4. Planner, mission and Nav2 inputs

### 4.1 Slope-aware mow angle (slice 1)
New per-area settings:
- `slope_mode`: `off` (default) | `auto` | `contour` | `updown`.
- `slope_contour_above_deg`: default 10, range 2..30.

The angle is applied only while `mow_angle_deg` is auto (−1):
- `updown`: swath heading = fall-line axis.
- `contour`: swath heading = axis + 90°.
- `auto`: contour when the area's RMS slope ≥ `slope_contour_above_deg`, else up/down.
- The planner keeps its own angle when the slope is under 3°, anisotropy is under 0.3 (no
  dominant direction, e.g. a dome), or there is too little data.

The mission logs the choice and its reason. `cross` and `alternate` modes rotate relative to
the chosen base angle.

### 4.2 Per-region angle and a slope objective (next, M)
1. Pass a downsampled slope raster to the coverage server: as a `PlanCoverage` goal field,
   or by having the server read the `.npz` directly (it already reads the maps dir).
2. Split areas with mixed slopes into sub-regions: threshold the slope magnitude, then
   morphological open/close, then polygons with at least 15 m² each. Plan each with its own
   angle.
3. Optionally add a `SlopeAlign` `SGObjective` that sums `|sin(swath − contour)|·slope` per
   swath sample, and keep BruteForce's angle search.

### 4.3 Turn placement (next, M-L)
- Turn cost for a turn at a swath end, from the raster at the turn footprint:
  `c = a·slope_up_component(turn heading) + b·(1 − traction) + c_pivot`.
  - A loop turn that starts heading uphill (a "climbing" turn) costs more than a descending
    one.
  - Pivots on slopes over 8° cost the most, because the caster digs in.
- **Racetrack block order**: for each pair of blocks, choose the visiting direction (which
  end comes first) so that the majority of turns of that pair happen while pointing
  downhill. That is a binary choice per block, evaluated with the cost above.
- **Dubins loop side**: F2C's Dubins can produce left or right loops. Generate both and keep
  the one with the lower slope-weighted cost (sample the curve every 0.1 m against the
  raster). With a turn radius of at least 0.5 m, the loop's apex is on the lower side.
- **Swath ends in low-traction cells**: shorten the swath end by up to 0.5 m when the end
  cell's traction score is over 50. The gap fill / coverage verification closes the strip
  later with a different approach direction.

### 4.4 Speed and pivot assist (next, S-M)
- **Speed**: the map server publishes a third grid, `~/terrain_speed`, as a Nav2 SpeedFilter
  mask in percent:
  - 100 on good cells.
  - 60 % where traction is over 50 or slope is over 12°.
  - 40 % above 18°.
  - Add `speed_filter` (type 1, percent) to the controller's global costmap filters and
    publish a second `CostmapFilterInfo` on `/speed_filter_info`.
  - Precondition: verify that the FTC controller implements `setSpeedLimit` (RPP does). If
    it does not, use per-pose speeds in the coverage path, which needs a FollowPath variant.
- **Pivot assist** (`cmd_vel_slew`): look up the traction score at the robot (subscribe to
  `~/terrain_grid` once per change) and scale `pivot_assist_linear_mps` (0.05 → up to 0.08)
  and `pivot_assist_delay_s` (1.5 → 0.8 s) on cells over 50. Also emit a `pivot_stall`
  incident when the pivot assist creeps its full `max_dist_m` with |Δyaw| < 10°.

### 4.5 Nav2 (slice 1)
- A `terrain_layer` StaticLayer is added to the global costmap on `~/terrain_cost`, with
  `trinary_costmap: false` and `use_maximum: true`. A value of 60 maps to cost 152: a
  preference, never lethal, so transits route around stuck spots when a reasonable
  alternative exists.
- The local costmap is untouched. Coverage swaths are not affected.

### 4.6 Offline population from mow bags (next, S)
`mower_map/terrain_import.py` (a CLI, run on the mower against `/userdata/ros2/bags/<mow>`):
- Replays `/odometry/filtered_map` and `/imu/data_aligned` through the same `terrain_sample`
  logic.
- Turns `/dig_stall` edges and `/mission/incident` into incidents.
- Derives `slip` bursts: |`/cmd_vel` − `/odom` twist| over 0.15 m/s for more than 0.5 s,
  RTK fixed only.
- Merges the result into the stored `.npz` (sums add) through the map server's
  `terrain_action` `{"action":"import","path":...}`, so that only one process writes the
  files.

The mow recorder now records `/dig_stall`, `/mission/incident` and `~/terrain_summary`.

---

## 5. Pieces, effort, risk

| Piece | Owner | Effort | Risk | Notes |
|---|---|---|---|---|
| Terrain raster, incidents, decay, persistence, topics | mower_map | S | Low | **done (slice 1)** |
| Mission incident hooks | mower_mission | S | Low | **done** |
| Keep-out from a cluster (owner-confirmed) | mower_map + GUI | S | Low | **done**. Reuses `add_obstacle`. |
| GUI heatmap, markers, actions, slope settings | GUI | S-M | Low | **done** (see section 6) |
| Nav2 traction cost layer | nav2_params | S | Med | **done**. Needs a Nav2 restart. Verify the planned transit around a fake incident. |
| Slope-aware area angle (`slope_mode`) | map server + mission | S | Med | **done**. Contour vs up/down is an owner rule and must be validated by cross-track RMS per mode. |
| IMU tilt quality (bias, dynamics) | mower_map | S | Med | Field check: drive one cell N/S/E/W and compare. |
| RTK altitude gradient fusion | mower_map | M | Med | Needs the RTK fix verified first (roadmap). |
| Offline bag import | mower_map | S | Low | |
| Per-region angle and SGObjective | mower_coverage (C++) | M | Med | |
| Downhill turn choice (racetrack order, Dubins side), swath-end shortening | mower_coverage | M-L | High | The turn geometry is untested on grass. Do this after the turn work in the roadmap. |
| SpeedFilter mask | map server + nav2 | S | Med | Check FTC `setSpeedLimit` first. |
| Pivot assist by traction, `pivot_stall` incident | mower_control | S | Low | |
| Tilt safety (blade off over X°) | mower_control / mission | S | High | **done 2026-10-09** (section 7), not yet run on the mower. |

---

## 6. First slice (implemented 2026-10-07, not committed)

- `src/mower_map/mower_map/terrain.py` (new): pure raster, incidents, clusters and hull,
  decay, slope axis and angle choice, npz/json persistence, grid stamping, traction cost.
- `src/mower_map/mower_map/map_server_node.py`:
  - `terrain_*` parameters.
  - IMU (sampled), `/dig_stall` and `/mission/incident` subscriptions.
  - Tilt sampling from `on_odom`.
  - Boundary rising-edge incidents.
  - `~/terrain_grid`, `~/terrain_cost`, `~/terrain_summary` and `~/terrain_action`.
  - Derived `slope_mow_angle_deg` in `get_area_settings`.
  - 60 s save timer and 10 s publish timer.
  - `terrain_enabled: false` disables all of it.
- `src/mower_map/mower_map/area_settings.py`: `slope_mode` and `slope_contour_above_deg`.
- `src/mower_mission`:
  - A `RecordIncident` effect, with `_incident` hooks on follow abort, transit abort,
    sub-path failed and detour start.
  - The node publishes it on `/mission/incident`.
  - `_merge_settings` applies `slope_mow_angle_deg`.
- `src/mower_navigation/config/nav2_params.yaml`: global costmap `terrain_layer`.
- `src/mower_control/mower_control/mow_recorder.py`: three topics added.
- GUI (`third_party/mowglinext/gui`):
  - The `terrainGrid` and `terrainSummary` topics.
  - `POST /mowglinext/terrain/action`.
  - The heatmap overlay through the mow-progress rasterizer.
  - Cluster markers with hulls, slope arrows, a Terrain card (Keep-out / Confirm / Dismiss /
    Clear) and the slope fields in the area settings panel.
- Tests:
  - `mower_map/test/test_terrain.py` (raster, splat, decay, prune, clusters, slope axis
    with bias cancellation, angle modes, persistence, re-grid, corrupt file, cost).
  - `mower_map/test/test_terrain_node.py` (node glue on stubs: incidents, dig edge, grids,
    keepout/dismiss/clear, tilt gating, derived angle, area resize).
  - `mower_mission/test/test_terrain_hooks.py` (incident hooks, angle use and precedence).

**Field validation plan:**
1. Mow once with `slope_mode: off` and read `~/terrain_summary`. Check that the slope axis
   matches the lawn (the vendor map, or a phone inclinometer).
2. Force a stall (hold the mower) and check for a `dig_stall` incident and the GUI marker.
3. Make a keep-out from it, check that the mask updates, then delete the obstacle.
4. Plan a transit across a fake high-cost cell and check the detour.
5. Set `slope_mode: auto` on the sloped area and compare cross-track RMS and completion
   against the previous mow.

---

## 7. Tilt guard (2026-10-09, not yet run on the mower)

Three bands from the IMU attitude, acted on by the monitor and the mission:

| Band | Roll / pitch (deg, filtered) | Action |
|---|---|---|
| caution | >= 15 / 18 | Mission caps `FollowCoveragePath` and `FollowPath` `desired_linear_vel` at `tilt_caution_speed_mps` (0.15). The blade stays on. The cap is restored at ok. |
| limit | >= 22 / 25 | In mission and docking phases: cancel, blade off, `steep` incident, then back out along the last 2 m driven, at 0.1 m/s on `/cmd_vel_emergency`. Pure pursuit runs forward or reverse, whichever side the track lies on. The back-out stops once the band drops, after 0.3-1.0 m. Then the mission resumes: mowing detours past the spot, transit and docking re-send the goal. In manual: blade off and refused. |
| critical | >= 30 / 32 | `tilt_monitor` sends zeros on `/cmd_vel_emergency` and calls `/cutter_off` itself. The mission enters EMERGENCY (`tilt critical (...)`), which clears to idle like lift. |

The roll thresholds cover side slope and rollover risk; the pitch thresholds cover up/down.
Critical stays below the cutter firmware's lift window (|roll| 38.5, pitch -36.8 / +40.1
deg; `docs/mcu_protocol_spec.md` 10c), so the host stops the mower before the MCU latches lift.
That latch takes about 17 s and needs a level IMU to clear.

**Filtering** (`mower_control/tilt_logic.py`):
- Input is the WIT orientation roll and pitch from `/imu/data`. Both are gravity referenced; yaw is not used.
- A low-pass with tau 0.25 s runs on the body-frame gravity vector, so it is wrap-safe at ±180°.
- Entering a band needs the filtered value over its threshold continuously for 1.0 / 0.6 / 0.3 s (caution / limit / critical).
- Leaving a band needs both axes below threshold − 3° for 2 s.
- A 0.2 s root bump to 35° does not trigger. A step to 33° roll reaches critical in about 0.9 s.

**Mounting:**
- `mount_roll_offset_deg` and `mount_pitch_offset_deg` are added like mcu_node's `forward_imu_*_offset_deg` and applied as a rotation.
- The vendor capture read roll ≈ −178° (board inverted relative to the vendor frame). Our `/imu/data` reads ≈ 0 upright (lift clear verified 2026-10-06), so the default is 0.
- A wrong offset reads ≈ 180°, which is critical. The guard fails safe and the node logs the hint.

**Repeated limits.** These go to `STUCK_NEEDS_HELP` with the sub-state `steep slope: needs help at (x, y)`; STOP, START or MANUAL clears it:
- a new limit within 0.75 m of a spot already backed out of;
- more than 3 back-outs in 5 min;
- still at limit after 1.0 m of back-out;
- no back-out progress for 3 s.

**Terrain memory.** `steep` is its own incident kind in `terrain.py` `KINDS`, with traction weight 1.0. It arrives on `/mission/incident` like `stuck`, so:
- `~/terrain_cost` steers Nav2 transits and detours around the spot;
- the GUI shows it as a cluster that the owner can turn into a keep-out.

Coverage swaths are not re-routed; the keep-out is the planner-side answer. The slope raster
is unchanged: it still drops samples over 35°.

**Topics:**
- `/tilt/status` (String JSON, latched, 2 Hz plus every band change) carries `band`, `level`, filtered and raw `roll_deg` / `pitch_deg`, `axis`, `stale`, `imu_age_s`, `since_s` and `thresholds`.
- The mission reads it, with a 2 s staleness limit. An unknown band does nothing.
- `gui_bridge` shows it as the diagnostics entry `tron: Tilt` (WARN for caution and limit, ERROR for critical).

**Field checks still open:**
1. Confirm `/tilt/status` reads about 0 / 0 on level ground, and has the right signs when the left side or the nose is lifted.
2. Tune the thresholds on the real lawn (the Tron's rated slope is unknown).
3. Check the RPP live parameter update of `desired_linear_vel` mid-FollowPath.
4. Check the back-out on grass.
5. Check that `/cutter_off` is reachable from tilt_monitor.
