# Vendor (original ROS 1) obstacle / no-go handling — review, 2026-10-09

INTERNAL. This reviews the original Airseekers ROS 1 stack's obstacle and
no-go-zone machinery and says what (if anything) is worth taking into the ROS 2
port. It is **not** part of the open-source handoff (`docs/mowglinext_handoff.md`)
and must not be folded into it: it names vendor binaries and internals.

Read-only. Nothing in `ros2_port_handoff/` was modified. Sources:

* `ros2_port_handoff/04_full_src_tree/src/mower_msgs/msg/` — `Map.msg`,
  `MapPolygon.msg`, `MapObstacle.msg`, `MapPose.msg`, `MapChannel.msg`,
  `MapSpecialZone.msg`, `ObstacleInfo.msg`, `ObstacleInfos.msg`, `RealObsInfo.msg`
* `.../mower_msgs/srv/` — `MapLayerObstacleInfos.srv`, `SetObstacleFiltering.srv`,
  `MapSet.srv`, `SaveGrid.srv`, `RecordBreakPoint.srv`, `PolygonService.srv`
* `.../mower_planner/mower_planning/src/map_layer/GlobalMap.h` (header only)
* `.../mower_task/mower_map/README.md` + `maps/*.geojson`
* `.../mower_proto/msg.proto`, `task.proto`, `map.proto`
* `.../mower_perception/seg_ros/src/ppseg.cc`, `seg_ros/readme.md`
* `ros2_port_handoff/10_ros_live_snapshot/{topics_verbose,services,nodes}.txt`
* `mower_docs/03-yolo-perception.md`, `05-maps-and-http-api.md`, `06-mqtt-and-ros-services.md`

Caveat up front: most of the runtime is **closed**. The `mower_map` and
`mower_logic` nodes (which own the HTTP map API and the layer publishers) have no
sources in the handoff, and `mower_planning` ships headers with no `.cpp`. So the
harvest is *interfaces and semantics*, not code to copy.

---

## 1. The vendor data model

Obstacles are first-class map objects, and they are **children of a zone** — the
same parenting the ROS 2 port already uses:

* `Map.msg` = `MapPolygon[] polygons` + `MapChannel[] channels` + `layers_dir` +
  `map_id`/`map_md5`.
* `MapPolygon.msg` = ring (`MapPose[]`) + `type` (1 work, 3 channel, 4 dock zone)
  + nested `MapObstacle[] obstacles` + `MapSpecialZone[] special_zones`.
* `MapObstacle.msg` = `uuid` + `MapPose[] poses` + `lable` (an `ObstacleInfo`
  class) + `source`:
  * `SOURCE_USER_MANUAL = 1` — drawn by the operator / imported
  * `SOURCE_USER_VERIFY = 2` — detected during a task, then confirmed by the user
  * `SOURCE_TEMP = 3` — auto-detected during the *current* task
  * `SOURCE_EXPLORE_MAP = 4` — auto-detected during exploration mapping
* `MapPose.msg` = pose **plus `rtk_solution`**; the comment says these poses are
  recorded *while driving in remote-control mode*. So the vendor stamped every
  recorded vertex with its RTK fix quality.
* `MapSpecialZone.msg` = a second, non-lethal zone type (only `trampoline` is
  defined): "do not mow" but not an obstacle.
* `ObstacleInfo.msg` = label + pose + `PolygonStamped[]` contours + a camera
  `Image`. Label ranges: dynamic 0–99 (person/dog/cat/…), static 100–199
  (stone/trashbin/brick/…), special 200–299 (small_stone/low_manhole),
  "no target" 300–399 (seg-only / line-only), `BUMP_TRIGER = 500`,
  `SKIDDING_TRIGER = 501` (not recorded), `UNKNOWN = 999`, and
  `SPACE = 1000` — "empty, used to clear obstacles".
* `RealObsInfo.msg` = `PoseArray[] obs_contours` — dynamic obstacle boundary.

GeoJSON on disk (`mower_map/maps/*.geojson`, WGS84): feature properties give
`area_id`/`obstacle_id`/`channel_id`; obstacle zones are **type 4** with a
`parent area_id`, exactly what `src/mower_map/mower_map/import_vendor_geojson.py`
already handles (including the drop-if-outside fallback and the navigation-area
rejection).

## 2. Recording a no-go zone by driving

The vendor already had the feature the port is adding as `CMD_RECORD_OBSTACLE`:

```
GET /maping/area/start?type=3&name=...   # 1 work area, 2 channel, 3 no-go zone
GET /maping/area/end
GET /maping/create/save
```

i.e. the operator drives the no-go ring with the joystick during mapping, poses
(and their RTK solution) are stamped, and the zone is saved with the work area.
This validates the design and is the strongest "we are rebuilding the right
thing" signal in the review.

## 3. Runtime pipeline and layers

* Perception: `det_ros` (YOLOv8 on the NPU, 22 classes) and `seg_ros`
  (segmentation; class 4 = "Obstacle / absolute no-go", class 5 = "Shrub / soft
  obstacle") publish `mower_msgs/ObstacleInfos` on `/controller/obstacleCloud`
  (1 publisher, 2 subscribers). `ObstacleInfo` carries the contour **and a
  camera image**. `SetObstacleFiltering.srv` tunes filtering.
* Layers are live `sensor_msgs/Image` topics — `/map/layer/forbid`,
  `/map/layer/static_obstacle`, `/map/layer/bumper`, `.../cutting`,
  `.../supplement`, `.../cover`, `.../mowing_region`, `.../channel`,
  `.../region` — and are cached as PNGs on disk (`layers/*.png`): `forbid`,
  `static`, `real_obstacle`, `dynamic_obstacle`, `inflate`, `outside`,
  `special_zone`, `blind_zone`, `along_border_cutting`, `stuck`, …
  Services `/map/layer/cutter` and `/map/layer/reset` manipulate them.
* Planner (`GlobalMap.h`, header only): obstacle polygons become coverage
  **holes** and change the boundary-follow order (`forbidden_contours`,
  `Break_FORBIDDEN_AREA`); API includes `fillUserVerifyObstacleToMask`,
  `setStaticObstacleLayer`, `AddToLayer`, `CleanLayer`,
  `AddBlindZone`/`AddTroubleZone`, `obsCheckCover`; publishers for obstacle,
  filtered obstacle, obstacle-boundary and
  "obstacle dynamic clear boundary" and an **obstacle sup-cut**
  (`/planing/map_obstacle_sup_cut`) — re-mow the ring around an obstacle.
* Auto no-go after a task: `task.proto` `TaskReport` carries
  `fake_obs_area_size` ("auto no-go count") and `has_fake_obs_area_pictures` /
  `pictures_file` — detected zones are surfaced with photos for the user to
  confirm (SOURCE_USER_VERIFY).
* Cloud/app: MQTT `MsgType` 170/171 `Obstacle_Dynamic_Clear`,
  172/173 `Obstacle_Dynamic_Boundary` — clear/set dynamic obstacles remotely.
* Live services observed: `/map/layer/cutter`, `/map/layer/reset`,
  `/seg/set_obstacle_filtering`, `/seg_oacam/set_obstacle_filtering`.

## 4. What the ROS 2 port already has

* `src/mower_map/mower_map/areas.py` — per-area obstacles with provenance
  (`SOURCE_USER/TRACKER/DIG`), session-scoped `pending` proposals,
  promote/discard services, centroid dedup, keepout + nav masks with obstacle
  margins; `import_vendor_geojson.py` imports vendor type-4 zones.
* `~/promote_obstacle` / `~/discard_obstacle` — the SOURCE_USER_VERIFY flow,
  already used by the wheel-slip dig detector (pending proposal → operator
  accepts).
* `mower_vision` obstacle guard: dynamic classes → stop/wait, static classes →
  detour (`/obstacle_policy`).
* GUI can draw keepouts and the map server persists them.
* `CMD_RECORD_OBSTACLE = 11` (this change set): drive-record a no-go ring, attach
  it to the mowing area containing its centroid (else the first mowing area),
  save via `promote_obstacle`, keep the ring in a fallback file on failure.

## 5. Worth adopting (prioritised)

1. **Stamp RTK quality on recorded vertices.** The vendor's `MapPose.rtk_solution`
   is one field and one line in the recorder, and it lets the recorder (or the
   GUI) warn/refuse a no-go recorded on a float fix. `mission_fsm.Inputs` already
   carries `fix_type`/`gps_quality`. Small, high value, directly applicable to
   the new `CMD_RECORD_OBSTACLE`.
2. **Auto no-go proposals with a photo.** Vendor: detect during the task,
   accumulate `fake_obs_area`, report with pictures, user confirms. The port has
   the seed (dig `pending` + `promote_obstacle`); the missing half is attaching a
   camera snapshot as evidence and surfacing a per-task list. Fits the roadmap's
   "event snapshots" idea.
3. **A clear-obstacle action for dynamic/tracker obstacles** (vendor MQTT
   170/171): clear session/pending obstacles without editing the map. The port
   has `discard_obstacle{id}`; a single "clear dynamic" command would match the
   vendor UX (and the `SPACE = 1000` clear semantics).
4. **Obstacle sup-cut / re-mow.** After a keepout is discarded or removed, re-mow
   the ring around it. Not present today (roadmap gap-filling is adjacent).
5. **Publish mask layers as debug images.** The vendor publishes every layer on
   `/map/layer/*`; publishing the port's keepout/nav masks would give the GUI and
   teleop the same observability cheaply.
6. **A non-lethal "no-mow zone" type** (vendor `MapSpecialZone`, e.g. trampoline)
   distinct from an obstacle: don't cut it, but allow driving across. Small model
   addition; useful for gravel strips / patio edges.
7. **Store a class label on a promoted obstacle**, not just free text — the
   vendor kept the detector class (`ObstacleInfo` ranges). The port's
   `MapObstacleInfo.name` carries a free-text/evidence line; a class field would
   let the GUI filter and the planner treat soft vs hard obstacles differently.
8. **Blind / stuck zones** (`AddBlindZone`/`AddTroubleZone`) as auto-keepouts —
   the `stuck` guard + dig detector are the natural source, mirroring the
   pending-proposal pattern.

Not worth porting: the vendor `uuid`-based obstacle storage (the port's
area-parented YAML is simpler and already validated), the layer-PNG cache, the
planner hole math (Fields2Cover covers it), and `mower_proto`/MQTT plumbing
(vendor-cloud only).

## 6. One concrete follow-up for the in-flight change

The only item that touches `CMD_RECORD_OBSTACLE` directly is (1). The FSM
recording loop is the place to add it: on each `_sample`, read
`inputs.fix_type` (already available) and either drop samples below
`rtk_fix_type` or record a per-vertex quality flag, then reject/warn at
`_record_finish` if too few good vertices. Everything else is a new feature
rather than a fix.