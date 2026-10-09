# Grass segmentation: avoid flat non-grass (flower beds, gravel, paving, water)

Date: 2026-10-09. Local edits and unit tests only; nothing below has run on the mower.

## Problem

Stereo (`/stereo_depth/points`) only sees things that stand up from the ground. Flower-bed
soil, gravel, paving, a pond edge and flat toys are at ground height, so the stereo ground
filter removes them and the planner happily routes transits and obstacle detours across them.
`seg_ros` (PP-LiteSeg, 6 classes) already existed but was off: nothing consumed it.

## seg_ros audit

| Item | Finding |
|---|---|
| Model | `pplite-seg_20260630-1-6cls.rknn`, int8, input 640x480 NHWC, one NCHW output `[1,6,480,640]` (dequantised logits) |
| Classes | 0 other/background, 1 grass, 2 passway (paving), 3 soil, 4 obstacle, 5 shrub (flower beds / hedges). No explicit water / gravel class: they land in "other" (unverified) |
| Old outputs | `/ai/seg/mask` (mono8 class index), `/ai/seg/traversability` (255 = grass/soil), `/ai/seg/image_overlay` (bgr8). No confidence |
| Camera (before) | Both OA cameras (`/left_oa_camera`, `/right_oa_camera`, GC2093 1920x1080). They have NO extrinsics in `config/urdf/mower.urdf.xacro` (box poses only), so a mask from them cannot be put on the ground. Each frame also meant three 1080p masks resized and published by Python |
| Camera (now) | `/vio/right/image_color` (front stereo right colour eye, 640x480 bgr8, frame `vio_camera`, `stereo_cam publish_color` ~5 Hz, on demand). Intrinsics `config/cameras/right_stereo_camera_info.yaml` (cam1) and extrinsics to the depth reference eye (`stereo_params.yaml`, the same values `det_range` uses). Model input = image size: no resize |
| Rate | `max_rate_hz` 5 -> 2 per camera. The memory needs several frames per cell from different viewpoints, not video rate; at 0.3 m/s the 0.4-2 m look-ahead band is seen ~13 times |
| NPU | Core 1 (`core_mask "1"`). det_ros uses per-camera cores `["0","1","2"]` (left OA, right OA, front eye), so seg shares core 1 with the right OA YOLO (~86 ms x 5 Hz = ~43 % of the core). Seg at 2 Hz adds an estimated 30-60 ms per frame (not measured). The RKNN runtime time-slices contexts on a core; det.yaml's alternative `["0","2","2"]` was NOT applied (it would put two YOLO streams on core 2). Measure `rknn` latency of both nodes live before changing |
| CPU | argmax + softmax over 6x480x640 float32 in numpy, the overlay, three mono8 + one bgr8 publish per frame: estimated 30-50 ms of one A76 core per frame at 2 Hz (measure with `top -H`). The projector samples 80x60 pixels per frame: negligible |

`seg_ros` changes: `extra_topics` (like det_ros), `seg.yaml` points at the front eye (the OA
topics are blank; vendor values in comments), new `/ai/seg/confidence` (mono8, top-1 softmax
probability x 255, same stamp / frame / size as the mask, published just before it).

## Design

```
seg_ros (/vio/right/image_color, 2 Hz, NPU core 1)
  /ai/seg/mask + /ai/seg/confidence (mono8, same stamp)
        |
seg_ros/nongrass_projector  (seg_ros/ground_projection.py, pure)
  + /odometry/filtered_map (pose at the stamp), /tf_static (stereo_camera_optical)
  + /stereo_depth/ground_plane (live plane), /stereo_depth/depth/image_raw (depth check)
  -> /ai/seg/ground_cells  PointCloud2 (map): one vote per 0.1 m cell per frame
        |
mower_map map_server_node  (mower_map/nongrass.py pure, nongrass_node.py glue)
  per mowing area: evidence raster (hit/miss/viewpoints), decay, confirmation, persistence
  -> ~/nongrass_cost     -> Nav2 GLOBAL costmap nongrass_layer (StaticLayer, soft)
  -> ~/nongrass_grid     -> GUI heat map (to do)
  -> ~/nongrass_summary  -> GUI keep-out suggestions (to do); ~/terrain_action acts on them
```

Why a memory in `mower_map` and not a costmap plugin or an obstacle-layer source: an
`ObstacleLayer` source marks LETHAL and forgets on the next raytrace; this needs soft cost,
memory across runs, per-area switches and owner-confirmed keep-outs, which `mower_map`
already does for the terrain memory (`docs/terrain_aware_planning.md`: same grid, same
persistence directory, same StaticLayer pattern, same `~/terrain_action` service).

### Projection (`seg_ros/ground_projection.py`)

- 80 x 60 sample pixels (stride 8) are undistorted once (plumb_bob) into rays.
- Camera pose: TF `base_link <- stereo_camera_optical` (URDF, measured 2026-10-06: 0.240 m
  high, 0.92 deg pitch up) composed with the cam1 -> cam0 extrinsic. Robot pose: nearest
  `/odometry/filtered_map` within 0.1 s of the mask stamp (no Python TF listener: CPU).
- Ground: the live `/stereo_depth/ground_plane` fit if fresh (< 1 s) and within 6 cm / 6 deg
  of the URDF flat ground (`z = 0` in base_link), else the flat ground. A hedge or wall that
  wins the RANSAC must not tilt the projection.
- Depth check: each ground point is projected into the hardware depth image (cam0, rectified,
  approximated by cam0 like det_range). Measured depth shorter than the ground point by more
  than max(8 cm, 10 %) means something stands there: the sample is dropped. Raised objects are
  stereo's and det_range's job (lethal, local costmap), and flat projection would place them
  far behind their real position.
- Labels: a non-grass class (0, 2, 4, 5) with confidence >= 0.8 is a hit; grass or SOIL
  (1, 3) with confidence >= 0.6 is a miss; anything else is ignored. Soil counts as grass:
  worn and dry patches are lawn and must never be avoided.
- Range 0.35-2.0 m from the camera (horizontal), |y| <= 1.5 m. Near: the nose hides the
  ground below ~0.4 m. Far: at 0.24 m height a 1 deg pitch error moves a 2 m point by ~0.3 m.
- One vote per 0.1 m map cell per frame (non-grass only if hits outnumber misses in the cell).
- Motion gate: a frame is used only after the robot moved 0.15 m or turned 10 deg since the
  last used frame. A parked robot sends nothing.

### Memory and confirmation (`mower_map/nongrass.py`)

Per mowing area, a raster on the mask grid (area bbox + 0.5 m), layers `hit`, `miss` (both
decayed, half-life 14 days), `vx0/vy0` (first viewpoint that saw non-grass), `spread` (how far
later non-grass viewpoints were from it) and `confirmed`.

A cell becomes non-grass only when all hold:

| Gate | Default | Rejects |
|---|---|---|
| `hit >= confirm_hits` | 3 frames | one-off misclassification |
| `hit / (hit + miss) >= confirm_ratio` | 0.75 | shadows that come and go, cells the model flips on |
| `spread >= confirm_spread_m` | 0.5 m | one viewpoint: parked robot, glare at one angle; the robot's own shadow moves with it so it never stays on one cell |
| confidence (projector) | 0.8 | pixels the model is unsure of (shadows, dry grass) |

Release (hysteresis): `hit < 1.5` or ratio < 0.5. A grass vote counts as much as a non-grass
vote, so a toy that was picked up is cleared the next time the camera sees the lawn there.

Cost: confirmed cells inside the area polygon get `nongrass_cost_max` 75, plus a 0.2 m linear
halo (the planner plans a point; the 0.5 m body would brush the bed). Through the non-trinary
StaticLayer 75 -> 190: above the nav mask's in-area transit cost (60 -> 151, when a drawn path
exists; with `use_maximum` a lower value would vanish under it), below the soft band
(90 -> 229) and the inscribed cost 253. Never lethal, never inflated, so a mowing area can
never become unplannable through this layer.

Per-area switch: area setting `avoid_non_grass` (default true). Off gates only the cost; the
memory keeps learning. Use it for a lawn with a gravel path the robot should cross.

Persistence: `nongrass_<area>.npz` next to `areas.dat` (saved every 120 s when changed).
Inputs are ignored while charging or lifted.

### Nav2

`src/mower_navigation/config/nav2_params.yaml`, global costmap only:
`plugins: [static_layer, terrain_layer, nongrass_layer, obstacle_layer, inflation_layer]`,
`nongrass_layer` = StaticLayer on `/map_server_node/nongrass_cost`, transient local,
`trinary_costmap: false`, `use_maximum: true`. map_server_node always publishes this grid
(zeros when the memory is empty or off), so the layer always has a map.

## Mission and coverage interplay

- Coverage keeps mowing the lawn. Swaths come from the coverage planner on the area polygon
  minus obstacles; the controller follows them and only the LOCAL costmap stops it. This
  layer is in the global costmap only, so it never removes or blocks a swath.
- Transits, obstacle detours (NavigateToPose, 1 Hz replanning) and recovery routes use the
  global costmap: they now prefer lawn over a confirmed bed, path or gravel patch, but still
  cross it when there is no reasonable alternative.
- Non-grass INSIDE a mowing area is therefore mowed over by the swaths until the owner turns
  it into a keep-out. That is deliberate: an automatic keep-out from a camera guess would
  silently leave lawn uncut. The keep-out path:
  1. `~/nongrass_summary` lists per area the confirmed clusters (8-connected, with
     `area_m2`, centroid, a hull grown by 0.1 m and `suggest_keepout` for >= 0.25 m2).
  2. The owner accepts with `~/terrain_action` `{"action": "nongrass_keepout",
     "cluster_id": n}` (area_index = the area): the hull becomes a map obstacle
     (`MapStore.add_obstacle`), the masks are rebuilt and the next coverage plan goes around
     it. `{"action": "nongrass_dismiss", "cluster_id": n}` says "this is lawn": its evidence
     is wiped and 20 frames of grass evidence added. `{"action": "nongrass_clear"}` wipes
     the area's (or all areas', area_index 255) memory.
- GUI (not implemented; small follow-up in `third_party/mowglinext/gui`): draw
  `/map_server_node/nongrass_grid` as a heat map and the `suggest_keepout` hulls as dashed
  polygons on the map page with "Make no-go" / "It's lawn" buttons calling the actions above;
  add `avoid_non_grass` to the area settings form (`AreaSettingsForm.tsx`, `areaSettings.ts`).
  Until then the actions can be called with `ros2 service call
  /map_server_node/terrain_action mower_interfaces/srv/SetAreaSettings ...`.

## Enable

`mower.launch.py nongrass:=true` (default false) -> `perception.launch.py nongrass:=true`
starts `seg_ros` (also when `seg:=false`) and `nongrass_projector`. The map server side is
always on and idle without input. Debug: `/ai/seg/image_overlay`, `/ai/seg/ground_cells`
(foxglove, map frame, colour by `nongrass`), `/nongrass_projector/stats` (JSON every 10 s:
frames, skips by reason, plane live/flat/rejected, elevated samples),
`/map_server_node/nongrass_summary`.

## Live checks needed

1. Class quality on the front eye (`/ai/seg/image_overlay`): the model was trained on the OA
   cameras. Drive the lawn edge, a bed, a paved path; check grass vs shadow vs dry patch.
   If the front eye is unusable, the OA cameras need a calibrated extrinsic first.
2. NPU / CPU: seg_ros inference time and det_ros right-OA rate with seg on core 1;
   `top -H` for seg_ros and nongrass_projector.
3. Projection accuracy: put a sheet / board on the lawn 1 m ahead, compare
   `/ai/seg/ground_cells` with its measured position (expect < 0.1 m at 1 m).
4. Confirmation: a mowing run past a bed confirms it (`nongrass_summary` clusters); a run
   on open lawn on a sunny afternoon confirms nothing (shadows). Tune `min_conf_nongrass`
   and `nongrass_confirm_*` from those two bags.
5. Planner: a transit next to a confirmed bed bends around it; with the bed between the
   robot and the only gap, the plan still crosses it.
6. planner_server CPU with the extra StaticLayer at 4 Hz (it re-applies its full extent each
   update, like terrain_layer).
