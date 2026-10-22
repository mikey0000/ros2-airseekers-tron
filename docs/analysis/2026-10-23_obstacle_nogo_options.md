# Auto no-go zones for detected obstacles: what exists, options (2026-10-23)

Owner request after the trunk incident (sessions 20:15–20:39 UTC on the 22nd, area index 5
"Area 7"): mark no-go zones for obvious obstacles once the score is high and the object is in
the way, sized to the object. Read-only review. The incident diagnosis is in
`docs/wheel_smoothness_field_tests.md` ("Field mow 2026-10-23 morning", cfe2faf).

## 1. What we have now

**Detection → policy.** det_ros (22 classes incl. trunk, stone, wood) → det_range ranges each
box with stereo depth (central 50 % of the box) and puts **one point at the box centre** on
`/ai/det/obstacle_points` for named classes at score ≥ 0.5. obstacle_guard publishes
`/obstacle_policy`: living classes → `dynamic`; other classes ranged within 1.5 m → `static`;
unranged → advisory. The mission acts only if the ranged point is in the swept corridor ahead
(`_obstacle_on_path`, half width 0.235 + 0.25 m, 2 m lookahead; the B6 on-path rule exists).

**Reaction.** On rows, a static on-path policy or a costmap "collision ahead" abort →
`_try_detour`: a `/navigate_to_pose` jump to the swath pose 2 m ahead (max 5 per sub-path), the
stretch re-tried at the area end. **Edge laps have no go-around:** an abort → `edge_blocked_skip`
2 m; `bumper_skip_hits` 3 within 1 m. MPPI sees 0.6 m ahead with PathAlign 30 against cost 3.81,
so it cannot deviate either. `ws/edge-layout3` keeps the blade on through these inside the area.

**Memory.** (a) Costmaps: stereo, det_range and bumper marks in `DecayingObstacleLayer`, 5 s
decay; a mark that leaves the camera view is gone. (b) `obstacle_tracker`: clusters the stereo +
det clouds, **no class, score or bbox**; size = cluster spread (min 0.15 m, +0.05 m); promotes
only after 120 s of sightings, ≥ 10 updates, still in view, and `auto_promote` is off. A
promotion is a PENDING keepout (live in `/nav_keepout_mask`, not saved), Promote/Ignore in the
GUI panel with a snapshot JPEG. (c) `areas.dat` obstacles (user, tracker, dig) are coverage
holes (grown 0.30 m, ringed) and lethal in the keepout filter; the route graph keeps 0.35 m.

**Why the trunk was not avoided (design level).**
1. The trunk at (-6.15, -0.57) is 0.36 m inside the west line, beside the lap-0 corner. Lap 0
   runs on the line with half width 0.235, so the plan itself clips it. Nothing in the pipeline
   can move lap 0: only a hole in the coverage input would.
2. 888 ranged trunk/wood detections landed in one 0.1 m cell, so the evidence for a zone was
   there. But the costmap got a single centroid disc (50–70 cells) only from 1–3 m away; within
   0.9 m the trunk was beside the camera and the mark decayed. Stereo never marked it. A
   one-cell mark 0.5 m from the line passes the footprint check, so the body scraped the trunk.
3. The tracker needs 120 s in view; a drive-by never promotes, and nothing survived the mow
   (`obstacle_snapshots/` is empty). The ranged-pose stream carries class and score, but the
   tracker ignores both.
4. Coverage holes are a snapshot at plan time; a new keepout changes nothing until the next
   START. Repeated bumper hits skip the stretch but leave no mark.
5. Side note: Area 5 has `obstacle_detection: none`, which silences vision policy and det
   points there entirely.

**Stock** keeps labelled YOLO + bumper obstacles per task (`MapObstacle.SOURCE_TEMP`,
`SOURCE_USER_VERIFY` = detected in the task, confirmed by the user; `LayerStaticObstacle`,
`LayerBumper`, `AddBumperObstacleCloud(same_place_count)`): back up, plan around, rejoin, and
the user verifies zones after the mow. It does not persist without confirmation.

## 2. Options

### A. Sized, class-voted zones in the tracker
- **Trigger:** ranged `static` box, score ≥ 0.6, whitelist class (trunk, stone, wood, brick,
  trashbin, potted plant, chair; never living classes), ≥ 5 frames within 10 s landing within
  0.3 m, and on-path (corridor test, the pivot footprint of the next turn, or a bumper hit
  within 0.5 m). Feed class, score and bbox width to the tracker (it already subscribes
  `/ai/det/detections_ranged`); a class-voted track skips the 120 s gate.
- **Size:** radius = max(bbox width × range / f, class prior) / 2 + 0.10 m, averaged over the
  observations (the std over 888 frames gives the uncertainty; the radius shrinks toward the
  mean as frames accumulate); bumper-only: a 0.25 m disc at the nose.
- **Lifecycle:** PENDING keepout for this mow (existing path), GUI accept/discard, auto-remove
  when seen gone (`gone_confirmations`), 0.10 m centroid dedup merges duplicates; a moved
  object is removed and re-proposed.
- **Use:** the keepout mask makes row detours and the route graph go round it at once; lap 0
  and rows change only with C. Blade per edge-layout3.
- **Effort** 2–3 days. **Risk** low (pending only). **Validation:** sim obstacle plus a
  scripted ranged detection (the sim has no YOLO), then a field pass at this trunk.

### B. Width and persistence in the costmap (no memory)
- det_range: mark a disc of the bbox width at the range instead of one point. Costmap: a
  second `DecayingObstacleLayer` for static-class det points with a 60 s decay (or none until
  stereo clear points cross it), so the trunk stays lethal while beside the camera.
- Effort 1 day. Risk: a wrong range leaves a 60 s false wall near the line; CPU unchanged.
  Alone it still gives edge laps no go-around (the skip stays), so B is the floor, not the fix.

### C. Edge-lap and row routing around a confirmed zone
- On a promotion inside the current area (pending or accepted), re-plan the remaining
  sub-paths from the resume cursor with the hole (0.30 m grown): rows route around it and lap 0
  is pulled inward locally (the hole's own ring joins the lap; soft edge rules unchanged). The
  trunk's stretch of the line is then left uncut deliberately instead of skipped by 2 m.
- Effort 3–4 days (cursor and skip offsets must survive the re-plan; full-mow sim gate on the
  real map). Risk: a few seconds of CPU per re-plan; a false promotion reshapes the mow.
- **Turns (owner, 2026-10-23: "identify things like a trunk during turns and change our
  routing").** Before every pivot (coverage corner, K-turn, route corner Spin) the mission asks
  `mower_map.nogo_zones.pivot_sweep_hits(x, y, yaw_from, yaw_to, direction, zones)` against the
  latest `/obstacle_tracker/nogo_zones` (`ZoneSet.from_json(...).discs()`: pending / accepted /
  candidate zones and the static det marks of the last 60 s). When the sweep in the planned
  direction intersects, in this order: (1) turn the other way if `pivot_clear_direction` says
  that sweep is clear; (2) a K-turn or a 0.3 m back-up then pivot (re-check the sweep at the
  backed-up pose); (3) move the pivot earlier or later along the path (slide the vertex back
  along the incoming leg, or pivot after advancing on the outgoing leg, re-checking each
  candidate within the area / body-boundary rules); (4) re-plan the remaining sub-paths around
  the zone (the C re-plan above). The route-corner "Collision Ahead" (incident 20:30) would then
  back off or skip instead of ending in STUCK_NEEDS_HELP. Needs mower_mission to depend on
  mower_map (it does not today) and a unit test per fallback; mission_fsm is not touched by A.

### D. Persistent auto zones without approval (stock-like, opt-in)
- As A, but a class-voted obstacle seen in ≥ 2 mows (or ≥ 3 bumper hits at one spot) is saved
  as `SOURCE_TRACKER` with the class in its name, expires after 3 mows unseen, Reject button
  in the GUI. Cap 10 per area, radius ≤ 1 m.
- Effort 1–2 days on A. Risk: permanent holes from false positives or a 0.5 m position error.

## 3. Recommendation and owner decisions

B + A first (about a week with the sim scenario), C next (it is the part that would have
saved lap 0 here), D only after two clean field mows. This matches stock (per-task zones,
user verification) and the B6 rule.

1. **Thresholds:** score ≥ 0.6, 5 frames in 10 s within 0.3 m for class-voted promotion?
2. **Whitelist:** trunk, stone, wood, brick, trashbin, potted plant, chair (sports ball and
   bottle are moveable; never person or animals).
3. **On-path rule:** corridor only, or also the next pivot footprint and a bumper hit?
4. **Size margin:** 0.10 m on the measured half width (the planner adds 0.30 m).
5. **Persistence:** pending per mow with GUI accept (A), or auto after two mows (D)?
6. **Re-plan mid-mow (C)** with lap 0 pulled inward, or costmap routing only until next START?
7. **GUI:** class and score in the Tracked Obstacles panel, pending zones on the map, a
   per-area auto no-go toggle; and whether Area 5 should go back to `standard`.

## 4. Implemented (B, A), branch `ws/nogo` (2026-10-23)

Owner order: B and A now, C next (separate job), D only after clean field mows with A. The
defaults below are the doc's recommendations; **owner to review** (all are parameters).

**B: width and persistence** (`det_range`, `nav2_params.yaml`, `fixed_frame_relay`).
- det_range: a stereo-ranged box is a disc of points instead of one centroid: width = box width
  x range / fx (the box edges projected at the range), centre half a width behind the measured
  front face, radius width/2 + 0.10 m, 0.05 m grid, width capped at 1.5 m
  (`geometry.object_disc`). The width also goes into the ranged detection's
  `results[0].pose.covariance[35]` (m) for the tracker.
- Static classes (`static_classes`: trunk, stone, wood, brick, trashbin, potted plant, chair) go
  to `/ai/det/obstacle_points_static` (relayed to `_odom` / `_map`), read by a new
  `static_det_layer` (DecayingObstacleLayer, **60 s** decay, marking only, no clearing source)
  in both costmaps. Every other class stays on the 5 s `obstacle_layer`. `stereo_costmap:=false`
  strips it. The 60 s also covers a trunk the camera loses mid-pivot.

**A: class-voted, sized, pending zones** (`obstacle_tracker_core` / `_node`, `nogo_zones.py`,
`map_server_node`, `areas.py`).
- Trigger: >= 5 frames of score >= 0.6 within 10 s landing within 0.3 m; the class vote (summed
  scores, >= 0.3 votes) on the static whitelist, never person / dog / cat / hedgehog / rabbit
  (a living-class detection at the spot vetoes it, and the existing 1 m dynamic veto applies);
  mowing only; not in an area whose `obstacle_detection` is `none`.
- On path (owner rule B6): the body would touch the zone along the coverage plan from 2 m
  behind the robot on (`/coverage/full_plan` with a forward-only cursor) or the current route
  (`/plan`), as the footprint corridor, or the **pivot sweep** at a corner >= 30 deg (the 0.60 m
  nose corners reach 0.645 m), or a bumper hit within 0.5 m (`/bumper_cloud_map`). Bumper-only
  zones (3 hits, 0.25 m) exist but are off (`bumper_zone_hits: 0`).
- Size: radius = max(median ranged width, class prior diameter) / 2 + 0.10 m, max 1.0 m
  (priors: trunk 0.25, stone 0.15, wood 0.10, brick 0.20, trashbin 0.45, potted plant 0.30,
  chair 0.50). A pending zone whose radius moves >= 0.05 m is re-sent; map_server replaces the
  polygon of the pending proposal in place.
- Lifecycle: promoted through the existing id-only path = a PENDING `SOURCE_TRACKER` keepout
  named `auto: <class> #<id>` (live in both keepout masks, so Nav2 transits and row detours
  route round it at once; never written to `areas.dat`). Accept in the GUI (the Tracked
  Obstacles panel's Promote) now accepts the proposal and keeps `source = SOURCE_TRACKER` (=
  auto) and its name; before, the GUI's polygon hit the 0.10 m dedup and nothing happened.
  Ignore calls `/obstacle_tracker/clear_obstacle` (the GUI already called it; the service did
  not exist): the track and its pending keepout go, and new zones within 0.5 m are suppressed
  for the session. A pending zone expires after 1 h of mowing in its area without a sighting,
  or when the robot looks at it (0.8-3 m, within +-30 deg) and does not detect it 8 steps
  running. Duplicates within 0.3 m merge; a stereo cluster at the spot joins the zone.
- Zone set for C: `/obstacle_tracker/nogo_zones` (JSON `nogo_zones.ZoneSet`: zones with
  state pending / accepted / candidate, + the 60 s static marks). Query:
  `nogo_zones.pivot_sweep_hits(...)` / `pivot_clear_direction(...)` (unit-tested).
- No GUI change was needed: the panel and the map already show promoted (status PERSISTENT)
  tracker obstacles, which now includes the pending zones. It cannot tell pending from
  accepted, or show the class (TrackedObstacle has no field; the name in `obstacle_info`
  carries it). A pending zone in a map the GUI then saves becomes a user keepout unless
  `ws/gui-obstacles` (preserves `obstacle_info`) is in.
- Still open: a pending zone is in `get_mowing_area`, so a re-START plans it as a coverage
  hole (mission_fsm does not read the pending flag; the next C job decides). Lap 0 still
  cannot go round a zone mid-mow (C).

**Side findings (not changed).**
- `area_settings.yaml` gives the area named **"Area 5"** (index 2) `obstacle_detection: none`:
  det_range sends no detection marks there and obstacle_guard ignores vision, and now no zones
  form there either (the tracker logs it at start). The trunk incident area is index 5 (one of
  two areas both named "Area 7"); settings are keyed by name, so the duplicate name shares one
  settings entry.
- `areas.dat` has a stored 0.33 m, 12-vertex ring obstacle at (-9.93, 1.60) in "Area 5", with
  no name and no source key (= user). Its shape matches a tracker / dig ring, so it is probably
  an auto promotion accepted (or saved through the GUI, which drops the source flag) before
  the source flag existed. Owner to check whether it is real.

- The relayed det / bumper clouds have no `sensor_frame`, so the costmap measures
  `obstacle_max_range` from the cloud frame's ORIGIN (odom / map, near the dock), not from the
  robot. det_range marks more than 3 m from the dock never reached the costmaps (found by the
  trunk gate: run 1 had no costmap marks at the trunk). Fixed for `det_range` / `det_static`
  (50 m). The `bumper` source has the same bug with 1.0 m (bumper marks only near the dock):
  **not changed** here, a one-line fix for the owner to approve.

**Validation.** Unit tests (`DEV_ROS_DOMAIN_ID=218 ./scripts/dev_build.sh test` on det_range,
mower_map, mower_sim, mower_navigation, mower_control: all pass; mower_bringup's
`cut_width_m == 0.20` failure is pre-existing on master): `det_range/test/test_width.py`,
`mower_map/test/test_nogo_zones.py` (width sizing, voting thresholds, living classes never,
on-path corridor / pivot / bumper, pending lifecycle, re-size, expiry, gone check, merge,
Ignore, adopt, the map_server accept / re-size paths and the SOURCE_TRACKER flag through
areas.dat, the pivot-sweep query), `mower_sim/test/test_detections.py`,
`mower_navigation/test/test_params_yaml.py` (static_det_layer, 60 s).

Sim gate (x1, domain 219, `accuracy-simout/nogo/`):

| run | world | result |
|---|---|---|
| `plain_x1` | real_run5 | MOWING_COMPLETE, docked, 0 trips, corners 0.051 m; FAIL only on d (1 costmap-fallback transit, goal off the route graph) and i (side overhang 0.219 m), the same two as the current baseline `cleanup/stage0_ideal_x1_r2` (1, 0.218 m): **no regression** |
| `trunk_x1` | real_run5_trunk (before 980ec89) | zone `trunk` r 0.25 m formed 22 s after START but 0.14 m short of the trunk (front face), auto-removed at 780 s (gone check used base_link), no costmap marks (origin bug): 73 bumper hits, 2 sub-paths not mowed, MOWING_INCOMPLETE |
| `trunk2_x1` | real_run5_trunk | zone formed 23 s after START at (-5.53, 1.27) (truth -5.529, 1.269), r 0.25 m, pending for the whole run; 14 bumper hits (was 73), lap 0 still hits it 3 times and skips 2 m (sub-path 0 not mowed: the plan clips the trunk, C); then a **route-corner Spin 0.88 m from the trunk aborted "Collision Ahead" 3 times -> STUCK_NEEDS_HELP** (field incident 20:30 reproduced: the persistent 0.25 m mark is inside the 0.645 m pivot sweep, physically 0.09 m clear) |

So A forms the right zone, the costmap now keeps the trunk, and the body stops hitting it
except on lap 0, but routing round it needs C: lap 0 pulled inward and the pivot fallbacks
above (other direction / back-up / move the pivot) before a Spin is sent.
