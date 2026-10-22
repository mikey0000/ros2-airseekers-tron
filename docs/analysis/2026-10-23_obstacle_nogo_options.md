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
