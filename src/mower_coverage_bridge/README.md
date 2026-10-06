# mower_coverage_bridge

Serves the MowgliNext coverage action, `/plan_coverage`
(`mowgli_interfaces/action/PlanCoverage`), on top of our own Fields2Cover 2.1
planner, the `/coverage/plan` service (`mower_interfaces/srv/PlanCoverage`,
package `mower_coverage`). With it, the mission layer's `PlanCoverageArea` →
`FollowStrip` flow runs unchanged: one Nav2 `FollowPath` goal
(`FollowCoveragePath`) per drivable sub-path, with a blade-off transit
between sub-paths.

| | |
|---|---|
| Node | `coverage_server` (executable `coverage_action_server`) |
| Serves | action `/plan_coverage` — `mowgli_interfaces/action/PlanCoverage` |
| Calls | service `/coverage/plan` — `mower_interfaces/srv/PlanCoverage` |
| Launch | `ros2 launch mower_coverage_bridge coverage_bridge.launch.py [start_planner:=false] [transit_gap_m:=0.6] [cut_width_m:=0.20] [swath_overlap_m:=0.02]` |

The geometry lives in `mower_coverage_bridge/splitter.py`. It is pure Python
with no ROS imports, and pytest covers it.

## Flow

1. Goal: `outer_boundary` → `boundary`; `obstacles` (≥ 3 points each) →
   `holes`; `mow_angle_deg` is passed through (< 0 = auto). The node sets
   `frame_id` to `map` and leaves `has_start`/`has_goal` false, so the
   planner adds no transit-in or transit-out poses.
   - `perpendicular` with a fixed angle plans at `angle + 90`.
   - `perpendicular` with auto plans once to learn the auto angle, then plans
     again at `auto + 90`.
2. The service is called synchronously. The node waits up to `service_wait_s`
   for it to appear and up to `service_timeout_s` for an answer. If the
   service is unavailable or times out, the goal is **ABORTED** with a clear
   `message`, for example
   `coverage planner service /coverage/plan unavailable (is mower_coverage_node running?)`.
3. The single returned `nav_msgs/Path` is split into segments, which are then
   joined into sub-paths. Each output Path is densified, gets a yaw on every
   pose, and is published in frame `map`.
4. Feedback `phase` reports progress: `waiting for …`, `planning …`,
   `splitting …`, `done: <summary>`. The result carries `success` and
   `message`. `message` holds the summary, followed by the planner's own
   message, which includes its drop notes.

## Segments vs drivable sub-paths

```
 planner path (one nav_msgs/Path, NOT densified):
   R1 ring (closed: last pose == first) , R2 ring , S1 , S2 , S3 , ... (2 poses per swath)

 segments  (GUI / resume bookkeeping; segment_types = RING 0 / SWATH 1)
   ┌──R1──┐ ┌──R2──┐   S1 ───────►      S3 ───────►   S5 ──►        S6 ──►
   └──────┘ └──────┘   S2 ◄───────      S4 ◄───────
                                            │<──── > 0.6 m ────>│

 drivable_subpaths (what FollowStrip drives, one FollowPath goal each)
   sub-path 0: R1 ─0.18─ R2                      gap ≤ 0.6 m → joined by a straight connector
               ╎ blade-off transit (gap > 0.6 m)
   sub-path 1: S1 ┐ S2 ┐ S3 ┐ S4 ┐ S5            U-turns (0.18 m) are ≤ 0.6 m → driven through
               ╎ blade-off transit (S5 → S6 is a trimmed piece on the same sweep line, far away)
   sub-path 2: S6 ...
```

**Segment rule.** Read `mower_coverage_node.cpp` for the path layout. The
planner path is **sparse**:

- A ring is the polygon's vertices, closed by repeating its first pose.
- A swath is exactly two poses.
- There are no connector poses.

Every swath edge is metres long, so a "pose spacing > 0.6 m" rule would cut
every swath. The splitter therefore uses the planner's own structure:

1. **Rings.** From the current pose, the first later pose that returns to it
   (within 1 mm) closes one ring. This is repeated `ring_count` times, using
   the count from the service response.
2. **Swaths.** If exactly `2 × swath_count` poses remain, they are paired.
   If the counts do not match the path, and all remaining 2-pose pairs are
   parallel, they are paired anyway.
3. **Fallback for densified paths** (a future planner): there is a segment
   boundary at
   - any edge longer than `transit_gap_m`;
   - any pose where the heading turns by more than `turn_split_deg`;
   - any short connector edge across which the heading reverses by more
     than `turn_split_deg` (a boustrophedon U-turn).

   Closed pieces are typed RING and the rest SWATH. When this fallback is
   used, the node logs a warning.

The result therefore has one segment per headland ring and one per swath
piece. A swath that the field clipped into several pieces becomes one segment
per piece. The enum has no TRANSIT type, so transits are never segments:
they are the gaps between sub-paths.

**Join rule.** Segments are taken in order. A segment is appended to the
current sub-path when the gap from the end of the current sub-path to its
start is ≤ `transit_gap_m`
(`mowgli_interfaces::coverage_geometry::kSegmentTransitGapM` = 0.6, inclusive,
the same as `FollowStrip`), **and** the straight connector neither properly
crosses the outer boundary nor crosses or enters an obstacle
(`split_at_keepouts`). Otherwise a new sub-path starts. A connector that
only touches the boundary does not count as crossing, so swath ends lying on
the line with `headland_passes = -1` still join.

## Result fields

| Field | Content |
|---|---|
| `segments`, `segment_types` | one Path per ring (out→in) then per swath piece; `SEGMENT_RING=0`, `SEGMENT_SWATH=1` |
| `drivable_subpaths` | joined sub-paths, frame `map`, densified to `densify_step_m` |
| `full_path` | concatenation of `drivable_subpaths` (as upstream) |
| `total_distance` | total length of the sub-paths, connectors included. This follows the upstream server's code; its comment in the `.action` file says "segments". |
| `ring_count`, `swath_count` | counted from `segment_types` |
| `planning_time_s` | wall time of the whole goal |
| `message` | summary: counts, segment and sub-path lengths, transit count and length, swath angle, split mode, plus the planner's message |

The action has no `swath_angle` field. The angle in use (first swath, folded
to [0, 180)) appears in `message`.

## Parameters

| Name | Default | Meaning |
|---|---|---|
| `action_name` | `/plan_coverage` | served action |
| `service_name` | `/coverage/plan` | planner service |
| `frame_id` | `map` | frame of every output Path, and of the request |
| `transit_gap_m` | `0.6` | join/split threshold; keep it equal to `kSegmentTransitGapM` |
| `turn_split_deg` | `150.0` | heading-reversal split, used only by the densified-path fallback |
| `densify_step_m` | `0.10` | maximum pose spacing in output Paths (upstream `kSwathStep`); `0` = keep the planner's sparse poses |
| `split_at_keepouts` | `true` | split a ≤ 0.6 m connector that crosses the boundary or an obstacle |
| `service_wait_s` | `2.0` | how long to wait for `/coverage/plan` to appear before aborting |
| `service_timeout_s` | `30.0` | how long to wait for the plan response before aborting |
| `operation_width` | `0.0` (launch: `cut_width_m - swath_overlap_m` = 0.18) | swath spacing forwarded to the planner; ≤ 0 = the planner default |
| `headland_width`, `headland_passes`, `min_swath_length` | `0.0`, `0`, `0.0` | forwarded to the planner; ≤ 0 (passes 0 = auto) means the planner default (0.20 m, auto, 0.15 m) |
| `headland_rings` | `-1` | per-area perimeter laps, forwarded as `headland_rings`: -1 = use `headland_passes`, 0 = none, > 0 = exactly (validated -1..20) |
| `path_mode` | `zigzag` | `zigzag` / `cross` / `alternate` (all sent as `zigzag`: cross and alternate are several mission-level zigzag plans), `spiral`, `contour_only`; anything else is refused |
| `mow_angle_deg` | `-1.0` | used only when the goal's `mow_angle_deg` is < 0 (the goal wins) |
| `edge_first` | `true` | `false`: the planner lays out the swaths first, then the rings innermost first; the splitter is told so (`swaths_first`) |

The last four are the per-area mowing settings. `behavior_tree_node` (mower_mission) sets
them with `set_parameters` right before every PlanCoverage goal, together with
`operation_width` (= `cut_width_m` - the area's `swath_overlap_m`). They stay set until
the next change, so a goal from another client plans with the last area's pattern. A
set-parameters callback validates `path_mode`, `headland_rings` and `operation_width` and
logs every change.

Planner modes (`mower_coverage` `planCoverage`):

* `zigzag`: headland rings + boustrophedon swaths (unchanged).
* `spiral`: concentric rings at `operation_width` spacing, outermost first (the first one on
  the recorded line, like a headland ring), until the field collapses; one last ring half a
  swath outside the collapse offset covers the centre strip. Holes get rings around them
  like headlands; where the field splits, each part is spiralled. Ring starts are chained
  (one `operation_width` step), so a plain field is one drivable sub-path. No swaths.
* `contour_only`: only the `headland_rings` perimeter laps (at least one), no swaths.
* `edge_first = false`: same rings and swaths; swaths first, then the rings innermost first,
  chained from the last swath end.

### Swath width

There is one source for the swath spacing: the launch arguments `cut_width_m`
(default `0.20`, the URDF cutter disc radius 0.10 m × 2) and `swath_overlap_m`
(default `0.02`). `coverage_bridge.launch.py` (and `mower.launch.py`, which
forwards both) passes them to `mower_coverage_node` (parameters of the same
name; its default spacing for requests with `operation_width ≤ 0` is their
difference) and sets this node's `operation_width` to `cut_width_m -
swath_overlap_m` = 0.18 m. **TODO:** measure the blade tip circle on the Tron
and update `cut_width_m` together with `cutter_radius` in
`config/urdf/mower.urdf.xacro`.

All parameters are read again for each goal, so a `ros2 param set` applies
to the next plan.

`mower_interfaces/srv/PlanCoverage` carries the per-area fields with IDL defaults that
keep the old behaviour for a client that leaves them alone: `int32 headland_rings -1`,
`string path_mode "zigzag"` (`""` = zigzag), `bool edge_first true`.

## Differences from upstream `mowgli_coverage`

- **Connectors.** Joins are straight connectors, not Dubins turn-around arcs.
  The Tron is diff-drive and pivots in place, and the planner does no turn
  planning on purpose.
- **Rings.** Each ring is one segment. Upstream splits a ring into one
  segment per corner-to-corner arc.
- **Hole headlands.** `mower_coverage` only emits each pass's exterior ring,
  so no headland rings go around holes.
- **Concave fields and holes.** In such fields, F2C 2.1's
  `BoustrophedonOrder` alternates between the pieces of each sweep line.
  The split is correct, but the plan has many sub-paths and transits. For
  example, a 10×6 m field with a 2×2 m hole gives 30 sub-paths and 29
  transits totalling 175 m. That is a planner ordering issue (no cell
  decomposition), not this bridge's.

## Tests

`python3 -m pytest -q test` runs on the host; `conftest.py` puts the package
on `sys.path`. Alternatively:
`./scripts/dev_build.sh test --packages-select mower_coverage_bridge`.
The suite has 25 tests. They cover synthetic layouts, including an L-shaped
field whose sweep lines were trimmed into far-apart pieces, plus two
regressions on real `/coverage/plan` output in `test/data/`.
