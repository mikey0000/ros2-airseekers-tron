# Public ROS 2 ecosystem scan for an RTK mower stack (2026-11-03)

Method: web search plus GitHub API metadata fetched 2026-11-03 (pushed_at = last push, a proxy for
"commits in the last year"). Our side: `docs/REVIEW_2026-11-02.md` sections 1-4 (verdict, H1-H6,
stock-firmware gaps, 10-step direction) and the `src/` package list. Items marked "(unverified)"
come from search snippets or memory and were not confirmed against a primary page. Our review
already decided not to swap docking, routing or coverage to upstream; this scan tests that.

## 1. Whole-mower projects

| Project | Maturity | Licence | Verdict |
|---|---|---|---|
| [MowgliNext](https://github.com/mowglinext/mowglinext) | Pushed 2026-11-02, 51 stars, beta on real hardware. Nav2 + BehaviorTree.CPP v4, GTSAM iSAM2 factor-graph localizer fusing GPS/IMU/wheels, optional LiDAR anchor through GNSS outages, React/Go GUI with in-UI update and automatic rollback, dashboard stating why the mower stopped, F9P/UM98x/NMEA receivers ([README](https://raw.githubusercontent.com/mowglinext/mowglinext/main/README.md)). | Dual: GPLv3 for personal/community use, separate commercial licence ([README](https://raw.githubusercontent.com/mowglinext/mowglinext/main/README.md)); API reports NOASSERTION ([API](https://api.github.com/search/repositories?q=mowglinext&per_page=5)). | Borrow ideas: "stop reason" surfaced to the user, GUI update with rollback (our review step 10), factor-graph fusion as a later heading/RTK-loss option. Do not copy code: the commercial clause conflicts with our open-source plan unless GPLv3 reuse is confirmed per file. Its guards and RTK-loss policy are not described in the README; read `behavior_tree` sources before claiming they beat ours. |
| [OpenMowerROS2 / OpenMowerNext](https://github.com/jkaflik/OpenMowerROS2) | Last push 2026-06-22, 52 stars; docs list control, localisation, odometry, IMU, GPS, Webots sim, map management as working and mower logic, config, UI as in progress ([docs](https://jkaflik.github.io/OpenMowerNext/getting-started.html)); one commit seen on 2026-05-24 within the last year ([API](https://api.github.com/repos/jkaflik/OpenMowerROS2/commits?per_page=1&since=2025-11-03T00:00:00Z)). Jazzy. | Apache-2.0 ([API](https://api.github.com/repos/jkaflik/OpenMowerROS2)) | Ignore as a stack (no mower logic). Borrow idea: Webots simulation as a second simulator for contact/slope physics, relevant to review step 2. |
| [open_mower_ros](https://github.com/ClemensElflein/open_mower_ros) (ROS 1 Noetic) | 714 stars, pushed 2026-10-31; releases v1.2.0 (2026-05), 1.3.0 (2026-08), v1.4.0 (2026-09) ([API](https://api.github.com/repos/ClemensElflein/open_mower_ros), [releases](https://api.github.com/repos/ClemensElflein/open_mower_ros/releases?per_page=3)). Requires Noetic ([search](https://github.com/ClemensElflein/open_mower_ros)). | GPL-3.0 | Borrow ideas only: largest field-tested mower community; its issue tracker is the best public catalogue of RTK-loss, docking and slope failures. Code reuse is possible under GPLv3 but ROS 1. |
| xbot (OpenMower firmware layer) | Search found no confirmed link to the ROS 2 port. | n/a | Ignore until a primary source exists. |
| CubeMower, other GitHub RTK mowers | API search for "cubemower" and "robot mower ros2 rtk" returned zero repositories ([API](https://api.github.com/search/repositories?q=cubemower+OR+robot+mower+ros2+rtk&per_page=8)); not verified further. | n/a | Ignore: nothing public found. |

Net: nobody public beats our recovery/guard coverage on evidence; MowgliNext is the only active
ROS 2 mower with a comparable scope and its README does not document RTK-loss or turn handling.

## 2. Coverage and turns

| Item | Maturity | Licence | Verdict |
|---|---|---|---|
| [Fields2Cover](https://github.com/Fields2Cover/Fields2Cover) 2.x | Pushed 2026-10-27, 921 stars; PyPI shows 2.1.0 current, ROS index lists 2.0.0 ([PyPI](https://pypi.org/project/fields2cover/), [ROS index](https://index.ros.org/p/fields2cover/)). Turns: Dubins and Reeds-Shepp, with or without continuous curvature ([ROS index](https://index.ros.org/p/fields2cover/)). | BSD-3-Clause ([API](https://api.github.com/repos/Fields2Cover/Fields2Cover)) | Borrow idea: the continuous-curvature (CC) turn and headland-order options as offline references to test our swath-end turn geometry in the sim. Do not adopt: our turns are tuned to the wheel deadband and the mission FSM (STATUS "Swath-end turns without pivot loops"). |
| [opennav_coverage](https://github.com/open-navigation/opennav_coverage) | Pushed 2026-11-02, 320 stars, main works with Fields2Cover v2 (verified against v2.0.0); navigator plugins need Iron or newer ([README](https://github.com/open-navigation/opennav_coverage)). | Apache-2.0 ([API](https://api.github.com/repos/open-navigation/opennav_coverage)) | Ignore as a replacement (our `mower_coverage` has edge laps, no-go replans, skip-and-revisit). Borrow idea: its swath-ordering/route-planner options only if we want a second opinion on area ordering. |

## 3. Navigation

| Item | Facts | Licence | Verdict |
|---|---|---|---|
| `opennav_docking` (Nav2) | Backward docking with `rotate_to_dock`: approach forward so sensors see the dock, then rotate and reverse using the first detection by dead reckoning; detectors can be started/stopped on demand; charging detected from battery current or wheel stall; `max_retries` default 3; `use_external_detection_pose` with a timeout and filter ([README](https://raw.githubusercontent.com/ros-navigation/navigation2/main/nav2_docking/README.md)). | Apache-2.0 (Nav2; unverified per file) | Borrow idea: on-demand detector start/stop to cut CPU (we run at load 8-12), and "freeze first good detection, reverse blind" which is exactly the failure mode of our night dock (lit marker flips yaw). Do not swap the docking node. |
| `nav2_route` | Dijkstra over a directed graph, plugin edge scorers (distance, penalties, costmap, orientation), route operations on edge/node enter/leave (e.g. speed limits), rerouting, route tracking feedback, GeoJSON graphs; off-graph start/goal must be bridged by the caller ([README](https://raw.githubusercontent.com/ros-navigation/navigation2/main/nav2_route/README.md)). | Apache-2.0 (unverified per file) | Borrow idea / later adopt (review already lists it as the one large later adoption): the off-graph bridging contract matches our "dock return joins the graph first" fix; edge penalties give per-edge slope or wet-zone costs. |
| MPPI critics | Jazzy MPPI has `PathAlignCritic` (`use_path_orientations`, skipped near the goal); a PathAlign fix improved tracking noticeably ([API docs](https://api.nav2.org/nav2-jazzy/html/path__align__critic_8hpp_source.html), [WG notes](https://discourse.openrobotics.org/t/nav2-feb-15-2024-working-group-notes/36110)). | Apache-2.0 | Borrow idea: tune PathAlign/PathFollow weights and orientations for line holding (review step 5 and the 2 cm weave lever); no new package. |
| Collision monitor `VelocityPolygon` | Added in Jazzy: polygons selected by current velocity ([Jazzy release](https://discourse.openrobotics.org/t/nav2-jazzy-release/38321), [migration](https://docs.nav2.org/migration/Iron.html)). | Apache-2.0 | Adopt if not already in `mower_navigation`: speed-scaled stop polygon replaces fixed polygons and cuts nuisance stops on grass (STATUS "Vision false stop fixes"). Check our config first; I did not diff it. |
| Speed filter (costmap filter) | Docs page fetch failed (404); the review already proposes it for slope speed scaling. | Apache-2.0 | Adopt (review item, small). |
| Nav2 BT recoveries vs our FSM | Not verified against a primary source. | n/a | Ignore: our FSM carries mow-specific budgets and persisted intent; replacing it with Nav2 BT recoveries would lose them. Learn only the naming of a single recovery planner (review step 5). |

## 4. Localisation and sensors

| Item | Maturity | Licence | Verdict |
|---|---|---|---|
| [robot_localization](https://github.com/cra-ros-pkg/robot_localization) | Pushed 2026-11-02, 1983 stars ([API](https://api.github.com/repos/cra-ros-pkg/robot_localization)). | NOASSERTION per API (BSD-3 in package.xml, unverified) | Already used (dual EKF). Keep. Adopt its yaw input from the dual-antenna receiver as absolute yaw (review step 4). Note: UM982 heading is 0.1 deg per metre of baseline ([manual](https://jp.unicore.com/uploads/file/UM982_User%20Manual_EN_R1.3.pdf)); the baseline on a mower is under 1 m so expect about 0.1-0.3 deg, a bench check, not an assumption. |
| Unicore ROS 2 drivers | Only small community ones: [lostDeers/UM982Driver-ros2](https://github.com/lostDeers/UM982Driver-ros2) pushed 2025-03-19, 4 stars, no licence ([API](https://api.github.com/repos/lostDeers/UM982Driver-ros2)); a PyPI parser supports GNHPR/PVTSLN/BESTNAV ([PyPI](https://pypi.org/project/um982-driver/)). | none / unknown | Ignore as a dependency; borrow the sentence list (UNIHEADINGA at a set rate enables heading; 10 Hz measured, [PX4 forum](https://discuss.px4.io/t/um982-dual-antenna-heading/33291)). Extend our own `um960_gps_driver`. |
| [septentrio_gnss_driver](https://github.com/septentrio-gnss/septentrio_gnss_driver) | ROS 1 and 2, pushed 2026-10-18, 133 stars ([API](https://api.github.com/repos/septentrio-gnss/septentrio_gnss_driver)). | BSD-3-Clause | Borrow idea: its heading/attitude (`AttEuler`) and quality/diagnostic message set as a template for ours. Adopt hardware only if a mosaic-H is chosen; not otherwise. |
| [KumarRobotics/ublox](https://github.com/KumarRobotics/ublox) | Pushed 2026-08-21, 539 stars ([API](https://api.github.com/repos/KumarRobotics/ublox)). | BSD-3-Clause | Ignore: F9P moving-base needs two receivers on one board; we plan a single dual-antenna chip. |
| RTK-loss practice (gps_waypoint_follower examples) | Not found/verified. | n/a | Ignore; MowgliNext's LiDAR anchor is the only public ROS 2 mitigation seen, and review rejected sensor-through-loss as too CPU heavy. |
| IMU drivers | Nothing verified; `wit_imu_driver` stays. | n/a | Ignore. |

## 5. Perception

| Item | Maturity | Licence | Verdict |
|---|---|---|---|
| [yolox_rk3588_ros](https://github.com/ar-ray-code/yolox_rk3588_ros) | ROS 2 YOLOX on RK3588, INT8 58 FPS Nano / 23 FPS S ([index](https://awesome.ecosyste.ms/projects/github.com%2Far-ray-code%2Fyolox_rk3588_ros)); activity unverified. | Apache-2.0 | Borrow idea only; `mower_rknn` and `det_ros` already cover this. |
| [rknpu2_ros](https://github.com/fateshelled/rknpu2_ros) | YOLOv5 on RK3588s, Humble ([README](https://github.com/fateshelled/rknpu2_ros/blob/main/README.md)); activity unverified. | unverified | Ignore. |
| Grass segmentation projects | No ROS 2 project surfaced in this scan. | n/a | Ignore; `seg_ros` is ahead of anything found. |

## 6. Operations

| Item | Facts | Licence | Verdict |
|---|---|---|---|
| rosbag2 snapshot mode | `--snapshot-mode` keeps messages in a circular in-memory buffer bounded by `--max-cache-size` and `--max-cache-duration`, saved by calling `~/snapshot`; memory can reach 2x the cache size ([rosbag2 mirror docs](https://github.com/ottojo/rosbag2), [index](https://docsearch.algolia.com/mcp/docs/repo/ros2/rosbag2)). | Apache-2.0 (unverified) | Borrow idea: review proposes it for the black box, but we now have an on-disk ring with verified zstd splits (STATUS "Bag ring"). Only add snapshot for a short pre-trigger window on EMERGENCY if the 2x RAM cost is acceptable (memory footprint report). Low priority. |
| Lifecycle/bond watchdogs | Not fetched. | n/a | Borrow idea: Nav2 lifecycle manager bonds for H2/H3 (the review's critical-node list). Unverified here; read the Nav2 lifecycle docs before committing. |
| [watchtower](https://github.com/containrrr/watchtower) | Archived, last push 2025-12-17 ([API](https://api.github.com/repos/containrrr/watchtower)). | Apache-2.0 | Ignore: archived, and unattended image pulls conflict with "deploy is done by the owner" and sim-gated releases. |
| [Mender](https://github.com/mendersoftware/mender) | Pushed 2026-10-29, 1243 stars ([API](https://api.github.com/repos/mendersoftware/mender)). | NOASSERTION per API (Apache-2.0 in docs, unverified) | Borrow idea: A/B rollback with commit-after-health-check. Adopt later only if a fleet exists; our tagged-image rollback (review step 10) is enough for one unit. |

## Top 5 adoptions, by reliability payoff per effort

1. Collision-monitor VelocityPolygon plus a Nav2 speed-filter mask for slopes: config-only, Jazzy-native, targets nuisance stops and slope speed (sources in section 3). Check the existing config first.
2. Dual-antenna yaw into robot_localization as absolute yaw, with the receiver driver extended in-house from the UNIHEADINGA/GNHPR sentences: removes the largest root cause in the review (single-antenna heading). No public ROS 2 driver is worth depending on.
3. Docking "freeze first good detection, reverse blind" plus on-demand detector start/stop, copied as logic from `opennav_docking` into `mower_docking`: addresses the night-dock five-approach incident and CPU.
4. Surface a user-visible stop reason and update-with-rollback in the GUI, following MowgliNext's design (idea only, due to its licence): cheap, and supports unattended-operation triage.
5. MPPI PathAlign/PathFollow retune for line holding, using the Jazzy critic parameters, validated on the sim gate: a parameter change, no new code. Defer `nav2_route` tracking to after the guard consolidation.

Gaps in this scan: Nav2 docs pages for the speed filter, docking parameters and lifecycle bonds returned 404;
MowgliNext's guard and RTK-loss logic, CubeMower, gps_waypoint_follower and IMU drivers were not verified.
