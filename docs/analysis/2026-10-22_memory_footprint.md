# Memory footprint of the Jazzy stack on the mower (2026-10-22)

Snapshot taken 2026-10-22 09:55 mower time, docked/idle after a morning mow, stack up 3 h 40 min
(`mower_jazzy` on `bbc616c`-era install), read-only: `free`, `/proc/meminfo`, `ps`, and
`/proc/<pid>/smaps_rollup` read from inside the container (`docker exec`, the host user cannot read
root-owned smaps). Dev-image micro-benchmarks (x86-64, `mower:jazzy-dev-amd64`, Fast-DDS, same
profile XML) are used only to attribute costs; they are marked **(dev)**. Nothing on the mower was
changed. Options only: nothing in this document has been implemented.

## 1. Headline numbers

| | MB |
|---|---|
| MemTotal | 3904 |
| MemAvailable **at idle** | **297** (MemFree 45) |
| AnonPages (private heaps) | 3209 |
| Page cache (Cached + Buffers) | 344, of which Shmem 116 |
| Slab + KernelStack + PageTables | 197 |
| CMA (camera/NPU DMA pool) | 128 total, **8 kB free** |
| Swap / zram | **none** (`/dev/zram0` exists, disksize 0, kernel has `CONFIG_ZRAM`; `swappiness` 60) |
| Committed_AS / CommitLimit | 12.1 GB / 1.9 GB (overcommit heuristic, mode 0) |
| `/dev/shm` | 31 MB used of 1952 (Fast-DDS segments; `shm_gc.sh` keeps it clean) |
| Load average | 8.10 on 8 cores, container at 380 % CPU while *idle* |

The 297 MB available at idle is the whole working margin. Camera, detection and docking nodes each
peak 50-60 MB above their idle RSS when they run at full rate (VmHWM column below), i.e. mowing
needs ~400 MB more than idle, which is exactly the observed drop to ~291 MB and today's freeze. With
no swap, the only thing the kernel can evict under pressure is file-backed text; it then refaults
`libpython`/`libfastrtps`/`libopencv` pages continuously (load 7-17 with the CPUs mostly waiting),
sshd cannot get a page, and the box looks dead without any OOM-kill line in `dmesg` (none seen).

Container totals (sum of PSS from `smaps_rollup`, 59 processes):

| Group | Procs | RSS MB | **PSS MB** | PSS anon | PSS shmem |
|---|---|---|---|---|---|
| Python rclpy nodes (incl. `ros2 bag record`, `ros2 launch`) | 37 | 3 560 | **2 422** | 2 300 | 22 |
| C++ nodes (Nav2, robot_localization, det_ros, foxglove, misc) | 22 | 1 552 | **795** | 690 | 90 |
| `mower_jazzy` total | 59 | 5 112 | **3 217** | 3 008 | 112 |
| `mower_gui` (`mowglinext` Go backend + sh) | 2 | 48 | **47** | | |
| Host: dockerd 43, containerd 21, fuse-overlayfs 22, journald 17, rkaiq_3A 12, NM/colord/etc ~40 | | | ~155 | | |

RSS over-counts by 1.9 GB because 59 processes map the same `libpython3.12`, `librmw_fastrtps`,
`libfastrtps`, OpenCV and message typesupport libraries; **PSS is the number to budget with.**

Note the **anon share**: 93 % of the stack's PSS is private heap, only 3 % is shared file text.
Shared libraries are already shared; what is not shared is *per-process state*: an interpreter, an
`rclpy` context, a Fast-DDS participant with its discovery database and histories, thread stacks
and malloc arenas, repeated 59 times.

## 2. Per-process table (PSS, MB, sorted)

`shmem` is the process's share of `/dev/shm` segments and DMA buffers. The `grew` column is
VmHWM - VmRSS (how much higher the process has been than it is now, during the morning mow).

| Node (`__node:=`) | Lang | RSS | PSS | PSS anon | PSS shmem |
|---|---|---|---|---|---|
| foxglove_bridge | C++ | 189 | 157 | 154 | 0 |
| det_ros | C++ | 175 | 145 | 59 | 82 |
| mower_docking | Py | 184 | 141 | 139 | 0 |
| behavior_tree_node | Py | 159 | 117 | 116 | 0 |
| ros2 bag record (child of bag_recorder) | Py | 135 | 92 | 88 | 0 |
| left_oa_camera | Py | 122 | 91 | 74 | 0 |
| right_oa_camera | Py | 121 | 90 | 73 | 0 |
| obstacle_tracker | Py | 127 | 84 | 82 | 0 |
| stereo_cam | Py | 115 | 84 | 78 | 0 |
| stereo_depth | Py | 112 | 79 | 78 | 0 |
| map_server_node | Py | 116 | 74 | 72 | 0 |
| mower_cameras | Py | 105 | 74 | 73 | 0 |
| det_range | Py | 108 | 70 | 69 | 0 |
| coverage_server | Py | 109 | 69 | 68 | 0 |
| gui_bridge | Py | 105 | 64 | 63 | 0 |
| supervisor | Py | 105 | 63 | 60 | 0 |
| heading_aligner | Py | 99 | 58 | 57 | 0 |
| cmd_vel_slew | Py | 97 | 58 | 56 | 0 |
| alert_node | Py | 97 | 57 | 56 | 0 |
| mower_mcu_driver | Py | 96 | 57 | 56 | 0 |
| obstacle_guard | Py | 94 | 56 | 54 | 0 |
| bag_recorder | Py | 94 | 56 | 54 | 0 |
| um960_gps_driver | Py | 93 | 55 | 54 | 0 |
| base_keys | Py | 94 | 55 | 53 | 0 |
| light_controller | Py | 94 | 55 | 53 | 0 |
| slip_detector | Py | 92 | 54 | 52 | 0 |
| wit_imu_driver | Py | 90 | 53 | 52 | 0 |
| cmd_vel_ws_relay | Py | 89 | 53 | 52 | 0 |
| activity_gated_lifecycle_manager | Py | 90 | 53 | 52 | 0 |
| stereo_imu | Py | 90 | 53 | 51 | 0 |
| tilt_monitor | Py | 90 | 53 | 51 | 0 |
| fixed_frame_relay | Py | 90 | 53 | 51 | 0 |
| localization_monitor | Py | 89 | 52 | 51 | 0 |
| fill_light | Py | 91 | 52 | 51 | 0 |
| gps_gate | Py | 88 | 51 | 50 | 0 |
| nav_lane_watchdog | Py | 88 | 51 | 50 | 0 |
| imu_cal | Py | 87 | 51 | 50 | 0 |
| collision_zone_gate | Py | 86 | 49 | 48 | 0 |
| controller_server | C++ | 78 | 47 | 45 | 0 |
| planner_server | C++ | 78 | 46 | 45 | 0 |
| bt_navigator | C++ | 72 | 42 | 40 | 0 |
| web_video_server | C++ | 71 | 41 | 40 | 0 |
| mower_coverage_node | C++ | 59 | 30 | 29 | 0 |
| behavior_server | C++ | 60 | 30 | 28 | 0 |
| ekf_map | C++ | 60 | 29 | 27 | 0 |
| ekf_node | C++ | 60 | 29 | 26 | 0 |
| collision_monitor | C++ | 59 | 29 | 27 | 0 |
| navsat_transform_node | C++ | 59 | 28 | 26 | 0 |
| lifecycle_manager_navigation | C++ | 56 | 28 | 27 | 0 |
| bumper_controller | C++ | 55 | 27 | 24 | 0 |
| ros2 launch (PID 1) | Py | 45 | 27 | 26 | 0 |
| velocity_smoother | C++ | 53 | 25 | 24 | 0 |
| robot_state_publisher | C++ | 53 | 25 | 24 | 0 |
| twist_mux | C++ | 51 | 25 | 23 | 0 |

Peak minus current RSS (VmHWM - VmRSS) for the processes that grow with activity: mower_docking
+61, right_oa_camera +56, obstacle_tracker +55, stereo_depth +53, left_oa_camera +52, stereo_cam
+49, mower_cameras +47, web_video_server +30, mower_coverage_node +21, controller_server +16,
foxglove_bridge +15, bt_navigator +14, det_ros +13. Sum ~480 MB: that is the idle-to-mowing swing.

### Top 10 and why they are big

| # | Process | PSS | Why |
|---|---|---|---|
| 1 | foxglove_bridge | 157 | C++, but it is the GUI's whole transport: every topic the GUI backend subscribes to gets a subscription with default depth up to 25 and a 10 MB per-client send buffer; images and costmaps on demand. 33 threads. |
| 2 | det_ros (YOLO on NPU) | 145 | 82 MB is shmem: RKNN runtime DMA buffers and the model (`librknnrt`), 59 MB anon for OpenCV + pre/post-processing buffers. Mostly irreducible except model size/input resolution. |
| 3 | mower_docking | 141 | Python, imports **cv2** (97 MB PSS on x86 for the import alone (dev); ~45-60 MB on the arm64 apt build) + numpy + tf2_ros (32 MB (dev)) on top of the 38 MB rclpy floor; peaks 61 MB higher while docking (image temporaries). |
| 4 | mission_node | 121 | Python, no heavy imports; 119 MB anon is its own data (mission/zone/route graph state, Nav2 action clients, parameter/log clients). Worth a `tracemalloc` snapshot: 2-3x what a logic node should need. |
| 5 | ros2 bag record | 93 | A *second* Python process: `ros2` CLI + rosbag2_py + sqlite3 + zstd compression thread. The 1 MiB cache is already set; the floor is interpreter + rclpy + participant + sqlite page cache + zstd context. |
| 6-7 | left/right_oa_camera (v4l2_cam) | 93 each | Python V4L2 capture + cv2/numpy conversion; 17 MB file-backed = mmapped V4L2 buffers; +52-56 MB when streaming. |
| 8 | obstacle_tracker | 85 | Python + numpy + cv2 (mask ops), peaks +55 MB. |
| 9 | stereo_cam | 84 | Python stereo capture, cv2. |
| 10 | stereo_depth | 79 | Python + cv2 depth; +53 MB active. |

Then the long tail: **21 "small" Python nodes at 51-60 MB each = 1 150 MB.** `tilt_monitor`
imports only `rclpy` and three message packages and still costs 54 MB PSS. On the dev image the
same empty node is 38 MB: 3 MB interpreter + 10 MB `import rclpy` + **25 MB for `rclpy.init()` +
one node with 3 endpoints**, i.e. the Fast-DDS participant, its builtin discovery endpoints, UDP +
SHM transports and ~10 threads. The arm64 build with 58 peers and a few hundred remote endpoints
is ~14 MB heavier. A C++ trivial node (`twist_mux`) is 25 MB on the mower (10 MB (dev)), so the
**DDS participant floor is ~20-25 MB per process and the Python floor another ~25-30 MB.**

## 3. Where the stack's memory goes (attribution)

| Bucket | MB (PSS) | Basis |
|---|---|---|
| 59 x DDS participant + rclcpp/rclpy context floor | ~1 300 | 22 MB x 59 (mower C++ trivial node = 25 MB; (dev) 25 MB per context) |
| 37 x Python interpreter + `rclpy` + typesupport imports | ~500 | 13 MB (dev) x 37 |
| cv2/numpy/tf2_ros imports in 9 Python processes | ~400 | cv2 ~45, numpy ~15, tf2_ros ~30 (dev) |
| Real working data (images, model, costmaps, maps, mission state) | ~700 | remainder; det_ros 145, foxglove 157, mission 121 included |
| GUI container + docker + host daemons | ~200 | measured |
| Kernel (slab, page tables, stacks, CMA) | ~330 | meminfo |

The stock firmware for comparison (`ota_v1.3.27` launch files): ~18 ROS 1 processes, all C++
except `ntrip_ros.py`, `seg_ros` and the MQTT node, and ROS 1 has no per-process DDS participant
(roscpp nodes idle at 5-10 MB). That is how the same workload fit in 3.9 GB with the vendor's
VIO, segmentation and YOLO running. Our stack has 59 processes with 37 interpreters; the per-process
floor alone is ~1.8 GB and is the problem, not any single node.

## 4. Ranked options

Savings are container PSS unless stated. **M** = measured on the mower or the dev image, **E** =
estimated from the measured floors. "Reversible" means a one-line rollback without a rebuild.

### Stage 1: quick wins (config/host only, no behaviour change)

| # | Option | MB saved | CPU / latency | Effort | Risk | Reversible | Owner decides |
|---|---|---|---|---|---|---|---|
| Q1 | **zram swap on the host**: `zramctl /dev/zram0 -s 1536M -a zstd` (or lz4) + `swapon -p 100`, `vm.swappiness=150` (valid for zram-only on 5.8+), `vm.page-cluster=0`, shipped as a systemd unit under `docker/host/`. | 0 PSS; **+500-900 MB effective** (E: cold anon in Python heaps and DDS histories compresses 3-4:1; the kernel swaps cold heap instead of thrashing executable text). | Compress/decompress only on cold pages; removes the refault storm that produced load 7-17. Negligible at idle. | 1-2 h | Low. If the stack ever becomes *all* hot, zram adds CPU but still never freezes sooner than today. | `swapoff`; one unit file. | Yes/no; size (1-1.5 GB), algorithm. Host-side change outside the container (needs the deploy loop). |
| Q2 | **Early OOM guard** on the host (`earlyoom` or a 20-line script on `MemAvailable`/PSI): when available < 4 %, SIGINT the lowest-value process (`ros2 bag record` child, then `web_video_server`, then `foxglove_bridge`), never the control chain. | 0; prevents the freeze/power-cycle. | None. | 1-2 h | Low; must be told the kill order. | Stop the unit. | The sacrifice order; whether a dropped recorder is acceptable vs an ssh-dead mower. |
| Q3 | **`MALLOC_ARENA_MAX=2`** (+ `MALLOC_TRIM_THRESHOLD_=131072`) in `docker-compose.yml` environment: 15-33 threads per process each get their own glibc arena today. | 60-200 (E; M (dev): -1.6 MB on an idle 1-node process, more under load with many allocating threads). | Slight lock contention in the Fast-DDS threads; unmeasurable at our rates. | 10 min | Very low. | Remove env line. | None. |
| Q4 | **foxglove_bridge limits**: `max_qos_depth: 1`, `send_buffer_limit: 2000000` (10 MB default per client), `capabilities` without `connectionGraph`/`parameters` if the GUI does not use them, `topic_whitelist` instead of advertising everything. | 30-60 (E; the node's 154 MB anon is mostly histories and client buffers). | Fewer queued messages, lower GUI latency. | 1 h + GUI check | Low; a too-small send buffer drops frames to the GUI. | Params. | Which GUI features (parameters, graph) must stay. |
| Q5 | **Drop `web_video_server`** if the GUI only streams via foxglove/mediamtx (docs/gui.md lists foxglove and the cmd_vel relay only). | 43 (M) + 30 when active. | -2 % CPU. | 10 min | Low if unused; check the GUI camera page first. | Launch arg. | Confirm nothing uses port 8080 MJPEG. |
| Q6 | **Recorder**: keep the curated list but move the ring to the `LOW` profile whenever the mower is docked (already supported); set `--max-cache-size` stays 1 MiB; consider `--storage mcap` (rebuild) for crash-safe splits: the corrupt splits are sqlite files cut by the freeze/power-cycle, not a recorder bug. | 0-20 at idle (the child exits in LOW-profile pause: -93 MB while docked) | None. | 1 h (+ image rebuild for mcap) | Low. | Params. | mcap rebuild yes/no; whether docked recording is needed. |
| Q7 | **Nav2**: not a memory target. `controller_server` is 48 MB with MPPI `batch_size: 1000 x time_steps: 40` (~15 MB of xtensor state); local costmap 6 x 6 m at 0.05 m is 14 k cells. The 20 Hz misses are CPU starvation (380 % idle load, 8 x 15-20 % Python nodes, refault storms), not memory. Lowering batch_size to 600-800 cuts MPPI CPU ~30 %. | 0-5 | MPPI 30 % cheaper at 600; slight path-quality cost; needs the sim gate. | 30 min + sim | Medium (behaviour). | Params. | Whether to trade MPPI samples for headroom. |
| Q8 | `PYTHONOPTIMIZE`, Cython/mypyc of node code | ~0 (M (dev): `-O2` did not reduce PSS; docstrings are < 1 MB/process; compiling does not remove the 38 MB floor) | | | | | Not recommended. |

Stage 1 total: ~150-300 MB PSS plus the zram headroom; it should stop the freezes on its own.

### Stage 2: medium (composition, dependency trimming; behaviour unchanged but process layout changes)

| # | Option | MB saved | CPU / latency | Effort | Risk | Reversible | Owner decides |
|---|---|---|---|---|---|---|---|
| M1 | **Python node hosts**: one generic `mower_bringup/node_host` that imports N node modules and spins their `Node` objects in one executor (one `rclpy` context = one DDS participant). Group the 21 small nodes into 3-4 hosts by CPU budget and failure domain, e.g. (a) sensors: `um960`, `wit_imu`, `stereo_imu`, `gps_gate`, `imu_cal`, `localization_monitor`, `heading_aligner`; (b) command chain: `cmd_vel_slew`, `fixed_frame_relay`, `slip_detector`, `cmd_vel_ws_relay`, `nav_lane_watchdog`, `collision_zone_gate`; (c) housekeeping: `supervisor`, `alert_node`, `bag_recorder`, `base_keys`, `light_controller`, `fill_light`, `activity_gated_lifecycle_manager`, `gui_bridge`. Keep `tilt_monitor` and `mower_mcu_driver` standalone (safety). | **~800-900** (M (dev): 10 nodes in one process 49 MB vs 10 x 38 = 381 MB; on the mower 21 x 54 = 1 134 MB -> 3 hosts x ~80 MB = 240 MB). | Fewer participants: discovery traffic and the ~10 DDS threads per process go away 18 times; CPU drops (today 21 processes x 1-19 % = ~110 % mostly executor/DDS overhead). GIL coupling: a slow callback in one node delays the others in that host, so cap each host at ~40 % Python CPU and keep safety nodes out. `sub_pump` threads keep working unchanged (one pump per node). | 2-3 days: factory `make_node(ctx)` per module instead of `main()`, host script, launch arg, per-node log tags, crash_record per host, sim gate. | Medium: shared fate on a crash (one host restart drops ~7 nodes for ~3 s; launch `respawn`), log file naming changes, `ros2 node list` unchanged. | Launch arg `python_hosts:=false` keeps separate processes. | Grouping, which nodes must stay isolated, acceptable restart blast radius. |
| M2 | **Camera host**: `left_oa_camera`, `right_oa_camera`, `stereo_cam`, `stereo_depth`, `camera_node` in one Python host (same mechanism as M1). | ~170 (E: 5 x ~40 MB floor + one cv2/numpy import instead of 5 x ~45 (dev)) and the 5 x 50 MB activity peaks partly overlap. | Images stay in-process (no 0.9 MB/frame UDP copies for the stereo pair -> depth path) but all five share one GIL: ~85 % Python CPU today, too much for one interpreter unless capture threads release the GIL in cv2/V4L2 calls (they do for the ioctl and cv2 ops). Try 2 hosts (OA pair; stereo trio). | 1-2 days | Medium: frame-rate regression under GIL; needs the camera test in docs/cameras_and_video.md. | Launch arg. | Whether an 85 % GIL budget per host is acceptable, or to go straight to L1. |
| M3 | **Import diet** in the 9 cv2/numpy processes: lazy-import cv2 only in the code paths that need it (`mower_docking` only while docking; `obstacle_tracker`/`map_server` use numpy mask ops that do not need cv2), drop `tf2_ros` Python (30 MB (dev)) where a static transform is all that is used. | 100-200 (E: 3-4 processes x cv2 45 MB (arm64) + 2 x tf2_ros 30 MB). | None. | 1 day | Low-medium: behaviour unchanged but easy to miss a code path (tests cover it). | Per file. | None. |
| M4 | **Profile `mission_node`** (121 MB anon with no heavy imports) with `tracemalloc`; likely large retained plan/route structures or action-client goal histories. | 40-80 (E) | None. | 0.5 day | Low. | n/a | None. |
| M5 | **rmw switch to Cyclone DDS** (`ros-jazzy-rmw-cyclonedds-cpp`, image rebuild): Cyclone participants idle at ~5-8 MB vs Fast-DDS ~20-25 MB (E; community figures, not measured here: the dev image has no Cyclone). | **600-900** (E) if processes stay separate; only ~100 after M1/M2 (fewer participants). | Cyclone in Jazzy apt has **no shared-memory transport** (iceoryx not packaged); the 2026-10-09 note shows the UDP-loopback path cost ~15 % per Python node and load ~13. Expect CPU to go back up unless composed. Lower discovery overhead. | 1 day + full sim gate + field | Medium-high: every QoS edge case re-validated, foxglove and the bag recorder included. | Env var. | Only worth it instead of M1, not with it. Recommend **M1 first, keep Fast-DDS**. |
| M6 | **Merge nodes with duplicate roles** (behaviour change, each needs a sim gate): `fixed_frame_relay` + `cmd_vel_slew` into `velocity_smoother`'s chain; `stereo_imu` into `stereo_cam`; `activity_gated_lifecycle_manager` into `supervisor`; `fill_light` into `light_controller`. | ~50 per node removed. | Small CPU win. | 0.5 day each | Medium. | Git. | Per node. |

Stage 2 total: ~1 100-1 300 MB (M1 + M2 + M3 + M4), leaving ~1.5 GB available at idle.

### Stage 3: large (compile or rewrite)

Rule of thumb from the measurements: compiling Python (Cython, mypyc, Nuitka) keeps the interpreter,
`rclpy` and the participant, so it removes at most the node's own heap (5-15 MB) and none of the
38 MB floor. **Composition removes the floor; rewriting removes the interpreter.** Rewrite only
what composition cannot fix: nodes that must stay isolated for safety or that burn CPU.

| # | Option | MB saved | CPU / latency | Effort | Risk | Reversible | Owner decides |
|---|---|---|---|---|---|---|---|
| L1 | **Camera pipeline in C++ composable nodes** (`v4l2_cam` x2, `stereo_cam`, `stereo_depth`, `camera_node`) in one `component_container_isolated` with intra-process comms; V4L2 mmap buffers, `cv::Mat` without numpy copies, `image_transport`. **C++**, not Rust: OpenCV, V4L2, cv_bridge, image_transport and the RKNN/RGA APIs are C/C++ first, and `rclrs` has no composition or image_transport. | ~350 (E: 5 x ~80 -> one container ~60) and the +250 MB activity peaks largely vanish (no numpy temporaries). | Removes ~85 % of CPU from the camera chain and the per-frame UDP copies; the main CPU win available. | 1-2 weeks | Medium: new code on the camera path; the vendor's `stereo_ros` is the reference. | Launch arg per node. | Go/no-go after Stage 2 numbers; priority vs feature work. |
| L2 | **Safety/control leaf nodes in Rust (`rclrs`)**: `tilt_monitor`, `slip_detector`, `gps_gate`, `fixed_frame_relay`, `cmd_vel_slew`, `collision_zone_gate`, `nav_lane_watchdog`, `imu_cal`. Pure message-transform logic with pytest suites to port; memory-safe, no GIL, each ~8-12 MB standalone (E from the 25 MB C++ floor minus rclcpp). Rust over C++ here because the logic is small, test-driven, and benefits from Rust's type checking; **but** `rclrs` on Jazzy lacks lifecycle nodes, actions and composition, so anything touching Nav2 lifecycle/actions (`activity_gated_lifecycle_manager`, `supervisor`, `mission_node`) stays Python or goes C++. | ~300 if kept separate (8 x (54 - 12)); only ~60 beyond M1 if those nodes are composed instead. | Deterministic latency for the safety chain; ~1 core freed. | 2-3 weeks incl. tests and sim gates | Medium: a toolchain (cargo + colcon-cargo) in the image; rclrs API churn. | Per node. | Whether Rust enters the stack at all; otherwise do these as C++ components in one container (same saving, known toolchain). |
| L3 | **`mission_node`, `mower_docking`, `obstacle_tracker`, `map_server_node`**: keep Python (logic-heavy, still changing weekly); compose (M1) and diet (M3) instead. | | | | | | No rewrite recommended. |
| L4 | **det_ros / segmentation**: already C++; the 82 MB shmem is the NPU runtime. Smaller input (640 -> 480) or INT8 model cuts the DMA buffers ~30 %. | 20-40 | NPU time down. | 1 day + retrain check | Medium (detection quality). | Model file. | Detection quality vs memory. |

### Recommended order

1. **Q1 zram + Q2 OOM guard + Q3 arena cap + Q5 drop web_video_server + Q4 foxglove limits** (one afternoon, config only): the freezes stop, ~150-300 MB PSS back, ~700 MB effective headroom.
2. **M1 Python hosts, then M3 import diet, M4 mission profile** (one week): ~1 GB back; this is the only item that changes the shape of the problem. Keep Fast-DDS (SHM works, CPU already tuned for it); revisit Cyclone (M5) only if M1 is rejected.
3. **M2 or L1 for the cameras** depending on the GIL result of M2's two-host trial; L1 also buys the CPU headroom the 20 Hz controller needs.
4. L2 only if the owner wants Rust in the stack; the same memory is available from composition in C++.

## 5. Method notes

* PSS per process: `docker exec mower_jazzy sh -c 'awk ... /proc/[0-9]*/smaps_rollup'`; host `ps`
  only gives RSS for root-owned container processes and `pss` is not in the device's `procps`.
* Growth: `VmHWM - VmRSS` from `/proc/<pid>/status`; a proper mowing-time snapshot should be taken
  with the same script during a run (`scripts/` candidate: `mem_snapshot.sh`, not written yet).
* Dev-image micro-benchmarks: `python3 -c "import X"` PSS, and an N-node `SingleThreadedExecutor`
  script with 20 peer participants, `ROS_LOCALHOST_ONLY=1`, Fast-DDS with the mower's profile XML.
  x86-64 numbers; the mower's arm64 numbers are 10-15 MB higher per process (more peers, 64 kB
  pages are not in play: 4 kB kernel).
* Nothing was changed on the mower; no sim gates were run (owner review requested first).

## 6. Implemented on `ws/mem-diet` (M1, M2, M6, M3; 2026-10-22)

Approved by the owner: M1, M2, M6, then M3. Not done (not approved): stage 1 (zram, earlyoom,
malloc env, foxglove trimming, web_video_server), M4. No Rust (owner: stay with C++; L1 next).

### 6.1 `node_host` design (`src/mower_bringup/mower_bringup/node_host.py`)

* **One process per group**, one `rclpy` context = one Fast-DDS participant. Each node runs its
  own **unmodified `main()`** (console_scripts entry point, or the `main` of an installed
  script such as `collision_zone_gate.py`) in its own thread. No node code was changed.
* **Per-node arguments**: the launch side (`node_host_launch.py`) evaluates each node exactly
  as launch_ros would and hands the host the node's standalone `--ros-args` argv (`__node`,
  `__ns`, the generated params files, `-p`, remaps, global `SetParameter`/`SetRemap`). The host
  passes it to that node only (`Node(cli_args=..., use_global_arguments=False)` via an
  `rclpy.node.Node.__init__` shim). Names, namespaces, parameters, topics and services are
  identical; checked in the dev image (16 nodes, `compose_nodes:=true` vs `false`: same graph,
  same parameter values, same `/cmd_vel_slew` publishers/subscribers, diff empty).
  `--log-level` is applied to the node's logger (rcl logging is per process).
* **Executor**: stronger than "MultiThreadedExecutor + per-node callback groups": every node
  gets its **own executor in its own thread** (`rclpy.spin(node)` uses a per-thread
  SingleThreadedExecutor via a `get_global_executor` shim; nodes that build their own
  MultiThreadedExecutor, gui_bridge and coverage_server, keep it). A node's callbacks are
  serialised exactly as in its standalone process, and a slow node never blocks another
  node's wait set; the only shared resources are the GIL and the participant's DDS threads.
  (One rclpy MTE over N nodes would also walk all N nodes' entities on every wake-up: more
  Python CPU, which the mower does not have.)
* **Crash / respawn**: fail fast. If any node's `main()` ends while the host runs (callback
  exception, `sys.exit`, or return), the host logs `node X ENDED ... stopping the whole group`,
  writes a crash record for that node (`<crash_dir>/<node>-*.txt`, traceback), shuts the
  context down so every other node runs its own `finally` (lights off, bag child stopped, ...),
  and exits 1; launch respawns the **whole group** (`respawn` = any member's, delay = the
  largest; all hosted nodes had `respawn=True, respawn_delay=2.0` except bag_recorder 5.0).
  No node can die silently while its peers keep running. SIGINT/SIGTERM: clean exit 0.
  `crash_record` is installed once per host (`node_host_<group>.faulthandler.log`);
  `mower.launch.py`'s process-exit record marks `node_host_control` SAFETY-CRITICAL.
* **Supervisor**: it watches the graph (`get_node_names_and_namespaces`), so the critical list
  `/mower_mcu_driver`, `/cmd_vel_slew`, `/twist_mux`, `/behavior_tree_node` works unchanged
  (cmd_vel_slew is visible under its own name in `node_host_control`). A host restart shows
  as all its members DOWN then UP.
* **Revert**: `compose_nodes:=false` (mower.launch.py, sim.launch.py; default true) = one
  process per node exactly as before. `node_host_standalone:=cameras[,group...]` keeps only
  the listed groups split. A launch file run on its own (e.g. `ros2 launch mower_navigation
  navigation.launch.py`) is never composed. A group with one active member runs it standalone.
* **Logs**: lines are prefixed `[node_host_<group>-N]` by launch, the rclpy logger name still
  says which node. `ps`/`top` show `node_host <spec>`: the spec JSON (in launch's temp dir)
  lists the members; the host prints `hosting N nodes in pid P: ...` at start.

### 6.2 Grouping (for the owner's review)

| Process | Members | CPU on the mower (ps lifetime %, 2026-10-22 10:16, just restarted) | Why together / why apart |
|---|---|---|---|
| **standalone** | `mower_mcu_driver`, `tilt_monitor`, `base_keys` (publishes `/estop_request`), `obstacle_guard` (zero burst on `/cmd_vel_emergency` + `/cutter_off`) | | Safety leaves and the e-stop/emergency path: nothing else may take them down or delay them. |
| **standalone** | `behavior_tree_node` (mission, signal handlers), `mower_docking`, `map_server_node` (55 %) | | Per the brief; map_server alone is half a core. |
| **standalone** | `um960_gps_driver`, `wit_imu_driver` | | Serial drivers with reader threads; a peer's crash would cost an RTK/IMU gap. |
| `node_host_control` | `cmd_vel_slew`, `slip_detector`, `fixed_frame_relay` (M6 pair), `collision_zone_gate`, `nav_lane_watchdog` | 7 + 5 + 16 + 4 + 3 = 35 % | Control-chain peers of the same criticality. Fail-safe: any member crash stops the host, `/cmd_vel` goes silent and mcu_node's 0.5 s `cmd_vel_timeout` stops the wheels; launch restarts it in ~2-3 s (as a standalone cmd_vel_slew crash would). fixed_frame_relay is the costmap's only obstacle path, so it already gated the control chain. Watch: its 16 % shares cmd_vel_slew's GIL. |
| `node_host_localization` | `gps_gate`, `heading_aligner`, `localization_monitor`, `imu_cal` (+ `vio_gate` with vio) | 7 + 21 + 4 + 3 = 35 % | One failure domain: localisation quality. A crash = ~3 s without gated fixes / heading alignment, which the EKFs and mission already tolerate (heading_aligner re-aligns). |
| `node_host_perception` | `det_range`, `obstacle_tracker` | 3 + 12 = 15 % | Perception helpers downstream of det_ros / stereo_depth; obstacle_guard stays out (emergency path). |
| `node_host_housekeeping` | `supervisor`, `activity_gated_lifecycle_manager` (M6), `alert_node`, `bag_recorder`, `light_controller`, `fill_light` (M6), `mow_recorder` (if on) | 7 + 3 + 6 + 7 + 5 + 3 = 31 % | Monitoring/UX. activity_gated_lifecycle is restart-safe by design (reconciles with the manager's `is_active`). bag_recorder's `ros2 bag record` child stays a separate process (PDEATHSIG is tied to the thread that spawned it, which lives as long as the host). |
| `node_host_gui` | `gui_bridge`, `coverage_server`, `cmd_vel_ws_relay` | 14 + 2 + 3 = 19 % | GUI-facing and mission utilities; the WebSocket relay (untrusted input) is kept out of the control host. |
| `node_host_cameras` (M2) | `left_oa_camera`, `right_oa_camera`, `stereo_cam`, `stereo_imu` (M6), `stereo_depth`, `mower_cameras` (rear) | 8 + 8 + 6 + 18 + 4 + 3 = 47 % idle; ~1.5 cores when all stream (2026-10-06: rear 29 %, each OA 44 %) | M2. Its environment is cameras.launch.py's scoped camera DDS profile (stereo_imu now also uses it). **GIL risk, field check required** (6.4). |

M6 was done as **co-location** (`fixed_frame_relay` with `cmd_vel_slew`, `stereo_imu` with
`stereo_cam`, `activity_gated_lifecycle_manager` with `supervisor`, `fill_light` with
`light_controller`), not by merging code: same node names, topics, services and parameters,
no behaviour change, the memory saving is the same (one participant + interpreter per node).
