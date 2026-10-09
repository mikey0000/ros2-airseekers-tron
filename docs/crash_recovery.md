# Crash recovery and crash recording

Owner question: "We need to be able to recover from crashes and record them to fix; what is
the correct ROS 2 way of doing this?"

Short answer, in the order ROS 2 Jazzy does it:

1. **Recover**: `launch_ros` restarts processes (`respawn=True, respawn_delay`). Lifecycle
   nodes (Nav2) are put back into ACTIVE by `nav2_lifecycle_manager` using bonds
   (`bond_timeout`, `attempt_respawn_reconnection`).
2. **Detect and surface**: a supervisor watches the ROS graph and publishes
   `diagnostic_msgs/DiagnosticArray` on `/diagnostics` (the standard ROS "node X is dead"
   channel; the GUI already turns ERROR entries into alerts) plus a latched `/supervisor/status`.
3. **Fail safe**: anything that commands motion must stop by itself when an upstream node
   disappears. In this stack that is three layers: the MCU driver's `/cmd_vel` timeout, the
   `cmd_vel_slew` watchdog and `/motion_enabled` gate, and the mission's emergency guard.
4. **Record**: core dumps for C++ (`kernel.core_pattern` + `ulimit -c`), Python tracebacks
   (`faulthandler` + `sys.excepthook`), persistent ROS logs (`ROS_LOG_DIR`), and a rosbag2
   "black box" of the last 120 s, written whenever something goes wrong.

## 1. Recovery: respawn on every node

Every `Node(...)` in every launch file now has `respawn=True, respawn_delay=2.0`:

| Launch file | Nodes |
|---|---|
| `launch/bringup.launch.py` | um960, mcu_node, wit_node, bumper_controller, base_keys, light_controller, fill_light (these already had it) |
| `launch/mower.launch.py` | robot_state_publisher, static_map_odom, cmd_vel_slew, slip_detector, imu_cal, gui_bridge, foxglove_bridge, det_range, **supervisor** (new) |
| `launch/nav2.launch.py` | navsat_transform, robot_state_publisher, gps_gate, heading_aligner, ekf_node, vio_gate |
| `launch/cameras.launch.py`, `camera.launch.py`, `vio.launch.py` | v4l2 cameras, stereo_cam, stereo_imu, stereo_depth, web_video_server; VIO nodes |
| `mower_teleop/teleop.launch.py` | twist_mux, cmd_vel_ws_relay |
| `mower_mission/mission.launch.py` | behavior_tree_node (mission) |
| `mower_docking/docking.launch.py` | docking_server |
| `mower_map/map_server.launch.py` | map_server_node |
| `mower_coverage_bridge/coverage_bridge.launch.py` | mower_coverage_node, coverage_action_server |
| `mower_vision/perception.launch.py` | det_ros / det_ros_cpp, seg_ros, obstacle_guard |
| `mower_navigation/navigation.launch.py` | controller/planner/behavior servers, bt_navigator, velocity_smoother, both lifecycle managers |

Respawn is launch-only. If `ros2 launch` itself dies, the container's main process is gone
and Docker's `restart: unless-stopped` restarts the whole container, which is the outer
recovery layer.

A respawned mission starts in IDLE and never resumes a mission by itself. A respawned
`mcu_node` starts with the cutter off.

### Nav2 lifecycle nodes

Nav2 servers are lifecycle nodes. A respawned server comes back UNCONFIGURED, so respawn
alone is not enough. Nav2's `nav2_lifecycle_manager` handles the rest. The installed
`libnav2_lifecycle_manager_core.so` on the mower contains `attempt_respawn_reconnection`,
`bond_respawn_max_duration` and `checkBondRespawnConnection`:

1. Each managed server holds a bond with the manager. When the server dies, its heartbeat
   stops and after `bond_timeout` (10 s in `nav2_params.yaml`) the manager logs "Have not
   received a heartbeat" and resets the whole stack. All servers deactivate, so
   `/navigate_to_pose` and `/follow_path` abort and the mission sees an action failure.
2. With `attempt_respawn_reconnection: true` (now set in `nav2_params.yaml` and in
   `navigation.launch.py`), the manager polls until every server is reachable again, for up
   to `bond_respawn_max_duration` (30 s, up from the Nav2 default of 10 s because the
   RK3588 is slow to start the servers). It then runs configure and activate again
   ("Successfully re-established connections from server respawns, starting back up").
3. Limitation: if `lifecycle_manager_navigation` itself crashes and respawns, its autostart
   tries to configure servers that are already ACTIVE. That transition is rejected and the
   stack stays up but unbonded. The supervisor reports the restart. Recovery then needs a
   container restart or `ros2 service call /lifecycle_manager_navigation/manage_nodes
   nav2_msgs/srv/ManageLifecycleNodes "{command: 3}"` (RESET) followed by `{command: 0}`
   (STARTUP).

## 2. Safety when a dependency dies

| What dies | What stops the wheels | Where |
|---|---|---|
| any lane producer (Nav2, teleop relay, docking) | `twist_mux` lane timeout; then the `cmd_vel_slew` watchdog (`cmd_timeout` 0.5 s without `/cmd_vel_raw` decelerates to zero) | `mower_teleop/config`, `cmd_vel_slew.py` |
| `twist_mux` | `cmd_vel_slew` watchdog (no `/cmd_vel_raw`) | `cmd_vel_slew.py` |
| `cmd_vel_slew` | **MCU driver zero-on-timeout**: after a non-zero command, 0.5 s of `/cmd_vel` silence (`cmd_vel_timeout`) sends `stop_frames` (3) zero SpeedData frames 0.1 s apart | `mcu_node.py` (`_speed_tick`, docstring "SpeedData TX policy") |
| the mission | `cmd_vel_slew` motion gate: the latched `/motion_enabled` must be true **and** fresher than 3 s (`MOTION_ENABLE_MAX_AGE`, the mission re-asserts it every 1 s); stale means forced zero and the held command is dropped (`gate_status` "/motion_enabled stale") | `cmd_vel_slew.py` `MotionGate` |
| `gui_bridge` (publishes `/hardware_bridge/emergency`) | the mission enters EMERGENCY when `/hardware_bridge/emergency` is silent for more than 2 s (`emergency_timeout_s`): blade off, goals cancelled, zero burst, `/motion_enabled` false | `mission_fsm._emergency_cause` |
| `mcu_node` | the host can no longer command anything. The MCU's own heartbeat failsafe is **not characterised yet** (heartbeat period 100 ms confirmed; the firmware timeout is unknown, see `mcu_node.py` "Known gaps"). The mission goes to EMERGENCY through the supervisor (next row), and the respawned driver starts with the cutter off. | open item |
| a **critical node** (`/mower_mcu_driver`, `/cmd_vel_slew`, `/twist_mux`) vanishes | the supervisor lists it in `critical_down`; the mission treats that as an emergency cause (`node down: …`): EMERGENCY, blade off, cancel, zero burst, motion gate closed. It is not resumed automatically once the node is back. | `mission_node._on_supervisor`, `mission_fsm.Inputs.critical_nodes_down` |

Unit tests in `mower_mission/test/test_mission_fsm.py` cover this:
`test_critical_node_down_mid_mission_is_a_safe_stop` and
`test_critical_node_down_refuses_start`. The motion-gate staleness is covered in
`mower_control/test/test_motion_gate.py`.

A critical node that never started (its launch group disabled, e.g. `teleop:=false` on a
bench) is reported as DOWN ("never started") after `startup_grace_s` (90 s). It is left out
of `critical_down`, so a reduced launch does not latch EMERGENCY.

## 3. Detection: supervisor (`mower_control/supervisor.py`)

Started by `mower.launch.py` (`supervisor:=true`, respawn).

- **Liveness**: polls `get_node_names_and_namespaces()` at 1 Hz
  (`mower_control/supervisor_logic.py`). A node present for at least 10 s is "adopted". An
  adopted node missing for 2.5 s or more is DOWN; when it reappears it is UP and its restart
  is counted. CLI and probe nodes (`/_ros2cli_*`, `/launch_ros_*`, names starting with `_`,
  anything shorter-lived than 10 s) are ignored. The supervisor uses graph polling, not
  per-node `diagnostic_updater` heartbeats: it needs no change in any node and also covers
  third-party nodes (Nav2, twist_mux, robot_localization).
- **`/supervisor/status`** (`std_msgs/String`, JSON, latched, published on change and every
  5 s): `{"ok", "down", "critical_down", "restarts", "nodes": {name: {alive, restarts,
  down_for_s, reason, critical}}}`.
- **`/diagnostics`**: `supervisor: nodes` (OK, or ERROR listing the down nodes) and one
  `supervisor: /<node>` entry per node that has ever gone down (ERROR while down, OK
  "running (restarted N x)" afterwards).
- **GUI**: no GUI change was needed. The MowgliNext GUI merges `/diagnostics` entries by
  name (`useDiagnostics.ts`) and raises level ERROR or WARN as alerts on the existing
  alert/fault banner (`diagnosticsAlerts.ts`, grouped under "supervisor: …"), with its
  existing en/fr strings. The Logs page lists containers and `/rosout` only; it has no file
  listing, so there is no crash-file link in the GUI. Use `scripts/crash_report.sh`.
- **Events log**: `/userdata/ros2/crashes/events.jsonl` (down, up and dump events).

### OnProcessExit crash records

`mower.launch.py` registers one `OnProcessExit` handler for every process in the launch,
including processes from included launch files. A non-zero exit outside launch shutdown
appends a structured record to `/userdata/ros2/crashes/process_exits.jsonl`:

```json
{"time": "...", "source": "launch", "event": "process_exit", "process": "obstacle_guard-31",
 "executable": "obstacle_guard", "pid": 1234, "returncode": -9, "signal": "SIGKILL",
 "core_dump_expected": false, "safety_critical": false, "respawn": true}
```

It also logs a `CRASH [SAFETY-CRITICAL] …` line for mcu_node, cmd_vel_slew, twist_mux and
the mission.

## 4. Recording

All recording goes to `/userdata/ros2/crashes` (`$MOWER_CRASH_DIR`).

### Python: `mower_control/crash_record.py`

Every Python node `main()` calls `crash_record.install('<node>')` (soft import, so packages
do not hard-depend on mower_control). The nodes covered are mission, gui_bridge,
cmd_vel_slew, slip_detector, imu_cal, mcu_node, docking, map_server, det_range, gps_gate,
heading_aligner, vio_gate, cmd_vel_ws_relay, obstacle_guard and supervisor.

- `faulthandler` writes to `<node>.faulthandler.log`, appended per start. On SIGSEGV,
  SIGABRT, SIGBUS, SIGFPE or SIGILL inside an extension (rclpy, cv2, rknnlite) it records
  every thread's Python stack. `PYTHONFAULTHANDLER=1` (compose) also covers Python
  processes that do not call `install`.
- `sys.excepthook` and `threading.excepthook` write an uncaught exception (the one that ends
  `rclpy.spin`) to `<node>.<YYYYmmdd-HHMMSS>.txt`: node, time, pid, thread, argv, deployed
  revision and traceback. The hook then chains to the default hook, so the traceback still
  reaches the launch log.
- Exceptions that nodes already catch and log ("callback raised" in `sub_pump`) are not
  crashes. They reach `/rosout` and therefore the black box.

### C++: core dumps

- The mower's kernel currently pipes cores to `systemd-coredump`, which stores them on the
  nearly full root filesystem. The new host file `docker/host/91-ros2-cores.conf` sets
  `kernel.core_pattern = /userdata/ros2/crashes/core.%e.%p.%t`. The container shares the
  host kernel, so this pattern is global, and `/userdata/ros2` is mounted at the same path
  inside the container, so the path resolves for both host and container processes.
  **Not applied yet: see the hand-off below.**
- The size limit is set per process: compose `ulimits: core: -1`, plus `ulimit -c unlimited`
  in `scripts/stack_entry.sh`.
- `scripts/ros_log_cleanup.sh` keeps cores for 14 days and only the newest 5.
- Analysis: `gdb` and `elfutils` are added to `docker/Dockerfile.jazzy`; they become
  available after an image rebuild:
  `gdb /work/install/det_ros_cpp/lib/det_ros_cpp/det_ros_cpp /userdata/ros2/crashes/core.det_ros_cpp.<pid>.<t> -ex bt`.
- **Symbols trade-off.** On the mower, `mower_coverage` is built `Release` with `-g0`, a
  workaround for 4 GB of RAM while compiling Fields2Cover. `det_ros_cpp` defaults to
  Release with no `-g`, and `mowgli_nav2_plugins` has an empty build type (no `-O`, no
  `-g`). A core from these binaries gives function names from the dynamic symbol table at
  best, and no line numbers. Recommended at the next planned C++ rebuild (never inside the
  running stack):
  `--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS="-g1"`.
  `-g1` is line tables only: about 1.3x compile memory instead of the 2 to 3x of `-g`.
  Optionally split the debug info so runtime binaries stay small:
  `objcopy --only-keep-debug X X.debug && objcopy --strip-debug --add-gnu-debuglink=X.debug X`.
  The `.debug` files live in `/work/install` (on `/userdata`), so they cost no image space.
  `mower_coverage` may need to stay at `-g0` if `-g1` runs out of memory, so try it first
  with `--parallel-workers 1`.

### ROS logs

Compose sets `ROS_LOG_DIR=/userdata/ros2/log`. Today the logs are in the container's
`/root/.ros/log` (839 dirs, 68 MB), which is lost when the container is recreated. Each
launch writes a dated dir containing `launch.log` and per-node rosout files.
`scripts/ros_log_cleanup.sh` runs at container start and every 6 h. It keeps logs for
14 days and at most 500 MB, oldest first, and never removes the running launch's dir. It
keeps crash dirs for 30 days and at most 2 GB, and trims the `*.jsonl` and faulthandler
logs to their last 5000 lines.

### Black box: rosbag2 snapshot (`mower_control/blackbox.py`, run by the supervisor)

`ros2 bag record --snapshot-mode` (available on Jazzy's rosbag2 0.26.11) is not used: the
supervisor instead keeps the last **120 s** of these topics in RAM as serialized CDR bytes, with no
deserialization cost, through the `sub_pump` raw path. Rates are capped per topic and the
whole buffer is capped at 24 MB, evicting the oldest messages first:

`/cmd_vel` and `/cmd_vel_raw` (20 Hz); `/cmd_vel_nav`, `_teleop`, `_docking`, `_bumper`
and `_emergency` (10 Hz); `/odometry/filtered_map` (5 Hz); `/mower_base/status`,
`/behavior_tree_node/high_level_status`, `/hardware_bridge/emergency`, `/motion_enabled`
and `/map_server_node/lethal_boundary_violation` (2 Hz); `/obstacle_policy` and
`/cmd_vel_slew/gate_status` (5 Hz); `/fix_status` (1 Hz); `/diagnostics` and `/rosout`
(all messages). The list is the `blackbox_topics` parameter (`topic|type|max_hz`).

**Dump triggers** (rate limited to one every 30 s; the manual service bypasses the limit):

- a node goes DOWN;
- `/hardware_bridge/emergency` `active_emergency` rising edge;
- `/map_server_node/lethal_boundary_violation` rising edge;
- a mission `/rosout` line containing "docking failed", "undock failed" or
  "BOUNDARY_EMERGENCY_STOP";
- `ros2 service call /supervisor/dump std_srvs/srv/Trigger`.

Each dump creates `/userdata/ros2/crashes/<YYYYmmdd-HHMMSS>_<reason>/` containing:

- `bag/`: a sqlite3 rosbag2 (`ros2 bag info` / `ros2 bag play`; verified on the mower);
- `reason.json`: trigger, detail, deployed revision, message count, buffer stats and the
  supervisor status at the time;
- `launch_log_tail.txt`: the last 500 lines of the current `launch.log`, which is the same
  content as the container stdout;
- copies of the `*.txt` crash records from the last 10 minutes.

The 50 newest dump dirs are kept.

### Rolling bag ring on disk (`mower_control/bag_recorder.py`, 2026-10-09)

The black box above holds 120 s of ~15 rate-capped topics, and `mow_recorder` only records
while a mission runs, so the minutes before an unflagged incident, or anything docked or in
manual drive, were lost. The 2026-10-08 tuning bags had to be recorded by hand with `ros2 bag record`.
`bag_recorder` (launch arg `bag_recorder`, default on; `bag_max_gb` 3.0, `bag_min_free_gb`
3.0) records all the time:

- **Recorder**: a supervised `ros2 bag record` child. It is the C++ recorder, with no
  deserialization and no Python per message. It records a curated set of 54 low-rate topics
  (`DEFAULT_TOPICS`, ~140 msg/s parked):
  - the cmd_vel lanes, `/cmd_vel`, slew gate/shape status, `/estop_request`, `/motion_enabled`,
    `/mcu/{measured,commanded}_speed`;
  - `/odometry/filtered` (20 Hz), `/odometry/filtered_map` (10 Hz), `/tf_static`, tilt,
    heading-aligner and localization status;
  - `/fix`, `/fix_status`, `/ntrip/status`;
  - mission status and `/mission/*`, `/obstacle_policy`, `/ai/det/detections_ranged`;
  - Nav2 plans, `/local_costmap/costmap` (1 Hz, 120x120) and `/behavior_tree_log`;
  - bumper/lift (`/mower_sensor_info`), battery, `/hardware_bridge/*`, dig stall, stuck events,
    docking, lethal boundary, boundary status and terrain summary;
  - `/diagnostics`.

  **CPU (2026-10-09)**: the first version recorded 73 topics (554 msg/s with `/imu/data_aligned`
  and `/mower_base/status` at 100 Hz, `/odom` at 50 Hz, `/tf` 30 Hz, `/rosout`) and its
  `ros2 bag record` measured **46-53 % of one core** on the mower (per-thread `/proc` deltas). The
  cost is per delivered message (Fast DDS UDP receive, no SHM in this container, plus one rosbag2
  executor wake that walks every subscription), not bytes; compression mode, the sqlite preset
  and cache size made no measurable difference. A dev-box benchmark replaying the measured
  topic rates and sizes (9 publisher processes + 50 padding nodes, Fast DDS UDP-only profile)
  gave: old set 25-29 % x86 parked / 31 % while driving; curated set **7.2 % parked / 9.1 %
  driving** (x86). Scaled by the measured mower/x86 ratio (~1.7) that is **~12 % parked,
  ~15 % driving** on the RK3588. The dropped topics and what still covers them are listed above
  `DEFAULT_TOPICS`; add any back with `extra_topics` (a 100 Hz topic costs ~8 % of a core).
  Topic discovery polls every 5 s (`polling_ms`; 1 s cost ~1 %), so a topic appearing late is
  picked up within 5 s.

  Extra topics go in the `extra_topics` parameter. Recording goes to
  `/userdata/ros2/bags/<YYYYmmdd-HHMMSS>/`, with a new session dir on every (re)start. The
  child is restarted with a 2 to 60 s backoff, gets SIGINT on shutdown (clean close and
  `metadata.yaml`), and gets PR_SET_PDEATHSIG if the node dies.
- **Format** (rosbag2 0.15.17; sqlite3 is the only storage in the image, because mcap would need
  `ros-jazzy-rosbag2-storage-mcap` and an image rebuild):
  - 60 s splits, using the sqlite3 `resilient` preset (WAL, synchronous=NORMAL), which survives
    a power cut.
  - `--max-cache-size` 1 MiB. The default 100 MiB double buffer would hold about 10 min in RAM.
  - File-mode zstd. Each closed split is compressed once by one rosbag2 thread. This costs about
    tens of ms of one core per minute, gives 3-5x on this data and cuts eMMC writes by the same
    factor. Per-message compression would cost more CPU on 100-byte messages for little gain.
  - Measured 2026-10-09: the old 73-topic set was 181 KB/s before compression (`/plan` 49 KB/s
    at 0.9 Hz, `/odom` 37 KB/s, IMU 33 KB/s); in the benchmark the zstd ring grew ~12 KB/s with
    it and ~3.4 KB/s with the curated set (~300 MB/day). See `/bag_recorder/status`
    `ring_bytes` on the mower.
- **Pruning**: every 10 s, delete the oldest closed splits while the ring is over `max_total_gb`
  or /userdata has less than `min_free_gb` free. These are never deleted:
  - the split being written;
  - a split being compressed;
  - split 0 of the current session, which holds `/tf_static` and the latched topics (rosbag2 has
    no `--repeat-transient-local`);
  - splits an incident pin still waits for.

  If the floor cannot be met, recording pauses and resumes at floor + 0.5 GB.
- **Incident pins**: `/userdata/ros2/incidents/<ts>_<reason>/`.
  - Triggers:
    - `/mission/incident` kinds `subpath_failed`, `transit_abort`, `follow_abort` and `stuck`
      (`detour` is routine);
    - the mission state entering `EMERGENCY`;
    - `/hardware_bridge/emergency` rising;
    - a new supervisor dump dir in `$MOWER_CRASH_DIR` (crash, node down, lethal boundary,
      docking failed);
    - `ros2 service call /bag_recorder/pin std_srvs/srv/Trigger`.
  - There is a 30 s cooldown between pins. `mission_fsm.py` is unchanged.
  - What a pin holds:
    - the 2 previous closed splits plus split 0, linked at once;
    - the current split and 1 more, linked as each one closes. The pin therefore covers about
      2-3 min before and 1-2 min after the trigger.
  - The files are **hard links** on the same filesystem: no copy, and no space until the ring
    prunes its own name.
  - Pins have their own retention: 50 dirs and 1 GB. Each pin has an `incident.json` with the
    reason, detail and files.
- **Reading**: each split is a complete sqlite3 bag once decompressed. The image has no
  `zstd` CLI. On the dev box:
  ```
  ./scripts/deploy_to_mower.sh bags            # incidents -> ../bags/mower, zstd -d
  ./scripts/deploy_to_mower.sh bags ring       # the whole ring (rsync -aH, wal/shm skipped)
  ros2 bag info ../bags/mower/incidents/<pin>/<session>__<session>_3.db3
  ros2 bag play <that .db3>                    # or: ros2 bag reindex <dir> for a merged view
  ```
  A session's `metadata.yaml` lists every split, including pruned ones, so use the per-file
  paths and not the session dir.
- **On/off at runtime**: `ros2 service call /bag_recorder/enable std_srvs/srv/SetBool
  "{data: false}"` (or the GUI: Settings > Advanced > Data recorder). Off stops the child with
  SIGINT (last split closed and compressed) and does not restart it; pruning, pins of what is
  already recorded and the status keep running. The choice persists in
  `/userdata/ros2/bag_recorder.json` (`state_file`; `enabled_default` when it does not exist).
  `/bag_recorder/get_state` (Trigger) returns the status JSON (the GUI uses it).
- **Per-mow bags (replace `mow_recorder`)**: the separate Python `mow_recorder` measured
  ~35 % of a core while a mission ran (raw subscriptions to 100 Hz IMU / 50 Hz odom). It is now
  off by default (launch arg `mow_recorder`, default false). Instead bag_recorder pins every
  split from 1 split before the mission left IDLE/IDLE_DOCKED/CHARGING to the split being
  written when it came back (5 s idle) into `/userdata/ros2/mows/<YYYYmmdd-HHMMSS>/`: hard
  links + `incident.json` (reason `mow`, start, end, files). Retention 10 mows / 2 GB
  (`max_mows`, `mow_max_gb`). These mows have the curated topic set (no 100 Hz IMU, no `/odom`);
  `mow_recorder:=true` restores the old recorder and turns the mow pins off. Nothing is pinned
  while recording is off.
- **Status**: latched `/bag_recorder/status` (JSON) reports:
  - enabled, recording, session, paused_low_disk, mow (dir being pinned), mow_pins, topics;
  - ring_bytes, free_bytes;
  - restarts, pending_pins, suppressed_triggers.

  The child's own output goes to `/userdata/ros2/bags/recorder.log`, which is truncated past
  5 MB.
- Disk budget on /userdata, worst case: ring 3 GB, pins 1 GB, mows 2 GB (mow pins),
  crashes 2 GB, and the floor of 3 GB kept free.

## 5. Retention and access

`scripts/crash_report.sh [dump-dir] [-o out.tar.gz]` runs on the mower host or in the
container. It bundles:

- the newest dump dir (or the one given);
- `events.jsonl` and `process_exits.jsonl`;
- Python crash records and faulthandler logs from the last 2 days;
- the core file list (cores themselves are too large; copy one explicitly);
- the latest ROS log dir (files over 20 MB truncated to their tail);
- `versions.txt`: `DEPLOYED_REV`, git HEAD if the tree is a git checkout, image id,
  container start time and restart count;
- on the host, `docker logs --tail 500 mower_jazzy`.

The tarball is written to `/userdata/ros2/crashes/crash_report_<ts>.tar.gz`:

```
ssh airseekers@192.168.1.105 /userdata/ros2_stack/scripts/crash_report.sh
scp airseekers@192.168.1.105:/userdata/ros2/crashes/crash_report_*.tar.gz .
```

The deployed tree has no `.git`, because rsync excludes it. `scripts/deploy_to_mower.sh
sync` now writes `DEPLOYED_REV` with the git HEAD and dirty count, or, outside git, a
sha256 over `src/ launch/ config/ scripts/` plus the sync time. Crash records and
`reason.json` report this file.

## 6. Files

| File | What |
|---|---|
| `src/mower_control/mower_control/supervisor.py` | supervisor node (liveness, diagnostics, black box, dump service) |
| `src/mower_control/mower_control/supervisor_logic.py` | pure liveness tracker |
| `src/mower_control/mower_control/blackbox.py` | ring buffer and rosbag2 writer |
| `src/mower_control/mower_control/bag_recorder.py`, `bag_ring.py` | always-on on-disk bag ring, pruning, incident pins (`test_bag_ring.py`, `test_bag_recorder_process.py`, `mower_bringup/test/test_bag_recorder_launch.py`) |
| `src/mower_control/mower_control/crash_record.py` | faulthandler and excepthook helper |
| `src/mower_control/test/test_{blackbox,supervisor_logic,crash_record}.py` | unit tests |
| `src/mower_mission/…/mission_{node,fsm}.py` | `/supervisor/status` leads to an emergency cause; tests in `test_mission_fsm.py` |
| all launch files, `mower_navigation/config/nav2_params.yaml` | respawn, lifecycle re-bond, OnProcessExit, supervisor |
| `scripts/stack_entry.sh` | new container command (ulimit, env, retention loop, shm_gc, launch) |
| `scripts/ros_log_cleanup.sh`, `scripts/crash_report.sh` | retention and bundling |
| `docker/docker-compose.yml` | `ROS_LOG_DIR`, `MOWER_CRASH_DIR`, `PYTHONFAULTHANDLER`, `ulimits.core`, `command` |
| `docker/Dockerfile.jazzy` | gdb and elfutils |
| `docker/host/91-ros2-cores.conf` | host sysctl for the core pattern |

## 7. Hand-off: host and container changes (not applied by the agent)

Apply them in this order. Each step restarts or recreates the stack, so run it only while
the mission is IDLE, IDLE_DOCKED or CHARGING.

1. **Host core pattern** (as root on the mower; affects the whole host):
   ```
   sudo install -m 644 /userdata/ros2_stack/docker/host/91-ros2-cores.conf /etc/sysctl.d/
   sudo mkdir -p /userdata/ros2/crashes /userdata/ros2/log
   sudo sysctl -p /etc/sysctl.d/91-ros2-cores.conf
   cat /proc/sys/kernel/core_pattern   # -> /userdata/ros2/crashes/core.%e.%p.%t
   ```
   Revert with `sudo rm /etc/sysctl.d/91-ros2-cores.conf && sudo sysctl --system`. Note
   that systemd may re-apply its own `50-coredump.conf`; the `91-` prefix sorts later and
   wins.
2. **Image** (gdb and elfutils; optional, only needed to read cores on the device):
   `cd /userdata/ros2_stack && DOCKER_BUILDKIT=0 docker compose -f docker/docker-compose.yml build mower_jazzy`
3. **Recreate the container** (compose env, ulimits and the `stack_entry.sh` command):
   `cd /userdata/ros2_stack && docker compose -f docker/docker-compose.yml up -d --force-recreate mower_jazzy`
   Then check:
   `docker exec mower_jazzy bash -c 'ulimit -c; echo $ROS_LOG_DIR'` should print
   `unlimited` and `/userdata/ros2/log`, and `ls /userdata/ros2/log` should show the new
   launch dir.

Until step 3, the code changes (respawn, supervisor, black box, Python crash records,
mission guard) are active after a plain `docker restart mower_jazzy`. ROS logs stay in
the container and cores still go to systemd-coredump until steps 1 and 3 are done.
