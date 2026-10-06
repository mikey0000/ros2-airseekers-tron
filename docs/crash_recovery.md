# Crash recovery and crash recording

Owner question: "We need to be able to recover from crashes and record them to fix; what is
the correct ROS 2 way of doing this?"

Short answer, in the order ROS 2 Humble does it:

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
alone is not enough. Humble's `nav2_lifecycle_manager` handles the rest. The installed
`libnav2_lifecycle_manager_core.so` on the mower contains `attempt_respawn_reconnection`,
`bond_respawn_max_duration` and `checkBondRespawnConnection`:

1. Each managed server holds a bond with the manager. When the server dies, its heartbeat
   stops and after `bond_timeout` (10 s in `nav2_params.yaml`) the manager logs "Have not
   received a heartbeat" and resets the whole stack. All servers deactivate, so
   `/navigate_to_pose` and `/follow_path` abort and the mission sees an action failure.
2. With `attempt_respawn_reconnection: true` (now set in `nav2_params.yaml` and in
   `navigation.launch.py`), the manager polls until every server is reachable again, for up
   to `bond_respawn_max_duration` (30 s, up from the Humble default of 10 s because the
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
- Analysis: `gdb` and `elfutils` are added to `docker/Dockerfile.humble`; they become
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

Humble's `ros2 bag record` has no `--snapshot-mode` (that arrived in Iron). The supervisor
instead keeps the last **120 s** of these topics in RAM as serialized CDR bytes, with no
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
- on the host, `docker logs --tail 500 mower_humble`.

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
| `src/mower_control/mower_control/crash_record.py` | faulthandler and excepthook helper |
| `src/mower_control/test/test_{blackbox,supervisor_logic,crash_record}.py` | unit tests |
| `src/mower_mission/…/mission_{node,fsm}.py` | `/supervisor/status` leads to an emergency cause; tests in `test_mission_fsm.py` |
| all launch files, `mower_navigation/config/nav2_params.yaml` | respawn, lifecycle re-bond, OnProcessExit, supervisor |
| `scripts/stack_entry.sh` | new container command (ulimit, env, retention loop, shm_gc, launch) |
| `scripts/ros_log_cleanup.sh`, `scripts/crash_report.sh` | retention and bundling |
| `docker/docker-compose.yml` | `ROS_LOG_DIR`, `MOWER_CRASH_DIR`, `PYTHONFAULTHANDLER`, `ulimits.core`, `command` |
| `docker/Dockerfile.humble` | gdb and elfutils |
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
   `cd /userdata/ros2_stack && DOCKER_BUILDKIT=0 docker compose -f docker/docker-compose.yml build mower_humble`
3. **Recreate the container** (compose env, ulimits and the `stack_entry.sh` command):
   `cd /userdata/ros2_stack && docker compose -f docker/docker-compose.yml up -d --force-recreate mower_humble`
   Then check:
   `docker exec mower_humble bash -c 'ulimit -c; echo $ROS_LOG_DIR'` should print
   `unlimited` and `/userdata/ros2/log`, and `ls /userdata/ros2/log` should show the new
   launch dir.

Until step 3, the code changes (respawn, supervisor, black box, Python crash records,
mission guard) are active after a plain `docker restart mower_humble`. ROS logs stay in
the container and cores still go to systemd-coredump until steps 1 and 3 are done.
