> Reconstruction date: 2026-10-05. Produced by a Claude Opus subagent (read-only: BT XML, msg/srv/proto files, `strings`/`nm` on the OTA binaries; no vendor C++ for the mission layer exists in `src/`).
> Point-in-time snapshot of the *vendor* ROS 1 stack, used as the specification for Phases B and C. See `docs/STATUS.md` for how it maps onto the ROS 2 rebuild.

# Airseekers Tron: vendor mission layer, reconstructed for the ROS 2 rebuild

**How this was built and what to trust.** The C++ for `mower_logic`, `mower_bt_nodes`, `mower_map` and `mower_charge` is not in `src/`. Those source dirs are empty, for example `src/mower_task/mower_bt_nodes/src/mower_bt_nodes/bt_*` and `src/mower_task/mower_logic/src/{db,task}`. The compiled binaries are in the OTA tree, which I'll call `OTA=/home/michael/git/airseekers-decompile/ota_v1.3.27-rc.1-tron+20260713/devel`. Everything below comes from three sources:
- the BT XML (exact),
- `.msg`/`.srv`/`.proto` files (exact),
- `strings` and `nm -DC` on the binaries. These give exact names and log text, but the control flow is inferred.

Where I say "(inferred)", read it that way. The binaries are:
- `$OTA/mower_logic/lib/libmower_logic_{task,net,db}.so`, plus the `mower_logic_master_node` executable
- `$OTA/mower_bt_nodes/lib/libmower_bt_nodes.so`
- `$OTA/mower_map/lib/libmower_map.so`
- `$OTA/mower_charge/lib/mower_charge/mower_charge`
- `$OTA/mower_mqtt/lib/libmower_mqtt.so`

I made no changes to any files and did not touch the mower.

---

## 0. Process architecture

- **`/mower_logic` node** runs `mower_logic_master_node` (`src/mower/launch/mower_logic.launch:12`). It links `libmower_logic_task.so`, `_db.so`, `_net.so`, `libmower_map.so` and `libbehaviortree_cpp`. All of this runs in one process:
  - **`TaskManager`** (`src/task_manager.cpp`): the singleton task state machine, timed schedules, auto-charge and key handling. It loads its BT XMLs from `$(find mower_bt_nodes)/behaviors_master/*.xml` (log string `"Loading XML: %s"`).
  - **Task classes**, all subclasses of `BaseTask` (`src/task/*.cpp`):
    - `MoweTask` runs tree `MasterMainTree`
    - `DockingTask` runs `GoDocking`
    - `UndockingTask` runs `Undocking`
    - `MappingTask` runs `CreateMap`
    - `ExploreMappingTask` runs `ExploreCreateMap`
  - **`RobotDataHandler`** (`src/robot_data.cpp`): all ROS publishers, subscribers and service clients.
  - **`FaultManager`** (`src/fault_manager.cpp`): notices and the alarm bitmask.
  - **`IotManager`** (`src/iot_manager.cpp`): the `/iot/ctrl` service that `mower_mqtt` calls.
  - **`APITask`/`APIMapping`** (`src/net/robot_api/api_{task,mapping}.cpp`): the Crow HTTP server.
  - **`WebSocketServer`** (`src/net/base/websocket_server.cpp`).
  - **`RobotDatabase`**: SQLite.
- **`/navigation`** owns the controller services: `/controller/ctrl`, `SetTypePath`, `SetTaskParams`, `GetNavStatus`, `CheckStatus`. That node's internals are out of scope here.
- **`/mower_planning`** owns `/planning/*`.
- **`/mower_charge`** owns `/controller/dock/*` (see the `node_info_mower_charge.txt` snapshot).
- **`/base_driver_node`** owns `/cutter_control`, `/charging`, `/clear_estop`, `/enable_bumper`, `/poweroff` and `/reset_odom` (`ros2_port_handoff/10_ros_live_snapshot/node_info_base_driver_node.txt`).

---

## 1. Task lifecycle

### 1.1 Internal state machine

**Task state strings** (`BaseTask::getTaskStrState`, `libmower_logic_task.so`): `idle | running | paused | finished | stopped | failed | unknown`.

**Task type strings:** `docking | undocking | mowing | mapping | explore_mapping`. There is also the pseudo-state `battery_temp_highing`.

**`BaseTask` guards** (log strings in `base_task.cpp`):
- `pause` is only allowed from `running`.
- `resume` is only allowed from `paused`. It sets blackboard `resume_trigged`.
- `stop` is only allowed from `running`.

When paused, each task's `execute()` loop logs `"enter pause mode, waiting resume..."` and sets blackboard `task_pause`. `HeadingCheck` skips the controller `resume` when `task_pause` is set (`heading_check.xml:22-25`).

**App-facing enum** (`mower_docs/reference/proto/status.proto:151-180`, `TaskStatusRsp`):
- `State`: Idle=0, Running=1, Paused=2, Finished=3, Stopped=4, Error=5
- `Type`: UnDock=0, Mow=1, GoDocking=2, CreateMap=3, ExploreMap=4, Unkonw=100

**Sub-state and fault flags.** These are set from the tree by `SubStateFaultNotify cmd=set_sub_state|set_fault|clear_fault|clear_all_fault`, with `context=docking|idle|open_cutter_failed`.

**Light/LED modes** (`TaskManager::lightControl`): PowerOff, PairIng, PairFinished, PairFailed, OTAIng, OTAFinished, WarnSensorTrigged, TaskStart, Charging, ChargeFinished, RobotUnusual, RemoteControl, Mapping, Idle, TaskPause, Docking, LowBattery.

### 1.2 Start preconditions

`TaskManager::checkTaskEnv` rejects a start with these log strings:
- `device is locked`
- `localization state error`
- `emergency stop`
- `lift triggered`
- `rain triggered`
- `battery temp high`
- `upgrading`
- `has a fault`

They map to the `StartTaskRsp.ErrorCode` values in `task.proto:55-74`: -100 Localization_Err, -101 EmergencyStop, -102 Lift, -103 LoadMap_Err, -107 Params_Err, -108 Rain, -110 Robot_Lock, -111 Start_Work_Outside, -112 Battery_Temp_High. Start-outside-the-zone is checked by `TaskMapInfo::robotIsInWorkZone`, which logs `IN OBS ZONE`, `robot in channel` and so on.

### 1.3 Mow sequence: `MasterMainTree` (`src/mower_task/mower_bt_nodes/behaviors_master/mower_logic_master.xml:54-125`)

**Blackboard set up by `MoweTask::execute`:** `is_need_undock` (true unless the robot is far from the dock point), `undock_distance`, `undock_pose`, `currentTaskMap`, `allow_work_time`, `rain_wait_msec`, `work_battery_vol`, `areaPathsQueue`, `skip_cutter_check`, `controller_recorving`.

1. `CameraControl /rear_camera/stop_capture` (`std_srvs/Empty`) — line 57
2. `Charging /charging enable_charging=false` (`mower_msgs/ChargingControl`) — line 60
3. `SubTree Undocking`, skipped if `!is_need_undock` (lines 63-65). It runs in this order:
   1. `/rear_camera/stop_capture`
   2. `/charging false`
   3. `SetMap → /localization/LoadMap` (`mower_msgs/LocatorLoadMap`: `map_name`, `charging_station_gps`, `base_station_gps`, `charging_station_orientation`)
   4. `TriggerControl /localization/ComputeHeading arg=start` (`mower_msgs/Trigger`)
   5. `UndockingController`: drives `undock_distance` (0.8 m, `mower_logic_master.xml:66`) using `/odom`; subscribes `/undock_state` and `/mower_base/status`
   6. `ComputeHeading arg=stop`, then `Sleep 3000`
   7. `HeadingInitCheck` on `/mower_localization_info` (`heading_initialized`). If it fails: `SoundLightCtrl audio 800005` and FAILURE (`undocking.xml:3-41`).
4. `LoopArea` over `areaPathsQueue` (`mower_core::AreaPlanResult`, one per zone, lines 68-109). For each area:
   1. **`AreaPathTransfer`** calls **`/planning/coverage`** (`mower_msgs/PlannerCoverage`) and fills `taskPathsQueue` (`deque<mower_msgs/PlannerPath>`), `path_params` (`mower_msgs/TypePath`), `cutter_height` and `is_need_patch`. It publishes `/scene_update_coverage_path`.
   2. **`LoopPath`** over `taskPathsQueue`, as a `PipelineSequence`. Each step re-ticks `AutoChargeCheckEx` (1.4), then:
      - **`NavigateToPose`** (lines 166-190):
        - `GlobalPlanner astar` via **`/planning/goal`** (`mower_msgs/PlannerGoal{MapPose start, goal} → PlannerPath`)
        - `CloseCutterCheck`, then `HeadingCheck`
        - `GlobalController controller_name=teb_to_pose`
      - **`FollowPath`** (lines 28-52):
        - `HeadingCheck`
        - `OpenCutterCheck(cutter_height)`, or `CloseCutterCheck` if `controller_recorving`
        - `CoverageController controller_name=teb`
   3. `AreaPatch`, if `is_need_patch` (lines 3-26): **`/planning/shortfall`**, then loops the patch paths.
   4. `CloseCutterCheck`
5. If `LoopArea` fails (lines 110-118): `CloseCutterCheck`, `GoDocking`, then FAILURE.
6. `clear_break_point := true`, then `SubTree GoDocking` (lines 120-123).

**`GlobalController` / `CoverageController`** are both the `TebController` class. For each path, `send2Controller` does the following:
1. `/controller/SetTypePath` (`mower_msgs/SetTypePath{TypePath[] path} → uint32 result`). Log: `"SetPath: type, action, strategy, turning_mode, nav_speed, scene, size, rad"`.
2. `/controller/ctrl arg=start` (`mower_msgs/Trigger`).
3. Polls `/controller/GetNavStatus` (`mower_msgs/GetNavStatus → NavStatus{state, sence, motion, exception_code}, line_pose, target_pose`) until `STATE_TRAJECTORY_COMPLETED` (logs `"Goal Reached!!"`).

It also does the following:
- Subscribes `/controller/event` and `/track_line_pose`.
- Publishes `/notice_code`, `/nav_path_info` and `/scene_curr_nav_path`, and can publish `/cmd_vel` directly.
- On "near patch" it uses `/planning/coverage_near`.
- On exception (`EXCEPTION_STUCK`, `ENCIRCLEMENT`, `BUMPER_EXCEPTION`, `MULTIPLE_COLLISIONS`, `DRAG`, `SIEGE`) it logs `"exception handled, try resume..."`.
- On geofence it logs `"Robot in forbidden zone"` / `"out work zone, relocation"`.

`NavExceptionHand` (lines 127-164) uses `FixPath` to re-plan around the error point, and retries every 180 s via `InterruptibleSleep`.

**Cutter subtrees** (`cutter_control.xml`):
- **`CloseCutterCheck`** (lines 3-25): unless `skip_cutter_check`, and if `CutterOnCheck` (`/mower_sensor_info.is_cutting`) is true:
  1. `/controller/ctrl pause`
  2. `CutterControl /cutter_control cutter_on=false height=90mm audio=900019 light=23`
  3. `Sleep 5000`
- **`OpenCutterCheck`** (lines 27-81):
  1. `clear_fault open_cutter_failed`
  2. `/controller/ctrl pause`
  3. Up to 9 tries of: `CutterControl cutter_on=true height={cutter_height} audio=900020 light=22`, `Sleep 5000`, verify `is_cutting`. If the check fails, turn the cutter off and sleep 5 s.
  4. If all 9 fail: `set_fault open_cutter_failed`, audio 900039, FAILURE.
- `CutterControl` also plays sound and light: `/mower_sound/play` (`Trigger`, arg = code) and `/light_control` (`mower_msgs/LightControl{LightMode mode}`).

**`HeadingCheck`** (`heading_check.xml:3-28`): if `HeadingMonitor` reports heading lost:
1. `/controller/ctrl pause`
2. `/localization/LoadMap`
3. `ComputeHeading start`
4. `RelativeMotion` 0.4 m at 0.2 m/s
5. `ComputeHeading stop`
6. `Sleep 3 s`
7. `/controller/ctrl resume`, unless `task_pause`

### 1.4 Auto-charge / rain / work-window: `AutoChargeCheckEx` (`auto_charge.xml:3-41`)

`ChargeEnvCheck check_type=0 low_battery_level=20` (line 5) returns SUCCESS, meaning carry on, unless one of these holds:
- battery < 20 %, from `/battery` (`sensor_msgs/BatteryState`)
- rain (`/mower_sensor_info.rain_triggered`)
- outside `allow_work_time`
- battery temperature high

It also publishes `/controller/event` (`ControllerEvent.event_charge_rain=7`, `_battery_low_level=8`, `_battery_high_temperature=9`).

When it does not return SUCCESS, the tree runs:
1. `CloseCutterCheck`
2. `/mower_sound/play 900024`
3. **`TaskSnapshot`**: saves the remaining path as a breakpoint
4. `GoDocking`
5. Blocks in `Inverter(KeepRunningUntilFailure(ChargeEnvCheck check_type=1, low_battery_level={work_battery_vol}))` until the resume conditions are met
6. `InterruptibleSleep {rain_wait_msec}`, if rain
7. `Undocking`
8. `controller_recorving := true`

There is also a separate, non-BT path: `TaskManager::handleAutoCharge` (`"low battery, auto charge start!!!"`, param `task/low_battery_charge_val: 20.0` in `src/mower_task/mower_logic/config/task.yaml:2`), plus `handleTimedLowBatteryShutdown` (below 5 % → `shutdown -h +1`).

### 1.5 Dock: `GoDocking` (`go_docking.xml:3-73`)

1. `HeadingCheck`, then `CloseCutterCheck`
2. `/mower_sound/play 900031`
3. `ReactiveFallback`:
   - **`DockOKCheck`** on `/mower_sensor_info` (`is_docking_done`)
   - **or** the approach sequence:
     1. `GlobalPlanner astar` to `{undock_pose}`
     2. `GlobalController teb_to_dock`
     3. `set_sub_state docking`
     4. **`DockController`**: `/controller/dock/SetDockPose`, then `/controller/dock/ctrl`, then poll `/controller/dock/GetDockStatus`
     5. `set_sub_state idle`
   - On failure: `set_sub_state idle`, `/rear_camera/stop_capture`, audio 900025 (FailReturnChargeStation), FAILURE
4. `Charging /charging enable_charging=true`
5. `set_sub_state idle`
6. `/rear_camera/stop_capture`

### 1.6 Triggers into the state machine

- **User command:** HTTP (section 2), MQTT through `/iot/ctrl` (`mower_msgs/IotControl{IotCmd cmd, string arg} → {bool result, int32 err_code, string msg}`; `IotCmd` StartTask=0, PauseTask=1, ResumeTask=2, StopTask=3, GoDocking=4, UnDock=5, …, GetAreaPath=60, in `src/mower_msgs/msg/IotCmd.msg`), or BLE.
- **Physical keys:** `TaskManager::sensorStateCallback` reads `/mower_sensor_info.key_pressed`:
  - KEY_WORKING_OR_PAUSE=1 toggles start/pause/resume (`"current task is running, pause task" / "is pause, resume" / "no task, start task"`)
  - KEY_GO_DOCKING=2 → `"go dock triggerd"`
  - power short/long → `shutdown -h now`
- **Timed schedules:** `timedTasksThread` → `findTimedTask` → `startTimedTaskIfReady`. Modes are `Single`/`Loop` with `startTime`, `interval`, `allowWorkTime`, `rainWaitMinute`, `enable`.
- **Low battery, rain, outside the work window, battery hot:** `ChargeEnvCheck` (1.4).
- **RTK/localization loss:** `HeadingMonitor`/`HeadingCheck`; `ControllerEvent.event_off_rtk=12`; notices 900030 / 900055 / 800005.
- **Boundary violation:** `TebController::checkRobotInForbiddenZone`, `ControllerEvent.event_outside=4`, notices 300001 / 300002.
- **Bumper:** handled inside `/navigation` (`libbumper_controller.so`, `MowerSensorInfo.bumper_routing_status`). It surfaces as `NavStatus.exception_code=3/4`.
- **Lift / e-stop during a mow:** `sensorStateCallback` logs them, and `FaultManager` raises 900018 / 900017. I could **not** find an explicit "lift → pause task" branch in the strings. It's likely a fault plus a controller exception (inferred).
- **Placed on the dock mid-task:** `"move to the charging pile, stop task and clear break point."`

---

## 2. Local HTTP API (13344) and WebSocket (13345)

**Where it lives.** It is implemented in `mower_logic_master_node` by `libmower_logic_net.so`, using Crow (`"Crow/0.1"`, `crow::Crow<HttpMiddleware>`) and nlohmann-json:
- `src/net/base/http_server_factory.cpp` logs `"Http server start, port : %d"` and binds 0.0.0.0.
- `api_task.cpp` and `api_mapping.cpp` hold the handlers.
- `websocket_server.cpp` handles `/robot/task/info`; the description in `mower_docs/reference/openapi.json:211` is `ws://127.0.0.1:13345/robot/task/info`.

The port numbers are immediates in the binary, so I couldn't read them from strings. 13344 was confirmed live in `mower_docs/tools/poc_13344.py:34`; 13345 comes only from the openapi description.

**Response envelope** (`openapi.json` `components.schemas.responsesData`):
```json
{"data": {}, "errorCode": 0, "msg": "successed", "successed": true}
```

**Full route list** (registration order in `libmower_logic_net.so`). Each handler calls `TaskManager` / `RobotDatabase` / `RobotDataHandler` from the task and db libs:

| Route | Handler → backend | Notes / JSON keys (from strings) |
|---|---|---|
| GET `/maping/area/start?type=&name=` | `RobotDataHandler::startRecordArea` | type 1 = area, 2 = channel, 3 = no-go |
| GET `/maping/area/end` | `endRecordArea` | |
| GET `/maping/pose/add?type=&name=` | `addPose` | type 1 = charging station |
| GET `/maping/create/save` | `saveRecordArea` → `/mapping_control` | |
| GET `/maping/area/list` | `getRecordAreaList` | not in openapi |
| POST `/map/save` `{"mapName","geoData"}` | `RobotDatabase::addMap(creatUUID)` + `TaskManager::setActiveMap` | returns `mapId`; **repoints `active_map.json`** (`mower_docs/05-maps-and-http-api.md:60-74`) |
| GET `/map/delete?map_id=` | `RobotDatabase::deleteMap` | |
| GET `/map/list` | `RobotDatabase::getMaps` | `data:[{mapId,mapName,geoData,createTime}]`. The SQL is `select * from tb_map ORDER BY ROWID DESC LIMIT 1`, so it returns **only the newest map** |
| GET/POST `/task/add`, `/task/update`, `/task/delete?task_id=`, `/task/list?map_id=` | `RobotDatabase::{add,update,delete}Task`, `getTasks` | `mapId, taskName, taskUnits, taskId, createTime`; not in openapi |
| POST `/task/start` | `TaskManager::parserTaskInfo` + `loadWorkMap` + `startTask` | see the body below |
| GET `/task/pause`, `/task/resume`, `/task/stop` | `TaskManager::{pause,resume,stop}Task` | |
| GET `/task/unDock`, `/task/dock` | `startTask` with an Undocking / Docking task type (inferred) | |
| GET `/task/getCoveragePath` | `TaskManager::getTaskPaths` | `data.features[]` of `LineString` |
| GET `/task/getWalkPath?point_index=` | `RobotDataHandler::getTaskTrackPose` | `data: [[x,y,...],...]` |
| GET `/system/clearAlarm` | `clearEstopAlarm` → `/clear_estop` + `TaskManager::clearAllFaultCode` | |
| POST `/system/setTime` `{"time"}` | runs `scripts/set_system_time.sh` | |
| `/system/restoreFactorySetting`, `/system/upgrade` `{url,version}`, `/system/log/take` | `UpgradeManager`, `scripts/log_take.sh` | |
| POST `/motor/cutter/control` `{"enable","height"}` | `RobotDataHandler::setCutterControl`; refuses if lift triggered | |
| WS `/robot/task/info` | `WebSocketServer::pubMsgManager` | openapi says `{"state":"idle/running/paused","taskName","startTime"}` |

**`/task/start` body.** These keys are parsed in `APITask::startTask`; log: `"area_id:%s, repeat:%d, path_angle:%f, path_mode:%d, cutterheight:%d, num_perimeters:%d"`. The openapi only shows `{"mapName":"10.geojson"}`. The `params` array below is inferred from the key strings:
```json
{"mapName":"24705761407971328",
 "params":[{"areaId":"a059b4f5-…","repeat":1,"pathAngle":0.0,"pathMode":0,"cutterHeight":50,
            "cutterSequence":0,"numPerimeters":2,"cutSpeed":0,"cutMode":1,"strategy":0,
            "turningMode":0,"aiRecommend":false}]}
```
If there is no `params`, it falls back to a synthetic `"ALL_ZONES_TASK"` (inferred from the string).

**HTTP → ROS mapping.**
- `start` → `MoweTask` → the `MasterMainTree` call order in 1.3.
- `pause` → `MoweTask::pause`. That logs `"cutter is cutting, set close!"`, then calls `/controller/ctrl pause` and `/controller/dock/ctrl pause` via `RobotDataHandler::updateController` / `updateDockController`.
- `resume` → `/controller/ctrl resume`.
- `stop` → the tree is halted and the controller stopped.
- `dock` / `unDock` → the `GoDocking` / `Undocking` trees.

---

## 3. Map and zone storage

**On-disk layout** (`mower_docs/05-maps-and-http-api.md:3-23`):
- `/userdata/RobotData/active_map.json` holds the bare id, e.g. `24705761407971328`.
- `/userdata/RobotData/map/<id>/map.geojson`, alongside `rtk_info.txt`, `vmap/`, `spatial/` and `layers/*.png`.
- `/userdata/RobotData/robot.db`. `libmower_logic_db.so` contains `/RobotData/map` and `/map.geojson`, and `INSERT INTO tb_map (id,name,geoJsonFile)…`.

**Tables** (live schema from `ros2_port_handoff/12_excluded_secrets_identity/RobotData/robot.db`):
- `tb_map(id,name UNIQUE,geoJsonFile,userId,createdAt)`
- `tb_task(id,name,mapId FK cascade,type,units)`
- `tb_run_task_info(mapName,mapId,taskId,point,pathSeq)`
- `tb_task_break_point_info(mapName,mapId,taskId,cutArea,x,y,pathSeq,areaId,taskInfo)`
- `tb_config(id,content)`, e.g. DeviceLock, Net4GAllowUploadPicture, SetDarkMode

**How a name resolves:** `mapName` → `RobotDatabase::getMapIdByName` (`select id from tb_map where name=…`) → `TaskManager::loadWorkMap(id)` → `/userdata/RobotData/map/<id>/map.geojson`. On this device name == id == `24705761407971328` (`tb_map` row `('24705761407971328','24705761407971328','map.geojson',…)`). Cloud maps come in via `IotManager::createMapCmd{mapName,mapUrl,mapId}`: download to `/tmp/map.geojson`, check the md5, then save.

**Parser:** `mower_map::MapParser` (`$OTA/mower_map/lib/libmower_map.so`, source `/workspace/src/mower_task/mower_map/src/map_parser.cpp`). It is a pluginlib `mower_core::MapInterface` with methods `loadMap`, `loadJsonString`, `loadJsonRelativeMap`, `loadJsonWorldMap`, `mapConvert`, `toGeoMapJsonString`. `MapConverter::mowerMapToOccupancyGrid` (`map_converter.cpp`) rasterises the map.

`TaskMapInfo` (`load_map.cpp`, in the task lib) does the following:
- names zones `AREA_<id>`, `NO_GO_ZONE_<id>`, `CHANNLE_<id>`
- publishes `/geojson_task` (`foxglove_msgs/GeoJSON`), `/scene_update_task_map`, `/map` and `/planning/map`
- calls `/map/load`, `/planning/task_set`, `/map/layer/{cutter,reset}`, `/planning/get_obs`, `/localization/{LoadMap,GetMapInfo}`, `/datum` (`robot_localization/SetDatum`)

**GeoJSON schema.** The live file is `…/12_excluded_secrets_identity/RobotData/map/24705761407971328/map.geojson`. Top level:
```json
{"type":"FeatureCollection","version":"1.3.6","properties":{"mapRtkType":"RTK"},"features":[…]}
```
Each feature has `properties {id: UUID, name, parent_id, type: int}`.

| `type` | Geometry | Meaning (live example) |
|---|---|---|
| 1 | Polygon | mowing area, `name:"A"`, 56 points, coordinates `[x,y,z,roll,pitch,quality]` |
| 3 | LineString | channel (dock → area), extra `geometry.specialOffSet: []` |
| 4 | Polygon | obstacle / no-go zone. Live example: `[[-0.275,-0.75],[-0.275,0.75],[1.725,0.75],[1.725,-0.75]]` around the dock |
| 5 | Polygon | dock zone / outline: `[[0.2,0.275],[0.2,-0.275],[-0.2,-0.275],[-0.2,0.275]]` |
| 6 | Point | `charge_point`: `[0,0,0.1,0.1,0]` |
| 7 | Point | `undock_point`: `[0.8,0]` |
| 8 | Point | RTK base station: `[0,0,0.1,0.1]` |

Other parser properties are `special_type` (special zones, `mower_core::SpecialZone`) and obstacles nested by `parent_id`. These are from strings like `"Special zone id: %s missing special_type property"` and `"Error: Mowing area %s does not exist when adding obstacle_id"`.

The parser also accepts the string point names `dock_point` / `undock_point` and `rtk_point`; the strings `"Missing dock point"` / `"Missing undock point"` are fatal errors.

The exploration-map writer (`explore_mapping_task.cpp`) emits names `explore_area_*`, `*_obstacle_*`, `channel`, `charge_point`, `rtk_point`, `undock_point`. The old `E_TYPE_*` names (`mower_docs/reference/mower_map_geojson_README.md`) are obsolete; that README's "WGS84" claim is wrong for current maps.

**Coordinate frame.** Coordinates are local metres:
- The origin is the robot's motion centre (rear axle) when it sits on the charger with both contacts touching.
- Robot frame is X forward, Y left, Z up. So `undock_point (0.8,0)` lies straight ahead along +X.
- Geographic anchoring is set by `LocatorLoadMap.charging_station_gps` and `charging_station_orientation` = the angle of local X from East, clockwise positive (`src/mower_msgs/srv/LocatorLoadMap.srv`).
- The coordinate's last element is quality: 1 = RTK fixed.
- There is also a world-coordinates mode, `loadJsonWorldMap`, with log `"dock point lat/lon/alt"` (when `mapRtkType` / datum is in use; inferred).

---

## 4. Coverage-planning inputs

**Per-zone `TaskUnit`** (`mower_docs/reference/proto/task.proto:38-51`), with JSON names in parentheses:
- `areaId`
- `repeat`
- `path_angle` (`pathAngle`, radians; 0 = undock direction)
- `path_mode` (0 zig-zag, 1 cross, 2 alternate zig-zag, 3 spiral)
- `cutter_height` (mm)
- `cutter_sequence` (0 edge first, 1 zig-zag first)
- `cut_speed` (`CutSpeed` 0-3)
- `strategy` (`ObsStrategy`: 0 auto, 1 bumper only, 2 bumper + AI except edges, 3 bumper + AI)
- `cut_mode` (1 overall, 2 contour only)
- `truning_mode` (`TurningMode` 0-6)
- `zigzag_dis` (`zigzagDis`, m)
- `ai_recommendation` (`aiRecommend`)

The HTTP side also has `numPerimeters`. The DB/IoT side adds `allowWorkTime {"1":"08:00-12:00,…"}`, `rainWaitMinute`, `mode` Single/Loop, `startTime`, `interval` (`task.proto:262-283`).

**Defaults** (`src/mower_task/mower_logic/config/coverage.yaml:1-8`): `zigzag.angle 0.0`, `distance 0.15`, `outer_offset 0.15`, `num_perimeters 2`.

**ROS contracts** (`src/mower_msgs`):
- **`/planning/task_set`** — `PlannerTaskSet.srv`: `PlannerOption[] options` → `int32 result`. Called by `TaskMapInfo::setPlannerTask` (`"set task area id:%s, sequence:%d"`).
- **`/planning/coverage`** — `PlannerCoverage.srv`: `PlannerOption option` → `PlannerCode code, PlannerPath[] paths, Cellpolygons cellpolygons`. Called by `AreaPathTransfer` (`"area zigzag dis, bow angle, ai recommend, cut sequence, path mode"`).
  - `PlannerOption.msg` fields:
    - `shape` (0 unknown, 1 auto, 2 bow, 3 spiral)
    - `bow_angle_radians`
    - `float64 distance`
    - `ai_recommend`
    - `polygon_uuid`
    - `geometry_msgs/Pose break_pose` (0,0 means no breakpoint)
    - `break_path_type`
    - `uint32 cut_sequence` (digit string, e.g. 1234 = edge, area, restricted-edge, patch)
  - `PlannerPath.msg` fields: `type` (1 coverage, 12 spiral, 2-4 edge origin/inset 1/2, 5-7 restricted edges, 8/10/11/14/15 patch variants, 9 goal, 13 obs-check, 16 exploration), `angles_radians`, `nav_msgs/Path path` (pose `z=1` means a point on a channel).
- **Related planner services:** `/planning/coverage_near`, `/planning/shortfall` (patch), `/planning/obs_check`, `/planning/obs_check_cover`, `/planning/goal` (`PlannerGoal`), `/planning/get_obs`. `/planning/info` (`PlannerInfo{task_area[], task_area_total, task_area_cut}`) drives remaining area.
- **`/controller/SetTypePath`** — `SetTypePath.srv`: `TypePath[] path` → `uint32 result`. `TypePath.msg` fields:
  - `PoseArray path`, `PlannerPath plan_path`
  - `type`: NAV 0, CONTOUR 1, COVERAGE 2, CHANNEL 3, BACKUP 4, MAP_EDGE 5, OBS_EDGE 6
  - `strategy` 0-3, `action` (speed 0-3), `scene` (1 = GO_DOCK), `turning_mode` 0-6, `float32 nav_speed`
- **`/controller/SetTaskParams`** — called from `RobotDataHandler::updateNavParams`. Its type is most likely `UpdateNavParams.srv`: `uint32 strategy, uint32 turning_mode, float32 nav_speed` → `uint32 result`. The log string `"updateNavParams service: nav_speed:%f, strategy:%d, truning_mode:%d"` supports this, but I didn't confirm the type with `rosservice type`.
- **`/controller/ctrl`** — `mower_msgs/Trigger{string arg}`, args `start | pause | resume | stop`; the last three were verified on hardware (`RESUME.md`).

**Not found:** how `cut_speed` maps to `nav_speed` numerically, and how `cutter_height` mm maps to `CutterControl.height.position` %.

---

## 5. Docking and charging

**Who does what.** `mower_logic` plans and drives to `undock_pose` (A* plus `teb_to_dock`). `mower_charge` then does vision docking with the **rear camera and an ArUco marker** (`cv::aruco`, `convertArucoToBasePose`, `marker_size_`), driving **in reverse**.

`mower_charge` interfaces (`node_info_mower_charge.txt`):
- **Subscribes:** `/rear_camera/image_raw`, `/rear_camera/camera_info`, `/odom_fused`, `/mower_sensor_info`
- **Publishes:** `/cmd_vel`, `/notice_code`, `/charge/{dock_path,lookahead_point,mark_pose}`
- **Calls:** `/rear_camera/start_capture`
- **Serves:**
  - `/controller/dock/SetDockPose` (`SetDockPose{Pose dock_pose}`)
  - `/controller/dock/GetDockStatus` (`→ uint32 state` 0 success, 1 fail, 2 searching, 3 docking; plus `dock_pose`; `GetDockStatus.srv`)
  - `/controller/dock/ctrl` (`Trigger`, args `stop | pause | resume` per `srvDockCtrl`; start is presumably triggered by SetDockPose, inferred)

**State machine** (`src/mower_charge/doc/state.jpg` plus strings `OnOpenCamera`, `OnSearch`, `OnSetParam`, `OnDocking`, `OnFinalDock`, `OnStuck`, `OnRetry`, `OnSuccess`):
1. INIT → open the camera. More than 10 camera errors → FAILED.
2. SEARCHING (2) → marker identified → DOCKING (3). If the marker estimate at start is worse than 0.5 m / 25°, it is rejected.
3. DOCKING → FINAL_DOCKING (4), a straight reverse. That reaches SUCCESS, or times out after 10 s / 30 cm and goes to RETRY.
4. RETRY (5) on deviation. The stuck sub-state is "脱困" (escape). Retries are capped, then FAILED.
5. PAUSE (7) is entered by button or `ctrl pause`.

Other recovered strings: `"[OnDocking] Docking timeout > 60s, retrying..."`, `BACKWARD_TRACKING`, `BLIND_REVERSE`, `FORWARD_RETRY`, `CheckLost "Too many frames lost"`. On failure it raises notices 900051 DockDetect, 900052 DockTimeout and 900053 DockMaxout.

**Contact detection.** `MowerSensorInfo.is_docking_done` comes from the MCU. `mower_charge` debounces it with `is_dock_done_num: 3` (`src/mower_charge/param/cfg.yaml:1`, `m_dock_done_deque_`). The BT `DockOKCheck` reads the same flag. `is_charging` reports charging, and the `/charging` (`ChargingControl{bool enable_charging}`, served by `base_driver_node`) call enables the charger relay.

**Dock geometry.** The dock is the map origin, and the undock/approach pose is `undock_point` = (0.8, 0), heading 0 (+X), reached with `undock_distance := 0.8` (`mower_logic_master.xml:66`). The dock outline is a 0.4 × 0.55 m polygon (type 5), and a no-go keep-out sits around it (type 4). PID gains are in `src/mower_task/mower_bt_nodes/config/dock_controller.yaml`: `Undocking` max 0.3 m/s, `FinalDock` max 0.05 m/s with 1 cm tolerance, plus a `DriveForward` section. `retry_dock.yaml` and `mpc_docking.yaml` are also in that config dir.

---

## 6. Safety chain

These are layers I found, not a verified end-to-end order:
1. **MCU firmware** (chassis/cutter boards). The docs claim it enforces e-stop, lift and bumper cut-offs (`mower_docs/09-reuse-strategy.md:15,71`; `ros2_port_handoff/README.md:103`). I did **not** find the code that proves it in `mcu_decompile/`.
2. **`base_driver_node`** (`mower_base_node`). It decodes `SensorInfo` and publishes `/mower_sensor_info` at 33 Hz: `bumper_triggered`, `rain_triggered`, `lift_triggered`, `stop_triggered`, `is_cutting` (cutter rpm > 500, `DevStatus::judgmentCutterStatus`, `native_decompile/dec/mower_base_node/mower_base_node.c:290359`), and `bumper_routing_status`. It also publishes `/notice_code`. `Motors::cutterOff()` (`mower_base_node.c:247548`) is called only from the constructor (line ~18328), so the host driver does **not** itself cut the blade on lift or e-stop. `/clear_estop` (`std_srvs/Empty`) sends MCU frame 10 (`mower_base_node.c:19943`).
3. **`/navigation`** (controller): bumper back-up and routing (`libbumper_controller.so`, `/enable_bumper` `BumperControl`), stuck/drag detection. It reports through `NavStatus.exception_code` and `/controller/event`.
4. **`mower_logic`:**
   - **Cutter interlock:** before every move between paths, every dock and every pause, `CloseCutterCheck` calls `/controller/ctrl pause` *then* `/cutter_control` off (`cutter_control.xml:11-21`).
   - **Pause** closes the cutter (`MoweTask::pauseTask`).
   - **Starts are refused** on e-stop, lift, rain, lock or fault (`checkTaskEnv`).
   - **Geofence:** `TebController::checkRobotInForbiddenZone` / out-of-work-zone → relocation or exception.
   - **Rain / battery** send the robot to the dock.
   - **`/motor/cutter/control`** refuses if lift is triggered.
   - `RobotDataHandler::setZeroVel` publishes a zero `/cmd_vel`.
   - `FaultManager` keeps the fault list and `/alarm_status`.

**Not found:** the exact host-side response to lift or e-stop mid-mow (whether it pauses or stops the task), and the host heartbeat timeout behaviour.

---

## 7. Telemetry contract for a UI

**ROS topics** (`topics_verbose.txt`):
- `/task_info` (`std_msgs/String` JSON)
- `/task_report` (String JSON)
- `/notice_code` and `/notice_info` (`mower_msgs/IotNotice{uint32 code, timestamp, module}`; module values LOGIC 0, BASE 1, LOCALIZATION 2, PERCEPTION 3, CONTROLLER 4)
- `/alarm_status` (`std_msgs/UInt64` bitmask)
- `/explore_map_info` (String)
- `/planning/info`, `/controller/event`, `/mower_sensor_info`, `/mower_localization_info`, `/battery`

**`/task_info` JSON.** The keys come from `BaseTask::getRunningInfo` / `MoweTask::getRunningInfo` / `TaskManager::getTaskInfo`, and are consumed by `mower_mqtt` `taskInfoCallback`. The nesting below is inferred; I have no captured sample:
```json
{"task":{"type":"mowing","state":"running","mapId":"…","taskId":"…","startTime":"…","runTime":"…",
         "topArea":123.4,"remainingArea":56.7,"areaId":"…",
         "params":[{"areaId","repeat","cutterHeight","cutterSequence","pathAngle","pathMode",
                    "cutSpeed","cutMode","strategy","truningMode"}]},
 "mapId":"…","hasLegacyTask":false,"legacyTaskId":"",
 "batteryPercentage":80,"isCharging":false,"pose":{…},"sensorStatus":0,"localization":{…}}
```
`mower_mqtt` maps this onto `TaskStatusRsp` (`status.proto:151-180`).

**`/task_report`** keys: `mapId, taskId, taskName, taskType, startType, result, startTime, endTime, mowArea, totalArea, reportFile, fakeObsSize, pictureFile, startPose, endPose, events, track`. These match `TaskReportReq` (`task.proto:233-253`). Files go to `/userdata/RobotData/reports/<ts>_<mapId>/report.json` plus `pictures.zip`.

**Full status** — `FullStatusRsp` (`status.proto:305-322`) combines:
- `VersionRsp`
- `BatteryStatusRsp{battery_error, battery_percentage, battery_temperature}`
- `UpgradeStatusRsp` ×2
- `SensorStatusRsp{int64 status}`, with bits: 0 bumper, 1 rain, 2 e-stop, 3 lift, 10 pause key, 11 go-dock key, 12 power-off, 15 charging, 16 dock done, 19 door open, 20 big cutter, 21 fill light (`status.proto:24-42`)
- `RtkStatusRsp{robot_pose, ref_station_pose, ref_station_state, localization_state (0 INIT, 1 RTK_VISION, 2 VISION_ONLY, 3 LOST), remaining_vision_buffer, rtk_status, num_satellites, …, heading_initialized, lat/lon/height_sigma, hdop, diff_age}`
- `TaskStatusRsp`
- `NetInfoRsp`, `DeviceOnlineStatusRsp`, `RTKinfoRsp`, `UpgradeMCUStatusReq`, `GetConfigRsp`

**Notice** — `NoticeRsp{time_stamp, code, content}`. The full code list is in `src/mower_msgs/msg/IotNotice.msg`, e.g. 900006 TaskStart, 900010 TaskPause, 900011 StartCharging, 900017 E-stop, 900018 Lift, 900024 LowPowerGoDock, 900025 FailReturnChargeStation, 900031 GoDock, 900039 OpenCutterFailed, 300001/300002 geofence, 400005 rain, 900051-53 dock failures.

**Alarm** — `AlarmStatusBitRsp.alarm_status` (int64). The bit → code table is at `status.proto:47-89`: bit 0 = 900017, 1 = 900018, 2 = 400005, 3 = 800002, 4 = 900055, 5 = 600004, …, 31 = 100105.

**Module events** — `ModuleEvent{time_stamp ms, module_code, event, raw_payload}`.

---

## What I could not find

- The C++ sources for `mower_logic`, `mower_bt_nodes`, `mower_map`, `mower_charge`, `mower_planning` and `navigation`. Only strings and symbols were available, and the OTA libraries weren't Ghidra-decompiled; only drivers and localization are in `native_decompile/dec`.
- 13344 / 13345 as constants in the binary (they are immediates). These come from a live test and the openapi text.
- Exact JSON nesting of `/task_info` and the WebSocket payload. There's no sample capture, only keys.
- The `cut_speed` → `nav_speed` and height mm → % mappings, and the `/controller/SetTaskParams` service type, which needs `rosservice type` to confirm.
- The `/navigation` controller internals (bumper or geofence cmd_vel zeroing).
- MCU firmware lift / e-stop cutoff logic.
- The host's reaction to lift or e-stop during a mow, beyond the fault notice.
