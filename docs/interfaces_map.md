# Interfaces Map — ROS 1 `mower_msgs` → ROS 2 `mower_interfaces`

This document maps the ROS 1 interface definitions (from `ros2_port_handoff/03_ros_interfaces/mower_msgs`)
to their ROS 2 counterparts in `ros2_stack/src/mower_interfaces`. Only the **core subset** is converted
in this phase; the remaining messages/services/actions are deferred (see [Deferred](#deferred)).

All conversions keep the original field names, types, constants, and comments faithfully.
`std_msgs/Header` is unchanged between ROS 1 and ROS 2.

## Package layout

```
ros2_stack/src/mower_interfaces/
├── package.xml              # ament_cmake, rosidl_default_generators
├── CMakeLists.txt           # rosidl_generate_interfaces(...)
├── msg/
│   ├── MotorStatus.msg
│   ├── MotorInfo.msg
│   ├── MowerBaseMotorsInfo.msg
│   ├── MowerSensorInfo.msg
│   ├── ControlInfo.msg
│   ├── MowerBaseDevStatus.msg
│   └── PlannerOption.msg
└── srv/
    └── PlannerTaskSet.srv
```

## Core subset — converted messages

| ROS 1 (`mower_msgs`) | ROS 2 (`mower_interfaces`) | Live topic / type | Notes |
|---|---|---|---|
| `msg/MotorStatus.msg` | `msg/MotorStatus.msg` | (embedded in `MotorInfo`) | Constants `STATUS_*` / `ERROR_*` kept verbatim, incl. negative values. |
| `msg/MotorInfo.msg` | `msg/MotorInfo.msg` | (embedded in `MowerBaseMotorsInfo`) | `speed_rpm`, `current` (10 mA), `voltage` (10 mV), `temperature`, `status`. |
| `msg/MowerBaseMotorsInfo.msg` | `msg/MowerBaseMotorsInfo.msg` | `/mower_base/motor_info` | 4 motors: cutter / left / right / height. |
| `msg/MowerSensorInfo.msg` | `msg/MowerSensorInfo.msg` | `/mower_sensor_info` | Board versions, key bits, bumper/rain/lift/stop, battery gate, working status, bumper routing, cutter size, motor info. |
| `msg/ControlInfo.msg` | `msg/ControlInfo.msg` | — | Controller `state` (IDLE/RUNING/ARRIVED/PAUSE/EXCEPTION) and `type` (follow-line/over-obj/bypass-obj/follow-point). |
| `msg/MowerBaseDevStatus.msg` | `msg/MowerBaseDevStatus.msg` | `/mower_base/status` | Working status, bumper routing, gate/press/stop/bumper/rain/lift flags. |
| `msg/PlannerOption.msg` | `msg/PlannerOption.msg` | (embedded in `PlannerTaskSet`) | Coverage shape, bow angle, spacing, AI recommend, polygon uuid, break pose/path, cut sequence. Uses `geometry_msgs/Pose`. |
| `srv/PlannerTaskSet.srv` | `srv/PlannerTaskSet.srv` | `/planning/task_set` | Request `PlannerOption[] options`, response `int32 result`. |

## Core subset — reused standard messages

The live mower already publishes these two core topics using standard ROS 2 types, so no custom
message is defined; consumers depend on the standard packages directly.

| Live topic | Standard type | ROS 1 custom msg it replaces | Field mapping / notes |
|---|---|---|---|
| `/battery` | `sensor_msgs/BatteryState` | `mower_msgs/BatteryHealthInfo` | `BatteryHealthInfo.battery_temperature` → `BatteryState.temperature`; `battery_error` → `power_supply_health` (and/or `present` for gate). Full `BatteryState` fields (voltage, current, charge, capacity, percentage, cell_voltage, …) are populated by the driver. |
| `/wheel_vel` | `geometry_msgs/TwistStamped` | — (no ROS 1 custom msg) | `TwistStamped` is the stamped form of `geometry_msgs/Twist` (`header` + `twist.linear` / `twist.angular`). Equivalent to `Twist` plus a header; the live topic uses the stamped variant, so it is reused as-is. |

`sensor_msgs` and `geometry_msgs` are therefore declared as dependencies of `mower_interfaces`
(alongside `std_msgs`, `builtin_interfaces`, `nav_msgs`) so that consumers of the core subset
have a single interfaces package to depend on.

## Dependency usage

| Dependency | Used by |
|---|---|
| `std_msgs` | `Header` in `MowerBaseMotorsInfo`, `MowerSensorInfo`, `ControlInfo`, `MowerBaseDevStatus`. |
| `geometry_msgs` | `Pose` in `PlannerOption`; `TwistStamped` for `/wheel_vel`. |
| `sensor_msgs` | `BatteryState` for `/battery`. |
| `builtin_interfaces` | Pulled in by `rosidl` for `std_msgs/Header` stamp (`builtin_interfaces/Time`). |
| `nav_msgs` | Declared for parity with the ROS 1 package; not referenced by the core messages yet (used by deferred nav topics). |

## Live topic / service reference (core)

From `ros2_port_handoff/10_ros_live_snapshot/`:

| Topic | Type | Publisher |
|---|---|---|
| `/battery` | `sensor_msgs/BatteryState` | `/base_driver_node` |
| `/mower_base/battery_health` | `mower_msgs/BatteryHealthInfo` | `/base_driver_node` |
| `/wheel_vel` | `geometry_msgs/TwistStamped` | `/base_driver_node` |
| `/mower_base/motor_info` | `mower_msgs/MowerBaseMotorsInfo` | `/base_driver_node` |
| `/mower_base/status` | `mower_msgs/MowerBaseDevStatus` | `/base_driver_node` |
| `/mower_sensor_info` | `mower_msgs/MowerSensorInfo` | `/base_driver_node` |
| `/planning/task_set` (service) | `mower_msgs/PlannerTaskSet` | `/mower_planning` |

## Deferred

The following ROS 1 interfaces are **not** yet converted. They remain in
`ros2_port_handoff/03_ros_interfaces/mower_msgs/` and can be ported in a later phase:

- **Messages (48):** `AIClouds`, `BatteryHealthInfo` (superseded by `sensor_msgs/BatteryState`),
  `Cellpolygons`, `ControllerEvent`, `ControllerOutput`, `ControllerStates`,
  `ControllerTrackingError`, `EskfState`, `Event`, `ExplorePose`, `ExploreRelocStatus`,
  `FusedObstacleCloud`, `FusedObstacleClouds`, `FusionObstacle`, `IotCmd`, `IotNotice`,
  `KinematicModel`, `LayerCropedMapInfo`, `LightMode`, `LineTrackingInfo`, `Map`, `MapChannel`,
  `MapObstacle`, `MapPolygon`, `MapPose`, `MapSpecialZone`, `MotorControl`,
  `MowerBaseButtonInfo`, `MowerBaseDevInfo`, `MowerControllerInfo`, `MowerLocalizationInfo`,
  `NavStatus`, `OAMonoMeasure`, `ObstacleInfo`, `ObstacleInfos`, `PlannerCode`, `PlannerInfo`,
  `PlannerPath`, `PlannerPaths`, `Pose6D`, `RealObsInfo`, `TypePath`, `VioPoseResult`.
- **Services (32):** `BumperControl`, `ChargingControl`, `CutterControl`, `EnableStereoPerc`,
  `GetCroppedLayerData`, `GetDockStatus`, `GetExploreResult`, `GetNRTKInfo`, `GetNavStatus`,
  `GetRemainPath`, `IotControl`, `LightControl`, `LocatorGetMapInfo`, `LocatorLoadMap`,
  `LocatorSaveMap`, `MapLayerObstacleInfos`, `MapSet`, `MappingControl`, `NRTKBind`,
  `OaSelectCamera`, `PlanPath`, `PlannerCoverage`, `PlannerExplore`, `PlannerGoal`,
  `RecordBreakPoint`, `SetDatum`, `SetDockPose`, `SetKinematicModel`, `SetObstacleFiltering`,
  `SetPath`, `SetTypePath`, `Trigger`, `UpdateNavParams`.
- **Actions (4):** `Docking`, `NavigatePath`, `NavigateToPose`, `Undock`.
- **Other packages:** `mower_gps_msgs`, `mower_proto` (protobuf, BLE/MQTT payloads), `vanjee_lidar_msg`.
