cmake_minimum_required(VERSION 3.8)
project(mower_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorStatus.msg"
  "msg/MotorInfo.msg"
  "msg/MowerBaseMotorsInfo.msg"
  "msg/MowerSensorInfo.msg"
  "msg/ControlInfo.msg"
  "msg/MowerBaseDevStatus.msg"
  "msg/PlannerOption.msg"
  "msg/MowerBaseButtonInfo.msg"
  "msg/MotorControl.msg"
  "msg/MowerBaseDevInfo.msg"
  "msg/BatteryHealthInfo.msg"
  "srv/PlannerTaskSet.srv"
  "srv/BumperControl.srv"
  "srv/CutterControl.srv"
  "srv/ChargingControl.srv"
  "srv/PlanCoverage.srv"
  "srv/MappingControl.srv"
  "srv/SetLoRa.srv"
  "srv/SetAreaSettings.srv"
  "srv/GetAreaSettings.srv"
  "srv/SetCutterHeight.srv"
  "srv/PlanRoute.srv"
  "action/Dock.action"
  "action/Undock.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs nav_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
