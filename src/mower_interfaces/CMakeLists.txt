cmake_minimum_required(VERSION 3.8)
project(mower_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Core subset only (see docs/interfaces_map.md for the full ROS1 -> ROS2 mapping).
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorStatus.msg"
  "msg/MotorInfo.msg"
  "msg/MowerBaseMotorsInfo.msg"
  "msg/MowerSensorInfo.msg"
  "msg/ControlInfo.msg"
  "msg/MowerBaseDevStatus.msg"
  "msg/PlannerOption.msg"
  "srv/PlannerTaskSet.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
