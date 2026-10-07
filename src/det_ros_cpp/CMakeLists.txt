cmake_minimum_required(VERSION 3.8)
project(det_ros_cpp)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -O2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# Pure decode/NMS/letterbox helpers (no ROS, no NPU) -> unit-testable anywhere
add_library(det_yolo STATIC src/yolo.cpp)
target_include_directories(det_yolo PUBLIC include)
set_target_properties(det_yolo PROPERTIES POSITION_INDEPENDENT_CODE ON)

# librknnrt.so: /usr/lib/librknnrt.so in the container (bind-mounted 2.3.0 runtime)
# A cached path from another container (e.g. the one-off build container without the
# bind-mount) would produce "No rule to make target"; re-search when it vanished.
if(RKNNRT_LIB AND NOT EXISTS "${RKNNRT_LIB}")
  unset(RKNNRT_LIB CACHE)
endif()
find_library(RKNNRT_LIB rknnrt PATHS /usr/lib /userdata/ros2/lib)

if(RKNNRT_LIB)
  add_executable(det_ros_cpp src/det_node.cpp src/rknn_model.cpp)
  target_include_directories(det_ros_cpp PRIVATE include src ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(det_ros_cpp det_yolo ${RKNNRT_LIB} ${OpenCV_LIBS})
  ament_target_dependencies(det_ros_cpp rclcpp sensor_msgs std_msgs vision_msgs cv_bridge)
  install(TARGETS det_ros_cpp DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "librknnrt.so not found: det_ros_cpp node not built (tests only)")
endif()

install(PROGRAMS scripts/det_parity.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_yolo test/test_yolo.cpp)
  target_link_libraries(test_yolo det_yolo)
endif()

ament_package()
