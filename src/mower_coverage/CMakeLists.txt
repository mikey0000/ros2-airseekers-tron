cmake_minimum_required(VERSION 3.8)
project(mower_coverage)

# Fields2Cover 2.1.0 headers are cxx_std_17; the F2C deb's exported target
# compiles clean under g++ 11 (jammy).
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(mower_interfaces REQUIRED)

# --- Fields2Cover -------------------------------------------------------------
# Consumed from the apt package `ros-humble-fields2cover` (2.1.0), NOT from
# rosdep: there is no `fields2cover` key in any rosdistro index, so it must
# NOT be a <depend> in package.xml. Install it in the Dockerfile:
#   apt-get install -y ros-humble-fields2cover
# That deb pulls ros-humble-ortools-vendor automatically (OR-Tools is a real
# NEEDED entry on libFields2Cover.so, satisfied at runtime via the vendor
# package's LD_LIBRARY_PATH), and its CMake config exports find_dependency for
# GDAL/GEOS/TBB/Eigen/Threads. Do NOT add a find_package(ortools) or any
# ortools-target workaround — that is only needed for Fields2Cover v3
# source builds (MowgliNext), not for the apt 2.1.0 config.
find_package(Fields2Cover 2.1.0 CONFIG REQUIRED)

# Pure-geometry planner core: no ROS types in its signature, so it is unit
# testable against the real F2C library with no robot and no ROS node.
add_library(mower_coverage_core SHARED src/coverage_planning.cpp)
target_include_directories(mower_coverage_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(mower_coverage_core Fields2Cover::Fields2Cover)

add_executable(mower_coverage_node src/mower_coverage_node.cpp)
target_link_libraries(mower_coverage_node
  mower_coverage_core
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  mower_interfaces::mower_interfaces)

# Standalone CLI test: plans a square field and prints the path. No ROS
# runtime needed — links only the planner core.
add_executable(coverage_cli src/coverage_cli.cpp)
target_link_libraries(coverage_cli mower_coverage_core)

install(TARGETS mower_coverage_node coverage_cli
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS mower_coverage_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp geometry_msgs nav_msgs trajectory_msgs
                         mower_interfaces Fields2Cover)
ament_package()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_coverage_planning test/test_coverage_planning.cpp)
  target_link_libraries(test_coverage_planning mower_coverage_core)
endif()
