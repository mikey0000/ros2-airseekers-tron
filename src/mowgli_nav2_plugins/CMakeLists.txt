cmake_minimum_required(VERSION 3.8)
project(mowgli_nav2_plugins)


# arm64 GCC >= 10.1 prints "parameter passing for argument of type std::pair ... changed in
# GCC 10.1" notes: harmless (one compiler builds the whole stack), they only clutter the log.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-Wno-psabi)
endif()
# Jazzy port of third_party/mowglinext/mowgli_nav2_plugins (Kilted):
# rclcpp_lifecycle node, setPlan/3-arg computeVelocityCommands, ControllerException,
# 3-arg GoalChecker, no mowgli_interfaces (blade-load slowdown fail-open).

# ---------------------------------------------------------------------------
# Compiler flags
# ---------------------------------------------------------------------------

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(xtensor REQUIRED)

# ---------------------------------------------------------------------------
# Shared library: mowgli_nav2_plugins (LegGoalChecker). The FTC controller, its
# PathProgressGoalChecker, obstacle deviation and oscillation detector were deleted in the
# 2026-10-23 cleanup (code review stage 0.1; MPPI is the only tracker).
# ---------------------------------------------------------------------------

add_library(${PROJECT_NAME} SHARED
  src/leg_goal_checker.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME}
  SYSTEM PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${rclcpp_TARGETS}
  ${rclcpp_lifecycle_TARGETS}
  ${nav2_core_TARGETS}
  ${nav2_costmap_2d_TARGETS}
  ${nav2_util_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${pluginlib_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${tf2_eigen_TARGETS}
  ${visualization_msgs_TARGETS}
)

# ---------------------------------------------------------------------------
# MPPI critic plugins (2026-10-15): WheelFloorCritic (+ PathAlignAheadCritic 2026-10-29), own library so the goal
# checker library does not link nav2_mppi_controller.
# ---------------------------------------------------------------------------

add_library(mowgli_mppi_critics SHARED
  src/wheel_floor_critic.cpp
  src/path_align_ahead_critic.cpp
)
target_include_directories(mowgli_mppi_critics
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(mowgli_mppi_critics
  ${rclcpp_TARGETS}
  ${rclcpp_lifecycle_TARGETS}
  ${nav2_costmap_2d_TARGETS}
  ${pluginlib_TARGETS}
  ${sensor_msgs_TARGETS}
  ${tf2_TARGETS}
  xtensor
)
ament_target_dependencies(mowgli_mppi_critics nav2_mppi_controller)

# ---------------------------------------------------------------------------
# FTCRowController (2026-11-05, row weave study): FTC row law (ported from OpenMower's
# ftc_local_planner, GPLv3) on straight coverage rows, an inner MPPIController (loaded through
# pluginlib under the same name) everywhere else. Own library: it does not link MPPI.
# ---------------------------------------------------------------------------

add_library(mowgli_ftc_row_controller SHARED
  src/ftc_row_controller.cpp
)
target_include_directories(mowgli_ftc_row_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(mowgli_ftc_row_controller
  ${rclcpp_TARGETS}
  ${rclcpp_lifecycle_TARGETS}
  ${nav2_core_TARGETS}
  ${nav2_costmap_2d_TARGETS}
  ${nav2_util_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${pluginlib_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
)

# pluginlib needs the export macro defined.
pluginlib_export_plugin_description_file(nav2_core goal_checker_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core ftc_row_controller_plugin.xml)
pluginlib_export_plugin_description_file(nav2_mppi_controller mppi_critics_plugin.xml)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME} mowgli_mppi_critics mowgli_ftc_row_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES goal_checker_plugin.xml mppi_critics_plugin.xml ftc_row_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Testing
# ---------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  # Exclude uncrustify — we use clang-format (see .clang-format)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # WheelFloorCritic math + batch scoring (no ROS graph).
  ament_add_gtest(test_wheel_floor_critic
    test/test_wheel_floor_critic.cpp
  )
  target_link_libraries(test_wheel_floor_critic mowgli_mppi_critics)

  # PathAlignAheadCritic scoring (B13-1): control point 0 == PathAlignCritic, look-ahead.
  ament_add_gtest(test_path_align_ahead_critic
    test/test_path_align_ahead_critic.cpp
  )
  target_link_libraries(test_path_align_ahead_critic mowgli_mppi_critics)

  # Path trim (slope creep, 2026-11-03): integral, feed-forward, path shift.
  ament_add_gtest(test_path_trim
    test/test_path_trim.cpp
  )
  target_link_libraries(test_path_trim mowgli_mppi_critics)

  # FTC row law (2026-11-05): path geometry, carrot, PID, row gate, closed loop.
  ament_add_gtest(test_ftc_row_law
    test/test_ftc_row_law.cpp
  )
  target_include_directories(test_ftc_row_law PRIVATE include)

  ament_add_gtest(test_leg_goal_checker
    test/test_leg_goal_checker.cpp
  )
  target_link_libraries(test_leg_goal_checker ${PROJECT_NAME})

  # PORT: test_lyrical_goal_checker dropped (Kilted-only goal-checker API).
endif()

# ---------------------------------------------------------------------------
# Package export
# ---------------------------------------------------------------------------

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} mowgli_mppi_critics mowgli_ftc_row_controller)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  visualization_msgs
  sensor_msgs
  nav2_mppi_controller
)

ament_package()
