cmake_minimum_required(VERSION 3.8)
project(mowgli_nav2_plugins)

# Humble port of third_party/mowglinext/mowgli_nav2_plugins (Kilted):
# rclcpp_lifecycle node, setPlan/3-arg computeVelocityCommands, PlannerException,
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
find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------------------
# Shared library: mowgli_nav2_plugins
# ---------------------------------------------------------------------------

add_library(${PROJECT_NAME} SHARED
  src/oscillation_detector.cpp
  src/obstacle_deviation.cpp
  src/ftc_controller.cpp
  src/ftc_controller_plugin.cpp
  src/path_progress_goal_checker.cpp
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

# pluginlib needs the export macro defined.
pluginlib_export_plugin_description_file(nav2_core ftc_controller_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core goal_checker_plugin.xml)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES ftc_controller_plugin.xml goal_checker_plugin.xml
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

  ament_add_gtest(test_leg_goal_checker
    test/test_leg_goal_checker.cpp
  )
  target_link_libraries(test_leg_goal_checker ${PROJECT_NAME})

  # PORT: test_lyrical_goal_checker dropped (Kilted-only goal-checker API).

  ament_add_gtest(test_oscillation_detector
    test/test_oscillation_detector.cpp
    src/oscillation_detector.cpp
  )

  target_include_directories(test_oscillation_detector
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_obstacle_deviation
    test/test_obstacle_deviation.cpp
    src/obstacle_deviation.cpp
  )

  target_include_directories(test_obstacle_deviation
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  target_link_libraries(test_obstacle_deviation
    ${geometry_msgs_TARGETS}
    ${nav2_costmap_2d_TARGETS}
    ${tf2_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
  )

  ament_add_gtest(test_ftc_start_index
    test/test_ftc_start_index.cpp
  )

  target_include_directories(test_ftc_start_index
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_ftc_stall
    test/test_ftc_stall.cpp
  )

  target_include_directories(test_ftc_stall
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_ftc_blade_load
    test/test_ftc_blade_load.cpp
  )

  target_include_directories(test_ftc_blade_load
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_ftc_reverse_escape
    test/test_ftc_reverse_escape.cpp
  )

  target_include_directories(test_ftc_reverse_escape
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_ftc_obstacle_wait
    test/test_ftc_obstacle_wait.cpp
  )

  target_include_directories(test_ftc_obstacle_wait
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  # Closed-loop unicycle simulation of the real FTCController (Tron MCU clamp).
  # Localhost-only + private domain: the dev container runs --network host and
  # must never be visible to the mower's DDS graph.
  ament_add_gtest(test_ftc_tracking_sim
    test/test_ftc_tracking_sim.cpp
    ENV ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=97
    TIMEOUT 120
  )
  target_link_libraries(test_ftc_tracking_sim ${PROJECT_NAME})
  target_include_directories(test_ftc_tracking_sim SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS})

  # PRE_ROTATE -> FOLLOWING hand-off with a 2*pi offset in the heading unwrap.
  ament_add_gtest(test_ftc_angle_wrap
    test/test_ftc_angle_wrap.cpp
    ENV ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=97
    TIMEOUT 60
  )
  target_link_libraries(test_ftc_angle_wrap ${PROJECT_NAME})
  target_include_directories(test_ftc_angle_wrap SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS})
endif()

# ---------------------------------------------------------------------------
# Package export
# ---------------------------------------------------------------------------

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
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
)

ament_package()
