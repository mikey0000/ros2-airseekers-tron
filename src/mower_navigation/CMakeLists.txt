cmake_minimum_required(VERSION 3.8)
project(mower_navigation NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch behavior_trees
  DESTINATION share/${PROJECT_NAME})

# collision_zone_gate (2026-10-09): makes the collision_monitor stop/slowdown zones forward-only.
install(PROGRAMS scripts/collision_zone_gate.py
  DESTINATION lib/${PROJECT_NAME})
# nav_lane_watchdog (2026-10-20): detects / recovers a silently dropping velocity lane.
install(PROGRAMS scripts/nav_lane_watchdog.py
  DESTINATION lib/${PROJECT_NAME})

# Activity-gated lifecycle manager: transitions Nav2 nodes inactive at idle
install(PROGRAMS mower_navigation/activity_gated_lifecycle.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_params_yaml test/test_params_yaml.py TIMEOUT 60)
  ament_add_pytest_test(test_nav2_param_tools test/test_nav2_param_tools.py TIMEOUT 60)
  ament_add_pytest_test(test_collision_zone_gate test/test_collision_zone_gate.py TIMEOUT 60)
  ament_add_pytest_test(test_nav_lane_watchdog test/test_nav_lane_watchdog.py TIMEOUT 60)
  ament_add_pytest_test(test_activity_gated_lifecycle test/test_activity_gated_lifecycle.py
    TIMEOUT 60)
endif()

ament_package()
