cmake_minimum_required(VERSION 3.8)
project(mower_coverage)

# Fields2Cover 2.1.0 headers are cxx_std_17; the F2C deb's exported target
# compiles clean under the Noble toolchain (g++ 13).
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(mower_interfaces REQUIRED)

# --- Fields2Cover -------------------------------------------------------------
# F2C_V3=ON (default): Fields2Cover v3 (3.0.0, upstream `v3.0` branch), built
#   ONCE per arch by docker/fields2cover/Dockerfile into the image layer
#   `mower-f2c:v3-<arch>` and copied to /opt/fields2cover by
#   Dockerfile.jazzy / Dockerfile.dev-amd64 (never compiled on the mower).
#   v3 links ortools::ortools PUBLIC but its Fields2CoverConfig.cmake does not
#   find_dependency(ortools), so define the target here from
#   ros-jazzy-ortools-vendor.
# F2C_V3=OFF: the apt package `ros-jazzy-fields2cover` (2.1.0) as before.
#   It installs libFields2Cover.so (no SONAME) + libmatplot into /opt/ros/jazzy,
#   which shadows v3 via LD_LIBRARY_PATH, so an image carries only ONE of the
#   two (Dockerfile.jazzy build-arg F2C_V3=0 selects apt 2.1).
#   Fields2Cover is NOT a <depend> in package.xml (no rosdep key).
# Fields2Cover version selection, re-evaluated on EVERY configure (a cached
# option went stale on the mower: an old "OFF" cache + a v3-only image produced
# a 2.1-API binary linked to the v3 library -> undefined symbol at start-up).
# v3 is used when its install tree is present unless F2C_FORCE_21=ON.
option(F2C_FORCE_21 "Force the apt Fields2Cover 2.1 package even if v3 is installed" OFF)
if(EXISTS "/opt/fields2cover/lib/cmake/Fields2Cover" AND NOT F2C_FORCE_21)
  set(F2C_V3 ON)
else()
  set(F2C_V3 OFF)
endif()
unset(F2C_V3 CACHE)
set(F2C_V3_PREFIX "/opt/fields2cover" CACHE PATH "Fields2Cover v3 install prefix")
if(F2C_V3)
  find_package(ortools_vendor QUIET)
  find_package(ortools CONFIG REQUIRED)
  find_package(Fields2Cover 3.0 CONFIG REQUIRED
    PATHS ${F2C_V3_PREFIX} NO_DEFAULT_PATH)
  message(STATUS "mower_coverage: Fields2Cover ${Fields2Cover_VERSION} (v3) from ${Fields2Cover_DIR}")
else()
  find_package(Fields2Cover 2.1.0 CONFIG REQUIRED)
  message(STATUS "mower_coverage: Fields2Cover ${Fields2Cover_VERSION} (apt 2.1 fallback)")
endif()

# Pure-geometry planner core: no ROS types in its signature, so it is unit
# testable against the real F2C library with no robot and no ROS node.
add_library(mower_coverage_core SHARED src/coverage_planning.cpp)
target_include_directories(mower_coverage_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
# v3 source installs export plain `Fields2Cover`; the apt 2.1 deb exports the
# namespaced target.
if(TARGET Fields2Cover::Fields2Cover)
  target_link_libraries(mower_coverage_core Fields2Cover::Fields2Cover)
else()
  target_link_libraries(mower_coverage_core Fields2Cover)
endif()
if(F2C_V3)
  target_compile_definitions(mower_coverage_core PUBLIC MOWER_COVERAGE_F2C_V3=1)
  # F2C headers trip -Wshadow etc.; keep them SYSTEM.
  target_include_directories(mower_coverage_core SYSTEM PUBLIC ${F2C_V3_PREFIX}/include)
  set_target_properties(mower_coverage_core PROPERTIES
    INSTALL_RPATH "${F2C_V3_PREFIX}/lib" BUILD_RPATH "${F2C_V3_PREFIX}/lib")
endif()

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
