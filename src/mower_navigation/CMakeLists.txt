cmake_minimum_required(VERSION 3.8)
project(mower_navigation NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch behavior_trees
  DESTINATION share/${PROJECT_NAME})

ament_package()
