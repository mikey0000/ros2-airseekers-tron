cmake_minimum_required(VERSION 3.8)
project(mower_costmap_layers)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
add_library(${PROJECT_NAME} SHARED src/decaying_obstacle_layer.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
  nav2_costmap_2d::layers nav2_costmap_2d::nav2_costmap_2d_core pluginlib::pluginlib
  rclcpp::rclcpp)
pluginlib_export_plugin_description_file(nav2_costmap_2d layers_plugin.xml)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_package()
