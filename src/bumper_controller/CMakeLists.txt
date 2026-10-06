cmake_minimum_required(VERSION 3.8)
project(bumper_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mower_interfaces REQUIRED)

add_library(${PROJECT_NAME} src/bumper_controller.cpp)
target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp geometry_msgs nav_msgs)

add_executable(bumper_controller_node src/bumper_controller_node.cpp)
ament_target_dependencies(bumper_controller_node
    rclcpp std_msgs std_srvs sensor_msgs mower_interfaces)
target_link_libraries(bumper_controller_node ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(TARGETS bumper_controller_node
    RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bumper_controller test/test_bumper_controller.cpp)
  target_link_libraries(test_bumper_controller ${PROJECT_NAME})
  ament_target_dependencies(test_bumper_controller rclcpp geometry_msgs nav_msgs)
endif()

ament_package()
