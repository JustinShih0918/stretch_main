cmake_minimum_required(VERSION 3.16)
project(dynamic_mapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)

add_library(dynamic_map_core
  src/dynamic_map_core.cpp
)
target_include_directories(dynamic_map_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(dynamic_map_updater_node
  src/dynamic_map_updater_node.cpp
)
target_include_directories(dynamic_map_updater_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(dynamic_map_updater_node dynamic_map_core)
ament_target_dependencies(dynamic_map_updater_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  btcpp_ros2_interfaces
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS dynamic_map_core dynamic_map_updater_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dynamic_map_core test/test_dynamic_map_core.cpp)
  target_link_libraries(test_dynamic_map_core dynamic_map_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(dynamic_map_core)
ament_package()
