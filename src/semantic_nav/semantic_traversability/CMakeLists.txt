cmake_minimum_required(VERSION 3.16)
project(semantic_traversability)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)

set(LAYER_DEPS
  rclcpp
  pluginlib
  nav2_costmap_2d
  geometry_msgs
  tf2
  tf2_geometry_msgs
  btcpp_ros2_interfaces
)

# --- Costmap layer plugin (shared lib loaded by nav2 via pluginlib) ---
add_library(semantic_traversability_layer SHARED
  src/semantic_traversability_layer.cpp
)
target_include_directories(semantic_traversability_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(semantic_traversability_layer ${LAYER_DEPS})

# --- Spatial projection node (detection + depth -> ground polygon) ---
set(PROJ_DEPS
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  btcpp_ros2_interfaces
)
add_executable(projection_node src/projection_node.cpp)
ament_target_dependencies(projection_node ${PROJ_DEPS})

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS semantic_traversability_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS projection_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export the costmap layer to pluginlib
pluginlib_export_plugin_description_file(nav2_costmap_2d
  nav2_semantic_traversability_plugin.xml)

ament_export_include_directories(include)
ament_export_libraries(semantic_traversability_layer)
ament_export_dependencies(${LAYER_DEPS})

ament_package()
