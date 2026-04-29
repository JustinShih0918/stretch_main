cmake_minimum_required(VERSION 3.16)
project(bt_engine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(bt_nav REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(bt_engine src/bt_engine.cpp)
target_include_directories(bt_engine PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(bt_engine
  rclcpp
  std_msgs
  behaviortree_cpp
  behaviortree_ros2
  bt_nav
  ament_index_cpp
)

install(TARGETS bt_engine
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY bt/
  DESTINATION share/${PROJECT_NAME}/bt
)

ament_package()
