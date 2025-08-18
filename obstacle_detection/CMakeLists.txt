cmake_minimum_required(VERSION 3.8)
project(obstacle_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_dynamic_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

add_executable(dynamic_obstacle_node
  src/odpp_node.cpp
  src/cluster_engine.cpp
)

target_include_directories(dynamic_obstacle_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dynamic_obstacle_node
  rclcpp
  rclcpp_lifecycle
  nav2_costmap_2d
  geometry_msgs
  visualization_msgs
  nav2_dynamic_msgs
  unique_identifier_msgs
)

install(TARGETS dynamic_obstacle_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()