cmake_minimum_required(VERSION 3.8)
project(obstacle_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(nav2_dynamic_msgs REQUIRED)

add_executable(odpp_node src/odpp_node.cpp)

ament_target_dependencies(odpp_node
  rclcpp
  rclcpp_lifecycle
  nav2_costmap_2d
  geometry_msgs
  std_msgs
  unique_identifier_msgs
  nav2_dynamic_msgs
)

install(TARGETS odpp_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()