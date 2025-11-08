cmake_minimum_required(VERSION 3.8)
project(lane_follower)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include/lane_follower
)

add_executable(lane_follower_node src/lane_follower_node.cpp)
ament_target_dependencies(lane_follower_node
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  lane_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()