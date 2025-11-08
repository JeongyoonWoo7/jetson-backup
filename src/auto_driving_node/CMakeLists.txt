cmake_minimum_required(VERSION 3.8)
project(auto_driving_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

add_executable(auto_driving_node src/auto_driving_node.cpp)
ament_target_dependencies(auto_driving_node
  rclcpp
  geometry_msgs
  vision_msgs
)

install(TARGETS
  auto_driving_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
