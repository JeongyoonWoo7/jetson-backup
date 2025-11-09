cmake_minimum_required(VERSION 3.8)
project(auto_drive_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(auto_driving_node src/auto_drive.cpp)
ament_target_dependencies(auto_driving_node
  rclcpp
  geometry_msgs
  vision_msgs
  sensor_msgs
  std_msgs
  tf2
)

install(TARGETS
  auto_driving_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
