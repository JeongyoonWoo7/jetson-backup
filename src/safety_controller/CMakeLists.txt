cmake_minimum_required(VERSION 3.10)
project(safety_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(psd_safety_controller src/psd_safety_controller.cpp)
ament_target_dependencies(psd_safety_controller
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  psd_safety_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

