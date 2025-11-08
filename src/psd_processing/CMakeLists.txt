cmake_minimum_required(VERSION 3.8)
project(psd_processing)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(psd_filter_node src/psd_filter_node.cpp)
ament_target_dependencies(psd_filter_node
  rclcpp
  std_msgs
)

install(TARGETS
  psd_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

