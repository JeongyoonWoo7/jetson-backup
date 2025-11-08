cmake_minimum_required(VERSION 3.8)
project(imu_yaw_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(imu_yaw_controller src/imu_yaw_controller.cpp)
ament_target_dependencies(imu_yaw_controller
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  imu_yaw_controller
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
