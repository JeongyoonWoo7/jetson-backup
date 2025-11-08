cmake_minimum_required(VERSION 3.8)
project(vision_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)  # Header 안에 time이 있어서 필요

# 여기 있는 msg 들로 C++/Python 인터페이스를 생성해라
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneInfo.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

