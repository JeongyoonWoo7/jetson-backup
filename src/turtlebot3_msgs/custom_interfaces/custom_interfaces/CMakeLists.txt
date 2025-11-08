cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 여기 있는 msg 들로 C++/Python 인터페이스를 생성해라
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Colcon.msg"
  "msg/PacketProtocol.msg"
  "msg/RobotArm.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

