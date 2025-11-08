cmake_minimum_required(VERSION 3.8)
project(turtlebot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)   # 여기서 꼭 찾는다

add_executable(master_node
  src/main.cpp
  src/master.cpp
)

# 이 패키지 안의 include/ 를 보이게
target_include_directories(master_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 의존성 붙이기: 이 줄이 있어야 custom_interfaces 가 만들어준 헤더 경로가 타겟에 붙는다
ament_target_dependencies(master_node
  rclcpp
  std_msgs
  custom_interfaces
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS master_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
