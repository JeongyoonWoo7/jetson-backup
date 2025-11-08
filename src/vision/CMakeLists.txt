cmake_minimum_required(VERSION 3.16)
project(vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Qt 자동 처리
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui Network)
find_package(vision_msgs REQUIRED)

# Sources, headers, UI
file(GLOB_RECURSE SOURCES src/*.cpp)
file(GLOB_RECURSE HEADERS include/vision/*.hpp)
set(UI_FILES
    ui/main_ui.ui
    ui/move_ui.ui
)

include_directories(
    include
    ${CMAKE_CURRENT_BINARY_DIR}  # uic-generated headers
)

add_executable(vision_node
    ${SOURCES}
    ${HEADERS}  # 헤더도 포함
    ${UI_FILES} # UI 파일 포함
)

ament_target_dependencies(vision_node
    rclcpp
    sensor_msgs
    cv_bridge
    vision_msgs
)

target_link_libraries(vision_node
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui
    Qt5::Network
    ${OpenCV_LIBS}
)

install(TARGETS vision_node DESTINATION lib/${PROJECT_NAME})

ament_package()

