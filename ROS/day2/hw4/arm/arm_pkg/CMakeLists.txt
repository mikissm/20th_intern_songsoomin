cmake_minimum_required(VERSION 3.5)
project(arm_pkg)

# C++ 표준
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt 자동 처리
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# ROS2 및 Qt 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)

# 소스, 헤더, 리소스
set(SOURCES
    src/main.cpp
    src/mainwindow.cpp
)
set(HEADERS
    include/arm_pkg/mainwindow.h
)

# include 디렉토리
include_directories(
    include/
    ${CMAKE_CURRENT_BINARY_DIR}  # ui_*.h 생성 위치
)

# 실행 파일 생성
add_executable(robot_arm_gui
    ${SOURCES}
    ${HEADERS}
)

# ROS2 + Qt 라이브러리 연결
ament_target_dependencies(robot_arm_gui
    rclcpp
    std_msgs
)
target_link_libraries(robot_arm_gui Qt5::Widgets Qt5::Core Qt5::Gui)

# 설치
install(TARGETS robot_arm_gui DESTINATION lib/${PROJECT_NAME})

# 테스트
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
