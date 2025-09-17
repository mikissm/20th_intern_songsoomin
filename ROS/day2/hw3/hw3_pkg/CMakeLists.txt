cmake_minimum_required(VERSION 3.5)
project(hw3_pkg)

# C++ 표준
set(CMAKE_CXX_STANDARD 11)
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
find_package(Qt5 COMPONENTS Widgets Core Gui Network REQUIRED)

# 소스, 헤더, 리소스
file(GLOB PRO_SOURCE_DIR src/*.cpp)
file(GLOB PRO_INCLUDE_DIR include/${PROJECT_NAME}/*.h include/${PROJECT_NAME}/*.hpp)
file(GLOB PRO_RESOURCE_DIR resources/*.qrc)

# Qt 리소스 처리
qt5_add_resources(QT_RESOURCE_HPP ${PRO_RESOURCE_DIR})

# include 디렉토리
include_directories(
    include/
    ${CMAKE_CURRENT_BINARY_DIR}  # ui_*.h 생성 위치
)

# 실행 파일 생성
add_executable(${PROJECT_NAME}
    ${PRO_SOURCE_DIR}
    ${PRO_INCLUDE_DIR}
    ${QT_RESOURCE_HPP}
)

# ROS2 + Qt 라이브러리 연결
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Core Qt5::Gui Qt5::Network)

# 설치
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

# 테스트
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()

