cmake_minimum_required(VERSION 3.5)
project(hw1_pkg VERSION 0.1 LANGUAGES CXX)

# C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Qt 자동 처리
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# ROS2 & Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost 1.71.0 REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)

# Qt5 Widgets
find_package(Qt5 COMPONENTS Widgets Core Gui Network REQUIRED)

# UI 경로 (현재 소스 기준)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# 소스, 헤더, UI, 리소스 파일
file(GLOB_RECURSE SOURCES src/*.cpp)
file(GLOB_RECURSE HEADERS include/${PROJECT_NAME}/*.h include/${PROJECT_NAME}/*.hpp)
file(GLOB_RECURSE UI_FILES ui/*.ui)
file(GLOB_RECURSE RESOURCES resources/*.qrc)

# Include directories
include_directories(
    include
    ${CMAKE_CURRENT_BINARY_DIR}  # UIC가 생성한 헤더 포함
)

# 실행 파일
add_executable(${PROJECT_NAME}
    ${SOURCES}
    ${HEADERS}
    ${RESOURCES}  # qrc 자동 처리
)

# ROS2 의존성
ament_target_dependencies(${PROJECT_NAME} rclcpp Boost)

# Qt5 라이브러리 링크
target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets Qt5::Core Qt5::Gui Qt5::Network
    ${OpenCV_LIBS}
)

# 설치
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

# 테스트
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
