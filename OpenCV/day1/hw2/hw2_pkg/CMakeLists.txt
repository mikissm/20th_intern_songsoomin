cmake_minimum_required(VERSION 3.5)
project(hw2_pkg VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# ROS2 & Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost 1.71.0 REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)

# Qt
find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets Core Gui Network REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Core Gui Network REQUIRED)

# UI 경로
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_SOURCE_DIR}/ui)

# 소스, 헤더, UI, 리소스 파일
file(GLOB_RECURSE SOURCES src/*.cpp)
file(GLOB_RECURSE HEADERS include/${PROJECT_NAME}/*.h include/${PROJECT_NAME}/*.hpp)
file(GLOB_RECURSE UI_FILES ui/*.ui)
file(GLOB_RECURSE RESOURCES resources/*.qrc)

# Qt wrap
qt5_wrap_ui(UI_HEADERS ${UI_FILES})
qt5_wrap_cpp(MOC_HEADERS ${HEADERS})
qt5_add_resources(QT_RESOURCES ${RESOURCES})

# Include directories
include_directories(
    include
    ${CMAKE_CURRENT_BINARY_DIR}
)

# 실행 파일
add_executable(${PROJECT_NAME}
    ${SOURCES}
    ${HEADERS}
    ${UI_HEADERS}
    ${MOC_HEADERS}
    ${QT_RESOURCES}
)

# 링크 및 의존성
ament_target_dependencies(${PROJECT_NAME} rclcpp Boost)
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

