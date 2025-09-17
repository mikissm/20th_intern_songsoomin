cmake_minimum_required(VERSION 3.8)
project(hw2_2_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# include 디렉토리
include_directories(include)

# 실행 파일 추가
add_executable(odom_broadcaster src/odom_broadcaster.cpp)

# 의존 패키지 연결
ament_target_dependencies(
  odom_broadcaster
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

# 설치 경로 지정 (실행 파일)
install(TARGETS
  odom_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

# launch 파일 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

