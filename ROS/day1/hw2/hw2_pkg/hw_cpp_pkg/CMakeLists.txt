cmake_minimum_required(VERSION 3.8)
project(hw_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 각각 다른 실행파일로 빌드
add_executable(hw2_node1 src/my_cpp_node1.cpp)
ament_target_dependencies(hw2_node1 rclcpp std_msgs)
target_include_directories(hw2_node1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(hw2_node2 src/my_cpp_node2.cpp)
ament_target_dependencies(hw2_node2 rclcpp std_msgs)
target_include_directories(hw2_node2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 설치 경로 지정
install(TARGETS
  hw2_node1
  hw2_node2
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

