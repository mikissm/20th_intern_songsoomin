cmake_minimum_required(VERSION 3.8)
project(cmd_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)

include_directories(include)

add_executable(cmd_node
  src/main.cpp
  src/mainwindow.cpp
)

ament_target_dependencies(cmd_node
  rclcpp
  custom_interfaces
)

install(TARGETS
  cmd_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

