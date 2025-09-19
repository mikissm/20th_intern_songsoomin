cmake_minimum_required(VERSION 3.8)
project(hw1_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(hw1 src/hw1.cpp src/hw1.cpp)

ament_target_dependencies(hw1 rclcpp)
target_link_libraries(hw1 ${OpenCV_LIBS})

install(TARGETS
  hw1
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

