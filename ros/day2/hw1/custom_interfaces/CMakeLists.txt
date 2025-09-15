cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

# C++ 컴파일 옵션 (선택 사항)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 의존성
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 메시지 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AddTwoInts.msg"
)

# 패키지 선언
ament_package()

