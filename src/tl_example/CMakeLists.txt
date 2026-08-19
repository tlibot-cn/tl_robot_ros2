cmake_minimum_required(VERSION 3.8)
project(tl_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tl_ros2_interface REQUIRED)

add_executable(medical_demo
  src/medical_demo.cpp
)

target_include_directories(medical_demo PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

ament_target_dependencies(medical_demo
  rclcpp
  std_srvs
  std_msgs
  tl_ros2_interface
)

install(TARGETS medical_demo
  DESTINATION lib/${PROJECT_NAME}
)

# Python 驱动快速自检脚本（ros2 run tl_example test_all_interfaces）
install(PROGRAMS scripts/test_all_interfaces.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
