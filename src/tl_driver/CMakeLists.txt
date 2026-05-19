cmake_minimum_required(VERSION 3.8)
project(tl_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)
find_package(tl_ros2_interface REQUIRED)
find_package(std_msgs REQUIRED)

# 导入已有动态库
add_library(nrc_host_lib SHARED IMPORTED)
set_target_properties(nrc_host_lib PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/_nrc_host.so"
)

add_executable(tl_driver
  src/tl_driver.cpp
)

target_include_directories(tl_driver PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/include/cpp/interface"
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/include/cpp/parameter"
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/include/c/interface"
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/include/c/parameter"
)

ament_target_dependencies(tl_driver
  rclcpp
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tl_ros2_interface
  std_msgs
)

target_link_libraries(tl_driver
  nrc_host_lib
  Python3::Python
  pthread
  dl
  rt
)

set_target_properties(tl_driver PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib"
  INSTALL_RPATH "$ORIGIN"
)

install(TARGETS tl_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/_nrc_host.so"
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/nrc_interface.py"
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()