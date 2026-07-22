cmake_minimum_required(VERSION 3.8)
project(tl_teleop_f710)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tl_ros2_interface REQUIRED)

# ─── tl_teleop_f710_node ───
add_executable(tl_teleop_f710_node
  src/tl_teleop_f710_node.cpp
)

target_include_directories(tl_teleop_f710_node PRIVATE
  include
)

ament_target_dependencies(tl_teleop_f710_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tl_ros2_interface
  kdl_parser
  orocos_kdl_vendor
  ament_index_cpp
)

# ─── tl_teleop_f710_sim_bridge ───
add_executable(tl_teleop_f710_sim_bridge
  src/tl_teleop_f710_sim_bridge.cpp
)

target_include_directories(tl_teleop_f710_sim_bridge PRIVATE
  include
)

ament_target_dependencies(tl_teleop_f710_sim_bridge
  rclcpp
  sensor_msgs
  std_msgs
  tl_ros2_interface
  kdl_parser
  orocos_kdl_vendor
  ament_index_cpp
)

# ─── Install ───
install(TARGETS
  tl_teleop_f710_node
  tl_teleop_f710_sim_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  udev
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
