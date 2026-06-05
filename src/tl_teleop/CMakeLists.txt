cmake_minimum_required(VERSION 3.8)
project(tl_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tl_ros2_interface REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(sensor_msgs REQUIRED)

# 根据目标架构选择 PXREA SDK 库目录
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(PXREA_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/arm")
  message(STATUS "tl_teleop: detected ARM64 architecture, using lib/arm/")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|i686|i386")
  set(PXREA_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/x86")
  message(STATUS "tl_teleop: detected x86_64 architecture, using lib/x86/")
else()
  message(FATAL_ERROR "tl_teleop: unsupported architecture '${CMAKE_SYSTEM_PROCESSOR}'")
endif()

# 导入 PXREA Robot SDK 预编译动态库
add_library(pxrea_sdk SHARED IMPORTED)
set_target_properties(pxrea_sdk PROPERTIES
  IMPORTED_LOCATION "${PXREA_SDK_DIR}/libPXREARobotSDK.so"
)

add_executable(tl_teleop
  src/tl_teleop.cpp
)

target_include_directories(tl_teleop PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/include"
)

ament_target_dependencies(tl_teleop
  rclcpp
  std_msgs
  std_srvs
  tl_ros2_interface
  nlohmann_json
  sensor_msgs
)

target_link_libraries(tl_teleop
  pxrea_sdk
  pthread
  dl
)

set_target_properties(tl_teleop PROPERTIES
  BUILD_RPATH "${PXREA_SDK_DIR}"
  INSTALL_RPATH "$ORIGIN"
)

install(TARGETS tl_teleop
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 PXREA Robot SDK 动态库
install(FILES
  "${PXREA_SDK_DIR}/libPXREARobotSDK.so"
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()