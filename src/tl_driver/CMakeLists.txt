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

# 根据目标架构选择对应的动态库目录
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(TL_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/arm")
  set(TL_ARCH "aarch64")
  message(STATUS "tl_driver: detected ARM64 architecture, using lib/arm/")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|i686|i386")
  set(TL_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/x86")
  set(TL_ARCH "x86_64")
  message(STATUS "tl_driver: detected x86_64 architecture, using lib/x86/")
else()
  message(FATAL_ERROR "tl_driver: unsupported architecture '${CMAKE_SYSTEM_PROCESSOR}', expected x86_64 or aarch64")
endif()

# 导入已有动态库（仅 _tl_host.so 被 C++ 链接）
add_library(tl_host_lib SHARED IMPORTED)
set_target_properties(tl_host_lib PROPERTIES
  IMPORTED_LOCATION "${TL_LIB_DIR}/_tl_host.so"
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
  tl_host_lib
  Python3::Python
  pthread
  dl
  rt
)

set_target_properties(tl_driver PROPERTIES
  BUILD_RPATH "${TL_LIB_DIR}"
  INSTALL_RPATH "$ORIGIN"
)

install(TARGETS tl_driver
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 TL 动态库及 Python 接口
# C++ 链接的 _tl_host.so 和 Python 使用的 tl_interface.py
install(FILES
  "${TL_LIB_DIR}/_tl_host.so"
  "${TL_LIB_DIR}/tl_interface.py"
  DESTINATION lib/${PROJECT_NAME}
)

# 创建 libnrc_host.so 符号链接，兼容旧版 SONAME 引用
install(CODE "
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
    _tl_host.so
    \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/libnrc_host.so\"
  )
")

# ARM64 额外的 Python ctypes 依赖库（x86_64 版没有这些）
if(TL_ARCH STREQUAL "aarch64")
  install(FILES
    "${TL_LIB_DIR}/libtl_host.so"
    "${TL_LIB_DIR}/libmath_wrapper.so"
    "${TL_LIB_DIR}/libmodbus_wrapper.so"
    "${TL_LIB_DIR}/libservoJ_wrapper.so"
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()