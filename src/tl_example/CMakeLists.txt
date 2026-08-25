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
find_package(sensor_msgs REQUIRED)
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

# 驱动快速自检：遍历测试 tl_driver 全部服务/话题（危险接口自动跳过）
add_executable(ex_driver_quick_test
  src/ex_driver_quick_test.cpp
)

target_include_directories(ex_driver_quick_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

ament_target_dependencies(ex_driver_quick_test
  rclcpp
  std_srvs
  std_msgs
  sensor_msgs
  tl_ros2_interface
)

install(TARGETS ex_driver_quick_test
  DESTINATION lib/${PROJECT_NAME}
)

# 队列运动模式测试：打开队列 -> 添加两个 moveJ -> 触发执行 -> 关闭
add_executable(ex_queue_test
  src/ex_queue_test.cpp
)

target_include_directories(ex_queue_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

ament_target_dependencies(ex_queue_test
  rclcpp
  std_srvs
  std_msgs
  sensor_msgs
  tl_ros2_interface
)

install(TARGETS ex_queue_test
  DESTINATION lib/${PROJECT_NAME}
)

# ServoJ 插补测试：示教模式下 open_servoj -> 从零点插值运动 -> close_servoj
add_executable(ex_servoj_test
  src/ex_servoj_test.cpp
)

target_include_directories(ex_servoj_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

ament_target_dependencies(ex_servoj_test
  rclcpp
  std_msgs
  std_srvs
  tl_ros2_interface
)

install(TARGETS ex_servoj_test
  DESTINATION lib/${PROJECT_NAME}
)

# 点动（示教）测试：示教模式下关节/直角坐标系点动
add_executable(ex_teach_control
  src/ex_teach_control.cpp
)

target_include_directories(ex_teach_control PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

ament_target_dependencies(ex_teach_control
  rclcpp
  std_srvs
  std_msgs
  sensor_msgs
  tl_ros2_interface
)

install(TARGETS ex_teach_control
  DESTINATION lib/${PROJECT_NAME}
)

# 数字 IO 测试：查询 DI/DO 状态并测试数字输出设置
add_executable(ex_digital_io
  src/ex_digital_io.cpp
)

target_include_directories(ex_digital_io PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

ament_target_dependencies(ex_digital_io
  rclcpp
  std_srvs
  std_msgs
  tl_ros2_interface
)

install(TARGETS ex_digital_io
  DESTINATION lib/${PROJECT_NAME}
)

# 信息查询测试：只读遍历说明书 §4 信息查询接口（话题 + 服务）
add_executable(ex_info_query
  src/ex_info_query.cpp
)

target_include_directories(ex_info_query PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

ament_target_dependencies(ex_info_query
  rclcpp
  std_srvs
  std_msgs
  sensor_msgs
  tl_ros2_interface
)

install(TARGETS ex_info_query
  DESTINATION lib/${PROJECT_NAME}
)

# 运动控制测试：通过话题下发 MoveJ / MoveL 并等待完成
add_executable(ex_move_control
  src/ex_move_control.cpp
)

target_include_directories(ex_move_control PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

ament_target_dependencies(ex_move_control
  rclcpp
  std_srvs
  std_msgs
  sensor_msgs
  tl_ros2_interface
)

install(TARGETS ex_move_control
  DESTINATION lib/${PROJECT_NAME}
)

# ServoJ 关节实时跟踪测试：100Hz 高频下发关节角（透传 7000 端口）
add_executable(ex_servoj
  src/ex_servoj.cpp
)

target_include_directories(ex_servoj PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

ament_target_dependencies(ex_servoj
  rclcpp
  std_srvs
  std_msgs
  sensor_msgs
  tl_ros2_interface
)

install(TARGETS ex_servoj
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
