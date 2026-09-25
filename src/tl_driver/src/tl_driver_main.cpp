/// @file tl_driver_main.cpp
/// @brief 进程入口：初始化 rclcpp、启动多线程执行器并运行节点。

#include <algorithm>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "node/driver_node.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto thread_num = std::max(4u, std::thread::hardware_concurrency());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), thread_num, true);
  auto node = std::make_shared<tl_driver::DriverNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
