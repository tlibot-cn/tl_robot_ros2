/// @file tl_driver_main.cpp
/// @brief 进程入口：初始化 rclcpp、启动多线程执行器并运行节点。

#include <cstdlib>
#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "node/driver_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  try
  {
    auto node = std::make_shared<tl_driver::DriverNode>();

    // 回调组固定 8 个（短服务/运动服务/topic/stream 看门狗/dynamics/thermal/status/action）。
    // 线程数由 executor_threads 启动参数决定（默认 max(4, min(hardware_concurrency, 8))）。
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), node->executor_threads(), true);
    executor.add_node(node);
    executor.spin();
  }
  catch (const std::exception& e)
  {
    // 参数类型错误等构造期异常不得逃逸成 std::terminate：返回非零码并给出原因。
    RCLCPP_FATAL(rclcpp::get_logger("tl_driver"), "Fatal error during startup: %s", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
