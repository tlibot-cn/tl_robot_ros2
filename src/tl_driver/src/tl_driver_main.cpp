/// @file tl_driver_main.cpp
/// @brief 进程入口：初始化 rclcpp、启动多线程执行器并运行节点。

#include <algorithm>
#include <cstdlib>
#include <exception>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "node/driver_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // 回调组共 9 个（6 个服务域 + get_robot_state + topic + timer）。线程数至少覆盖全部组，
  // 保证慢服务占用线程时 100 Hz 状态发布仍有线程可用。
  constexpr unsigned int kMinExecutorThreads = 10;
  auto thread_num = std::max(kMinExecutorThreads, std::thread::hardware_concurrency());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), thread_num, true);

  try
  {
    auto node = std::make_shared<tl_driver::DriverNode>();
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
