#include <algorithm>
#include <cstdlib>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "tl_driver/tl_driver.h"

/**
 * @brief tl_driver 进程入口：多线程执行器装配与节点生命周期。
 * @note 线程数为 max(4, 硬件并发数)，与拆分前一致。
 */

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
