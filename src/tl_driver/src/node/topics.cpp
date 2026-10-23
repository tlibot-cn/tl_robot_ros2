/// @file topics.cpp
/// @brief 话题订阅：servoJ 目标位置（set_servoj_pos）的 handler 与订阅注册。

#include <algorithm>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "arm_client.hpp"
#include "arm_sdk.hpp"
#include "convert/units.hpp"
#include "node/driver_node.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

namespace tl_driver
{
namespace
{
/**
 * @brief 透传模式必须先使用open_servoJ开启跟踪模式
 */
void handle_set_servoj_pos_topic(DriverNode& node, const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!node.session().is_connected())
  {
    RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "[ServoJ]: arm is not connected");
    return;
  }
  if (node.motion_action_active())
  {
    RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000,
                         "[ServoJ]: ignored while a motion action is active");
    return;
  }

  // ROS 侧为 rad，SDK 要求度；SDK 固定 7 元素，6 轴在末尾补 0。
  const auto& data = msg->data;
  if (data.size() != 6 && data.size() != 7)
  {
    RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000,
                         "[ServoJ]: data must have 6 or 7 elements, got %zu", data.size());
    return;
  }
  std::vector<double> pos(7, 0.0); // 一次分配；6 轴时第 7 位保持 0
  std::copy(data.begin(), data.end(), pos.begin());
  convert::joint_rad_to_deg(pos, pos.size());
  // SDK 按值接收：move 掉本次缓冲，避免再拷贝一份。
  tl::Result ret = tl::set_servoJ_pos(node.session().fd_aux(), std::move(pos));
  // 250 Hz 调用：成功降为可节流的 DEBUG，失败保留 WARN 但每秒最多一条。
  if (ret == tl::Result::SUCCESS)
  {
    RCLCPP_DEBUG_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "[ServoJ]: result=%s",
                          arm_client::result_to_string(ret));
  }
  else
  {
    RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "[ServoJ]: result=%s",
                         arm_client::result_to_string(ret));
  }
}
} // namespace

void register_topics(DriverNode& node)
{
  node.create_subscription<std_msgs::msg::Float64MultiArray>(
      "/tl_driver/set_servoj_pos", 10,
      [&node](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
      {
        handle_set_servoj_pos_topic(node, msg);
      });
}
} // namespace tl_driver
