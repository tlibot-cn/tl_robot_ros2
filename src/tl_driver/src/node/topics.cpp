/// @file topics.cpp
/// @brief 话题订阅：servoJ 目标位置、CSV 目标速度、CST 目标扭矩的 handler 与订阅注册。

#include <algorithm>
#include <cmath>
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
/// CST 扭矩上下限，单位：额定扭矩的 0.1%（即 ‰），与 SDK 单位一致。
constexpr double kCstTorqueMin = -1000.0;
constexpr double kCstTorqueMax = 1000.0;

/// @brief 取 6/7 元素输入并补齐到 7 元素（6 轴时第 7 位外部轴保持 0）。
/// @return false = 元素数非法
bool copy_to_7(const std::vector<double>& data, std::vector<double>& out)
{
  if (data.size() != 6 && data.size() != 7)
  {
    return false;
  }
  out.assign(7, 0.0);
  std::copy(data.begin(), data.end(), out.begin());
  return true;
}

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

/**
 * @brief 周期同步速度透传（CSV）：ROS 侧 rad/s → SDK 度/s。
 * @note 必须先调用 /tl_driver/open_servo_csv，否则 SDK 返回 OPERATION_NOT_ALLOWED。
 */
void handle_set_servo_csv_velocity_topic(DriverNode& node, const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!node.session().is_connected())
  {
    RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "[CSV]: arm is not connected");
    return;
  }
  if (node.motion_action_active())
  {
    RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "[CSV]: ignored while a motion action is active");
    return;
  }

  std::vector<double> velocity;
  if (!copy_to_7(msg->data, velocity))
  {
    RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "[CSV]: data must have 6 or 7 elements, got %zu",
                         msg->data.size());
    return;
  }
  for (const double value : velocity)
  {
    if (!std::isfinite(value))
    {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "[CSV]: data contains NaN/Inf");
      return;
    }
  }
  convert::joint_rad_to_deg(velocity, velocity.size());
  const tl::Result ret = tl::set_servo_csv_velocity(node.session().fd_aux(), velocity);
  if (ret == tl::Result::SUCCESS)
  {
    RCLCPP_DEBUG_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "[CSV]: result=%s",
                          arm_client::result_to_string(ret));
  }
  else
  {
    RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "[CSV]: result=%s",
                         arm_client::result_to_string(ret));
  }
}

/**
 * @brief 周期同步扭矩透传（CST）。
 * @note ROS 侧单位与 SDK 一致（额定扭矩的 0.1%，即 ‰），因此无需换算，只做范围与有效性校验。
 * @note 力矩补偿尚未实现，本话题只做原始力矩透传。
 */
void handle_set_servo_cst_torque_topic(DriverNode& node, const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!node.session().is_connected())
  {
    RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "[CST]: arm is not connected");
    return;
  }
  if (node.motion_action_active())
  {
    RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "[CST]: ignored while a motion action is active");
    return;
  }

  std::vector<double> torque;
  if (!copy_to_7(msg->data, torque))
  {
    RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "[CST]: data must have 6 or 7 elements, got %zu",
                         msg->data.size());
    return;
  }
  for (std::size_t i = 0; i < msg->data.size(); ++i)
  {
    const double value = msg->data[i];
    if (!std::isfinite(value) || value < kCstTorqueMin || value > kCstTorqueMax)
    {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000,
                           "[CST]: torque[%zu]=%.1f out of range [-1000, 1000] (‰ rated torque), ignored", i, value);
      return;
    }
  }
  const tl::Result ret = tl::set_servo_cst_torque(node.session().fd_aux(), torque);
  if (ret == tl::Result::SUCCESS)
  {
    RCLCPP_DEBUG_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "[CST]: result=%s",
                          arm_client::result_to_string(ret));
  }
  else
  {
    RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "[CST]: result=%s",
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
  node.create_subscription<std_msgs::msg::Float64MultiArray>(
      "/tl_driver/set_servo_csv_velocity", 10,
      [&node](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
      {
        handle_set_servo_csv_velocity_topic(node, msg);
      });
  node.create_subscription<std_msgs::msg::Float64MultiArray>(
      "/tl_driver/set_servo_cst_torque", 10,
      [&node](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
      {
        handle_set_servo_cst_torque_topic(node, msg);
      });
}
} // namespace tl_driver
