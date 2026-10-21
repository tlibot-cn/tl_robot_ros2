/// @file topics.cpp
/// @brief 话题订阅：moveJ/moveL/servoJ 的 handler 与订阅注册。

#include <algorithm>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "arm_client.hpp"
#include "arm_sdk.hpp"
#include "convert/msg_convert.hpp"
#include "convert/units.hpp"
#include "node/driver_node.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "tl_ros2_interface/msg/move_command.hpp"

namespace tl_driver
{
namespace
{
void handle_movej_topic(DriverNode& node, const tl_ros2_interface::msg::MoveCommand::SharedPtr msg)
{
  if (!node.session().is_connected())
  {
    RCLCPP_WARN(node.get_logger(), "[MoveJ]: arm is not connected");
    return;
  }
  if (node.motion_action_active())
  {
    RCLCPP_WARN(node.get_logger(), "[MoveJ]: ignored while a motion action is active");
    return;
  }

  tl::MoveCmd cmd = convert::to_move_cmd(*msg, convert::MotionKind::kJoint);

  tl::Result ret = tl::robot_movej(node.session().fd(), cmd);
  // 即使返回超时，指令也可能已生效；无条件记录以覆盖紧随其后的 action goal。
  node.note_external_motion();

  RCLCPP_INFO(node.get_logger(), "[MoveJ]: result=%s", arm_client::result_to_string(ret));
}

void handle_movel_topic(DriverNode& node, const tl_ros2_interface::msg::MoveCommand::SharedPtr msg)
{
  if (!node.session().is_connected())
  {
    RCLCPP_WARN(node.get_logger(), "[MoveL]: arm is not connected");
    return;
  }
  if (node.motion_action_active())
  {
    RCLCPP_WARN(node.get_logger(), "[MoveL]: ignored while a motion action is active");
    return;
  }

  tl::MoveCmd cmd = convert::to_move_cmd(*msg, convert::MotionKind::kLinear);

  tl::Result ret = tl::robot_movel(node.session().fd(), cmd);
  node.note_external_motion();

  RCLCPP_INFO(node.get_logger(), "[MoveL]: result=%s", arm_client::result_to_string(ret));
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
} // namespace

void register_topics(DriverNode& node)
{
  node.create_subscription<tl_ros2_interface::msg::MoveCommand>(
      "/tl_driver/moveJ", 10,
      [&node](const tl_ros2_interface::msg::MoveCommand::SharedPtr msg)
      {
        handle_movej_topic(node, msg);
      });
  node.create_subscription<tl_ros2_interface::msg::MoveCommand>(
      "/tl_driver/moveL", 10,
      [&node](const tl_ros2_interface::msg::MoveCommand::SharedPtr msg)
      {
        handle_movel_topic(node, msg);
      });
  node.create_subscription<std_msgs::msg::Float64MultiArray>(
      "/tl_driver/set_servoj_pos", 10,
      [&node](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
      {
        handle_set_servoj_pos_topic(node, msg);
      });
}
} // namespace tl_driver
