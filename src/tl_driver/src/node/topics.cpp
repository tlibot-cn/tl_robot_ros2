/// @file topics.cpp
/// @brief 话题订阅：moveJ/moveL/servoJ 的 handler 与订阅注册。

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

  tl::MoveCmd cmd = convert::to_move_cmd(*msg, convert::MotionKind::kJoint);

  tl::Result ret = tl::robot_movej(node.session().fd(), cmd);

  RCLCPP_INFO(node.get_logger(), "[MoveJ]: result=%s", arm_client::result_to_string(ret));
}

void handle_movel_topic(DriverNode& node, const tl_ros2_interface::msg::MoveCommand::SharedPtr msg)
{
  if (!node.session().is_connected())
  {
    RCLCPP_WARN(node.get_logger(), "[MoveL]: arm is not connected");
    return;
  }

  tl::MoveCmd cmd = convert::to_move_cmd(*msg, convert::MotionKind::kLinear);

  tl::Result ret = tl::robot_movel(node.session().fd(), cmd);

  RCLCPP_INFO(node.get_logger(), "[MoveL]: result=%s", arm_client::result_to_string(ret));
}

/**
 * @brief 透传模式必须先使用open_servoJ开启跟踪模式
 */
void handle_set_servoj_pos_topic(DriverNode& node, const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!node.session().is_connected())
  {
    RCLCPP_WARN(node.get_logger(), "[ServoJ]: arm is not connected");
    return;
  }

  // ROS 侧为 rad，SDK 要求度；SDK 固定 7 元素，6 轴在末尾补 0
  std::vector<double> pos = msg->data;
  if (pos.size() == 6)
  {
    pos.push_back(0.0);
  }
  if (pos.size() != 7)
  {
    RCLCPP_WARN(node.get_logger(), "[ServoJ]: data must have 6 or 7 elements, got %zu", pos.size());
    return;
  }
  convert::joint_rad_to_deg(pos, pos.size());
  tl::Result ret = tl::set_servoJ_pos(node.session().fd_aux(), pos);
  RCLCPP_INFO(node.get_logger(), "[ServoJ]: result=%s", arm_client::result_to_string(ret));
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
