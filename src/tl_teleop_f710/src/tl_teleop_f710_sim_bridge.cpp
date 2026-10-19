/**
 * @file tl_teleop_f710_sim_bridge.cpp
 * @brief Gazebo 仿真桥接节点 — 转发关节角到 position controller
 *
 * 仿真链路：
 *   tl_teleop_f710_node (simulation_mode=true, KDL 本地 IK)
 *     -> /tl_driver/set_servoj_pos (Float64MultiArray, 关节角 rad)
 *     -> 本节点
 *     -> Gazebo position controller
 *
 * 本节点不依赖 tl_driver，只做话题转发。
 */

#include "tl_teleop_f710/tl_teleop_f710_sim_bridge.h"

ServojSimBridge::ServojSimBridge() : rclcpp::Node("tl_teleop_f710_sim_bridge")
{
  this->declare_parameter("position_controller_topic", "/tcb_group_position_controller/commands");
  position_controller_topic_ = this->get_parameter("position_controller_topic").as_string();

  servoj_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/tl_driver/set_servoj_pos", 10, std::bind(&ServojSimBridge::servojCallback, this, std::placeholders::_1));

  pos_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(position_controller_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "仿真桥接已启动: /tl_driver/set_servoj_pos -> %s",
              position_controller_topic_.c_str());
}

void ServojSimBridge::servojCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.empty())
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "收到空关节指令，已忽略");
    return;
  }
  pos_pub_->publish(*msg);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServojSimBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
