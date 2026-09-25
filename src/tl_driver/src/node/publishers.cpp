/// @file publishers.cpp
/// @brief Publishers 的实现（与拆分前 TL_Arm 内同名实现等价）。

#include "node/publishers.h"

#include <algorithm>
#include <utility>

#include "arm_sdk.hpp"

namespace tl_driver
{
Publishers::Publishers(rclcpp::Node& node, const ArmSession& session, std::vector<std::string> joint_names, int ndof)
    : node_(node), session_(session), joint_names_(std::move(joint_names)), ndof_(ndof)
{
  joint_state_pub_ = node_.create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  tcp_pose_pub_ = node_.create_publisher<tl_ros2_interface::msg::CartesianPose>("/tcp_pose", 10);

  running_status_pub_ = node_.create_publisher<tl_ros2_interface::msg::ArmStatus>("/arm_status", 10);
}

void Publishers::publish_arm_state()
{
  if (!session_.is_connected())
  {
    RCLCPP_WARN(node_.get_logger(), "[Pub]: arm is not connected");
    return;
  }

  publish_running_status();

  std::vector<double> joint_pose;
  std::vector<double> tcp_pose;

  joint_pose.clear();
  tcp_pose.clear();

  if (tl::get_current_position(session_.fd(), tl::Coord::JOINT, joint_pose) == tl::Result::SUCCESS)
  {
    // 将角度转换为弧度(批量转换)
    const double deg_to_rad = M_PI / 180.0;
    std::transform(joint_pose.begin(), joint_pose.end(), joint_pose.begin(),
                   [deg_to_rad](double deg)
                   {
                     return deg * deg_to_rad;
                   });

    publish_joint_pose(joint_pose);
  }

  if (tl::get_current_position(session_.fd(), tl::Coord::BASE, tcp_pose) == tl::Result::SUCCESS)
  {
    publish_tcp_pose(tcp_pose);
  }
}

void Publishers::publish_joint_pose(const std::vector<double>& joint_pose)
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_.now();
  msg.name = joint_names_;
  switch (ndof_)
  {
    case 6:
      msg.position.assign(joint_pose.begin(), joint_pose.end() - 1);
      break;
    case 7:
      msg.position.assign(joint_pose.begin(), joint_pose.end());
      break;
  }

  joint_state_pub_->publish(msg);
}

void Publishers::publish_tcp_pose(const std::vector<double>& tcp_pose)
{
  // SDK 返回：位置 mm、欧拉角弧度（rad）→ 统一转换为 ROS 标准单位：m 和 rad
  constexpr double kMmToM = 1.0 / 1000.0;

  tl_ros2_interface::msg::CartesianPose msg;
  msg.header.stamp = node_.now();
  msg.header.frame_id = "base_link";

  msg.position.x = tcp_pose[0] * kMmToM;
  msg.position.y = tcp_pose[1] * kMmToM;
  msg.position.z = tcp_pose[2] * kMmToM;

  msg.rpy.x = tcp_pose[3];
  msg.rpy.y = tcp_pose[4];
  msg.rpy.z = tcp_pose[5];

  switch (ndof_)
  {
    case 6:
      msg.arm_angle = 0;
      break;
    case 7:
      msg.arm_angle = tcp_pose[6];
      break;
  }

  tcp_pose_pub_->publish(msg);
}

void Publishers::publish_running_status()
{
  tl::RunState running_status = tl::RunState::STOP;
  tl::Result ret = tl::get_robot_running_state(session_.fd(), running_status);
  if (ret != tl::Result::SUCCESS)
  {
    RCLCPP_INFO(node_.get_logger(), "[Read Running Status]: failed to read running status, result=%s",
                tl_driver::result_to_string(ret));
    return;
  }

  tl_ros2_interface::msg::ArmStatus msg;

  msg.stamp = node_.now();
  switch (running_status)
  {
    case tl::RunState::STOP:
      msg.run_state = "STOP";
      break;
    case tl::RunState::PAUSE:
      msg.run_state = "PAUSE";
      break;
    case tl::RunState::RUNNING:
      msg.run_state = "RUNNING";
      break;
  }

  running_status_pub_->publish(msg);
}
} // namespace tl_driver
