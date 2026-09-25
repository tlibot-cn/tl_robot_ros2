#include <algorithm>

#include "tl_driver/arm_client.hpp"
#include "tl_driver/arm_sdk.hpp"
#include "tl_driver/node/publishers.hpp"

/**
 * @brief 状态发布实现（joint_states / tcp_pose / arm_status）。
 */

namespace tl_driver
{

StatePublishers::StatePublishers(rclcpp::Node& node, ArmSession& session, const std::vector<std::string>& joint_names,
                                 const int& ndof)
    : node_(node), session_(session), joint_names_(joint_names), ndof_(ndof)
{
  joint_state_pub_ = node_.create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  tcp_pose_pub_ = node_.create_publisher<tl_ros2_interface::msg::CartesianPose>("/tcp_pose", 10);

  running_status_pub_ = node_.create_publisher<tl_ros2_interface::msg::ArmStatus>("/arm_status", 10);
}

void StatePublishers::publish_arm_state()
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

  if (tl::get_current_position(session_.fd(), 0, joint_pose) == tl::Result::SUCCESS)
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

  if (tl::get_current_position(session_.fd(), 1, tcp_pose) == tl::Result::SUCCESS)
  {
    publish_tcp_pose(tcp_pose);
  }
}

void StatePublishers::publish_joint_pose(const std::vector<double>& joint_pose)
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

void StatePublishers::publish_tcp_pose(const std::vector<double>& tcp_pose)
{
  tl_ros2_interface::msg::CartesianPose msg;
  msg.header.stamp = node_.now();
  msg.header.frame_id = "base_link";

  msg.position.x = tcp_pose[0];
  msg.position.y = tcp_pose[1];
  msg.position.z = tcp_pose[2];

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

void StatePublishers::publish_running_status()
{
  int running_status = -1;
  tl::Result ret = tl::get_robot_running_state(session_.fd(), running_status);
  if (ret != tl::Result::SUCCESS)
  {
    RCLCPP_INFO(node_.get_logger(), "[Read Running Status]: failed to read running status, result=%s",
                arm_client::result_to_string(ret));
    return;
  }

  tl_ros2_interface::msg::ArmStatus msg;

  msg.stamp = node_.now();
  switch (running_status)
  {
    case 0:
      msg.run_state = "STOP";
      break;
    case 1:
      msg.run_state = "PAUSE";
      break;
    case 2:
      msg.run_state = "RUNNING";
      break;
  }

  running_status_pub_->publish(msg);
}

} // namespace tl_driver
