/// @file publishers.cpp
/// @brief StatePublishers 的实现（与拆分前的同名成员实现等价）。

#include "node/publishers.hpp"

#include <algorithm>
#include <utility>

#include "arm_client.hpp"
#include "arm_sdk.hpp"
#include "convert/units.hpp"

namespace tl_driver
{
StatePublishers::StatePublishers(rclcpp::Node& node, const ArmSession& session, std::vector<std::string> joint_names,
                                 int ndof)
    : node_(node), session_(session), joint_names_(std::move(joint_names)), ndof_(ndof)
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

  if (tl::get_current_position(session_.fd(), tl::Coord::JOINT, joint_pose) == tl::Result::SUCCESS)
  {
    // 将角度转换为弧度(批量转换)
    convert::joint_deg_to_rad(joint_pose, joint_pose.size());

    publish_joint_pose(joint_pose);
  }

  if (tl::get_current_position(session_.fd(), tl::Coord::BASE, tcp_pose) == tl::Result::SUCCESS)
  {
    publish_tcp_pose(tcp_pose);
  }
}

void StatePublishers::publish_joint_pose(const std::vector<double>& joint_pose)
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_.now();

  // 按实际数据与关节名数量裁剪：6 轴时 SDK 末位为补 0，丢掉；数据不足时不越界。
  const std::size_t count = std::min(joint_pose.size(), joint_names_.size());
  if (count == 0)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "[Pub]: empty joint pose or joint names, skip /joint_states");
    return;
  }
  if (joint_pose.size() < joint_names_.size())
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "[Pub]: joint pose has %zu values, fewer than %zu joint names", joint_pose.size(),
                         joint_names_.size());
  }
  // name 必须与 position 等长，否则 JointState 契约不成立。
  msg.name.assign(joint_names_.begin(), joint_names_.begin() + count);
  msg.position.assign(joint_pose.begin(), joint_pose.begin() + count);

  joint_state_pub_->publish(msg);
}

void StatePublishers::publish_tcp_pose(const std::vector<double>& tcp_pose)
{
  if (tcp_pose.size() < 6)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "[Pub]: tcp pose has %zu values, need at least 6, skip /tcp_pose", tcp_pose.size());
    return;
  }

  // SDK 返回：位置 mm、欧拉角弧度（rad）→ 统一转换为 ROS 标准单位：m 和 rad
  tl_ros2_interface::msg::CartesianPose msg;
  msg.header.stamp = node_.now();
  msg.header.frame_id = "base_link";

  msg.position.x = convert::mm_to_m(tcp_pose[0]);
  msg.position.y = convert::mm_to_m(tcp_pose[1]);
  msg.position.z = convert::mm_to_m(tcp_pose[2]);

  msg.rpy.x = tcp_pose[3];
  msg.rpy.y = tcp_pose[4];
  msg.rpy.z = tcp_pose[5];

  switch (ndof_)
  {
    case 6:
      msg.arm_angle = 0;
      break;
    case 7:
      msg.arm_angle = tcp_pose.size() > 6 ? tcp_pose[6] : 0.0;
      break;
    default:
      msg.arm_angle = tcp_pose.size() > 6 ? tcp_pose[6] : 0.0;
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                           "[Pub]: unsupported ndof=%d, arm_angle fallback to %s", ndof_,
                           tcp_pose.size() > 6 ? "seventh value" : "0");
      break;
  }

  tcp_pose_pub_->publish(msg);
}

void StatePublishers::publish_running_status()
{
  tl::RunState running_status = tl::RunState::STOP;
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
    case tl::RunState::STOP:
      msg.run_state = "STOP";
      break;
    case tl::RunState::PAUSE:
      msg.run_state = "PAUSE";
      break;
    case tl::RunState::RUNNING:
      msg.run_state = "RUNNING";
      break;
    default:
      msg.run_state = "UNKNOWN";
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[Pub]: unknown run state %d",
                           static_cast<int>(running_status));
      break;
  }

  running_status_pub_->publish(msg);
}
} // namespace tl_driver
