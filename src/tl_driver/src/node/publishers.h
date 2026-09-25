#ifndef TL_DRIVER__PUBLISHERS_H_
#define TL_DRIVER__PUBLISHERS_H_

/// @file publishers.h
/// @brief 状态发布：/joint_states、/tcp_pose、/arm_status。
/// @note 拆分阶段为纯搬移，字段裁剪与日志行为与拆分前一致
///       （switch 未覆盖、尺寸裁剪、日志节流见架构评审 P1-4 / P1-6，另行修复）。

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "arm_session.h"
#include "tl_ros2_interface/msg/arm_status.hpp"
#include "tl_ros2_interface/msg/cartesian_pose.hpp"

namespace tl_driver
{
/// @brief 三个状态话题的发布器与填充逻辑。
class StatePublishers
{
public:
  /// @brief 构造并创建三个发布器。
  /// @param[in] node 节点，提供时钟与日志器，生命周期须长于本对象
  /// @param[in] session 会话，提供连接状态与 fd，生命周期须长于本对象
  /// @param[in] joint_names 关节名，写入 /joint_states.name
  /// @param[in] ndof 关节数，决定位姿字段裁剪
  StatePublishers(rclcpp::Node& node, const ArmSession& session, std::vector<std::string> joint_names, int ndof);

  /// @brief 定时器入口：未连接时告警返回，否则发布运行状态、关节角与 TCP 位姿。
  void publish_arm_state();

  /// @brief 发布 /joint_states。
  /// @param[in] joint_pose 关节角 (rad)
  void publish_joint_pose(const std::vector<double>& joint_pose);

  /// @brief 发布 /tcp_pose，位置由 SDK 的 mm 换算为 m。
  /// @param[in] tcp_pose SDK 位姿 (mm, rad)
  void publish_tcp_pose(const std::vector<double>& tcp_pose);

  /// @brief 发布 /arm_status。
  void publish_running_status();

private:
  rclcpp::Node& node_;
  const ArmSession& session_;
  std::vector<std::string> joint_names_;
  int ndof_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::CartesianPose>::SharedPtr tcp_pose_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::ArmStatus>::SharedPtr running_status_pub_;
};
} // namespace tl_driver

#endif // TL_DRIVER__PUBLISHERS_H_
