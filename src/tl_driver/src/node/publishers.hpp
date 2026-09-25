#ifndef TL_DRIVER__NODE__PUBLISHERS_HPP_
#define TL_DRIVER__NODE__PUBLISHERS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <string>
#include <vector>

#include "arm_session.hpp"
#include "tl_ros2_interface/msg/arm_status.hpp"
#include "tl_ros2_interface/msg/cartesian_pose.hpp"

namespace tl_driver
{

/**
 * @brief 状态发布：`/joint_states`、`/tcp_pose`、`/arm_status`。
 * @note 仅在会话可用时查询控制器；查询失败静默跳过（与拆分前一致，掉线降级见架构评审 3.4）。
 */
class StatePublishers
{
public:
  /**
   * @brief 创建三个状态发布器。
   * @param[in] node 承载发布器的节点
   * @param[in] session 会话（提供 fd 与连接状态）
   * @param[in] joint_names 关节名，用于 `/joint_states`
   * @param[in] ndof 关节数（6 或 7），决定位姿向量裁剪方式
   */
  StatePublishers(rclcpp::Node& node, ArmSession& session, const std::vector<std::string>& joint_names,
                  const int& ndof);

  /**
   * @brief 定时回调：查询并发布运行状态、关节角、TCP 位姿。
   */
  void publish_arm_state();

private:
  /**
   * @brief 发布 `/joint_states`。
   * @param[in] joint_pose 关节角（rad）
   */
  void publish_joint_pose(const std::vector<double>& joint_pose);

  /**
   * @brief 发布 `/tcp_pose`。
   * @param[in] tcp_pose TCP 位姿，位置 mm、姿态 rad
   */
  void publish_tcp_pose(const std::vector<double>& tcp_pose);

  /**
   * @brief 发布 `/arm_status`。
   */
  void publish_running_status();

  rclcpp::Node& node_;
  ArmSession& session_;
  const std::vector<std::string>& joint_names_;
  const int& ndof_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::CartesianPose>::SharedPtr tcp_pose_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::ArmStatus>::SharedPtr running_status_pub_;
};

} // namespace tl_driver

#endif // TL_DRIVER__NODE__PUBLISHERS_HPP_
