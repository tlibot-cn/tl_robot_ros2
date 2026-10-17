/// @file topics.cpp
/// @brief 话题订阅：moveJ/moveL/servoJ/servoL 的 handler 与订阅注册。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "arm_client.hpp"
#include "arm_sdk.hpp"
#include "convert/msg_convert.hpp"
#include "convert/units.hpp"
#include "math/quaternion.hpp"
#include "node/driver_node.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "tl_ros2_interface/msg/move_command.hpp"
#include "tl_ros2_interface/msg/servol_move.hpp"

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

// ================ ServoL 笛卡尔空间直线伺服 ================

/**
 * @brief ServoL 笛卡尔空间直线伺服运动
 * 前置条件：需先调用 open_servoj 开启关节跟踪模式
 * 收到目标笛卡尔位姿后，自动获取当前位姿，插值并 IK 转为关节角后通过 servoj 发送
 */
void handle_set_servol_pos_topic(DriverNode& node, const tl_ros2_interface::msg::ServolMove::SharedPtr msg)
{
  if (!node.session().is_connected())
  {
    RCLCPP_WARN(node.get_logger(), "[ServoL]: arm is not connected");
    return;
  }

  // ========= 1. 获取当前位姿 =========
  int coord = msg->coord;
  if (coord < 1 || coord > 3)
  {
    coord = 1; // 默认基座标系
  }

  std::vector<double> current_pos;
  tl::Result ret = tl::get_current_position(node.session().fd(), static_cast<tl::Coord>(coord), current_pos);
  if (ret != tl::Result::SUCCESS || current_pos.size() < 6)
  {
    RCLCPP_ERROR(node.get_logger(), "[ServoL]: Failed to get current position");
    return;
  }

  // 当前位姿 [x, y, z, rx, ry, rz]
  double cx = current_pos[0], cy = current_pos[1], cz = current_pos[2];
  double crx = current_pos[3], cry = current_pos[4], crz = current_pos[5];

  // 目标位姿：ROS 侧位置 m、姿态 rad；SDK 需要位置 mm
  std::vector<double> target_pose = msg->target_pose;
  if (target_pose.size() < 6)
  {
    RCLCPP_ERROR(node.get_logger(), "[ServoL]: target_pose must have at least 6 elements");
    return;
  }
  for (std::size_t i = 0; i < 3 && i < target_pose.size(); ++i)
  {
    target_pose[i] = convert::m_to_mm(target_pose[i]);
  }
  double tx = target_pose[0], ty = target_pose[1], tz = target_pose[2];
  double trx = target_pose[3], try_ = target_pose[4], trz = target_pose[5];

  // ========= 2. 计算插值点数 =========
  double dx = tx - cx, dy = ty - cy, dz = tz - cz;
  double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  // ROS 侧步长 m，SDK 侧计算用 mm；≤0 时取默认 0.002 m = 2 mm
  double step_size = msg->step_size > 0.0 ? convert::m_to_mm(msg->step_size) : 2.0;

  int N = std::max(1, static_cast<int>(std::ceil(dist / step_size)));
  RCLCPP_INFO(node.get_logger(), "[ServoL] dist=%.1fmm, step=%.1f, points=%d", dist, step_size, N);

  // ========= 3. 准备 IK 参数 =========
  math::Quat cur_quat = math::rpy_to_quat({crx, cry, crz});
  math::Quat target_quat = math::rpy_to_quat({trx, try_, trz});

  std::vector<double> ref_pos(7, 0.0);

  // ========= 4. 插值 + IK + servoj 发送 (250Hz) =========
  const auto period = std::chrono::nanoseconds(4000000); // 250Hz = 4ms
  auto next_time = std::chrono::steady_clock::now();

  for (int i = 1; i <= N; ++i)
  {
    double t = static_cast<double>(i) / static_cast<double>(N);

    // 位置线性插值
    double ix = cx + t * dx;
    double iy = cy + t * dy;
    double iz = cz + t * dz;

    // 姿态四元数 Slerp
    math::Quat iq = math::quat_slerp(cur_quat, target_quat, t);
    math::Rpy irpy = math::quat_to_rpy(iq);

    // 构建插值位姿 (7 元素: x,y,z,rx,ry,rz,0)
    std::vector<double> interp_pos = {ix, iy, iz, irpy[0], irpy[1], irpy[2], 0.0};

    // IK: 笛卡尔(coord) -> 关节(0)
    std::vector<double> joint_pos;
    ret = tl::get_origin_coord_to_target_coord(node.session().fd(), static_cast<tl::Coord>(coord), interp_pos,
                                               tl::Coord::JOINT, joint_pos, 0, ref_pos);

    if (ret != tl::Result::SUCCESS)
    {
      RCLCPP_WARN(node.get_logger(), "[ServoL] IK failed at point %d/%d, ret=%d", i, N, static_cast<int>(ret));
      next_time += period;
      continue;
    }

    // 通过 servoj 发送关节角
    ret = tl::set_servoJ_pos(node.session().fd_aux(), joint_pos);
    if (ret != tl::Result::SUCCESS)
    {
      RCLCPP_WARN(node.get_logger(), "[ServoL] set_servoJ_pos failed at point %d/%d, ret=%d", i, N,
                  static_cast<int>(ret));
    }

    next_time += period;
    std::this_thread::sleep_until(next_time);
  }

  // ========= 5. 记录完成 =========
  RCLCPP_INFO(node.get_logger(), "[ServoL] completed to [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f], %d points", tx, ty, tz,
              trx, try_, trz, N);
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
  node.create_subscription<tl_ros2_interface::msg::ServolMove>(
      "/tl_driver/set_servol_pos", 10,
      [&node](const tl_ros2_interface::msg::ServolMove::SharedPtr msg)
      {
        handle_set_servol_pos_topic(node, msg);
      });
}
} // namespace tl_driver
