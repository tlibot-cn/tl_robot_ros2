/// @file services_motion.cpp
/// @brief 运动类服务与话题：速度/点动/轨迹/队列/ServoJ·ServoL 与 moveJ·moveL的实现（由 driver_node.cpp
/// 按域搬移，行为不变）。

#include "node/services_motion.h"

#include "convert/msg_convert.h"
#include "math/quaternion.h"
#include <chrono>
#include <string>
#include <vector>

#include "arm_sdk.hpp"
#include "node/driver_node.h"

namespace tl_driver::services
{
void handle_set_speed_service(DriverNode& node,
                              const std::shared_ptr<tl_ros2_interface::srv::SetSpeed::Request> request,
                              std::shared_ptr<tl_ros2_interface::srv::SetSpeed::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->speed < 0.0 || request->speed > 100.0)
  {
    response->success = false;
    response->message = "Speed out of range [0, 100]";
    return;
  }

  tl::Result ret = tl::set_speed(node.session().fd(), request->speed);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set speed successfully" : "Failed to set speed";
}

void handle_get_speed_service(DriverNode& node,
                              const std::shared_ptr<tl_ros2_interface::srv::GetSpeed::Request> request,
                              std::shared_ptr<tl_ros2_interface::srv::GetSpeed::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int speed;
  tl::Result ret = tl::get_speed(node.session().fd(), speed);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to get speed";
    return;
  }
  response->success = true;
  response->message = "Get speed successfully";
  response->speed = speed;
}

void handle_start_jogging_service(DriverNode& node,
                                  const std::shared_ptr<tl_ros2_interface::srv::Jogging::Request> request,
                                  std::shared_ptr<tl_ros2_interface::srv::Jogging::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->axis < 1 || request->axis > node.ndof())
  {
    response->success = false;
    response->message = "Invalid axis";
    return;
  }

  tl::Result ret = tl::robot_start_jogging(node.session().fd(), request->axis, request->direction);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Start jogging successfully" : "Failed to start jogging";
}

void handle_stop_jogging_service(DriverNode& node,
                                 const std::shared_ptr<tl_ros2_interface::srv::Jogging::Request> request,
                                 std::shared_ptr<tl_ros2_interface::srv::Jogging::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->axis < 1 || request->axis > node.ndof())
  {
    response->success = false;
    response->message = "Invalid axis";
    return;
  }

  tl::Result ret = tl::robot_stop_jogging(node.session().fd(), request->axis);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Stop jogging successfully" : "Failed to stop jogging";
}

void handle_track_save_service(DriverNode& node,
                               const std::shared_ptr<tl_ros2_interface::srv::TrackSave::Request> request,
                               std::shared_ptr<tl_ros2_interface::srv::TrackSave::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::track_record_save(node.session().fd(), request->traj_name);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Track save successfully" : "Failed to save track";
}

void handle_track_playback_service(DriverNode& node,
                                   const std::shared_ptr<tl_ros2_interface::srv::TrackPlayback::Request> request,
                                   std::shared_ptr<tl_ros2_interface::srv::TrackPlayback::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::track_record_playback(node.session().fd(), request->vel);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Track playback successfully" : "Failed to playback track";
}

void handle_open_servoj_service(DriverNode& node,
                                const std::shared_ptr<tl_ros2_interface::srv::OpenServoJ::Request> request,
                                std::shared_ptr<tl_ros2_interface::srv::OpenServoJ::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::vector<double> vmax = request->vmax;
  std::vector<double> amax = request->amax;
  std::vector<double> jmax = request->jmax;

  tl::Result ret = tl::open_servoJ(node.session().fd_aux(), vmax, amax, jmax);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "ServoJ open successfully" : "Failed to open ServoJ";
}

void handle_close_servoj_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::close_servoJ(node.session().fd_aux());
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "ServoJ close successfully" : "Failed to close ServoJ";
}

void handle_queue_motion_set_status_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::QueueMotionSetStatus::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::QueueMotionSetStatus::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::queue_motion_set_status(node.session().fd(), request->status);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set queue motion status successfully" : "Failed to set queue motion status";

  // 关闭连续运动模式后设置为示教模式
  if (!request->status)
  {
    if (tl::set_current_mode(node.session().fd(), tl::RobotMode::TEACH) == tl::Result::SUCCESS)
    {
      node.session().power_off();
    }
  }
}

void handle_queue_motion_movej_service(DriverNode& node,
                                       const std::shared_ptr<tl_ros2_interface::srv::QueueMotionMoveJ::Request> request,
                                       std::shared_ptr<tl_ros2_interface::srv::QueueMotionMoveJ::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::MoveCmd cmd = convert::to_move_cmd(request->cmd);

  tl::Result ret = tl::queue_motion_push_back_moveJ(node.session().fd(), cmd);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to push back queue motion movej";
    return;
  }

  // 新 SDK 签名: queue_motion_send_to_controller(socketFd, int size, bool isContinue = false)
  //   size=0 表示发送全部队列指令；isContinue=true 继续排队不运动，false 立即执行
  // 旧写法把 bool 传给了 size 参数，导致 is_continue=true 时只发送第 1 条，已修正
  ret = tl::queue_motion_send_to_controller(node.session().fd(), 0, request->is_continue);
  response->success = (ret == tl::Result::SUCCESS);
  response->message =
      response->success ? "Queue motion movej execute successfully" : "Failed to execute queue motion movej";
}

void handle_queue_motion_stop_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  bool status = false;
  tl::Result ret = tl::queue_motion_get_status(node.session().fd(), status);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to get queue motion status";
    return;
  }
  if (!status)
  {
    response->success = false;
    response->message = "Queue motion is not enabled";
    return;
  }

  ret = tl::queue_motion_stop_not_power_off(node.session().fd());
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Queue motion movej stop successfully" : "Failed to stop queue motion movej";
}

void handle_movej_topic(DriverNode& node, const tl_ros2_interface::msg::MoveCommand::SharedPtr msg)
{
  if (!node.session().is_connected())
  {
    RCLCPP_WARN(node.get_logger(), "[MoveJ]: arm is not connected");
    return;
  }

  tl::MoveCmd cmd = convert::to_move_cmd(*msg);

  tl::Result ret = tl::robot_movej(node.session().fd(), cmd);

  RCLCPP_INFO(node.get_logger(), "[MoveJ]: result=%s", result_to_string(ret));
}

void handle_movel_topic(DriverNode& node, const tl_ros2_interface::msg::MoveCommand::SharedPtr msg)
{
  if (!node.session().is_connected())
  {
    RCLCPP_WARN(node.get_logger(), "[MoveL]: arm is not connected");
    return;
  }

  tl::MoveCmd cmd = convert::to_move_cmd(*msg);

  tl::Result ret = tl::robot_movel(node.session().fd(), cmd);

  RCLCPP_INFO(node.get_logger(), "[MoveL]: result=%s", result_to_string(ret));
}

void handle_set_servoj_pos_topic(DriverNode& node, const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!node.session().is_connected())
  {
    RCLCPP_WARN(node.get_logger(), "[ServoJ]: arm is not connected");
    return;
  }

  std::vector<double> pos = msg->data;
  tl::Result ret = tl::set_servoJ_pos(node.session().fd_aux(), pos);
  RCLCPP_INFO(node.get_logger(), "[ServoJ]: result=%s", result_to_string(ret));
}

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

  // 目标位姿
  const auto& target_pose = msg->target_pose;
  if (target_pose.size() < 6)
  {
    RCLCPP_ERROR(node.get_logger(), "[ServoL]: target_pose must have at least 6 elements");
    return;
  }
  double tx = target_pose[0], ty = target_pose[1], tz = target_pose[2];
  double trx = target_pose[3], try_ = target_pose[4], trz = target_pose[5];

  // ========= 2. 计算插值点数 =========
  double dx = tx - cx, dy = ty - cy, dz = tz - cz;
  double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  double step_size = msg->step_size;
  if (step_size <= 0.0)
  {
    step_size = 2.0; // 默认步长 2mm
  }

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

void register_motion_services(DriverNode& node, rclcpp::CallbackGroup::SharedPtr group)
{
  node.register_service<tl_ros2_interface::srv::SetSpeed>("/tl_driver/set_speed", handle_set_speed_service, group);
  node.register_service<tl_ros2_interface::srv::GetSpeed>("/tl_driver/get_speed", handle_get_speed_service, group);
  node.register_service<tl_ros2_interface::srv::Jogging>("/tl_driver/start_jogging", handle_start_jogging_service,
                                                         group);
  node.register_service<tl_ros2_interface::srv::Jogging>("/tl_driver/stop_jogging", handle_stop_jogging_service, group);
  node.register_service<tl_ros2_interface::srv::TrackSave>("/tl_driver/track_save", handle_track_save_service, group);
  node.register_service<tl_ros2_interface::srv::TrackPlayback>("/tl_driver/track_playback",
                                                               handle_track_playback_service, group);
  node.register_service<tl_ros2_interface::srv::OpenServoJ>("/tl_driver/open_servoj", handle_open_servoj_service,
                                                            group);
  node.register_service<std_srvs::srv::Trigger>("/tl_driver/close_servoj", handle_close_servoj_service, group);
  node.register_service<tl_ros2_interface::srv::QueueMotionSetStatus>("/tl_driver/queue_motion_set_status",
                                                                      handle_queue_motion_set_status_service, group);
  node.register_service<tl_ros2_interface::srv::QueueMotionMoveJ>("/tl_driver/queue_motion_movej",
                                                                  handle_queue_motion_movej_service, group);
  node.register_service<std_srvs::srv::Trigger>("/tl_driver/queue_motion_stop", handle_queue_motion_stop_service,
                                                group);
}
} // namespace tl_driver::services
