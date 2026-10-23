/// @file services_motion.cpp
/// @brief 运动类服务：速度/点动/轨迹/队列/ServoJ 开关与队列运动实现（由 driver_node.cpp
/// 按域搬移，行为不变）。

#include "convert/msg_convert.hpp"
#include "convert/units.hpp"
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "arm_sdk.hpp"
#include "node/registrars.hpp"
#include "node/driver_node.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "tl_ros2_interface/srv/jogging.hpp"
#include "tl_ros2_interface/srv/open_servo_j.hpp"
#include "tl_ros2_interface/srv/queue_motion_move_j.hpp"
#include "tl_ros2_interface/srv/queue_motion_set_status.hpp"
#include "tl_ros2_interface/srv/set_speed.hpp"
#include "tl_ros2_interface/srv/track_save.hpp"

namespace tl_driver
{
namespace
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
  if (node.motion_action_active())
  {
    response->success = false;
    response->message = "A motion action is active";
    return;
  }

  // ROS 侧 rad/s、rad/s²、rad/s³；SDK 要求度制且固定 7 元素
  std::vector<double> vmax = request->vmax;
  std::vector<double> amax = request->amax;
  std::vector<double> jmax = request->jmax;
  for (auto *values : {&vmax, &amax, &jmax})
  {
    if (values->size() == 6)
    {
      values->push_back(0.0);
    }
    if (values->size() != 7)
    {
      response->success = false;
      response->message = "vmax/amax/jmax must have 6 or 7 elements";
      return;
    }
    convert::joint_rad_to_deg(*values, values->size());
  }

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

  const int fd = node.session().fd();
  tl::Result ret = tl::queue_motion_set_status(fd, request->status);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set queue motion status successfully" : "Failed to set queue motion status";

  // 关闭连续运动模式后设置为示教模式
  if (!request->status)
  {
    const bool queue_status_ok = response->success;
    tl::Result mode_ret = tl::set_current_mode(fd, tl::RobotMode::TEACH);
    if (mode_ret != tl::Result::SUCCESS)
    {
      response->success = false;
      response->message = queue_status_ok ? "Queue motion status set but failed to switch to teach mode"
                                          : "Failed to set queue motion status and switch to teach mode";
    }
    else if (!node.session().power_off())
    {
      response->success = false;
      response->message = queue_status_ok ? "Queue motion status set but failed to power off"
                                          : "Failed to set queue motion status and power off";
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
  if (node.motion_action_active())
  {
    response->success = false;
    response->message = "A motion action is active";
    return;
  }

  tl::MoveCmd cmd = convert::to_move_cmd(request->cmd, convert::MotionKind::kJoint);

  const int fd = node.session().fd();
  tl::Result ret = tl::queue_motion_push_back_moveJ(fd, cmd);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to push back queue motion movej";
    return;
  }

  // 新 SDK 签名: queue_motion_send_to_controller(socketFd, int size, bool isContinue = false)
  //   size=0 表示发送全部队列指令；isContinue=true 继续排队不运动，false 立即执行
  // 旧写法把 bool 传给了 size 参数，导致 is_continue=true 时只发送第 1 条，已修正
  ret = tl::queue_motion_send_to_controller(fd, 0, request->is_continue);
  node.note_external_motion();
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
  const int fd = node.session().fd();
  tl::Result ret = tl::queue_motion_get_status(fd, status);
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

  ret = tl::queue_motion_stop_not_power_off(fd);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Queue motion movej stop successfully" : "Failed to stop queue motion movej";
}

} // namespace

void register_motion_services(DriverNode& node)
{
  node.create_service<tl_ros2_interface::srv::SetSpeed>(
      "/tl_driver/set_speed",
      std::bind(&handle_set_speed_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::Jogging>(
      "/tl_driver/start_jogging",
      std::bind(&handle_start_jogging_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::Jogging>(
      "/tl_driver/stop_jogging",
      std::bind(&handle_stop_jogging_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::TrackSave>(
      "/tl_driver/track_save",
      std::bind(&handle_track_save_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::OpenServoJ>(
      "/tl_driver/open_servoj",
      std::bind(&handle_open_servoj_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<std_srvs::srv::Trigger>(
      "/tl_driver/close_servoj",
      std::bind(&handle_close_servoj_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::QueueMotionSetStatus>(
      "/tl_driver/queue_motion_set_status",
      std::bind(&handle_queue_motion_set_status_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::QueueMotionMoveJ>(
      "/tl_driver/queue_motion_movej",
      std::bind(&handle_queue_motion_movej_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<std_srvs::srv::Trigger>(
      "/tl_driver/queue_motion_stop",
      std::bind(&handle_queue_motion_stop_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
}
} // namespace tl_driver
