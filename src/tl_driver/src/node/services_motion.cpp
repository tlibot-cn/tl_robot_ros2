#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "arm_sdk.hpp"
#include "convert/msg_convert.hpp"
#include "node/registrars.hpp"
#include "node/driver_node.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "tl_ros2_interface/srv/get_speed.hpp"
#include "tl_ros2_interface/srv/jogging.hpp"
#include "tl_ros2_interface/srv/open_servo_j.hpp"
#include "tl_ros2_interface/srv/queue_motion_move_j.hpp"
#include "tl_ros2_interface/srv/queue_motion_set_status.hpp"
#include "tl_ros2_interface/srv/set_speed.hpp"

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
    if (tl::set_current_mode(node.session().fd(), 0) == tl::Result::SUCCESS)
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

  ret = tl::queue_motion_send_to_controller(node.session().fd(), request->is_continue);
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

} // namespace

void register_motion_services(DriverNode& node)
{
  node.create_service<tl_ros2_interface::srv::SetSpeed>(
      "/tl_driver/set_speed",
      std::bind(&handle_set_speed_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetSpeed>(
      "/tl_driver/get_speed",
      std::bind(&handle_get_speed_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::Jogging>(
      "/tl_driver/start_jogging",
      std::bind(&handle_start_jogging_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::Jogging>(
      "/tl_driver/stop_jogging",
      std::bind(&handle_stop_jogging_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
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
