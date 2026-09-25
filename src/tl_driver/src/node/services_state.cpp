#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "arm_client.hpp"
#include "arm_sdk.hpp"
#include "node/registrars.hpp"
#include "node/driver_node.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "tl_ros2_interface/srv/get_current_line_joint_speed.hpp"
#include "tl_ros2_interface/srv/get_current_motor_torque.hpp"
#include "tl_ros2_interface/srv/get_joint_software_version.hpp"
#include "tl_ros2_interface/srv/get_joint_temperature.hpp"
#include "tl_ros2_interface/srv/get_joint_voltage.hpp"
#include "tl_ros2_interface/srv/get_motor_current.hpp"
#include "tl_ros2_interface/srv/get_robot_joint_param.hpp"
#include "tl_ros2_interface/srv/get_robot_state.hpp"
#include "tl_ros2_interface/srv/set_drag_mode.hpp"
#include "tl_ros2_interface/srv/set_robot_joint_param.hpp"

namespace tl_driver
{
namespace
{

void handle_get_robot_state_service(DriverNode& node,
                                    const std::shared_ptr<tl_ros2_interface::srv::GetRobotState::Request> request,
                                    std::shared_ptr<tl_ros2_interface::srv::GetRobotState::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::RobotState param{};
  param.channel = request->channel;
  param.stop = request->stop;
  param.mode = request->mode;
  param.interval = request->interval;
  param.ioState = request->io_state;
  param.position = request->position;
  param.dataildmotionpos = request->detail_motion_pos;
  param.posSum = request->pos_sum;
  param.ioPort = request->io_port;
  param.optional = request->optional;

  const uint64_t start_seq = arm_client::robot_state_seq();

  tl::Result ret = tl::get_robot_state(node.session().fd(), param);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to get robot state";
    return;
  }

  std::string state_msg;
  if (!arm_client::wait_robot_state(start_seq, std::chrono::seconds(5), state_msg))
  {
    response->success = false;
    response->message = "Timeout waiting for robot state";
    return;
  }

  response->success = true;
  response->message = state_msg;
}

void handle_get_library_version_service(DriverNode& node,
                                        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::string version = tl::get_library_version();
  response->success = (version != "");
  response->message = response->success ? version : "Failed to get library version";
}

void handle_get_nexmotion_lib_version_service(DriverNode& node,
                                              const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                              std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::string version;
  tl::Result ret = tl::get_nexmotion_lib_version(node.session().fd(), version);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? version : "Failed to get nexmotion lib version";
}

void handle_get_joint_software_version_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::GetJointSoftwareVersion::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetJointSoftwareVersion::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->axis_num < 1 || request->axis_num > node.ndof())
  {
    response->success = false;
    response->message = "Invalid axis num";
    return;
  }

  std::string version;
  tl::Result ret = tl::query_joint_software_version(node.session().fd(), request->axis_num, version);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? version : "Failed to get joint software version";
}

void handle_get_robot_joint_param_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::GetRobotJointParam::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetRobotJointParam::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->id < 1 || request->id > node.ndof())
  {
    response->success = false;
    response->message = "Invalid id";
    return;
  }

  tl::RobotJointParam param{};
  tl::Result ret = tl::get_robot_joint_param(node.session().fd(), request->id, param);
  if (ret == tl::Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get robot joint param successfully";
    response->param.reduction_ratio = param.reducRatio;
    response->param.encoder_resolution = param.encoderResolution;
    response->param.pos_sw_limit = param.posSWLimit;
    response->param.neg_sw_limit = param.negSWLimit;
    response->param.rated_rot_speed = param.ratedRotSpeed;
    response->param.rated_derot_speed = param.ratedDeRotSpeed;
    response->param.max_rot_speed = param.maxRotSpeed;
    response->param.max_derot_speed = param.maxDeRotSpeed;
    response->param.rated_vel = param.ratedVel;
    response->param.rated_devel = param.deRatedVel;
    response->param.max_acc = param.maxAcc;
    response->param.max_dec = param.maxDecel;
    response->param.direction = param.direction;
  }
  else
  {
    response->success = true;
    response->message = "Falied to get robot joint param";
  }
}

void handle_set_robot_joint_param_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::SetRobotJointParam::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetRobotJointParam::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->id < 1 || request->id > node.ndof())
  {
    response->success = false;
    response->message = "Invalid id";
    return;
  }

  tl::RobotJointParam param{};
  param.reducRatio = request->param.reduction_ratio;
  param.encoderResolution = request->param.encoder_resolution;
  param.posSWLimit = request->param.pos_sw_limit;
  param.negSWLimit = request->param.neg_sw_limit;
  param.ratedRotSpeed = request->param.rated_rot_speed;
  param.ratedDeRotSpeed = request->param.rated_derot_speed;
  param.maxRotSpeed = request->param.max_rot_speed;
  param.maxDeRotSpeed = request->param.max_derot_speed;
  param.ratedVel = request->param.rated_vel;
  param.deRatedVel = request->param.rated_devel;
  param.maxAcc = request->param.max_acc;
  param.maxDecel = request->param.max_dec;
  param.direction = request->param.direction;

  tl::Result ret = tl::set_robot_joint_param(node.session().fd(), request->id, param);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set robot joint param successfully" : "Falied to set robot joint param";

  // 设置参数后机械臂会下电
  node.session().power_off();
  node.session().power_on();
}

void handle_get_joint_temperature_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::GetJointTemperature::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetJointTemperature::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::vector<double> temperatures;
  tl::Result ret = tl::get_joint_temperature(node.session().fd(), temperatures);
  if (ret == tl::Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get joint temperatures successfully";
    response->temperatures = temperatures;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get joint temperatures";
  }
}

void handle_get_joint_voltage_service(DriverNode& node,
                                      const std::shared_ptr<tl_ros2_interface::srv::GetJointVoltage::Request> request,
                                      std::shared_ptr<tl_ros2_interface::srv::GetJointVoltage::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::vector<double> joint_voltage;
  std::vector<double> positioner_voltage;
  tl::Result ret = tl::get_joint_voltage(node.session().fd(), joint_voltage, positioner_voltage);
  if (ret == tl::Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get joint voltage successfully";
    response->joint_voltage = joint_voltage;
    response->positioner_voltage = positioner_voltage;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get joint voltage";
  }
}

void handle_get_motor_current_service(DriverNode& node,
                                      const std::shared_ptr<tl_ros2_interface::srv::GetMotorCurrent::Request> request,
                                      std::shared_ptr<tl_ros2_interface::srv::GetMotorCurrent::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::vector<double> motor_current, motor_current_sync;
  tl::Result ret = tl::get_current_motor_current(node.session().fd(), motor_current, motor_current_sync);
  if (ret == tl::Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get motor current successfully";
    response->motor_current = motor_current;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get motor current";
  }
}

void handle_get_current_motor_torque_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::GetCurrentMotorTorque::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetCurrentMotorTorque::Response> response)
{
  (void)request;

  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::vector<int> motor_torque{};
  std::vector<int> motor_torque_sync{};
  tl::Result ret = tl::get_current_motor_torque(node.session().fd(), motor_torque, motor_torque_sync);
  if (ret == tl::Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get current motor torque successfully";
    response->motor_torque = motor_torque;
    response->motor_torque_sync = motor_torque_sync;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get current motor torque";
  }
}

void handle_get_current_line_joint_speed_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::GetCurrentLineJointSpeed::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetCurrentLineJointSpeed::Response> response)
{
  (void)request;

  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  double line_speed{0.0};
  std::vector<double> joint_speed{};
  std::vector<double> joint_speed_sync{};
  tl::Result ret =
      tl::get_current_line_speed_and_joint_speed(node.session().fd(), line_speed, joint_speed, joint_speed_sync);
  if (ret == tl::Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get current line and joint speed successfully";
    response->line_speed = line_speed;
    response->joint_speed = joint_speed;
    response->joint_speed_sync = joint_speed_sync;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get current line and joint speed";
  }
}

void handle_set_drag_mode_service(DriverNode& node,
                                  const std::shared_ptr<tl_ros2_interface::srv::SetDragMode::Request> request,
                                  std::shared_ptr<tl_ros2_interface::srv::SetDragMode::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->mode < 0 || request->mode > 3)
  {
    response->success = false;
    response->message = "Invalid mode";
    return;
  }

  tl::Result ret = tl::set_darg_mode(node.session().fd(), request->mode);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set drag mode successfully" : "Failed to set drag mode";
}

void handle_get_drag_status_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  bool endFlag = false;
  tl::Result ret = tl::get_drag_thread_is_end(node.session().fd(), endFlag);
  if (ret == tl::Result::SUCCESS)
  {
    response->success = endFlag;
    response->message = response->success ? "Drag ended" : "Drag not ended";
  }
  else
  {
    response->success = true;
    response->message = "Failed to get drag status";
  }
}

} // namespace

void register_state_services(DriverNode& node)
{
  node.create_service<tl_ros2_interface::srv::GetRobotState>(
      "/tl_driver/get_robot_state",
      std::bind(&handle_get_robot_state_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<std_srvs::srv::Trigger>(
      "/tl_driver/get_library_version",
      std::bind(&handle_get_library_version_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<std_srvs::srv::Trigger>("/tl_driver/get_nexmotion_lib_version",
                                              std::bind(&handle_get_nexmotion_lib_version_service, std::ref(node),
                                                        std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetJointSoftwareVersion>(
      "/tl_driver/get_joint_software_version", std::bind(&handle_get_joint_software_version_service, std::ref(node),
                                                         std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetRobotJointParam>(
      "/tl_driver/get_robot_joint_param",
      std::bind(&handle_get_robot_joint_param_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetRobotJointParam>(
      "/tl_driver/set_robot_joint_param",
      std::bind(&handle_set_robot_joint_param_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetJointTemperature>(
      "/tl_driver/get_joint_temperature",
      std::bind(&handle_get_joint_temperature_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetJointVoltage>(
      "/tl_driver/get_joint_voltage",
      std::bind(&handle_get_joint_voltage_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetMotorCurrent>(
      "/tl_driver/get_motor_current",
      std::bind(&handle_get_motor_current_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetCurrentMotorTorque>(
      "/tl_driver/get_current_motor_torque", std::bind(&handle_get_current_motor_torque_service, std::ref(node),
                                                       std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetCurrentLineJointSpeed>(
      "/tl_driver/get_current_line_joint_speed", std::bind(&handle_get_current_line_joint_speed_service, std::ref(node),
                                                           std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetDragMode>(
      "/tl_driver/set_drag_mode",
      std::bind(&handle_set_drag_mode_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  // 用不了
  node.create_service<std_srvs::srv::Trigger>(
      "/tl_driver/get_drag_status",
      std::bind(&handle_get_drag_status_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
}

} // namespace tl_driver
