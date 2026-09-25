/// @file services_state.cpp
/// @brief 状态类服务：连接/上下电/清错/版本/温度/电压/电流/力矩/模式的实现（由 driver_node.cpp 按域搬移，行为不变）。

#include "node/services_state.h"

#include <chrono>
#include <string>
#include <vector>

#include "arm_client.hpp"
#include "arm_sdk.hpp"
#include "node/driver_node.h"

namespace tl_driver::services
{
void handle_connect_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (node.session().is_connected())
  {
    response->success = true; // shc 26.5.19 修改：连接了就响应true
    response->message = "Arm already connected";
    return;
  }

  response->success = node.connect();
  response->message = response->success ? "Arm connected successfully" : "Failed to connect arm";
}

void handle_disconnect_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  response->success = node.session().disconnect();
  response->message = response->success ? "Arm disconnected successfully" : "Failed to disconnect arm";
}

void handle_poweron_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  response->success = node.session().power_on();
  response->message = response->success ? "Arm power on successfully" : "Failed to power on arm";
}

void handle_poweroff_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (!node.session().is_powered())
  {
    response->success = true;
    response->message = "Arm already power off";
    return;
  }

  response->success = node.session().power_off();
  response->message = response->success ? "Arm power off successfully" : "Failed to power off arm";
}

void handle_clear_error_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::clear_error(node.session().fd());
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Clear error successfully"
                                        : std::string("Clear error failed: ") + arm_client::result_to_string(ret);
}

void handle_set_controller_ip_service(DriverNode& node,
                                      const std::shared_ptr<tl_ros2_interface::srv::SetControllerIP::Request> request,
                                      std::shared_ptr<tl_ros2_interface::srv::SetControllerIP::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret =
      tl::set_controller_ip(node.session().fd(), request->name, request->addr, request->gateway, request->dns);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set controller IP successfully" : "Failed to set controller IP";
}

void handle_get_controller_id_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::string id{};
  tl::Result ret = tl::get_controller_id(node.session().fd(), id);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? id : "Failed to get controller id, ret=" + std::to_string(ret);
}

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

  // 新 SDK：io_state=true 但 io_port 为空会返回 PARAM_ERR(-3)，请求 IO 时必须指定端口
  if (param.ioState && param.ioPort.empty())
  {
    RCLCPP_WARN(node.get_logger(), "[GetRobotState]: io_state=true but io_port empty, force io_state=false");
    param.ioState = false;
  }

  const uint64_t start_seq = arm_client::robot_state_seq();

  // get_robot_state 需使用 7000 端口（fd_aux），回调亦注册在 7000 端口
  tl::Result ret = tl::get_robot_state(node.session().fd_aux(), param);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to get robot state";
    return;
  }

  std::string state_message;
  if (!arm_client::wait_robot_state(start_seq, std::chrono::seconds(5), state_message))
  {
    response->success = false;
    response->message = "Timeout waiting for robot state";
    return;
  }

  response->success = true;
  response->message = state_message;
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
  RCLCPP_INFO(node.get_logger(), "Library version: %s", version.c_str());
  response->success = (version != "");
  response->message = response->success ? version : "Failed to get library version";
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

  std::vector<double> motor_current{};
  std::vector<double> motor_current_sync{};
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
    response->message = "Failed to get motor current, ret=" + std::to_string(ret);
  }
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
  response->message = response->success ? version : "Failed to get nexmotion lib version, ret=" + std::to_string(ret);
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

void handle_set_current_mode_service(DriverNode& node,
                                     const std::shared_ptr<tl_ros2_interface::srv::SetCurrentMode::Request> request,
                                     std::shared_ptr<tl_ros2_interface::srv::SetCurrentMode::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->mode < 0 || request->mode > 2)
  {
    response->success = false;
    response->message = "Invalid mode";
    return;
  }

  tl::Result ret = tl::set_current_mode(node.session().fd(), static_cast<tl::RobotMode>(request->mode));
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set current mode successfully" : "Failed to set current mode";
}

void handle_get_current_mode_service(DriverNode& node,
                                     const std::shared_ptr<tl_ros2_interface::srv::GetCurrentMode::Request> request,
                                     std::shared_ptr<tl_ros2_interface::srv::GetCurrentMode::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::RobotMode mode = static_cast<tl::RobotMode>(-1); // -1 哨兵值：查询失败时 response->mode 保持 -1
  tl::Result ret = tl::get_current_mode(node.session().fd(), mode);
  response->mode = static_cast<int>(mode);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Get current mode successfully" : "Failed to get current mode";
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
    response->message = "Failed to get current motor torque, ret=" + std::to_string(ret);
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

  double line_speed = 0.0;
  std::vector<double> joint_speed{};
  std::vector<double> joint_speed_sync{};
  tl::Result ret =
      tl::get_current_line_speed_and_joint_speed(node.session().fd(), line_speed, joint_speed, joint_speed_sync);
  if (ret == tl::Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get current line joint speed successfully";
    response->line_speed = line_speed;
    response->joint_speed = joint_speed;
    response->joint_speed_sync = joint_speed_sync;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get current line joint speed, ret=" + std::to_string(ret);
  }
}

void register_state_services(DriverNode& node, rclcpp::CallbackGroup::SharedPtr group)
{
  node.register_service<std_srvs::srv::Trigger>("/tl_driver/connect_arm", handle_connect_service, group);
  node.register_service<std_srvs::srv::Trigger>("/tl_driver/disconnect_arm", handle_disconnect_service, group);
  node.register_service<std_srvs::srv::Trigger>("/tl_driver/power_on", handle_poweron_service, group);
  node.register_service<std_srvs::srv::Trigger>("/tl_driver/power_off", handle_poweroff_service, group);
  node.register_service<std_srvs::srv::Trigger>("/tl_driver/clear_error", handle_clear_error_service, group);
  node.register_service<std_srvs::srv::Trigger>("/tl_driver/get_controller_id", handle_get_controller_id_service,
                                                group);
  node.register_service<tl_ros2_interface::srv::SetControllerIP>("/tl_driver/set_controller_ip",
                                                                 handle_set_controller_ip_service, group);
  node.register_service<tl_ros2_interface::srv::GetRobotState>("/tl_driver/get_robot_state",
                                                               handle_get_robot_state_service, group);
  node.register_service<std_srvs::srv::Trigger>("/tl_driver/get_library_version", handle_get_library_version_service,
                                                group);
  node.register_service<std_srvs::srv::Trigger>("/tl_driver/get_nexmotion_lib_version",
                                                handle_get_nexmotion_lib_version_service, group);
  node.register_service<tl_ros2_interface::srv::GetJointTemperature>("/tl_driver/get_joint_temperature",
                                                                     handle_get_joint_temperature_service, group);
  node.register_service<tl_ros2_interface::srv::GetJointVoltage>("/tl_driver/get_joint_voltage",
                                                                 handle_get_joint_voltage_service, group);
  node.register_service<tl_ros2_interface::srv::GetMotorCurrent>("/tl_driver/get_motor_current",
                                                                 handle_get_motor_current_service, group);
  node.register_service<tl_ros2_interface::srv::GetJointSoftwareVersion>(
      "/tl_driver/get_joint_software_version", handle_get_joint_software_version_service, group);
  node.register_service<tl_ros2_interface::srv::GetCurrentMotorTorque>("/tl_driver/get_current_motor_torque",
                                                                       handle_get_current_motor_torque_service, group);
  node.register_service<tl_ros2_interface::srv::GetCurrentLineJointSpeed>(
      "/tl_driver/get_current_line_joint_speed", handle_get_current_line_joint_speed_service, group);
  node.register_service<std_srvs::srv::Trigger>("/tl_driver/get_drag_status", handle_get_drag_status_service, group);
  node.register_service<tl_ros2_interface::srv::SetCurrentMode>("/tl_driver/set_current_mode",
                                                                handle_set_current_mode_service, group);
  node.register_service<tl_ros2_interface::srv::GetCurrentMode>("/tl_driver/get_current_mode",
                                                                handle_get_current_mode_service, group);
}
} // namespace tl_driver::services
