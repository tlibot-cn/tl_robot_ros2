#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "arm_sdk.hpp"
#include "convert/msg_convert.hpp"
#include "node/registrars.hpp"
#include "node/driver_node.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "tl_ros2_interface/srv/get_coord_num.hpp"
#include "tl_ros2_interface/srv/get_current_coord.hpp"
#include "tl_ros2_interface/srv/get_current_mode.hpp"
#include "tl_ros2_interface/srv/get_dh_param.hpp"
#include "tl_ros2_interface/srv/get_global_pos.hpp"
#include "tl_ros2_interface/srv/restore_default_dh_param.hpp"
#include "tl_ros2_interface/srv/set_axis_zero_pos.hpp"
#include "tl_ros2_interface/srv/set_controller_ip.hpp"
#include "tl_ros2_interface/srv/set_coord_num.hpp"
#include "tl_ros2_interface/srv/set_current_coord.hpp"
#include "tl_ros2_interface/srv/set_current_mode.hpp"
#include "tl_ros2_interface/srv/set_dh_param.hpp"
#include "tl_ros2_interface/srv/set_global_pos.hpp"
#include "tl_ros2_interface/srv/set_tool_param.hpp"
#include "tl_ros2_interface/srv/set_user_coord.hpp"

namespace tl_driver
{
namespace
{

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

  std::string id = {0};
  tl::Result ret = tl::get_controller_id(node.session().fd(), id);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? std::string(id) : "Failed to get controller ID";
}

void handle_set_tool_param_service(DriverNode& node,
                                   const std::shared_ptr<tl_ros2_interface::srv::SetToolParam::Request> request,
                                   std::shared_ptr<tl_ros2_interface::srv::SetToolParam::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::ToolParam param{};
  param.X = request->param.x;
  param.Y = request->param.y;
  param.Z = request->param.z;
  param.A = request->param.a;
  param.B = request->param.b;
  param.C = request->param.c;
  param.payloadMass = request->param.payload_mass;
  param.payloadInertia = request->param.payload_inertia;
  param.payloadMassCenter_X = request->param.payload_mass_center_x;
  param.payloadMassCenter_Y = request->param.payload_mass_center_y;
  param.payloadMassCenter_Z = request->param.payload_mass_center_z;

  tl::Result ret = tl::set_tool_hand_param(node.session().fd(), request->tool_num, param);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set tool hand param successfully" : "Failed to set tool hand param";
}

void handle_set_user_coord_service(DriverNode& node,
                                   const std::shared_ptr<tl_ros2_interface::srv::SetUserCoord::Request> request,
                                   std::shared_ptr<tl_ros2_interface::srv::SetUserCoord::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::vector<double> pos = {request->pos.position.x, request->pos.position.y, request->pos.position.z,
                             request->pos.rpy.x,      request->pos.rpy.y,      request->pos.rpy.z};

  tl::Result ret = tl::set_user_coordinate_data(node.session().fd(), request->user_num, pos);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set user coordinate successfully" : "Failed to set user coordinate";
}

void handle_set_axis_zero_pos_service(DriverNode& node,
                                      const std::shared_ptr<tl_ros2_interface::srv::SetAxisZeroPos::Request> request,
                                      std::shared_ptr<tl_ros2_interface::srv::SetAxisZeroPos::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::set_axis_zero_position(node.session().fd(), request->axis);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set user coordinate successfully" : "Failed to set user coordinate";
}

void handle_set_current_coord_service(DriverNode& node,
                                      const std::shared_ptr<tl_ros2_interface::srv::SetCurrentCoord::Request> request,
                                      std::shared_ptr<tl_ros2_interface::srv::SetCurrentCoord::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::set_current_coord(node.session().fd(), request->coord);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set current coordinate successfully" : "Failed to set current coordinate";
}

void handle_get_current_coord_service(DriverNode& node,
                                      const std::shared_ptr<tl_ros2_interface::srv::GetCurrentCoord::Request> request,
                                      std::shared_ptr<tl_ros2_interface::srv::GetCurrentCoord::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int coord;
  tl::Result ret = tl::get_current_coord(node.session().fd(), coord);
  if (ret == tl::Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get current coordinate successfully";
    response->coord = coord;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get current coordinate";
  }
}

void handle_set_coord_num_service(DriverNode& node,
                                  const std::shared_ptr<tl_ros2_interface::srv::SetCoordNum::Request> request,
                                  std::shared_ptr<tl_ros2_interface::srv::SetCoordNum::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::set_tool_hand_number(node.session().fd(), request->tool_num);
  tl::Result ret1 = tl::set_user_coord_number(node.session().fd(), request->user_num);
  if (ret == tl::Result::SUCCESS && ret1 == tl::Result::SUCCESS)
  {
    response->success = true;
    response->message = "Set all coordinate number successfully";
  }
  else
  {
    response->success = false;
    response->message = "Failed to set all coordinate number";
  }
}

void handle_get_coord_num_service(DriverNode& node,
                                  const std::shared_ptr<tl_ros2_interface::srv::GetCoordNum::Request> request,
                                  std::shared_ptr<tl_ros2_interface::srv::GetCoordNum::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int toolNum = -1, userNum = -1;
  tl::Result ret = tl::get_tool_hand_number(node.session().fd(), toolNum);
  tl::Result ret1 = tl::get_user_coord_number(node.session().fd(), userNum);
  if (ret == tl::Result::SUCCESS && ret1 == tl::Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get all coordinate number successfully";
    response->tool_num = toolNum;
    response->user_num = userNum;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get all coordinate number";
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

  tl::Result ret = tl::set_current_mode(node.session().fd(), request->mode);
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

  int mode = -1;
  tl::Result ret = tl::get_current_mode(node.session().fd(), mode);
  response->mode = mode;
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Get current mode successfully" : "Failed to get current mode";
}

void handle_set_global_pos_service(DriverNode& node,
                                   const std::shared_ptr<tl_ros2_interface::srv::SetGlobalPos::Request> request,
                                   std::shared_ptr<tl_ros2_interface::srv::SetGlobalPos::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  auto isValidGP = [](const std::string& str) -> bool
  {
    if (str.length() != 6 || str.substr(0, 2) != "GP")
    {
      return false;
    }
    int num = std::stoi(str.substr(2));
    return (num >= 1 && num <= 9999);
  };

  if (!isValidGP(request->pos_name))
  {
    response->success = false;
    response->message = "Invalid global pos name";
    return;
  }

  tl::Result ret = tl::set_global_position(node.session().fd(), request->pos_name, request->pos_info);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set global pos successfully" : "Failed to set global pos";
}

void handle_get_global_pos_service(DriverNode& node,
                                   const std::shared_ptr<tl_ros2_interface::srv::GetGlobalPos::Request> request,
                                   std::shared_ptr<tl_ros2_interface::srv::GetGlobalPos::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  auto isValidGP = [](const std::string& str) -> bool
  {
    if (str.length() != 6 || str.substr(0, 2) != "GP")
    {
      return false;
    }
    int num = std::stoi(str.substr(2));
    return (num >= 1 && num <= 9999);
  };

  if (!isValidGP(request->pos_name))
  {
    response->success = false;
    response->message = "Invalid global pos name";
    return;
  }

  std::vector<double> pos;
  tl::Result ret = tl::get_global_position(node.session().fd(), request->pos_name, pos);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Get global pos successfully" : "Failed to get global pos";
  response->pos = pos;
}

void handle_set_dh_param_service(DriverNode& node,
                                 const std::shared_ptr<tl_ros2_interface::srv::SetDHParam::Request> request,
                                 std::shared_ptr<tl_ros2_interface::srv::SetDHParam::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::RobotDHParam dh_param{};
  convert::to_sdk_dh_param(request->param, dh_param);

  tl::Result ret = tl::set_robot_dh_param(node.session().fd(), dh_param);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set DH param successfully" : "Failed to set DH param";

  // 设置参数后机械臂会下电
  node.session().power_off();
  node.session().power_on();
}

void handle_get_dh_param_service(DriverNode& node,
                                 const std::shared_ptr<tl_ros2_interface::srv::GetDHParam::Request> request,
                                 std::shared_ptr<tl_ros2_interface::srv::GetDHParam::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::RobotDHParam dh_param{};
  tl::Result ret = tl::get_robot_dh_param(node.session().fd(), dh_param);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Get DH param successfully" : "Failed to get DH param";

  convert::to_msg_dh_param(dh_param, response->param);
}

void handle_restore_default_dh_param_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::RestoreDefaultDHParam::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::RestoreDefaultDHParam::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::restore_default_param_DH(node.session().fd(), request->robot_num);
  response->success = (ret == tl::Result::SUCCESS);
  response->message =
      response->success ? "Restore default DH param successfully" : "Failed to restore default DH param";
}

void handle_set_default_cartesian_param_service(DriverNode& node,
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

  tl::Result ret = tl::set_default_cartesian_params(node.session().fd());
  response->success = (ret == tl::Result::SUCCESS);
  response->message =
      response->success ? "Set default cartesian param successfully" : "Failed to set default cartesian param";
}

} // namespace

void register_config_services(DriverNode& node)
{
  node.create_service<tl_ros2_interface::srv::SetControllerIP>(
      "/tl_driver/set_controller_ip",
      std::bind(&handle_set_controller_ip_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<std_srvs::srv::Trigger>(
      "/tl_driver/get_controller_id",
      std::bind(&handle_get_controller_id_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetToolParam>(
      "/tl_driver/set_tool_param",
      std::bind(&handle_set_tool_param_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetUserCoord>(
      "/tl_driver/set_user_coord",
      std::bind(&handle_set_user_coord_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetAxisZeroPos>(
      "/tl_driver/set_axis_zero_pos",
      std::bind(&handle_set_axis_zero_pos_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetCurrentCoord>(
      "/tl_driver/set_current_coord",
      std::bind(&handle_set_current_coord_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetCurrentCoord>(
      "/tl_driver/get_current_coord",
      std::bind(&handle_get_current_coord_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetCoordNum>(
      "/tl_driver/set_coord_num",
      std::bind(&handle_set_coord_num_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetCoordNum>(
      "/tl_driver/get_coord_num",
      std::bind(&handle_get_coord_num_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetCurrentMode>(
      "/tl_driver/set_current_mode",
      std::bind(&handle_set_current_mode_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetCurrentMode>(
      "/tl_driver/get_current_mode",
      std::bind(&handle_get_current_mode_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetGlobalPos>(
      "/tl_driver/set_global_pos",
      std::bind(&handle_set_global_pos_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetGlobalPos>(
      "/tl_driver/get_global_pos",
      std::bind(&handle_get_global_pos_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetDHParam>(
      "/tl_driver/set_dh_param",
      std::bind(&handle_set_dh_param_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetDHParam>(
      "/tl_driver/get_dh_param",
      std::bind(&handle_get_dh_param_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::RestoreDefaultDHParam>(
      "/tl_driver/restore_default_dh_param", std::bind(&handle_restore_default_dh_param_service, std::ref(node),
                                                       std::placeholders::_1, std::placeholders::_2));
  node.create_service<std_srvs::srv::Trigger>("/tl_driver/set_default_cartesian_param",
                                              std::bind(&handle_set_default_cartesian_param_service, std::ref(node),
                                                        std::placeholders::_1, std::placeholders::_2));
}

} // namespace tl_driver
