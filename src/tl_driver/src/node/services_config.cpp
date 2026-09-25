/// @file services_config.cpp
/// @brief 配置类服务：DH 参数/关节参数/工具/坐标系/全局点位/可达性的实现（由 driver_node.cpp 按域搬移，行为不变）。

#include "convert/msg_convert.hpp"
#include <functional>
#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "arm_sdk.hpp"
#include "node/registrars.hpp"
#include "node/driver_node.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "tl_ros2_interface/srv/get_coord_num.hpp"
#include "tl_ros2_interface/srv/get_dh_param.hpp"
#include "tl_ros2_interface/srv/get_global_pos.hpp"
#include "tl_ros2_interface/srv/get_pos_reachable.hpp"
#include "tl_ros2_interface/srv/get_robot_joint_param.hpp"
#include "tl_ros2_interface/srv/restore_default_dh_param.hpp"
#include "tl_ros2_interface/srv/set_axis_zero_pos.hpp"
#include "tl_ros2_interface/srv/set_coord_num.hpp"
#include "tl_ros2_interface/srv/set_dh_param.hpp"
#include "tl_ros2_interface/srv/set_drag_mode.hpp"
#include "tl_ros2_interface/srv/set_global_pos.hpp"
#include "tl_ros2_interface/srv/set_robot_joint_param.hpp"
#include "tl_ros2_interface/srv/set_tool_param.hpp"
#include "tl_ros2_interface/srv/set_user_coord.hpp"

namespace tl_driver
{
namespace
{
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
    response->param.reduction_ratio = param.reduceRatio;
    response->param.encoder_resolution = param.encoderResolution;
    response->param.pos_sw_limit = param.maxPos;
    response->param.neg_sw_limit = param.minPos;
    response->param.rated_rot_speed = param.ratedRotSpeed;
    response->param.max_rot_speed = param.maxRotSpeed;
    response->param.max_acc = param.maxAcc;
    response->param.max_dec = param.maxDec;
    response->param.direction = param.axisDirection;
    // 新 SDK 无 rated_derot_speed / max_derot_speed / rated_vel / rated_devel 对应字段，保持默认 0
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
  param.reduceRatio = request->param.reduction_ratio;
  param.encoderResolution = request->param.encoder_resolution;
  param.maxPos = request->param.pos_sw_limit;
  param.minPos = request->param.neg_sw_limit;
  param.ratedRotSpeed = request->param.rated_rot_speed;
  param.maxRotSpeed = request->param.max_rot_speed;
  param.maxAcc = request->param.max_acc;
  param.maxDec = request->param.max_dec;
  param.axisDirection = request->param.direction;
  // 新 SDK 无 rated_derot_speed / max_derot_speed / rated_vel / rated_devel 对应字段，忽略

  tl::Result ret = tl::set_robot_joint_param(node.session().fd(), request->id, param);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set robot joint param successfully" : "Falied to set robot joint param";

  // 设置参数后机械臂会下电
  node.session().power_off();
  node.session().power_on();
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

  tl::Result ret = tl::set_darg_mode(node.session().fd(), static_cast<tl::DragMode>(request->mode));
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set drag mode successfully" : "Failed to set drag mode";
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

  // 使用新 SDK 推荐的 UserCoordParam 重载（旧的 vector 重载已标记 deprecated）
  tl::UserCoordParam param{};
  param.location_type = 0; // 静态用户坐标
  param.position[0] = request->pos.position.x;
  param.position[1] = request->pos.position.y;
  param.position[2] = request->pos.position.z;
  param.position[3] = request->pos.rpy.x;
  param.position[4] = request->pos.rpy.y;
  param.position[5] = request->pos.rpy.z;

  tl::Result ret = tl::set_user_coordinate_data(node.session().fd(), request->user_num, param);
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

void handle_get_pos_reachable_service(DriverNode& node,
                                      const std::shared_ptr<tl_ros2_interface::srv::GetPosReachable::Request> request,
                                      std::shared_ptr<tl_ros2_interface::srv::GetPosReachable::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->move_type != "MOVJ" && request->move_type != "MOVL")
  {
    response->success = false;
    response->message = "Invalid move type";
    return;
  }

  std::vector<double> queryPos = request->pos;

  bool result = false;
  tl::Result ret = tl::get_pos_reachable(node.session().fd(), queryPos, request->move_type, result);
  if (ret == tl::Result::SUCCESS)
  {
    response->success = result;
    response->message = response->success ? "Target pos is reachable" : "Target pos is not reachable";
  }
  else
  {
    response->success = false;
    response->message = "Fail to get pos reachable status" + std::to_string(ret);
  }
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

  // 新 SDK: 标准DH参数 alpha/a/theta/d，逐字段映射到 SDK 的 tl::RobotDHParam
  tl::RobotDHParam dh = convert::to_dh_param(request->param);

  tl::Result ret = tl::set_robot_dh_param(node.session().fd(), dh);
  if (ret == tl::Result::SUCCESS)
  {
    response->success = true;
    response->message = "Set DH param successfully";
  }
  else
  {
    response->success = false;
    response->message = "Failed to set DH param, ret=" + std::to_string(ret);
  }
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

  // 新 SDK: 标准DH参数 alpha/a/theta/d，逐字段映射到 ROS2 消息
  tl::RobotDHParam dh{};
  tl::Result ret = tl::get_robot_dh_param(node.session().fd(), dh);
  if (ret == tl::Result::SUCCESS)
  {
    convert::from_dh_param(dh, response->param);
    response->success = true;
    response->message = "Get DH param successfully";
  }
  else
  {
    response->success = false;
    response->message = "Failed to get DH param, ret=" + std::to_string(ret);
  }
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

  if (!convert::is_valid_gp_name(request->pos_name))
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

  if (!convert::is_valid_gp_name(request->pos_name))
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

} // namespace

void register_config_services(DriverNode& node)
{
  node.create_service<tl_ros2_interface::srv::RestoreDefaultDHParam>(
      "/tl_driver/restore_default_dh_param", std::bind(&handle_restore_default_dh_param_service, std::ref(node),
                                                       std::placeholders::_1, std::placeholders::_2));
  node.create_service<std_srvs::srv::Trigger>("/tl_driver/set_default_cartesian_param",
                                              std::bind(&handle_set_default_cartesian_param_service, std::ref(node),
                                                        std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetDragMode>(
      "/tl_driver/set_drag_mode",
      std::bind(&handle_set_drag_mode_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetDHParam>(
      "/tl_driver/set_dh_param",
      std::bind(&handle_set_dh_param_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetDHParam>(
      "/tl_driver/get_dh_param",
      std::bind(&handle_get_dh_param_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetRobotJointParam>(
      "/tl_driver/set_robot_joint_param",
      std::bind(&handle_set_robot_joint_param_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetRobotJointParam>(
      "/tl_driver/get_robot_joint_param",
      std::bind(&handle_get_robot_joint_param_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetToolParam>(
      "/tl_driver/set_tool_param",
      std::bind(&handle_set_tool_param_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetUserCoord>(
      "/tl_driver/set_user_coord",
      std::bind(&handle_set_user_coord_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetAxisZeroPos>(
      "/tl_driver/set_axis_zero_pos",
      std::bind(&handle_set_axis_zero_pos_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetCoordNum>(
      "/tl_driver/set_coord_num",
      std::bind(&handle_set_coord_num_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetCoordNum>(
      "/tl_driver/get_coord_num",
      std::bind(&handle_get_coord_num_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetPosReachable>(
      "/tl_driver/get_pos_reachable",
      std::bind(&handle_get_pos_reachable_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetGlobalPos>(
      "/tl_driver/set_global_pos",
      std::bind(&handle_set_global_pos_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetGlobalPos>(
      "/tl_driver/get_global_pos",
      std::bind(&handle_get_global_pos_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
}
} // namespace tl_driver
