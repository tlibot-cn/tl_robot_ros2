/// @file services_transform.cpp
/// @brief 位姿换算与坐标系服务：rpy↔四元数、r↔tr、坐标系变换、当前坐标系读写的实现（由 driver_node.cpp
/// 按域搬移，行为不变）。

#include "node/services_transform.h"

#include <chrono>
#include <string>
#include <vector>

#include "arm_sdk.hpp"
#include "node/driver_node.h"

namespace tl_driver::services
{
void handle_get_quat2rpy_service(DriverNode& node,
                                 const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
                                 std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->input.size() < 4)
  {
    response->success = false;
    response->message = "Invalid quat input";
    return;
  }

  std::vector<double> rpy;
  tl::Result ret = tl::get_quat2rpy(node.session().fd(), request->input, rpy);
  if (ret == tl::Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get quat2rpy successfully";
    response->output = rpy;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get quat2rpy";
  }
}

void handle_get_rpy2quat_service(DriverNode& node,
                                 const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
                                 std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->input.size() < 3)
  {
    response->success = false;
    response->message = "Invalid rpy input";
    return;
  }

  std::vector<double> quat;
  tl::Result ret = tl::get_rpy2quat(node.session().fd(), request->input, quat);
  if (ret == tl::Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get rpy2quat successfully";
    response->output = quat;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get rpy2quat";
  }
}

void handle_get_rpy2r_service(DriverNode& node,
                              const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
                              std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->input.size() < 3)
  {
    response->success = false;
    response->message = "Invalid rpy input";
    return;
  }

  std::vector<double> rot;
  tl::Result ret = tl::get_rpy2r(node.session().fd(), request->input, rot);
  if (ret == tl::Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get rpy2r successfully";
    response->output = rot;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get rpy2r";
  }
}

void handle_get_tr2r_service(DriverNode& node,
                             const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
                             std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->input.size() < 16)
  {
    response->success = false;
    response->message = "Invalid tr input";
    return;
  }

  std::vector<double> rot;
  tl::Result ret = tl::get_tr2r(node.session().fd(), request->input, rot);
  if (ret == tl::Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get tr2r successfully";
    response->output = rot;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get tr2r";
  }
}

void handle_get_r2tr_service(DriverNode& node,
                             const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
                             std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->input.size() < 9)
  {
    response->success = false;
    response->message = "Invalid tr matrix input";
    return;
  }

  std::vector<double> tr_matrix;
  tl::Result ret = tl::get_r2tr(node.session().fd(), request->input, tr_matrix);
  if (ret == tl::Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get r2tr successfully";
    response->output = tr_matrix;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get r2tr";
  }
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

  tl::Result ret = tl::set_current_coord(node.session().fd(), static_cast<tl::Coord>(request->coord));
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

  tl::Coord coord = tl::Coord::JOINT;
  tl::Result ret = tl::get_current_coord(node.session().fd(), coord);
  if (ret == tl::Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get current coordinate successfully";
    response->coord = static_cast<int>(coord);
  }
  else
  {
    response->success = false;
    response->message = "Failed to get current coordinate";
  }
}

void handle_coord_transform_service(DriverNode& node,
                                    const std::shared_ptr<tl_ros2_interface::srv::CoordTransform::Request> request,
                                    std::shared_ptr<tl_ros2_interface::srv::CoordTransform::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->origin_coord < 0 || request->origin_coord > 3)
  {
    response->success = false;
    response->message = "Invalid origin coordinate";
    return;
  }

  if (request->target_coord < 0 || request->target_coord > 3)
  {
    response->success = false;
    response->message = "Invalid target coordinate";
    return;
  }

  std::vector<double> originPos = request->origin_pos;
  std::vector<double> referencePos = request->reference_pos;
  std::vector<double> targetPos;

  tl::Result ret = tl::get_origin_coord_to_target_coord(
      node.session().fd(), static_cast<tl::Coord>(request->origin_coord), originPos,
      static_cast<tl::Coord>(request->target_coord), targetPos, request->form, referencePos);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Coord transform successfully" : "Failed to transform coord";
  response->target_pos = targetPos;
}

void register_transform_services(DriverNode& node, rclcpp::CallbackGroup::SharedPtr group)
{
  node.register_service<tl_ros2_interface::srv::GetPosTransform>("/tl_driver/get_quat2rpy", handle_get_quat2rpy_service,
                                                                 group);
  node.register_service<tl_ros2_interface::srv::GetPosTransform>("/tl_driver/get_rpy2quat", handle_get_rpy2quat_service,
                                                                 group);
  node.register_service<tl_ros2_interface::srv::GetPosTransform>("/tl_driver/get_rpy2r", handle_get_rpy2r_service,
                                                                 group);
  node.register_service<tl_ros2_interface::srv::GetPosTransform>("/tl_driver/get_tr2r", handle_get_tr2r_service, group);
  node.register_service<tl_ros2_interface::srv::GetPosTransform>("/tl_driver/get_r2tr", handle_get_r2tr_service, group);
  node.register_service<tl_ros2_interface::srv::CoordTransform>("/tl_driver/coord_transform",
                                                                handle_coord_transform_service, group);
  node.register_service<tl_ros2_interface::srv::SetCurrentCoord>("/tl_driver/set_current_coord",
                                                                 handle_set_current_coord_service, group);
  node.register_service<tl_ros2_interface::srv::GetCurrentCoord>("/tl_driver/get_current_coord",
                                                                 handle_get_current_coord_service, group);
}
} // namespace tl_driver::services
