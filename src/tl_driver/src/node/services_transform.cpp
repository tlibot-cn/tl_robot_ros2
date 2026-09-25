#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "tl_driver/arm_sdk.hpp"
#include "tl_driver/node/registrars.hpp"
#include "tl_driver/tl_driver.h"

#include "tl_ros2_interface/srv/coord_transform.hpp"
#include "tl_ros2_interface/srv/get_pos_reachable.hpp"
#include "tl_ros2_interface/srv/get_pos_transform.hpp"

namespace tl_driver
{
namespace
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

  tl::Result ret = tl::get_origin_coord_to_target_coord(node.session().fd(), request->origin_coord, originPos,
                                                        request->target_coord, targetPos, request->form, referencePos);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Coord transform successfully" : "Failed to transform coord";
  response->target_pos = targetPos;
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
    response->message = "Fail to get pos reachable status";
  }
}

} // namespace

void register_transform_services(DriverNode& node)
{
  node.create_service<tl_ros2_interface::srv::GetPosTransform>(
      "/tl_driver/get_quat2rpy",
      std::bind(&handle_get_quat2rpy_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetPosTransform>(
      "/tl_driver/get_rpy2quat",
      std::bind(&handle_get_rpy2quat_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetPosTransform>(
      "/tl_driver/get_rpy2r",
      std::bind(&handle_get_rpy2r_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetPosTransform>(
      "/tl_driver/get_tr2r",
      std::bind(&handle_get_tr2r_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetPosTransform>(
      "/tl_driver/get_r2tr",
      std::bind(&handle_get_r2tr_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::CoordTransform>(
      "/tl_driver/coord_transform",
      std::bind(&handle_coord_transform_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetPosReachable>(
      "/tl_driver/get_pos_reachable",
      std::bind(&handle_get_pos_reachable_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
}

} // namespace tl_driver
