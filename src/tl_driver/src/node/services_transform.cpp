/// @file services_transform.cpp
/// @brief 坐标系服务：坐标系变换、当前坐标系读写的实现（由 driver_node.cpp 按域搬移，行为不变）。

#include <functional>
#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "arm_sdk.hpp"
#include "convert/units.hpp"
#include "node/registrars.hpp"
#include "node/driver_node.hpp"

#include "tl_ros2_interface/srv/coord_transform.hpp"
#include "tl_ros2_interface/srv/get_current_coord.hpp"
#include "tl_ros2_interface/srv/set_current_coord.hpp"

namespace tl_driver
{
namespace
{
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

  // ROS: 关节 rad / 笛卡尔 m + rad；SDK: 关节度 / 笛卡尔 mm + rad
  std::vector<double> originPos = request->origin_pos;
  if (request->origin_coord == 0)
  {
    convert::joint_rad_to_deg(originPos, 7);
  }
  else
  {
    convert::cartesian_m_to_mm(originPos);
  }

  // reference_pos 为逆解参考关节角（SDK 度为制；空向量保持为空）
  std::vector<double> referencePos = request->reference_pos;
  convert::joint_rad_to_deg(referencePos, referencePos.size());

  std::vector<double> targetPos;
  tl::Result ret = tl::get_origin_coord_to_target_coord(
      node.session().fd(), static_cast<tl::Coord>(request->origin_coord), originPos,
      static_cast<tl::Coord>(request->target_coord), targetPos, request->form, referencePos);

  if (ret == tl::Result::SUCCESS)
  {
    if (request->target_coord == 0)
    {
      convert::joint_deg_to_rad(targetPos, targetPos.size());
    }
    else
    {
      convert::cartesian_mm_to_m(targetPos);
    }
  }
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Coord transform successfully" : "Failed to transform coord";
  response->target_pos = targetPos;
}

} // namespace

void register_transform_services(DriverNode& node)
{
  node.create_service<tl_ros2_interface::srv::CoordTransform>(
      "/tl_driver/coord_transform",
      std::bind(&handle_coord_transform_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetCurrentCoord>(
      "/tl_driver/set_current_coord",
      std::bind(&handle_set_current_coord_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetCurrentCoord>(
      "/tl_driver/get_current_coord",
      std::bind(&handle_get_current_coord_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
}
} // namespace tl_driver
