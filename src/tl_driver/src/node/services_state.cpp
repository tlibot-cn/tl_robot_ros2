/// @file services_state.cpp
/// @brief 状态类服务：控制器 IP/序列号、库版本、拖拽状态与模式的实现。

#include <functional>
#include <memory>
#include <string>

#include "arm_sdk.hpp"
#include "node/registrars.hpp"
#include "node/driver_node.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "tl_ros2_interface/srv/get_current_mode.hpp"
#include "tl_ros2_interface/srv/set_controller_ip.hpp"
#include "tl_ros2_interface/srv/set_current_mode.hpp"

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

  std::string id{};
  tl::Result ret = tl::get_controller_id(node.session().fd(), id);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? id : "Failed to get controller id, ret=" + std::to_string(ret);
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
    response->success = false;
    response->message = "Failed to get drag status, ret=" + std::to_string(ret);
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

} // namespace

void register_state_services(DriverNode& node)
{
  node.create_service<std_srvs::srv::Trigger>(
      "/tl_driver/get_controller_id",
      std::bind(&handle_get_controller_id_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetControllerIP>(
      "/tl_driver/set_controller_ip",
      std::bind(&handle_set_controller_ip_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<std_srvs::srv::Trigger>(
      "/tl_driver/get_library_version",
      std::bind(&handle_get_library_version_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<std_srvs::srv::Trigger>(
      "/tl_driver/get_drag_status",
      std::bind(&handle_get_drag_status_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetCurrentMode>(
      "/tl_driver/set_current_mode",
      std::bind(&handle_set_current_mode_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetCurrentMode>(
      "/tl_driver/get_current_mode",
      std::bind(&handle_get_current_mode_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
}
} // namespace tl_driver
