/// @file services_session.cpp
/// @brief 会话类服务：连接/断开/上下电/清错的实现（由 driver_node.cpp 按域搬移，行为不变）。

#include <functional>
#include <memory>
#include <string>

#include "arm_client.hpp"
#include "arm_sdk.hpp"
#include "node/registrars.hpp"
#include "node/driver_node.h"

#include "std_srvs/srv/trigger.hpp"

namespace tl_driver
{
namespace
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
} // namespace

void register_session_services(DriverNode& node)
{
  node.create_service<std_srvs::srv::Trigger>(
      "/tl_driver/connect_arm",
      std::bind(&handle_connect_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<std_srvs::srv::Trigger>(
      "/tl_driver/disconnect_arm",
      std::bind(&handle_disconnect_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<std_srvs::srv::Trigger>(
      "/tl_driver/power_on",
      std::bind(&handle_poweron_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<std_srvs::srv::Trigger>(
      "/tl_driver/power_off",
      std::bind(&handle_poweroff_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<std_srvs::srv::Trigger>(
      "/tl_driver/clear_error",
      std::bind(&handle_clear_error_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
}
} // namespace tl_driver
