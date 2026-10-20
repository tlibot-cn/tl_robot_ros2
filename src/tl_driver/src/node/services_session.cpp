/// @file services_session.cpp
/// @brief 会话类服务：上下电/清错的实现（由 driver_node.cpp 按域搬移，行为不变）。
/// @note 连接只在 DriverNode::init() 建立、断开只在 ~DriverNode() 执行，不对外提供运行时连接/断开服务
///       （架构评审 D-10）。

#include <functional>
#include <memory>
#include <string>

#include "arm_client.hpp"
#include "arm_sdk.hpp"
#include "node/registrars.hpp"
#include "node/driver_node.hpp"

#include "std_srvs/srv/trigger.hpp"

namespace tl_driver
{
namespace
{
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

  const int fd = node.session().fd();
  tl::Result ret = tl::clear_error(fd);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Clear error successfully"
                                        : std::string("Clear error failed: ") + arm_client::result_to_string(ret);
}
} // namespace

void register_session_services(DriverNode& node)
{
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
