/// @file services_io.cpp
/// @brief IO 类服务：数字输出、数字输入输出、Modbus 读写、日志下载的实现（由 driver_node.cpp 按域搬移，行为不变）。

#include "convert/msg_convert.h"
#include <functional>
#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "arm_sdk.hpp"
#include "node/registrars.hpp"
#include "node/driver_node.h"

#include "tl_ros2_interface/srv/get_digital_input_output.hpp"
#include "tl_ros2_interface/srv/log_download.hpp"
#include "tl_ros2_interface/srv/modbus_read.hpp"
#include "tl_ros2_interface/srv/modbus_write.hpp"
#include "tl_ros2_interface/srv/set_digital_output.hpp"

namespace tl_driver
{
namespace
{
void handle_log_download_service(DriverNode& node,
                                 const std::shared_ptr<tl_ros2_interface::srv::LogDownload::Request> request,
                                 std::shared_ptr<tl_ros2_interface::srv::LogDownload::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::log_download_by_quantity(node.session().fd(), request->count, request->directory_path);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Log download successfully" : "Failed to download log";
}

void handle_set_digital_output_service(DriverNode& node,
                                       const std::shared_ptr<tl_ros2_interface::srv::SetDigitalOutput::Request> request,
                                       std::shared_ptr<tl_ros2_interface::srv::SetDigitalOutput::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->value < 0 || request->value > 1)
  {
    response->success = false;
    response->message = "Invalid value";
    return;
  }

  tl::Result ret = tl::set_digital_output(node.session().fd(), request->port, static_cast<tl::IoLevel>(request->value));
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set digital output successfully" : "Failed to set digital output";
}

void handle_get_digital_input_output_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::GetDigitalInputOutput::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetDigitalInputOutput::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::vector<int> digitalInput;
  std::vector<int> digitalOutput;
  tl::Result ret = tl::get_digital_input(node.session().fd(), digitalInput);
  tl::Result ret1 = tl::get_digital_output(node.session().fd(), digitalOutput);
  if (ret == tl::Result::SUCCESS && ret1 == tl::Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get digital input and output successfully";
    response->input = digitalInput;
    response->output = digitalOutput;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get digital input and output";
  }
}

void handle_modbus_write_service(DriverNode& node,
                                 const std::shared_ptr<tl_ros2_interface::srv::ModbusWrite::Request> request,
                                 std::shared_ptr<tl_ros2_interface::srv::ModbusWrite::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::ModbusMasterParameter master_param;
  if (!convert::to_modbus_master_param(request->master_param, master_param))
  {
    response->success = false;
    response->message = "Invalid master type";
    return;
  }

  tl::Result ret = tl::modbus_set_master_parameter(node.session().fd(), request->master_id, master_param);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to set master paramter";
    return;
  }

  ret = tl::modbus_open_master(node.session().fd(), request->master_id);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to open master";
    return;
  }

  std::vector<int> data = request->data;
  ret = tl::modbus_write_multiple_holding_registers(node.session().fd(), request->master_id, request->addr, data);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Modbus write successfully" : "Failed to write Modbus";
}

void handle_modbus_read_service(DriverNode& node,
                                const std::shared_ptr<tl_ros2_interface::srv::ModbusRead::Request> request,
                                std::shared_ptr<tl_ros2_interface::srv::ModbusRead::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::ModbusMasterParameter master_param;
  if (!convert::to_modbus_master_param(request->master_param, master_param))
  {
    response->success = false;
    response->message = "Invalid master type";
    return;
  }

  tl::Result ret = tl::modbus_set_master_parameter(node.session().fd(), request->master_id, master_param);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to set master paramter";
    return;
  }

  ret = tl::modbus_open_master(node.session().fd(), request->master_id);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to open master";
    return;
  }

  std::vector<int> data{};
  ret = tl::modbus_read_holding_registers(node.session().fd(), request->master_id, request->addr, request->quantity,
                                          data);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Modbus read successfully" : "Failed to read Modbus";
  response->data = data;
}

} // namespace

void register_io_services(DriverNode& node)
{
  node.create_service<tl_ros2_interface::srv::SetDigitalOutput>(
      "/tl_driver/set_digital_output",
      std::bind(&handle_set_digital_output_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetDigitalInputOutput>(
      "/tl_driver/get_digital_input_output", std::bind(&handle_get_digital_input_output_service, std::ref(node),
                                                       std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::ModbusWrite>(
      "/tl_driver/modbus_write",
      std::bind(&handle_modbus_write_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::ModbusRead>(
      "/tl_driver/modbus_read",
      std::bind(&handle_modbus_read_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::LogDownload>(
      "/tl_driver/log_download",
      std::bind(&handle_log_download_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
}
} // namespace tl_driver
