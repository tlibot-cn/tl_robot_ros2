#ifndef TL_DRIVER__SERVICES_IO_H_
#define TL_DRIVER__SERVICES_IO_H_

/// @file services_io.h
/// @brief IO 类服务：数字输出、数字输入输出、Modbus 读写、日志下载。

#include <memory>

#include <rclcpp/callback_group.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tl_ros2_interface/srv/get_digital_input_output.hpp>
#include <tl_ros2_interface/srv/log_download.hpp>
#include <tl_ros2_interface/srv/modbus_read.hpp>
#include <tl_ros2_interface/srv/modbus_write.hpp>
#include <tl_ros2_interface/srv/set_digital_output.hpp>

namespace tl_driver
{
class DriverNode;

namespace services
{
/// @brief 处理 /tl_driver/log_download 服务。
void handle_log_download_service(DriverNode& node,
                                 const std::shared_ptr<tl_ros2_interface::srv::LogDownload::Request> request,
                                 std::shared_ptr<tl_ros2_interface::srv::LogDownload::Response> response);

/// @brief 处理 /tl_driver/set_digital_output 服务。
void handle_set_digital_output_service(DriverNode& node,
                                       const std::shared_ptr<tl_ros2_interface::srv::SetDigitalOutput::Request> request,
                                       std::shared_ptr<tl_ros2_interface::srv::SetDigitalOutput::Response> response);

/// @brief 处理 /tl_driver/get_digital_input_output 服务。
void handle_get_digital_input_output_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::GetDigitalInputOutput::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetDigitalInputOutput::Response> response);

/// @brief 处理 /tl_driver/modbus_write 服务。
void handle_modbus_write_service(DriverNode& node,
                                 const std::shared_ptr<tl_ros2_interface::srv::ModbusWrite::Request> request,
                                 std::shared_ptr<tl_ros2_interface::srv::ModbusWrite::Response> response);

/// @brief 处理 /tl_driver/modbus_read 服务。
void handle_modbus_read_service(DriverNode& node,
                                const std::shared_ptr<tl_ros2_interface::srv::ModbusRead::Request> request,
                                std::shared_ptr<tl_ros2_interface::srv::ModbusRead::Response> response);

/// @brief 注册本域服务。
/// @param[in] node 节点
/// @param[in] group 回调组
void register_io_services(DriverNode& node, rclcpp::CallbackGroup::SharedPtr group);
} // namespace services
} // namespace tl_driver

#endif // TL_DRIVER__SERVICES_IO_H_
