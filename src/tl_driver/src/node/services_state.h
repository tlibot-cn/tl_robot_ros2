#ifndef TL_DRIVER__SERVICES_STATE_H_
#define TL_DRIVER__SERVICES_STATE_H_

/// @file services_state.h
/// @brief 状态类服务：连接/上下电/清错/版本/温度/电压/电流/力矩/模式。

#include <memory>

#include <rclcpp/callback_group.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tl_ros2_interface/srv/get_current_line_joint_speed.hpp>
#include <tl_ros2_interface/srv/get_current_mode.hpp>
#include <tl_ros2_interface/srv/get_current_motor_torque.hpp>
#include <tl_ros2_interface/srv/get_joint_software_version.hpp>
#include <tl_ros2_interface/srv/get_joint_temperature.hpp>
#include <tl_ros2_interface/srv/get_joint_voltage.hpp>
#include <tl_ros2_interface/srv/get_motor_current.hpp>
#include <tl_ros2_interface/srv/get_robot_state.hpp>
#include <tl_ros2_interface/srv/set_controller_ip.hpp>
#include <tl_ros2_interface/srv/set_current_mode.hpp>

namespace tl_driver
{
class DriverNode;

namespace services
{
/// @brief 处理 /tl_driver/connect_arm 服务。
void handle_connect_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

/// @brief 处理 /tl_driver/disconnect_arm 服务。
void handle_disconnect_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response);

/// @brief 处理 /tl_driver/power_on 服务。
void handle_poweron_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

/// @brief 处理 /tl_driver/power_off 服务。
void handle_poweroff_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response);

/// @brief 处理 /tl_driver/clear_error 服务。
void handle_clear_error_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response);

/// @brief 处理 /tl_driver/set_controller_ip 服务。
void handle_set_controller_ip_service(DriverNode& node,
                                      const std::shared_ptr<tl_ros2_interface::srv::SetControllerIP::Request> request,
                                      std::shared_ptr<tl_ros2_interface::srv::SetControllerIP::Response> response);

/// @brief 处理 /tl_driver/get_controller_id 服务。
void handle_get_controller_id_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

/// @brief 处理 /tl_driver/get_robot_state 服务。
void handle_get_robot_state_service(DriverNode& node,
                                    const std::shared_ptr<tl_ros2_interface::srv::GetRobotState::Request> request,
                                    std::shared_ptr<tl_ros2_interface::srv::GetRobotState::Response> response);

/// @brief 处理 /tl_driver/get_library_version 服务。
void handle_get_library_version_service(DriverNode& node,
                                        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

/// @brief 处理 /tl_driver/get_joint_temperature 服务。
void handle_get_joint_temperature_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::GetJointTemperature::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetJointTemperature::Response> response);

/// @brief 处理 /tl_driver/get_joint_voltage 服务。
void handle_get_joint_voltage_service(DriverNode& node,
                                      const std::shared_ptr<tl_ros2_interface::srv::GetJointVoltage::Request> request,
                                      std::shared_ptr<tl_ros2_interface::srv::GetJointVoltage::Response> response);

/// @brief 处理 /tl_driver/get_motor_current 服务。
void handle_get_motor_current_service(DriverNode& node,
                                      const std::shared_ptr<tl_ros2_interface::srv::GetMotorCurrent::Request> request,
                                      std::shared_ptr<tl_ros2_interface::srv::GetMotorCurrent::Response> response);

/// @brief 处理 /tl_driver/get_joint_software_version 服务。
void handle_get_joint_software_version_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::GetJointSoftwareVersion::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetJointSoftwareVersion::Response> response);

/// @brief 处理 /tl_driver/get_nexmotion_lib_version 服务。
void handle_get_nexmotion_lib_version_service(DriverNode& node,
                                              const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                              std::shared_ptr<std_srvs::srv::Trigger::Response> response);

/// @brief 处理 /tl_driver/get_drag_status 服务。
void handle_get_drag_status_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

/// @brief 处理 /tl_driver/set_current_mode 服务。
void handle_set_current_mode_service(DriverNode& node,
                                     const std::shared_ptr<tl_ros2_interface::srv::SetCurrentMode::Request> request,
                                     std::shared_ptr<tl_ros2_interface::srv::SetCurrentMode::Response> response);

/// @brief 处理 /tl_driver/get_current_mode 服务。
void handle_get_current_mode_service(DriverNode& node,
                                     const std::shared_ptr<tl_ros2_interface::srv::GetCurrentMode::Request> request,
                                     std::shared_ptr<tl_ros2_interface::srv::GetCurrentMode::Response> response);

/// @brief 处理 /tl_driver/get_current_motor_torque 服务。
void handle_get_current_motor_torque_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::GetCurrentMotorTorque::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetCurrentMotorTorque::Response> response);

/// @brief 处理 /tl_driver/get_current_line_joint_speed 服务。
void handle_get_current_line_joint_speed_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::GetCurrentLineJointSpeed::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetCurrentLineJointSpeed::Response> response);

/// @brief 注册本域服务。
/// @param[in] node 节点
/// @param[in] group 回调组
void register_state_services(DriverNode& node, rclcpp::CallbackGroup::SharedPtr group);
} // namespace services
} // namespace tl_driver

#endif // TL_DRIVER__SERVICES_STATE_H_
