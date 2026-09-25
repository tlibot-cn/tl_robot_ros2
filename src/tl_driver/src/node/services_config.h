#ifndef TL_DRIVER__SERVICES_CONFIG_H_
#define TL_DRIVER__SERVICES_CONFIG_H_

/// @file services_config.h
/// @brief 配置类服务：DH 参数/关节参数/工具/坐标系/全局点位/可达性。

#include <memory>

#include <rclcpp/callback_group.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tl_ros2_interface/srv/get_coord_num.hpp>
#include <tl_ros2_interface/srv/get_dh_param.hpp>
#include <tl_ros2_interface/srv/get_global_pos.hpp>
#include <tl_ros2_interface/srv/get_pos_reachable.hpp>
#include <tl_ros2_interface/srv/get_robot_joint_param.hpp>
#include <tl_ros2_interface/srv/restore_default_dh_param.hpp>
#include <tl_ros2_interface/srv/set_axis_zero_pos.hpp>
#include <tl_ros2_interface/srv/set_coord_num.hpp>
#include <tl_ros2_interface/srv/set_dh_param.hpp>
#include <tl_ros2_interface/srv/set_drag_mode.hpp>
#include <tl_ros2_interface/srv/set_global_pos.hpp>
#include <tl_ros2_interface/srv/set_robot_joint_param.hpp>
#include <tl_ros2_interface/srv/set_tool_param.hpp>
#include <tl_ros2_interface/srv/set_user_coord.hpp>

namespace tl_driver
{
class DriverNode;

namespace services
{
/// @brief 处理 /tl_driver/get_robot_joint_param 服务。
void handle_get_robot_joint_param_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::GetRobotJointParam::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetRobotJointParam::Response> response);

/// @brief 处理 /tl_driver/set_robot_joint_param 服务。
void handle_set_robot_joint_param_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::SetRobotJointParam::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetRobotJointParam::Response> response);

/// @brief 处理 /tl_driver/restore_default_dh_param 服务。
void handle_restore_default_dh_param_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::RestoreDefaultDHParam::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::RestoreDefaultDHParam::Response> response);

/// @brief 处理 /tl_driver/set_default_cartesian_param 服务。
void handle_set_default_cartesian_param_service(DriverNode& node,
                                                const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                std::shared_ptr<std_srvs::srv::Trigger::Response> response);

/// @brief 处理 /tl_driver/set_drag_mode 服务。
void handle_set_drag_mode_service(DriverNode& node,
                                  const std::shared_ptr<tl_ros2_interface::srv::SetDragMode::Request> request,
                                  std::shared_ptr<tl_ros2_interface::srv::SetDragMode::Response> response);

/// @brief 处理 /tl_driver/set_tool_param 服务。
void handle_set_tool_param_service(DriverNode& node,
                                   const std::shared_ptr<tl_ros2_interface::srv::SetToolParam::Request> request,
                                   std::shared_ptr<tl_ros2_interface::srv::SetToolParam::Response> response);

/// @brief 处理 /tl_driver/set_user_coord 服务。
void handle_set_user_coord_service(DriverNode& node,
                                   const std::shared_ptr<tl_ros2_interface::srv::SetUserCoord::Request> request,
                                   std::shared_ptr<tl_ros2_interface::srv::SetUserCoord::Response> response);

/// @brief 处理 /tl_driver/set_axis_zero_pos 服务。
void handle_set_axis_zero_pos_service(DriverNode& node,
                                      const std::shared_ptr<tl_ros2_interface::srv::SetAxisZeroPos::Request> request,
                                      std::shared_ptr<tl_ros2_interface::srv::SetAxisZeroPos::Response> response);

/// @brief 处理 /tl_driver/set_coord_num 服务。
void handle_set_coord_num_service(DriverNode& node,
                                  const std::shared_ptr<tl_ros2_interface::srv::SetCoordNum::Request> request,
                                  std::shared_ptr<tl_ros2_interface::srv::SetCoordNum::Response> response);

/// @brief 处理 /tl_driver/get_coord_num 服务。
void handle_get_coord_num_service(DriverNode& node,
                                  const std::shared_ptr<tl_ros2_interface::srv::GetCoordNum::Request> request,
                                  std::shared_ptr<tl_ros2_interface::srv::GetCoordNum::Response> response);

/// @brief 处理 /tl_driver/get_pos_reachable 服务。
void handle_get_pos_reachable_service(DriverNode& node,
                                      const std::shared_ptr<tl_ros2_interface::srv::GetPosReachable::Request> request,
                                      std::shared_ptr<tl_ros2_interface::srv::GetPosReachable::Response> response);

/// @brief 处理 /tl_driver/set_dh_param 服务。
void handle_set_dh_param_service(DriverNode& node,
                                 const std::shared_ptr<tl_ros2_interface::srv::SetDHParam::Request> request,
                                 std::shared_ptr<tl_ros2_interface::srv::SetDHParam::Response> response);

/// @brief 处理 /tl_driver/get_dh_param 服务。
void handle_get_dh_param_service(DriverNode& node,
                                 const std::shared_ptr<tl_ros2_interface::srv::GetDHParam::Request> request,
                                 std::shared_ptr<tl_ros2_interface::srv::GetDHParam::Response> response);

/// @brief 处理 /tl_driver/set_global_pos 服务。
void handle_set_global_pos_service(DriverNode& node,
                                   const std::shared_ptr<tl_ros2_interface::srv::SetGlobalPos::Request> request,
                                   std::shared_ptr<tl_ros2_interface::srv::SetGlobalPos::Response> response);

/// @brief 处理 /tl_driver/get_global_pos 服务。
void handle_get_global_pos_service(DriverNode& node,
                                   const std::shared_ptr<tl_ros2_interface::srv::GetGlobalPos::Request> request,
                                   std::shared_ptr<tl_ros2_interface::srv::GetGlobalPos::Response> response);

/// @brief 注册本域服务。
/// @param[in] node 节点
/// @param[in] group 回调组
void register_config_services(DriverNode& node, rclcpp::CallbackGroup::SharedPtr group);
} // namespace services
} // namespace tl_driver

#endif // TL_DRIVER__SERVICES_CONFIG_H_
