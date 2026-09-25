#ifndef TL_DRIVER__SERVICES_TRANSFORM_H_
#define TL_DRIVER__SERVICES_TRANSFORM_H_

/// @file services_transform.h
/// @brief 位姿换算与坐标系服务：rpy↔四元数、r↔tr、坐标系变换、当前坐标系读写。

#include <memory>

#include <rclcpp/callback_group.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tl_ros2_interface/srv/coord_transform.hpp>
#include <tl_ros2_interface/srv/get_current_coord.hpp>
#include <tl_ros2_interface/srv/get_pos_transform.hpp>
#include <tl_ros2_interface/srv/set_current_coord.hpp>

namespace tl_driver
{
class DriverNode;

namespace services
{
/// @brief 处理 /tl_driver/get_quat2rpy 服务。
void handle_get_quat2rpy_service(DriverNode& node,
                                 const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
                                 std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response);

/// @brief 处理 /tl_driver/get_rpy2quat 服务。
void handle_get_rpy2quat_service(DriverNode& node,
                                 const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
                                 std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response);

/// @brief 处理 /tl_driver/get_rpy2r 服务。
void handle_get_rpy2r_service(DriverNode& node,
                              const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
                              std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response);

/// @brief 处理 /tl_driver/get_tr2r 服务。
void handle_get_tr2r_service(DriverNode& node,
                             const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
                             std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response);

/// @brief 处理 /tl_driver/get_r2tr 服务。
void handle_get_r2tr_service(DriverNode& node,
                             const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
                             std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response);

/// @brief 处理 /tl_driver/set_current_coord 服务。
void handle_set_current_coord_service(DriverNode& node,
                                      const std::shared_ptr<tl_ros2_interface::srv::SetCurrentCoord::Request> request,
                                      std::shared_ptr<tl_ros2_interface::srv::SetCurrentCoord::Response> response);

/// @brief 处理 /tl_driver/get_current_coord 服务。
void handle_get_current_coord_service(DriverNode& node,
                                      const std::shared_ptr<tl_ros2_interface::srv::GetCurrentCoord::Request> request,
                                      std::shared_ptr<tl_ros2_interface::srv::GetCurrentCoord::Response> response);

/// @brief 处理 /tl_driver/coord_transform 服务。
void handle_coord_transform_service(DriverNode& node,
                                    const std::shared_ptr<tl_ros2_interface::srv::CoordTransform::Request> request,
                                    std::shared_ptr<tl_ros2_interface::srv::CoordTransform::Response> response);

/// @brief 注册本域服务。
/// @param[in] node 节点
/// @param[in] group 回调组
void register_transform_services(DriverNode& node, rclcpp::CallbackGroup::SharedPtr group);
} // namespace services
} // namespace tl_driver

#endif // TL_DRIVER__SERVICES_TRANSFORM_H_
