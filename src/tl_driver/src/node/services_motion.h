#ifndef TL_DRIVER__SERVICES_MOTION_H_
#define TL_DRIVER__SERVICES_MOTION_H_

/// @file services_motion.h
/// @brief 运动类服务与话题：速度/点动/轨迹/队列/ServoJ·ServoL 与 moveJ·moveL。

#include <memory>

#include <rclcpp/callback_group.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tl_ros2_interface/srv/get_speed.hpp>
#include <tl_ros2_interface/srv/jogging.hpp>
#include <tl_ros2_interface/srv/open_servo_j.hpp>
#include <tl_ros2_interface/srv/queue_motion_move_j.hpp>
#include <tl_ros2_interface/srv/queue_motion_set_status.hpp>
#include <tl_ros2_interface/srv/set_speed.hpp>
#include <tl_ros2_interface/srv/track_playback.hpp>
#include <tl_ros2_interface/srv/track_save.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tl_ros2_interface/msg/move_command.hpp>
#include <tl_ros2_interface/msg/servol_move.hpp>

namespace tl_driver
{
class DriverNode;

namespace services
{
/// @brief 处理 /tl_driver/set_speed 服务。
void handle_set_speed_service(DriverNode& node,
                              const std::shared_ptr<tl_ros2_interface::srv::SetSpeed::Request> request,
                              std::shared_ptr<tl_ros2_interface::srv::SetSpeed::Response> response);

/// @brief 处理 /tl_driver/get_speed 服务。
void handle_get_speed_service(DriverNode& node,
                              const std::shared_ptr<tl_ros2_interface::srv::GetSpeed::Request> request,
                              std::shared_ptr<tl_ros2_interface::srv::GetSpeed::Response> response);

/// @brief 处理 /tl_driver/start_jogging 服务。
void handle_start_jogging_service(DriverNode& node,
                                  const std::shared_ptr<tl_ros2_interface::srv::Jogging::Request> request,
                                  std::shared_ptr<tl_ros2_interface::srv::Jogging::Response> response);

/// @brief 处理 /tl_driver/stop_jogging 服务。
void handle_stop_jogging_service(DriverNode& node,
                                 const std::shared_ptr<tl_ros2_interface::srv::Jogging::Request> request,
                                 std::shared_ptr<tl_ros2_interface::srv::Jogging::Response> response);

/// @brief 处理 /tl_driver/track_save 服务。
void handle_track_save_service(DriverNode& node,
                               const std::shared_ptr<tl_ros2_interface::srv::TrackSave::Request> request,
                               std::shared_ptr<tl_ros2_interface::srv::TrackSave::Response> response);

/// @brief 处理 /tl_driver/track_playback 服务。
void handle_track_playback_service(DriverNode& node,
                                   const std::shared_ptr<tl_ros2_interface::srv::TrackPlayback::Request> request,
                                   std::shared_ptr<tl_ros2_interface::srv::TrackPlayback::Response> response);

/// @brief 处理 /tl_driver/open_servoj 服务。
void handle_open_servoj_service(DriverNode& node,
                                const std::shared_ptr<tl_ros2_interface::srv::OpenServoJ::Request> request,
                                std::shared_ptr<tl_ros2_interface::srv::OpenServoJ::Response> response);

/// @brief 处理 /tl_driver/close_servoj 服务。
void handle_close_servoj_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response);

/// @brief 处理 /tl_driver/queue_motion_set_status 服务。
void handle_queue_motion_set_status_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::QueueMotionSetStatus::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::QueueMotionSetStatus::Response> response);

/// @brief 处理 /tl_driver/queue_motion_movej 服务。
void handle_queue_motion_movej_service(DriverNode& node,
                                       const std::shared_ptr<tl_ros2_interface::srv::QueueMotionMoveJ::Request> request,
                                       std::shared_ptr<tl_ros2_interface::srv::QueueMotionMoveJ::Response> response);

/// @brief 处理 /tl_driver/queue_motion_stop 服务。
void handle_queue_motion_stop_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

/// @brief 注册本域服务。
/// @param[in] node 节点
/// @param[in] group 回调组
void register_motion_services(DriverNode& node, rclcpp::CallbackGroup::SharedPtr group);
} // namespace services
} // namespace tl_driver

#endif // TL_DRIVER__SERVICES_MOTION_H_
