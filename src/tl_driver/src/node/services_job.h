#ifndef TL_DRIVER__SERVICES_JOB_H_
#define TL_DRIVER__SERVICES_JOB_H_

/// @file services_job.h
/// @brief 作业类服务：作业文件列表、运行/删除、插入运动指令。

#include <memory>

#include <rclcpp/callback_group.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tl_ros2_interface/srv/get_all_job_file_name.hpp>
#include <tl_ros2_interface/srv/job_insert_move.hpp>
#include <tl_ros2_interface/srv/job_run.hpp>

namespace tl_driver
{
class DriverNode;

namespace services
{
/// @brief 处理 /tl_driver/get_all_job_filename 服务。
void handle_get_all_job_filename_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::GetAllJobFileName::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetAllJobFileName::Response> response);

/// @brief 处理 /tl_driver/job_run 服务。
void handle_job_run_service(DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::JobRun::Request> request,
                            std::shared_ptr<tl_ros2_interface::srv::JobRun::Response> response);

/// @brief 处理 /tl_driver/job_delete 服务。
void handle_job_delete_service(DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::JobRun::Request> request,
                               std::shared_ptr<tl_ros2_interface::srv::JobRun::Response> response);

/// @brief 处理 /tl_driver/job_insert_moveJ 服务。
void handle_job_insert_movej_service(DriverNode& node,
                                     const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
                                     std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response);

/// @brief 处理 /tl_driver/job_insert_moveL 服务。
void handle_job_insert_movel_service(DriverNode& node,
                                     const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
                                     std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response);

/// @brief 处理 /tl_driver/job_insert_imove 服务。
void handle_job_insert_imove_service(DriverNode& node,
                                     const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
                                     std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response);

/// @brief 处理 /tl_driver/job_insert_moveC 服务。
void handle_job_insert_movec_service(DriverNode& node,
                                     const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
                                     std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response);

/// @brief 注册本域服务。
/// @param[in] node 节点
/// @param[in] group 回调组
void register_job_services(DriverNode& node, rclcpp::CallbackGroup::SharedPtr group);
} // namespace services
} // namespace tl_driver

#endif // TL_DRIVER__SERVICES_JOB_H_
