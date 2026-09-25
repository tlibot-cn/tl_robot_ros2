#ifndef TL_DRIVER__DRIVER_NODE_H_
#define TL_DRIVER__DRIVER_NODE_H_

// common
#include <algorithm>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <cstdint>

// ROS2
#include <rmw/qos_profiles.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// msg
#include "tl_ros2_interface/msg/arm_status.hpp"
#include "tl_ros2_interface/msg/move_command.hpp"
#include "tl_ros2_interface/msg/cartesian_pose.hpp"
#include "tl_ros2_interface/msg/tool_param.hpp"
#include "tl_ros2_interface/msg/modbus_tcp_param.hpp"
#include "tl_ros2_interface/msg/modbus_rtu_param.hpp"
#include "tl_ros2_interface/msg/modbus_master_param.hpp"
#include "tl_ros2_interface/msg/robot_dh_param.hpp"
#include "tl_ros2_interface/msg/job_file_name.hpp"
#include "tl_ros2_interface/msg/robot_joint_param.hpp"
#include "tl_ros2_interface/msg/servol_move.hpp"

// srv
#include "tl_ros2_interface/srv/get_robot_state.hpp"
#include "tl_ros2_interface/srv/get_robot_joint_param.hpp"
#include "tl_ros2_interface/srv/set_robot_joint_param.hpp"
#include "tl_ros2_interface/srv/get_joint_temperature.hpp"
#include "tl_ros2_interface/srv/get_joint_voltage.hpp"
#include "tl_ros2_interface/srv/get_motor_current.hpp"
#include "tl_ros2_interface/srv/get_joint_software_version.hpp"
#include "tl_ros2_interface/srv/restore_default_dh_param.hpp"
#include "tl_ros2_interface/srv/log_download.hpp"
#include "tl_ros2_interface/srv/set_speed.hpp"
#include "tl_ros2_interface/srv/get_speed.hpp"
#include "tl_ros2_interface/srv/get_pos_transform.hpp"
#include "tl_ros2_interface/srv/set_controller_ip.hpp"
#include "tl_ros2_interface/srv/jogging.hpp"
#include "tl_ros2_interface/srv/set_drag_mode.hpp"
#include "tl_ros2_interface/srv/track_save.hpp"
#include "tl_ros2_interface/srv/track_playback.hpp"
#include "tl_ros2_interface/srv/set_tool_param.hpp"
#include "tl_ros2_interface/srv/set_user_coord.hpp"
#include "tl_ros2_interface/srv/set_axis_zero_pos.hpp"
#include "tl_ros2_interface/srv/set_current_coord.hpp"
#include "tl_ros2_interface/srv/get_current_coord.hpp"
#include "tl_ros2_interface/srv/set_coord_num.hpp"
#include "tl_ros2_interface/srv/get_coord_num.hpp"
#include "tl_ros2_interface/srv/set_digital_output.hpp"
#include "tl_ros2_interface/srv/get_digital_input_output.hpp"
#include "tl_ros2_interface/srv/modbus_write.hpp"
#include "tl_ros2_interface/srv/modbus_read.hpp"
#include "tl_ros2_interface/srv/coord_transform.hpp"
#include "tl_ros2_interface/srv/get_pos_reachable.hpp"
#include "tl_ros2_interface/srv/set_dh_param.hpp"
#include "tl_ros2_interface/srv/get_dh_param.hpp"
#include "tl_ros2_interface/srv/get_all_job_file_name.hpp"
#include "tl_ros2_interface/srv/job_run.hpp"
#include "tl_ros2_interface/srv/job_insert_move.hpp"
#include "tl_ros2_interface/srv/set_global_pos.hpp"
#include "tl_ros2_interface/srv/get_global_pos.hpp"
#include "tl_ros2_interface/srv/open_servo_j.hpp"
#include "tl_ros2_interface/srv/set_current_mode.hpp"
#include "tl_ros2_interface/srv/get_current_mode.hpp"
#include "tl_ros2_interface/srv/queue_motion_set_status.hpp"
#include "tl_ros2_interface/srv/queue_motion_move_j.hpp"
#include "tl_ros2_interface/srv/get_current_motor_torque.hpp"
#include "tl_ros2_interface/srv/get_current_line_joint_speed.hpp"

#include "arm_session.h"
#include "node/publishers.h"

namespace tl_driver
{
/// @brief tl_driver 节点：参数解析、会话、订阅、服务注册与状态发布调度。
class DriverNode : public rclcpp::Node
{
public:
  DriverNode();
  ~DriverNode();

  /// @brief 会话访问，供各域 handler 使用。
  /// @return 会话对象引用
  ArmSession& session() noexcept
  {
    return *session_;
  }

  /// @brief 发布器访问。
  /// @return 发布器对象引用
  Publishers& publishers() noexcept
  {
    return *publishers_;
  }

  /// @brief 控制器 IP 参数。
  /// @return 参数值
  const std::string& arm_ip() const noexcept
  {
    return arm_ip_;
  }

  /// @brief 6001 端口参数。
  /// @return 参数值
  const std::string& arm_port() const noexcept
  {
    return arm_port_;
  }

  /// @brief 7000 端口参数。
  /// @return 参数值
  const std::string& arm_port_aux() const noexcept
  {
    return arm_port_aux_;
  }

  /// @brief 臂型标识参数（仅作标识，不参与行为）。
  /// @return 参数值
  const std::string& arm_type() const noexcept
  {
    return arm_type_;
  }

  /// @brief 关节名列表参数。
  /// @return 参数值
  const std::vector<std::string>& arm_joints() const noexcept
  {
    return arm_joints_;
  }

  /// @brief 关节数（由 arm_joints 长度决定）。
  /// @return 关节数
  int ndof() const noexcept
  {
    return ndof_;
  }

  /// @brief 注册服务并托管其生命周期，供各域 register_*_services 使用。
  /// @tparam SrvT 服务类型，如 tl_ros2_interface::srv::SetSpeed
  /// @param[in] name 服务名
  /// @param[in] handler 该域的处理函数
  /// @param[in] group 回调组
  template <typename SrvT>
  void register_service(const std::string& name,
                        void (*handler)(DriverNode&, typename SrvT::Request::SharedPtr,
                                        typename SrvT::Response::SharedPtr),
                        rclcpp::CallbackGroup::SharedPtr group)
  {
    auto service = this->create_service<SrvT>(
        name,
        [this, handler](typename SrvT::Request::SharedPtr request, typename SrvT::Response::SharedPtr response)
        {
          handler(*this, request, response);
        },
        rmw_qos_profile_services_default, group);
    services_.push_back(service);
  }

  // 服务

private:
  /// @brief 连接控制器、切入示教模式并上电（构造函数内调用）。
  void init();

  std::vector<rclcpp::ServiceBase::SharedPtr> services_;

  std::unique_ptr<tl_driver::ArmSession> session_;
  std::unique_ptr<tl_driver::Publishers> publishers_;

  std::string arm_ip_;
  std::string arm_port_;
  std::string arm_port_aux_;
  std::string arm_type_;

  std::vector<std::string> arm_joints_;
  double publish_rate_{100.0};
  int ndof_{6};

  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr topic_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;

  rclcpp::Subscription<tl_ros2_interface::msg::MoveCommand>::SharedPtr movej_sub_;
  rclcpp::Subscription<tl_ros2_interface::msg::MoveCommand>::SharedPtr movel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr set_servoj_pos_sub_;
  rclcpp::Subscription<tl_ros2_interface::msg::ServolMove>::SharedPtr set_servol_pos_sub_;

  rclcpp::TimerBase::SharedPtr state_publish_timer_;
};

} // namespace tl_driver

#endif // TL_DRIVER__DRIVER_NODE_H_
