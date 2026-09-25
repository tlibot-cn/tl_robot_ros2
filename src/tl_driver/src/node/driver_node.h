#ifndef TL_DRIVER__DRIVER_NODE_H_
#define TL_DRIVER__DRIVER_NODE_H_

// common
#include <chrono>
#include <memory>
#include <string>
#include <vector>

// ROS2
#include <rmw/qos_profiles.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// msg
#include "tl_ros2_interface/msg/move_command.hpp"
#include "tl_ros2_interface/msg/servol_move.hpp"

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
    return session_;
  }

  /// @brief 按节点参数（arm_ip/arm_port/arm_port_aux）连接控制器。
  /// @return 两个端口均连接成功为 true
  bool connect();

  /// @brief 连接控制器、切入示教模式并上电（构造函数内调用）。
  void init();

  /// @brief 关节数（由 arm_joints 长度决定）。
  /// @return 关节数
  int ndof() const noexcept
  {
    return ndof_;
  }

  /// @brief 注册服务并托管其生命周期，供各域 register_*_services 使用。
  /// @note SrvT 必须是 rosidl 生成的服务类型（需具备 Request/Response 成员类型）。
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
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;

  ArmSession session_;
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
