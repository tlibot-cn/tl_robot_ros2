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

  /// @brief 创建订阅并持有其生命周期（回调组为话题组）。
  /// @note 存入 `subscriptions_` 保活——rclcpp Humble 只弱引用已创建的订阅，不持有即注销；
  ///       订阅选项取自 `topic_options_`（构造期已绑定话题回调组）。
  /// @tparam MessageT 消息类型
  /// @tparam CallbackT 回调类型
  /// @param[in] name 话题名（含 `/tl_driver/` 前缀）
  /// @param[in] qos 队列深度
  /// @param[in] callback 话题回调
  /// @return 订阅句柄
  template <typename MessageT, typename CallbackT>
  typename rclcpp::Subscription<MessageT>::SharedPtr create_subscription(const std::string& name, const size_t qos,
                                                                         CallbackT&& callback)
  {
    auto subscription =
        rclcpp::Node::create_subscription<MessageT>(name, qos, std::forward<CallbackT>(callback), topic_options_);
    subscriptions_.push_back(subscription);
    return subscription;
  }

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
  rclcpp::SubscriptionOptions topic_options_;

  // 节点需持有服务/订阅对象，否则对象析构即注销
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;


  rclcpp::TimerBase::SharedPtr state_publish_timer_;
};

} // namespace tl_driver

#endif // TL_DRIVER__DRIVER_NODE_H_
