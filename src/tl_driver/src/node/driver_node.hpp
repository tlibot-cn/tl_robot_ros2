#ifndef TL_DRIVER__DRIVER_NODE_HPP_
#define TL_DRIVER__DRIVER_NODE_HPP_

// common
#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// ROS2
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rmw/qos_profiles.h>
#include <rclcpp/rclcpp.hpp>

#include "arm_session.hpp"
#include "node/publishers.hpp"

namespace tl_driver
{
/// @brief 机械臂驱动节点：参数、回调组、实体创建与服务/话题注册。
/// @note 回调实现按域分布：服务在 src/node/services_*.cpp，话题在 src/node/topics.cpp，
///       由 node/registrars.hpp 声明的注册函数挂载；对外 ROS 接口与拆分前逐字一致。
class DriverNode : public rclcpp::Node
{
public:
  DriverNode();
  ~DriverNode();

  /// @brief 连接控制器并完成上电流程，由构造函数末尾调用。
  /// @note 连接或切入示教模式失败即以 `EXIT_FAILURE` 退出，便于 systemd/launch 识别。
  void init();

  /// @brief 会话访问，供各域 handler 使用。
  /// @return 会话对象引用
  ArmSession& session() noexcept
  {
    return session_;
  }

  /// @brief 关节数，由 arm_joints 参数长度决定。
  /// @return 关节数
  int ndof() const noexcept
  {
    return ndof_;
  }

  /// @brief 创建服务并持有其生命周期，供各域 register_*_services 使用。
  /// @note 包装 `rclcpp::Node::create_service`（固定 `rmw_qos_profile_services_default` 与服务回调组），
  ///       并把服务存入 `services_` 保活——rclcpp Humble 只弱引用已创建的服务，不持有即注销。
  /// @note 与下方 `create_subscription` 一并隐藏 `rclcpp::Node` 的同名重载（名称隐藏）。
  /// @tparam ServiceT 服务类型，如 tl_ros2_interface::srv::SetSpeed
  /// @tparam CallbackT 回调类型
  /// @param[in] name 服务名（含 `/tl_driver/` 前缀）
  /// @param[in] callback 该域的服务回调
  /// @return 服务句柄
  template <typename ServiceT, typename CallbackT>
  typename rclcpp::Service<ServiceT>::SharedPtr create_service(const std::string& name, CallbackT&& callback)
  {
    // 异常绝不允许越过回调边界：rclcpp 不捕获服务回调异常，逃逸会终止整个进程。
    using GuardedCallback =
        std::function<void(std::shared_ptr<typename ServiceT::Request>, std::shared_ptr<typename ServiceT::Response>)>;
    GuardedCallback guarded =
        [logger = this->get_logger(), service_name = name, cb = std::forward<CallbackT>(callback)](
            std::shared_ptr<typename ServiceT::Request> request,
            std::shared_ptr<typename ServiceT::Response> response) mutable
    {
      try
      {
        cb(request, response);
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(logger, "[Service %s]: callback threw exception: %s", service_name.c_str(), e.what());
        response->success = false;
        response->message = std::string("Internal error: ") + e.what();
      }
      catch (...)
      {
        RCLCPP_ERROR(logger, "[Service %s]: callback threw unknown exception", service_name.c_str());
        response->success = false;
        response->message = "Internal error: unknown exception";
      }
    };

    // 注册期必须先经 select_service_group() 选定域组；注册结束后该指针清空。
    if (!service_group_)
    {
      throw std::logic_error("service callback group not selected for " + name);
    }
    auto service = rclcpp::Node::create_service<ServiceT>(name, std::move(guarded), rmw_qos_profile_services_default,
                                                          service_group_);
    services_.push_back(service);
    return service;
  }

  /// @brief 创建订阅并持有其生命周期（回调组为话题组）。
  /// @note 同 create_service：存入 `subscriptions_` 保活；订阅选项取自 `topic_options_`，
  ///       该成员须在 create_callback_groups() 中绑定话题回调组。
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
  /// @brief 声明并读取节点参数。
  void declare_parameters();

  /// @brief 创建服务域/话题/定时器互斥回调组。
  /// @note 服务按域各建一个 `MutuallyExclusive` 组，跨域可并行；`get_robot_state` 因等待
  ///       状态帧（mode=1 最长 interval+1 s）单独成组，避免冻结其它服务（架构评审 P1-1）。
  void create_callback_groups();

  /// @brief 设置后续 `create_service` 注册到哪个服务回调组（仅构造期使用）。
  /// @param[in] group 服务回调组
  void select_service_group(const rclcpp::CallbackGroup::SharedPtr& group);

  /// @brief 按域注册全部服务与话题回调。
  void register_entities();

  /// @brief 参数更新回调：校验并应用 `publish_rate`（重建定时器，立即生效）。
  rcl_interfaces::msg::SetParametersResult on_parameters_set(const std::vector<rclcpp::Parameter>& parameters);

  /// @brief 按指定频率创建状态发布定时器；成功后替换旧定时器。
  /// @param[in] rate 发布频率（Hz）
  void create_state_timer(double rate);

  std::string arm_ip_;
  std::string arm_port_;
  std::string arm_port_aux_;
  std::string arm_type_;

  std::vector<std::string> arm_joints_;
  double publish_rate_{100.0};
  int ndof_{6};

  ArmSession session_;

  std::unique_ptr<tl_driver::StatePublishers> publishers_;

  // 构造期当前注册目标；注册每个域前由 select_service_group() 切换。
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr session_service_group_;
  rclcpp::CallbackGroup::SharedPtr motion_service_group_;
  rclcpp::CallbackGroup::SharedPtr transform_service_group_;
  rclcpp::CallbackGroup::SharedPtr config_service_group_;
  rclcpp::CallbackGroup::SharedPtr io_service_group_;
  rclcpp::CallbackGroup::SharedPtr state_service_group_;
  rclcpp::CallbackGroup::SharedPtr robot_state_service_group_;

  rclcpp::CallbackGroup::SharedPtr topic_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::SubscriptionOptions topic_options_;

  // 节点需持有服务/订阅对象，否则对象析构即注销
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

  rclcpp::TimerBase::SharedPtr state_publish_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr publish_rate_param_handle_;
};

} // namespace tl_driver

#endif // TL_DRIVER__DRIVER_NODE_HPP_
