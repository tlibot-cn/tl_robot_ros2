#ifndef TL_DRIVER__DRIVER_NODE_HPP_
#define TL_DRIVER__DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "arm_session.hpp"
#include "node/publishers.hpp"
#include "tl_ros2_interface/msg/move_command.hpp"
#include "tl_ros2_interface/msg/servol_move.hpp"

namespace tl_driver
{

/**
 * @brief 机械臂驱动节点：参数、回调组、实体创建与服务/话题注册。
 * @note 回调实现按域分布：服务在 src/node/services_*.cpp，话题在 src/node/topics.cpp，
 *       由 node/registrars.hpp 声明的注册函数挂载；对外 ROS 接口与拆分前逐字一致。
 */
class DriverNode : public rclcpp::Node
{
public:
  DriverNode();
  ~DriverNode() override;

  /**
   * @brief 连接控制器并完成上电流程，由构造函数末尾调用。
   * @note 连接失败即 `exit(0)`（与拆分前一致；退出码缺陷见架构评审 3.3）。
   */
  void init();

  /**
   * @brief 按节点参数（arm_ip/arm_port/arm_port_aux）连接控制器。
   * @return true=两个端口均连接成功
   */
  bool connect();

  /**
   * @brief 会话：fd、连接状态与上下电状态。
   */
  ArmSession& session();

  /**
   * @brief 关节数，由 arm_joints 参数长度决定。
   */
  int ndof() const;

  /**
   * @brief 关节名列表（arm_joints 参数）。
   */
  const std::vector<std::string>& joint_names() const;

  /**
   * @brief 服务回调组，65 个服务共用（MutuallyExclusive，串行）。
   */
  rclcpp::CallbackGroup::SharedPtr service_group() const;

  /**
   * @brief 创建服务并持有其生命周期。
   * @note 包装 `rclcpp::Node::create_service`（固定 `rmw_qos_profile_services_default` 与服务回调组），
   *       并把服务存入 `services_` 保活——rclcpp Humble 只弱引用已创建的服务，不持有即注销。
   * @tparam ServiceT 服务类型
   * @tparam CallbackT 回调类型
   * @param[in] name 服务名（含 `/tl_driver/` 前缀）
   * @param[in] callback 服务回调
   * @return 服务句柄
   */
  template <typename ServiceT, typename CallbackT>
  typename rclcpp::Service<ServiceT>::SharedPtr create_service(const std::string& name, CallbackT&& callback)
  {
    auto service = rclcpp::Node::create_service<ServiceT>(name, std::forward<CallbackT>(callback),
                                                          rmw_qos_profile_services_default, service_group_);
    services_.push_back(service);
    return service;
  }

  /**
   * @brief 创建订阅并持有其生命周期（回调组为话题组）。
   * @note 同 create_service：存入 `subscriptions_` 保活；订阅选项取自 `topic_options_`，
   *       该成员须在 create_callback_groups() 中初始化。
   * @tparam MessageT 消息类型
   * @tparam CallbackT 回调类型
   * @param[in] name 话题名（含 `/tl_driver/` 前缀）
   * @param[in] qos 队列深度
   * @param[in] callback 话题回调
   * @return 订阅句柄
   */
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
  /**
   * @brief 声明并读取节点参数。
   */
  void declare_parameters();

  /**
   * @brief 创建服务/话题/定时器三个互斥回调组。
   */
  void create_callback_groups();

  /**
   * @brief 按域注册全部服务与话题回调。
   */
  void register_entities();

  /**
   * @brief 创建 100 Hz 状态发布定时器。
   */
  void create_state_timer();

  std::string arm_ip_;
  std::string arm_port_;
  std::string arm_port_aux_;
  std::string arm_type_;

  std::vector<std::string> arm_joints_;
  double publish_rate_{100.0};
  int ndof_{6};

  ArmSession session_;

  std::unique_ptr<StatePublishers> publishers_;

  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr topic_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::SubscriptionOptions topic_options_;

  // 节点需持有服务/订阅对象，否则对象析构即注销
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

  rclcpp::TimerBase::SharedPtr state_publish_timer_;
};

} // namespace tl_driver

#endif // TL_DRIVER__DRIVER_NODE_HPP_
