#ifndef TL_DRIVER__ARM_SESSION_HPP_
#define TL_DRIVER__ARM_SESSION_HPP_

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace tl_driver
{

/**
 * @brief 机械臂会话状态：fd 所有权、连接建立/断开、上电/下电。
 * @note 本类目前是拆分前的直通实现（普通成员 + 无锁），行为与拆分前一致；原子化、租约与重连看门狗
 *       见架构评审 3.2/3.4，随"会话安全与自愈"阶段实施。
 */
class ArmSession
{
public:
  /**
   * @brief 构造会话。
   * @param[in] logger 日志器，通常取节点 logger
   */
  explicit ArmSession(rclcpp::Logger logger);

  /**
   * @brief 建立与控制器的主/辅端口连接，并注册错误与状态推送回调。
   * @param[in] ip 控制器 IP
   * @param[in] port 主端口（6001，请求/响应式 SDK 调用）
   * @param[in] port_aux 辅助端口（7000，servoJ 全系列与状态推送）
   * @return true=两个端口均连接成功
   */
  bool connect(const std::string& ip, const std::string& port, const std::string& port_aux);

  /**
   * @brief 断开两个端口并清零 fd。
   * @return 始终为 true（重复调用安全）
   */
  bool disconnect();

  /**
   * @brief 上电（含报警态先清错）。
   * @return true=伺服状态回到运行态
   */
  bool power_on();

  /**
   * @brief 下电。
   * @return true=已下电（含本来即未上电）
   */
  bool power_off();

  /**
   * @brief 会话是否可用。
   * @return true=已标记连接且两个 fd 均有效
   */
  bool is_connected() const;

  /**
   * @brief 是否处于上电（示教）状态。
   */
  bool is_powered() const;

  /**
   * @brief 主端口 fd 快照。
   * @return fd，未连接时为 0
   */
  int fd() const;

  /**
   * @brief 辅助端口 fd 快照。
   * @return fd，未连接时为 0
   */
  int fd_aux() const;

private:
  rclcpp::Logger logger_;

  int socket_fd_{0};
  int socket_fd_aux_{0};
  bool is_connected_{false}; // 机械臂是否连接
  bool is_powered_{false};   // 机械臂是否上电(示教模式)
};

} // namespace tl_driver

#endif // TL_DRIVER__ARM_SESSION_HPP_
