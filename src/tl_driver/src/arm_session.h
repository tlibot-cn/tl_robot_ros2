#ifndef TL_DRIVER__ARM_SESSION_H_
#define TL_DRIVER__ARM_SESSION_H_

/// @file arm_session.h
/// @brief 机械臂会话：6001/7000 双端口连接、上下电。
/// @note 拆分阶段为纯搬移，会话状态语义与拆分前的成员实现一致，未引入同步原语与状态机
///       （并发缺陷与掉线自愈见架构评审 P0-2 / P0-4，另行修复）。
/// @note 7000 口状态帧异步通道（缓冲、回调、序号与等待）由 arm_client 承担。

#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace tl_driver
{
/// @brief 与控制器之间的会话（连接、上下电与状态帧）。
class ArmSession
{
public:
  /// @brief 构造会话，不发起连接。
  /// @param[in] logger 日志器
  explicit ArmSession(rclcpp::Logger logger);

  /// @brief 连接 6001 与 7000 两个端口，并注册错误/告警与状态回调。
  /// @param[in] ip 控制器 IP
  /// @param[in] port 6001 端口（SDK 以字符串接收）
  /// @param[in] port_aux 7000 端口（SDK 以字符串接收）
  /// @return 两个端口均连接成功为 true
  bool connect(const std::string& ip, const std::string& port, const std::string& port_aux);

  /// @brief 断开两个端口并清零 fd。
  /// @return 未连接或断开完成为 true
  bool disconnect();

  /// @brief 上电：按当前伺服状态切 READY 后 poweron。
  /// @return 上电后伺服状态为 RUNNING 时为 true
  bool power_on();

  /// @brief 下电。
  /// @return 已处于下电状态或下电成功为 true
  bool power_off();

  /// @brief 两个端口均已连接。
  bool is_connected() const;

  /// @brief 已上电。
  bool is_powered() const;

  /// @brief 6001 端口 fd：请求/响应式 SDK 调用。
  int fd() const;

  /// @brief 7000 端口 fd：servoJ 全系列与机器人状态异步推送。
  int fd_aux() const;

private:
  rclcpp::Logger logger_;

  int socket_fd_{0};
  int socket_fd_aux_{0};
  bool is_connected_{false}; // 机械臂是否连接
  bool is_powered_{false};   // 机械臂是否上电(示教模式)
};
} // namespace tl_driver

#endif // TL_DRIVER__ARM_SESSION_H_
