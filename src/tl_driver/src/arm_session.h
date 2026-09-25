#ifndef TL_DRIVER__ARM_SESSION_H_
#define TL_DRIVER__ARM_SESSION_H_

/// @file arm_session.h
/// @brief 机械臂会话：6001/7000 双端口连接、上下电、7000 口状态帧缓冲。
/// @note 拆分阶段为纯搬移，会话状态语义与拆分前的成员实现一致，未引入同步原语与状态机
///       （并发缺陷与掉线自愈见架构评审 P0-2 / P0-4，另行修复）。
/// @attention 状态帧缓冲为进程级单例（static 成员），一个进程只支持一个会话实例。

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
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
  /// @param[in] arm_ip 控制器 IP
  /// @param[in] arm_port 6001 端口（SDK 以字符串接收）
  /// @param[in] arm_port_aux 7000 端口（SDK 以字符串接收）
  ArmSession(rclcpp::Logger logger, std::string arm_ip, std::string arm_port, std::string arm_port_aux);

  /// @brief 连接 6001 与 7000 两个端口，并注册错误/告警与状态回调。
  /// @return 两个端口均连接成功为 true
  bool connect();

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

  /// @brief 读取状态帧缓冲当前序号，作为等待基准。
  /// @return 当前序号
  static uint64_t robot_state_seq();

  /// @brief 等待 7000 口推送的新状态帧。
  /// @param[in] start_seq 起始序号（由 robot_state_seq 取得）
  /// @param[in] timeout 最长等待时间
  /// @param[out] message 收到的状态帧内容
  /// @return 超时前收到新状态帧为 true
  static bool wait_robot_state(uint64_t start_seq, std::chrono::milliseconds timeout, std::string& message);

private:
  /// 7000 口推送的状态帧缓冲（单实例）
  struct RobotStateMessageBuffer
  {
    std::mutex mutex;
    std::condition_variable cv;

    int last_msg_id = -1;
    std::string last_msg;
    uint64_t seq = 0;
  };

  /// @brief 7000 口机器人状态回调（SDK 接收线程调用）。
  /// @param[in] state 状态帧内容
  static void on_robot_state_message(const char *state);

  static RobotStateMessageBuffer g_robot_state_msg_buffer;

  rclcpp::Logger logger_;

  std::string arm_ip_;
  std::string arm_port_;
  std::string arm_port_aux_;

  int socket_fd_{0};
  int socket_fd_aux_{0};
  bool is_connected_{false}; // 机械臂是否连接
  bool is_powered_{false};   // 机械臂是否上电(示教模式)
};
} // namespace tl_driver

#endif // TL_DRIVER__ARM_SESSION_H_
