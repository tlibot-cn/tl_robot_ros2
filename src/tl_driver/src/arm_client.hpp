#ifndef TL_DRIVER__ARM_CLIENT_HPP_
#define TL_DRIVER__ARM_CLIENT_HPP_

/// @file arm_client.hpp
/// @brief SDK 返回值文本化，以及错误/告警与机器人状态异步推送。
/// @note 状态帧缓冲、日志器与节流状态归属 `ArmClient` 实例；SDK 回调无 userdata，
///       因此进程内只允许一个实例，构造第二个实例抛 `std::logic_error`。

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include "arm_sdk.hpp"

namespace tl_driver::arm_client
{
/// @brief SDK 返回值转文本，用于日志诊断。
/// @param[in] ret SDK 返回值
/// @return 静态文本，未识别的值返回 "UNKNOWN_ERROR"
const char *result_to_string(tl::Result ret) noexcept;

/// @brief 异步推送通道：错误/告警回调与 7000 口机器人状态帧。
class ArmClient
{
public:
  /// @brief 构造并注册为进程内唯一实例。
  /// @param[in] logger 日志器
  /// @throws std::logic_error 已存在另一个实例
  explicit ArmClient(rclcpp::Logger logger);

  /// @brief 注销进程内实例。
  ~ArmClient();

  ArmClient(const ArmClient&) = delete;
  ArmClient& operator=(const ArmClient&) = delete;

  /// @brief 在指定端口注册错误/告警异步推送回调。
  /// @param[in] fd 已建立的 SDK 连接（6001 与 7000 均需注册）
  void register_error_callback(int fd);

  /// @brief 在辅助端口注册机器人状态异步推送。
  /// @param[in] fd_aux 7000 辅助端口连接
  void start_robot_state_channel(int fd_aux);

  /// @brief 读取当前状态帧序号，作为等待新帧的基准。
  /// @return 单调递增的状态帧序号
  uint64_t robot_state_seq() const;

  /// @brief 等待一帧机器人状态消息。
  /// @param[in] start_seq 调用前的序号（见 robot_state_seq）
  /// @param[in] timeout 最长等待时间
  /// @param[out] message 成功时写入状态消息原文
  /// @return 收到序号更新的状态帧为 true，超时为 false
  bool wait_robot_state(uint64_t start_seq, std::chrono::milliseconds timeout, std::string& message);

private:
  static void error_callback(int message_type, const char *message, int message_code);
  static void state_callback(const char *state);

  void on_error(int message_type, const char *message, int message_code);
  void on_state(const char *state);
  bool should_log(std::atomic<int64_t>& last_ns, std::chrono::milliseconds period);

  // SDK 回调无 userdata：用受锁保护的进程级指针转发到实例。
  static std::mutex registry_mutex_;
  static ArmClient *active_;

  rclcpp::Logger logger_;
  rclcpp::Clock clock_{RCL_STEADY_TIME};
  std::atomic<int64_t> last_error_log_ns_{0};
  std::atomic<int64_t> last_state_log_ns_{0};

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::string last_msg_;
  uint64_t seq_{0};
};
} // namespace tl_driver::arm_client

#endif // TL_DRIVER__ARM_CLIENT_HPP_
