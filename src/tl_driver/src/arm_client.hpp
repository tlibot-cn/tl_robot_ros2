#ifndef TL_DRIVER__ARM_CLIENT_HPP_
#define TL_DRIVER__ARM_CLIENT_HPP_

/// @file arm_client.hpp
/// @brief SDK 返回值文本化，以及错误/告警与机器人状态异步推送。
/// @note 状态帧按 `channel` 分流缓存（架构评审 §14）：回调只解析并写快照，
///       发布由定时器读取快照完成。SDK 回调无 userdata，因此进程内只允许一个实例。

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include "arm_sdk.hpp"

namespace tl_driver::arm_client
{
/// @brief SDK 返回值转文本，用于日志诊断。
/// @param[in] ret SDK 返回值
/// @return 静态文本，未识别的值返回 "UNKNOWN_ERROR"
const char *result_to_string(tl::Result ret) noexcept;

/// @brief 单通道状态快照。
struct StateFrame
{
  std::string raw;                                     ///< 原始报文，供 get_robot_state 服务返回
  std::vector<double> acs;                             ///< ACS 关节角（度）
  std::vector<double> mcs;                             ///< MCS 位姿（位置 mm、姿态度）
  uint64_t seq{0};                                     ///< 该通道帧序号，0 表示尚未收到
  std::chrono::steady_clock::time_point received_at{}; ///< 接收时刻（单调时钟）
};

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

  /// @brief 请求某通道进入连续推送（mode=1）。
  /// @param[in] fd_aux 7000 辅助端口连接
  /// @param[in] channel 通道号 [1, 9]
  /// @param[in] position 0=ACS 关节坐标，1=MCS 基坐标
  /// @param[in] interval_ms 推送间隔 [10, 60000] ms
  /// @return SDK 返回 SUCCESS 时为 true
  bool request_state_stream(int fd_aux, int channel, int position, int interval_ms);

  /// @brief 请求停止某通道连续推送（stop=true）。
  /// @param[in] fd_aux 7000 辅助端口连接
  /// @param[in] channel 通道号
  /// @param[in] position 0=ACS，1=MCS
  /// @return SDK 返回 SUCCESS 时为 true
  bool stop_state_stream(int fd_aux, int channel, int position);

  /// @brief 读取通道快照（拷贝）。
  /// @param[in] channel 通道号
  /// @param[out] out 快照
  /// @return 该通道至少收到过一帧时为 true
  bool get_state_frame(int channel, StateFrame& out) const;

  /// @brief 只读取通道数值向量与帧元数据（100 Hz 发布路径，避免拷贝原始报文）。
  /// @param[in] channel 1=ACS，2=MCS
  /// @param[out] values 通道数值（ACS 为度；MCS 为 mm/度）
  /// @param[out] seq 帧序号
  /// @param[out] received_at 接收时刻
  /// @return 该通道至少收到过一帧时为 true
  bool get_state_values(int channel, std::vector<double>& values, uint64_t& seq,
                        std::chrono::steady_clock::time_point& received_at) const;

  /// @brief 判断通道最近是否有新帧。
  /// @param[in] channel 通道号
  /// @param[in] max_age 允许的最大帧龄
  /// @return 在 max_age 内收到过帧时为 true
  bool is_state_frame_fresh(int channel, std::chrono::milliseconds max_age) const;

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

  // 通道 1..9；索引 0 不用。
  mutable std::mutex frame_mutex_;
  std::array<StateFrame, 10> frames_;
};
} // namespace tl_driver::arm_client

#endif // TL_DRIVER__ARM_CLIENT_HPP_
