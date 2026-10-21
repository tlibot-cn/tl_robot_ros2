#ifndef TL_DRIVER__ARM_SESSION_HPP_
#define TL_DRIVER__ARM_SESSION_HPP_

/// @file arm_session.hpp
/// @brief 机械臂会话：6001/7000 双端口连接、上下电。
/// @note 会话状态已用原子量消除发布定时器与服务回调并发读取的数据竞争（架构评审 P0-2）；
///       掉线自愈仍属 P0-4，暂缓实施。
/// @note 7000 口状态帧异步通道（缓冲、回调、序号与等待）由 arm_client 承担。

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "arm_client.hpp"

namespace tl_driver
{
/// @brief 与控制器之间的会话（连接、上下电与状态帧）。
/// @note 线程约束（架构评审 P0-2 / D-10）：会话状态仅由 `connect()` / `disconnect()`、
///       上下电接口写入；前者只在 `DriverNode::init()`（`spin()` 之前）与 `~DriverNode()`
///       （`spin()` 返回之后）调用，因此运行期没有并发写者。成员仍用原子类型，
///       保证发布定时器与服务回调并发读取时无数据竞争。
class ArmSession
{
public:
  /// @brief 构造会话，不发起连接。
  /// @param[in] logger 日志器
  explicit ArmSession(rclcpp::Logger logger);

  /// @brief 析构时若仍连接则断开（覆盖构造期异常退出的路径）。
  ~ArmSession();

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

  /// @brief 6001 端口 fd 快照：请求/响应式 SDK 调用。
  /// @note 每次 SDK 调用只取一次快照；未连接时断言失败（定义 NDEBUG 的 Release 构建中不生效）。
  int fd() const;

  /// @brief 7000 端口 fd 快照：servoJ 全系列与机器人状态异步推送。
  int fd_aux() const;

  /// @brief 启动 ACS（channel=1）/MCS（channel=2）双通道连续反馈。
  /// @param[in] interval_ms 推送间隔 [10, 60000] ms
  /// @return 两个通道的 SDK 请求均成功时为 true
  /// @note 必须在 `start_robot_state_channel()`（connect 内）之后调用，否则回调不会收到帧。
  bool start_state_stream(int interval_ms);

  /// @brief 单独重启一个通道的连续反馈（看门狗使用）。
  /// @param[in] channel 1=ACS，2=MCS
  /// @param[in] interval_ms 推送间隔
  /// @return SDK 请求成功时为 true
  bool request_state_stream(int channel, int interval_ms);

  /// @brief 停止两个通道的连续反馈（仅析构/断开路径）。
  void stop_state_stream();

  /// @brief 读取通道快照。
  /// @param[in] channel 1=ACS，2=MCS
  /// @param[out] out 快照
  /// @return 至少收到过一帧时为 true
  bool get_state_frame(int channel, arm_client::StateFrame& out) const;

  /// @brief 只读取通道数值与帧元数据（100 Hz 发布路径）。
  bool get_state_values(int channel, std::vector<double>& values, uint64_t& seq,
                        std::chrono::steady_clock::time_point& received_at) const;

  /// @brief 通道在 `max_age` 内是否收到过帧。
  bool is_state_frame_fresh(int channel, std::chrono::milliseconds max_age) const;

private:
  rclcpp::Logger logger_;
  arm_client::ArmClient client_;

  std::atomic<int> socket_fd_{0};
  std::atomic<int> socket_fd_aux_{0};
  std::atomic<bool> is_connected_{false};           // 机械臂是否连接
  std::atomic<bool> is_powered_{false};             // 机械臂是否上电(示教模式)
  std::atomic<bool> state_stream_requested_{false}; // 是否已向控制器请求过连续状态流
};
} // namespace tl_driver

#endif // TL_DRIVER__ARM_SESSION_HPP_
