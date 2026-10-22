#ifndef TL_DRIVER__PUBLISHERS_HPP_
#define TL_DRIVER__PUBLISHERS_HPP_

/// @file publishers.hpp
/// @brief 状态发布：/joint_states、/tcp_pose、/arm_status 与诊断话题。
/// @note 架构评审 §14：`/joint_states` 与 `/tcp_pose` 由 7000 端口连续状态帧通知的独立发布线程发布，
///       不经过执行器；`state_source=poll` 或流长时间失效时回退旧的同步查询路径（20 Hz）。
///       `/arm_status` 由 10 Hz 查询缓存、100 Hz 定时器只发布缓存。

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "arm_session.hpp"
#include "tl_ros2_interface/msg/arm_status.hpp"
#include "tl_ros2_interface/msg/cartesian_pose.hpp"
#include "tl_ros2_interface/msg/io_state.hpp"
#include "tl_ros2_interface/msg/joint_dynamics.hpp"
#include "tl_ros2_interface/msg/joint_thermal.hpp"

namespace tl_driver
{
/// @brief 状态发布配置（来自节点参数，构造后不变）。
struct StatePublishConfig
{
  bool stream_enabled{true};  ///< state_source == "stream"
  int stream_interval_ms{10}; ///< 连续状态流间隔
  int ndof{6};                ///< 关节数
};

/// @brief 状态话题的发布器与填充逻辑。
class StatePublishers
{
public:
  /// @brief 构造并创建发布器。
  /// @param[in] node 节点，提供时钟与日志器，生命周期须长于本对象
  /// @param[in] session 会话，提供连接状态、fd 与状态快照，生命周期须长于本对象
  /// @param[in] joint_names 关节名，写入 /joint_states.name
  /// @param[in] config 发布配置
  StatePublishers(rclcpp::Node& node, const ArmSession& session, std::vector<std::string> joint_names,
                  const StatePublishConfig& config);

  /// @brief 停止发布线程并释放发布器。
  ~StatePublishers();

  /// @brief 100 Hz 定时器入口：发布缓存的 /arm_status，并做流健康检查与 poll 回退。
  /// @note 仅可在 timer_group_ 内调用。健康流路径不调用 SDK；`state_source=poll` 或通道
  ///       失效回退时，本回调以 20 Hz 上限执行同步查询，因此那时的频率不保证 100 Hz。
  void publish_arm_state();

  /// @brief 启动/停止 7000 状态帧驱动的独立发布线程。
  /// @note 仅 `state_source=stream` 时启动；`stop` 必须在会话断开前调用，否则线程会读到已关闭的 fd。
  void start_stream_thread();
  void stop_stream_thread();

  /// @brief 10 Hz 定时器入口：同步查询运行状态并写入缓存，供 100 Hz 发布复用。
  void query_running_status();

  /// @brief 发布 `/tl_driver/joint_dynamics`（默认 10 Hz）。
  void publish_dynamics();

  /// @brief 发布 `/tl_driver/joint_thermal`（默认 1 Hz，可关闭）。
  void publish_thermal();

  /// @brief 发布 `/tl_driver/io_state`（默认 10 Hz）。
  void publish_io();

private:
  void publish_joint_stream();
  void publish_tcp_stream();
  void poll_joint(int fd);
  void poll_tcp(int fd);

  /// @brief 发布线程主体：等待 7000 帧通知后发布新帧，不触碰 6001 端口。
  void stream_publish_loop();

  /// @brief 按通道健康状态发布各自的新流帧（只在新 seq 时发布）。
  void publish_stream_updates();

  /// @brief 发布 /joint_states（rad）。
  /// @param[in] joint_pose 关节角（rad）
  /// @param[in] stamp 消息时间戳（流路径用帧接收时刻，poll 路径用当前时刻）
  void publish_joint_pose(const std::vector<double>& joint_pose, const rclcpp::Time& stamp);

  /// @brief 发布 /tcp_pose；位置 mm→m，rpy 按调用方给出的来源处理。
  /// @param[in] tcp_pose [mm, mm, mm, rad, rad, rad, ...]（旧同步查询路径）
  /// @param[in] stamp 消息时间戳
  void publish_tcp_pose(const std::vector<double>& tcp_pose, const rclcpp::Time& stamp);

  /// @brief 发布 /tcp_pose；MCS 流原始值 [mm..., deg...]。
  /// @param[in] mcs MCS 原始值
  /// @param[in] stamp 消息时间戳（帧接收时刻）
  void publish_tcp_pose_from_mcs(const std::vector<double>& mcs, const rclcpp::Time& stamp);

  /// @brief 发布缓存的 /arm_status（不查询 SDK）。
  void publish_running_status();

  struct ChannelHealth
  {
    std::atomic<bool> stream_active{true};
    std::chrono::steady_clock::time_point stale_since{};
    std::chrono::steady_clock::time_point fresh_since{};
  };

  /// @brief 单通道健康计时：连续失效 3 s 后该通道回退 poll，恢复 1 s 后切回 stream。
  /// @param[in,out] health 通道健康状态
  /// @param[in] unavailable 本周期是否无可用流数据
  /// @param[in] name 日志用通道名
  void update_channel_health(ChannelHealth& health, bool unavailable, const char *name);

  rclcpp::Node& node_;
  const ArmSession& session_;
  const std::vector<std::string> joint_names_;
  const StatePublishConfig config_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::CartesianPose>::SharedPtr tcp_pose_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::ArmStatus>::SharedPtr running_status_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::JointDynamics>::SharedPtr joint_dynamics_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::JointThermal>::SharedPtr joint_thermal_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::IoState>::SharedPtr io_state_pub_;

  // 流发布缓冲：独立发布线程与 poll 回退路径都会写，统一由 publish_mutex_ 串行化。
  std::mutex publish_mutex_;
  std::vector<double> joint_pose_;
  std::vector<double> tcp_pose_;
  sensor_msgs::msg::JointState joint_state_msg_;
  tl_ros2_interface::msg::CartesianPose tcp_pose_msg_;
  // /arm_status 缓存：10 Hz 查询线程写、100 Hz 定时器读，统一由 status_mutex_ 保护。
  tl_ros2_interface::msg::ArmStatus arm_status_msg_;
  bool arm_status_sampled_{false};
  int arm_status_failures_{0}; ///< 连续查询失败次数，达到 3 次后停止发布过期缓存
  std::mutex status_mutex_;
  tl_ros2_interface::msg::JointDynamics joint_dynamics_msg_;
  tl_ros2_interface::msg::JointThermal joint_thermal_msg_;
  tl_ros2_interface::msg::IoState io_state_msg_;

  ChannelHealth acs_health_;
  ChannelHealth mcs_health_;
  uint64_t last_acs_seq_{0};
  uint64_t last_mcs_seq_{0};
  std::chrono::steady_clock::time_point last_poll_{};

  // 7000 帧驱动发布线程：只在 stream 模式启动，停止由 DriverNode 在会话断开前触发。
  std::thread stream_thread_;
  std::atomic<bool> stream_thread_stop_{false};
  uint64_t stream_notify_seen_{0};
};
} // namespace tl_driver

#endif // TL_DRIVER__PUBLISHERS_HPP_
