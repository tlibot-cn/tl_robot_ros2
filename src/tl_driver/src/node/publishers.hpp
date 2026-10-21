#ifndef TL_DRIVER__PUBLISHERS_HPP_
#define TL_DRIVER__PUBLISHERS_HPP_

/// @file publishers.hpp
/// @brief 状态发布：/joint_states、/tcp_pose、/arm_status 与诊断话题。
/// @note 架构评审 §14：/joint_states 与 /tcp_pose 默认由 7000 端口连续状态流的快照驱动，
///       每周期只读快照、不调用 SDK；`state_source=poll` 或流长时间失效时回退旧的同步查询路径。

#include <chrono>
#include <cstdint>
#include <string>
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

  /// @brief 100 Hz 定时器入口：/arm_status 始终同步查询；位姿优先使用状态流。
  /// @note 仅可在 timer_group_ 内调用；内部复用成员消息，非线程安全。
  void publish_arm_state();

  /// @brief 发布 `/tl_driver/joint_dynamics`（默认 10 Hz）。
  void publish_dynamics();

  /// @brief 发布 `/tl_driver/joint_thermal`（默认 0.5 Hz，可关闭）。
  void publish_thermal();

  /// @brief 发布 `/tl_driver/io_state`（默认 10 Hz）。
  void publish_io();

private:
  /// @brief 单通道状态流的本周期结果。
  enum class StreamResult
  {
    kPublished,  ///< 发布了新帧
    kNoNewFrame, ///< 流健康，但本周期尚无新帧（跳过发布）
    kUnavailable ///< 无快照或帧龄超过阈值
  };

  StreamResult publish_joint_stream();
  StreamResult publish_tcp_stream();
  void poll_joint(int fd);
  void poll_tcp(int fd);

  /// @brief 发布 /joint_states（rad）。
  void publish_joint_pose(const std::vector<double>& joint_pose);

  /// @brief 发布 /tcp_pose；位置 mm→m，rpy 按调用方给出的来源处理。
  /// @param[in] tcp_pose [mm, mm, mm, rad, rad, rad, ...]（旧同步查询路径）
  void publish_tcp_pose(const std::vector<double>& tcp_pose);

  /// @brief 发布 /tcp_pose；MCS 流原始值 [mm..., deg...]。
  void publish_tcp_pose_from_mcs(const std::vector<double>& mcs);

  /// @brief 发布 /arm_status。
  void publish_running_status(int fd);

  struct ChannelHealth
  {
    bool stream_active{true};
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

  // 100 Hz 发布路径复用缓冲，避免每周期分配；仅在 timer_group_ 内使用。
  std::vector<double> joint_pose_;
  std::vector<double> tcp_pose_;
  sensor_msgs::msg::JointState joint_state_msg_;
  tl_ros2_interface::msg::CartesianPose tcp_pose_msg_;
  tl_ros2_interface::msg::ArmStatus arm_status_msg_;
  tl_ros2_interface::msg::JointDynamics joint_dynamics_msg_;
  tl_ros2_interface::msg::JointThermal joint_thermal_msg_;
  tl_ros2_interface::msg::IoState io_state_msg_;

  ChannelHealth acs_health_;
  ChannelHealth mcs_health_;
  uint64_t last_acs_seq_{0};
  uint64_t last_mcs_seq_{0};
  std::chrono::steady_clock::time_point last_poll_{};
};
} // namespace tl_driver

#endif // TL_DRIVER__PUBLISHERS_HPP_
