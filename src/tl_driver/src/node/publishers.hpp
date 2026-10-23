#ifndef TL_DRIVER__PUBLISHERS_HPP_
#define TL_DRIVER__PUBLISHERS_HPP_

/// @file publishers.hpp
/// @brief 状态发布：/joint_states、/tcp_pose、/arm_status 与诊断话题。
/// @note `/joint_states` 与 `/tcp_pose` 由 7000 端口连续状态帧通知的独立发布线程发布，
///       不经过执行器；流失效时话题停更，1 Hz 看门狗重发 mode=1 请求，不再回退 6001 同步查询。
///       `/arm_status` 在每次 10 Hz 运行状态查询成功后发布（不再由独立定时器重复发布缓存）。

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "arm_session.hpp"
#include "convert/state_frame.hpp"
#include "tl_ros2_interface/msg/arm_status.hpp"
#include "tl_ros2_interface/msg/cartesian_pose.hpp"
#include "tl_ros2_interface/msg/joint_dynamics.hpp"
#include "tl_ros2_interface/msg/joint_thermal.hpp"

namespace tl_driver
{
/// @brief 状态发布配置（来自节点参数，构造后不变）。
struct StatePublishConfig
{
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

  /// @brief 启动/停止 7000 状态帧驱动的独立发布线程。
  /// @note `stop` 必须在会话断开前调用，否则线程会读到已关闭的 fd。
  void start_stream_thread();
  void stop_stream_thread();

  /// @brief 10 Hz 定时器入口：同步查询运行状态，更新缓存并发布 /arm_status。
  void query_running_status();

  /// @brief 1 Hz 定时器入口：查询运行速度并写入 /arm_status 缓存。
  void query_speed();

  /// @brief 发布 `/tl_driver/joint_dynamics`（默认 10 Hz）。
  void publish_dynamics();

  /// @brief 发布 `/tl_driver/joint_thermal`（默认 1 Hz，可关闭）。
  void publish_thermal();

private:
  void publish_joint_stream();
  void publish_tcp_stream();

  /// @brief 发布线程主体：等待 7000 帧通知后发布新帧，不触碰 6001 端口。
  void stream_publish_loop();

  /// @brief 按通道发布各自的新流帧（只在新 seq 时发布）。
  void publish_stream_updates();

  /// @brief 发布 /joint_states（rad）。
  /// @param[in] joint_pose 关节角（rad）
  /// @param[in] count 有效元素个数
  /// @param[in] stamp 消息时间戳（帧接收时刻）
  void publish_joint_pose(const std::array<double, convert::kMaxStateAxes>& joint_pose, std::size_t count,
                          const rclcpp::Time& stamp);

  /// @brief 发布 /tcp_pose；MCS 流原始值 [mm..., deg...]。
  /// @param[in] mcs MCS 原始值
  /// @param[in] count 有效元素个数
  /// @param[in] stamp 消息时间戳（帧接收时刻）
  void publish_tcp_pose_from_mcs(const std::array<double, convert::kMaxStateAxes>& mcs, std::size_t count,
                                 const rclcpp::Time& stamp);

  rclcpp::Node& node_;
  const ArmSession& session_;
  const std::vector<std::string> joint_names_;
  const StatePublishConfig config_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::CartesianPose>::SharedPtr tcp_pose_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::ArmStatus>::SharedPtr running_status_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::JointDynamics>::SharedPtr joint_dynamics_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::JointThermal>::SharedPtr joint_thermal_pub_;

  // 流发布缓冲：只由状态帧发布线程访问。
  std::array<double, convert::kMaxStateAxes> joint_pose_{};
  sensor_msgs::msg::JointState joint_state_msg_;
  tl_ros2_interface::msg::CartesianPose tcp_pose_msg_;
  // /arm_status 缓存：10 Hz 查询线程写、1 Hz 速度查询线程改 speed 字段，统一由 status_mutex_ 保护。
  tl_ros2_interface::msg::ArmStatus arm_status_msg_;
  std::mutex status_mutex_;
  tl_ros2_interface::msg::JointDynamics joint_dynamics_msg_;
  tl_ros2_interface::msg::JointThermal joint_thermal_msg_;

  uint64_t last_acs_seq_{0};
  uint64_t last_mcs_seq_{0};

  // 7000 帧驱动发布线程：停止由 DriverNode 在会话断开前触发。
  std::thread stream_thread_;
  std::atomic<bool> stream_thread_stop_{false};
  uint64_t stream_notify_seen_{0};
};
} // namespace tl_driver

#endif // TL_DRIVER__PUBLISHERS_HPP_
