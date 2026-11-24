#ifndef TL_DRIVER__PUBLISHERS_HPP_
#define TL_DRIVER__PUBLISHERS_HPP_

/// @file publishers.hpp
/// @brief 状态发布：/joint_states、/tcp_pose 与监测/反馈话题。
/// @note `/joint_states` 与 `/tcp_pose` 由 7000 端口连续状态帧通知的独立发布线程发布，
///       不经过执行器；流中断时话题停更，不回退 6001 同步查询，也不自动重发订阅。
///       运行状态与运行速度均改为按需服务查询（get_arm_status / get_speed），不周期发布。
///       `/joint_states` 同时携带关节速度（`velocity`，rad/s）与力矩（`effort`，‰ 额定扭矩）：控制器按
///       `jointSpeed`/`jointTorque` 开关在 ACS 帧内附带 `jointVel` / `torque`，字段缺失时对应数组为空。
///       诊断话题：`/tl_driver/line_speed` 由 `line_speed_rate` 定时器做 6001 同步查询；
///       `/tl_driver/joint_monitor`（电流/温度/电压）走独立的慢速查询定时器。
///       `/tl_driver/error_info` 为事件驱动：本类在构造时向会话注册 SDK 错误/告警回调，
///       回调线程直接发布，不经过执行器。

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "arm_session.hpp"
#include "convert/state_frame.hpp"
#include "tl_ros2_interface/msg/cartesian_pose.hpp"
#include "tl_ros2_interface/msg/error_info.hpp"
#include "tl_ros2_interface/msg/joint_monitor.hpp"
#include "tl_ros2_interface/msg/line_speed.hpp"

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
  /// @param[in] session 会话，提供连接状态、fd 与状态快照，并接收错误/告警回调；生命周期须长于本对象
  /// @param[in] joint_names 关节名，写入 /joint_states.name
  /// @param[in] config 发布配置
  /// @note 构造时向会话注册错误/告警回调（转发到 `publish_error`），析构时注销；
  ///       因此本对象必须先于会话销毁。
  StatePublishers(rclcpp::Node& node, ArmSession& session, std::vector<std::string> joint_names,
                  const StatePublishConfig& config);

  /// @brief 停止发布线程并释放发布器。
  ~StatePublishers();

  /// @brief 启动/停止 7000 状态帧驱动的独立发布线程。
  /// @note `stop` 必须在会话断开前调用，否则线程会读到已关闭的 fd。
  void start_stream_thread();
  void stop_stream_thread();

  /// @brief 发布 `/tl_driver/joint_monitor`（默认 2 Hz）：同一回调内依次查询电流、温度、电压。
  /// @note 不做缓存；任一子查询失败时对应字段整段 NaN，仍发布本帧。
  void publish_monitor();

  /// @brief 发布 `/tl_driver/line_speed`（`line_speed_rate`，默认 10 Hz）。
  /// @note 一次 6001 同步查询；关节速度与力矩已改为随 7000 状态帧发布（见 `publish_joint_stream`），
  ///       同一查询返回的关节速度结果被丢弃。
  void publish_line_speed();

  /// @brief 发布 `/tl_driver/error_info`：控制器错误/告警异步推送。
  /// @param[in] message_type SDK 回调 messageType
  /// @param[in] message SDK 回调 message 文本
  /// @param[in] message_code SDK 回调 messageCode
  /// @note 由 SDK 回调线程直接调用，事件驱动、不节流；消息在栈上构造，每次发布独立成帧。
  void publish_error(int message_type, const std::string& message, int message_code);

private:
  void publish_joint_stream();
  void publish_tcp_stream();

  /// @brief 发布线程主体：等待 7000 帧通知后发布新帧，不触碰 6001 端口。
  void stream_publish_loop();

  /// @brief 按通道发布各自的新流帧（只在新 seq 时发布）。
  void publish_stream_updates();

  /// @brief 发布 /joint_states：position（rad）、velocity（rad/s）、effort（‰ 额定扭矩）。
  /// @param[in] joint_pose 关节角（rad）
  /// @param[in] count 关节角有效元素个数
  /// @param[in] joint_vel 关节速度原始值（度/秒）
  /// @param[in] joint_vel_count 关节速度有效元素个数；0 = 字段缺失，velocity 数组留空
  /// @param[in] torque 关节力矩原始值（‰ 额定扭矩）
  /// @param[in] torque_count 关节力矩有效元素个数；0 = 字段缺失，effort 数组留空
  /// @param[in] stamp 消息时间戳（帧接收时刻）
  void publish_joint_pose(const std::array<double, convert::kMaxStateAxes>& joint_pose, std::size_t count,
                          const std::array<double, convert::kMaxStateAxes>& joint_vel, std::size_t joint_vel_count,
                          const std::array<double, convert::kMaxStateAxes>& torque, std::size_t torque_count,
                          const rclcpp::Time& stamp);

  /// @brief 发布 /tcp_pose；MCS 流原始值 [mm..., deg...]。
  /// @param[in] mcs MCS 原始值
  /// @param[in] count 有效元素个数
  /// @param[in] stamp 消息时间戳（帧接收时刻）
  void publish_tcp_pose_from_mcs(const std::array<double, convert::kMaxStateAxes>& mcs, std::size_t count,
                                 const rclcpp::Time& stamp);

  rclcpp::Node& node_;
  ArmSession& session_;
  const std::vector<std::string> joint_names_;
  const StatePublishConfig config_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::CartesianPose>::SharedPtr tcp_pose_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::JointMonitor>::SharedPtr joint_monitor_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::LineSpeed>::SharedPtr line_speed_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::ErrorInfo>::SharedPtr error_info_pub_;

  // 流发布缓冲：只由状态帧发布线程访问。
  std::array<double, convert::kMaxStateAxes> joint_pose_{};
  sensor_msgs::msg::JointState joint_state_msg_;
  tl_ros2_interface::msg::CartesianPose tcp_pose_msg_;
  // /joint_monitor 缓存：只由监测定时器线程访问，整帧覆盖电流/温度/电压，不做跨帧复用。
  tl_ros2_interface::msg::JointMonitor joint_monitor_msg_;
  // /line_speed 缓存：只由 line_speed 定时器线程访问。
  tl_ros2_interface::msg::LineSpeed line_speed_msg_;

  uint64_t last_acs_seq_{0};
  uint64_t last_mcs_seq_{0};

  // 7000 帧驱动发布线程：停止由 DriverNode 在会话断开前触发。
  std::thread stream_thread_;
  std::atomic<bool> stream_thread_stop_{false};
  uint64_t stream_notify_seen_{0};
};
} // namespace tl_driver

#endif // TL_DRIVER__PUBLISHERS_HPP_
