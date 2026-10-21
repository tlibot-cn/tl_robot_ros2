#ifndef TL_DRIVER__ACTIONS_MOTION_HPP_
#define TL_DRIVER__ACTIONS_MOTION_HPP_

/// @file actions_motion.hpp
/// @brief 运动 action：/tl_driver/move_joint、/tl_driver/move_linear、/tl_driver/play_track。
/// @note 架构评审 §15：只对长任务提供 action；现有话题/服务保留。
///       控制器 SDK 无通用停止接口，cancel 一律显式拒绝，绝不返回“取消成功”。

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "tl_ros2_interface/action/move.hpp"
#include "tl_ros2_interface/action/play_track.hpp"

namespace tl_driver
{
class DriverNode;

/// @brief 三个 action server 的持有者；execute 在单个工作线程中串行执行。
class MotionActionServers
{
public:
  /// @brief 创建 action server 并启动工作线程。
  /// @param[in] node 驱动节点，生命周期须长于本对象
  /// @param[in] group goal/cancel 回调组
  MotionActionServers(DriverNode& node, const rclcpp::CallbackGroup::SharedPtr& group);

  /// @brief 停止工作线程；未执行完的 goal 不再开始。
  ~MotionActionServers();

  MotionActionServers(const MotionActionServers&) = delete;
  MotionActionServers& operator=(const MotionActionServers&) = delete;

  /// @brief 是否有运动 goal 正在执行（供话题/服务入口拒绝并发运动）。
  bool is_motion_active() const noexcept
  {
    return motion_active_.load(std::memory_order_acquire);
  }

  /// @brief 记录一次外部运动指令（moveJ/moveL 话题），用于拒绝紧随其后的 action goal。
  void note_external_motion();

private:
  using Move = tl_ros2_interface::action::Move;
  using PlayTrack = tl_ros2_interface::action::PlayTrack;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;
  using GoalHandlePlayTrack = rclcpp_action::ServerGoalHandle<PlayTrack>;

  rclcpp_action::GoalResponse handle_move_goal(const rclcpp_action::GoalUUID& uuid,
                                               std::shared_ptr<const Move::Goal> goal, bool linear);
  rclcpp_action::CancelResponse handle_move_cancel(const std::shared_ptr<GoalHandleMove> goal_handle);
  void handle_move_accepted(const std::shared_ptr<GoalHandleMove> goal_handle, bool linear);

  rclcpp_action::GoalResponse handle_play_goal(const rclcpp_action::GoalUUID& uuid,
                                               std::shared_ptr<const PlayTrack::Goal> goal);
  rclcpp_action::CancelResponse handle_play_cancel(const std::shared_ptr<GoalHandlePlayTrack> goal_handle);
  void handle_play_accepted(const std::shared_ptr<GoalHandlePlayTrack> goal_handle);

  void execute_move(const std::shared_ptr<GoalHandleMove> goal_handle, bool linear);
  void execute_play(const std::shared_ptr<GoalHandlePlayTrack> goal_handle);

  // 异常边界：实现抛异常时转为 abort 并释放运动槽，绝不让异常逃出工作线程。
  void execute_move_impl(const std::shared_ptr<GoalHandleMove>& goal_handle, bool linear);
  void execute_play_impl(const std::shared_ptr<GoalHandlePlayTrack>& goal_handle);
  void abort_move(const std::shared_ptr<GoalHandleMove>& goal_handle, const std::string& message) noexcept;
  void abort_play(const std::shared_ptr<GoalHandlePlayTrack>& goal_handle, const std::string& message) noexcept;

  /// @brief 工作线程主循环：串行执行已接受的 goal。
  void worker_loop();

  /// @brief 尝试占用唯一的运动槽；已占用时返回 false。
  bool acquire_motion_slot();

  /// @brief 释放运动槽。
  void release_motion_slot();

  DriverNode& node_;
  rclcpp_action::Server<Move>::SharedPtr move_joint_server_;
  rclcpp_action::Server<Move>::SharedPtr move_linear_server_;
  rclcpp_action::Server<PlayTrack>::SharedPtr play_track_server_;

  std::atomic<bool> motion_active_{false};
  std::atomic<bool> stop_worker_{false};
  std::atomic<int64_t> last_external_motion_ns_{0};

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<std::function<void()>> jobs_;
  std::thread worker_;
};
} // namespace tl_driver

#endif // TL_DRIVER__ACTIONS_MOTION_HPP_
