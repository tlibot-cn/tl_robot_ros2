/// @file actions_motion.cpp
/// @brief 运动/轨迹回放 action 的实现。

#include "node/actions_motion.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "arm_client.hpp"
#include "arm_sdk.hpp"
#include "convert/msg_convert.hpp"
#include "convert/units.hpp"
#include "node/driver_node.hpp"

namespace tl_driver
{
namespace
{
constexpr auto kPollPeriod = std::chrono::milliseconds(50);
constexpr auto kFeedbackPeriod = std::chrono::milliseconds(100);
constexpr auto kMotionStartTimeout = std::chrono::seconds(3);
constexpr auto kSdkSamplePeriod = std::chrono::milliseconds(200); // 流不可用时的同步采样周期
constexpr auto kMotionSettleTimeout = std::chrono::seconds(2);    // 非 RUNNING 后等待目标收敛的时间
constexpr auto kExternalMotionGrace = std::chrono::milliseconds(500); // 外部运动（队列/轨迹回放）后拒绝 action 的窗口
constexpr int kMotionConfirmPolls = 2;                                // 位姿变化连续确认次数
constexpr int kInactiveConfirmPolls = 4;         // 非 RUNNING 连续确认次数（覆盖 ~200 ms 状态延迟）
constexpr double kJointMoveEpsilonRad = 1e-3;    // rad，约 0.057°，位姿变化判据
constexpr double kJointMoveEpsilonDeg = 0.1;     // 度，progress 起点判据
constexpr double kJointReachEpsilonDeg = 1.0;    // 度，目标到达判据
constexpr double kCartesianMoveEpsilonM = 1e-3;  // m，1 mm，位姿变化判据
constexpr double kCartesianMoveEpsilonMm = 1.0;  // mm，progress 起点判据
constexpr double kCartesianReachEpsilonMm = 2.0; // mm，目标到达判据
constexpr double kMaxActionTimeoutSec = 3600.0;

bool read_run_state(const DriverNode& node, tl::RunState& state)
{
  if (!node.session().is_connected())
  {
    return false;
  }
  return tl::get_robot_running_state(node.session().fd(), state) == tl::Result::SUCCESS;
}

const char *run_state_text(tl::RunState state)
{
  switch (state)
  {
    case tl::RunState::STOP:
      return "STOP";
    case tl::RunState::PAUSE:
      return "PAUSE";
    case tl::RunState::RUNNING:
      return "RUNNING";
    default:
      return "UNKNOWN";
  }
}

tl_ros2_interface::msg::CartesianPose make_current_pose(const DriverNode& node, const std::vector<double>& mcs)
{
  tl_ros2_interface::msg::CartesianPose pose;
  pose.header.stamp = node.now();
  pose.header.frame_id = "base_link";
  if (mcs.size() < 6)
  {
    return pose;
  }

  pose.position.x = convert::mm_to_m(mcs[0]);
  pose.position.y = convert::mm_to_m(mcs[1]);
  pose.position.z = convert::mm_to_m(mcs[2]);
  pose.rpy.x = convert::deg_to_rad(mcs[3]);
  pose.rpy.y = convert::deg_to_rad(mcs[4]);
  pose.rpy.z = convert::deg_to_rad(mcs[5]);
  pose.arm_angle = (node.ndof() == 7 && mcs.size() > 6) ? convert::deg_to_rad(mcs[6]) : 0.0;
  return pose;
}

/// @brief 状态流不可用时用同步查询补一个位姿样本。
/// @param[in] fd 6001 端口连接
/// @param[in] coord JOINT（度）或 BASE（位置 mm、姿态 rad）
/// @param[out] out 位姿向量
/// @return 查询成功时为 true
bool sample_position(int fd, tl::Coord coord, std::vector<double>& out)
{
  out.clear();
  if (tl::get_current_position(fd, coord, out) != tl::Result::SUCCESS)
  {
    return false;
  }

  // 同步查询的 BASE 姿态为 rad；统一成 MCS 流的 mm + 度，供 feedback/进度共用。
  if (coord == tl::Coord::BASE)
  {
    for (std::size_t i = 3; i < 6 && i < out.size(); ++i)
    {
      out[i] = convert::rad_to_deg(out[i]);
    }
  }
  return true;
}

/// @brief 按坐标系估算完成度；无法估算时返回 -1。
double compute_progress(const tl_ros2_interface::action::Move::Goal& goal, const std::vector<double>& start_acs,
                        const std::vector<double>& start_mcs, const std::vector<double>& current_acs,
                        const std::vector<double>& current_mcs)
{
  const auto& target = goal.command.target_pos_value;
  if (goal.command.coord == 0)
  {
    if (start_acs.empty() || current_acs.empty() || target.size() < start_acs.size())
    {
      return -1.0;
    }
    const std::size_t count = start_acs.size();
    double initial = 0.0;
    double remaining = 0.0;
    for (std::size_t i = 0; i < count; ++i)
    {
      const double target_deg = convert::rad_to_deg(target[i]);
      initial += std::abs(target_deg - start_acs[i]);
      remaining += std::abs(target_deg - current_acs[i]);
    }
    if (initial < kJointMoveEpsilonDeg)
    {
      return 1.0;
    }
    return std::clamp(1.0 - remaining / initial, 0.0, 1.0);
  }

  if (goal.command.coord == 1)
  {
    if (start_mcs.size() < 3 || current_mcs.size() < 3 || target.size() < 3)
    {
      return -1.0;
    }
    const double tx = convert::m_to_mm(target[0]);
    const double ty = convert::m_to_mm(target[1]);
    const double tz = convert::m_to_mm(target[2]);
    const double initial = std::hypot(std::hypot(tx - start_mcs[0], ty - start_mcs[1]), tz - start_mcs[2]);
    const double remaining = std::hypot(std::hypot(tx - current_mcs[0], ty - current_mcs[1]), tz - current_mcs[2]);
    if (initial < kCartesianMoveEpsilonMm)
    {
      return 1.0;
    }
    return std::clamp(1.0 - remaining / initial, 0.0, 1.0);
  }

  return -1.0;
}

bool pose_changed(const tl_ros2_interface::action::Move::Goal& goal, const std::vector<double>& start_acs,
                  const std::vector<double>& start_mcs, const std::vector<double>& current_acs,
                  const std::vector<double>& current_mcs)
{
  if (goal.command.coord == 0)
  {
    const std::size_t count = std::min(start_acs.size(), current_acs.size());
    for (std::size_t i = 0; i < count; ++i)
    {
      if (std::abs(convert::deg_to_rad(current_acs[i] - start_acs[i])) > kJointMoveEpsilonRad)
      {
        return true;
      }
    }
    return false;
  }

  if (goal.command.coord == 1)
  {
    if (start_mcs.size() < 3 || current_mcs.size() < 3)
    {
      return false;
    }
    const double dx = convert::mm_to_m(current_mcs[0] - start_mcs[0]);
    const double dy = convert::mm_to_m(current_mcs[1] - start_mcs[1]);
    const double dz = convert::mm_to_m(current_mcs[2] - start_mcs[2]);
    return std::hypot(std::hypot(dx, dy), dz) > kCartesianMoveEpsilonM;
  }

  return false;
}
/// @brief 目标到达判定；action 仅支持 coord=0（关节）与 coord=1（基坐标）。
enum class TargetState
{
  kReached,
  kNotReached,
  kUnknown
};

TargetState evaluate_target(const tl_ros2_interface::action::Move::Goal& goal, const std::vector<double>& current_acs,
                            const std::vector<double>& current_mcs)
{
  const auto& target = goal.command.target_pos_value;
  if (goal.command.coord == 0)
  {
    const std::size_t count = std::min(target.size(), current_acs.size());
    if (count == 0)
    {
      return TargetState::kUnknown;
    }
    for (std::size_t i = 0; i < count; ++i)
    {
      if (std::abs(convert::rad_to_deg(target[i]) - current_acs[i]) > kJointReachEpsilonDeg)
      {
        return TargetState::kNotReached;
      }
    }
    return TargetState::kReached;
  }

  if (goal.command.coord == 1)
  {
    if (target.size() < 3 || current_mcs.size() < 3)
    {
      return TargetState::kUnknown;
    }
    const double dx = convert::m_to_mm(target[0]) - current_mcs[0];
    const double dy = convert::m_to_mm(target[1]) - current_mcs[1];
    const double dz = convert::m_to_mm(target[2]) - current_mcs[2];
    const double distance = std::hypot(std::hypot(dx, dy), dz);
    return distance <= kCartesianReachEpsilonMm ? TargetState::kReached : TargetState::kNotReached;
  }

  return TargetState::kUnknown;
}

} // namespace

MotionActionServers::MotionActionServers(DriverNode& node, const rclcpp::CallbackGroup::SharedPtr& group) : node_(node)
{
  // Humble 的 create_server 接收 rcl_action_server_options_t 与独立回调组参数。
  const rcl_action_server_options_t options = rcl_action_server_get_default_options();

  const auto handle_goal_joint = [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Move::Goal> goal)
  {
    (void)uuid;
    return handle_move_goal(uuid, std::move(goal), false);
  };
  const auto handle_goal_linear = [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Move::Goal> goal)
  {
    (void)uuid;
    return handle_move_goal(uuid, std::move(goal), true);
  };
  const auto handle_cancel_move = [this](const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    return handle_move_cancel(goal_handle);
  };
  const auto handle_accepted_joint = [this](const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    handle_move_accepted(goal_handle, false);
  };
  const auto handle_accepted_linear = [this](const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    handle_move_accepted(goal_handle, true);
  };

  move_joint_server_ = rclcpp_action::create_server<Move>(
      node_.get_node_base_interface(), node_.get_node_clock_interface(), node_.get_node_logging_interface(),
      node_.get_node_waitables_interface(), "/tl_driver/move_joint", handle_goal_joint, handle_cancel_move,
      handle_accepted_joint, options, group);
  move_linear_server_ = rclcpp_action::create_server<Move>(
      node_.get_node_base_interface(), node_.get_node_clock_interface(), node_.get_node_logging_interface(),
      node_.get_node_waitables_interface(), "/tl_driver/move_linear", handle_goal_linear, handle_cancel_move,
      handle_accepted_linear, options, group);

  const auto handle_goal_play = [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const PlayTrack::Goal> goal)
  {
    (void)uuid;
    return handle_play_goal(uuid, std::move(goal));
  };
  const auto handle_cancel_play = [this](const std::shared_ptr<GoalHandlePlayTrack> goal_handle)
  {
    return handle_play_cancel(goal_handle);
  };
  const auto handle_accepted_play = [this](const std::shared_ptr<GoalHandlePlayTrack> goal_handle)
  {
    handle_play_accepted(goal_handle);
  };

  play_track_server_ = rclcpp_action::create_server<PlayTrack>(
      node_.get_node_base_interface(), node_.get_node_clock_interface(), node_.get_node_logging_interface(),
      node_.get_node_waitables_interface(), "/tl_driver/play_track", handle_goal_play, handle_cancel_play,
      handle_accepted_play, options, group);

  worker_ = std::thread(&MotionActionServers::worker_loop, this);
}

MotionActionServers::~MotionActionServers()
{
  stop_worker_.store(true, std::memory_order_release);
  queue_cv_.notify_all();
  if (worker_.joinable())
  {
    worker_.join();
  }
}

void MotionActionServers::note_external_motion()
{
  const int64_t now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  last_external_motion_ns_.store(now_ns, std::memory_order_release);
}

bool MotionActionServers::acquire_motion_slot()
{
  bool expected = false;
  return motion_active_.compare_exchange_strong(expected, true, std::memory_order_acq_rel);
}

void MotionActionServers::release_motion_slot()
{
  motion_active_.store(false, std::memory_order_release);
}

rclcpp_action::GoalResponse MotionActionServers::handle_move_goal(const rclcpp_action::GoalUUID&,
                                                                  std::shared_ptr<const Move::Goal> goal, bool linear)
{
  if (!node_.session().is_connected())
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: move goal rejected: arm is not connected");
    return rclcpp_action::GoalResponse::REJECT;
  }

  const int64_t last_external_ns = last_external_motion_ns_.load(std::memory_order_acquire);
  const int64_t now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  if (last_external_ns != 0 &&
      now_ns - last_external_ns < std::chrono::duration_cast<std::chrono::nanoseconds>(kExternalMotionGrace).count())
  {
    RCLCPP_WARN(node_.get_logger(),
                "[Action]: move goal rejected: an external motion command was issued less than 500 ms ago");
    return rclcpp_action::GoalResponse::REJECT;
  }

  const auto& command = goal->command;
  for (const double value : command.target_pos_value)
  {
    if (!std::isfinite(value))
    {
      RCLCPP_WARN(node_.get_logger(), "[Action]: move goal rejected: target_pos_value contains non-finite value");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  if ((linear && command.coord != 1) || (!linear && command.coord != 0))
  {
    RCLCPP_WARN(node_.get_logger(),
                "[Action]: move goal rejected: move_joint requires coord=0, move_linear requires coord=1");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!std::isfinite(command.velocity) || !std::isfinite(command.acc) || !std::isfinite(command.dec) ||
      command.velocity <= 0.0 || command.acc <= 0.0 || command.acc > 100.0 || command.dec <= 0.0 || command.dec > 100.0)
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: move goal rejected: invalid velocity/acc/dec");
    return rclcpp_action::GoalResponse::REJECT;
  }
  // MOVJ 速度单位为 %，MOVL 为 m/s（MOVL 上限 1 m/s）。
  if (linear ? command.velocity > 1.0 : command.velocity > 100.0)
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: move goal rejected: velocity out of range");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!std::isfinite(command.velocity_sync) || command.velocity_sync < 0.0 ||
      (linear ? command.velocity_sync > 1.0 : command.velocity_sync > 100.0))
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: move goal rejected: velocity_sync out of range");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (command.pl < 0 || command.pl > 5 || command.tool_num < 0 || command.user_num < 0)
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: move goal rejected: invalid pl/tool_num/user_num");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!std::isfinite(goal->timeout_sec) || goal->timeout_sec > kMaxActionTimeoutSec)
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: move goal rejected: timeout_sec must be <= %.0f s",
                kMaxActionTimeoutSec);
    return rclcpp_action::GoalResponse::REJECT;
  }

  tl::RunState state = tl::RunState::STOP;
  if (!read_run_state(node_, state))
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: move goal rejected: failed to read run state");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (state == tl::RunState::RUNNING)
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: move goal rejected: arm is already RUNNING");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (state == tl::RunState::PAUSE)
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: move goal rejected: arm is PAUSED (disable queue motion mode first)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!acquire_motion_slot())
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: move goal rejected: another motion action is active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionActionServers::handle_move_cancel(const std::shared_ptr<GoalHandleMove>)
{
  // SDK 没有通用运动停止接口；拒绝而不是返回“取消成功”。
  RCLCPP_WARN(node_.get_logger(),
              "[Action]: move cancel rejected: controller SDK has no generic motion stop interface");
  return rclcpp_action::CancelResponse::REJECT;
}

void MotionActionServers::handle_move_accepted(const std::shared_ptr<GoalHandleMove> goal_handle, bool linear)
{
  RCLCPP_INFO(node_.get_logger(), "[Action]: move goal accepted on %s",
              linear ? "/tl_driver/move_linear" : "/tl_driver/move_joint");

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    jobs_.push_back(
        [this, goal_handle, linear]()
        {
          execute_move(goal_handle, linear);
        });
  }
  queue_cv_.notify_one();
}

void MotionActionServers::execute_move(const std::shared_ptr<GoalHandleMove> goal_handle, bool linear)
{
  if (stop_worker_.load(std::memory_order_acquire))
  {
    abort_move(goal_handle, "Driver is shutting down; goal was not started");
    return;
  }
  try
  {
    execute_move_impl(goal_handle, linear);
  }
  catch (const std::exception& e)
  {
    abort_move(goal_handle, std::string("Internal error: ") + e.what());
  }
  catch (...)
  {
    abort_move(goal_handle, "Internal error: unknown exception");
  }
}

void MotionActionServers::abort_move(const std::shared_ptr<GoalHandleMove>& goal_handle,
                                     const std::string& message) noexcept
{
  release_motion_slot();
  auto result = std::make_shared<Move::Result>();
  result->success = false;
  result->message = message;
  try
  {
    if (goal_handle->is_executing())
    {
      goal_handle->abort(result);
    }
  }
  catch (...)
  {
    // 终态发送失败不能反向抛出，否则会越过 worker 的异常边界。
  }
}

void MotionActionServers::execute_move_impl(const std::shared_ptr<GoalHandleMove>& goal_handle, bool linear)
{
  const auto goal = goal_handle->get_goal();
  const auto result = std::make_shared<Move::Result>();
  const auto abort_goal = [&](const std::string& message)
  {
    release_motion_slot();
    result->success = false;
    result->message = message;
    try
    {
      if (goal_handle->is_executing())
      {
        goal_handle->abort(result);
      }
    }
    catch (...)
    {
    }
  };

  const int fd = node_.session().fd();
  const auto max_age = std::chrono::milliseconds(3 * node_.state_stream_interval_ms());
  const auto read_stream = [&](int channel, std::vector<double>& values)
  {
    arm_client::StateFrame frame;
    if (!node_.session().is_state_frame_fresh(channel, max_age) || !node_.session().get_state_frame(channel, frame))
    {
      return false;
    }
    const std::size_t count = channel == 1 ? frame.acs_count : frame.mcs_count;
    if (count == 0)
    {
      return false;
    }
    const auto& source = channel == 1 ? frame.acs : frame.mcs;
    values.assign(source.begin(), source.begin() + count);
    return true;
  };

  std::vector<double> start_acs;
  std::vector<double> start_mcs;
  const bool acs_start_fresh = read_stream(1, start_acs);
  const bool mcs_start_fresh = read_stream(2, start_mcs);
  // 流不可用时用同步查询补起始位姿；否则位姿判据不可用。
  if (!acs_start_fresh)
  {
    sample_position(fd, tl::Coord::JOINT, start_acs);
  }
  if (!mcs_start_fresh)
  {
    sample_position(fd, tl::Coord::BASE, start_mcs);
  }

  if (stop_worker_.load(std::memory_order_acquire))
  {
    abort_goal("Driver is shutting down; motion command not sent");
    return;
  }

  tl::MoveCmd cmd =
      convert::to_move_cmd(goal->command, linear ? convert::MotionKind::kLinear : convert::MotionKind::kJoint);
  const tl::Result ret = linear ? tl::robot_movel(fd, cmd) : tl::robot_movej(fd, cmd);
  if (ret != tl::Result::SUCCESS)
  {
    abort_goal(std::string("Failed to send motion command: ") + arm_client::result_to_string(ret));
    return;
  }

  const double timeout_sec = goal->timeout_sec > 0.0 ? goal->timeout_sec : node_.motion_action_timeout();
  const auto started = std::chrono::steady_clock::now();
  auto last_feedback = started;
  auto last_sample = started - kSdkSamplePeriod; // 首轮立即采样，避免同步采样前 200 ms 无位姿
  // 保留最近一次有效位姿；单次查询失败时不能清空，否则反馈与目标判定会随机失效。
  std::vector<double> current_acs;
  std::vector<double> current_mcs;
  std::vector<double> sample;
  bool saw_running = false;
  bool saw_motion = false;
  int motion_confirm = 0;
  int inactive_confirm = 0;
  std::chrono::steady_clock::time_point stopped_since{};

  while (rclcpp::ok())
  {
    if (stop_worker_.load(std::memory_order_acquire))
    {
      abort_goal("Driver is shutting down; controller may still be moving (SDK has no stop interface)");
      return;
    }
    if (!node_.session().is_connected())
    {
      abort_goal("Arm disconnected; controller may still be moving (SDK has no stop interface)");
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool acs_fresh = read_stream(1, current_acs);
    const bool mcs_fresh = read_stream(2, current_mcs);
    if (now - last_sample >= kSdkSamplePeriod)
    {
      if (!acs_fresh && sample_position(fd, tl::Coord::JOINT, sample) && sample.size() >= 6)
      {
        current_acs = sample;
      }
      if (!mcs_fresh && sample_position(fd, tl::Coord::BASE, sample) && sample.size() >= 6)
      {
        current_mcs = sample;
      }
      last_sample = now;
    }

    tl::RunState state = tl::RunState::STOP;
    const bool state_ok = read_run_state(node_, state);

    // 位姿变化连续确认，避免单帧噪声/残留差值触发“已运动”。
    if (pose_changed(*goal, start_acs, start_mcs, current_acs, current_mcs))
    {
      if (++motion_confirm >= kMotionConfirmPolls)
      {
        saw_motion = true;
      }
    }
    else
    {
      motion_confirm = 0;
    }

    const TargetState target_state = evaluate_target(*goal, current_acs, current_mcs);

    if (state_ok && state == tl::RunState::RUNNING)
    {
      saw_running = true;
      saw_motion = true;
      inactive_confirm = 0;
      stopped_since = {};
    }
    else if (state_ok && (saw_running || target_state == TargetState::kReached))
    {
      // 控制器状态更新有约 200 ms 延迟；连续多次非 RUNNING 才判定结束。
      if (stopped_since == std::chrono::steady_clock::time_point{})
      {
        stopped_since = now;
      }
      if (++inactive_confirm >= kInactiveConfirmPolls)
      {
        if (target_state == TargetState::kReached)
        {
          result->success = true;
          result->message = std::string("Motion finished, run_state=") + run_state_text(state);
          release_motion_slot();
          try
          {
            if (goal_handle->is_executing())
            {
              goal_handle->succeed(result);
            }
          }
          catch (...)
          {
          }
          return;
        }
        // 非 RUNNING 但目标未到/不可校验：再给 2 s 收敛或补采时间，仍不满足则明确失败。
        if (now - stopped_since >= kMotionSettleTimeout)
        {
          abort_goal(target_state == TargetState::kUnknown
                         ? "Motion ended but current target pose is unavailable (run_state=" +
                               std::string(run_state_text(state)) + ")"
                         : "Motion ended before reaching target (run_state=" + std::string(run_state_text(state)) +
                               ")");
          return;
        }
      }
    }
    else
    {
      inactive_confirm = 0;
      stopped_since = {};
    }

    const auto elapsed = now - started;
    if (!saw_running && !saw_motion && elapsed >= kMotionStartTimeout)
    {
      abort_goal("Motion did not start within 3 s");
      return;
    }
    if (std::chrono::duration<double>(elapsed).count() >= timeout_sec)
    {
      abort_goal("Motion timeout after " + std::to_string(timeout_sec) +
                 " s; controller may still be moving (SDK has no stop interface)");
      return;
    }

    if (now - last_feedback >= kFeedbackPeriod)
    {
      auto feedback = std::make_shared<Move::Feedback>();
      feedback->progress = compute_progress(*goal, start_acs, start_mcs, current_acs, current_mcs);
      feedback->current_pose = make_current_pose(node_, current_mcs);
      feedback->state = state_ok ? run_state_text(state) : "UNKNOWN";
      goal_handle->publish_feedback(feedback);
      last_feedback = now;
    }

    std::this_thread::sleep_for(kPollPeriod);
  }

  abort_goal("ROS context stopped");
}

rclcpp_action::GoalResponse MotionActionServers::handle_play_goal(const rclcpp_action::GoalUUID&,
                                                                  std::shared_ptr<const PlayTrack::Goal> goal)
{
  if (!node_.session().is_connected())
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: play goal rejected: arm is not connected");
    return rclcpp_action::GoalResponse::REJECT;
  }
  const int64_t last_external_ns = last_external_motion_ns_.load(std::memory_order_acquire);
  const int64_t now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  if (last_external_ns != 0 &&
      now_ns - last_external_ns < std::chrono::duration_cast<std::chrono::nanoseconds>(kExternalMotionGrace).count())
  {
    RCLCPP_WARN(node_.get_logger(),
                "[Action]: play goal rejected: an external motion command was issued less than 500 ms ago");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->vel < 1 || goal->vel > 100)
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: play goal rejected: vel must be in [1, 100]");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!std::isfinite(goal->timeout_sec) || goal->timeout_sec > kMaxActionTimeoutSec)
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: play goal rejected: timeout_sec must be <= %.0f s",
                kMaxActionTimeoutSec);
    return rclcpp_action::GoalResponse::REJECT;
  }

  tl::RunState state = tl::RunState::STOP;
  if (!read_run_state(node_, state))
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: play goal rejected: failed to read run state");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (state == tl::RunState::RUNNING)
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: play goal rejected: arm is already RUNNING");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (state == tl::RunState::PAUSE)
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: play goal rejected: arm is PAUSED (disable queue motion mode first)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!acquire_motion_slot())
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: play goal rejected: another motion action is active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionActionServers::handle_play_cancel(const std::shared_ptr<GoalHandlePlayTrack>)
{
  RCLCPP_WARN(node_.get_logger(),
              "[Action]: play cancel rejected: controller SDK has no track playback stop interface");
  return rclcpp_action::CancelResponse::REJECT;
}

void MotionActionServers::handle_play_accepted(const std::shared_ptr<GoalHandlePlayTrack> goal_handle)
{
  RCLCPP_INFO(node_.get_logger(), "[Action]: play goal accepted on /tl_driver/play_track");
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    jobs_.push_back(
        [this, goal_handle]()
        {
          execute_play(goal_handle);
        });
  }
  queue_cv_.notify_one();
}

void MotionActionServers::execute_play(const std::shared_ptr<GoalHandlePlayTrack> goal_handle)
{
  if (stop_worker_.load(std::memory_order_acquire))
  {
    abort_play(goal_handle, "Driver is shutting down; goal was not started");
    return;
  }
  try
  {
    execute_play_impl(goal_handle);
  }
  catch (const std::exception& e)
  {
    abort_play(goal_handle, std::string("Internal error: ") + e.what());
  }
  catch (...)
  {
    abort_play(goal_handle, "Internal error: unknown exception");
  }
}

void MotionActionServers::abort_play(const std::shared_ptr<GoalHandlePlayTrack>& goal_handle,
                                     const std::string& message) noexcept
{
  release_motion_slot();
  auto result = std::make_shared<PlayTrack::Result>();
  result->success = false;
  result->message = message;
  try
  {
    if (goal_handle->is_executing())
    {
      goal_handle->abort(result);
    }
  }
  catch (...)
  {
  }
}

void MotionActionServers::execute_play_impl(const std::shared_ptr<GoalHandlePlayTrack>& goal_handle)
{
  const auto goal = goal_handle->get_goal();
  const auto result = std::make_shared<PlayTrack::Result>();
  const auto abort_goal = [&](const std::string& message)
  {
    release_motion_slot();
    result->success = false;
    result->message = message;
    try
    {
      if (goal_handle->is_executing())
      {
        goal_handle->abort(result);
      }
    }
    catch (...)
    {
    }
  };

  if (stop_worker_.load(std::memory_order_acquire))
  {
    abort_goal("Driver is shutting down; playback command not sent");
    return;
  }

  const tl::Result ret = tl::track_record_playback(node_.session().fd(), goal->vel);
  if (ret != tl::Result::SUCCESS)
  {
    abort_goal(std::string("Failed to start track playback: ") + arm_client::result_to_string(ret));
    return;
  }

  const double timeout_sec = goal->timeout_sec > 0.0 ? goal->timeout_sec : node_.track_action_timeout();
  const auto started = std::chrono::steady_clock::now();
  auto last_feedback = started;
  bool saw_running = false;
  int inactive_confirm = 0;

  while (rclcpp::ok())
  {
    if (stop_worker_.load(std::memory_order_acquire))
    {
      abort_goal("Driver is shutting down; playback may still be running (SDK has no stop interface)");
      return;
    }
    if (!node_.session().is_connected())
    {
      abort_goal("Arm disconnected; playback may still be running (SDK has no stop interface)");
      return;
    }

    tl::RunState state = tl::RunState::STOP;
    const bool state_ok = read_run_state(node_, state);
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = now - started;

    if (state_ok && state == tl::RunState::RUNNING)
    {
      saw_running = true;
      inactive_confirm = 0;
    }
    else if (state_ok && saw_running)
    {
      // 状态更新有约 200 ms 延迟，连续多次非 RUNNING 才判定结束。
      if (++inactive_confirm >= kInactiveConfirmPolls)
      {
        result->success = true;
        result->message = std::string("Track playback finished, run_state=") + run_state_text(state);
        release_motion_slot();
        try
        {
          if (goal_handle->is_executing())
          {
            goal_handle->succeed(result);
          }
        }
        catch (...)
        {
        }
        return;
      }
    }
    else
    {
      inactive_confirm = 0;
    }

    if (!saw_running && elapsed >= kMotionStartTimeout)
    {
      abort_goal("Track playback did not start within 3 s");
      return;
    }
    if (std::chrono::duration<double>(elapsed).count() >= timeout_sec)
    {
      abort_goal("Track playback timeout after " + std::to_string(timeout_sec) +
                 " s; playback may still be running (SDK has no stop interface)");
      return;
    }

    if (now - last_feedback >= kFeedbackPeriod)
    {
      auto feedback = std::make_shared<PlayTrack::Feedback>();
      feedback->progress = -1.0; // 控制器不提供回放进度
      feedback->state = state_ok ? run_state_text(state) : "UNKNOWN";
      goal_handle->publish_feedback(feedback);
      last_feedback = now;
    }

    std::this_thread::sleep_for(kPollPeriod);
  }

  abort_goal("ROS context stopped");
}

void MotionActionServers::worker_loop()
{
  for (;;)
  {
    std::function<void()> job;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock,
                     [this]()
                     {
                       return stop_worker_.load(std::memory_order_acquire) || !jobs_.empty();
                     });
      if (jobs_.empty())
      {
        if (stop_worker_.load(std::memory_order_acquire))
        {
          return;
        }
        continue;
      }
      job = std::move(jobs_.front());
      jobs_.pop_front();
    }
    job();
  }
}
} // namespace tl_driver
