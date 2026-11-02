/// @file actions_motion.cpp
/// @brief 运动/轨迹回放 action 的实现。

#include "node/actions_motion.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

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
constexpr std::size_t kCartesianDims = 3; // 位置判据只比较 x/y/z

/// @brief steady_clock 当前时刻（ns），用于外部运动拒绝窗口。
int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

bool read_run_state(const DriverNode& node, tl::RunState& state)
{
  if (!node.session().is_connected())
  {
    return false;
  }
  return tl::get_robot_running_state(node.session().fd(), state) == tl::Result::SUCCESS;
}

/// @brief 发送 goal 终态；发送失败不能反向抛出，否则会越过工作线程的异常边界。
template <typename GoalHandleT, typename ResultT>
void finish_goal(const std::shared_ptr<GoalHandleT>& goal_handle, const std::shared_ptr<ResultT>& result, bool success,
                 std::string message) noexcept
{
  result->success = success;
  result->message = std::move(message);
  try
  {
    if (!goal_handle->is_executing())
    {
      return;
    }
    if (success)
    {
      goal_handle->succeed(result);
    }
    else
    {
      goal_handle->abort(result);
    }
  }
  catch (...)
  {
  }
}

/// @brief 工作线程异常边界：实现抛异常时转为 abort，绝不让异常逃出 worker_loop。
template <typename ExecuteFn, typename AbortFn>
void run_guarded(ExecuteFn&& execute, AbortFn&& abort_fn)
{
  try
  {
    execute();
  }
  catch (const std::exception& e)
  {
    abort_fn(std::string("Internal error: ") + e.what());
  }
  catch (...)
  {
    abort_fn("Internal error: unknown exception");
  }
}

/// @brief 三个 action server 共用的创建壳：Humble 的 create_server 需要逐个传 Node 接口。
template <typename ActionT, typename GoalCb, typename CancelCb, typename AcceptedCb>
typename rclcpp_action::Server<ActionT>::SharedPtr create_action_server(DriverNode& node, const char *name,
                                                                        GoalCb goal_callback, CancelCb cancel_callback,
                                                                        AcceptedCb accepted_callback,
                                                                        const rcl_action_server_options_t& options,
                                                                        const rclcpp::CallbackGroup::SharedPtr& group)
{
  return rclcpp_action::create_server<ActionT>(node.get_node_base_interface(), node.get_node_clock_interface(),
                                               node.get_node_logging_interface(), node.get_node_waitables_interface(),
                                               name, std::move(goal_callback), std::move(cancel_callback),
                                               std::move(accepted_callback), options, group);
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

/// @brief 目标位姿，已换算成与状态采样一致的比较域：coord=0 为关节角（度），coord=1 为位置（mm）。
struct MotionTarget
{
  int coord = -1;
  std::vector<double> values;
};

/// @brief 目标换算到采样域，避免在每轮判据里重复 rad→度 / m→mm。
MotionTarget make_target(const tl_ros2_interface::action::Move::Goal& goal)
{
  MotionTarget target;
  target.coord = goal.command.coord;
  const auto& source = goal.command.target_pos_value;
  if (target.coord == 0)
  {
    target.values.reserve(source.size());
    for (const double rad : source)
    {
      target.values.push_back(convert::rad_to_deg(rad));
    }
  }
  else if (target.coord == 1 && source.size() >= kCartesianDims)
  {
    target.values.reserve(kCartesianDims);
    for (std::size_t i = 0; i < kCartesianDims; ++i)
    {
      target.values.push_back(convert::m_to_mm(source[i]));
    }
  }
  return target;
}

/// @brief 前 count 个分量绝对差之和（关节进度判据）。
double sum_abs_delta(const std::vector<double>& a, const std::vector<double>& b, std::size_t count)
{
  double total = 0.0;
  for (std::size_t i = 0; i < count; ++i)
  {
    total += std::abs(a[i] - b[i]);
  }
  return total;
}

/// @brief 前 count 个分量绝对差的最大值（“已运动/已到达”判据，单关节超差即不满足）。
double max_abs_delta(const std::vector<double>& a, const std::vector<double>& b, std::size_t count)
{
  double worst = 0.0;
  for (std::size_t i = 0; i < count; ++i)
  {
    worst = std::max(worst, std::abs(a[i] - b[i]));
  }
  return worst;
}

/// @brief 前 3 个分量（mm）的欧氏距离。
double distance_mm(const std::vector<double>& a, const std::vector<double>& b)
{
  return std::hypot(std::hypot(a[0] - b[0], a[1] - b[1]), a[2] - b[2]);
}

/// @brief 按坐标系估算完成度；无法估算时返回 -1。
double compute_progress(const MotionTarget& target, const std::vector<double>& start,
                        const std::vector<double>& current)
{
  if (target.coord == 0)
  {
    const std::size_t count = start.size();
    if (count == 0 || current.size() < count || target.values.size() < count)
    {
      return -1.0;
    }
    const double initial = sum_abs_delta(target.values, start, count);
    if (initial < kJointMoveEpsilonDeg)
    {
      return 1.0;
    }
    return std::clamp(1.0 - sum_abs_delta(target.values, current, count) / initial, 0.0, 1.0);
  }

  if (target.coord == 1)
  {
    if (start.size() < kCartesianDims || current.size() < kCartesianDims || target.values.size() < kCartesianDims)
    {
      return -1.0;
    }
    const double initial = distance_mm(target.values, start);
    if (initial < kCartesianMoveEpsilonMm)
    {
      return 1.0;
    }
    return std::clamp(1.0 - distance_mm(target.values, current) / initial, 0.0, 1.0);
  }

  return -1.0;
}

/// @brief 是否已离开起始位姿；实际判定由调用方连续确认，以滤掉单帧噪声。
bool pose_changed(const MotionTarget& target, const std::vector<double>& start, const std::vector<double>& current)
{
  if (target.coord == 0)
  {
    const std::size_t count = std::min(start.size(), current.size());
    return count > 0 && convert::deg_to_rad(max_abs_delta(start, current, count)) > kJointMoveEpsilonRad;
  }

  if (target.coord == 1)
  {
    if (start.size() < kCartesianDims || current.size() < kCartesianDims)
    {
      return false;
    }
    return convert::mm_to_m(distance_mm(start, current)) > kCartesianMoveEpsilonM;
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

TargetState evaluate_target(const MotionTarget& target, const std::vector<double>& current)
{
  if (target.coord == 0)
  {
    const std::size_t count = std::min(target.values.size(), current.size());
    if (count == 0)
    {
      return TargetState::kUnknown;
    }
    const bool reached = max_abs_delta(target.values, current, count) <= kJointReachEpsilonDeg;
    return reached ? TargetState::kReached : TargetState::kNotReached;
  }

  if (target.coord == 1)
  {
    if (target.values.size() < kCartesianDims || current.size() < kCartesianDims)
    {
      return TargetState::kUnknown;
    }
    const bool reached = distance_mm(target.values, current) <= kCartesianReachEpsilonMm;
    return reached ? TargetState::kReached : TargetState::kNotReached;
  }

  return TargetState::kUnknown;
}

/// @brief 轮询期的运行状态判定：封装“连续非 RUNNING 才判结束”的确认与两个超时窗口。
class RunStateWatcher
{
public:
  /// @brief 单轮判定；字段互相独立，调用方按原有优先级取用（结束优先于超时）。
  struct Status
  {
    bool ended = false;       // 连续多次非 RUNNING，判定控制器已结束
    bool not_started = false; // 起始窗口内未进入 RUNNING 且未观察到位姿变化
    bool timed_out = false;   // 总超时
  };

  explicit RunStateWatcher(double timeout_sec) : timeout_sec_(timeout_sec) {}

  /// @brief 已观察到位姿变化，可豁免起始超时。
  void mark_motion_observed() noexcept
  {
    motion_observed_ = true;
  }

  /// @param[in] state_ok 本轮状态查询是否成功
  /// @param[in] state 控制器运行状态
  /// @param[in] target_reached 目标位姿是否已到达（短运动可能在首次轮询前就结束）
  /// @param[in] now 本轮时刻
  Status update(bool state_ok, tl::RunState state, bool target_reached, std::chrono::steady_clock::time_point now)
  {
    if (state_ok && state == tl::RunState::RUNNING)
    {
      saw_running_ = true;
      motion_observed_ = true;
      inactive_confirm_ = 0;
      stopped_since_ = {};
    }
    else if (state_ok && (saw_running_ || target_reached))
    {
      // 控制器状态更新有约 200 ms 延迟；连续多次非 RUNNING 才判定结束。
      if (stopped_since_ == std::chrono::steady_clock::time_point{})
      {
        stopped_since_ = now;
      }
      ++inactive_confirm_;
    }
    else
    {
      inactive_confirm_ = 0;
      stopped_since_ = {};
    }

    Status status;
    status.ended = state_ok && (saw_running_ || target_reached) && inactive_confirm_ >= kInactiveConfirmPolls;
    const double elapsed = std::chrono::duration<double>(now - started_).count();
    status.not_started =
        !saw_running_ && !motion_observed_ && elapsed >= std::chrono::duration<double>(kMotionStartTimeout).count();
    status.timed_out = elapsed >= timeout_sec_;
    return status;
  }

  /// @brief 首次判定非 RUNNING 的时刻；结束确认后用于等待目标收敛。
  std::chrono::steady_clock::time_point stopped_since() const noexcept
  {
    return stopped_since_;
  }

private:
  double timeout_sec_;
  std::chrono::steady_clock::time_point started_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point stopped_since_{};
  bool saw_running_ = false;
  bool motion_observed_ = false;
  int inactive_confirm_ = 0;
};

/// @brief 校验 MoveCommand 的取值；合法时返回 nullptr，否则返回拒绝原因。
const char *validate_move_command(const tl_ros2_interface::msg::MoveCommand& command, bool linear)
{
  for (const double value : command.target_pos_value)
  {
    if (!std::isfinite(value))
    {
      return "target_pos_value contains non-finite value";
    }
  }
  if ((linear && command.coord != 1) || (!linear && command.coord != 0))
  {
    return "move_joint requires coord=0, move_linear requires coord=1";
  }
  if (!std::isfinite(command.velocity) || !std::isfinite(command.acc) || !std::isfinite(command.dec) ||
      command.velocity <= 0.0 || command.acc <= 0.0 || command.acc > 100.0 || command.dec <= 0.0 || command.dec > 100.0)
  {
    return "invalid velocity/acc/dec";
  }
  // MOVJ 速度单位为 %，MOVL 为 m/s（MOVL 上限 1 m/s）。
  if (linear ? command.velocity > 1.0 : command.velocity > 100.0)
  {
    return "velocity out of range";
  }
  if (!std::isfinite(command.velocity_sync) || command.velocity_sync < 0.0 ||
      (linear ? command.velocity_sync > 1.0 : command.velocity_sync > 100.0))
  {
    return "velocity_sync out of range";
  }
  if (command.pl < 0 || command.pl > 5 || command.tool_num < 0 || command.user_num < 0)
  {
    return "invalid pl/tool_num/user_num";
  }
  return nullptr;
}

} // namespace

MotionActionServers::MotionActionServers(DriverNode& node, const rclcpp::CallbackGroup::SharedPtr& group) : node_(node)
{
  // Humble 的 create_server 接收 rcl_action_server_options_t 与独立回调组参数。
  const rcl_action_server_options_t options = rcl_action_server_get_default_options();

  const auto move_goal = [this](bool linear)
  {
    return [this, linear](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Move::Goal> goal)
    {
      return handle_move_goal(uuid, std::move(goal), linear);
    };
  };
  const auto move_cancel = [this](const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    return handle_move_cancel(goal_handle);
  };
  const auto move_accepted = [this](bool linear)
  {
    return [this, linear](const std::shared_ptr<GoalHandleMove> goal_handle)
    {
      handle_move_accepted(goal_handle, linear);
    };
  };
  const auto play_goal = [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const PlayTrack::Goal> goal)
  {
    return handle_play_goal(uuid, std::move(goal));
  };
  const auto play_cancel = [this](const std::shared_ptr<GoalHandlePlayTrack> goal_handle)
  {
    return handle_play_cancel(goal_handle);
  };
  const auto play_accepted = [this](const std::shared_ptr<GoalHandlePlayTrack> goal_handle)
  {
    handle_play_accepted(goal_handle);
  };

  move_joint_server_ = create_action_server<Move>(node_, "/tl_driver/move_joint", move_goal(false), move_cancel,
                                                  move_accepted(false), options, group);
  move_linear_server_ = create_action_server<Move>(node_, "/tl_driver/move_linear", move_goal(true), move_cancel,
                                                   move_accepted(true), options, group);
  play_track_server_ = create_action_server<PlayTrack>(node_, "/tl_driver/play_track", play_goal, play_cancel,
                                                       play_accepted, options, group);

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
  last_external_motion_ns_.store(steady_now_ns(), std::memory_order_release);
}

bool MotionActionServers::external_motion_recent() const
{
  const int64_t last_ns = last_external_motion_ns_.load(std::memory_order_acquire);
  if (last_ns == 0)
  {
    return false;
  }
  const int64_t grace_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(kExternalMotionGrace).count();
  return steady_now_ns() - last_ns < grace_ns;
}

const char *MotionActionServers::abort_cause() const
{
  if (stop_worker_.load(std::memory_order_acquire))
  {
    return "Driver is shutting down";
  }
  if (!node_.session().is_connected())
  {
    return "Arm disconnected";
  }
  return nullptr;
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

rclcpp_action::GoalResponse MotionActionServers::precheck_goal(double timeout_sec, const char *action_name,
                                                               const char *field_error)
{
  if (!node_.session().is_connected())
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: %s goal rejected: arm is not connected", action_name);
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (external_motion_recent())
  {
    RCLCPP_WARN(node_.get_logger(),
                "[Action]: %s goal rejected: an external motion command was issued less than %ld ms ago", action_name,
                static_cast<long>(kExternalMotionGrace.count()));
    return rclcpp_action::GoalResponse::REJECT;
  }

  // 字段校验放在连接/外部运动之后：保持现场排障时首个 WARN 仍是根因。
  if (field_error != nullptr)
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: %s goal rejected: %s", action_name, field_error);
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!std::isfinite(timeout_sec) || timeout_sec > kMaxActionTimeoutSec)
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: %s goal rejected: timeout_sec must be <= %.0f s", action_name,
                kMaxActionTimeoutSec);
    return rclcpp_action::GoalResponse::REJECT;
  }

  tl::RunState state = tl::RunState::STOP;
  if (!read_run_state(node_, state))
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: %s goal rejected: failed to read run state", action_name);
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (state == tl::RunState::RUNNING)
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: %s goal rejected: arm is already RUNNING", action_name);
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (state == tl::RunState::PAUSE)
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: %s goal rejected: arm is PAUSED (disable queue motion mode first)",
                action_name);
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!acquire_motion_slot())
  {
    RCLCPP_WARN(node_.get_logger(), "[Action]: %s goal rejected: another motion action is active", action_name);
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse MotionActionServers::handle_move_goal(const rclcpp_action::GoalUUID&,
                                                                  std::shared_ptr<const Move::Goal> goal, bool linear)
{
  return precheck_goal(goal->timeout_sec, "move", validate_move_command(goal->command, linear));
}

rclcpp_action::CancelResponse MotionActionServers::handle_move_cancel(const std::shared_ptr<GoalHandleMove>)
{
  // SDK 没有通用运动停止接口；拒绝而不是返回“取消成功”。
  RCLCPP_WARN(node_.get_logger(),
              "[Action]: move cancel rejected: controller SDK has no generic motion stop interface");
  return rclcpp_action::CancelResponse::REJECT;
}

void MotionActionServers::enqueue(std::function<void()> job)
{
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    jobs_.push_back(std::move(job));
  }
  queue_cv_.notify_one();
}

void MotionActionServers::handle_move_accepted(const std::shared_ptr<GoalHandleMove> goal_handle, bool linear)
{
  RCLCPP_INFO(node_.get_logger(), "[Action]: move goal accepted on %s",
              linear ? "/tl_driver/move_linear" : "/tl_driver/move_joint");
  enqueue(
      [this, goal_handle, linear]()
      {
        execute_move(goal_handle, linear);
      });
}

void MotionActionServers::execute_move(const std::shared_ptr<GoalHandleMove> goal_handle, bool linear)
{
  if (stop_worker_.load(std::memory_order_acquire))
  {
    abort_move(goal_handle, "Driver is shutting down; goal was not started");
    return;
  }
  run_guarded(
      [this, &goal_handle, linear]()
      {
        execute_move_impl(goal_handle, linear);
      },
      [this, &goal_handle](const std::string& message)
      {
        abort_move(goal_handle, message);
      });
}

void MotionActionServers::abort_move(const std::shared_ptr<GoalHandleMove>& goal_handle,
                                     const std::string& message) noexcept
{
  release_motion_slot();
  finish_goal(goal_handle, std::make_shared<Move::Result>(), false, message);
}

void MotionActionServers::execute_move_impl(const std::shared_ptr<GoalHandleMove>& goal_handle, bool linear)
{
  const auto goal = goal_handle->get_goal();
  const auto result = std::make_shared<Move::Result>();
  const auto finish = [&](bool success, const std::string& message)
  {
    release_motion_slot();
    finish_goal(goal_handle, result, success, message);
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
    finish(false, "Driver is shutting down; motion command not sent");
    return;
  }

  tl::MoveCmd cmd =
      convert::to_move_cmd(goal->command, linear ? convert::MotionKind::kLinear : convert::MotionKind::kJoint);
  const tl::Result ret = linear ? tl::robot_movel(fd, cmd) : tl::robot_movej(fd, cmd);
  if (ret != tl::Result::SUCCESS)
  {
    finish(false, std::string("Failed to send motion command: ") + arm_client::result_to_string(ret));
    return;
  }

  const MotionTarget target = make_target(*goal);
  const bool joint = target.coord == 0;
  const std::vector<double>& start_sample = joint ? start_acs : start_mcs;
  const double timeout_sec = goal->timeout_sec > 0.0 ? goal->timeout_sec : node_.motion_action_timeout();
  RunStateWatcher watcher(timeout_sec);
  auto last_feedback = std::chrono::steady_clock::now();
  auto last_sample = last_feedback - kSdkSamplePeriod; // 首轮立即采样，避免同步采样前 200 ms 无位姿
  // 保留最近一次有效位姿；单次查询失败时不能清空，否则反馈与目标判定会随机失效。
  std::vector<double> current_acs;
  std::vector<double> current_mcs;
  std::vector<double> sample;
  int motion_confirm = 0;

  while (rclcpp::ok())
  {
    if (const char *cause = abort_cause())
    {
      finish(false, std::string(cause) + "; controller may still be moving (SDK has no stop interface)");
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
    const std::vector<double>& current_sample = joint ? current_acs : current_mcs;

    tl::RunState state = tl::RunState::STOP;
    const bool state_ok = read_run_state(node_, state);

    // 位姿变化连续确认，避免单帧噪声/残留差值触发“已运动”。
    if (pose_changed(target, start_sample, current_sample))
    {
      if (++motion_confirm >= kMotionConfirmPolls)
      {
        watcher.mark_motion_observed();
      }
    }
    else
    {
      motion_confirm = 0;
    }

    const TargetState target_state = evaluate_target(target, current_sample);
    const RunStateWatcher::Status status = watcher.update(state_ok, state, target_state == TargetState::kReached, now);

    if (status.ended)
    {
      if (target_state == TargetState::kReached)
      {
        finish(true, std::string("Motion finished, run_state=") + run_state_text(state));
        return;
      }
      // 非 RUNNING 但目标未到/不可校验：再给 2 s 收敛或补采时间，仍不满足则明确失败。
      if (now - watcher.stopped_since() >= kMotionSettleTimeout)
      {
        finish(false,
               target_state == TargetState::kUnknown
                   ? "Motion ended but current target pose is unavailable (run_state=" +
                         std::string(run_state_text(state)) + ")"
                   : "Motion ended before reaching target (run_state=" + std::string(run_state_text(state)) + ")");
        return;
      }
    }

    if (status.not_started)
    {
      finish(false, "Motion did not start within 3 s");
      return;
    }
    if (status.timed_out)
    {
      finish(false, "Motion timeout after " + std::to_string(timeout_sec) +
                        " s; controller may still be moving (SDK has no stop interface)");
      return;
    }

    if (now - last_feedback >= kFeedbackPeriod)
    {
      auto feedback = std::make_shared<Move::Feedback>();
      feedback->progress = compute_progress(target, start_sample, current_sample);
      feedback->current_pose = make_current_pose(node_, current_mcs);
      feedback->state = state_ok ? run_state_text(state) : "UNKNOWN";
      goal_handle->publish_feedback(feedback);
      last_feedback = now;
    }

    std::this_thread::sleep_for(kPollPeriod);
  }

  finish(false, "ROS context stopped");
}

rclcpp_action::GoalResponse MotionActionServers::handle_play_goal(const rclcpp_action::GoalUUID&,
                                                                  std::shared_ptr<const PlayTrack::Goal> goal)
{
  const char *field_error = (goal->vel < 1 || goal->vel > 100) ? "vel must be in [1, 100]" : nullptr;
  return precheck_goal(goal->timeout_sec, "play", field_error);
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
  enqueue(
      [this, goal_handle]()
      {
        execute_play(goal_handle);
      });
}

void MotionActionServers::execute_play(const std::shared_ptr<GoalHandlePlayTrack> goal_handle)
{
  if (stop_worker_.load(std::memory_order_acquire))
  {
    abort_play(goal_handle, "Driver is shutting down; goal was not started");
    return;
  }
  run_guarded(
      [this, &goal_handle]()
      {
        execute_play_impl(goal_handle);
      },
      [this, &goal_handle](const std::string& message)
      {
        abort_play(goal_handle, message);
      });
}

void MotionActionServers::abort_play(const std::shared_ptr<GoalHandlePlayTrack>& goal_handle,
                                     const std::string& message) noexcept
{
  release_motion_slot();
  finish_goal(goal_handle, std::make_shared<PlayTrack::Result>(), false, message);
}

void MotionActionServers::execute_play_impl(const std::shared_ptr<GoalHandlePlayTrack>& goal_handle)
{
  const auto goal = goal_handle->get_goal();
  const auto result = std::make_shared<PlayTrack::Result>();
  const auto finish = [&](bool success, const std::string& message)
  {
    release_motion_slot();
    finish_goal(goal_handle, result, success, message);
  };

  if (stop_worker_.load(std::memory_order_acquire))
  {
    finish(false, "Driver is shutting down; playback command not sent");
    return;
  }

  const tl::Result ret = tl::track_record_playback(node_.session().fd(), goal->vel);
  if (ret != tl::Result::SUCCESS)
  {
    finish(false, std::string("Failed to start track playback: ") + arm_client::result_to_string(ret));
    return;
  }

  const double timeout_sec = goal->timeout_sec > 0.0 ? goal->timeout_sec : node_.track_action_timeout();
  RunStateWatcher watcher(timeout_sec);
  auto last_feedback = std::chrono::steady_clock::now();

  while (rclcpp::ok())
  {
    if (const char *cause = abort_cause())
    {
      finish(false, std::string(cause) + "; playback may still be running (SDK has no stop interface)");
      return;
    }

    tl::RunState state = tl::RunState::STOP;
    const bool state_ok = read_run_state(node_, state);
    const auto now = std::chrono::steady_clock::now();
    const RunStateWatcher::Status status = watcher.update(state_ok, state, false, now);

    if (status.ended)
    {
      finish(true, std::string("Track playback finished, run_state=") + run_state_text(state));
      return;
    }
    if (status.not_started)
    {
      finish(false, "Track playback did not start within 3 s");
      return;
    }
    if (status.timed_out)
    {
      finish(false, "Track playback timeout after " + std::to_string(timeout_sec) +
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

  finish(false, "ROS context stopped");
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
