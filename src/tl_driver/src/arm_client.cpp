/// @file arm_client.cpp
/// @brief `ArmClient` 的实现：回调转发、双通道状态快照与日志节流。

#include "arm_client.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

#include <rclcpp/logging.hpp>

#include "arm_sdk.hpp"
#include "convert/state_frame.hpp"

namespace tl_driver::arm_client
{
std::mutex ArmClient::registry_mutex_;
ArmClient *ArmClient::active_ = nullptr;

const char *result_to_string(tl::Result ret) noexcept
{
  switch (ret)
  {
    case tl::Result::SUCCESS:
      return "SUCCESS";
    case tl::Result::RECEIVE_FAILED:
      return "RECEIVE_FAILED";
    case tl::Result::DISCONNECT:
      return "DISCONNECT";
    case tl::Result::PARAM_ERR:
      return "PARAM_ERR";
    case tl::Result::OPERATION_NOT_ALLOWED:
      return "OPERATION_NOT_ALLOWED";
    case tl::Result::EXCEPTION:
      return "EXCEPTION";
    case tl::Result::TIMEOUT:
      return "TIMEOUT";
    default:
      return "UNKNOWN_ERROR";
  }
}

ArmClient::ArmClient(rclcpp::Logger logger) : logger_(std::move(logger))
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  if (active_ != nullptr)
  {
    throw std::logic_error("ArmClient is process-wide singleton; only one instance is allowed");
  }
  active_ = this;
}

ArmClient::~ArmClient()
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  if (active_ == this)
  {
    active_ = nullptr;
  }
}

void ArmClient::register_error_callback(int fd)
{
  const tl::Result ret = tl::set_receive_error_or_warnning_message_callback(fd, &ArmClient::error_callback);
  if (ret != tl::Result::SUCCESS)
  {
    RCLCPP_WARN(logger_, "[SDK]: failed to register error callback on fd %d, result=%s", fd, result_to_string(ret));
  }
}

void ArmClient::start_robot_state_channel(int fd_aux)
{
  const tl::Result ret = tl::robot_state_callback(fd_aux, &ArmClient::state_callback);
  if (ret != tl::Result::SUCCESS)
  {
    RCLCPP_WARN(logger_, "[SDK]: failed to start robot state channel on fd %d, result=%s", fd_aux,
                result_to_string(ret));
  }
}

bool ArmClient::request_state_stream(int fd_aux, int channel, int position, int interval_ms)
{
  tl::RobotState param{};
  param.channel = channel;
  param.position = position;
  param.mode = 1; // 连续反馈
  param.interval = interval_ms;
  param.stop = false;

  const tl::Result ret = tl::get_robot_state(fd_aux, param);
  if (ret != tl::Result::SUCCESS)
  {
    RCLCPP_WARN_THROTTLE(logger_, clock_, 1000, "[SDK]: channel %d stream request failed, result=%s", channel,
                         result_to_string(ret));
    return false;
  }
  return true;
}

bool ArmClient::stop_state_stream(int fd_aux, int channel, int position)
{
  tl::RobotState param{};
  param.channel = channel;
  param.position = position;
  param.mode = 1;
  param.stop = true;

  const tl::Result ret = tl::get_robot_state(fd_aux, param);
  if (ret != tl::Result::SUCCESS)
  {
    RCLCPP_WARN(logger_, "[SDK]: channel %d stream stop failed, result=%s", channel, result_to_string(ret));
    return false;
  }
  return true;
}

bool ArmClient::get_state_frame(int channel, StateFrame& frame) const
{
  if (channel < 1 || channel >= static_cast<int>(frames_.size()))
  {
    return false;
  }

  std::lock_guard<std::mutex> lock(frame_mutex_);
  const StateFrame& snapshot = frames_[channel];
  if (snapshot.seq == 0)
  {
    return false;
  }

  // 通道 1 只承载 ACS，通道 2 只承载 MCS（§14.2）。
  frame = snapshot;
  return true;
}

bool ArmClient::is_state_frame_fresh(int channel, std::chrono::milliseconds max_age) const
{
  if (channel < 1 || channel >= static_cast<int>(frames_.size()))
  {
    return false;
  }

  std::lock_guard<std::mutex> lock(frame_mutex_);
  const StateFrame& frame = frames_[channel];
  if (frame.seq == 0)
  {
    return false;
  }
  return std::chrono::steady_clock::now() - frame.received_at <= max_age;
}

bool ArmClient::wait_state_update(uint64_t& seen, std::chrono::milliseconds timeout) const
{
  std::unique_lock<std::mutex> lock(frame_mutex_);
  // 谓词比较通知序号，避免“写入发生在等待之前”导致发布线程睡到超时。
  const bool updated = state_cv_.wait_for(lock, timeout,
                                          [this, seen]
                                          {
                                            return state_notify_seq_ != seen;
                                          });
  seen = state_notify_seq_;
  return updated;
}

void ArmClient::error_callback(int message_type, const char *message, int message_code)
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  if (active_ != nullptr)
  {
    active_->on_error(message_type, message, message_code);
  }
}

void ArmClient::state_callback(const char *state)
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  if (active_ != nullptr)
  {
    active_->on_state(state);
  }
}

void ArmClient::on_error(int message_type, const char *message, int message_code)
{
  if (!should_log(last_error_log_ns_, std::chrono::seconds(1)))
  {
    return;
  }
  RCLCPP_WARN(logger_, "[SDK]: messageType=%d, message=%s, messageCode=%d", message_type, message ? message : "",
              message_code);
}

void ArmClient::on_state(const char *state)
{
  convert::RobotStateFrameData parsed;
  if (state == nullptr || !convert::parse_robot_state_frame(state, parsed))
  {
    // 非状态报文或未知布局：回调线程不得抛异常；节流记录避免刷屏。
    if (should_log(last_state_log_ns_, std::chrono::seconds(1)))
    {
      RCLCPP_WARN(logger_, "[SDK]: unparsable robot state frame: %s", state != nullptr ? state : "");
    }
    return;
  }

  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (parsed.channel < 1 || parsed.channel >= static_cast<int>(frames_.size()))
    {
      // 解析层已限制 channel ∈ [1, 9]，此处防御两个模块常量不同步。
      return;
    }
    StateFrame& frame = frames_[parsed.channel];
    frame.acs = parsed.acs;
    frame.mcs = parsed.mcs;
    frame.acs_count = parsed.acs_count;
    frame.mcs_count = parsed.mcs_count;
    frame.received_at = std::chrono::steady_clock::now();
    ++frame.seq;
    ++state_notify_seq_;
  }
  // 通知放在锁外：发布线程被唤醒后会重新取锁读快照，避免持锁唤醒造成的伪竞争。
  state_cv_.notify_one();
  update_frame_stats(parsed.channel);
}

void ArmClient::update_frame_stats(int channel)
{
  if (channel < 1 || channel >= static_cast<int>(channel_stats_.size()))
  {
    return;
  }

  ChannelStats& stats = channel_stats_[channel];
  stats.frames.fetch_add(1, std::memory_order_relaxed);

  constexpr int64_t kWindowNs = 5'000'000'000LL;
  const int64_t now_ns = clock_.now().nanoseconds();
  int64_t window_start = stats.window_start_ns.load(std::memory_order_relaxed);
  if (window_start == 0)
  {
    stats.window_start_ns.compare_exchange_strong(window_start, now_ns);
    return;
  }
  if (now_ns - window_start < kWindowNs)
  {
    return;
  }
  // 只有一个回调线程累计计数；CAS 失败说明窗口刚被别的线程切换，直接跳过本次统计。
  if (!stats.window_start_ns.compare_exchange_strong(window_start, now_ns))
  {
    return;
  }

  const uint64_t frames = stats.frames.exchange(0, std::memory_order_relaxed);
  const double elapsed_s = static_cast<double>(now_ns - window_start) / 1e9;
  RCLCPP_DEBUG(logger_, "[SDK]: channel %d state stream %.1f fps", channel, static_cast<double>(frames) / elapsed_s);
}

bool ArmClient::should_log(std::atomic<int64_t>& last_ns, std::chrono::milliseconds period)
{
  const int64_t now = clock_.now().nanoseconds();
  int64_t last = last_ns.load(std::memory_order_relaxed);
  if (now - last < std::chrono::duration_cast<std::chrono::nanoseconds>(period).count())
  {
    return false;
  }
  return last_ns.compare_exchange_strong(last, now, std::memory_order_relaxed);
}
} // namespace tl_driver::arm_client
