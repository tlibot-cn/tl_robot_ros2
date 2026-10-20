/// @file arm_client.cpp
/// @brief `ArmClient` 的实现：回调转发、状态帧缓冲与日志节流。

#include "arm_client.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

#include <rclcpp/logging.hpp>

#include "arm_sdk.hpp"

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

uint64_t ArmClient::robot_state_seq() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return seq_;
}

bool ArmClient::wait_robot_state(uint64_t start_seq, std::chrono::milliseconds timeout, std::string& message)
{
  std::unique_lock<std::mutex> lock(mutex_);
  const bool received = cv_.wait_for(lock, timeout,
                                     [this, start_seq]()
                                     {
                                       return seq_ > start_seq;
                                     });
  if (!received)
  {
    return false;
  }

  message = last_msg_;
  return true;
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
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_msg_ = state != nullptr ? state : "";
    ++seq_;
  }
  cv_.notify_all();

  if (should_log(last_state_log_ns_, std::chrono::seconds(1)))
  {
    RCLCPP_INFO(logger_, "[SDK]: robot_state = %s", state != nullptr ? state : "");
  }
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
