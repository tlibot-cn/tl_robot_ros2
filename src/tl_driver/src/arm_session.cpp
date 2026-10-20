/// @file arm_session.cpp
/// @brief ArmSession 的实现（与拆分前的同名成员实现等价）。

#include "arm_session.hpp"

#include <cassert>
#include <chrono>
#include <thread>

#include "arm_client.hpp"
#include "arm_sdk.hpp"

namespace tl_driver
{
ArmSession::ArmSession(rclcpp::Logger logger) : logger_(logger) {}

int ArmSession::fd() const
{
  const int fd = socket_fd_.load(std::memory_order_acquire);
  assert(fd > 0 && "ArmSession::fd() called without a live session");
  return fd;
}

int ArmSession::fd_aux() const
{
  const int fd = socket_fd_aux_.load(std::memory_order_acquire);
  assert(fd > 0 && "ArmSession::fd_aux() called without a live session");
  return fd;
}

bool ArmSession::is_connected() const
{
  return is_connected_.load(std::memory_order_acquire) && socket_fd_.load(std::memory_order_acquire) > 0 &&
         socket_fd_aux_.load(std::memory_order_acquire) > 0;
}

bool ArmSession::is_powered() const
{
  return is_powered_.load(std::memory_order_acquire);
}

bool ArmSession::power_on()
{
  const int fd = this->fd();
  tl::ServoState state = static_cast<tl::ServoState>(-1);
  tl::Result ret = tl::get_servo_state(fd, state);
  if (ret != tl::Result::SUCCESS)
  {
    RCLCPP_ERROR(logger_, "[PowerOn]: failed to read servo state, ret=%s", arm_client::result_to_string(ret));
    return false;
  }

  switch (state)
  {
    case tl::ServoState::STOP:
      tl::set_servo_state(fd, tl::ServoState::READY);
      RCLCPP_INFO(logger_, "[PowerOn]: waiting 2s for servo ready...");
      std::this_thread::sleep_for(std::chrono::seconds(2));
      tl::set_servo_poweron(fd);
      break;
    case tl::ServoState::READY:
      tl::set_servo_poweron(fd);
      break;
    case tl::ServoState::ALARM:
      RCLCPP_WARN(logger_, "[PowerOn]: servo alarm state (2), clearing error first");
      tl::clear_error(fd);
      tl::set_servo_state(fd, tl::ServoState::READY);
      tl::set_servo_poweron(fd);
      break;
    case tl::ServoState::RUNNING:
      RCLCPP_INFO(logger_, "[PowerOn]: servo_state=3, re-executing power on");
      tl::set_servo_state(fd, tl::ServoState::READY);
      tl::set_servo_poweron(fd);
      break;

    default:
      RCLCPP_ERROR(logger_, "[PowerOn]: unknown servo state %d", static_cast<int>(state));
      return false;
  }

  ret = tl::get_servo_state(fd, state);
  if (ret != tl::Result::SUCCESS)
  {
    RCLCPP_ERROR(logger_, "[PowerOn]: failed to read servo state after power on, ret=%s",
                 arm_client::result_to_string(ret));
    return false;
  }
  RCLCPP_INFO(logger_, "[PowerOn]: servo_state = %d", static_cast<int>(state));
  if (state == tl::ServoState::RUNNING)
  {
    is_powered_.store(true, std::memory_order_release);
    return true;
  }
  else
  {
    RCLCPP_ERROR(logger_, "[PowerOn]: failed to power on, servo_state = %d", static_cast<int>(state));
    return false;
  }
}

bool ArmSession::power_off()
{
  const int fd = this->fd();
  tl::ServoState state = static_cast<tl::ServoState>(-1);
  tl::Result ret = tl::get_servo_state(fd, state);
  if (ret != tl::Result::SUCCESS)
  {
    RCLCPP_ERROR(logger_, "[PowerOff]: failed to read servo state, ret=%s", arm_client::result_to_string(ret));
    return false;
  }

  switch (state)
  {
    case tl::ServoState::STOP:
      RCLCPP_INFO(logger_, "[PowerOff]: servo already stopped");
      is_powered_.store(false, std::memory_order_release);
      return true;
    case tl::ServoState::ALARM:
      RCLCPP_WARN(logger_, "[PowerOff]: servo is in alarm state, cannot confirm power off");
      return false;
    case tl::ServoState::READY:
      RCLCPP_INFO(logger_, "[PowerOff]: already power off");
      is_powered_.store(false, std::memory_order_release);
      return true;
    case tl::ServoState::RUNNING:
    {
      ret = tl::set_servo_poweroff(fd);
      if (ret != tl::Result::SUCCESS)
      {
        RCLCPP_ERROR(logger_, "[PowerOff]: set_servo_poweroff failed, ret=%s", arm_client::result_to_string(ret));
        return false;
      }

      ret = tl::get_servo_state(fd, state);
      if (ret != tl::Result::SUCCESS)
      {
        RCLCPP_ERROR(logger_, "[PowerOff]: failed to read servo state after power off, ret=%s",
                     arm_client::result_to_string(ret));
        return false;
      }
      if (state != tl::ServoState::READY)
      {
        RCLCPP_ERROR(logger_, "[PowerOff]: power off not effective, servo_state = %d", static_cast<int>(state));
        return false;
      }
      RCLCPP_INFO(logger_, "[PowerOff]: successfully power off, servo_state = %d", static_cast<int>(state));
      is_powered_.store(false, std::memory_order_release);
      return true;
    }
    default:
      RCLCPP_ERROR(logger_, "[PowerOff]: unknown servo state %d", static_cast<int>(state));
      return false;
  }
}

bool ArmSession::connect(const std::string& ip, const std::string& port, const std::string& port_aux)
{
  if (is_connected_.load(std::memory_order_acquire))
  {
    RCLCPP_INFO(logger_, "[Connect]: arm already connected");
    return true;
  }

  const int fd = tl::connect_robot(ip, port);
  const int fd_aux = tl::connect_robot(ip, port_aux);

  if (fd <= 0 || fd_aux <= 0)
  {
    if (fd <= 0)
    {
      RCLCPP_ERROR(logger_, "[Connect]: failed to connect to %s:%s", ip.c_str(), port.c_str());
    }
    if (fd_aux <= 0)
    {
      RCLCPP_ERROR(logger_, "[Connect]: failed to connect to %s:%s", ip.c_str(), port_aux.c_str());
    }

    // 任一端口失败都必须关闭另一端口，否则重试会覆盖 fd，导致控制器侧连接堆积。
    if (fd > 0)
    {
      tl::Result ret = tl::disconnect_robot(fd);
      if (ret != tl::Result::SUCCESS)
      {
        RCLCPP_WARN(logger_, "[Connect]: failed to close %s port, ret=%s", port.c_str(),
                    arm_client::result_to_string(ret));
      }
    }
    if (fd_aux > 0)
    {
      tl::Result ret = tl::disconnect_robot(fd_aux);
      if (ret != tl::Result::SUCCESS)
      {
        RCLCPP_WARN(logger_, "[Connect]: failed to close %s port, ret=%s", port_aux.c_str(),
                    arm_client::result_to_string(ret));
      }
    }
    socket_fd_.store(0);
    socket_fd_aux_.store(0);
    is_connected_.store(false, std::memory_order_release);
    return false;
  }

  socket_fd_.store(fd, std::memory_order_relaxed);
  socket_fd_aux_.store(fd_aux, std::memory_order_relaxed);

  arm_client::register_error_callback(fd);

  arm_client::register_error_callback(fd_aux);

  // 新 SDK：机器人状态通过 robot_state_callback（7000 端口）接收，不再使用旧的 recv_message 机制
  arm_client::start_robot_state_channel(fd_aux);

  // release：最后发布“已连接”，保证读者看到 true 时 fd 与回调/状态通道均已就绪。
  is_connected_.store(true, std::memory_order_release);

  RCLCPP_INFO(logger_, "[Connect]: successfully connected to arm at %s:%s,%s", ip.c_str(), port.c_str(),
              port_aux.c_str());

  return true;
}

bool ArmSession::disconnect()
{
  if (!is_connected_.load(std::memory_order_acquire))
  {
    RCLCPP_INFO(logger_, "[Disconnect]: arm already disconnected");
    return true;
  }

  is_connected_.store(false, std::memory_order_release);

  const int fd = socket_fd_.load(std::memory_order_acquire);
  const int fd_aux = socket_fd_aux_.load(std::memory_order_acquire);

  tl::disconnect_robot(fd);
  tl::disconnect_robot(fd_aux);

  socket_fd_.store(0);
  socket_fd_aux_.store(0);

  return true;
}
} // namespace tl_driver
