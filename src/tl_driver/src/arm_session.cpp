/// @file arm_session.cpp
/// @brief ArmSession 的实现（与拆分前的同名成员实现等价）。

#include "arm_session.hpp"

#include <chrono>
#include <thread>

#include "arm_client.hpp"
#include "arm_sdk.hpp"

namespace tl_driver
{
ArmSession::ArmSession(rclcpp::Logger logger) : logger_(logger) {}

int ArmSession::fd() const
{
  return socket_fd_;
}

int ArmSession::fd_aux() const
{
  return socket_fd_aux_;
}

bool ArmSession::is_connected() const
{
  return is_connected_ && socket_fd_ > 0 && socket_fd_aux_ > 0;
}

bool ArmSession::is_powered() const
{
  return is_powered_;
}

bool ArmSession::power_on()
{
  tl::ServoState state = static_cast<tl::ServoState>(-1); // -1 哨兵值：查询失败时落入 default 分支
  tl::get_servo_state(socket_fd_, state);

  switch (state)
  {
    case tl::ServoState::STOP:
      tl::set_servo_state(socket_fd_, tl::ServoState::READY);
      RCLCPP_INFO(logger_, "[PowerOn]: waiting 2s for servo ready...");
      std::this_thread::sleep_for(std::chrono::seconds(2));
      tl::set_servo_poweron(socket_fd_);
      break;
    case tl::ServoState::READY:
      tl::set_servo_poweron(socket_fd_);
      break;
    case tl::ServoState::ALARM:
      RCLCPP_WARN(logger_, "[PowerOn]: servo alarm state (2), clearing error first");
      tl::clear_error(socket_fd_);
      tl::set_servo_state(socket_fd_, tl::ServoState::READY);
      tl::set_servo_poweron(socket_fd_);
      break;
    case tl::ServoState::RUNNING:
      RCLCPP_INFO(logger_, "[PowerOn]: servo_state=3, re-executing power on");
      tl::set_servo_state(socket_fd_, tl::ServoState::READY);
      tl::set_servo_poweron(socket_fd_);
      break;

    default:
      RCLCPP_ERROR(logger_, "[PowerOn]: unknown servo state %d", static_cast<int>(state));
      return false;
  }

  tl::get_servo_state(socket_fd_, state);
  RCLCPP_INFO(logger_, "[PowerOn]: servo_state = %d", static_cast<int>(state));
  if (state == tl::ServoState::RUNNING)
  {
    is_powered_ = true;
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
  tl::ServoState state = static_cast<tl::ServoState>(-1); // -1 哨兵值：查询失败时落入 default 分支
  tl::get_servo_state(socket_fd_, state);
  switch (state)
  {
    case tl::ServoState::STOP:
    case tl::ServoState::ALARM:
      break;
    case tl::ServoState::READY:
      RCLCPP_INFO(logger_, "[PowerOff]: already power off");
      return true;
    case tl::ServoState::RUNNING:
      tl::set_servo_poweroff(socket_fd_);
      tl::get_servo_state(socket_fd_, state);
      RCLCPP_INFO(logger_, "[PowerOff]: successfully power off, servo_state = %d", static_cast<int>(state));
      is_powered_ = false;
      return true;
  }
  RCLCPP_INFO(logger_, "[PowerOff]: fail to power off, servo_state = %d", static_cast<int>(state));
  return false;
}

bool ArmSession::connect(const std::string& ip, const std::string& port, const std::string& port_aux)
{
  if (is_connected_)
  {
    RCLCPP_INFO(logger_, "[Connect]: arm already connected");
    return true;
  }

  socket_fd_ = tl::connect_robot(ip, port);
  socket_fd_aux_ = tl::connect_robot(ip, port_aux);

  if (socket_fd_ <= 0)
  {
    RCLCPP_ERROR(logger_, "[Connect]: failed to connect to %s:%s", ip.c_str(), port.c_str());
    socket_fd_ = 0;
    is_connected_ = false;
    return false;
  }

  if (socket_fd_aux_ <= 0)
  {
    RCLCPP_ERROR(logger_, "[Connect]: failed to connect to %s:%s", ip.c_str(), port_aux.c_str());
    socket_fd_aux_ = 0;
    is_connected_ = false;
    return false;
  }

  is_connected_ = true;

  arm_client::register_error_callback(socket_fd_);

  arm_client::register_error_callback(socket_fd_aux_);

  // 新 SDK：机器人状态通过 robot_state_callback（7000 端口）接收，不再使用旧的 recv_message 机制
  arm_client::start_robot_state_channel(socket_fd_aux_);

  RCLCPP_INFO(logger_, "[Connect]: successfully connected to arm at %s:%s,%s", ip.c_str(), port.c_str(),
              port_aux.c_str());

  return true;
}

bool ArmSession::disconnect()
{
  if (!is_connected_)
  {
    RCLCPP_INFO(logger_, "[Disconnect]: arm already disconnected");
    return true;
  }

  is_connected_ = false;

  tl::disconnect_robot(socket_fd_);
  tl::disconnect_robot(socket_fd_aux_);

  socket_fd_ = 0;
  socket_fd_aux_ = 0;

  return true;
}
} // namespace tl_driver
