/// @file arm_client.cpp
/// @brief arm_client 的实现（由 arm_sdk.hpp / arm_session.cpp 搬移，行为不变）。

#include "arm_client.hpp"

#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <string>

#include "arm_sdk.hpp"

namespace
{
struct RobotStateMessageBuffer
{
  std::mutex mutex;
  std::condition_variable cv;

  int last_msg_id = -1;
  std::string last_msg;
  uint64_t seq = 0;
};

enum MessageLists
{
  ROBOT_STATE = 7681
};

RobotStateMessageBuffer g_robot_state_msg_buffer;

/// @brief 错误/告警异步推送回调（SDK 接收线程调用，仅打印，保持拆分前行为）。
/// @param[in] messageType 消息类型
/// @param[in] message 消息内容
/// @param[in] messageCode 消息码
void receive_error_or_warning_message_callback(int messageType, const char *message, int messageCode)
{
  std::cout << "\033[31m"
            << "messageType = " << messageType << ", message = " << message << ", messageCode = " << messageCode
            << "\033[0m" << std::endl;
}


// 新 SDK：7000 端口机器人状态专用回调（robot_state_callback）
void on_robot_state_message(const char *state)
{
  {
    std::lock_guard<std::mutex> lock(g_robot_state_msg_buffer.mutex);

    g_robot_state_msg_buffer.last_msg_id = MessageLists::ROBOT_STATE;
    g_robot_state_msg_buffer.last_msg = state ? state : "";
    ++g_robot_state_msg_buffer.seq;
  }

  g_robot_state_msg_buffer.cv.notify_all();

  std::cout << "\033[32m"
            << "robot_state = " << (state ? state : "") << "\033[0m" << std::endl;
}
} // namespace

namespace tl_driver::arm_client
{
/// @brief SDK 返回值转文本，用于日志诊断。
/// @param[in] ret SDK 返回值
/// @return 静态文本，未识别的值返回 "UNKNOWN_ERROR"
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

void register_error_callback(int fd)
{
  tl::set_receive_error_or_warnning_message_callback(fd, receive_error_or_warning_message_callback);
}

void start_robot_state_channel(int fd_aux)
{
  tl::robot_state_callback(fd_aux, on_robot_state_message);
}

uint64_t robot_state_seq()
{
  std::lock_guard<std::mutex> lock(g_robot_state_msg_buffer.mutex);
  return g_robot_state_msg_buffer.seq;
}

bool wait_robot_state(uint64_t start_seq, std::chrono::milliseconds timeout, std::string& message)
{
  std::unique_lock<std::mutex> lock(g_robot_state_msg_buffer.mutex);

  const bool received =
      g_robot_state_msg_buffer.cv.wait_for(lock, timeout,
                                           [start_seq]()
                                           {
                                             return g_robot_state_msg_buffer.seq > start_seq &&
                                                    g_robot_state_msg_buffer.last_msg_id == MessageLists::ROBOT_STATE;
                                           });

  if (!received)
  {
    return false;
  }

  message = g_robot_state_msg_buffer.last_msg;
  return true;
}
} // namespace tl_driver::arm_client
