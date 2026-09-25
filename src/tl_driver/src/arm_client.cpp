#include <condition_variable>
#include <iostream>
#include <mutex>

#include "arm_client.hpp"

/**
 * @brief SDK 返回值映射与机器人状态异步推送通道实现。
 */

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

RobotStateMessageBuffer g_robot_state_msg_buffer;

// 控制器状态帧 id（SDK 头未定义该常量，取值来自控制器协议）
constexpr int kRobotStateMessageId = 7681;

// 普通 C 风格函数指针回调
void robot_state_recv_callback(int msg_id, const char *msg)
{
  {
    std::lock_guard<std::mutex> lock(g_robot_state_msg_buffer.mutex);

    g_robot_state_msg_buffer.last_msg_id = msg_id;
    g_robot_state_msg_buffer.last_msg = msg ? msg : "";
    ++g_robot_state_msg_buffer.seq;
  }

  g_robot_state_msg_buffer.cv.notify_all();

  std::cout << "\033[32m"
            << "id = " << msg_id << ", msg = " << (msg ? msg : "") << "\033[0m" << std::endl;
}

void receive_error_or_warning_message_callback(int messageType, const char *message, int messageCode)
{
  std::cout << "\033[31m"
            << "messageType = " << messageType << ", message = " << message << ", messageCode = " << messageCode
            << "\033[0m" << std::endl;
}

} // namespace

namespace tl_driver::arm_client
{

const char *result_to_string(tl::Result ret)
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
  tl::recv_message(fd_aux, robot_state_recv_callback);
}

uint64_t robot_state_seq()
{
  std::lock_guard<std::mutex> lock(g_robot_state_msg_buffer.mutex);
  return g_robot_state_msg_buffer.seq;
}

bool wait_robot_state(uint64_t start_seq, std::chrono::milliseconds timeout, std::string& msg)
{
  std::unique_lock<std::mutex> lock(g_robot_state_msg_buffer.mutex);

  const bool received = g_robot_state_msg_buffer.cv.wait_for(
      lock, timeout,
      [start_seq]()
      {
        return g_robot_state_msg_buffer.seq > start_seq && g_robot_state_msg_buffer.last_msg_id == kRobotStateMessageId;
      });

  if (!received)
  {
    return false;
  }

  msg = g_robot_state_msg_buffer.last_msg;
  return true;
}

} // namespace tl_driver::arm_client
