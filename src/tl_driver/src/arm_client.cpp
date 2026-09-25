/// @file arm_client.cpp
/// @brief arm_client 的实现（由 arm_sdk.hpp / arm_session.cpp 搬移，行为不变）。

#include "arm_client.hpp"

#include <iostream>

#include "arm_sdk.hpp"

namespace
{
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
} // namespace tl_driver::arm_client
