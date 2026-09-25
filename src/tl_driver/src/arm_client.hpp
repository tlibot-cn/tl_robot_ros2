#ifndef TL_DRIVER__ARM_CLIENT_HPP_
#define TL_DRIVER__ARM_CLIENT_HPP_

/// @file arm_client.hpp
/// @brief SDK 侧薄封装：返回值文本化与异步推送回调注册。
/// @note 仅包装 SDK 调用，不持有会话状态（fd 由调用方传入），故不引入额外的生命周期约束。

#include <chrono>
#include <cstdint>
#include <string>

#include "arm_sdk.hpp"

namespace tl_driver::arm_client
{
/// @brief SDK 返回值转文本，用于日志诊断。
/// @param[in] ret SDK 返回值
/// @return 静态文本，未识别的值返回 "UNKNOWN_ERROR"
const char *result_to_string(tl::Result ret) noexcept;

/// @brief 在指定端口注册错误/告警异步推送回调。
/// @param[in] fd 已建立的 SDK 连接（6001 与 7000 均需注册）
void register_error_callback(int fd);

/// @brief 在辅助端口注册机器人状态异步推送。
/// @param[in] fd_aux 7000 辅助端口连接
void start_robot_state_channel(int fd_aux);

/// @brief 读取当前状态帧序号，作为等待新帧的基准。
/// @return 单调递增的状态帧序号
uint64_t robot_state_seq();

/// @brief 等待一帧机器人状态消息。
/// @param[in] start_seq 调用前的序号（见 robot_state_seq）
/// @param[in] timeout 最长等待时间
/// @param[out] message 成功时写入状态消息原文
/// @return 收到序号更新的状态帧为 true，超时为 false
bool wait_robot_state(uint64_t start_seq, std::chrono::milliseconds timeout, std::string& message);
} // namespace tl_driver::arm_client

#endif // TL_DRIVER__ARM_CLIENT_HPP_
