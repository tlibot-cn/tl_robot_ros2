#ifndef TL_DRIVER__ARM_CLIENT_HPP_
#define TL_DRIVER__ARM_CLIENT_HPP_

/// @file arm_client.hpp
/// @brief SDK 侧薄封装：返回值文本化与异步推送回调注册。
/// @note 仅包装 SDK 调用，不持有会话状态（fd 由调用方传入），故不引入额外的生命周期约束。

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
} // namespace tl_driver::arm_client

#endif // TL_DRIVER__ARM_CLIENT_HPP_
