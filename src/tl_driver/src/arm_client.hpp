#ifndef TL_DRIVER__ARM_CLIENT_HPP_
#define TL_DRIVER__ARM_CLIENT_HPP_

#include <chrono>
#include <cstdint>
#include <string>

#include "arm_sdk.hpp"

namespace tl_driver::arm_client
{

/**
 * @brief SDK 返回值转可读字符串。
 * @param[in] ret SDK 调用返回值
 * @return 静态字符串，未知取值返回 `"UNKNOWN_ERROR"`
 */
const char *result_to_string(tl::Result ret);

/**
 * @brief 在指定端口注册错误/告警异步推送回调。
 * @param[in] fd 已建立的 SDK 连接（6001 与 7000 均需注册）
 */
void register_error_callback(int fd);

/**
 * @brief 在辅助端口注册机器人状态异步推送。
 * @param[in] fd_aux 7000 辅助端口连接
 */
void start_robot_state_channel(int fd_aux);

/**
 * @brief 读取当前状态帧序号，作为等待新帧的基准。
 * @return 单调递增的状态帧序号
 */
uint64_t robot_state_seq();

/**
 * @brief 等待一帧机器人状态消息。
 * @param[in] start_seq 调用前的序号（见 robot_state_seq）
 * @param[in] timeout 最长等待时间
 * @param[out] msg 成功时写入状态消息原文
 * @return true=收到序号更新且为机器人状态帧；false=超时
 */
bool wait_robot_state(uint64_t start_seq, std::chrono::milliseconds timeout, std::string& msg);

} // namespace tl_driver::arm_client

#endif // TL_DRIVER__ARM_CLIENT_HPP_
