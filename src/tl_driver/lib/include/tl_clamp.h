/**
 * @file tl_clamp.h
 * @brief 夹爪接口（namespace tl）
 *
 * 对应控制器 0827 固件新增的夹爪控制协议，支持：
 *  - 参数设置/查询（通讯模式、速度、力阈值）
 *  - 夹紧 / 松开 / 持续夹紧动作
 *  - 状态查询（使能、连接、压力、张开度、温度、状态码）
 */
#ifndef TL_SDK_TL_CLAMP_H
#define TL_SDK_TL_CLAMP_H

#include "tl_types.h"

namespace tl
{

/**
 * @brief 设置夹爪参数
 * @param param 夹爪参数（mode 通讯模式 0:modbus_rtu 1:RS485 / speed 速度 / force_threshold 力阈值）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_clamp_param(SOCKETFD socketFd, const ClampParam& param);

/**
 * @brief 查询夹爪参数
 * @param param 用于接收夹爪参数（结构体 ClampParam 详见 tl_types.h）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_clamp_param(SOCKETFD socketFd, ClampParam& param);

/**
 * @brief 夹爪夹紧
 * @param param 夹爪动作参数（mode/speed/force_threshold，参考当前夹爪参数）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result clamp_claim(SOCKETFD socketFd, const ClampParam& param);

/**
 * @brief 夹爪松开
 * @param param 夹爪动作参数（mode/speed/force_threshold，参考当前夹爪参数）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result clamp_claim_release(SOCKETFD socketFd, const ClampParam& param);

/**
 * @brief 夹爪持续夹紧（保持夹紧力）
 * @param param 夹爪动作参数（mode/speed/force_threshold，参考当前夹爪参数）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result clamp_continuous_clamping(SOCKETFD socketFd, const ClampParam& param);

/**
 * @brief 查询夹爪状态
 * @param mode 夹爪模式
 * @param status 用于接收夹爪状态（使能/连接/压力/张开度/温度/状态码，结构体 ClampStatus 详见 tl_types.h）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_clamp_status(SOCKETFD socketFd, int mode, ClampStatus& status);

} // namespace tl

#endif /* TL_SDK_TL_CLAMP_H */