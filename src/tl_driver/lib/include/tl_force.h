/**
 * @file tl_force.h
 * @brief 六维力传感器接口（namespace tl）
 *
 * 六维力传感器的通讯配置、数据读取、基础参数与标定。
 */
#ifndef TL_SDK_TL_FORCE_H
#define TL_SDK_TL_FORCE_H

#include <vector>
#include "tl_types.h"

namespace tl
{
// 注意：以下 5 个接口与 include/tl_interface.h 中的声明双份并存（历史遗留），
// 签名必须逐字一致，改动需同步两处；权威实现位于 src/tl_host/force.cpp。

/**
 * @brief 设置六维力传感器通讯
 * @param params 用于设置六维力传感器通讯参数的结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_six_dimensional_force_communication_params(
    SOCKETFD socketFd, SixDimensionalForceCommunicationParams& params);

/**
 * @brief 获取六维力传感器通讯参数
 * @param params 输出：用于接收六维力传感器通讯参数的结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_six_dimensional_force_communication_params(
    SOCKETFD socketFd, SixDimensionalForceCommunicationParams& params);

/**
 * @brief 获取六维力传感器数据
 * @param sensorData 用于接收六维力传感器数据的结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_sensor_6d_data(SOCKETFD socketFd, Sensor6DData& sensorData);

/**
 * @brief 获取六维力传感器的基础参数（质量、质心、标零状态）
 * @param baseParam 用于接收传感器基础参数的结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_sensor_base_param(SOCKETFD socketFd, SensorBaseParam& baseParam);

/**
 * @brief 执行六维力传感器标定
 * @param[out] success 标定是否成功的标志
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result mark_base_sensor(SOCKETFD socketFd, bool& success);

} // namespace tl

#endif /* TL_SDK_TL_FORCE_H */
