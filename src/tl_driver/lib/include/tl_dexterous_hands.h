/**
 * @file tl_dexterous_hands.h
 * @brief 灵巧手及手势/动作序列接口（namespace tl）
 *
 * 对应控制器 0827 固件新增的灵巧手协议：
 *  - 灵巧手参数设置/查询（通讯模式、速度、按指力阈值）
 *  - 力阈值设置
 *  - 手势序号执行与手势名查询
 *  - 动作序号执行与动作名查询
 */
#ifndef TL_SDK_TL_DEXTEROUS_HANDS_H
#define TL_SDK_TL_DEXTEROUS_HANDS_H

#include "tl_types.h"

namespace tl
{

/**
 * @brief 设置灵巧手参数
 * @param param 灵巧手参数（mode 通讯模式 0:modbus_rtu 1:RS485 / speed 速度 / force_threshold 按指力阈值，结构体 DexterousHandsParam 详见 tl_types.h）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_dexterous_hands_param(SOCKETFD socketFd, const DexterousHandsParam& param);

/**
 * @brief 查询灵巧手参数
 * @param param 用于接收灵巧手参数（结构体 DexterousHandsParam 详见 tl_types.h）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_dexterous_hands_param(SOCKETFD socketFd, DexterousHandsParam& param);

/**
 * @brief 设置灵巧手力阈值（按指设置）
 * @param mode 灵巧手模式
 * @param speed 速度
 * @param force_threshold 按指力阈值（结构体 ForceThreshold 详见 tl_types.h）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_dexterous_hands_force_threshold(SOCKETFD socketFd, int mode, int speed, const ForceThreshold& force_threshold);

/**
 * @brief 执行手势序号
 * @param gesture_serial_number 手势序号
 * @param mode 灵巧手模式
 * @param speed 速度
 * @param process_number 工艺号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_gesture_serial_number(SOCKETFD socketFd, int gesture_serial_number, int mode, int speed, int process_number);

/**
 * @brief 查询手势序号对应的手势名
 * @param gesture_id 手势序号
 * @param gesture_name 用于接收手势名
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_gesture_serial(SOCKETFD socketFd, int gesture_id, std::string& gesture_name);

/**
 * @brief 执行动作序号
 * @param action_serial_number 动作序号
 * @param mode 灵巧手模式
 * @param speed 速度
 * @param process_number 工艺号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_action_serial_number(SOCKETFD socketFd, int action_serial_number, int mode, int speed, int process_number);

/**
 * @brief 查询动作序号对应的动作名
 * @param action_id 动作序号
 * @param action_name 用于接收动作名
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_action_serial(SOCKETFD socketFd, int action_id, std::string& action_name);

} // namespace tl

#endif /* TL_SDK_TL_DEXTEROUS_HANDS_H */