/**
 * @file tl_queue.h
 * @brief 队列运动接口（namespace tl）
 */
#ifndef TL_SDK_TL_QUEUE_H
#define TL_SDK_TL_QUEUE_H


#include "tl_types.h"

namespace tl
{


/**
 * @brief 打开or关闭控制器的队列运动模式
 * @param status 0-关闭 1-打开
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @warning 无论是打开还是关闭，都将清空远端控制器已经存储的队列
 * @note 打开队列模式（status=true）时，控制器会先执行伺服下电再上电；
 *       关闭队列模式（status=false）退出后，控制器会执行伺服下电，
 *       后续如需继续运动需重新执行上电流程（set_servo_state(1) → set_servo_poweron）
 */
TL_API Result queue_motion_set_status(SOCKETFD socketFd, bool status);

/**
 * @brief 查询控制器当前是否打开队列运动模式
 * @param status 用来存储返回值 0-关闭 1-已打开
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result queue_motion_get_status(SOCKETFD socketFd, bool& status);

/**
 * @brief 清空缓存的运动队列数据
 * @note 仅清除上位机侧缓存的队列数据，不清除已经下发给控制器的队列；
 *       控制器已接收的队列会继续执行，如需停止需调用 queue_motion_stop
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result queue_motion_clear_Data(SOCKETFD socketFd);

/**
 * @brief 查询当前运动队列的长度
 * @param[out] size 当前运动队列长度
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result queue_motion_size(SOCKETFD socketFd, int& size);

/**
 * @brief 查询当前运动队列剩余的指令数量
 * @param[out] len 当前运动队列剩余的指令数量
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result queue_motion_get_queuelen(SOCKETFD socketFd, int& len);

/**
 * @brief 将本地队列的前size个数据发送到控制器
 * @param size 要发送的队列大小 size = 0时将当前运动队列全部指令发送  范围[0,31];
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误（传入的size超过队列长度）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @warning 控制器接受到队列后，将会立马开始运动。调用前请先调用queue_motion_set_status(true);
 * @iscontinue 是否选择继续发送，true -继续；false
 * -不继续，继续发送时可以继续发送点位，机器人不运动;不继续时机器人接收点位后立刻运动,默认为false
 */
TL_API Result queue_motion_send_to_controller(SOCKETFD socketFd, int size, bool isContinue = false);

/**
 * @brief 暂停连续运动
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result queue_motion_suspend(SOCKETFD socketFd);

/**
 * @brief 暂停后再次运行运动队列
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result queue_motion_restart(SOCKETFD socketFd);

/**
 * @brief 停止连续运动
 *
 * @deprecated 该接口已废弃，建议使用 `queue_motion_stop_not_power_off` 替代。
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API TL_DEPRECATED("use queue_motion_stop_not_power_off instead") Result queue_motion_stop(SOCKETFD socketFd);

/**
 * @brief 停止连续运动保持上电状态
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result queue_motion_stop_not_power_off(SOCKETFD socketFd);

/**
 * @brief 队列运动模式的本地队列最后插入一条moveJ运动
 * @param moveCmd 运动指令结构体，字段说明：
 *  targetPos 目标点坐标 长度7位
 *  velocity 参数范围(1,100] 度/s
 *  acc，dec 参数范围(1,100]
 *  pl 多条轨迹之间平滑参数 参数范围[0,5]
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result queue_motion_push_back_moveJ(SOCKETFD socketFd, MoveCmd moveCmd);

/**
 * @brief 队列运动模式的本地队列最后插入一条moveL运动
 * @param moveCmd 运动指令结构体，字段说明：
 *  targetPos 目标点坐标 长度7位
 *  velocity 参数范围(1,1000] mm/s
 *  acc，dec 参数范围(1,100]
 *  pl 多条轨迹之间平滑参数 参数范围[0,5]
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result queue_motion_push_back_moveL(SOCKETFD socketFd, MoveCmd moveCmd);

/**
 * @brief 队列运动模式的本地队列最后插入一条moveC运动
 * @param moveCmd 运动指令结构体，字段说明：
 *  targetPos 目标点坐标 长度7位
 *  velocity 参数范围(1,1000] mm/s
 *  acc，dec 参数范围(1,100]
 *  pl 多条轨迹之间平滑参数 参数范围[0,5]
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result queue_motion_push_back_moveC(SOCKETFD socketFd, MoveCmd moveCmd);

/**
 * @brief 队列运动模式的本地队列最后插入一条moveS运动
 * @param moveCmd 运动指令结构体，字段说明：
 *  targetPos 目标点坐标 长度7位
 *  velocity 参数范围(1,1000] mm/s
 *  acc，dec 参数范围(1,100]
 *  pl 多条轨迹之间平滑参数 参数范围[0,5]
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result queue_motion_push_back_moveS(SOCKETFD socketFd, MoveCmd moveCmd);

} // namespace tl

#endif /* TL_SDK_TL_QUEUE_H */
