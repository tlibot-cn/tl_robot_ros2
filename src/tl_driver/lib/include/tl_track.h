/**
 * @file tl_track.h
 * @brief 轨迹记录/回放与拖动轨迹文件管理接口（namespace tl）
 */
#ifndef TL_SDK_TL_TRACK_H
#define TL_SDK_TL_TRACK_H

#include <string>

#include "tl_types.h"

namespace tl
{


/**
 * @brief 轨迹记录开始
 * @param maxSamplingNum 最大采样点数 [200,12000]
 * @param samplingInterval 采样间隔  [0.03,1]
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result track_record_start(SOCKETFD socketFd, double maxSamplingNum, double samplingInterval);

/**
 * @brief 轨迹记录关闭
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result track_record_stop(SOCKETFD socketFd);

/**
 * @brief 轨迹记录开启状态查询
 * @param recordStart 记录状态  true 开启  false 关闭
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_track_record_status(SOCKETFD socketFd, bool& recordStart);

/**
 * @brief 轨迹记录保存
 * @param trajName 保存的轨迹名称
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result track_record_save(SOCKETFD socketFd, std::string trajName);

/**
 * @brief 轨迹回放
 * @param vel 回放速度
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result track_record_playback(SOCKETFD socketFd, int vel);

/**
 * @brief 轨迹清除
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result track_record_delete(SOCKETFD socketFd);

// ==================== 拖动轨迹文件管理 ====================

/**
 * @brief 导出控制器中的全部拖动轨迹。
 * @note 需控制器固件 24.03 及以上版本支持。
 * @param directoryPath 本地保存目录；末级目录不存在时自动创建，父目录须存在
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result drag_trajectory_export(SOCKETFD socketFd, const std::string& directoryPath);

/**
 * @brief 导入本地目录中的全部拖动轨迹。
 * @note 控制器中存在同名轨迹时将被覆盖；需控制器固件 24.03 及以上版本支持。
 * @param directoryPath 包含 .json 轨迹文件的本地目录
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result drag_trajectory_import(SOCKETFD socketFd, const std::string& directoryPath);

/**
 * @brief 回放指定的拖动轨迹。
 * @param trajectoryName 控制器中的轨迹文件名，含 .json 后缀
 * @param velocity 回放速度，范围 [1,100]
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result drag_trajectory_playback(SOCKETFD socketFd, const std::string& trajectoryName,
                                       int velocity);

/**
 * @brief 删除指定的拖动轨迹。
 * @note 删除后不可恢复；需控制器固件 24.03 及以上版本支持。
 * @param trajectoryName 控制器中的轨迹文件名，含 .json 后缀
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result drag_trajectory_delete(SOCKETFD socketFd, const std::string& trajectoryName);

} // namespace tl

#endif /* TL_SDK_TL_TRACK_H */
