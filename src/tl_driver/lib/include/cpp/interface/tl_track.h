/*
 * tl_track.h
 *
 *  Created on: 2024年11月13日
 *      Author: yiixiong
 */

#ifndef INCLUDE_CPP_INTERFACE_TL_TRACK_H_
#define INCLUDE_CPP_INTERFACE_TL_TRACK_H_

#include "cpp/parameter/tl_define.h"

/**
 * @brief 轨迹记录开始
 * @param maxSamplingNum 最大采样点数 [200,12000]
 * @param samplingInterval 采样间隔  [0.03,1]
 */
EXPORT_API Result track_record_start(SOCKETFD socketFd, double maxSamplingNum, double samplingInterval);
EXPORT_API Result track_record_start_robot(SOCKETFD socketFd, int robotNum, double maxSamplingNum, double samplingInterval);

/**
 * @brief 轨迹记录关闭
 */
EXPORT_API Result track_record_stop(SOCKETFD socketFd);
EXPORT_API Result track_record_stop_robot(SOCKETFD socketFd, int robotNum);

/**
 * @brief 轨迹记录开启状态查询
 * @param recordStart 记录状态  true 开启  false 关闭
 */
EXPORT_API Result get_track_record_status(SOCKETFD socketFd, bool& recordStart);
EXPORT_API Result get_track_record_status_robot(SOCKETFD socketFd, int robotNum, bool& recordStart);

/**
 * @brief 轨迹记录保存
 * @param trajName 保存名字
 */
EXPORT_API Result track_record_save(SOCKETFD socketFd, std::string trajName);
EXPORT_API Result track_record_save_robot(SOCKETFD socketFd, int robotNum, std::string trajName);

/**
 * @brief 轨迹回放
 * @param vel 速度
 */
EXPORT_API Result track_record_playback(SOCKETFD socketFd, int vel);
EXPORT_API Result track_record_playback_robot(SOCKETFD socketFd, int robotNum, int vel);

/**
 * @brief 轨迹清除
 */
EXPORT_API Result track_record_delete(SOCKETFD socketFd);
EXPORT_API Result track_record_delete_robot(SOCKETFD socketFd, int robotNum);

#endif /* INCLUDE_API_TL_TRACK_H_ */
