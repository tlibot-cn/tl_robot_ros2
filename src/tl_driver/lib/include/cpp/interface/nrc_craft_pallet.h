/*
 * nrc_craft_pallet.h
 *
 *  Created on: 2025年9月11日
 *      Author: yiixiong
 */

#ifndef INCLUDE_CPP_INTERFACE_NRC_CRAFT_PALLET_H_
#define INCLUDE_CPP_INTERFACE_NRC_CRAFT_PALLET_H_

#include "cpp/parameter/nrc_define.h"

/**
 * @brief 设置码垛运行状态
 * @param craftID 工艺号
 * @param layerNum 层数
 * @param workpiecesNum 工件数
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 */
EXPORT_API Result pallet_set_running_state(SOCKETFD socketFd, int craftID, int layerNum, int workpiecesNum);
EXPORT_API Result pallet_set_running_state_robot(SOCKETFD socketFd, int robotNum, int craftID, int layerNum, int workpiecesNum);

/**
 * @brief 获取码垛运行状态
 * @param totalWpNum 总工件数
 * @param totalLayerNum 总层数
 * @param curLayerWpSum 当前层总工件数
 * @param curLayerNum 当前层数
 * @param curPalletedWpSum 当前已码总工件数
 * @param curLayerPalletedWpNum 当前层已码工件数
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 */
EXPORT_API Result pallet_get_running_state(SOCKETFD socketFd, int craftID, int& totalWpNum, int& totalLayerNum, int& curLayerWpSum, int& curLayerNum, int& curPalletedWpSum, int& curLayerPalletedWpNum);
EXPORT_API Result pallet_get_running_state_robot(SOCKETFD socketFd, int robotNum, int craftID, int& totalWpNum, int& totalLayerNum, int& curLayerWpSum, int& curLayerNum, int& curPalletedWpSum, int& curLayerPalletedWpNum);

/**
 * @brief 获取当前运行的作业文件包含的码垛工艺号
 * @param craftID 工艺号
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 */
EXPORT_API Result pallet_get_current_run_craftid(SOCKETFD socketFd, std::vector<int>& craftIds);
EXPORT_API Result pallet_get_current_run_craftid_robot(SOCKETFD socketFd, int robotNum, std::vector<int>& craftIds);

#endif /* INCLUDE_CPP_INTERFACE_NRC_CRAFT_PALLET_H_ */
