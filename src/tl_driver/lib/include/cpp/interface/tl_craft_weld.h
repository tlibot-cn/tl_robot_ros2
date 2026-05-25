#ifndef INCLUDE_CPP_INTERFACE_TL_CRAFT_WELD_H_
#define INCLUDE_CPP_INTERFACE_TL_CRAFT_WELD_H_

#include "cpp/parameter/tl_define.h"
#include "cpp/parameter/tl_craft_weld_parameter.h"

/**
 * @brief 获取焊接参数
 * @param index 工艺号
 * @param param 焊接参数
 * @return
 */
EXPORT_API Result weld_get_config(SOCKETFD socketFd, int index, ArcParam& param);
EXPORT_API Result weld_get_config_robot(SOCKETFD socketFd, int robotNum, int index, ArcParam& param);

/**
 * @brief 设置焊接参数
 * @param index 工艺号
 * @param param 焊接参数
 * @return
 */
EXPORT_API Result weld_set_config(SOCKETFD socketFd, int index, ArcParam& param);
EXPORT_API Result weld_set_config_robot(SOCKETFD socketFd, int robotNum, int index, ArcParam& param);

/**
 * @brief  送丝
 * @param  state 1:开启 0:关闭
 */
EXPORT_API Result weld_set_feed_wire(SOCKETFD socketFd, int state);
EXPORT_API Result weld_set_feed_wire_robot(SOCKETFD socketFd, int robotNum, int state);

/**
 * @brief  退丝
 * @param  state 1:开启 0:关闭
 */
EXPORT_API Result weld_set_rewind_wire(SOCKETFD socketFd, int state);
EXPORT_API Result weld_set_rewind_wire_robot(SOCKETFD socketFd, int robotNum, int state);

/**
 * @brief  送气
 * @param  state 1:开启 0:关闭
 */
EXPORT_API Result weld_set_supply_gas(SOCKETFD socketFd, int state);
EXPORT_API Result weld_set_supply_gas_robot(SOCKETFD socketFd, int robotNum, int state);

/**
 * @brief  焊接使能
 * @param  state 1:开启 0:关闭
 */
EXPORT_API Result weld_set_enable(SOCKETFD socketFd, int state);
EXPORT_API Result weld_set_enable_robot(SOCKETFD socketFd, int robotNum, int state);

/**
 * @brief  手动点焊
 * @param  state 1:开启 0:关闭
 */
EXPORT_API Result weld_set_hand_spot(SOCKETFD socketFd, int state);
EXPORT_API Result weld_set_hand_spot_robot(SOCKETFD socketFd, int robotNum, int state);

/**
 * @brief  送丝、退丝、送气、焊接使能、手动点焊状态获取
 * @param  status 送丝、退丝、送气、焊接使能、手动点焊状态数组，
 *         status[0]：送丝状态，status[1]：退丝状态，status[2]：送气状态，status[3]：焊接使能，status[4]：手动点焊
 */
EXPORT_API Result weld_get_feed_wire_status(SOCKETFD socketFd,std::vector<int>& status);
EXPORT_API Result weld_get_feed_wire_status_robot(SOCKETFD socketFd, int robotNum, std::vector<int>& status);

/**
 * @brief 获取摆焊参数数据
 * @param num 摆焊文件 [1,9]
 * @param param 摆焊参数
 */
EXPORT_API Result weld_get_wave_weld_param(SOCKETFD socketFd, int num,WaveParam& param);
EXPORT_API Result weld_get_wave_weld_param_robot(SOCKETFD socketFd, int robotNum, int num, WaveParam& param);

/**
 * @brief 设置摆焊参数
 * @param num 焊摆文件 [1,9]
 * @param param 摆焊参数
 */
EXPORT_API Result weld_set_wave_weld_param(SOCKETFD socketFd, int num,const WaveParam& param);
EXPORT_API Result weld_set_wave_weld_param_robot(SOCKETFD socketFd, int robotNum, int num, const WaveParam& param);

/**
 * @brief 获取焊接状态
 * @param status 焊接参数
 */
EXPORT_API Result weld_get_monitor_status(SOCKETFD socketFd,WeldState& status);
EXPORT_API Result weld_get_monitor_status_robot(SOCKETFD socketFd, int robotNum, WeldState& status);

#endif /* INCLUDE_API_TL_CRAFT_WELD_H_ */

