/*
 * nrc_dual_arm.h
 *
 *  Created on: 2025年9月16日
 *      Author: liuhao
 */

#ifndef INCLUDE_CPP_INTERFACE_NRC_DUAL_ARM_H_
#define INCLUDE_CPP_INTERFACE_NRC_DUAL_ARM_H_

#include "cpp/parameter/nrc_define.h"
#include "cpp/parameter/nrc_interface_parameter.h"

/**
 * @brief 设置一条move指令执行完成时回调函数。
 * @param fun 指向回调函数的指针。该回调函数在一条move指令执行完成时被调用。
 * @note 回调函数由用户实现，用于在一条move指令执行完成时处理任务。
 */
EXPORT_API Result set_one_movecomd_completion_callback(SOCKETFD socketFd,void(*function)());


/**
 * @brief 设置用户坐标编号
 * @param userNum 用户坐标编号
 */
EXPORT_API Result set_dualarm_user_coord_number(SOCKETFD socketFd, int userNum);
EXPORT_API Result set_dualarm_user_coord_number_robot(SOCKETFD socketFd, int robotNum, int userNum);

/**
 * @brief 获取当前使用的用户坐标编号
 * @userNum 用户坐标编号
 */
EXPORT_API Result get_dualarm_user_coord_number(SOCKETFD socketFd, int& userNum);
EXPORT_API Result get_dualarm_user_coord_number_robot(SOCKETFD socketFd, int robotNum, int& userNum);

/**
 * @brief 获取当前使用的用户坐标的参数
 * @param userNum 用户坐标编号
 * @param pos 用户坐标参数
 * @param locationType 坐标系类型 0为静态用户坐标系 1为联动坐标系
 * @param mechID 机器人号
 */
EXPORT_API Result get_dualarm_user_coord_para(SOCKETFD socketFd,int userNum, std::vector<double>& pos,int& locationType, int& mechID);
EXPORT_API Result get_dualarm_user_coord_para_robot(SOCKETFD socketFd, int robotNum, int userNum, std::vector<double>& pos, int& locationType, int& mechID);

/**
 * @brief 标定用户坐标
 * @param userNum 用户坐标编号
 * @param pos 坐标数据
 * @param locationType 坐标系类型 0为静态用户坐标系 1为联动坐标系
 * @param mechID 机器人号
 */
EXPORT_API Result set_dualarm_user_coordinate_data(SOCKETFD socketFd,int userNum, std::vector<double> pos, int locationType, int mechID);
EXPORT_API Result set_dualarm_user_coordinate_data_robot(SOCKETFD socketFd, int robotNum, int userNum, std::vector<double> pos, int locationType, int mechID);

/**
 * @brief 标定OXY
 * @param userNum 用户坐标编号
 * @param xyo 值 'X' 'Y' 'O'
 */
EXPORT_API Result dualarm_calibration_oxy(SOCKETFD socketFd, int userNum, std::string xyo);
EXPORT_API Result dualarm_calibration_oxy_robot(SOCKETFD socketFd, int robotNum, int userNum, std::string xyo);

/**
 * @brief 计算坐标
 * @param userNum 用户坐标编号
 */
EXPORT_API Result dualarm_calculate_user_coordinate(SOCKETFD socketFd, int userNumber);
EXPORT_API Result dualarm_calculate_user_coordinate_robot(SOCKETFD socketFd, int robotNum, int userNum);

#endif /* INCLUDE_CPP_INTERFACE_NRC_DUAL_ARM_H_ */
