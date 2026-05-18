/*
 * nrc_vision.h
 *
 *  Created on: 2024年5月14日
 *      Author: inexbot
 */

#ifndef INCLUDE_CPP_INTERFACE_NRC_CRAFT_VISION_H_
#define INCLUDE_CPP_INTERFACE_NRC_CRAFT_VISION_H_

#include "cpp/parameter/nrc_define.h"
#include "cpp/parameter/nrc_craft_vision_parameter.h"

/**
 * @brief 设置视觉的基本参数
 * @param vsPamrm 视觉参数结构体 详情参考结构体定义文件
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 * @note 此函数将配置视觉的基本参数
 */
EXPORT_API Result vision_set_basic_parameter(SOCKETFD socketFd, int visionNum,VisionParam vsPamrm);
EXPORT_API Result vision_set_basic_parameter_robot(SOCKETFD socketFd, int robotNum, int visionNum,VisionParam vsPamrm);

/**
 * @brief 查询视觉的基本参数
 * @param visionNum 视觉ID
 * @param vsPamrm 接收视觉参数结构体 详情参考结构体定义文件
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 * @note 此函数将获得配置的视觉基本参数
 */
EXPORT_API Result vision_get_basic_parameter(SOCKETFD socketFd, int visionNum, VisionParam& vsPamrm);
EXPORT_API Result vision_get_basic_parameter_robot(SOCKETFD socketFd, int robotNum, int visionNum, VisionParam& vsPamrm);

/**
 * @brief 设置视觉范围
 * @param visionNum 视觉id
 * @param vsPamrm 视觉范围参数 详情参考结构体定义文件
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 * @note 此函数将配置视觉的范围参数
 */
EXPORT_API Result vision_set_range(SOCKETFD socketFd, int visionNum, VisionRange vsPamrm);
EXPORT_API Result vision_set_range_robot(SOCKETFD socketFd, int robotNum, int visionNum, VisionRange vsPamrm);

/**
 * @brief 查询视觉范围数据
 * @param visionNum 视觉id
 * @param vsPamrm 用于存储视觉范围参数 详情参考结构体定义文件
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 * @note 此函数将获得配置视觉的范围参数
 */
EXPORT_API Result vision_get_range(SOCKETFD socketFd, int visionNum, VisionRange &vsPamrm);
EXPORT_API Result vision_get_range_robot(SOCKETFD socketFd, int robotNum, int visionNum, VisionRange &vsPamrm);

/**
 * @brief 设置视觉坐标参数
 * @param vsPamrm 视觉坐标参数 详情参考结构体定义文件
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 * @note 此函数将配置视觉的范围参数
 */
EXPORT_API Result vision_set_position_parameter(SOCKETFD socketFd, int visionNum, VisionPositionParam vsPamrm);
EXPORT_API Result vision_set_position_parameter_robot(SOCKETFD socketFd, int robotNum, int visionNum, VisionPositionParam vsPamrm);

/**
 * @brief 查询视觉坐标参数
 * @param visionId 视觉id
 * @param vsPamrm 用于存储视觉坐标参数 详情参考结构体 VisionPositionParam定义
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 * @note 此函数将获得配置视觉的范围位置参数
 */
EXPORT_API Result vision_get_position_parameter(SOCKETFD socketFd, int visionId, VisionPositionParam &vsPamrm);
EXPORT_API Result vision_get_position_parameter_robot(SOCKETFD socketFd, int robotNum, int visionId, VisionPositionParam &vsPamrm);

/**
 * @brief 视觉标定
 * @param vsPamrm 标定参数 详情参考结构体 VisionCalibrationData 定义
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 * @note 此函数将获得配置视觉的范围位置参数
 */
EXPORT_API Result vision_calibrate(SOCKETFD socketFd, int visionId, VisionCalibrationData vsPamrm);
EXPORT_API Result vision_calibrate_robot(SOCKETFD socketFd, int robotNum, int visionId, VisionCalibrationData vsPamrm);

/**
 * @brief 查询视觉数据
 * @param visionId 视觉id
 * @param vsPamrm 标定参数 详情参考结构体 VisionCalibrationData 定义
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 * @note 此函数将获得配置视觉的范围位置参数
 */
EXPORT_API Result vision_get_calibrate_data(SOCKETFD socketFd, int visionId, VisionCalibrationData &vsPamrm);
EXPORT_API Result vision_get_calibrate_data_robot(SOCKETFD socketFd, int robotNum, int visionId, VisionCalibrationData &vsPamrm);

/**
 * @brief 视觉标定计算
 * @param visionNum 视觉id
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 * @note 此函数将获得配置视觉的范围位置参数
 */
EXPORT_API Result vision_hand_eye_calibration_calculation(SOCKETFD socketFd, int visionNum);
EXPORT_API Result vision_hand_eye_calibration_calculation_robot(SOCKETFD socketFd, int robotNum, int visionNum);

#endif /* INCLUDE_API_NRC_CRAFT_VISION_H_ */
