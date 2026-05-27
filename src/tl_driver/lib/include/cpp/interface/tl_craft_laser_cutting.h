#ifndef INCLUDE_CPP_INTERFACE_TL_CRAFT_LASER_CUTTING_H_
#define INCLUDE_CPP_INTERFACE_TL_CRAFT_LASER_CUTTING_H_

#include "cpp/parameter/tl_define.h"
#include "cpp/parameter/tl_craft_laser_cutting_parameter.h"

/**
 * @brief 设置切割的全局参数
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 */
EXPORT_API Result laser_cutting_set_global_parameter(SOCKETFD socketFd, LaserCuttingGlobalParam param);
EXPORT_API Result laser_cutting_set_global_parameter_robot(SOCKETFD socketFd, int robotNum, LaserCuttingGlobalParam param);

/**
 * @brief 查询切割的全局参数
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 */
EXPORT_API Result laser_cutting_get_global_parameter(SOCKETFD socketFd, LaserCuttingGlobalParam& param);
EXPORT_API Result laser_cutting_get_global_parameter_robot(SOCKETFD socketFd, int robotNum, LaserCuttingGlobalParam& param);

/**
 * @brief 设置切割的工艺参数
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 */
EXPORT_API Result laser_cutting_set_craft_parameter(SOCKETFD socketFd, LaserCuttingCraftParam param);
EXPORT_API Result laser_cutting_set_craft_parameter_robot(SOCKETFD socketFd, int robotNum, LaserCuttingCraftParam param);

/**
 * @brief 查询切割的工艺参数
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 */
EXPORT_API Result laser_cutting_get_craft_parameter(SOCKETFD socketFd, LaserCuttingCraftParam& param);
EXPORT_API Result laser_cutting_get_craft_parameter_robot(SOCKETFD socketFd, int robotNum, LaserCuttingCraftParam& param);

/**
 * @brief 设置切割的模拟量参数
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 */
EXPORT_API Result laser_cutting_set_analog_parameter(SOCKETFD socketFd, LaserCuttingAnalogParam param);
EXPORT_API Result laser_cutting_set_analog_parameter_robot(SOCKETFD socketFd, int robotNum, LaserCuttingAnalogParam param);

/**
 * @brief 查询切割的模拟量参数
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 */
EXPORT_API Result laser_cutting_get_analog_parameter(SOCKETFD socketFd, LaserCuttingAnalogParam& param);
EXPORT_API Result laser_cutting_get_analog_parameter_robot(SOCKETFD socketFd, int robotNum, LaserCuttingAnalogParam& param);

/**
 * @brief 设置切割的模拟量参数
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 */
EXPORT_API Result laser_cutting_set_io_parameter(SOCKETFD socketFd, LaserCuttingIOParam param);
EXPORT_API Result laser_cutting_set_io_parameter_robot(SOCKETFD socketFd, int robotNum, LaserCuttingIOParam param);

/**
 * @brief 查询切割的模拟量参数
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 */
EXPORT_API Result laser_cutting_get_io_parameter(SOCKETFD socketFd, LaserCuttingIOParam& param);
EXPORT_API Result laser_cutting_get_io_parameter_robot(SOCKETFD socketFd, int robotNum, LaserCuttingIOParam& param);

#endif /* INTERFACE_CPP_API_TL_CRAFT_LASER_CUTTING_H_ */
