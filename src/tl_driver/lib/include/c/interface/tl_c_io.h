/*
 * tl_c_io.h
 *
 *  Created on: 2025年1月24日
 *      Author: yiixiong
 */

#ifndef INCLUDE_C_INTERFACE_TL_C_IO_H_
#define INCLUDE_C_INTERFACE_TL_C_IO_H_

#include "c/parameter/tl_define.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 设置数字输出
 * @param port 端口号 【1，最大端口数】
 * @param value 输出端口值 0 or 1
 */
EXPORT_API int set_digital_output_c(SOCKETFD socketFd, int port, int value);

/**
 * @brief 一次获取所有数字输出
 * @param out 存储结果的数组，长度为64
 */
EXPORT_API int get_digital_output_c(SOCKETFD socketFd, double* out);

/**
 * @brief 一次获取所有数字输入
 * @param in 存储结果的数组，长度为64
 */
EXPORT_API int get_digital_input_c(SOCKETFD socketFd, double* in);

#ifdef __cplusplus
}
#endif

#endif /* INTERFACE_C_INTERFACE_TL_C_IO_H_ */
