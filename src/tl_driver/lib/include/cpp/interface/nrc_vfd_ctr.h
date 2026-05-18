/*
 * nrc_vfd_ctr.h
 *
 *  Created on: 2025年8月28日
 *      Author: syf
 *      annotation: Customized functions for hanyu
 */


#ifndef INCLUDE_API_NRC_VFD_CTR_H_
#define INCLUDE_API_NRC_VFD_CTR_H_

#include "cpp/parameter/nrc_define.h"
#include "cpp/parameter/nrc_interface_parameter.h"
#include <string>
#include <vector>

/*
 * @brief VFD运行
 * @param Param 主轴运行参数
 */
EXPORT_API Result vfd_run(SOCKETFD socketFd, VFDRunParam& param);

/*
 * @brief VFD停机
 * @param independentAxis  是否为独立轴
 */
EXPORT_API Result vfd_shutdown(SOCKETFD socketFd, bool independentAxis);

/*
 * @brief VFD准停
 * @param stopangle 准停角度
 */
EXPORT_API Result vfd_spindle_positioning_stop(SOCKETFD socketFd, double stopangle);

/*
 * @brief 查询VFD电流,速度,角度
 * @param state VFD电流,速度,角度
 */
EXPORT_API Result get_vfd_vel(SOCKETFD socketFd, VFDState& state);

#endif  /* INCLUDE_API_NRC_VFD_CTR_H_ */
