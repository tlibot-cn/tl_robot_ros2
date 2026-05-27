#ifndef INCLUDE_CPP_INTERFACE_TL_CRAFT_TRACK_H_
#define INCLUDE_CPP_INTERFACE_TL_CRAFT_TRACK_H_

/*目前只有2403添加*/
#include "cpp/parameter/tl_define.h"
#include "cpp/parameter/tl_craft_track_parameter.h"


/**
 * @brief 设置跟踪传感器类型
 * @param index 跟踪文件号(工艺号)
 * @param type 跟踪类型，0:线激光；1:电弧；2:弧压
 * @return
 */
EXPORT_API Result track_set_config(SOCKETFD socketFd, int index,int type);
EXPORT_API Result track_set_config_robot(SOCKETFD socketFd, int robotNum, int index,int type);

/**
 * @brief 查询跟踪传感器类型
 * @param index 跟踪文件号(工艺号)
 * @param type 跟踪类型，0:线激光；1:电弧；2:弧压
 * @return
 */
EXPORT_API Result track_get_config(SOCKETFD socketFd, int index,int& type);
EXPORT_API Result track_get_config_robot(SOCKETFD socketFd, int robotNum, int index,int& type);

/**
 * @brief 设置电弧通讯跟踪参数
 * @param index 跟踪文件号(工艺号)
 * @param type 采样数据类型，0：电流；1：电压
 * @param cycle 采样周期；取值范围[0,1000]ms
 * @return
 */
EXPORT_API Result track_set_comm_param(SOCKETFD socketFd, int index,int type,int cycle);
EXPORT_API Result track_set_comm_param_robot(SOCKETFD socketFd, int robotNum, int index,int type,int cycle);

/**
 * @brief 获取电弧通讯跟踪参数
 * @param index 跟踪文件号(工艺号)
 * @param type 采样数据类型，0：电流；1：电压
 * @param cycle 采样周期；取值范围[0,1000]ms
 * @return
 */
EXPORT_API Result track_get_comm_param(SOCKETFD socketFd, int index,int& type,int& cycle);
EXPORT_API Result track_get_comm_param_robot(SOCKETFD socketFd, int robotNum, int index,int& type,int& cycle);

/**
 * @brief 设置电弧跟踪左右补偿参数
 * @param index 跟踪文件号(工艺号)
 * @param TrackOffsetParam 电弧跟踪补偿参数
 * @return
 */
EXPORT_API Result track_set_lateral_offset_param(SOCKETFD socketFd, int index,const TrackOffsetParam& param);
EXPORT_API Result track_set_lateral_offset_param_robot(SOCKETFD socketFd, int robotNum, int index,const TrackOffsetParam& param);

/**
 * @brief 获取电弧跟踪左右补偿参数
 * @param index 跟踪文件号(工艺号)
 * @param TrackOffsetParam 电弧跟踪补偿参数
 * @return
 */
EXPORT_API Result track_get_lateral_offset_param(SOCKETFD socketFd, int index,TrackOffsetParam& param);
EXPORT_API Result track_get_lateral_offset_param_robot(SOCKETFD socketFd, int robotNum, int index,TrackOffsetParam& param);

/**
 * @brief 设置电弧跟踪高低补偿参数
 * @param index 跟踪文件号(工艺号)
 * @param TrackOffsetParam 电弧跟踪补偿参数
 * @return
 */
EXPORT_API Result track_set_vertical_offset_param(SOCKETFD socketFd, int index,const TrackOffsetParam& param);
EXPORT_API Result track_set_vertical_offset_param_robot(SOCKETFD socketFd, int robotNum, int index,const TrackOffsetParam& param);

/**
 * @brief 获取电弧跟踪高低补偿参数
 * @param index 跟踪文件号(工艺号)
 * @param TrackOffsetParam 电弧跟踪补偿参数
 * @return
 */
EXPORT_API Result track_get_vertical_offset_param(SOCKETFD socketFd, int index,TrackOffsetParam& param);
EXPORT_API Result track_get_vertical_offset_param_robot(SOCKETFD socketFd, int robotNum, int index,TrackOffsetParam& param);

/**
 * @brief 设置电弧跟踪路径数据记录
 * @param index 跟踪文件号(工艺号)
 * @param cycle 跟踪路径数据记录周期
 * @param type 跟踪路径数据记录方式0:距离；1:时间
 * @return
 */
EXPORT_API Result track_set_path_record_param(SOCKETFD socketFd, int index,double cycle ,int type);
EXPORT_API Result track_set_path_record_param_robot(SOCKETFD socketFd, int robotNum, int index,double cycle ,int type);

/**
 * @brief 获取电弧跟踪路径数据记录
 * @param index 跟踪文件号(工艺号)
 * @param cycle 跟踪路径数据记录周期
 * @param type 跟踪路径数据记录方式0:距离；1:时间
 * @return
 */
EXPORT_API Result track_get_path_record_param(SOCKETFD socketFd, int index,double& cycle ,int& type);
EXPORT_API Result track_get_path_record_param_robot(SOCKETFD socketFd, int robotNum, int index,double& cycle ,int& type);

#endif /* INCLUDE_API_TL_CRAFT_TRACK_H_ */