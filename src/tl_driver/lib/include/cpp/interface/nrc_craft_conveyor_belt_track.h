#ifndef INCLUDE_CPP_INTERFACE_NRC_CRAFT_CONVEYOR_BELT_TRACK_H_
#define INCLUDE_CPP_INTERFACE_NRC_CRAFT_CONVEYOR_BELT_TRACK_H_

#include "cpp/parameter/nrc_define.h"
#include "cpp/parameter/nrc_craft_conveyor_belt_track_parameter.h"

/**
 * @brief 设置传送带跟踪的基本参数
 * @param encoderVal 编码器的值 单位线
 * @param time 跟踪补偿时间 单位毫秒
 * @param encoderDirection 编码器的方向，用于确定正反转
 * @param encoderResolution 编码器的分辨率，指每转或每单位的脉冲数
 * @param maxEncoderVal 编码器的最大值，用于限制或标定编码器的读数范围  单位线
 * @param minEncoderVal 编码器的最小值，用于限制或标定编码器的读数范围  单位线
 * @param posRecordMode 位置记录模式   编码器/恒速设置
 * @param speed 传送带速度 单位毫米/秒
 * @param userCoord 用户坐标系统的标识，指定操作在哪个坐标下进行
 * @param conveyorID 传送带的ID
 * @param robot 机器人标识，指定哪个机器人与此操作相关
 * @param height 跟踪目标高度  传感器感知/跟踪指令示教
 * @param trackOnRunModeWithTargetOverrun 目标超限运行方式  等待下一个目标/跳行到追踪结束运行
 * @param compensationEncoderVal 跟踪补偿编码器值 单位线
 * @param robotName 机器人的名称，用于标识特定的机器人实例
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 * @note 此函数将配置传送带跟踪的基本参数，包括编码器的设置、传送带与机器人的关联等。
 */
EXPORT_API Result conveyor_belt_tracking_set_basic_parameter(SOCKETFD socketFd, int encoderVal, int time, int encoderDirection,
		double encoderResolution, double maxEncoderVal, double minEncoderVal, int posRecordMode, double speed, int userCoord,
		int conveyorID, int height, int trackOnRunModeWithTargetOverrun, int compensationEncoderVal);
EXPORT_API Result conveyor_belt_tracking_set_basic_parameter_robot(SOCKETFD socketFd, int robotNum, int encoderVal, int time, int encoderDirection,
		double encoderResolution, double maxEncoderVal, double minEncoderVal, int posRecordMode, double speed, int userCoord,
		int conveyorID, int height, int trackOnRunModeWithTargetOverrun, int compensationEncoderVal);

/**
 * @brief 设置传送带参数识别的配置
 * @param conveyorID 传送带的ID
 * @param detectSrcType 检测源的类型
 * @param capturePos 信号源参数
 * @param visionID 视觉系统的ID
 * @param visionIoFilterType 视觉输入输出过滤类型
 * @param visionLatchEncoderValueType 视觉锁存编码器值的类型
 * @param communication 通讯方式
 * @param sensorTrg 传感器触发方式
 * @param type 识别类型
 * @param robot 机器人标识，指定哪个机器人与此操作相关
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 * @note 此函数用于配置传送带的参数识别设置，包括检测源和识别方式的详细信息。
 */
EXPORT_API Result conveyor_belt_tracking_set_identification_parameter(SOCKETFD socketFd, int conveyorID, int detectSrcType, int capturePos, int visionID, int visionIoFilterType, int visionLatchEncoderValueType, int communication, int sensorTrg, int type);
EXPORT_API Result conveyor_belt_tracking_set_identification_parameter_robot(SOCKETFD socketFd, int robotNum, int conveyorID, int detectSrcType, int capturePos, int visionID, int visionIoFilterType, int visionLatchEncoderValueType, int communication, int sensorTrg, int type);

/**
 * @brief 设置传送带上的传感器标定参数
 * @param conveyorID 传送带的ID
 * @param robot 机器人标识，指定哪个机器人与此操作相关
 * @param sensorPos 传感器位置，长度 6
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 * @note 此函数用于配置传送带传感器的标定参数，以确保传感器能准确地测量传送带上物体的位置。
 */
EXPORT_API Result conveyor_belt_tracking_set_sensor_calibration(SOCKETFD socketFd, int conveyorID,const std::vector<double> sensorPos);
EXPORT_API Result conveyor_belt_tracking_set_sensor_calibration_robot(SOCKETFD socketFd, int robotNum, int conveyorID,const std::vector<double> sensorPos);

/**
 * @brief 设置传送带跟踪范围参数
 * @param conveyorID 传送带的ID
 * @param robot 机器人标识，指定哪个机器人与此操作相关
 * @param receLatestPos 最新接收位置
 * @param trackRangeXMax X轴最大跟踪范围
 * @param trackRangeYMax Y轴最大跟踪范围
 * @param trackRangeYMin Y轴最小跟踪范围
 * @param trackRangeZMax Z轴最大跟踪范围
 * @param trackRangeZMin Z轴最小跟踪范围
 * @param trackStartXPoint 跟踪起始X点
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 * @note 此函数用于配置传送带跟踪范围，以确保机器人能在指定的空间范围内进行有效跟踪。
 */
EXPORT_API Result conveyor_belt_tracking_set_tracking_range(SOCKETFD socketFd, int conveyorID, double receLatestPos, double trackRangeXMax, double trackRangeYMax, double trackRangeYMin, double trackRangeZMax, double trackRangeZMin, double trackStartXPoint);
EXPORT_API Result conveyor_belt_tracking_set_tracking_range_robot(SOCKETFD socketFd, int robotNum, int conveyorID, double receLatestPos, double trackRangeXMax, double trackRangeYMax, double trackRangeYMin, double trackRangeZMax, double trackRangeZMin, double trackStartXPoint);

/**
 * @brief 标定传送带跟踪等待点的位置参数
 * @param conveyorID 传送带的ID
 * @param isWait 无工件是否到指定点等待
 * @param delayDetectTime 等待延时的时间参数
 * @param pos 机器人的位置坐标，包括X、Y、Z轴坐标以及三个旋转轴的角度（弧度制）长度7
 * @param robot 机器人标识，指定哪个机器人与此操作相关
 * @return 函数执行的结果状态码，0表示成功，-1表示等待消息失败，-2表示找不到指定的客户端
 * @note 此函数用于标定与传送带相关的机器人跟踪等待点的位置参数。
 */
EXPORT_API Result conveyor_belt_tracking_set_tracking_wait_point(SOCKETFD socketFd, int conveyorID, bool isWait, double delayDetectTime,const std::vector<double> pos);
EXPORT_API Result conveyor_belt_tracking_set_tracking_wait_point_robot(SOCKETFD socketFd, int robotNum, int conveyorID, bool isWait, double delayDetectTime,const std::vector<double> pos);

/**
 * @brief  标定获得传感器位置
 * @param conveyorID 传送带的ID
 */
EXPORT_API Result conveyor_belt_set_sensor_calibration(SOCKETFD socketFd, int conveyorID);
EXPORT_API Result conveyor_belt_set_sensor_calibration_robot(SOCKETFD socketFd, int robotNum, int conveyorID);
/**
 * @brief 标定获得传感器位置
 * @param conveyorID 传送带的ID
 */
EXPORT_API Result conveyor_belt_calibrate_for_sensor_point(SOCKETFD socketFd, int conveyorID);
EXPORT_API Result conveyor_belt_calibrate_for_sensor_point_robot(SOCKETFD socketFd, int robotNum, int conveyorID);

/**
 * @brief 计算传感器位置
 * @param conveyorID 传送带的ID
 */
EXPORT_API Result conveyor_belt_calculate_for_sensor_point(SOCKETFD socketFd, int conveyorID);
EXPORT_API Result conveyor_belt_calculate_for_sensor_point_robot(SOCKETFD socketFd, int robotNum, int conveyorID);


/**
 * @brief 获取基本参数
 * @param conveyorID 传送带的ID
 */
EXPORT_API Result conveyor_belt_get_basic_paramters(SOCKETFD socketFd, int conveyorID, ConveyorBasicParams& param);
EXPORT_API Result conveyor_belt_get_basic_paramters_robot(SOCKETFD socketFd, int robotNum, int conveyorID, ConveyorBasicParams& param);

/**
 * @brief 获取识别参数
 * @param conveyorID 传送带的ID
 */
EXPORT_API Result conveyor_belt_get_identification_paramters(SOCKETFD socketFd, int conveyorID, ConveyorIdentificationParams& param);
EXPORT_API Result conveyor_belt_get_identification_paramters_robot(SOCKETFD socketFd, int robotNum, int conveyorID, ConveyorIdentificationParams& param);

/**
 * @brief 获取传感器参数
 * @param conveyorID 传送带的ID
 */
EXPORT_API Result conveyor_belt_get_sensor_paramters(SOCKETFD socketFd, int conveyorID, ConveyorSensorParams& param);
EXPORT_API Result conveyor_belt_get_sensor_paramters_robot(SOCKETFD socketFd, int robotNum, int conveyorID, ConveyorSensorParams& param);

/**
 * @brief 获取跟踪范围参数
 * @param conveyorID 传送带的ID
 */
EXPORT_API Result conveyor_belt_get_track_range_paramters(SOCKETFD socketFd, int conveyorID, ConveyorTrackRangeParams& param);
EXPORT_API Result conveyor_belt_get_track_range_paramters_robot(SOCKETFD socketFd, int robotNum, int conveyorID, ConveyorTrackRangeParams& param);

/**
 * @brief 获取等待点参数
 * @param conveyorID 传送带的ID
 */
EXPORT_API Result conveyor_belt_get_wait_point_paramters(SOCKETFD socketFd, int conveyorID, ConveyorWaitPointParams& param);
EXPORT_API Result conveyor_belt_get_wait_point_paramters_robot(SOCKETFD socketFd, int robotNum, int conveyorID, ConveyorWaitPointParams& param);

#endif /* INCLUDE_API_NRC_CRAFT_CONVEYOR_BELT_TRACK_H_ */
