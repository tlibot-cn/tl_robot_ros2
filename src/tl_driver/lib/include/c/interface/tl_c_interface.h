#ifndef INCLUDE_C_INTERFACE_TL_C_INTERFACE_H_
#define INCLUDE_C_INTERFACE_TL_C_INTERFACE_H_

#include "c/parameter/tl_define.h"
#include "c/parameter/tl_interface_parameter.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 连接控制器
 * @param ip 控制器ip,"192.168.1.13"
 * @param port 端口号,"6001"
 * @use 此函数是同步方式连接，因此函数会阻塞，直到返回连接结果。
 * @return -1-失败
 */
EXPORT_API SOCKETFD connect_robot_c(const char* ip, const char* port);

/**
 * @brief 断开控制器
 */
EXPORT_API int disconnect_robot_c(SOCKETFD socketFd);

/**
* @brief 伺服清错
* @note 出错前如果时伺服运行状态，清错后需要手动进行下电操作，释放控制器的占用状态才可以继续上电（清错后不能直接上电，先下电再上电）
*/
EXPORT_API int clear_error_c(SOCKETFD socketFd);
EXPORT_API int clear_error_robot_c(SOCKETFD socketFd, int robotNum);

/**
 * @brief 设置伺服状态
 * @param 0 停止 1 就绪
 * @warning 设置伺服就绪应该先确保系统没有错误 clear_servo_error(SOCKETFD socketFd)
 * 该函数只有伺服状态为0（停止状态）或1（就绪状态）时调用生效，伺服状态为2（报警状态）或3（运行状态）时不能直接设置伺服状态
 */
EXPORT_API int set_servo_state_c(SOCKETFD socketFd, int state);
EXPORT_API int set_servo_state_robot_c(SOCKETFD socketFd, int robotNum, int state);

/**
 * @brief 获取伺服状态
 * @param status 接收获取结果 0：停止状态 1：就绪状态 2：报警状态 3：运行状态
 */
EXPORT_API int get_servo_state_c(SOCKETFD socketFd, int* status);
EXPORT_API int get_servo_state_robot_c(SOCKETFD socketFd, int robotNum, int* status);

/**
 * @brief 机器人上电
 * @attention 调用该函数之前需要先调用set_servo_state(SOCKETFD socketFd,1)将伺服设置为1（就绪状态）
 * 			,机器人上电成功后调用 get_servo_state(SOCKETFD socketFd)为3伺服运行状态
 * @return 机器人当前伺服状态servoStatus
 * 该函数只有伺服状态为1（就绪状态）时调用生效
 */
EXPORT_API int set_servo_poweron_c(SOCKETFD socketFd);
EXPORT_API int set_servo_poweron_robot_c(SOCKETFD socketFd, int robotNum);

/**
 * @brief 机器人下电
 * @attention 机器人下电成功后调用 get_servo_state(SOCKETFD socketFd)为1伺服就绪状态
 * @return 机器人当前伺服状态servoStatus
 * 该函数只有伺服状态为3（运行状态）时调用生效
 */
EXPORT_API int set_servo_poweroff_c(SOCKETFD socketFd);
EXPORT_API int set_servo_poweroff_robot_c(SOCKETFD socketFd, int robotNum);

/**
 * @brief 获取机器人当前位置
 * @param[in] coord 指定需要查询的坐标的坐标系
 * @param[out] pos 存储返回结果点位的容器，长度7
 * @return result 函数调用结果
 */
EXPORT_API int get_current_position_c(SOCKETFD socketFd, int coord, double* pos);
EXPORT_API int get_current_position_robot_c(SOCKETFD socketFd, int robotNum, int coord, double* pos);

/**
 * @brief 获取机器人当前所有关节点的直角坐标
 * @param axisNum 指定需要查询的关节
 * @param pos 存储返回结果点位的容器，长度7
 */
EXPORT_API int get_joint_position_c(SOCKETFD socketFd, int axisNum, double* pos);
EXPORT_API int get_joint_position_robot_c(SOCKETFD socketFd, int robotNum, int axisNum, double* pos);

/**
 * @brief 获取机器人外部轴当前位置
 * @param pos 点位数组，长度5
 */
EXPORT_API int get_current_extra_position_c(SOCKETFD socketFd, double* pos);
EXPORT_API int get_current_extra_position_robot_c(SOCKETFD socketFd, int robotNum, double* pos);

/**
 * @brief 设置当前模式的速度 有三种模式 示教模式，运行模式，远程模式
 * @param speed 速度，参数范围：0<speed≤100
 */
EXPORT_API int set_speed_c(SOCKETFD socketFd, int speed);
EXPORT_API int set_speed_robot_c(SOCKETFD socketFd, int robotNum, int speed);

/**
 * @brief 获得当前模式的速度 有三种模式 示教模式，运行模式，远程模式
 * @param speed 速度，参数范围：0<speed≤100
 */
EXPORT_API int get_speed_c(SOCKETFD socketFd, int* speed);
EXPORT_API int get_speed_robot_c(SOCKETFD socketFd, int robotNum, int* speed);

/**
 * @brief 设置机器人当前坐标系
 * @param coord 坐标系 0：关节 1：直角 2：工具 3：用户
 */
EXPORT_API int set_current_coord_c(SOCKETFD socketFd, int coord);
EXPORT_API int set_current_coord_robot_c(SOCKETFD socketFd, int robotNum, int coord);

/**
 * @brief 获取机器人当前坐标系
 * @param coord 坐标系 0：关节 1：直角 2：工具 3：用户
 */
EXPORT_API int get_current_coord_c(SOCKETFD socketFd, int* coord);
EXPORT_API int get_current_coord_robot_c(SOCKETFD socketFd, int robotNum, int* coord);

/**
 * @brief 设置机器人当前模式
 * @param mode 模式 0：示教 1：远程 2：运行
 */
EXPORT_API int set_current_mode_c(SOCKETFD socketFd, int mode);
EXPORT_API int set_current_mode_robot_c(SOCKETFD socketFd, int robotNum, int mode);

/**
 * @brief 获取机器人当前模式
 * @param mode 当前模式 0：示教 1：远程 2：运行
 */
EXPORT_API int get_current_mode_c(SOCKETFD socketFd, int* mode);
EXPORT_API int get_current_mode_robot_c(SOCKETFD socketFd, int robotNum, int* mode);

/**
 * @brief 获取机器人运行状态
 * @param status 机器人运行状态
 *  - 0 停止
 *  - 1 暂停
 *  - 2 运行
 */
EXPORT_API int get_robot_running_state_c(SOCKETFD socketFd, int* status);

/**
* @brief 设置全局GP点位
* @param posName 需要修改全局位置名 例如 "GP0001"
* @param posInfo[14] [0]坐标系 0：关节 1：直角 2：工具 3：用户  [1]:0 角度制 1弧度制 [2]形态 [3]工具手坐标序号 [4]用户坐标序号 [5][6] 备用 [7-13] 点位信息
*/
EXPORT_API int set_global_position_c(SOCKETFD socketFd, const char* posName, double* posInfo);
EXPORT_API int set_global_position_robot_c(SOCKETFD socketFd, int robotNum, const char* posName, double* posInfo);

/**
 * @brief 查询全局GP点位
 * @param posName 全局位置名 例如 "GP0001"
 * @param pos 全局点位数组 长度14 前7位为点位的坐标、姿态等信息，后7位为机器人位置
 */
EXPORT_API int get_global_position_c(SOCKETFD socketFd, const char* posName, double* pos);
EXPORT_API int get_global_position_robot_c(SOCKETFD socketFd, int robotNum, const char* posName, double* pos);

/**
* @brief 设置全局GE点位
* @param posName 需要修改全局位置名 例如 "GE0001"
* @param posInfo[21] [0]坐标系 0：关节 1：直角 2：工具 3：用户  [1]:0 角度制 1弧度制 [2]形态 [3]工具手坐标序号 [4]用户坐标序号 [5][6] 备用
*                  [7-13] 机器人本体点位信息  [14-20]  外部轴点位信息
*/
EXPORT_API int set_global_sync_position_c(SOCKETFD socketFd, const char* posName, double* posInfo);
EXPORT_API int set_global_sync_position_robot_c(SOCKETFD socketFd, int robotNum, const char* posName, double* posInfo);

/**
 * @brief 查询全局GE点位
 * @param posName 全局位置名 例如 "GE0001"
 * @param pos 全局点位数组 长度21 前7位为点位的坐标、姿态等信息，中间7位为机器人位置 后7位为外部轴位置
 */
EXPORT_API int get_global_sync_position_c(SOCKETFD socketFd, const char* posName, double* pos);
EXPORT_API int get_global_sync_position_robot_c(SOCKETFD socketFd, int robotNum, const char* posName, double* pos);

/**
 * @brief 设置全局变量
 * @param varName 全局变量名 例如 "GI001" "GD001" "GB001"
 * @param varValue 变量值
 */
EXPORT_API int set_global_variant_c(SOCKETFD socketFd, const char* varName, double varValue);
EXPORT_API int set_global_variant_robot_c(SOCKETFD socketFd, int robotNum, const char* varName, double varValue);

/**
 * @brief 查询全局变量
 * @param varName 全局变量名 例如 "GI001" "GD001" "GB001"
 */
EXPORT_API int get_global_variant_c(SOCKETFD socketFd, const char* varName, double* vaule);
EXPORT_API int get_global_variant_robot_c(SOCKETFD socketFd, int robotNum, const char* varName, double* vaule);

/**
 * @brief 原坐标值转换为其他坐标值
 * @param originCoord 原坐标系    0 1 2 3 关节 直角 工具 用户
 * @param originPos 要进行转换的坐标值 [0,1,2,3,4,5,6]
 *        关节取值范围    0-6[-10000,10000]
 *        直角取值范围    0-2[-10000,10000] 3-6[-3.1416,3.1416]rad
 *        工具取值范围    0-2[-10000,10000] 3-6[-3.1416,3.1416]rad
 *        用户取值范围    0-2[-10000,10000] 3-6[-3.1416,3.1416]rad
 * @param targetCoord 目标坐标系  0 1 2 3 关节 直角 工具 用户
 * @param targetPos 转换后的坐标系
 */
EXPORT_API int get_origin_coord_to_target_coord_c(SOCKETFD socketFd, int originCoord, double* originPos, int targetCoord, double* targetPos);

/**
 * @brief 获取当前机器人DH参数
 * @param param 结构体参数
*/
EXPORT_API int get_robot_dh_param_c(SOCKETFD socketFd, CRobotDHParam* param);

/**
 * @brief 切换当前机器人
 * @param robot 切换到的机器人
*/
EXPORT_API int set_robot_switch_c(SOCKETFD socketFd, int robot);

/**
 * @brief 获取当前机器人
 * @param robot 当前机器人
*/
EXPORT_API int get_robot_switch_c(SOCKETFD socketFd, int* robot);

/**
 * @brief 获取当前机器人关节参数
 * @param id 关节参数[1,6]
 * @param param 结构体参数
 */
EXPORT_API int get_robot_joint_param_c(SOCKETFD socketFd, int id, CRobotJointParam* param);

/**
 * @brief 关节运动
 * @param targetPosValue 点位数组，n个轴就赋值前n位数组,其余置0
 * @param vel 速度，参数范围：0<vel≤100 单位 %
 * @param coord 坐标系，参数范围：0≤coord≤3
 * @param acc 加速度，参数范围：0<acc≤100
 * @param dec 减速度，参数范围：0<dec≤100
 * @param isSync 是否同步模式 true同步 false不同步
 */
EXPORT_API int robot_movej_c(SOCKETFD socketFd, int coord, double vel, double acc, double dec, double* targetPos);

/**
 * @brief 直线运动
 * @param targetPosValue 点位数组，n个轴就赋值前n位数组,其余置0
 * @param vel 速度，参数范围：0<vel≤1000 单位mm/s
 * @param coord 坐标系，参数范围：0≤coord≤3
 * @param acc 加速度，参数范围：0<acc≤100
 * @param dec 减速度，参数范围：0<dec≤100
 * @param isSync 是否同步模式 true同步 false不同步
 */
EXPORT_API int robot_movel_c(SOCKETFD socketFd, int coord, double vel, double acc, double dec, double* targetPos);

/**
 * @brief 设置伺服SDO值
 * @param axisNum  轴号（从 1 开始）
 * @param index    SDO 索引（如 6040）
 * @param subindex SDO 子索引
 * @param value    要写入的数值
 * @param size     写入字节数（常见：8/16/32）
 * @return 结果码（0 表示 SUCCESS，负值见错误码定义）
 */
EXPORT_API int set_axis_sdo_c(SOCKETFD socketFd, int axisNum, unsigned int index, unsigned int subindex, int cmdvalue, unsigned int size);
/**
 * @brief 新建独立轴参数(22.07版本没有该功能)
 * @param param 独立轴参数(详细参数在CIndependentAxisParam结构体中说明)
 */
EXPORT_API int new_independent_axis_param_c(SOCKETFD socketFd, int axis_num);
EXPORT_API int new_independent_axis_param_robot_c(SOCKETFD socketFd, int robotNum,int axis_num);
/**
 * @brief 修改独立轴参数，新建后才能修改(22.07版本没有该功能)
 * @param param 独立轴参数(详细参数在CIndependentAxisParam结构体中说明)
 */
EXPORT_API int modify_independent_axis_param_c(SOCKETFD socketFd, CIndependentAxisParam param);
EXPORT_API int modify_independent_axis_param_robot_c(SOCKETFD socketFd, int robotNum,CIndependentAxisParam param);

/**
 * @brief 获取独立轴参数(22.07版本没有该功能)
 * @param param 独立轴参数(详细参数在CIndependentAxisParam结构体中说明)
 * @param num 独立轴编号
 */
EXPORT_API int get_independent_axis_param_c(SOCKETFD socketFd,int num,CIndependentAxisParam* param);
EXPORT_API int get_independent_axis_param_robot_c(SOCKETFD socketFd, int robotNum,int num,CIndependentAxisParam* param);

/**
 * @brief 获取独立轴总数(22.07版本没有该功能)
 * @param sum 独立轴总数
 */
EXPORT_API int get_axis_sum_c(SOCKETFD socketFd, int* sum);
EXPORT_API int get_axis_sum_robot_c(SOCKETFD socketFd, int robotNum,int* sum);

/**
 * @brief 删除某个独立轴(22.07版本没有该功能)
 * @param num 将要删除的独立轴
 */
EXPORT_API int delete_axis_sum_c(SOCKETFD socketFd, int num);
EXPORT_API int delete_axis_sum_robot_c(SOCKETFD socketFd, int robotNum,int num);

/**
 * @brief 查询独立轴位置(22.07版本没有该功能)
 * @param num 要查询的独立轴
 * @param currentPos 独立轴位置
 */
EXPORT_API int get_axis_position_c(SOCKETFD socketFd, int num,double* currentPos);
EXPORT_API int get_axis_position_robot_c(SOCKETFD socketFd, int robotNum,int num,double* currentPos);

/**
 * @brief 独立轴零点标定(22.07版本没有该功能)
 * @param num 要标定的独立轴
 */
EXPORT_API int independent_axis_zero_calibration_c(SOCKETFD socketFd, int num);
/**
 * @brief 独立控制轴PV运动 仅支持外部轴(22.07版本没有该功能)
 * @param param 运动参数(详细参数在CIndependentAxisRun结构体中说明)
 */
EXPORT_API int set_independent_axis_PV_run_c(SOCKETFD socketFd, CIndependentAxisRun param);
/**
 * @brief 独立控制轴PV停止 仅支持外部轴(22.07版本没有该功能)
 * @param num 轴编号
 */
EXPORT_API int set_independent_axis_PV_stop_c(SOCKETFD socketFd, int num);
/**
 * @brief 取消轴独立控制运动 仅支持外部轴(22.07版本没有该功能)
 * @param num 轴编号
 */
EXPORT_API int cancel_independent_axis_move_c(SOCKETFD socketFd, int num);

/**
 * @brief 独立轴回零(22.07版本没有该功能)
 * @param num 轴编号
 */
EXPORT_API int independent_axis_homing_c(SOCKETFD socketFd, int num);
/**
 * @brief 独立轴回零停止(22.07版本没有该功能)
 * @param num 轴编号
 */
EXPORT_API int independent_axis_homing_stop_c(SOCKETFD socketFd, int num);
/**
 * @brief 独立轴点动(22.07版本没有该功能)
 * @param param 点动参数(详细参数在IndependentAxisRun结构体中说明)
 */
EXPORT_API int independent_axis_jog_c(SOCKETFD socketFd, CIndependentAxisRun param);

#ifdef __cplusplus
}
#endif
#endif /* INTERFACE_C_API_TL_C_INTERFACE_H_ */
