/**
 * @file tl_interface.h
 * @brief 连接/伺服/运动/坐标/传感器接口（namespace tl）
 *
 * 仅含常用接口。
 */
#ifndef TL_SDK_TL_INTERFACE_H
#define TL_SDK_TL_INTERFACE_H

#include <string>
#include <vector>
#include "tl_types.h"

namespace tl
{

// ==================== 日志控制 ====================

typedef enum
{
  DEBUG = 0,   ///< 调试
  INFO = 1,    ///< 一般（默认）
  WARNING = 2, ///< 警告
  ERROR = 3,   ///< 仅错误
} LogLevel;

/**
 * @brief 设置日志级别
 * @param level 日志级别（LogLevel 枚举）
 */
TL_API void set_log_level(LogLevel level);


// ==================== 连接/版本 ====================

/**
 * @brief 获取库信息
 * @return 库版本相关信息
 */
TL_API std::string get_library_version();

/**
 * @brief 设置连接超时时间，连接超过限制时间后直接返回错误
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_connect_timeout_seconds(int timeoutSeconds);

/*
 * @brief 连接控制器
 * @param ip 控制器ip,"192.168.1.13"
 * @param port 端口号,"6001"
 * @use 此函数是同步方式连接，因此函数会阻塞，直到返回连接结果。
 * @return -1-失败
 */
TL_API SOCKETFD connect_robot(const std::string& ip, const std::string& port);

/**
 * @brief 连接控制器 UDP（7000 伺服端口）
 * @param ip 控制器ip,"192.168.1.13"
 * @param port 端口号,"7000"
 * @return 成功返回 socket 句柄；失败返回 -1
 */
TL_API SOCKETFD connect_robot_udp(const std::string& ip, const std::string& port);

/**
 * @brief 断开控制器
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result disconnect_robot(SOCKETFD socketFd);

/**
 * @brief 获得控制器连接状态
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_connection_status(SOCKETFD socketFd);

/**
 * @brief 设置是否打开断开后自动重连功能 默认关闭
 * @param reconnect true 打开
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_reconnect(SOCKETFD socketFd, bool reconnect);

/**
 * @brief 设置接收错误或警告信息的回调函数。
 * @param function 指向回调函数的指针。该回调函数在接收到错误或警告信息时被调用。
 * @note 回调函数由用户实现，用于处理接收到的错误信息。
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_receive_error_or_warnning_message_callback(SOCKETFD socketFd,
                                                      void (*function)(int messageType, const char *message,
                                                                       int messageCode));

// ==================== 伺服/清错/状态 ====================

/**
 * @brief 伺服清错
 * @note
 * 出错前如果时伺服运行状态，清错后需要手动进行下电操作，释放控制器的占用状态才可以继续上电（清错后不能直接上电，先下电再上电）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result clear_error(SOCKETFD socketFd);

/**
 * @brief 设置伺服状态
 * @param state 0 停止 1 就绪
 * @warning 设置伺服就绪应该先确保系统没有错误 clear_servo_error(SOCKETFD socketFd)
 * 该函数只有伺服状态为0（停止状态）或1（就绪状态）时调用生效，伺服状态为2（报警状态）或3（运行状态）时不能直接设置伺服状态
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_servo_state(SOCKETFD socketFd, int state);

/**
 * @brief 获取伺服状态
 * @param status 接收获取结果 0：停止状态 1：就绪状态 2：报警状态 3：运行状态
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_servo_state(SOCKETFD socketFd, int& status);

/**
 * @brief 机器人上电
 * @attention 调用该函数之前需要先调用set_servo_state(SOCKETFD socketFd,1)将伺服设置为1（就绪状态）
 * 			,机器人上电成功后调用 get_servo_state(SOCKETFD socketFd)为3伺服运行状态
 * @return 机器人当前伺服状态servoStatus
 * 该函数只有伺服状态为1（就绪状态）时调用生效
 */
TL_API Result set_servo_poweron(SOCKETFD socketFd);

/**
 * @brief 机器人下电
 * @attention 机器人下电成功后调用 get_servo_state(SOCKETFD socketFd)为1伺服就绪状态
 * @return 机器人当前伺服状态servoStatus
 * 该函数只有伺服状态为3（运行状态）时调用生效
 */
TL_API Result set_servo_poweroff(SOCKETFD socketFd);

/**
 * @brief 7000端口查询状态
 * 需要连接7000端口
  SOCKETFD fd7000 = connect_robot("192.168.1.13","7000");
 *
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_robot_state(SOCKETFD socketFd, RobotState param);

/**
 * @brief 获取程序运行状态
 * @param status 程序运行状态
 *  - 0 停止
 *  - 1 暂停
 *  - 2 运行
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_robot_running_state(SOCKETFD socketFd, int& status);

/**
 * @brief 获取当前机器人DH参数
 * @param param 结构体参数
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_robot_dh_param(SOCKETFD socketFd, RobotDHParam& param);

/**
 * @brief 查询碰撞防护等级
 * @param param 接收获取结果
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_collision_detection_param(SOCKETFD socketFd, CollisionPara& param);

/**
 * @brief 获取碰撞防护参数
 * @param param 输出：碰撞防护参数（结构体 CollisionPara 详见 tl_types.h）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_collision(SOCKETFD socketFd, CollisionPara& param);

/**
 * @brief 设置碰撞防护参数
 * @param param 碰撞防护参数（结构体 CollisionPara 详见 tl_types.h）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_collision(SOCKETFD socketFd, CollisionPara& param);

/**
 * @brief 获取电流环拖动示教灵敏度
 * @param sensitivity 输出参数，用于接收各关节电流环拖动示教灵敏度参数的向量，按实际轴数排列（6轴为 J1-J6，7轴为 J1-J7）
 * @return Result 操作结果，SUCCESS表示成功，其他值表示失败
 */
TL_API Result get_current_teach_sensitivity(SOCKETFD socketFd, std::vector<double>& sensitivity);

/**
 * @brief 查询指定关节（轴）的软件版本号 (SDO 0x100A)
 * @param[out] version 输出参数，用于接收查询到的软件版本号字符串
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result query_joint_software_version(SOCKETFD socketFd, int axisNum, std::string& version);

/**
 * @brief 获取机器人类型
 * @param type 输出：机器人类型
 *  - 1 六轴串联多关节
 *  - 2 四轴 SCARA
 *  - 3 四轴码垛
 *  - 4 四轴串联多关节
 *  - 5 单轴
 *  - 6 五轴串联多关节
 *  - 7 六轴协作
 *  - 8 二轴 SCARA
 *  - 9 三轴 SCARA
 *  - 10 三轴直角
 *  - 11 三轴异形
 *  - 12 七轴串联多关节
 *  - 13 SCARA 异形一
 *  - 14 四轴码垛丝杆
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @note 推荐使用枚举重载（RobotType::SIX_AXIS_SERIAL 等），避免魔法数字
 */
TL_API Result get_robot_type(SOCKETFD socketFd, int& type);

/**
 * @brief 获取机器人类型（枚举重载）
 * @param type 输出：机器人类型 RobotType 枚举
 *  - RobotType::SIX_AXIS_SERIAL 六轴串联多关节
 *  - RobotType::FOUR_AXIS_SCARA 四轴 SCARA
 *  - RobotType::FOUR_AXIS_PALLETIZING 四轴码垛
 *  - RobotType::FOUR_AXIS_SERIAL 四轴串联多关节
 *  - RobotType::SINGLE_AXIS 单轴
 *  - RobotType::FIVE_AXIS_SERIAL 五轴串联多关节
 *  - RobotType::SIX_AXIS_COLLABORATIVE 六轴协作
 *  - RobotType::TWO_AXIS_SCARA 二轴 SCARA
 *  - RobotType::THREE_AXIS_SCARA 三轴 SCARA
 *  - RobotType::THREE_AXIS_CARTESIAN 三轴直角
 *  - RobotType::THREE_AXIS_SPECIAL 三轴异形
 *  - RobotType::SEVEN_AXIS_SERIAL 七轴串联多关节
 *  - RobotType::SCARA_SPECIAL SCARA 异形一
 *  - RobotType::FOUR_AXIS_PALLETIZING_LEAD 四轴码垛丝杆
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_robot_type(SOCKETFD socketFd, RobotType& type);

/**
 * @brief 获取当前机器人编号（多机器人模式下用于区分机器人）
 * @param robot 输出：当前机器人编号（多机器人模式下的机器人序号）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_robot_switch(SOCKETFD socketFd, int& robot);

/**
 * @brief 获取指定关节参数
 * @param id 关节编号，取值范围 [1,6]（6轴，对应 J1~J6）；7轴为 [1,7]（对应 J1~J7）
 * @param param 输出：关节参数（结构体）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_robot_joint_param(SOCKETFD socketFd, int id, RobotJointParam& param);

/**
 * @brief 获取笛卡尔空间参数
 * @param param 输出：用于接收笛卡尔空间运动参数的结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_cartesian_params(SOCKETFD socketFd, CartesianParam& param);

/**
 * @brief 7000端口状态返回的回调函数
 * 需要连接7000端口
  SOCKETFD fd7000 = connect_robot("192.168.1.13","7000");
 *
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result robot_state_callback(SOCKETFD socketFd, void (*function)(const char *state));

// ==================== 运动 ====================

/**
 * @brief 关节运动
 * @param targetPosValue 点位数组，n个轴就赋值前n位数组,其余置0
 * @param vel 速度，参数范围：0<vel≤100 单位 %
 * @param coord 坐标系，参数范围：0≤coord≤3
 * @param acc 加速度，参数范围：0<acc≤100
 * @param dec 减速度，参数范围：0<dec≤100
 * @param isSync 是否同步模式 true同步 false不同步
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result robot_movej(SOCKETFD socketFd, MoveCmd moveCmd);

/**
 * @brief 直线运动
 * @param targetPosValue 点位数组，n个轴就赋值前n位数组,其余置0
 * @param vel 速度，参数范围：0<vel≤1000 单位mm/s
 * @param coord 坐标系，参数范围：0≤coord≤3
 * @param acc 加速度，参数范围：0<acc≤100
 * @param dec 减速度，参数范围：0<dec≤100
 * @param isSync 是否同步模式 true同步 false不同步
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result robot_movel(SOCKETFD socketFd, MoveCmd moveCmd);

/**
 * @brief 设置当前模式的速度（旧版兼容：类型=0 速度档位, segment/micro_dot 默认 0）
 * @param speed 速度百分比, 0<speed≤100（仅 type=0 生效）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT 超时
 */
TL_API Result set_speed(SOCKETFD socketFd, int speed);

/**
 * @brief 设置当前模式的速度（新版：完整参数）
 * @param type   速度类型 0:速度档位 1:关节坐标步进 2:直角坐标步进
 * @param speed  速度档位百分比(0,100]（type=0 时生效）
 * @param segment 步进档位 (0,20]（type=1/2 时生效）
 * @param micro_dot_speed_ACS 步进距离 (0,2000]（type=1/2 时生效）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT 超时
 */
TL_API Result set_speed(SOCKETFD socketFd, int type, int speed, int segment, int micro_dot_speed_ACS);

/**
 * @brief 获得当前模式的速度（旧版兼容：输出 int speed）
 * @param speed 速度百分比, 0<speed≤100
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT 超时
 */
TL_API Result get_speed(SOCKETFD socketFd, int& speed);

/**
 * @brief 获得当前模式的速度（新版：结构化输出）
 * @param param 输出：速度参数（RobotSpeedParam）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT 超时
 */
TL_API Result get_speed(SOCKETFD socketFd, RobotSpeedParam& param);

/**
 * @brief 设置全局速度段参数
 * @param param 速度段参数（名称列表 + 速度值列表）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT
 */
TL_API Result set_speed_segment(SOCKETFD socketFd, const RobotSpeedSegmentParam& param);

/**
 * @brief 查询全局速度段参数
 * @param param 输出：当前速度段参数
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT
 */
TL_API Result get_speed_segment(SOCKETFD socketFd, RobotSpeedSegmentParam& param);

/**
 * @brief 设置机器人当前模式
 * @param mode 模式 0：示教 1：远程 2：运行
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_current_mode(SOCKETFD socketFd, int mode);

/**
 * @brief 获取机器人当前模式
 * @param mode 当前模式 0：示教 1：远程 2：运行
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_current_mode(SOCKETFD socketFd, int& mode);

/**
 * @brief 获取机器人当前位置
 * @param coord 入参 指定需要查询的坐标的坐标系
 * @param pos 出参 存储返回结果点位的容器，长度7
 * @note 推荐使用枚举重载（Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER），避免魔法数字
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_current_position(SOCKETFD socketFd, int coord, std::vector<double>& pos);

/**
 * @brief 获取机器人当前位置（枚举坐标系重载）
 * @param coord 坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
 * @param pos 出参 存储返回结果点位的容器，长度7
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_current_position(SOCKETFD socketFd, Coord coord, std::vector<double>& pos);

/**
 * @brief 获取机器人当前指定关节点在基坐标系（直角坐标系）中的位置(2403版本专用)
 * @param axisNum 指定需要查询的关节
 * @param pos 存储返回结果点位的容器，长度7
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_joint_position(SOCKETFD socketFd, int axisNum, std::vector<double>& pos);

/**
 * @brief 获取机器人当前坐标系
 * @param coord 坐标系 0：关节 1：基坐标 2：工具 3：用户
 * @note 推荐使用枚举重载（Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER），避免魔法数字
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_current_coord(SOCKETFD socketFd, int& coord);

/**
 * @brief 获取机器人当前坐标系（枚举重载）
 * @param coord 出参 坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_current_coord(SOCKETFD socketFd, Coord& coord);

/**
 * @brief 设置机器人当前坐标系
 * @param coord 坐标系 0：关节 1：基坐标 2：工具 3：用户
 * @note 推荐使用枚举重载（Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER），避免魔法数字
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_current_coord(SOCKETFD socketFd, int coord);

/**
 * @brief 设置机器人当前坐标系（枚举重载）
 * @param coord 坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_current_coord(SOCKETFD socketFd, Coord coord);

/**
 * @brief 查询全局GP点位
 * @param posName 全局位置名 例如 "GP0001"
 * @param pos 全局点位数组 长度14 前7位为点位的坐标、姿态等信息，后7位为机器人位置
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_global_position(SOCKETFD socketFd, std::string posName, std::vector<double>& pos);

/**
 * @brief 设置全局GP点位
 * @param posName 需要修改全局位置名 例如 "GP0001"
 * @param pos[14] [0]坐标系 0：关节 1：基坐标 2：工具 3：用户  [1]:0 角度制 1弧度制 [2]形态 [3]工具手坐标序号
 * [4]用户坐标序号 [5][6] 备用 [7-13] 点位信息
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_global_position(SOCKETFD socketFd, std::string posName, std::vector<double> pos);

/**
 * @brief 开始点动
 * @param axis 轴号
 * @param dir 方向
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result robot_start_jogging(SOCKETFD socketFd, int axis, bool dir);

/**
 * @brief 停止点动
 * @param axis 轴号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result robot_stop_jogging(SOCKETFD socketFd, int axis);

/**
 * @brief 设置电流环拖动示教灵敏度
 * @param sensitivity 包含各关节电流环拖动示教灵敏度参数的向量，按实际轴数排列（6轴为 J1-J6，7轴为 J1-J7），范围应在0-3之间
 * @return Result 操作结果，SUCCESS表示成功，其他值表示失败
 */
TL_API Result set_current_teach_sensitivity(SOCKETFD socketFd, const std::vector<double>& sensitivity);

/**
 * @brief 设置拖拽示教的拖拽方式
 * @param mode 拖拽模式  0-无  1-3D鼠标  2-力矩模式 3-位置 (22.07版本没有位置模式)
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_darg_mode(SOCKETFD socketFd, int mode);

/**
 * @brief 设置示教模式类型
 * @param type 0 点动 1 拖拽
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_teach_type(SOCKETFD socketFd, int type);

/**
 * @brief 获取示教模式类型
 * @param type 输出：0 点动 1 拖拽
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_teach_type(SOCKETFD socketFd, int& type);

/**
 * @brief 回到设定的零点
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result robot_go_home(SOCKETFD socketFd);

/**
 * @brief 回到控制器预设的复位点
 * @note 若控制器未配置复位点，接口可能返回错误码
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result robot_go_to_reset_position(SOCKETFD socketFd);

// ==================== 坐标变换/标定 ====================

/**
 * @brief 四元数转欧拉角
 * @param quat_vector 被转换的四元数向量，vector长度 = 3
 * @param rpy_res 接收欧拉角向量结果，vector长度 = 4
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_quat2rpy(SOCKETFD socketFd, std::vector<double> quat_vector, std::vector<double>& rpy_res);

/**
 * @brief 欧拉角转四元数
 * @param rpy_vector 被转换的欧拉角向量，vector长度 = 4
 * @param quat_res 接收四元数向量结果，vector长度 = 3
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_rpy2quat(SOCKETFD socketFd, std::vector<double> rpy_vector, std::vector<double>& quat_res);

/**
 * @brief 旋转矩阵转位姿
 * @param r_matrix 被转换的旋转矩阵，vector长度 = 9（行主序）
 * @param tr_res 接收位姿矩阵结果，vector长度 = 16（行主序）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_r2tr(SOCKETFD socketFd, std::vector<double> r_matrix, std::vector<double>& tr_res);

/**
 * @brief 欧拉角转旋转矩阵
 * @param rpy_vector 被转换的欧拉角向量，vector长度 = 4
 * @param r_res 接收旋转矩阵结果，vector长度 = 9（行主序）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_rpy2r(SOCKETFD socketFd, std::vector<double> rpy_vector, std::vector<double>& r_res);

/**
 * @brief 位姿转旋转矩阵
 * @param tr_matrix 被转换的位姿矩阵，vector长度 = 16（行主序）
 * @param r_res 接收旋转矩阵结果，vector长度 = 9（行主序）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_tr2r(SOCKETFD socketFd, std::vector<double> tr_matrix, std::vector<double>& r_res);

/**
 * @brief 获取当前使用的工具手编号
 * @param toolNum 工具手编号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_tool_hand_number(SOCKETFD socketFd, int& toolNum);

/**
 * @brief 设置工具手编号
 * @param toolNum 工具手编号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_tool_hand_number(SOCKETFD socketFd, int toolNum);

/**
 * @brief 设置工具手参数
 * @param toolNum 工具手编号
 * @param param 要修改的参数 长度6 X轴偏移量 Y轴偏移量 Z轴偏移量 绕A轴旋转量 绕B轴旋转量 绕C轴旋转量
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_tool_hand_param(SOCKETFD socketFd, int toolNum, ToolParam param);

/**
 * @brief 获取当前工具手参数
 * @param toolNum 工具手编号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_tool_hand_param(SOCKETFD socketFd, int toolNum, ToolParam& param);

/**
 * @brief 获取工具坐标范围参数
 * @param tool_number 工具编号
 * @param[out] range_param 工具坐标范围参数结构体
 * @return Result 操作结果
 */
TL_API Result get_tool_coordinate_range(SOCKETFD socketFd, int tool_number, ToolCoordinateRange& range_param);

/**
 * @brief 设置工具坐标范围参数
 * @param tool_number 工具编号
 * @param range_param 工具坐标范围参数结构体
 * @return Result 操作结果
 */
TL_API Result set_tool_coordinate_range(SOCKETFD socketFd, int tool_number, const ToolCoordinateRange& range_param);

/**
 * @brief 设置用户坐标编号
 * @param userNum 用户坐标编号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_user_coord_number(SOCKETFD socketFd, int userNum);

/**
 * @brief 标定用户坐标（旧版：vector<double> 参数，无联动信息）
 * @deprecated 请使用 UserCoordParam 重载版本
 * @param userNum 用户坐标编号
 * @param pos 坐标数据
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API TL_DEPRECATED("use UserCoordParam overload instead") Result set_user_coordinate_data(SOCKETFD socketFd, int userNum, std::vector<double> pos);

/**
 * @brief 标定用户坐标（新版：UserCoordParam 结构体，含类型/联动）
 * @param userNum 用户坐标编号
 * @param param 用户坐标参数结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_user_coordinate_data(SOCKETFD socketFd, int userNum, const UserCoordParam& param);

/**
 * @brief 设置笛卡尔参数为默认值
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_default_cartesian_params(SOCKETFD socketFd);

/**
 * @brief 计算坐标
 * @param userNumber 用户坐标编号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result calculate_user_coordinate(SOCKETFD socketFd, int userNumber);

/**
 * @brief 标定OXY
 * @param userNum 用户坐标编号
 * @param xyo 值 'X' 'Y' 'O'
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result calibration_oxy(SOCKETFD socketFd, int userNum, std::string xyo);

/**
 * @brief 原坐标值转换为其他坐标值(点位精确到小数点后四位)
 * @param originCoord 原坐标系    0 1 2 3 关节 基坐标 工具 用户
 * @param originPos 要进行转换的坐标值 [0,1,2,3,4,5,6]
 *        关节取值范围    0-6[-10000,10000]
 *        基坐标取值范围    0-2[-10000,10000] 3-6[-3.1416,3.1416]rad
 *        工具取值范围    0-2[-10000,10000] 3-6[-3.1416,3.1416]rad
 *        用户取值范围    0-2[-10000,10000] 3-6[-3.1416,3.1416]rad
 * @param targetCoord 目标坐标系  0 1 2 3 关节 基坐标 工具 用户
 * @param targetPos 转换后的坐标值（点位数组）
 * @param convert_state true-逆解成功, false-逆解失败.
 * @param form 形态
 * @param reference_pos 参考点
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @note 推荐使用枚举重载（Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER），避免魔法数字
 */
TL_API Result get_origin_coord_to_target_coord(SOCKETFD socketFd, int originCoord, std::vector<double> originPos,
                                        int targetCoord, std::vector<double>& targetPos, bool& convert_state,
                                        int form = 0, const std::vector<double>& reference_pos = {});

/**
 * @brief 原坐标值转换为其他坐标值（无 convert_state 简化版）
 * @param originCoord 原坐标系 0 1 2 3 关节 基坐标 工具 用户
 * @param originPos 要进行转换的坐标值 [0,1,2,3,4,5,6]
 * @param targetCoord 目标坐标系 0 1 2 3 关节 基坐标 工具 用户
 * @param targetPos 转换后的坐标值（点位数组）
 * @param form 形态（默认 0，与 nrc 对齐）
 * @param reference_pos 参考点（默认空）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误或逆解失败；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @note 不带 convert_state 的简化版：逆解失败（convert_state=false）时统一返回 PARAM_ERR，
 *       调用方只需检查返回值；需要区分"转换执行成功但逆解失败"时请使用带 convert_state 的重载。
 */
TL_API Result get_origin_coord_to_target_coord(SOCKETFD socketFd, int originCoord, std::vector<double> originPos,
                                        int targetCoord, std::vector<double>& targetPos,
                                        int form = 0, const std::vector<double>& reference_pos = {});

/**
 * @brief 原坐标值转换为其他坐标值（枚举坐标系重载）
 * @param originCoord 原坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
 * @param originPos 要进行转换的坐标值 [0,1,2,3,4,5,6]，取值范围同 int 版本
 * @param targetCoord 目标坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
 * @param targetPos 转换后的坐标值（点位数组）
 * @param convert_state true-逆解成功, false-逆解失败.
 * @param form 形态
 * @param reference_pos 参考点
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @note 推荐使用枚举重载（Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER），避免魔法数字
 */
TL_API Result get_origin_coord_to_target_coord(SOCKETFD socketFd, Coord originCoord, std::vector<double> originPos,
                                        Coord targetCoord, std::vector<double>& targetPos, bool& convert_state,
                                        int form = 0, const std::vector<double>& reference_pos = {});

/**
 * @brief 原坐标值转换为其他坐标值（枚举坐标系，无 convert_state 简化版）
 * @param originCoord 原坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
 * @param originPos 要进行转换的坐标值 [0,1,2,3,4,5,6]
 * @param targetCoord 目标坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
 * @param targetPos 转换后的坐标值（点位数组）
 * @param form 形态（默认 0，与 nrc 对齐）
 * @param reference_pos 参考点（默认空）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误或逆解失败；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @note 不带 convert_state 的简化版：逆解失败（convert_state=false）时统一返回 PARAM_ERR，
 *       调用方只需检查返回值；需要区分"转换执行成功但逆解失败"时请使用带 convert_state 的重载。
 * @note 推荐使用枚举重载（Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER），避免魔法数字
 */
TL_API Result get_origin_coord_to_target_coord(SOCKETFD socketFd, Coord originCoord, std::vector<double> originPos,
                                        Coord targetCoord, std::vector<double>& targetPos,
                                        int form = 0, const std::vector<double>& reference_pos = {});

/**
 * @brief 查询四点标定结果
 * @param result 输出：四点标定结果（查询到的标记点数据）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_four_point(SOCKETFD socketFd, std::vector<double>& result);

/**
 * @brief 获取单圈值
 * @param single_cycle 输出：单圈值数组，长度7
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_single_cycle(SOCKETFD socketFd, std::vector<int>& single_cycle);

/**
 * @brief 获取当前使用的用户坐标编号
 * @param userNum 输出：当前使用的用户坐标编号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_user_coord_number(SOCKETFD socketFd, int& userNum);

/**
 * @brief 查询用户坐标参数（旧版：输出 vector，无联动信息）
 * @deprecated 请使用 UserCoordParam 重载版本
 * @param userNum 用户坐标编号
 * @param pos 输出：位置数组
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT
 */
TL_API TL_DEPRECATED("use UserCoordParam overload instead") Result get_user_coord_para(SOCKETFD socketFd, int userNum, std::vector<double>& pos);

/**
 * @brief 查询用户坐标参数（新版：UserCoordParam 结构体，含类型/联动）
 * @param userNum 用户坐标编号
 * @param param 输出：用户坐标参数结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT
 */
TL_API Result get_user_coord_para(SOCKETFD socketFd, int userNum, UserCoordParam& param);

/**
 * @brief 设置用户坐标参数（新版：UserCoordParam 结构体）
 * @param userNum 用户坐标编号
 * @param param 用户坐标参数结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT
 */
TL_API Result set_user_coord_para(SOCKETFD socketFd, int userNum, const UserCoordParam& param);

/**
 * @brief 设置字符串全局变量
 * @param varName 变量名，如 "GS001"
 * @param varValue 字符串变量值
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT
 */
TL_API Result set_global_string(SOCKETFD socketFd, const std::string& varName, const std::string& varValue);

/**
 * @brief 查询字符串全局变量
 * @param varName 变量名，如 "GS001"
 * @param value 输出：变量值
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT
 */
TL_API Result get_global_string(SOCKETFD socketFd, const std::string& varName, std::string& value);

/**
 * @brief 设置本体轴零点位置
 * @param axis 轴号（0=全部轴，1-7=指定轴）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT
 */
TL_API Result set_axis_zero_position(SOCKETFD socketFd, int axis);

/**
 * @brief 设置零点偏移
 * @param axis 轴号 (1-7)
 * @param shift 偏移量 (-360° < shift < 360°)
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT
 */
TL_API Result set_zero_pos_deviation(SOCKETFD socketFd, int axis, double shift);

/**
 * @brief 获取编码器单圈值
 * @param single_cycle 输出：单圈值数组，长度7
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT
 */
TL_API Result get_single_cycle(SOCKETFD socketFd, std::vector<int>& single_cycle);

// ==================== 工具手标定（VERSION_DEV 新增） ====================

/**
 * @brief 记录工具手标定点（2/15/20点标定 — 新版带 toolNum + calibrationType）
 * @param point 标定点下标 0-19
 * @param toolNum 工具手编号 1-999
 * @param calibrationType 标定类型 2/12/15/20/21
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT
 */
TL_API Result tool_hand_2_or_20_point_calibrate(SOCKETFD socketFd, int point, int toolNum, int calibrationType);

/**
 * @brief 计算工具手标定结果（新版）
 * @param toolNum 工具手编号 1-999
 * @param calibrationType 标定类型 2/12/15/20/21
 * @param noCalZero true=不校准零点 false=校准零点
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT
 */
TL_API Result tool_hand_2_or_20_point_calibrate_caculate(SOCKETFD socketFd, int toolNum, int calibrationType, bool noCalZero = false);

/**
 * @brief 清除工具手标定点状态（新版）
 * @param point 标定点下标 0-19（20=全部清除）
 * @param toolNum 工具手编号 1-999
 * @param calibrationType 标定类型
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT
 */
TL_API Result tool_hand_2_or_20_point_calibrate_clear(SOCKETFD socketFd, int point, int toolNum, int calibrationType);

/**
 * @brief 记录工具手 4/6/7 点标定点
 * @param point 标定点下标 0-6
 * @param toolNum 工具手编号 1-999
 * @param calibrationType 标定类型 6 或 7
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT
 */
TL_API Result tool_hand_7_point_calibrate(SOCKETFD socketFd, int point, int toolNum, int calibrationType = 7);

/**
 * @brief 计算工具手 4/6/7 点标定结果
 * @param toolNum 工具手编号 1-999
 * @param calibrationType 标定类型 6 或 7
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT
 */
TL_API Result tool_hand_7_point_calibrate_caculate(SOCKETFD socketFd, int toolNum, int calibrationType = 7);

/**
 * @brief 清除工具手 4/6/7 点标定点
 * @param point 标定点下标 0-6
 * @param toolNum 工具手编号 1-999
 * @param calibrationType 标定类型 6 或 7
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT
 */
TL_API Result tool_hand_7_point_calibrate_clear(SOCKETFD socketFd, int point, int toolNum, int calibrationType = 7);

/**
 * @brief 设置手眼标定类型并计算（新版带 calculateType）
 * @param visionNum 视觉ID
 * @param calculateType 计算类型 0=眼在手内 1=眼在手外
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT
 */
TL_API Result vision_hand_eye_calibration_calculation(SOCKETFD socketFd, int visionNum, int calculateType);

/**
 * @brief 获取传感器负载参数
 * @param param 输出：负载参数结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT
 */
TL_API Result get_payload_param_by_sensor(SOCKETFD socketFd, PayloadParamBySensor& param);

/**
 * @brief 获取关节电压
 * @param joint_voltage 输出：各关节电压
 * @param positioner_voltage 输出：外部轴各关节电压
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED …；-6=TIMEOUT
 */
TL_API Result get_joint_voltage(SOCKETFD socketFd, std::vector<double>& joint_voltage, std::vector<double>& positioner_voltage);

// ==================== 全局变量 ====================

/**
 * @brief 查询全局 GE 点位
 * @param posName 全局位置名，例如 "GE0001"
 * @param pos 输出：全局点位数组，长度21；前7位为点位的坐标、姿态等信息，中间7位为机器人位置，后7位为外部轴位置
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_global_sync_position(SOCKETFD socketFd, const std::string& posName, std::vector<double>& pos);

/**
 * @brief 查询全局变量
 * @param varName 全局变量名，支持形式如 "GI001" / "GD001" / "GB001"
 * @param value 输出：变量值
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_global_variant(SOCKETFD socketFd, const std::string& varName, double& value);

/**
 * @brief 获取指定坐标下的位置
 * @param name 点位名（P点、GP点、E点、GE点），如 "GP0001"
 * @param targetCoord 目标坐标系 0：关节 1：基坐标 2：工具 3：用户
 * @param targetPos 输出：转换后的位置，14位点位（坐标系,角度/弧度,形态/左右手,工具号,用户坐标号,预留,预留,1轴,2轴,3轴,4轴,5轴,6轴,7轴）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @note 推荐使用枚举重载（Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER），避免魔法数字
 */
TL_API Result get_target_coord_pos_value(SOCKETFD socketFd, std::string name, int targetCoord, std::vector<double>& targetPos);

/**
 * @brief 获取指定坐标下的位置（枚举坐标系重载）
 * @param name 点位名（P点、GP点、E点、GE点），如 "GP0001"
 * @param targetCoord 目标坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
 * @param targetPos 输出：转换后的位置，14位点位（坐标系,角度/弧度,形态/左右手,工具号,用户坐标号,预留,预留,1轴,2轴,3轴,4轴,5轴,6轴,7轴）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @note 推荐使用枚举重载（Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER），避免魔法数字
 */
TL_API Result get_target_coord_pos_value(SOCKETFD socketFd, std::string name, Coord targetCoord, std::vector<double>& targetPos);

/**
 * @brief 获取局部 P 点
 * @param name 点位名，范围 P0001 - P9999
 * @param pos 输出：转换后的位置，14位点位（坐标系,角度/弧度,形态/左右手,工具号,用户坐标号,预留,预留,1轴,2轴,3轴,4轴,5轴,6轴,7轴）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_local_pos_p_value(SOCKETFD socketFd, std::string name, std::vector<double>& pos);

// ==================== 独立轴 ====================

/**
 * @brief 获取拖拽结束标志
 * @param endFlag 输出：true 拖拽结束
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_drag_thread_is_end(SOCKETFD socketFd, bool& endFlag);

// ==================== 传感器 ====================

/**
 * @brief 获取六维力传感器数据
 * @param sensorData 用于接收六维力传感器数据的结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_sensor_6d_data(SOCKETFD socketFd, Sensor6DData& sensorData);

/**
 * @brief 获取六维力传感器的基础参数（质量、质心、标零状态）
 * @param baseParam 用于接收传感器基础参数的结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_sensor_base_param(SOCKETFD socketFd, SensorBaseParam& baseParam);

/**
 * @brief 执行六维力传感器标定
 * @param[out] success 标定是否成功的标志
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result mark_base_sensor(SOCKETFD socketFd, bool& success);

/**
 * @brief 设置六维力传感器通讯
 * @param params 用于设置六维力传感器通讯参数的结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_six_dimensional_force_communication_params(SOCKETFD socketFd,
                                                      SixDimensionalForceCommunicationParams& params);

/**
 * @brief 获取六维力传感器通讯参数
 * @param params 输出：用于接收六维力传感器通讯参数的结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_six_dimensional_force_communication_params(SOCKETFD socketFd,
                                                      SixDimensionalForceCommunicationParams& params);

} // namespace tl

#endif /* TL_SDK_TL_INTERFACE_H */
