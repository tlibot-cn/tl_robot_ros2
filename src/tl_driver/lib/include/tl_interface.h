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

// Windows 头文件（winerror.h）定义 ERROR 为 0，push/pop 保护枚举不被宏展开破坏
#pragma push_macro("ERROR")
#pragma push_macro("DEBUG")
#undef ERROR
#undef DEBUG

typedef enum
{
  DEBUG = 0,   ///< 调试
  INFO = 1,    ///< 一般（默认）
  WARNING = 2, ///< 警告
  ERROR = 3,   ///< 仅错误
} LogLevel;

#pragma pop_macro("DEBUG")
#pragma pop_macro("ERROR")

/**
 * @brief 设置日志级别
 * @param level 日志级别（LogLevel 枚举）
 */
TL_API void set_log_level(LogLevel level);


// ==================== 连接/版本 ====================

/**
 * @brief 获取版本信息（包含 SDK 封装层版本 + 底层控制器库版本）
 * @return 版本信息字符串，格式 "SDK v<sdk版本> (Base: <底层库版本>)"
 */
TL_API std::string get_library_version();

/**
 * @brief 设置连接超时时间，连接超过限制时间后直接返回错误
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
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
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result disconnect_robot(SOCKETFD socketFd);

/**
 * @brief 获得控制器连接状态
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_connection_status(SOCKETFD socketFd);

/**
 * @brief 设置是否打开断开后自动重连功能 默认关闭
 * @param reconnect true 打开
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_reconnect(SOCKETFD socketFd, bool reconnect);

/**
 * @brief 设置接收错误或警告信息的回调函数。
 * @param function 指向回调函数的指针。该回调函数在接收到错误或警告信息时被调用。
 * @note 回调函数由用户实现，用于处理接收到的错误信息。
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_receive_error_or_warnning_message_callback(
    SOCKETFD socketFd, void (*function)(int messageType, const char *message, int messageCode));

/**
 * @brief 收到控制器消息时触发设置的回调函数
 * @param function 指向回调函数的指针，签名 `void (int messageID, const char* message)`。
 *        回调收到控制器主动推送的消息 id 与消息内容。
 * @warning 回调函数内不能做耗时操作或阻塞（控制器消息线程内触发）。
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result recv_message(SOCKETFD socketFd, void (*function)(int messageID, const char *message));

/**
 * @brief 配置控制器有线网口 IP
 * @param name 配置名
 * @param address ip地址
 * @param gateway 网关
 * @param dns DNS
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_controller_ip(SOCKETFD socketFd, const std::string& name,
                                const std::string& address, const std::string& gateway,
                                const std::string& dns);

/**
 * @brief 获取控制器ID
 * @param id 输出：控制器ID字符串
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_controller_id(SOCKETFD socketFd, std::string& id);

/**
 * @brief 获取算法库版本
 * @param version 输出：算法库版本字符串
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_nexmotion_lib_version(SOCKETFD socketFd, std::string& version);

// ==================== 伺服/清错/状态 ====================

/**
 * @brief 伺服清错
 * @note
 * 出错前如果时伺服运行状态，清错后需要手动进行下电操作，释放控制器的占用状态才可以继续上电（清错后不能直接上电，先下电再上电）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result clear_error(SOCKETFD socketFd);

/**
 * @brief 设置伺服状态
 * @param state 0 停止 1 就绪
 * @warning 设置伺服就绪应该先确保系统没有错误 clear_servo_error(SOCKETFD socketFd)
 * 该函数只有伺服状态为0（停止状态）或1（就绪状态）时调用生效，伺服状态为2（报警状态）或3（运行状态）时不能直接设置伺服状态
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_servo_state(SOCKETFD socketFd, int state);

/**
 * @brief 获取伺服状态
 * @param status 接收获取结果 0：停止状态 1：就绪状态 2：报警状态 3：运行状态
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
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
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_robot_state(SOCKETFD socketFd, RobotState param);

/**
 * @brief 获取程序运行状态
 * @param status 程序运行状态
 *  - 0 停止
 *  - 1 暂停
 *  - 2 运行
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_robot_running_state(SOCKETFD socketFd, int& status);

/**
 * @brief 设置当前机器人DH参数
 * @param param 结构体参数（标准 DH 参数：alpha[6]/a[6]/theta[6]/d[6] + eulerAngle/mountingAngle）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_robot_dh_param(SOCKETFD socketFd, const RobotDHParam& param);

/**
 * @brief 恢复机械臂默认DH参数
 * @param robotNum 机器人编号，0 为默认（单机器人模式）；多机器人模式下为机器人序号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result restore_default_param_DH(SOCKETFD socketFd, int robotNum);

/**
 * @brief 获取当前机器人DH参数
 * @param param 结构体参数
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_robot_dh_param(SOCKETFD socketFd, RobotDHParam& param);

/**
 * @brief 查询碰撞防护等级
 * @param param 接收获取结果
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_collision_detection_param(SOCKETFD socketFd, CollisionPara& param);

/**
 * @brief 获取碰撞防护参数（24.03+ 固件）
 * @param param 输出：碰撞防护参数（详细参数在 CollisionSafeParam 结构体中说明）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_collision(SOCKETFD socketFd, CollisionSafeParam& param);

/**
 * @brief 设置碰撞防护参数（24.03+ 固件）
 * @param param 碰撞防护参数（详细参数在 CollisionSafeParam 结构体中说明）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_collision(SOCKETFD socketFd, CollisionSafeParam& param);

/**
 * @brief 获取电流环拖动示教灵敏度
 * @param sensitivity 输出参数，用于接收各关节电流环拖动示教灵敏度参数的向量，按实际轴数排列（6轴为
 * J1-J6，7轴为 J1-J7）
 * @return Result 操作结果，SUCCESS表示成功，其他值表示失败
 */
TL_API Result get_current_teach_sensitivity(SOCKETFD socketFd, std::vector<double>& sensitivity);

/**
 * @brief 查询指定关节（轴）的软件版本号 (SDO 0x100A)
 * @param[out] version 输出参数，用于接收查询到的软件版本号字符串
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
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
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
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
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_robot_type(SOCKETFD socketFd, RobotType& type);

/**
 * @brief 获取当前机器人编号（多机器人模式下用于区分机器人）
 * @param robot 输出：当前机器人编号（多机器人模式下的机器人序号）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_robot_switch(SOCKETFD socketFd, int& robot);

/**
 * @brief 设置当前机器人指定关节参数
 * @param id 关节编号，取值范围 [1,6]（6轴，对应 J1~J6）；7轴为 [1,7]（对应 J1~J7）
 * @param param 关节参数（结构体）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_robot_joint_param(SOCKETFD socketFd, int id, RobotJointParam& param);

/**
 * @brief 获取指定关节参数
 * @param id 关节编号，取值范围 [1,6]（6轴，对应 J1~J6）；7轴为 [1,7]（对应 J1~J7）
 * @param param 输出：关节参数（结构体）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_robot_joint_param(SOCKETFD socketFd, int id, RobotJointParam& param);

/**
 * @brief 获取笛卡尔空间参数
 * @param param 输出：用于接收笛卡尔空间运动参数的结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_cartesian_params(SOCKETFD socketFd, CartesianParam& param);

/**
 * @brief 7000端口状态返回的回调函数
 * 需要连接7000端口
  SOCKETFD fd7000 = connect_robot("192.168.1.13","7000");
 *
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result robot_state_callback(SOCKETFD socketFd, void (*function)(const char *state));

/**
 * @brief 获取关节当前温度
 * @param temperatures 输出：关节温度向量，按实际轴数排列（6轴为 J1-J6，7轴为 J1-J7）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_joint_temperature(SOCKETFD socketFd, std::vector<double>& temperatures);

/**
 * @brief 获取当前电机电流（24.03接口）
 * @param motor_current 输出：机器人电机电流，长度7，单位[‰]
 * @param motor_current_sync 输出：外部轴电机电流，长度5，单位[‰]
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_current_motor_current(SOCKETFD socketFd, std::vector<double>& motor_current,
                                        std::vector<double>& motor_current_sync);

/**
 * @brief 获取当前电机扭矩
 * @param motorTorque 输出：机器人扭矩，长度7，单位[%]
 * @param motorTorqueSync 输出：外部轴扭矩，长度5，单位[%]
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_current_motor_torque(SOCKETFD socketFd, std::vector<int>& motorTorque,
                                       std::vector<int>& motorTorqueSync);

// ==================== 运动 ====================

/**
 * @brief 关节运动
 * @param targetPosValue 点位数组，n个轴就赋值前n位数组,其余置0
 * @param vel 速度，参数范围：0<vel≤100 单位 %
 * @param coord 坐标系，参数范围：0≤coord≤3
 * @param acc 加速度，参数范围：0<acc≤100
 * @param dec 减速度，参数范围：0<dec≤100
 * @param isSync 是否同步模式 true同步 false不同步
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
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
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result robot_movel(SOCKETFD socketFd, MoveCmd moveCmd);

/**
 * @brief 设置当前模式的速度 有三种模式 示教模式，运行模式，远程模式
 * @param speed 速度，参数范围：0<speed≤100
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_speed(SOCKETFD socketFd, int speed);

/**
 * @brief 获得当前模式的速度 有三种模式 示教模式，运行模式，远程模式
 * @param speed 速度，参数范围：0<speed≤100
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_speed(SOCKETFD socketFd, int& speed);

/**
 * @brief 设置机器人当前模式
 * @param mode 模式 0：示教 1：远程 2：运行
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_current_mode(SOCKETFD socketFd, int mode);

/**
 * @brief 获取机器人当前模式
 * @param mode 当前模式 0：示教 1：远程 2：运行
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_current_mode(SOCKETFD socketFd, int& mode);

/**
 * @brief 获取机器人当前位置
 * @param coord 入参 指定需要查询的坐标的坐标系
 * @param pos 出参 存储返回结果点位的容器，长度7
 * @note 推荐使用枚举重载（Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER），避免魔法数字
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_current_position(SOCKETFD socketFd, int coord, std::vector<double>& pos);

/**
 * @brief 获取机器人当前位置（枚举坐标系重载）
 * @param coord 坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
 * @param pos 出参 存储返回结果点位的容器，长度7
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_current_position(SOCKETFD socketFd, Coord coord, std::vector<double>& pos);

/**
 * @brief 获取机器人当前指定关节点在基坐标系（直角坐标系）中的位置(2403版本专用)
 * @param axisNum 指定需要查询的关节
 * @param pos 存储返回结果点位的容器，长度7
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_joint_position(SOCKETFD socketFd, int axisNum, std::vector<double>& pos);

/**
 * @brief 获取点位是否可达
 * @param pos 目标点位坐标数据，容器长度为14：[0]坐标系 0：关节 1：直角 2：工具 3：用户 [1]:0 角度制
 * 1弧度制 [2]形态 [3]工具手坐标序号 [4]用户坐标序号 [5][6] 备用 [7-13] 点位信息
 * @param movetype 移动方式 "MOVJ" 或 "MOVL"
 * @param result 输出：点位是否可达
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_pos_reachable(SOCKETFD socketFd, std::vector<double> pos, std::string movetype,
                                bool& result);

/**
 * @brief 获取当前末端线速度和关节速度
 * @param lineSpeed 输出：末端线速度，单位[mm/s]
 * @param jointSpeed 输出：关节速度，长度7，单位[度/s]
 * @param jointSpeedSync 输出：外部轴关节速度，长度7，单位[度/s]
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_current_line_speed_and_joint_speed(SOCKETFD socketFd, double& lineSpeed,
                                                     std::vector<double>& jointSpeed,
                                                     std::vector<double>& jointSpeedSync);

/**
 * @brief 获取机器人当前坐标系
 * @param coord 坐标系 0：关节 1：基坐标 2：工具 3：用户
 * @note 推荐使用枚举重载（Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER），避免魔法数字
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_current_coord(SOCKETFD socketFd, int& coord);

/**
 * @brief 获取机器人当前坐标系（枚举重载）
 * @param coord 出参 坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_current_coord(SOCKETFD socketFd, Coord& coord);

/**
 * @brief 设置机器人当前坐标系
 * @param coord 坐标系 0：关节 1：基坐标 2：工具 3：用户
 * @note 推荐使用枚举重载（Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER），避免魔法数字
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_current_coord(SOCKETFD socketFd, int coord);

/**
 * @brief 设置机器人当前坐标系（枚举重载）
 * @param coord 坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_current_coord(SOCKETFD socketFd, Coord coord);

/**
 * @brief 查询全局GP点位
 * @param posName 全局位置名 例如 "GP0001"
 * @param pos 全局点位数组 长度14 前7位为点位的坐标、姿态等信息，后7位为机器人位置
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_global_position(SOCKETFD socketFd, std::string posName, std::vector<double>& pos);

/**
 * @brief 设置全局GP点位
 * @param posName 需要修改全局位置名 例如 "GP0001"
 * @param pos[14] [0]坐标系 0：关节 1：基坐标 2：工具 3：用户  [1]:0 角度制 1弧度制 [2]形态
 * [3]工具手坐标序号 [4]用户坐标序号 [5][6] 备用 [7-13] 点位信息
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_global_position(SOCKETFD socketFd, std::string posName, std::vector<double> pos);

/**
 * @brief 开始点动
 * @param axis 轴号
 * @param dir 方向
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result robot_start_jogging(SOCKETFD socketFd, int axis, bool dir);

/**
 * @brief 停止点动
 * @param axis 轴号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result robot_stop_jogging(SOCKETFD socketFd, int axis);

/**
 * @brief 设置电流环拖动示教灵敏度
 * @param sensitivity 包含各关节电流环拖动示教灵敏度参数的向量，按实际轴数排列（6轴为 J1-J6，7轴为
 * J1-J7），范围应在0-3之间
 * @return Result 操作结果，SUCCESS表示成功，其他值表示失败
 */
TL_API Result set_current_teach_sensitivity(SOCKETFD socketFd,
                                            const std::vector<double>& sensitivity);

/**
 * @brief 设置拖拽示教的拖拽方式
 * @param mode 拖拽模式  0-无  1-3D鼠标  2-力矩模式 3-位置 (22.07版本没有位置模式)
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_darg_mode(SOCKETFD socketFd, int mode);

/**
 * @brief 设置示教模式类型
 * @param type 0 点动 1 拖拽
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_teach_type(SOCKETFD socketFd, int type);

/**
 * @brief 获取示教模式类型
 * @param type 输出：0 点动 1 拖拽
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_teach_type(SOCKETFD socketFd, int& type);

/**
 * @brief 回到设定的零点
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result robot_go_home(SOCKETFD socketFd);

/**
 * @brief 回到控制器预设的复位点
 * @note 若控制器未配置复位点，接口可能返回错误码
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result robot_go_to_reset_position(SOCKETFD socketFd);

// ==================== 坐标变换/标定 ====================

/**
 * @brief 四元数转欧拉角
 * @param quat_vector 被转换的四元数向量，vector长度 = 3
 * @param rpy_res 接收欧拉角向量结果，vector长度 = 4
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_quat2rpy(SOCKETFD socketFd, std::vector<double> quat_vector,
                           std::vector<double>& rpy_res);

/**
 * @brief 欧拉角转四元数
 * @param rpy_vector 被转换的欧拉角向量，vector长度 = 4
 * @param quat_res 接收四元数向量结果，vector长度 = 3
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_rpy2quat(SOCKETFD socketFd, std::vector<double> rpy_vector,
                           std::vector<double>& quat_res);

/**
 * @brief 旋转矩阵转位姿
 * @param r_matrix 被转换的旋转矩阵，vector长度 = 9（行主序）
 * @param tr_res 接收位姿矩阵结果，vector长度 = 16（行主序）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_r2tr(SOCKETFD socketFd, std::vector<double> r_matrix,
                       std::vector<double>& tr_res);

/**
 * @brief 欧拉角转旋转矩阵
 * @param rpy_vector 被转换的欧拉角向量，vector长度 = 4
 * @param r_res 接收旋转矩阵结果，vector长度 = 9（行主序）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_rpy2r(SOCKETFD socketFd, std::vector<double> rpy_vector,
                        std::vector<double>& r_res);

/**
 * @brief 位姿转旋转矩阵
 * @param tr_matrix 被转换的位姿矩阵，vector长度 = 16（行主序）
 * @param r_res 接收旋转矩阵结果，vector长度 = 9（行主序）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_tr2r(SOCKETFD socketFd, std::vector<double> tr_matrix,
                       std::vector<double>& r_res);

/**
 * @brief 获取当前使用的工具手编号
 * @param toolNum 工具手编号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_tool_hand_number(SOCKETFD socketFd, int& toolNum);

/**
 * @brief 设置工具手编号
 * @param toolNum 工具手编号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_tool_hand_number(SOCKETFD socketFd, int toolNum);

/**
 * @brief 设置工具手参数
 * @param toolNum 工具手编号
 * @param param 要修改的参数 长度6 X轴偏移量 Y轴偏移量 Z轴偏移量 绕A轴旋转量 绕B轴旋转量 绕C轴旋转量
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_tool_hand_param(SOCKETFD socketFd, int toolNum, ToolParam param);

/**
 * @brief 获取当前工具手参数
 * @param toolNum 工具手编号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_tool_hand_param(SOCKETFD socketFd, int toolNum, ToolParam& param);

/**
 * @brief 获取工具坐标范围参数
 * @param tool_number 工具编号
 * @param[out] range_param 工具坐标范围参数结构体
 * @return Result 操作结果
 */
TL_API Result get_tool_coordinate_range(SOCKETFD socketFd, int tool_number,
                                        ToolCoordinateRange& range_param);

/**
 * @brief 设置工具坐标范围参数
 * @param tool_number 工具编号
 * @param range_param 工具坐标范围参数结构体
 * @return Result 操作结果
 */
TL_API Result set_tool_coordinate_range(SOCKETFD socketFd, int tool_number,
                                        const ToolCoordinateRange& range_param);

/**
 * @brief 设置用户坐标编号
 * @param userNum 用户坐标编号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_user_coord_number(SOCKETFD socketFd, int userNum);

/**
 * @brief 标定用户坐标
 * @param userNum 用户坐标编号
 * @param pos 坐标数据
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_user_coordinate_data(SOCKETFD socketFd, int userNum, std::vector<double> pos);

/**
 * @brief 设置笛卡尔参数为默认值
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_default_cartesian_params(SOCKETFD socketFd);

/**
 * @brief 计算坐标
 * @param userNumber 用户坐标编号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result calculate_user_coordinate(SOCKETFD socketFd, int userNumber);

/**
 * @brief 标定OXY
 * @param userNum 用户坐标编号
 * @param xyo 值 'X' 'Y' 'O'
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
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
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @note 推荐使用枚举重载（Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER），避免魔法数字
 */
TL_API Result get_origin_coord_to_target_coord(SOCKETFD socketFd, int originCoord,
                                               std::vector<double> originPos, int targetCoord,
                                               std::vector<double>& targetPos, bool& convert_state,
                                               int form = 0,
                                               const std::vector<double>& reference_pos = {});

/**
 * @brief 原坐标值转换为其他坐标值（无 convert_state 简化版）
 * @param originCoord 原坐标系 0 1 2 3 关节 基坐标 工具 用户
 * @param originPos 要进行转换的坐标值 [0,1,2,3,4,5,6]
 * @param targetCoord 目标坐标系 0 1 2 3 关节 基坐标 工具 用户
 * @param targetPos 转换后的坐标值（点位数组）
 * @param form 形态（默认 0，与 nrc 对齐）
 * @param reference_pos 参考点（默认空）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误或逆解失败；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @note 不带 convert_state 的简化版：逆解失败（convert_state=false）时统一返回 PARAM_ERR，
 *       调用方只需检查返回值；需要区分"转换执行成功但逆解失败"时请使用带 convert_state 的重载。
 */
TL_API Result get_origin_coord_to_target_coord(SOCKETFD socketFd, int originCoord,
                                               std::vector<double> originPos, int targetCoord,
                                               std::vector<double>& targetPos, int form = 0,
                                               const std::vector<double>& reference_pos = {});

/**
 * @brief 原坐标值转换为其他坐标值（枚举坐标系重载）
 * @param originCoord 原坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
 * @param originPos 要进行转换的坐标值 [0,1,2,3,4,5,6]，取值范围同 int 版本
 * @param targetCoord 目标坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
 * @param targetPos 转换后的坐标值（点位数组）
 * @param convert_state true-逆解成功, false-逆解失败.
 * @param form 形态
 * @param reference_pos 参考点
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @note 推荐使用枚举重载（Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER），避免魔法数字
 */
TL_API Result get_origin_coord_to_target_coord(SOCKETFD socketFd, Coord originCoord,
                                               std::vector<double> originPos, Coord targetCoord,
                                               std::vector<double>& targetPos, bool& convert_state,
                                               int form = 0,
                                               const std::vector<double>& reference_pos = {});

/**
 * @brief 原坐标值转换为其他坐标值（枚举坐标系，无 convert_state 简化版）
 * @param originCoord 原坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
 * @param originPos 要进行转换的坐标值 [0,1,2,3,4,5,6]
 * @param targetCoord 目标坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
 * @param targetPos 转换后的坐标值（点位数组）
 * @param form 形态（默认 0，与 nrc 对齐）
 * @param reference_pos 参考点（默认空）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误或逆解失败；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @note 不带 convert_state 的简化版：逆解失败（convert_state=false）时统一返回 PARAM_ERR，
 *       调用方只需检查返回值；需要区分"转换执行成功但逆解失败"时请使用带 convert_state 的重载。
 * @note 推荐使用枚举重载（Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER），避免魔法数字
 */
TL_API Result get_origin_coord_to_target_coord(SOCKETFD socketFd, Coord originCoord,
                                               std::vector<double> originPos, Coord targetCoord,
                                               std::vector<double>& targetPos, int form = 0,
                                               const std::vector<double>& reference_pos = {});

/**
 * @brief 查询四点标定结果
 * @param result 输出：四点标定结果（查询到的标记点数据）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_four_point(SOCKETFD socketFd, std::vector<double>& result);

/**
 * @brief 获取单圈值
 * @param single_cycle 输出：单圈值数组，长度7
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_single_cycle(SOCKETFD socketFd, std::vector<double>& single_cycle);

/**
 * @brief 获取当前使用的用户坐标编号
 * @param userNum 输出：当前使用的用户坐标编号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_user_coord_number(SOCKETFD socketFd, int& userNum);

/**
 * @brief 获取用户坐标参数
 * @param userNum 用户坐标编号（入参，指定要查询的用户坐标）
 * @param pos 输出：用户坐标参数
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_user_coord_para(SOCKETFD socketFd, int userNum, std::vector<double>& pos);

// ==================== 全局变量 ====================

/**
 * @brief 查询全局 GE 点位
 * @param posName 全局位置名，例如 "GE0001"
 * @param pos
 * 输出：全局点位数组，长度21；前7位为点位的坐标、姿态等信息，中间7位为机器人位置，后7位为外部轴位置
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_global_sync_position(SOCKETFD socketFd, const std::string& posName,
                                       std::vector<double>& pos);

/**
 * @brief 查询全局变量
 * @param varName 全局变量名，支持形式如 "GI001" / "GD001" / "GB001"
 * @param value 输出：变量值
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_global_variant(SOCKETFD socketFd, const std::string& varName, double& value);

/**
 * @brief 获取指定坐标下的位置
 * @param name 点位名（P点、GP点、E点、GE点），如 "GP0001"
 * @param targetCoord 目标坐标系 0：关节 1：基坐标 2：工具 3：用户
 * @param targetPos
 * 输出：转换后的位置，14位点位（坐标系,角度/弧度,形态/左右手,工具号,用户坐标号,预留,预留,1轴,2轴,3轴,4轴,5轴,6轴,7轴）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @note 推荐使用枚举重载（Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER），避免魔法数字
 */
TL_API Result get_target_coord_pos_value(SOCKETFD socketFd, std::string name, int targetCoord,
                                         std::vector<double>& targetPos);

/**
 * @brief 获取指定坐标下的位置（枚举坐标系重载）
 * @param name 点位名（P点、GP点、E点、GE点），如 "GP0001"
 * @param targetCoord 目标坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
 * @param targetPos
 * 输出：转换后的位置，14位点位（坐标系,角度/弧度,形态/左右手,工具号,用户坐标号,预留,预留,1轴,2轴,3轴,4轴,5轴,6轴,7轴）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @note 推荐使用枚举重载（Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER），避免魔法数字
 */
TL_API Result get_target_coord_pos_value(SOCKETFD socketFd, std::string name, Coord targetCoord,
                                         std::vector<double>& targetPos);

/**
 * @brief 获取局部 P 点
 * @param name 点位名，范围 P0001 - P9999
 * @param pos
 * 输出：转换后的位置，14位点位（坐标系,角度/弧度,形态/左右手,工具号,用户坐标号,预留,预留,1轴,2轴,3轴,4轴,5轴,6轴,7轴）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_local_pos_p_value(SOCKETFD socketFd, std::string name, std::vector<double>& pos);

// ==================== 独立轴 ====================

/**
 * @brief 获取拖拽结束标志
 * @param endFlag 输出：true 拖拽结束
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_drag_thread_is_end(SOCKETFD socketFd, bool& endFlag);

// ==================== 传感器 ====================

/**
 * @brief 获取六维力传感器数据
 * @param sensorData 用于接收六维力传感器数据的结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_sensor_6d_data(SOCKETFD socketFd, Sensor6DData& sensorData);

/**
 * @brief 获取六维力传感器的基础参数（质量、质心、标零状态）
 * @param baseParam 用于接收传感器基础参数的结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_sensor_base_param(SOCKETFD socketFd, SensorBaseParam& baseParam);

/**
 * @brief 执行六维力传感器标定
 * @param[out] success 标定是否成功的标志
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result mark_base_sensor(SOCKETFD socketFd, bool& success);

/**
 * @brief 设置六维力传感器通讯
 * @param params 用于设置六维力传感器通讯参数的结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_six_dimensional_force_communication_params(
    SOCKETFD socketFd, SixDimensionalForceCommunicationParams& params);
/**
 * @brief 获取六维力传感器通讯参数
 * @param params 六维力通讯参数，详见 SixDimensionalForceCommunicationParams
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_six_dimensional_force_communication_params(
    SOCKETFD socketFd, SixDimensionalForceCommunicationParams& params);


/**
 * @brief 获取六维力传感器通讯参数
 * @param params 输出：用于接收六维力传感器通讯参数的结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
// ==================== 协议接口层扩展封装 ====================

/**
 * @brief 设置重连成功后的回调函数
 * @param function 指向回调函数的指针，重连成功后会被调用
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_reconnect_callback(SOCKETFD socketFd, void (*function)());

/**
 * @brief 向控制器发送一条自定义消息
 * @param messageID 消息ID
 * @param message 消息内容
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result send_message(SOCKETFD socketFd, int messageID, const std::string& message);

/**
 * @brief 设置多机器人并行模式
 * @param open true 打开并行；false 关闭并行
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_robots_parallel(SOCKETFD socketFd, bool open);

/**
 * @brief 设置伺服命令字（SDO）
 * @param axisNum 机器人轴编号
 * @param index 命令字编码（对象字典索引）
 * @param subindex 命令字子编码
 * @param cmdvalue 要设置进去的值
 * @param size 命令字对应值的字节数
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_axis_sdo(SOCKETFD socketFd, int axisNum, unsigned int index,
                           unsigned int subindex, int cmdvalue, unsigned int size);

/**
 * @brief 读取轴的 SDO 值
 * @param axisNum 轴号
 * @param index 对象字典索引
 * @param subindex 子索引
 * @param size 数据大小（8/16/32 位）
 * @param value 输出：读取到的数值
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_axis_sdo(SOCKETFD socketFd, int axisNum, unsigned int index,
                           unsigned int subindex, unsigned int size, long long& value);

/**
 * @brief 读取轴的 SDO 字符串值
 * @param axisNum 轴号
 * @param index 对象字典索引
 * @param subindex 子索引
 * @param size 数据大小
 * @param value 输出：读取到的字符串值
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_axis_sdo_string(SOCKETFD socketFd, int axisNum, unsigned int index,
                                  unsigned int subindex, unsigned int size, std::string& value);

/**
 * @brief 查询示教盒的连接状态
 * @param connected 接收连接状态
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_teachbox_connection_status(SOCKETFD socketFd, bool& connected);

/**
 * @brief 获取控制器序列号 ID（C# 兼容接口）
 * @param id 返回控制器序列号 ID 的字符容器
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_controller_id_csharp(SOCKETFD socketFd, std::vector<char>& id);

/**
 * @brief 设置零点位置
 * @param axis 轴号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_axis_zero_position(SOCKETFD socketFd, int axis);

/**
 * @brief 设置外部轴零点位置
 * @param axis 轴号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_sync_axis_zero_position(SOCKETFD socketFd, int axis);

/**
 * @brief 设置零点偏移
 * @param axis 需要偏移的轴数
 * @param shift 偏移量，范围 -360° < shift < 360°
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_zero_pos_deviation(SOCKETFD socketFd, int axis, double shift);

/**
 * @brief 进行 4 点标记
 * @param point 标记点位编号，取值范围 0-3
 * @param status 标记状态：0 取消标记；1 标记
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_four_point_mark(SOCKETFD socketFd, int point, int status);

/**
 * @brief 4 点标定计算
 * @param L1 参数 L1
 * @param L2 参数 L2
 * @param result 输出：标定计算结果
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result four_point_calculation(SOCKETFD socketFd, double L1, double L2,
                                     std::vector<double>& result);

/**
 * @brief 将 4 点标定计算的结果写入机器人 DH 参数
 * @param apply 写入是否成功：成功/失败（1/0）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_result_for_DH(SOCKETFD socketFd, int& apply);

/**
 * @brief 切换当前机器人
 * @param robot 切换到目标机器人编号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_robot_switch(SOCKETFD socketFd, int robot);

/**
 * @brief 设置机器人笛卡尔参数
 * @param param 包含笛卡尔空间运动参数的结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_cartesian_params(SOCKETFD socketFd, const CartesianParam& param);

/**
 * @brief 记录工具手标定点（4/6/7 点标定，24.03 版本）
 * @param point 标定点下标，范围 0-6
 * @param toolNum 工具手编号，范围 1-999
 * @param calibrationType 标定类型：4、6 或 7（默认 7）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result tool_hand_point_calibrate(SOCKETFD socketFd, int point, int toolNum,
                                        int calibrationType = 7);

/**
 * @brief 计算工具手标定结果（4/6/7 点标定，24.03 版本）
 * @param toolNum 工具手编号，范围 1-999
 * @param calibrationType 标定类型：4、6 或 7（默认 7）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result tool_hand_point_calibrate_caculate(SOCKETFD socketFd, int toolNum,
                                                 int calibrationType = 7);

/**
 * @brief 清除工具手标定点（4/6/7 点标定，24.03 版本）
 * @param point 标定点下标，范围 0-6
 * @param toolNum 工具手编号，范围 1-999
 * @param calibrationType 标定类型：4、6 或 7（默认 7）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result tool_hand_point_calibrate_clear(SOCKETFD socketFd, int point, int toolNum,
                                              int calibrationType = 7);

/**
 * @brief 2 点标定/20 点标定：记录当前选择的工具手标定点
 * @param point 标定点 1、2
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result tool_hand_2_or_20_point_calibrate(SOCKETFD socketFd, int point);

/**
 * @brief 2 点标定/20 点标定：计算
 * @param calNum 标定计算点数（默认 1）
 * @param noCalZero true 校准工具尺寸+姿态，不校准零点；false 其它情况，校准零点
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result tool_hand_2_or_20_point_calibrate_caculate(SOCKETFD socketFd, int calNum = 1,
                                                         bool noCalZero = false);

/**
 * @brief 2 点标定/20 点标定：清除标定点
 * @param pointNum 清除的标定点编号 1-20
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result tool_hand_2_or_20_point_calibrate_clear(SOCKETFD socketFd, int pointNum);

/**
 * @brief 7 点标定：记录当前选择的工具手标定点
 * @param point 标定点 1-7
 * @param toolNum 工具手编号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result tool_hand_7_point_calibrate(SOCKETFD socketFd, int point, int toolNum);

/**
 * @brief 7 点标定：计算
 * @param toolNum 工具手编号
 * @param calibrationPointNum 标定点数量（默认 7）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result tool_hand_7_point_calibrate_caculate(SOCKETFD socketFd, int toolNum,
                                                   int calibrationPointNum = 7);

/**
 * @brief 7 点标定：清除标定点
 * @param pointNum 清除的标定点 1-7
 * @param toolNum 工具手编号
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result tool_hand_7_point_calibrate_clear(SOCKETFD socketFd, int pointNum, int toolNum);

/**
 * @brief 设置全局 GE 点位
 * @param posName 全局位置名，例如 "GE0001"
 * @param posInfo 全局点位数组，长度 21：前 7 位为点位坐标/姿态信息，中间 7 位为机器人位置，后 7
 * 位为外部轴位置
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_global_sync_position(SOCKETFD socketFd, const std::string& posName,
                                       std::vector<double> posInfo);

/**
 * @brief 设置全局变量
 * @param varName 全局变量名，例如 "GI001" / "GD001" / "GB001"
 * @param varValue 变量值
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_global_variant(SOCKETFD socketFd, const std::string& varName, double varValue);

/**
 * @brief 设置全局字符串变量
 * @param varName 全局变量名，例如 "GS001"
 * @param varValue 变量值
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_global_string(SOCKETFD socketFd, const std::string& varName,
                                const std::string& varValue);

/**
 * @brief 查询全局字符串变量
 * @param varName 全局变量名，例如 "GS001"
 * @param value 输出：变量值
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_global_string(SOCKETFD socketFd, const std::string& varName, std::string& value);

/**
 * @brief 获取机器人外部轴当前位置
 * @param pos 输出：点位数组，长度 5
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_current_extra_position(SOCKETFD socketFd, std::vector<double>& pos);

/**
 * @brief 获取机器人和外部轴的当前位置
 * @param coord 指定需要查询的坐标系
 * @param pos 输出：点位数组，长度 12
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_current_positon_and_extra_position(SOCKETFD socketFd, int coord,
                                                     std::vector<double>& pos);

/**
 * @brief 获取 4 轴机器人的形态（SCARA）
 * @param configuration 输出：1-左手；2-右手
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_robot_configuration(SOCKETFD socketFd, int& configuration);

/**
 * @brief 获取机器人逆运动学全解
 * @param transRPY 目标位姿，vector 长度 = 6（前 3 位位置，后 3 位欧拉角）
 * @param full_solution 输出：逆运动学全解结果，每组 6 个关节角
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_full_solution(SOCKETFD socketFd, std::vector<double> transRPY,
                                std::vector<std::vector<double>>& full_solution);

/**
 * @brief 获取静态寻位坐标
 * @param fileid 寻位文件号
 * @param tableid 寻位参数表号
 * @param delaytime 参数表延时
 * @param pos 输出：寻位坐标
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_static_search_position(SOCKETFD socketFd, int fileid, int tableid, int delaytime,
                                         std::vector<double>& pos);

/**
 * @brief 判断世界坐标系下的某点是否触发干涉区
 * @param pos 点的世界坐标 [x, y, z]
 * @param return_val 输出：true 触发；false 不触发
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_pos_trigger_interfer(SOCKETFD socketFd, std::vector<double> pos,
                                       bool& return_val);

/**
 * @brief 判断工具手（带干涉区立方体）在某位置是否触发干涉区
 * @param pos 工具手在世界坐标下的位姿 [x, y, z, a, b, c]
 * @param return_val 输出：true 触发；false 不触发
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_tool_trigger_interfer(SOCKETFD socketFd, std::vector<double> pos,
                                        bool& return_val);

/**
 * @brief 获取六维力传感器的数据
 * @param data 输出：传感器数据数组
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_sensor_data(SOCKETFD socketFd, std::vector<int>& data);

/**
 * @brief 获取六维力传感器的关节摩擦力补偿阈值
 * @param thresholds 输出：6 个关节阈值的数组
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_sensor_joint_thresholds(SOCKETFD socketFd, std::vector<int>& thresholds);

/**
 * @brief 获取通过传感器计算出的负载参数（质量、质心）
 * @param payloadParam 输出：负载参数结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_payload_param_by_sensor(SOCKETFD socketFd, PayloadParamBySensor& payloadParam);

/**
 * @brief 获取当前电机转速
 * @param motorSpeed 输出：机器人电机转速，长度 7，单位 [RPM]
 * @param motorSpeedSync 输出：外部轴电机转速，长度 5，单位 [RPM]
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_current_motor_speed(SOCKETFD socketFd, std::vector<int>& motorSpeed,
                                      std::vector<int>& motorSpeedSync);

/**
 * @brief 获取当前电机负载
 * @param motorPayload 输出：机器人电机负载，长度 7，单位 [%]
 * @param motorPayloadSync 输出：外部轴电机负载，长度 5，单位 [%]
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_current_motor_payload(SOCKETFD socketFd, std::vector<double>& motorPayload,
                                        std::vector<double>& motorPayloadSync);

/**
 * @brief 获取当前编码位置
 * @param current_encoded 输出：机器人当前编码位置，长度 7
 * @param current_encoded_sync 输出：外部轴当前编码位置，长度 5
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_current_encoded_pos(SOCKETFD socketFd, std::vector<double>& current_encoded,
                                      std::vector<double>& current_encoded_sync);

/**
 * @brief 获取局部 E 点
 * @param name 点位名，范围 E0001 - E9999
 * @param pos 输出：转换后的位置，21 位点位
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_local_pos_e_value(SOCKETFD socketFd, std::string name, std::vector<double>& pos);

/**
 * @brief 获取关节电压
 * @param joint_voltage 输出：机器人本体各关节电压数组
 * @param positioner_voltage 输出：外部轴各关节电压数组
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_joint_voltage(SOCKETFD socketFd, std::vector<double>& joint_voltage,
                                std::vector<double>& positioner_voltage);

/**
 * @brief 设置碰撞检测阈值
 * @param collisionpara
 * 结构体，包含指令位置响应时间、误差允许时间、碰撞检测阈值（点动/指令）、机器人轴数
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_collision_para(SOCKETFD socketFd, CollisionPara collisionpara);

/**
 * @brief 设置力矩参数（拖拽示教，24.03 版本支持）
 * @param param 力矩参数（详细参数在 DragTorqueParam 结构体中说明）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_drag_param(SOCKETFD socketFd, DragTorqueParam& param);

/**
 * @brief 获取力矩参数（拖拽示教，24.03 版本支持）
 * @param param 输出：力矩参数（详细参数在 DragTorqueParam 结构体中说明）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_drag_param(SOCKETFD socketFd, DragTorqueParam& param);

/**
 * @brief 设置位置拖动参数（笛卡尔空间线速度限制与关节空间速度限制，22.07 版本没有该功能）
 * @param param 位置拖动参数（详细参数在 DragParam 结构体中说明）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_position_dragParams(SOCKETFD socketFd, DragParam& param);

/**
 * @brief 设置控制器网络配置
 * @param name 控制器主机名
 * @param ip 新的 IP 地址
 * @param gateway 网关地址
 * @param dns DNS 服务器地址
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_controller_network_config(SOCKETFD socketFd, const std::string& name,
                                            const std::string& ip, const std::string& gateway,
                                            const std::string& dns);

/**
 * @brief 获取控制器当前的网络配置信息
 * @param name 输出：控制器主机名
 * @param address 输出：IP 地址
 * @param gateway 输出：网关地址
 * @param dns 输出：DNS 服务器地址
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_controller_network_config(SOCKETFD socketFd, std::string& name,
                                            std::string& address, std::string& gateway,
                                            std::string& dns);

/**
 * @brief 恢复网络出厂设置
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result restore_network_factory_settings(SOCKETFD socketFd);

} // namespace tl

#endif /* TL_SDK_TL_INTERFACE_H */
