/*
 * nrc_io.h
 *
 *  Created on: 2024年11月14日
 *      Author: yiixiong
 */

#ifndef INCLUDE_CPP_INTERFACE_NRC_IO_H_
#define INCLUDE_CPP_INTERFACE_NRC_IO_H_

#include "cpp/parameter/nrc_define.h"
#include "cpp/parameter/nrc_io_parameter.h"

/**
 * @brief IO型号查询
 * @param type IO型号和端口号
 */
EXPORT_API Result get_io_type(SOCKETFD socketFd, IOtype& io_type);

/**
 * @brief 设置数字输出
 * @param port 端口号 【1，最大端口数】
 * @param value 输出端口值 0 or 1
 */
EXPORT_API Result set_digital_output(SOCKETFD socketFd, int port, int value);

/**
 * @brief 一次获取所有数字输出
 * @param out 存储结果的数组，长度为64
 */
EXPORT_API Result get_digital_output(SOCKETFD socketFd, std::vector<int>& out);

/**
 * @brief 一次获取所有数字输入
 * @param in 存储结果的数组，长度为64
 */
EXPORT_API Result get_digital_input(SOCKETFD socketFd,std::vector<int>& in);

/**
 * @brief 设置模拟输出
 * @param port 端口号
 * @param value 数值,参数范围：0≤value≤10
 */
EXPORT_API Result set_analog_output(SOCKETFD socketFd,int port,double value);

/**
 * @brief 查询模拟输出
 * @param aout 模拟输出数组，最大长度为 64
 */
EXPORT_API Result get_analog_output(SOCKETFD socketFd,std::vector<double>& aout);

/**
 * @brief 查询模拟输入
 * @param ain 模拟输入数组，最大长度为 64
 */
EXPORT_API Result get_analog_input(SOCKETFD socketFd,std::vector<double>& ain);

/**
 * @brief 设置数字输入端口是否强制打开
 * @param port 输入端口
 * @param force 是否强制  1 强制  0 非强制
 * @param value 设置端口强制状态 0 或 1
 */
EXPORT_API Result set_force_digital_input(SOCKETFD socketFd, int port, int force, int value);

/**
 * @brief 获取当前已打开强制功能的输入端口及其状态
 * @param port 当前打开强制输入的端口
 * @param status 当前打开强制输入的端口的状态，下标与 port一一对应
 */
EXPORT_API Result get_force_digital_input(SOCKETFD socketFd, std::vector<int>& port, std::vector<double>& status);

/**
 * @brief 设置IO复位功能
 * @param robot 机器人编号(1-4)
 * @param type 设置类型, 1:远程IO复位, 2:切模式停止, 3:程序报错
 * @param enable 是否复位容器, 大小为所有IO板输出端口数，从第二块IO板开始，每一块IO板的起始位置为上一块IO板的末位端口的顺延
 * @param value 复位值容器, 大小为所有IO板输出端口数，从第二块IO板开始，每一块IO板的起始位置为上一块IO板的末位端口的顺延
 * @attention 每一次设置,需要设置的端口均要设置，否则回覆盖上一次的修改
 * @example 两块IO板,每块IO板的输出端口数为16,1-1端口选择复位,复位值1,2-1端口选择复位,复位值1,容器应设置为
 *          enable = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 1 ,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
 *          value = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 1 ,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
 */
EXPORT_API Result set_IO_reset_function(SOCKETFD socketFd, int robotNum, int type, std::vector<int> enable, std::vector<int> value);

/**
 * @brief 获取IO复位相关参数
 * @param robot 机器人编号(1-4)
 * @param type 设置类型, 1:远程IO复位, 2:切模式停止, 3:程序报错
 * @param enable 是否复位容器, 大小为所有IO板输出端口数，从第二块IO板开始，每一块IO板的起始位置为上一块IO板的末位端口的顺延
 * @param value 复位值容器, 大小为所有IO板输出端口数，从第二块IO板开始，每一块IO板的起始位置为上一块IO板的末位端口的顺延
 */
EXPORT_API Result get_IO_reset_function(SOCKETFD socketFd, int robotNum, int type, std::vector<int>& enable, std::vector<int>& value);

/**
 * @brief 设置IO/报警信息数字输出端口报警功能
 * @param msg 消息设置结构体,详见 AlarmdIO
 * @attention 注意！std::vector<AlarmdIO> msg 的大小和IO板端口数一致，每一次设置相应端口均不能忽略，否则回覆盖上一次的设置，
 *              若有多块IO板，端口顺序为上一块IO板末位端口的顺延
 * @example 如有两块IO板，每块IO板输入输出各16位，IO板1输出端口1设置错误消息"QQQ"，消息类型2，参数1，使能1；
 *              IO板2输出端口2设置错误消息"YYY"，消息类型2，参数1，使能1，容器应设置为
 *              msg[0].msgType = 2; msg[0].enable = 1; msg[0].value = 1; mas[0].msg = "QQQ";
 *              随后中间间隔15个,如msg[i].msgType = 0; msg[i].enable = 0; msg[i].value = 0; mas[i].msg = ""; 再设置
 *              msg[16].msgType = 2; msg[16].enable = 1; msg[16].value = 1; mas[16].msg = "YYY";
 */
EXPORT_API Result set_error_msg_of_digital_input(SOCKETFD socketFd, std::vector<AlarmdIO> msg);

/**
 * @brief 获取IO/报警信息数字输入端口报警信息设置
 * @param msg 详见 AlarmdIO
 */
EXPORT_API Result get_error_msg_of_digital_input(SOCKETFD socketFd, std::vector<AlarmdIO>& msg);

/**
 * @brief 设置IO/报警信息数字输出端口报警功能
 * @param msg 消息设置结构体,详见 AlarmdIO
 * @attention 注意！std::vector<AlarmdIO> msg 的大小和IO板端口数一致，每一次设置相应端口均不能忽略，否则回覆盖上一次的设置，
 *              若有多块IO板，端口顺序为上一块IO板末位端口的顺延
 * @example 如有两块IO板，每块IO板输入输出各16位，IO板1输出端口1设置错误消息"QQQ"，消息类型2，参数1，使能1；
 *              IO板2输出端口2设置错误消息"YYY"，消息类型2，参数1，使能1，容器应设置为
 *              msg[0].msgType = 2; msg[0].enable = 1; msg[0].value = 1; mas[0].msg = "QQQ";
 *              随后中间间隔15个,如msg[i].msgType = 0; msg[i].enable = 0; msg[i].value = 0; mas[i].msg = ""; 再设置
 *              msg[16].msgType = 2; msg[16].enable = 1; msg[16].value = 1; mas[16].msg = "YYY";
 */
EXPORT_API Result set_error_msg_of_digital_output(SOCKETFD socketFd, std::vector<AlarmdIO> msg);

/**
 * @brief 获取IO/报警信息数字输出端口报警信息设置
 * @param msg 详见 AlarmdIO
 */
EXPORT_API Result get_error_msg_of_digital_output(SOCKETFD socketFd, std::vector<AlarmdIO>& msg);

/**
 * @brief 远程参数设置
 * @param robot 机器人编号(1-4)
 * @param speed 远程模式默认速度[1,100]
 * @param start 是否自动启动
 * @param time IO重复触发屏蔽时间,单位 ms
 * @param startTime 启动确认时间
 * @param num 远程IO数量,22.07版本没有该参数,默认为10,24.03版本以上可修改该参数
 */
EXPORT_API Result set_remote_param(SOCKETFD socketFd, int robotNum, int speed, bool start, int time, int startTime, int num = 10);

/**
 * @brief 获取远程参数设置数据
 * @param robot 机器人编号(1-4)
 * @param speed 远程模式默认速度[1,100]
 * @param start 是否自动启动
 * @param time IO重复触发屏蔽时间,单位 ms
 * @param startTime 启动确认时间
 * @param num 远程IO数量,22.07版本没有该参数,22.07版本调用此接口num将返回-1
 */
EXPORT_API Result get_remote_param(SOCKETFD socketFd, int robotNum, int& speed, bool& start, int& time, int& startTime, int& num);

/**
 * @brief 设置远程IO功能
 * @param robot 机器人编号(1-4)
 * @param general 通用功能远程IO参数设置,如启动、暂停、停止,清除报警等,详见 RemoteControl
 * @param program 远程控制程序参数设置, 详见 RemoteProgram, program.size() 必须与 num 相等
 * @param num 远程IO数量,若是24.03版本必须与 set_remote_param 中的 num 一致,22.07版本没有该参数
 */
EXPORT_API Result set_remote_function(SOCKETFD socketFd, int robotNum, RemoteControl general,  std::vector<RemoteProgram> program, int num = 10);

/**
 * @brief 获取远程IO功能设置数据
 * @param robot 机器人编号(1-4)
 * @param num 远程IO数量,22.07版本没有该参数,22.07版本调用此接口num将返回-1
 * @param time IO重复触发屏蔽时间,单位 ms
 * @param general 通用功能远程IO参数设置,如启动、暂停、停止,清除报警等,详见 RemoteControl
 * @param program 远程控制程序参数设置
 */
EXPORT_API Result get_remote_function(SOCKETFD socketFd, int robotNum, int& num, int& time, RemoteControl& general,  std::vector<RemoteProgram>& program);

/**
 * @brief 设置远程状态提示功能
 * @param robot 机器人编号(1-4)
 * @param outagePort 断电保持数据恢复端口
 * @param outageValue 断电保持数据恢复端口有效值(0/1/2)
 * @param program 远程状态提示参数设置, 详见 RemoteProgram, 若是24.03版本,program.size()必须与set_remote_param中的num相等,22.07版本program.size()必须等于 10
 */
EXPORT_API Result set_remote_status_tips(SOCKETFD socketFd, int robotNum, int outagePort, int outageValue, std::vector<RemoteProgram> program);

/**
 * @brief 获取远程状态提示功能数据
 * @param robot 机器人编号(1-4)
 * @param outagePort 断电保持数据恢复端口
 * @param outageValue 断电保持数据恢复端口有效值(0/1/2)
 * @param num 远程IO数量(22.07版本默认10个，最多也只能是10个,24.03版本以上可以设置该数量)
 * @param program 远程控制程序参数设置
 */
EXPORT_API Result get_remote_status_tips(SOCKETFD socketFd, int robotNum, int& num, int& outagePort, int& outageValue,  std::vector<RemoteProgram>& program);

/**
 * @brief IO远程程序选择
 * @param robot 机器人编号(1-4)
 * @param program 详见 RemoteProgramSetting
 * @attention program.size(), 22.07版本固定大小为10
 */
EXPORT_API Result set_remote_program(SOCKETFD socketFd, int robotNum, std::vector<RemoteProgramSetting> program);

/**
 * @brief 获取IO远程程序设置数据
 * @param robot 机器人编号(1-4)
 * @param num 远程IO数量(22.07版本默认10个，最多也只能是10个,24.03版本以上可以设置该数量)
 * @param program 详见 RemoteProgramSetting
 */
EXPORT_API Result get_remote_program(SOCKETFD socketFd, int robotNum, int& num, std::vector<RemoteProgramSetting>& program);

/**
 * @brief 设置是否使能硬接及相关端口
 * @param enable 是否打开使能硬接
 * @param port1 使能硬接端口1
 * @param port2 使能硬接端口2
 */
EXPORT_API Result set_hard_enable_port(SOCKETFD socketFd, int enable, int port1, int port2);

/**
 * @brief 获取使能硬接开关是否打开及相关绑定端口
 * @param enable 是否打开使能硬接
 * @param port1 使能硬接端口1
 * @param port2 使能硬接端口2
 * @return
 */
EXPORT_API Result get_hard_enable_port(SOCKETFD socketFd, int& enable, int& port1, int& port2);

/**
 * @brief 设置IO安全设置参数
 * @param robot 机器人编号(1-4)
 * @param safeIO 详见结构体参数说明
 */
EXPORT_API Result set_safe_IO_function(SOCKETFD socketFd, int robotNum, SafeIO safeIO);

/**
 * @brief 获取IO安全设置参数
 * @param robot 机器人编号(1-4)
 * @param safeIO 详见结构体参数说明
 */
EXPORT_API Result get_safe_IO_function(SOCKETFD socketFd, int robotNum, SafeIO& safeIO);

#endif /* INCLUDE_API_NRC_IO_H_ */
