/**
 * @file tl_io.h
 * @brief IO 接口（namespace tl）
 */
#ifndef TL_SDK_TL_IO_H
#define TL_SDK_TL_IO_H

#include <vector>
#include "tl_types.h"

namespace tl
{


/**
 * @brief IO型号查询
 * @param io_type IO型号和端口号，详见 IOtype 结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_io_type(SOCKETFD socketFd, IOtype& io_type);

/**
 * @brief 设置数字输出
 * @param port 端口号 【1，最大端口数】
 * @param value 输出端口值 0 or 1
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误（如端口号越界）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_digital_output(SOCKETFD socketFd, int port, int value);

/**
 * @brief 一次获取所有数字输出
 * @param out 存储结果的数组，长度为64
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_digital_output(SOCKETFD socketFd, std::vector<int>& out);

/**
 * @brief 一次获取所有数字输入
 * @param in 存储结果的数组，长度为64
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_digital_input(SOCKETFD socketFd, std::vector<int>& in);

/**
 * @brief 设置模拟输出
 * @param port 端口号
 * @param value 数值，参数范围：0≤value≤10
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误（如数值越界）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_analog_output(SOCKETFD socketFd, int port, double value);

/**
 * @brief 查询模拟输出
 * @param aout 模拟输出数组，最大长度为 64
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_analog_output(SOCKETFD socketFd, std::vector<double>& aout);

/**
 * @brief 查询模拟输入
 * @param ain 模拟输入数组，最大长度为 64
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_analog_input(SOCKETFD socketFd, std::vector<double>& ain);

/**
 * @brief 设置远程IO功能
 * @param robotNum 机器人编号(1-4)
 * @param general 通用功能远程IO参数设置,如启动、暂停、停止,清除报警等,详见 RemoteControl
 * @param program 远程控制程序参数设置, 详见 RemoteProgram, program.size() 必须与 num 相等
 * @param num 远程IO数量,若是24.03版本必须与控制器端远程IO参数设置中的 num 一致,22.07版本没有该参数
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误（如 program.size() 与 num 不一致）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION
 * 异常；-6=TIMEOUT 超时
 */
TL_API Result set_remote_function(SOCKETFD socketFd, int robotNum, RemoteControl general,
                                  std::vector<RemoteProgram> program, int num = 10);
/**
 * @brief 获取远程IO功能设置数据
 * @param robotNum 机器人编号(1-4)
 * @param num 远程IO数量,22.07版本没有该参数,调用此接口num将返回-1
 * @param time IO重复触发屏蔽时间,单位 ms
 * @param general 通用功能远程IO参数设置,如启动、暂停、停止,清除报警等,详见 RemoteControl
 * @param program 远程控制程序参数设置
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_remote_function(SOCKETFD socketFd, int robotNum, int& num, int& time,
                                  RemoteControl& general, std::vector<RemoteProgram>& program);


/**
 * @brief 设置数字输入端口是否强制打开
 * @param port 输入端口
 * @param force 是否强制 1 强制 0 非强制
 * @param value 设置端口强制状态 0 或 1
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_force_digital_input(SOCKETFD socketFd, int port, int force, int value);

/**
 * @brief 获取当前已打开强制功能的输入端口及其状态
 * @param port 当前打开强制输入的端口
 * @param status 当前打开强制输入的端口的状态，下标与 port一一对应
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_force_digital_input(SOCKETFD socketFd, std::vector<int>& port,
                                      std::vector<double>& status);

/**
 * @brief 设置IO复位功能
 * @param robotNum 机器人编号(1-4)
 * @param type 设置类型, 1:远程IO复位, 2:切模式停止, 3:程序报错
 * @param enable 是否复位容器,
 * 大小为所有IO板输出端口数,从第二块IO板开始，每一块IO板的起始位置为上一块IO板的末位端口的顺延
 * @param value 复位值容器
 * @attention 每一次设置,需要设置的端口均要设置，否则会覆盖上一次的修改
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_IO_reset_function(SOCKETFD socketFd, int robotNum, int type,
                                    std::vector<int> enable, std::vector<int> value);

/**
 * @brief 获取IO复位相关参数
 * @param robotNum 机器人编号(1-4)
 * @param type 设置类型, 1:远程IO复位, 2:切模式停止, 3:程序报错
 * @param enable 是否复位容器
 * @param value 复位值容器
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_IO_reset_function(SOCKETFD socketFd, int robotNum, int type,
                                    std::vector<int>& enable, std::vector<int>& value);

/**
 * @brief 设置IO/报警信息数字输入端口报警功能
 * @param msg 消息设置结构体数组,详见 AlarmdIO
 * @attention 注意！std::vector<AlarmdIO> msg
 * 的大小和IO板端口数一致，每一次设置相应端口均不能忽略，否则会覆盖上一次的设置
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_error_msg_of_digital_input(SOCKETFD socketFd, std::vector<AlarmdIO> msg);

/**
 * @brief 获取IO/报警信息数字输入端口报警信息设置
 * @param msg 详见 AlarmdIO
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_error_msg_of_digital_input(SOCKETFD socketFd, std::vector<AlarmdIO>& msg);

/**
 * @brief 设置IO/报警信息数字输出端口报警功能
 * @param msg 消息设置结构体数组,详见 AlarmdIO
 * @attention 注意！std::vector<AlarmdIO> msg
 * 的大小和IO板端口数一致，每一次设置相应端口均不能忽略，否则会覆盖上一次的设置
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_error_msg_of_digital_output(SOCKETFD socketFd, std::vector<AlarmdIO> msg);

/**
 * @brief 获取IO/报警信息数字输出端口报警信息设置
 * @param msg 详见 AlarmdIO
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_error_msg_of_digital_output(SOCKETFD socketFd, std::vector<AlarmdIO>& msg);

/**
 * @brief 远程参数设置
 * @param robotNum 机器人编号(1-4)
 * @param speed 远程模式默认速度[1,100]
 * @param start 是否自动启动
 * @param time IO重复触发屏蔽时间,单位 ms
 * @param startTime 启动确认时间
 * @param num 远程IO数量,22.07版本没有该参数,默认为10,24.03版本以上可修改该参数
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_remote_param(SOCKETFD socketFd, int robotNum, int speed, bool start, int time,
                               int startTime, int num = 10);

/**
 * @brief 获取远程参数设置
 * @param robotNum 机器人编号(1-4)
 * @param speed 远程模式默认速度[1,100]
 * @param start 是否自动启动
 * @param time IO重复触发屏蔽时间,单位 ms
 * @param startTime 启动确认时间
 * @param num 远程IO数量,22.07版本没有该参数,22.07版本调用此接口num将返回-1
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_remote_param(SOCKETFD socketFd, int robotNum, int& speed, bool& start, int& time,
                               int& startTime, int& num);

/**
 * @brief 设置远程状态提示功能
 * @param robotNum 机器人编号(1-4)
 * @param outagePort 断电保持数据恢复端口
 * @param outageValue 断电保持数据恢复端口的有效值(0/1/2)
 * @param program 远程状态提示参数设置,详见 RemoteProgram
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_remote_status_tips(SOCKETFD socketFd, int robotNum, int outagePort,
                                     int outageValue, std::vector<RemoteProgram> program);

/**
 * @brief 获取远程状态提示功能数据
 * @param robotNum 机器人编号(1-4)
 * @param num 远程IO数量(22.07版本默认10个，24.03版本以上可以设置该数量)
 * @param outagePort 断电保持数据恢复端口
 * @param outageValue 断电保持数据恢复端口有效值(0/1/2)
 * @param program 远程状态提示参数设置
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_remote_status_tips(SOCKETFD socketFd, int robotNum, int& num, int& outagePort,
                                     int& outageValue, std::vector<RemoteProgram>& program);

/**
 * @brief IO远程程序选择
 * @param robotNum 机器人编号(1-4)
 * @param program 详见 RemoteProgramSetting
 * @attention program.size() 22.07版本固定为10
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_remote_program(SOCKETFD socketFd, int robotNum,
                                 std::vector<RemoteProgramSetting> program);

/**
 * @brief 获取IO远程程序设置数据
 * @param robotNum 机器人编号(1-4)
 * @param num 远程IO数量(22.07版本默认10个，最多也只能是10个,24.03版本以上可以设置该数量)
 * @param program 详见 RemoteProgramSetting
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_remote_program(SOCKETFD socketFd, int robotNum, int& num,
                                 std::vector<RemoteProgramSetting>& program);

/**
 * @brief 设置是否使能硬接及相关端口
 * @param enable 是否打开使能硬接
 * @param port1 使能硬接端口1
 * @param port2 使能硬接端口2
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_hard_enable_port(SOCKETFD socketFd, int enable, int port1, int port2);

/**
 * @brief 获取使能硬接开关是否打开及相关绑定端口
 * @param enable 是否打开使能硬接
 * @param port1 使能硬接端口1
 * @param port2 使能硬接端口2
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_hard_enable_port(SOCKETFD socketFd, int& enable, int& port1, int& port2);

/**
 * @brief 设置IO安全设置参数
 * @param robotNum 机器人编号(1-4)
 * @param safeIO 详见 SafeIO 结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_safe_IO_function(SOCKETFD socketFd, int robotNum, SafeIO safeIO);

/**
 * @brief 获取IO安全设置参数
 * @param robotNum 机器人编号(1-4)
 * @param safeIO 详见 SafeIO 结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_safe_IO_function(SOCKETFD socketFd, int robotNum, SafeIO& safeIO);

} // namespace tl

#endif /* TL_SDK_TL_IO_H */
