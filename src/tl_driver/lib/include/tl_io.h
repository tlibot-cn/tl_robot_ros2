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
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_io_type(SOCKETFD socketFd, IOtype& io_type);

/**
 * @brief 设置数字输出
 * @param port 端口号 【1，最大端口数】
 * @param value 输出端口值 0 or 1
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误（如端口号越界）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_digital_output(SOCKETFD socketFd, int port, int value);

/**
 * @brief 一次获取所有数字输出
 * @param out 存储结果的数组，长度为64
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_digital_output(SOCKETFD socketFd, std::vector<int>& out);

/**
 * @brief 一次获取所有数字输入
 * @param in 存储结果的数组，长度为64
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_digital_input(SOCKETFD socketFd, std::vector<int>& in);

/**
 * @brief 设置模拟输出
 * @param port 端口号
 * @param value 数值，参数范围：0≤value≤10
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误（如数值越界）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_analog_output(SOCKETFD socketFd, int port, double value);

/**
 * @brief 查询模拟输出
 * @param aout 模拟输出数组，最大长度为 64
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_analog_output(SOCKETFD socketFd, std::vector<double>& aout);

/**
 * @brief 查询模拟输入
 * @param ain 模拟输入数组，最大长度为 64
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_analog_input(SOCKETFD socketFd, std::vector<double>& ain);

/**
 * @brief 设置远程IO功能
 * @param robotNum 机器人编号(1-4)
 * @param general 通用功能远程IO参数设置,如启动、暂停、停止,清除报警等,详见 RemoteControl
 * @param program 远程控制程序参数设置, 详见 RemoteProgram, program.size() 必须与 num 相等
 * @param num 远程IO数量,若是24.03版本必须与控制器端远程IO参数设置中的 num 一致,22.07版本没有该参数
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误（如 program.size() 与 num 不一致）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_remote_function(SOCKETFD socketFd, int robotNum, RemoteControl general, std::vector<RemoteProgram> program,
                           int num = 10);

/**
 * @brief 获取远程IO功能设置数据
 * @param robotNum 机器人编号(1-4)
 * @param num 远程IO数量,22.07版本没有该参数,22.07版本调用此接口num将返回-1
 * @param time IO重复触发屏蔽时间,单位 ms
 * @param general 通用功能远程IO参数设置,如启动、暂停、停止,清除报警等,详见 RemoteControl
 * @param program 远程控制程序参数设置
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_remote_function(SOCKETFD socketFd, int robotNum, int& num, int& time, RemoteControl& general,
                           std::vector<RemoteProgram>& program);

} // namespace tl

#endif /* TL_SDK_TL_IO_H */
