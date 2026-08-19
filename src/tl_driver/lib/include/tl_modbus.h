/**
 * @file tl_modbus.h
 * @brief Modbus 主站接口（namespace tl）
 */
#ifndef TL_SDK_TL_MODBUS_H
#define TL_SDK_TL_MODBUS_H

#include <vector>
#include "tl_types.h"

namespace tl
{


/**
 * @brief 设置主站参数
 * @param id 配方id参数，最多保存9个id
 * @param param 主站参数，详见 ModbusMasterParameter（type："TCP"/"RTU"）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result modbus_set_master_parameter(SOCKETFD socketFd, int id, const ModbusMasterParameter& param);

/**
 * @brief 打开主站
 * @param id 配方id参数，最多保存9个id，需与 modbus_set_master_parameter 设置的 id 对应
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result modbus_open_master(SOCKETFD socketFd, int id);

/**
 * @brief 读取输入状态（功能码 02H）
 * @param id 配方id参数，最多保存9个id
 * @param address 起始地址
 * @param quantity 读取数量
 * @param data 读取数据（失败时输出向量被清空）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result modbus_read_input_status(SOCKETFD socketFd, int id, int address, int quantity, std::vector<int>& data);

/**
 * @brief 读取线圈状态（功能码 01H）
 * @param id 配方id参数，最多保存9个id
 * @param address 起始地址
 * @param quantity 读取数量
 * @param data 读取数据（失败时输出向量被清空）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result modbus_read_coil_status(SOCKETFD socketFd, int id, int address, int quantity, std::vector<int>& data);

/**
 * @brief 读取保持寄存器（功能码 03H）
 * @param id 配方id参数，最多保存9个id
 * @param address 起始地址
 * @param quantity 读取数量
 * @param data 读取数据（失败时输出向量被清空）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result modbus_read_holding_registers(SOCKETFD socketFd, int id, int address, int quantity, std::vector<int>& data);

/**
 * @brief 读取输入寄存器（功能码 04H）
 * @param id 配方id参数，最多保存9个id
 * @param address 起始地址
 * @param quantity 读取数量
 * @param data 读取数据（失败时输出向量被清空）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result modbus_read_input_registers(SOCKETFD socketFd, int id, int address, int quantity, std::vector<int>& data);

/**
 * @brief 写单个线圈状态（功能码 05H）
 * @param id 配方id参数，最多保存9个id
 * @param address 线圈地址
 * @param data 写入的线圈状态值
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result modbus_write_signal_coil_status(SOCKETFD socketFd, int id, int address, int data);

/**
 * @brief 写单个保持寄存器（功能码 06H）
 * @param id 配方id参数，最多保存9个id
 * @param address 寄存器地址
 * @param data 写入的寄存器数据
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result modbus_write_signal_holding_registers(SOCKETFD socketFd, int id, int address, int data);

/**
 * @brief 写多个保持寄存器（功能码 10H）
 * @param id 配方id参数，最多保存9个id
 * @param address 起始寄存器地址
 * @param data 写入的寄存器数据
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result modbus_write_multiple_holding_registers(SOCKETFD socketFd, int id, int address, const std::vector<int>& data);

} // namespace tl

#endif /* TL_SDK_TL_MODBUS_H */
