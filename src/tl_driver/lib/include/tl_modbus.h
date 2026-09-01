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

// ==================== 灵巧手（Modbus 从站） ====================

/**
 * @brief 获取运行灵巧手目标手势序列号
 * @param id Modbus 从站 ID
 * @param start_address 起始地址
 * @param address_length 地址长度
 * @param data 输出：接收数据
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result read_hand_gesture_sequence(SOCKETFD socketFd, int id, int start_address, int address_length, std::string& data);

/**
 * @brief 获取运行灵巧手动作序列号
 * @param id Modbus 从站 ID
 * @param start_address 起始地址
 * @param address_length 地址长度
 * @param data 输出：接收数据
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result read_hand_action_sequence(SOCKETFD socketFd, int id, int start_address, int address_length, std::string& data);

/**
 * @brief 设置灵巧手各自由度角度
 * @param id Modbus 从站 ID
 * @param start_address 起始地址
 * @param address_length 地址长度
 * @param data 设置的数据
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result write_hand_freedom_angle(SOCKETFD socketFd, int id, int start_address, size_t address_length, const std::vector<std::string>& data);

/**
 * @brief 设置灵巧手速度
 * @param id Modbus 从站 ID
 * @param start_address 起始地址
 * @param address_length 地址长度
 * @param data 设置的数据
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result write_hand_speed(SOCKETFD socketFd, int id, int start_address, size_t address_length, const std::vector<int>& data);

/**
 * @brief 设置灵巧手力阈值
 * @param id Modbus 从站 ID
 * @param start_address 起始地址
 * @param address_length 地址长度
 * @param data 设置的数据
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result write_hand_force_threshold(SOCKETFD socketFd, int id, int start_address, size_t address_length, const std::vector<int>& data);

/**
 * @brief 设置灵巧手角度跟随控制
 * @param id Modbus 从站 ID
 * @param start_address 起始地址
 * @param address_length 地址长度
 * @param data 设置的数据
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result write_hand_angle_follow(SOCKETFD socketFd, int id, int start_address, size_t address_length, const std::vector<bool>& data);

/**
 * @brief 设置灵巧手位置跟随控制
 * @param id Modbus 从站 ID
 * @param start_address 起始地址
 * @param address_length 地址长度
 * @param data 设置的数据
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result write_hand_position_follow(SOCKETFD socketFd, int id, int start_address, size_t address_length, const std::vector<bool>& data);

// ==================== 夹爪（Modbus 从站） ====================

/**
 * @brief 设置夹爪行程
 * @param id Modbus 从站 ID
 * @param data 设置的数据
 * @param addr_list 表示要写入寄存器地址数组
 * @param addr_length 表示要写入的连续寄存器数量
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result write_gripper_stroke(SOCKETFD socketFd, int id, const std::vector<int>& data, std::vector<int>& addr_list, int addr_length);

/**
 * @brief 松开夹爪
 * @param id Modbus 从站 ID
 * @param data 设置的数据
 * @param addr_list 表示要写入寄存器地址数组
 * @param addr_length 表示要写入的连续寄存器数量
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result write_gripper_release(SOCKETFD socketFd, int id, const std::vector<bool>& data, std::vector<int>& addr_list, int addr_length);

/**
 * @brief 夹爪力控夹取
 * @param id Modbus 从站 ID
 * @param data 设置的数据（包含三个 int 值）
 * @param addr_list 表示要写入寄存器地址数组
 * @param addr_length 表示要写入的连续寄存器数量
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result write_gripper_force_control_grab(SOCKETFD socketFd, int id, const std::vector<int>& data, std::vector<int>& addr_list, int addr_length);

/**
 * @brief 夹爪持续力控夹取
 * @param id Modbus 从站 ID
 * @param data 设置的数据
 * @param addr_list 表示要写入寄存器地址数组
 * @param addr_length 表示要写入的连续寄存器数量
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result write_gripper_continuous_force_control_grab(SOCKETFD socketFd, int id, const std::vector<int>& data, std::vector<int>& addr_list, int addr_length);

/**
 * @brief 设置夹爪达到指定位置
 * @param id Modbus 从站 ID
 * @param data 设置的数据
 * @param addr_list 表示要写入寄存器地址数组
 * @param addr_length 表示要写入的连续寄存器数量
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result write_gripper_move_to_position(SOCKETFD socketFd, int id, const std::vector<int>& data, std::vector<int>& addr_list, int addr_length);

/**
 * @brief 查询夹爪状态
 * @param id Modbus 从站 ID
 * @param data 输出：接收数据（hex 格式）
 * @param addr_list 表示要写入寄存器地址数组
 * @param addr_length 表示要写入的连续寄存器数量
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result read_gripper_status(SOCKETFD socketFd, int id, std::vector<std::string>& data, std::vector<int>& addr_list, int addr_length);

} // namespace tl

#endif /* TL_SDK_TL_MODBUS_H */
