/*
 * tl_modbus.h
 *
 *  Created on: 2025年3月19日
 *      Author: yiixiong
 */

#ifndef INCLUDE_CPP_INTERFACE_TL_MODBUS_H_
#define INCLUDE_CPP_INTERFACE_TL_MODBUS_H_

#include "cpp/parameter/tl_define.h"
#include "cpp/parameter/tl_modbus_parameter.h"

/**
 * @brief 设置末端io的can通讯参数
 * @param id 需要设置的id
 * @param port 端口
 * @param arbitration_baudrate 需要设置的仲裁波特率
 * @param data_baudrate 需要设置的数据波特率
 */
EXPORT_API Result set_terminal_io_can_param(SOCKETFD socketFd, int id,int port,int arbitration_baudrate,int data_baudrate);
/**
 * @brief 设置末端io的485通讯参数
 * @param id 需要设置的id
 * @param port 端口
 * @param baudrate 485波特率
 */
EXPORT_API Result set_terminal_io_485_param(SOCKETFD socketFd, int id,int port,int baudrate);
/**
 * @brief 末端modbus读数据
 * @param func_code 功能码
 * @param start_addr 起始地址
 * @param read_addr_num 读取数量
 * @param read_data 读到的数据(int数组)范围[0,65535]
 */
EXPORT_API Result terminal_io_read_modbus_data(SOCKETFD socketFd, int func_code,int start_addr,int read_addr_num,std::vector<int>& read_data);
/**
 * @brief 末端modbus写数据
 * @param func_code 功能码
 * @param start_addr 起始地址
 * @param send_data 写数据(int数组)范围[0,65535]
 */
EXPORT_API Result terminal_io_write_modbus_data(SOCKETFD socketFd, int func_code,int start_addr,int write_addr_num,std::vector<int>& read_data);
//==========================
//==========主站=============
//==========================
/**
 * @brief 设置主站参数
 * @param processNumber 配方id参数,最多保存9个id
 * @param TCPMasterParameter 参数
 */
EXPORT_API Result modbus_set_master_parameter(SOCKETFD socketFd, int id, const ModbusMasterParameter& param);

/**
 * @brief 打开主站
 * @param TCPMasterParameter 参数
 */
EXPORT_API Result modbus_open_master(SOCKETFD socketFd, int id);

/**
 * @brief 获取主站连接状态
 * @param TCPMasterParameter 参数
 */
EXPORT_API Result modbus_get_master_connection_status(SOCKETFD socketFd, int id, int& status);

/**
 * 功能码 01H
 */
EXPORT_API Result modbus_read_coil_status(SOCKETFD socketFd, int id, int address, int quantity, std::vector<int>& data);
/**
 * 功能码 02H
 */
EXPORT_API Result modbus_read_input_status(SOCKETFD socketFd, int id, int address, int quantity, std::vector<int>& data);
/**
 * 功能码 03H
 */
EXPORT_API Result modbus_read_holding_registers(SOCKETFD socketFd, int id, int address, int quantity, std::vector<int>& data);
/**
 * 功能码 04H
 */
EXPORT_API Result modbus_read_input_registers(SOCKETFD socketFd, int id, int address, int quantity, std::vector<int>& data);
/**
 * 功能码 05H
 */
EXPORT_API Result modbus_write_signal_coil_status(SOCKETFD socketFd, int id, int address, int data);
/**
 * 功能码 06H
 */
EXPORT_API Result modbus_write_signal_holding_registers(SOCKETFD socketFd, int id, int address, int data);
/**
 * 功能码 0FH
 */
EXPORT_API Result modbus_write_multiple_coil_status(SOCKETFD socketFd, int id, int address, const std::vector<int>& data);
/**
 * 功能码 10H
 */
EXPORT_API Result modbus_write_multiple_holding_registers(SOCKETFD socketFd, int id, int address, const std::vector<int>& data);

//==========================
//==========从站=============
//==========================
// 待开放


// 天链二次开发接口
/**
 * @brief 获取运行灵巧手目标手势序列号
 * @param data 接收数据
 */
EXPORT_API Result read_hand_gesture_sequence(SOCKETFD socketFd, int id, int start_address, int address_length, std::string& data);
/**
 * @brief 获取运行灵巧手动作序列号
 * @param data 接收数据
 */
EXPORT_API Result read_hand_action_sequence(SOCKETFD socketFd, int id, int start_address, int address_length, std::string& data);
/**
 * @brief 设置灵巧手各自由度角度
 * @param data 设置的数据
 */
EXPORT_API Result write_hand_freedom_angle(SOCKETFD socketFd, int id, int start_address, size_t address_length, const std::vector<std::string>& data);
/**
 * @brief 设置灵巧手速度
 * @param data 设置的数据
 */
EXPORT_API Result write_hand_speed(SOCKETFD socketFd, int id, int start_address, size_t address_length, const std::vector<int>& data);
/**
 * @brief 设置灵巧手力阈值
 * @param data 设置的数据
 */
EXPORT_API Result write_hand_force_threshold(SOCKETFD socketFd, int id, int start_address, size_t address_length, const std::vector<int>& data);

/**
 * @brief 设置灵巧手角度跟随控制
 * @param data 设置的数据
 */
EXPORT_API Result write_hand_angle_follow(SOCKETFD socketFd, int id, int start_address, size_t address_length, const std::vector<bool>& data);

/**
 * @brief 设置灵巧手位置跟随控制
 * @param data 设置的数据
 */
EXPORT_API Result write_hand_position_follow(SOCKETFD socketFd, int id, int start_address, size_t address_length, const std::vector<bool>& data);

/**
 * @brief 设置夹爪行程
 * @param socketFd Socket文件描述符
 * @param id Modbus从站ID
 * @param data 设置的数据
 * @param addr_list 表示要写入寄存器地址数组
 * @param addr_length 表示要写入的连续寄存器数量
 */
EXPORT_API Result write_gripper_stroke(SOCKETFD socketFd, int id, const std::vector<int>& data,std::vector<int>& addr_list,int addr_length);
/**
 * @brief 松开夹爪
 * @param socketFd Socket文件描述符
 * @param id Modbus从站ID
 * @param data 设置的数据
 * * @param addr_list 表示要写入寄存器地址数组
 * @param addr_length 表示要写入的连续寄存器数量
 */
EXPORT_API Result write_gripper_release(SOCKETFD socketFd, int id, const std::vector<bool>& data,std::vector<int>& addr_list,int addr_length);
/**
 * @brief 夹爪力控夹取
 * @param socketFd Socket文件描述符
 * @param id Modbus从站ID
 * @param data 设置的数据 (包含三个int值)
 * * @param addr_list 表示要写入寄存器地址数组
 * @param addr_length 表示要写入的连续寄存器数量
 */
EXPORT_API Result write_gripper_force_control_grab(SOCKETFD socketFd, int id, const std::vector<int>& data,std::vector<int>& addr_list,int addr_length);
/**
 * @brief 夹爪持续力控夹取
 * @param socketFd Socket文件描述符
 * @param id Modbus从站ID
 * @param data 设置的数据
 * @param addr_list 表示要写入寄存器地址数组
 * @param addr_length 表示要写入的连续寄存器数量
 */
EXPORT_API Result write_gripper_continuous_force_control_grab(SOCKETFD socketFd, int id, const std::vector<int>& data,std::vector<int>& addr_list,int addr_length);
/**
 * @brief 设置夹爪达到指定位置
 * @param socketFd Socket文件描述符
 * @param id Modbus从站ID
 * @param data 设置的数据
 * @param addr_list 表示要写入寄存器地址数组
 * @param addr_length 表示要写入的连续寄存器数量
 */
EXPORT_API Result write_gripper_move_to_position(SOCKETFD socketFd, int id, const std::vector<int>& data,std::vector<int>& addr_list,int addr_length);
/**
 * @brief 查询夹爪状态
 * @param socketFd Socket文件描述符
 * @param id Modbus从站ID
 * @param data 接收数据 (hex格式)
 * @param addr_list 表示要写入寄存器地址数组
 * @param addr_length 表示要写入的连续寄存器数量
 */
EXPORT_API Result read_gripper_status(SOCKETFD socketFd, int id, std::vector<std::string>& data,std::vector<int>& addr_list,int addr_length);

#endif /* INTERFACE_CPP_INTERFACE_TL_MODBUS_H_ */
