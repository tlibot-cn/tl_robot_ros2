/*
 * nrc_modbus_parameter.h
 *
 *  Created on: 2025年3月19日
 *      Author: yiixiong
 */

#ifndef INCLUDE_CPP_PARAMETER_NRC_MODBUS_PARAMETER_H_
#define INCLUDE_CPP_PARAMETER_NRC_MODBUS_PARAMETER_H_
#include <string>

// TCP参数
struct ModbusTCPParameter {
	std::string IP{"192.168.1.13"};
	int port{503};
};

// RTU参数
struct ModbusRTUParameter {
	int slaveId;                    // 从站号
	int port{1};                    // 端口
	int baudrate{115200};           // 波特率
	std::string checkBit{"None"};     // 奇偶校验位"None","Even","Odd"
	int dataBit{8};     		    // 数据位,5,6,7,8
	int stopBit{1};                 // 停止位,1,2
};

struct ModbusMasterParameter {
	std::string type{"TCP"}; 		// 主站类型 "TCP","RTU"
	bool startAddress{false};              // false:起始地址为1；true:起始地址为0
	// TCP参数
	ModbusTCPParameter TCP;
	// RTU参数
	ModbusRTUParameter RTU;
};

#endif /* INTERFACE_PARAMETER_NRC_MODBUS_PARAMETER_H_ */
