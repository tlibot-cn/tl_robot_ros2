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
 * @deprecated 请使用 IoLevel 枚举重载版本
 * @note 推荐使用枚举重载（IoLevel::OFF / IoLevel::ON），避免魔法数字
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误（如端口号越界）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_DEPRECATED("use IoLevel enum overload instead")
TL_API Result set_digital_output(SOCKETFD socketFd, int port, int value);

/**
 * @brief 设置数字输出（IoLevel 枚举重载）
 * @param port 端口号 【1，最大端口数】
 * @param value IoLevel::OFF（输出关闭）/ IoLevel::ON（输出打开）
 * @note 推荐使用本枚举重载版本，避免魔法数字
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误（如端口号越界）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_digital_output(SOCKETFD socketFd, int port, IoLevel value);

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
 * @brief 获取IO复位相关参数
 * @param robotNum 机器人编号(1-4)
 * @param type 设置类型：1 远程IO复位 / 2 切模式停止 / 3 程序报错
 * @param enable 输出：是否复位容器，大小为所有IO板输出端口数，从第二块IO板开始，每一块IO板的起始位置为上一块IO板的末位端口的顺延
 * @param value 输出：复位值容器，大小为所有IO板输出端口数，从第二块IO板开始，每一块IO板的起始位置为上一块IO板的末位端口的顺延
 * @deprecated 请使用 IoResetType 枚举重载版本
 * @note 推荐使用枚举重载（IoResetType::REMOTE_IO / IoResetType::MODE_STOP / IoResetType::PROGRAM_ERROR），避免魔法数字
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_DEPRECATED("use IoResetType enum overload instead")
TL_API Result get_IO_reset_function(SOCKETFD socketFd, int robotNum, int type, std::vector<int>& enable,
                                    std::vector<int>& value);

/**
 * @brief 获取IO复位相关参数（IoResetType 枚举重载）
 * @param robotNum 机器人编号(1-4)
 * @param type IoResetType::REMOTE_IO（远程IO复位）/ IoResetType::MODE_STOP（切模式停止）/ IoResetType::PROGRAM_ERROR（程序报错）
 * @param enable 输出：是否复位容器，大小为所有IO板输出端口数，从第二块IO板开始，每一块IO板的起始位置为上一块IO板的末位端口的顺延
 * @param value 输出：复位值容器，大小为所有IO板输出端口数，从第二块IO板开始，每一块IO板的起始位置为上一块IO板的末位端口的顺延
 * @note 推荐使用本枚举重载版本，避免魔法数字
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_IO_reset_function(SOCKETFD socketFd, int robotNum, IoResetType type, std::vector<int>& enable,
                                    std::vector<int>& value);

} // namespace tl

#endif /* TL_SDK_TL_IO_H */
