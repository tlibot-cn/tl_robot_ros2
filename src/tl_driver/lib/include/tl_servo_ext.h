/**
 * @file tl_servo_ext.h
 * @brief 伺服插值/高频透传接口（namespace tl）
 * @attention 本文件 servo_movej/movel 系列为 SDK 扩展实现（非控制器原生功能），接口与行为可能随版本调整
 */
#ifndef TL_SDK_TL_SERVO_EXT_H
#define TL_SDK_TL_SERVO_EXT_H

#include <vector>
#include "tl_types.h"

namespace tl
{

/**
 * @brief 开启伺服跟踪模式（servo_movej 前置）
 * @attention 实验性接口：SDK 扩展实现（非控制器原生功能），接口与行为可能随版本调整
 *
 * 将百分比速度约束换算为绝对约束并开启底层 servoJ 跟踪，
 * 同时启动后台 250Hz 插值发送线程（引用计数管理，可嵌套开启）。
 *
 * @param socketFd TCP socket（7000 伺服端口）
 * @param vmax_pct 最大速度百分比，范围 (0,100]，默认 30
 * @param amax_pct 最大加速度百分比，范围 (0,100]，默认 30
 * @param jmax_pct 最大加加速度百分比，范围 (0,100]，默认 30
 * @return SUCCESS / DISCONNECT（未连接）/ 底层 open_servoJ 错误码
 *
 * @warning 需先连接 7000 端口；与 close_servo_movej 成对调用
 */
TL_API Result open_servo_movej(SOCKETFD socketFd, double vmax_pct = 30.0, double amax_pct = 30.0, double jmax_pct = 30.0);

/**
 * @brief 关闭伺服跟踪模式（servo_movej）
 * @attention 实验性接口：SDK 扩展实现（非控制器原生功能），接口与行为可能随版本调整
 * @param socketFd TCP socket（7000 伺服端口）
 * @return SUCCESS / 底层 close_servoJ 错误码
 * @note 引用计数归零时停止后台发送线程并关闭 servoJ
 */
TL_API Result close_servo_movej(SOCKETFD socketFd);

/**
 * @brief 开启伺服跟踪模式（servo_movel 前置）
 * @attention 实验性接口：SDK 扩展实现（非控制器原生功能），接口与行为可能随版本调整
 *
 * 与 open_servo_movej 行为一致（同一底层 servoJ），仅作为 servo_movel 的配套开关。
 *
 * @param socketFd TCP socket（7000 伺服端口）
 * @param vmax_pct 最大速度百分比，范围 (0,100]，默认 30
 * @param amax_pct 最大加速度百分比，范围 (0,100]，默认 30
 * @param jmax_pct 最大加加速度百分比，范围 (0,100]，默认 30
 * @return SUCCESS / DISCONNECT（未连接）/ 底层 open_servoJ 错误码
 *
 * @warning 需先连接 7000 端口；与 close_servo_movel 成对调用
 */
TL_API Result open_servo_movel(SOCKETFD socketFd, double vmax_pct = 30.0, double amax_pct = 30.0, double jmax_pct = 30.0);

/**
 * @brief 关闭伺服跟踪模式（servo_movel）
 * @attention 实验性接口：SDK 扩展实现（非控制器原生功能），接口与行为可能随版本调整
 * @param socketFd TCP socket（7000 伺服端口）
 * @return SUCCESS / 底层 close_servoJ 错误码
 * @note 引用计数归零时停止后台发送线程并关闭 servoJ
 */
TL_API Result close_servo_movel(SOCKETFD socketFd);

/**
 * @brief Servo MoveJ 关节空间伺服运动（异步，非阻塞）
 * @attention 实验性接口：SDK 扩展实现（非控制器原生功能），接口与行为可能随版本调整
 *
 * 从当前关节角到目标关节角做线性插值并整体入队，
 * 后台线程以 250Hz 周期逐点发送；新目标会清空旧排队任务（不打断当前批次）。
 *
 * @param socketFd TCP socket（6001 控制端口）：仅用于查询当前关节角/逆解
 * @param socket_servo TCP socket（7000 伺服端口）：servoJ 数据发送
 * @param target 目标关节角，6 或 7 个元素（7 轴含真实 J7；6 轴时第 7 位补 0）
 * @param coord 坐标系，仅支持 Coord::JOINT（关节角直接插值，默认）/ Coord::BASE（基坐标系，先逆解转关节角）
 * @param step_size 插值步长（关节角，度），<=0 时使用默认 0.72
 * @return SUCCESS / OPERATION_NOT_ALLOWED（未先 open_servo_movej）/
 *         DISCONNECT（未连接）/ PARAM_ERR（参数错误）
 *
 * @warning 需先调用 open_servo_movej 开启跟踪模式
 * @note 本接口自引入即使用 Coord 枚举（不提供 int 版本）；仅支持关节/基坐标系两个取值
 */
TL_API Result servo_movej(SOCKETFD socketFd, SOCKETFD socket_servo, const std::vector<double>& target,
                   Coord coord = Coord::JOINT, double step_size = 0.72);

/**
 * @brief Servo MoveL 笛卡尔直线伺服运动（异步，非阻塞）
 * @attention 实验性接口：SDK 扩展实现（非控制器原生功能），接口与行为可能随版本调整
 *
 * 目标位姿经 FK/IK 转关节角后做笛卡尔直线插值（位置线性 + 姿态四元数 slerp），
 * 每步逆解后整体入队，后台线程以 250Hz 周期逐点发送。
 *
 * @param socketFd TCP socket（6001 控制端口）：仅用于查询当前位姿/逆解
 * @param socket_servo TCP socket（7000 伺服端口）：servoJ 数据发送
 * @param target_pose 目标位姿 [X,Y,Z,RX,RY,RZ]（mm, rad），6 或 7 个元素
 * @param coord 坐标系，仅支持 Coord::JOINT（按关节角直线插值）/ Coord::BASE（基坐标系直线插值，默认）
 * @param step_size 插值步长（关节角，度），<=0 时使用默认 0.72
 * @return SUCCESS / OPERATION_NOT_ALLOWED（未先 open_servo_movel）/
 *         DISCONNECT（未连接）/ PARAM_ERR（参数错误）/ 逆解失败错误码
 *
 * @warning 需先调用 open_servo_movel 开启跟踪模式
 * @note 本接口自引入即使用 Coord 枚举（不提供 int 版本）；仅支持关节/基坐标系两个取值
 */
TL_API Result servo_movel(SOCKETFD socketFd, SOCKETFD socket_servo, const std::vector<double>& target_pose,
                   Coord coord = Coord::BASE, double step_size = 2.0);

/**
 * @brief 阻塞等待排队运动执行完毕
 * @attention 实验性接口：SDK 扩展实现（非控制器原生功能），接口与行为可能随版本调整
 *
 * 等待当前批次发送完成且队列为空；未开启跟踪模式或已关闭时立即返回。
 */
TL_API void wait_servo();

/**
 * @brief 取消当前运动并清空排队任务
 * @attention 实验性接口：SDK 扩展实现（非控制器原生功能），接口与行为可能随版本调整
 *
 * 置取消标志并清空队列，当前批次在下一个插值点停止发送。
 */
TL_API void cancel_servo();

/**
 * @brief 是否有任务执行或排队中
 * @attention 实验性接口：SDK 扩展实现（非控制器原生功能），接口与行为可能随版本调整
 * @return true=后台线程正在发送或队列非空 / false=空闲或跟踪模式未开启
 */
TL_API bool servo_busy();

/**
 * @brief 打开关节跟踪模式（底层 servoJ 透传）
 * @param socketFd socket（7000 伺服端口）
 * @param vmax 速度约束，7 元素向量（度/秒）
 * @param amax 加速度约束，7 元素向量（度/秒²）
 * @param jmax 加加速度约束，7 元素向量（度/秒³）
 * @return SUCCESS / PARAM_ERR（任一约束向量长度 ≠ 7）/ 底层错误码
 * @note SDK 底层协议统一使用 7 元素 vector 兼容 6/7 轴机械臂，三个约束向量必须固定为 7 元素：
 *       6 轴机器人在第 7 元素（索引 6）补 0（外部轴），7 轴机器人第 7 元素为真实 J7 约束值。
 *       与 set_servoJ_pos 的 q 长度约定保持一致。
 * @warning 约束向量传 6 元素会导致 vector marshaling 长度不足，控制器协议解析错乱并断开 7000 连接；
 *       不要硬编码 6 轴写死，应根据实际轴数动态写入约束值，剩余元素保持 0（vector 初始化即置零）
 */
TL_API Result open_servoJ(SOCKETFD socketFd, std::vector<double> vmax, std::vector<double> amax, std::vector<double> jmax);

/**
 * @brief 关闭关节跟踪模式（底层 servoJ 透传）
 * @param socketFd socket（7000 伺服端口）
 * @return SUCCESS / 底层错误码
 */
TL_API Result close_servoJ(SOCKETFD socketFd);

/**
 * @brief 发送跟踪关节位置（底层 servoJ 透传，10ms 周期高频调用）
 * @param socketFd socket（7000 伺服端口）
 * @param q 目标关节角，7 元素向量（度）；6 轴机器人在第 7 元素（索引 6）补 0（外部轴），
 *          7 轴机器人第 7 元素为真实 J7 关节角
 * @return SUCCESS / 底层错误码
 * @note 与 open_servoJ 的 vmax/amax/jmax 长度约定一致：固定 7 元素，
 *       按实际轴数动态写入，剩余元素保持 0（vector 初始化即置零）
 */
TL_API Result set_servoJ_pos(SOCKETFD socketFd, std::vector<double> q);

/**
 * @brief 打开周期同步速度模式（CSV，TCP 7000 连接）
 * @param socketFd socket（7000 伺服端口）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result open_servo_csv_mode(SOCKETFD socketFd);

/**
 * @brief 关闭周期同步速度模式
 * @param socketFd socket（7000 伺服端口）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result close_servo_csv_mode(SOCKETFD socketFd);

/**
 * @brief 下发周期同步速度
 * @param socketFd socket（7000 伺服端口）
 * @param targetVelocity 各关节目标速度，单位 度/秒，长度为 7
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_servo_csv_velocity(SOCKETFD socketFd, const std::vector<double>& targetVelocity);

/**
 * @brief 打开周期同步扭矩模式（CST，TCP 7000 连接）
 * @param socketFd socket（7000 伺服端口）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result open_servo_cst_mode(SOCKETFD socketFd);

/**
 * @brief 关闭周期同步扭矩模式
 * @param socketFd socket（7000 伺服端口）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result close_servo_cst_mode(SOCKETFD socketFd);

/**
 * @brief 下发周期同步扭矩
 * @param socketFd socket（7000 伺服端口）
 * @param targetTorque 各关节目标扭矩，单位：额定扭矩的 0.1%，范围 [-1000, 1000]，长度为 7
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_servo_cst_torque(SOCKETFD socketFd, const std::vector<double>& targetTorque);

} // namespace tl

#endif /* TL_SDK_TL_SERVO_EXT_H */