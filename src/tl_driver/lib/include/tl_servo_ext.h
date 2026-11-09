/**
 * @file tl_servo_ext.h
 * @brief 伺服插值/高频透传接口（namespace tl）
 * @attention servo_movej/movel 系列为实验性接口，接口与行为可能随版本调整
 */
#ifndef TL_SDK_TL_SERVO_EXT_H
#define TL_SDK_TL_SERVO_EXT_H

#include <vector>
#include "tl_types.h"

namespace tl
{

/**
 * @brief 开启伺服跟踪模式（servo_movej 前置）
 * @attention 实验性接口：接口与行为可能随版本调整
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
 * @warning 与恒力驻留控制（open_constforce）不能同时使用：两者均独占 7000
 * 端口的 servoJ 会话，SDK 不做互斥检测，须由调用方保证——先 close_constforce
 * 再开启伺服跟踪
 */
TL_API Result open_servo_movej(SOCKETFD socketFd, double vmax_pct = 30.0, double amax_pct = 30.0,
                               double jmax_pct = 30.0);

/**
 * @brief 关闭伺服跟踪模式（servo_movej）
 * @attention 实验性接口：接口与行为可能随版本调整
 * @param socketFd TCP socket（7000 伺服端口）
 * @return SUCCESS / 底层 close_servoJ 错误码
 * @note 引用计数归零时停止后台发送线程并关闭 servoJ
 */
TL_API Result close_servo_movej(SOCKETFD socketFd);

/**
 * @brief 开启伺服跟踪模式（servo_movel 前置）
 * @attention 实验性接口：接口与行为可能随版本调整
 *
 * 与 open_servo_movej 行为一致（同一底层 servoJ），仅作为 servo_movel
 * 的配套开关。
 *
 * @param socketFd TCP socket（7000 伺服端口）
 * @param vmax_pct 最大速度百分比，范围 (0,100]，默认 30
 * @param amax_pct 最大加速度百分比，范围 (0,100]，默认 30
 * @param jmax_pct 最大加加速度百分比，范围 (0,100]，默认 30
 * @return SUCCESS / DISCONNECT（未连接）/ 底层 open_servoJ 错误码
 *
 * @warning 需先连接 7000 端口；与 close_servo_movel 成对调用
 * @warning 与恒力驻留控制（open_constforce）不能同时使用：两者均独占 7000
 * 端口的 servoJ 会话，SDK 不做互斥检测，须由调用方保证——先 close_constforce
 * 再开启伺服跟踪
 */
TL_API Result open_servo_movel(SOCKETFD socketFd, double vmax_pct = 30.0, double amax_pct = 30.0,
                               double jmax_pct = 30.0);

/**
 * @brief 关闭伺服跟踪模式（servo_movel）
 * @attention 实验性接口：接口与行为可能随版本调整
 * @param socketFd TCP socket（7000 伺服端口）
 * @return SUCCESS / 底层 close_servoJ 错误码
 * @note 引用计数归零时停止后台发送线程并关闭 servoJ
 */
TL_API Result close_servo_movel(SOCKETFD socketFd);

/**
 * @brief Servo MoveJ 关节空间伺服运动（异步，非阻塞）
 * @attention 实验性接口：接口与行为可能随版本调整
 *
 * 从最后发送的插补点（无历史时为当前关节角）到目标关节角做线性插值，
 * 逐点入流式队列，后台线程以 250Hz 周期逐点发送；
 * 新目标即时接管：清空未发送旧点并从最后发送点衔接，高频连续下发无批次间停顿。
 *
 * @param socketFd TCP socket（6001 控制端口）：用于查询当前关节角
 * @note 只传 6001：7000 伺服 socket 在 open_servo_movej 开启跟踪模式时已登记，
 *       后台 250Hz 插值线程统一使用该连接发送 servoJ 数据流，本接口无需也无法
 *       更换——若在此另传 socket，将与 open 时 servoJ 会话所属连接不一致，导致
 *       下发失败，因此接口不再接收 7000 socket
 * @param target 目标关节角，6 或 7 个元素；插值点统一按 7 元素下发，传 6
 * 元素时第 7 位（外部轴）保持当前值不动
 * @param step_size 插值步长（关节角，度），<=0 时使用默认 0.72，有效范围 [0.01,
 * 20]，超出截断； 建议按 vmax 计算：等效进给速度 = step_size ×
 * 250Hz（后台发送周期 4ms），不超过 vmax 绝对速度（300 × vmax_pct/100
 * 度/秒），即 step_size ≤ 1.2 × vmax_pct/100； 超出时控制器按 vmax
 * 兜底限速，机器人将滞后于最后发送的插补点，下次下发以该点
 *                  衔接会出现实际跳变。默认 0.72 对应等效 180 度/秒（需
 * vmax_pct ≥ 60）
 * @return SUCCESS / OPERATION_NOT_ALLOWED（未先 open_servo_movej）/
 *         DISCONNECT（未连接）/ PARAM_ERR（参数错误）
 *
 * @warning 需先调用 open_servo_movej 开启跟踪模式
 * @warning 仅支持关节角输入，笛卡尔位姿目标请使用 servo_movel
 */
TL_API Result servo_movej(SOCKETFD socketFd, const std::vector<double>& target,
                          double step_size = 0.72);

/**
 * @brief Servo MoveL 笛卡尔直线伺服运动（异步，非阻塞）
 * @attention 实验性接口：接口与行为可能随版本调整
 *
 * 目标位姿经控制器逆解后逐点入流式队列，后台线程以 250Hz 周期逐点发送；
 * 新目标即时接管（清空未发送旧点）。
 * 逆解中途失败时，已入队部分继续执行到该点为止，并返回逆解错误码。
 *
 * @param socketFd TCP socket（6001 控制端口）：用于查询当前位姿/逆解
 * @note 只传 6001：7000 伺服 socket 在 open_servo_movel 开启跟踪模式时已登记，
 *       后台 250Hz 插值线程统一使用该连接发送 servoJ 数据流，本接口无需也无法
 *       更换——若在此另传 socket，将与 open 时 servoJ 会话所属连接不一致，导致
 *       下发失败，因此接口不再接收 7000 socket
 * @param target_pose 目标位姿 [X,Y,Z,RX,RY,RZ]（mm, rad），6 或 7 个元素
 * @param coord 目标位姿所在坐标系，支持 Coord::BASE（基坐标系，默认）/
 * Coord::TOOL（工具坐标系）/ Coord::USER（用户坐标系）。TOOL/USER
 * 于接管时刻（机器人静止）一次性换算到基坐标系，
 *              之后直线插值与逐点逆解均在基坐标系内进行，避免流式下发期间坐标系重锚定漂移；传
 * Coord.JOINT 返回 PARAM_ERR
 * @param step_size 笛卡尔采样密度：相邻采样点的最大关节增量（度），≤0
 * 时使用默认 2.0，有效范围 [0.01, 20]，超出截断； IK
 * 结果不做关节级插值，直线精度取决于采样密度。 建议按 vmax 计算：等效进给速度 =
 * step_size × 250Hz，不超过 vmax 绝对速度 （300 × vmax_pct/100 度/秒），即
 * step_size ≤ 1.2 × vmax_pct/100；超出时控制器按 vmax
 * 兜底限速，机器人滞后于点流。注意默认 2.0 对应等效 500 度/秒，即使
 * vmax_pct=100 也超出，对衔接精度有要求时建议显式传更小的 step_size
 * @return SUCCESS / OPERATION_NOT_ALLOWED（未先 open_servo_movel）/
 *         DISCONNECT（未连接）/ PARAM_ERR（参数错误）/ 逆解失败错误码
 *
 * @warning 需先调用 open_servo_movel 开启跟踪模式
 * @warning 仅支持笛卡尔位姿输入（BASE/TOOL/USER），关节角目标请使用 servo_movej
 * @note 流式模式下单点逆解位于 6001 往返关键路径上：单点逆解耗时应显著小于
 * 4ms（250Hz 周期），否则实际下发速率随逆解耗时下降，需评估控制器 servoJ
 * 流超时容忍度
 */
TL_API Result servo_movel(SOCKETFD socketFd, const std::vector<double>& target_pose,
                          Coord coord = Coord::BASE, double step_size = 2.0);

/**
 * @brief 阻塞等待排队运动执行完毕
 * @attention 实验性接口：接口与行为可能随版本调整
 *
 * 等待已入队插补点发送完毕（队列为空且线程空闲）；未开启跟踪模式或已关闭时立即返回。
 */
TL_API void wait_servo();

/**
 * @brief 取消当前运动并清空排队任务
 * @attention 实验性接口：接口与行为可能随版本调整
 *
 * 清空未发送的插补点，运动随即停止（最多补发一个已取出的点）。
 */
TL_API void cancel_servo();

/**
 * @brief 是否有任务执行或排队中
 * @attention 实验性接口：接口与行为可能随版本调整
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
 * @note SDK 底层协议统一使用 7 元素 vector 兼容 6/7
 * 轴机械臂，三个约束向量必须固定为 7 元素： 6 轴机器人在第 7 元素（索引 6）补
 * 0（外部轴），7 轴机器人第 7 元素为真实 J7 约束值。 与 set_servoJ_pos 的 q
 * 长度约定保持一致。
 * @warning 约束向量传 6 元素会导致 vector marshaling
 * 长度不足，控制器协议解析错乱并断开 7000 连接； 不要硬编码 6
 * 轴写死，应根据实际轴数动态写入约束值，剩余元素保持 0（vector 初始化即置零）
 */
TL_API Result open_servoJ(SOCKETFD socketFd, std::vector<double> vmax, std::vector<double> amax,
                          std::vector<double> jmax);

/**
 * @brief 关闭关节跟踪模式（底层 servoJ 透传）
 * @param socketFd socket（7000 伺服端口）
 * @return SUCCESS / 底层错误码
 */
TL_API Result close_servoJ(SOCKETFD socketFd);

/**
 * @brief 发送跟踪关节位置（底层 servoJ 透传，10ms 周期高频调用）
 * @param socketFd socket（7000 伺服端口）
 * @param q 目标关节角，7 元素向量（度）；6 轴机器人在第 7 元素（索引 6）补
 * 0（外部轴）， 7 轴机器人第 7 元素为真实 J7 关节角
 * @return SUCCESS / PARAM_ERR（q 长度 ≠ 7）/ 底层错误码
 * @note 与 open_servoJ 的 vmax/amax/jmax 长度约定一致：固定 7 元素，
 *       按实际轴数动态写入，剩余元素保持 0（vector 初始化即置零）
 */
TL_API Result set_servoJ_pos(SOCKETFD socketFd, std::vector<double> q);

/**
 * @brief 打开周期同步速度模式（CSV，TCP 7000 连接）
 * @param socketFd socket（7000 伺服端口）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result open_servo_csv_mode(SOCKETFD socketFd);

/**
 * @brief 关闭周期同步速度模式
 * @param socketFd socket（7000 伺服端口）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result close_servo_csv_mode(SOCKETFD socketFd);

/**
 * @brief 下发周期同步速度
 * @param socketFd socket（7000 伺服端口）
 * @param targetVelocity 各关节目标速度，单位 度/秒，长度为 7
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_servo_csv_velocity(SOCKETFD socketFd, const std::vector<double>& targetVelocity);

/**
 * @brief 打开周期同步扭矩模式（CST，TCP 7000 连接）
 * @param socketFd socket（7000 伺服端口）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result open_servo_cst_mode(SOCKETFD socketFd);

/**
 * @brief 关闭周期同步扭矩模式
 * @param socketFd socket（7000 伺服端口）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result close_servo_cst_mode(SOCKETFD socketFd);

/**
 * @brief 下发周期同步扭矩
 * @param socketFd socket（7000 伺服端口）
 * @param targetTorque 各关节目标扭矩，单位：额定扭矩的 0.1%，范围 [-1000,
 * 1000]，长度为 7
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT
 * 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED
 * 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_servo_cst_torque(SOCKETFD socketFd, const std::vector<double>& targetTorque);

} // namespace tl

#endif /* TL_SDK_TL_SERVO_EXT_H */