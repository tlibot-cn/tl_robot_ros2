/**
 * @file tl_force.h
 * @brief 恒力控制接口（namespace tl）
 * @attention 恒力驻留控制为 SDK 扩展实现（非控制器原生功能），接口与行为可能随版本调整
 */
#ifndef TL_SDK_TL_FORCE_H
#define TL_SDK_TL_FORCE_H

#include <vector>
#include "tl_types.h"
#include "tl_constant_force.h"
#include "tl_admittance.h"

namespace tl
{


/**
 * @brief 设置六维力传感器通讯
 * @param params 用于设置六维力传感器通讯参数的结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result set_six_dimensional_force_communication_params(SOCKETFD socketFd,
                                                      SixDimensionalForceCommunicationParams& params);

/**
 * @brief 获取六维力传感器数据
 * @param sensorData 用于接收六维力传感器数据的结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_sensor_6d_data(SOCKETFD socketFd, Sensor6DData& sensorData);

/**
 * @brief 获取六维力传感器的基础参数（质量、质心、标零状态）
 * @param baseParam 用于接收传感器基础参数的结构体
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result get_sensor_base_param(SOCKETFD socketFd, SensorBaseParam& baseParam);

/**
 * @brief 执行六维力传感器标定
 * @param[out] success 标定是否成功的标志
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result mark_base_sensor(SOCKETFD socketFd, bool& success);

/**
 * @brief 开启恒力驻留控制（异步，立即返回）
 * @attention 实验性接口：SDK 扩展实现（非控制器原生功能），接口与行为可能随版本调整
 *
 * 后台线程持续运行，无时长上限：
 *   sensor_callback → EMA 滤波 → 死区 → 导纳补偿 → 工具系→基系旋转 → IK → set_servoJ_pos
 *
 * @param socketFd        TCP socket (6001)：仅 IK 使用
 * @param socket_servo    TCP socket (7000)：servoJ
 * @param base_pose        初始目标位姿 [X,Y,Z,RX,RY,RZ] (mm, rad)，后台闭环以此为基准；
 *                         运行中可用 set_constforce_target_pose 实时更新
 * @param admittance       已 Configure + SetDesiredWrench（含重力补偿 + 目标力）的导纳实例
 * @param sensor_callback  六维力传感器读取回调（nullptr → PARAM_ERR）
 * @param sensor_user_data 回调透传指针，可为 nullptr
 * @param params           恒力参数（force_control_axis 指定时校验导纳刚度 K=0）
 * @return SUCCESS / PARAM_ERR（参数错误）/ DISCONNECT（未连接）/ OPERATION_NOT_ALLOWED（已在运行或与 servo
 * 异步模式冲突）
 *
 * @warning admittance 必须存活到 close_constforce() 返回之后才能销毁
 * @warning F_desired 必须在 open_constforce() 前通过 SetDesiredWrench 设置，运行中不支持修改（并发数据竞争）
 * @warning 与 servo_movej/movel 异步模式互斥：两者均写 7000 端口，检测到冲突返回 OPERATION_NOT_ALLOWED
 * @warning 互斥检测仅恒力侧进行（open_constforce 检查异步模式）；反向（恒力运行中调用
 *          open_servo_movej/movel）不受保护，须由调用方保证：先 close_constforce 再开异步伺服
 * @warning 控制频率由 admittance.GetControlPeriod() 决定，循环定时与导纳积分共享同一时间源
 * @warning open_constforce 与 close_constforce 需串行调用（非线程安全）
 */
TL_API Result open_constforce(SOCKETFD socketFd, SOCKETFD socket_servo, const std::vector<double>& base_pose,
                       AdmittanceController& admittance, ForceSensorCallback sensor_callback, void *sensor_user_data,
                       const ConstantForceParams& params);

/**
 * @brief 关闭恒力控制并阻塞等待后台循环退出（幂等，可随时调用）
 * @attention 实验性接口：SDK 扩展实现（非控制器原生功能），接口与行为可能随版本调整
 *
 * 三种场景：
 *   1. 后台循环运行中 → 置停止标志 → join → 返回结束原因
 *   2. 线程已自行退出（断连超时 / open_servoJ 失败）→ join 已退出线程 → 返回结束原因
 *   3. 未 open 过或已 close 过 → 无可清理内容，返回 SUCCESS
 *
 * @return SUCCESS（用户主动关闭，或无可清理内容）/ EXCEPTION（传感器断连超时自动停止）/ open_servoJ 原始错误码
 *
 * @note 必须调用本函数完成清理（join 后台线程）；不调用则 std::thread 泄漏，
 *       进程退出时 std::terminate。
 * @warning 禁止在传感器回调（ForceSensorCallback）内调用本函数：回调运行于后台
 *          线程，此处 join 自身线程会抛出 std::system_error 并终止进程。
 *          回调中需要停止时请返回 false（计入断连计数，触发安全自动停止）。
 */
TL_API Result close_constforce();

/**
 * @brief 实时更新恒力控制的目标位姿（运动接口）
 * @attention 实验性接口：SDK 扩展实现（非控制器原生功能），接口与行为可能随版本调整
 *
 * 后台闭环每周期读取最新目标位姿，下一周期即以新目标为基准做导纳补偿。
 * 主线程可据此实现任意运动（直线移动、扫动、圆弧等），轨迹平滑由调用方保证
 * （大跨度跳变会直接反映到关节指令上）。
 *
 * @param target_pose 新目标位姿 [X,Y,Z,RX,RY,RZ] (mm, rad)，至少 6 个元素
 * @return SUCCESS / PARAM_ERR（长度不足或含非有限值）/ OPERATION_NOT_ALLOWED（未在运行）
 *
 * @warning 仅恒力控制运行中（open_constforce 后、close_constforce 前）可调用
 * @warning 与导纳 F_desired 类似，目标位姿更新不保证原子性之外的平滑性，
 *          大跳变请由调用方分步逼近
 */
TL_API Result set_constforce_target_pose(const std::vector<double>& target_pose);

/**
 * @brief 查询恒力控制运行时状态（线程安全）
 * @attention 实验性接口：SDK 扩展实现（非控制器原生功能），接口与行为可能随版本调整
 *
 * 返回后台闭环最近一个控制周期更新的快照：运行标志、原始/滤波后六维力、
 * 当前目标位姿、结束原因。未 open 时返回 SUCCESS 且 running=false。
 *
 * @param status [out] 状态快照
 * @return SUCCESS
 */
TL_API Result get_constforce_status(ConstForceStatus& status);

} // namespace tl

#endif /* TL_SDK_TL_FORCE_H */
