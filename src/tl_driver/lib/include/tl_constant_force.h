/**
 * @file tl_constant_force.h
 * @brief TL 机械臂 SDK 恒力驻留控制接口
 *
 * 将恒力伺服闭环（读力 → 滤波 → 导纳补偿 → IK → servoj）封装为异步接口。
 * 后台线程持续运行，无时长上限，由 close_constforce() 或安全自动停止终止。
 *
 * 使用流程:
 *   // 1. 配置导纳（力控轴 stiffness=0），SetDesiredWrench 含重力补偿 + 目标力
 *   AdmittanceController ctrl;
 *   ctrl.Configure(adm_params);
 *   ctrl.SetDesiredWrench(desired_wrench);
 *
 *   // 2. 开启恒力控制（立即返回）
 *   open_constforce(socketFd, socket_servo, base_pose, ctrl, my_sensor_cb, user_data, params);
 *
 *   // 3. 主线程可做其他事；需要停止时
 *   Result reason = close_constforce();
 *
 * @attention 需要六维力传感器正常连接（数据由调用方回调提供）
 * @note 运行前提: 机器人已上电、已切运行模式(2)、已运动到接触位
 * @attention 本文件恒力驻留控制为 SDK 扩展实现（非控制器原生功能），接口与行为可能随版本调整
 */

#ifndef TL_EXTENSION_TL_CONSTANT_FORCE_H
#define TL_EXTENSION_TL_CONSTANT_FORCE_H

#include <array>
#include <vector>
#include "tl_types.h"

#include "tl_admittance.h"

namespace tl
{


// ==================== 恒力驻留控制 ====================

/**
 * @brief 六维力传感器读取回调（在恒力后台线程中同步调用）
 *
 * @param wrench    [out] 六维力 [Fx,Fy,Fz,Mx,My,Mz]（N, N·m），返回 true 时必须填充
 * @param user_data 调用方透传指针（open_constforce 传入，原样回传）
 * @return true=读取成功 / false=读取失败或断连（计入连续断连计数）
 *
 * @warning 回调在后台线程中执行，内部需自行保证线程安全；
 *          读取耗时须远小于控制周期，否则拉长循环周期导致定时漂移
 */
typedef bool (*ForceSensorCallback)(double wrench[6], void *user_data);

/**
 * 恒力驻留控制参数。
 *
 * 力控轴行为由调用方通过 AdmittanceController 表达：
 *   力控轴 stiffness 设 0（零稳态误差），SetDesiredWrench 含重力补偿 + 目标力。
 */
struct TL_API ConstantForceParams
{
  /** EMA 滤波系数 [Fx,Fy,Fz,Mx,My,Mz]，每维独立，(0,1] */
  std::array<double, 6> filter_alpha{{0.15, 0.15, 0.3, 0.15, 0.15, 0.15}};
  /** 死区阈值 [Fx,Fy,Fz,Mx,My,Mz]（力 N / 力矩 N·m），小于该值置零 */
  std::array<double, 6> deadband{{0.5, 0.5, 0.1, 0.02, 0.02, 0.12}};
  /** 力控轴 (0=X, 1=Y, 2=Z, 3=RX, 4=RY, 5=RZ)；-1=不指定（默认，不校验） */
  int force_control_axis{-1};
  /** 伺服速度限制百分比 (0-100) */
  double sv_pct{100.0};
  /** 伺服加速度限制百分比 (0-100) */
  double sa_pct{100.0};
  /** 伺服加加速度限制百分比 (0-100) */
  double sj_pct{50.0};
  /** 传感器连续断连超时 (ms)，超过则自动停止（安全底线），<=0 禁用自动停止 */
  int sensor_loss_timeout_ms{500};
};

/**
 * 恒力控制运行时状态（get_constforce_status 输出）。
 *
 * 后台线程每控制周期更新一次；主线程任意时刻调用 get_constforce_status
 * 获取最近一次快照（线程安全）。
 */
struct TL_API ConstForceStatus
{
  /** 后台闭环是否运行中（open 后 true，close/异常停止后 false） */
  bool running{false};
  /** 最近一次原始六维力 [Fx,Fy,Fz,Mx,My,Mz] (N, N·m)（滤波前） */
  std::array<double, 6> sensor_wrench{};
  /** 最近一次滤波+死区后六维力 [Fx,Fy,Fz,Mx,My,Mz] (N, N·m)（喂给导纳） */
  std::array<double, 6> filtered_wrench{};
  /** 当前目标位姿 [X,Y,Z,RX,RY,RZ] (mm, rad)（set_constforce_target_pose 更新） */
  std::array<double, 6> target_pose{};
  /** 结束原因：SUCCESS 正常 / EXCEPTION 传感器断连自动停止 / 其他异常码 */
  int last_result{static_cast<int>(Result::SUCCESS)};
};

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


} // namespace tl

#endif // TL_EXTENSION_TL_CONSTANT_FORCE_H
