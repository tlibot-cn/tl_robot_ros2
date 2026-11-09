/**
 * @file tl_constant_force.h
 * @brief TL 机械臂 SDK 恒力驻留控制接口
 *
 * 将恒力伺服闭环（读力 → 滤波 → 导纳补偿 → IK → servoj）实现为异步接口。
 * 后台线程持续运行，无时长上限，由 close_constforce() 或安全自动停止终止。
 *
 * 使用流程:
 *   // 1. 配置恒力参数（导纳配置与期望力均在 ConstantForceParams 中）
 *   ConstantForceParams params;
 *   params.desired_wrench = {...}; // 重力补偿 + 目标接触力
 *
 *   // 2. 开启恒力控制（立即返回，传感器读取默认内置）
 *   open_constforce(socketFd, socket_servo, base_pose, params);
 *
 *   // 3. 运行中可实时更新设定（可选）
 *   set_constforce_desired_wrench({...});   // 期望力（目标接触力）
 *   set_constforce_target_pose({...});      // 目标位姿
 *
 *   // 4. 主线程可做其他事；需要停止时
 *   Result reason = close_constforce();
 *
 * @attention 需要六维力传感器正常连接（默认经控制连接读取控制器端数据，
 *            外置数据源时由调用方回调提供）
 * @note 运行前提: 机器人已上电、已切运行模式(2)、已运动到接触位
 * @attention 恒力驻留控制为实验性功能，接口与行为可能随版本调整
 */

#ifndef TL_SDK_TL_CONSTANT_FORCE_H
#define TL_SDK_TL_CONSTANT_FORCE_H

#include <array>
#include <functional>
#include <vector>
#include "tl_types.h"

#include "tl_admittance.h"

namespace tl
{


// ==================== 恒力驻留控制 ====================

/**
 * @brief 六维力传感器读取回调（在恒力后台线程中同步调用）
 *
 * 闭包捕获传感器/连接对象即可，例如：
 *   [sock](double w[6]) { return read_my_sensor(sock, w); }
 * 的形式包装自定义数据源。回调捕获的对象须存活到 close_constforce()
 * 返回之后才能销毁。
 *
 * @param wrench [out] 六维力 [Fx,Fy,Fz,Mx,My,Mz]（N, N·m），返回 true
 * 时必须填充
 * @return true=读取成功 / false=读取失败或断连（计入连续断连计数）
 *
 * @warning 回调在后台线程中执行，内部需自行保证线程安全；
 *          读取耗时须远小于控制周期，否则拉长循环周期导致定时漂移
 */
using ForceSensorCallback = std::function<bool(double wrench[6])>;

/**
 * 伺服限幅百分比 (0-100]。
 *
 * 透传给 open_servoJ：v/sa/sj 分别对应关节速度/加速度/加加速度上限，
 * 100 表示不额外限幅（默认），数值越小闭环运动越保守。
 */
struct TL_API ServoLimits
{
  /** 关节速度限制百分比 (0-100) */
  double v{100.0};
  /** 关节加速度限制百分比 (0-100) */
  double a{100.0};
  /** 关节加加速度限制百分比 (0-100) */
  double j{50.0};
};

/**
 * 恒力驻留控制参数。
 *
 * 力控轴行为通过导纳字段表达：
 *   admittance.stiffness 中力控轴设 0（零稳态误差），desired_wrench
 *   含重力补偿 + 目标接触力。
 */
struct TL_API ConstantForceParams
{
  /** EMA 滤波系数 [Fx,Fy,Fz,Mx,My,Mz]，每维独立，(0,1] */
  std::array<double, 6> filter_alpha{{0.15, 0.15, 0.3, 0.15, 0.15, 0.15}};
  /** 死区阈值 [Fx,Fy,Fz,Mx,My,Mz]（力 N / 力矩 N·m），小于该值置零 */
  std::array<double, 6> deadband{{0.5, 0.5, 0.1, 0.02, 0.02, 0.12}};
  /** open 时的初始期望六维力/力矩 [Fx,Fy,Fz,Mx,My,Mz] (N, N·m)：重力补偿 +
   *  目标接触力，以传感器/末端坐标系表达；运行中改力请用
   *  set_constforce_desired_wrench（本字段不回写），当前值见
   *  ConstForceStatus::desired_wrench */
  std::array<double, 6> desired_wrench{};
  /** 导纳参数 M/B/K/单步限幅/控制周期（SI 单位，见 tl_admittance.h） */
  AdmittanceParams admittance;
  /** 伺服限幅（速度/加速度/加加速度百分比） */
  ServoLimits servo;
  /** 传感器连续断连超时 (ms)，超过则自动停止（安全底线），<=0 禁用自动停止 */
  int sensor_loss_timeout_ms{500};
};

/**
 * 恒力控制运行时状态（get_constforce_status 输出）。
 *
 * 力/力矩与运行标志由后台线程每控制周期更新；设定值 target_pose /
 * desired_wrench 由 open_constforce 与 set_constforce_* 在调用瞬间写入，
 * 不经周期快照。主线程任意时刻调用 get_constforce_status 获取最新值
 * （线程安全）。
 */
struct TL_API ConstForceStatus
{
  /** 后台闭环是否运行中（open 后 true，close/异常停止后 false） */
  bool running{false};
  /** 最近一次原始六维力 [Fx,Fy,Fz,Mx,My,Mz] (N, N·m)（滤波前） */
  std::array<double, 6> sensor_wrench{};
  /** 最近一次滤波+死区后六维力 [Fx,Fy,Fz,Mx,My,Mz] (N, N·m)（喂给导纳） */
  std::array<double, 6> filtered_wrench{};
  /** 当前期望六维力/力矩 [Fx,Fy,Fz,Mx,My,Mz] (N, N·m)，以传感器/末端坐标系
   *  表达（与 open 时 params.desired_wrench 同系，含重力补偿项）：open 时取自
   *  params，运行中由 set_constforce_desired_wrench 更新 */
  std::array<double, 6> desired_wrench{};
  /** 当前目标位姿 [X,Y,Z,RX,RY,RZ] (mm, rad)（set_constforce_target_pose 更新）
   */
  std::array<double, 6> target_pose{};
  /** 结束原因：SUCCESS 正常 / EXCEPTION 传感器断连自动停止 / 其他异常码 */
  int last_result{static_cast<int>(Result::SUCCESS)};
};

/**
 * @brief 开启恒力驻留控制（异步，立即返回）
 * @attention 实验性接口：接口与行为可能随版本调整
 *
 * 后台线程持续运行，无时长上限：
 *   传感器读取 → EMA 滤波 → 死区 → 导纳补偿 → 工具系→基系旋转 → IK →
 * set_servoJ_pos
 *
 * @param socketFd        TCP socket (6001)：IK 与内置传感器读取使用；
 * 恒力运行期间勿在主线程并发调用该连接上的接口
 * @param socket_servo    TCP socket (7000)：servoJ
 * @param base_pose       初始目标位姿 [X,Y,Z,RX,RY,RZ] (mm,
 * rad)，至少 6 个元素；后台闭环以此为基准；运行中可用
 * set_constforce_target_pose 实时更新
 * @param params          恒力参数（含导纳配置与初始期望力），缺省值即可运行
 * @param sensor_callback 六维力传感器读取回调（std::function，闭包捕获
 * 数据源对象），可省略；默认空 → 内置读取：经 socketFd 调
 * get_sensor_6d_data 取控制器端去皮分量；外置/自定义数据源时传入
 * @return SUCCESS / PARAM_ERR（参数错误）/ DISCONNECT（未连接）/
 * OPERATION_NOT_ALLOWED（已在运行）
 *
 * @warning 传感器回调闭包捕获的对象必须存活到 close_constforce()
 * 返回之后才能销毁
 * @warning 导纳参数（M/B/K/单步限幅/控制周期）在 open 时一次性写入，
 * 运行中不支持修改（并发数据竞争）；期望力运行中可用
 * set_constforce_desired_wrench 实时更新
 * @warning 与 servo_movej/movel 异步模式不能同时使用：两者均独占 7000
 * 端口的 servoJ 会话，同时下发会互相冲突破坏运动。SDK 不做互斥检测，须由
 * 调用方保证串行——先 close_servo_movej/movel 再 open_constforce；
 * 恒力运行中亦不得调用 open_servo_movej/movel，需先 close_constforce
 * @warning 控制频率由 params.admittance.control_period
 * 决定，循环定时与导纳积分共享同一时间源
 * @warning open_constforce 与 close_constforce 需串行调用（非线程安全）
 */
TL_API Result open_constforce(SOCKETFD socketFd, SOCKETFD socket_servo,
                              const std::vector<double>& base_pose,
                              const ConstantForceParams& params = {},
                              ForceSensorCallback sensor_callback = {});

/**
 * @brief 实时更新恒力控制的目标位姿（运动接口）
 * @attention 实验性接口：接口与行为可能随版本调整
 *
 * 后台闭环每周期读取最新目标位姿，下一周期即以新目标为基准做导纳补偿。
 * 主线程可据此实现任意运动（直线移动、扫动、圆弧等），轨迹平滑由调用方保证
 * （大跨度跳变会直接反映到关节指令上）。
 *
 * @param target_pose 新目标位姿 [X,Y,Z,RX,RY,RZ] (mm, rad)，至少 6 个元素
 * @return SUCCESS / PARAM_ERR（长度不足或含非有限值）/
 * OPERATION_NOT_ALLOWED（未在运行）
 *
 * @warning 仅恒力闭环存活期间可调用（open_constforce 成功且未 close、后台线程未
 *          自行停止）；闭环已停止（如传感器断连超时自动停止）时返回
 *          OPERATION_NOT_ALLOWED，不会静默丢弃设定值
 * @warning 与导纳 F_desired 类似，目标位姿更新不保证原子性之外的平滑性，
 *          大跳变请由调用方分步逼近
 */
TL_API Result set_constforce_target_pose(const std::vector<double>& target_pose);

/**
 * @brief 实时更新恒力控制的期望力（力接口）
 * @attention 实验性接口：接口与行为可能随版本调整
 *
 * 期望力即导纳控制的目标六维力（重力补偿 + 目标接触力），导纳每周期按
 * (传感器力 − 期望力) 求力误差。后台闭环每周期读取最新期望力，因此运行中
 * 调整立即改变稳态接触力，无需 close/open 重来：7000 端口伺服会话与导纳
 * 累计运动状态（接触位置）保持连续，不产生重新进给的冲击。
 *
 * @param desired_wrench 期望六维力/力矩 [Fx,Fy,Fz,Mx,My,Mz] (N, N·m)；以
 * 传感器/末端坐标系表达（与 open 时 ConstantForceParams::desired_wrench
 * 同系），含重力补偿项——改变末端姿态后偏置需重算
 * @return SUCCESS / PARAM_ERR（含非有限值）/
 * OPERATION_NOT_ALLOWED（未在运行）
 *
 * @warning 仅恒力闭环存活期间可调用（open_constforce 成功且未 close、后台线程未
 *          自行停止）；闭环已停止（如传感器断连超时自动停止）时返回
 *          OPERATION_NOT_ALLOWED，不会静默丢弃设定值
 * @warning 期望力整组原子生效（六维同一周期切换）；调整量表现为力误差跳变，
 *          由导纳动力学平滑消化，大跨度调整建议由调用方分步逼近
 */
TL_API Result set_constforce_desired_wrench(const std::array<double, 6>& desired_wrench);

/**
 * @brief 查询恒力控制运行时状态（线程安全）
 * @attention 实验性接口：接口与行为可能随版本调整
 *
 * 返回运行时状态：运行标志、当前设定值（target_pose / desired_wrench，由
 * open_constforce 与 set_constforce_* 即时写入）、原始/滤波后六维力与结束原因
 * （后台闭环每控制周期更新）。未 open 时返回 SUCCESS 且 running=false。
 * close_constforce 后各项保留最后一次值（running=false）。
 *
 * @param status [out] 状态快照
 * @return SUCCESS
 */
TL_API Result get_constforce_status(ConstForceStatus& status);

/**
 * @brief 关闭恒力控制并阻塞等待后台循环退出（幂等，可随时调用）
 * @attention 实验性接口：接口与行为可能随版本调整
 *
 * 三种场景：
 *   1. 后台循环运行中 → 置停止标志 → join → 返回结束原因
 *   2. 线程已自行退出（断连超时 / open_servoJ 失败）→ join 已退出线程 →
 * 返回结束原因
 *   3. 未 open 过或已 close 过 → 无可清理内容，返回 SUCCESS
 *
 * @return SUCCESS（用户主动关闭，或无可清理内容）/
 * EXCEPTION（传感器断连超时自动停止）/ open_servoJ 原始错误码
 *
 * @note 必须调用本函数完成清理（join 后台线程）；不调用则 std::thread 泄漏，
 *       进程退出时 std::terminate。
 * @warning 禁止在传感器回调（ForceSensorCallback）内调用本函数：回调运行于后台
 *          线程，此处 join 自身线程会抛出 std::system_error 并终止进程。
 *          回调中需要停止时请返回 false（计入断连计数，触发安全自动停止）。
 */
TL_API Result close_constforce();


} // namespace tl

#endif // TL_SDK_TL_CONSTANT_FORCE_H
