/**
 * @file tl_admittance.h
 * @brief 导纳 API — M-B-K 导纳控制器与机器人坐标系适配
 *
 * 分层定位:
 *   AdmittanceController — 纯 M-B-K 数值积分，SI 单位，内部 XYZ 坐标序
 *   AdmittanceController::compensate_pose — 单位转换 + 叠加的便捷方法
 *
 * 坐标系约定:
 *   位姿内部顺序 [X, Y, Z, RX, RY, RZ]（m, rad）
 *   传感器输入   [Fx,Fy,Fz, Mx,My,Mz]（N, N·m）
 *   力矩直通     Mx→RX, My→RY, Mz→RZ
 *
 * 六维量按语义分两类容器:
 *   - 力/力矩与导纳参数（wrench、M/B/K/限幅、滤波/死区）：std::array<double, 6>，
 *     定长、栈上、无堆分配；
 *   - 位姿/位姿增量/相对位姿（pose、Δpose）：std::vector<double>，与机器人坐标
 *     接口（get_current_position / servo_movel 等）同容器，可直接互操作。
 * 顺序均在字段/参数注释中逐项标注。
 *
 * 使用流程:
 *   // 1. 创建并配置导纳控制器
 *   AdmittanceController ctrl;
 *   ctrl.configure(params);
 *   ctrl.set_desired_wrench(bias);  // 重力补偿
 *
 *   // 2. 主循环中调用补偿
 *   std::array<double, 6> wrench = {fx, fy, fz, mx, my, mz};
 *   std::vector<double> compensated(6);
 *   ctrl.compensate_pose(wrench, target, compensated);
 *   servo_movel(sock_tcp, compensated);  // 笛卡尔位姿 → 基坐标系直线伺服
 */

#ifndef TL_SDK_TL_ADMITTANCE_H_
#define TL_SDK_TL_ADMITTANCE_H_

#include <array>
#include <vector>
#include "tl_types.h"


namespace tl
{

/**
 * M-B-K 导纳参数（六维对角形式）。
 *
 * M/B/K 数组顺序为 [X,Y,Z,RX,RY,RZ]。
 * 传感器/期望力/力矩顺序为 [Fx,Fy,Fz,Mx,My,Mz]。
 * 位移使用米，角度使用弧度；力为 N，力矩为 N·m。
 *
 * 所有字段均带默认初值（与示例 ex_admittance_control.cpp 一致的典型配置），
 * 直接 configure 默认对象即可运行；按需覆盖具体字段。
 */
struct AdmittanceParams
{
  /** 惯性 M，须 > 0（SI）：线轴 kg，角轴 kg·m² */
  std::array<double, 6> mass{{8.0, 8.0, 8.0, 0.5, 0.5, 0.5}};
  /** 阻尼 B，>= 0（SI）：线轴 N/(m/s)，角轴 N·m/(rad/s) */
  std::array<double, 6> damping{{150.0, 150.0, 150.0, 2.0, 2.0, 2.0}};
  /** 刚度 K，>= 0（SI，线轴为 N/m） */
  std::array<double, 6> stiffness{{80.0, 80.0, 80.0, 3.0, 3.0, 3.0}};
  /** 单周期位姿增量限幅：线轴 [0..2] 单位 m，姿态 [3..5] 单位 rad */
  std::array<double, 6> max_delta{{0.0005, 0.0005, 0.0005, 0.005, 0.005, 0.005}};
  /** 控制周期 dt（秒），须 > 0 且 <= 1.0；默认 100 Hz */
  double control_period{0.01};
};

/**
 * M-B-K 导纳控制器 — 输入六维传感器力/力矩，输出本周期位姿增量 Δpose[6]。
 *
 * 位姿六维顺序 [X, Y, Z, RX, RY, RZ]：
 * - [0..2] 位移，单位 m
 * - [3..5] 角位移，单位 rad
 *
 * 传感器力矩直通映射：Mx→RX, My→RY, Mz→RZ。
 * 使用毫米制机器人接口的调用方应在边界处将线位移乘以 1000。
 *
 * 推荐主循环：configure →（可选 set_desired_wrench）→ 周期调用 step。
 * reset / get_relative_pose / get_params 为辅助接口。
 */
class TL_API AdmittanceController
{
public:
  AdmittanceController();

  /** @brief 配置 M/B/K、单步限幅与控制周期。成功后才允许调用 step。
   *  @return 0=SUCCESS 成功；-3=PARAM_ERR 参数错误（如 M 非正、周期非正） */
  Result configure(const AdmittanceParams& params);

  /** @brief 设置期望六维力/力矩 F_desired，顺序 [Fx,Fy,Fz,Mx,My,Mz]。
   *  @return 0=SUCCESS 成功；-3=PARAM_ERR 参数错误（含非有限值） */
  Result set_desired_wrench(const std::array<double, 6>& desired_wrench);

  /**
   * @brief 单步导纳计算（主循环接口）。
   * @param sensor_wrench [Fx,Fy,Fz,Mx,My,Mz] (N, N·m)
   * @param delta_pose    [out] [ΔX,ΔY,ΔZ,ΔRX,ΔRY,ΔRZ] (m, rad)，写满 6 个元素（无需预分配）
   * @return 0=SUCCESS 成功；-3=PARAM_ERR 参数错误；-5=EXCEPTION 未 configure
   */
  Result step(const std::array<double, 6>& sensor_wrench, std::vector<double>& delta_pose);

  /** @brief 复位内部运动状态（速度和累计位姿），保留参数和期望力。
   *  @return 0=SUCCESS 成功 */
  Result reset();

  /** @brief 读取当前累计相对位姿 [X,Y,Z,RX,RY,RZ] (m, rad)，写满 6 个元素（无需预分配）。
   *  @return 0=SUCCESS 成功；-5=EXCEPTION 未 configure */
  Result get_relative_pose(std::vector<double>& relative_pose) const;

  /** @brief 读取当前生效的完整导纳参数（M/B/K/单步限幅/控制周期）。
   *
   * 返回 configure 时写入的参数快照，便于诊断与日志；控制周期亦可用
   * get_control_period() 单独读取。
   *
   * @param params [out] 参数快照
   * @return 0=SUCCESS 成功；-5=EXCEPTION 未 configure */
  Result get_params(AdmittanceParams& params) const;

  /** @brief 读取控制周期 dt（秒）。未 configure 时返回 0。 */
  double get_control_period() const
  {
    return configured_ ? control_period_ : 0.0;
  }

  /**
   * @brief 单周期导纳补偿 — 输入传感器力和目标位姿，输出补偿后位姿
   *
   * 内部流程:
   *   step() → 读累计相对位姿
   *   → [X,Y,Z,RX,RY,RZ] (m,rad) → 缩放 + 叠加到 target_pose (mm,rad)
   *
   * 传感器方向取反（坐标系对齐）和低通滤波由调用方在传入前处理。
   *
   * @param sensor_wrench  当前力传感器数据 [Fx,Fy,Fz,Mx,My,Mz] (N, N·m)
   * @param target_pose    本周期目标位姿 [X,Y,Z,RX,RY,RZ] (mm, rad)，至少 6 个元素
   * @param out_pose       [out] 补偿后位姿 [X,Y,Z,RX,RY,RZ] (mm, rad)
   * @param out_delta      [out] 可选: 本周期导纳增量 (mm, rad)；nullptr 跳过
   * @return SUCCESS / PARAM_ERR / EXCEPTION
   *
   * @note 失败时 out_pose 回填为 target_pose、out_delta（非 nullptr 时）清零，
   *       调用方可安全使用出参，无需另行判断未初始化值。
   * @note 必须周期性连续调用以维持导纳动力学连续性。
   */
  Result compensate_pose(const std::array<double, 6>& sensor_wrench,
                         const std::vector<double>& target_pose, std::vector<double>& out_pose,
                         std::vector<double> *out_delta = nullptr);

private:
  void reset_motion_state();

  std::array<double, 6> mass_{};
  std::array<double, 6> damping_{};
  std::array<double, 6> stiffness_{};
  std::array<double, 6> max_delta_{};
  double control_period_;

  std::array<double, 6> desired_wrench_mapped_{};
  std::array<double, 6> last_velocities_{};
  std::array<double, 6> relative_position_{};

  /** compensate_pose 承接 step 本轮增量的复用缓冲：构造时分配一次，之后
   *  主循环路径不再堆分配；对外不可见 */
  std::vector<double> step_scratch_;

  bool configured_;
};

} // namespace tl

#endif // TL_SDK_TL_ADMITTANCE_H_
