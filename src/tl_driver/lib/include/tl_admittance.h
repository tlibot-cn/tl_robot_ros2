/**
 * @file tl_admittance.h
 * @brief 导纳扩展 API — M-B-K 导纳控制器 + 机器人坐标系封装
 *
 * 分层定位:
 *   AdmittanceController — 纯 M-B-K 数值积分，SI 单位，内部 XYZ 坐标序
 *   AdmittanceController::CompensatePose — 单位转换 + 叠加的便捷封装
 *
 * 坐标系约定:
 *   位姿内部顺序 [X, Y, Z, RX, RY, RZ]（m, rad）
 *   传感器输入   [Fx,Fy,Fz, Mx,My,Mz]（N, N·m）
 *   力矩直通     Mx→RX, My→RY, Mz→RZ
 *
 * 使用流程:
 *   // 1. 创建并配置导纳控制器
 *   AdmittanceController ctrl;
 *   ctrl.Configure(params);
 *   ctrl.SetDesiredWrench(bias);  // 重力补偿
 *
 *   // 2. 主循环中调用补偿
 *   std::vector<double> wrench = {fx, fy, fz, mx, my, mz};
 *   std::vector<double> compensated(6);
 *   ctrl.CompensatePose(wrench, target, compensated);
 *   servo_movej(sock_tcp, sock_udp, compensated, 1);
 */

#ifndef TL_EXTENSION_TL_ADMITTANCE_H_
#define TL_EXTENSION_TL_ADMITTANCE_H_

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
 */
struct AdmittanceParams
{
  double mass[6];        /**< 惯性 M，须 > 0（SI） */
  double damping[6];     /**< 阻尼 B，>= 0（SI） */
  double stiffness[6];   /**< 刚度 K，>= 0（SI，线轴为 N/m） */
  double max_delta[6];   /**< 单周期位姿增量限幅：线轴 [0..2] 单位 m，姿态 [3..5] 单位 rad */
  double control_period; /**< 控制周期 dt（秒），须 > 0 */
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
 * 推荐主循环：Configure →（可选 SetDesiredWrench）→ 周期调用 Step。
 * Reset / GetRelativePose 为辅助接口。
 */
class TL_API AdmittanceController
{
public:
  AdmittanceController();

  /** @brief 配置 M/B/K、单步限幅与控制周期。成功后才允许调用 Step。
   *  @return 0=SUCCESS 成功；-3=PARAM_ERR 参数错误（如 M 非正、周期非正） */
  Result Configure(const AdmittanceParams& params);

  /** @brief 设置期望六维力/力矩 F_desired，顺序 [Fx,Fy,Fz,Mx,My,Mz]。
   *  @return 0=SUCCESS 成功；-3=PARAM_ERR 参数错误（向量长度不足） */
  Result SetDesiredWrench(const std::vector<double>& desired_wrench);

  /**
   * @brief 单步导纳计算（主循环接口）。
   * @param sensor_wrench [Fx,Fy,Fz,Mx,My,Mz] (N, N·m)
   * @param delta_pose    [out] [ΔX,ΔY,ΔZ,ΔRX,ΔRY,ΔRZ] (m, rad)
   * @return 0=SUCCESS 成功；-3=PARAM_ERR 参数错误；-5=EXCEPTION 未 Configure
   */
  Result Step(const std::vector<double>& sensor_wrench, std::vector<double>& delta_pose);

  /** @brief 复位内部运动状态（速度和累计位姿），保留参数和期望力。
   *  @return 0=SUCCESS 成功 */
  Result Reset();

  /** @brief 读取当前累计相对位姿 [X,Y,Z,RX,RY,RZ] (m, rad)。
   *  @return 0=SUCCESS 成功；-5=EXCEPTION 未 Configure */
  Result GetRelativePose(std::vector<double>& relative_pose) const;

  /** @brief 读取控制周期 dt（秒）。未 Configure 时返回 0。 */
  double GetControlPeriod() const
  {
    return configured_ ? control_period_ : 0.0;
  }

  /**
   * @brief 读取指定轴的刚度 K（SI：线轴 N/m，角轴 N·m/rad）
   * @param axis 轴序号 0..5（[X,Y,Z,RX,RY,RZ]）
   * @return K；未 Configure 或轴越界时返回 0
   * @note 力控轴 K=0（零稳态误差）。未 Configure 时返回 0 与力控轴相同，
   *       调用方应先确认已 Configure（如 GetControlPeriod() > 0）再据此判断。
   */
  double GetStiffness(int axis) const
  {
    return (configured_ && axis >= 0 && axis < 6) ? stiffness_[axis] : 0.0;
  }

  /**
   * @brief 单周期导纳补偿 — 输入传感器力和目标位姿，输出补偿后位姿
   *
   * 内部流程:
   *   Step() → GetRelativePose()
   *   → [X,Y,Z,RX,RY,RZ] (m,rad) → 缩放 + 叠加到 target_pose (mm,rad)
   *
   * 传感器方向取反（坐标系对齐）和低通滤波由调用方在传入前处理。
   *
   * @param sensor_wrench  当前力传感器数据 [Fx,Fy,Fz,Mx,My,Mz] (N, N·m)
   * @param target_pose    本周期目标位姿 [X,Y,Z,RX,RY,RZ] (mm, rad)
   * @param out_pose       [out] 补偿后位姿 [X,Y,Z,RX,RY,RZ] (mm, rad)
   * @param out_delta      [out] 可选: 本周期导纳增量 (mm, rad)；nullptr 跳过
   * @return SUCCESS / PARAM_ERR / EXCEPTION
   *
   * @note 必须周期性连续调用以维持导纳动力学连续性。
   */
  Result CompensatePose(const std::vector<double>& sensor_wrench, const std::vector<double>& target_pose,
                        std::vector<double>& out_pose, std::vector<double> *out_delta = nullptr);

private:
  void ResetMotionState();

  double mass_[6];
  double damping_[6];
  double stiffness_[6];
  double max_delta_[6];
  double control_period_;

  double desired_wrench_mapped_[6];
  double last_velocities_[6];
  double relative_position_[6];

  bool configured_;
};

} // namespace tl

#endif // TL_EXTENSION_TL_ADMITTANCE_H_
