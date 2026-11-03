#ifndef TL_DRIVER__KINEMATICS_LOCAL_HPP_
#define TL_DRIVER__KINEMATICS_LOCAL_HPP_

/// @file kinematics_local.hpp
/// @brief 启动时由控制器参数构建一次、之后只读的本地运动学求解器。
/// @note 模型参数（DH、关节限位、当前工具手）在启动时回读控制器，运行期不重建；
///       解算为纯本地计算，不占用 6001 请求通道。仅支持基坐标系（BASE）与 6/7 轴。

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "tl_kinematics.h"

namespace tl_driver
{
/// @brief 本地正逆解封装；不可用时所有解算入口返回 PARAM_ERR。
class LocalKinematics
{
public:
  /// @brief 位姿 [X, Y, Z, A, B, C]，位移 mm、姿态 rad。
  using Pose = std::array<double, 6>;

  /// @brief 从控制器读取 DH、关节限位与当前工具手，构建本地模型。
  /// @note 只应在启动时调用一次；失败只记录原因并把对象置为不可用，不抛异常。
  /// @param[in] fd 已连接的 6001 端口
  /// @param[in] ndof 期望轴数（6 或 7）
  void build(int fd, int ndof);

  /// @brief 模型是否可用。
  bool available() const noexcept
  {
    return available_;
  }

  /// @brief 模型轴数；不可用时为 0。
  int dof() const noexcept
  {
    return dof_;
  }

  /// @brief 不可用原因；可用时为空。
  const std::string& error() const noexcept
  {
    return error_;
  }

  /// @brief 正解：关节角 (deg) → 基坐标系位姿 (mm + rad)。
  /// @return OK / SINGULAR 成功；其余失败（见 KinematicsStatus）
  tl::KinematicsStatus fk(const std::vector<double>& joints_deg, Pose& pose) const noexcept;

  /// @brief 逆解：基坐标系位姿 (mm + rad) → 关节角 (deg)。
  /// @param[in] arm_angle_rad 7 轴的臂角 ψ (rad)；6 轴忽略（内部改传 kArmAngleNone）
  /// @param[out] joints_deg 长度 7，前 axes 项有效
  /// @param[out] axes 有效关节数（6 或 7）
  tl::KinematicsStatus ik(const Pose& pose, const std::vector<double>& ref_joints_deg, int form, double arm_angle_rad,
                          std::array<double, 7>& joints_deg, int& axes) const noexcept;

  /// @brief 关节角 (deg) → 臂角 ψ (rad)；仅 7 轴。
  tl::KinematicsStatus arm_angle_of(const std::vector<double>& joints_deg, double& arm_angle_rad) const noexcept;

  /// @brief KinematicsStatus → 可读字符串（失败信息用）。
  static const char *status_text(tl::KinematicsStatus status) noexcept;

private:
  std::optional<tl::Kinematics<6>> kin6_;
  std::optional<tl::Kinematics<7>> kin7_;
  int dof_{0};
  bool available_{false};
  std::string error_;
};

} // namespace tl_driver

#endif // TL_DRIVER__KINEMATICS_LOCAL_HPP_
