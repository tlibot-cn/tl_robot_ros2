#ifndef TL_DRIVER__QUATERNION_HPP_
#define TL_DRIVER__QUATERNION_HPP_

/// @file quaternion.h
/// @brief 欧拉角与四元数互转、四元数球面插值（纯函数，无 ROS / SDK 依赖）。
/// @note 欧拉角约定为 XYZ 内旋，单位 rad。

#include <array>

namespace tl_driver::math
{
/// 欧拉角 (rx, ry, rz)，XYZ 内旋，单位 rad
using Rpy = std::array<double, 3>;
/// 四元数 (w, x, y, z)
using Quat = std::array<double, 4>;

/// @brief 欧拉角转四元数（XYZ 内旋）。
/// @param[in] rpy 欧拉角 (rad)
/// @return 单位四元数 (w, x, y, z)
Quat rpy_to_quat(const Rpy& rpy) noexcept;

/// @brief 四元数转欧拉角（XYZ 内旋）。
/// @param[in] q 四元数 (w, x, y, z)
/// @return 欧拉角 (rad)
Rpy quat_to_rpy(const Quat& q) noexcept;

/// @brief 四元数球面线性插值。
/// @note |dot| > 0.9995 时退化为线性插值后归一化，避免 sin 除法发散。
/// @param[in] q1 起始四元数
/// @param[in] q2 目标四元数（与 q1 夹角大于 90° 时按最短路径取反）
/// @param[in] t 插值系数，0 对应 q1、1 对应 q2
/// @return 插值四元数
Quat quat_slerp(const Quat& q1, const Quat& q2, double t) noexcept;
} // namespace tl_driver::math

#endif // TL_DRIVER__QUATERNION_HPP_
