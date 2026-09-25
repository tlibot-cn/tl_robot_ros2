/// @file quaternion.cpp
/// @brief 欧拉角/四元数互转与球面插值的实现（与拆分前 TL_Arm 内的实现等价）。

#include "math/quaternion.h"

#include <algorithm>
#include <cmath>

namespace tl_driver::math
{
Quat rpy_to_quat(const Rpy& rpy) noexcept
{
  const double cr = std::cos(rpy[0] * 0.5);
  const double sr = std::sin(rpy[0] * 0.5);
  const double cp = std::cos(rpy[1] * 0.5);
  const double sp = std::sin(rpy[1] * 0.5);
  const double cy = std::cos(rpy[2] * 0.5);
  const double sy = std::sin(rpy[2] * 0.5);
  return {
      cr * cp * cy + sr * sp * sy, // w
      sr * cp * cy - cr * sp * sy, // x
      cr * sp * cy + sr * cp * sy, // y
      cr * cp * sy - sr * sp * cy, // z
  };
}

Rpy quat_to_rpy(const Quat& q) noexcept
{
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  const double t0 = 2.0 * (w * x + y * z);
  const double t1 = 1.0 - 2.0 * (x * x + y * y);
  const double rx = std::atan2(t0, t1);
  double t2 = 2.0 * (w * y - z * x);
  t2 = std::max(-1.0, std::min(1.0, t2));
  const double ry = std::asin(t2);
  const double t3 = 2.0 * (w * z + x * y);
  const double t4 = 1.0 - 2.0 * (y * y + z * z);
  const double rz = std::atan2(t3, t4);
  return {rx, ry, rz};
}

Quat quat_slerp(const Quat& q1, const Quat& q2, double t) noexcept
{
  double q2w = q2[0], q2x = q2[1], q2y = q2[2], q2z = q2[3];
  double dot = q1[0] * q2w + q1[1] * q2x + q1[2] * q2y + q1[3] * q2z;
  if (dot < 0.0)
  {
    q2w = -q2w;
    q2x = -q2x;
    q2y = -q2y;
    q2z = -q2z;
    dot = -dot;
  }

  constexpr double kDotThreshold = 0.9995;
  if (dot > kDotThreshold)
  {
    Quat result = {
        q1[0] + t * (q2w - q1[0]),
        q1[1] + t * (q2x - q1[1]),
        q1[2] + t * (q2y - q1[2]),
        q1[3] + t * (q2z - q1[3]),
    };
    const double norm =
        std::sqrt(result[0] * result[0] + result[1] * result[1] + result[2] * result[2] + result[3] * result[3]);
    for (auto& v : result)
    {
      v /= norm;
    }
    return result;
  }

  const double theta_0 = std::acos(dot);
  const double sin_theta_0 = std::sin(theta_0);
  const double theta = theta_0 * t;
  const double s0 = std::cos(theta) - dot * std::sin(theta) / sin_theta_0;
  const double s1 = std::sin(theta) / sin_theta_0;
  return {
      s0 * q1[0] + s1 * q2w,
      s0 * q1[1] + s1 * q2x,
      s0 * q1[2] + s1 * q2y,
      s0 * q1[3] + s1 * q2z,
  };
}
} // namespace tl_driver::math
