#include <algorithm>
#include <cmath>

#include "math/quaternion.hpp"

/**
 * @brief 四元数与欧拉角转换、球面插值的纯函数实现。
 */

namespace tl_driver::math
{

std::vector<double> rpy_to_quat(const std::vector<double>& rpy)
{
  double cr = std::cos(rpy[0] * 0.5);
  double sr = std::sin(rpy[0] * 0.5);
  double cp = std::cos(rpy[1] * 0.5);
  double sp = std::sin(rpy[1] * 0.5);
  double cy = std::cos(rpy[2] * 0.5);
  double sy = std::sin(rpy[2] * 0.5);
  return {
      cr * cp * cy + sr * sp * sy, // w
      sr * cp * cy - cr * sp * sy, // x
      cr * sp * cy + sr * cp * sy, // y
      cr * cp * sy - sr * sp * cy, // z
  };
}

std::vector<double> quat_to_rpy(const std::vector<double>& q)
{
  double w = q[0], x = q[1], y = q[2], z = q[3];
  double t0 = 2.0 * (w * x + y * z);
  double t1 = 1.0 - 2.0 * (x * x + y * y);
  double rx = std::atan2(t0, t1);
  double t2 = 2.0 * (w * y - z * x);
  t2 = std::max(-1.0, std::min(1.0, t2));
  double ry = std::asin(t2);
  double t3 = 2.0 * (w * z + x * y);
  double t4 = 1.0 - 2.0 * (y * y + z * z);
  double rz = std::atan2(t3, t4);
  return {rx, ry, rz};
}

std::vector<double> quat_slerp(const std::vector<double>& q1, const std::vector<double>& q2, double t)
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
  const double DOT_THRESHOLD = 0.9995;
  if (dot > DOT_THRESHOLD)
  {
    std::vector<double> result = {
        q1[0] + t * (q2w - q1[0]),
        q1[1] + t * (q2x - q1[1]),
        q1[2] + t * (q2y - q1[2]),
        q1[3] + t * (q2z - q1[3]),
    };
    double norm =
        std::sqrt(result[0] * result[0] + result[1] * result[1] + result[2] * result[2] + result[3] * result[3]);
    for (auto& v : result)
      v /= norm;
    return result;
  }
  double theta_0 = std::acos(dot);
  double sin_theta_0 = std::sin(theta_0);
  double theta = theta_0 * t;
  double s0 = std::cos(theta) - dot * std::sin(theta) / sin_theta_0;
  double s1 = std::sin(theta) / sin_theta_0;
  return {
      s0 * q1[0] + s1 * q2w,
      s0 * q1[1] + s1 * q2x,
      s0 * q1[2] + s1 * q2y,
      s0 * q1[3] + s1 * q2z,
  };
}

} // namespace tl_driver::math
