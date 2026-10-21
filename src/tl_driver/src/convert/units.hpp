#ifndef TL_DRIVER__CONVERT__UNITS_HPP_
#define TL_DRIVER__CONVERT__UNITS_HPP_

/// @file units.hpp
/// @brief ROS 接口单位与控制器 / SDK 原生单位之间的集中换算。
/// @note ROS 侧遵循 REP 103（m / rad / m·s⁻¹ / rad·s⁻¹）；SDK / 控制器侧为
///       mm / 度 / 度·s⁻¹。所有换算只允许出现在本层或调用本层的边界代码中。

#include <cstddef>
#include <vector>

namespace tl_driver::convert
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kMmToM = 0.001;
constexpr double kDegToRad = kPi / 180.0;
/// 电机轴转速 rpm → rad/s（1 rpm = 2π/60 rad/s）
constexpr double kRpmToRadPerSec = 2.0 * kPi / 60.0;

constexpr double mm_to_m(double mm)
{
  return mm * kMmToM;
}

constexpr double m_to_mm(double m)
{
  return m / kMmToM;
}

constexpr double deg_to_rad(double deg)
{
  return deg * kDegToRad;
}

constexpr double rad_to_deg(double rad)
{
  return rad / kDegToRad;
}

constexpr double mm_s_to_m_s(double mm_s)
{
  return mm_s * kMmToM;
}

constexpr double m_s_to_mm_s(double m_s)
{
  return m_s / kMmToM;
}

constexpr double rpm_to_rad_s(double rpm)
{
  return rpm * kRpmToRadPerSec;
}

constexpr double rad_s_to_rpm(double rad_s)
{
  return rad_s / kRpmToRadPerSec;
}

/// @brief 原地把前 count 个关节角由度转换为弧度。
inline void joint_deg_to_rad(std::vector<double>& values, std::size_t count)
{
  const std::size_t n = count < values.size() ? count : values.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    values[i] = deg_to_rad(values[i]);
  }
}

/// @brief 原地把前 count 个关节角由弧度转换为度。
inline void joint_rad_to_deg(std::vector<double>& values, std::size_t count)
{
  const std::size_t n = count < values.size() ? count : values.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    values[i] = rad_to_deg(values[i]);
  }
}

/// @brief 原地把前 count 个笛卡尔位置由米转换为毫米（姿态元素不动）。
inline void cartesian_m_to_mm(std::vector<double>& values, std::size_t count = 3)
{
  const std::size_t n = count < values.size() ? count : values.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    values[i] = m_to_mm(values[i]);
  }
}

/// @brief 原地把前 count 个笛卡尔位置由毫米转换为米（姿态元素不动）。
inline void cartesian_mm_to_m(std::vector<double>& values, std::size_t count = 3)
{
  const std::size_t n = count < values.size() ? count : values.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    values[i] = mm_to_m(values[i]);
  }
}

} // namespace tl_driver::convert

#endif // TL_DRIVER__CONVERT__UNITS_HPP_
