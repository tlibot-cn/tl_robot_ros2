#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "math/quaternion.hpp"

/**
 * @brief 四元数/欧拉角转换与球面插值的单元测试（纯函数，无需机械臂）。
 */

namespace
{
constexpr double kPi = 3.14159265358979323846;

bool near(double a, double b, double eps = 1e-9)
{
  return std::fabs(a - b) < eps;
}

double norm(const std::vector<double>& q)
{
  double n = 0.0;
  for (double v : q)
  {
    n += v * v;
  }
  return std::sqrt(n);
}
} // namespace

TEST(Quaternion, RpyToQuatIdentity)
{
  const std::vector<double> q = tl_driver::math::rpy_to_quat({0.0, 0.0, 0.0});
  ASSERT_EQ(q.size(), 4u);
  EXPECT_TRUE(near(q[0], 1.0));
  EXPECT_TRUE(near(q[1], 0.0));
  EXPECT_TRUE(near(q[2], 0.0));
  EXPECT_TRUE(near(q[3], 0.0));
}

TEST(Quaternion, RpyQuatRoundTrip)
{
  const std::vector<double> rpy = {0.3, -0.4, 0.9};
  const std::vector<double> back = tl_driver::math::quat_to_rpy(tl_driver::math::rpy_to_quat(rpy));
  ASSERT_EQ(back.size(), 3u);
  for (std::size_t i = 0; i < 3; ++i)
  {
    EXPECT_TRUE(near(rpy[i], back[i])) << "axis " << i;
  }
}

TEST(Quaternion, RotationsAreUnitNorm)
{
  const std::vector<double> q = tl_driver::math::rpy_to_quat({1.1, -0.7, 2.2});
  EXPECT_TRUE(near(norm(q), 1.0)) << "norm=" << norm(q);
}

TEST(Quaternion, SlerpHitsBothEndpoints)
{
  const std::vector<double> q1 = tl_driver::math::rpy_to_quat({0.2, 0.1, -0.3});
  const std::vector<double> q2 = tl_driver::math::rpy_to_quat({0.9, -0.5, 0.4});

  const std::vector<double> start = tl_driver::math::quat_slerp(q1, q2, 0.0);
  const std::vector<double> end = tl_driver::math::quat_slerp(q1, q2, 1.0);
  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_TRUE(near(start[i], q1[i])) << "t=0 component " << i;
    EXPECT_TRUE(near(end[i], q2[i])) << "t=1 component " << i;
  }
}

TEST(Quaternion, SlerpMidpointOfQuarterTurn)
{
  const std::vector<double> q1 = tl_driver::math::rpy_to_quat({0.0, 0.0, 0.0});
  const std::vector<double> q2 = tl_driver::math::rpy_to_quat({0.0, 0.0, kPi / 2.0});
  const std::vector<double> mid = tl_driver::math::quat_slerp(q1, q2, 0.5);

  // 绕 Z 轴 45°：w = cos(22.5°)、z = sin(22.5°)
  EXPECT_TRUE(near(mid[0], std::cos(kPi / 8.0), 1e-12));
  EXPECT_TRUE(near(mid[3], std::sin(kPi / 8.0), 1e-12));
  EXPECT_TRUE(near(norm(mid), 1.0, 1e-12));
}

TEST(Quaternion, SlerpTakesShortestPathWhenTargetSignFlipped)
{
  const std::vector<double> q1 = tl_driver::math::rpy_to_quat({0.2, 0.1, -0.3});
  const std::vector<double> q2 = tl_driver::math::rpy_to_quat({0.9, -0.5, 0.4});
  const std::vector<double> q2_negated = {-q2[0], -q2[1], -q2[2], -q2[3]};

  // -q 与 q 表示同一旋转，插值路径必须一致
  const std::vector<double> a = tl_driver::math::quat_slerp(q1, q2, 0.35);
  const std::vector<double> b = tl_driver::math::quat_slerp(q1, q2_negated, 0.35);
  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_TRUE(near(a[i], b[i], 1e-12)) << "component " << i;
  }
}

TEST(Quaternion, SlerpNearlyIdenticalEndpointsStaysOnQuaternion)
{
  const std::vector<double> q = tl_driver::math::rpy_to_quat({0.5, 0.5, 0.5});
  const std::vector<double> mid = tl_driver::math::quat_slerp(q, q, 0.4);
  EXPECT_TRUE(near(norm(mid), 1.0, 1e-12)) << "norm=" << norm(mid);
  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_TRUE(near(mid[i], q[i], 1e-12)) << "component " << i;
  }
}
