/// @file test_quaternion.cpp
/// @brief math/quaternion 纯函数的单元测试。

#include <cmath>

#include <gtest/gtest.h>

#include "math/quaternion.h"

namespace
{
using tl_driver::math::Quat;
using tl_driver::math::Rpy;

constexpr double kPi = 3.14159265358979323846;

/// @brief 断言两个欧拉角在给定容差内相等。
void expect_rpy_near(const Rpy& actual, const Rpy& expected, double tol = 1e-9)
{
  for (std::size_t i = 0; i < 3; ++i)
  {
    EXPECT_NEAR(actual[i], expected[i], tol) << "分量 " << i;
  }
}

/// @brief 断言四元数为单位四元数。
void expect_unit(const Quat& q, double tol = 1e-12)
{
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  EXPECT_NEAR(n, 1.0, tol);
}
} // namespace

TEST(Quaternion, RpyToQuatIdentity)
{
  const Quat q = tl_driver::math::rpy_to_quat({0.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(q[0], 1.0);
  EXPECT_DOUBLE_EQ(q[1], 0.0);
  EXPECT_DOUBLE_EQ(q[2], 0.0);
  EXPECT_DOUBLE_EQ(q[3], 0.0);
}

TEST(Quaternion, RpyToQuatRotateAboutZ)
{
  // 绕 z 轴 90°（XYZ 内旋）→ (√2/2, 0, 0, √2/2)
  const Quat q = tl_driver::math::rpy_to_quat({0.0, 0.0, kPi / 2.0});
  EXPECT_NEAR(q[0], std::sqrt(2.0) / 2.0, 1e-12);
  EXPECT_NEAR(q[1], 0.0, 1e-12);
  EXPECT_NEAR(q[2], 0.0, 1e-12);
  EXPECT_NEAR(q[3], std::sqrt(2.0) / 2.0, 1e-12);
  expect_unit(q);
}

TEST(Quaternion, RoundTripRpy)
{
  const Rpy cases[] = {
      {0.0, 0.0, 0.0},
      {0.3, -0.4, 0.5},
      {-1.2, 0.9, 2.5},
      {kPi / 2.0, 0.0, -kPi / 3.0},
  };
  for (const auto& rpy : cases)
  {
    expect_rpy_near(tl_driver::math::quat_to_rpy(tl_driver::math::rpy_to_quat(rpy)), rpy, 1e-12);
  }
}

TEST(Quaternion, SlerpEndpointsReturnEndpoints)
{
  const Quat q1 = tl_driver::math::rpy_to_quat({0.1, 0.2, 0.3});
  const Quat q2 = tl_driver::math::rpy_to_quat({-0.5, 0.7, 1.1});

  const Quat at0 = tl_driver::math::quat_slerp(q1, q2, 0.0);
  const Quat at1 = tl_driver::math::quat_slerp(q1, q2, 1.0);
  for (std::size_t i = 0; i < 4; ++i)
  {
    EXPECT_NEAR(at0[i], q1[i], 1e-12);
    EXPECT_NEAR(at1[i], q2[i], 1e-12);
  }
}

TEST(Quaternion, SlerpTakesShortestPathForNegatedQuaternion)
{
  // q 与 -q 表示同一姿态：插值必须走短路径（10° 中点 ≈ 5°），而不是绕远 175°
  const Quat q1 = tl_driver::math::rpy_to_quat({0.0, 0.0, 0.0});
  const Quat q2 = tl_driver::math::rpy_to_quat({0.0, 0.0, 10.0 * kPi / 180.0});
  const Quat q2_neg = {-q2[0], -q2[1], -q2[2], -q2[3]};

  const Rpy mid = tl_driver::math::quat_to_rpy(tl_driver::math::quat_slerp(q1, q2_neg, 0.5));
  expect_rpy_near(mid, {0.0, 0.0, 5.0 * kPi / 180.0}, 1e-12);
}

TEST(Quaternion, SlerpKeepsUnitNormOverSweep)
{
  const Quat q1 = tl_driver::math::rpy_to_quat({0.2, -0.6, 1.3});
  const Quat q2 = tl_driver::math::rpy_to_quat({-1.0, 0.4, -0.2});
  for (int i = 0; i <= 20; ++i)
  {
    const double t = static_cast<double>(i) / 20.0;
    expect_unit(tl_driver::math::quat_slerp(q1, q2, t), 1e-12);
  }
}

TEST(Quaternion, SlerpNearParallelBranchStaysUnitAndHalvesAngle)
{
  // dot > 0.9995 走线性插值 + 归一化分支
  const double tiny = 1e-5; // 弧度
  const Quat q1 = tl_driver::math::rpy_to_quat({0.0, 0.0, 0.0});
  const Quat q2 = tl_driver::math::rpy_to_quat({0.0, 0.0, tiny});

  const Quat mid = tl_driver::math::quat_slerp(q1, q2, 0.5);
  expect_unit(mid);
  expect_rpy_near(tl_driver::math::quat_to_rpy(mid), {0.0, 0.0, tiny / 2.0}, 1e-12);
}

TEST(Quaternion, QuatToRpyClampsGimbalLock)
{
  // 俯仰角锁死时 asin 入参被夹紧到 1，结果应为有限值（±π/2）而非 NaN
  const Quat q = {std::sqrt(2.0) / 2.0, 0.0, std::sqrt(2.0) / 2.0, 0.0};
  const Rpy rpy = tl_driver::math::quat_to_rpy(q);
  for (std::size_t i = 0; i < 3; ++i)
  {
    EXPECT_TRUE(std::isfinite(rpy[i]));
  }
  EXPECT_NEAR(rpy[1], kPi / 2.0, 1e-12);
}
