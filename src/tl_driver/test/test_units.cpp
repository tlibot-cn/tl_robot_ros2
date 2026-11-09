/// @file test_units.cpp
/// @brief 单位换算工具的边界与往返测试。

#include <gtest/gtest.h>

#include <vector>

#include "convert/units.hpp"

namespace
{
constexpr double kTolerance = 1e-12;

TEST(Units, LengthRoundTrip)
{
  EXPECT_NEAR(tl_driver::convert::mm_to_m(1000.0), 1.0, kTolerance);
  EXPECT_NEAR(tl_driver::convert::m_to_mm(0.25), 250.0, kTolerance);
  const double mm = -123.456;
  EXPECT_NEAR(tl_driver::convert::m_to_mm(tl_driver::convert::mm_to_m(mm)), mm, kTolerance);
}

TEST(Units, AngleRoundTrip)
{
  EXPECT_NEAR(tl_driver::convert::deg_to_rad(180.0), tl_driver::convert::kPi, kTolerance);
  EXPECT_NEAR(tl_driver::convert::rad_to_deg(tl_driver::convert::kPi / 2.0), 90.0, kTolerance);
  const double deg = -37.5;
  EXPECT_NEAR(tl_driver::convert::rad_to_deg(tl_driver::convert::deg_to_rad(deg)), deg, kTolerance);
}

TEST(Units, LinearSpeedRoundTrip)
{
  EXPECT_NEAR(tl_driver::convert::mm_s_to_m_s(250.0), 0.25, kTolerance);
  EXPECT_NEAR(tl_driver::convert::m_s_to_mm_s(0.5), 500.0, kTolerance);
}

TEST(Units, RpmToRadPerSec)
{
  EXPECT_NEAR(tl_driver::convert::rpm_to_rad_s(60.0), 2.0 * tl_driver::convert::kPi, kTolerance);
  EXPECT_NEAR(tl_driver::convert::rad_s_to_rpm(2.0 * tl_driver::convert::kPi), 60.0, kTolerance);
}

TEST(Units, JointVectorConversionHonorsCount)
{
  std::vector<double> degrees{180.0, -90.0, 45.0, 10.0};
  tl_driver::convert::joint_deg_to_rad(degrees, 2);
  EXPECT_NEAR(degrees[0], tl_driver::convert::kPi, kTolerance);
  EXPECT_NEAR(degrees[1], -tl_driver::convert::kPi / 2.0, kTolerance);
  EXPECT_DOUBLE_EQ(degrees[2], 45.0);
  EXPECT_DOUBLE_EQ(degrees[3], 10.0);

  std::vector<double> radians{tl_driver::convert::kPi, -tl_driver::convert::kPi / 2.0, 0.7853981633974483,
                              0.17453292519943295};
  tl_driver::convert::joint_rad_to_deg(radians, 10); // count 超过长度时只处理已有元素
  EXPECT_NEAR(radians[0], 180.0, kTolerance);
  EXPECT_NEAR(radians[1], -90.0, kTolerance);
  EXPECT_NEAR(radians[2], 45.0, kTolerance);
  EXPECT_NEAR(radians[3], 10.0, kTolerance);
}
} // namespace
