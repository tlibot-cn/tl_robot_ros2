/// @file test_kinematics_status.cpp
/// @brief 本地解算状态的枚举数值与文本映射测试。
/// @note 只验证状态文本与 ROS 契约数值，不构建 RobotModel、不链接 SDK。

#include <gtest/gtest.h>

#include <string>

#include "kinematics_status.hpp"

namespace
{

using tl::KinematicsStatus;

TEST(KinematicsStatus, EnumValuesMatchRosContract)
{
  // `/tl_driver/ik` 的 status 直接透传枚举值，重排会让消费方误判，必须锁死。
  EXPECT_EQ(static_cast<int>(KinematicsStatus::OK), 0);
  EXPECT_EQ(static_cast<int>(KinematicsStatus::SINGULAR_WRIST), 1);
  EXPECT_EQ(static_cast<int>(KinematicsStatus::SINGULAR_ELBOW), 2);
  EXPECT_EQ(static_cast<int>(KinematicsStatus::SINGULAR_SHOULDER), 3);
  EXPECT_EQ(static_cast<int>(KinematicsStatus::SINGULAR_ARM_ANGLE), 4);
  EXPECT_EQ(static_cast<int>(KinematicsStatus::BRANCH_SWITCHED), 5);
  EXPECT_EQ(static_cast<int>(KinematicsStatus::UNREACHABLE), 6);
  EXPECT_EQ(static_cast<int>(KinematicsStatus::NO_SOLUTION), 7);
  EXPECT_EQ(static_cast<int>(KinematicsStatus::LIMIT_EXCEEDED), 8);
  EXPECT_EQ(static_cast<int>(KinematicsStatus::NOT_CONVERGED), 9);
  EXPECT_EQ(static_cast<int>(KinematicsStatus::PARAM_ERR), -1);
}

TEST(KinematicsStatus, StatusTextIsDistinct)
{
  const KinematicsStatus states[] = {
      KinematicsStatus::OK,
      KinematicsStatus::SINGULAR_WRIST,
      KinematicsStatus::SINGULAR_ELBOW,
      KinematicsStatus::SINGULAR_SHOULDER,
      KinematicsStatus::SINGULAR_ARM_ANGLE,
      KinematicsStatus::BRANCH_SWITCHED,
      KinematicsStatus::UNREACHABLE,
      KinematicsStatus::NO_SOLUTION,
      KinematicsStatus::LIMIT_EXCEEDED,
      KinematicsStatus::NOT_CONVERGED,
      KinematicsStatus::PARAM_ERR,
  };
  for (std::size_t i = 0; i < sizeof(states) / sizeof(states[0]); ++i)
  {
    const std::string text = tl_driver::status_text(states[i]);
    EXPECT_NE(text, "UNKNOWN");
    for (std::size_t j = i + 1; j < sizeof(states) / sizeof(states[0]); ++j)
    {
      EXPECT_NE(text, tl_driver::status_text(states[j]));
    }
  }
}

} // namespace
