/// @file test_state_frame.cpp
/// @brief 连续状态帧解析测试（纯逻辑，不依赖 SDK）。

#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "convert/state_frame.hpp"

namespace
{
using tl_driver::convert::parse_robot_state_frame;
using tl_driver::convert::RobotStateFrameData;
} // namespace

TEST(StateFrame, ParsesTopLevelAcs)
{
  const std::string message = R"({"channel":1,"realPosACS":[14.4717,-17.1725,14.3629,0.2096,-1.6568,14.4546]})";

  RobotStateFrameData frame;
  ASSERT_TRUE(parse_robot_state_frame(message, frame));
  EXPECT_EQ(frame.channel, 1);
  ASSERT_EQ(frame.acs_count, 6u);
  EXPECT_DOUBLE_EQ(frame.acs[0], 14.4717);
  EXPECT_DOUBLE_EQ(frame.acs[5], 14.4546);
  EXPECT_TRUE(std::isnan(frame.acs[6]));
  EXPECT_EQ(frame.mcs_count, 0u);
}

TEST(StateFrame, ParsesNestedReplyDataMcs)
{
  const std::string message =
      R"({"replyData":{"channel":2,"realPosMCS":[358.6336,93.1636,394.0389,179.9136,-1.17045,-0.00991]}})";

  RobotStateFrameData frame;
  ASSERT_TRUE(parse_robot_state_frame(message, frame));
  EXPECT_EQ(frame.channel, 2);
  ASSERT_EQ(frame.mcs_count, 6u);
  EXPECT_DOUBLE_EQ(frame.mcs[0], 358.6336);
  EXPECT_DOUBLE_EQ(frame.mcs[5], -0.00991);
  EXPECT_EQ(frame.acs_count, 0u);
}

TEST(StateFrame, AcceptsStringNumbers)
{
  const std::string message = R"({"channel":"1","realPosACS":["1.5","-2.5",0,3]})";

  RobotStateFrameData frame;
  ASSERT_TRUE(parse_robot_state_frame(message, frame));
  EXPECT_EQ(frame.channel, 1);
  ASSERT_EQ(frame.acs_count, 4u);
  EXPECT_DOUBLE_EQ(frame.acs[1], -2.5);
  EXPECT_DOUBLE_EQ(frame.acs[3], 3.0);
  EXPECT_TRUE(std::isnan(frame.acs[4]));
}

TEST(StateFrame, AcceptsSevenAxes)
{
  const std::string message = R"({"channel":1,"realPosACS":[1,2,3,4,5,6,7]})";

  RobotStateFrameData frame;
  ASSERT_TRUE(parse_robot_state_frame(message, frame));
  EXPECT_EQ(frame.acs_count, 7u);
  EXPECT_DOUBLE_EQ(frame.acs[6], 7.0);
}

TEST(StateFrame, AcceptsMcsWithRedundantArmAngle)
{
  const std::string message = R"({"channel":2,"realPosMCS":[358.6,93.1,394.0,179.9,-1.1,-0.01,12.5]})";

  RobotStateFrameData frame;
  ASSERT_TRUE(parse_robot_state_frame(message, frame));
  EXPECT_EQ(frame.mcs_count, 7u);
  EXPECT_DOUBLE_EQ(frame.mcs[6], 12.5);
}

TEST(StateFrame, RejectsMcsBeyondSevenAxes)
{
  RobotStateFrameData frame;
  EXPECT_FALSE(parse_robot_state_frame(R"({"channel":2,"realPosMCS":[1,2,3,4,5,6,7,8]})", frame));
}

TEST(StateFrame, RejectsInvalidChannel)
{
  RobotStateFrameData frame;
  EXPECT_FALSE(parse_robot_state_frame(R"({"channel":0,"realPosACS":[1,2,3]})", frame));
  EXPECT_FALSE(parse_robot_state_frame(R"({"channel":10,"realPosACS":[1,2,3]})", frame));
}

TEST(StateFrame, RejectsMalformedOrIrrelevantPayload)
{
  RobotStateFrameData frame;
  EXPECT_FALSE(parse_robot_state_frame("not json", frame));
  EXPECT_FALSE(parse_robot_state_frame(R"({"channel":1})", frame));
  EXPECT_FALSE(parse_robot_state_frame(R"({"channel":1,"realPosACS":"oops"})", frame));
  EXPECT_FALSE(parse_robot_state_frame(R"({"channel":1,"realPosACS":["1.5oops"]})", frame));
  EXPECT_FALSE(parse_robot_state_frame(R"({"channel":"1oops","realPosACS":[1,2,3]})", frame));
  EXPECT_FALSE(parse_robot_state_frame("", frame));
  // 超过定长 7 轴的帧视为未知布局，直接拒绝而不是静默截断。
  EXPECT_FALSE(parse_robot_state_frame(R"({"channel":1,"realPosACS":[1,2,3,4,5,6,7,8]})", frame));
}

TEST(StateFrame, RejectsChannelPayloadMismatch)
{
  RobotStateFrameData frame;
  // channel=1 必须带 realPosACS，channel=2 必须带 realPosMCS。
  EXPECT_FALSE(parse_robot_state_frame(R"({"channel":1,"realPosMCS":[1,2,3]})", frame));
  EXPECT_FALSE(parse_robot_state_frame(R"({"channel":2,"realPosACS":[1,2,3]})", frame));
}

TEST(StateFrame, KeepsOnlyMatchingChannelPayload)
{
  RobotStateFrameData frame;
  ASSERT_TRUE(parse_robot_state_frame(R"({"channel":1,"realPosACS":[1,2,3],"realPosMCS":[4,5,6]})", frame));
  EXPECT_EQ(frame.acs_count, 3u);
  EXPECT_EQ(frame.mcs_count, 0u);
  EXPECT_TRUE(std::isnan(frame.mcs[0]));

  ASSERT_TRUE(parse_robot_state_frame(R"({"channel":2,"realPosACS":[1,2,3],"realPosMCS":[4,5,6]})", frame));
  EXPECT_EQ(frame.acs_count, 0u);
  EXPECT_EQ(frame.mcs_count, 3u);
  EXPECT_TRUE(std::isnan(frame.acs[0]));
}
