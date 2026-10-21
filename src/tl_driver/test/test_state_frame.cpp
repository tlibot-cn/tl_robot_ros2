/// @file test_state_frame.cpp
/// @brief 连续状态帧解析测试（纯逻辑，不依赖 SDK）。

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "convert/state_frame.hpp"

namespace
{
using tl_driver::convert::RobotStateFrameData;
using tl_driver::convert::parse_robot_state_frame;
} // namespace

TEST(StateFrame, ParsesTopLevelAcs)
{
  const std::string message = R"({"channel":1,"realPosACS":[14.4717,-17.1725,14.3629,0.2096,-1.6568,14.4546]})";

  RobotStateFrameData frame;
  ASSERT_TRUE(parse_robot_state_frame(message, frame));
  EXPECT_EQ(frame.channel, 1);
  ASSERT_EQ(frame.acs.size(), 6u);
  EXPECT_DOUBLE_EQ(frame.acs[0], 14.4717);
  EXPECT_DOUBLE_EQ(frame.acs[5], 14.4546);
  EXPECT_TRUE(frame.mcs.empty());
}

TEST(StateFrame, ParsesNestedReplyDataMcs)
{
  const std::string message =
      R"({"replyData":{"channel":2,"realPosMCS":[358.6336,93.1636,394.0389,179.9136,-1.17045,-0.00991]}})";

  RobotStateFrameData frame;
  ASSERT_TRUE(parse_robot_state_frame(message, frame));
  EXPECT_EQ(frame.channel, 2);
  ASSERT_EQ(frame.mcs.size(), 6u);
  EXPECT_DOUBLE_EQ(frame.mcs[0], 358.6336);
  EXPECT_DOUBLE_EQ(frame.mcs[5], -0.00991);
  EXPECT_TRUE(frame.acs.empty());
}

TEST(StateFrame, AcceptsStringNumbers)
{
  const std::string message = R"({"channel":"1","realPosACS":["1.5","-2.5",0,3]})";

  RobotStateFrameData frame;
  ASSERT_TRUE(parse_robot_state_frame(message, frame));
  EXPECT_EQ(frame.channel, 1);
  ASSERT_EQ(frame.acs.size(), 4u);
  EXPECT_DOUBLE_EQ(frame.acs[1], -2.5);
  EXPECT_DOUBLE_EQ(frame.acs[3], 3.0);
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
  EXPECT_EQ(frame.acs.size(), 3u);
  EXPECT_TRUE(frame.mcs.empty());

  ASSERT_TRUE(parse_robot_state_frame(R"({"channel":2,"realPosACS":[1,2,3],"realPosMCS":[4,5,6]})", frame));
  EXPECT_TRUE(frame.acs.empty());
  EXPECT_EQ(frame.mcs.size(), 3u);
}
