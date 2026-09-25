#include <gtest/gtest.h>

#include <vector>

#include "convert/msg_convert.hpp"

/**
 * @brief ROS 消息 ↔ SDK 结构转换的单元测试（覆盖字段交叉映射与分支选择，无需机械臂）。
 */

TEST(MsgConvert, MoveCmdMapsEveryFieldToDistinctValue)
{
  tl_ros2_interface::msg::MoveCommand msg;
  msg.coord = 1;
  msg.velocity = 11.0;
  msg.velocity_sync = 12.0;
  msg.acc = 13.0;
  msg.dec = 14.0;
  msg.pl = 15;
  msg.time = 16;
  msg.tool_num = 17;
  msg.user_num = 18;
  msg.posidtype = 19;
  msg.configuration = 20;
  msg.spin = 2;
  msg.para_sync = true;
  msg.target_pos_value = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};

  const tl::MoveCmd cmd = tl_driver::convert::to_move_cmd(msg);

  // 当前驱动只支持自定义数组点位：无论消息里的 target_pos_type/target_pos_name 是什么，
  // 下发前一律改写为 PosType::data + 空点位名。若后续支持 GP 内置点位，本用例需同步调整。
  EXPECT_EQ(cmd.targetPosType, tl::PosType::data);
  EXPECT_TRUE(cmd.targetPosName.empty());
  EXPECT_EQ(cmd.coord, static_cast<tl::Coord>(1));
  EXPECT_DOUBLE_EQ(cmd.velocity, 11.0);
  EXPECT_DOUBLE_EQ(cmd.velocitySync, 12.0);
  EXPECT_DOUBLE_EQ(cmd.acc, 13.0);
  EXPECT_DOUBLE_EQ(cmd.dec, 14.0);
  EXPECT_EQ(cmd.pl, 15);
  EXPECT_EQ(cmd.time, 16);
  EXPECT_EQ(cmd.toolNum, 17);
  EXPECT_EQ(cmd.userNum, 18);
  EXPECT_EQ(cmd.posidtype, 19);
  EXPECT_EQ(cmd.configuration, 20);
  EXPECT_EQ(cmd.spin, 2);
  EXPECT_TRUE(cmd.parasync);
  EXPECT_EQ(cmd.targetPosValue, msg.target_pos_value);
}

TEST(MsgConvert, DhParamRoundTripKeepsValues)
{
  tl_ros2_interface::msg::RobotDHParam in;
  in.l1 = 1.5;
  in.l20 = -2.5;
  in.couple_coe_1_2 = 0.25;
  in.couple_coe_5_6 = -0.75;
  in.pitch = 3.0;
  in.sliding_lead_value = 3.5;
  in.uplift_lead_value = 3.25;
  in.spray_distance = 4.0;
  in.three_axis_direction = 1.0;
  in.five_axis_direction = 2.0;
  in.two_axis_convertion_ratio = 5.0;
  in.three_axis_convertion_ratio = 5.5;
  in.amplification_ratio = 6.0;
  in.convertion_ratio_x = 7.0;
  in.convertion_ratio_y = 7.5;
  in.convertion_ratio_z = 7.25;
  in.convertion_ratio_j1 = 8.0;
  in.convertion_ratio_j2 = 8.5;
  in.convertion_ratio_j3 = 8.25;
  in.upside_down = 1;
  in.pc = 9.0;
  in.sp = {0.1, 0.2, 0.3};
  in.tl = {0.4, 0.5, 0.6};

  tl::RobotDHParam sdk{};
  tl_driver::convert::to_sdk_dh_param(in, sdk);

  tl_ros2_interface::msg::RobotDHParam out;
  tl_driver::convert::to_msg_dh_param(sdk, out);

  EXPECT_DOUBLE_EQ(out.l1, in.l1);
  EXPECT_DOUBLE_EQ(out.l20, in.l20);
  EXPECT_DOUBLE_EQ(out.couple_coe_1_2, in.couple_coe_1_2);
  EXPECT_DOUBLE_EQ(out.couple_coe_5_6, in.couple_coe_5_6);
  EXPECT_DOUBLE_EQ(out.pitch, in.pitch);
  EXPECT_DOUBLE_EQ(out.sliding_lead_value, in.sliding_lead_value);
  EXPECT_DOUBLE_EQ(out.uplift_lead_value, in.uplift_lead_value);
  EXPECT_DOUBLE_EQ(out.spray_distance, in.spray_distance);
  EXPECT_DOUBLE_EQ(out.three_axis_direction, in.three_axis_direction);
  EXPECT_DOUBLE_EQ(out.five_axis_direction, in.five_axis_direction);
  EXPECT_DOUBLE_EQ(out.two_axis_convertion_ratio, in.two_axis_convertion_ratio);
  EXPECT_DOUBLE_EQ(out.three_axis_convertion_ratio, in.three_axis_convertion_ratio);
  EXPECT_DOUBLE_EQ(out.amplification_ratio, in.amplification_ratio);
  EXPECT_DOUBLE_EQ(out.convertion_ratio_x, in.convertion_ratio_x);
  EXPECT_DOUBLE_EQ(out.convertion_ratio_y, in.convertion_ratio_y);
  EXPECT_DOUBLE_EQ(out.convertion_ratio_z, in.convertion_ratio_z);
  EXPECT_DOUBLE_EQ(out.convertion_ratio_j1, in.convertion_ratio_j1);
  EXPECT_DOUBLE_EQ(out.convertion_ratio_j2, in.convertion_ratio_j2);
  EXPECT_DOUBLE_EQ(out.convertion_ratio_j3, in.convertion_ratio_j3);
  EXPECT_EQ(out.upside_down, in.upside_down);
  EXPECT_DOUBLE_EQ(out.pc, in.pc);
  EXPECT_EQ(out.sp, in.sp);
  EXPECT_EQ(out.tl, in.tl);
}

TEST(MsgConvert, DhParamPadsShortHanYuVectorsWithZero)
{
  tl_ros2_interface::msg::RobotDHParam in;
  in.sp = {0.1};
  in.tl = {};

  tl::RobotDHParam sdk{};
  tl_driver::convert::to_sdk_dh_param(in, sdk);

  EXPECT_DOUBLE_EQ(sdk.hanyu.SP[0], 0.1);
  EXPECT_DOUBLE_EQ(sdk.hanyu.SP[1], 0.0);
  EXPECT_DOUBLE_EQ(sdk.hanyu.SP[2], 0.0);
  EXPECT_DOUBLE_EQ(sdk.hanyu.TL[0], 0.0);
  EXPECT_DOUBLE_EQ(sdk.hanyu.TL[1], 0.0);
  EXPECT_DOUBLE_EQ(sdk.hanyu.TL[2], 0.0);

  tl_ros2_interface::msg::RobotDHParam out;
  tl_driver::convert::to_msg_dh_param(sdk, out);
  EXPECT_EQ(out.sp, std::vector<double>({0.1, 0.0, 0.0}));
  EXPECT_EQ(out.tl, std::vector<double>({0.0, 0.0, 0.0}));
}

TEST(MsgConvert, ModbusTcpBranchFillsTcpFields)
{
  tl_ros2_interface::msg::ModbusMasterParam msg;
  msg.type = "TCP";
  msg.start_addr = true;
  msg.tcp.ip = "10.0.0.7";
  msg.tcp.port = 1502;

  tl::ModbusMasterParameter sdk;
  ASSERT_TRUE(tl_driver::convert::to_sdk_modbus_param(msg, sdk));
  EXPECT_EQ(sdk.type, "TCP");
  EXPECT_TRUE(sdk.startAddress);
  EXPECT_EQ(sdk.TCP.IP, "10.0.0.7");
  EXPECT_EQ(sdk.TCP.port, 1502);
}

TEST(MsgConvert, ModbusRtuBranchFillsRtuFields)
{
  tl_ros2_interface::msg::ModbusMasterParam msg;
  msg.type = "RTU";
  msg.start_addr = false;
  msg.rtu.slave_id = 3;
  msg.rtu.port = 2;
  msg.rtu.baudrate = 9600;
  msg.rtu.check_bit = "Even";
  msg.rtu.data_bit = 7;
  msg.rtu.stop_bit = 2;

  tl::ModbusMasterParameter sdk;
  ASSERT_TRUE(tl_driver::convert::to_sdk_modbus_param(msg, sdk));
  EXPECT_EQ(sdk.type, "RTU");
  EXPECT_FALSE(sdk.startAddress);
  EXPECT_EQ(sdk.RTU.slaveId, 3);
  EXPECT_EQ(sdk.RTU.port, 2);
  EXPECT_EQ(sdk.RTU.baudrate, 9600);
  EXPECT_EQ(sdk.RTU.checkBit, "Even");
  EXPECT_EQ(sdk.RTU.dataBit, 7);
  EXPECT_EQ(sdk.RTU.stopBit, 2);
}

TEST(MsgConvert, ModbusUnknownTypeIsRejected)
{
  tl_ros2_interface::msg::ModbusMasterParam msg;
  msg.type = "ASCII";

  tl::ModbusMasterParameter sdk;
  EXPECT_FALSE(tl_driver::convert::to_sdk_modbus_param(msg, sdk));
}
