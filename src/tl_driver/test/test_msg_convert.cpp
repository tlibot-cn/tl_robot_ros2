/// @file test_msg_convert.cpp
/// @brief convert/msg_convert 转换与校验的单元测试。

#include <gtest/gtest.h>

#include "convert/msg_convert.h"
#include "tl_ros2_interface/msg/modbus_master_param.hpp"
#include "tl_ros2_interface/msg/move_command.hpp"
#include "tl_ros2_interface/msg/robot_dh_param.hpp"

namespace
{
using tl_ros2_interface::msg::ModbusMasterParam;
using tl_ros2_interface::msg::MoveCommand;
using tl_ros2_interface::msg::RobotDHParam;
} // namespace

TEST(MsgConvert, ToMoveCmdMapsAllFields)
{
  MoveCommand msg;
  msg.coord = 2;
  msg.velocity = 11.5;
  msg.velocity_sync = 22.5;
  msg.acc = 33.5;
  msg.dec = 44.5;
  msg.pl = 5;
  msg.time = 6;
  msg.tool_num = 7;
  msg.user_num = 8;
  msg.posidtype = 9;
  msg.configuration = 10;
  msg.spin = 11;
  msg.para_sync = true;
  msg.target_pos_value = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0};

  const tl::MoveCmd cmd = tl_driver::convert::to_move_cmd(msg);

  EXPECT_EQ(cmd.targetPosType, static_cast<tl::PosType>(tl::PosType::data));
  EXPECT_EQ(cmd.targetPosName, "");
  EXPECT_EQ(static_cast<int>(cmd.coord), 2);
  EXPECT_DOUBLE_EQ(cmd.velocity, 11.5);
  EXPECT_DOUBLE_EQ(cmd.velocitySync, 22.5);
  EXPECT_DOUBLE_EQ(cmd.acc, 33.5);
  EXPECT_DOUBLE_EQ(cmd.dec, 44.5);
  EXPECT_EQ(cmd.pl, 5);
  EXPECT_EQ(cmd.time, 6);
  EXPECT_EQ(cmd.toolNum, 7);
  EXPECT_EQ(cmd.userNum, 8);
  EXPECT_EQ(cmd.posidtype, 9);
  EXPECT_EQ(cmd.configuration, 10);
  EXPECT_EQ(cmd.spin, 11);
  EXPECT_EQ(cmd.parasync, true);
  ASSERT_EQ(cmd.targetPosValue.size(), msg.target_pos_value.size());
  for (std::size_t i = 0; i < msg.target_pos_value.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(cmd.targetPosValue[i], msg.target_pos_value[i]) << "分量 " << i;
  }
}

TEST(MsgConvert, ToMoveCmdOnDefaultMessageKeepsEmptyTarget)
{
  const MoveCommand msg;
  const tl::MoveCmd cmd = tl_driver::convert::to_move_cmd(msg);
  EXPECT_TRUE(cmd.targetPosValue.empty());
  EXPECT_EQ(cmd.toolNum, 0);
  EXPECT_DOUBLE_EQ(cmd.velocity, 0.0);
}

TEST(MsgConvert, ModbusTcpTypeIsUpperCasedAndMapped)
{
  ModbusMasterParam msg;
  msg.type = "tcp"; // 小写输入应被接受
  msg.start_addr = true;
  msg.tcp.ip = "192.168.1.50";
  msg.tcp.port = 502;

  tl::ModbusMasterParameter param;
  ASSERT_TRUE(tl_driver::convert::to_modbus_master_param(msg, param));
  EXPECT_EQ(param.type, "TCP");
  EXPECT_EQ(param.startAddress, 1);
  EXPECT_EQ(param.TCP.IP, "192.168.1.50");
  EXPECT_EQ(param.TCP.port, 502);
}

TEST(MsgConvert, ModbusRtuTypeIsUpperCasedAndMapped)
{
  ModbusMasterParam msg;
  msg.type = "Rtu";
  msg.start_addr = false;
  msg.rtu.slave_id = 1;
  msg.rtu.port = 1;
  msg.rtu.baudrate = 115200;
  msg.rtu.check_bit = "N";
  msg.rtu.data_bit = 8;
  msg.rtu.stop_bit = 1;

  tl::ModbusMasterParameter param;
  ASSERT_TRUE(tl_driver::convert::to_modbus_master_param(msg, param));
  EXPECT_EQ(param.type, "RTU");
  EXPECT_EQ(param.startAddress, 0);
  EXPECT_EQ(param.RTU.slaveId, 1);
  EXPECT_EQ(param.RTU.port, 1);
  EXPECT_EQ(param.RTU.baudrate, 115200);
  EXPECT_EQ(param.RTU.checkBit, "N");
  EXPECT_EQ(param.RTU.dataBit, 8);
  EXPECT_EQ(param.RTU.stopBit, 1);
}

TEST(MsgConvert, ModbusUnknownTypeIsRejected)
{
  ModbusMasterParam msg;
  msg.type = "udp";
  tl::ModbusMasterParameter param;
  EXPECT_FALSE(tl_driver::convert::to_modbus_master_param(msg, param));
}

TEST(MsgConvert, DhParamRoundTripKeepsSixJointsAndAngles)
{
  RobotDHParam msg;
  msg.alpha = {0.0, -1.5707963267948966, 0.5, 0.6, 0.7, 0.8};
  msg.a = {100.0, 200.0, 300.0, 400.0, 500.0, 600.0};
  msg.theta = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  msg.d = {7.0, 8.0, 9.0, 10.0, 11.0, 12.0};
  msg.euler_angle = 1;
  msg.mounting_angle = -0.5;

  const tl::RobotDHParam dh = tl_driver::convert::to_dh_param(msg);
  for (int i = 0; i < 6; ++i)
  {
    EXPECT_DOUBLE_EQ(dh.alpha[i], msg.alpha[i]) << "alpha " << i;
    EXPECT_DOUBLE_EQ(dh.a[i], msg.a[i]) << "a " << i;
    EXPECT_DOUBLE_EQ(dh.theta[i], msg.theta[i]) << "theta " << i;
    EXPECT_DOUBLE_EQ(dh.d[i], msg.d[i]) << "d " << i;
  }
  EXPECT_EQ(dh.eulerAngle, 1);
  EXPECT_DOUBLE_EQ(dh.mountingAngle, -0.5);

  RobotDHParam back;
  tl_driver::convert::from_dh_param(dh, back);
  EXPECT_EQ(back.alpha, msg.alpha);
  EXPECT_EQ(back.a, msg.a);
  EXPECT_EQ(back.theta, msg.theta);
  EXPECT_EQ(back.d, msg.d);
  EXPECT_EQ(back.euler_angle, 1);
  EXPECT_DOUBLE_EQ(back.mounting_angle, -0.5);
}

TEST(MsgConvert, DhParamIgnoresJointsBeyondSixth)
{
  RobotDHParam msg;
  msg.alpha = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0}; // 第 7 项应被忽略
  msg.a = {1.0, 2.0};                              // 不足 6 项时仅映射已有项
  msg.theta = {};
  msg.d = {};

  const tl::RobotDHParam dh = tl_driver::convert::to_dh_param(msg);
  EXPECT_DOUBLE_EQ(dh.alpha[5], 6.0);
  EXPECT_DOUBLE_EQ(dh.a[1], 2.0);
  EXPECT_DOUBLE_EQ(dh.a[2], 0.0); // 未提供 → 保持零初始化
}

TEST(MsgConvert, GlobalPosNameValidation)
{
  EXPECT_TRUE(tl_driver::convert::is_valid_gp_name("GP0001"));
  EXPECT_TRUE(tl_driver::convert::is_valid_gp_name("GP9999"));
  EXPECT_FALSE(tl_driver::convert::is_valid_gp_name("GP0000"));  // 0 超出 1..9999
  EXPECT_FALSE(tl_driver::convert::is_valid_gp_name("GP1"));     // 长度不足
  EXPECT_FALSE(tl_driver::convert::is_valid_gp_name("GP00001")); // 长度超出
  EXPECT_FALSE(tl_driver::convert::is_valid_gp_name("XX0001"));  // 前缀不符
  EXPECT_FALSE(tl_driver::convert::is_valid_gp_name("gp0001"));  // 大小写敏感
  EXPECT_FALSE(tl_driver::convert::is_valid_gp_name(""));
}
