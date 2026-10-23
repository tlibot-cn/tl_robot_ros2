/// @file test_msg_convert.cpp
/// @brief convert/msg_convert 转换与校验的单元测试。

#include <gtest/gtest.h>

#include <cmath>

#include "convert/msg_convert.hpp"
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
  msg.tool_num = 7;
  msg.user_num = 8;
  msg.para_sync = true;
  msg.target_pos_value.fill(0.0);
  msg.target_pos_value[0] = 1.0;
  msg.target_pos_value[1] = 2.0;
  msg.target_pos_value[2] = 3.0;
  msg.target_pos_value[3] = 4.0;
  msg.target_pos_value[4] = 5.0;
  msg.target_pos_value[5] = 6.0;

  const tl::MoveCmd cmd = tl_driver::convert::to_move_cmd(msg, tl_driver::convert::MotionKind::kLinear);

  EXPECT_EQ(cmd.targetPosType, static_cast<tl::PosType>(tl::PosType::data));
  EXPECT_EQ(cmd.targetPosName, "");
  EXPECT_EQ(static_cast<int>(cmd.coord), 2);
  EXPECT_DOUBLE_EQ(cmd.velocity, 11500.0);     // m/s → mm/s
  EXPECT_DOUBLE_EQ(cmd.velocitySync, 22500.0); // m/s → mm/s
  EXPECT_DOUBLE_EQ(cmd.acc, 33.5);             // 百分比不换算
  EXPECT_DOUBLE_EQ(cmd.dec, 44.5);
  EXPECT_EQ(cmd.pl, 5);
  // 已从消息中删除的字段按控制器默认语义硬编码。
  EXPECT_EQ(cmd.time, 0);
  EXPECT_EQ(cmd.toolNum, 7);
  EXPECT_EQ(cmd.userNum, 8);
  EXPECT_EQ(cmd.posidtype, 0);
  EXPECT_EQ(cmd.configuration, 0);
  EXPECT_EQ(cmd.spin, 0);
  EXPECT_EQ(cmd.parasync, true);
  ASSERT_EQ(cmd.targetPosValue.size(), 14u);
  // 笛卡尔坐标：前 3 位 m → mm，姿态 rad 原值
  EXPECT_DOUBLE_EQ(cmd.targetPosValue[0], 1000.0);
  EXPECT_DOUBLE_EQ(cmd.targetPosValue[1], 2000.0);
  EXPECT_DOUBLE_EQ(cmd.targetPosValue[2], 3000.0);
  EXPECT_DOUBLE_EQ(cmd.targetPosValue[3], 4.0);
  EXPECT_DOUBLE_EQ(cmd.targetPosValue[4], 5.0);
  EXPECT_DOUBLE_EQ(cmd.targetPosValue[5], 6.0);
  EXPECT_DOUBLE_EQ(cmd.targetPosValue[6], 0.0);
}

TEST(MsgConvert, ToMoveCmdConvertsJointRadiansToDegrees)
{
  MoveCommand msg;
  msg.coord = 0;
  msg.velocity = 50.0;
  msg.target_pos_value.fill(0.0);
  msg.target_pos_value[0] = M_PI;
  msg.target_pos_value[1] = -M_PI / 2.0;

  const tl::MoveCmd cmd = tl_driver::convert::to_move_cmd(msg, tl_driver::convert::MotionKind::kJoint);

  EXPECT_DOUBLE_EQ(cmd.velocity, 50.0); // 关节速度为百分比，不换算
  EXPECT_NEAR(cmd.targetPosValue[0], 180.0, 1e-9);
  EXPECT_NEAR(cmd.targetPosValue[1], -90.0, 1e-9);
  EXPECT_DOUBLE_EQ(cmd.targetPosValue[2], 0.0);
}

TEST(MsgConvert, ToMoveCmdOnDefaultMessageKeepsZeroTarget)
{
  const MoveCommand msg;
  const tl::MoveCmd cmd = tl_driver::convert::to_move_cmd(msg, tl_driver::convert::MotionKind::kJoint);
  // 定长 14 维始终存在，未填写的位置为 0。
  ASSERT_EQ(cmd.targetPosValue.size(), 14u);
  EXPECT_DOUBLE_EQ(cmd.targetPosValue[0], 0.0);
  EXPECT_DOUBLE_EQ(cmd.targetPosValue[13], 0.0);
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
  constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
  for (int i = 0; i < 6; ++i)
  {
    EXPECT_NEAR(dh.alpha[i], msg.alpha[i] * kRadToDeg, 1e-9) << "alpha " << i;
    EXPECT_NEAR(dh.a[i], msg.a[i] * 1000.0, 1e-9) << "a " << i;
    EXPECT_NEAR(dh.theta[i], msg.theta[i] * kRadToDeg, 1e-9) << "theta " << i;
    EXPECT_NEAR(dh.d[i], msg.d[i] * 1000.0, 1e-9) << "d " << i;
  }
  EXPECT_EQ(dh.eulerAngle, 1);
  EXPECT_NEAR(dh.mountingAngle, -0.5 * kRadToDeg, 1e-9);

  RobotDHParam back;
  tl_driver::convert::from_dh_param(dh, back);
  for (int i = 0; i < 6; ++i)
  {
    EXPECT_NEAR(back.alpha[i], msg.alpha[i], 1e-12);
    EXPECT_NEAR(back.a[i], msg.a[i], 1e-12);
    EXPECT_NEAR(back.theta[i], msg.theta[i], 1e-12);
    EXPECT_NEAR(back.d[i], msg.d[i], 1e-12);
  }
  EXPECT_EQ(back.euler_angle, 1);
  EXPECT_NEAR(back.mounting_angle, -0.5, 1e-12);
}

TEST(MsgConvert, DhParamIgnoresJointsBeyondSixth)
{
  RobotDHParam msg;
  msg.alpha = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0}; // 第 7 项应被忽略
  msg.a = {1.0, 2.0};                              // 不足 6 项时仅映射已有项
  msg.theta = {};
  msg.d = {};

  const tl::RobotDHParam dh = tl_driver::convert::to_dh_param(msg);
  EXPECT_NEAR(dh.alpha[5], 6.0 * 180.0 / 3.14159265358979323846, 1e-9);
  EXPECT_NEAR(dh.a[1], 2000.0, 1e-9);
  EXPECT_DOUBLE_EQ(dh.a[2], 0.0); // 未提供 → 保持零初始化
}
