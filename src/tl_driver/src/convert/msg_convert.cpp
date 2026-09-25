#include <cstddef>

#include "convert/msg_convert.hpp"

/**
 * @brief ROS 消息与 SDK 协议结构之间的纯转换实现。
 */

namespace tl_driver::convert
{

tl::MoveCmd to_move_cmd(const tl_ros2_interface::msg::MoveCommand& msg)
{
  tl::MoveCmd cmd{};
  cmd.targetPosType = static_cast<tl::PosType>(tl::PosType::data);
  cmd.targetPosName = "";
  cmd.coord = static_cast<tl::Coord>(msg.coord);
  cmd.velocity = msg.velocity;
  cmd.velocitySync = msg.velocity_sync;
  cmd.acc = msg.acc;
  cmd.dec = msg.dec;
  cmd.pl = msg.pl;
  cmd.time = msg.time;
  cmd.toolNum = msg.tool_num;
  cmd.userNum = msg.user_num;
  cmd.posidtype = msg.posidtype;
  cmd.configuration = msg.configuration;
  cmd.spin = msg.spin;
  cmd.parasync = msg.para_sync;
  cmd.targetPosValue = msg.target_pos_value;
  return cmd;
}

void to_sdk_dh_param(const tl_ros2_interface::msg::RobotDHParam& param, tl::RobotDHParam& dh_param)
{
  dh_param.L1 = param.l1;
  dh_param.L2 = param.l2;
  dh_param.L3 = param.l3;
  dh_param.L4 = param.l4;
  dh_param.L5 = param.l5;
  dh_param.L6 = param.l6;
  dh_param.L7 = param.l7;
  dh_param.L8 = param.l8;
  dh_param.L9 = param.l9;
  dh_param.L10 = param.l10;
  dh_param.L11 = param.l11;
  dh_param.L12 = param.l12;
  dh_param.L13 = param.l13;
  dh_param.L14 = param.l14;
  dh_param.L15 = param.l15;
  dh_param.L16 = param.l16;
  dh_param.L17 = param.l17;
  dh_param.L18 = param.l18;
  dh_param.L19 = param.l19;
  dh_param.L20 = param.l20;

  dh_param.Couple_Coe_1_2 = param.couple_coe_1_2;
  dh_param.Couple_Coe_2_3 = param.couple_coe_2_3;
  dh_param.Couple_Coe_3_2 = param.couple_coe_3_2;
  dh_param.Couple_Coe_3_4 = param.couple_coe_3_4;
  dh_param.Couple_Coe_4_5 = param.couple_coe_4_5;
  dh_param.Couple_Coe_4_6 = param.couple_coe_4_6;
  dh_param.Couple_Coe_5_6 = param.couple_coe_5_6;

  dh_param.dynamicLimit_max = param.dynamic_limit_max;
  dh_param.dynamicLimit_min = param.dynamic_limit_max;

  dh_param.pitch = param.pitch;
  dh_param.sliding_lead_value = param.sliding_lead_value;
  dh_param.uplift_lead_value = param.uplift_lead_value;
  dh_param.spray_distance = param.spray_distance;

  dh_param.threeAxisDirection = param.three_axis_direction;
  dh_param.fiveAxisDirection = param.five_axis_direction;

  dh_param.twoAxisConversionRatio = param.two_axis_convertion_ratio;
  dh_param.threeAxisConversionRatio = param.three_axis_convertion_ratio;
  dh_param.amplificationRatio = param.amplification_ratio;

  dh_param.conversionratio_x = param.convertion_ratio_x;
  dh_param.conversionratio_y = param.convertion_ratio_y;
  dh_param.conversionratio_z = param.convertion_ratio_z;

  dh_param.conversionratio_J1 = param.convertion_ratio_j1;
  dh_param.conversionratio_J2 = param.convertion_ratio_j2;
  dh_param.conversionratio_J3 = param.convertion_ratio_j3;
  dh_param.upsideDown = param.upside_down;
  dh_param.hanyu.PC = param.pc;

  for (size_t i = 0; i < 3; i++)
  {
    dh_param.hanyu.SP[i] = (i < param.sp.size()) ? param.sp[i] : 0.0;
    dh_param.hanyu.TL[i] = (i < param.tl.size()) ? param.tl[i] : 0.0;
  }
}

void to_msg_dh_param(const tl::RobotDHParam& dh_param, tl_ros2_interface::msg::RobotDHParam& param)
{
  param.l1 = dh_param.L1;
  param.l2 = dh_param.L2;
  param.l3 = dh_param.L3;
  param.l4 = dh_param.L4;
  param.l5 = dh_param.L5;
  param.l6 = dh_param.L6;
  param.l7 = dh_param.L7;
  param.l8 = dh_param.L8;
  param.l9 = dh_param.L9;
  param.l10 = dh_param.L10;
  param.l11 = dh_param.L11;
  param.l12 = dh_param.L12;
  param.l13 = dh_param.L13;
  param.l14 = dh_param.L14;
  param.l15 = dh_param.L15;
  param.l16 = dh_param.L16;
  param.l17 = dh_param.L17;
  param.l18 = dh_param.L18;
  param.l19 = dh_param.L19;
  param.l20 = dh_param.L20;

  param.couple_coe_1_2 = dh_param.Couple_Coe_1_2;
  param.couple_coe_2_3 = dh_param.Couple_Coe_2_3;
  param.couple_coe_3_2 = dh_param.Couple_Coe_3_2;
  param.couple_coe_3_4 = dh_param.Couple_Coe_3_4;
  param.couple_coe_4_5 = dh_param.Couple_Coe_4_5;
  param.couple_coe_4_6 = dh_param.Couple_Coe_4_6;
  param.couple_coe_5_6 = dh_param.Couple_Coe_5_6;

  param.dynamic_limit_max = dh_param.dynamicLimit_max;
  param.dynamic_limit_min = dh_param.dynamicLimit_min;

  param.pitch = dh_param.pitch;
  param.sliding_lead_value = dh_param.sliding_lead_value;
  param.uplift_lead_value = dh_param.uplift_lead_value;
  param.spray_distance = dh_param.spray_distance;

  param.three_axis_direction = dh_param.threeAxisDirection;
  param.five_axis_direction = dh_param.fiveAxisDirection;

  param.two_axis_convertion_ratio = dh_param.twoAxisConversionRatio;
  param.three_axis_convertion_ratio = dh_param.threeAxisConversionRatio;
  param.amplification_ratio = dh_param.amplificationRatio;

  param.convertion_ratio_x = dh_param.conversionratio_x;
  param.convertion_ratio_y = dh_param.conversionratio_y;
  param.convertion_ratio_z = dh_param.conversionratio_z;

  param.convertion_ratio_j1 = dh_param.conversionratio_J1;
  param.convertion_ratio_j2 = dh_param.conversionratio_J2;
  param.convertion_ratio_j3 = dh_param.conversionratio_J3;

  param.upside_down = dh_param.upsideDown;
  param.pc = dh_param.hanyu.PC;
  param.sp.assign(dh_param.hanyu.SP, dh_param.hanyu.SP + 3);
  param.tl.assign(dh_param.hanyu.TL, dh_param.hanyu.TL + 3);
}

bool to_sdk_modbus_param(const tl_ros2_interface::msg::ModbusMasterParam& param,
                         tl::ModbusMasterParameter& master_param)
{
  master_param.type = param.type;
  master_param.startAddress = param.start_addr;

  if (master_param.type == "TCP")
  {
    master_param.TCP.IP = param.tcp.ip;
    master_param.TCP.port = param.tcp.port;
  }
  else if (master_param.type == "RTU")
  {
    master_param.RTU.slaveId = param.rtu.slave_id;
    master_param.RTU.port = param.rtu.port;
    master_param.RTU.baudrate = param.rtu.baudrate;
    master_param.RTU.checkBit = param.rtu.check_bit;
    master_param.RTU.dataBit = param.rtu.data_bit;
    master_param.RTU.stopBit = param.rtu.stop_bit;
  }
  else
  {
    return false;
  }

  return true;
}

} // namespace tl_driver::convert
