/// @file msg_convert.cpp
/// @brief ROS↔SDK 结构转换的实现（与拆分前各 handler 内的内联填充代码等价）。

#include "convert/msg_convert.hpp"

namespace tl_driver::convert
{
bool is_valid_gp_name(const std::string& pos_name)
{
  if (pos_name.length() != 6 || pos_name.substr(0, 2) != "GP")
  {
    return false;
  }
  const int num = std::stoi(pos_name.substr(2));
  return (num >= 1 && num <= 9999);
}

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

bool to_modbus_master_param(const tl_ros2_interface::msg::ModbusMasterParam& msg, tl::ModbusMasterParameter& out)
{
  out.type = msg.type;
  // 兼容大小写：统一转大写后判断（SDK 期望 "TCP"/"RTU"）
  for (auto& c : out.type)
  {
    if (c >= 'a' && c <= 'z')
    {
      c -= 32;
    }
  }
  out.startAddress = msg.start_addr;

  if (out.type == "TCP")
  {
    out.TCP.IP = msg.tcp.ip;
    out.TCP.port = msg.tcp.port;
  }
  else if (out.type == "RTU")
  {
    out.RTU.slaveId = msg.rtu.slave_id;
    out.RTU.port = msg.rtu.port;
    out.RTU.baudrate = msg.rtu.baudrate;
    out.RTU.checkBit = msg.rtu.check_bit;
    out.RTU.dataBit = msg.rtu.data_bit;
    out.RTU.stopBit = msg.rtu.stop_bit;
  }
  else
  {
    return false;
  }

  return true;
}

tl::RobotDHParam to_dh_param(const tl_ros2_interface::msg::RobotDHParam& msg)
{
  // 新 SDK: 标准DH参数 alpha/a/theta/d，逐字段映射到 SDK 的 tl::RobotDHParam
  tl::RobotDHParam dh{};
  for (int i = 0; i < 6 && i < static_cast<int>(msg.alpha.size()); ++i)
  {
    dh.alpha[i] = msg.alpha[i];
  }
  for (int i = 0; i < 6 && i < static_cast<int>(msg.a.size()); ++i)
  {
    dh.a[i] = msg.a[i];
  }
  for (int i = 0; i < 6 && i < static_cast<int>(msg.theta.size()); ++i)
  {
    dh.theta[i] = msg.theta[i];
  }
  for (int i = 0; i < 6 && i < static_cast<int>(msg.d.size()); ++i)
  {
    dh.d[i] = msg.d[i];
  }
  dh.eulerAngle = msg.euler_angle;
  dh.mountingAngle = msg.mounting_angle;
  return dh;
}

void from_dh_param(const tl::RobotDHParam& dh, tl_ros2_interface::msg::RobotDHParam& msg)
{
  // 新 SDK: 标准DH参数 alpha/a/theta/d，逐字段映射到 ROS2 消息
  msg.alpha.assign(dh.alpha, dh.alpha + 6);
  msg.a.assign(dh.a, dh.a + 6);
  msg.theta.assign(dh.theta, dh.theta + 6);
  msg.d.assign(dh.d, dh.d + 6);
  msg.euler_angle = dh.eulerAngle;
  msg.mounting_angle = dh.mountingAngle;
}
} // namespace tl_driver::convert
