/// @file msg_convert.cpp
/// @brief ROS↔SDK 结构转换的实现（与拆分前各 handler 内的内联填充代码等价）。

#include "convert/msg_convert.hpp"

#include "convert/units.hpp"

namespace tl_driver::convert
{
tl::MoveCmd to_move_cmd(const tl_ros2_interface::msg::MoveCommand& msg, MotionKind kind)
{
  tl::MoveCmd cmd{};
  cmd.targetPosType = static_cast<tl::PosType>(tl::PosType::data);
  cmd.targetPosName = "";
  cmd.coord = static_cast<tl::Coord>(msg.coord);
  // 关节运动速度为百分比、加减速度为百分比，不换算；直线运动速度 m/s → mm/s。
  cmd.velocity = (kind == MotionKind::kLinear) ? m_s_to_mm_s(msg.velocity) : msg.velocity;
  cmd.velocitySync = (kind == MotionKind::kLinear) ? m_s_to_mm_s(msg.velocity_sync) : msg.velocity_sync;
  cmd.acc = msg.acc;
  cmd.dec = msg.dec;
  cmd.pl = msg.pl;
  // 以下字段在 ROS 接口中已裁剪，按控制器默认语义硬编码：
  // time=0（无提前执行）、posidtype=0（一级变量）、configuration=0（不指定形态）、spin=0（非 MOVCA）。
  cmd.time = 0;
  cmd.toolNum = msg.tool_num;
  cmd.userNum = msg.user_num;
  cmd.posidtype = 0;
  cmd.configuration = 0;
  cmd.spin = 0;
  cmd.parasync = msg.para_sync;

  cmd.targetPosValue.assign(msg.target_pos_value.begin(), msg.target_pos_value.end());
  if (msg.coord == 0)
  {
    // 关节坐标：前 7 位为 J1..J7（ROS rad → SDK 度）
    joint_rad_to_deg(cmd.targetPosValue, 7);
  }
  else
  {
    // 直角 / 工具 / 用户坐标：位置 m → mm；姿态本身为 rad，无需换算
    for (std::size_t i = 0; i < 3 && i < cmd.targetPosValue.size(); ++i)
    {
      cmd.targetPosValue[i] = m_to_mm(cmd.targetPosValue[i]);
    }
  }
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
  // ROS: alpha/theta/mounting_angle 为 rad，a/d 为 m；SDK: deg / mm
  tl::RobotDHParam dh{};
  for (int i = 0; i < 6 && i < static_cast<int>(msg.alpha.size()); ++i)
  {
    dh.alpha[i] = rad_to_deg(msg.alpha[i]);
  }
  for (int i = 0; i < 6 && i < static_cast<int>(msg.a.size()); ++i)
  {
    dh.a[i] = m_to_mm(msg.a[i]);
  }
  for (int i = 0; i < 6 && i < static_cast<int>(msg.theta.size()); ++i)
  {
    dh.theta[i] = rad_to_deg(msg.theta[i]);
  }
  for (int i = 0; i < 6 && i < static_cast<int>(msg.d.size()); ++i)
  {
    dh.d[i] = m_to_mm(msg.d[i]);
  }
  dh.eulerAngle = msg.euler_angle;
  dh.mountingAngle = rad_to_deg(msg.mounting_angle);
  return dh;
}

void from_dh_param(const tl::RobotDHParam& dh, tl_ros2_interface::msg::RobotDHParam& msg)
{
  // SDK: deg / mm → ROS: rad / m
  msg.alpha.assign(dh.alpha, dh.alpha + 6);
  msg.a.assign(dh.a, dh.a + 6);
  msg.theta.assign(dh.theta, dh.theta + 6);
  msg.d.assign(dh.d, dh.d + 6);
  for (auto& value : msg.alpha)
  {
    value = deg_to_rad(value);
  }
  for (auto& value : msg.theta)
  {
    value = deg_to_rad(value);
  }
  for (auto& value : msg.a)
  {
    value = mm_to_m(value);
  }
  for (auto& value : msg.d)
  {
    value = mm_to_m(value);
  }
  msg.euler_angle = dh.eulerAngle;
  msg.mounting_angle = deg_to_rad(dh.mountingAngle);
}
} // namespace tl_driver::convert
