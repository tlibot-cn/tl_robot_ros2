#ifndef TL_DRIVER__MSG_CONVERT_HPP_
#define TL_DRIVER__MSG_CONVERT_HPP_

/// @file msg_convert.hpp
/// @brief ROS 消息 / 服务与 SDK 结构之间的转换（含全局点位名校验）。
/// @note SDK 类型（`tl::`）经 arm_sdk.hpp 引入；本模块只做字段搬运，不发 SDK 请求。

#include <string>

#include "tl_ros2_interface/msg/modbus_master_param.hpp"
#include "tl_ros2_interface/msg/move_command.hpp"
#include "tl_ros2_interface/msg/robot_dh_param.hpp"

// SDK 头最后引入：tl_interface.h 会 #undef ERROR/DEBUG 等宏
#include "arm_sdk.hpp"

namespace tl_driver::convert
{
/// @brief 运动指令的速度字段语义：关节运动为百分比，直线类运动为线速度。
enum class MotionKind
{
  kJoint, ///< robot_movej / queue_motion_push_back_moveJ：velocity 为 %
  kLinear ///< robot_movel / moveC / imove：velocity 为 m/s（SDK 侧 mm/s）
};

/// @brief 校验全局点位名：长度 6、前缀 "GP"、编号 1..9999。
/// @param[in] pos_name 待校验点位名
/// @return 通过校验为 true
/// @note 当前实现沿用 std::stoi，含非数字尾部的输入不会被拒绝、
///       非数字首部的输入会抛 std::invalid_argument（架构评审 P0-1，修复另行进行）。
bool is_valid_gp_name(const std::string& pos_name);

/// @brief MoveCommand 消息（话题与 job/queue 服务共用）转 SDK 运动指令。
/// @param[in] msg 运动指令消息（ROS 侧 m / rad / m·s⁻¹）
/// @param[in] kind 速度字段语义（关节 % 或直线 m/s）
/// @return 填充后的 SDK 运动指令（控制器侧 mm / 度 / mm·s⁻¹）
/// @note target_pos_value 的 [0..6] 按 coord 转换；[7..13] 为外部轴点位，单位随
///       外部轴类型，保持原值透传。
tl::MoveCmd to_move_cmd(const tl_ros2_interface::msg::MoveCommand& msg, MotionKind kind);

/// @brief ModbusMasterParam 消息转 SDK 主站参数。
/// @param[in] msg 主站参数消息
/// @param[out] out SDK 主站参数，仅在返回 true 时有效
/// @return type 为 TCP / RTU 时为 true，其它取值返回 false
/// @note type 统一转大写后判断（SDK 期望 "TCP"/"RTU"）。
bool to_modbus_master_param(const tl_ros2_interface::msg::ModbusMasterParam& msg, tl::ModbusMasterParameter& out);

/// @brief RobotDHParam 服务消息转 SDK DH 参数。
/// @param[in] msg DH 参数消息，各数组仅前 6 项参与映射
/// @return 填充后的 SDK DH 参数
tl::RobotDHParam to_dh_param(const tl_ros2_interface::msg::RobotDHParam& msg);

/// @brief SDK DH 参数转 RobotDHParam 服务消息。
/// @param[in] dh SDK DH 参数
/// @param[out] msg DH 参数消息，各数组写为 6 项
void from_dh_param(const tl::RobotDHParam& dh, tl_ros2_interface::msg::RobotDHParam& msg);
} // namespace tl_driver::convert

#endif // TL_DRIVER__MSG_CONVERT_HPP_
