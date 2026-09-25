#ifndef TL_DRIVER__CONVERT__MSG_CONVERT_HPP_
#define TL_DRIVER__CONVERT__MSG_CONVERT_HPP_

#include "tl_driver/arm_sdk.hpp"

#include "tl_ros2_interface/msg/modbus_master_param.hpp"
#include "tl_ros2_interface/msg/move_command.hpp"
#include "tl_ros2_interface/msg/robot_dh_param.hpp"

namespace tl_driver::convert
{

/**
 * @brief 由 ROS 运动指令消息构造 SDK 运动指令。
 * @param[in] msg ROS 运动指令（`/tl_driver/moveJ`、`/tl_driver/moveL`、队列与 job 指令共用）
 * @return SDK 运动指令，`targetPosValue` 取消息原值（SDK 默认构造为 14 元素）
 */
tl::MoveCmd to_move_cmd(const tl_ros2_interface::msg::MoveCommand& msg);

/**
 * @brief ROS DH 参数消息写入 SDK DH 参数结构。
 * @param[in] param ROS DH 参数
 * @param[out] dh_param SDK DH 参数，未涉及的字段保持调用方传入的初值
 */
void to_sdk_dh_param(const tl_ros2_interface::msg::RobotDHParam& param, tl::RobotDHParam& dh_param);

/**
 * @brief SDK DH 参数结构写入 ROS DH 参数消息。
 * @param[in] dh_param SDK DH 参数
 * @param[out] param ROS DH 参数
 */
void to_msg_dh_param(const tl::RobotDHParam& dh_param, tl_ros2_interface::msg::RobotDHParam& param);

/**
 * @brief ROS Modbus 主站参数消息写入 SDK 主站参数结构。
 * @param[in] param ROS Modbus 主站参数
 * @param[out] master_param SDK 主站参数
 * @return true=写入成功；false=`type` 既非 TCP 也非 RTU
 */
bool to_sdk_modbus_param(const tl_ros2_interface::msg::ModbusMasterParam& param,
                         tl::ModbusMasterParameter& master_param);

} // namespace tl_driver::convert

#endif // TL_DRIVER__CONVERT__MSG_CONVERT_HPP_
