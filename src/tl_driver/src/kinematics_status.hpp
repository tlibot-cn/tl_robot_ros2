#ifndef TL_DRIVER__KINEMATICS_STATUS_HPP_
#define TL_DRIVER__KINEMATICS_STATUS_HPP_

/// @file kinematics_status.hpp
/// @brief 本地解算状态的文本映射，与 RobotModel 实例无关。

#include "tl_robot_model.h"

namespace tl_driver
{
/// @brief KinematicsStatus → 可读字符串（失败信息用）。
const char *status_text(tl::KinematicsStatus status) noexcept;

} // namespace tl_driver

#endif // TL_DRIVER__KINEMATICS_STATUS_HPP_
