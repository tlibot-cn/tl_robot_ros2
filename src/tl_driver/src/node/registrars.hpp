#ifndef TL_DRIVER__REGISTRARS_HPP_
#define TL_DRIVER__REGISTRARS_HPP_

/// @file registrars.hpp
/// @brief 各域服务与话题注册函数的跨 TU 声明（实现分散在 services_*.cpp / topics.cpp）。

namespace tl_driver
{
class DriverNode;

/// @brief 注册运动订阅话题（moveJ/moveL/servoJ/servoL）。
/// @param[in] node 驱动节点
void register_topics(DriverNode& node);
} // namespace tl_driver

#endif // TL_DRIVER__REGISTRARS_HPP_
