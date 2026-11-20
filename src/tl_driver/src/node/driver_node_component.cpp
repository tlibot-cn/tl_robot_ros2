/// @file driver_node_component.cpp
/// @brief 组件注册：把 DriverNode 导出为可由组件容器动态加载的节点。
/// @note 插件名 `tl_driver::DriverNode` 须与 CMakeLists.txt 中
///       `rclcpp_components_register_nodes` 的实参一致。
/// @note 该宏要求节点类提供 `explicit 类名(const rclcpp::NodeOptions&)` 构造，见 DriverNode。
/// @note 加载到同一容器内的两个 DriverNode 会争用进程内唯一的 ArmClient（SDK 回调无 userdata），
///       第二个实例构造失败；需要多台臂请各占一个容器进程。

#include <rclcpp_components/register_node_macro.hpp>

#include "node/driver_node.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(tl_driver::DriverNode)
