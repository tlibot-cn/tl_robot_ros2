#ifndef TL_DRIVER__CONVERT__STATE_FRAME_HPP_
#define TL_DRIVER__CONVERT__STATE_FRAME_HPP_

/// @file state_frame.hpp
/// @brief 7000 端口连续状态帧的 JSON 解析（纯逻辑，不依赖 SDK/ROS）。

#include <string>
#include <vector>

namespace tl_driver::convert
{
/// @brief 一帧机器人状态中驱动关心的字段。
/// @note 保留控制器原始单位：`acs` 为度；`mcs` 为 [mm, mm, mm, 度, 度, 度, ...]。
///       单位换算在发布边界完成，解析层不做任何假设。
struct RobotStateFrameData
{
  int channel{-1};
  std::vector<double> acs;
  std::vector<double> mcs;
};

/// @brief 解析一帧连续状态报文。
/// @param[in] message 回调原文
/// @param[out] out 解析结果（成功时写入）
/// @return 报文为合法 JSON、`channel` 在 [1, 9] 且至少包含 `realPosACS`/`realPosMCS` 之一时为 true
/// @note 兼容字段位于顶层或 `replyData`/`data`/`response` 嵌套层；数值允许为字符串。
bool parse_robot_state_frame(const std::string& message, RobotStateFrameData& out);

} // namespace tl_driver::convert

#endif // TL_DRIVER__CONVERT__STATE_FRAME_HPP_
