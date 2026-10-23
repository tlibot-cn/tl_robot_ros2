#ifndef TL_DRIVER__CONVERT__STATE_FRAME_HPP_
#define TL_DRIVER__CONVERT__STATE_FRAME_HPP_

/// @file state_frame.hpp
/// @brief 7000 端口连续状态帧的 JSON 解析（纯逻辑，不依赖 SDK/ROS）。

#include <array>
#include <cstddef>
#include <limits>
#include <string>

namespace tl_driver::convert
{
/// @brief 控制器状态帧的固定轴数上限（ACS 7 轴；MCS 为 6 位姿 + 冗余臂角，共 7 维）。
constexpr std::size_t kMaxStateAxes = 7;

/// @brief 生成全 NaN 的定长轴数组（表示该字段无效/缺失）。
inline std::array<double, kMaxStateAxes> invalid_axes()
{
  std::array<double, kMaxStateAxes> values{};
  values.fill(std::numeric_limits<double>::quiet_NaN());
  return values;
}

/// @brief 一帧机器人状态中驱动关心的字段。
/// @note 保留控制器原始单位：`acs` 为度；`mcs` 为 [mm, mm, mm, 度, 度, 度, ...]。
///       单位换算在发布边界完成，解析层不做任何假设。
/// @note 数值承载在定长数组里，避免每帧动态分配；`*_count` 为本次帧的有效元素个数。
///       无帧时 count 为 0；帧中缺少该字段时 count 为 0 且数组元素为 NaN。
struct RobotStateFrameData
{
  int channel{-1};
  std::array<double, kMaxStateAxes> acs = invalid_axes();
  std::array<double, kMaxStateAxes> mcs = invalid_axes();
  std::size_t acs_count{0};
  std::size_t mcs_count{0};
};

/// @brief 解析一帧连续状态报文。
/// @param[in] message 回调原文
/// @param[out] out 解析结果（成功时写入）
/// @return 报文为合法 JSON、`channel` 在 [1, 9]、数组不超过 7 个元素，且至少包含
///         `realPosACS`/`realPosMCS` 之一时为 true
/// @note 兼容字段位于顶层或 `replyData`/`data`/`response` 嵌套层；数值允许为字符串。
bool parse_robot_state_frame(const std::string& message, RobotStateFrameData& out);

} // namespace tl_driver::convert

#endif // TL_DRIVER__CONVERT__STATE_FRAME_HPP_
