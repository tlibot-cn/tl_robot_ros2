/// @file state_frame.cpp
/// @brief 连续状态帧 JSON 解析实现。

#include "convert/state_frame.hpp"

#include <exception>
#include <utility>

#include <nlohmann/json.hpp>

namespace tl_driver::convert
{
namespace
{
using Json = nlohmann::json;

/// @brief 在顶层及已知包装层中查找字段。
const Json *find_field(const Json& root, const char *key)
{
  if (!root.is_object())
  {
    return nullptr;
  }

  const auto direct = root.find(key);
  if (direct != root.end())
  {
    return &*direct;
  }

  // 控制器不同固件版本曾出现顶层/包装层两种布局，按已知包装名逐一探测。
  for (const char *wrapper : {"replyData", "data", "response"})
  {
    const auto layer = root.find(wrapper);
    if (layer == root.end() || !layer->is_object())
    {
      continue;
    }
    const auto nested = layer->find(key);
    if (nested != layer->end())
    {
      return &*nested;
    }
  }
  return nullptr;
}

bool to_channel(const Json& value, int& channel)
{
  if (value.is_number_integer())
  {
    channel = value.get<int>();
    return true;
  }
  if (value.is_string())
  {
    const std::string text = value.get<std::string>();
    try
    {
      std::size_t consumed = 0;
      channel = std::stoi(text, &consumed);
      return consumed == text.size();
    }
    catch (const std::exception&)
    {
      return false;
    }
  }
  return false;
}

/// @brief 数组写入定长轴缓冲；控制器可能把数值序列化为字符串。
/// @param[out] out 已预填 NaN 的定长数组
/// @param[out] count 有效元素个数
/// @return 元素均为有限数值且不超过 `kMaxStateAxes` 时为 true
bool to_axes(const Json& value, std::array<double, kMaxStateAxes>& out, std::size_t& count)
{
  if (!value.is_array() || value.size() > kMaxStateAxes)
  {
    return false;
  }

  std::size_t index = 0;
  for (const auto& item : value)
  {
    if (item.is_number())
    {
      out[index++] = item.get<double>();
      continue;
    }
    if (item.is_string())
    {
      const std::string text = item.get<std::string>();
      try
      {
        std::size_t consumed = 0;
        const double parsed = std::stod(text, &consumed);
        if (consumed != text.size())
        {
          return false;
        }
        out[index++] = parsed;
        continue;
      }
      catch (const std::exception&)
      {
        return false;
      }
    }
    return false;
  }
  count = index;
  return true;
}
} // namespace

bool parse_robot_state_frame(const std::string& message, RobotStateFrameData& out)
{
  if (message.empty())
  {
    return false;
  }

  // 回调每秒可能收到 200 帧；解析失败不得抛异常越过 SDK 回调边界。
  const Json root = Json::parse(message, nullptr, false);
  if (root.is_discarded() || !root.is_object())
  {
    return false;
  }

  const Json *channel_field = find_field(root, "channel");
  const Json *acs_field = find_field(root, "realPosACS");
  const Json *mcs_field = find_field(root, "realPosMCS");
  if (channel_field == nullptr || (acs_field == nullptr && mcs_field == nullptr))
  {
    return false;
  }

  RobotStateFrameData parsed;
  if (!to_channel(*channel_field, parsed.channel) || parsed.channel < 1 || parsed.channel > 9)
  {
    return false;
  }
  if (acs_field != nullptr && !to_axes(*acs_field, parsed.acs, parsed.acs_count))
  {
    return false;
  }
  if (mcs_field != nullptr && !to_axes(*mcs_field, parsed.mcs, parsed.mcs_count))
  {
    return false;
  }
  // 通道与载荷必须一致：channel=1 只承载 ACS，channel=2 只承载 MCS。
  // 不一致的帧若被接受，会让看门狗误判“通道健康”，对应话题却永远无数据。
  if (parsed.channel == 1)
  {
    if (parsed.acs_count == 0)
    {
      return false;
    }
    parsed.mcs_count = 0;
    parsed.mcs = invalid_axes();
  }
  else if (parsed.channel == 2)
  {
    if (parsed.mcs_count == 0)
    {
      return false;
    }
    parsed.acs_count = 0;
    parsed.acs = invalid_axes();
  }
  else if (parsed.acs_count == 0 && parsed.mcs_count == 0)
  {
    return false;
  }

  out = std::move(parsed);
  return true;
}
} // namespace tl_driver::convert
