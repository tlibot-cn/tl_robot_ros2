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

/// @brief 数组转 double 向量；控制器可能把数值序列化为字符串。
bool to_vector(const Json& value, std::vector<double>& out)
{
  if (!value.is_array())
  {
    return false;
  }

  out.clear();
  out.reserve(value.size());
  for (const auto& item : value)
  {
    if (item.is_number())
    {
      out.push_back(item.get<double>());
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
        out.push_back(parsed);
        continue;
      }
      catch (const std::exception&)
      {
        return false;
      }
    }
    return false;
  }
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
  if (acs_field != nullptr && !to_vector(*acs_field, parsed.acs))
  {
    return false;
  }
  if (mcs_field != nullptr && !to_vector(*mcs_field, parsed.mcs))
  {
    return false;
  }
  // 通道与载荷必须一致：channel=1 只承载 ACS，channel=2 只承载 MCS。
  // 不一致的帧若被接受，会让看门狗误判“通道健康”，对应话题却永远无数据。
  if (parsed.channel == 1)
  {
    if (parsed.acs.empty())
    {
      return false;
    }
    parsed.mcs.clear();
  }
  else if (parsed.channel == 2)
  {
    if (parsed.mcs.empty())
    {
      return false;
    }
    parsed.acs.clear();
  }
  else if (parsed.acs.empty() && parsed.mcs.empty())
  {
    return false;
  }

  out = std::move(parsed);
  return true;
}
} // namespace tl_driver::convert
