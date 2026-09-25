#ifndef TL_DRIVER__ARM_SDK_HPP_
#define TL_DRIVER__ARM_SDK_HPP_

/// @file arm_sdk.hpp
/// @brief 供应商 SDK（libtl_host.so）头文件的唯一入口，并提供 SDK 返回值到文本的映射。
/// @note SDK 符号一律以 `tl::` 显式限定，禁止 `using namespace tl;`
///       ——其 unscoped enum 成员（SUCCESS/TIMEOUT 等）会与其它头或全局符号重名。
/// @note SDK 返回值的接收类型一律使用其自身类型（如 `tl::Result`），不用 `int`。

#include "tl_interface.h"
#include "tl_io.h"
#include "tl_job_operate.h"
#include "tl_modbus.h"
#include "tl_queue_operate.h"
#include "tl_servo_ext.h"
#include "tl_track.h"
#include "tl_types.h"

namespace tl_driver
{
/// @brief SDK 返回值转文本，用于日志诊断。
/// @param[in] ret SDK 返回值
/// @return 静态文本，未识别的值返回 "UNKNOWN_ERROR"
inline const char *result_to_string(tl::Result ret) noexcept
{
  switch (ret)
  {
    case tl::Result::SUCCESS:
      return "SUCCESS";
    case tl::Result::RECEIVE_FAILED:
      return "RECEIVE_FAILED";
    case tl::Result::DISCONNECT:
      return "DISCONNECT";
    case tl::Result::PARAM_ERR:
      return "PARAM_ERR";
    case tl::Result::OPERATION_NOT_ALLOWED:
      return "OPERATION_NOT_ALLOWED";
    case tl::Result::EXCEPTION:
      return "EXCEPTION";
    case tl::Result::TIMEOUT:
      return "TIMEOUT";
    default:
      return "UNKNOWN_ERROR";
  }
}
} // namespace tl_driver

#endif // TL_DRIVER__ARM_SDK_HPP_
