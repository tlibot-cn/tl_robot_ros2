#ifndef TL_DRIVER__ARM_SDK_HPP_
#define TL_DRIVER__ARM_SDK_HPP_

/// @file arm_sdk.hpp
/// @brief SDK（libtl_host.so）头文件的唯一入口。
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

#endif // TL_DRIVER__ARM_SDK_HPP_
