/// @file kinematics_status.cpp
/// @brief 本地解算状态的纯文本映射。
/// @note 与 kinematics_local.cpp 分开编译：本文件只依赖 SDK 枚举，可被单元测试直接链接。

#include "kinematics_status.hpp"

namespace tl_driver
{
const char *status_text(tl::KinematicsStatus status) noexcept
{
  switch (status)
  {
    case tl::KinematicsStatus::OK:
      return "OK";
    case tl::KinematicsStatus::SINGULAR_WRIST:
      return "SINGULAR_WRIST";
    case tl::KinematicsStatus::SINGULAR_ELBOW:
      return "SINGULAR_ELBOW";
    case tl::KinematicsStatus::SINGULAR_SHOULDER:
      return "SINGULAR_SHOULDER";
    case tl::KinematicsStatus::SINGULAR_ARM_ANGLE:
      return "SINGULAR_ARM_ANGLE";
    case tl::KinematicsStatus::BRANCH_SWITCHED:
      return "BRANCH_SWITCHED";
    case tl::KinematicsStatus::UNREACHABLE:
      return "UNREACHABLE";
    case tl::KinematicsStatus::NO_SOLUTION:
      return "NO_SOLUTION";
    case tl::KinematicsStatus::LIMIT_EXCEEDED:
      return "LIMIT_EXCEEDED";
    case tl::KinematicsStatus::NOT_CONVERGED:
      return "NOT_CONVERGED";
    case tl::KinematicsStatus::PARAM_ERR:
      return "PARAM_ERR";
    default:
      return "UNKNOWN";
  }
}

} // namespace tl_driver
