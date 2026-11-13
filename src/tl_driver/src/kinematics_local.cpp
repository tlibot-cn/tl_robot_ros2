/// @file kinematics_local.cpp
/// @brief 本地运动学封装实现：启动构建、工具手同步与 6/7 轴分派。

#include "kinematics_local.hpp"

#include <algorithm>
#include <cstddef>

#include "arm_client.hpp"
#include "convert/units.hpp"

namespace tl_driver
{
namespace
{
/// @brief 把控制器当前生效工具手同步到本地模型。
/// @note 编号 0 视为无工具手（恒等变换）；非 0 编号读取失败则整体失败，
///       避免本地模型按法兰解算却对外声称是 TCP。
template <int DOF>
tl::Result set_tool_from_controller(tl::SOCKETFD fd, tl::RobotModel<DOF>& kin, std::string& error)
{
  int tool_num = 0;
  tl::Result ret = tl::get_tool_hand_number(fd, tool_num);
  if (ret != tl::Result::SUCCESS)
  {
    error = std::string("read active tool number failed: ") + arm_client::result_to_string(ret);
    return ret;
  }
  if (tool_num == 0)
  {
    return tl::Result::SUCCESS;
  }

  tl::ToolParam param{};
  ret = tl::get_tool_hand_param(fd, tool_num, param);
  if (ret != tl::Result::SUCCESS)
  {
    error = "read tool #" + std::to_string(tool_num) + " failed: " + arm_client::result_to_string(ret);
    return ret;
  }

  typename tl::RobotModel<DOF>::Poses tool{param.X,
                                           param.Y,
                                           param.Z,
                                           convert::deg_to_rad(param.A),
                                           convert::deg_to_rad(param.B),
                                           convert::deg_to_rad(param.C)};
  if (kin.set_tool(tool) != tl::KinematicsStatus::OK)
  {
    error = "tool #" + std::to_string(tool_num) + " transform rejected (non-finite or invalid values)";
    return tl::Result::PARAM_ERR;
  }
  return tl::Result::SUCCESS;
}
} // namespace

void LocalKinematics::build(int fd, int ndof)
{
  kin6_.reset();
  kin7_.reset();
  dof_ = 0;
  available_ = false;
  error_.clear();

  if (ndof != 6 && ndof != 7)
  {
    error_ = "local kinematics supports 6- or 7-axis robots only, got " + std::to_string(ndof);
    return;
  }

  tl::Result ret = tl::Result::SUCCESS;
  if (ndof == 6)
  {
    kin6_.emplace();
    ret = tl::make_model<6>(fd, *kin6_);
  }
  else
  {
    kin7_.emplace();
    ret = tl::make_model<7>(fd, *kin7_);
  }
  if (ret != tl::Result::SUCCESS)
  {
    kin6_.reset();
    kin7_.reset();
    error_ = std::string("make_model failed: ") + arm_client::result_to_string(ret);
    return;
  }

  std::string tool_error;
  ret = ndof == 6 ? set_tool_from_controller(fd, *kin6_, tool_error) : set_tool_from_controller(fd, *kin7_, tool_error);
  if (ret != tl::Result::SUCCESS)
  {
    kin6_.reset();
    kin7_.reset();
    error_ = tool_error;
    return;
  }

  dof_ = ndof;
  available_ = true;
}

tl::KinematicsStatus LocalKinematics::fk(const std::vector<double>& joints_deg, Pose& pose) const noexcept
{
  if (!available_ || joints_deg.size() < static_cast<std::size_t>(dof_))
  {
    return tl::KinematicsStatus::PARAM_ERR;
  }

  if (dof_ == 6)
  {
    tl::RobotModel<6>::Joints q{};
    std::copy_n(joints_deg.begin(), 6, q.begin());
    return kin6_->fk(q, pose);
  }

  tl::RobotModel<7>::Joints q{};
  std::copy_n(joints_deg.begin(), 7, q.begin());
  return kin7_->fk(q, pose);
}

tl::KinematicsStatus LocalKinematics::ik(const Pose& pose, const std::vector<double>& ref_joints_deg, int form,
                                         double arm_angle_rad, std::array<double, 7>& joints_deg,
                                         int& axes) const noexcept
{
  axes = 0;
  if (!available_ || ref_joints_deg.size() < static_cast<std::size_t>(dof_))
  {
    return tl::KinematicsStatus::PARAM_ERR;
  }

  if (dof_ == 6)
  {
    tl::RobotModel<6>::Joints q{};
    tl::RobotModel<6>::Joints ref{};
    std::copy_n(ref_joints_deg.begin(), 6, ref.begin());
    // 6 轴无冗余自由度：给有限臂角会被 SDK 判为 PARAM_ERR，必须显式传“未指定”。
    const tl::KinematicsStatus status = kin6_->ik(pose, q, ref, form, tl::kArmAngleNone);
    if (status == tl::KinematicsStatus::OK || status == tl::KinematicsStatus::SINGULAR)
    {
      std::copy_n(q.begin(), 6, joints_deg.begin());
      axes = 6;
    }
    return status;
  }

  tl::RobotModel<7>::Joints q{};
  tl::RobotModel<7>::Joints ref{};
  std::copy_n(ref_joints_deg.begin(), 7, ref.begin());
  const tl::KinematicsStatus status = kin7_->ik(pose, q, ref, form, arm_angle_rad);
  if (status == tl::KinematicsStatus::OK || status == tl::KinematicsStatus::SINGULAR)
  {
    std::copy_n(q.begin(), 7, joints_deg.begin());
    axes = 7;
  }
  return status;
}

tl::KinematicsStatus LocalKinematics::arm_angle_of(const std::vector<double>& joints_deg,
                                                   double& arm_angle_rad) const noexcept
{
  if (!available_ || dof_ != 7 || joints_deg.size() < 7)
  {
    return tl::KinematicsStatus::PARAM_ERR;
  }

  tl::RobotModel<7>::Joints q{};
  std::copy_n(joints_deg.begin(), 7, q.begin());
  return kin7_->arm_angle_of(q, arm_angle_rad);
}

const char *LocalKinematics::status_text(tl::KinematicsStatus status) noexcept
{
  switch (status)
  {
    case tl::KinematicsStatus::OK:
      return "OK";
    case tl::KinematicsStatus::SINGULAR:
      return "SINGULAR";
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
