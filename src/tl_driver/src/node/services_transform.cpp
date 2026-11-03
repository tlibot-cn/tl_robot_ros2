/// @file services_transform.cpp
/// @brief 坐标系服务：坐标系变换、当前坐标系读写与正逆运动学解算的实现（由 driver_node.cpp 按域搬移）。

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "arm_client.hpp"
#include "arm_sdk.hpp"
#include "convert/units.hpp"
#include "node/registrars.hpp"
#include "node/driver_node.hpp"

#include "tl_ros2_interface/srv/coord_transform.hpp"
#include "tl_ros2_interface/srv/fk.hpp"
#include "tl_ros2_interface/srv/get_current_coord.hpp"
#include "tl_ros2_interface/srv/ik.hpp"
#include "tl_ros2_interface/srv/set_current_coord.hpp"

namespace tl_driver
{
namespace
{
constexpr int kMinPoseFrame = static_cast<int>(tl::Coord::BASE);
constexpr int kMaxPoseFrame = static_cast<int>(tl::Coord::USER);
constexpr std::size_t kMaxJoints = 7;
// 求解来源：0=控制器（未设置时取 0），1=本地模型。
constexpr int kSourceController = 0;
constexpr int kSourceLocal = 1;

bool is_valid_pose_frame(int frame)
{
  return frame >= kMinPoseFrame && frame <= kMaxPoseFrame;
}

bool is_valid_source(int source)
{
  return source == kSourceController || source == kSourceLocal;
}

/// @brief 关节向量长度校验：只允许 6/7 元素，且不得少于本体轴数（7 轴机不接受 6 元素）。
std::string joint_count_error(const DriverNode& node, std::size_t count, const char *field)
{
  if (count != 6 && count != 7)
  {
    return std::string(field) + " must contain 6 or 7 elements";
  }
  if (count < static_cast<std::size_t>(node.ndof()))
  {
    return std::string(field) + " must contain at least " + std::to_string(node.ndof()) + " elements";
  }
  return {};
}

/// @brief 本地后端可用性与坐标系校验；返回空串表示通过。
std::string local_request_error(const DriverNode& node, int frame)
{
  const LocalKinematics& local = node.local_kinematics();
  if (!local.available())
  {
    return std::string("Local kinematics is unavailable: ") + local.error();
  }
  if (frame != static_cast<int>(tl::Coord::BASE))
  {
    return "Local kinematics only supports frame 1 (base)";
  }
  return {};
}

const char *pose_frame_id(int frame)
{
  switch (frame)
  {
    case static_cast<int>(tl::Coord::BASE):
      return "base_link";
    case static_cast<int>(tl::Coord::TOOL):
      return "tool0";
    case static_cast<int>(tl::Coord::USER):
      return "user_coord";
    default:
      return "";
  }
}

void fill_invalid_joints(std::array<double, kMaxJoints>& joints)
{
  joints.fill(std::numeric_limits<double>::quiet_NaN());
}

void handle_set_current_coord_service(DriverNode& node,
                                      const std::shared_ptr<tl_ros2_interface::srv::SetCurrentCoord::Request> request,
                                      std::shared_ptr<tl_ros2_interface::srv::SetCurrentCoord::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::set_current_coord(node.session().fd(), static_cast<tl::Coord>(request->coord));
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set current coordinate successfully" : "Failed to set current coordinate";
}

void handle_get_current_coord_service(DriverNode& node,
                                      const std::shared_ptr<tl_ros2_interface::srv::GetCurrentCoord::Request> request,
                                      std::shared_ptr<tl_ros2_interface::srv::GetCurrentCoord::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Coord coord = tl::Coord::JOINT;
  tl::Result ret = tl::get_current_coord(node.session().fd(), coord);
  if (ret == tl::Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get current coordinate successfully";
    response->coord = static_cast<int>(coord);
  }
  else
  {
    response->success = false;
    response->message = "Failed to get current coordinate";
  }
}

void handle_coord_transform_service(DriverNode& node,
                                    const std::shared_ptr<tl_ros2_interface::srv::CoordTransform::Request> request,
                                    std::shared_ptr<tl_ros2_interface::srv::CoordTransform::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->origin_coord < 0 || request->origin_coord > 3)
  {
    response->success = false;
    response->message = "Invalid origin coordinate";
    return;
  }

  if (request->target_coord < 0 || request->target_coord > 3)
  {
    response->success = false;
    response->message = "Invalid target coordinate";
    return;
  }

  // ROS: 关节 rad / 笛卡尔 m + rad；SDK: 关节度 / 笛卡尔 mm + rad
  std::vector<double> originPos = request->origin_pos;
  if (request->origin_coord == 0)
  {
    convert::joint_rad_to_deg(originPos, 7);
  }
  else
  {
    convert::cartesian_m_to_mm(originPos);
  }

  // reference_pos 为逆解参考关节角（SDK 度为制；空向量保持为空）
  std::vector<double> referencePos = request->reference_pos;
  convert::joint_rad_to_deg(referencePos, referencePos.size());

  std::vector<double> targetPos;
  tl::Result ret = tl::get_origin_coord_to_target_coord(
      node.session().fd(), static_cast<tl::Coord>(request->origin_coord), originPos,
      static_cast<tl::Coord>(request->target_coord), targetPos, request->form, referencePos);

  if (ret == tl::Result::SUCCESS)
  {
    if (request->target_coord == 0)
    {
      convert::joint_deg_to_rad(targetPos, targetPos.size());
    }
    else
    {
      convert::cartesian_mm_to_m(targetPos);
    }
  }
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Coord transform successfully" : "Failed to transform coord";
  response->target_pos = targetPos;
}

void handle_fk_service(DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::Fk::Request> request,
                       std::shared_ptr<tl_ros2_interface::srv::Fk::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (!is_valid_pose_frame(request->frame))
  {
    response->success = false;
    response->message = "frame must be 1 (base), 2 (tool) or 3 (user)";
    return;
  }

  if (!is_valid_source(request->source))
  {
    response->success = false;
    response->message = "source must be 0 (controller) or 1 (local model)";
    return;
  }

  const std::string joints_error = joint_count_error(node, request->joints.size(), "joints");
  if (!joints_error.empty())
  {
    response->success = false;
    response->message = joints_error;
    return;
  }

  // ROS 关节角为 rad，SDK / 本地解算均要求度制。
  std::vector<double> joints_deg = request->joints;
  convert::joint_rad_to_deg(joints_deg, joints_deg.size());

  if (request->source == kSourceLocal)
  {
    const std::string local_error = local_request_error(node, request->frame);
    if (!local_error.empty())
    {
      response->success = false;
      response->message = local_error;
      return;
    }
    const LocalKinematics& local = node.local_kinematics();

    LocalKinematics::Pose pose{};
    const tl::KinematicsStatus status = local.fk(joints_deg, pose);
    if (status != tl::KinematicsStatus::OK && status != tl::KinematicsStatus::SINGULAR)
    {
      response->success = false;
      response->message =
          std::string("Failed to compute forward kinematics (local): ") + LocalKinematics::status_text(status);
      return;
    }

    auto& out = response->pose;
    out.header.stamp = node.now();
    out.header.frame_id = pose_frame_id(request->frame);
    out.position.x = convert::mm_to_m(pose[0]);
    out.position.y = convert::mm_to_m(pose[1]);
    out.position.z = convert::mm_to_m(pose[2]);
    out.rpy.x = pose[3];
    out.rpy.y = pose[4];
    out.rpy.z = pose[5];
    out.arm_angle = 0.0;
    std::string message = "Forward kinematics computed successfully (local)";
    if (local.dof() == 7)
    {
      double arm_angle_rad = 0.0;
      if (local.arm_angle_of(joints_deg, arm_angle_rad) == tl::KinematicsStatus::OK)
      {
        out.arm_angle = arm_angle_rad;
      }
      else
      {
        // ψ 在奇异/退化位姿处无定义，用 NaN 明确表示不可用，避免伪造 0。
        out.arm_angle = std::numeric_limits<double>::quiet_NaN();
        message += "; arm angle undefined";
      }
    }
    if (status == tl::KinematicsStatus::SINGULAR)
    {
      message += " (wrist singular)";
    }
    response->success = true;
    response->message = message;
    return;
  }

  std::vector<double> pose;
  const tl::Result ret =
      tl::get_forward_kinematics(node.session().fd(), static_cast<tl::Coord>(request->frame), joints_deg, pose);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = std::string("Failed to compute forward kinematics: ") + arm_client::result_to_string(ret);
    return;
  }

  if (pose.size() < 6)
  {
    response->success = false;
    response->message = "SDK returned malformed pose";
    return;
  }

  // SDK 输出 [x, y, z] mm + [rx, ry, rz] rad；7 轴时第 7 位为冗余臂角 ψ（度），6 轴不回填。
  auto& out = response->pose;
  out.header.stamp = node.now();
  out.header.frame_id = pose_frame_id(request->frame);
  out.position.x = convert::mm_to_m(pose[0]);
  out.position.y = convert::mm_to_m(pose[1]);
  out.position.z = convert::mm_to_m(pose[2]);
  out.rpy.x = pose[3];
  out.rpy.y = pose[4];
  out.rpy.z = pose[5];
  out.arm_angle = (node.ndof() == 7 && pose.size() > 6) ? convert::deg_to_rad(pose[6]) : 0.0;

  response->success = true;
  response->message = "Forward kinematics computed successfully";
}

void handle_ik_service(DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::Ik::Request> request,
                       std::shared_ptr<tl_ros2_interface::srv::Ik::Response> response)
{
  // 失败路径一律返回 NaN，避免消费方把默认 0 当成有效关节角。
  fill_invalid_joints(response->joints);

  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (!is_valid_pose_frame(request->frame))
  {
    response->success = false;
    response->message = "frame must be 1 (base), 2 (tool) or 3 (user)";
    return;
  }

  if (!is_valid_source(request->source))
  {
    response->success = false;
    response->message = "source must be 0 (controller) or 1 (local model)";
    return;
  }

  if (request->reference_pos.empty())
  {
    response->success = false;
    response->message = "reference_pos is required";
    return;
  }

  const std::string reference_error = joint_count_error(node, request->reference_pos.size(), "reference_pos");
  if (!reference_error.empty())
  {
    response->success = false;
    response->message = reference_error;
    return;
  }

  std::vector<double> reference_pos = request->reference_pos;
  convert::joint_rad_to_deg(reference_pos, reference_pos.size());

  // SDK / 本地模型位姿均为 mm + rad；ROS 位姿为 m + rad，只换算前 3 个位置元素。
  std::vector<double> pose = {
      request->pose.position.x, request->pose.position.y, request->pose.position.z,
      request->pose.rpy.x,      request->pose.rpy.y,      request->pose.rpy.z,
  };
  convert::cartesian_m_to_mm(pose);

  if (request->source == kSourceLocal)
  {
    const std::string local_error = local_request_error(node, request->frame);
    if (!local_error.empty())
    {
      response->success = false;
      response->message = local_error;
      return;
    }
    const LocalKinematics& local = node.local_kinematics();

    const LocalKinematics::Pose local_pose{pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]};
    std::array<double, kMaxJoints> local_joints_deg{};
    int axes = 0;
    // 7 轴把 arm_angle 当 ψ 硬约束；6 轴由封装内部忽略，避免 SDK 返回 PARAM_ERR。
    const tl::KinematicsStatus status =
        local.ik(local_pose, reference_pos, request->form, request->pose.arm_angle, local_joints_deg, axes);
    if (status != tl::KinematicsStatus::OK && status != tl::KinematicsStatus::SINGULAR)
    {
      response->success = false;
      response->message =
          std::string("Failed to compute inverse kinematics (local): ") + LocalKinematics::status_text(status);
      return;
    }

    for (int i = 0; i < axes; ++i)
    {
      response->joints[i] = convert::deg_to_rad(local_joints_deg[i]);
    }
    // 与控制器输出保持一致：6 轴第 7 位为外部轴 0，而不是 NaN。
    if (axes == 6)
    {
      response->joints[6] = 0.0;
    }
    response->success = true;
    response->message = "Inverse kinematics computed successfully (local)";
    if (status == tl::KinematicsStatus::SINGULAR)
    {
      response->message += " (wrist singular)";
    }
    return;
  }

  // 控制器路径：7 轴把 arm_angle 作为 ψ 写入 SDK 第 7 位；6 轴无臂角，固定传 0。
  pose.push_back(node.ndof() == 7 ? convert::rad_to_deg(request->pose.arm_angle) : 0.0);

  std::vector<double> joints_deg;
  const tl::Result ret = tl::get_inverse_kinematics(node.session().fd(), static_cast<tl::Coord>(request->frame), pose,
                                                    joints_deg, request->form, reference_pos);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = std::string("Failed to compute inverse kinematics: ") + arm_client::result_to_string(ret);
    return;
  }

  // SDK 输出度制；定长 7 元素，缺失元素保持 NaN。
  const std::size_t count = std::min(joints_deg.size(), kMaxJoints);
  for (std::size_t i = 0; i < count; ++i)
  {
    response->joints[i] = convert::deg_to_rad(joints_deg[i]);
  }

  response->success = true;
  response->message = "Inverse kinematics computed successfully";
}

} // namespace

void register_transform_services(DriverNode& node)
{
  node.create_service<tl_ros2_interface::srv::CoordTransform>(
      "/tl_driver/coord_transform",
      std::bind(&handle_coord_transform_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::Fk>(
      "/tl_driver/fk", std::bind(&handle_fk_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::Ik>(
      "/tl_driver/ik", std::bind(&handle_ik_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::SetCurrentCoord>(
      "/tl_driver/set_current_coord",
      std::bind(&handle_set_current_coord_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetCurrentCoord>(
      "/tl_driver/get_current_coord",
      std::bind(&handle_get_current_coord_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
}
} // namespace tl_driver
