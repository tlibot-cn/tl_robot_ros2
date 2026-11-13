/// @file services_motion.cpp
/// @brief 运动类服务：速度/点动/轨迹/队列/ServoJ 开关与队列运动实现（由 driver_node.cpp
/// 按域搬移，行为不变）。

#include "convert/msg_convert.hpp"
#include "convert/units.hpp"
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "arm_client.hpp"
#include "arm_sdk.hpp"
#include "node/registrars.hpp"
#include "node/driver_node.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tl_ros2_interface/srv/drag_trajectory_delete.hpp"
#include "tl_ros2_interface/srv/drag_trajectory_playback.hpp"
#include "tl_ros2_interface/srv/get_speed.hpp"
#include "tl_ros2_interface/srv/jogging.hpp"
#include "tl_ros2_interface/srv/open_servo_cst.hpp"
#include "tl_ros2_interface/srv/open_servo_j.hpp"
#include "tl_ros2_interface/srv/queue_motion_move_j.hpp"
#include "tl_ros2_interface/srv/queue_motion_set_status.hpp"
#include "tl_ros2_interface/srv/set_speed.hpp"
#include "tl_ros2_interface/srv/track_save.hpp"

namespace tl_driver
{
namespace
{
void handle_set_speed_service(DriverNode& node,
                              const std::shared_ptr<tl_ros2_interface::srv::SetSpeed::Request> request,
                              std::shared_ptr<tl_ros2_interface::srv::SetSpeed::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->speed < 0.0 || request->speed > 100.0)
  {
    response->success = false;
    response->message = "Speed out of range [0, 100]";
    return;
  }

  tl::Result ret = tl::set_speed(node.session().fd(), request->speed);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set speed successfully" : "Failed to set speed";
}

void handle_get_speed_service(DriverNode& node,
                              const std::shared_ptr<tl_ros2_interface::srv::GetSpeed::Request> request,
                              std::shared_ptr<tl_ros2_interface::srv::GetSpeed::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int speed = 0;
  const tl::Result ret = tl::get_speed(node.session().fd(), speed);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to get speed";
    return;
  }
  response->success = true;
  response->message = "Get speed successfully";
  response->speed = static_cast<double>(speed);
}

void handle_start_jogging_service(DriverNode& node,
                                  const std::shared_ptr<tl_ros2_interface::srv::Jogging::Request> request,
                                  std::shared_ptr<tl_ros2_interface::srv::Jogging::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->axis < 1 || request->axis > node.ndof())
  {
    response->success = false;
    response->message = "Invalid axis";
    return;
  }

  tl::Result ret = tl::robot_start_jogging(node.session().fd(), request->axis, request->direction);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Start jogging successfully" : "Failed to start jogging";
}

void handle_stop_jogging_service(DriverNode& node,
                                 const std::shared_ptr<tl_ros2_interface::srv::Jogging::Request> request,
                                 std::shared_ptr<tl_ros2_interface::srv::Jogging::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->axis < 1 || request->axis > node.ndof())
  {
    response->success = false;
    response->message = "Invalid axis";
    return;
  }

  tl::Result ret = tl::robot_stop_jogging(node.session().fd(), request->axis);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Stop jogging successfully" : "Failed to stop jogging";
}

void handle_track_save_service(DriverNode& node,
                               const std::shared_ptr<tl_ros2_interface::srv::TrackSave::Request> request,
                               std::shared_ptr<tl_ros2_interface::srv::TrackSave::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::track_record_save(node.session().fd(), request->traj_name);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Track save successfully" : "Failed to save track";
}

/// @brief 控制器内拖动轨迹文件名的后缀；V3 控制器/SDK 的轨迹文件为 `.txt`
///        （SDK 虽兼容旧 `.json`，本驱动只接受 `.txt`）。
constexpr char kTrajectoryNameSuffix[] = ".txt";
constexpr std::size_t kTrajectoryNameSuffixLen = sizeof(kTrajectoryNameSuffix) - 1;
/// @brief 控制器内轨迹文件名的最大长度（字节）；与常见文件系统 NAME_MAX 对齐。
constexpr std::size_t kMaxTrajectoryNameLength = 255;
/// @brief 拖动轨迹回放速度范围（%）。
constexpr int kMinTrajectoryVelocity = 1;
constexpr int kMaxTrajectoryVelocity = 100;

/// @brief 校验控制器内拖动轨迹文件名。
/// @note SDK 只接受文件名（不带路径），驱动侧先挡路径分隔符、上跳与控制字符，
///       避免越权访问控制器目录或让闭源 SDK 按 C 字符串截断。
/// @param[in] name 用户输入的文件名
/// @param[out] error 失败原因
/// @return true = 合法
bool validate_drag_trajectory_name(const std::string& name, std::string& error)
{
  if (name.empty())
  {
    error = "Trajectory name is empty";
    return false;
  }
  if (name.size() > kMaxTrajectoryNameLength)
  {
    error = "Trajectory name is too long (max " + std::to_string(kMaxTrajectoryNameLength) + " bytes)";
    return false;
  }
  for (const unsigned char ch : name)
  {
    if (ch < 0x20 || ch == 0x7f)
    {
      error = "Trajectory name contains control characters";
      return false;
    }
  }
  if (name.find('/') != std::string::npos || name.find('\\') != std::string::npos ||
      name.find("..") != std::string::npos)
  {
    error = "Trajectory name must not contain path separators or '..'";
    return false;
  }
  if (name.size() <= kTrajectoryNameSuffixLen ||
      name.compare(name.size() - kTrajectoryNameSuffixLen, kTrajectoryNameSuffixLen, kTrajectoryNameSuffix) != 0)
  {
    error = std::string("Trajectory name must end with ") + kTrajectoryNameSuffix;
    return false;
  }
  return true;
}

void handle_drag_trajectory_playback_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::DragTrajectoryPlayback::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::DragTrajectoryPlayback::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }
  if (node.motion_action_active())
  {
    response->success = false;
    response->message = "A motion action is active";
    return;
  }

  std::string error;
  if (!validate_drag_trajectory_name(request->trajectory_name, error))
  {
    response->success = false;
    response->message = error;
    return;
  }
  if (request->velocity < kMinTrajectoryVelocity || request->velocity > kMaxTrajectoryVelocity)
  {
    response->success = false;
    response->message = "Velocity out of range [" + std::to_string(kMinTrajectoryVelocity) + ", " +
                        std::to_string(kMaxTrajectoryVelocity) + "]";
    return;
  }

  const tl::Result ret = tl::drag_trajectory_playback(node.session().fd(), request->trajectory_name, request->velocity);
  response->success = (ret == tl::Result::SUCCESS);
  if (response->success)
  {
    // 回放是长任务且无完成回调：登记外部运动，使紧随其后的 action goal 在宽限期内被拒绝
    node.note_external_motion();
    response->message = "Drag trajectory playback accepted";
  }
  else
  {
    response->message = std::string("Failed to play drag trajectory: ") + arm_client::result_to_string(ret);
  }
}

void handle_drag_trajectory_delete_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::DragTrajectoryDelete::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::DragTrajectoryDelete::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::string error;
  if (!validate_drag_trajectory_name(request->trajectory_name, error))
  {
    response->success = false;
    response->message = error;
    return;
  }

  const tl::Result ret = tl::drag_trajectory_delete(node.session().fd(), request->trajectory_name);
  response->success = (ret == tl::Result::SUCCESS);
  if (response->success)
  {
    response->message = "Drag trajectory deleted";
  }
  else
  {
    response->message = std::string("Failed to delete drag trajectory: ") + arm_client::result_to_string(ret);
  }
}

void handle_open_servoj_service(DriverNode& node,
                                const std::shared_ptr<tl_ros2_interface::srv::OpenServoJ::Request> request,
                                std::shared_ptr<tl_ros2_interface::srv::OpenServoJ::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }
  if (node.motion_action_active())
  {
    response->success = false;
    response->message = "A motion action is active";
    return;
  }

  // ROS 侧 rad/s、rad/s²、rad/s³；SDK 要求度制且固定 7 元素
  std::vector<double> vmax = request->vmax;
  std::vector<double> amax = request->amax;
  std::vector<double> jmax = request->jmax;
  for (auto *values : {&vmax, &amax, &jmax})
  {
    if (values->size() == 6)
    {
      values->push_back(0.0);
    }
    if (values->size() != 7)
    {
      response->success = false;
      response->message = "vmax/amax/jmax must have 6 or 7 elements";
      return;
    }
    convert::joint_rad_to_deg(*values, values->size());
  }

  tl::Result ret = tl::open_servoJ(node.session().fd_aux(), vmax, amax, jmax);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "ServoJ open successfully" : "Failed to open ServoJ";
}

void handle_close_servoj_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::close_servoJ(node.session().fd_aux());
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "ServoJ close successfully" : "Failed to close ServoJ";
}

/**
 * @brief 开启周期同步速度模式（CSV）。
 * @note CSV 与 CST / ServoJ 共用 7000 端口的 servoJ 会话，SDK 不做互斥检测，
 *       调用方必砇保证同一时刻只开启其中一种模式。
 */
void handle_open_servo_csv_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }
  if (node.motion_action_active())
  {
    response->success = false;
    response->message = "A motion action is active";
    return;
  }

  tl::Result ret = tl::open_servo_csv_mode(node.session().fd_aux());
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "CSV mode open successfully" : "Failed to open CSV mode";
}

void handle_close_servo_csv_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::close_servo_csv_mode(node.session().fd_aux());
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "CSV mode close successfully" : "Failed to close CSV mode";
}

/**
 * @brief 开启周期同步扭矩模式（CST）。
 * @note CST 与 CSV / ServoJ 共用 7000 端口的 servoJ 会话，SDK 不做互斥检测。
 *       重力补偿默认开启；请求显式 disable_gravity_compensation=true 时关闭（纯扭矩透传）。
 *       开关先于模式打开下发，开关失败则不进入 CST，避免在非预期补偿状态下运行。
 */
void handle_open_servo_cst_service(DriverNode& node,
                                   const std::shared_ptr<tl_ros2_interface::srv::OpenServoCst::Request> request,
                                   std::shared_ptr<tl_ros2_interface::srv::OpenServoCst::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }
  if (node.motion_action_active())
  {
    response->success = false;
    response->message = "A motion action is active";
    return;
  }

  const bool enable_gravity_compensation = !request->disable_gravity_compensation;
  tl::Result ret = tl::set_servo_cst_gravity_compensation_enable(node.session().fd_aux(), enable_gravity_compensation);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = std::string("Failed to set CST gravity compensation: ") + arm_client::result_to_string(ret);
    return;
  }

  ret = tl::open_servo_cst_mode(node.session().fd_aux());
  response->success = (ret == tl::Result::SUCCESS);
  if (!response->success)
  {
    // SDK 无回读接口，开关已生效但打开失败时无法回滚；在响应中披露当前开关状态，避免误判
    response->message = std::string("Failed to open CST mode (gravity compensation already ") +
                        (enable_gravity_compensation ? "enabled" : "disabled") +
                        "): " + arm_client::result_to_string(ret);
    return;
  }
  response->message = enable_gravity_compensation ? "CST mode open successfully (gravity compensation enabled)"
                                                  : "CST mode open successfully (gravity compensation disabled)";
}

void handle_close_servo_cst_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::close_servo_cst_mode(node.session().fd_aux());
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "CST mode close successfully" : "Failed to close CST mode";
}

/**
 * @brief 设置 CST 模式的重力补偿开关。
 * @note 透传控制器开关，驱动不缓存状态：可在 open_servo_cst 之前或运行期间设置。
 *       开启后 set_servo_cst_torque 下发的是重力之外的附加扭矩。
 */
void handle_set_servo_cst_gravity_compensation_service(DriverNode& node,
                                                       const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                                       std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  const tl::Result ret = tl::set_servo_cst_gravity_compensation_enable(node.session().fd_aux(), request->data);
  response->success = (ret == tl::Result::SUCCESS);
  if (response->success)
  {
    response->message = request->data ? "CST gravity compensation enabled" : "CST gravity compensation disabled";
  }
  else
  {
    response->message = std::string("Failed to set CST gravity compensation: ") + arm_client::result_to_string(ret);
  }
}

void handle_queue_motion_set_status_service(
    DriverNode& node, const std::shared_ptr<tl_ros2_interface::srv::QueueMotionSetStatus::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::QueueMotionSetStatus::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  const int fd = node.session().fd();
  tl::Result ret = tl::queue_motion_set_status(fd, request->status);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set queue motion status successfully" : "Failed to set queue motion status";

  // 关闭连续运动模式后设置为示教模式
  if (!request->status)
  {
    const bool queue_status_ok = response->success;
    tl::Result mode_ret = tl::set_current_mode(fd, tl::RobotMode::TEACH);
    if (mode_ret != tl::Result::SUCCESS)
    {
      response->success = false;
      response->message = queue_status_ok ? "Queue motion status set but failed to switch to teach mode"
                                          : "Failed to set queue motion status and switch to teach mode";
    }
    else if (!node.session().power_off())
    {
      response->success = false;
      response->message = queue_status_ok ? "Queue motion status set but failed to power off"
                                          : "Failed to set queue motion status and power off";
    }
  }
}

void handle_queue_motion_movej_service(DriverNode& node,
                                       const std::shared_ptr<tl_ros2_interface::srv::QueueMotionMoveJ::Request> request,
                                       std::shared_ptr<tl_ros2_interface::srv::QueueMotionMoveJ::Response> response)
{
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }
  if (node.motion_action_active())
  {
    response->success = false;
    response->message = "A motion action is active";
    return;
  }

  tl::MoveCmd cmd = convert::to_move_cmd(request->cmd, convert::MotionKind::kJoint);

  const int fd = node.session().fd();
  tl::Result ret = tl::queue_motion_push_back_moveJ(fd, cmd);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to push back queue motion movej";
    return;
  }

  // 新 SDK 签名: queue_motion_send_to_controller(socketFd, int size, bool isContinue = false)
  //   size=0 表示发送全部队列指令；isContinue=true 继续排队不运动，false 立即执行
  // 旧写法把 bool 传给了 size 参数，导致 is_continue=true 时只发送第 1 条，已修正
  ret = tl::queue_motion_send_to_controller(fd, 0, request->is_continue);
  node.note_external_motion();
  response->success = (ret == tl::Result::SUCCESS);
  response->message =
      response->success ? "Queue motion movej execute successfully" : "Failed to execute queue motion movej";
}

void handle_queue_motion_stop_service(DriverNode& node, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!node.session().is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  bool status = false;
  const int fd = node.session().fd();
  tl::Result ret = tl::queue_motion_get_status(fd, status);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to get queue motion status";
    return;
  }
  if (!status)
  {
    response->success = false;
    response->message = "Queue motion is not enabled";
    return;
  }

  ret = tl::queue_motion_stop_not_power_off(fd);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Queue motion movej stop successfully" : "Failed to stop queue motion movej";
}

} // namespace

void register_motion_services(DriverNode& node)
{
  node.create_service<tl_ros2_interface::srv::SetSpeed>(
      "/tl_driver/set_speed",
      std::bind(&handle_set_speed_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::GetSpeed>(
      "/tl_driver/get_speed",
      std::bind(&handle_get_speed_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::Jogging>(
      "/tl_driver/start_jogging",
      std::bind(&handle_start_jogging_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::Jogging>(
      "/tl_driver/stop_jogging",
      std::bind(&handle_stop_jogging_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::TrackSave>(
      "/tl_driver/track_save",
      std::bind(&handle_track_save_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::DragTrajectoryPlayback>(
      "/tl_driver/drag_trajectory_playback", std::bind(&handle_drag_trajectory_playback_service, std::ref(node),
                                                       std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::DragTrajectoryDelete>(
      "/tl_driver/drag_trajectory_delete",
      std::bind(&handle_drag_trajectory_delete_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::OpenServoJ>(
      "/tl_driver/open_servoj",
      std::bind(&handle_open_servoj_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<std_srvs::srv::Trigger>(
      "/tl_driver/close_servoj",
      std::bind(&handle_close_servoj_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<std_srvs::srv::Trigger>(
      "/tl_driver/open_servo_csv",
      std::bind(&handle_open_servo_csv_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<std_srvs::srv::Trigger>(
      "/tl_driver/close_servo_csv",
      std::bind(&handle_close_servo_csv_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::OpenServoCst>(
      "/tl_driver/open_servo_cst",
      std::bind(&handle_open_servo_cst_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<std_srvs::srv::Trigger>(
      "/tl_driver/close_servo_cst",
      std::bind(&handle_close_servo_cst_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<std_srvs::srv::SetBool>("/tl_driver/set_servo_cst_gravity_compensation",
                                              std::bind(&handle_set_servo_cst_gravity_compensation_service,
                                                        std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::QueueMotionSetStatus>(
      "/tl_driver/queue_motion_set_status",
      std::bind(&handle_queue_motion_set_status_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<tl_ros2_interface::srv::QueueMotionMoveJ>(
      "/tl_driver/queue_motion_movej",
      std::bind(&handle_queue_motion_movej_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
  node.create_service<std_srvs::srv::Trigger>(
      "/tl_driver/queue_motion_stop",
      std::bind(&handle_queue_motion_stop_service, std::ref(node), std::placeholders::_1, std::placeholders::_2));
}
} // namespace tl_driver
