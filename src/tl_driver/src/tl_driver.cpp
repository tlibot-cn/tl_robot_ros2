#include <cmath>
#include <thread>

#include "tl_driver/tl_driver.h"
using namespace tl;

namespace
{

struct RobotStateMessageBuffer
{
  std::mutex mutex;
  std::condition_variable cv;

  int last_msg_id = -1;
  std::string last_msg;
  uint64_t seq = 0;
};

RobotStateMessageBuffer g_robot_state_msg_buffer;

// 新 SDK：7000 端口机器人状态专用回调（robot_state_callback）
void robot_state_callback_handler(const char *state)
{
  {
    std::lock_guard<std::mutex> lock(g_robot_state_msg_buffer.mutex);

    g_robot_state_msg_buffer.last_msg_id = MessageLists::ROBOT_STATE;
    g_robot_state_msg_buffer.last_msg = state ? state : "";
    ++g_robot_state_msg_buffer.seq;
  }

  g_robot_state_msg_buffer.cv.notify_all();

  std::cout << "\033[32m"
            << "robot_state = " << (state ? state : "") << "\033[0m" << std::endl;
}

void receive_error_or_warning_message_callback(int messageType, const char *message, int messageCode)
{
  std::cout << "\033[31m"
            << "messageType = " << messageType << ", message = " << message << ", messageCode = " << messageCode
            << "\033[0m" << std::endl;
}

const char *result_to_string(int ret)
{
  switch (ret)
  {
    case Result::SUCCESS:
      return "SUCCESS";
    case Result::RECEIVE_FAILED:
      return "RECEIVE_FAILED";
    case Result::DISCONNECT:
      return "DISCONNECT";
    case Result::PARAM_ERR:
      return "PARAM_ERR";
    case Result::OPERATION_NOT_ALLOWED:
      return "OPERATION_NOT_ALLOWED";
    case Result::EXCEPTION:
      return "EXCEPTION";
    case Result::TIMEOUT:
      return "TIMEOUT";
    default:
      return "UNKNOWN_ERROR";
  }
}
} // namespace


TL_Arm::TL_Arm() : rclcpp::Node("tl_driver")
{
  this->declare_parameter("arm_ip", "192.168.1.13");
  this->declare_parameter("arm_port", "6001");
  this->declare_parameter("arm_port_aux", "7000");
  this->declare_parameter<std::vector<std::string>>("arm_joints",
                                                    {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
  this->declare_parameter("arm_type", "TCB605");

  arm_ip_ = this->get_parameter("arm_ip").as_string();
  arm_port_ = this->get_parameter("arm_port").as_string();
  arm_port_aux_ = this->get_parameter("arm_port_aux").as_string();
  arm_type_ = this->get_parameter("arm_type").as_string();
  arm_joints_ = this->get_parameter("arm_joints").as_string_array();
  ndof_ = arm_joints_.size();

  // 多线程
  service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  topic_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto topic_group_option = rclcpp::SubscriptionOptions();
  topic_group_option.callback_group = topic_group_;

  timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // 服务
  connect_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/tl_driver/connect_arm",
      std::bind(&TL_Arm::handle_connect_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  disconnect_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/tl_driver/disconnect_arm",
      std::bind(&TL_Arm::handle_disconnect_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  poweron_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/tl_driver/power_on",
      std::bind(&TL_Arm::handle_poweron_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  poweroff_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/tl_driver/power_off",
      std::bind(&TL_Arm::handle_poweroff_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  clear_error_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/tl_driver/clear_error",
      std::bind(&TL_Arm::handle_clear_error_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  set_speed_service_ = this->create_service<tl_ros2_interface::srv::SetSpeed>(
      "/tl_driver/set_speed",
      std::bind(&TL_Arm::handle_set_speed_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_speed_service_ = this->create_service<tl_ros2_interface::srv::GetSpeed>(
      "/tl_driver/get_speed",
      std::bind(&TL_Arm::handle_get_speed_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_quat2rpy_service_ = this->create_service<tl_ros2_interface::srv::GetPosTransform>(
      "/tl_driver/get_quat2rpy",
      std::bind(&TL_Arm::handle_get_quat2rpy_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_rpy2quat_service_ = this->create_service<tl_ros2_interface::srv::GetPosTransform>(
      "/tl_driver/get_rpy2quat",
      std::bind(&TL_Arm::handle_get_rpy2quat_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_rpy2r_service_ = this->create_service<tl_ros2_interface::srv::GetPosTransform>(
      "/tl_driver/get_rpy2r",
      std::bind(&TL_Arm::handle_get_rpy2r_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_tr2r_service_ = this->create_service<tl_ros2_interface::srv::GetPosTransform>(
      "/tl_driver/get_tr2r",
      std::bind(&TL_Arm::handle_get_tr2r_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_r2tr_service_ = this->create_service<tl_ros2_interface::srv::GetPosTransform>(
      "/tl_driver/get_r2tr",
      std::bind(&TL_Arm::handle_get_r2tr_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  set_controller_ip_service_ = this->create_service<tl_ros2_interface::srv::SetControllerIP>(
      "/tl_driver/set_controller_ip",
      std::bind(&TL_Arm::handle_set_controller_ip_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_controller_id_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/tl_driver/get_controller_id",
      std::bind(&TL_Arm::handle_get_controller_id_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  start_jogging_service_ = this->create_service<tl_ros2_interface::srv::Jogging>(
      "/tl_driver/start_jogging",
      std::bind(&TL_Arm::handle_start_jogging_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  stop_jogging_service_ = this->create_service<tl_ros2_interface::srv::Jogging>(
      "/tl_driver/stop_jogging",
      std::bind(&TL_Arm::handle_stop_jogging_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_robot_state_service_ = this->create_service<tl_ros2_interface::srv::GetRobotState>(
      "/tl_driver/get_robot_state",
      std::bind(&TL_Arm::handle_get_robot_state_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_library_version_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/tl_driver/get_library_version",
      std::bind(&TL_Arm::handle_get_library_version_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_robot_joint_param_service_ = this->create_service<tl_ros2_interface::srv::GetRobotJointParam>(
      "/tl_driver/get_robot_joint_param",
      std::bind(&TL_Arm::handle_get_robot_joint_param_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  set_robot_joint_param_service_ = this->create_service<tl_ros2_interface::srv::SetRobotJointParam>(
      "/tl_driver/set_robot_joint_param",
      std::bind(&TL_Arm::handle_set_robot_joint_param_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_joint_temperature_service_ = this->create_service<tl_ros2_interface::srv::GetJointTemperature>(
      "/tl_driver/get_joint_temperature",
      std::bind(&TL_Arm::handle_get_joint_temperature_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_joint_voltage_service_ = this->create_service<tl_ros2_interface::srv::GetJointVoltage>(
      "/tl_driver/get_joint_voltage",
      std::bind(&TL_Arm::handle_get_joint_voltage_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_motor_current_service_ = this->create_service<tl_ros2_interface::srv::GetMotorCurrent>(
      "/tl_driver/get_motor_current",
      std::bind(&TL_Arm::handle_get_motor_current_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_joint_software_version_service_ = this->create_service<tl_ros2_interface::srv::GetJointSoftwareVersion>(
      "/tl_driver/get_joint_software_version",
      std::bind(&TL_Arm::handle_get_joint_software_version_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_nexmotion_lib_version_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/tl_driver/get_nexmotion_lib_version",
      std::bind(&TL_Arm::handle_get_nexmotion_lib_version_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  restore_default_dh_param_service_ = this->create_service<tl_ros2_interface::srv::RestoreDefaultDHParam>(
      "/tl_driver/restore_default_dh_param",
      std::bind(&TL_Arm::handle_restore_default_dh_param_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  set_default_cartesian_param_service_ =
      this->create_service<std_srvs::srv::Trigger>("/tl_driver/set_default_cartesian_param",
                                                   std::bind(&TL_Arm::handle_set_default_cartesian_param_service, this,
                                                             std::placeholders::_1, std::placeholders::_2),
                                                   rmw_qos_profile_services_default, service_group_);

  log_download_service_ = this->create_service<tl_ros2_interface::srv::LogDownload>(
      "/tl_driver/log_download",
      std::bind(&TL_Arm::handle_log_download_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  set_drag_mode_service_ = this->create_service<tl_ros2_interface::srv::SetDragMode>(
      "/tl_driver/set_drag_mode",
      std::bind(&TL_Arm::handle_set_drag_mode_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_drag_status_service_ = this->create_service<std_srvs::srv::Trigger>( // 用不了
      "/tl_driver/get_drag_status",
      std::bind(&TL_Arm::handle_get_drag_status_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  track_save_service_ = this->create_service<tl_ros2_interface::srv::TrackSave>(
      "/tl_driver/track_save",
      std::bind(&TL_Arm::handle_track_save_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  track_playback_service_ = this->create_service<tl_ros2_interface::srv::TrackPlayback>(
      "/tl_driver/track_playback",
      std::bind(&TL_Arm::handle_track_playback_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  set_tool_param_service_ = this->create_service<tl_ros2_interface::srv::SetToolParam>(
      "/tl_driver/set_tool_param",
      std::bind(&TL_Arm::handle_set_tool_param_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  set_user_coord_service_ = this->create_service<tl_ros2_interface::srv::SetUserCoord>(
      "/tl_driver/set_user_coord",
      std::bind(&TL_Arm::handle_set_user_coord_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  set_axis_zero_pos_service_ = this->create_service<tl_ros2_interface::srv::SetAxisZeroPos>(
      "/tl_driver/set_axis_zero_pos",
      std::bind(&TL_Arm::handle_set_axis_zero_pos_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  set_current_coord_service_ = this->create_service<tl_ros2_interface::srv::SetCurrentCoord>(
      "/tl_driver/set_current_coord",
      std::bind(&TL_Arm::handle_set_current_coord_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_current_coord_service_ = this->create_service<tl_ros2_interface::srv::GetCurrentCoord>(
      "/tl_driver/get_current_coord",
      std::bind(&TL_Arm::handle_get_current_coord_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  set_coord_num_service_ = this->create_service<tl_ros2_interface::srv::SetCoordNum>(
      "/tl_driver/set_coord_num",
      std::bind(&TL_Arm::handle_set_coord_num_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_coord_num_service_ = this->create_service<tl_ros2_interface::srv::GetCoordNum>(
      "/tl_driver/get_coord_num",
      std::bind(&TL_Arm::handle_get_coord_num_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  set_digital_output_service_ = this->create_service<tl_ros2_interface::srv::SetDigitalOutput>(
      "/tl_driver/set_digital_output",
      std::bind(&TL_Arm::handle_set_digital_output_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_digital_input_output_service_ = this->create_service<tl_ros2_interface::srv::GetDigitalInputOutput>(
      "/tl_driver/get_digital_input_output",
      std::bind(&TL_Arm::handle_get_digital_input_output_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  modbus_write_service_ = this->create_service<tl_ros2_interface::srv::ModbusWrite>(
      "/tl_driver/modbus_write",
      std::bind(&TL_Arm::handle_modbus_write_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  modbus_read_service_ = this->create_service<tl_ros2_interface::srv::ModbusRead>(
      "/tl_driver/modbus_read",
      std::bind(&TL_Arm::handle_modbus_read_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  coord_transform_service_ = this->create_service<tl_ros2_interface::srv::CoordTransform>(
      "/tl_driver/coord_transform",
      std::bind(&TL_Arm::handle_coord_transform_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_pos_reachable_service_ = this->create_service<tl_ros2_interface::srv::GetPosReachable>(
      "/tl_driver/get_pos_reachable",
      std::bind(&TL_Arm::handle_get_pos_reachable_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  set_dh_param_service_ = this->create_service<tl_ros2_interface::srv::SetDHParam>(
      "/tl_driver/set_dh_param",
      std::bind(&TL_Arm::handle_set_dh_param_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_dh_param_service_ = this->create_service<tl_ros2_interface::srv::GetDHParam>(
      "/tl_driver/get_dh_param",
      std::bind(&TL_Arm::handle_get_dh_param_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_all_job_filename_service_ = this->create_service<tl_ros2_interface::srv::GetAllJobFileName>(
      "/tl_driver/get_all_job_filename",
      std::bind(&TL_Arm::handle_get_all_job_filename_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  job_run_service_ = this->create_service<tl_ros2_interface::srv::JobRun>(
      "/tl_driver/job_run",
      std::bind(&TL_Arm::handle_job_run_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  job_delete_service_ = this->create_service<tl_ros2_interface::srv::JobRun>(
      "/tl_driver/job_delete",
      std::bind(&TL_Arm::handle_job_delete_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  job_insert_movej_service_ = this->create_service<tl_ros2_interface::srv::JobInsertMove>(
      "/tl_driver/job_insert_moveJ",
      std::bind(&TL_Arm::handle_job_insert_movej_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  job_insert_movel_service_ = this->create_service<tl_ros2_interface::srv::JobInsertMove>(
      "/tl_driver/job_insert_moveL",
      std::bind(&TL_Arm::handle_job_insert_movel_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  job_insert_imove_service_ = this->create_service<tl_ros2_interface::srv::JobInsertMove>(
      "/tl_driver/job_insert_imove",
      std::bind(&TL_Arm::handle_job_insert_imove_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  job_insert_movec_service_ = this->create_service<tl_ros2_interface::srv::JobInsertMove>(
      "/tl_driver/job_insert_moveC",
      std::bind(&TL_Arm::handle_job_insert_movec_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  set_global_pos_service_ = this->create_service<tl_ros2_interface::srv::SetGlobalPos>(
      "/tl_driver/set_global_pos",
      std::bind(&TL_Arm::handle_set_global_pos_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_global_pos_service_ = this->create_service<tl_ros2_interface::srv::GetGlobalPos>(
      "/tl_driver/get_global_pos",
      std::bind(&TL_Arm::handle_get_global_pos_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  set_current_mode_service_ = this->create_service<tl_ros2_interface::srv::SetCurrentMode>(
      "/tl_driver/set_current_mode",
      std::bind(&TL_Arm::handle_set_current_mode_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_current_mode_service_ = this->create_service<tl_ros2_interface::srv::GetCurrentMode>(
      "/tl_driver/get_current_mode",
      std::bind(&TL_Arm::handle_get_current_mode_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  open_servoj_service_ = this->create_service<tl_ros2_interface::srv::OpenServoJ>(
      "/tl_driver/open_servoj",
      std::bind(&TL_Arm::handle_open_servoj_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  close_servoj_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/tl_driver/close_servoj",
      std::bind(&TL_Arm::handle_close_servoj_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  queue_motion_set_status_service_ = this->create_service<tl_ros2_interface::srv::QueueMotionSetStatus>(
      "/tl_driver/queue_motion_set_status",
      std::bind(&TL_Arm::handle_queue_motion_set_status_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  queue_motion_movej_service_ = this->create_service<tl_ros2_interface::srv::QueueMotionMoveJ>(
      "/tl_driver/queue_motion_movej",
      std::bind(&TL_Arm::handle_queue_motion_movej_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  queue_motion_stop_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/tl_driver/queue_motion_stop",
      std::bind(&TL_Arm::handle_queue_motion_stop_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_current_motor_torque_service_ = this->create_service<tl_ros2_interface::srv::GetCurrentMotorTorque>(
      "/tl_driver/get_current_motor_torque",
      std::bind(&TL_Arm::handle_get_current_motor_torque_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_current_line_joint_speed_service_ = this->create_service<tl_ros2_interface::srv::GetCurrentLineJointSpeed>(
      "/tl_driver/get_current_line_joint_speed",
      std::bind(&TL_Arm::handle_get_current_line_joint_speed_service, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  // 话题pub
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  tcp_pose_pub_ = this->create_publisher<tl_ros2_interface::msg::CartesianPose>("/tcp_pose", 10);

  running_status_pub_ = this->create_publisher<tl_ros2_interface::msg::ArmStatus>("/arm_status", 10);

  // 话题sub
  movej_sub_ = this->create_subscription<tl_ros2_interface::msg::MoveCommand>(
      "/tl_driver/moveJ", 10, std::bind(&TL_Arm::handle_movej_topic, this, std::placeholders::_1), topic_group_option);

  movel_sub_ = this->create_subscription<tl_ros2_interface::msg::MoveCommand>(
      "/tl_driver/moveL", 10, std::bind(&TL_Arm::handle_movel_topic, this, std::placeholders::_1), topic_group_option);

  set_servoj_pos_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/tl_driver/set_servoj_pos", 10, std::bind(&TL_Arm::handle_set_servoj_pos_topic, this, std::placeholders::_1),
      topic_group_option);

  set_servol_pos_sub_ = this->create_subscription<tl_ros2_interface::msg::ServolMove>(
      "/tl_driver/set_servol_pos", 10, std::bind(&TL_Arm::handle_set_servol_pos_topic, this, std::placeholders::_1),
      topic_group_option);

  auto period =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / publish_rate_));

  state_publish_timer_ = this->create_wall_timer(period, std::bind(&TL_Arm::publish_arm_state, this), timer_group_);

  // 初始化
  init();
  RCLCPP_INFO(this->get_logger(), "%s_driver is running ", arm_type_.c_str());
}

TL_Arm::~TL_Arm()
{
  if (is_connected())
  {
    power_off();
    disconnect();
  }
}

void TL_Arm::init()
{
  RCLCPP_INFO(this->get_logger(), "Trying to connect to %s:%s,%s", arm_ip_.c_str(), arm_port_.c_str(),
              arm_port_aux_.c_str());
  if (connect())
  {
    // 切换到示教模式
    int ret = set_current_mode(socket_fd_, 0);
    if (ret != Result::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "[Init]: failed to set teach mode, result=%s", result_to_string(ret));
      rclcpp::shutdown();
      exit(0);
    }

    // 清除控制器错误和报警
    ret = clear_error(socket_fd_);
    if (ret != Result::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(), "[Init]: clear_error result=%s", result_to_string(ret));
    }

    // 清错后下电，释放控制器占用状态
    power_off();

    power_on();
    RCLCPP_INFO(this->get_logger(), "上电延时2s...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(this->get_logger(), "上电延时完成");
  }
  else
  {
    rclcpp::shutdown();
    exit(0);
  }
}

bool TL_Arm::is_connected()
{
  return is_connected_ && socket_fd_ > 0 && socket_fd_aux_ > 0;
}

bool TL_Arm::is_powered()
{
  return is_powered_;
}

bool TL_Arm::power_on()
{
  int state = -1;
  get_servo_state(socket_fd_, state);

  switch (state)
  {
    case 0:
      set_servo_state(socket_fd_, 1);
      RCLCPP_INFO(this->get_logger(), "[PowerOn]: waiting 2s for servo ready...");
      std::this_thread::sleep_for(std::chrono::seconds(2));
      set_servo_poweron(socket_fd_);
      break;
    case 1:
      set_servo_poweron(socket_fd_);
      break;
    case 2:
      RCLCPP_WARN(this->get_logger(), "[PowerOn]: servo alarm state (2), clearing error first");
      clear_error(socket_fd_);
      set_servo_state(socket_fd_, 1);
      set_servo_poweron(socket_fd_);
      break;
    case 3:
      RCLCPP_INFO(this->get_logger(), "[PowerOn]: servo_state=3, re-executing power on");
      set_servo_state(socket_fd_, 1);
      set_servo_poweron(socket_fd_);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "[PowerOn]: unknown servo state %d", state);
      return false;
  }

  get_servo_state(socket_fd_, state);
  RCLCPP_INFO(this->get_logger(), "[PowerOn]: servo_state = %d", state);
  if (state == 3)
  {
    is_powered_ = true;
    return true;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "[PowerOn]: failed to power on, servo_state = %d", state);
    return false;
  }
}

bool TL_Arm::power_off()
{
  int state = -1;
  get_servo_state(socket_fd_, state);
  switch (state)
  {
    case 0:
    case 2:
      break;
    case 1:
      RCLCPP_INFO(this->get_logger(), "[PowerOff]: already power off");
      return true;
    case 3:
      set_servo_poweroff(socket_fd_);
      get_servo_state(socket_fd_, state);
      RCLCPP_INFO(this->get_logger(), "[PowerOff]: successfully power off, servo_state = %d", state);
      is_powered_ = false;
      return true;
  }
  RCLCPP_INFO(this->get_logger(), "[PowerOff]: fail to power off, servo_state = %d", state);
  return false;
}

bool TL_Arm::connect()
{
  if (is_connected_)
  {
    RCLCPP_INFO(this->get_logger(), "[Connect]: arm already connected");
    return true;
  }

  socket_fd_ = connect_robot(arm_ip_, arm_port_);
  socket_fd_aux_ = connect_robot(arm_ip_, arm_port_aux_);

  if (socket_fd_ <= 0)
  {
    RCLCPP_ERROR(this->get_logger(), "[Connect]: failed to connect to %s:%s", arm_ip_.c_str(), arm_port_.c_str());
    socket_fd_ = 0;
    is_connected_ = false;
    return false;
  }

  if (socket_fd_aux_ <= 0)
  {
    RCLCPP_ERROR(this->get_logger(), "[Connect]: failed to connect to %s:%s", arm_ip_.c_str(), arm_port_aux_.c_str());
    socket_fd_aux_ = 0;
    is_connected_ = false;
    return false;
  }

  is_connected_ = true;

  set_receive_error_or_warnning_message_callback(socket_fd_, receive_error_or_warning_message_callback);

  set_receive_error_or_warnning_message_callback(socket_fd_aux_, receive_error_or_warning_message_callback);

  // 新 SDK：机器人状态通过 robot_state_callback（7000 端口）接收，不再使用旧的 recv_message 机制
  robot_state_callback(socket_fd_aux_, robot_state_callback_handler);

  RCLCPP_INFO(this->get_logger(), "[Connect]: successfully connected to arm at %s:%s,%s", arm_ip_.c_str(),
              arm_port_.c_str(), arm_port_aux_.c_str());

  return true;
}

bool TL_Arm::disconnect()
{
  if (!is_connected_)
  {
    RCLCPP_INFO(this->get_logger(), "[Disconnect]: arm already disconnected");
    return true;
  }

  is_connected_ = false;

  disconnect_robot(socket_fd_);
  disconnect_robot(socket_fd_aux_);

  socket_fd_ = 0;
  socket_fd_aux_ = 0;

  return true;
}

void TL_Arm::handle_connect_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (is_connected())
  {
    response->success = true; // shc 26.5.19 修改：连接了就响应true
    response->message = "Arm already connected";
    return;
  }

  response->success = connect();
  response->message = response->success ? "Arm connected successfully" : "Failed to connect arm";
}

void TL_Arm::handle_disconnect_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  response->success = disconnect();
  response->message = response->success ? "Arm disconnected successfully" : "Failed to disconnect arm";
}

void TL_Arm::handle_poweron_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  response->success = power_on();
  response->message = response->success ? "Arm power on successfully" : "Failed to power on arm";
}

void TL_Arm::handle_poweroff_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (!is_powered_)
  {
    response->success = true;
    response->message = "Arm already power off";
    return;
  }

  response->success = power_off();
  response->message = response->success ? "Arm power off successfully" : "Failed to power off arm";
}

void TL_Arm::handle_clear_error_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int state = -1;
  get_servo_state(socket_fd_, state);

  int ret = clear_error(socket_fd_);
  response->success = (ret == Result::SUCCESS);
  response->message =
      response->success ? "Clear error successfully" : std::string("Clear error failed: ") + result_to_string(ret);
}

void TL_Arm::handle_set_speed_service(const std::shared_ptr<tl_ros2_interface::srv::SetSpeed::Request> request,
                                      std::shared_ptr<tl_ros2_interface::srv::SetSpeed::Response> response)
{
  if (!is_connected())
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

  int ret = set_speed(socket_fd_, request->speed);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Set speed successfully" : "Failed to set speed";
}

void TL_Arm::handle_get_speed_service(const std::shared_ptr<tl_ros2_interface::srv::GetSpeed::Request> request,
                                      std::shared_ptr<tl_ros2_interface::srv::GetSpeed::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int speed;
  int ret = get_speed(socket_fd_, speed);
  if (ret != Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to get speed";
    return;
  }
  response->success = true;
  response->message = "Get speed successfully";
  response->speed = speed;
}

void TL_Arm::handle_get_quat2rpy_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->input.size() < 4)
  {
    response->success = false;
    response->message = "Invalid quat input";
    return;
  }

  std::vector<double> rpy;
  int ret = get_quat2rpy(socket_fd_, request->input, rpy);
  if (ret == Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get quat2rpy successfully";
    response->output = rpy;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get quat2rpy";
  }
}

void TL_Arm::handle_get_rpy2quat_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->input.size() < 3)
  {
    response->success = false;
    response->message = "Invalid rpy input";
    return;
  }

  std::vector<double> quat;
  int ret = get_rpy2quat(socket_fd_, request->input, quat);
  if (ret == Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get rpy2quat successfully";
    response->output = quat;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get rpy2quat";
  }
}

void TL_Arm::handle_get_rpy2r_service(const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
                                      std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->input.size() < 3)
  {
    response->success = false;
    response->message = "Invalid rpy input";
    return;
  }

  std::vector<double> rot;
  int ret = get_rpy2r(socket_fd_, request->input, rot);
  if (ret == Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get rpy2r successfully";
    response->output = rot;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get rpy2r";
  }
}

void TL_Arm::handle_get_tr2r_service(const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
                                     std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->input.size() < 16)
  {
    response->success = false;
    response->message = "Invalid tr input";
    return;
  }

  std::vector<double> rot;
  int ret = get_tr2r(socket_fd_, request->input, rot);
  if (ret == Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get tr2r successfully";
    response->output = rot;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get tr2r";
  }
}

void TL_Arm::handle_get_r2tr_service(const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
                                     std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->input.size() < 9)
  {
    response->success = false;
    response->message = "Invalid tr matrix input";
    return;
  }

  std::vector<double> tr_matrix;
  int ret = get_r2tr(socket_fd_, request->input, tr_matrix);
  if (ret == Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get r2tr successfully";
    response->output = tr_matrix;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get r2tr";
  }
}

void TL_Arm::handle_set_controller_ip_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetControllerIP::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetControllerIP::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int ret = set_controller_ip(socket_fd_, request->name, request->addr, request->gateway, request->dns);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Set controller IP successfully" : "Failed to set controller IP";
}

void TL_Arm::handle_get_controller_id_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                              std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::string id{};
  Result ret = get_controller_id(socket_fd_, id);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? id : "Failed to get controller id, ret=" + std::to_string(ret);
}

void TL_Arm::handle_start_jogging_service(const std::shared_ptr<tl_ros2_interface::srv::Jogging::Request> request,
                                          std::shared_ptr<tl_ros2_interface::srv::Jogging::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->axis < 1 || request->axis > ndof_)
  {
    response->success = false;
    response->message = "Invalid axis";
    return;
  }

  int ret = robot_start_jogging(socket_fd_, request->axis, request->direction);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Start jogging successfully" : "Failed to start jogging";
}

void TL_Arm::handle_stop_jogging_service(const std::shared_ptr<tl_ros2_interface::srv::Jogging::Request> request,
                                         std::shared_ptr<tl_ros2_interface::srv::Jogging::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->axis < 1 || request->axis > ndof_)
  {
    response->success = false;
    response->message = "Invalid axis";
    return;
  }

  int ret = robot_stop_jogging(socket_fd_, request->axis);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Stop jogging successfully" : "Failed to stop jogging";
}

void TL_Arm::handle_get_robot_state_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetRobotState::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetRobotState::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  RobotState param{};
  param.channel = request->channel;
  param.stop = request->stop;
  param.mode = request->mode;
  param.interval = request->interval;
  param.ioState = request->io_state;
  param.position = request->position;
  param.dataildmotionpos = request->detail_motion_pos;
  param.posSum = request->pos_sum;
  param.ioPort = request->io_port;
  param.optional = request->optional;

  // 新 SDK：io_state=true 但 io_port 为空会返回 PARAM_ERR(-3)，请求 IO 时必须指定端口
  if (param.ioState && param.ioPort.empty())
  {
    RCLCPP_WARN(this->get_logger(), "[GetRobotState]: io_state=true but io_port empty, force io_state=false");
    param.ioState = false;
  }

  uint64_t start_seq = 0;
  {
    std::lock_guard<std::mutex> lock(g_robot_state_msg_buffer.mutex);
    start_seq = g_robot_state_msg_buffer.seq;
  }

  // get_robot_state 需使用 7000 端口（socket_fd_aux_），回调亦注册在 7000 端口
  Result ret = get_robot_state(socket_fd_aux_, param);
  if (ret != Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to get robot state";
    return;
  }

  std::unique_lock<std::mutex> lock(g_robot_state_msg_buffer.mutex);

  bool received =
      g_robot_state_msg_buffer.cv.wait_for(lock, std::chrono::seconds(5),
                                           [start_seq]()
                                           {
                                             return g_robot_state_msg_buffer.seq > start_seq &&
                                                    g_robot_state_msg_buffer.last_msg_id == MessageLists::ROBOT_STATE;
                                           });

  if (!received)
  {
    response->success = false;
    response->message = "Timeout waiting for robot state";
    return;
  }

  response->success = true;
  response->message = g_robot_state_msg_buffer.last_msg;
}

void TL_Arm::handle_get_library_version_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::string version = get_library_version();
  response->success = (version != "");
  response->message = response->success ? version : "Failed to get library version";
}

void TL_Arm::handle_get_robot_joint_param_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetRobotJointParam::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetRobotJointParam::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->id < 1 || request->id > ndof_)
  {
    response->success = false;
    response->message = "Invalid id";
    return;
  }

  RobotJointParam param{};
  int ret = get_robot_joint_param(socket_fd_, request->id, param);
  if (ret == Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get robot joint param successfully";
    response->param.reduction_ratio = param.reduceRatio;
    response->param.encoder_resolution = param.encoderResolution;
    response->param.pos_sw_limit = param.maxPos;
    response->param.neg_sw_limit = param.minPos;
    response->param.rated_rot_speed = param.ratedRotSpeed;
    response->param.max_rot_speed = param.maxRotSpeed;
    response->param.max_acc = param.maxAcc;
    response->param.max_dec = param.maxDec;
    response->param.direction = param.axisDirection;
    // 新 SDK 无 rated_derot_speed / max_derot_speed / rated_vel / rated_devel 对应字段，保持默认 0
  }
  else
  {
    response->success = true;
    response->message = "Falied to get robot joint param";
  }
}

void TL_Arm::handle_set_robot_joint_param_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetRobotJointParam::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetRobotJointParam::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->id < 1 || request->id > ndof_)
  {
    response->success = false;
    response->message = "Invalid id";
    return;
  }

  RobotJointParam param{};
  param.reduceRatio = request->param.reduction_ratio;
  param.encoderResolution = request->param.encoder_resolution;
  param.maxPos = request->param.pos_sw_limit;
  param.minPos = request->param.neg_sw_limit;
  param.ratedRotSpeed = request->param.rated_rot_speed;
  param.maxRotSpeed = request->param.max_rot_speed;
  param.maxAcc = request->param.max_acc;
  param.maxDec = request->param.max_dec;
  param.axisDirection = request->param.direction;
  // 新 SDK 无 rated_derot_speed / max_derot_speed / rated_vel / rated_devel 对应字段，忽略

  int ret = set_robot_joint_param(socket_fd_, request->id, param);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Set robot joint param successfully" : "Falied to set robot joint param";

  // 设置参数后机械臂会下电
  power_off();
  power_on();
}

void TL_Arm::handle_get_joint_temperature_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetJointTemperature::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetJointTemperature::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::vector<double> temperatures;
  int ret = get_joint_temperature(socket_fd_, temperatures);
  if (ret == Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get joint temperatures successfully";
    response->temperatures = temperatures;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get joint temperatures";
  }
}

void TL_Arm::handle_get_joint_voltage_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetJointVoltage::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetJointVoltage::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::vector<double> joint_voltage;
  std::vector<double> positioner_voltage;
  int ret = get_joint_voltage(socket_fd_, joint_voltage, positioner_voltage);
  if (ret == Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get joint voltage successfully";
    response->joint_voltage = joint_voltage;
    response->positioner_voltage = positioner_voltage;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get joint voltage";
  }
}

void TL_Arm::handle_get_motor_current_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetMotorCurrent::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetMotorCurrent::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::vector<double> motor_current{};
  std::vector<double> motor_current_sync{};
  Result ret = get_current_motor_current(socket_fd_, motor_current, motor_current_sync);
  if (ret == Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get motor current successfully";
    response->motor_current = motor_current;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get motor current, ret=" + std::to_string(ret);
  }
}

void TL_Arm::handle_get_joint_software_version_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetJointSoftwareVersion::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetJointSoftwareVersion::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->axis_num < 1 || request->axis_num > ndof_)
  {
    response->success = false;
    response->message = "Invalid axis num";
    return;
  }

  std::string version;
  int ret = query_joint_software_version(socket_fd_, request->axis_num, version);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? version : "Failed to get joint software version";
}

void TL_Arm::handle_get_nexmotion_lib_version_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::string version;
  Result ret = get_nexmotion_lib_version(socket_fd_, version);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? version : "Failed to get nexmotion lib version, ret=" + std::to_string(ret);
}

void TL_Arm::handle_restore_default_dh_param_service(
    const std::shared_ptr<tl_ros2_interface::srv::RestoreDefaultDHParam::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::RestoreDefaultDHParam::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int ret = restore_default_param_DH(socket_fd_, request->robot_num);
  response->success = (ret == Result::SUCCESS);
  response->message =
      response->success ? "Restore default DH param successfully" : "Failed to restore default DH param";
}

void TL_Arm::handle_set_default_cartesian_param_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int ret = set_default_cartesian_params(socket_fd_);
  response->success = (ret == Result::SUCCESS);
  response->message =
      response->success ? "Set default cartesian param successfully" : "Failed to set default cartesian param";
}

void TL_Arm::handle_log_download_service(const std::shared_ptr<tl_ros2_interface::srv::LogDownload::Request> request,
                                         std::shared_ptr<tl_ros2_interface::srv::LogDownload::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int ret = log_download_by_quantity(socket_fd_, request->count, request->directory_path);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Log download successfully" : "Failed to download log";
}

void TL_Arm::handle_set_drag_mode_service(const std::shared_ptr<tl_ros2_interface::srv::SetDragMode::Request> request,
                                          std::shared_ptr<tl_ros2_interface::srv::SetDragMode::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->mode < 0 || request->mode > 3)
  {
    response->success = false;
    response->message = "Invalid mode";
    return;
  }

  int ret = set_darg_mode(socket_fd_, request->mode);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Set drag mode successfully" : "Failed to set drag mode";
}

void TL_Arm::handle_get_drag_status_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  bool endFlag = false;
  int ret = get_drag_thread_is_end(socket_fd_, endFlag);
  if (ret == Result::SUCCESS)
  {
    response->success = endFlag;
    response->message = response->success ? "Drag ended" : "Drag not ended";
  }
  else
  {
    response->success = true;
    response->message = "Failed to get drag status";
  }
}

void TL_Arm::handle_track_save_service(const std::shared_ptr<tl_ros2_interface::srv::TrackSave::Request> request,
                                       std::shared_ptr<tl_ros2_interface::srv::TrackSave::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int ret = track_record_save(socket_fd_, request->traj_name);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Track save successfully" : "Failed to save track";
}

void TL_Arm::handle_track_playback_service(
    const std::shared_ptr<tl_ros2_interface::srv::TrackPlayback::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::TrackPlayback::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int ret = track_record_playback(socket_fd_, request->vel);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Track playback successfully" : "Failed to playback track";
}

void TL_Arm::handle_set_tool_param_service(const std::shared_ptr<tl_ros2_interface::srv::SetToolParam::Request> request,
                                           std::shared_ptr<tl_ros2_interface::srv::SetToolParam::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  ToolParam param{};
  param.X = request->param.x;
  param.Y = request->param.y;
  param.Z = request->param.z;
  param.A = request->param.a;
  param.B = request->param.b;
  param.C = request->param.c;
  param.payloadMass = request->param.payload_mass;
  param.payloadInertia = request->param.payload_inertia;
  param.payloadMassCenter_X = request->param.payload_mass_center_x;
  param.payloadMassCenter_Y = request->param.payload_mass_center_y;
  param.payloadMassCenter_Z = request->param.payload_mass_center_z;

  int ret = set_tool_hand_param(socket_fd_, request->tool_num, param);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Set tool hand param successfully" : "Failed to set tool hand param";
}

void TL_Arm::handle_set_user_coord_service(const std::shared_ptr<tl_ros2_interface::srv::SetUserCoord::Request> request,
                                           std::shared_ptr<tl_ros2_interface::srv::SetUserCoord::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  // 使用新 SDK 推荐的 UserCoordParam 重载（旧的 vector 重载已标记 deprecated）
  UserCoordParam param{};
  param.location_type = 0; // 静态用户坐标
  param.position[0] = request->pos.position.x;
  param.position[1] = request->pos.position.y;
  param.position[2] = request->pos.position.z;
  param.position[3] = request->pos.rpy.x;
  param.position[4] = request->pos.rpy.y;
  param.position[5] = request->pos.rpy.z;

  int ret = set_user_coordinate_data(socket_fd_, request->user_num, param);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Set user coordinate successfully" : "Failed to set user coordinate";
}

void TL_Arm::handle_set_axis_zero_pos_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetAxisZeroPos::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetAxisZeroPos::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int ret = set_axis_zero_position(socket_fd_, request->axis);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Set user coordinate successfully" : "Failed to set user coordinate";
}

void TL_Arm::handle_set_current_coord_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetCurrentCoord::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetCurrentCoord::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int ret = set_current_coord(socket_fd_, request->coord);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Set current coordinate successfully" : "Failed to set current coordinate";
}

void TL_Arm::handle_get_current_coord_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetCurrentCoord::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetCurrentCoord::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int coord;
  int ret = get_current_coord(socket_fd_, coord);
  if (ret == Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get current coordinate successfully";
    response->coord = coord;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get current coordinate";
  }
}

void TL_Arm::handle_set_coord_num_service(const std::shared_ptr<tl_ros2_interface::srv::SetCoordNum::Request> request,
                                          std::shared_ptr<tl_ros2_interface::srv::SetCoordNum::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int ret = set_tool_hand_number(socket_fd_, request->tool_num);
  int ret1 = set_user_coord_number(socket_fd_, request->user_num);
  if (ret == Result::SUCCESS && ret1 == Result::SUCCESS)
  {
    response->success = true;
    response->message = "Set all coordinate number successfully";
  }
  else
  {
    response->success = false;
    response->message = "Failed to set all coordinate number";
  }
}

void TL_Arm::handle_get_coord_num_service(const std::shared_ptr<tl_ros2_interface::srv::GetCoordNum::Request> request,
                                          std::shared_ptr<tl_ros2_interface::srv::GetCoordNum::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int toolNum = -1, userNum = -1;
  int ret = get_tool_hand_number(socket_fd_, toolNum);
  int ret1 = get_user_coord_number(socket_fd_, userNum);
  if (ret == Result::SUCCESS && ret1 == Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get all coordinate number successfully";
    response->tool_num = toolNum;
    response->user_num = userNum;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get all coordinate number";
  }
}

void TL_Arm::handle_set_digital_output_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetDigitalOutput::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetDigitalOutput::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->value < 0 || request->value > 1)
  {
    response->success = false;
    response->message = "Invalid value";
    return;
  }

  int ret = set_digital_output(socket_fd_, request->port, request->value);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Set digital output successfully" : "Failed to set digital output";
}

void TL_Arm::handle_get_digital_input_output_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetDigitalInputOutput::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetDigitalInputOutput::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::vector<int> digitalInput;
  std::vector<int> digitalOutput;
  int ret = get_digital_input(socket_fd_, digitalInput);
  int ret1 = get_digital_output(socket_fd_, digitalOutput);
  if (ret == Result::SUCCESS && ret1 == Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get digital input and output successfully";
    response->input = digitalInput;
    response->output = digitalOutput;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get digital input and output";
  }
}

void TL_Arm::handle_modbus_write_service(const std::shared_ptr<tl_ros2_interface::srv::ModbusWrite::Request> request,
                                         std::shared_ptr<tl_ros2_interface::srv::ModbusWrite::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  ModbusMasterParameter master_param;
  master_param.type = request->master_param.type;
  // 兼容大小写：统一转大写后判断（SDK 期望 "TCP"/"RTU"）
  for (auto &c : master_param.type)
  {
    if (c >= 'a' && c <= 'z') c -= 32;
  }
  master_param.startAddress = request->master_param.start_addr;

  if (master_param.type == "TCP")
  {
    master_param.TCP.IP = request->master_param.tcp.ip;
    master_param.TCP.port = request->master_param.tcp.port;
  }
  else if (master_param.type == "RTU")
  {
    master_param.RTU.slaveId = request->master_param.rtu.slave_id;
    master_param.RTU.port = request->master_param.rtu.port;
    master_param.RTU.baudrate = request->master_param.rtu.baudrate;
    master_param.RTU.checkBit = request->master_param.rtu.check_bit;
    master_param.RTU.dataBit = request->master_param.rtu.data_bit;
    master_param.RTU.stopBit = request->master_param.rtu.stop_bit;
  }
  else
  {
    response->success = false;
    response->message = "Invalid master type";
    return;
  }

  int ret = modbus_set_master_parameter(socket_fd_, request->master_id, master_param);
  if (ret != Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to set master paramter";
    return;
  }

  ret = modbus_open_master(socket_fd_, request->master_id);
  if (ret != Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to open master";
    return;
  }

  std::vector<int> data = request->data;
  ret = modbus_write_multiple_holding_registers(socket_fd_, request->master_id, request->addr, data);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Modbus write successfully" : "Failed to write Modbus";
}

void TL_Arm::handle_modbus_read_service(const std::shared_ptr<tl_ros2_interface::srv::ModbusRead::Request> request,
                                        std::shared_ptr<tl_ros2_interface::srv::ModbusRead::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  ModbusMasterParameter master_param;
  master_param.type = request->master_param.type;
  // 兼容大小写：统一转大写后判断（SDK 期望 "TCP"/"RTU"）
  for (auto &c : master_param.type)
  {
    if (c >= 'a' && c <= 'z') c -= 32;
  }
  master_param.startAddress = request->master_param.start_addr;

  if (master_param.type == "TCP")
  {
    master_param.TCP.IP = request->master_param.tcp.ip;
    master_param.TCP.port = request->master_param.tcp.port;
  }
  else if (master_param.type == "RTU")
  {
    master_param.RTU.slaveId = request->master_param.rtu.slave_id;
    master_param.RTU.port = request->master_param.rtu.port;
    master_param.RTU.baudrate = request->master_param.rtu.baudrate;
    master_param.RTU.checkBit = request->master_param.rtu.check_bit;
    master_param.RTU.dataBit = request->master_param.rtu.data_bit;
    master_param.RTU.stopBit = request->master_param.rtu.stop_bit;
  }
  else
  {
    response->success = false;
    response->message = "Invalid master type";
    return;
  }

  int ret = modbus_set_master_parameter(socket_fd_, request->master_id, master_param);
  if (ret != Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to set master paramter";
    return;
  }

  ret = modbus_open_master(socket_fd_, request->master_id);
  if (ret != Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to open master";
    return;
  }

  std::vector<int> data{};
  ret = modbus_read_holding_registers(socket_fd_, request->master_id, request->addr, request->quantity, data);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Modbus read successfully" : "Failed to read Modbus";
  response->data = data;
}

void TL_Arm::handle_coord_transform_service(
    const std::shared_ptr<tl_ros2_interface::srv::CoordTransform::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::CoordTransform::Response> response)
{
  if (!is_connected())
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

  std::vector<double> originPos = request->origin_pos;
  std::vector<double> referencePos = request->reference_pos;
  std::vector<double> targetPos;

  int ret = get_origin_coord_to_target_coord(socket_fd_, request->origin_coord, originPos, request->target_coord,
                                             targetPos, request->form, referencePos);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Coord transform successfully" : "Failed to transform coord";
  response->target_pos = targetPos;
}

void TL_Arm::handle_get_pos_reachable_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetPosReachable::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetPosReachable::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->move_type != "MOVJ" && request->move_type != "MOVL")
  {
    response->success = false;
    response->message = "Invalid move type";
    return;
  }

  std::vector<double> queryPos = request->pos;

  bool result = false;
  Result ret = get_pos_reachable(socket_fd_, queryPos, request->move_type, result);
  if (ret == Result::SUCCESS)
  {
    response->success = result;
    response->message = response->success ? "Target pos is reachable" : "Target pos is not reachable";
  }
  else
  {
    response->success = false;
    response->message = "Fail to get pos reachable status" + std::to_string(ret);
  }
}

void TL_Arm::handle_set_dh_param_service(const std::shared_ptr<tl_ros2_interface::srv::SetDHParam::Request> request,
                                         std::shared_ptr<tl_ros2_interface::srv::SetDHParam::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  // 新 SDK: 标准DH参数 alpha/a/theta/d，逐字段映射到 SDK 的 tl::RobotDHParam
  RobotDHParam dh{};
  const auto &p = request->param;
  for (int i = 0; i < 6 && i < static_cast<int>(p.alpha.size()); ++i) dh.alpha[i] = p.alpha[i];
  for (int i = 0; i < 6 && i < static_cast<int>(p.a.size()); ++i) dh.a[i] = p.a[i];
  for (int i = 0; i < 6 && i < static_cast<int>(p.theta.size()); ++i) dh.theta[i] = p.theta[i];
  for (int i = 0; i < 6 && i < static_cast<int>(p.d.size()); ++i) dh.d[i] = p.d[i];
  dh.eulerAngle = p.euler_angle;
  dh.mountingAngle = p.mounting_angle;

  int ret = set_robot_dh_param(socket_fd_, dh);
  if (ret == Result::SUCCESS)
  {
    response->success = true;
    response->message = "Set DH param successfully";
  }
  else
  {
    response->success = false;
    response->message = "Failed to set DH param, ret=" + std::to_string(ret);
  }
}

void TL_Arm::handle_get_dh_param_service(const std::shared_ptr<tl_ros2_interface::srv::GetDHParam::Request> request,
                                         std::shared_ptr<tl_ros2_interface::srv::GetDHParam::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  // 新 SDK: 标准DH参数 alpha/a/theta/d，逐字段映射到 ROS2 消息
  RobotDHParam dh{};
  Result ret = get_robot_dh_param(socket_fd_, dh);
  if (ret == Result::SUCCESS)
  {
    auto &p = response->param;
    p.alpha.assign(dh.alpha, dh.alpha + 6);
    p.a.assign(dh.a, dh.a + 6);
    p.theta.assign(dh.theta, dh.theta + 6);
    p.d.assign(dh.d, dh.d + 6);
    p.euler_angle = dh.eulerAngle;
    p.mounting_angle = dh.mountingAngle;
    response->success = true;
    response->message = "Get DH param successfully";
  }
  else
  {
    response->success = false;
    response->message = "Failed to get DH param, ret=" + std::to_string(ret);
  }
}

void TL_Arm::handle_get_all_job_filename_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetAllJobFileName::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetAllJobFileName::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::vector<std::vector<std::string>> robotsFile;
  int ret = job_get_all_jobfile_name(socket_fd_, robotsFile);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Get all job filename successfully" : "Failed to get all job filename";
  response->robots_file.clear();
  for (const auto& file_names : robotsFile)
  {
    tl_ros2_interface::msg::JobFileName msg;
    msg.file_name = file_names;
    response->robots_file.push_back(msg);
  }
}

void TL_Arm::handle_job_run_service(const std::shared_ptr<tl_ros2_interface::srv::JobRun::Request> request,
                                    std::shared_ptr<tl_ros2_interface::srv::JobRun::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int ret = job_run(socket_fd_, request->job_name);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Job run successfully" : "Failed to run job";

  // // 运行完成后会自动下电
  // power_off();
  // power_on();
}

void TL_Arm::handle_job_delete_service(const std::shared_ptr<tl_ros2_interface::srv::JobRun::Request> request,
                                       std::shared_ptr<tl_ros2_interface::srv::JobRun::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int ret = job_delete(socket_fd_, request->job_name);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Job delete successfully" : "Failed to delete job";
}

void TL_Arm::handle_job_insert_movej_service(
    const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  MoveCmd cmd{};
  cmd.targetPosType = static_cast<PosType>(PosType::data);
  cmd.targetPosName = "";
  cmd.coord = static_cast<Coord>(request->cmd.coord);
  cmd.velocity = request->cmd.velocity;
  cmd.velocitySync = request->cmd.velocity_sync;
  cmd.acc = request->cmd.acc;
  cmd.dec = request->cmd.dec;
  cmd.pl = request->cmd.pl;
  cmd.time = request->cmd.time;
  cmd.toolNum = request->cmd.tool_num;
  cmd.userNum = request->cmd.user_num;
  cmd.posidtype = request->cmd.posidtype;
  cmd.configuration = request->cmd.configuration;
  cmd.spin = request->cmd.spin;
  cmd.parasync = request->cmd.para_sync;
  cmd.targetPosValue = request->cmd.target_pos_value;

  int ret = job_insert_moveJ(socket_fd_, request->line, cmd);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Job insert movej successfully" : "Failed to insert job movej";
}

void TL_Arm::handle_job_insert_movel_service(
    const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  MoveCmd cmd{};
  cmd.targetPosType = static_cast<PosType>(PosType::data);
  cmd.targetPosName = "";
  cmd.coord = static_cast<Coord>(request->cmd.coord);
  cmd.velocity = request->cmd.velocity;
  cmd.velocitySync = request->cmd.velocity_sync;
  cmd.acc = request->cmd.acc;
  cmd.dec = request->cmd.dec;
  cmd.pl = request->cmd.pl;
  cmd.time = request->cmd.time;
  cmd.toolNum = request->cmd.tool_num;
  cmd.userNum = request->cmd.user_num;
  cmd.posidtype = request->cmd.posidtype;
  cmd.configuration = request->cmd.configuration;
  cmd.spin = request->cmd.spin;
  cmd.parasync = request->cmd.para_sync;
  cmd.targetPosValue = request->cmd.target_pos_value;

  int ret = job_insert_moveL(socket_fd_, request->line, cmd);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Job insert movel successfully" : "Failed to insert job movel";
}

void TL_Arm::handle_job_insert_imove_service(
    const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  MoveCmd cmd{};
  cmd.targetPosType = static_cast<PosType>(PosType::data);
  cmd.targetPosName = "";
  cmd.coord = static_cast<Coord>(request->cmd.coord);
  cmd.velocity = request->cmd.velocity;
  cmd.velocitySync = request->cmd.velocity_sync;
  cmd.acc = request->cmd.acc;
  cmd.dec = request->cmd.dec;
  cmd.pl = request->cmd.pl;
  cmd.time = request->cmd.time;
  cmd.toolNum = request->cmd.tool_num;
  cmd.userNum = request->cmd.user_num;
  cmd.posidtype = request->cmd.posidtype;
  cmd.configuration = request->cmd.configuration;
  cmd.spin = request->cmd.spin;
  cmd.parasync = request->cmd.para_sync;
  cmd.targetPosValue = request->cmd.target_pos_value;

  int ret = job_insert_imove(socket_fd_, request->line, cmd);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Job insert imove successfully" : "Failed to insert job imove";
}

void TL_Arm::handle_job_insert_movec_service(
    const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  MoveCmd cmd{};
  cmd.targetPosType = static_cast<PosType>(PosType::data);
  cmd.targetPosName = "";
  cmd.coord = static_cast<Coord>(request->cmd.coord);
  cmd.velocity = request->cmd.velocity;
  cmd.velocitySync = request->cmd.velocity_sync;
  cmd.acc = request->cmd.acc;
  cmd.dec = request->cmd.dec;
  cmd.pl = request->cmd.pl;
  cmd.time = request->cmd.time;
  cmd.toolNum = request->cmd.tool_num;
  cmd.userNum = request->cmd.user_num;
  cmd.posidtype = request->cmd.posidtype;
  cmd.configuration = request->cmd.configuration;
  cmd.spin = request->cmd.spin;
  cmd.parasync = request->cmd.para_sync;
  cmd.targetPosValue = request->cmd.target_pos_value;

  int ret = job_insert_moveC(socket_fd_, request->line, cmd);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Job insert movec successfully" : "Failed to insert job movec";
}

void TL_Arm::handle_set_global_pos_service(const std::shared_ptr<tl_ros2_interface::srv::SetGlobalPos::Request> request,
                                           std::shared_ptr<tl_ros2_interface::srv::SetGlobalPos::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  auto isValidGP = [](const std::string& str) -> bool
  {
    if (str.length() != 6 || str.substr(0, 2) != "GP")
    {
      return false;
    }
    int num = std::stoi(str.substr(2));
    return (num >= 1 && num <= 9999);
  };

  if (!isValidGP(request->pos_name))
  {
    response->success = false;
    response->message = "Invalid global pos name";
    return;
  }

  int ret = set_global_position(socket_fd_, request->pos_name, request->pos_info);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Set global pos successfully" : "Failed to set global pos";
}

void TL_Arm::handle_get_global_pos_service(const std::shared_ptr<tl_ros2_interface::srv::GetGlobalPos::Request> request,
                                           std::shared_ptr<tl_ros2_interface::srv::GetGlobalPos::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  auto isValidGP = [](const std::string& str) -> bool
  {
    if (str.length() != 6 || str.substr(0, 2) != "GP")
    {
      return false;
    }
    int num = std::stoi(str.substr(2));
    return (num >= 1 && num <= 9999);
  };

  if (!isValidGP(request->pos_name))
  {
    response->success = false;
    response->message = "Invalid global pos name";
    return;
  }

  std::vector<double> pos;
  int ret = get_global_position(socket_fd_, request->pos_name, pos);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Get global pos successfully" : "Failed to get global pos";
  response->pos = pos;
}

void TL_Arm::handle_set_current_mode_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetCurrentMode::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetCurrentMode::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (request->mode < 0 || request->mode > 2)
  {
    response->success = false;
    response->message = "Invalid mode";
    return;
  }

  int ret = set_current_mode(socket_fd_, request->mode);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Set current mode successfully" : "Failed to set current mode";
}

void TL_Arm::handle_get_current_mode_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetCurrentMode::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetCurrentMode::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int mode = -1;
  int ret = get_current_mode(socket_fd_, mode);
  response->mode = mode;
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Get current mode successfully" : "Failed to get current mode";
}

void TL_Arm::handle_open_servoj_service(const std::shared_ptr<tl_ros2_interface::srv::OpenServoJ::Request> request,
                                        std::shared_ptr<tl_ros2_interface::srv::OpenServoJ::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::vector<double> vmax = request->vmax;
  std::vector<double> amax = request->amax;
  std::vector<double> jmax = request->jmax;

  int ret = open_servoJ(socket_fd_aux_, vmax, amax, jmax);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "ServoJ open successfully" : "Failed to open ServoJ";
}

void TL_Arm::handle_close_servoj_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int ret = close_servoJ(socket_fd_aux_);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "ServoJ close successfully" : "Failed to close ServoJ";
}

void TL_Arm::handle_queue_motion_set_status_service(
    const std::shared_ptr<tl_ros2_interface::srv::QueueMotionSetStatus::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::QueueMotionSetStatus::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  int ret = queue_motion_set_status(socket_fd_, request->status);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Set queue motion status successfully" : "Failed to set queue motion status";

  // 关闭连续运动模式后设置为示教模式
  if (!request->status)
  {
    if (set_current_mode(socket_fd_, 0) == Result::SUCCESS)
    {
      power_off();
    }
  }
}

void TL_Arm::handle_queue_motion_movej_service(
    const std::shared_ptr<tl_ros2_interface::srv::QueueMotionMoveJ::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::QueueMotionMoveJ::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  MoveCmd cmd{};
  cmd.targetPosType = static_cast<PosType>(PosType::data);
  cmd.targetPosName = "";
  cmd.coord = static_cast<Coord>(request->cmd.coord);
  cmd.velocity = request->cmd.velocity;
  cmd.velocitySync = request->cmd.velocity_sync;
  cmd.acc = request->cmd.acc;
  cmd.dec = request->cmd.dec;
  cmd.pl = request->cmd.pl;
  cmd.time = request->cmd.time;
  cmd.toolNum = request->cmd.tool_num;
  cmd.userNum = request->cmd.user_num;
  cmd.posidtype = request->cmd.posidtype;
  cmd.configuration = request->cmd.configuration;
  cmd.spin = request->cmd.spin;
  cmd.parasync = request->cmd.para_sync;
  cmd.targetPosValue = request->cmd.target_pos_value;

  int ret = queue_motion_push_back_moveJ(socket_fd_, cmd);
  if (ret != Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to push back queue motion movej";
    return;
  }

  // 新 SDK 签名: queue_motion_send_to_controller(socketFd, int size, bool isContinue = false)
  //   size=0 表示发送全部队列指令；isContinue=true 继续排队不运动，false 立即执行
  // 旧写法把 bool 传给了 size 参数，导致 is_continue=true 时只发送第 1 条，已修正
  ret = queue_motion_send_to_controller(socket_fd_, 0, request->is_continue);
  response->success = (ret == Result::SUCCESS);
  response->message =
      response->success ? "Queue motion movej execute successfully" : "Failed to execute queue motion movej";
}

void TL_Arm::handle_queue_motion_stop_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                              std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  bool status = false;
  int ret = queue_motion_get_status(socket_fd_, status);
  if (ret != Result::SUCCESS)
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

  ret = queue_motion_stop_not_power_off(socket_fd_);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Queue motion movej stop successfully" : "Failed to stop queue motion movej";
}

void TL_Arm::handle_get_current_motor_torque_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetCurrentMotorTorque::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetCurrentMotorTorque::Response> response)
{
  (void)request;

  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::vector<int> motor_torque{};
  std::vector<int> motor_torque_sync{};
  Result ret = get_current_motor_torque(socket_fd_, motor_torque, motor_torque_sync);
  if (ret == Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get current motor torque successfully";
    response->motor_torque = motor_torque;
    response->motor_torque_sync = motor_torque_sync;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get current motor torque, ret=" + std::to_string(ret);
  }
}

void TL_Arm::handle_get_current_line_joint_speed_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetCurrentLineJointSpeed::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetCurrentLineJointSpeed::Response> response)
{
  (void)request;

  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  double line_speed = 0.0;
  std::vector<double> joint_speed{};
  std::vector<double> joint_speed_sync{};
  Result ret = get_current_line_speed_and_joint_speed(socket_fd_, line_speed, joint_speed, joint_speed_sync);
  if (ret == Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get current line joint speed successfully";
    response->line_speed = line_speed;
    response->joint_speed = joint_speed;
    response->joint_speed_sync = joint_speed_sync;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get current line joint speed, ret=" + std::to_string(ret);
  }
}

void TL_Arm::handle_movej_topic(const tl_ros2_interface::msg::MoveCommand::SharedPtr msg)
{
  if (!is_connected())
  {
    RCLCPP_WARN(this->get_logger(), "[MoveJ]: arm is not connected");
    return;
  }

  MoveCmd cmd{};
  cmd.targetPosType = static_cast<PosType>(PosType::data);
  cmd.targetPosName = "";
  cmd.coord = static_cast<Coord>(msg->coord);
  cmd.velocity = msg->velocity;
  cmd.velocitySync = msg->velocity_sync;
  cmd.acc = msg->acc;
  cmd.dec = msg->dec;
  cmd.pl = msg->pl;
  cmd.time = msg->time;
  cmd.toolNum = msg->tool_num;
  cmd.userNum = msg->user_num;
  cmd.posidtype = msg->posidtype;
  cmd.configuration = msg->configuration;
  cmd.spin = msg->spin;
  cmd.parasync = msg->para_sync;
  cmd.targetPosValue = msg->target_pos_value;

  int ret = robot_movej(socket_fd_, cmd);

  RCLCPP_INFO(this->get_logger(), "[MoveJ]: result=%s", result_to_string(ret));
}

void TL_Arm::handle_movel_topic(const tl_ros2_interface::msg::MoveCommand::SharedPtr msg)
{
  if (!is_connected())
  {
    RCLCPP_WARN(this->get_logger(), "[MoveL]: arm is not connected");
    return;
  }

  MoveCmd cmd{};
  cmd.targetPosType = static_cast<PosType>(PosType::data);
  cmd.targetPosName = "";
  cmd.coord = static_cast<Coord>(msg->coord);
  cmd.velocity = msg->velocity;
  cmd.velocitySync = msg->velocity_sync;
  cmd.acc = msg->acc;
  cmd.dec = msg->dec;
  cmd.pl = msg->pl;
  cmd.time = msg->time;
  cmd.toolNum = msg->tool_num;
  cmd.userNum = msg->user_num;
  cmd.posidtype = msg->posidtype;
  cmd.configuration = msg->configuration;
  cmd.spin = msg->spin;
  cmd.parasync = msg->para_sync;
  cmd.targetPosValue = msg->target_pos_value;

  int ret = robot_movel(socket_fd_, cmd);

  RCLCPP_INFO(this->get_logger(), "[MoveL]: result=%s", result_to_string(ret));
}

/**
 * @brief 透传模式必须先使用open_servoJ开启跟踪模式
 */
void TL_Arm::handle_set_servoj_pos_topic(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!is_connected())
  {
    RCLCPP_WARN(this->get_logger(), "[ServoJ]: arm is not connected");
    return;
  }

  std::vector<double> pos = msg->data;
  int ret = set_servoJ_pos(socket_fd_aux_, pos);
  RCLCPP_INFO(this->get_logger(), "[ServoJ]: result=%s", result_to_string(ret));
}

// ================ 四元数辅助函数 ================

std::vector<double> TL_Arm::_rpy_to_quat(const std::vector<double>& rpy)
{
  double cr = std::cos(rpy[0] * 0.5);
  double sr = std::sin(rpy[0] * 0.5);
  double cp = std::cos(rpy[1] * 0.5);
  double sp = std::sin(rpy[1] * 0.5);
  double cy = std::cos(rpy[2] * 0.5);
  double sy = std::sin(rpy[2] * 0.5);
  return {
      cr * cp * cy + sr * sp * sy, // w
      sr * cp * cy - cr * sp * sy, // x
      cr * sp * cy + sr * cp * sy, // y
      cr * cp * sy - sr * sp * cy, // z
  };
}

std::vector<double> TL_Arm::_quat_to_rpy(const std::vector<double>& q)
{
  double w = q[0], x = q[1], y = q[2], z = q[3];
  double t0 = 2.0 * (w * x + y * z);
  double t1 = 1.0 - 2.0 * (x * x + y * y);
  double rx = std::atan2(t0, t1);
  double t2 = 2.0 * (w * y - z * x);
  t2 = std::max(-1.0, std::min(1.0, t2));
  double ry = std::asin(t2);
  double t3 = 2.0 * (w * z + x * y);
  double t4 = 1.0 - 2.0 * (y * y + z * z);
  double rz = std::atan2(t3, t4);
  return {rx, ry, rz};
}

std::vector<double> TL_Arm::_quat_slerp(const std::vector<double>& q1, const std::vector<double>& q2, double t)
{
  double q2w = q2[0], q2x = q2[1], q2y = q2[2], q2z = q2[3];
  double dot = q1[0] * q2w + q1[1] * q2x + q1[2] * q2y + q1[3] * q2z;
  if (dot < 0.0)
  {
    q2w = -q2w;
    q2x = -q2x;
    q2y = -q2y;
    q2z = -q2z;
    dot = -dot;
  }
  const double DOT_THRESHOLD = 0.9995;
  if (dot > DOT_THRESHOLD)
  {
    std::vector<double> result = {
        q1[0] + t * (q2w - q1[0]),
        q1[1] + t * (q2x - q1[1]),
        q1[2] + t * (q2y - q1[2]),
        q1[3] + t * (q2z - q1[3]),
    };
    double norm =
        std::sqrt(result[0] * result[0] + result[1] * result[1] + result[2] * result[2] + result[3] * result[3]);
    for (auto& v : result)
      v /= norm;
    return result;
  }
  double theta_0 = std::acos(dot);
  double sin_theta_0 = std::sin(theta_0);
  double theta = theta_0 * t;
  double s0 = std::cos(theta) - dot * std::sin(theta) / sin_theta_0;
  double s1 = std::sin(theta) / sin_theta_0;
  return {
      s0 * q1[0] + s1 * q2w,
      s0 * q1[1] + s1 * q2x,
      s0 * q1[2] + s1 * q2y,
      s0 * q1[3] + s1 * q2z,
  };
}

// ================ ServoL 笛卡尔空间直线伺服 ================

/**
 * @brief ServoL 笛卡尔空间直线伺服运动
 * 前置条件：需先调用 open_servoj 开启关节跟踪模式
 * 收到目标笛卡尔位姿后，自动获取当前位姿，插值并 IK 转为关节角后通过 servoj 发送
 */
void TL_Arm::handle_set_servol_pos_topic(const tl_ros2_interface::msg::ServolMove::SharedPtr msg)
{
  if (!is_connected())
  {
    RCLCPP_WARN(this->get_logger(), "[ServoL]: arm is not connected");
    return;
  }

  // ========= 1. 获取当前位姿 =========
  int coord = msg->coord;
  if (coord < 1 || coord > 3)
  {
    coord = 1; // 默认基座标系
  }

  std::vector<double> current_pos;
  int ret = get_current_position(socket_fd_, coord, current_pos);
  if (ret != Result::SUCCESS || current_pos.size() < 6)
  {
    RCLCPP_ERROR(this->get_logger(), "[ServoL]: Failed to get current position");
    return;
  }

  // 当前位姿 [x, y, z, rx, ry, rz]
  double cx = current_pos[0], cy = current_pos[1], cz = current_pos[2];
  double crx = current_pos[3], cry = current_pos[4], crz = current_pos[5];

  // 目标位姿
  const auto& target_pose = msg->target_pose;
  if (target_pose.size() < 6)
  {
    RCLCPP_ERROR(this->get_logger(), "[ServoL]: target_pose must have at least 6 elements");
    return;
  }
  double tx = target_pose[0], ty = target_pose[1], tz = target_pose[2];
  double trx = target_pose[3], try_ = target_pose[4], trz = target_pose[5];

  // ========= 2. 计算插值点数 =========
  double dx = tx - cx, dy = ty - cy, dz = tz - cz;
  double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  double step_size = msg->step_size;
  if (step_size <= 0.0)
  {
    step_size = 2.0; // 默认步长 2mm
  }

  int N = std::max(1, static_cast<int>(std::ceil(dist / step_size)));
  RCLCPP_INFO(this->get_logger(), "[ServoL] dist=%.1fmm, step=%.1f, points=%d", dist, step_size, N);

  // ========= 3. 准备 IK 参数 =========
  std::vector<double> cur_quat = _rpy_to_quat({crx, cry, crz});
  std::vector<double> target_quat = _rpy_to_quat({trx, try_, trz});

  std::vector<double> ref_pos(7, 0.0);

  // ========= 4. 插值 + IK + servoj 发送 (250Hz) =========
  const auto period = std::chrono::nanoseconds(4000000); // 250Hz = 4ms
  auto next_time = std::chrono::steady_clock::now();

  for (int i = 1; i <= N; ++i)
  {
    double t = static_cast<double>(i) / static_cast<double>(N);

    // 位置线性插值
    double ix = cx + t * dx;
    double iy = cy + t * dy;
    double iz = cz + t * dz;

    // 姿态四元数 Slerp
    std::vector<double> iq = _quat_slerp(cur_quat, target_quat, t);
    std::vector<double> irpy = _quat_to_rpy(iq);

    // 构建插值位姿 (7 元素: x,y,z,rx,ry,rz,0)
    std::vector<double> interp_pos = {ix, iy, iz, irpy[0], irpy[1], irpy[2], 0.0};

    // IK: 笛卡尔(coord) -> 关节(0)
    std::vector<double> joint_pos;
    ret = get_origin_coord_to_target_coord(socket_fd_, coord, interp_pos, 0, joint_pos, 0, ref_pos);

    if (ret != Result::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(), "[ServoL] IK failed at point %d/%d, ret=%d", i, N, ret);
      next_time += period;
      continue;
    }

    // 通过 servoj 发送关节角
    ret = set_servoJ_pos(socket_fd_aux_, joint_pos);
    if (ret != Result::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(), "[ServoL] set_servoJ_pos failed at point %d/%d, ret=%d", i, N, ret);
    }

    next_time += period;
    std::this_thread::sleep_until(next_time);
  }

  // ========= 5. 记录完成 =========
  RCLCPP_INFO(this->get_logger(), "[ServoL] completed to [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f], %d points", tx, ty, tz,
              trx, try_, trz, N);
}

void TL_Arm::publish_arm_state()
{
  if (!is_connected())
  {
    RCLCPP_WARN(this->get_logger(), "[Pub]: arm is not connected");
    return;
  }

  publish_running_status();

  std::vector<double> joint_pose;
  std::vector<double> tcp_pose;

  joint_pose.clear();
  tcp_pose.clear();

  if (get_current_position(socket_fd_, 0, joint_pose) == Result::SUCCESS)
  {
    // 将角度转换为弧度(批量转换)
    const double deg_to_rad = M_PI / 180.0;
    std::transform(joint_pose.begin(), joint_pose.end(), joint_pose.begin(),
                   [deg_to_rad](double deg)
                   {
                     return deg * deg_to_rad;
                   });

    publish_joint_pose(joint_pose);
  }

  if (get_current_position(socket_fd_, 1, tcp_pose) == Result::SUCCESS)
  {
    publish_tcp_pose(tcp_pose);
  }
}

void TL_Arm::publish_joint_pose(const std::vector<double>& joint_pose)
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();
  msg.name = arm_joints_;
  switch (ndof_)
  {
    case 6:
      msg.position.assign(joint_pose.begin(), joint_pose.end() - 1);
      break;
    case 7:
      msg.position.assign(joint_pose.begin(), joint_pose.end());
      break;
  }

  joint_state_pub_->publish(msg);
}

void TL_Arm::publish_tcp_pose(const std::vector<double>& tcp_pose)
{
  // SDK 返回：位置 mm、欧拉角 度（°）→ 统一转换为 ROS 标准单位：m 和 rad
  constexpr double kMmToM = 1.0 / 1000.0;
  constexpr double kDegToRad = M_PI / 180.0;

  tl_ros2_interface::msg::CartesianPose msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";

  msg.position.x = tcp_pose[0] * kMmToM;
  msg.position.y = tcp_pose[1] * kMmToM;
  msg.position.z = tcp_pose[2] * kMmToM;

  msg.rpy.x = tcp_pose[3] * kDegToRad;
  msg.rpy.y = tcp_pose[4] * kDegToRad;
  msg.rpy.z = tcp_pose[5] * kDegToRad;

  switch (ndof_)
  {
    case 6:
      msg.arm_angle = 0;
      break;
    case 7:
      msg.arm_angle = tcp_pose[6] * kDegToRad;
      break;
  }

  tcp_pose_pub_->publish(msg);
}

void TL_Arm::publish_running_status()
{
  int running_status = -1;
  int ret = get_robot_running_state(socket_fd_, running_status);
  if (ret != Result::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "[Read Running Status]: failed to read running status, result=%s",
                result_to_string(ret));
    return;
  }

  tl_ros2_interface::msg::ArmStatus msg;

  msg.stamp = this->now();
  switch (running_status)
  {
    case 0:
      msg.run_state = "STOP";
      break;
    case 1:
      msg.run_state = "PAUSE";
      break;
    case 2:
      msg.run_state = "RUNNING";
      break;
  }

  running_status_pub_->publish(msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto thread_num = std::max(4u, std::thread::hardware_concurrency());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), thread_num, true);
  auto node = std::make_shared<TL_Arm>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
