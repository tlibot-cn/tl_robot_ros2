#include "tl_driver/tl_driver.h"

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

// 普通 C 风格函数指针回调
void robot_state_recv_callback(int msg_id, const char* msg)
{
  {
    std::lock_guard<std::mutex> lock(g_robot_state_msg_buffer.mutex);

    g_robot_state_msg_buffer.last_msg_id = msg_id;
    g_robot_state_msg_buffer.last_msg = msg ? msg : "";
    ++g_robot_state_msg_buffer.seq;
  }

  g_robot_state_msg_buffer.cv.notify_all();

  std::cout << "\033[32m"
            << "id = " << msg_id
            << ", msg = " << (msg ? msg : "")
            << "\033[0m" << std::endl;
}

void receive_error_or_warning_message_callback(int messageType, const char* message, int messageCode)
{
  std::cout << "\033[31m" << "messageType = " << messageType <<  ", message = " 
             << message << ", messageCode = " << messageCode << "\033[0m" << std::endl;
}

const char * result_to_string(int ret)
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
}  // namespace


TL_Arm::TL_Arm()
: rclcpp::Node("tl_driver")
{
  this->declare_parameter("arm_ip", "192.168.1.13");
  this->declare_parameter("arm_port", "6001");
  this->declare_parameter("arm_port_aux", "7000");
  this->declare_parameter<std::vector<std::string>>(
    "arm_joints",
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

  timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  
  // 服务
  connect_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/tl_driver/connect_arm",
    std::bind(
      &TL_Arm::handle_connect_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  disconnect_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/tl_driver/disconnect_arm",
    std::bind(
      &TL_Arm::handle_disconnect_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  poweron_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/tl_driver/power_on",
    std::bind(
      &TL_Arm::handle_poweron_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  poweroff_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/tl_driver/power_off",
    std::bind(
      &TL_Arm::handle_poweroff_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );
    
  clear_error_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/tl_driver/clear_error",
    std::bind(
      &TL_Arm::handle_clear_error_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );
  
  set_speed_service_ = this->create_service<tl_ros2_interface::srv::SetSpeed>(
    "/tl_driver/set_speed",
    std::bind(
      &TL_Arm::handle_set_speed_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  get_speed_service_ = this->create_service<tl_ros2_interface::srv::GetSpeed>(
    "/tl_driver/get_speed",
    std::bind(
      &TL_Arm::handle_get_speed_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );
  
  get_quat2rpy_service_ = this->create_service<tl_ros2_interface::srv::GetPosTransform>(
    "/tl_driver/get_quat2rpy",
    std::bind(
      &TL_Arm::handle_get_quat2rpy_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  get_rpy2quat_service_ = this->create_service<tl_ros2_interface::srv::GetPosTransform>(
    "/tl_driver/get_rpy2quat",
    std::bind(
      &TL_Arm::handle_get_rpy2quat_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  get_rpy2r_service_ = this->create_service<tl_ros2_interface::srv::GetPosTransform>(
    "/tl_driver/get_rpy2r",
    std::bind(
      &TL_Arm::handle_get_rpy2r_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  get_tr2r_service_ = this->create_service<tl_ros2_interface::srv::GetPosTransform>(
    "/tl_driver/get_tr2r",
    std::bind(
      &TL_Arm::handle_get_tr2r_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  get_r2tr_service_ = this->create_service<tl_ros2_interface::srv::GetPosTransform>(
    "/tl_driver/get_r2tr",
    std::bind(
      &TL_Arm::handle_get_r2tr_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );
  
  set_controller_ip_service_ = this->create_service<tl_ros2_interface::srv::SetControllerIP>(
    "/tl_driver/set_controller_ip",
    std::bind(
      &TL_Arm::handle_set_controller_ip_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  get_controller_id_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/tl_driver/get_controller_id",
    std::bind(
      &TL_Arm::handle_get_controller_id_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  start_jogging_service_ = this->create_service<tl_ros2_interface::srv::Jogging>(
    "/tl_driver/start_jogging",
    std::bind(
      &TL_Arm::handle_start_jogging_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  stop_jogging_service_ = this->create_service<tl_ros2_interface::srv::Jogging>(
    "/tl_driver/stop_jogging",
    std::bind(
      &TL_Arm::handle_stop_jogging_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  get_robot_state_service_ = this->create_service<tl_ros2_interface::srv::GetRobotState>(
    "/tl_driver/get_robot_state",
    std::bind(
      &TL_Arm::handle_get_robot_state_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  get_library_version_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/tl_driver/get_library_version",
    std::bind(
      &TL_Arm::handle_get_library_version_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  get_robot_joint_param_service_ = this->create_service<tl_ros2_interface::srv::GetRobotJointParam>(
    "/tl_driver/get_robot_joint_param",
    std::bind(
      &TL_Arm::handle_get_robot_joint_param_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );
  
  set_robot_joint_param_service_ = this->create_service<tl_ros2_interface::srv::SetRobotJointParam>(
    "/tl_driver/set_robot_joint_param",
    std::bind(
      &TL_Arm::handle_set_robot_joint_param_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  get_joint_temperature_service_ = this->create_service<tl_ros2_interface::srv::GetJointTemperature>(
    "/tl_driver/get_joint_temperature",
    std::bind(
      &TL_Arm::handle_get_joint_temperature_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  get_joint_voltage_service_ = this->create_service<tl_ros2_interface::srv::GetJointVoltage>(
    "/tl_driver/get_joint_voltage",
    std::bind(
      &TL_Arm::handle_get_joint_voltage_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  get_motor_current_service_ = this->create_service<tl_ros2_interface::srv::GetMotorCurrent>(
    "/tl_driver/get_motor_current",
    std::bind(
      &TL_Arm::handle_get_motor_current_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  get_joint_software_version_service_ = this->create_service<tl_ros2_interface::srv::GetJointSoftwareVersion>(
    "/tl_driver/get_joint_software_version",
    std::bind(
      &TL_Arm::handle_get_joint_software_version_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  get_nexmotion_lib_version_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/tl_driver/get_nexmotion_lib_version",
    std::bind(
      &TL_Arm::handle_get_nexmotion_lib_version_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  restore_default_dh_param_service_ = this->create_service<tl_ros2_interface::srv::RestoreDefaultDHParam>(
    "/tl_driver/restore_default_dh_param",
    std::bind(
      &TL_Arm::handle_restore_default_dh_param_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );
  
  set_default_cartesian_param_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/tl_driver/set_default_cartesian_param",
    std::bind(
      &TL_Arm::handle_set_default_cartesian_param_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  log_download_service_ = this->create_service<tl_ros2_interface::srv::LogDownload>(
    "/tl_driver/log_download",
    std::bind(
      &TL_Arm::handle_log_download_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );
  
  set_drag_mode_service_ = this->create_service<tl_ros2_interface::srv::SetDragMode>(
    "/tl_driver/set_drag_mode",
    std::bind(
      &TL_Arm::handle_set_drag_mode_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );
  
  get_drag_status_service_ = this->create_service<std_srvs::srv::Trigger>(     // 用不了
    "/tl_driver/get_drag_status",
    std::bind(
      &TL_Arm::handle_get_drag_status_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );
  
  track_save_service_ = this->create_service<tl_ros2_interface::srv::TrackSave>(
    "/tl_driver/track_save",
    std::bind(
      &TL_Arm::handle_track_save_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  track_playback_service_ = this->create_service<tl_ros2_interface::srv::TrackPlayback>(
    "/tl_driver/track_playback",
    std::bind(
      &TL_Arm::handle_track_playback_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  set_tool_param_service_ = this->create_service<tl_ros2_interface::srv::SetToolParam>(
    "/tl_driver/set_tool_param",
    std::bind(
      &TL_Arm::handle_set_tool_param_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  set_user_coord_service_ = this->create_service<tl_ros2_interface::srv::SetUserCoord>(
    "/tl_driver/set_user_coord",
    std::bind(
      &TL_Arm::handle_set_user_coord_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  set_axis_zero_pos_service_ = this->create_service<tl_ros2_interface::srv::SetAxisZeroPos>(
    "/tl_driver/set_axis_zero_pos",
    std::bind(
      &TL_Arm::handle_set_axis_zero_pos_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  set_current_coord_service_ = this->create_service<tl_ros2_interface::srv::SetCurrentCoord>(
    "/tl_driver/set_current_coord",
    std::bind(
      &TL_Arm::handle_set_current_coord_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );
  
  get_current_coord_service_ = this->create_service<tl_ros2_interface::srv::GetCurrentCoord>(
    "/tl_driver/get_current_coord",
    std::bind(
      &TL_Arm::handle_get_current_coord_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );    

  set_coord_num_service_ = this->create_service<tl_ros2_interface::srv::SetCoordNum>( 
    "/tl_driver/set_coord_num",
    std::bind(
      &TL_Arm::handle_set_coord_num_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );
  
  get_coord_num_service_ = this->create_service<tl_ros2_interface::srv::GetCoordNum>(
    "/tl_driver/get_coord_num",
    std::bind(
      &TL_Arm::handle_get_coord_num_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  set_digital_output_service_ = this->create_service<tl_ros2_interface::srv::SetDigitalOutput>(
    "/tl_driver/set_digital_output",
    std::bind(
      &TL_Arm::handle_set_digital_output_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  get_digital_input_output_service_ = this->create_service<tl_ros2_interface::srv::GetDigitalInputOutput>(
    "/tl_driver/get_digital_input_output",
    std::bind(
      &TL_Arm::handle_get_digital_input_output_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  modbus_write_service_ = this->create_service<tl_ros2_interface::srv::ModbusWrite>(
    "/tl_driver/modbus_write",
    std::bind(
      &TL_Arm::handle_modbus_write_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );
  
  modbus_read_service_ = this->create_service<tl_ros2_interface::srv::ModbusRead>(
    "/tl_driver/modbus_read",
    std::bind(
      &TL_Arm::handle_modbus_read_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  coord_transform_service_ = this->create_service<tl_ros2_interface::srv::CoordTransform>(
    "/tl_driver/coord_transform",
    std::bind(
      &TL_Arm::handle_coord_transform_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  get_pos_reachable_service_ = this->create_service<tl_ros2_interface::srv::GetPosReachable>(
    "/tl_driver/get_pos_reachable",
    std::bind(
      &TL_Arm::handle_get_pos_reachable_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );
  
  set_dh_param_service_ = this->create_service<tl_ros2_interface::srv::SetDHParam>(
    "/tl_driver/set_dh_param",
    std::bind(
      &TL_Arm::handle_set_dh_param_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  get_dh_param_service_ = this->create_service<tl_ros2_interface::srv::GetDHParam>(
    "/tl_driver/get_dh_param",
    std::bind(
      &TL_Arm::handle_get_dh_param_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );
  
  get_all_job_filename_service_ = this->create_service<tl_ros2_interface::srv::GetAllJobFileName>(
    "/tl_driver/get_all_job_filename",
    std::bind(
      &TL_Arm::handle_get_all_job_filename_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  job_run_service_ = this->create_service<tl_ros2_interface::srv::JobRun>(
    "/tl_driver/job_run",
    std::bind(
      &TL_Arm::handle_job_run_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );  
    
  job_delete_service_ = this->create_service<tl_ros2_interface::srv::JobRun>(
    "/tl_driver/job_delete",
    std::bind(
      &TL_Arm::handle_job_delete_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  job_insert_movej_service_ = this->create_service<tl_ros2_interface::srv::JobInsertMove>(
    "/tl_driver/job_insert_moveJ",
    std::bind(
      &TL_Arm::handle_job_insert_movej_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  job_insert_movel_service_ = this->create_service<tl_ros2_interface::srv::JobInsertMove>(
    "/tl_driver/job_insert_moveL",
    std::bind(
      &TL_Arm::handle_job_insert_movel_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  job_insert_imove_service_ = this->create_service<tl_ros2_interface::srv::JobInsertMove>(
    "/tl_driver/job_insert_imove",
    std::bind(
      &TL_Arm::handle_job_insert_imove_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  job_insert_movec_service_ = this->create_service<tl_ros2_interface::srv::JobInsertMove>(
    "/tl_driver/job_insert_moveC",
    std::bind(
      &TL_Arm::handle_job_insert_movec_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );
  
  set_global_pos_service_ = this->create_service<tl_ros2_interface::srv::SetGlobalPos>(
    "/tl_driver/set_global_pos",
    std::bind(
      &TL_Arm::handle_set_global_pos_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  get_global_pos_service_ = this->create_service<tl_ros2_interface::srv::GetGlobalPos>(
    "/tl_driver/get_global_pos",
    std::bind(
      &TL_Arm::handle_get_global_pos_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );
  
  set_current_mode_service_ = this->create_service<tl_ros2_interface::srv::SetCurrentMode>(
    "/tl_driver/set_current_mode",
    std::bind(
      &TL_Arm::handle_set_current_mode_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  open_servoj_service_ = this->create_service<tl_ros2_interface::srv::OpenServoJ>(
    "/tl_driver/open_servoj",
    std::bind(
      &TL_Arm::handle_open_servoj_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  close_servoj_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/tl_driver/close_servoj",
    std::bind(
      &TL_Arm::handle_close_servoj_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );
  
  queue_motion_set_status_service_ = this->create_service<tl_ros2_interface::srv::QueueMotionSetStatus>(
    "/tl_driver/queue_motion_set_status",
    std::bind(
      &TL_Arm::handle_queue_motion_set_status_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  queue_motion_movej_service_ = this->create_service<tl_ros2_interface::srv::QueueMotionMoveJ>(
    "/tl_driver/queue_motion_movej",
    std::bind(
      &TL_Arm::handle_queue_motion_movej_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );

  queue_motion_stop_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/tl_driver/queue_motion_stop",
    std::bind(
      &TL_Arm::handle_queue_motion_stop_service,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_
  );
  
  // 话题pub
  joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  tcp_pose_pub_ =
    this->create_publisher<tl_ros2_interface::msg::CartesianPose>("/tcp_pose", 10);

  running_status_pub_ =
    this->create_publisher<tl_ros2_interface::msg::ArmStatus>("/arm_status", 10);
  
  // 话题sub
  movej_sub_ =
    this->create_subscription<tl_ros2_interface::msg::MoveCommand>(
    "/tl_driver/moveJ",
    10,
    std::bind(&TL_Arm::handle_movej_topic, this, std::placeholders::_1),
    topic_group_option
  );

  movel_sub_ =
    this->create_subscription<tl_ros2_interface::msg::MoveCommand>(
    "/tl_driver/moveL",
    10,
    std::bind(&TL_Arm::handle_movel_topic, this, std::placeholders::_1),
    topic_group_option
  );
  
  set_servoj_pos_sub_ = 
    this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/tl_driver/set_servoj_pos",
    10,
    std::bind(&TL_Arm::handle_set_servoj_pos_topic, this, std::placeholders::_1),
    topic_group_option
  );

  auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_));

  state_publish_timer_ =
    this->create_wall_timer(
      period, 
      std::bind(&TL_Arm::publish_arm_state, this),
      timer_group_
  );
  
  // 初始化
  init();
  RCLCPP_INFO (this->get_logger(),"%s_driver is running ",arm_type_.c_str());
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
  RCLCPP_INFO(this->get_logger(), "Trying to connect to %s:%s,%s", arm_ip_.c_str(), arm_port_.c_str(), arm_port_aux_.c_str());
  if (connect())
  {
    power_on();
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
      set_servo_poweron(socket_fd_);
      break;
    case 1:
      set_servo_poweron(socket_fd_);
      break;
    case 2:
      clear_error(socket_fd_);
      set_servo_state(socket_fd_, 1);
      set_servo_poweron(socket_fd_);
      break;
    case 3:
      RCLCPP_INFO(this->get_logger(), "[PowerOn]: already power on");
      return true;
  }

  get_servo_state(socket_fd_, state);
  if (state == 3)
  {
    is_powered_ = true;
    RCLCPP_INFO(this->get_logger(), "[PowerOn]: successfully power on, servo_state = %d", state);
    return true;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "[PowerOn]: failed to power on, servo_state = %d", state);
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

  set_receive_error_or_warnning_message_callback(
    socket_fd_,
    receive_error_or_warning_message_callback);

  set_receive_error_or_warnning_message_callback(
    socket_fd_aux_,
    receive_error_or_warning_message_callback);
  
  recv_message(socket_fd_aux_, robot_state_recv_callback);

  RCLCPP_INFO(
    this->get_logger(),
    "[Connect]: successfully connected to arm at %s:%s,%s", arm_ip_.c_str(), arm_port_.c_str(), arm_port_aux_.c_str());

  return true;
}

bool TL_Arm::disconnect()
{
  if (!is_connected_)
  {
    RCLCPP_INFO(this->get_logger(), "[Disconnect]: arm already disconnected");
    return true;
  }

  disconnect_robot(socket_fd_);
  disconnect_robot(socket_fd_aux_);

  socket_fd_ = 0;
  socket_fd_aux_ = 0;
  is_connected_ = false;

  return true;
}

void TL_Arm::handle_connect_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
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

void TL_Arm::handle_disconnect_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
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

void TL_Arm::handle_poweron_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  if (is_powered())
  {
    response->success = true;
    response->message = "Arm already power on";
    return;
  }

  response->success = power_on();
  response->message = response->success ? "Arm power on successfully" : "Failed to power on arm";
}

void TL_Arm::handle_poweroff_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
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

void TL_Arm::handle_clear_error_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
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

  if (state == 2)
  {
    int ret = clear_error(socket_fd_);
    response->success = (ret == Result::SUCCESS);
    response->message = response->success ?
      "Clear error successfully" :
      std::string("Clear error failed: ") + result_to_string(ret);
  }
  else
  {
    response->success = false;
    response->message = "Not an error state";
  }
}

void TL_Arm::handle_set_speed_service(
  const std::shared_ptr<tl_ros2_interface::srv::SetSpeed::Request> request,
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

  int ret = set_speed(socket_fd_,request->speed);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Set speed successfully" : "Failed to set speed";
}

void TL_Arm::handle_get_speed_service(
  const std::shared_ptr<tl_ros2_interface::srv::GetSpeed::Request> request,
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
  int ret = get_speed(socket_fd_,speed);
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

void TL_Arm::handle_get_rpy2r_service(
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

void TL_Arm::handle_get_tr2r_service(
  const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
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

void TL_Arm::handle_get_r2tr_service(
  const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
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

void TL_Arm::handle_get_controller_id_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  char id[128] = {0};
  int ret = get_controller_id(socket_fd_, id);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? std::string(id) : "Failed to get controller ID";
}

void TL_Arm::handle_start_jogging_service(
  const std::shared_ptr<tl_ros2_interface::srv::Jogging::Request> request,
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

void TL_Arm::handle_stop_jogging_service(                      
  const std::shared_ptr<tl_ros2_interface::srv::Jogging::Request> request,
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

  RobotState param {};
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

  uint64_t start_seq = 0;
  {
    std::lock_guard<std::mutex> lock(g_robot_state_msg_buffer.mutex);
    start_seq = g_robot_state_msg_buffer.seq;
  }

  int ret = get_robot_state(socket_fd_, param);
  if (ret != Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to get robot state";
    return;
  }

  std::unique_lock<std::mutex> lock(g_robot_state_msg_buffer.mutex);

  bool received = g_robot_state_msg_buffer.cv.wait_for(
    lock,
    std::chrono::seconds(5),
    [start_seq]() {
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

void TL_Arm::handle_get_library_version_service(                      
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
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

  RobotJointParam param {};
  int ret = get_robot_joint_param(socket_fd_, request->id, param);
  if (ret == Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get robot joint param successfully";
    response->param.reduction_ratio = param.reducRatio;
    response->param.encoder_resolution = param.encoderResolution;
    response->param.pos_sw_limit = param.posSWLimit;
    response->param.neg_sw_limit = param.negSWLimit;
    response->param.rated_rot_speed = param.ratedRotSpeed;
    response->param.rated_derot_speed = param.ratedDeRotSpeed;
    response->param.max_rot_speed = param.maxRotSpeed;
    response->param.max_derot_speed = param.maxDeRotSpeed;
    response->param.rated_vel = param.ratedVel;
    response->param.rated_devel = param.deRatedVel;
    response->param.max_acc = param.maxAcc;
    response->param.max_dec = param.maxDecel;
    response->param.direction = param.direction;
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

  RobotJointParam param {};
  param.reducRatio = request->param.reduction_ratio;
  param.encoderResolution = request->param.encoder_resolution;
  param.posSWLimit = request->param.pos_sw_limit;
  param.negSWLimit = request->param.neg_sw_limit;
  param.ratedRotSpeed = request->param.rated_rot_speed;
  param.ratedDeRotSpeed = request->param.rated_derot_speed;
  param.maxRotSpeed = request->param.max_rot_speed;
  param.maxDeRotSpeed = request->param.max_derot_speed;
  param.ratedVel = request->param.rated_vel;
  param.deRatedVel = request->param.rated_devel;
  param.maxAcc = request->param.max_acc;
  param.maxDecel = request->param.max_dec;
  param.direction = request->param.direction;

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

  std::vector<double> motor_current;
  int ret = get_current_motor_current_independent(socket_fd_, motor_current);
  if (ret == Result::SUCCESS)
  {
    response->success = true;
    response->message = "Get motor current successfully";
    response->motor_current = motor_current;
  }
  else
  {
    response->success = false;
    response->message = "Failed to get motor current";
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

void TL_Arm::handle_get_nexmotion_lib_version_service(                      
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
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
  int ret = get_nexmotion_lib_version(socket_fd_, version);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? version : "Failed to get nexmotion lib version";
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
  response->message = response->success ? "Restore default DH param successfully" : "Failed to restore default DH param";
}

void TL_Arm::handle_set_default_cartesian_param_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
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
  response->message = response->success ? "Set default cartesian param successfully" : "Failed to set default cartesian param";
}

void TL_Arm::handle_log_download_service(
  const std::shared_ptr<tl_ros2_interface::srv::LogDownload::Request> request,
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

void TL_Arm::handle_set_drag_mode_service(
  const std::shared_ptr<tl_ros2_interface::srv::SetDragMode::Request> request,
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

void TL_Arm::handle_get_drag_status_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
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

void TL_Arm::handle_track_save_service(
  const std::shared_ptr<tl_ros2_interface::srv::TrackSave::Request> request,
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

void TL_Arm::handle_set_tool_param_service(
  const std::shared_ptr<tl_ros2_interface::srv::SetToolParam::Request> request,
  std::shared_ptr<tl_ros2_interface::srv::SetToolParam::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  ToolParam param {};
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

void TL_Arm::handle_set_user_coord_service(
  const std::shared_ptr<tl_ros2_interface::srv::SetUserCoord::Request> request,
  std::shared_ptr<tl_ros2_interface::srv::SetUserCoord::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::vector<double> pos = {
      request->pos.position.x,
      request->pos.position.y,
      request->pos.position.z,
      request->pos.rpy.x,
      request->pos.rpy.y,
      request->pos.rpy.z
  };

  int ret = set_user_coordinate_data(socket_fd_, request->user_num, pos);
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

void TL_Arm::handle_set_coord_num_service(
  const std::shared_ptr<tl_ros2_interface::srv::SetCoordNum::Request> request,
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

void TL_Arm::handle_get_coord_num_service(
  const std::shared_ptr<tl_ros2_interface::srv::GetCoordNum::Request> request,
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

void TL_Arm::handle_modbus_write_service(
  const std::shared_ptr<tl_ros2_interface::srv::ModbusWrite::Request> request,
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

void TL_Arm::handle_modbus_read_service(
  const std::shared_ptr<tl_ros2_interface::srv::ModbusRead::Request> request,
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

  std::vector<int> data;
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

  if (request->origin_coord < 0 || request->origin_coord >3)
  {
    response->success = false;
    response->message = "Invalid origin coordinate";
    return;
  }

  if (request->target_coord < 0 || request->target_coord >3)
  {
    response->success = false;
    response->message = "Invalid target coordinate";
    return;
  }

  std::vector<double> originPos = request->origin_pos;
  std::vector<double> referencePos = request->reference_pos;
  std::vector<double> targetPos;

  int ret = get_origin_coord_to_target_coord(socket_fd_, request->origin_coord, originPos, 
                                              request->target_coord, targetPos, request->form, referencePos);
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
  int ret = get_pos_reachable(socket_fd_, queryPos, request->move_type, result);
  if (ret == Result::SUCCESS)
  {
    response->success = result;
    response->message = response->success ? "Target pos is reachable" : "Target pos is not reachable"; 
  }
  else
  {
    response->success = false;
    response->message = "Fail to get pos reachable status"; 
  }
}

void TL_Arm::handle_set_dh_param_service(
  const std::shared_ptr<tl_ros2_interface::srv::SetDHParam::Request> request,
  std::shared_ptr<tl_ros2_interface::srv::SetDHParam::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  RobotDHParam dh_param {};
  dh_param.L1 = request->param.l1;
  dh_param.L2 = request->param.l2;
  dh_param.L3 = request->param.l3;
  dh_param.L4 = request->param.l4;
  dh_param.L5 = request->param.l5;
  dh_param.L6 = request->param.l6;
  dh_param.L7 = request->param.l7;
  dh_param.L8 = request->param.l8;
  dh_param.L9 = request->param.l9;
  dh_param.L10 = request->param.l10;
  dh_param.L11 = request->param.l11;
  dh_param.L12 = request->param.l12;
  dh_param.L13 = request->param.l13;
  dh_param.L14 = request->param.l14;
  dh_param.L15 = request->param.l15;
  dh_param.L16 = request->param.l16;
  dh_param.L17 = request->param.l17;
  dh_param.L18 = request->param.l18;
  dh_param.L19 = request->param.l19;
  dh_param.L20 = request->param.l20;

  dh_param.Couple_Coe_1_2 = request->param.couple_coe_1_2;
  dh_param.Couple_Coe_2_3 = request->param.couple_coe_2_3;
  dh_param.Couple_Coe_3_2 = request->param.couple_coe_3_2;
  dh_param.Couple_Coe_3_4 = request->param.couple_coe_3_4;
  dh_param.Couple_Coe_4_5 = request->param.couple_coe_4_5;
  dh_param.Couple_Coe_4_6 = request->param.couple_coe_4_6;
  dh_param.Couple_Coe_5_6 = request->param.couple_coe_5_6;

  dh_param.dynamicLimit_max = request->param.dynamic_limit_max;
  dh_param.dynamicLimit_min = request->param.dynamic_limit_max;

  dh_param.pitch = request->param.pitch;
  dh_param.sliding_lead_value = request->param.sliding_lead_value;
  dh_param.uplift_lead_value = request->param.uplift_lead_value;
  dh_param.spray_distance = request->param.spray_distance;

  dh_param.threeAxisDirection = request->param.three_axis_direction;
  dh_param.fiveAxisDirection = request->param.five_axis_direction;

  dh_param.twoAxisConversionRatio = request->param.two_axis_convertion_ratio;
  dh_param.threeAxisConversionRatio = request->param.three_axis_convertion_ratio;
  dh_param.amplificationRatio = request->param.amplification_ratio;

  dh_param.conversionratio_x = request->param.convertion_ratio_x;
  dh_param.conversionratio_y = request->param.convertion_ratio_y;
  dh_param.conversionratio_z = request->param.convertion_ratio_z;

  dh_param.conversionratio_J1 = request->param.convertion_ratio_j1;
  dh_param.conversionratio_J2 = request->param.convertion_ratio_j2;
  dh_param.conversionratio_J3 = request->param.convertion_ratio_j3;
  dh_param.upsideDown = request->param.upside_down;
  dh_param.hanyu.PC = request->param.pc;

  for (size_t i = 0; i < 3; i++) 
  {
      dh_param.hanyu.SP[i] = (i < request->param.sp.size()) ? request->param.sp[i] : 0.0;
      dh_param.hanyu.TL[i] = (i < request->param.tl.size()) ? request->param.tl[i] : 0.0;
  }

  int ret = set_robot_dh_param(socket_fd_, dh_param);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Set DH param successfully" : "Failed to set DH param";

  // 设置参数后机械臂会下电
  power_off();
  power_on();
}

void TL_Arm::handle_get_dh_param_service(
  const std::shared_ptr<tl_ros2_interface::srv::GetDHParam::Request> request,
  std::shared_ptr<tl_ros2_interface::srv::GetDHParam::Response> response)
{
  (void)request;
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  RobotDHParam dh_param {};
  int ret = get_robot_dh_param(socket_fd_, dh_param);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Get DH param successfully" : "Failed to get DH param";
  
  response->param.l1 = dh_param.L1;
  response->param.l2 = dh_param.L2;
  response->param.l3 = dh_param.L3;
  response->param.l4 = dh_param.L4;
  response->param.l5 = dh_param.L5;
  response->param.l6 = dh_param.L6;
  response->param.l7 = dh_param.L7;
  response->param.l8 = dh_param.L8;
  response->param.l9 = dh_param.L9;
  response->param.l10 = dh_param.L10;
  response->param.l11 = dh_param.L11;
  response->param.l12 = dh_param.L12;
  response->param.l13 = dh_param.L13;
  response->param.l14 = dh_param.L14;
  response->param.l15 = dh_param.L15;
  response->param.l16 = dh_param.L16;
  response->param.l17 = dh_param.L17;
  response->param.l18 = dh_param.L18;
  response->param.l19 = dh_param.L19;
  response->param.l20 = dh_param.L20;

  response->param.couple_coe_1_2 = dh_param.Couple_Coe_1_2;
  response->param.couple_coe_2_3 = dh_param.Couple_Coe_2_3;
  response->param.couple_coe_3_2 = dh_param.Couple_Coe_3_2;
  response->param.couple_coe_3_4 = dh_param.Couple_Coe_3_4;
  response->param.couple_coe_4_5 = dh_param.Couple_Coe_4_5;
  response->param.couple_coe_4_6 = dh_param.Couple_Coe_4_6;
  response->param.couple_coe_5_6 = dh_param.Couple_Coe_5_6;

  response->param.dynamic_limit_max = dh_param.dynamicLimit_max;
  response->param.dynamic_limit_min = dh_param.dynamicLimit_min;

  response->param.pitch = dh_param.pitch;
  response->param.sliding_lead_value = dh_param.sliding_lead_value;
  response->param.uplift_lead_value = dh_param.uplift_lead_value;
  response->param.spray_distance = dh_param.spray_distance;

  response->param.three_axis_direction = dh_param.threeAxisDirection;
  response->param.five_axis_direction = dh_param.fiveAxisDirection;

  response->param.two_axis_convertion_ratio = dh_param.twoAxisConversionRatio;
  response->param.three_axis_convertion_ratio = dh_param.threeAxisConversionRatio;
  response->param.amplification_ratio = dh_param.amplificationRatio;

  response->param.convertion_ratio_x = dh_param.conversionratio_x;
  response->param.convertion_ratio_y = dh_param.conversionratio_y;
  response->param.convertion_ratio_z = dh_param.conversionratio_z;

  response->param.convertion_ratio_j1 = dh_param.conversionratio_J1;
  response->param.convertion_ratio_j2 = dh_param.conversionratio_J2;
  response->param.convertion_ratio_j3 = dh_param.conversionratio_J3;

  response->param.upside_down = dh_param.upsideDown;
  response->param.pc = dh_param.hanyu.PC;
  response->param.sp.assign(dh_param.hanyu.SP, dh_param.hanyu.SP + 3);
  response->param.tl.assign(dh_param.hanyu.TL, dh_param.hanyu.TL + 3);
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
  for (const auto& file_names : robotsFile) {
    tl_ros2_interface::msg::JobFileName msg;
    msg.file_name = file_names;
    response->robots_file.push_back(msg);
  }
}

void TL_Arm::handle_job_run_service(
  const std::shared_ptr<tl_ros2_interface::srv::JobRun::Request> request,
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

void TL_Arm::handle_job_delete_service(
  const std::shared_ptr<tl_ros2_interface::srv::JobRun::Request> request,
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

  MoveCmd cmd {};
  cmd.targetPosType = static_cast<PosType>(PosType::data);
  cmd.targetPosName = "";
  cmd.coord = request->cmd.coord;
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

  MoveCmd cmd {};
  cmd.targetPosType = static_cast<PosType>(PosType::data);
  cmd.targetPosName = "";
  cmd.coord = request->cmd.coord;
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

  MoveCmd cmd {};
  cmd.targetPosType = static_cast<PosType>(PosType::data);
  cmd.targetPosName = "";
  cmd.coord = request->cmd.coord;
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

  MoveCmd cmd {};
  cmd.targetPosType = static_cast<PosType>(PosType::data);
  cmd.targetPosName = "";
  cmd.coord = request->cmd.coord;
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

void TL_Arm::handle_set_global_pos_service( 
  const std::shared_ptr<tl_ros2_interface::srv::SetGlobalPos::Request> request,
  std::shared_ptr<tl_ros2_interface::srv::SetGlobalPos::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  auto isValidGP = [](const std::string& str) -> bool {
    if (str.length() != 6 || str.substr(0, 2) != "GP") {
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

void TL_Arm::handle_get_global_pos_service(
  const std::shared_ptr<tl_ros2_interface::srv::GetGlobalPos::Request> request,
  std::shared_ptr<tl_ros2_interface::srv::GetGlobalPos::Response> response)
{
  if (!is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  auto isValidGP = [](const std::string& str) -> bool {
    if (str.length() != 6 || str.substr(0, 2) != "GP") {
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

void TL_Arm::handle_open_servoj_service(
  const std::shared_ptr<tl_ros2_interface::srv::OpenServoJ::Request> request,
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

void TL_Arm::handle_close_servoj_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
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

  MoveCmd cmd {};
  cmd.targetPosType = static_cast<PosType>(PosType::data);
  cmd.targetPosName = "";
  cmd.coord = request->cmd.coord;
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

  ret = queue_motion_send_to_controller(socket_fd_, request->is_continue);
  response->success = (ret == Result::SUCCESS);
  response->message = response->success ? "Queue motion movej execute successfully" : "Failed to execute queue motion movej";
}

void TL_Arm::handle_queue_motion_stop_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
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

void TL_Arm::handle_movej_topic(
  const tl_ros2_interface::msg::MoveCommand::SharedPtr msg)
{
  if (!is_connected())
  {
    RCLCPP_WARN(this->get_logger(), "[MoveJ]: arm is not connected");
    return;
  }

  MoveCmd cmd {};
  cmd.targetPosType = static_cast<PosType>(PosType::data);
  cmd.targetPosName = "";
  cmd.coord = msg->coord;
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

void TL_Arm::handle_movel_topic(
  const tl_ros2_interface::msg::MoveCommand::SharedPtr msg)
{
  if (!is_connected())
  {
    RCLCPP_WARN(this->get_logger(), "[MoveL]: arm is not connected");
    return;
  }

  MoveCmd cmd {};
  cmd.targetPosType = static_cast<PosType>(PosType::data);
  cmd.targetPosName = "";
  cmd.coord = msg->coord;
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
void TL_Arm::handle_set_servoj_pos_topic(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
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
    std::transform(joint_pose.begin(), joint_pose.end(),
                  joint_pose.begin(),
                  [deg_to_rad](double deg) { return deg * deg_to_rad; });

    publish_joint_pose(joint_pose);
  }

  if (get_current_position(socket_fd_, 1, tcp_pose) == Result::SUCCESS)
  {
    publish_tcp_pose(tcp_pose);
  }
}

void TL_Arm::publish_joint_pose(const std::vector<double> & joint_pose)
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();
  msg.name = arm_joints_;
  switch (ndof_)
  {
    case 6:
      msg.position.assign(joint_pose.begin(), joint_pose.end()-1);
      break;
    case 7:
      msg.position.assign(joint_pose.begin(), joint_pose.end());
      break;
  }
  
  joint_state_pub_->publish(msg);
}

void TL_Arm::publish_tcp_pose(const std::vector<double> & tcp_pose)
{
  tl_ros2_interface::msg::CartesianPose msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";

  msg.position.x = tcp_pose[0];
  msg.position.y = tcp_pose[1];
  msg.position.z = tcp_pose[2];

  msg.rpy.x = tcp_pose[3];
  msg.rpy.y = tcp_pose[4];
  msg.rpy.z = tcp_pose[5];

  switch (ndof_)
  {
    case 6:
      msg.arm_angle = 0;
      break;
    case 7:
      msg.arm_angle = tcp_pose[6];
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
    RCLCPP_INFO(this->get_logger(), "[Read Running Status]: failed to read running status, result=%s", result_to_string(ret));
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto thread_num = std::max(4u, std::thread::hardware_concurrency());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),thread_num,true);
  auto node = std::make_shared<TL_Arm>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
