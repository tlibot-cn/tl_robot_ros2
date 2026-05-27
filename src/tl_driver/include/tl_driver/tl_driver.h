#ifndef TL_DRIVER__TL_DRIVER_H_
#define TL_DRIVER__TL_DRIVER_H_

// common
#include <algorithm>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <cstdint>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// msg
#include "tl_ros2_interface/msg/arm_status.hpp"
#include "tl_ros2_interface/msg/move_command.hpp"
#include "tl_ros2_interface/msg/cartesian_pose.hpp"
#include "tl_ros2_interface/msg/tool_param.hpp"
#include "tl_ros2_interface/msg/modbus_tcp_param.hpp"
#include "tl_ros2_interface/msg/modbus_rtu_param.hpp"
#include "tl_ros2_interface/msg/modbus_master_param.hpp"
#include "tl_ros2_interface/msg/robot_dh_param.hpp"
#include "tl_ros2_interface/msg/job_file_name.hpp"
#include "tl_ros2_interface/msg/robot_joint_param.hpp"

// srv
#include "tl_ros2_interface/srv/get_robot_state.hpp"
#include "tl_ros2_interface/srv/get_robot_joint_param.hpp"
#include "tl_ros2_interface/srv/set_robot_joint_param.hpp"
#include "tl_ros2_interface/srv/get_joint_temperature.hpp"
#include "tl_ros2_interface/srv/get_joint_voltage.hpp"
#include "tl_ros2_interface/srv/get_motor_current.hpp"
#include "tl_ros2_interface/srv/get_joint_software_version.hpp"
#include "tl_ros2_interface/srv/restore_default_dh_param.hpp"
#include "tl_ros2_interface/srv/log_download.hpp"
#include "tl_ros2_interface/srv/set_speed.hpp"
#include "tl_ros2_interface/srv/get_speed.hpp"
#include "tl_ros2_interface/srv/get_pos_transform.hpp"
#include "tl_ros2_interface/srv/set_controller_ip.hpp"
#include "tl_ros2_interface/srv/jogging.hpp"
#include "tl_ros2_interface/srv/set_drag_mode.hpp"
#include "tl_ros2_interface/srv/track_save.hpp"
#include "tl_ros2_interface/srv/track_playback.hpp"
#include "tl_ros2_interface/srv/set_tool_param.hpp"
#include "tl_ros2_interface/srv/set_user_coord.hpp"
#include "tl_ros2_interface/srv/set_axis_zero_pos.hpp"
#include "tl_ros2_interface/srv/set_current_coord.hpp"
#include "tl_ros2_interface/srv/get_current_coord.hpp"
#include "tl_ros2_interface/srv/set_coord_num.hpp"
#include "tl_ros2_interface/srv/get_coord_num.hpp"
#include "tl_ros2_interface/srv/set_digital_output.hpp"
#include "tl_ros2_interface/srv/get_digital_input_output.hpp"
#include "tl_ros2_interface/srv/modbus_write.hpp"
#include "tl_ros2_interface/srv/modbus_read.hpp"
#include "tl_ros2_interface/srv/coord_transform.hpp"
#include "tl_ros2_interface/srv/get_pos_reachable.hpp"
#include "tl_ros2_interface/srv/set_dh_param.hpp"
#include "tl_ros2_interface/srv/get_dh_param.hpp"
#include "tl_ros2_interface/srv/get_all_job_file_name.hpp"
#include "tl_ros2_interface/srv/job_run.hpp"
#include "tl_ros2_interface/srv/job_insert_move.hpp"
#include "tl_ros2_interface/srv/set_global_pos.hpp"
#include "tl_ros2_interface/srv/get_global_pos.hpp"
#include "tl_ros2_interface/srv/open_servo_j.hpp"
#include "tl_ros2_interface/srv/set_current_mode.hpp"
#include "tl_ros2_interface/srv/queue_motion_set_status.hpp"
#include "tl_ros2_interface/srv/queue_motion_move_j.hpp"

// lib
#include "tl_interface.h"
#include "tl_job_operate.h"
#include "tl_track.h"
#include "tl_io.h"
#include "tl_modbus.h"
#include "tl_queue_operate.h"

enum MessageLists
{
  ROBOT_STATE = 7681
};

class TL_Arm : public rclcpp::Node
{
public:
  TL_Arm();
  ~TL_Arm();

  bool connect();
  bool disconnect();
  bool power_on();
  bool power_off();
  bool is_connected();
  bool is_powered();
  void init();

  // 服务
  void handle_connect_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_disconnect_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_poweron_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_poweroff_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_clear_error_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_set_speed_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetSpeed::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetSpeed::Response> response);
  
  void handle_get_speed_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetSpeed::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetSpeed::Response> response);

  void handle_get_rpy2quat_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response);

  void handle_get_quat2rpy_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response);

  void handle_get_rpy2r_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response);

  void handle_get_tr2r_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response);

  void handle_get_r2tr_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetPosTransform::Response> response);

  void handle_set_controller_ip_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetControllerIP::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetControllerIP::Response> response);

  void handle_get_controller_id_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_start_jogging_service(
      const std::shared_ptr<tl_ros2_interface::srv::Jogging::Request> request,
      std::shared_ptr<tl_ros2_interface::srv::Jogging::Response> response);
  
  void handle_stop_jogging_service(
    const std::shared_ptr<tl_ros2_interface::srv::Jogging::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::Jogging::Response> response);
  
  void handle_get_robot_state_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetRobotState::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetRobotState::Response> response);

  void handle_get_library_version_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_get_robot_joint_param_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetRobotJointParam::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetRobotJointParam::Response> response);
  
  void handle_set_robot_joint_param_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetRobotJointParam::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetRobotJointParam::Response> response);

  void handle_get_joint_temperature_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetJointTemperature::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetJointTemperature::Response> response);
  
  void handle_get_joint_voltage_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetJointVoltage::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetJointVoltage::Response> response);

  void handle_get_motor_current_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetMotorCurrent::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetMotorCurrent::Response> response);

  void handle_get_joint_software_version_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetJointSoftwareVersion::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetJointSoftwareVersion::Response> response);

  void handle_get_nexmotion_lib_version_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_restore_default_dh_param_service(
    const std::shared_ptr<tl_ros2_interface::srv::RestoreDefaultDHParam::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::RestoreDefaultDHParam::Response> response);

  void handle_set_default_cartesian_param_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_log_download_service(
    const std::shared_ptr<tl_ros2_interface::srv::LogDownload::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::LogDownload::Response> response);

  void handle_set_drag_mode_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetDragMode::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetDragMode::Response> response);

  void handle_get_drag_status_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_track_save_service(
    const std::shared_ptr<tl_ros2_interface::srv::TrackSave::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::TrackSave::Response> response);

  void handle_track_playback_service(
    const std::shared_ptr<tl_ros2_interface::srv::TrackPlayback::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::TrackPlayback::Response> response);

  void handle_set_tool_param_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetToolParam::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetToolParam::Response> response);

  void handle_set_user_coord_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetUserCoord::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetUserCoord::Response> response);

  void handle_set_axis_zero_pos_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetAxisZeroPos::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetAxisZeroPos::Response> response);

  void handle_set_current_coord_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetCurrentCoord::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetCurrentCoord::Response> response);

  void handle_get_current_coord_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetCurrentCoord::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetCurrentCoord::Response> response);

  void handle_set_coord_num_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetCoordNum::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetCoordNum::Response> response);

  void handle_get_coord_num_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetCoordNum::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetCoordNum::Response> response);

  void handle_set_digital_output_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetDigitalOutput::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetDigitalOutput::Response> response);

  void handle_get_digital_input_output_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetDigitalInputOutput::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetDigitalInputOutput::Response> response);
  
  void handle_modbus_write_service(
    const std::shared_ptr<tl_ros2_interface::srv::ModbusWrite::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::ModbusWrite::Response> response);

  void handle_modbus_read_service(
    const std::shared_ptr<tl_ros2_interface::srv::ModbusRead::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::ModbusRead::Response> response);
  
  void handle_coord_transform_service(
    const std::shared_ptr<tl_ros2_interface::srv::CoordTransform::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::CoordTransform::Response> response);

  void handle_get_pos_reachable_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetPosReachable::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetPosReachable::Response> response);

  void handle_set_dh_param_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetDHParam::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetDHParam::Response> response);

  void handle_get_dh_param_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetDHParam::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetDHParam::Response> response);

  void handle_get_all_job_filename_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetAllJobFileName::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetAllJobFileName::Response> response);

  void handle_job_run_service(
    const std::shared_ptr<tl_ros2_interface::srv::JobRun::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::JobRun::Response> response);

  void handle_job_delete_service(
    const std::shared_ptr<tl_ros2_interface::srv::JobRun::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::JobRun::Response> response);

  void handle_job_insert_movej_service(
    const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response);

  void handle_job_insert_movel_service(
    const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response);

  void handle_job_insert_imove_service(
    const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response);

  void handle_job_insert_movec_service(
    const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response);

  void handle_set_global_pos_service( 
    const std::shared_ptr<tl_ros2_interface::srv::SetGlobalPos::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetGlobalPos::Response> response);

  void handle_get_global_pos_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetGlobalPos::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetGlobalPos::Response> response);

  void handle_set_current_mode_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetCurrentMode::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetCurrentMode::Response> response);

  void handle_open_servoj_service(
    const std::shared_ptr<tl_ros2_interface::srv::OpenServoJ::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::OpenServoJ::Response> response);

  void handle_close_servoj_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_queue_motion_set_status_service(
    const std::shared_ptr<tl_ros2_interface::srv::QueueMotionSetStatus::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::QueueMotionSetStatus::Response> response);

  void handle_queue_motion_movej_service(
    const std::shared_ptr<tl_ros2_interface::srv::QueueMotionMoveJ::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::QueueMotionMoveJ::Response> response);
  
  void handle_queue_motion_stop_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // 话题
  void handle_movej_topic(
    const tl_ros2_interface::msg::MoveCommand::SharedPtr msg);

  void handle_movel_topic(
    const tl_ros2_interface::msg::MoveCommand::SharedPtr msg);

  void handle_set_servoj_pos_topic(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  void publish_arm_state();
  void publish_joint_pose(const std::vector<double> & joint_pose);
  void publish_tcp_pose(const std::vector<double> & tcp_pose);
  void publish_running_status();

private:
  std::string arm_ip_;
  std::string arm_port_;
  std::string arm_port_aux_;
  std::string arm_type_;

  int socket_fd_ {0};
  int socket_fd_aux_ {0};
  bool is_connected_ {false};           // 机械臂是否连接
  bool is_powered_ {false};             // 机械臂是否上电(示教模式)

  std::vector<std::string> arm_joints_;
  double publish_rate_ {100.0};
  int ndof_ {6};

  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr topic_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disconnect_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr poweron_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr poweroff_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_error_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetSpeed>::SharedPtr set_speed_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetSpeed>::SharedPtr get_speed_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetPosTransform>::SharedPtr get_quat2rpy_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetPosTransform>::SharedPtr get_rpy2quat_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetPosTransform>::SharedPtr get_rpy2r_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetPosTransform>::SharedPtr get_tr2r_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetPosTransform>::SharedPtr get_r2tr_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetControllerIP>::SharedPtr set_controller_ip_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_controller_id_service_;
  rclcpp::Service<tl_ros2_interface::srv::Jogging>::SharedPtr start_jogging_service_;
  rclcpp::Service<tl_ros2_interface::srv::Jogging>::SharedPtr stop_jogging_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetRobotState>::SharedPtr get_robot_state_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_library_version_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetRobotJointParam>::SharedPtr get_robot_joint_param_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetRobotJointParam>::SharedPtr set_robot_joint_param_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetJointTemperature>::SharedPtr get_joint_temperature_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetJointVoltage>::SharedPtr get_joint_voltage_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetMotorCurrent>::SharedPtr get_motor_current_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetJointSoftwareVersion>::SharedPtr get_joint_software_version_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_nexmotion_lib_version_service_;
  rclcpp::Service<tl_ros2_interface::srv::RestoreDefaultDHParam>::SharedPtr restore_default_dh_param_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_default_cartesian_param_service_;
  rclcpp::Service<tl_ros2_interface::srv::LogDownload>::SharedPtr log_download_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetDragMode>::SharedPtr set_drag_mode_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_drag_status_service_;
  rclcpp::Service<tl_ros2_interface::srv::TrackSave>::SharedPtr track_save_service_;
  rclcpp::Service<tl_ros2_interface::srv::TrackPlayback>::SharedPtr track_playback_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetToolParam>::SharedPtr set_tool_param_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetUserCoord>::SharedPtr set_user_coord_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetAxisZeroPos>::SharedPtr set_axis_zero_pos_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetCurrentCoord>::SharedPtr set_current_coord_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetCurrentCoord>::SharedPtr get_current_coord_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetCoordNum>::SharedPtr set_coord_num_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetCoordNum>::SharedPtr get_coord_num_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetDigitalOutput>::SharedPtr set_digital_output_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetDigitalInputOutput>::SharedPtr get_digital_input_output_service_;
  rclcpp::Service<tl_ros2_interface::srv::ModbusWrite>::SharedPtr modbus_write_service_;
  rclcpp::Service<tl_ros2_interface::srv::ModbusRead>::SharedPtr modbus_read_service_;
  rclcpp::Service<tl_ros2_interface::srv::CoordTransform>::SharedPtr coord_transform_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetPosReachable>::SharedPtr get_pos_reachable_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetDHParam>::SharedPtr set_dh_param_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetDHParam>::SharedPtr get_dh_param_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetAllJobFileName>::SharedPtr get_all_job_filename_service_;
  rclcpp::Service<tl_ros2_interface::srv::JobRun>::SharedPtr job_run_service_;
  rclcpp::Service<tl_ros2_interface::srv::JobInsertMove>::SharedPtr job_insert_movej_service_;
  rclcpp::Service<tl_ros2_interface::srv::JobInsertMove>::SharedPtr job_insert_movel_service_;
  rclcpp::Service<tl_ros2_interface::srv::JobInsertMove>::SharedPtr job_insert_imove_service_;
  rclcpp::Service<tl_ros2_interface::srv::JobInsertMove>::SharedPtr job_insert_movec_service_;
  rclcpp::Service<tl_ros2_interface::srv::JobRun>::SharedPtr job_delete_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetGlobalPos>::SharedPtr set_global_pos_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetGlobalPos>::SharedPtr get_global_pos_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetCurrentMode>::SharedPtr set_current_mode_service_;
  rclcpp::Service<tl_ros2_interface::srv::OpenServoJ>::SharedPtr open_servoj_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr close_servoj_service_;
  rclcpp::Service<tl_ros2_interface::srv::QueueMotionSetStatus>::SharedPtr queue_motion_set_status_service_;
  rclcpp::Service<tl_ros2_interface::srv::QueueMotionMoveJ>::SharedPtr queue_motion_movej_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr queue_motion_stop_service_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::CartesianPose>::SharedPtr tcp_pose_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::ArmStatus>::SharedPtr running_status_pub_;

  rclcpp::Subscription<tl_ros2_interface::msg::MoveCommand>::SharedPtr movej_sub_;
  rclcpp::Subscription<tl_ros2_interface::msg::MoveCommand>::SharedPtr movel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr set_servoj_pos_sub_;

  rclcpp::TimerBase::SharedPtr state_publish_timer_;
};

#endif  // TL_DRIVER__TL_DRIVER_H_