#include <cmath>
#include <thread>

#include "node/driver_node.h"
#include "node/services_state.h"
#include "node/services_config.h"
#include "node/services_transform.h"
#include "node/services_motion.h"
#include "tl_driver/version.hpp"

#include "arm_sdk.hpp"
#include "arm_session.h"
#include "node/publishers.h"
#include "convert/msg_convert.h"
#include "math/quaternion.h"

namespace tl_driver
{
DriverNode::DriverNode() : rclcpp::Node("tl_driver")
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

  // 会话与状态发布
  session_ = std::make_unique<tl_driver::ArmSession>(this->get_logger(), arm_ip_, arm_port_, arm_port_aux_);
  publishers_ = std::make_unique<tl_driver::Publishers>(*this, *session_, arm_joints_, ndof_);

  // 多线程
  service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  topic_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto topic_group_option = rclcpp::SubscriptionOptions();
  topic_group_option.callback_group = topic_group_;

  timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // 服务
  services::register_state_services(*this, service_group_);
  services::register_config_services(*this, service_group_);
  services::register_transform_services(*this, service_group_);
  services::register_motion_services(*this, service_group_);


  log_download_service_ = this->create_service<tl_ros2_interface::srv::LogDownload>(
      "/tl_driver/log_download",
      std::bind(&DriverNode::handle_log_download_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);


  set_digital_output_service_ = this->create_service<tl_ros2_interface::srv::SetDigitalOutput>(
      "/tl_driver/set_digital_output",
      std::bind(&DriverNode::handle_set_digital_output_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  get_digital_input_output_service_ = this->create_service<tl_ros2_interface::srv::GetDigitalInputOutput>(
      "/tl_driver/get_digital_input_output",
      std::bind(&DriverNode::handle_get_digital_input_output_service, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  modbus_write_service_ = this->create_service<tl_ros2_interface::srv::ModbusWrite>(
      "/tl_driver/modbus_write",
      std::bind(&DriverNode::handle_modbus_write_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  modbus_read_service_ = this->create_service<tl_ros2_interface::srv::ModbusRead>(
      "/tl_driver/modbus_read",
      std::bind(&DriverNode::handle_modbus_read_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);


  get_all_job_filename_service_ = this->create_service<tl_ros2_interface::srv::GetAllJobFileName>(
      "/tl_driver/get_all_job_filename",
      std::bind(&DriverNode::handle_get_all_job_filename_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  job_run_service_ = this->create_service<tl_ros2_interface::srv::JobRun>(
      "/tl_driver/job_run",
      std::bind(&DriverNode::handle_job_run_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  job_delete_service_ = this->create_service<tl_ros2_interface::srv::JobRun>(
      "/tl_driver/job_delete",
      std::bind(&DriverNode::handle_job_delete_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  job_insert_movej_service_ = this->create_service<tl_ros2_interface::srv::JobInsertMove>(
      "/tl_driver/job_insert_moveJ",
      std::bind(&DriverNode::handle_job_insert_movej_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  job_insert_movel_service_ = this->create_service<tl_ros2_interface::srv::JobInsertMove>(
      "/tl_driver/job_insert_moveL",
      std::bind(&DriverNode::handle_job_insert_movel_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  job_insert_imove_service_ = this->create_service<tl_ros2_interface::srv::JobInsertMove>(
      "/tl_driver/job_insert_imove",
      std::bind(&DriverNode::handle_job_insert_imove_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);

  job_insert_movec_service_ = this->create_service<tl_ros2_interface::srv::JobInsertMove>(
      "/tl_driver/job_insert_moveC",
      std::bind(&DriverNode::handle_job_insert_movec_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);


  // 话题sub
  movej_sub_ = this->create_subscription<tl_ros2_interface::msg::MoveCommand>(
      "/tl_driver/moveJ", 10,
      [this](const tl_ros2_interface::msg::MoveCommand::SharedPtr msg)
      {
        services::handle_movej_topic(*this, msg);
      },
      topic_group_option);

  movel_sub_ = this->create_subscription<tl_ros2_interface::msg::MoveCommand>(
      "/tl_driver/moveL", 10,
      [this](const tl_ros2_interface::msg::MoveCommand::SharedPtr msg)
      {
        services::handle_movel_topic(*this, msg);
      },
      topic_group_option);

  set_servoj_pos_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/tl_driver/set_servoj_pos", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
      {
        services::handle_set_servoj_pos_topic(*this, msg);
      },
      topic_group_option);

  set_servol_pos_sub_ = this->create_subscription<tl_ros2_interface::msg::ServolMove>(
      "/tl_driver/set_servol_pos", 10,
      [this](const tl_ros2_interface::msg::ServolMove::SharedPtr msg)
      {
        services::handle_set_servol_pos_topic(*this, msg);
      },
      topic_group_option);

  auto period =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / publish_rate_));

  state_publish_timer_ = this->create_wall_timer(
      period,
      [this]()
      {
        publishers_->publish_arm_state();
      },
      timer_group_);

  // 获取本包版本与sdk库版本
  RCLCPP_INFO(this->get_logger(), "Driver Version: %s", TL_DRIVER_VERSION);
  RCLCPP_INFO(this->get_logger(), "SDK Version: %s", tl::get_library_version().c_str());

  // 初始化
  init();
  RCLCPP_INFO(this->get_logger(), "%s_driver is running ", arm_type_.c_str());
}

DriverNode::~DriverNode()
{
  if (session_->is_connected())
  {
    session_->power_off();
    session_->disconnect();
  }
}

void DriverNode::init()
{
  RCLCPP_INFO(this->get_logger(), "Trying to connect to %s:%s,%s", arm_ip_.c_str(), arm_port_.c_str(),
              arm_port_aux_.c_str());

  if (session_->connect())
  {
    // 切换到示教模式
    tl::Result ret = tl::set_current_mode(session_->fd(), tl::RobotMode::TEACH);
    if (ret != tl::Result::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "[Init]: failed to set teach mode, result=%s", tl_driver::result_to_string(ret));
      rclcpp::shutdown();
      exit(0);
    }

    // 清除控制器错误和报警
    ret = tl::clear_error(session_->fd());
    if (ret != tl::Result::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(), "[Init]: clear_error result=%s", tl_driver::result_to_string(ret));
    }

    // 清错后下电，释放控制器占用状态
    session_->power_off();

    session_->power_on();
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


void DriverNode::handle_log_download_service(
    const std::shared_ptr<tl_ros2_interface::srv::LogDownload::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::LogDownload::Response> response)
{
  if (!session_->is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::log_download_by_quantity(session_->fd(), request->count, request->directory_path);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Log download successfully" : "Failed to download log";
}


void DriverNode::handle_set_digital_output_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetDigitalOutput::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetDigitalOutput::Response> response)
{
  if (!session_->is_connected())
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

  tl::Result ret = tl::set_digital_output(session_->fd(), request->port, static_cast<tl::IoLevel>(request->value));
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Set digital output successfully" : "Failed to set digital output";
}

void DriverNode::handle_get_digital_input_output_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetDigitalInputOutput::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetDigitalInputOutput::Response> response)
{
  (void)request;
  if (!session_->is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::vector<int> digitalInput;
  std::vector<int> digitalOutput;
  tl::Result ret = tl::get_digital_input(session_->fd(), digitalInput);
  tl::Result ret1 = tl::get_digital_output(session_->fd(), digitalOutput);
  if (ret == tl::Result::SUCCESS && ret1 == tl::Result::SUCCESS)
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

void DriverNode::handle_modbus_write_service(
    const std::shared_ptr<tl_ros2_interface::srv::ModbusWrite::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::ModbusWrite::Response> response)
{
  if (!session_->is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::ModbusMasterParameter master_param;
  if (!tl_driver::convert::to_modbus_master_param(request->master_param, master_param))
  {
    response->success = false;
    response->message = "Invalid master type";
    return;
  }

  tl::Result ret = tl::modbus_set_master_parameter(session_->fd(), request->master_id, master_param);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to set master paramter";
    return;
  }

  ret = tl::modbus_open_master(session_->fd(), request->master_id);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to open master";
    return;
  }

  std::vector<int> data = request->data;
  ret = tl::modbus_write_multiple_holding_registers(session_->fd(), request->master_id, request->addr, data);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Modbus write successfully" : "Failed to write Modbus";
}

void DriverNode::handle_modbus_read_service(const std::shared_ptr<tl_ros2_interface::srv::ModbusRead::Request> request,
                                            std::shared_ptr<tl_ros2_interface::srv::ModbusRead::Response> response)
{
  if (!session_->is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::ModbusMasterParameter master_param;
  if (!tl_driver::convert::to_modbus_master_param(request->master_param, master_param))
  {
    response->success = false;
    response->message = "Invalid master type";
    return;
  }

  tl::Result ret = tl::modbus_set_master_parameter(session_->fd(), request->master_id, master_param);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to set master paramter";
    return;
  }

  ret = tl::modbus_open_master(session_->fd(), request->master_id);
  if (ret != tl::Result::SUCCESS)
  {
    response->success = false;
    response->message = "Failed to open master";
    return;
  }

  std::vector<int> data{};
  ret = tl::modbus_read_holding_registers(session_->fd(), request->master_id, request->addr, request->quantity, data);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Modbus read successfully" : "Failed to read Modbus";
  response->data = data;
}


void DriverNode::handle_get_all_job_filename_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetAllJobFileName::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetAllJobFileName::Response> response)
{
  (void)request;
  if (!session_->is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  std::vector<std::vector<std::string>> robotsFile;
  tl::Result ret = tl::job_get_all_jobfile_name(session_->fd(), robotsFile);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Get all job filename successfully" : "Failed to get all job filename";
  response->robots_file.clear();
  for (const auto& file_names : robotsFile)
  {
    tl_ros2_interface::msg::JobFileName msg;
    msg.file_name = file_names;
    response->robots_file.push_back(msg);
  }
}

void DriverNode::handle_job_run_service(const std::shared_ptr<tl_ros2_interface::srv::JobRun::Request> request,
                                        std::shared_ptr<tl_ros2_interface::srv::JobRun::Response> response)
{
  if (!session_->is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::job_run(session_->fd(), request->job_name);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Job run successfully" : "Failed to run job";

  // // 运行完成后会自动下电
  // session_->power_off();
  // session_->power_on();
}

void DriverNode::handle_job_delete_service(const std::shared_ptr<tl_ros2_interface::srv::JobRun::Request> request,
                                           std::shared_ptr<tl_ros2_interface::srv::JobRun::Response> response)
{
  if (!session_->is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::Result ret = tl::job_delete(session_->fd(), request->job_name);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Job delete successfully" : "Failed to delete job";
}

void DriverNode::handle_job_insert_movej_service(
    const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response)
{
  if (!session_->is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::MoveCmd cmd = tl_driver::convert::to_move_cmd(request->cmd);

  tl::Result ret = tl::job_insert_moveJ(session_->fd(), request->line, cmd);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Job insert movej successfully" : "Failed to insert job movej";
}

void DriverNode::handle_job_insert_movel_service(
    const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response)
{
  if (!session_->is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::MoveCmd cmd = tl_driver::convert::to_move_cmd(request->cmd);

  tl::Result ret = tl::job_insert_moveL(session_->fd(), request->line, cmd);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Job insert movel successfully" : "Failed to insert job movel";
}

void DriverNode::handle_job_insert_imove_service(
    const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response)
{
  if (!session_->is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::MoveCmd cmd = tl_driver::convert::to_move_cmd(request->cmd);

  tl::Result ret = tl::job_insert_imove(session_->fd(), request->line, cmd);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Job insert imove successfully" : "Failed to insert job imove";
}

void DriverNode::handle_job_insert_movec_service(
    const std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::JobInsertMove::Response> response)
{
  if (!session_->is_connected())
  {
    response->success = false;
    response->message = "Arm is not connected";
    return;
  }

  tl::MoveCmd cmd = tl_driver::convert::to_move_cmd(request->cmd);

  tl::Result ret = tl::job_insert_moveC(session_->fd(), request->line, cmd);
  response->success = (ret == tl::Result::SUCCESS);
  response->message = response->success ? "Job insert movec successfully" : "Failed to insert job movec";
}


/**
 * @brief 透传模式必须先使用open_servoJ开启跟踪模式
 */

// ================ ServoL 笛卡尔空间直线伺服 ================

/**
 * @brief ServoL 笛卡尔空间直线伺服运动
 * 前置条件：需先调用 open_servoj 开启关节跟踪模式
 * 收到目标笛卡尔位姿后，自动获取当前位姿，插值并 IK 转为关节角后通过 servoj 发送
 */
} // namespace tl_driver
