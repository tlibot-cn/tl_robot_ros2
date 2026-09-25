#include <cmath>
#include <thread>

#include "node/driver_node.h"
#include "node/services_state.h"
#include "node/services_config.h"
#include "node/services_transform.h"
#include "node/services_motion.h"
#include "node/services_io.h"
#include "node/services_job.h"
#include "tl_driver/version.hpp"

#include "arm_client.hpp"
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
  services::register_io_services(*this, service_group_);
  services::register_job_services(*this, service_group_);


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
      RCLCPP_ERROR(this->get_logger(), "[Init]: failed to set teach mode, result=%s",
                   arm_client::result_to_string(ret));
      rclcpp::shutdown();
      exit(0);
    }

    // 清除控制器错误和报警
    ret = tl::clear_error(session_->fd());
    if (ret != tl::Result::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(), "[Init]: clear_error result=%s", arm_client::result_to_string(ret));
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
} // namespace tl_driver
