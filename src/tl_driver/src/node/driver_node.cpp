#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <thread>

#include "node/driver_node.hpp"
#include "node/registrars.hpp"
#include "tl_driver/version.hpp"

#include "arm_client.hpp"
#include "arm_sdk.hpp"
#include "arm_session.hpp"
#include "node/publishers.hpp"

namespace tl_driver
{
DriverNode::DriverNode() : rclcpp::Node("tl_driver"), session_(this->get_logger())
{
  declare_parameters();

  create_callback_groups();

  register_entities();

  // 获取本包版本与sdk库版本
  RCLCPP_INFO(this->get_logger(), "Driver Version: %s", TL_DRIVER_VERSION);
  RCLCPP_INFO(this->get_logger(), "SDK Version: %s", tl::get_library_version().c_str());

  // 初始化
  init();
  RCLCPP_INFO(this->get_logger(), "%s_driver is running ", arm_type_.c_str());
}

void DriverNode::declare_parameters()
{
  this->declare_parameter("arm_ip", "192.168.1.13");
  this->declare_parameter("arm_port", "6001");
  this->declare_parameter("arm_port_aux", "7000");
  this->declare_parameter<std::vector<std::string>>("arm_joints",
                                                    {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
  this->declare_parameter("arm_type", "TCB605");
  this->declare_parameter("publish_rate", 100.0);

  arm_ip_ = this->get_parameter("arm_ip").as_string();
  arm_port_ = this->get_parameter("arm_port").as_string();
  arm_port_aux_ = this->get_parameter("arm_port_aux").as_string();
  arm_type_ = this->get_parameter("arm_type").as_string();
  arm_joints_ = this->get_parameter("arm_joints").as_string_array();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  ndof_ = static_cast<int>(arm_joints_.size());

  constexpr double kMinPublishRate = 1.0;
  constexpr double kMaxPublishRate = 1000.0;
  if (!std::isfinite(publish_rate_) || publish_rate_ < kMinPublishRate || publish_rate_ > kMaxPublishRate)
  {
    RCLCPP_FATAL(this->get_logger(), "publish_rate must be in [%.1f, %.1f] Hz, got %.3f", kMinPublishRate,
                 kMaxPublishRate, publish_rate_);
    rclcpp::shutdown();
    exit(EXIT_FAILURE);
  }

  constexpr int kMinJoints = 1;
  constexpr int kMaxJoints = 8;
  if (ndof_ < kMinJoints || ndof_ > kMaxJoints)
  {
    RCLCPP_FATAL(this->get_logger(), "arm_joints size must be in [%d, %d], got %d", kMinJoints, kMaxJoints, ndof_);
    rclcpp::shutdown();
    exit(EXIT_FAILURE);
  }
}

void DriverNode::create_callback_groups()
{
  // 多线程
  service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  topic_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  topic_options_.callback_group = topic_group_;

  timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

void DriverNode::register_entities()
{
  // 服务
  register_session_services(*this);
  register_motion_services(*this);
  register_transform_services(*this);
  register_config_services(*this);
  register_io_services(*this);
  register_state_services(*this);

  // 话题pub
  publishers_ = std::make_unique<tl_driver::StatePublishers>(*this, session_, arm_joints_, ndof_);

  // 话题sub
  register_topics(*this);

  create_state_timer();
}

void DriverNode::create_state_timer()
{
  // 直接使用浮点周期，避免截断到整毫秒造成系统性频率偏差（如 60 Hz → 62.5 Hz）。
  const auto period = std::chrono::duration<double>(1.0 / publish_rate_);

  state_publish_timer_ = this->create_wall_timer(
      period,
      [this]()
      {
        publishers_->publish_arm_state();
      },
      timer_group_);
}

DriverNode::~DriverNode()
{
  if (session_.is_connected())
  {
    session_.power_off();
    session_.disconnect();
  }
}

void DriverNode::init()
{
  RCLCPP_INFO(this->get_logger(), "Trying to connect to %s:%s,%s", arm_ip_.c_str(), arm_port_.c_str(),
              arm_port_aux_.c_str());

  if (session_.connect(arm_ip_, arm_port_, arm_port_aux_))
  {
    // init() 在 spin() 之前执行，此处取得的 fd 在本次初始化中不会变化。
    const int fd = session_.fd();

    // 切换到示教模式
    tl::Result ret = tl::set_current_mode(fd, tl::RobotMode::TEACH);
    if (ret != tl::Result::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "[Init]: failed to set teach mode, result=%s",
                   arm_client::result_to_string(ret));
      rclcpp::shutdown();
      exit(EXIT_FAILURE);
    }

    // 清除控制器错误和报警
    ret = tl::clear_error(fd);
    if (ret != tl::Result::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(), "[Init]: clear_error result=%s", arm_client::result_to_string(ret));
    }

    // 清错后下电，释放控制器占用状态
    session_.power_off();

    session_.power_on();
    RCLCPP_INFO(this->get_logger(), "上电延时2s...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(this->get_logger(), "上电延时完成");
  }
  else
  {
    rclcpp::shutdown();
    exit(EXIT_FAILURE);
  }
}
} // namespace tl_driver
