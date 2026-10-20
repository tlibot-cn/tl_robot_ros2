#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
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
namespace
{
constexpr double kMinPublishRate = 1.0;
constexpr double kMaxPublishRate = 1000.0;

bool is_valid_publish_rate(double rate)
{
  return std::isfinite(rate) && rate >= kMinPublishRate && rate <= kMaxPublishRate;
}
} // namespace

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

  if (!is_valid_publish_rate(publish_rate_))
  {
    throw std::invalid_argument("publish_rate must be in [1.0, 1000.0] Hz");
  }

  constexpr int kMinJoints = 1;
  constexpr int kMaxJoints = 8;
  if (ndof_ < kMinJoints || ndof_ > kMaxJoints)
  {
    throw std::invalid_argument("arm_joints size must be in [1, 8]");
  }

  publish_rate_param_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters)
      {
        return on_parameters_set(parameters);
      });
}

rcl_interfaces::msg::SetParametersResult DriverNode::on_parameters_set(const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& parameter : parameters)
  {
    if (parameter.get_name() != "publish_rate")
    {
      continue;
    }

    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      result.successful = false;
      result.reason = "publish_rate must be a double";
      return result;
    }

    const double rate = parameter.as_double();
    if (!is_valid_publish_rate(rate))
    {
      result.successful = false;
      result.reason = "publish_rate must be in [1.0, 1000.0] Hz";
      return result;
    }

    if (rate != publish_rate_)
    {
      try
      {
        // 先建新定时器，成功后再提交频率并取消旧定时器；失败时旧频率继续生效。
        create_state_timer(rate);
      }
      catch (const std::exception& e)
      {
        result.successful = false;
        result.reason = std::string("failed to recreate timer: ") + e.what();
        return result;
      }
      publish_rate_ = rate;
      RCLCPP_INFO(this->get_logger(), "publish_rate updated to %.3f Hz", rate);
    }
  }

  return result;
}

void DriverNode::create_callback_groups()
{
  // 服务按域分组：同域内串行，跨域可并行（MultiThreadedExecutor）。
  session_service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  motion_service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  transform_service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  config_service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  io_service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  state_service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // get_robot_state 等待状态帧（mode=1 最长 interval+1 s），单独成组，避免阻塞其它服务。
  robot_state_service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  topic_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  topic_options_.callback_group = topic_group_;

  timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

void DriverNode::select_service_group(const rclcpp::CallbackGroup::SharedPtr& group)
{
  service_group_ = group;
}

void DriverNode::register_entities()
{
  // 服务：每个域注册到各自的互斥回调组。
  select_service_group(session_service_group_);
  register_session_services(*this);
  select_service_group(motion_service_group_);
  register_motion_services(*this);
  select_service_group(transform_service_group_);
  register_transform_services(*this);
  select_service_group(config_service_group_);
  register_config_services(*this);
  select_service_group(io_service_group_);
  register_io_services(*this);
  select_service_group(state_service_group_);
  register_state_services(*this);
  select_service_group(robot_state_service_group_);
  register_robot_state_service(*this);
  service_group_.reset(); // 注册结束，后续 create_service 未显式选组会抛异常

  // 话题pub
  publishers_ = std::make_unique<tl_driver::StatePublishers>(*this, session_, arm_joints_, ndof_);

  // 话题sub
  register_topics(*this);

  create_state_timer(publish_rate_);
}

void DriverNode::create_state_timer(double rate)
{
  // 直接使用浮点周期，避免截断到整毫秒造成系统性频率偏差（如 60 Hz → 62.5 Hz）。
  const auto period = std::chrono::duration<double>(1.0 / rate);

  // 先创建新定时器，成功后再替换并取消旧定时器；创建失败时旧频率继续生效。
  auto new_timer = this->create_wall_timer(
      period,
      [this]()
      {
        publishers_->publish_arm_state();
      },
      timer_group_);

  auto old_timer = std::move(state_publish_timer_);
  state_publish_timer_ = std::move(new_timer);
  if (old_timer)
  {
    old_timer->cancel();
  }
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
      throw std::runtime_error(std::string("[Init]: failed to set teach mode, result=") +
                               arm_client::result_to_string(ret));
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
    return;
  }
  throw std::runtime_error("failed to connect to " + arm_ip_ + ":" + arm_port_ + "," + arm_port_aux_);
}
} // namespace tl_driver
