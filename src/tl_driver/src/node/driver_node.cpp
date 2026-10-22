#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "node/actions_motion.hpp"
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
constexpr int kMinStreamIntervalMs = 10;
constexpr int kMaxStreamIntervalMs = 60000;
constexpr double kMaxAuxRate = 100.0;
constexpr double kMaxThermalRate = 1.0;
// 运行状态只用于 /arm_status 缓存；10 Hz 足够覆盖 STOP/PAUSE/RUNNING 切换，且不再占用 100 Hz 周期。
constexpr double kRunningStatusRate = 10.0;
// 运行速度只用于 /arm_status.speed；1 Hz 足够反映 SetSpeed 后的变化，避免增加 6001 查询负载。
constexpr double kSpeedQueryRate = 1.0;
constexpr double kMaxActionTimeoutSec = 3600.0;

bool is_valid_publish_rate(double rate)
{
  return std::isfinite(rate) && rate >= kMinPublishRate && rate <= kMaxPublishRate;
}

bool is_valid_rate(double rate, double max_rate)
{
  return std::isfinite(rate) && rate >= 0.0 && rate <= max_rate;
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
  this->declare_parameter("state_source", "stream");
  this->declare_parameter("state_stream_interval_ms", 10);
  this->declare_parameter("joint_dynamics_rate", 10.0);
  this->declare_parameter("joint_thermal_rate", 1.0);
  this->declare_parameter("motion_action_timeout", 60.0);
  this->declare_parameter("track_action_timeout", 600.0);

  arm_ip_ = this->get_parameter("arm_ip").as_string();
  arm_port_ = this->get_parameter("arm_port").as_string();
  arm_port_aux_ = this->get_parameter("arm_port_aux").as_string();
  arm_type_ = this->get_parameter("arm_type").as_string();
  arm_joints_ = this->get_parameter("arm_joints").as_string_array();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  state_source_ = this->get_parameter("state_source").as_string();
  state_stream_interval_ms_ = this->get_parameter("state_stream_interval_ms").as_int();
  joint_dynamics_rate_ = this->get_parameter("joint_dynamics_rate").as_double();
  joint_thermal_rate_ = this->get_parameter("joint_thermal_rate").as_double();
  motion_action_timeout_ = this->get_parameter("motion_action_timeout").as_double();
  track_action_timeout_ = this->get_parameter("track_action_timeout").as_double();
  ndof_ = static_cast<int>(arm_joints_.size());

  if (!is_valid_publish_rate(publish_rate_))
  {
    throw std::invalid_argument("publish_rate must be in [1.0, 1000.0] Hz");
  }
  if (state_source_ != "stream" && state_source_ != "poll")
  {
    throw std::invalid_argument("state_source must be 'stream' or 'poll'");
  }
  if (state_stream_interval_ms_ < kMinStreamIntervalMs || state_stream_interval_ms_ > kMaxStreamIntervalMs)
  {
    throw std::invalid_argument("state_stream_interval_ms must be in [10, 60000]");
  }
  if (!is_valid_rate(joint_dynamics_rate_, kMaxAuxRate))
  {
    throw std::invalid_argument("joint_dynamics_rate must be in [0, 100] Hz");
  }
  if (!is_valid_rate(joint_thermal_rate_, kMaxThermalRate))
  {
    throw std::invalid_argument("joint_thermal_rate must be in [0, 1] Hz");
  }
  if (!std::isfinite(motion_action_timeout_) || motion_action_timeout_ <= 0.0 ||
      motion_action_timeout_ > kMaxActionTimeoutSec || !std::isfinite(track_action_timeout_) ||
      track_action_timeout_ <= 0.0 || track_action_timeout_ > kMaxActionTimeoutSec)
  {
    throw std::invalid_argument("action timeouts must be in (0, 3600] seconds");
  }

  constexpr int kMinJoints = 1;
  constexpr int kMaxJoints = 8;
  if (ndof_ < kMinJoints || ndof_ > kMaxJoints)
  {
    throw std::invalid_argument("arm_joints size must be in [1, 8]");
  }

  read_only_param_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters)
      {
        return on_parameters_set(parameters);
      });
}

rcl_interfaces::msg::SetParametersResult DriverNode::on_parameters_set(const std::vector<rclcpp::Parameter>& parameters)
{
  // 所有参数都在构造期读取并决定定时器、回调组与连接拓扑；运行期修改不会生效，
  // 明确返回失败，避免 ros2 param set 假装成功。
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = parameters.empty() ? "parameters are read-only after startup"
                                     : parameters.front().get_name() + " is read-only after startup";
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

  topic_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  topic_options_.callback_group = topic_group_;

  timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // 诊断话题与看门狗各自成组：热状态单次查询可达 150–350 ms，不能阻塞 100 Hz 位姿发布。
  stream_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  dynamics_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  thermal_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  status_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // action 的 execute 在独立工作线程中运行；goal/cancel 回调仍单独成组，避免占用服务组。
  action_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
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
  service_group_.reset(); // 注册结束，后续 create_service 未显式选组会抛异常

  // 话题pub
  StatePublishConfig publish_config;
  publish_config.stream_enabled = state_source_ == "stream";
  publish_config.stream_interval_ms = state_stream_interval_ms_;
  publish_config.ndof = ndof_;
  publishers_ = std::make_unique<tl_driver::StatePublishers>(*this, session_, arm_joints_, publish_config);

  // 话题sub
  register_topics(*this);

  create_state_timer(publish_rate_);
  create_aux_timers();

  if (state_source_ == "stream")
  {
    stream_watchdog_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]()
        {
          check_state_stream();
        },
        stream_group_);
  }

  actions_ = std::make_unique<tl_driver::MotionActionServers>(*this, action_group_);
}

void DriverNode::create_state_timer(double rate)
{
  // 直接使用浮点周期，避免截断到整毫秒造成系统性频率偏差（如 60 Hz → 62.5 Hz）。
  const auto period = std::chrono::duration<double>(1.0 / rate);

  state_publish_timer_ = this->create_wall_timer(
      period,
      [this]()
      {
        publishers_->publish_arm_state();
      },
      timer_group_);
}

bool DriverNode::motion_action_active() const
{
  return actions_ && actions_->is_motion_active();
}

void DriverNode::note_external_motion()
{
  if (actions_)
  {
    actions_->note_external_motion();
  }
}

void DriverNode::create_aux_timers()
{
  const auto make_timer = [this](double rate, std::function<void()> callback,
                                 const rclcpp::CallbackGroup::SharedPtr& group) -> rclcpp::TimerBase::SharedPtr
  {
    if (rate <= 0.0)
    {
      return rclcpp::TimerBase::SharedPtr{};
    }
    return this->create_wall_timer(std::chrono::duration<double>(1.0 / rate), std::move(callback), group);
  };

  dynamics_timer_ = make_timer(
      joint_dynamics_rate_,
      [this]()
      {
        publishers_->publish_dynamics();
      },
      dynamics_group_);
  thermal_timer_ = make_timer(
      joint_thermal_rate_,
      [this]()
      {
        publishers_->publish_thermal();
      },
      thermal_group_);
  status_query_timer_ = make_timer(
      kRunningStatusRate,
      [this]()
      {
        publishers_->query_running_status();
      },
      status_group_);
  speed_query_timer_ = make_timer(
      kSpeedQueryRate,
      [this]()
      {
        publishers_->query_speed();
      },
      status_group_);
}

void DriverNode::check_state_stream()
{
  if (state_source_ != "stream" || !session_.is_connected())
  {
    return;
  }

  const auto max_age = std::chrono::milliseconds(3 * state_stream_interval_ms_);
  for (int channel : {1, 2})
  {
    if (session_.is_state_frame_fresh(channel, max_age))
    {
      if (stream_request_failures_[channel] >= 3)
      {
        RCLCPP_INFO(this->get_logger(), "[StateStream]: channel %d recovered", channel);
      }
      stream_request_failures_[channel] = 0;
      continue;
    }

    // 计数在“仍无新帧”时递增；请求返回 SUCCESS 不代表控制器真的推送，不能据此清零。
    ++stream_request_failures_[channel];
    if (stream_request_failures_[channel] == 3)
    {
      RCLCPP_WARN(this->get_logger(),
                  "[StateStream]: channel %d has no fresh frames; retrying mode=1 every 5 s "
                  "(dependent topic falls back to polling)",
                  channel);
    }

    // 前 3 s 每秒重试，之后每 5 s 重试一次，避免链路已断时刷屏。
    if (stream_request_failures_[channel] <= 3 || stream_request_failures_[channel] % 5 == 0)
    {
      session_.request_state_stream(channel, state_stream_interval_ms_);
    }
  }
}

DriverNode::~DriverNode()
{
  // 先停状态发布线程：它会读 7000 快照，必须在会话断开前退出。
  if (publishers_)
  {
    publishers_->stop_stream_thread();
  }

  // 再停 action 工作线程，避免析构过程中仍访问节点/会话。
  actions_.reset();

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
    // 连续状态流：ACS（channel=1）/MCS（channel=2）。失败不阻塞启动，1 Hz 看门狗会重试。
    if (state_source_ == "stream")
    {
      if (session_.start_state_stream(state_stream_interval_ms_))
      {
        RCLCPP_INFO(this->get_logger(), "[StateStream]: ACS/MCS streams started at %d ms interval",
                    state_stream_interval_ms_);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(),
                    "[StateStream]: failed to start state streams; watchdog will retry, or use "
                    "state_source:=poll");
      }
    }

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

    // 关节软件版本只在启动时记录；查询失败仅告警，不得阻断启动（部分固件要求伺服使能后才能查询）。
    for (int axis = 1; axis <= ndof_; ++axis)
    {
      std::string version;
      const tl::Result version_ret = tl::query_joint_software_version(fd, axis, version);
      if (version_ret == tl::Result::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "[Init]: joint %d software version: %s", axis, version.c_str());
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "[Init]: joint %d software version query failed: %s", axis,
                    arm_client::result_to_string(version_ret));
      }
    }

    // 所有可能抛异常的初始化步骤已完成后再启动发布线程：
    // 若在 set_current_mode() 抛异常前启动，构造失败时 StatePublishers 析构若未 join 会 terminate。
    if (state_source_ == "stream")
    {
      publishers_->start_stream_thread();
    }
    return;
  }
  throw std::runtime_error("failed to connect to " + arm_ip_ + ":" + arm_port_ + "," + arm_port_aux_);
}
} // namespace tl_driver
