/// @file publishers.cpp
/// @brief StatePublishers 的实现：状态流快照发布、poll 回退与诊断话题。

#include "node/publishers.hpp"

#include <algorithm>
#include <utility>

#include "arm_client.hpp"
#include "arm_sdk.hpp"
#include "convert/units.hpp"

namespace tl_driver
{
namespace
{
// 双通道连续失效 3 s 后回退同步查询；恢复连续 1 s 后切回状态流。
constexpr std::chrono::seconds kStreamFallbackAfter{3};
constexpr std::chrono::seconds kStreamRecoverAfter{1};
constexpr std::chrono::milliseconds kPollPeriod{50}; // 20 Hz
// 连续 3 次查询失败（约 300 ms）即认为缓存过期，停止发布旧状态而不是冻结在最后一次成功值。
constexpr int kMaxRunningStatusFailures = 3;
constexpr int kAcsChannel = 1;
constexpr int kMcsChannel = 2;
} // namespace

StatePublishers::StatePublishers(rclcpp::Node& node, const ArmSession& session, std::vector<std::string> joint_names,
                                 const StatePublishConfig& config)
    : node_(node), session_(session), joint_names_(std::move(joint_names)), config_(config)
{
  joint_state_pub_ = node_.create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  tcp_pose_pub_ = node_.create_publisher<tl_ros2_interface::msg::CartesianPose>("/tcp_pose", 10);

  running_status_pub_ = node_.create_publisher<tl_ros2_interface::msg::ArmStatus>("/arm_status", 10);

  // 诊断话题：只关心最新值，晚加入的订阅者应立即拿到最后一帧。
  const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  joint_dynamics_pub_ =
      node_.create_publisher<tl_ros2_interface::msg::JointDynamics>("/tl_driver/joint_dynamics", state_qos);
  joint_thermal_pub_ =
      node_.create_publisher<tl_ros2_interface::msg::JointThermal>("/tl_driver/joint_thermal", state_qos);

  // 静态字段只填一次；动态字段每周期覆盖，容量复用。发布路径不清空消息，frame_id 保持不变。
  joint_state_msg_.name = joint_names_;
  tcp_pose_msg_.header.frame_id = "base_link";
}

StatePublishers::~StatePublishers()
{
  // 任何析构路径（包括 DriverNode 构造抛异常）都必须先 join，避免 std::thread 析构 terminate。
  stop_stream_thread();
}

void StatePublishers::publish_arm_state()
{
  if (!session_.is_connected())
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[Pub]: arm is not connected");
    return;
  }

  // 只发布 10 Hz 查询缓存的运行状态。健康流路径不调用 SDK；
  // 下面的 poll 模式与通道失效回退仍会同步查询，受 20 Hz 上限约束。
  publish_running_status();

  const auto now = std::chrono::steady_clock::now();
  // 回退轮询限制在 20 Hz：断链时单次同步查询可能阻塞数秒，不能以 100 Hz 打满 timer_group_。
  const bool poll_due = last_poll_ == std::chrono::steady_clock::time_point{} || now - last_poll_ >= kPollPeriod;

  if (!config_.stream_enabled)
  {
    if (poll_due)
    {
      const int fd = session_.fd();
      poll_joint(fd);
      poll_tcp(fd);
      last_poll_ = now;
    }
    return;
  }

  // stream 模式：/joint_states 与 /tcp_pose 由 7000 帧通知驱动的独立线程发布。
  // 这里只按帧龄维护通道健康状态，并在流失效后驱动 poll 回退。
  const auto max_age = std::chrono::milliseconds(3 * config_.stream_interval_ms);
  update_channel_health(acs_health_, !session_.is_state_frame_fresh(kAcsChannel, max_age), "ACS /joint_states");
  update_channel_health(mcs_health_, !session_.is_state_frame_fresh(kMcsChannel, max_age), "MCS /tcp_pose");

  if (!poll_due)
  {
    return;
  }

  const int fd = session_.fd();
  if (!acs_health_.stream_active)
  {
    poll_joint(fd);
  }
  if (!mcs_health_.stream_active)
  {
    poll_tcp(fd);
  }
  last_poll_ = now;
}

void StatePublishers::start_stream_thread()
{
  if (!config_.stream_enabled || stream_thread_.joinable())
  {
    return;
  }
  stream_thread_stop_.store(false, std::memory_order_relaxed);
  stream_thread_ = std::thread(&StatePublishers::stream_publish_loop, this);
}

void StatePublishers::stop_stream_thread()
{
  stream_thread_stop_.store(true, std::memory_order_relaxed);
  if (stream_thread_.joinable())
  {
    stream_thread_.join();
  }
}

void StatePublishers::stream_publish_loop()
{
  constexpr auto kWait = std::chrono::milliseconds(20);
  // shutdown 后不再发布，避免析构 join 之前仍向 DDS 写入。
  while (rclcpp::ok() && !stream_thread_stop_.load(std::memory_order_relaxed))
  {
    // 没有新帧时按 20 ms 超时醒来，仅用于检查停止标志；不轮询 SDK。
    session_.wait_state_update(stream_notify_seen_, kWait);
    if (stream_thread_stop_.load(std::memory_order_relaxed))
    {
      break;
    }
    publish_stream_updates();
  }
}

void StatePublishers::publish_stream_updates()
{
  if (!session_.is_connected())
  {
    return;
  }

  // 锁只覆盖“填消息 + DDS 发布”的短临界区；拿不到锁说明另一条发布路径正在写同一组缓冲，
  // 跳过本帧即可（下一次帧通知会带最新 seq 重试）。
  std::unique_lock<std::mutex> publish_lock(publish_mutex_, std::try_to_lock);
  if (!publish_lock.owns_lock())
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "[Pub]: publish buffer busy (concurrent publish), skip stream frame");
    return;
  }

  // 单通道恢复/失效互不影响；两个发布函数内部按 seq 去重并检查帧龄。
  if (acs_health_.stream_active.load(std::memory_order_relaxed))
  {
    publish_joint_stream();
  }
  if (mcs_health_.stream_active.load(std::memory_order_relaxed))
  {
    publish_tcp_stream();
  }
}

void StatePublishers::poll_joint(int fd)
{
  // 查询用局部缓冲：同步调用可能阻塞数百毫秒，不能持发布锁执行。
  std::vector<double> joint_pose;
  const tl::Result ret = tl::get_current_position(fd, tl::Coord::JOINT, joint_pose);
  if (ret != tl::Result::SUCCESS)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "[Pub]: failed to poll joint position, result=%s", arm_client::result_to_string(ret));
    return;
  }

  // 同步查询返回度；批量转换为弧度。
  convert::joint_deg_to_rad(joint_pose, joint_pose.size());
  std::lock_guard<std::mutex> publish_lock(publish_mutex_);
  publish_joint_pose(joint_pose, node_.now());
}

void StatePublishers::poll_tcp(int fd)
{
  std::vector<double> tcp_pose;
  const tl::Result ret = tl::get_current_position(fd, tl::Coord::BASE, tcp_pose);
  if (ret != tl::Result::SUCCESS)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[Pub]: failed to poll TCP pose, result=%s",
                         arm_client::result_to_string(ret));
    return;
  }

  // 同步查询的 rpy 已是 rad，位置为 mm。
  std::lock_guard<std::mutex> publish_lock(publish_mutex_);
  publish_tcp_pose(tcp_pose, node_.now());
}

void StatePublishers::publish_joint_stream()
{
  uint64_t seq = 0;
  std::chrono::steady_clock::time_point received_at;
  if (!session_.get_state_values(kAcsChannel, joint_pose_, seq, received_at) || joint_pose_.empty())
  {
    return;
  }

  const auto max_age = std::chrono::milliseconds(3 * config_.stream_interval_ms);
  const auto now = std::chrono::steady_clock::now();
  if (now - received_at > max_age)
  {
    return;
  }
  if (seq == last_acs_seq_)
  {
    return;
  }
  last_acs_seq_ = seq;

  convert::joint_deg_to_rad(joint_pose_, joint_pose_.size());
  // 时间戳取帧接收时刻而不是发布时刻，避免消费者把排队延迟算进运动学时间。
  // 这里用 ROS 时钟减去 steady 时钟的帧龄；`use_sim_time` 下若时钟暂停，两者可能短暂不同步。
  const auto age = std::chrono::duration_cast<std::chrono::nanoseconds>(now - received_at);
  const rclcpp::Time stamp = node_.now() - rclcpp::Duration(age);
  publish_joint_pose(joint_pose_, stamp);
}

void StatePublishers::publish_tcp_stream()
{
  uint64_t seq = 0;
  std::chrono::steady_clock::time_point received_at;
  if (!session_.get_state_values(kMcsChannel, tcp_pose_, seq, received_at) || tcp_pose_.size() < 6)
  {
    return;
  }

  const auto max_age = std::chrono::milliseconds(3 * config_.stream_interval_ms);
  const auto now = std::chrono::steady_clock::now();
  if (now - received_at > max_age)
  {
    return;
  }
  if (seq == last_mcs_seq_)
  {
    return;
  }
  last_mcs_seq_ = seq;

  const auto age = std::chrono::duration_cast<std::chrono::nanoseconds>(now - received_at);
  const rclcpp::Time stamp = node_.now() - rclcpp::Duration(age);
  publish_tcp_pose_from_mcs(tcp_pose_, stamp);
}

void StatePublishers::update_channel_health(ChannelHealth& health, bool unavailable, const char *name)
{
  const auto now = std::chrono::steady_clock::now();

  if (unavailable)
  {
    health.fresh_since = {};
    if (health.stale_since == std::chrono::steady_clock::time_point{})
    {
      health.stale_since = now;
    }
    else if (health.stream_active && now - health.stale_since >= kStreamFallbackAfter)
    {
      health.stream_active = false;
      RCLCPP_WARN(node_.get_logger(), "[Pub]: %s stream stale for %lld s, fall back to synchronous polling", name,
                  static_cast<long long>(kStreamFallbackAfter.count()));
    }
    return;
  }

  health.stale_since = {};
  if (!config_.stream_enabled || health.stream_active)
  {
    return;
  }

  if (health.fresh_since == std::chrono::steady_clock::time_point{})
  {
    health.fresh_since = now;
  }
  else if (now - health.fresh_since >= kStreamRecoverAfter)
  {
    health.stream_active = true;
    health.fresh_since = {};
    RCLCPP_INFO(node_.get_logger(), "[Pub]: %s stream recovered, resume stream publishing", name);
  }
}

void StatePublishers::publish_joint_pose(const std::vector<double>& joint_pose, const rclcpp::Time& stamp)
{
  auto& msg = joint_state_msg_;
  msg.header.stamp = stamp;

  // 按实际数据与关节名数量裁剪：6 轴时 SDK 末位为补 0，丢掉；数据不足时不越界。
  const std::size_t count = std::min(joint_pose.size(), joint_names_.size());
  if (count == 0)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "[Pub]: empty joint pose or joint names, skip /joint_states");
    return;
  }
  if (joint_pose.size() < joint_names_.size())
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "[Pub]: joint pose has %zu values, fewer than %zu joint names", joint_pose.size(),
                         joint_names_.size());
  }
  // name 必须与 position 等长，否则 JointState 契约不成立；稳态下长度不变，跳过字符串拷贝。
  if (msg.name.size() != count)
  {
    msg.name.assign(joint_names_.begin(), joint_names_.begin() + count);
  }
  msg.position.assign(joint_pose.begin(), joint_pose.begin() + count);

  joint_state_pub_->publish(msg);
}

void StatePublishers::publish_tcp_pose(const std::vector<double>& tcp_pose, const rclcpp::Time& stamp)
{
  if (tcp_pose.size() < 6)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "[Pub]: tcp pose has %zu values, need at least 6, skip /tcp_pose", tcp_pose.size());
    return;
  }

  auto& msg = tcp_pose_msg_;
  msg.header.stamp = stamp;

  // 同步查询返回：位置 mm、欧拉角 rad → m 和 rad。
  msg.position.x = convert::mm_to_m(tcp_pose[0]);
  msg.position.y = convert::mm_to_m(tcp_pose[1]);
  msg.position.z = convert::mm_to_m(tcp_pose[2]);

  msg.rpy.x = tcp_pose[3];
  msg.rpy.y = tcp_pose[4];
  msg.rpy.z = tcp_pose[5];

  switch (config_.ndof)
  {
    case 6:
      msg.arm_angle = 0;
      break;
    case 7:
      msg.arm_angle = tcp_pose.size() > 6 ? tcp_pose[6] : 0.0;
      break;
    default:
      msg.arm_angle = tcp_pose.size() > 6 ? tcp_pose[6] : 0.0;
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                           "[Pub]: unsupported ndof=%d, arm_angle fallback to %s", config_.ndof,
                           tcp_pose.size() > 6 ? "seventh value" : "0");
      break;
  }

  tcp_pose_pub_->publish(msg);
}

void StatePublishers::publish_tcp_pose_from_mcs(const std::vector<double>& mcs, const rclcpp::Time& stamp)
{
  if (mcs.size() < 6)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "[Pub]: MCS frame has %zu values, need at least 6, skip /tcp_pose", mcs.size());
    return;
  }

  // 关键单位差异（架构评审 §14.3）：MCS 流的 rpy 是度，不能复用同步查询（rad）的分支。
  auto& msg = tcp_pose_msg_;
  msg.header.stamp = stamp;

  msg.position.x = convert::mm_to_m(mcs[0]);
  msg.position.y = convert::mm_to_m(mcs[1]);
  msg.position.z = convert::mm_to_m(mcs[2]);

  msg.rpy.x = convert::deg_to_rad(mcs[3]);
  msg.rpy.y = convert::deg_to_rad(mcs[4]);
  msg.rpy.z = convert::deg_to_rad(mcs[5]);

  switch (config_.ndof)
  {
    case 6:
      msg.arm_angle = 0;
      break;
    case 7:
      msg.arm_angle = mcs.size() > 6 ? convert::deg_to_rad(mcs[6]) : 0.0;
      break;
    default:
      msg.arm_angle = mcs.size() > 6 ? convert::deg_to_rad(mcs[6]) : 0.0;
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                           "[Pub]: unsupported ndof=%d, arm_angle fallback to %s", config_.ndof,
                           mcs.size() > 6 ? "seventh value" : "0");
      break;
  }

  tcp_pose_pub_->publish(msg);
}

void StatePublishers::query_running_status()
{
  if (!session_.is_connected())
  {
    return;
  }

  tl::RunState running_status = tl::RunState::STOP;
  const tl::Result ret = tl::get_robot_running_state(session_.fd(), running_status);
  if (ret != tl::Result::SUCCESS)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "[Read Running Status]: failed to read running status, result=%s",
                         arm_client::result_to_string(ret));
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (++arm_status_failures_ >= kMaxRunningStatusFailures)
    {
      // 停止发布：消费者看到话题中断，比继续收到冻结的 RUNNING/STOP 更安全。
      arm_status_sampled_ = false;
    }
    return;
  }

  std::string state;
  switch (running_status)
  {
    case tl::RunState::STOP:
      state = "STOP";
      break;
    case tl::RunState::PAUSE:
      state = "PAUSE";
      break;
    case tl::RunState::RUNNING:
      state = "RUNNING";
      break;
    default:
      state = "UNKNOWN";
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[Pub]: unknown run state %d",
                           static_cast<int>(running_status));
      break;
  }

  std::lock_guard<std::mutex> lock(status_mutex_);
  arm_status_msg_.stamp = node_.now();
  arm_status_msg_.run_state = std::move(state);
  arm_status_sampled_ = true;
  arm_status_failures_ = 0;
}

void StatePublishers::query_speed()
{
  if (!session_.is_connected())
  {
    return;
  }

  int speed = 0;
  const tl::Result ret = tl::get_speed(session_.fd(), speed);
  std::lock_guard<std::mutex> lock(status_mutex_);
  if (ret != tl::Result::SUCCESS)
  {
    arm_status_msg_.speed_valid = false;
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[Read Speed]: failed, result=%s",
                         arm_client::result_to_string(ret));
    return;
  }
  arm_status_msg_.speed = static_cast<double>(speed);
  arm_status_msg_.speed_valid = true;
}

void StatePublishers::publish_running_status()
{
  // 拷贝后发布：避免持锁执行 DDS 写，也避免 100 Hz 发布与 10 Hz 查询互相阻塞。
  tl_ros2_interface::msg::ArmStatus msg;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (!arm_status_sampled_)
    {
      return;
    }
    msg = arm_status_msg_;
  }
  running_status_pub_->publish(msg);
}

void StatePublishers::publish_dynamics()
{
  if (!session_.is_connected())
  {
    return;
  }

  const int fd = session_.fd();

  double line_speed = 0.0;
  std::vector<double> joint_speed;
  std::vector<double> joint_speed_sync;
  std::vector<double> motor_current;
  std::vector<double> motor_current_sync;
  std::vector<int> motor_torque;
  std::vector<int> motor_torque_sync;

  const bool speed_ok =
      tl::get_current_line_speed_and_joint_speed(fd, line_speed, joint_speed, joint_speed_sync) == tl::Result::SUCCESS;
  const bool current_ok = tl::get_current_motor_current(fd, motor_current, motor_current_sync) == tl::Result::SUCCESS;
  const bool torque_ok = tl::get_current_motor_torque(fd, motor_torque, motor_torque_sync) == tl::Result::SUCCESS;

  if (!speed_ok && !current_ok && !torque_ok)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "[Pub]: all joint_dynamics queries failed, skip publish");
    return;
  }

  auto& msg = joint_dynamics_msg_;
  msg.header.stamp = node_.now();
  msg.line_speed = speed_ok ? convert::mm_s_to_m_s(line_speed) : 0.0;

  if (speed_ok)
  {
    convert::joint_deg_to_rad(joint_speed, joint_speed.size());
    convert::joint_deg_to_rad(joint_speed_sync, joint_speed_sync.size());
  }
  msg.joint_speed = speed_ok ? joint_speed : std::vector<double>{};
  msg.joint_speed_sync = speed_ok ? joint_speed_sync : std::vector<double>{};
  msg.motor_current = current_ok ? motor_current : std::vector<double>{};
  msg.motor_current_sync = current_ok ? motor_current_sync : std::vector<double>{};
  msg.motor_torque = torque_ok ? motor_torque : std::vector<int>{};
  msg.motor_torque_sync = torque_ok ? motor_torque_sync : std::vector<int>{};

  joint_dynamics_pub_->publish(msg);
}

void StatePublishers::publish_thermal()
{
  if (!session_.is_connected())
  {
    return;
  }

  const int fd = session_.fd();

  std::vector<double> temperature;
  std::vector<double> joint_voltage;
  std::vector<double> positioner_voltage;

  const bool temperature_ok = tl::get_joint_temperature(fd, temperature) == tl::Result::SUCCESS;
  const bool voltage_ok = tl::get_joint_voltage(fd, joint_voltage, positioner_voltage) == tl::Result::SUCCESS;
  if (!temperature_ok && !voltage_ok)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 10000,
                         "[Pub]: all joint_thermal queries failed, skip publish");
    return;
  }

  auto& msg = joint_thermal_msg_;
  msg.header.stamp = node_.now();
  msg.temperature = temperature_ok ? temperature : std::vector<double>{};
  msg.joint_voltage = voltage_ok ? joint_voltage : std::vector<double>{};
  msg.positioner_voltage = voltage_ok ? positioner_voltage : std::vector<double>{};

  joint_thermal_pub_->publish(msg);
}

} // namespace tl_driver
