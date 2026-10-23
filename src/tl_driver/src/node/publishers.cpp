/// @file publishers.cpp
/// @brief StatePublishers 的实现：状态流快照发布与诊断话题。

#include "node/publishers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "arm_client.hpp"
#include "arm_sdk.hpp"
#include "convert/units.hpp"

namespace tl_driver
{
namespace
{
// 运行状态查询失败只记节流 WARN；连续失败期间话题自然停更。
constexpr int kAcsChannel = 1;
constexpr int kMcsChannel = 2;
constexpr std::size_t kBodyAxes = 7;
constexpr std::size_t kSyncAxes = 5;

/// @brief 定长数组填 NaN，表示该查询失败或字段缺失。
template <std::size_t N>
void fill_invalid(std::array<double, N>& out)
{
  out.fill(std::numeric_limits<double>::quiet_NaN());
}

/// @brief 把查询结果写入定长数组，缺失元素保持 NaN。
template <std::size_t N>
void fill_from_vector(std::array<double, N>& out, const std::vector<double>& values)
{
  fill_invalid(out);
  const std::size_t count = std::min(values.size(), N);
  std::copy_n(values.begin(), count, out.begin());
}

/// @brief 力矩查询返回 int 向量，定长数组统一为 float64 + NaN。
template <std::size_t N>
void fill_from_int_vector(std::array<double, N>& out, const std::vector<int>& values)
{
  fill_invalid(out);
  const std::size_t count = std::min(values.size(), N);
  for (std::size_t i = 0; i < count; ++i)
  {
    out[i] = static_cast<double>(values[i]);
  }
}
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

void StatePublishers::start_stream_thread()
{
  if (stream_thread_.joinable())
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
    if (session_.wait_state_update(stream_notify_seen_, kWait))
    {
      publish_stream_updates();
    }
  }
}

void StatePublishers::publish_stream_updates()
{
  if (!session_.is_connected())
  {
    return;
  }

  // 只有状态帧发布线程调用本函数；两个发布函数内部按 seq 去重并检查帧龄。
  publish_joint_stream();
  publish_tcp_stream();
}

void StatePublishers::publish_joint_stream()
{
  arm_client::StateFrame frame;
  if (!session_.get_state_frame(kAcsChannel, frame) || frame.acs_count == 0)
  {
    return;
  }

  const auto max_age = std::chrono::milliseconds(3 * config_.stream_interval_ms);
  const auto now = std::chrono::steady_clock::now();
  if (now - frame.received_at > max_age)
  {
    return;
  }
  if (frame.seq == last_acs_seq_)
  {
    return;
  }
  last_acs_seq_ = frame.seq;

  joint_pose_ = frame.acs;
  convert::joint_deg_to_rad(joint_pose_, frame.acs_count);
  // 时间戳取帧接收时刻而不是发布时刻，避免消费者把排队延迟算进运动学时间。
  // 这里用 ROS 时钟减去 steady 时钟的帧龄；`use_sim_time` 下若时钟暂停，两者可能短暂不同步。
  const auto age = std::chrono::duration_cast<std::chrono::nanoseconds>(now - frame.received_at);
  const rclcpp::Time stamp = node_.now() - rclcpp::Duration(age);
  publish_joint_pose(joint_pose_, frame.acs_count, stamp);
}

void StatePublishers::publish_tcp_stream()
{
  arm_client::StateFrame frame;
  if (!session_.get_state_frame(kMcsChannel, frame) || frame.mcs_count < 6)
  {
    return;
  }

  const auto max_age = std::chrono::milliseconds(3 * config_.stream_interval_ms);
  const auto now = std::chrono::steady_clock::now();
  if (now - frame.received_at > max_age)
  {
    return;
  }
  if (frame.seq == last_mcs_seq_)
  {
    return;
  }
  last_mcs_seq_ = frame.seq;

  const auto age = std::chrono::duration_cast<std::chrono::nanoseconds>(now - frame.received_at);
  const rclcpp::Time stamp = node_.now() - rclcpp::Duration(age);
  publish_tcp_pose_from_mcs(frame.mcs, frame.mcs_count, stamp);
}

void StatePublishers::publish_joint_pose(const std::array<double, convert::kMaxStateAxes>& joint_pose,
                                         std::size_t count, const rclcpp::Time& stamp)
{
  auto& msg = joint_state_msg_;
  msg.header.stamp = stamp;

  // 按实际数据与关节名数量裁剪：6 轴时 SDK 末位为补 0，丢掉；数据不足时不越界。
  const std::size_t n = std::min({count, joint_names_.size(), joint_pose.size()});
  if (n == 0)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "[Pub]: empty joint pose or joint names, skip /joint_states");
    return;
  }
  if (count < joint_names_.size())
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "[Pub]: joint pose has %zu values, fewer than %zu joint names", count, joint_names_.size());
  }
  // name 必须与 position 等长，否则 JointState 契约不成立；稳态下长度不变，跳过字符串拷贝。
  if (msg.name.size() != n)
  {
    msg.name.assign(joint_names_.begin(), joint_names_.begin() + n);
  }
  msg.position.assign(joint_pose.begin(), joint_pose.begin() + n);

  joint_state_pub_->publish(msg);
}

void StatePublishers::publish_tcp_pose_from_mcs(const std::array<double, convert::kMaxStateAxes>& mcs,
                                                std::size_t count, const rclcpp::Time& stamp)
{
  if (count < 6)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "[Pub]: MCS frame has %zu values, need at least 6, skip /tcp_pose", count);
    return;
  }

  // MCS 流的位置为 mm、rpy 为度（与 6001 同步查询的 rad 不同）。
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
      msg.arm_angle = count > 6 ? convert::deg_to_rad(mcs[6]) : 0.0;
      break;
    default:
      msg.arm_angle = count > 6 ? convert::deg_to_rad(mcs[6]) : 0.0;
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                           "[Pub]: unsupported ndof=%d, arm_angle fallback to %s", config_.ndof,
                           count > 6 ? "seventh value" : "0");
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

  // 拷贝后发布：避免持锁执行 DDS 写，也避免与 1 Hz 速度查询互相阻塞。
  tl_ros2_interface::msg::ArmStatus msg;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    arm_status_msg_.stamp = node_.now();
    arm_status_msg_.run_state = std::move(state);
    msg = arm_status_msg_;
  }
  running_status_pub_->publish(msg);
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

void StatePublishers::publish_dynamics()
{
  if (!session_.is_connected())
  {
    return;
  }

  const int fd = session_.fd();
  const double kInvalid = std::numeric_limits<double>::quiet_NaN();

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
  msg.line_speed = speed_ok ? convert::mm_s_to_m_s(line_speed) : kInvalid;

  // 子查询失败时对应字段整段 NaN；成功但元素不足时缺失元素保持 NaN。
  if (speed_ok)
  {
    convert::joint_deg_to_rad(joint_speed, joint_speed.size());
    convert::joint_deg_to_rad(joint_speed_sync, joint_speed_sync.size());
    fill_from_vector<kBodyAxes>(msg.joint_speed, joint_speed);
    fill_from_vector<kSyncAxes>(msg.joint_speed_sync, joint_speed_sync);
  }
  else
  {
    fill_invalid(msg.joint_speed);
    fill_invalid(msg.joint_speed_sync);
  }

  if (current_ok)
  {
    fill_from_vector<kBodyAxes>(msg.motor_current, motor_current);
    fill_from_vector<kSyncAxes>(msg.motor_current_sync, motor_current_sync);
  }
  else
  {
    fill_invalid(msg.motor_current);
    fill_invalid(msg.motor_current_sync);
  }

  if (torque_ok)
  {
    fill_from_int_vector<kBodyAxes>(msg.motor_torque, motor_torque);
    fill_from_int_vector<kSyncAxes>(msg.motor_torque_sync, motor_torque_sync);
  }
  else
  {
    fill_invalid(msg.motor_torque);
    fill_invalid(msg.motor_torque_sync);
  }

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
  if (temperature_ok)
  {
    fill_from_vector<kBodyAxes>(msg.temperature, temperature);
  }
  else
  {
    fill_invalid(msg.temperature);
  }
  if (voltage_ok)
  {
    fill_from_vector<kBodyAxes>(msg.joint_voltage, joint_voltage);
    fill_from_vector<kSyncAxes>(msg.positioner_voltage, positioner_voltage);
  }
  else
  {
    fill_invalid(msg.joint_voltage);
    fill_invalid(msg.positioner_voltage);
  }

  joint_thermal_pub_->publish(msg);
}

} // namespace tl_driver
