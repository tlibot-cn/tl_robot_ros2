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
constexpr int kAcsChannel = 1;
constexpr int kMcsChannel = 2;
constexpr std::size_t kBodyAxes = 7;

/// @brief 定长数组填 NaN，表示该查询失败或字段缺失。
template <std::size_t N>
void fill_invalid(std::array<double, N>& out)
{
  out.fill(std::numeric_limits<double>::quiet_NaN());
}

/// @brief 把查询结果写入定长数组，只保留前 valid_count 个有效关节，其余置 NaN
///        （6 轴机型的第 7 位补 0 不再冒充有效值）。
template <std::size_t N>
void fill_from_vector(std::array<double, N>& out, const std::vector<double>& values, std::size_t valid_count)
{
  fill_invalid(out);
  const std::size_t count = std::min({values.size(), N, valid_count});
  std::copy_n(values.begin(), count, out.begin());
}

/// @brief 把状态帧辅助字段写入 `JointState.velocity` / `.effort`。
/// @param[out] out 目标数组，长度与 `name`/`position` 对齐
/// @param[in] values 状态帧原始值（定长，NaN 填充表示无效）
/// @param[in] count 帧内有效元素个数；0 = 字段缺失，此时清空数组（JointState 语义：空 = 不可用）
/// @param[in] n 目标长度（= 本帧 name/position 长度）
/// @param[in] scale 单位换算系数（速度 deg/s→rad/s；力矩 ‰ 原样传 1）
void fill_joint_state_aux(std::vector<double>& out, const std::array<double, convert::kMaxStateAxes>& values,
                          std::size_t count, std::size_t n, double scale)
{
  if (count == 0 || n == 0)
  {
    out.clear();
    return;
  }
  // 稳态下长度不变，不重复分配；数据不足的部分显式置 NaN。
  if (out.size() != n)
  {
    out.resize(n);
  }
  const std::size_t m = std::min(count, n);
  for (std::size_t i = 0; i < m; ++i)
  {
    out[i] = values[i] * scale;
  }
  std::fill(out.begin() + static_cast<std::ptrdiff_t>(m), out.end(), std::numeric_limits<double>::quiet_NaN());
}
} // namespace

StatePublishers::StatePublishers(rclcpp::Node& node, ArmSession& session, std::vector<std::string> joint_names,
                                 const StatePublishConfig& config)
    : node_(node), session_(session), joint_names_(std::move(joint_names)), config_(config)
{
  joint_state_pub_ = node_.create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  tcp_pose_pub_ = node_.create_publisher<tl_ros2_interface::msg::CartesianPose>("/tcp_pose", 10);

  // 诊断话题：只关心最新值，晚加入的订阅者应立即拿到最后一帧。
  const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  joint_monitor_pub_ =
      node_.create_publisher<tl_ros2_interface::msg::JointMonitor>("/tl_driver/joint_monitor", state_qos);
  line_speed_pub_ = node_.create_publisher<tl_ros2_interface::msg::LineSpeed>("/tl_driver/line_speed", state_qos);

  // 错误/告警是离散事件：不能用 transient_local 让新订阅者收到陈旧故障，也不做节流丢帧。
  error_info_pub_ = node_.create_publisher<tl_ros2_interface::msg::ErrorInfo>(
      "/tl_driver/error_info", rclcpp::QoS(rclcpp::KeepLast(100)).reliable());

  // 静态字段只填一次；动态字段每周期覆盖，容量复用。发布路径不清空消息，frame_id 保持不变。
  joint_state_msg_.name = joint_names_;
  tcp_pose_msg_.header.frame_id = "base_link";

  // SDK 错误/告警只在 6001 主端口回调；回调线程直接发布，不经过执行器。
  session_.set_error_handler(
      [this](int message_type, const std::string& message, int message_code)
      {
        publish_error(message_type, message, message_code);
      });
}

StatePublishers::~StatePublishers()
{
  // 先注销 SDK 回调：之后即使回调线程仍在运行，也不会访问正在析构的发布器。
  session_.set_error_handler(nullptr);
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
  // 关节速度/力矩与关节角同帧同时间戳：控制器按 RobotState 的 jointSpeed/jointTorque 开关
  // 在 ACS 帧中附带 jointVel（deg/s）与 torque（‰），直接写入 JointState.velocity/effort。
  publish_joint_pose(joint_pose_, frame.acs_count, frame.joint_vel, frame.joint_vel_count, frame.torque,
                     frame.torque_count, stamp);
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
                                         std::size_t count, const std::array<double, convert::kMaxStateAxes>& joint_vel,
                                         std::size_t joint_vel_count,
                                         const std::array<double, convert::kMaxStateAxes>& torque,
                                         std::size_t torque_count, const rclcpp::Time& stamp)
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

  // velocity/effort 与 name/position 等长；帧内缺字段（固件不支持查询开关）时为空数组（不可用）。
  if (joint_vel_count == 0)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "[Pub]: ACS frame has no jointVel field, /joint_states.velocity unavailable");
  }
  if (torque_count == 0)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "[Pub]: ACS frame has no torque field, /joint_states.effort unavailable");
  }
  fill_joint_state_aux(msg.velocity, joint_vel, joint_vel_count, n, convert::kDegToRad);
  fill_joint_state_aux(msg.effort, torque, torque_count, n, 1.0);

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

void StatePublishers::publish_monitor()
{
  if (!session_.is_connected())
  {
    return;
  }

  const int fd = session_.fd();
  std::vector<double> motor_current;
  std::vector<double> motor_current_sync; // 外部轴电流不发布，但 SDK 要求出参非空。
  std::vector<double> temperature;
  std::vector<double> joint_voltage;
  std::vector<double> positioner_voltage; // 外部轴电压不发布，但 SDK 要求出参非空。

  const bool current_ok = tl::get_current_motor_current(fd, motor_current) == tl::Result::SUCCESS;
  const bool temperature_ok = tl::get_joint_temperature(fd, temperature) == tl::Result::SUCCESS;
  const bool voltage_ok = tl::get_joint_voltage(fd, joint_voltage) == tl::Result::SUCCESS;

  if (!current_ok || !temperature_ok || !voltage_ok)
  {
    RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 5000,
        "[Pub]: joint_monitor query failed (current=%d temperature=%d voltage=%d), failed fields set to NaN",
        current_ok, temperature_ok, voltage_ok);
  }

  // 三个子查询在同一回调内完成，整帧同一时间戳；失败的字段整段 NaN，不跨帧复用旧值。
  auto& msg = joint_monitor_msg_;
  msg.header.stamp = node_.now();
  const std::size_t valid_axes = static_cast<std::size_t>(config_.ndof);

  if (current_ok)
  {
    fill_from_vector<kBodyAxes>(msg.motor_current, motor_current, valid_axes);
  }
  else
  {
    fill_invalid(msg.motor_current);
  }

  if (temperature_ok)
  {
    fill_from_vector<kBodyAxes>(msg.temperature, temperature, valid_axes);
  }
  else
  {
    fill_invalid(msg.temperature);
  }

  if (voltage_ok)
  {
    fill_from_vector<kBodyAxes>(msg.joint_voltage, joint_voltage, valid_axes);
  }
  else
  {
    fill_invalid(msg.joint_voltage);
  }

  joint_monitor_pub_->publish(msg);
}

void StatePublishers::publish_line_speed()
{
  if (!session_.is_connected())
  {
    return;
  }

  const int fd = session_.fd();
  double line_speed = 0.0;
  std::vector<double> joint_speed; // 关节速度已随 /joint_states.velocity 发布；SDK 出参，结果丢弃。
  std::vector<double> joint_speed_sync; // 外部轴同步速度不发布；SDK 出参，结果丢弃。

  const tl::Result ret = tl::get_current_line_speed_and_joint_speed(fd, line_speed, joint_speed, joint_speed_sync);
  if (ret != tl::Result::SUCCESS)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "[Pub]: line speed query failed, result=%s",
                         arm_client::result_to_string(ret));
    return;
  }

  // 线速度查询返回 mm/s，入口换算到 REP 103。
  auto& line_msg = line_speed_msg_;
  line_msg.header.stamp = node_.now();
  line_msg.line_speed = convert::mm_s_to_m_s(line_speed);
  line_speed_pub_->publish(line_msg);
}

void StatePublishers::publish_error(int message_type, const std::string& message, int message_code)
{
  // 错误是低频事件，不做节流；每条推送独立成帧，允许一次局部构造（避免与发布线程共享缓冲）。
  tl_ros2_interface::msg::ErrorInfo msg;
  msg.header.stamp = node_.now();
  msg.message_type = message_type;
  msg.message_code = message_code;
  msg.message = message;
  error_info_pub_->publish(msg);
}


} // namespace tl_driver
