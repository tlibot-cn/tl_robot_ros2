#include "tl_hardware/tl_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <string>
#include <thread>

#include <pluginlib/class_list_macros.hpp>

using namespace std::chrono_literals;

namespace tl_hardware
{

TLHardwareInterface::TLHardwareInterface()
: logger_(rclcpp::get_logger("tl_hardware_interface")),
  last_joint_state_time_(0, 0, RCL_ROS_TIME)
{
}

TLHardwareInterface::~TLHardwareInterface()
{
  stop_executor();
}

hardware_interface::CallbackReturn TLHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(logger_, "Initializing TL hardware interface");

  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;

  if (!validate_interfaces()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  joint_names_.reserve(info_.joints.size());

  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);
  }

  const size_t joint_count = joint_names_.size();

  joint_positions_.assign(joint_count, 0.0);
  joint_velocities_.assign(joint_count, 0.0);
  joint_efforts_.assign(joint_count, 0.0);

  joint_position_commands_.assign(joint_count, 0.0);

  received_positions_.assign(joint_count, 0.0);
  received_velocities_.assign(joint_count, 0.0);
  received_efforts_.assign(joint_count, 0.0);

  joint_states_topic_ = get_hardware_parameter(
    "joint_states_topic", "/tl_driver/joint_states");

  joint_command_topic_ = get_hardware_parameter(
    "joint_command_topic", "/tl_driver/joint_command");

  const auto state_timeout = get_hardware_parameter("state_timeout_sec", "1.0");
  try {
    state_timeout_sec_ = std::stod(state_timeout);
  } catch (const std::exception &) {
    RCLCPP_WARN(
      logger_,
      "Invalid state_timeout_sec '%s', using default 1.0",
      state_timeout.c_str());
    state_timeout_sec_ = 1.0;
  }

  RCLCPP_INFO(logger_, "Initialized with %zu joints", joint_count);
  RCLCPP_INFO(logger_, "Joint states topic: %s", joint_states_topic_.c_str());
  RCLCPP_INFO(logger_, "Joint command topic: %s", joint_command_topic_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TLHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Configuring TL hardware interface");

  try {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);

    node_ = std::make_shared<rclcpp::Node>("tl_hardware", node_options);

    joint_command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      joint_command_topic_,
      rclcpp::QoS(10).reliable());

    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_,
      rclcpp::QoS(10).best_effort(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_state_callback(msg);
      });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    executor_thread_ = std::thread([this]() {
      RCLCPP_INFO(logger_, "Starting TL hardware executor thread");
      executor_->spin();
      RCLCPP_INFO(logger_, "TL hardware executor thread stopped");
    });

    hardware_connected_ = true;
    hardware_active_ = false;
    has_received_state_ = false;
    last_joint_state_time_ = node_->now();

    RCLCPP_INFO(logger_, "TL hardware interface configured successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to configure hardware interface: %s", e.what());
    stop_executor();
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TLHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating TL hardware interface");

  if (!hardware_connected_ || !node_) {
    RCLCPP_ERROR(logger_, "Hardware interface is not configured");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto start_time = std::chrono::steady_clock::now();

  while (!has_received_state_ &&
    std::chrono::steady_clock::now() - start_time < 5s)
  {
    std::this_thread::sleep_for(50ms);
  }

  if (!has_received_state_) {
    RCLCPP_ERROR(logger_, "Timeout waiting for initial joint states");
    return hardware_interface::CallbackReturn::ERROR;
  }

  {
    std::lock_guard<std::mutex> lock(received_state_mutex_);

    joint_positions_ = received_positions_;
    joint_velocities_ = received_velocities_;
    joint_efforts_ = received_efforts_;

    // Start the controller from the current robot pose to avoid jumps.
    joint_position_commands_ = joint_positions_;
  }

  hardware_active_ = true;

  RCLCPP_INFO(logger_, "TL hardware interface activated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TLHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating TL hardware interface");

  hardware_active_ = false;

  RCLCPP_INFO(logger_, "TL hardware interface deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TLHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up TL hardware interface");

  hardware_active_ = false;
  hardware_connected_ = false;
  has_received_state_ = false;

  stop_executor();

  RCLCPP_INFO(logger_, "TL hardware interface cleaned up");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
TLHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(joint_names_.size() * 3);

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &joint_positions_[i]);

    state_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_VELOCITY,
      &joint_velocities_[i]);

    state_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_EFFORT,
      &joint_efforts_[i]);
  }

  RCLCPP_INFO(logger_, "Exported %zu state interfaces", state_interfaces.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TLHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &joint_position_commands_[i]);
  }

  RCLCPP_INFO(logger_, "Exported %zu command interfaces", command_interfaces.size());

  return command_interfaces;
}

hardware_interface::return_type TLHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!node_) {
    return hardware_interface::return_type::OK;
  }

  {
    std::lock_guard<std::mutex> lock(received_state_mutex_);

    if (has_received_state_) {
      joint_positions_ = received_positions_;
      joint_velocities_ = received_velocities_;
      joint_efforts_ = received_efforts_;
    }
  }

  const auto now = node_->now();
  const auto time_since_update = now - last_joint_state_time_;

  if (time_since_update.seconds() > state_timeout_sec_) {
    static auto last_warning = std::chrono::steady_clock::now();
    const auto now_steady = std::chrono::steady_clock::now();

    if (now_steady - last_warning >= 5s) {
      RCLCPP_WARN(
        logger_,
        "No joint state update for %.3f seconds",
        time_since_update.seconds());

      last_warning = now_steady;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TLHardwareInterface::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & /*period*/)
{
  if (!hardware_active_ || !joint_command_pub_) {
    return hardware_interface::return_type::OK;
  }

  sensor_msgs::msg::JointState command_msg;
  command_msg.header.stamp = time;
  command_msg.name = joint_names_;
  command_msg.position = joint_position_commands_;

  joint_command_pub_->publish(command_msg);

  return hardware_interface::return_type::OK;
}

bool TLHardwareInterface::validate_interfaces() const
{
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        logger_,
        "Joint '%s' has %zu command interfaces, expected exactly 1.",
        joint.name.c_str(),
        joint.command_interfaces.size());
      return false;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        logger_,
        "Joint '%s' has command interface '%s', expected '%s'.",
        joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return false;
    }

    bool has_position_state = false;
    bool has_velocity_state = false;

    for (const auto & state_interface : joint.state_interfaces) {
      if (state_interface.name == hardware_interface::HW_IF_POSITION) {
        has_position_state = true;
      }

      if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity_state = true;
      }
    }

    if (!has_position_state || !has_velocity_state) {
      RCLCPP_FATAL(
        logger_,
        "Joint '%s' must provide position and velocity state interfaces.",
        joint.name.c_str());
      return false;
    }
  }

  return true;
}

std::string TLHardwareInterface::get_hardware_parameter(
  const std::string & name,
  const std::string & default_value) const
{
  const auto it = info_.hardware_parameters.find(name);

  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }

  return it->second;
}

void TLHardwareInterface::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(received_state_mutex_);

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);

    if (it == msg->name.end()) {
      continue;
    }

    const size_t msg_index = std::distance(msg->name.begin(), it);

    if (msg_index < msg->position.size()) {
      received_positions_[i] = msg->position[msg_index];
    }

    if (msg_index < msg->velocity.size()) {
      received_velocities_[i] = msg->velocity[msg_index];
    }

    if (msg_index < msg->effort.size()) {
      received_efforts_[i] = msg->effort[msg_index];
    }
  }

  has_received_state_ = true;

  if (node_) {
    last_joint_state_time_ = node_->now();
  }
}

void TLHardwareInterface::stop_executor()
{
  if (executor_) {
    executor_->cancel();
  }

  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }

  if (executor_ && node_) {
    executor_->remove_node(node_);
  }

  joint_state_sub_.reset();
  joint_command_pub_.reset();
  executor_.reset();
  node_.reset();
}

}  // namespace tl_hardware

PLUGINLIB_EXPORT_CLASS(
  tl_hardware::TLHardwareInterface,
  hardware_interface::SystemInterface)