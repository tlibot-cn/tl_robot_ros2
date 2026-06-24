#include "tl_hardware/tl_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace tl_hardware
{

TLHardwareInterface::TLHardwareInterface()
    : logger_(rclcpp::get_logger("tl_hardware_interface")), last_read_time_(0, 0, RCL_ROS_TIME),
      last_joint_state_time_(0, 0, RCL_ROS_TIME)
{
}

TLHardwareInterface::~TLHardwareInterface()
{
  stop_executor();
}

hardware_interface::CallbackReturn TLHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_INFO(logger_, "Initializing TL hardware interface");

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;

  if (!validate_interfaces())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  joint_names_.reserve(info_.joints.size());

  for (const auto& joint : info_.joints)
  {
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

  last_positions_.assign(joint_count, 0.0);

  joint_states_topic_ = get_hardware_parameter("joint_states_topic", "/joint_states");

  servoj_topic_ = get_hardware_parameter("servoj_topic", "/tl_driver/set_servoj_pos");

  open_servoj_service_ = get_hardware_parameter("open_servoj_service", "/tl_driver/open_servoj");

  close_servoj_service_ = get_hardware_parameter("close_servoj_service", "/tl_driver/close_servoj");

  // Parse servoj velocity/acceleration/jerk per joint.
  // Parameters accept a single scalar (expand to all joints) or
  // comma-separated per-joint values.
  auto parse_vector_param = [&](const std::string& name, double default_val) -> std::vector<double>
  {
    const auto raw = get_hardware_parameter(name, std::to_string(default_val));
    std::vector<double> result(joint_count, default_val);

    // Check if comma-separated.
    if (raw.find(',') != std::string::npos)
    {
      result.clear();
      std::string token;
      for (size_t pos = 0, end = 0; end != std::string::npos; pos = end + 1)
      {
        end = raw.find(',', pos);
        token = raw.substr(pos, end - pos);
        try
        {
          result.push_back(std::stod(token));
        }
        catch (const std::exception&)
        {
          RCLCPP_WARN(logger_,
                      "Invalid value '%s' in parameter '%s', using "
                      "default %.1f",
                      token.c_str(), name.c_str(), default_val);
          return std::vector<double>(joint_count, default_val);
        }
      }
      if (result.size() != joint_count)
      {
        RCLCPP_WARN(logger_,
                    "Parameter '%s' has %zu values but %zu joints, "
                    "using default %.1f",
                    name.c_str(), result.size(), joint_count, default_val);
        return std::vector<double>(joint_count, default_val);
      }
    }
    else
    {
      try
      {
        double val = std::stod(raw);
        result.assign(joint_count, val);
      }
      catch (const std::exception&)
      {
        RCLCPP_WARN(logger_, "Invalid value '%s' for parameter '%s', using default %.1f", raw.c_str(), name.c_str(),
                    default_val);
      }
    }
    return result;
  };

  servoj_vmax_ = parse_vector_param("servoj_vmax", 30.0);
  servoj_amax_ = parse_vector_param("servoj_amax", 100.0);
  servoj_jmax_ = parse_vector_param("servoj_jmax", 500.0);

  const auto state_timeout = get_hardware_parameter("state_timeout_sec", "1.0");
  try
  {
    state_timeout_sec_ = std::stod(state_timeout);
  }
  catch (const std::exception&)
  {
    RCLCPP_WARN(logger_, "Invalid state_timeout_sec '%s', using default 1.0", state_timeout.c_str());
    state_timeout_sec_ = 1.0;
  }

  RCLCPP_INFO(logger_, "Initialized with %zu joints", joint_count);
  RCLCPP_INFO(logger_, "Joint states topic: %s", joint_states_topic_.c_str());
  RCLCPP_INFO(logger_, "Servoj topic: %s", servoj_topic_.c_str());
  RCLCPP_INFO(logger_, "Open servoj service: %s", open_servoj_service_.c_str());
  RCLCPP_INFO(logger_, "Close servoj service: %s", close_servoj_service_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TLHardwareInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Configuring TL hardware interface");

  try
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }

    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);

    node_ = std::make_shared<rclcpp::Node>("tl_hardware", node_options);

    // Publisher for servoj position commands (Float64MultiArray in
    // degrees).
    servoj_pos_pub_ =
        node_->create_publisher<std_msgs::msg::Float64MultiArray>(servoj_topic_, rclcpp::QoS(10).reliable());

    // Service clients for servoj lifecycle.
    open_servoj_client_ = node_->create_client<tl_ros2_interface::srv::OpenServoJ>(open_servoj_service_);

    close_servoj_client_ = node_->create_client<std_srvs::srv::Trigger>(close_servoj_service_);

    // Subscriber for joint states from tl_driver.
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, rclcpp::QoS(10).best_effort(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
          joint_state_callback(msg);
        });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    executor_thread_ = std::thread(
        [this]()
        {
          RCLCPP_INFO(logger_, "Starting TL hardware executor thread");
          executor_->spin();
          RCLCPP_INFO(logger_, "TL hardware executor thread stopped");
        });

    hardware_connected_ = true;
    hardware_active_ = false;
    has_received_state_ = false;
    last_joint_state_time_ = node_->now();
    last_read_time_ = node_->now();

    RCLCPP_INFO(logger_, "TL hardware interface configured successfully");
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger_, "Failed to configure hardware interface: %s", e.what());
    stop_executor();
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TLHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating TL hardware interface");

  if (!hardware_connected_ || !node_)
  {
    RCLCPP_ERROR(logger_, "Hardware interface is not configured");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Open servoj streaming before activating.
  if (!open_servoj_client_->wait_for_service(5s))
  {
    RCLCPP_ERROR(logger_, "open_servoj service not available");
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto request = std::make_shared<tl_ros2_interface::srv::OpenServoJ::Request>();
  request->vmax = servoj_vmax_;
  request->amax = servoj_amax_;
  request->jmax = servoj_jmax_;

  auto future = open_servoj_client_->async_send_request(request);
  auto status = future.wait_for(5s);

  if (status != std::future_status::ready)
  {
    RCLCPP_ERROR(logger_, "Timeout waiting for open_servoj response");
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto response = future.get();
  if (!response->success)
  {
    RCLCPP_ERROR(logger_, "open_servoj failed: %s", response->message.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "open_servoj succeeded");

  // Wait for initial joint states.
  const auto start_time = std::chrono::steady_clock::now();

  while (!has_received_state_ && std::chrono::steady_clock::now() - start_time < 5s)
  {
    std::this_thread::sleep_for(50ms);
  }

  if (!has_received_state_)
  {
    RCLCPP_ERROR(logger_, "Timeout waiting for initial joint states");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Seed state vectors and initialize velocity computation state.
  {
    std::lock_guard<std::mutex> lock(received_state_mutex_);

    joint_positions_ = received_positions_;
    joint_velocities_ = received_velocities_;
    joint_efforts_ = received_efforts_;

    last_positions_ = received_positions_;
    last_read_time_ = node_->now();
  }

  hardware_active_ = true;

  RCLCPP_INFO(logger_, "TL hardware interface activated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TLHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating TL hardware interface");

  hardware_active_ = false;

  // Close servoj streaming — warn on failure but don't block deactivation.
  if (close_servoj_client_ && close_servoj_client_->wait_for_service(5s))
  {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = close_servoj_client_->async_send_request(request);
    auto status = future.wait_for(5s);

    if (status != std::future_status::ready)
    {
      RCLCPP_WARN(logger_, "Timeout waiting for close_servoj response");
    }
    else
    {
      auto response = future.get();
      if (!response->success)
      {
        RCLCPP_WARN(logger_, "close_servoj failed: %s", response->message.c_str());
      }
      else
      {
        RCLCPP_INFO(logger_, "close_servoj succeeded");
      }
    }
  }
  else
  {
    RCLCPP_WARN(logger_, "close_servoj service not available");
  }

  RCLCPP_INFO(logger_, "TL hardware interface deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TLHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up TL hardware interface");

  hardware_active_ = false;
  hardware_connected_ = false;
  has_received_state_ = false;

  stop_executor();

  RCLCPP_INFO(logger_, "TL hardware interface cleaned up");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TLHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(joint_names_.size() * 3);

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_positions_[i]);

    state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]);

    state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]);
  }

  RCLCPP_INFO(logger_, "Exported %zu state interfaces", state_interfaces.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TLHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]);
  }

  RCLCPP_INFO(logger_, "Exported %zu command interfaces", command_interfaces.size());

  return command_interfaces;
}

hardware_interface::return_type TLHardwareInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  if (!node_)
  {
    return hardware_interface::return_type::OK;
  }

  {
    std::lock_guard<std::mutex> lock(received_state_mutex_);

    if (has_received_state_)
    {
      joint_positions_ = received_positions_;
      joint_efforts_ = received_efforts_;
    }
  }

  // Compute velocity from position difference.
  const double dt = period.seconds();
  if (dt > 0.0 && has_received_state_ && !last_positions_.empty())
  {
    for (size_t i = 0; i < joint_positions_.size(); ++i)
    {
      joint_velocities_[i] = (joint_positions_[i] - last_positions_[i]) / dt;
    }
  }
  else
  {
    std::fill(joint_velocities_.begin(), joint_velocities_.end(), 0.0);
  }

  if (has_received_state_)
  {
    last_positions_ = joint_positions_;
    last_read_time_ = node_->now();
  }

  const auto now = node_->now();
  const auto time_since_update = now - last_joint_state_time_;

  if (time_since_update.seconds() > state_timeout_sec_)
  {
    static auto last_warning = std::chrono::steady_clock::now();
    const auto now_steady = std::chrono::steady_clock::now();

    if (now_steady - last_warning >= 5s)
    {
      RCLCPP_WARN(logger_, "No joint state update for %.3f seconds", time_since_update.seconds());

      last_warning = now_steady;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TLHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                           const rclcpp::Duration& /*period*/)
{
  if (!hardware_active_ || !servoj_pos_pub_)
  {
    return hardware_interface::return_type::OK;
  }

  // Convert radians → degrees and publish to servoj topic.
  auto command_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  command_msg->data.reserve(joint_position_commands_.size());

  for (const auto& rad_cmd : joint_position_commands_)
  {
    command_msg->data.push_back(rad_cmd * 180.0 / M_PI);
  }

  servoj_pos_pub_->publish(std::move(command_msg));

  return hardware_interface::return_type::OK;
}

bool TLHardwareInterface::validate_interfaces() const
{
  for (const auto& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(logger_, "Joint '%s' has %zu command interfaces, expected exactly 1.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return false;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(logger_, "Joint '%s' has command interface '%s', expected '%s'.", joint.name.c_str(),
                   joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return false;
    }

    bool has_position_state = false;
    bool has_velocity_state = false;

    for (const auto& state_interface : joint.state_interfaces)
    {
      if (state_interface.name == hardware_interface::HW_IF_POSITION)
      {
        has_position_state = true;
      }

      if (state_interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        has_velocity_state = true;
      }
    }

    if (!has_position_state || !has_velocity_state)
    {
      RCLCPP_FATAL(logger_,
                   "Joint '%s' must provide position and velocity state "
                   "interfaces.",
                   joint.name.c_str());
      return false;
    }
  }

  return true;
}

std::string TLHardwareInterface::get_hardware_parameter(const std::string& name, const std::string& default_value) const
{
  const auto it = info_.hardware_parameters.find(name);

  if (it == info_.hardware_parameters.end())
  {
    return default_value;
  }

  return it->second;
}

void TLHardwareInterface::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(received_state_mutex_);

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    const auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);

    if (it == msg->name.end())
    {
      continue;
    }

    const size_t msg_index = std::distance(msg->name.begin(), it);

    if (msg_index < msg->position.size())
    {
      received_positions_[i] = msg->position[msg_index];
    }

    if (msg_index < msg->velocity.size())
    {
      received_velocities_[i] = msg->velocity[msg_index];
    }

    if (msg_index < msg->effort.size())
    {
      received_efforts_[i] = msg->effort[msg_index];
    }
  }

  has_received_state_ = true;

  if (node_)
  {
    last_joint_state_time_ = node_->now();
  }
}

void TLHardwareInterface::stop_executor()
{
  if (executor_)
  {
    executor_->cancel();
  }

  if (executor_thread_.joinable())
  {
    executor_thread_.join();
  }

  if (executor_ && node_)
  {
    executor_->remove_node(node_);
  }

  joint_state_sub_.reset();
  servoj_pos_pub_.reset();
  open_servoj_client_.reset();
  close_servoj_client_.reset();
  executor_.reset();
  node_.reset();
}

} // namespace tl_hardware

PLUGINLIB_EXPORT_CLASS(tl_hardware::TLHardwareInterface, hardware_interface::SystemInterface)
