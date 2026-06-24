#ifndef TL_HARDWARE_INTERFACE_HPP
#define TL_HARDWARE_INTERFACE_HPP

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/client.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tl_ros2_interface/srv/open_servo_j.hpp>

namespace tl_hardware
{

class TLHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TLHardwareInterface)

  TLHardwareInterface();
  ~TLHardwareInterface() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  bool validate_interfaces() const;

  std::string get_hardware_parameter(const std::string& name, const std::string& default_value) const;

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  void stop_executor();

  rclcpp::Logger logger_;

  hardware_interface::HardwareInfo info_;

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr servoj_pos_pub_;

  rclcpp::Client<tl_ros2_interface::srv::OpenServoJ>::SharedPtr open_servoj_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr close_servoj_client_;

  std::string joint_states_topic_;
  std::string servoj_topic_;
  std::string open_servoj_service_;
  std::string close_servoj_service_;

  std::vector<std::string> joint_names_;

  // These vectors are exported to ros2_control state interfaces.
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;

  // These vectors are written by the joint_trajectory_controller.
  std::vector<double> joint_position_commands_;

  // These vectors are updated only by the subscription callback.
  std::vector<double> received_positions_;
  std::vector<double> received_velocities_;
  std::vector<double> received_efforts_;

  // Servoj streaming parameters (one value per joint, in deg units).
  std::vector<double> servoj_vmax_;
  std::vector<double> servoj_amax_;
  std::vector<double> servoj_jmax_;

  // Previous read state for velocity computation.
  std::vector<double> last_positions_;
  rclcpp::Time last_read_time_;

  std::mutex received_state_mutex_;

  std::atomic<bool> hardware_connected_{false};
  std::atomic<bool> hardware_active_{false};
  std::atomic<bool> has_received_state_{false};

  rclcpp::Time last_joint_state_time_;

  double state_timeout_sec_{1.0};
};

} // namespace tl_hardware

#endif // TL_HARDWARE_INTERFACE_HPP
