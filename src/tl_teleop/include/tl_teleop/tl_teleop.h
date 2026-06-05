#ifndef TL_TELEOP__TL_TELEOP_H_
#define TL_TELEOP__TL_TELEOP_H_

#include <array>
#include <atomic>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nlohmann/json.hpp>

#include "PXREARobotSDK.h"

#include <tl_ros2_interface/msg/cartesian_pose.hpp>
#include <tl_ros2_interface/srv/set_speed.hpp>
#include <tl_ros2_interface/srv/set_current_mode.hpp>
#include <tl_ros2_interface/srv/open_servo_j.hpp>
#include <tl_ros2_interface/srv/coord_transform.hpp>
#include <tl_ros2_interface/srv/get_pos_transform.hpp>

struct VRState
{
  std::array<double, 7> pose_{0.0};
  double grip_{0.0};
  bool a_button_{false};
  std::mutex mutex_;

  void update(const std::array<double, 7> & pose, double grip, bool a_button)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_ = pose;
    grip_ = grip;
    a_button_ = a_button;
  }

  void get(std::array<double, 7> & pose, double & grip, bool & a_button)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pose = pose_;
    grip = grip_;
    a_button = a_button_;
  }
};

class TL_Teleop : public rclcpp::Node
{
public:
  TL_Teleop();
  ~TL_Teleop();

  // C-linkage callback; must be static for PXREA SDK
  static void on_pxrea_client_cb(void * context,
    PXREAClientCallbackType type, int status, void * userData);

  void run_control_loop();
  void stop() { running_ = false; }

private:
  VRState vr_state_;
  std::atomic<bool> pxrea_ready_{false};

  std::array<double, 7> vr_home_pose_{0.0};
  std::array<double, 4> base_arm_quat_{0.0};
  std::array<double, 4> vr_home_quat_{0.0};
  bool vr_homed_{false};
  std::vector<double> last_joints_;
  std::mutex teleop_mutex_;

  int arm_joints_{7};
  double pos_scale_{0.5};
  double pos_deadzone_{0.005};
  double max_pos_delta_mm_{300.0};
  double singular_angle_{160.0};
  double singular_scale_{0.2};
  double joint_jump_threshold_{30.0};
  std::vector<std::pair<double, double>> joint_limits_;

  std::vector<double> servo_vmax_;
  std::vector<double> servo_amax_;
  std::vector<double> servo_jmax_;
  int servo_speed_{25};

  // ROS2 service clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_on_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_off_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_error_client_;
  rclcpp::Client<tl_ros2_interface::srv::SetSpeed>::SharedPtr set_speed_client_;
  rclcpp::Client<tl_ros2_interface::srv::SetCurrentMode>::SharedPtr set_current_mode_client_;
  rclcpp::Client<tl_ros2_interface::srv::OpenServoJ>::SharedPtr open_servoj_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr close_servoj_client_;
  rclcpp::Client<tl_ros2_interface::srv::CoordTransform>::SharedPtr coord_transform_client_;
  rclcpp::Client<tl_ros2_interface::srv::GetPosTransform>::SharedPtr get_rpy2quat_client_;

  // Topic publisher
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr servoj_pos_pub_;

  // Topic subscriber + cache
  rclcpp::Subscription<tl_ros2_interface::msg::CartesianPose>::SharedPtr tcp_pose_sub_;
  tl_ros2_interface::msg::CartesianPose latest_tcp_pose_;
  std::mutex tcp_pose_mutex_;
  bool tcp_pose_received_{false};

  std::atomic<bool> running_{true};

  bool init_servo();
  void close_servo();
  std::vector<double> get_arm_cartesian_pose();
  std::vector<double> get_inverse_kinematics(
    double x, double y, double z,
    double rx, double ry, double rz);
  std::array<double, 4> get_rpy2quat_sdk(double rx, double ry, double rz);
  std::vector<double> clamp_joints(const std::vector<double> & joints);
  bool joints_safe(const std::vector<double> & new_joints);
  void servoJ_send(const std::vector<double> & joint_target);

  static std::array<double, 4> quat_multiply(
    const std::array<double, 4> & q1, const std::array<double, 4> & q2);
  static std::array<double, 4> quat_inverse(const std::array<double, 4> & q);
  static std::array<double, 3> quat2rpy(const std::array<double, 4> & q);

  void control_loop();

  static std::array<double, 7> parse_pose_str(const std::string & s);
};

#endif  // TL_TELEOP__TL_TELEOP_H_