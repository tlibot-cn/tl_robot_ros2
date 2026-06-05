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
#include <sensor_msgs/msg/joint_state.hpp>
#include <nlohmann/json.hpp>

#include "PXREARobotSDK.h"
#include "tl_interface.h"

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
  SOCKETFD socket_fd_{0};
  SOCKETFD socket_fd_aux_{0};
  bool arm_connected_{false};
  bool is_powered_{false};
  std::mutex arm_mutex_;

  VRState vr_state_;
  std::atomic<bool> pxrea_ready_{false};

  std::array<double, 7> vr_home_pose_{0.0};
  std::array<double, 4> base_arm_quat_{0.0};
  std::array<double, 4> vr_home_quat_{0.0};
  bool vr_homed_{false};
  std::vector<double> last_joints_;
  std::mutex teleop_mutex_;

  std::vector<double> latest_target_joints_;
  std::mutex joints_mutex_;

  std::string arm_ip_;
  std::string arm_port_;
  std::string arm_port_aux_;
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

  double publish_rate_{20.0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::vector<std::string> joint_names_;

  std::atomic<bool> running_{true};

  bool connect_arm();
  void disconnect_arm();
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

  void publish_joints();

  static std::array<double, 7> parse_pose_str(const std::string & s);
};

#endif  // TL_TELEOP__TL_TELEOP_H_