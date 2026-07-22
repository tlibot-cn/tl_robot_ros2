#ifndef TL_TELEOP_F710__TL_TELEOP_F710_NODE_H_
#define TL_TELEOP_F710__TL_TELEOP_F710_NODE_H_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <future>
#include <mutex>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tl_ros2_interface/msg/servol_move.hpp>
#include <tl_ros2_interface/srv/set_current_mode.hpp>
#include <tl_ros2_interface/srv/set_speed.hpp>
#include <tl_ros2_interface/srv/open_servo_j.hpp>
#include <tl_ros2_interface/srv/coord_transform.hpp>

// KDL FK (仿真模式)
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

class F710TeleopNode : public rclcpp::Node
{
public:
  F710TeleopNode();
  ~F710TeleopNode();

private:
  // ========== 参数声明与加载 ==========
  void declareParameters();
  void loadParameters();

  // ========== 工具函数 ==========
  static double applyDeadzone(double value, double deadzone);
  void publishServoj(const std::vector<double>& joint_pos);
  void publishServol();
  bool cartesianToJoint(const std::vector<double>& cart_pose, std::vector<double>& joint_out);
  bool currentJointToPose(std::vector<double>& pose_out);

  // ========== ServoJ 初始化 ==========
  void initServoj();
  void closeServoj();

  // ========== FK（关节→笛卡尔，用于回零和初始位姿）==========
  void initKDLFK();
  bool homeJointsToPose(std::vector<double>& pose_out);

  // ========== 回调 ==========
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void controlLoop();

  // ========== 参数 ==========
  double control_rate_{250.0};
  bool simulation_mode_;
  std::string arm_type_;
  double speed_default_, speed_min_, speed_max_, speed_step_;
  double servo_speed_, servo_vmax_{180.0}, servo_amax_, servo_jmax_;
  double pos_sensitivity_, rot_sensitivity_;
  double deadzone_;
  std::vector<double> home_joints_;
  int axis_left_x_, axis_left_y_;
  int axis_right_x_, axis_right_y_;
  int axis_dpad_x_, axis_dpad_y_;
  int btn_a_, btn_b_, btn_lb_, btn_rb_, btn_back_, btn_start_;

  // ========== 内部状态 ==========
  sensor_msgs::msg::Joy::SharedPtr latest_joy_;
  double speed_value_;
  std::vector<double> target_pose_;    // [x, y, z, rx, ry, rz]
  std::vector<double> last_joint_cmd_; // 上一帧关节角（每 4ms 发送）
  std::mutex joint_mutex_;
  std::atomic<bool> ik_pending_{false};
  bool stop_mode_;
  bool target_pose_ready_;
  bool joint_state_ready_{false};
  std::vector<double> current_joint_state_; // 当前关节角（来自 /joint_states 回调）

  rclcpp::Time last_dpad_time_;
  rclcpp::Time last_a_press_;
  bool prev_bs_{false}; // Back+Start 边缘检测

  // ========== ServoJ 初始化状态机 ==========
  // 0=等待服务 1=connect_arm 2=power_on 3=set_mode 4=set_speed 5=open_servoj 6=完成
  int init_state_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr connect_arm_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_on_client_;
  rclcpp::Client<tl_ros2_interface::srv::SetCurrentMode>::SharedPtr set_mode_client_;
  rclcpp::Client<tl_ros2_interface::srv::SetSpeed>::SharedPtr set_speed_client_;
  rclcpp::Client<tl_ros2_interface::srv::OpenServoJ>::SharedPtr open_servoj_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr close_servoj_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_off_client_;
  rclcpp::Client<tl_ros2_interface::srv::CoordTransform>::SharedPtr coord_transform_client_;

  // ========== KDL FK（仿真模式） ==========
  bool kdl_fk_ready_{false};
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver_;
  int ndof_{6};

  // ========== 异步 FK/IK ==========
  std::future<void> async_future_;

  // ========== ROS2 通信 ==========
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr servoj_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::ServolMove>::SharedPtr servol_pub_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

#endif // TL_TELEOP_F710__TL_TELEOP_F710_NODE_H_
