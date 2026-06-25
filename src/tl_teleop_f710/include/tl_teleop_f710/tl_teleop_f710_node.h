#ifndef TL_TELEOP_F710__TL_TELEOP_F710_NODE_H_
#define TL_TELEOP_F710__TL_TELEOP_F710_NODE_H_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
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
  void publishServol();

  // ========== ServoJ 初始化 ==========
  void initServoj();
  void afterInit();
  void closeServoj();

  // ========== FK（关节→笛卡尔，用于回零和初始位姿）==========
  void initKDLFK();
  bool homeJointsToPose(std::vector<double> & pose_out);

  // ========== 回调 ==========
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void controlLoop();

  // ========== 参数 ==========
  // 控制
  double control_rate_;
  bool simulation_mode_;
  std::string arm_type_;
  // 速度
  double speed_default_, speed_min_, speed_max_, speed_step_;
  // ServoJ
  double servo_speed_, servo_vmax_, servo_amax_, servo_jmax_;
  // 灵敏度
  double pos_sensitivity_, rot_sensitivity_, step_size_;
  double deadzone_;
  std::vector<double> home_joints_;  // 回零关节角度（度）
  // 轴映射
  int axis_left_x_, axis_left_y_;
  int axis_right_x_, axis_right_y_;
  int axis_dpad_x_, axis_dpad_y_;
  // 按键映射
  int btn_a_, btn_b_, btn_lb_, btn_rb_;

  // ========== 内部状态 ==========
  sensor_msgs::msg::Joy::SharedPtr latest_joy_;
  double speed_value_;
  std::vector<double> target_pose_;  // [x, y, z, rx, ry, rz]
  bool stop_mode_;                   // B 键按下后进入停止模式
  bool target_pose_ready_;           // FK 初始化完成前为 false
  bool home_request_pending_;        // A 键回零请求待处理（避免重复触发）

  // 防抖时间戳
  rclcpp::Time last_dpad_time_;
  rclcpp::Time last_a_press_;
  rclcpp::Time last_b_press_;

  // ========== ServoJ 初始化状态机 ==========
  // 0=等待服务 1=等待set_mode 2=等待set_speed 3=等待open_servoj 4=完成
  int init_state_;
  rclcpp::Client<tl_ros2_interface::srv::SetCurrentMode>::SharedPtr set_mode_client_;
  rclcpp::Client<tl_ros2_interface::srv::SetSpeed>::SharedPtr set_speed_client_;
  rclcpp::Client<tl_ros2_interface::srv::OpenServoJ>::SharedPtr open_servoj_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr close_servoj_client_;
  rclcpp::Client<tl_ros2_interface::srv::CoordTransform>::SharedPtr coord_transform_client_;

  // ========== KDL FK（仿真模式） ==========
  bool kdl_fk_ready_{false};
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver_;
  int ndof_{6};

  // ========== 异步 FK 初始化 ==========
  std::future<void> after_init_future_;

  // ========== ROS2 通信 ==========
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<tl_ros2_interface::msg::ServolMove>::SharedPtr servol_pub_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

#endif  // TL_TELEOP_F710__TL_TELEOP_F710_NODE_H_
