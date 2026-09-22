#ifndef TL_TELEOP_F710__TL_TELEOP_F710_SIM_BRIDGE_H_
#define TL_TELEOP_F710__TL_TELEOP_F710_SIM_BRIDGE_H_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tl_ros2_interface/msg/servol_move.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

class ServolSimBridge : public rclcpp::Node
{
public:
  ServolSimBridge();
  ~ServolSimBridge() = default;

private:
  // ========== 初始化 ==========
  void initKDL();

  // ========== IK 求解 ==========
  bool solveIK(double x, double y, double z, double rx, double ry, double rz, const KDL::JntArray& q_guess,
               KDL::JntArray& q_out);

  // ========== 回调 ==========
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void servolCallback(const tl_ros2_interface::msg::ServolMove::SharedPtr msg);

  // ========== 参数 ==========
  std::string arm_type_;
  std::string position_controller_topic_;
  std::string joint_state_topic_;
  double ik_eps_;
  int ik_max_iter_;
  double ik_dt_;

  // ========== KDL 运动学 ==========
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
  int ndof_{0};

  // ========== 关节名 ==========
  std::vector<std::string> joint_names_;

  // ========== 内部状态 ==========
  KDL::JntArray current_joints_;
  std::mutex joints_mutex_;

  // ========== ROS2 通信 ==========
  rclcpp::Subscription<tl_ros2_interface::msg::ServolMove>::SharedPtr servol_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pos_pub_;
};

#endif // TL_TELEOP_F710__TL_TELEOP_F710_SIM_BRIDGE_H_
