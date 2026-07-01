/**
 * @file tl_teleop_f710_sim_bridge.cpp
 * @brief Gazebo 仿真桥接节点 — 使用 KDL 做 IK 求解
 *
 * 在 Gazebo 仿真环境中模拟 tl_driver 的 servol 控制链路：
 *   1. 订阅 /tl_driver/set_servol_pos（ServolMove）
 *   2. 使用 KDL ChainIkSolverPos_LMA 对目标笛卡尔位姿做逆运动学（IK）
 *   3. 将求解的关节角度发送到 Gazebo 的 position controller
 *
 * 无需 MoveIt2，IK 在节点内本地计算（基于 KDL）。
 */

#include "tl_teleop_f710/tl_teleop_f710_sim_bridge.h"

#include <cmath>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

ServolSimBridge::ServolSimBridge() : rclcpp::Node("tl_teleop_f710_sim_bridge"), ndof_(0)
{
  // ===== 参数 =====
  this->declare_parameter("arm_type", "tcb605");
  this->declare_parameter("position_controller_topic", "/tcb_group_position_controller/commands");
  this->declare_parameter("joint_state_topic", "/joint_states");
  this->declare_parameter("ik_eps", 1e-4);
  this->declare_parameter("ik_max_iter", 200);
  this->declare_parameter("ik_dt", 0.5);

  arm_type_ = this->get_parameter("arm_type").as_string();
  position_controller_topic_ = this->get_parameter("position_controller_topic").as_string();
  joint_state_topic_ = this->get_parameter("joint_state_topic").as_string();
  ik_eps_ = this->get_parameter("ik_eps").as_double();
  ik_max_iter_ = this->get_parameter("ik_max_iter").as_int();
  ik_dt_ = this->get_parameter("ik_dt").as_double();

  // ===== 初始化 KDL 运动学（动态轴数） =====
  initKDL();

  // 初始化关节角缓存
  current_joints_.resize(ndof_);
  for (int i = 0; i < ndof_; ++i)
  {
    current_joints_(i) = 0.0;
  }

  // ===== 订阅 =====
  servol_sub_ = this->create_subscription<tl_ros2_interface::msg::ServolMove>(
      "/tl_driver/set_servol_pos", 10, std::bind(&ServolSimBridge::servolCallback, this, std::placeholders::_1));

  js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 10, std::bind(&ServolSimBridge::jointStateCallback, this, std::placeholders::_1));

  // ===== 发布 =====
  pos_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(position_controller_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "仿真桥接节点已启动 (KDL IK, max_iter=%d, eps=%.1e)", ik_max_iter_, ik_eps_);
}

void ServolSimBridge::initKDL()
{
  // 查找 URDF 文件（基于 arm_type_）
  std::string urdf_path;
  std::string urdf_file = arm_type_ + ".urdf";

  try
  {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("tl_description");
    urdf_path = pkg_path + "/urdf/" + urdf_file;
  }
  catch (...)
  {
    urdf_path = std::string(std::getenv("HOME")) + "/tl_robot_ros2_py/src/tl_description/urdf/" + urdf_file;
  }

  if (access(urdf_path.c_str(), F_OK) != 0)
  {
    RCLCPP_FATAL(this->get_logger(), "找不到 URDF 文件: %s", urdf_path.c_str());
    throw std::runtime_error("URDF not found");
  }

  RCLCPP_INFO(this->get_logger(), "加载 URDF: %s", urdf_path.c_str());

  KDL::Tree tree;
  if (!kdl_parser::treeFromFile(urdf_path, tree))
  {
    RCLCPP_FATAL(this->get_logger(), "无法从 URDF 解析 KDL tree");
    throw std::runtime_error("Failed to parse URDF to KDL tree");
  }

  // 动态确定轴数：从 joint_names 统计或从 URDF 推断
  // 先获取根到所有可能 tip 的 chain 来确定 ndof
  // 从高到低尝试（7→6），确保 7 轴臂优先匹配 7 轴链
  ndof_ = 0;
  std::string tip_link;
  for (int n = 7; n >= 6; --n)
  {
    tip_link = "link" + std::to_string(n);
    KDL::Chain test_chain;
    if (tree.getChain("link0", tip_link, test_chain))
    {
      ndof_ = test_chain.getNrOfJoints();
      chain_ = test_chain;
      break;
    }
  }

  if (ndof_ == 0)
  {
    RCLCPP_FATAL(this->get_logger(), "无法获取 KDL chain (link0 -> linkN)");
    throw std::runtime_error("Failed to get KDL chain");
  }

  RCLCPP_INFO(this->get_logger(), "KDL chain 加载完成: %d 个关节, tip=%s", ndof_, tip_link.c_str());

  // 动态构建关节名
  joint_names_.clear();
  for (int i = 1; i <= ndof_; ++i)
  {
    joint_names_.push_back("joint" + std::to_string(i));
  }

  // 创建 FK 和 IK 求解器
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);
}

bool ServolSimBridge::solveIK(double x, double y, double z, double rx, double ry, double rz,
                              const KDL::JntArray& q_guess, KDL::JntArray& q_out)
{
  // 构建目标位姿
  KDL::Frame target_frame;
  target_frame.p = KDL::Vector(x, y, z);
  target_frame.M = KDL::Rotation::RPY(rx, ry, rz);

  // IK 求解（LMA 迭代法，内建阻尼）
  q_out.resize(ndof_);
  int ret = ik_solver_->CartToJnt(q_guess, target_frame, q_out);

  return (ret == 0); // KDL 返回 0 表示成功
}

void ServolSimBridge::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // 缓存最新关节状态
  std::vector<double> positions(ndof_, 0.0);
  int found = 0;

  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
  {
    for (int j = 0; j < ndof_; ++j)
    {
      if (msg->name[i] == joint_names_[j])
      {
        positions[j] = msg->position[i];
        found++;
        break;
      }
    }
  }

  if (found == ndof_)
  {
    std::lock_guard<std::mutex> lock(joints_mutex_);
    for (int i = 0; i < ndof_; ++i)
    {
      current_joints_(i) = positions[i];
    }
  }
}

void ServolSimBridge::servolCallback(const tl_ros2_interface::msg::ServolMove::SharedPtr msg)
{
  const auto& target = msg->target_pose;
  if (target.size() < 6)
  {
    RCLCPP_ERROR(this->get_logger(), "target_pose 长度不足: %zu", target.size());
    return;
  }

  // 单位换算：servol 使用 mm，KDL 使用 m
  double x_m = target[0] / 1000.0;
  double y_m = target[1] / 1000.0;
  double z_m = target[2] / 1000.0;

  // 取当前关节角作为 IK 初始猜测
  KDL::JntArray q_guess(ndof_), q_out(ndof_);
  {
    std::lock_guard<std::mutex> lock(joints_mutex_);
    q_guess = current_joints_;
  }

  // IK 求解
  bool ok = solveIK(x_m, y_m, z_m, target[3], target[4], target[5], q_guess, q_out);

  if (!ok)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "IK 求解失败 target=(%.1f, %.1f, %.1f)",
                         target[0], target[1], target[2]);
    return;
  }

  // 更新关节缓存为 IK 结果
  {
    std::lock_guard<std::mutex> lock(joints_mutex_);
    current_joints_ = q_out;
  }

  // 发送到 position controller（弧度）
  std_msgs::msg::Float64MultiArray cmd_msg;
  cmd_msg.data.resize(ndof_);
  for (int i = 0; i < ndof_; ++i)
  {
    cmd_msg.data[i] = q_out(i);
  }
  pos_pub_->publish(cmd_msg);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServolSimBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
