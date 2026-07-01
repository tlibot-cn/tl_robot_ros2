/**
 * @file tl_teleop_f710_node.cpp
 * @brief 天链机械臂 Logitech F710 手柄遥操作节点 (C++)
 *
 * 直接 ServoJ 方案（250Hz）：
 *   F710 -> joy_node -> /joy -> 本节点 (IK) -> /tl_driver/set_servoj_pos -> 机械臂
 *
 * 每 4ms 稳定输出一帧关节角，指令流从不中断。
 * 有摇杆输入时异步调用 coord_transform 做 IK，更新目标关节角。
 *
 * 控制模式（笛卡尔空间，基座标系）：
 *   - 左摇杆  -> X/Y 平移
 *   - 右摇杆  -> Z 平移 + 偏航（默认）/ 翻滚（LB）/ 俯仰（RB）
 *   - 十字键  -> 速度倍率调节
 *   - A 键    -> 回零
 *   - Back+Start -> 紧急停止
 */

#include "tl_teleop_f710/tl_teleop_f710_node.h"

#include <cmath>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>

F710TeleopNode::F710TeleopNode()
    : rclcpp::Node("tl_teleop_f710_node"), latest_joy_(nullptr), speed_value_(50.0), stop_mode_(false),
      target_pose_ready_(false), init_state_(0), kdl_fk_ready_(false)
{
  declareParameters();
  loadParameters();

  speed_value_ = speed_default_;

  // 初始化防抖时间戳
  last_dpad_time_ = this->now();
  last_a_press_ = this->now();

  // 初始化缓存关节角（6 或 7 轴）
  int ndof = static_cast<int>(home_joints_.size());
  last_joint_cmd_.assign(ndof, 0.0);

  // 订阅 /joy
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&F710TeleopNode::joyCallback, this, std::placeholders::_1));

  // 订阅 /joint_states（用于获取当前关节角初始化）
  js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&F710TeleopNode::jointStateCallback, this, std::placeholders::_1));

  // 发布 /tl_driver/set_servoj_pos（Float64MultiArray）
  servoj_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tl_driver/set_servoj_pos", 10);
  // 发布 /tl_driver/set_servol_pos（ServolMove，仿真模式用）
  servol_pub_ = this->create_publisher<tl_ros2_interface::msg::ServolMove>("/tl_driver/set_servol_pos", 10);

  // 服务客户端
  set_mode_client_ = this->create_client<tl_ros2_interface::srv::SetCurrentMode>("/tl_driver/set_current_mode");
  set_speed_client_ = this->create_client<tl_ros2_interface::srv::SetSpeed>("/tl_driver/set_speed");
  open_servoj_client_ = this->create_client<tl_ros2_interface::srv::OpenServoJ>("/tl_driver/open_servoj");
  close_servoj_client_ = this->create_client<std_srvs::srv::Trigger>("/tl_driver/close_servoj");
  connect_arm_client_ = this->create_client<std_srvs::srv::Trigger>("/tl_driver/connect_arm");
  power_on_client_ = this->create_client<std_srvs::srv::Trigger>("/tl_driver/power_on");
  coord_transform_client_ = this->create_client<tl_ros2_interface::srv::CoordTransform>("/tl_driver/coord_transform");
  power_off_client_ = this->create_client<std_srvs::srv::Trigger>("/tl_driver/power_off");

  // 仿真模式：提前初始化 KDL FK（jointStateCallback 首帧触发初始位姿）
  if (simulation_mode_)
  {
    initKDLFK();
  }

  // ServoJ 初始化定时器（1Hz）
  init_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&F710TeleopNode::initServoj, this));

  // 控制定时器（250Hz = 4ms）
  control_timer_ = this->create_wall_timer(std::chrono::milliseconds(4), std::bind(&F710TeleopNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "F710 遥操作节点已启动 (250Hz, 灵敏度 %.1fmm/s, vmax=%.1f°/s)", pos_sensitivity_,
              servo_vmax_);
}

F710TeleopNode::~F710TeleopNode()
{
  closeServoj();
}

// ========== 参数声明 ==========

void F710TeleopNode::declareParameters()
{
  this->declare_parameter("control_rate", 250.0);
  this->declare_parameter("simulation_mode", false);
  this->declare_parameter("arm_type", "tcb605");
  this->declare_parameter("speed_default", 50.0);
  this->declare_parameter("speed_min", 5.0);
  this->declare_parameter("speed_max", 100.0);
  this->declare_parameter("speed_step", 5.0);
  this->declare_parameter("servo_speed", 25.0);
  this->declare_parameter("servo_vmax", 180.0);
  this->declare_parameter("servo_amax", 3000.0);
  this->declare_parameter("servo_jmax", 50000.0);
  this->declare_parameter("pos_sensitivity", 160.0);
  this->declare_parameter("rot_sensitivity", 1.0);
  this->declare_parameter("deadzone", 0.15);
  this->declare_parameter("home_joints", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("axis_left_x", 0);
  this->declare_parameter("axis_left_y", 1);
  this->declare_parameter("axis_right_x", 3);
  this->declare_parameter("axis_right_y", 4);
  this->declare_parameter("axis_dpad_x", 6);
  this->declare_parameter("axis_dpad_y", 7);
  this->declare_parameter("btn_x", 0);
  this->declare_parameter("btn_a", 1);
  this->declare_parameter("btn_b", 2);
  this->declare_parameter("btn_y", 3);
  this->declare_parameter("btn_lb", 4);
  this->declare_parameter("btn_rb", 5);
  this->declare_parameter("btn_lt", 6);
  this->declare_parameter("btn_rt", 7);
  this->declare_parameter("btn_back", 8);
  this->declare_parameter("btn_start", 9);
}

// ========== 参数加载 ==========

void F710TeleopNode::loadParameters()
{
  control_rate_ = this->get_parameter("control_rate").as_double();
  simulation_mode_ = this->get_parameter("simulation_mode").as_bool();
  arm_type_ = this->get_parameter("arm_type").as_string();
  speed_default_ = this->get_parameter("speed_default").as_double();
  speed_min_ = this->get_parameter("speed_min").as_double();
  speed_max_ = this->get_parameter("speed_max").as_double();
  speed_step_ = this->get_parameter("speed_step").as_double();
  servo_speed_ = this->get_parameter("servo_speed").as_double();
  servo_vmax_ = this->get_parameter("servo_vmax").as_double();
  servo_amax_ = this->get_parameter("servo_amax").as_double();
  servo_jmax_ = this->get_parameter("servo_jmax").as_double();
  pos_sensitivity_ = this->get_parameter("pos_sensitivity").as_double();
  rot_sensitivity_ = this->get_parameter("rot_sensitivity").as_double();
  deadzone_ = this->get_parameter("deadzone").as_double();
  home_joints_ = this->get_parameter("home_joints").as_double_array();

  axis_left_x_ = this->get_parameter("axis_left_x").as_int();
  axis_left_y_ = this->get_parameter("axis_left_y").as_int();
  axis_right_x_ = this->get_parameter("axis_right_x").as_int();
  axis_right_y_ = this->get_parameter("axis_right_y").as_int();
  axis_dpad_x_ = this->get_parameter("axis_dpad_x").as_int();
  axis_dpad_y_ = this->get_parameter("axis_dpad_y").as_int();

  btn_a_ = this->get_parameter("btn_a").as_int();
  btn_b_ = this->get_parameter("btn_b").as_int();
  btn_lb_ = this->get_parameter("btn_lb").as_int();
  btn_rb_ = this->get_parameter("btn_rb").as_int();
  btn_back_ = this->get_parameter("btn_back").as_int();
  btn_start_ = this->get_parameter("btn_start").as_int();
}

// ========== 工具函数 ==========

double F710TeleopNode::applyDeadzone(double value, double deadzone)
{
  if (std::abs(value) < deadzone)
    return 0.0;
  double sign = (value > 0.0) ? 1.0 : -1.0;
  return sign * (std::abs(value) - deadzone) / (1.0 - deadzone);
}

void F710TeleopNode::publishServoj(const std::vector<double>& joint_pos)
{
  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data = joint_pos;
  servoj_pub_->publish(msg);
}

void F710TeleopNode::publishServol()
{
  if (target_pose_.size() < 6)
    return;
  auto msg = tl_ros2_interface::msg::ServolMove();
  msg.target_pose = target_pose_;
  msg.step_size = 5.0;
  msg.coord = 1; // 基座标系
  servol_pub_->publish(msg);
}

bool F710TeleopNode::cartesianToJoint(const std::vector<double>& cart_pose, std::vector<double>& joint_out)
{
  if (!coord_transform_client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(this->get_logger(), "coord_transform 服务不可用");
    return false;
  }

  auto req = std::make_shared<tl_ros2_interface::srv::CoordTransform::Request>();
  req->origin_coord = 1; // 笛卡尔坐标系
  req->target_coord = 0; // 关节坐标系
  req->form = 0;

  // origin_pos: 6 元素笛卡尔位姿，补齐到 7
  std::vector<double> pos = cart_pose;
  pos.resize(7, 0.0);
  req->origin_pos = pos;

  // reference_pos: 传零让求解器自由求解（与 Python 版本一致）
  req->reference_pos = std::vector<double>(7, 0.0);

  auto future = coord_transform_client_->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
  {
    RCLCPP_WARN(this->get_logger(), "IK 调用超时");
    return false;
  }

  auto ret = future.get();
  if (!ret->success || ret->target_pos.empty())
  {
    RCLCPP_WARN(this->get_logger(), "IK 求解失败: %s", ret->message.c_str());
    return false;
  }

  joint_out = ret->target_pos;
  // IK 结果可能包含额外元素（如第 7 轴），截取到 ndof
  if (static_cast<int>(joint_out.size()) > ndof_)
  {
    joint_out.resize(ndof_);
  }
  return true;
}

// ========== KDL FK 初始化（仿真模式）==========

void F710TeleopNode::initKDLFK()
{
  std::string urdf_path;
  try
  {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("tl_description");
    urdf_path = pkg_path + "/urdf/" + arm_type_ + ".urdf";
  }
  catch (...)
  {
    urdf_path = std::string(std::getenv("HOME")) + "/tl_robot_ros2_py/src/tl_description/urdf/" + arm_type_ + ".urdf";
  }

  if (access(urdf_path.c_str(), F_OK) != 0)
  {
    RCLCPP_WARN(this->get_logger(), "KDL FK: URDF 未找到: %s", urdf_path.c_str());
    return;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromFile(urdf_path, tree))
  {
    RCLCPP_WARN(this->get_logger(), "KDL FK: 无法解析 URDF");
    return;
  }

  ndof_ = static_cast<int>(home_joints_.size());
  std::string tip_link = "link" + std::to_string(ndof_);
  if (!tree.getChain("link0", tip_link, kdl_chain_))
  {
    RCLCPP_WARN(this->get_logger(), "KDL FK: 无法获取 chain link0 -> %s", tip_link.c_str());
    return;
  }

  kdl_fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  kdl_fk_ready_ = true;
  RCLCPP_INFO(this->get_logger(), "KDL FK 就绪: %d 轴, tip=%s", ndof_, tip_link.c_str());
}

// ========== 关节→笛卡尔（FK）==========

bool F710TeleopNode::homeJointsToPose(std::vector<double>& pose_out)
{
  int ndof = static_cast<int>(home_joints_.size());

  if (!simulation_mode_)
  {
    // 真机：调用 coord_transform 服务
    if (!coord_transform_client_->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(this->get_logger(), "coord_transform 服务不可用");
      return false;
    }
    auto req = std::make_shared<tl_ros2_interface::srv::CoordTransform::Request>();
    req->origin_coord = 0;
    req->target_coord = 1;
    req->form = 0;
    std::vector<double> pos = home_joints_;
    pos.resize(7, 0.0);
    req->origin_pos = pos;
    req->reference_pos = std::vector<double>(7, 0.0);

    auto future = coord_transform_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
      RCLCPP_ERROR(this->get_logger(), "coord_transform 调用超时");
      return false;
    }
    auto ret = future.get();
    if (!ret->success || ret->target_pos.size() < 6)
    {
      RCLCPP_ERROR(this->get_logger(), "coord_transform 失败: %s", ret->message.c_str());
      return false;
    }
    pose_out.assign(ret->target_pos.begin(), ret->target_pos.begin() + 6);
    return true;
  }

  // 仿真：KDL 本地 FK
  if (!kdl_fk_ready_)
  {
    RCLCPP_ERROR(this->get_logger(), "FK 未就绪");
    return false;
  }

  KDL::JntArray q_in(ndof);
  for (int i = 0; i < ndof; ++i)
  {
    q_in(i) = home_joints_[i] * M_PI / 180.0;
  }

  KDL::Frame frame_out;
  int ret = kdl_fk_solver_->JntToCart(q_in, frame_out);
  if (ret != 0)
  {
    RCLCPP_ERROR(this->get_logger(), "KDL FK 求解失败, ret=%d", ret);
    return false;
  }

  double x = frame_out.p.x() * 1000.0;
  double y = frame_out.p.y() * 1000.0;
  double z = frame_out.p.z() * 1000.0;
  double rx, ry, rz;
  frame_out.M.GetRPY(rx, ry, rz);

  pose_out = {x, y, z, rx, ry, rz};
  return true;
}

// ========== ServoJ 初始化状态机 ==========

void F710TeleopNode::initServoj()
{
  if (simulation_mode_)
  {
    init_state_ = 6;
    return;
  }
  if (init_state_ == 6)
    return;

  auto& clk = *this->get_clock();

  // 状态 0：等待所有服务就绪
  if (init_state_ == 0)
  {
    if (!connect_arm_client_->wait_for_service(std::chrono::milliseconds(100)) ||
        !power_on_client_->wait_for_service(std::chrono::milliseconds(100)) ||
        !set_mode_client_->wait_for_service(std::chrono::milliseconds(100)) ||
        !set_speed_client_->wait_for_service(std::chrono::milliseconds(100)) ||
        !open_servoj_client_->wait_for_service(std::chrono::milliseconds(100)))
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 3000, "等待 tl_driver 服务就绪...");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "ServoJ 初始化中...");
    connect_arm_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    init_state_ = 1;
    return;
  }

  // 状态 1：连接机械臂
  if (init_state_ == 1)
  {
    RCLCPP_INFO(this->get_logger(), "机械臂已连接");
    power_on_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    init_state_ = 2;
    return;
  }

  // 状态 2：上电
  if (init_state_ == 2)
  {
    RCLCPP_INFO(this->get_logger(), "伺服已上电");
    auto req = std::make_shared<tl_ros2_interface::srv::SetCurrentMode::Request>();
    req->mode = 2;
    set_mode_client_->async_send_request(req);
    init_state_ = 3;
    return;
  }

  // 状态 3：设置远程模式
  if (init_state_ == 3)
  {
    RCLCPP_INFO(this->get_logger(), "远程模式已设置");
    auto req = std::make_shared<tl_ros2_interface::srv::SetSpeed::Request>();
    req->speed = servo_speed_;
    set_speed_client_->async_send_request(req);
    init_state_ = 4;
    return;
  }

  // 状态 4：设置 ServoJ 速度
  if (init_state_ == 4)
  {
    RCLCPP_INFO(this->get_logger(), "ServoJ 速度已设为 %.1f", servo_speed_);
    auto req = std::make_shared<tl_ros2_interface::srv::OpenServoJ::Request>();
    req->vmax = std::vector<double>(7, servo_vmax_);
    req->amax = std::vector<double>(7, servo_amax_);
    req->jmax = std::vector<double>(7, servo_jmax_);
    open_servoj_client_->async_send_request(req);
    init_state_ = 5;
    return;
  }

  // 状态 5：ServoJ 开启完成
  if (init_state_ == 5)
  {
    RCLCPP_INFO(this->get_logger(), "ServoJ 已开启，等待 joint_states 初始化位姿...");
    init_state_ = 6;
    // 初始化位姿由 jointStateCallback 首帧触发
    return;
  }
}

void F710TeleopNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // 缓存当前关节角
  std::vector<double> positions(ndof_, 0.0);
  int found = 0;
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
  {
    for (int j = 0; j < ndof_; ++j)
    {
      std::string jn = "joint" + std::to_string(j + 1);
      if (msg->name[i] == jn)
      {
        positions[j] = msg->position[i];
        found++;
        break;
      }
    }
  }
  if (found == ndof_)
  {
    current_joint_state_ = positions;
    joint_state_ready_ = true;
  }

  // 首帧到达且初始化完成 → 用当前关节角初始化位姿
  if (joint_state_ready_ && init_state_ >= 6 && !target_pose_ready_)
  {
    // /joint_states 为弧度，set_servoJ_pos 需角度 → 转换
    std::vector<double> joint_deg = current_joint_state_;
    for (auto& v : joint_deg)
      v *= 180.0 / M_PI;

    // 用当前关节角作为初始指令（不移动）
    {
      std::lock_guard<std::mutex> lock(joint_mutex_);
      last_joint_cmd_ = joint_deg;
    }

    // 异步 FK 算出当前笛卡尔位姿作为 target_pose_
    async_future_ =
        std::async(std::launch::async,
                   [this]()
                   {
                     std::vector<double> pose;
                     if (currentJointToPose(pose))
                     {
                       target_pose_ = pose;
                       target_pose_ready_ = true;
                       RCLCPP_INFO(this->get_logger(), "初始位姿（当前关节角FK）: [%.1f, %.1f, %.1f, %.2f, %.2f, %.2f]",
                                   target_pose_[0], target_pose_[1], target_pose_[2], target_pose_[3], target_pose_[4],
                                   target_pose_[5]);
                     }
                   });
  }
}

bool F710TeleopNode::currentJointToPose(std::vector<double>& pose_out)
{
  if (!simulation_mode_)
  {
    // 真机：调用 coord_transform 做 FK（关节→笛卡尔）
    if (!coord_transform_client_->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(this->get_logger(), "coord_transform 服务不可用");
      return false;
    }
    auto req = std::make_shared<tl_ros2_interface::srv::CoordTransform::Request>();
    req->origin_coord = 0;
    req->target_coord = 1;
    req->form = 0;
    // /joint_states 为弧度，coord_transform 需角度 → 转换
    std::vector<double> pos = current_joint_state_;
    for (auto& v : pos)
      v *= 180.0 / M_PI;
    pos.resize(7, 0.0);
    req->origin_pos = pos;
    req->reference_pos = std::vector<double>(7, 0.0);

    auto future = coord_transform_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
      RCLCPP_WARN(this->get_logger(), "currentJointToPose 调用超时");
      return false;
    }
    auto ret = future.get();
    if (!ret->success || ret->target_pos.size() < 6)
      return false;
    pose_out.assign(ret->target_pos.begin(), ret->target_pos.begin() + 6);
    return true;
  }

  // 仿真：KDL 本地 FK
  if (!kdl_fk_ready_)
    return false;

  KDL::JntArray q_in(ndof_);
  for (int i = 0; i < ndof_; ++i)
    q_in(i) = current_joint_state_[i];
  KDL::Frame frame_out;
  if (kdl_fk_solver_->JntToCart(q_in, frame_out) != 0)
    return false;
  double x = frame_out.p.x() * 1000.0;
  double y = frame_out.p.y() * 1000.0;
  double z = frame_out.p.z() * 1000.0;
  double rx, ry, rz;
  frame_out.M.GetRPY(rx, ry, rz);
  pose_out = {x, y, z, rx, ry, rz};
  return true;
}

void F710TeleopNode::closeServoj()
{
  if (init_state_ < 5)
    return;
  try
  {
    close_servoj_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    RCLCPP_INFO(this->get_logger(), "ServoJ 已关闭");
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "关闭 ServoJ 异常: %s", e.what());
  }
  // 切回示教模式并下电（匹配 Python 版本行为）
  try
  {
    auto mode_req = std::make_shared<tl_ros2_interface::srv::SetCurrentMode::Request>();
    mode_req->mode = 0;
    set_mode_client_->async_send_request(mode_req);
    RCLCPP_INFO(this->get_logger(), "已切换至示教模式");
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "设置示教模式异常: %s", e.what());
  }
  try
  {
    power_off_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    RCLCPP_INFO(this->get_logger(), "机械臂已下电");
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "下电异常: %s", e.what());
  }
}

// ========== 话题回调 ==========

void F710TeleopNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  latest_joy_ = msg;
}

// ========== 主控制循环（250Hz 直接 ServoJ） ==========

void F710TeleopNode::controlLoop()
{
  if (!latest_joy_ || !target_pose_ready_)
    return;

  const auto& joy = *latest_joy_;
  auto now = this->now();

  int needed_axes = std::max({axis_left_x_, axis_left_y_, axis_right_x_, axis_right_y_});
  int needed_btns = std::max({btn_a_, btn_lb_, btn_rb_, btn_back_, btn_start_});
  if (static_cast<int>(joy.axes.size()) <= needed_axes || static_cast<int>(joy.buttons.size()) <= needed_btns)
  {
    return;
  }

  double dpad_y = (static_cast<int>(joy.axes.size()) > axis_dpad_y_) ? joy.axes[axis_dpad_y_] : 0.0;
  bool has_dpad = static_cast<int>(joy.axes.size()) > axis_dpad_y_;

  // ========== 1. Back+Start 切换 停止/恢复（边缘检测） ==========
  bool bs_now = (joy.buttons[btn_back_] == 1 && joy.buttons[btn_start_] == 1);
  if (bs_now && !prev_bs_)
  {
    if (stop_mode_)
    {
      // 检查所有摇杆归零
      double lx_c = applyDeadzone(joy.axes[axis_left_x_], deadzone_);
      double ly_c = applyDeadzone(joy.axes[axis_left_y_], deadzone_);
      double rx_c = applyDeadzone(joy.axes[axis_right_x_], deadzone_);
      double ry_c = applyDeadzone(joy.axes[axis_right_y_], deadzone_);
      if (std::abs(lx_c) < 0.01 && std::abs(ly_c) < 0.01 && std::abs(rx_c) < 0.01 && std::abs(ry_c) < 0.01)
      {
        stop_mode_ = false;
        RCLCPP_INFO(this->get_logger(), "▶️ 遥操作恢复");
      }
    }
    else
    {
      stop_mode_ = true;
      RCLCPP_WARN(this->get_logger(), "⏸️ 遥操作暂停 (Back+Start 恢复)");
    }
  }
  prev_bs_ = bs_now;

  if (stop_mode_)
    return;

  // ========== 2. 速度调节（十字键防抖 300ms） ==========
  if (has_dpad && dpad_y != 0.0 && (now - last_dpad_time_).seconds() > 0.3)
  {
    if (dpad_y > 0.0)
      speed_value_ = std::min(speed_max_, speed_value_ + speed_step_);
    else
      speed_value_ = std::max(speed_min_, speed_value_ - speed_step_);
    last_dpad_time_ = now;
    RCLCPP_INFO(this->get_logger(), "速度: %.0f", speed_value_);
  }

  // ========== 3. A 键回零（防抖 500ms，直接发 home 关节角+异步 FK 更新 target_pose_） ==========
  if (joy.buttons[btn_a_] == 1 && !ik_pending_.load() && (now - last_a_press_).seconds() > 0.5)
  {
    last_a_press_ = now;
    stop_mode_ = false;
    speed_value_ = speed_default_;
    // 直接发 home_joints（度），不做 IK（与 Python 版本一致）
    {
      std::lock_guard<std::mutex> lock(joint_mutex_);
      last_joint_cmd_ = home_joints_;
    }
    // 仿真模式：立即发 ServolMove，否则 sim_bridge 不会收到回零指令
    if (simulation_mode_)
    {
      std::vector<double> home_pose;
      if (homeJointsToPose(home_pose))
      {
        target_pose_ = home_pose;
        publishServol();
        RCLCPP_INFO(this->get_logger(), "回零 -> [%.1f, %.1f, %.1f, %.2f, %.2f, %.2f]", target_pose_[0],
                    target_pose_[1], target_pose_[2], target_pose_[3], target_pose_[4], target_pose_[5]);
      }
    }
    else
    {
      // 真机：异步 FK 更新 target_pose_
      async_future_ = std::async(std::launch::async,
                                 [this]()
                                 {
                                   std::vector<double> home_pose;
                                   if (homeJointsToPose(home_pose))
                                   {
                                     target_pose_ = home_pose;
                                     RCLCPP_INFO(this->get_logger(), "回零 -> [%.1f, %.1f, %.1f, %.2f, %.2f, %.2f]",
                                                 target_pose_[0], target_pose_[1], target_pose_[2], target_pose_[3],
                                                 target_pose_[4], target_pose_[5]);
                                   }
                                 });
    }
    return;
  }

  // ========== 4. 读摇杆 + 笛卡尔速度积分 ==========
  double lx = applyDeadzone(joy.axes[axis_left_x_], deadzone_);
  double ly = applyDeadzone(joy.axes[axis_left_y_], deadzone_);
  double rx = applyDeadzone(joy.axes[axis_right_x_], deadzone_);
  double ry = applyDeadzone(joy.axes[axis_right_y_], deadzone_);

  double dt = 1.0 / control_rate_;
  double scale = speed_value_ / 100.0;

  double cart_vel[6] = {0.0};
  cart_vel[0] = lx * pos_sensitivity_ * scale;
  cart_vel[1] = ly * pos_sensitivity_ * scale;
  cart_vel[2] = ry * pos_sensitivity_ * scale;
  cart_vel[5] = rx * rot_sensitivity_ * scale;

  bool lb = joy.buttons[btn_lb_] == 1;
  bool rb = joy.buttons[btn_rb_] == 1;
  if (lb && !rb)
  {
    cart_vel[3] = rx * rot_sensitivity_ * scale;
    cart_vel[5] = 0.0;
  }
  else if (rb && !lb)
  {
    cart_vel[4] = rx * rot_sensitivity_ * scale;
    cart_vel[5] = 0.0;
  }

  bool has_input = false;
  for (int i = 0; i < 6; ++i)
  {
    if (std::abs(cart_vel[i]) > 1e-6)
    {
      target_pose_[i] += cart_vel[i] * dt;
      has_input = true;
    }
  }

  // ========== 5. 发送上一帧关节角（保持指令流不断） ==========
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    publishServoj(last_joint_cmd_);
  }

  // ========== 6. 有输入时更新关节角 / Servol ==========
  if (has_input)
  {
    if (simulation_mode_)
    {
      // 仿真：发布 ServolMove（笛卡尔位姿），由 sim_bridge 做 IK
      publishServol();
    }
    else if (!ik_pending_.load())
    {
      // 真机：异步 IK 更新关节角
      ik_pending_.store(true);
      std::vector<double> target_copy = target_pose_;
      async_future_ = std::async(std::launch::async,
                                 [this, target_copy]()
                                 {
                                   std::vector<double> joint_cmd;
                                   if (cartesianToJoint(target_copy, joint_cmd))
                                   {
                                     std::lock_guard<std::mutex> lock(joint_mutex_);
                                     last_joint_cmd_ = joint_cmd;
                                   }
                                   ik_pending_.store(false);
                                 });
    }
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<F710TeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
