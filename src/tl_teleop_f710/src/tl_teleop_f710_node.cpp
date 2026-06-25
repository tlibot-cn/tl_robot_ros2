/**
 * @file tl_teleop_f710_node.cpp
 * @brief 天链机械臂 Logitech F710 手柄遥操作节点 (C++)
 *
 * 通过 F710 游戏手柄远程控制天链机械臂，基于笛卡尔空间伺服（servol）
 * 实现直观的末端位置控制。
 *
 * 工作流程：
 *   F710 -> joy_node -> /joy 话题 -> 本节点 -> /tl_driver/set_servol_pos -> tl_driver -> 机械臂
 *
 * 控制模式（笛卡尔空间，基座标系）：
 *   - 左摇杆  -> X/Y 平移
 *   - 右摇杆  -> Z 平移 + 偏航（默认）/ 翻滚（LB）/ 俯仰（RB）
 *   - 十字键  -> 速度倍率调节
 *   - A 键    -> 回零（通过 FK 将 home_joints 转为笛卡尔位姿）
 *   - B 键    -> 停止
 *
 * 关键改进：
 *   - 使用 FK（真机调用 coord_transform 服务 / 仿真 KDL）将 home_joints
 *     转为笛卡尔位姿，替代硬编码 initial_pose
 *   - target_pose_ 在 FK 完成前为无效状态，不发布运动
 */

#include "tl_teleop_f710/tl_teleop_f710_node.h"

#include <cmath>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>

F710TeleopNode::F710TeleopNode()
: rclcpp::Node("tl_teleop_f710_node"),
  latest_joy_(nullptr),
  speed_value_(50.0),
  stop_mode_(false),
  target_pose_ready_(false),
  home_request_pending_(false),
  init_state_(0),
  kdl_fk_ready_(false)
{
  declareParameters();
  loadParameters();

  speed_value_ = speed_default_;

  // 初始化防抖时间戳（与 this->now() 保持相同时钟源）
  // 不能在初始化列表中使用 this->now()，因为节点尚未完全构造
  last_dpad_time_ = this->now();
  last_a_press_ = this->now();
  last_b_press_ = this->now();

  // 订阅 /joy
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&F710TeleopNode::joyCallback, this, std::placeholders::_1));

  // 发布 /tl_driver/set_servol_pos
  servol_pub_ = this->create_publisher<tl_ros2_interface::msg::ServolMove>(
    "/tl_driver/set_servol_pos", 10);

  // 服务客户端
  set_mode_client_ = this->create_client<tl_ros2_interface::srv::SetCurrentMode>(
    "/tl_driver/set_current_mode");
  set_speed_client_ = this->create_client<tl_ros2_interface::srv::SetSpeed>(
    "/tl_driver/set_speed");
  open_servoj_client_ = this->create_client<tl_ros2_interface::srv::OpenServoJ>(
    "/tl_driver/open_servoj");
  close_servoj_client_ = this->create_client<std_srvs::srv::Trigger>(
    "/tl_driver/close_servoj");
  coord_transform_client_ = this->create_client<tl_ros2_interface::srv::CoordTransform>(
    "/tl_driver/coord_transform");

  // 仿真模式：提前初始化 KDL FK
  if (simulation_mode_) {
    initKDLFK();
    std::vector<double> init_pose;
    if (homeJointsToPose(init_pose)) {
      target_pose_ = init_pose;
      target_pose_ready_ = true;
      RCLCPP_INFO(this->get_logger(),
        "初始位姿（FK）: [%.1f, %.1f, %.1f, %.2f, %.2f, %.2f]",
        target_pose_[0], target_pose_[1], target_pose_[2],
        target_pose_[3], target_pose_[4], target_pose_[5]);
    }
  }

  // ServoJ 初始化定时器（1Hz）
  init_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&F710TeleopNode::initServoj, this));

  // 控制定时器
  auto period = std::chrono::duration<double>(1.0 / control_rate_);
  control_timer_ = this->create_wall_timer(
    period, std::bind(&F710TeleopNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(),
    "F710 遥操作节点已启动 (%.1fHz, 灵敏度 %.1fmm/s)",
    control_rate_, pos_sensitivity_);
}

F710TeleopNode::~F710TeleopNode()
{
  closeServoj();
}

// ========== 参数声明 ==========

void F710TeleopNode::declareParameters()
{
  this->declare_parameter("control_rate", 20.0);
  this->declare_parameter("simulation_mode", false);
  this->declare_parameter("arm_type", "tcb605");
  this->declare_parameter("speed_default", 50.0);
  this->declare_parameter("speed_min", 5.0);
  this->declare_parameter("speed_max", 100.0);
  this->declare_parameter("speed_step", 5.0);
  this->declare_parameter("servo_speed", 25.0);
  this->declare_parameter("servo_vmax", 80.0);
  this->declare_parameter("servo_amax", 3000.0);
  this->declare_parameter("servo_jmax", 50000.0);
  this->declare_parameter("pos_sensitivity", 50.0);
  this->declare_parameter("rot_sensitivity", 1.0);
  this->declare_parameter("step_size", 2.0);
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
  step_size_ = this->get_parameter("step_size").as_double();
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
}

// ========== 工具函数 ==========

double F710TeleopNode::applyDeadzone(double value, double deadzone)
{
  if (std::abs(value) < deadzone) return 0.0;
  double sign = (value > 0.0) ? 1.0 : -1.0;
  return sign * (std::abs(value) - deadzone) / (1.0 - deadzone);
}

void F710TeleopNode::publishServol()
{
  if (!target_pose_ready_) return;
  auto msg = tl_ros2_interface::msg::ServolMove();
  msg.target_pose = target_pose_;
  msg.step_size = step_size_;
  msg.coord = 1;
  servol_pub_->publish(msg);
}

// ========== KDL FK 初始化（仿真模式）==========

void F710TeleopNode::initKDLFK()
{
  std::string urdf_path;
  try {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("tl_description");
    urdf_path = pkg_path + "/urdf/" + arm_type_ + ".urdf";
  } catch (...) {
    urdf_path = std::string(std::getenv("HOME")) +
      "/tl_robot_ros2_py/src/tl_description/urdf/" + arm_type_ + ".urdf";
  }

  if (access(urdf_path.c_str(), F_OK) != 0) {
    RCLCPP_WARN(this->get_logger(), "KDL FK: URDF 未找到: %s", urdf_path.c_str());
    return;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromFile(urdf_path, tree)) {
    RCLCPP_WARN(this->get_logger(), "KDL FK: 无法解析 URDF");
    return;
  }

  ndof_ = static_cast<int>(home_joints_.size());
  std::string tip_link = "link" + std::to_string(ndof_);
  if (!tree.getChain("link0", tip_link, kdl_chain_)) {
    RCLCPP_WARN(this->get_logger(), "KDL FK: 无法获取 chain link0 -> %s", tip_link.c_str());
    return;
  }

  kdl_fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  kdl_fk_ready_ = true;
  RCLCPP_INFO(this->get_logger(), "KDL FK 就绪: %d 轴, tip=%s", ndof_, tip_link.c_str());
}

// ========== 关节→笛卡尔（FK）==========

bool F710TeleopNode::homeJointsToPose(std::vector<double> & pose_out)
{
  int ndof = static_cast<int>(home_joints_.size());

  if (!simulation_mode_) {
    // 真机：调用 coord_transform 服务
    if (!coord_transform_client_->wait_for_service(std::chrono::seconds(2))) {
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
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "coord_transform 调用超时");
      return false;
    }
    auto ret = future.get();
    if (!ret->success || ret->target_pos.size() < 6) {
      RCLCPP_ERROR(this->get_logger(), "coord_transform 失败: %s", ret->message.c_str());
      return false;
    }
    pose_out.assign(ret->target_pos.begin(), ret->target_pos.begin() + 6);
    return true;
  }

  // 仿真：KDL 本地 FK
  if (!kdl_fk_ready_) {
    RCLCPP_ERROR(this->get_logger(), "FK 未就绪");
    return false;
  }

  KDL::JntArray q_in(ndof);
  for (int i = 0; i < ndof; ++i) {
    q_in(i) = home_joints_[i] * M_PI / 180.0;
  }

  KDL::Frame frame_out;
  int ret = kdl_fk_solver_->JntToCart(q_in, frame_out);
  if (ret != 0) {
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
  if (simulation_mode_) {
    init_state_ = 4;
    return;
  }
  if (init_state_ == 4) return;

  // 状态 0：等待所有服务就绪
  if (init_state_ == 0) {
    if (!set_mode_client_->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "等待 tl_driver 服务就绪...");
      return;
    }
    if (!set_speed_client_->wait_for_service(std::chrono::milliseconds(100))) return;
    if (!open_servoj_client_->wait_for_service(std::chrono::milliseconds(100))) return;
    RCLCPP_INFO(this->get_logger(), "ServoJ 初始化中...");

    auto req = std::make_shared<tl_ros2_interface::srv::SetCurrentMode::Request>();
    req->mode = 2;
    set_mode_client_->async_send_request(req);
    init_state_ = 1;
    return;
  }

  if (init_state_ == 1) {
    RCLCPP_INFO(this->get_logger(), "ServoJ 模式已设为远程(2)");
    auto req = std::make_shared<tl_ros2_interface::srv::SetSpeed::Request>();
    req->speed = servo_speed_;
    set_speed_client_->async_send_request(req);
    init_state_ = 2;
    return;
  }

  if (init_state_ == 2) {
    RCLCPP_INFO(this->get_logger(), "ServoJ 速度已设为 %.1f", servo_speed_);
    auto req = std::make_shared<tl_ros2_interface::srv::OpenServoJ::Request>();
    req->vmax = std::vector<double>(7, servo_vmax_);
    req->amax = std::vector<double>(7, servo_amax_);
    req->jmax = std::vector<double>(7, servo_jmax_);
    open_servoj_client_->async_send_request(req);
    init_state_ = 3;
    return;
  }

  if (init_state_ == 3) {
    RCLCPP_INFO(this->get_logger(), "ServoJ 已开启，遥操作就绪 ✅");
    init_state_ = 4;
    // 在独立线程中执行阻塞的 coord_transform 服务调用
    afterInit();
    return;
  }
}

void F710TeleopNode::afterInit()
{
  if (simulation_mode_) return;
  if (target_pose_ready_) return;

  // 在独立线程中执行阻塞的服务调用，避免阻塞 executor
  after_init_future_ = std::async(std::launch::async, [this]() {
    std::vector<double> init_pose;
    if (homeJointsToPose(init_pose)) {
      target_pose_ = init_pose;
      target_pose_ready_ = true;
      RCLCPP_INFO(this->get_logger(),
        "初始位姿（FK）: [%.1f, %.1f, %.1f, %.2f, %.2f, %.2f]",
        target_pose_[0], target_pose_[1], target_pose_[2],
        target_pose_[3], target_pose_[4], target_pose_[5]);
    }
  });
}

void F710TeleopNode::closeServoj()
{
  if (init_state_ < 3) return;
  try {
    close_servoj_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    RCLCPP_INFO(this->get_logger(), "ServoJ 已关闭");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "关闭 ServoJ 异常: %s", e.what());
  }
}

// ========== 话题回调 ==========

void F710TeleopNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  latest_joy_ = msg;
}

// ========== 主控制循环 ==========

void F710TeleopNode::controlLoop()
{
  if (!latest_joy_ || !target_pose_ready_) return;

  const auto & joy = *latest_joy_;
  auto now = this->now();

  int needed_axes = std::max({axis_left_x_, axis_left_y_, axis_right_x_, axis_right_y_});
  int needed_btns = std::max({btn_a_, btn_b_, btn_lb_, btn_rb_});
  if (static_cast<int>(joy.axes.size()) <= needed_axes ||
      static_cast<int>(joy.buttons.size()) <= needed_btns) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "摇杆数据异常: axes=%zu, buttons=%zu", joy.axes.size(), joy.buttons.size());
    return;
  }

  double dpad_y = (static_cast<int>(joy.axes.size()) > axis_dpad_y_)
    ? joy.axes[axis_dpad_y_] : 0.0;
  bool has_dpad = static_cast<int>(joy.axes.size()) > axis_dpad_y_;

  // ========== A 键：回零（防抖 500ms，异步调用避免阻塞 executor） ==========
  if (joy.buttons[btn_a_] == 1 && !home_request_pending_ &&
      (now - last_a_press_).seconds() > 0.5) {
    last_a_press_ = now;
    home_request_pending_ = true;
    stop_mode_ = false;
    after_init_future_ = std::async(std::launch::async, [this]() {
      std::vector<double> home_pose;
      if (homeJointsToPose(home_pose)) {
        target_pose_ = home_pose;
        speed_value_ = speed_default_;
        publishServol();
        RCLCPP_INFO(this->get_logger(),
          "回零 -> [%.1f, %.1f, %.1f, %.2f, %.2f, %.2f]",
          target_pose_[0], target_pose_[1], target_pose_[2],
          target_pose_[3], target_pose_[4], target_pose_[5]);
      }
      home_request_pending_ = false;
    });
    return;
  }

  // ========== B 键：停止（防抖 500ms） ==========
  if (joy.buttons[btn_b_] == 1 && (now - last_b_press_).seconds() > 0.5) {
    last_b_press_ = now;
    stop_mode_ = true;
    RCLCPP_INFO(this->get_logger(), "停止运动");
    return;
  }

  if (stop_mode_) return;

  // ========== 速度调节（十字键上下，防抖 300ms） ==========
  if (has_dpad && dpad_y != 0.0 && (now - last_dpad_time_).seconds() > 0.3) {
    if (dpad_y > 0.0) {
      speed_value_ = std::min(speed_max_, speed_value_ + speed_step_);
    } else {
      speed_value_ = std::max(speed_min_, speed_value_ - speed_step_);
    }
    last_dpad_time_ = now;
    RCLCPP_INFO(this->get_logger(), "速度: %.0f", speed_value_);
  }

  // ========== 读取摇杆（带死区） ==========
  double lx = applyDeadzone(joy.axes[axis_left_x_], deadzone_);
  double ly = applyDeadzone(joy.axes[axis_left_y_], deadzone_);
  double rx = applyDeadzone(joy.axes[axis_right_x_], deadzone_);
  double ry = applyDeadzone(joy.axes[axis_right_y_], deadzone_);

  if (std::abs(lx) < 0.001 && std::abs(ly) < 0.001 &&
      std::abs(rx) < 0.001 && std::abs(ry) < 0.001) {
    return;
  }

  // ========== 计算运动增量 ==========
  double dt = 1.0 / control_rate_;
  double scale = speed_value_ / 100.0;

  double dx = lx * pos_sensitivity_ * scale * dt;
  double dy = ly * pos_sensitivity_ * scale * dt;
  double dz = ry * pos_sensitivity_ * scale * dt;
  double dyaw = rx * rot_sensitivity_ * scale * dt;

  bool lb = joy.buttons[btn_lb_] == 1;
  bool rb = joy.buttons[btn_rb_] == 1;
  double roll = 0.0, pitch = 0.0;

  if (lb && !rb) {
    roll = rx * rot_sensitivity_ * scale * dt;
    dyaw = 0.0;
  } else if (rb && !lb) {
    pitch = rx * rot_sensitivity_ * scale * dt;
    dyaw = 0.0;
  }

  // ========== 更新目标位姿 ==========
  target_pose_[0] += dx;
  target_pose_[1] += dy;
  target_pose_[2] += dz;
  target_pose_[3] += roll;
  target_pose_[4] += pitch;
  target_pose_[5] += dyaw;

  publishServol();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<F710TeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
