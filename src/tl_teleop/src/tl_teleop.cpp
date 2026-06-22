#include "tl_teleop/tl_teleop.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <sstream>
#include <thread>

using json = nlohmann::json;

static constexpr double GRIP_THRESHOLD = 0.9;
static constexpr double CONTROL_PERIOD_S = 0.01;

TL_Teleop::TL_Teleop() : Node("tl_teleop_node") {
  declare_parameter("arm_axis_mode", 0);
  declare_parameter("pos_scale", 0.5);
  declare_parameter("pos_deadzone", 0.005);
  declare_parameter("max_pos_delta_mm", 300.0);
  declare_parameter("singular_angle", 160.0);
  declare_parameter("singular_scale", 0.2);
  declare_parameter("joint_jump_threshold", 30.0);
  declare_parameter("servo_speed", 25.0);
  declare_parameter("joint_limits", std::vector<double>{0.0});
  declare_parameter("servo_vmax", 80.0);
  declare_parameter("servo_amax", 3000.0);
  declare_parameter("servo_jmax", 50000.0);

  arm_joints_ = get_parameter("arm_axis_mode").as_int();
  if (arm_joints_ != 6 && arm_joints_ != 7) {
    RCLCPP_FATAL(
        get_logger(),
        "arm_axis_mode must be 6 or 7, got: %d. "
        "Please provide a valid arm_axis_mode in the YAML config file.",
        arm_joints_);
    init_failed_ = true;
    return;
  }
  pos_scale_ = get_parameter("pos_scale").as_double();
  pos_deadzone_ = get_parameter("pos_deadzone").as_double();
  max_pos_delta_mm_ = get_parameter("max_pos_delta_mm").as_double();
  singular_angle_ = get_parameter("singular_angle").as_double();
  singular_scale_ = get_parameter("singular_scale").as_double();
  joint_jump_threshold_ = get_parameter("joint_jump_threshold").as_double();
  servo_speed_ = get_parameter("servo_speed").as_double();
  auto joint_limits_flat = get_parameter("joint_limits").as_double_array();
  if (joint_limits_flat.size() != static_cast<size_t>(arm_joints_ * 2)) {
    RCLCPP_FATAL(
        get_logger(),
        "joint_limits flat array length mismatch: arm_axis_mode=%d, expected "
        "exactly %d values, "
        "got %zu. Each joint needs [min, max] — %d joints × 2 = %d values. "
        "Please fix the joint_limits parameter in the YAML config file.",
        arm_joints_, arm_joints_ * 2, joint_limits_flat.size(), arm_joints_,
        arm_joints_ * 2);
    init_failed_ = true;
    return;
  }

  joint_limits_.clear();
  for (size_t i = 0; i < joint_limits_flat.size(); i += 2) {
    joint_limits_.emplace_back(joint_limits_flat[i], joint_limits_flat[i + 1]);
  }

  double servo_vmax_val = get_parameter("servo_vmax").as_double();
  double servo_amax_val = get_parameter("servo_amax").as_double();
  double servo_jmax_val = get_parameter("servo_jmax").as_double();
  servo_vmax_.assign(arm_joints_, servo_vmax_val);
  servo_amax_.assign(arm_joints_, servo_amax_val);
  servo_jmax_.assign(arm_joints_, servo_jmax_val);

  set_speed_client_ =
      create_client<tl_ros2_interface::srv::SetSpeed>("/tl_driver/set_speed");
  set_current_mode_client_ =
      create_client<tl_ros2_interface::srv::SetCurrentMode>(
          "/tl_driver/set_current_mode");
  open_servoj_client_ = create_client<tl_ros2_interface::srv::OpenServoJ>(
      "/tl_driver/open_servoj");
  close_servoj_client_ =
      create_client<std_srvs::srv::Trigger>("/tl_driver/close_servoj");
  coord_transform_client_ =
      create_client<tl_ros2_interface::srv::CoordTransform>(
          "/tl_driver/coord_transform");
  get_rpy2quat_client_ = create_client<tl_ros2_interface::srv::GetPosTransform>(
      "/tl_driver/get_rpy2quat");

  servoj_pos_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/tl_driver/set_servoj_pos", 10);

  tcp_pose_sub_ = create_subscription<tl_ros2_interface::msg::CartesianPose>(
      "/tcp_pose", 10,
      [this](const tl_ros2_interface::msg::CartesianPose::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(tcp_pose_mutex_);
        latest_tcp_pose_ = *msg;
        tcp_pose_received_ = true;
      });

  RCLCPP_INFO(get_logger(), "TL_Teleop node initialized");
}

TL_Teleop::~TL_Teleop() { RCLCPP_INFO(get_logger(), "TL_Teleop destroyed"); }

void TL_Teleop::on_pxrea_client_cb(void *context, PXREAClientCallbackType type,
                                   int status, void *userData) {
  auto *self = static_cast<TL_Teleop *>(context);
  (void)status;

  switch (type) {
  case PXREAServerConnect:
    RCLCPP_INFO(self->get_logger(), "PXREA: server connected");
    break;
  case PXREAServerDisconnect:
    RCLCPP_WARN(self->get_logger(), "PXREA: server disconnected");
    break;
  case PXREADeviceFind:
    RCLCPP_INFO(self->get_logger(), "PXREA: device found %s",
                userData ? reinterpret_cast<const char *>(userData) : "");
    break;
  case PXREADeviceMissing:
    RCLCPP_WARN(self->get_logger(), "PXREA: device missing %s",
                userData ? reinterpret_cast<const char *>(userData) : "");
    break;
  case PXREADeviceConnect:
    RCLCPP_INFO(self->get_logger(), "PXREA: device connected %s",
                userData ? reinterpret_cast<const char *>(userData) : "");
    break;
  case PXREADeviceStateJson: {
    auto &dsj = *reinterpret_cast<PXREADevStateJson *>(userData);
    try {
      // PXREA SDK 的 JSON 是双层嵌套结构，外层 value 字段包含内层 JSON 字符串
      // 格式：{"value": "{\"Controller\": {\"right\": {\"pose\": \"...\",
      // \"grip\": 0.0, ...}}}"}
      json data = json::parse(dsj.stateJson);
      if (!data.contains("value")) {
        break;
      }
      json value = json::parse(data["value"].get<std::string>());
      if (!value.contains("Controller") ||
          !value["Controller"].contains("right")) {
        break;
      }
      auto &right = value["Controller"]["right"];

      // pose 格式: "x,y,z,qx,qy,qz,qw"（逗号分隔的七个浮点数，m 和 无量纲）
      auto pose = parse_pose_str(right["pose"].get<std::string>());
      // grip 值域 [0, 1]，0=完全松开，1=完全握紧
      double grip = right.value("grip", 0.0);
      // primaryButton 是 VR 手柄 A 键，用于复位到零点
      bool a_button = right.value("primaryButton", false);

      self->vr_state_.update(pose, grip, a_button);
    } catch (const json::exception &e) {
      RCLCPP_ERROR(self->get_logger(), "PXREA JSON parse error: %s", e.what());
    }
    break;
  }
  default:
    break;
  }
}

// 将 VR 姿态字符串解析为数组，格式: "x,y,z,qx,qy,qz,qw"（分别表示位置 m
// 和四元数）
std::array<double, 7> TL_Teleop::parse_pose_str(const std::string &s) {
  std::array<double, 7> result{};
  std::string token;
  std::istringstream iss(s);
  int i = 0;
  while (std::getline(iss, token, ',') && i < 7) {
    result[i++] = std::stod(token);
  }
  return result;
}

bool TL_Teleop::init_servo() {
  // 1. Set current mode to 2 (run mode)
  auto mode_req =
      std::make_shared<tl_ros2_interface::srv::SetCurrentMode::Request>();
  mode_req->mode = 2;
  auto mode_future = set_current_mode_client_->async_send_request(mode_req);
  if (mode_future.wait_for(std::chrono::seconds(5)) ==
      std::future_status::timeout) {
    RCLCPP_ERROR(get_logger(), "set_current_mode timeout");
    return false;
  }
  if (!mode_future.get()->success) {
    RCLCPP_ERROR(get_logger(), "set_current_mode failed: %s",
                 mode_future.get()->message.c_str());
    return false;
  }

  // 2. Set speed
  auto speed_req =
      std::make_shared<tl_ros2_interface::srv::SetSpeed::Request>();
  speed_req->speed = servo_speed_;
  auto speed_future = set_speed_client_->async_send_request(speed_req);
  if (speed_future.wait_for(std::chrono::seconds(5)) ==
      std::future_status::timeout) {
    RCLCPP_ERROR(get_logger(), "set_speed timeout");
    return false;
  }
  if (!speed_future.get()->success) {
    RCLCPP_ERROR(get_logger(), "set_speed failed");
    return false;
  }

  // 3. Open ServoJ
  auto servoj_req =
      std::make_shared<tl_ros2_interface::srv::OpenServoJ::Request>();
  servoj_req->vmax = servo_vmax_;
  servoj_req->amax = servo_amax_;
  servoj_req->jmax = servo_jmax_;
  auto servoj_future = open_servoj_client_->async_send_request(servoj_req);
  if (servoj_future.wait_for(std::chrono::seconds(5)) ==
      std::future_status::timeout) {
    RCLCPP_ERROR(get_logger(), "open_servoj timeout");
    return false;
  }
  if (!servoj_future.get()->success) {
    RCLCPP_ERROR(get_logger(), "open_servoj failed");
    return false;
  }

  RCLCPP_INFO(get_logger(), "ServoJ initialized via tl_driver services");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  return true;
}

void TL_Teleop::shutdown_teleop(rclcpp::Executor &exec) {
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(get_logger(), "rclcpp not ok, cannot close ServoJ");
    return;
  }
  auto close_req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = close_servoj_client_->async_send_request(close_req);

  auto ret = exec.spin_until_future_complete(future, std::chrono::seconds(2));
  if (ret != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "close_servoj failed (code %d)",
                 static_cast<int>(ret));
    return;
  }
  if (!future.get()->success) {
    RCLCPP_ERROR(get_logger(), "close_servoj service reported failure");
    return;
  }
  RCLCPP_INFO(get_logger(), "ServoJ closed via tl_driver services");
}

std::vector<double> TL_Teleop::get_arm_cartesian_pose() {
  std::lock_guard<std::mutex> lock(tcp_pose_mutex_);
  if (!tcp_pose_received_) {
    return {};
  }
  // tl_driver 发布的 /tcp_pose：position 单位 mm（直接从 SDK 透传），rpy 单位
  // rad
  return {latest_tcp_pose_.position.x, latest_tcp_pose_.position.y,
          latest_tcp_pose_.position.z, latest_tcp_pose_.rpy.x,
          latest_tcp_pose_.rpy.y,      latest_tcp_pose_.rpy.z};
}

std::vector<double> TL_Teleop::get_inverse_kinematics(double x, double y,
                                                      double z, double rx,
                                                      double ry, double rz) {
  auto req =
      std::make_shared<tl_ros2_interface::srv::CoordTransform::Request>();
  req->origin_coord = 1; // 笛卡尔坐标系
  req->target_coord = 0; // 关节坐标系
  req->form = 0;
  req->origin_pos = {x, y, z, rx, ry, rz, 0.0};
  req->reference_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  auto future = coord_transform_client_->async_send_request(req);
  if (future.wait_for(std::chrono::milliseconds(500)) ==
      std::future_status::timeout) {
    RCLCPP_WARN(get_logger(), "coord_transform (IK) timeout");
    return {};
  }
  auto resp = future.get();
  if (!resp->success) {
    RCLCPP_WARN(get_logger(), "coord_transform (IK) failed: %s",
                resp->message.c_str());
    return {};
  }
  return resp->target_pos;
}

std::array<double, 4> TL_Teleop::get_rpy2quat_sdk(double rx, double ry,
                                                  double rz) {
  auto req =
      std::make_shared<tl_ros2_interface::srv::GetPosTransform::Request>();
  req->input = {rx, ry, rz};

  auto future = get_rpy2quat_client_->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(5)) == std::future_status::timeout) {
    RCLCPP_WARN(get_logger(), "get_rpy2quat timeout");
    return {1.0, 0.0, 0.0, 0.0};
  }
  auto resp = future.get();
  if (!resp->success || resp->output.size() < 4) {
    RCLCPP_WARN(get_logger(), "get_rpy2quat failed");
    return {1.0, 0.0, 0.0, 0.0};
  }
  // get_rpy2quat 返回值顺序: [w, x, y, z]
  return {resp->output[0], resp->output[1], resp->output[2], resp->output[3]};
}

// 将关节角度裁剪到限位范围内，防止运动超出机械臂物理限制
// 输入/输出单位：度
std::vector<double> TL_Teleop::clamp_joints(const std::vector<double> &joints) {
  std::vector<double> clamped;
  for (size_t i = 0; i < joints.size() && i < joint_limits_.size(); ++i) {
    double lo = joint_limits_[i].first;
    double hi = joint_limits_[i].second;
    clamped.push_back(std::max(lo, std::min(hi, joints[i])));
  }
  return clamped;
}

// 关节跳变检测：新指令与上一帧差值超过阈值则拒绝，防止异常指令或通信错误
// 阈值单位：度
bool TL_Teleop::joints_safe(const std::vector<double> &new_joints) {
  if (last_joints_.empty()) {
    last_joints_ = new_joints;
    return true;
  }
  for (size_t i = 0; i < new_joints.size() && i < last_joints_.size(); ++i) {
    if (std::abs(new_joints[i] - last_joints_[i]) > joint_jump_threshold_) {
      RCLCPP_WARN(get_logger(), "Joint %zu jump too large: %.1f deg", i,
                  std::abs(new_joints[i] - last_joints_[i]));
      return false;
    }
  }
  last_joints_ = new_joints;
  return true;
}

void TL_Teleop::servoJ_send(const std::vector<double> &joint_target) {
  auto clamped = clamp_joints(joint_target);
  if (!joints_safe(clamped)) {
    return;
  }

  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data = clamped;
  servoj_pos_pub_->publish(msg);
}

// 四元数乘法：q1 * q2，表示先旋转 q2 再旋转 q1
// 约定：[w, x, y, z] 顺序
std::array<double, 4>
TL_Teleop::quat_multiply(const std::array<double, 4> &q1,
                         const std::array<double, 4> &q2) {
  double w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
  double w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];
  return {w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
          w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
          w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
          w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2};
}

// 四元数共轭（逆），用于计算旋转差值
std::array<double, 4> TL_Teleop::quat_inverse(const std::array<double, 4> &q) {
  return {q[0], -q[1], -q[2], -q[3]};
}

// 四元数 → RPY 欧拉角（XYZ 内旋格式），返回值单位：rad
std::array<double, 3> TL_Teleop::quat2rpy(const std::array<double, 4> &q) {
  double w = q[0], x = q[1], y = q[2], z = q[3];
  double sinr_cosp = 2.0 * (w * x + y * z);
  double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  double roll = std::atan2(sinr_cosp, cosr_cosp);

  double sinp = 2.0 * (w * y - z * x);
  double pitch = std::asin(std::max(-1.0, std::min(1.0, sinp)));

  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  return {roll, pitch, yaw};
}

void TL_Teleop::control_loop() {
  // 基准点：握下扳机瞬间的机械臂笛卡尔位置（mm）
  double base_x = 0.0, base_y = 0.0, base_z = 0.0;

  while (running_) {
    auto t0 = std::chrono::steady_clock::now();

    std::array<double, 7> pose;
    double grip;
    bool a_button;
    vr_state_.get(pose, grip, a_button);

    double x = pose[0], y = pose[1], z = pose[2];
    double qx = pose[3], qy = pose[4], qz = pose[5], qw = pose[6];

    // A 键：将所有关节复位到零点，清除 home 状态
    if (a_button) {
      std::vector<double> zero_joints(arm_joints_, 0.0);
      servoJ_send(zero_joints);
      {
        std::lock_guard<std::mutex> lock(teleop_mutex_);
        vr_homed_ = false;
        last_joints_.clear();
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }

    if (grip > GRIP_THRESHOLD) {
      if (!vr_homed_) {
        // 刚握下扳机时，记录当前机械臂位姿作为基准点
        // 后续手柄位移以此刻为原点，避免绝对坐标系漂移
        auto cart = get_arm_cartesian_pose();
        if (!cart.empty() && cart.size() >= 6) {
          base_x = cart[0];
          base_y = cart[1];
          base_z = cart[2];
          double base_rx = cart[3], base_ry = cart[4], base_rz = cart[5];
          // 将当前位姿的 RPY 通过 SDK 转为四元数，作为角度基准
          auto arm_quat = get_rpy2quat_sdk(base_rx, base_ry, base_rz);
          // VR 手柄当前姿态（qx,qy,qz,qw 顺序）
          std::array<double, 4> current_vr_quat = {qx, qy, qz, qw};
          {
            std::lock_guard<std::mutex> lock(teleop_mutex_);
            vr_home_pose_ = pose;
            base_arm_quat_ = arm_quat;
            vr_home_quat_ = current_vr_quat;
            vr_homed_ = true;
          }
        }
      }
    } else {
      // 松开扳机则取消 home，下次握下时重新标定
      std::lock_guard<std::mutex> lock(teleop_mutex_);
      vr_homed_ = false;
    }

    bool homed;
    {
      std::lock_guard<std::mutex> lock(teleop_mutex_);
      homed = vr_homed_;
    }

    if (homed) {
      std::array<double, 7> home_pose;
      std::array<double, 4> arm_q, vr_hq;
      {
        std::lock_guard<std::mutex> lock(teleop_mutex_);
        home_pose = vr_home_pose_;
        arm_q = base_arm_quat_;
        vr_hq = vr_home_quat_;
      }

      // 手柄当前位置相对于 home 的偏移（m）
      double dx = x - home_pose[0];
      double dy = y - home_pose[1];
      double dz = z - home_pose[2];
      if (std::abs(dx) < pos_deadzone_)
        dx = 0.0;
      if (std::abs(dy) < pos_deadzone_)
        dy = 0.0;
      if (std::abs(dz) < pos_deadzone_)
        dz = 0.0;

      // 奇异点保护：末端关节接近极限角度时大幅降低运动速度，防止关节速度爆炸
      double scale = 1.0;
      {
        std::lock_guard<std::mutex> lock(teleop_mutex_);
        if (!last_joints_.empty() &&
            static_cast<int>(last_joints_.size()) >= arm_joints_) {
          if (arm_joints_ == 6) {
            if (std::abs(last_joints_[4]) > singular_angle_ ||
                std::abs(last_joints_[5]) > singular_angle_) {
              scale = singular_scale_;
            }
          } else {
            if (std::abs(last_joints_[5]) > singular_angle_ ||
                std::abs(last_joints_[6]) > singular_angle_) {
              scale = singular_scale_;
            }
          }
        }
      }

      // VR 手柄坐标系 → 机械臂基座坐标系映射
      // VR: X右 Y上 Z前  |  机械臂: X前 Y左 Z上
      // 映射关系：X_arm = -Z_vr, Y_arm = -X_vr, Z_arm = Y_vr
      // 手柄单位 m → 机械臂单位 mm（×1000），再乘位置缩放系数
      double arm_delta_X = -dz * 1000.0 * pos_scale_ * scale;
      double arm_delta_Y = -dx * 1000.0 * pos_scale_ * scale;
      double arm_delta_Z = dy * 1000.0 * pos_scale_ * scale;
      arm_delta_X = std::max(-max_pos_delta_mm_,
                             std::min(max_pos_delta_mm_, arm_delta_X));
      arm_delta_Y = std::max(-max_pos_delta_mm_,
                             std::min(max_pos_delta_mm_, arm_delta_Y));
      arm_delta_Z = std::max(-max_pos_delta_mm_,
                             std::min(max_pos_delta_mm_, arm_delta_Z));

      double target_x = base_x + arm_delta_X;
      double target_y = base_y + arm_delta_Y;
      double target_z = base_z + arm_delta_Z;

      // 取最短旋转路径：若当前 quat 与 home quat 点积为负，翻转其中一个
      // 避免 360°→0° 走长路
      std::array<double, 4> vr_quat_wxyz = {qw, qx, qy, qz};
      std::array<double, 4> vr_home_wxyz = {vr_hq[3], vr_hq[0], vr_hq[1],
                                            vr_hq[2]};

      double dot = 0.0;
      for (int i = 0; i < 4; ++i) {
        dot += vr_quat_wxyz[i] * vr_home_wxyz[i];
      }
      if (dot < 0.0) {
        for (int i = 0; i < 4; ++i) {
          vr_quat_wxyz[i] = -vr_quat_wxyz[i];
        }
      }

      auto vr_inv = quat_inverse(vr_home_wxyz);
      auto vr_delta = quat_multiply(vr_quat_wxyz, vr_inv);

      // VR 四元数 → 机械臂四元数坐标系映射：[w, -z, -x, -y]
      std::array<double, 4> vr_delta_mapped = {vr_delta[0], -vr_delta[3],
                                               -vr_delta[1], -vr_delta[2]};
      auto target_quat = quat_multiply(vr_delta_mapped, arm_q);

      // 目标四元数 → RPY → 逆运动学求解
      auto rpy = quat2rpy(target_quat);
      double target_rx = rpy[0];
      double target_ry = -rpy[1];
      double target_rz = rpy[2];

      auto ik = get_inverse_kinematics(target_x, target_y, target_z, target_rx,
                                       target_ry, target_rz);
      if (!ik.empty()) {
        // tl_driver CoordTransform 总是返回 7 个关节值，6 轴模式截取前 6 个
        if (arm_joints_ == 6 && ik.size() > 6) {
          ik.resize(6);
        }
        servoJ_send(ik);
      }
    }

    // 固定频率控制：100Hz，睡眠补足剩余时间
    auto t1 = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(t1 - t0).count();
    double sleep_time = CONTROL_PERIOD_S - elapsed;
    if (sleep_time > 0) {
      std::this_thread::sleep_for(
          std::chrono::microseconds(static_cast<int64_t>(sleep_time * 1e6)));
    }
  }
}

void TL_Teleop::run_control_loop() {
  if (init_failed_) {
    RCLCPP_FATAL(get_logger(),
                 "Initialization failed, refusing to start control loop");
    return;
  }

  RCLCPP_INFO(get_logger(), "Initializing PXREA SDK...");
  int pxrea_ret = PXREAInit(this, on_pxrea_client_cb, PXREAFullMask);
  if (pxrea_ret != 0) {
    RCLCPP_ERROR(get_logger(), "PXREAInit failed: %d", pxrea_ret);
    return;
  }
  RCLCPP_INFO(get_logger(), "PXREA SDK initialized");

  RCLCPP_INFO(get_logger(), "Waiting for tl_driver services...");
  if (!set_speed_client_->wait_for_service(std::chrono::seconds(30))) {
    RCLCPP_ERROR(get_logger(),
                 "tl_driver services not available after 30s, shutting down");
    PXREADeinit();
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(get_logger(), "tl_driver services available");

  if (!init_servo()) {
    RCLCPP_ERROR(get_logger(), "Servo init failed, shutting down");
    PXREADeinit();
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(get_logger(), "Teleop ready (singularity guard enabled)");

  control_loop();

  // control_loop() 已退出（running_ = false），清理非 ROS 资源
  // close_servo() 由 shutdown_teleop() 在主线程调用，此时 rclcpp 仍存活
  PXREADeinit();
}

namespace {
std::atomic<bool> g_sigint_received{false};

extern "C" void sigint_handler(int) { g_sigint_received = true; }
} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TL_Teleop>();

  if (!node->init_ok()) {
    rclcpp::shutdown();
    return 1;
  }

  // 控制循环在单独线程运行，避免 100Hz 循环阻塞 ROS2 spin
  // 主线程负责 ROS2 回调（服务调用、话题订阅）
  std::thread control_thread([&node]() { node->run_control_loop(); });

  // 替换 rclcpp 的 SIGINT/SIGTERM 处理：仅设标志，不立即 shutdown
  // 确保 close_servo() 在 rclcpp 存活时执行
  // timeout=2s 保证退出时间 < ros2 launch 的 5s SIGINT→SIGTERM 升级阈值
  signal(SIGINT, sigint_handler);
  signal(SIGTERM, sigint_handler);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  // 自定义 spin 循环：检测 SIGINT 标志而非依赖 rclcpp::shutdown()
  while (rclcpp::ok() && !g_sigint_received) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  if (g_sigint_received) {
    // Ctrl+C：rclcpp 仍存活，有序关闭
    node->stop();
    control_thread.join();       // control_loop 退出 → PXREADeinit()
    node->shutdown_teleop(exec); // exec.spin_until_future_complete 等 response
    rclcpp::shutdown();
  } else {
    // 控制线程提前失败（服务不可用/init_servo 失败）
    // 控制线程已调 PXREADeinit() + rclcpp::shutdown()
    node->stop();
    if (control_thread.joinable()) {
      control_thread.join();
    }
    // close_servo() 跳过——ServoJ 从未成功开启
    rclcpp::shutdown();
  }

  return 0;
}
