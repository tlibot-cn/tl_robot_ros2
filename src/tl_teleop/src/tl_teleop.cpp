#include "tl_teleop/tl_teleop.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>
#include <thread>

using json = nlohmann::json;

static constexpr double POS_SCALE = 0.5;
static constexpr double POS_DEADZONE = 0.005;
static constexpr double MAX_POS_DELTA_MM = 300.0;
static constexpr double SINGULAR_ANGLE = 160.0;
static constexpr double SINGULAR_SCALE = 0.2;
static constexpr double JOINT_JUMP_THRESHOLD = 30.0;
static constexpr double CONTROL_PERIOD_S = 0.01;
static constexpr double GRIP_THRESHOLD = 0.9;
static constexpr double DEG_TO_RAD = M_PI / 180.0;
static constexpr double RAD_TO_DEG = 180.0 / M_PI;
static constexpr double LOG_INTERVAL_S = 1.0;

static const std::vector<std::pair<double, double>> DEFAULT_JOINT_LIMITS = {
  {-180, 180}, {-180, 180}, {-180, 180}, {-180, 180},
  {-180, 180}, {-170, 170}, {-170, 170}
};

TL_Teleop::TL_Teleop()
: Node("tl_teleop")
{
  declare_parameter("arm_ip", "192.168.1.13");
  declare_parameter("arm_port", "6001");
  declare_parameter("arm_port_aux", "7000");
  declare_parameter("arm_joints", 7);
  declare_parameter("pos_scale", POS_SCALE);
  declare_parameter("pos_deadzone", POS_DEADZONE);
  declare_parameter("max_pos_delta_mm", MAX_POS_DELTA_MM);
  declare_parameter("singular_angle", SINGULAR_ANGLE);
  declare_parameter("singular_scale", SINGULAR_SCALE);
  declare_parameter("joint_jump_threshold", JOINT_JUMP_THRESHOLD);
  declare_parameter("servo_speed", 25);
  declare_parameter("publish_rate", 20.0);

  arm_ip_ = get_parameter("arm_ip").as_string();
  arm_port_ = get_parameter("arm_port").as_string();
  arm_port_aux_ = get_parameter("arm_port_aux").as_string();
  arm_joints_ = get_parameter("arm_joints").as_int();
  pos_scale_ = get_parameter("pos_scale").as_double();
  pos_deadzone_ = get_parameter("pos_deadzone").as_double();
  max_pos_delta_mm_ = get_parameter("max_pos_delta_mm").as_double();
  singular_angle_ = get_parameter("singular_angle").as_double();
  singular_scale_ = get_parameter("singular_scale").as_double();
  joint_jump_threshold_ = get_parameter("joint_jump_threshold").as_double();
  servo_speed_ = get_parameter("servo_speed").as_int();
  publish_rate_ = get_parameter("publish_rate").as_double();

  joint_limits_ = DEFAULT_JOINT_LIMITS;
  for (int i = 0; i < arm_joints_; ++i) {
    joint_names_.push_back("joint" + std::to_string(i + 1));
    latest_target_joints_.push_back(0.0);
    servo_vmax_.push_back(80.0);
    servo_amax_.push_back(3000.0);
    servo_jmax_.push_back(50000.0);
  }

  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  auto publish_ms = static_cast<int64_t>(1000.0 / publish_rate_);
  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(publish_ms),
    std::bind(&TL_Teleop::publish_joints, this));

  RCLCPP_INFO(get_logger(), "TL_Teleop node initialized (arm_ip=%s, arm_port=%s, arm_port_aux=%s)",
    arm_ip_.c_str(), arm_port_.c_str(), arm_port_aux_.c_str());
}

TL_Teleop::~TL_Teleop()
{
  running_ = false;
  close_servo();
  disconnect_arm();
  PXREADeinit();
  RCLCPP_INFO(get_logger(), "TL_Teleop destroyed");
}

void TL_Teleop::on_pxrea_client_cb(void * context,
  PXREAClientCallbackType type, int status, void * userData)
{
  auto * self = static_cast<TL_Teleop *>(context);
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
      auto & dsj = *reinterpret_cast<PXREADevStateJson *>(userData);
      try {
        json data = json::parse(dsj.stateJson);
        if (!data.contains("value")) {
          break;
        }
        json value = json::parse(data["value"].get<std::string>());
        if (!value.contains("Controller") || !value["Controller"].contains("right")) {
          break;
        }
        auto & right = value["Controller"]["right"];

        auto pose = parse_pose_str(right["pose"].get<std::string>());
        double grip = right.value("grip", 0.0);
        bool a_button = right.value("primaryButton", false);

        self->vr_state_.update(pose, grip, a_button);
        self->pxrea_ready_ = true;
      } catch (const json::exception & e) {
        RCLCPP_ERROR(self->get_logger(), "PXREA JSON parse error: %s", e.what());
      }
      break;
    }
    default:
      break;
  }
}

std::array<double, 7> TL_Teleop::parse_pose_str(const std::string & s)
{
  std::array<double, 7> result{};
  std::string token;
  std::istringstream iss(s);
  int i = 0;
  while (std::getline(iss, token, ',') && i < 7) {
    result[i++] = std::stod(token);
  }
  return result;
}

bool TL_Teleop::connect_arm()
{
  std::lock_guard<std::mutex> lock(arm_mutex_);
  if (arm_connected_) {
    RCLCPP_INFO(get_logger(), "Arm already connected");
    return true;
  }

  socket_fd_ = connect_robot(arm_ip_, arm_port_);
  socket_fd_aux_ = connect_robot(arm_ip_, arm_port_aux_);

  if (socket_fd_ <= 0 || socket_fd_aux_ <= 0) {
    RCLCPP_ERROR(get_logger(), "Failed to connect to arm at %s:%s,%s",
      arm_ip_.c_str(), arm_port_.c_str(), arm_port_aux_.c_str());
    socket_fd_ = 0;
    socket_fd_aux_ = 0;
    arm_connected_ = false;
    return false;
  }

  arm_connected_ = true;
  RCLCPP_INFO(get_logger(), "Connected to arm at %s:%s,%s",
    arm_ip_.c_str(), arm_port_.c_str(), arm_port_aux_.c_str());

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return true;
}

void TL_Teleop::disconnect_arm()
{
  std::lock_guard<std::mutex> lock(arm_mutex_);
  if (!arm_connected_) {
    return;
  }
  disconnect_robot(socket_fd_);
  disconnect_robot(socket_fd_aux_);
  socket_fd_ = 0;
  socket_fd_aux_ = 0;
  arm_connected_ = false;
  RCLCPP_INFO(get_logger(), "Disconnected from arm");
}

bool TL_Teleop::init_servo()
{
  std::lock_guard<std::mutex> lock(arm_mutex_);
  if (!arm_connected_) {
    RCLCPP_ERROR(get_logger(), "Cannot init servo: arm not connected");
    return false;
  }

  int ret = set_servo_state(socket_fd_, 1);
  if (ret != Result::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "set_servo_state(1) failed: %d", ret);
    return false;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  ret = set_current_mode(socket_fd_, 2);
  if (ret != Result::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "set_current_mode(2) failed: %d", ret);
    return false;
  }

  ret = set_speed(socket_fd_, servo_speed_);
  if (ret != Result::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "set_speed(%d) failed: %d", servo_speed_, ret);
    return false;
  }

  int state = -1;
  get_servo_state(socket_fd_, state);
  switch (state) {
    case 0:
      set_servo_state(socket_fd_, 1);
      set_servo_poweron(socket_fd_);
      break;
    case 1:
      set_servo_poweron(socket_fd_);
      break;
    case 2:
      clear_error(socket_fd_);
      set_servo_state(socket_fd_, 1);
      set_servo_poweron(socket_fd_);
      break;
    case 3:
      RCLCPP_INFO(get_logger(), "[PowerOn]: already power on");
      is_powered_ = true;
      break;
  }

  get_servo_state(socket_fd_, state);
  if (state == 3) {
    is_powered_ = true;
    RCLCPP_INFO(get_logger(), "[PowerOn]: successfully power on, servo_state = %d", state);
  } else {
    RCLCPP_ERROR(get_logger(), "[PowerOn]: failed to power on, servo_state = %d", state);
    return false;
  }

  ret = open_servoJ(socket_fd_aux_, servo_vmax_, servo_amax_, servo_jmax_);
  if (ret != Result::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "open_servoJ failed: %d", ret);
    return false;
  }

  RCLCPP_INFO(get_logger(), "ServoJ initialized");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  return true;
}

void TL_Teleop::close_servo()
{
  std::lock_guard<std::mutex> lock(arm_mutex_);
  if (!arm_connected_) {
    return;
  }
  close_servoJ(socket_fd_aux_);
  set_servo_poweroff(socket_fd_);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  set_servo_state(socket_fd_, 0);
  RCLCPP_INFO(get_logger(), "Servo closed");
}

std::vector<double> TL_Teleop::get_arm_cartesian_pose()
{
  std::lock_guard<std::mutex> lock(arm_mutex_);
  if (!arm_connected_) {
    return {};
  }
  std::vector<double> pos;
  int ret = get_current_position(socket_fd_, 1, pos);
  if (ret != Result::SUCCESS || pos.size() < 6) {
    RCLCPP_WARN(get_logger(), "get_current_position failed: %d", ret);
    return {};
  }
  return pos;
}

std::vector<double> TL_Teleop::get_inverse_kinematics(
  double x, double y, double z, double rx, double ry, double rz)
{
  std::lock_guard<std::mutex> lock(arm_mutex_);
  if (!arm_connected_) {
    return {};
  }

  std::vector<double> origin_pos = {x, y, z, rx, ry, rz, 0.0};
  std::vector<double> target_pos(7, 0.0);

  int ret = get_origin_coord_to_target_coord(
    socket_fd_, 1, origin_pos, 0, target_pos);
  if (ret != Result::SUCCESS) {
    return {};
  }
  return target_pos;
}

std::array<double, 4> TL_Teleop::get_rpy2quat_sdk(double rx, double ry, double rz)
{
  std::lock_guard<std::mutex> lock(arm_mutex_);
  if (!arm_connected_) {
    return {1.0, 0.0, 0.0, 0.0};
  }

  std::vector<double> rpy = {rx, ry, rz};
  std::vector<double> quat(4, 0.0);
  int ret = get_rpy2quat(socket_fd_, rpy, quat);
  if (ret != Result::SUCCESS) {
    return {1.0, 0.0, 0.0, 0.0};
  }
  return {quat[0], quat[1], quat[2], quat[3]};
}

std::vector<double> TL_Teleop::clamp_joints(const std::vector<double> & joints)
{
  std::vector<double> clamped;
  for (size_t i = 0; i < joints.size() && i < joint_limits_.size(); ++i) {
    double lo = joint_limits_[i].first;
    double hi = joint_limits_[i].second;
    clamped.push_back(std::max(lo, std::min(hi, joints[i])));
  }
  return clamped;
}

bool TL_Teleop::joints_safe(const std::vector<double> & new_joints)
{
  if (last_joints_.empty()) {
    last_joints_ = new_joints;
    return true;
  }
  for (size_t i = 0; i < new_joints.size() && i < last_joints_.size(); ++i) {
    if (std::abs(new_joints[i] - last_joints_[i]) > joint_jump_threshold_) {
      RCLCPP_WARN(get_logger(), "Joint %zu jump too large: %.1f deg",
        i, std::abs(new_joints[i] - last_joints_[i]));
      return false;
    }
  }
  last_joints_ = new_joints;
  return true;
}

void TL_Teleop::servoJ_send(const std::vector<double> & joint_target)
{
  std::lock_guard<std::mutex> lock(arm_mutex_);
  if (!arm_connected_) {
    return;
  }
  auto clamped = clamp_joints(joint_target);
  if (!joints_safe(clamped)) {
    return;
  }
  set_servoJ_pos(socket_fd_aux_, clamped);

  {
    std::lock_guard<std::mutex> jlock(joints_mutex_);
    latest_target_joints_ = clamped;
  }
}

std::array<double, 4> TL_Teleop::quat_multiply(
  const std::array<double, 4> & q1, const std::array<double, 4> & q2)
{
  double w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
  double w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];
  return {
    w1*w2 - x1*x2 - y1*y2 - z1*z2,
    w1*x2 + x1*w2 + y1*z2 - z1*y2,
    w1*y2 - x1*z2 + y1*w2 + z1*x2,
    w1*z2 + x1*y2 - y1*x2 + z1*w2
  };
}

std::array<double, 4> TL_Teleop::quat_inverse(const std::array<double, 4> & q)
{
  return {q[0], -q[1], -q[2], -q[3]};
}

std::array<double, 3> TL_Teleop::quat2rpy(const std::array<double, 4> & q)
{
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

void TL_Teleop::control_loop()
{
  double base_x = 0.0, base_y = 0.0, base_z = 0.0;

  while (running_) {
    auto t0 = std::chrono::steady_clock::now();

    std::array<double, 7> pose;
    double grip;
    bool a_button;
    vr_state_.get(pose, grip, a_button);

    double x = pose[0], y = pose[1], z = pose[2];
    double qx = pose[3], qy = pose[4], qz = pose[5], qw = pose[6];

    // A button: zero out and reset home
    if (a_button && arm_connected_) {
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

    if (grip > GRIP_THRESHOLD && arm_connected_) {
      if (!vr_homed_) {
        auto cart = get_arm_cartesian_pose();
        if (!cart.empty() && cart.size() >= 6) {
          base_x = cart[0]; base_y = cart[1]; base_z = cart[2];
          double base_rx = cart[3], base_ry = cart[4], base_rz = cart[5];
          auto arm_quat = get_rpy2quat_sdk(base_rx, base_ry, base_rz);
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
      std::lock_guard<std::mutex> lock(teleop_mutex_);
      vr_homed_ = false;
    }

    bool homed;
    {
      std::lock_guard<std::mutex> lock(teleop_mutex_);
      homed = vr_homed_;
    }

    if (homed && arm_connected_) {
      std::array<double, 7> home_pose;
      std::array<double, 4> arm_q, vr_hq;
      {
        std::lock_guard<std::mutex> lock(teleop_mutex_);
        home_pose = vr_home_pose_;
        arm_q = base_arm_quat_;
        vr_hq = vr_home_quat_;
      }

      double dx = x - home_pose[0];
      double dy = y - home_pose[1];
      double dz = z - home_pose[2];
      if (std::abs(dx) < pos_deadzone_) dx = 0.0;
      if (std::abs(dy) < pos_deadzone_) dy = 0.0;
      if (std::abs(dz) < pos_deadzone_) dz = 0.0;

      // Singular guard
      double scale = 1.0;
      {
        std::lock_guard<std::mutex> alock(arm_mutex_);
        if (!last_joints_.empty() && last_joints_.size() >= 7) {
          if (std::abs(last_joints_[5]) > singular_angle_ ||
              std::abs(last_joints_[6]) > singular_angle_) {
            scale = singular_scale_;
          }
        }
      }

      double arm_delta_X = -dz * 1000.0 * pos_scale_ * scale;
      double arm_delta_Y = -dx * 1000.0 * pos_scale_ * scale;
      double arm_delta_Z =  dy * 1000.0 * pos_scale_ * scale;
      arm_delta_X = std::max(-max_pos_delta_mm_, std::min(max_pos_delta_mm_, arm_delta_X));
      arm_delta_Y = std::max(-max_pos_delta_mm_, std::min(max_pos_delta_mm_, arm_delta_Y));
      arm_delta_Z = std::max(-max_pos_delta_mm_, std::min(max_pos_delta_mm_, arm_delta_Z));

      double target_x = base_x + arm_delta_X;
      double target_y = base_y + arm_delta_Y;
      double target_z = base_z + arm_delta_Z;

      // Ensure shortest quaternion rotation
      std::array<double, 4> vr_quat_wxyz = {qw, qx, qy, qz};
      std::array<double, 4> vr_home_wxyz = {vr_hq[3], vr_hq[0], vr_hq[1], vr_hq[2]};

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

      // Remap: [wd, -zd, -xd, yd] matches Python reference
      std::array<double, 4> vr_delta_mapped = {
        vr_delta[0], -vr_delta[2], -vr_delta[1], vr_delta[3]
      };
      auto target_quat = quat_multiply(vr_delta_mapped, arm_q);

      // Convert target quaternion to RPY for IK
      auto rpy = quat2rpy(target_quat);
      double target_rx = rpy[0];
      double target_ry = -rpy[1];  // Negate pitch per Python reference
      double target_rz = rpy[2];

      auto ik = get_inverse_kinematics(
        target_x, target_y, target_z, target_rx, target_ry, target_rz);
      if (!ik.empty()) {
        servoJ_send(ik);
      }
    }

    auto t1 = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(t1 - t0).count();
    double sleep_time = CONTROL_PERIOD_S - elapsed;
    if (sleep_time > 0) {
      std::this_thread::sleep_for(
        std::chrono::microseconds(static_cast<int64_t>(sleep_time * 1e6)));
    }
  }
}

void TL_Teleop::publish_joints()
{
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = get_clock()->now();
  msg.name = joint_names_;

  std::lock_guard<std::mutex> lock(joints_mutex_);
  msg.position.resize(latest_target_joints_.size());
  for (size_t i = 0; i < latest_target_joints_.size(); ++i) {
    msg.position[i] = latest_target_joints_[i] * DEG_TO_RAD;
  }
  joint_state_pub_->publish(msg);
}

void TL_Teleop::run_control_loop()
{
  RCLCPP_INFO(get_logger(), "Initializing PXREA SDK...");
  int pxrea_ret = PXREAInit(this, on_pxrea_client_cb, PXREAFullMask);
  if (pxrea_ret != 0) {
    RCLCPP_ERROR(get_logger(), "PXREAInit failed: %d", pxrea_ret);
    return;
  }
  RCLCPP_INFO(get_logger(), "PXREA SDK initialized");

  RCLCPP_INFO(get_logger(), "Connecting to arm...");
  if (!connect_arm()) {
    RCLCPP_ERROR(get_logger(), "Arm connection failed, shutting down");
    rclcpp::shutdown();
    return;
  }

  if (!init_servo()) {
    RCLCPP_ERROR(get_logger(), "Servo init failed, shutting down");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(get_logger(), "Teleop ready (singularity guard enabled)");

  control_loop();

  PXREADeinit();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TL_Teleop>();
  std::thread control_thread([&node]() {
    node->run_control_loop();
  });
  rclcpp::spin(node);
  node->stop();
  if (control_thread.joinable()) {
    control_thread.join();
  }
  rclcpp::shutdown();
  return 0;
}
