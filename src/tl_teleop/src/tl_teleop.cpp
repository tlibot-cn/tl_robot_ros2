#include <atomic>
#include <cmath>
#include <future>
#include <mutex>
#include <signal.h>
#include <sstream>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>

#include "tl_teleop/tl_teleop.h"

using json = nlohmann::json;

// 遥操作控制参数
constexpr double POS_SCALE = 0.5;          // 位置缩放系数
constexpr double POS_DEADZONE = 0.005;     // 位置死区（m），小于此值视为无移动
constexpr double MAX_POS_DELTA_MM = 300.0; // 单帧最大位置增量（mm）
constexpr double SINGULAR_ANGLE = 160.0;   // 奇异位形判定角度（deg）
constexpr double SINGULAR_SCALE = 0.2;     // 奇异位形下的缩放系数
constexpr double JOINT_JUMP_THRESHOLD = 30.0; // 关节跳变阈值（deg），超过则丢弃
constexpr double GRIP_THRESHOLD = 0.9;        // 握持键触发阈值，大于此值激活遥操作
constexpr int    CONTROL_PERIOD_MS = 10;      // 控制循环周期（ms），100Hz
constexpr int    SPIN_PERIOD_MS = 100;        // spin 周期（ms），10Hz

static constexpr std::array<double, 7> JOINT_LIMITS_LOW  = {-180, -180, -180, -180, -180, -170, -170};
static constexpr std::array<double, 7> JOINT_LIMITS_HIGH = { 180,  180,  180,  180,  180,  170,  170};

static std::atomic<bool> g_shutting_down{false};

void signal_handler(int sig)
{
    (void)sig;
    g_shutting_down.store(true);
}

// ===================================================================
// 构造 / 析构 / 清理
// ===================================================================
TL_Teleop::TL_Teleop() : rclcpp::Node("tl_teleop")
{
    servo_j_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/tl_driver/set_servoj_pos", 10);

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);

    open_servo_j_client_ = this->create_client<tl_ros2_interface::srv::OpenServoJ>(
        "/tl_driver/open_servoj", rmw_qos_profile_services_default);
    if (!open_servo_j_client_->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(this->get_logger(), "\"/tl_driver/open_servoj\" Service not available");
    }

    close_servo_j_client_ = this->create_client<std_srvs::srv::Trigger>(
        "/tl_driver/close_servoj", rmw_qos_profile_services_default);
    if (!close_servo_j_client_->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(this->get_logger(), "\"/tl_driver/close_servoj\" Service not available");
    }

    coord_transform_client_ = this->create_client<tl_ros2_interface::srv::CoordTransform>(
        "/tl_driver/coord_transform", rmw_qos_profile_services_default);

    rpy2quat_client_ = this->create_client<tl_ros2_interface::srv::GetPosTransform>(
        "/tl_driver/get_rpy2quat", rmw_qos_profile_services_default);

    set_current_mode_client_ = this->create_client<tl_ros2_interface::srv::SetCurrentMode>(
        "/tl_driver/set_current_mode", rmw_qos_profile_services_default);
    if (!set_current_mode_client_->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(this->get_logger(), "\"/tl_driver/set_current_mode\" Service not available");
    }

    set_speed_client_ = this->create_client<tl_ros2_interface::srv::SetSpeed>(
        "/tl_driver/set_speed", rmw_qos_profile_services_default);
    if (!set_speed_client_->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(this->get_logger(), "\"/tl_driver/set_speed\" Service not available");
    }

    for (int i = 1; i <= 7; ++i) {
        joint_names_.push_back("joint" + std::to_string(i));
    }

    tcp_pose_sub_ = this->create_subscription<tl_ros2_interface::msg::CartesianPose>(
        "/tcp_pose", 10,
        std::bind(&TL_Teleop::tcp_pose_callback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&TL_Teleop::joint_state_callback, this, std::placeholders::_1));

    PXREAInit(this, &TL_Teleop::on_pxrea_client_cb, PXREAFullMask);

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(CONTROL_PERIOD_MS),
        std::bind(&TL_Teleop::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Teleop control timer created (%d Hz)",
                1000 / CONTROL_PERIOD_MS);
}

TL_Teleop::~TL_Teleop()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down...");
    cleanup();
    RCLCPP_INFO(this->get_logger(), "Shutdown complete");
    rclcpp::shutdown();
}

void TL_Teleop::cleanup()
{
    if (cleaned_up_.exchange(true)) return;
    PXREADeinit();
    RCLCPP_INFO(this->get_logger(), "PXREADeinit completed");
}

// ===================================================================
// PXREA 设备回调 — 运行在 SDK 内部线程
// JSON 结构：外层 {"value": "<内层JSON字符串>"}
//           内层 {"Controller": {"right": {"pose": "x,y,z,qx,qy,qz,qw", "grip": 0.0~1.0}}}
// ===================================================================
void TL_Teleop::on_pxrea_client_cb(void* context,
                                    PXREAClientCallbackType type,
                                    int status, void* userData)
{
    (void)status;
    auto* self = static_cast<TL_Teleop*>(context);

    switch (type) {
    case PXREAServerConnect:
        RCLCPP_INFO(self->get_logger(), "PXREA: server connect");
        break;
    case PXREAServerDisconnect:
        RCLCPP_WARN(self->get_logger(), "PXREA: server disconnect");
        break;
    case PXREADeviceFind:
        RCLCPP_INFO(self->get_logger(), "PXREA: device find %s", (const char*)userData);
        break;
    case PXREADeviceMissing:
        RCLCPP_WARN(self->get_logger(), "PXREA: device missing %s", (const char*)userData);
        break;
    case PXREADeviceConnect:
        RCLCPP_INFO(self->get_logger(), "PXREA: device connect %s", (const char*)userData);
        break;
    case PXREADeviceStateJson: {
        auto& dsj = *reinterpret_cast<PXREADevStateJson*>(userData);
        try {
            json data = json::parse(dsj.stateJson);
            if (!data.contains("value")) {
                RCLCPP_WARN(self->get_logger(),
                    "PXREA stateJson missing 'value' key. Top-level keys: %s",
                    [&](){ std::string ks; for (auto& [k,_] : data.items()) { ks += k + " "; } return ks; }().c_str());
                break;
            }

            json value = json::parse(data["value"].get<std::string>());
            if (!value.contains("Controller")) {
                RCLCPP_WARN(self->get_logger(),
                    "PXREA inner JSON missing 'Controller' key. Top-level keys: %s",
                    [&](){ std::string ks; for (auto& [k,_] : value.items()) { ks += k + " "; } return ks; }().c_str());
                break;
            }
            if (!value["Controller"].contains("right")) {
                RCLCPP_WARN(self->get_logger(),
                    "PXREA 'Controller' missing 'right' key. Controller keys: %s",
                    [&](){ std::string ks; for (auto& [k,_] : value["Controller"].items()) { ks += k + " "; } return ks; }().c_str());
                break;
            }

            auto& right = value["Controller"]["right"];

            auto pose = parse_pose_str(right["pose"].get<std::string>());
            double grip = right.value("grip", 0.0);

            self->vr_state_.update(pose, grip);

            // RCLCPP_INFO(self->get_logger(),
            //     "PXREA pose: x=%.3f y=%.3f z=%.3f qx=%.3f qy=%.3f qz=%.3f qw=%.3f  grip=%.3f  ready=%d",
            //     pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6],
            //     grip, 1);
        } catch (const json::exception& e) {
            RCLCPP_ERROR(self->get_logger(), "PXREA JSON parse error: %s", e.what());
        }
        break;
    }
    default:
        break;
    }
}

// ===================================================================
// 伺服关节 API（同步调用，在 executor 空闲时执行）
// ===================================================================
bool TL_Teleop::open_servo_j()
{
    auto request = std::make_shared<tl_ros2_interface::srv::OpenServoJ_Request>();
    request->vmax = {80, 80, 80, 80, 80, 80, 80};
    request->amax = {3000, 3000, 3000, 3000, 3000, 3000, 3000};
    request->jmax = {50000, 50000, 50000, 50000, 50000, 50000, 50000};

    auto future = open_servo_j_client_->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(
        this->shared_from_this(), future, std::chrono::seconds(1));

    if (result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "open_servo_j timed out");
        return false;
    }

    auto response = future.get();
    servo_j_opened_ = response->success;
    RCLCPP_INFO(this->get_logger(), "open_servo_j: %s", response->message.c_str());
    return response->success;
}

bool TL_Teleop::close_servo_j()
{
    auto request = std::make_shared<std_srvs::srv::Trigger_Request>();
    auto future = close_servo_j_client_->async_send_request(request);

    auto result = rclcpp::spin_until_future_complete(
        this->shared_from_this(), future, std::chrono::seconds(1));

    if (result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "close_servo_j timed out");
        return false;
    }

    auto response = future.get();
    servo_j_opened_ = false;
    RCLCPP_INFO(this->get_logger(), "close_servo_j: %s", response->message.c_str());
    return response->success;
}

bool TL_Teleop::set_current_mode(int mode)
{
    auto request = std::make_shared<tl_ros2_interface::srv::SetCurrentMode::Request>();
    request->mode = mode;

    auto future = set_current_mode_client_->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(
        this->shared_from_this(), future, std::chrono::seconds(1));

    if (result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "set_current_mode timed out");
        return false;
    }

    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "set_current_mode(%d): %s", mode, response->message.c_str());
    return response->success;
}

bool TL_Teleop::set_speed(double speed)
{
    auto request = std::make_shared<tl_ros2_interface::srv::SetSpeed::Request>();
    request->speed = speed;

    auto future = set_speed_client_->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(
        this->shared_from_this(), future, std::chrono::seconds(1));

    if (result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "set_speed timed out");
        return false;
    }

    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "set_speed(%.0f): %s", speed, response->message.c_str());
    return response->success;
}

// ===================================================================
// 工具函数
// ===================================================================
std::array<double, 7> TL_Teleop::parse_pose_str(const std::string& s)
{
    std::array<double, 7> pose{};
    std::stringstream ss(s);
    std::string token;
    int i = 0;
    while (std::getline(ss, token, ',') && i < 7) {
        pose[i++] = std::stod(token);
    }
    return pose;
}

// ===================================================================
// 四元数运算（标量在前：[w, x, y, z]）
// ===================================================================
std::array<double, 4> TL_Teleop::quat_multiply(const std::array<double,4>& a,
                                                const std::array<double,4>& b)
{
    double w1 = a[0], x1 = a[1], y1 = a[2], z1 = a[3];
    double w2 = b[0], x2 = b[1], y2 = b[2], z2 = b[3];
    return {
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    };
}

std::array<double, 4> TL_Teleop::quat_inverse(const std::array<double,4>& q)
{
    return {q[0], -q[1], -q[2], -q[3]};
}

std::array<double, 3> TL_Teleop::quat2rpy(const std::array<double,4>& q_wxyz)
{
    double w = q_wxyz[0], x = q_wxyz[1], y = q_wxyz[2], z = q_wxyz[3];
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

// ===================================================================
// 安全检查
// ===================================================================
std::vector<double> TL_Teleop::clamp_joints(const std::vector<double>& joints) const
{
    std::vector<double> clamped(joints.size());
    for (size_t i = 0; i < joints.size() && i < 7; ++i) {
        clamped[i] = std::max(JOINT_LIMITS_LOW[i], std::min(JOINT_LIMITS_HIGH[i], joints[i]));
    }
    return clamped;
}

bool TL_Teleop::joints_safe(const std::vector<double>& new_joints,
                             std::vector<double>& last_joints) const
{
    if (last_joints.empty()) {
        last_joints = new_joints;
        return true;
    }
    for (size_t i = 0; i < new_joints.size() && i < last_joints.size(); ++i) {
        if (std::abs(new_joints[i] - last_joints[i]) > JOINT_JUMP_THRESHOLD) {
            RCLCPP_WARN(this->get_logger(),
                        "Joint jump too large: j%zu = %.1f deg", i + 1,
                        std::abs(new_joints[i] - last_joints[i]));
            return false;
        }
    }
    last_joints = new_joints;
    return true;
}

// ===================================================================
// IK 响应回调 — 安全检查 + 发布
// ===================================================================
void TL_Teleop::on_ik_response(
    rclcpp::Client<tl_ros2_interface::srv::CoordTransform>::SharedFuture future)
{
    ik_pending_ = false;

    auto response = future.get();
    if (!response->success) {
        RCLCPP_WARN(this->get_logger(),
            "[IK OUTPUT] Unreachable target — IK solver returned failure, skipping frame");
        return;
    }

    auto raw_joints = response->target_pos;
    RCLCPP_INFO(this->get_logger(),
        "[IK OUTPUT] Joints(deg): j1=%.2f j2=%.2f j3=%.2f j4=%.2f j5=%.2f j6=%.2f j7=%.2f",
        raw_joints[0], raw_joints[1], raw_joints[2], raw_joints[3],
        raw_joints[4], raw_joints[5], raw_joints[6]);

    auto joints = clamp_joints(response->target_pos);
    if (!joints_safe(joints, last_joints_)) return;

    std_msgs::msg::Float64MultiArray servo_msg;
    servo_msg.data = joints;
    servo_j_pub_->publish(servo_msg); // (mm, deg)

    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name = joint_names_;
    js.position.resize(joints.size());
    for (size_t i = 0; i < joints.size(); ++i) {
        js.position[i] = joints[i] * M_PI / 180.0;
    }
    joint_state_pub_->publish(js);
}

// ===================================================================
// tcp_pose 回调 — 缓存机械臂末端位姿（单位：mm, rad）
// ===================================================================
void TL_Teleop::tcp_pose_callback(const tl_ros2_interface::msg::CartesianPose& msg)
{
    latest_arm_cart_[0] = msg.position.x;
    latest_arm_cart_[1] = msg.position.y;
    latest_arm_cart_[2] = msg.position.z;
    latest_arm_cart_[3] = msg.rpy.x;
    latest_arm_cart_[4] = msg.rpy.y;
    latest_arm_cart_[5] = msg.rpy.z;
    latest_arm_cart_[6] = msg.arm_angle;
    arm_pose_valid_ = true;
}

void TL_Teleop::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    latest_joint_positions_deg_.resize(msg->position.size());
    for (size_t i = 0; i < msg->position.size(); ++i) {
        latest_joint_positions_deg_[i] = msg->position[i] * 180.0 / M_PI;
    }
    joint_state_valid_ = true;
}

// ===================================================================
// 运行入口 — 开启伺服关节，进入事件循环
// ===================================================================
void TL_Teleop::run()
{
    RCLCPP_INFO(this->get_logger(), "Init sequence: set mode=2 (remote), speed=25");
    if (!set_current_mode(2)) {
        RCLCPP_ERROR(this->get_logger(), "set_current_mode(2) failed");
    }
    if (!set_speed(25)) {
        RCLCPP_ERROR(this->get_logger(), "set_speed(25) failed");
    }

    if (open_servo_j()) {
        RCLCPP_INFO(this->get_logger(), "open_servo_j succeeded, waiting 2s...");
        std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
        RCLCPP_ERROR(this->get_logger(), "open_servo_j failed");
    }

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(this->get_node_base_interface());
    while (rclcpp::ok() && !g_shutting_down.load()) {
        exec.spin_once(std::chrono::milliseconds(SPIN_PERIOD_MS));
    }
    exec.remove_node(this->get_node_base_interface());

    if (servo_j_opened_) {
        RCLCPP_INFO(this->get_logger(), "Closing servo J...");
        close_servo_j();
    }
}

// ===================================================================
// 坐标变换 & IK 请求（100Hz）
//
// 单位约定：
//   位置：mm（VR 手柄 m→mm 已转换）
//   欧拉角：弧度（quat2rpy / CoordTransform 服务均用弧度）
//   关节角：°（/joint_states 收到的转为 ° 存储，servoJ 发布也用 °）
//
// 整体流程：
//   1. 读取 VR 手柄数据（pose + grip）
//   2. grip > 0.9 时激活遥操作，记录 home 快照（VR 位姿 + 机械臂位姿）
//   3. 每帧计算 VR 相对于 home 的位移和旋转增量
//   4. 增量映射到机械臂基座坐标系（VR坐标系 ≠ 机械臂坐标系）
//   5. 发送 CoordTransform 服务做逆运动学求解
//
// 坐标系映射：
//   位置：VR前后(z) → Arm X, VR左右(x) → Arm Y, VR上下(y) → Arm Z
//   旋转：q_delta = q_vr * home_q_vr⁻¹  →  q_mapped  →  q_target = q_mapped * home_q_arm
// ===================================================================
void TL_Teleop::control_loop()
{
    bool ready;
    std::array<double, 7> pose;
    double grip;

    // --- 读取 VR 数据 ---
    ready = vr_state_.read(pose, grip);
    if (!ready) return;  // PXREA 设备尚未发送数据，快速退出

    double x = pose[0], y = pose[1], z = pose[2];
    // pose 存储顺序为 [x,y,z, qx,qy,qz,qw]，转为标量在前 [w,x,y,z]
    std::array<double, 4> vr_quat_wxyz = {pose[6], pose[3], pose[4], pose[5]};

    // --- 握持键状态机 ---

    if (grip > GRIP_THRESHOLD) {
        // 握持键按下
        if (!teleop_active_) {
            // 激活瞬间：必须已收到机械臂末端位姿（/tcp_pose）
            if (!arm_pose_valid_) {
                RCLCPP_WARN(this->get_logger(),
                            "No arm pose received yet, cannot activate teleop");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Teleop activated (grip=%.2f)", grip);
            teleop_active_ = true;

            // 记录 home 快照，后续增量均以此为基准
            home_vr_pose_ = pose;          // VR 手柄位姿
            home_vr_quat_ = vr_quat_wxyz;  // VR 手柄四元数
            home_arm_cart_ = latest_arm_cart_;  // 机械臂末端位姿 [x,y,z,rx,ry,rz,arm_angle]

RCLCPP_INFO(this->get_logger(),
    "[HOME_ARM] Pos(mm): x=%.2f y=%.2f z=%.2f  RPY(rad): rx=%.2f ry=%.2f rz=%.2f  arm_angle=%.2f",
                home_arm_cart_[0], home_arm_cart_[1], home_arm_cart_[2],
                home_arm_cart_[3], home_arm_cart_[4], home_arm_cart_[5],
                home_arm_cart_[6]);

            home_quat_ready_ = false;
            if (joint_state_valid_ && !latest_joint_positions_deg_.empty()) {
                last_joints_ = latest_joint_positions_deg_;
            } else {
                last_joints_.clear();
            }

            // 请求 rpy2quat 服务：将机械臂末端 RPY 转为四元数
            auto rpy_req = std::make_shared<tl_ros2_interface::srv::GetPosTransform::Request>();
            rpy_req->input = {home_arm_cart_[3], home_arm_cart_[4], home_arm_cart_[5]};
            rpy2quat_client_->async_send_request(rpy_req,
                [this](rclcpp::Client<tl_ros2_interface::srv::GetPosTransform>::SharedFuture f) {
                    auto& out = f.get()->output;
                    if (out.size() >= 4) {
                        home_arm_quat_ = {out[3], out[0], out[1], out[2]};
                        RCLCPP_INFO(this->get_logger(),
                            "[HOME_ARM_QUAT] w=%.4f x=%.4f y=%.4f z=%.4f",
                            home_arm_quat_[0], home_arm_quat_[1],
                            home_arm_quat_[2], home_arm_quat_[3]);
                    }
                    home_quat_ready_ = true;
                });
        }
    } else {
        // 握持键松开
        if (teleop_active_) {
            RCLCPP_INFO(this->get_logger(), "Teleop deactivated");
            teleop_active_ = false;
        }
        return;
    }

    // --- 前置条件 ---
    if (!teleop_active_ || !home_quat_ready_) return;  // 未激活或 rpy2quat 还在等待响应
    if (ik_pending_) return;  // 上一帧 IK 请求尚未返回，跳过本帧

    // --- 步骤4：位置增量（VR 手柄位移 → 机械臂末端位移） ---

    // 当前 VR 手柄相对于激活瞬间的位移（单位：m）
    double dx = x - home_vr_pose_[0];
    double dy = y - home_vr_pose_[1];
    double dz = z - home_vr_pose_[2];

    // 死区过滤：小于 5mm 的位移视为抖动，清零
    if (std::abs(dx) < POS_DEADZONE) dx = 0.0;
    if (std::abs(dy) < POS_DEADZONE) dy = 0.0;
    if (std::abs(dz) < POS_DEADZONE) dz = 0.0;

    // 奇异位形检测：关节5或关节6接近180°时降速，防止关节突变
    double scale = 1.0;
    if (!last_joints_.empty() && last_joints_.size() >= 7) {
        if (std::abs(last_joints_[5]) > SINGULAR_ANGLE ||
            std::abs(last_joints_[6]) > SINGULAR_ANGLE) {
            scale = SINGULAR_SCALE;
        }
    }

    // VR 坐标系 → 机械臂基座坐标系
    //   VR前后(z) → Arm X, VR左右(x) → Arm Y, VR上下(y) → Arm Z
    //   并做单位转换（m→mm）和缩放
    double arm_delta_X = -dz * 1000.0 * POS_SCALE * scale;
    double arm_delta_Y = -dx * 1000.0 * POS_SCALE * scale;
    double arm_delta_Z =  dy * 1000.0 * POS_SCALE * scale;

    // 增量限幅：单帧不超过 ±300mm
    arm_delta_X = std::max(-MAX_POS_DELTA_MM, std::min(MAX_POS_DELTA_MM, arm_delta_X));
    arm_delta_Y = std::max(-MAX_POS_DELTA_MM, std::min(MAX_POS_DELTA_MM, arm_delta_Y));
    arm_delta_Z = std::max(-MAX_POS_DELTA_MM, std::min(MAX_POS_DELTA_MM, arm_delta_Z));

    RCLCPP_INFO(this->get_logger(),
        "[DELTA] VR_raw(m): dx=%.4f dy=%.4f dz=%.4f  "
        "Arm_mapped(mm): dX=%.2f dY=%.2f dZ=%.2f  scale(%.2f)",
        dx, dy, dz, arm_delta_X, arm_delta_Y, arm_delta_Z, scale);

    // 目标末端位置 = home 位置 + 增量
    double target_x = home_arm_cart_[0] + arm_delta_X;
    double target_y = home_arm_cart_[1] + arm_delta_Y;
    double target_z = home_arm_cart_[2] + arm_delta_Z;

    // --- 旋转增量（VR 四元数 → 机械臂末端 RPY） ---

    // 四元数 q 和 -q 表示相同旋转，通过点积取较短路径
    double dot = vr_quat_wxyz[0] * home_vr_quat_[0] +
                 vr_quat_wxyz[1] * home_vr_quat_[1] +
                 vr_quat_wxyz[2] * home_vr_quat_[2] +
                 vr_quat_wxyz[3] * home_vr_quat_[3];
    std::array<double, 4> q_signed = vr_quat_wxyz;
    if (dot < 0.0) {
        q_signed = {-q_signed[0], -q_signed[1], -q_signed[2], -q_signed[3]};
    }

    // 相对旋转：q_delta = q_signed × home_q^-1
    auto q_inv = quat_inverse(home_vr_quat_);
    auto q_delta = quat_multiply(q_signed, q_inv);
    double wd = q_delta[0], xd = q_delta[1], yd = q_delta[2], zd = q_delta[3];

    // 坐标映射：VR 四元数分量 → 机械臂四元数分量
    //   q_mapped = {wd, -zd, -xd, yd}   （w → w, z → -x, x → -y, y → z）
    std::array<double, 4> q_mapped = {wd, -zd, -xd, yd};

    // 叠加到机械臂 home 位姿：q_target = q_mapped × home_q_arm
    auto q_target = quat_multiply(q_mapped, home_arm_quat_);
    auto rpy = quat2rpy(q_target);

    // quat2rpy 返回弧度，CoordTransform 服务（NRC IK）期望弧度 — 不要转角度
    double target_rx = rpy[0];       // [rad]
    double target_ry = -rpy[1];      // [rad]  取反：VR→机械臂坐标系映射
    double target_rz = rpy[2];       // [rad]

    // --- 发送逆运动学请求 ---
    // origin_pos: [x(mm), y(mm), z(mm), rx(rad), ry(rad), rz(rad), arm_angle(rad)]
    auto ik_req = std::make_shared<tl_ros2_interface::srv::CoordTransform::Request>();
    ik_req->origin_coord = 1;
    ik_req->target_coord = 0;
    ik_req->form = 0;
    ik_req->origin_pos = {target_x, target_y, target_z, target_rx, target_ry, target_rz, 0.0};

RCLCPP_INFO(this->get_logger(),
    "[IK INPUT]  Pos(mm): x=%.2f y=%.2f z=%.2f  RPY(rad): rx=%.2f ry=%.2f rz=%.2f",
        target_x, target_y, target_z, target_rx, target_ry, target_rz);

    ik_pending_ = true;  // 标记 IK 进行中，阻止下一帧发送新请求
    coord_transform_client_->async_send_request(ik_req,
        std::bind(&TL_Teleop::on_ik_response, this, std::placeholders::_1));
    // 响应到达 → on_ik_response: 安全检查 → 发布关节角度 → ik_pending_ = false
}

// ===================================================================
// 主函数
// ===================================================================
int main(int argc, char* argv[])
{
    rclcpp::InitOptions init_opts;
    init_opts.shutdown_on_signal = false;
    rclcpp::init(argc, argv, init_opts);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    auto node = std::make_shared<TL_Teleop>();
    node->run();
    node.reset();

    return 0;
}
