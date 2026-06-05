#ifndef TL_TELEOP_H_
#define TL_TELEOP_H_

#include <PXREARobotSDK.h>
#include <stddef.h>

#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tl_ros2_interface/msg/cartesian_pose.hpp>
#include <tl_ros2_interface/srv/coord_transform.hpp>
#include <tl_ros2_interface/srv/get_pos_transform.hpp>
#include <tl_ros2_interface/srv/open_servo_j.hpp>
#include <tl_ros2_interface/srv/set_current_mode.hpp>
#include <tl_ros2_interface/srv/set_speed.hpp>

class TL_Teleop : public rclcpp::Node
{
public:
    TL_Teleop();
    ~TL_Teleop();

    TL_Teleop(const TL_Teleop&) = delete;
    TL_Teleop& operator=(const TL_Teleop&) = delete;

    void cleanup();
    void run();

    static void on_pxrea_client_cb(void* context,
                                   PXREAClientCallbackType type,
                                   int status,
                                   void* userData);

private:
    bool open_servo_j();
    bool close_servo_j();
    bool set_current_mode(int mode);
    bool set_speed(double speed);
    void pub_servo_j_pos();
    void control_loop();
    void tcp_pose_callback(const tl_ros2_interface::msg::CartesianPose& msg);
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void on_ik_response(rclcpp::Client<tl_ros2_interface::srv::CoordTransform>::SharedFuture future);

    static std::array<double, 4> quat_multiply(const std::array<double,4>& a,
                                                const std::array<double,4>& b);
    static std::array<double, 4> quat_inverse(const std::array<double,4>& q);
    static std::array<double, 3> quat2rpy(const std::array<double,4>& q_wxyz);
    static std::array<double, 7> parse_pose_str(const std::string& s);

    std::vector<double> clamp_joints(const std::vector<double>& joints) const;
    bool joints_safe(const std::vector<double>& new_joints,
                     std::vector<double>& last_joints) const;

    rclcpp::Client<tl_ros2_interface::srv::OpenServoJ>::SharedPtr open_servo_j_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr close_servo_j_client_;
    rclcpp::Client<tl_ros2_interface::srv::CoordTransform>::SharedPtr coord_transform_client_;
    rclcpp::Client<tl_ros2_interface::srv::GetPosTransform>::SharedPtr rpy2quat_client_;
    rclcpp::Client<tl_ros2_interface::srv::SetCurrentMode>::SharedPtr set_current_mode_client_;
    rclcpp::Client<tl_ros2_interface::srv::SetSpeed>::SharedPtr set_speed_client_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr servo_j_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<tl_ros2_interface::msg::CartesianPose>::SharedPtr tcp_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::atomic<bool> cleaned_up_{false};
    bool servo_j_opened_{false};

    // VR 手柄状态 — 读写均通过 update() / read() 保证线程安全
    struct VRState {
        mutable std::mutex mutex;
        std::array<double, 7> pose{};   // x,y,z, qx,qy,qz,qw
        double grip{0.0};               // 握持键值 0.0~1.0
        bool ready{false};

        void update(const std::array<double, 7>& p, double g) {
            std::lock_guard<std::mutex> lock(mutex);
            pose = p;
            grip = g;
            ready = true;
        }

        bool read(std::array<double, 7>& out_pose, double& out_grip) const {
            std::lock_guard<std::mutex> lock(mutex);
            out_pose = pose;
            out_grip = grip;
            return ready;
        }
    };
    VRState vr_state_;

    // /tcp_pose 发布的是弧度（tl_driver 中 publish_arm_state 对 tcp_pose 无 deg→rad 转换）
    std::array<double, 7> latest_arm_cart_{};   // 机械臂当前末端位姿（mm, rad）
    bool arm_pose_valid_{false};    // 是否已收到 /tcp_pose 数据
    std::vector<double> latest_joint_positions_deg_;  // 当前关节位置（deg），来自 /joint_states
    bool joint_state_valid_{false}; // 是否已收到 /joint_states 数据

    // 遥操作状态机
    bool teleop_active_{false};     // 遥操作激活中（grip > 0.9 时置 true）
    bool home_quat_ready_{false};   // rpy2quat 服务已返回 home 四元数
    bool ik_pending_{false};        // IK 请求已发送，等待响应
    std::array<double, 7> home_arm_cart_{};   // 激活瞬间机械臂末端位姿
    std::array<double, 7> home_vr_pose_{};    // 激活瞬间 VR 手柄位姿
    std::array<double, 4> home_arm_quat_{};   // 激活瞬间机械臂末端四元数（WXYZ）
    std::array<double, 4> home_vr_quat_{};    // 激活瞬间 VR 手柄四元数（WXYZ）

    mutable std::vector<double> last_joints_;
    std::vector<std::string> joint_names_;
};

#endif  // TL_TELEOP_H_
