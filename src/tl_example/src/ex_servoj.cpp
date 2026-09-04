/**
 * @file ex_servoj.cpp
 * @brief ServoJ 关节实时跟踪测试
 *
 * 对应 tl_sdk/examples/cpp/ex_servoj.cpp 的 ROS2 实现。
 * 演示透传模式下的关节实时跟踪（ServoJ）：以 100Hz 频率向 7000 端口
 * 高频下发目标关节角度，控制器直接响应，实现平滑的关节轨迹实时伺服。
 *
 *   connect（自动双端口 6001+7000）→ clear_error → power_off（复位）→
 *   set_current_mode(0 示教) → power_on（含 set_servo_state(1)）→
 *   set_current_mode(2 运行) → open_servoj(vmax/amax/jmax) →
 *   动作1: J1 0°→30° → 动作2: J1 回 0° + J2 0°→20° →
 *   动作3: J2 回 0° + J3 0°→15° → 动作4: 全轴回零 →
 *   close_servoj → set_current_mode(0 恢复) → disconnect
 *
 * ⚠ 安全警告：ServoJ 会真实驱动机器人运动！
 *   运行前请确保工作区安全、无人靠近，并随时准备急停。
 *
 * ⚠ 注意：servoj 相关向量必须为 7 元素（前 6 轴角度，第 7 轴补 0），
 *   传 6 元素会导致协议解析错乱断开 7000 连接。
 *
 * ⚠ 与 SDK 差异（无跳过项，全部接口均有 ROS2 对应）：
 *   - SDK 手动双端口连接(6001+7000) → ROS2 connect_arm 服务自动双端口连接
 *   - SDK set_servo_state(1) → ROS2 无独立服务，power_on 服务内部含此步
 *   - 若关节不动或运行缓慢，请先用 /tl_driver/set_speed 增大运行速度
 *
 * @usage
 *   ros2 run tl_example ex_servoj
 *   # 自定义 ServoJ 约束与发送周期
 *   ros2 run tl_example ex_servoj --ros-args -p vmax:=30.0 -p amax:=60.0 \
 *       -p jmax:=100.0 -p period_ms:=10
 *
 * @see ex_move_control.cpp — MoveJ/MoveL 常规轨迹运动测试
 * @see ex_servoj_test.cpp — 单关节 ServoJ 插补测试（更简单的起点）
 */

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include "tl_ros2_interface/msg/arm_status.hpp"
#include "tl_ros2_interface/srv/open_servo_j.hpp"
#include "tl_ros2_interface/srv/set_current_mode.hpp"

namespace tl_example
{

// 类型别名 — 简化代码
using Trigger = std_srvs::srv::Trigger;
using ArmStatus = tl_ros2_interface::msg::ArmStatus;
using JointState = sensor_msgs::msg::JointState;
using Float64Array = std_msgs::msg::Float64MultiArray;
using OpenServoJ = tl_ros2_interface::srv::OpenServoJ;
using SetCurrentMode = tl_ros2_interface::srv::SetCurrentMode;

// 关节数（servoj 向量固定 7 元素：前 6 轴 + 第 7 轴补 0）
static constexpr size_t kNJoints = 7;

/**
 * @brief ServoJ 关节实时跟踪测试节点
 *
 * 流程（同步顺序执行）：
 *   connect（自动双端口）→ clear_error → power_off（复位）→ set_current_mode(0 示教) →
 *   power_on（含 set_servo_state(1)）→ set_current_mode(2 运行) →
 *   open_servoj(vmax/amax/jmax) → 4 段 100Hz 关节轨迹 →
 *   close_servoj → set_current_mode(0 恢复) → disconnect
 * 每段轨迹完成后打印 /joint_states 实际角度（rad→度）用于验证实时跟踪。
 */
class ServoJDemo : public rclcpp::Node
{
public:
  explicit ServoJDemo(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("ex_servoj", options)
  {
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "  ServoJ 关节实时跟踪测试节点启动");
    RCLCPP_INFO(this->get_logger(), "  ⚠ 本示例会真实驱动机器人运动，请注意安全");
    RCLCPP_INFO(this->get_logger(), "============================================================");

    // ── 参数 ──
    this->declare_parameter("vmax", 30.0);    // ServoJ 最大速度（°/s）
    this->declare_parameter("amax", 60.0);    // ServoJ 最大加速度（°/s²）
    this->declare_parameter("jmax", 100.0);   // ServoJ 最大加加速度（°/s³）
    this->declare_parameter("period_ms", 10); // 发送周期（毫秒）→ 100Hz

    vmax_ = this->get_parameter("vmax").as_double();
    amax_ = this->get_parameter("amax").as_double();
    jmax_ = this->get_parameter("jmax").as_double();
    period_ms_ = this->get_parameter("period_ms").as_int();

    // ── 服务客户端 ──
    connect_cli_ = this->create_client<Trigger>("/tl_driver/connect_arm");
    disconnect_cli_ = this->create_client<Trigger>("/tl_driver/disconnect_arm");
    clear_error_cli_ = this->create_client<Trigger>("/tl_driver/clear_error");
    power_on_cli_ = this->create_client<Trigger>("/tl_driver/power_on");
    power_off_cli_ = this->create_client<Trigger>("/tl_driver/power_off");
    set_mode_cli_ = this->create_client<SetCurrentMode>("/tl_driver/set_current_mode");
    open_servoj_cli_ = this->create_client<OpenServoJ>("/tl_driver/open_servoj");
    close_servoj_cli_ = this->create_client<Trigger>("/tl_driver/close_servoj");

    // ── 高频下发话题（100Hz 向 7000 端口发送目标关节角度，单位 °）──
    servoj_pos_pub_ = this->create_publisher<Float64Array>("/tl_driver/set_servoj_pos", 10);

    // ── 状态订阅（运行状态 + 关节角度，用于验证实时跟踪）──
    arm_status_sub_ = this->create_subscription<ArmStatus>(
        "/arm_status", 10, std::bind(&ServoJDemo::onArmStatus, this, std::placeholders::_1));
    joint_state_sub_ = this->create_subscription<JointState>(
        "/joint_states", 10, std::bind(&ServoJDemo::onJointState, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "  客户端/订阅创建完成 (ServoJ 向量维度 %zu: 前 6 轴 + 第 7 轴补 0, %dms @100Hz)",
                kNJoints, period_ms_);
  }

  // ── 主流程（main 直接调用）──
  void run();

private:
  // ── ROS2 通信 ──
  rclcpp::Client<Trigger>::SharedPtr connect_cli_;
  rclcpp::Client<Trigger>::SharedPtr disconnect_cli_;
  rclcpp::Client<Trigger>::SharedPtr clear_error_cli_;
  rclcpp::Client<Trigger>::SharedPtr power_on_cli_;
  rclcpp::Client<Trigger>::SharedPtr power_off_cli_;
  rclcpp::Client<SetCurrentMode>::SharedPtr set_mode_cli_;
  rclcpp::Client<OpenServoJ>::SharedPtr open_servoj_cli_;
  rclcpp::Client<Trigger>::SharedPtr close_servoj_cli_;
  rclcpp::Publisher<Float64Array>::SharedPtr servoj_pos_pub_;
  rclcpp::Subscription<ArmStatus>::SharedPtr arm_status_sub_;
  rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;

  // 最近一次接收到的机械臂运行状态（/arm_status）
  std::string arm_run_state_ = "unknown";
  // 最近一次接收到的关节角度（话题为弧度，打印时转角度）
  JointState::SharedPtr last_joint_state_;

  // ── 参数 ──
  // ServoJ 速度/加速度/加加速度约束（°/s, °/s², °/s³，open 时按 kNJoints 填充）
  double vmax_ = 30.0;
  double amax_ = 60.0;
  double jmax_ = 100.0;
  // 发送周期（毫秒）→ 100Hz
  int period_ms_ = 10;

  // ── 话题回调 ──
  void onArmStatus(const ArmStatus::SharedPtr msg)
  {
    arm_run_state_ = msg->run_state;
  }
  void onJointState(const JointState::SharedPtr msg)
  {
    last_joint_state_ = msg;
  }

  // ── 辅助方法 ──

  /// 等待单个服务就绪
  bool waitService(const std::string& name, rclcpp::ClientBase::SharedPtr cli, double timeout_s = 3.0)
  {
    if (!cli->wait_for_service(std::chrono::duration<double>(timeout_s)))
    {
      RCLCPP_ERROR(this->get_logger(), "  服务 %s 未就绪（%.0fs 超时）", name.c_str(), timeout_s);
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "  服务 %s 已就绪", name.c_str());
    return true;
  }

  /// 调用 Trigger 服务
  bool callTrigger(rclcpp::Client<Trigger>::SharedPtr cli, const std::string& label)
  {
    if (!cli->service_is_ready())
    {
      RCLCPP_WARN(this->get_logger(), "  [%s] 服务未就绪，跳过", label.c_str());
      return false;
    }
    auto req = std::make_shared<Trigger::Request>();
    auto future = cli->async_send_request(req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(10)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(), "  [%s] 调用超时/异常", label.c_str());
      return false;
    }
    const auto& resp = future.get();
    if (resp->success)
    {
      RCLCPP_INFO(this->get_logger(), "  ✓ %s 成功", label.c_str());
      if (!resp->message.empty())
        RCLCPP_INFO(this->get_logger(), "      返回: %s", resp->message.c_str());
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "  ✗ %s 失败: %s", label.c_str(), resp->message.c_str());
    }
    return resp->success;
  }

  /// 泛型服务调用：同步等待并打印结果，返回响应供调用方打印额外字段
  template <typename Srv>
  typename Srv::Response::SharedPtr callService(const typename rclcpp::Client<Srv>::SharedPtr& cli,
                                                typename Srv::Request::SharedPtr req, const std::string& label)
  {
    if (!cli->service_is_ready())
    {
      RCLCPP_WARN(this->get_logger(), "  [%s] 服务未就绪，跳过", label.c_str());
      return nullptr;
    }
    auto future = cli->async_send_request(req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(10)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(), "  [%s] 调用超时/异常", label.c_str());
      return nullptr;
    }
    auto resp = future.get();
    if (resp->success)
    {
      RCLCPP_INFO(this->get_logger(), "  ✓ %s 成功", label.c_str());
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "  ✗ %s 失败: %s", label.c_str(), resp->message.c_str());
    }
    return resp;
  }

  /// 打印最近一次接收到的关节实际角度（rad → 度）
  void printJointStateDeg(const std::string& desc)
  {
    if (!last_joint_state_)
    {
      RCLCPP_WARN(this->get_logger(), "      %s：尚未收到 /joint_states", desc.c_str());
      return;
    }
    std::string s;
    for (size_t i = 0; i < last_joint_state_->position.size(); ++i)
    {
      if (i)
        s += ", ";
      s += std::to_string(last_joint_state_->position[i] * 180.0 / M_PI) + "°";
    }
    RCLCPP_INFO(this->get_logger(), "      %s：实际关节角[%zu] = [%s]", desc.c_str(),
                last_joint_state_->position.size(), s.c_str());
  }

  /// 执行一段 ServoJ 轨迹：从 q 起按 step 逐步累加发送 steps 次（默认 100Hz）
  /// @param q    当前关节角度起点（7 元素，单位 °）
  /// @param step 每步增量（7 元素，单位 °）
  /// @param steps 发送步数
  /// @param desc 动作描述
  void servoJTraj(std::vector<double> q, const std::vector<double>& step, int steps, const std::string& desc)
  {
    RCLCPP_INFO(this->get_logger(), "  ▶ %s（%d 步 × %dms ≈ %.1fs）", desc.c_str(), steps, period_ms_,
                steps * period_ms_ / 1000.0);
    for (int i = 0; i < steps; ++i)
    {
      for (size_t j = 0; j < q.size() && j < step.size(); ++j)
        q[j] += step[j];
      Float64Array msg;
      msg.data = q; // 7 元素，前 6 轴角度 + 第 7 轴补 0
      servoj_pos_pub_->publish(msg);
      rclcpp::sleep_for(std::chrono::milliseconds(period_ms_));
      // 收集 /joint_states 反馈（同时避免回调饥饿）
      rclcpp::spin_some(this->get_node_base_interface());
    }
    // 等待机械臂跟上目标后打印实际角度
    rclcpp::sleep_for(std::chrono::milliseconds(300));
    rclcpp::spin_some(this->get_node_base_interface());
    printJointStateDeg(desc);
  }
};

} // namespace tl_example

// ====================================================================
//  主流程实现（类外定义，main 直接调用）
// ====================================================================

void tl_example::ServoJDemo::run()
{
  // ── 1. 检查核心服务就绪 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 1. 检查核心服务就绪 ==========");
  if (!waitService("connect_arm", connect_cli_) || !waitService("clear_error", clear_error_cli_) ||
      !waitService("power_on", power_on_cli_) || !waitService("power_off", power_off_cli_) ||
      !waitService("set_current_mode", set_mode_cli_) || !waitService("open_servoj", open_servoj_cli_) ||
      !waitService("close_servoj", close_servoj_cli_) || !waitService("disconnect_arm", disconnect_cli_))
  {
    RCLCPP_ERROR(this->get_logger(), "tl_driver 核心服务未就绪，请先启动 tl_driver 节点");
    rclcpp::shutdown();
    return;
  }

  // ── 2. 连接机械臂（驱动自动双端口连接 6001+7000）──
  RCLCPP_INFO(this->get_logger(), "\n========== 2. 连接机械臂（自动双端口 6001+7000）==========");
  if (!callTrigger(connect_cli_, "connect_arm"))
  {
    RCLCPP_ERROR(this->get_logger(), "连接失败，退出测试");
    rclcpp::shutdown();
    return;
  }

  // ── 3. 清错 + 下电复位 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 3. 清错 + 下电复位 ==========");
  callTrigger(clear_error_cli_, "clear_error");
  rclcpp::sleep_for(std::chrono::milliseconds(300));
  callTrigger(power_off_cli_, "power_off");
  rclcpp::sleep_for(std::chrono::milliseconds(300));

  // ── 4. 切换示教模式 → 上电（含 set_servo_state(1)）──
  RCLCPP_INFO(this->get_logger(), "\n========== 4. 示教模式 + 上电 ==========");
  {
    auto req = std::make_shared<SetCurrentMode::Request>();
    req->mode = 0; // 示教模式
    callService<SetCurrentMode>(set_mode_cli_, req, "set_current_mode(0)");
    rclcpp::sleep_for(std::chrono::milliseconds(300));
  }
  // 说明：SDK 的 set_servo_state(1) 在 ROS2 中无独立服务，power_on 内部含此步
  if (!callTrigger(power_on_cli_, "power_on（含 set_servo_state(1)）"))
  {
    RCLCPP_ERROR(this->get_logger(), "上电失败，退出测试");
    callTrigger(disconnect_cli_, "disconnect_arm");
    rclcpp::shutdown();
    return;
  }
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // ── 5. 切换运行模式 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 5. 切换运行模式 ==========");
  {
    auto req = std::make_shared<SetCurrentMode::Request>();
    req->mode = 2; // 运行模式（0=示教，1=远程，2=运行）
    callService<SetCurrentMode>(set_mode_cli_, req, "set_current_mode(2)");
    rclcpp::sleep_for(std::chrono::seconds(2));
  }

  // ── 6. 打开 ServoJ 跟踪模式 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 6. 打开 ServoJ 跟踪模式 ==========");
  {
    auto req = std::make_shared<OpenServoJ::Request>();
    req->vmax.assign(kNJoints, vmax_);
    req->amax.assign(kNJoints, amax_);
    req->jmax.assign(kNJoints, jmax_);
    auto resp = callService<OpenServoJ>(open_servoj_cli_, req, "open_servoj");
    if (!resp || !resp->success)
    {
      RCLCPP_ERROR(this->get_logger(), "open_servoj 失败，退出测试");
      callTrigger(power_off_cli_, "power_off");
      callTrigger(disconnect_cli_, "disconnect_arm");
      rclcpp::shutdown();
      return;
    }
  }
  rclcpp::sleep_for(std::chrono::milliseconds(300));

  // ── 7. ServoJ 关节实时跟踪（100Hz，7 元素向量）──
  RCLCPP_INFO(this->get_logger(), "\n========== 7. ServoJ 关节实时跟踪 ==========");

  // 动作 1: J1 从 0° 平滑转到 30°
  servoJTraj({0, 0, 0, 0, 0, 0, 0}, {30.0 / 200, 0, 0, 0, 0, 0, 0}, 200, "动作1: J1 0°→ 30°");
  rclcpp::sleep_for(std::chrono::seconds(1));

  // 动作 2: J1 回到 0°，同时 J2 转到 20°
  servoJTraj({30, 0, 0, 0, 0, 0, 0}, {-30.0 / 200, 20.0 / 200, 0, 0, 0, 0, 0}, 200, "动作2: J1 回 0° + J2 0°→ 20°");
  rclcpp::sleep_for(std::chrono::seconds(1));

  // 动作 3: J2 回到 0°，同时 J3 转到 15°
  servoJTraj({0, 20, 0, 0, 0, 0, 0}, {0, -20.0 / 200, 15.0 / 200, 0, 0, 0, 0}, 200, "动作3: J2 回 0° + J3 0°→ 15°");
  rclcpp::sleep_for(std::chrono::seconds(1));

  // 动作 4: J3 回到 0°，全轴回零
  servoJTraj({0, 0, 15, 0, 0, 0, 0}, {0, 0, -15.0 / 150, 0, 0, 0, 0}, 150, "动作4: J3 回 0°，全轴回零");
  rclcpp::sleep_for(std::chrono::seconds(1));

  // ── 8. 关闭 ServoJ 跟踪模式 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 8. 关闭 ServoJ 跟踪模式 ==========");
  callTrigger(close_servoj_cli_, "close_servoj");

  RCLCPP_INFO(this->get_logger(), "\n  [信息] ServoJ 运动全部完成");

  // ── 9. 切回示教模式（自动下电）──
  RCLCPP_INFO(this->get_logger(), "\n========== 9. 切回示教模式 ==========");
  {
    auto req = std::make_shared<SetCurrentMode::Request>();
    req->mode = 0; // 示教模式
    callService<SetCurrentMode>(set_mode_cli_, req, "set_current_mode(0)");
    rclcpp::sleep_for(std::chrono::milliseconds(300));
  }

  // ── 10. 断开连接 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 10. 断开连接 ==========");
  callTrigger(disconnect_cli_, "disconnect_arm");

  RCLCPP_INFO(this->get_logger(), "\n============================================================");
  RCLCPP_INFO(this->get_logger(), "  ServoJ 关节实时跟踪测试完成，节点退出");
  RCLCPP_INFO(this->get_logger(), "============================================================");
  rclcpp::shutdown();
}

// ====================================================================
//  main
// ====================================================================

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tl_example::ServoJDemo>();
  node->run(); // run() 内部已处理 rclcpp::shutdown()
  return 0;
}
