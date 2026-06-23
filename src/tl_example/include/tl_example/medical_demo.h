#ifndef TL_EXAMPLE__MEDICAL_DEMO_H_
#define TL_EXAMPLE__MEDICAL_DEMO_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "tl_ros2_interface/msg/arm_status.hpp"
#include "tl_ros2_interface/msg/move_command.hpp"
#include "tl_ros2_interface/srv/set_current_mode.hpp"
#include "tl_ros2_interface/srv/set_speed.hpp"

namespace tl_example {

/**
 * @brief 队列 MoveL 运动演示节点
 *
 * 演示场景：通过状态机驱动，将 MoveL 指令逐条下发到 /tl_driver/moveL，
 * 模拟医学检验科机械臂的典型操作流程：
 *   安全位 → 样本架 → 取样 → 试剂架 → 分装加样 → 检测仪器进样口 → 放样 → 安全位
 *
 * 状态机：INIT → CONNECTING → SET_MODE → POWER_ON → SET_SPEED → MOVING →
 *          POWER_OFF → DONE
 * 连接后切换到示教模式（mode=0），然后显式上电；结束时显式下电，不切换模式。
 */
class MedicalDemo : public rclcpp::Node {
public:
  explicit MedicalDemo(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  // ── 类型定义 ──
  using ArmStatus = tl_ros2_interface::msg::ArmStatus;
  using MoveCommand = tl_ros2_interface::msg::MoveCommand;
  using SetCurrentMode = tl_ros2_interface::srv::SetCurrentMode;
  using SetSpeed = tl_ros2_interface::srv::SetSpeed;
  using Trigger = std_srvs::srv::Trigger;

  struct SequenceStep {
    std::string pos_key;     // 点位名称
    std::string description; // 动作描述
    double velocity;         // 速度百分比
  };

private:
  // ── 辅助方法 ──
  MoveCommand buildMoveL(const std::string &pos_key, double velocity) const;
  void ensureClients();
  bool waitService(rclcpp::ClientBase::SharedPtr client,
                   const std::string &name, double timeout_s);

  // ── 状态机 ──
  void stateMachine();

  void startConnect();
  void onConnectDone(rclcpp::Client<Trigger>::SharedFuture future);

  void startSetMode();
  void onSetModeDone(rclcpp::Client<SetCurrentMode>::SharedFuture future);

  void startPowerOn();
  void onPowerOnDone(rclcpp::Client<Trigger>::SharedFuture future);

  void startSetSpeed();
  void onSetSpeedDone(rclcpp::Client<SetSpeed>::SharedFuture future);

  void moveNextStep();

  void onArmStatus(const ArmStatus::SharedPtr msg);

  void startPowerOff();
  void onPowerOffDone(rclcpp::Client<Trigger>::SharedFuture future);

  // ── ROS2 通信 ──
  rclcpp::Publisher<MoveCommand>::SharedPtr movel_pub_;
  rclcpp::Subscription<ArmStatus>::SharedPtr arm_status_sub_;
  rclcpp::Client<Trigger>::SharedPtr connect_cli_;
  rclcpp::Client<SetCurrentMode>::SharedPtr set_mode_cli_;
  rclcpp::Client<Trigger>::SharedPtr power_on_cli_;
  rclcpp::Client<Trigger>::SharedPtr power_off_cli_;
  rclcpp::Client<SetSpeed>::SharedPtr set_speed_cli_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  // ── 状态机变量 ──
  enum class State {
    INIT,
    CONNECTING,
    SET_MODE,
    POWER_ON,
    SET_SPEED,
    MOVING,
    POWER_OFF,
    DONE
  };
  State state_ = State::INIT;
  size_t seq_index_ = 0;

  // 运动等待（通过 /arm_status 判断运动完成）
  std::string arm_run_state_ = "STOP";

  enum class MovePhase { IDLE, WAIT_RUNNING, WAIT_DONE };
  MovePhase move_phase_ = MovePhase::IDLE;

  double demo_speed_ = 30.0;
  double safety_timeout_s_ = 60.0; // 安全超时（秒）
  double move_start_time_s_ = 0.0;
};

} // namespace tl_example

#endif // TL_EXAMPLE__MEDICAL_DEMO_H_
