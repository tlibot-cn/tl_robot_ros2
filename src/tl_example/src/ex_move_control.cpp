/**
 * @file ex_move_control.cpp
 * @brief 运动控制测试 — MoveJ / MoveL
 *
 * 对应 tl_sdk/examples/py/ex_move_control.py 的 ROS2 实现。
 * 通过话题向 tl_driver 下发 MoveJ（关节空间）与 MoveL（笛卡尔直线）运动，
 * 完整演示「连接 → 示教模式 → 上电 → 运动 → 下电 → 断开」流程。
 *
 * 与 Python 示例的主要差异：
 *   - 不直接管理 TCP Socket，通过 ROS2 服务/话题与 tl_driver 交互
 *   - 运动完成判断：订阅 /arm_status 话题（RUNNING → STOP），替代 SDK 轮询
 *
 * ⚠ 安全警告：本示例会真实控制机械臂运动！
 *   运行前请确保工作区安全、速度合理，并随时准备急停。
 *
 * @usage
 *   ros2 run tl_example ex_move_control
 *
 * @see medical_demo.cpp — 状态机驱动的多点位 MoveL 演示
 * @see ex_queue_test.cpp — 队列运动（Queue Motion）测试（14 维 target_pos_value 布局参考）
 */

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

namespace tl_example
{

// 类型别名 — 简化代码
using Trigger = std_srvs::srv::Trigger;
using ArmStatus = tl_ros2_interface::msg::ArmStatus;
using MoveCommand = tl_ros2_interface::msg::MoveCommand;
using SetCurrentMode = tl_ros2_interface::srv::SetCurrentMode;
using SetSpeed = tl_ros2_interface::srv::SetSpeed;

/**
 * @brief 运动控制测试节点
 *
 * 流程（同步顺序执行）：
 *   connect → set_current_mode(示教) → power_on → set_speed →
 *   MoveJ（关节坐标）→ 等待完成 → MoveL（直角坐标）→ 等待完成 →
 *   power_off → disconnect
 *
 * 运动完成通过订阅 /arm_status 判断：状态从 STOP → RUNNING（启动）再回到 STOP（完成）。
 */
class MoveControlDemo : public rclcpp::Node
{
public:
  explicit MoveControlDemo(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("ex_move_control", options)
  {
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "  运动控制测试节点启动 (MoveJ / MoveL)");
    RCLCPP_INFO(this->get_logger(), "  ⚠ 本示例会真实控制机械臂运动，请注意安全");
    RCLCPP_INFO(this->get_logger(), "============================================================");

    // ── 服务客户端 ──
    connect_cli_      = this->create_client<Trigger>("/tl_driver/connect_arm");
    disconnect_cli_   = this->create_client<Trigger>("/tl_driver/disconnect_arm");
    power_on_cli_     = this->create_client<Trigger>("/tl_driver/power_on");
    power_off_cli_    = this->create_client<Trigger>("/tl_driver/power_off");
    set_mode_cli_     = this->create_client<SetCurrentMode>("/tl_driver/set_current_mode");
    set_speed_cli_    = this->create_client<SetSpeed>("/tl_driver/set_speed");

    // ── 运动指令发布 ──
    movej_pub_ = this->create_publisher<MoveCommand>("/tl_driver/moveJ", 10);
    movel_pub_ = this->create_publisher<MoveCommand>("/tl_driver/moveL", 10);

    // ── 运行状态订阅（用于判断运动完成）──
    arm_status_sub_ = this->create_subscription<ArmStatus>(
        "/arm_status", 10, std::bind(&MoveControlDemo::onArmStatus, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "  客户端/发布器/订阅创建完成");
  }

  // ── 主流程（main 直接调用）──
  void run();

private:
  // ── ROS2 通信 ──
  rclcpp::Client<Trigger>::SharedPtr connect_cli_;
  rclcpp::Client<Trigger>::SharedPtr disconnect_cli_;
  rclcpp::Client<Trigger>::SharedPtr power_on_cli_;
  rclcpp::Client<Trigger>::SharedPtr power_off_cli_;
  rclcpp::Client<SetCurrentMode>::SharedPtr set_mode_cli_;
  rclcpp::Client<SetSpeed>::SharedPtr set_speed_cli_;
  rclcpp::Publisher<MoveCommand>::SharedPtr movej_pub_;
  rclcpp::Publisher<MoveCommand>::SharedPtr movel_pub_;
  rclcpp::Subscription<ArmStatus>::SharedPtr arm_status_sub_;

  // 最近一次接收到的机械臂运行状态
  std::string arm_run_state_ = "unknown";

  // ── 参数 ──
  double global_speed_ = 30.0;   // 全局速度（%）
  double motion_speed_ = 20.0;   // 指令速度（1~100）
  double acc_dec_ = 10.0;        // 加减速（1~100）
  double wait_timeout_s_ = 60.0; // 等待运动完成超时（秒）

  // ── 话题回调 ──
  void onArmStatus(const ArmStatus::SharedPtr msg) { arm_run_state_ = msg->run_state; }

  // ── 辅助方法 ──

  /// 等待单个服务就绪
  bool waitService(const std::string& name, rclcpp::ClientBase::SharedPtr cli, double timeout_s = 3.0)
  {
    if (!cli->wait_for_service(std::chrono::duration<double>(timeout_s))) {
      RCLCPP_ERROR(this->get_logger(), "  服务 %s 未就绪（%.0fs 超时）", name.c_str(), timeout_s);
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "  服务 %s 已就绪", name.c_str());
    return true;
  }

  /// 调用 Trigger 服务
  bool callTrigger(rclcpp::Client<Trigger>::SharedPtr cli, const std::string& label)
  {
    if (!cli->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "  [%s] 服务未就绪，跳过", label.c_str());
      return false;
    }
    auto req = std::make_shared<Trigger::Request>();
    auto future = cli->async_send_request(req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future,
                                           std::chrono::seconds(10)) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "  [%s] 调用超时/异常", label.c_str());
      return false;
    }
    const auto& resp = future.get();
    if (resp->success) {
      RCLCPP_INFO(this->get_logger(), "  ✓ %s 成功", label.c_str());
      if (!resp->message.empty()) RCLCPP_INFO(this->get_logger(), "      返回: %s", resp->message.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "  ✗ %s 失败: %s", label.c_str(), resp->message.c_str());
    }
    return resp->success;
  }

  /// 泛型服务调用：同步等待并打印结果，返回响应供调用方打印额外字段
  template <typename Srv>
  typename Srv::Response::SharedPtr callService(const typename rclcpp::Client<Srv>::SharedPtr& cli,
                                                typename Srv::Request::SharedPtr req,
                                                const std::string& label)
  {
    if (!cli->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "  [%s] 服务未就绪，跳过", label.c_str());
      return nullptr;
    }
    auto future = cli->async_send_request(req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future,
                                           std::chrono::seconds(10)) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "  [%s] 调用超时/异常", label.c_str());
      return nullptr;
    }
    auto resp = future.get();
    if (resp->success) {
      RCLCPP_INFO(this->get_logger(), "  ✓ %s 成功", label.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "  ✗ %s 失败: %s", label.c_str(), resp->message.c_str());
    }
    return resp;
  }

  /// 短暂 spin 直到条件满足或超时（用于等待话题消息 / 运动状态变化）
  bool spinWaitFor(const std::function<bool()>& cond, double timeout_s)
  {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (rclcpp::ok() && !cond() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(this->get_node_base_interface());
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    return cond();
  }

  /// 构造 MoveCommand 公共字段
  /// 注意：target_pos_value 必须为 14 维（官方 make_movej_cmd / make_movel_cmd 布局）：
  ///   - coord=0（关节）：index 0~6 = J1~J7 关节角（度），后 7 位置 0
  ///   - coord=1（直角）：index 0~5 = [X,Y,Z,RX,RY,RZ]（mm/rad），后 8 位置 0
  /// 这里统一在开头按序写入 pos 并补齐到 14 维，调用方无需关心维度。
  MoveCommand makeMoveCmd(int coord, const std::vector<double>& pos) const
  {
    MoveCommand msg;
    msg.target_pos_value.assign(14, 0.0);
    for (size_t i = 0; i < pos.size() && i < 14; ++i) {
      msg.target_pos_value[i] = pos[i];
    }
    msg.target_pos_name = "";
    msg.target_pos_type = 0;   // 自定义数组
    msg.coord = coord;         // 0=关节 1=直角
    msg.velocity = motion_speed_;
    msg.velocity_sync = 0.0;
    msg.acc = acc_dec_;
    msg.dec = acc_dec_;
    msg.pl = 0;                // 精确到达
    msg.time = 0;
    msg.tool_num = 0;
    msg.user_num = 0;
    msg.posidtype = 0;
    msg.configuration = 0;
    msg.spin = 0;
    msg.para_sync = false;
    return msg;
  }

  /// 等待运动完成：先等 RUNNING（启动），再等 STOP（完成）
  bool waitMotionDone(const std::string& label)
  {
    RCLCPP_INFO(this->get_logger(), "  等待 %s 启动...", label.c_str());
    if (!spinWaitFor([this]() { return arm_run_state_ == "RUNNING"; }, 10.0)) {
      RCLCPP_WARN(this->get_logger(), "  未检测到运动启动（run_state=%s），继续等待完成", arm_run_state_.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "  等待 %s 完成...", label.c_str());
    if (!spinWaitFor([this]() { return arm_run_state_ == "STOP"; }, wait_timeout_s_)) {
      RCLCPP_WARN(this->get_logger(), "  %s 等待超时（%.0fs），run_state=%s",
                  label.c_str(), wait_timeout_s_, arm_run_state_.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "  ✓ %s 完成", label.c_str());
    return true;
  }
};

} // namespace tl_example

// ====================================================================
//  主流程实现（类外定义，main 直接调用）
// ====================================================================

void tl_example::MoveControlDemo::run()
{
  // ── 1. 检查核心服务就绪 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 1. 检查核心服务就绪 ==========");
  if (!waitService("connect_arm", connect_cli_) ||
      !waitService("set_current_mode", set_mode_cli_) ||
      !waitService("power_on", power_on_cli_) ||
      !waitService("disconnect_arm", disconnect_cli_)) {
    RCLCPP_ERROR(this->get_logger(), "tl_driver 核心服务未就绪，请先启动 tl_driver 节点");
    rclcpp::shutdown();
    return;
  }

  // ── 2. 连接机械臂 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 2. 连接机械臂 ==========");
  if (!callTrigger(connect_cli_, "connect_arm")) {
    RCLCPP_ERROR(this->get_logger(), "连接失败，退出测试");
    rclcpp::shutdown();
    return;
  }

  // ── 3. 切换示教模式 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 3. 切换示教模式 ==========");
  {
    auto req = std::make_shared<SetCurrentMode::Request>();
    req->mode = 0;  // 示教模式（0=示教，1=远程，2=运行）
    callService<SetCurrentMode>(set_mode_cli_, req, "set_current_mode(0)");
  }

  // ── 4. 上电 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 4. 机械臂上电 ==========");
  if (!callTrigger(power_on_cli_, "power_on")) {
    RCLCPP_ERROR(this->get_logger(), "上电失败，退出测试");
    callTrigger(disconnect_cli_, "disconnect_arm（清理）");
    rclcpp::shutdown();
    return;
  }

  // ── 5. 设置全局速度 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 5. 设置全局速度 ==========");
  {
    auto req = std::make_shared<SetSpeed::Request>();
    req->speed = global_speed_;
    callService<SetSpeed>(set_speed_cli_, req, "set_speed");
  }

  // ── 6. MoveJ 关节空间运动 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 6. MoveJ 运动控制（关节坐标，单位：度）==========");
  {
    // 与 Python 示例一致：J1=20°, J2=10°, J3=-10°，其余 0
    MoveCommand msg = makeMoveCmd(0, {20.0, 10.0, -10.0, 0.0, 0.0, 0.0, 0.0});
    RCLCPP_INFO(this->get_logger(), "  目标关节角: [20, 10, -10, 0, 0, 0]°，速度 %.0f", motion_speed_);
    movej_pub_->publish(msg);
    if (!waitMotionDone("MoveJ")) {
      RCLCPP_WARN(this->get_logger(), "MoveJ 未确认完成，继续执行 MoveL");
    }
  }

  // ── 7. MoveL 笛卡尔直线运动 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 7. MoveL 运动控制（直角坐标，mm / rad）==========");
  {
    // 与 Python 示例一致：位置(280, 90, 270)mm，姿态(3.14, 0, 0)rad
    MoveCommand msg = makeMoveCmd(1, {280.0, 90.0, 270.0, 3.14, 0.0, 0.0});
    RCLCPP_INFO(this->get_logger(), "  目标位姿: pos(280, 90, 270)mm rpy(3.14, 0, 0)rad，速度 %.0f", motion_speed_);
    movel_pub_->publish(msg);
    if (!waitMotionDone("MoveL")) {
      RCLCPP_WARN(this->get_logger(), "MoveL 未确认完成");
    }
  }

  // ── 8. 下电 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 8. 机械臂下电 ==========");
  callTrigger(power_off_cli_, "power_off");

  // ── 9. 断开连接 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 9. 断开连接 ==========");
  callTrigger(disconnect_cli_, "disconnect_arm");

  RCLCPP_INFO(this->get_logger(), "\n============================================================");
  RCLCPP_INFO(this->get_logger(), "  运动控制测试完成，节点退出");
  RCLCPP_INFO(this->get_logger(), "============================================================");
  rclcpp::shutdown();
}

// ====================================================================
//  main
// ====================================================================

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tl_example::MoveControlDemo>();
  node->run();  // run() 内部已处理 rclcpp::shutdown()
  return 0;
}
