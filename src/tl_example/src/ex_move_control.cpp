/**
 * @file ex_move_control.cpp
 * @brief 运动控制测试 — MoveJ / MoveL
 *
 * 对应 tl_sdk/examples/py/ex_move_control.py 的 ROS2 实现。
 * 通过 action 向 tl_driver 下发 MoveJ（关节空间）与 MoveL（笛卡尔直线）运动，
 * 完整演示「连接 → 示教模式 → 上电 → 运动 → 下电 → 断开」流程。
 *
 * 与 Python 示例的主要差异：
 *   - 不直接管理 TCP Socket，通过 ROS2 服务/action 与 tl_driver 交互
 *   - 运动完成判断：等待 action result（含成功/失败原因），替代 SDK 轮询
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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "tl_ros2_interface/action/move.hpp"
#include "tl_ros2_interface/msg/move_command.hpp"
#include "tl_ros2_interface/srv/set_current_mode.hpp"
#include "tl_ros2_interface/srv/set_speed.hpp"

namespace tl_example
{

// 类型别名 — 简化代码
using Trigger = std_srvs::srv::Trigger;
using MoveCommand = tl_ros2_interface::msg::MoveCommand;
using Move = tl_ros2_interface::action::Move;

namespace
{
constexpr double kDegToRad = M_PI / 180.0;
// goal 超时比客户端等待短，确保 action 的 abort/result 先于本地超时到达。
constexpr double kActionResultMarginSec = 5.0;
} // namespace
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
 * 运动完成以 action result 为准（成功/失败原因），不再依赖 /arm_status。
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
    power_on_cli_ = this->create_client<Trigger>("/tl_driver/power_on");
    power_off_cli_ = this->create_client<Trigger>("/tl_driver/power_off");
    set_mode_cli_ = this->create_client<SetCurrentMode>("/tl_driver/set_current_mode");
    set_speed_cli_ = this->create_client<SetSpeed>("/tl_driver/set_speed");

    // ── 运动 action 客户端 ──
    move_joint_cli_ = rclcpp_action::create_client<Move>(this, "/tl_driver/move_joint");
    move_linear_cli_ = rclcpp_action::create_client<Move>(this, "/tl_driver/move_linear");

    RCLCPP_INFO(this->get_logger(), "  服务客户端 / action 客户端创建完成");
  }

  // ── 主流程（main 直接调用）──
  bool run();

private:
  // ── ROS2 通信 ──
  rclcpp::Client<Trigger>::SharedPtr power_on_cli_;
  rclcpp::Client<Trigger>::SharedPtr power_off_cli_;
  rclcpp::Client<SetCurrentMode>::SharedPtr set_mode_cli_;
  rclcpp::Client<SetSpeed>::SharedPtr set_speed_cli_;
  rclcpp_action::Client<Move>::SharedPtr move_joint_cli_;
  rclcpp_action::Client<Move>::SharedPtr move_linear_cli_;

  // 最近一次 action goal 的执行状态
  bool goal_done_ = false;
  bool goal_success_ = false;

  // ── 参数 ──
  double global_speed_ = 30.0;   // 全局速度（%）
  double motion_speed_ = 20.0;   // MoveJ 速度（%，1~100）
  double linear_speed_ = 0.02;   // MoveL 速度（m/s）
  double acc_dec_ = 10.0;        // 加减速（1~100）
  double wait_timeout_s_ = 60.0; // 等待运动完成超时（秒）

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

  /// 短暂 spin 直到条件满足或超时（用于等待 action result 回调）
  bool spinWaitFor(const std::function<bool()>& cond, double timeout_s)
  {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (rclcpp::ok() && !cond() && std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(this->get_node_base_interface());
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    return cond();
  }

  /// 构造 MoveCommand 公共字段
  /// 注意：target_pos_value 必须为 14 维（官方 make_movej_cmd / make_movel_cmd 布局）：
  ///   - coord=0（关节）：index 0~6 = J1~J7 关节角（rad），后 7 位置 0
  ///   - coord=1（直角）：index 0~5 = [X,Y,Z,RX,RY,RZ]（m/rad），后 8 位置 0
  /// 这里统一在开头按序写入 pos 并补齐到 14 维，调用方无需关心维度。
  MoveCommand makeMoveCmd(int coord, const std::vector<double>& pos) const
  {
    MoveCommand msg;
    msg.target_pos_value.fill(0.0);
    for (size_t i = 0; i < pos.size() && i < msg.target_pos_value.size(); ++i)
    {
      msg.target_pos_value[i] = pos[i];
    }
    msg.coord = coord; // 0=关节 1=直角
    msg.velocity = (coord == 0) ? motion_speed_ : linear_speed_;
    msg.velocity_sync = 0.0;
    msg.acc = acc_dec_;
    msg.dec = acc_dec_;
    msg.pl = 0; // 精确到达
    msg.tool_num = 0;
    msg.user_num = 0;
    msg.para_sync = false;
    return msg;
  }

  /// 发送运动 goal；返回 false 表示 action server 未就绪
  bool sendMoveGoal(const rclcpp_action::Client<Move>::SharedPtr& cli, const MoveCommand& command,
                    const std::string& label)
  {
    if (!cli->action_server_is_ready())
    {
      RCLCPP_ERROR(this->get_logger(), "  %s action server 未就绪", label.c_str());
      return false;
    }

    goal_done_ = false;
    goal_success_ = false;

    Move::Goal goal;
    goal.command = command;
    goal.timeout_sec = std::max(1.0, wait_timeout_s_ - kActionResultMarginSec);

    rclcpp_action::Client<Move>::SendGoalOptions options;
    options.goal_response_callback = [this, label](const rclcpp_action::Client<Move>::GoalHandle::SharedPtr& handle)
    {
      if (handle)
      {
        RCLCPP_INFO(this->get_logger(), "  %s goal 已接受", label.c_str());
      }
      else
      {
        goal_done_ = true;
        RCLCPP_ERROR(this->get_logger(), "  %s goal 被拒绝", label.c_str());
      }
    };
    options.result_callback = [this, label](const rclcpp_action::Client<Move>::WrappedResult& result)
    {
      goal_success_ = result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result && result.result->success;
      goal_done_ = true;
      if (goal_success_)
      {
        RCLCPP_INFO(this->get_logger(), "  ✓ %s action 完成", label.c_str());
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "  %s action 失败: %s", label.c_str(),
                    result.result ? result.result->message.c_str() : "no result");
      }
    };
    cli->async_send_goal(goal, options);
    return true;
  }

  /// 等待 action result；返回 false 表示 goal 被拒绝、执行失败或等待超时
  bool waitActionResult(const std::string& label)
  {
    if (!spinWaitFor(
            [this]()
            {
              return goal_done_;
            },
            wait_timeout_s_))
    {
      RCLCPP_ERROR(this->get_logger(), "  %s 等待 action result 超时（%.0fs）", label.c_str(), wait_timeout_s_);
      return false;
    }
    return goal_success_;
  }
};

} // namespace tl_example

// ====================================================================
//  主流程实现（类外定义，main 直接调用）
// ====================================================================

bool tl_example::MoveControlDemo::run()
{
  // ── 1. 检查核心服务就绪 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 1. 检查核心服务就绪 ==========");
  if (!waitService("set_current_mode", set_mode_cli_) || !waitService("power_on", power_on_cli_))
  {
    RCLCPP_ERROR(this->get_logger(), "tl_driver 核心服务未就绪，请先启动 tl_driver 节点");
    rclcpp::shutdown();
    return false;
  }
  if (!move_joint_cli_->wait_for_action_server(std::chrono::seconds(3)) ||
      !move_linear_cli_->wait_for_action_server(std::chrono::seconds(3)))
  {
    RCLCPP_ERROR(this->get_logger(), "tl_driver 运动 action 未就绪，请先启动 tl_driver 节点");
    rclcpp::shutdown();
    return false;
  }

  // ── 2. 切换示教模式 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 2. 切换示教模式 ==========");
  {
    auto req = std::make_shared<SetCurrentMode::Request>();
    req->mode = 0; // 示教模式（0=示教，1=远程，2=运行）
    callService<SetCurrentMode>(set_mode_cli_, req, "set_current_mode(0)");
  }

  // ── 3. 上电 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 3. 机械臂上电 ==========");
  if (!callTrigger(power_on_cli_, "power_on"))
  {
    RCLCPP_ERROR(this->get_logger(), "上电失败，退出测试");
    rclcpp::shutdown();
    return false;
  }

  // ── 4. 设置全局速度 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 4. 设置全局速度 ==========");
  {
    auto req = std::make_shared<SetSpeed::Request>();
    req->speed = global_speed_;
    callService<SetSpeed>(set_speed_cli_, req, "set_speed");
  }

  // ── 5. MoveJ 关节空间运动 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 5. MoveJ 运动控制（关节坐标，单位：rad）==========");
  bool move_ok = false;
  {
    // 与 Python 示例一致：J1=20°, J2=10°, J3=-10°，其余 0
    MoveCommand msg = makeMoveCmd(0, {20.0 * kDegToRad, 10.0 * kDegToRad, -10.0 * kDegToRad, 0.0, 0.0, 0.0, 0.0});
    RCLCPP_INFO(this->get_logger(), "  目标关节角: [20, 10, -10, 0, 0, 0]°，速度 %.0f%%", motion_speed_);
    move_ok = sendMoveGoal(move_joint_cli_, msg, "MoveJ") && waitActionResult("MoveJ");
  }

  // ── 6. MoveL 笛卡尔直线运动（MoveJ 成功后才执行）──
  if (move_ok)
  {
    RCLCPP_INFO(this->get_logger(), "\n========== 6. MoveL 运动控制（直角坐标，m / rad）==========");
    // 与 Python 示例一致：位置(0.28, 0.09, 0.27)m，姿态(3.14, 0, 0)rad
    MoveCommand msg = makeMoveCmd(1, {0.28, 0.09, 0.27, 3.14, 0.0, 0.0});
    RCLCPP_INFO(this->get_logger(), "  目标位姿: pos(0.28, 0.09, 0.27)m rpy(3.14, 0, 0)rad，速度 %.3fm/s",
                linear_speed_);
    move_ok = sendMoveGoal(move_linear_cli_, msg, "MoveL") && waitActionResult("MoveL");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "  MoveJ 未成功，跳过 MoveL");
  }

  // ── 7. 下电 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 7. 机械臂下电 ==========");
  callTrigger(power_off_cli_, "power_off");

  RCLCPP_INFO(this->get_logger(), "\n============================================================");
  if (move_ok)
  {
    RCLCPP_INFO(this->get_logger(), "  运动控制测试成功，节点退出");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "  运动控制测试失败，节点退出");
  }
  RCLCPP_INFO(this->get_logger(), "============================================================");
  rclcpp::shutdown();
  return move_ok;
}

// ====================================================================
//  main
// ====================================================================

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tl_example::MoveControlDemo>();
  return node->run() ? 0 : 1; // run() 内部已处理 rclcpp::shutdown()
}
