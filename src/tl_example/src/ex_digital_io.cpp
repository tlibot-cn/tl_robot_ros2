/**
 * @file ex_digital_io.cpp
 * @brief 数字 IO 测试
 *
 * 对应 tl_sdk/examples/cpp/ex_digital_io.cpp 的 ROS2 实现。
 * 演示数字输入/输出（DI/DO）的查询与设置：
 *
 *   connect → set_current_mode(远程) →
 *   查询当前 IO → 设置 DO1=0 → 查询验证 → 设置 DO1=1 → 查询验证 →
 *   运行状态（/arm_status 话题）→ 恢复示教模式 → disconnect
 *
 * ⚠ 跳过项：SDK 中的 set_remote_function（设置远程 IO 功能）在 ROS2 驱动中
 *   无对应接口，程序中跳过并列出。
 *
 * ⚠ 注意：set_digital_output 会真实改变数字输出端口电平（默认 DO1），
 *   请确认该端口所接外部设备安全后再运行。
 *
 * @usage
 *   ros2 run tl_example ex_digital_io
 *   # 自定义测试端口
 *   ros2 run tl_example ex_digital_io --ros-args -p io_port:=1
 *
 * @see ex_info_query.cpp — 信息查询示例（§4 信息查询接口）
 */

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "tl_ros2_interface/msg/arm_status.hpp"
#include "tl_ros2_interface/srv/get_digital_input_output.hpp"
#include "tl_ros2_interface/srv/set_current_mode.hpp"
#include "tl_ros2_interface/srv/set_digital_output.hpp"

namespace tl_example
{

// 类型别名 — 简化代码
using Trigger = std_srvs::srv::Trigger;
using ArmStatus = tl_ros2_interface::msg::ArmStatus;
using GetDigitalInputOutput = tl_ros2_interface::srv::GetDigitalInputOutput;
using SetCurrentMode = tl_ros2_interface::srv::SetCurrentMode;
using SetDigitalOutput = tl_ros2_interface::srv::SetDigitalOutput;

/**
 * @brief 数字 IO 测试节点
 *
 * 流程（同步顺序执行）：
 *   connect → set_current_mode(1 远程) →
 *   查询当前 IO → set_digital_output(DO1=0/1) → 查询验证 →
 *   运行状态（/arm_status）→ set_current_mode(0 恢复) → disconnect
 */
class DigitalIoDemo : public rclcpp::Node
{
public:
  explicit DigitalIoDemo(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("ex_digital_io", options)
  {
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "  数字 IO 测试节点启动");
    RCLCPP_INFO(this->get_logger(), "  查询 DI/DO 状态，并测试数字输出设置");
    RCLCPP_INFO(this->get_logger(), "============================================================");

    // ── 参数 ──
    this->declare_parameter("io_port", 1); // 测试的数字输出端口号
    io_port_ = this->get_parameter("io_port").as_int();

    // ── 服务客户端 ──
    connect_cli_ = this->create_client<Trigger>("/tl_driver/connect_arm");
    disconnect_cli_ = this->create_client<Trigger>("/tl_driver/disconnect_arm");
    set_mode_cli_ = this->create_client<SetCurrentMode>("/tl_driver/set_current_mode");
    get_dio_cli_ = this->create_client<GetDigitalInputOutput>("/tl_driver/get_digital_input_output");
    set_dio_cli_ = this->create_client<SetDigitalOutput>("/tl_driver/set_digital_output");

    // ── 运行状态订阅（替代 SDK 的 get_robot_running_state）──
    arm_status_sub_ = this->create_subscription<ArmStatus>(
        "/arm_status", 10, std::bind(&DigitalIoDemo::onArmStatus, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "  客户端/订阅创建完成 (测试端口 DO%d)", io_port_);
  }

  // ── 主流程（main 直接调用）──
  void run();

private:
  // ── ROS2 通信 ──
  rclcpp::Client<Trigger>::SharedPtr connect_cli_;
  rclcpp::Client<Trigger>::SharedPtr disconnect_cli_;
  rclcpp::Client<SetCurrentMode>::SharedPtr set_mode_cli_;
  rclcpp::Client<GetDigitalInputOutput>::SharedPtr get_dio_cli_;
  rclcpp::Client<SetDigitalOutput>::SharedPtr set_dio_cli_;
  rclcpp::Subscription<ArmStatus>::SharedPtr arm_status_sub_;

  // 最近一次接收到的机械臂运行状态（/arm_status）
  std::string arm_run_state_ = "unknown";

  // ── 参数 ──
  int io_port_ = 1; // 测试的数字输出端口号

  // ── 话题回调 ──
  void onArmStatus(const ArmStatus::SharedPtr msg)
  {
    arm_run_state_ = msg->run_state;
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

  /// 打印 IO 数组（前 max 个），并在端口号标注测试端口
  void printIoArray(const std::vector<int32_t>& v, const std::string& name, size_t max = 16)
  {
    std::string s;
    size_t n = std::min(v.size(), max);
    for (size_t i = 0; i < n; ++i)
    {
      if (i)
        s += ", ";
      s += std::to_string(v[i]);
      if (static_cast<int>(i) == io_port_ - 1 && name == "DO")
      {
        s += "(DO" + std::to_string(io_port_) + ")"; // 标记测试端口
      }
    }
    if (v.size() > max)
      s += ", ...";
    RCLCPP_INFO(this->get_logger(), "      %s[%zu] = [%s]", name.c_str(), v.size(), s.c_str());
  }
};

} // namespace tl_example

// ====================================================================
//  主流程实现（类外定义，main 直接调用）
// ====================================================================

void tl_example::DigitalIoDemo::run()
{
  // ── 1. 检查核心服务就绪 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 1. 检查核心服务就绪 ==========");
  if (!waitService("connect_arm", connect_cli_) || !waitService("set_current_mode", set_mode_cli_) ||
      !waitService("get_digital_input_output", get_dio_cli_) || !waitService("set_digital_output", set_dio_cli_) ||
      !waitService("disconnect_arm", disconnect_cli_))
  {
    RCLCPP_ERROR(this->get_logger(), "tl_driver 核心服务未就绪，请先启动 tl_driver 节点");
    rclcpp::shutdown();
    return;
  }

  // ── 2. 连接机械臂 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 2. 连接机械臂 ==========");
  if (!callTrigger(connect_cli_, "connect_arm"))
  {
    RCLCPP_ERROR(this->get_logger(), "连接失败，退出测试");
    rclcpp::shutdown();
    return;
  }

  // ── 3. 设置远程模式 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 3. 设置机器人模式（远程） ==========");
  {
    auto req = std::make_shared<SetCurrentMode::Request>();
    req->mode = 1; // 远程模式（0=示教，1=远程，2=运行）
    callService<SetCurrentMode>(set_mode_cli_, req, "set_current_mode(1)");
  }

  // ── 4. 查询当前 IO 状态 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 4. 查询当前 IO 状态 ==========");
  {
    auto req = std::make_shared<GetDigitalInputOutput::Request>();
    auto resp = callService<GetDigitalInputOutput>(get_dio_cli_, req, "get_digital_input_output");
    if (resp && resp->success)
    {
      printIoArray(resp->input, "DI");
      printIoArray(resp->output, "DO");
    }
  }

  // ── 5. 设置数字输出 DO1=0 → 查询验证 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 5. 设置数字输出（DO%d=0）==========", io_port_);
  {
    auto req = std::make_shared<SetDigitalOutput::Request>();
    req->port = io_port_;
    req->value = 0;
    callService<SetDigitalOutput>(set_dio_cli_, req, "set_digital_output");
  }
  {
    auto req = std::make_shared<GetDigitalInputOutput::Request>();
    auto resp = callService<GetDigitalInputOutput>(get_dio_cli_, req, "get_digital_input_output（验证）");
    if (resp && resp->success)
      printIoArray(resp->output, "DO");
  }

  // ── 6. 设置数字输出 DO1=1 → 查询验证 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 6. 设置数字输出（DO%d=1）==========", io_port_);
  {
    auto req = std::make_shared<SetDigitalOutput::Request>();
    req->port = io_port_;
    req->value = 1;
    callService<SetDigitalOutput>(set_dio_cli_, req, "set_digital_output");
  }
  {
    auto req = std::make_shared<GetDigitalInputOutput::Request>();
    auto resp = callService<GetDigitalInputOutput>(get_dio_cli_, req, "get_digital_input_output（验证）");
    if (resp && resp->success)
      printIoArray(resp->output, "DO");
  }

  // ── 7. 运行状态（话题替代 SDK 的 get_robot_running_state）──
  RCLCPP_INFO(this->get_logger(), "\n========== 7. 机械臂运行状态（/arm_status）==========");
  RCLCPP_INFO(this->get_logger(), "  说明：SDK 的 get_robot_running_state（0=停止/1=暂停/2=运行）");
  RCLCPP_INFO(this->get_logger(), "        在 ROS2 中用 /arm_status 话题（STOP/PAUSE/RUNNING）近似");
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (rclcpp::ok() && arm_run_state_ == "unknown" && std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(this->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }
  RCLCPP_INFO(this->get_logger(), "  run_state = %s",
              arm_run_state_ == "unknown" ? "(未收到)" : arm_run_state_.c_str());

  // ── 8. 恢复示教模式 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 8. 恢复示教模式 ==========");
  {
    auto req = std::make_shared<SetCurrentMode::Request>();
    req->mode = 0; // 示教模式
    callService<SetCurrentMode>(set_mode_cli_, req, "set_current_mode(0)");
  }

  // ── 9. 断开连接 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 9. 断开连接 ==========");
  callTrigger(disconnect_cli_, "disconnect_arm");

  RCLCPP_INFO(this->get_logger(), "\n============================================================");
  RCLCPP_INFO(this->get_logger(), "  数字 IO 测试完成，节点退出");
  RCLCPP_INFO(this->get_logger(), "============================================================");
  rclcpp::shutdown();
}

// ====================================================================
//  main
// ====================================================================

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tl_example::DigitalIoDemo>();
  node->run(); // run() 内部已处理 rclcpp::shutdown()
  return 0;
}
