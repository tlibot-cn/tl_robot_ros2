/**
 * @file ex_teach_control.cpp
 * @brief 点动（示教）测试
 *
 * 对应 tl_sdk/examples/cpp/ex_teach_control.cpp 的 ROS2 实现。
 * 演示示教模式下的关节点动控制：
 *
 *   connect → set_current_mode(示教) → power_on →
 *   关节坐标系点动（set_current_coord=0，axis=关节轴号）→
 *   直角坐标系点动（set_current_coord=1，axis: 1~3=X/Y/Z, 4~6=RX/RY/RZ）→
 *   power_off → disconnect
 *
 * ⚠ 安全警告：start_jogging 会直接驱动机器人运动！
 *   运行前请确保工作区安全、无人靠近，并随时准备急停。
 *
 * ⚠ 点动维持说明：按《tl_driver服务与话题说明书》，点动指令需周期性调用才能
 *   维持运动（通常每 40ms 一次）。本示例在点动持续期间按 jog_refresh_ms 周期
 *   重发 start_jogging，结束后再调用 stop_jogging，保证实机可持续运动。
 *
 * ⚠ 跳过项：SDK 中的 set_current_teach_sensitivity / get_current_teach_sensitivity
 *   （电流环拖动示教灵敏度设置/查询）在 ROS2 驱动中无对应接口，程序中跳过并列出。
 *
 * @usage
 *   ros2 run tl_example ex_teach_control
 *   # 自定义参数
 *   ros2 run tl_example ex_teach_control --ros-args -p jog_axis:=1 -p jog_seconds:=5 \
 *       -p cart_jog_seconds:=3 -p jog_refresh_ms:=40
 *
 * @see ex_servoj_test.cpp — ServoJ 关节插补测试
 */

#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include "tl_ros2_interface/msg/arm_status.hpp"
#include "tl_ros2_interface/msg/cartesian_pose.hpp"
#include "tl_ros2_interface/srv/jogging.hpp"
#include "tl_ros2_interface/srv/set_current_coord.hpp"
#include "tl_ros2_interface/srv/set_current_mode.hpp"

namespace tl_example
{

// 类型别名 — 简化代码
using Trigger = std_srvs::srv::Trigger;
using ArmStatus = tl_ros2_interface::msg::ArmStatus;
using CartesianPose = tl_ros2_interface::msg::CartesianPose;
using JointState = sensor_msgs::msg::JointState;
using Jogging = tl_ros2_interface::srv::Jogging;
using SetCurrentCoord = tl_ros2_interface::srv::SetCurrentCoord;
using SetCurrentMode = tl_ros2_interface::srv::SetCurrentMode;

/**
 * @brief 点动（示教）测试节点
 *
 * 流程（同步顺序执行）：
 *   connect → set_current_mode(示教) → power_on →
 *   关节坐标系点动（set_current_coord=0，axis=关节轴号）→
 *   直角坐标系点动（set_current_coord=1，axis: 1~3=X/Y/Z, 4~6=RX/RY/RZ）→
 *   power_off → disconnect
 * 每次点动前后打印关节角度 / 末端位姿（话题为弧度/米，打印转角度/mm）用于对比。
 */
class TeachControlDemo : public rclcpp::Node
{
public:
  explicit TeachControlDemo(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("ex_teach_control", options)
  {
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "  点动（示教）测试节点启动");
    RCLCPP_INFO(this->get_logger(), "  ⚠ 本示例会真实驱动机器人运动，请注意安全");
    RCLCPP_INFO(this->get_logger(), "============================================================");

    // ── 参数 ──
    this->declare_parameter("jog_axis", 1);             // 关节坐标系点动：axis 1~7 = 关节轴号 J1~J7
    this->declare_parameter("jog_dir", true);           // 关节点动方向：true=正方向
    this->declare_parameter("jog_seconds", 5);          // 关节坐标系点动持续时间（秒）
    this->declare_parameter("cart_x_axis", 1);          // 直角坐标系：位置轴（1~3 = X/Y/Z）
    this->declare_parameter("cart_rx_axis", 4);         // 直角坐标系：姿态轴（4~6 = RX/RY/RZ）
    this->declare_parameter("cart_jog_seconds", 3);     // 直角坐标系点动持续时间（秒）
    this->declare_parameter("jog_refresh_ms", 40);      // 点动维持重发周期（ms），<=0 表示只发一次

    jog_axis_           = this->get_parameter("jog_axis").as_int();
    jog_dir_            = this->get_parameter("jog_dir").as_bool();
    jog_seconds_        = this->get_parameter("jog_seconds").as_int();
    cart_x_axis_        = this->get_parameter("cart_x_axis").as_int();
    cart_rx_axis_       = this->get_parameter("cart_rx_axis").as_int();
    cart_jog_seconds_   = this->get_parameter("cart_jog_seconds").as_int();
    jog_refresh_ms_     = this->get_parameter("jog_refresh_ms").as_int();

    // ── 服务客户端 ──
    connect_cli_      = this->create_client<Trigger>("/tl_driver/connect_arm");
    disconnect_cli_   = this->create_client<Trigger>("/tl_driver/disconnect_arm");
    power_on_cli_     = this->create_client<Trigger>("/tl_driver/power_on");
    power_off_cli_    = this->create_client<Trigger>("/tl_driver/power_off");
    set_mode_cli_     = this->create_client<SetCurrentMode>("/tl_driver/set_current_mode");
    set_current_coord_cli_ = this->create_client<SetCurrentCoord>("/tl_driver/set_current_coord");
    start_jogging_cli_ = this->create_client<Jogging>("/tl_driver/start_jogging");
    stop_jogging_cli_  = this->create_client<Jogging>("/tl_driver/stop_jogging");

    // ── 状态订阅（运行状态 + 关节角度 + 末端位姿，用于点动前后对比）──
    arm_status_sub_ = this->create_subscription<ArmStatus>(
        "/arm_status", 10, std::bind(&TeachControlDemo::onArmStatus, this, std::placeholders::_1));
    joint_state_sub_ = this->create_subscription<JointState>(
        "/joint_states", 10, std::bind(&TeachControlDemo::onJointState, this, std::placeholders::_1));
    tcp_pose_sub_ = this->create_subscription<CartesianPose>(
        "/tcp_pose", 10, std::bind(&TeachControlDemo::onTcpPose, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "  客户端/订阅创建完成 (J%d 点动%d秒, 直角X轴=%d/RX轴=%d各%d秒, 刷新=%dms)",
                jog_axis_, jog_seconds_, cart_x_axis_, cart_rx_axis_, cart_jog_seconds_, jog_refresh_ms_);
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
  rclcpp::Client<SetCurrentCoord>::SharedPtr set_current_coord_cli_;
  rclcpp::Client<Jogging>::SharedPtr start_jogging_cli_;
  rclcpp::Client<Jogging>::SharedPtr stop_jogging_cli_;
  rclcpp::Subscription<ArmStatus>::SharedPtr arm_status_sub_;
  rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<CartesianPose>::SharedPtr tcp_pose_sub_;

  // 最近一次接收到的机械臂运行状态（/arm_status）
  std::string arm_run_state_ = "unknown";
  // 最近一次接收到的关节角度 / 末端位姿（话题均为弧度/米，打印时转角度/mm）
  JointState::SharedPtr last_joint_state_;
  CartesianPose::SharedPtr last_tcp_pose_;

  // ── 参数 ──
  int jog_axis_ = 1;         // 关节坐标系点动：axis 1~7 = 关节轴号 J1~J7
  bool jog_dir_ = true;      // true=正方向
  int jog_seconds_ = 2;      // 关节坐标系点动持续时间（秒）
  int cart_x_axis_ = 1;      // 直角坐标系位置轴（1~3 = X/Y/Z）
  int cart_rx_axis_ = 4;     // 直角坐标系姿态轴（4~6 = RX/RY/RZ）
  int cart_jog_seconds_ = 2; // 直角坐标系点动持续时间（秒）
  int jog_refresh_ms_ = 40;  // 点动维持重发周期（ms），<=0 表示只发一次

  // ── 话题回调 ──
  void onArmStatus(const ArmStatus::SharedPtr msg) { arm_run_state_ = msg->run_state; }
  void onJointState(const JointState::SharedPtr msg) { last_joint_state_ = msg; }
  void onTcpPose(const CartesianPose::SharedPtr msg) { last_tcp_pose_ = msg; }

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

  /// 执行一次点动（start → 周期性维持 → stop），期间持续 spin 收集 /arm_status
  void jogOnce(int axis, bool dir, int seconds, const std::string& desc)
  {
    RCLCPP_INFO(this->get_logger(), "  ▶ %s（axis=%d, %s, %d 秒）",
                desc.c_str(), axis, dir ? "正方向" : "负方向", seconds);
    auto req = std::make_shared<Jogging::Request>();
    req->axis = axis;
    req->direction = dir;
    if (!callService<Jogging>(start_jogging_cli_, req, "start_jogging")) {
      RCLCPP_WARN(this->get_logger(), "  点动启动失败，跳过本次点动");
      return;
    }
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
    auto next_refresh = std::chrono::steady_clock::now() + std::chrono::milliseconds(jog_refresh_ms_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(this->get_node_base_interface());
      rclcpp::sleep_for(std::chrono::milliseconds(50));
      // 部分控制器需要周期性重发点动指令才能维持运动（通常每 40ms 一次）
      if (jog_refresh_ms_ > 0 && std::chrono::steady_clock::now() >= next_refresh) {
        callService<Jogging>(start_jogging_cli_, req, "start_jogging(维持)");
        next_refresh = std::chrono::steady_clock::now() + std::chrono::milliseconds(jog_refresh_ms_);
      }
    }
    RCLCPP_INFO(this->get_logger(), "  点动结束，期间 run_state = %s",
                arm_run_state_ == "unknown" ? "(未收到)" : arm_run_state_.c_str());
    auto stop_req = std::make_shared<Jogging::Request>();
    stop_req->axis = axis;
    stop_req->direction = dir;
    callService<Jogging>(stop_jogging_cli_, stop_req, "stop_jogging");
  }

  /// 短暂 spin 收集话题数据（确保拿到最新关节角度 / 末端位姿）
  void spinCollectTopics(double timeout_s = 0.5)
  {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(this->get_node_base_interface());
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
  }

  /// 打印当前关节角度（话题为弧度，转为角度更直观）
  void printJointStateDeg(const std::string& tag)
  {
    if (!last_joint_state_) {
      RCLCPP_INFO(this->get_logger(), "  %s 关节角度: (未收到 /joint_states)", tag.c_str());
      return;
    }
    constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
    std::string s;
    for (size_t i = 0; i < last_joint_state_->position.size(); ++i) {
      if (i) s += ", ";
      char buf[32];
      snprintf(buf, sizeof(buf), "J%zu=%.2f°", i + 1, last_joint_state_->position[i] * kRad2Deg);
      s += buf;
    }
    RCLCPP_INFO(this->get_logger(), "  %s 关节角度(度): [%s]", tag.c_str(), s.c_str());
  }

  /// 打印当前末端位姿（话题位置为 mm、姿态为弧度，姿态转角度更直观）
  void printTcpPose(const std::string& tag)
  {
    if (!last_tcp_pose_) {
      RCLCPP_INFO(this->get_logger(), "  %s 末端位姿: (未收到 /tcp_pose)", tag.c_str());
      return;
    }
    constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
    RCLCPP_INFO(this->get_logger(),
                "  %s 末端位姿: pos(%.1f, %.1f, %.1f)mm  rpy(%.2f, %.2f, %.2f)°",
                tag.c_str(),
                last_tcp_pose_->position.x,
                last_tcp_pose_->position.y,
                last_tcp_pose_->position.z,
                last_tcp_pose_->rpy.x * kRad2Deg,
                last_tcp_pose_->rpy.y * kRad2Deg,
                last_tcp_pose_->rpy.z * kRad2Deg);
  }
};

} // namespace tl_example

// ====================================================================
//  主流程实现（类外定义，main 直接调用）
// ====================================================================

void tl_example::TeachControlDemo::run()
{
  // ── 1. 检查核心服务就绪 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 1. 检查核心服务就绪 ==========");
  if (!waitService("connect_arm", connect_cli_) ||
      !waitService("set_current_mode", set_mode_cli_) ||
      !waitService("power_on", power_on_cli_) ||
      !waitService("start_jogging", start_jogging_cli_) ||
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
    if (!callService<SetCurrentMode>(set_mode_cli_, req, "set_current_mode(0=示教)")) {
      RCLCPP_WARN(this->get_logger(), "  切换示教模式失败，后续点动可能无效，继续尝试");
    }
  }

  // ── 4. 机械臂上电 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 4. 机械臂上电 ==========");
  if (!callTrigger(power_on_cli_, "power_on")) {
    RCLCPP_ERROR(this->get_logger(), "上电失败，退出测试");
    callTrigger(disconnect_cli_, "disconnect_arm（清理）");
    rclcpp::shutdown();
    return;
  }

  // ── 5. 关节坐标系点动（⚠ 真实运动）──
  // 说明：坐标系=0（关节）时，start_jogging 的 axis 1~7 代表关节轴号 J1~J7，
  //       即「几号轴就转几号关节」。
  RCLCPP_INFO(this->get_logger(), "\n========== 5. 关节坐标系点动（⚠ 真实运动）==========");
  RCLCPP_INFO(this->get_logger(), "  说明：坐标系=0（关节）时，axis 1~7 = 关节轴号 J1~J7");
  {
    auto req = std::make_shared<SetCurrentCoord::Request>();
    req->coord = 0;  // 关节坐标系
    callService<SetCurrentCoord>(set_current_coord_cli_, req, "set_current_coord(0=关节)");
  }
  // 点动前：读取当前关节角度（用于与点动后对比）
  spinCollectTopics(0.5);
  printJointStateDeg("点动前");
  // 演示：J<jog_axis> 轴 jog_dir 方向点动
  jogOnce(jog_axis_, jog_dir_, jog_seconds_,
          "关节 J" + std::to_string(jog_axis_) + (jog_dir_ ? " 正方向" : " 负方向") + "点动");
  printJointStateDeg("点动后（对比 J" + std::to_string(jog_axis_) + " 应发生变化）");

  // ── 6. 直角坐标系点动（⚠ 真实运动）──
  // 说明：坐标系=1（直角）时，start_jogging 的 axis 含义变化：
  //       axis 1~3 = 位置 X/Y/Z（直线平移），axis 4~6 = 姿态 RX/RY/RZ（旋转）。
  RCLCPP_INFO(this->get_logger(), "\n========== 6. 直角坐标系点动（⚠ 真实运动）==========");
  RCLCPP_INFO(this->get_logger(), "  说明：坐标系=1（直角）时，axis 1~3 = 位置 X/Y/Z，axis 4~6 = 姿态 RX/RY/RZ");
  {
    auto req = std::make_shared<SetCurrentCoord::Request>();
    req->coord = 1;  // 直角坐标系
    callService<SetCurrentCoord>(set_current_coord_cli_, req, "set_current_coord(1=直角)");
  }
  // 点动前：读取当前末端位姿（用于与点动后对比）
  spinCollectTopics(0.5);
  printTcpPose("点动前");
  // 演示：X 正方向平移（axis=cart_x_axis_）与 RX 正方向旋转（axis=cart_rx_axis_）
  jogOnce(cart_x_axis_, true, cart_jog_seconds_, "直角坐标 X 正方向平移");
  printTcpPose("点动后（对比 X 应发生变化）");
  jogOnce(cart_rx_axis_, true, cart_jog_seconds_, "直角坐标 RX 正方向旋转");
  printTcpPose("点动后（对比 RX 应发生变化）");

  // 恢复关节坐标系（避免影响后续操作）
  {
    auto req = std::make_shared<SetCurrentCoord::Request>();
    req->coord = 0;
    callService<SetCurrentCoord>(set_current_coord_cli_, req, "set_current_coord(0=关节，恢复)");
  }

  // ── 7. 机械臂下电 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 7. 机械臂下电 ==========");
  callTrigger(power_off_cli_, "power_off");

  // ── 8. 断开连接 ──
  RCLCPP_INFO(this->get_logger(), "\n========== 8. 断开连接 ==========");
  callTrigger(disconnect_cli_, "disconnect_arm");

  RCLCPP_INFO(this->get_logger(), "\n============================================================");
  RCLCPP_INFO(this->get_logger(), "  点动（示教）测试完成，节点退出");
  RCLCPP_INFO(this->get_logger(), "============================================================");
  rclcpp::shutdown();
}

// ====================================================================
//  main
// ====================================================================

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tl_example::TeachControlDemo>();
  node->run();  // run() 内部已处理 rclcpp::shutdown()
  return 0;
}
