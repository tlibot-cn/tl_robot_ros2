/**
 * @file ex_queue_test.cpp
 * @brief 队列运动模式（Queue Motion）测试 — 添加两个 moveJ 并执行
 *
 * 流程：
 *   1. 连接 tl_driver，订阅 /joint_states 获取当前关节角（作为安全基准位）
 *   2. queue_motion_set_status(true)  —— 打开队列模式（控制器先下电再上电）
 *   3. 添加 moveJ #1（相对当前位姿偏移 d1 度，is_continue=true 先排队、不立即动）
 *   4. 添加 moveJ #2（相对当前位姿偏移 d2 度，is_continue=false 触发执行）
 *   5. 轮询 /arm_status 等待运动开始（run_state=RUNNING）
 *   6. queue_motion_stop              —— 运动中测试停止服务
 *   7. 轮询 /arm_status 等待运动停止（run_state=STOP）
 *   8. queue_motion_set_status(false) —— 关闭队列模式（执行伺服下电）
 *
 * 说明：
 *   - 关节空间 moveJ（coord=0），target_pos_value 为 14 维数组：
 *       [0,0,0,0,0,0,0, j1..j7]（j1..j7 为关节角，单位 度）
 *   - 偏移量通过 ROS 参数控制（d1/d2，单位 度），默认小幅值，安全演示。
 *   - queue_motion_stop 必须在队列模式已开启且有运动在跑时测试（SDK 规定：
 *     队列未开启会返回失败；运动未开始时调用会把队列清空，导致后续指令不执行）。
 *   - 测试结束后机械臂会下电（关闭队列模式的副作用），需要重新上电可调
 *     /tl_driver/power_on。
 *
 * @usage
 *   ros2 run tl_example ex_queue_test
 *   # 自定义偏移（度）
 *   ros2 run tl_example ex_queue_test --ros-args -p d1:=5.0 -p d2:=10.0
 *
 * @see ex_move_control.cpp — 运动控制示例（MoveJ/MoveL）
 */

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "tl_ros2_interface/msg/arm_status.hpp"
#include "tl_ros2_interface/msg/cartesian_pose.hpp"
#include "tl_ros2_interface/msg/move_command.hpp"
#include "tl_ros2_interface/srv/queue_motion_move_j.hpp"
#include "tl_ros2_interface/srv/queue_motion_set_status.hpp"
#include "tl_ros2_interface/srv/set_current_mode.hpp"

namespace tl_example
{

using ArmStatus = tl_ros2_interface::msg::ArmStatus;
using CartesianPose = tl_ros2_interface::msg::CartesianPose;
using JointState = sensor_msgs::msg::JointState;
using MoveCommand = tl_ros2_interface::msg::MoveCommand;

using QueueMotionSetStatus = tl_ros2_interface::srv::QueueMotionSetStatus;
using QueueMotionMoveJ = tl_ros2_interface::srv::QueueMotionMoveJ;
using SetCurrentMode = tl_ros2_interface::srv::SetCurrentMode;

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

/**
 * @brief 队列运动测试节点
 *
 * 以当前关节位姿为基准，添加两个带小幅偏移的 moveJ 指令，
 * 验证队列模式的 开→入队→执行→关 全流程。
 */
class QueueTestDemo : public rclcpp::Node
{
public:
  explicit QueueTestDemo(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("ex_queue_test", options)
  {
    // ── 参数 ──
    this->declare_parameter("d1", 10.0);           // moveJ #1 关节偏移（度）
    this->declare_parameter("d2", 20.0);           // moveJ #2 关节偏移（度）
    this->declare_parameter("velocity", 20.0);     // moveJ 速度（°/s，范围 (1,100]）
    this->declare_parameter("l_dx", 50.0);         // moveL X 偏移（mm）
    this->declare_parameter("l_dy", 0.0);          // moveL Y 偏移（mm）
    this->declare_parameter("l_dz", 0.0);          // moveL Z 偏移（mm）
    this->declare_parameter("l_vel", 30.0);        // moveL 速度（mm/s）
    this->declare_parameter("wait_timeout", 30.0); // 等待运动完成超时（秒）
    this->declare_parameter("startup_delay", 3.0); // 打开队列模式后等待伺服就绪（秒）
    this->declare_parameter("push_interval", 1.0); // 两条指令之间的入队间隔（秒）
    this->declare_parameter("run_duration", 3.0);  // 检测到 RUNNING 后先让机械臂走多久再 stop（秒）
    d1_ = this->get_parameter("d1").as_double();
    d2_ = this->get_parameter("d2").as_double();
    velocity_ = this->get_parameter("velocity").as_double();
    l_dx_ = this->get_parameter("l_dx").as_double();
    l_dy_ = this->get_parameter("l_dy").as_double();
    l_dz_ = this->get_parameter("l_dz").as_double();
    l_vel_ = this->get_parameter("l_vel").as_double();
    wait_timeout_ = this->get_parameter("wait_timeout").as_double();
    startup_delay_ = this->get_parameter("startup_delay").as_double();
    push_interval_ = this->get_parameter("push_interval").as_double();
    run_duration_ = this->get_parameter("run_duration").as_double();

    // ── 服务客户端 ──
    set_status_cli_ = this->create_client<QueueMotionSetStatus>("/tl_driver/queue_motion_set_status");
    movej_cli_ = this->create_client<QueueMotionMoveJ>("/tl_driver/queue_motion_movej");
    stop_cli_ = this->create_client<std_srvs::srv::Trigger>("/tl_driver/queue_motion_stop");
    power_on_cli_ = this->create_client<std_srvs::srv::Trigger>("/tl_driver/power_on");
    set_mode_cli_ = this->create_client<SetCurrentMode>("/tl_driver/set_current_mode");

    // ── 话题订阅 ──
    joint_state_sub_ = this->create_subscription<JointState>(
        "/joint_states", 10, std::bind(&QueueTestDemo::jointStateCb, this, std::placeholders::_1));
    arm_status_sub_ = this->create_subscription<ArmStatus>(
        "/arm_status", 10, std::bind(&QueueTestDemo::armStatusCb, this, std::placeholders::_1));
    tcp_pose_sub_ = this->create_subscription<CartesianPose>(
        "/tcp_pose", 10, std::bind(&QueueTestDemo::tcpPoseCb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "队列模式测试节点启动 (d1=%.1f° d2=%.1f° vel=%.1f°/s)", d1_, d2_, velocity_);
  }

  /// 运行测试主流程
  void run()
  {
    RCLCPP_INFO(this->get_logger(), "\n========== 队列运动测试 ==========");

    // 1. 等待服务就绪
    if (!waitService("queue_motion_set_status", set_status_cli_) || !waitService("queue_motion_movej", movej_cli_) ||
        !waitService("queue_motion_stop", stop_cli_) || !waitService("set_current_mode", set_mode_cli_))
    {
      RCLCPP_ERROR(this->get_logger(), "tl_driver 队列服务未就绪，请先启动 tl_driver 节点");
      rclcpp::shutdown();
      return;
    }

    // 2. 等待当前关节角（作为安全基准）
    spinWaitFor(
        [this]()
        {
          return got_joint_state_;
        },
        5.0);
    if (!got_joint_state_)
    {
      RCLCPP_ERROR(this->get_logger(), "未收到 /joint_states，无法确定安全基准位，中止测试");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "当前关节角(度): [%s]", joinDeg(current_joint_deg_).c_str());

    // 3. 切换到运行模式（官方示例：队列运动前必须 set_current_mode(2)，示教模式下不执行）
    setCurrentMode(2);

    // 4. 打开队列模式（SDK 会先下电再上电，需重新使能伺服）
    if (!setQueueStatus(true))
    {
      RCLCPP_ERROR(this->get_logger(), "打开队列模式失败，中止测试");
      rclcpp::shutdown();
      return;
    }

    // 4.5 打开队列模式后显式重新上电，确保伺服完整使能
    RCLCPP_INFO(this->get_logger(), "打开队列模式后重新上电使能伺服...");
    powerOn();

    RCLCPP_INFO(this->get_logger(), "等待伺服上电就绪 %.1fs...", startup_delay_);
    std::this_thread::sleep_for(std::chrono::duration<double>(startup_delay_));

    // 5. 添加两个 moveJ + 一条 moveL
    std::vector<double> target1 = current_joint_deg_;
    target1[2] += d1_; // 第3关节（index 2）偏移

    std::vector<double> target2 = current_joint_deg_;
    target2[2] += d2_;
    target2[3] -= d1_ * 0.5; // 第4关节小幅反向，让路径更明显

    pushMoveJ(target1, true); // is_continue=true  → 排队，不立即执行

    RCLCPP_INFO(this->get_logger(), "等待第 1 条入队完成 %.1fs...", push_interval_);
    std::this_thread::sleep_for(std::chrono::duration<double>(push_interval_));

    // moveL：基于当前 TCP 位姿 + 偏移（m→mm，姿态保持当前 rad）
    if (got_tcp_pose_)
    {
      std::vector<double> moveL_target = {current_tcp_mm_[0] + l_dx_, current_tcp_mm_[1] + l_dy_,
                                          current_tcp_mm_[2] + l_dz_, current_tcp_rad_[3],
                                          current_tcp_rad_[4],        current_tcp_rad_[5]};
      pushMoveL(moveL_target, true); // 排队，不立即执行

      RCLCPP_INFO(this->get_logger(), "等待 moveL 入队完成 %.1fs...", push_interval_);
      std::this_thread::sleep_for(std::chrono::duration<double>(push_interval_));
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "未收到 /tcp_pose，跳过 moveL 测试");
    }

    pushMoveJ(target2, false); // is_continue=false → 触发执行·

    // 5. 等待运动开始
    waitMotionStart();

    // 5.5 让机械臂真正走一段，再测试停止（否则刚启动就被 stop，看不出有没有动）
    RCLCPP_INFO(this->get_logger(), "机械臂运动中，等待 %.1fs 后再停止...", run_duration_);
    std::this_thread::sleep_for(std::chrono::duration<double>(run_duration_));

    // 6. 运动中测试 queue_motion_stop（停止/清空队列）
    queueMotionStop();

    // 7. 等待运动停止
    waitMotionStop();

    // 8. 关闭队列模式（驱动内部会自动切回示教模式并下电；需给足超时，
    //    因为刚 stop 后控制器忙，关闭时内部会执行 set_current_mode + power_off 可能较慢）
    setQueueStatus(false);

    RCLCPP_INFO(this->get_logger(), "\n========== 测试结束 ==========");
    RCLCPP_INFO(this->get_logger(), "队列模式已关闭（机械臂已下电）。如需重新上电：");
    RCLCPP_INFO(this->get_logger(), "  ros2 service call /tl_driver/power_on std_srvs/srv/Trigger");
    rclcpp::shutdown();
  }

private:
  // ── 回调 ──
  void jointStateCb(const JointState::SharedPtr msg)
  {
    if (msg->position.empty())
      return;
    current_joint_deg_.clear();
    for (const auto& rad : msg->position)
    {
      current_joint_deg_.push_back(rad * kRadToDeg);
    }
    got_joint_state_ = true;
  }

  void armStatusCb(const ArmStatus::SharedPtr msg)
  {
    arm_run_state_ = msg->run_state;
  }

  void tcpPoseCb(const CartesianPose::SharedPtr msg)
  {
    // /tcp_pose 已是 ROS 单位：位置 m、姿态 rad → 转 mm 存一份
    current_tcp_mm_ = {msg->position.x * 1000.0,
                       msg->position.y * 1000.0,
                       msg->position.z * 1000.0,
                       msg->rpy.x,
                       msg->rpy.y,
                       msg->rpy.z};
    current_tcp_rad_ = {msg->position.x * 1000.0,
                        msg->position.y * 1000.0,
                        msg->position.z * 1000.0,
                        msg->rpy.x,
                        msg->rpy.y,
                        msg->rpy.z};
    got_tcp_pose_ = true;
  }

  // ── 工具函数 ──
  bool waitService(const std::string& name, rclcpp::ClientBase::SharedPtr cli, double timeout_s = 5.0)
  {
    if (!cli->wait_for_service(std::chrono::duration<double>(timeout_s)))
    {
      RCLCPP_ERROR(this->get_logger(), "服务 %s 未就绪", name.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "服务 %s 已就绪", name.c_str());
    return true;
  }

  /// 打开/关闭队列模式
  bool setQueueStatus(bool status)
  {
    auto req = std::make_shared<QueueMotionSetStatus::Request>();
    req->status = status;
    // 关闭队列模式（status=false）时驱动内部会执行 set_current_mode + power_off，
    // 刚 stop 后可能较慢，给 30s 超时避免误判
    double timeout_s = status ? 10.0 : 30.0;
    auto resp = callService<QueueMotionSetStatus>(
        set_status_cli_, req,
        status ? "queue_motion_set_status(true)  打开队列模式" : "queue_motion_set_status(false) 关闭队列模式",
        timeout_s);
    return resp != nullptr && resp->success;
  }

  /// 测试 queue_motion_stop（队列模式开启时调用，清除已下发但未执行的队列）
  bool queueMotionStop()
  {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto resp = callService<std_srvs::srv::Trigger>(stop_cli_, req, "queue_motion_stop（停止/清空队列）");
    return resp != nullptr && resp->success;
  }

  /// 重新上电使能伺服（打开队列模式后 SDK 会下电再上电，需走完整使能流程）
  bool powerOn()
  {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto resp = callService<std_srvs::srv::Trigger>(power_on_cli_, req, "power_on（使能伺服）");
    return resp != nullptr && resp->success;
  }

  /// 切换运行模式（0=示教 1=远程 2=运行）。队列运动前必须切到运行模式(2)
  bool setCurrentMode(int mode)
  {
    auto req = std::make_shared<SetCurrentMode::Request>();
    req->mode = mode;
    const char *name = (mode == 2) ? "set_current_mode(2) 运行模式" : "set_current_mode(0) 示教模式";
    auto resp = callService<SetCurrentMode>(set_mode_cli_, req, name);
    return resp != nullptr && resp->success;
  }

  /// 添加一条 moveJ 并决定是否立即执行（is_continue）
  bool pushMoveJ(const std::vector<double>& joint_deg, bool is_continue)
  {
    auto req = std::make_shared<QueueMotionMoveJ::Request>();
    req->is_continue = is_continue;

    req->cmd.target_pos_type = 0; // 0 = 直接点位（data）
    req->cmd.target_pos_name = "";
    req->cmd.coord = 0; // 0 = 关节坐标系
    req->cmd.velocity = velocity_;
    req->cmd.velocity_sync = 0.0;
    req->cmd.acc = velocity_;
    req->cmd.dec = velocity_;
    req->cmd.pl = 0;
    req->cmd.time = 0;
    req->cmd.tool_num = 0;
    req->cmd.user_num = 0;
    req->cmd.posidtype = 0;
    req->cmd.configuration = 0;
    req->cmd.spin = 0;
    req->cmd.para_sync = false;

    // 关节角放在 index 0~6（官方示例 make_movej_cmd 布局，前 7 位为关节角）
    req->cmd.target_pos_value.assign(14, 0.0);
    for (size_t i = 0; i < joint_deg.size() && i < 7; ++i)
    {
      req->cmd.target_pos_value[i] = joint_deg[i];
    }

    std::string label =
        "queue_motion_movej #" + std::string(is_continue ? "1(排队)" : "2(执行)") + " -> [" + joinDeg(joint_deg) + "]";
    auto resp = callService<QueueMotionMoveJ>(movej_cli_, req, label);
    return resp != nullptr && resp->success;
  }

  /// 添加一条 moveL（coord=1 基坐标系，位姿 [X,Y,Z,Rx,Ry,Rz]：mm + rad，放 index 0~5）
  bool pushMoveL(const std::vector<double>& pose, bool is_continue)
  {
    auto req = std::make_shared<QueueMotionMoveJ::Request>();
    req->is_continue = is_continue;

    req->cmd.target_pos_type = 0; // 0 = 直接点位（data）
    req->cmd.target_pos_name = "";
    req->cmd.coord = 1;         // 1 = 基坐标系（直角坐标系）
    req->cmd.velocity = l_vel_; // mm/s
    req->cmd.velocity_sync = 0.0;
    req->cmd.acc = l_vel_;
    req->cmd.dec = l_vel_;
    req->cmd.pl = 0;
    req->cmd.time = 0;
    req->cmd.tool_num = 0;
    req->cmd.user_num = 0;
    req->cmd.posidtype = 0;
    req->cmd.configuration = 0;
    req->cmd.spin = 0;
    req->cmd.para_sync = false;

    // 位姿放 index 0~5（官方示例 make_movel_cmd 布局），后 8 位置 0
    req->cmd.target_pos_value.assign(14, 0.0);
    for (size_t i = 0; i < pose.size() && i < 6; ++i)
    {
      req->cmd.target_pos_value[i] = pose[i];
    }

    std::string label = "queue_motion_movel(排队) -> [" + joinPose(pose) + "]";
    auto resp = callService<QueueMotionMoveJ>(movej_cli_, req, label);
    return resp != nullptr && resp->success;
  }

  /// 轮询 /arm_status，等待运动开始（出现 RUNNING）
  void waitMotionStart()
  {
    RCLCPP_INFO(this->get_logger(), "等待运动开始 (RUNNING)...");
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(wait_timeout_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(this->get_node_base_interface());
      if (arm_run_state_ == "RUNNING")
      {
        RCLCPP_INFO(this->get_logger(), "  运动已开始 (run_state=RUNNING)");
        return;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    RCLCPP_WARN(this->get_logger(), "  等待运动开始超时（%.0fs），当前 run_state=%s", wait_timeout_,
                arm_run_state_.c_str());
  }

  /// 轮询 /arm_status，等待运动停止（回到 STOP）
  void waitMotionStop()
  {
    RCLCPP_INFO(this->get_logger(), "等待运动停止 (STOP)...");
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(wait_timeout_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(this->get_node_base_interface());
      if (arm_run_state_ == "STOP")
      {
        RCLCPP_INFO(this->get_logger(), "  运动已停止 (run_state=STOP)");
        return;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    RCLCPP_WARN(this->get_logger(), "  等待运动停止超时（%.0fs），当前 run_state=%s", wait_timeout_,
                arm_run_state_.c_str());
  }

  /// 泛型服务调用：同步等待并打印成功/失败
  /// @param timeout_s 超时秒数（默认 10s；关闭队列模式等慢操作建议传更大值）
  template <typename Srv>
  typename Srv::Response::SharedPtr callService(const typename rclcpp::Client<Srv>::SharedPtr& cli,
                                                typename Srv::Request::SharedPtr req, const std::string& label,
                                                double timeout_s = 10.0)
  {
    if (!cli->service_is_ready())
    {
      RCLCPP_WARN(this->get_logger(), "  [%s] 服务未就绪，跳过", label.c_str());
      return nullptr;
    }
    auto future = cli->async_send_request(req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future,
                                           std::chrono::duration<double>(timeout_s)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(), "  [%s] 调用超时/异常（%.0fs）", label.c_str(), timeout_s);
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

  /// 短暂 spin 直到条件满足或超时
  void spinWaitFor(const std::function<bool()>& cond, double timeout_s = 2.0)
  {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (rclcpp::ok() && !cond() && std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(this->get_node_base_interface());
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
  }

  /// 角度数组 → 字符串
  static std::string joinDeg(const std::vector<double>& deg)
  {
    std::string s;
    for (size_t i = 0; i < deg.size(); ++i)
    {
      if (i)
        s += ", ";
      s += std::to_string(static_cast<int>(deg[i]));
    }
    return s;
  }

  /// 位姿数组 → 字符串（位置显示整数，姿态显示 2 位小数）
  static std::string joinPose(const std::vector<double>& pose)
  {
    std::string s;
    for (size_t i = 0; i < pose.size(); ++i)
    {
      if (i)
        s += ", ";
      char buf[32];
      if (i < 3)
      {
        snprintf(buf, sizeof(buf), "%.1f", pose[i]);
      }
      else
      {
        snprintf(buf, sizeof(buf), "%.3f", pose[i]);
      }
      s += buf;
    }
    return s;
  }

  // ── 成员 ──
  rclcpp::Client<QueueMotionSetStatus>::SharedPtr set_status_cli_;
  rclcpp::Client<QueueMotionMoveJ>::SharedPtr movej_cli_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_cli_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_on_cli_;
  rclcpp::Client<SetCurrentMode>::SharedPtr set_mode_cli_;

  rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<ArmStatus>::SharedPtr arm_status_sub_;
  rclcpp::Subscription<CartesianPose>::SharedPtr tcp_pose_sub_;

  double d1_{10.0};
  double d2_{20.0};
  double velocity_{20.0};
  double l_dx_{50.0};
  double l_dy_{0.0};
  double l_dz_{0.0};
  double l_vel_{30.0};
  double wait_timeout_{30.0};
  double startup_delay_{3.0};
  double push_interval_{1.0};
  double run_duration_{3.0};

  bool got_joint_state_{false};
  bool got_tcp_pose_{false};
  std::vector<double> current_joint_deg_;
  std::vector<double> current_tcp_mm_;
  std::vector<double> current_tcp_rad_;
  std::string arm_run_state_{"unknown"};
};

} // namespace tl_example

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tl_example::QueueTestDemo>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
