/**
 * @file ex_servoj_test.cpp
 * @brief ServoJ 关节插补测试 — 示教模式下从零点位置插值运动
 *
 * 流程：
 *   1. 等待 /tl_driver/open_servoj、/tl_driver/close_servoj 服务就绪
 *   2. open_servoj                   —— 开启 ServoJ 跟踪模式（示教模式下即可）
 *   3. 从零点位置（全 0°）出发，指定关节线性插值到目标角度，
 *      共 num_points 个插值点，按 rate Hz 频率发布到 /tl_driver/set_servoj_pos
 *   4. 到达目标后保持 hold_time 秒（避免停发导致机械臂急停）
 *   5. close_servoj                  —— 关闭跟踪模式
 *
 * 说明：
 *   - /tl_driver/set_servoj_pos 的数据单位为「度」。
 *   - 目标角度方向：target_angle 为正表示关节正向运动。
 *   - 测试前请确保机械臂处于零点位置（关节角全为 0°）。
 *
 * @usage
 *   ros2 run tl_example ex_servoj_test
 *   # 自定义参数
 *   ros2 run tl_example ex_servoj_test --ros-args -p joint:=0 -p target_angle:=50.0 \
 *       -p num_points:=200 -p rate:=100.0
 *
 * @see ex_driver_quick_test.cpp — 驱动全接口快速自检（§15 ServoJ）
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "tl_ros2_interface/srv/open_servo_j.hpp"

namespace tl_example
{

using OpenServoJ = tl_ros2_interface::srv::OpenServoJ;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;

/**
 * @brief ServoJ 插补测试节点
 *
 * 示教模式下开启 ServoJ 跟踪，从零点位置对指定关节做线性插值运动。
 */
class ServoJTestDemo : public rclcpp::Node
{
public:
  explicit ServoJTestDemo(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("ex_servoj_test", options)
  {
    // ── 参数 ──
    this->declare_parameter("arm_joints", 7);       // 机械臂关节数
    this->declare_parameter("joint", 0);            // 要运动的关节索引（J1 = 0）
    this->declare_parameter("target_angle", 50.0);  // 目标角度（度）
    this->declare_parameter("num_points", 200);     // 插值点数
    this->declare_parameter("rate", 100.0);         // 下发频率（Hz）
    this->declare_parameter("hold_time", 1.0);      // 到达目标后保持时间（秒）
    this->declare_parameter("vmax", 80.0);          // ServoJ 最大速度
    this->declare_parameter("amax", 3000.0);        // ServoJ 最大加速度
    this->declare_parameter("jmax", 50000.0);       // ServoJ 最大加加速度

    arm_joints_ = this->get_parameter("arm_joints").as_int();
    joint_ = this->get_parameter("joint").as_int();
    target_angle_ = this->get_parameter("target_angle").as_double();
    num_points_ = this->get_parameter("num_points").as_int();
    rate_ = this->get_parameter("rate").as_double();
    hold_time_ = this->get_parameter("hold_time").as_double();
    vmax_ = this->get_parameter("vmax").as_double();
    amax_ = this->get_parameter("amax").as_double();
    jmax_ = this->get_parameter("jmax").as_double();

    if (joint_ < 0 || joint_ >= arm_joints_) {
      RCLCPP_ERROR(this->get_logger(), "joint 索引 %d 超出关节数 %d，中止", joint_, arm_joints_);
      rclcpp::shutdown();
    }

    // ── 服务客户端 ──
    open_cli_ = this->create_client<OpenServoJ>("/tl_driver/open_servoj");
    close_cli_ = this->create_client<std_srvs::srv::Trigger>("/tl_driver/close_servoj");

    // ── 话题发布 ──
    servoj_pub_ = this->create_publisher<Float64MultiArray>("/tl_driver/set_servoj_pos", 10);

    RCLCPP_INFO(this->get_logger(), "ServoJ 插补测试节点启动 (J%d: 0° -> %.1f°, %d 点 @ %.0fHz)",
                joint_ + 1, target_angle_, num_points_, rate_);
  }

  /// 运行测试主流程
  void run()
  {
    RCLCPP_INFO(this->get_logger(), "\n========== ServoJ 插补测试 ==========");

    // 1. 等待服务就绪
    if (!waitService("open_servoj", open_cli_) || !waitService("close_servoj", close_cli_)) {
      RCLCPP_ERROR(this->get_logger(), "tl_driver ServoJ 服务未就绪，请先启动 tl_driver 节点");
      rclcpp::shutdown();
      return;
    }

    // 2. 开启 ServoJ 跟踪模式（示教模式下即可，无需 set_current_mode）
    if (!openServoJ()) {
      RCLCPP_ERROR(this->get_logger(), "open_servoj 失败，中止测试");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_WARN(this->get_logger(),
                "请确保机械臂当前处于零点位置（关节角全为 0°），即将开始运动，Ctrl-C 可随时中断 ...");
    rclcpp::sleep_for(std::chrono::seconds(1));

    // 3. 从零点位置开始，对指定关节线性插值（0° -> target_angle°）
    rclcpp::Rate rate(rate_);
    std::vector<double> pos(arm_joints_, 0.0);
    bool aborted = false;
    try {
      // 3.1 插值 num_points 个点
      for (int i = 0; i < num_points_; ++i) {
        if (!rclcpp::ok()) { aborted = true; break; }
        double fraction = (num_points_ > 1) ? static_cast<double>(i) / (num_points_ - 1) : 1.0;
        pos[joint_] = target_angle_ * fraction;
        publishPos(pos);
        rate.sleep();
      }

      // 3.2 到达目标后保持 hold_time，避免停发导致机械臂急停
      pos[joint_] = target_angle_;
      auto hold_end = std::chrono::steady_clock::now() + std::chrono::duration<double>(hold_time_);
      while (rclcpp::ok() && std::chrono::steady_clock::now() < hold_end) {
        publishPos(pos);
        rate.sleep();
      }

      if (rclcpp::ok() && !aborted) {
        RCLCPP_INFO(this->get_logger(), "插补完成：J%d 已到达 %.1f°", joint_ + 1, target_angle_);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "插补过程异常: %s", e.what());
      aborted = true;
    }

    // 4. 关闭 ServoJ 跟踪模式
    closeServoJ();

    RCLCPP_INFO(this->get_logger(), "\n========== 测试结束 ==========");
    rclcpp::shutdown();
  }

private:
  // ── 工具函数 ──
  bool waitService(const std::string& name, rclcpp::ClientBase::SharedPtr cli, double timeout_s = 5.0)
  {
    if (!cli->wait_for_service(std::chrono::duration<double>(timeout_s))) {
      RCLCPP_ERROR(this->get_logger(), "服务 %s 未就绪", name.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "服务 %s 已就绪", name.c_str());
    return true;
  }

  /// 泛型服务调用：同步等待并打印成功/失败
  template <typename Srv>
  typename Srv::Response::SharedPtr callService(const typename rclcpp::Client<Srv>::SharedPtr& cli,
                                                typename Srv::Request::SharedPtr req,
                                                const std::string& label,
                                                double timeout_s = 5.0)
  {
    if (!cli->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "  [%s] 服务未就绪，跳过", label.c_str());
      return nullptr;
    }
    auto future = cli->async_send_request(req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future,
                                           std::chrono::duration<double>(timeout_s)) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "  [%s] 调用超时/异常（%.0fs）", label.c_str(), timeout_s);
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

  /// 开启 ServoJ 跟踪模式
  bool openServoJ()
  {
    auto req = std::make_shared<OpenServoJ::Request>();
    req->vmax.assign(arm_joints_, vmax_);
    req->amax.assign(arm_joints_, amax_);
    req->jmax.assign(arm_joints_, jmax_);
    auto resp = callService<OpenServoJ>(open_cli_, req, "open_servoj（开启跟踪模式）");
    return resp != nullptr && resp->success;
  }

  /// 关闭 ServoJ 跟踪模式
  bool closeServoJ()
  {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto resp = callService<std_srvs::srv::Trigger>(close_cli_, req, "close_servoj（关闭跟踪模式）");
    return resp != nullptr && resp->success;
  }

  /// 发布一组目标关节角（度）
  void publishPos(const std::vector<double>& pos)
  {
    Float64MultiArray msg;
    msg.data = pos;
    servoj_pub_->publish(msg);
  }

  // ── 成员 ──
  rclcpp::Client<OpenServoJ>::SharedPtr open_cli_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr close_cli_;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr servoj_pub_;

  int arm_joints_{7};
  int joint_{0};
  int num_points_{200};
  double target_angle_{50.0};
  double rate_{100.0};
  double hold_time_{1.0};
  double vmax_{80.0};
  double amax_{3000.0};
  double jmax_{50000.0};
};

} // namespace tl_example

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tl_example::ServoJTestDemo>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
