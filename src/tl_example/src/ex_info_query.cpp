/**
 * @file ex_info_query.cpp
 * @brief 信息查询测试 — 仅测试说明书 §4 信息查询接口
 *
 * 聚焦测试信息查询类接口（对应 tl_sdk/examples/cpp/ex_info_query.cpp）：
 *
 * §4 信息查询接口（按说明书编号 4.1~4.20，终端打印顺序与说明书一致）：
 *   [4.1]  /joint_states 话题           查询关节角度（rad）
 *   [4.2]  /tcp_pose 话题               查询末端位姿（m/rad）
 *   [4.3]  /arm_status 话题             speed 运行速度
 *   [4.4]  get_controller_id            查询控制器序列号
 *   [4.5]  /arm_status 话题             run_state 机械臂状态
 *   [4.6]  get_library_version          查询库版本
 *   [4.7]  get_robot_joint_param        查询关节参数
 *   [4.8]  /tl_driver/joint_thermal     关节温度
 *   [4.9]  /tl_driver/joint_thermal     关节/外部轴电压
 *   [4.10] /tl_driver/joint_dynamics    电机电流
 *   [4.11] 启动日志                     关节软件版本（驱动启动时打印）
 *   [4.13] get_current_coord            查询当前坐标系
 *   [4.14] get_coord_num                查询坐标系编号
 *   [4.15] get_dh_param                 查询机械臂DH参数
 *   [4.17] get_pos_reachable            查询目标位姿可达状态（关节+直角两坐标系，打印目标位姿）
 *   [4.18] /arm_status 话题             查询机械臂运行状态
 *   [4.19] /tl_driver/joint_dynamics    电机力矩（额定扭矩千分之一）
 *   [4.20] /tl_driver/joint_dynamics    线速度和关节速度
 *
 * 仅做查询与读取，不执行任何设置/运动操作，可放心在真实机械臂上运行。
 *
 * @usage
 *   ros2 run tl_example ex_info_query
 *
 * @see ex_move_control.cpp — 运动控制示例（MoveJ/MoveL）
 */

#include <algorithm>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include "tl_ros2_interface/msg/arm_status.hpp"
#include "tl_ros2_interface/msg/cartesian_pose.hpp"
#include "tl_ros2_interface/msg/joint_dynamics.hpp"
#include "tl_ros2_interface/msg/joint_thermal.hpp"

#include "tl_ros2_interface/srv/get_coord_num.hpp"
#include "tl_ros2_interface/srv/get_current_coord.hpp"
#include "tl_ros2_interface/srv/get_dh_param.hpp"
#include "tl_ros2_interface/srv/get_pos_reachable.hpp"
#include "tl_ros2_interface/srv/get_robot_joint_param.hpp"

namespace tl_example
{

// 类型别名 — 简化代码
using Trigger = std_srvs::srv::Trigger;
using ArmStatus = tl_ros2_interface::msg::ArmStatus;
using CartesianPose = tl_ros2_interface::msg::CartesianPose;
using JointDynamics = tl_ros2_interface::msg::JointDynamics;
using JointThermal = tl_ros2_interface::msg::JointThermal;
using JointState = sensor_msgs::msg::JointState;

/**
 * @brief 信息查询测试节点
 *
 * 流程：连接机械臂 → 依次查询全部信息查询类服务 → 读取 3 个信息话题 → 断开连接。
 * 统一通过 callService<> / callTrigger() 同步调用并打印结果。
 */
class InfoQueryDemo : public rclcpp::Node
{
public:
  explicit InfoQueryDemo(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("ex_info_query", options)
  {
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "  信息查询测试节点启动");
    RCLCPP_INFO(this->get_logger(), "  仅测试说明书 §4 信息查询接口（只读，不设置/不动）");
    RCLCPP_INFO(this->get_logger(), "============================================================");

    // ── Trigger 类查询服务 ──
    trig_clis_["get_controller_id"] = create_client<Trigger>("/tl_driver/get_controller_id");
    trig_clis_["get_library_version"] = create_client<Trigger>("/tl_driver/get_library_version");

    // ── 信息查询类服务（仅保留没有对应话题的只读接口）──
    get_joint_param_cli_ =
        create_client<tl_ros2_interface::srv::GetRobotJointParam>("/tl_driver/get_robot_joint_param");
    get_current_coord_cli_ = create_client<tl_ros2_interface::srv::GetCurrentCoord>("/tl_driver/get_current_coord");
    get_coord_num_cli_ = create_client<tl_ros2_interface::srv::GetCoordNum>("/tl_driver/get_coord_num");
    get_dh_param_cli_ = create_client<tl_ros2_interface::srv::GetDHParam>("/tl_driver/get_dh_param");
    get_pos_reachable_cli_ = create_client<tl_ros2_interface::srv::GetPosReachable>("/tl_driver/get_pos_reachable");

    // ── 话题订阅（信息查询类）──
    arm_status_sub_ = this->create_subscription<ArmStatus>(
        "/arm_status", 10, std::bind(&InfoQueryDemo::onArmStatus, this, std::placeholders::_1));
    joint_state_sub_ = this->create_subscription<JointState>(
        "/joint_states", 10, std::bind(&InfoQueryDemo::onJointState, this, std::placeholders::_1));
    tcp_pose_sub_ = this->create_subscription<CartesianPose>(
        "/tcp_pose", 10, std::bind(&InfoQueryDemo::onTcpPose, this, std::placeholders::_1));
    // 诊断话题为 transient_local + depth 1，晚加入也能拿到最后一帧。
    const auto diagnostic_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    dynamics_sub_ =
        this->create_subscription<JointDynamics>("/tl_driver/joint_dynamics", diagnostic_qos,
                                                 std::bind(&InfoQueryDemo::onDynamics, this, std::placeholders::_1));
    thermal_sub_ = this->create_subscription<JointThermal>(
        "/tl_driver/joint_thermal", diagnostic_qos, std::bind(&InfoQueryDemo::onThermal, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "  客户端与订阅创建完成");
  }

  // ── 主流程（main 直接调用）──
  void run();

private:
  // ── Trigger 类客户端 ──
  std::map<std::string, rclcpp::Client<Trigger>::SharedPtr> trig_clis_;

  // ── 查询类客户端 ──
  rclcpp::Client<tl_ros2_interface::srv::GetRobotJointParam>::SharedPtr get_joint_param_cli_;
  rclcpp::Client<tl_ros2_interface::srv::GetCurrentCoord>::SharedPtr get_current_coord_cli_;
  rclcpp::Client<tl_ros2_interface::srv::GetCoordNum>::SharedPtr get_coord_num_cli_;
  rclcpp::Client<tl_ros2_interface::srv::GetDHParam>::SharedPtr get_dh_param_cli_;
  rclcpp::Client<tl_ros2_interface::srv::GetPosReachable>::SharedPtr get_pos_reachable_cli_;

  // ── 话题订阅 ──
  rclcpp::Subscription<ArmStatus>::SharedPtr arm_status_sub_;
  rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<CartesianPose>::SharedPtr tcp_pose_sub_;
  rclcpp::Subscription<JointDynamics>::SharedPtr dynamics_sub_;
  rclcpp::Subscription<JointThermal>::SharedPtr thermal_sub_;

  // 话题最近收到的数据
  std::string arm_run_state_ = "unknown";
  double arm_speed_ = 0.0;
  bool arm_speed_valid_ = false;
  rclcpp::Time arm_status_stamp_{0, 0, RCL_ROS_TIME};
  bool got_joint_state_ = false;
  bool got_tcp_pose_ = false;
  bool got_dynamics_ = false;
  bool got_thermal_ = false;
  JointState::SharedPtr last_joint_state_;
  CartesianPose::SharedPtr last_tcp_pose_;
  JointDynamics::SharedPtr last_dynamics_;
  JointThermal::SharedPtr last_thermal_;

  // ── 话题回调 ──
  void onArmStatus(const ArmStatus::SharedPtr msg)
  {
    arm_run_state_ = msg->run_state;
    arm_speed_ = msg->speed;
    arm_speed_valid_ = msg->speed_valid;
    arm_status_stamp_ = rclcpp::Time(msg->stamp);
  }
  void onJointState(const JointState::SharedPtr msg)
  {
    last_joint_state_ = msg;
    got_joint_state_ = true;
  }
  void onTcpPose(const CartesianPose::SharedPtr msg)
  {
    last_tcp_pose_ = msg;
    got_tcp_pose_ = true;
  }
  void onDynamics(const JointDynamics::SharedPtr msg)
  {
    last_dynamics_ = msg;
    got_dynamics_ = true;
  }
  void onThermal(const JointThermal::SharedPtr msg)
  {
    last_thermal_ = msg;
    got_thermal_ = true;
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

  /// 调用 Trigger 服务（通过 map key）
  bool callTrigger(const std::string& key, const std::string& label)
  {
    auto it = trig_clis_.find(key);
    if (it == trig_clis_.end())
      return false;
    auto cli = it->second;
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

  /// 泛型服务调用：同步等待并打印 success/message，返回响应供调用方打印额外字段
  template <typename Srv>
  typename Srv::Response::SharedPtr callService(const typename rclcpp::Client<Srv>::SharedPtr& cli,
                                                typename Srv::Request::SharedPtr req, const std::string& label)
  {
    if (!cli)
      return nullptr;
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

  /// 短暂 spin 直到条件满足或超时（用于等待话题消息）
  void spinWaitFor(const std::function<bool()>& cond, double timeout_s = 2.0)
  {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (rclcpp::ok() && !cond() && std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(this->get_node_base_interface());
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
  }

  /// 打印 float64 数组
  template <typename T>
  void printArray(const std::vector<T>& v, const std::string& prefix, size_t max = 16)
  {
    std::string s;
    size_t n = std::min(v.size(), max);
    for (size_t i = 0; i < n; ++i)
    {
      if (i)
        s += ", ";
      s += std::to_string(v[i]);
    }
    if (v.size() > max)
      s += ", ...";
    RCLCPP_INFO(this->get_logger(), "      %s[%zu] = [%s]", prefix.c_str(), v.size(), s.c_str());
  }

  // ── 测试方法 ──
  void testInfoQuery(); // §4 信息查询（按说明书编号 4.1~4.20 顺序打印）
};

} // namespace tl_example

// ====================================================================
//  主流程实现（类外定义，main 直接调用）
// ====================================================================

void tl_example::InfoQueryDemo::run()
{
  RCLCPP_INFO(this->get_logger(), "\n========== 1. 检查核心服务就绪 ==========");
  if (!waitService("get_library_version", trig_clis_["get_library_version"]))
  {
    RCLCPP_ERROR(this->get_logger(), "tl_driver 核心服务未就绪，请先启动 tl_driver 节点");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(this->get_logger(), "\n========== §4 信息查询（按说明书编号 4.1~4.20）==========");
  testInfoQuery();

  RCLCPP_INFO(this->get_logger(), "\n============================================================");
  RCLCPP_INFO(this->get_logger(), "  信息查询测试完成，节点退出");
  RCLCPP_INFO(this->get_logger(), "============================================================");
  rclcpp::shutdown();
}

// ── §4 信息查询（按说明书编号 4.1~4.20 顺序执行）──

void tl_example::InfoQueryDemo::testInfoQuery()
{
  // [4.1] 查询关节角度（话题 /joint_states）
  RCLCPP_INFO(this->get_logger(), "  --- [4.1] 查询关节角度 /joint_states ---");
  spinWaitFor(
      [this]()
      {
        return got_joint_state_;
      },
      2.0);
  if (last_joint_state_)
  {
    RCLCPP_INFO(this->get_logger(), "      关节数 = %zu", last_joint_state_->position.size());
    printArray(last_joint_state_->position, "      position(rad) ", 12);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "      未收到 /joint_states");
  }

  // [4.2] 查询末端位姿（话题 /tcp_pose）
  RCLCPP_INFO(this->get_logger(), "  --- [4.2] 查询末端位姿 /tcp_pose ---");
  spinWaitFor(
      [this]()
      {
        return got_tcp_pose_;
      },
      2.0);
  if (last_tcp_pose_)
  {
    // /tcp_pose 为 ROS 标准单位（位置 m、姿态 rad），见 publish_tcp_pose 的 mm→m 转换
    RCLCPP_INFO(this->get_logger(), "      pos = [%.3f, %.3f, %.3f] m", last_tcp_pose_->position.x,
                last_tcp_pose_->position.y, last_tcp_pose_->position.z);
    RCLCPP_INFO(this->get_logger(), "      rpy = [%.3f, %.3f, %.3f] rad", last_tcp_pose_->rpy.x, last_tcp_pose_->rpy.y,
                last_tcp_pose_->rpy.z);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "      未收到 /tcp_pose");
  }

  // [4.3] 查询运行速度（话题 /arm_status.speed，1 Hz 缓存）
  RCLCPP_INFO(this->get_logger(), "  --- [4.3] 查询运行速度 /arm_status.speed ---");
  spinWaitFor(
      [this]()
      {
        return arm_speed_valid_;
      },
      2.0);
  if (arm_speed_valid_)
  {
    RCLCPP_INFO(this->get_logger(), "      speed = %.1f%%", arm_speed_);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "      speed 尚未就绪（speed_valid=false）");
  }

  // [4.4] 查询控制器序列号ID
  RCLCPP_INFO(this->get_logger(), "  --- [4.4] 查询控制器序列号 get_controller_id ---");
  callTrigger("get_controller_id", "get_controller_id");

  // [4.5] 查询机械臂状态（话题 /arm_status.run_state）
  RCLCPP_INFO(this->get_logger(), "  --- [4.5] 查询机械臂状态 /arm_status.run_state ---");
  spinWaitFor(
      [this]()
      {
        return arm_run_state_ != "unknown";
      },
      2.0);
  RCLCPP_INFO(this->get_logger(), "      run_state = %s",
              arm_run_state_ == "unknown" ? "(未收到)" : arm_run_state_.c_str());

  // [4.6] 查询库版本信息
  RCLCPP_INFO(this->get_logger(), "  --- [4.6] 查询库版本 get_library_version ---");
  callTrigger("get_library_version", "get_library_version");

  // [4.7] 查询关节参数
  RCLCPP_INFO(this->get_logger(), "  --- [4.7] 查询关节参数 get_robot_joint_param ---");
  {
    auto req = std::make_shared<tl_ros2_interface::srv::GetRobotJointParam::Request>();
    req->id = 1;
    auto resp = callService<tl_ros2_interface::srv::GetRobotJointParam>(get_joint_param_cli_, req,
                                                                        "get_robot_joint_param(id=1)");
    if (resp && resp->success)
    {
      RCLCPP_INFO(this->get_logger(), "      减速比=%.3f 限位[%.3f, %.3f]rad 额定速度=%.3frad/s",
                  resp->param.reduction_ratio, resp->param.neg_sw_limit, resp->param.pos_sw_limit,
                  resp->param.rated_vel);
    }
  }

  // [4.8] 查询关节温度（话题 /tl_driver/joint_thermal）
  RCLCPP_INFO(this->get_logger(), "  --- [4.8] 查询关节温度 /tl_driver/joint_thermal ---");
  spinWaitFor(
      [this]()
      {
        return got_thermal_;
      },
      2.0);
  if (last_thermal_ && !last_thermal_->temperature.empty())
  {
    printArray(last_thermal_->temperature, "      temperature(℃) ");
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "      未收到关节温度");
  }

  // [4.9] 查询关节电压（同一话题）
  RCLCPP_INFO(this->get_logger(), "  --- [4.9] 查询关节电压 /tl_driver/joint_thermal ---");
  if (last_thermal_ && !last_thermal_->joint_voltage.empty())
  {
    printArray(last_thermal_->joint_voltage, "      joint_voltage(V) ");
    printArray(last_thermal_->positioner_voltage, "      positioner_voltage(V) ");
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "      未收到关节电压");
  }

  // [4.10] 查询电机电流（话题 /tl_driver/joint_dynamics）
  RCLCPP_INFO(this->get_logger(), "  --- [4.10] 查询电机电流 /tl_driver/joint_dynamics ---");
  spinWaitFor(
      [this]()
      {
        return got_dynamics_;
      },
      2.0);
  if (last_dynamics_ && !last_dynamics_->motor_current.empty())
  {
    printArray(last_dynamics_->motor_current, "      motor_current(A) ");
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "      未收到电机电流");
  }

  // [4.11] 查询关节软件版本
  RCLCPP_INFO(this->get_logger(), "  --- [4.11] 关节软件版本（驱动启动日志 [Init]: joint N software version）---");
  RCLCPP_INFO(this->get_logger(), "      已改为启动时打印，失败不阻断驱动启动；此处不再提供服务调用");

  // [4.13] 查询当前坐标系
  RCLCPP_INFO(this->get_logger(), "  --- [4.13] 查询当前坐标系 get_current_coord ---");
  {
    auto req = std::make_shared<tl_ros2_interface::srv::GetCurrentCoord::Request>();
    auto resp = callService<tl_ros2_interface::srv::GetCurrentCoord>(get_current_coord_cli_, req, "get_current_coord");
    if (resp && resp->success)
      RCLCPP_INFO(this->get_logger(), "      coord = %d（0=关节 1=直角 2=工具 3=用户）", resp->coord);
  }

  // [4.14] 查询坐标系编号
  RCLCPP_INFO(this->get_logger(), "  --- [4.14] 查询坐标系编号 get_coord_num ---");
  {
    auto req = std::make_shared<tl_ros2_interface::srv::GetCoordNum::Request>();
    auto resp = callService<tl_ros2_interface::srv::GetCoordNum>(get_coord_num_cli_, req, "get_coord_num");
    if (resp && resp->success)
      RCLCPP_INFO(this->get_logger(), "      tool_num=%d user_num=%d", resp->tool_num, resp->user_num);
  }

  // [4.15] 查询机械臂DH参数
  RCLCPP_INFO(this->get_logger(), "  --- [4.15] 查询机械臂DH参数 get_dh_param ---");
  {
    auto req = std::make_shared<tl_ros2_interface::srv::GetDHParam::Request>();
    auto resp = callService<tl_ros2_interface::srv::GetDHParam>(get_dh_param_cli_, req, "get_dh_param");
    if (resp && resp->success)
    {
      printArray(resp->param.alpha, "      alpha(rad) ");
      printArray(resp->param.a, "      a(m) ");
      printArray(resp->param.theta, "      theta(rad) ");
      printArray(resp->param.d, "      d(m) ");
      RCLCPP_INFO(this->get_logger(), "      euler_angle=%d mounting_angle=%.3f", resp->param.euler_angle,
                  resp->param.mounting_angle);
    }
  }

  // [4.17] 查询目标位姿可达状态（打印测试目标位姿，关节 + 直角两种坐标系）
  RCLCPP_INFO(this->get_logger(), "  --- [4.17] 查询目标位姿可达 get_pos_reachable ---");
  // 14 维位姿：[0]=坐标系(0=关节 1=直角) [1]=单位制(0=度 1=弧度) [2]=形态 [3]=工具序号
  //           [4]=用户序号 [5][6]=备用 [7~13]=点位信息
  {
    // ① 关节坐标系（MOVJ）：J1=10°, J2=20°
    RCLCPP_INFO(this->get_logger(), "      ① 关节坐标系(0)/角度制(0), move_type=MOVJ");
    const std::vector<double> target_joint = {10.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    pos.insert(pos.end(), target_joint.begin(), target_joint.end());
    printArray(target_joint, "      目标关节角(°) ");
    printArray(pos, "      pos(14维) ");
    auto req = std::make_shared<tl_ros2_interface::srv::GetPosReachable::Request>();
    req->pos = pos;
    req->move_type = "MOVJ";
    auto resp =
        callService<tl_ros2_interface::srv::GetPosReachable>(get_pos_reachable_cli_, req, "get_pos_reachable(MOVJ)");
    if (resp && resp->success)
    {
      if (resp->reachable)
        RCLCPP_INFO(this->get_logger(), "      ✓ 目标可达");
      else
        RCLCPP_WARN(this->get_logger(), "      ✗ 目标不可达");
    }
  }
  {
    // ② 直角坐标系（MOVL）：位置 [280, 90, 270] mm，姿态 [3.14, 0, 0] rad
    //    （该点位与 MoveL 常用目标一致，已在真实机械臂上验证可达）
    RCLCPP_INFO(this->get_logger(), "      ② 直角坐标系(1)/弧度制(1), move_type=MOVL");
    const std::vector<double> target_pose = {280.0, 90.0, 270.0, 3.14, 0.0, 0.0, 0.0};
    std::vector<double> pos = {1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    pos.insert(pos.end(), target_pose.begin(), target_pose.end());
    RCLCPP_INFO(this->get_logger(), "      目标直角位姿 [X,Y,Z,RX,RY,RZ] = [280mm, 90mm, 270mm, 3.14rad, 0, 0]");
    printArray(target_pose, "      目标位姿(7维) ");
    printArray(pos, "      pos(14维) ");
    auto req = std::make_shared<tl_ros2_interface::srv::GetPosReachable::Request>();
    req->pos = pos;
    req->move_type = "MOVL";
    auto resp =
        callService<tl_ros2_interface::srv::GetPosReachable>(get_pos_reachable_cli_, req, "get_pos_reachable(MOVL)");
    if (resp && resp->success)
    {
      if (resp->reachable)
        RCLCPP_INFO(this->get_logger(), "      ✓ 目标可达");
      else
        RCLCPP_WARN(this->get_logger(), "      ✗ 目标不可达");
    }
  }

  // [4.18] /arm_status 采样时间（验证 10 Hz 查询缓存）
  RCLCPP_INFO(this->get_logger(), "  --- [4.18] /arm_status 采样时间 ---");
  spinWaitFor(
      [this]()
      {
        return arm_run_state_ != "unknown";
      },
      2.0);
  if (arm_run_state_ != "unknown")
  {
    const double age_ms = (this->now() - arm_status_stamp_).seconds() * 1000.0;
    RCLCPP_INFO(this->get_logger(), "      run_state=%s stamp_age=%.1f ms", arm_run_state_.c_str(), age_ms);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "      未收到 /arm_status");
  }

  // [4.19] 查询当前电机力矩（话题 /tl_driver/joint_dynamics，单位：额定扭矩千分之一）
  RCLCPP_INFO(this->get_logger(), "  --- [4.19] 查询当前电机力矩 /tl_driver/joint_dynamics ---");
  spinWaitFor(
      [this]()
      {
        return got_dynamics_;
      },
      2.0);
  if (last_dynamics_ && !last_dynamics_->motor_torque.empty())
  {
    printArray(last_dynamics_->motor_torque, "      motor_torque(‰ 额定) ");
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "      未收到电机力矩");
  }

  // [4.20] 查询当前线速度和关节速度（同一话题）
  RCLCPP_INFO(this->get_logger(), "  --- [4.20] 查询当前线速度和关节速度 /tl_driver/joint_dynamics ---");
  if (last_dynamics_)
  {
    RCLCPP_INFO(this->get_logger(), "      line_speed = %.3f m/s", last_dynamics_->line_speed);
    printArray(last_dynamics_->joint_speed, "      joint_speed(rad/s) ");
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "      未收到线速度/关节速度");
  }
}

// ====================================================================
//  main
// ====================================================================

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tl_example::InfoQueryDemo>();
  node->run(); // run() 内部已处理 rclcpp::shutdown()
  return 0;
}
