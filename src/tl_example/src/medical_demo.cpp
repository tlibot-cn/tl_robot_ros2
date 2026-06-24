/**
 * @file medical_demo.cpp
 * @brief 队列 MoveL 运动演示 — 医学检验科自动化场景
 *
 * 通过逐点直角坐标直线运动（MoveL, coord=1），模拟检验科典型操作流程。
 * 先切换到示教模式（mode=0），显式上电，再逐条下发队列运动；结束时显式下电。
 */

#include "tl_example/medical_demo.h"

#include <cmath>
#include <map>

namespace tl_example
{

// ====================================================================
// 演示点位定义（直角坐标系, coord=1）
//
// target_pos_value 字段 14 维：
//   [X(mm), Y(mm), Z(mm), RX(rad), RY(rad), RZ(rad),
//    外部轴1..7 备用 0]
//
// RX = -π 表示工具末端竖直向下，适合移液/夹取操作
// ====================================================================

namespace
{
constexpr double TOOL_DOWN_RX = -M_PI; // -180° 工具竖直向下
constexpr double TOOL_RY = 0.0;
constexpr double TOOL_RZ = -M_PI / 2.0; // -90°

/**
 * @brief 构造 14 维直角坐标数组
 * @param x,y,z  末端位置（mm）
 * @param rx,ry,rz  末端姿态（rad）
 */
std::vector<double> makePos(double x, double y, double z, double rx = TOOL_DOWN_RX, double ry = TOOL_RY,
                            double rz = TOOL_RZ)
{
  return {x, y, z, rx, ry, rz, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

// 直角坐标点位字典
const std::map<std::string, std::vector<double>> POSITIONS = {
    // 安全等待位（机械臂收缩，远离工作区）
    {"home", makePos(230.004, -0.009, 359.008, -3.141, 0.0, 0.0)},
    // 样本架上方（检验科采血管架）
    {"above_sample", makePos(150.0, 100.0, 350.0)},
    // 样本位（下降吸取 / 夹取）
    {"at_sample", makePos(150.0, 100.0, 180.0)},
    // 试剂架 / 分装位上方
    {"above_reagent", makePos(250.0, 100.0, 350.0)},
    // 试剂位（下降加样 / 分装）
    {"at_reagent", makePos(250.0, 100.0, 180.0)},
    // 检测仪器进样口上方
    {"above_instrument", makePos(200.0, 250.0, 350.0)},
    // 检测仪器进样口（放置样本）
    {"at_instrument", makePos(200.0, 250.0, 200.0)},
};

// 演示序列：{ 点位, 动作描述, 速度% }
const std::vector<MedicalDemo::SequenceStep> DEMO_SEQUENCE = {
    {"home", "返回安全等待位", 80},
    {"above_sample", "移动到样本架上方（检验科采血管架）", 80},
    {"at_sample", "下降至取样位（吸取血液 / 体液样本）", 80},
    {"above_sample", "抬起样本（避免碰撞转移）", 80},
    {"above_reagent", "移动到试剂架上方（生化 / 免疫试剂位）", 80},
    {"at_reagent", "下降至分装加样位（样本注入试剂孔板）", 80},
    {"above_reagent", "抬起末端（避免拖拽污染）", 80},
    {"above_instrument", "移动到检测仪器进样口（化学发光 / 生化分析仪）", 80},
    {"at_instrument", "下降放置样本（将样本架送入仪器）", 80},
    {"above_instrument", "抬起末端", 80},
    {"home", "返回安全位置，流程结束", 80},
};

} // anonymous namespace

// ====================================================================
// MedicalDemo
// ====================================================================

MedicalDemo::MedicalDemo(const rclcpp::NodeOptions& options) : Node("medical_demo", options)
{
  movel_pub_ = this->create_publisher<MoveCommand>("/tl_driver/moveL", 10);

  state_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MedicalDemo::stateMachine, this));

  arm_status_sub_ = this->create_subscription<ArmStatus>(
      "/arm_status", 10, std::bind(&MedicalDemo::onArmStatus, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "============================================================");
  RCLCPP_INFO(this->get_logger(), "  队列 MoveL 演示节点已启动 (Queue MoveL Demo)");
  RCLCPP_INFO(this->get_logger(), "  演示步骤数: %zu", DEMO_SEQUENCE.size());
  RCLCPP_INFO(this->get_logger(), "  全局速度:   %.0f%%", demo_speed_);
  RCLCPP_INFO(this->get_logger(), "  等待 tl_driver 就绪...");
  RCLCPP_INFO(this->get_logger(), "============================================================");
}

// ──────────────────────────────────────────────────────────────
//  辅助方法
// ──────────────────────────────────────────────────────────────

void MedicalDemo::ensureClients()
{
  if (!connect_cli_)
  {
    connect_cli_ = this->create_client<Trigger>("/tl_driver/connect_arm");
  }
  if (!set_speed_cli_)
  {
    set_speed_cli_ = this->create_client<SetSpeed>("/tl_driver/set_speed");
  }
  if (!set_mode_cli_)
  {
    set_mode_cli_ = this->create_client<SetCurrentMode>("/tl_driver/set_current_mode");
  }
  if (!power_on_cli_)
  {
    power_on_cli_ = this->create_client<Trigger>("/tl_driver/power_on");
  }
  if (!power_off_cli_)
  {
    power_off_cli_ = this->create_client<Trigger>("/tl_driver/power_off");
  }
}

bool MedicalDemo::waitService(rclcpp::ClientBase::SharedPtr client, const std::string& name, double timeout_s)
{
  if (!client->wait_for_service(std::chrono::duration<double>(timeout_s)))
  {
    RCLCPP_ERROR(this->get_logger(), "服务 %s 未就绪（%.0fs 超时）", name.c_str(), timeout_s);
    return false;
  }
  return true;
}

MedicalDemo::MoveCommand MedicalDemo::buildMoveL(const std::string& pos_key, double velocity) const
{
  const auto& pos = POSITIONS.at(pos_key);

  MoveCommand msg;
  msg.target_pos_value = pos;
  msg.target_pos_name = "";
  msg.target_pos_type = 0; // 自定义数组模式
  msg.coord = 1;           // 直角坐标系
  msg.velocity = velocity;
  msg.velocity_sync = 0.0;
  msg.acc = velocity;
  msg.dec = velocity;
  msg.pl = 0; // 精确到达
  msg.time = 0;
  msg.tool_num = 0;
  msg.user_num = 0;
  msg.posidtype = 0;
  msg.configuration = 0;
  msg.spin = 0;
  msg.para_sync = false;
  return msg;
}

// ====================================================================
//  状态机（10 Hz 定时器驱动）
// ====================================================================

void MedicalDemo::stateMachine()
{
  switch (state_)
  {
    case State::INIT:
      ensureClients();
      if (waitService(connect_cli_, "connect_arm", 2.0))
      {
        RCLCPP_INFO(this->get_logger(), "tl_driver 服务已就绪，开始连接机械臂...");
        state_ = State::CONNECTING;
      }
      break;

    case State::CONNECTING:
      startConnect();
      break;

    case State::SET_MODE:
      startSetMode();
      break;

    case State::POWER_ON:
      startPowerOn();
      break;

    case State::SET_SPEED:
      startSetSpeed();
      break;

    case State::MOVING:
      moveNextStep();
      break;

    case State::POWER_OFF:
      startPowerOff();
      break;

    case State::DONE:
      break;
  }
}

// ── CONNECTING ──

void MedicalDemo::startConnect()
{
  auto req = std::make_shared<Trigger::Request>();
  auto future =
      connect_cli_->async_send_request(req, std::bind(&MedicalDemo::onConnectDone, this, std::placeholders::_1));
  state_ = State::DONE; // 等待回调
}

void MedicalDemo::onConnectDone(rclcpp::Client<Trigger>::SharedFuture future)
{
  const auto& resp = future.get();
  if (resp->success)
  {
    RCLCPP_INFO(this->get_logger(), "✓ 机械臂连接成功");
    state_ = State::SET_MODE;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "✗ 连接失败: %s", resp->message.c_str());
    RCLCPP_ERROR(this->get_logger(), "请确认机械臂状态后重试 (Ctrl-C 退出)");
  }
}

// ── SET_MODE ──

void MedicalDemo::startSetMode()
{
  auto req = std::make_shared<SetCurrentMode::Request>();
  req->mode = 0; // 示教模式（不自动上电）
  auto future =
      set_mode_cli_->async_send_request(req, std::bind(&MedicalDemo::onSetModeDone, this, std::placeholders::_1));
  state_ = State::DONE;
}

void MedicalDemo::onSetModeDone(rclcpp::Client<SetCurrentMode>::SharedFuture future)
{
  const auto& resp = future.get();
  if (resp->success)
  {
    RCLCPP_INFO(this->get_logger(), "✓ 已切换到示教模式 (mode=0)");
    state_ = State::POWER_ON;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "✗ 切换示教模式失败: %s", resp->message.c_str());
    state_ = State::DONE;
  }
}

// ── POWER_ON ──

void MedicalDemo::startPowerOn()
{
  auto req = std::make_shared<Trigger::Request>();
  auto future =
      power_on_cli_->async_send_request(req, std::bind(&MedicalDemo::onPowerOnDone, this, std::placeholders::_1));
  state_ = State::DONE;
}

void MedicalDemo::onPowerOnDone(rclcpp::Client<Trigger>::SharedFuture future)
{
  const auto& resp = future.get();
  if (resp->success)
  {
    RCLCPP_INFO(this->get_logger(), "✓ 机械臂上电成功");
    state_ = State::SET_SPEED;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "✗ 上电失败: %s", resp->message.c_str());
    state_ = State::DONE;
  }
}

// ── SET_SPEED ──

void MedicalDemo::startSetSpeed()
{
  auto req = std::make_shared<SetSpeed::Request>();
  req->speed = demo_speed_;
  auto future =
      set_speed_cli_->async_send_request(req, std::bind(&MedicalDemo::onSetSpeedDone, this, std::placeholders::_1));
  state_ = State::DONE;
}

void MedicalDemo::onSetSpeedDone(rclcpp::Client<SetSpeed>::SharedFuture future)
{
  const auto& resp = future.get();
  if (resp->success)
  {
    RCLCPP_INFO(this->get_logger(), "✓ 速度设置为 %.0f%%", demo_speed_);
    state_ = State::MOVING;
    seq_index_ = 0;
    move_start_time_s_ = 0.0;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "✗ 设置速度失败: %s", resp->message.c_str());
    state_ = State::DONE;
  }
}

// ── MOVING ──

void MedicalDemo::moveNextStep()
{
  if (seq_index_ >= DEMO_SEQUENCE.size())
  {
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "  演示序列完成！准备下电...");
    RCLCPP_INFO(this->get_logger(), "============================================================");
    state_ = State::POWER_OFF;
    return;
  }

  switch (move_phase_)
  {
    case MovePhase::IDLE:
    {
      // 发送当前 MoveL
      const auto& step = DEMO_SEQUENCE[seq_index_];
      RCLCPP_INFO(this->get_logger(), "▶ [%zu/%zu] %s  → 位置=%s  速度=%.0f%%", seq_index_ + 1, DEMO_SEQUENCE.size(),
                  step.description.c_str(), step.pos_key.c_str(), step.velocity);

      MoveCommand msg = buildMoveL(step.pos_key, step.velocity);
      movel_pub_->publish(msg);

      move_start_time_s_ = this->now().seconds();
      move_phase_ = MovePhase::WAIT_RUNNING;
      break;
    }

    case MovePhase::WAIT_RUNNING:
    {
      // 等待机械臂进入 RUNNING 状态（确认运动已启动）
      if (arm_run_state_ == "RUNNING")
      {
        RCLCPP_INFO(this->get_logger(), "  运动已启动，等待完成...");
        move_phase_ = MovePhase::WAIT_DONE;
      }
      break;
    }

    case MovePhase::WAIT_DONE:
    {
      // 等待机械臂回到 STOP 状态（运动完成）
      if (arm_run_state_ == "STOP" || arm_run_state_ == "PAUSE")
      {
        double elapsed = this->now().seconds() - move_start_time_s_;
        RCLCPP_INFO(this->get_logger(), "  ✓ 运动完成（耗时 %.1fs）", elapsed);
        seq_index_++;
        move_phase_ = MovePhase::IDLE;
      }
      break;
    }
  }

  // 安全超时兜底：运动总时长超过安全阈值时跳过
  if (move_phase_ != MovePhase::IDLE)
  {
    double elapsed = this->now().seconds() - move_start_time_s_;
    if (elapsed > safety_timeout_s_)
    {
      RCLCPP_WARN(this->get_logger(), "步骤 %zu/%zu 安全超时（%.1fs > %.0fs），跳过", seq_index_ + 1,
                  DEMO_SEQUENCE.size(), elapsed, safety_timeout_s_);
      seq_index_++;
      move_phase_ = MovePhase::IDLE;
    }
  }
}

void MedicalDemo::onArmStatus(const ArmStatus::SharedPtr msg)
{
  arm_run_state_ = msg->run_state;
}

// ── POWER_OFF ──

void MedicalDemo::startPowerOff()
{
  RCLCPP_INFO(this->get_logger(), "机械臂下电中...");
  auto req = std::make_shared<Trigger::Request>();
  auto future =
      power_off_cli_->async_send_request(req, std::bind(&MedicalDemo::onPowerOffDone, this, std::placeholders::_1));
  state_ = State::DONE;
}

void MedicalDemo::onPowerOffDone(rclcpp::Client<Trigger>::SharedFuture future)
{
  try
  {
    const auto& resp = future.get();
    if (resp->success)
    {
      RCLCPP_INFO(this->get_logger(), "✓ 机械臂下电成功");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "下电失败: %s", resp->message.c_str());
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(this->get_logger(), "下电异常: %s", e.what());
  }
  RCLCPP_INFO(this->get_logger(), "演示结束。可 Ctrl-C 退出。");
}

} // namespace tl_example

// ====================================================================
//  main
// ====================================================================

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tl_example::MedicalDemo>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
