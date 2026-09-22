# Watchdog — tl_robot_ros2

控制器协议与单位约定。这些不变量横跨 `tl_driver` / `tl_teleop` / `tl_teleop_f710` / `tl_hardware`，
改动相关代码前先对照本节。本节只记录**代码里已实现、但容易被改错或改回去**的约定；
工作空间结构、命名规范、构建命令见 `AGENTS.md`，接口逐条说明见 `src/tl_driver/doc/tl_driver服务与话题说明书.md`。
本节与代码冲突时**以代码为准**，并同步修正文档。

## 双端口连接

| 端口 | 参数 | 职责 |
|---|---|---|
| 6001 | `arm_port` | 主端口：请求/响应式 SDK 调用（运动、IO、Modbus、作业、参数查询、外部轴运动） |
| 7000 | `arm_port_aux` | 辅助端口：servoJ / servoP 全系列、伺服点位运动、独立轴、碰撞检测参数、拖拽示教、工具坐标范围与电流环示教灵敏度，以及**机器人状态的异步推送** |

- 划分依据：`src/tl_driver/lib/include/cpp/interface/tl_interface.h` 第 771 行 `/*---7000端口功能---*/` 之后的 80 个导出函数；逐条说明见说明书第 15 章。
- 两端口都必须连上：任一失败即 `is_connected()` 为假（`tl_driver.cpp:588-604`；`is_connected()` 另要求两个 fd 均 > 0，`:503`）。
- servoJ 一律走 aux：`open_servoJ` / `close_servoJ` / `set_servoJ_pos` 全部传 `socket_fd_aux_`（`:2290`、`:2306`、`:2546`、`:2718`）。
- **状态推送回调只在 aux 注册**：`recv_message(socket_fd_aux_, robot_state_recv_callback)`（`:613`）。
- **错误/告警回调两个端口都注册**（`:609` 主端口 + `:611` aux）。该接口本身不属 7000 段，别因为「错误消息是 7000 端口的事」而删掉主端口那次注册。
- ⚠️ **端口现状与文档不一致（未修正，改动前需真机验证）**：三个 SDK 文档标注「需要连接7000端口」的接口，代码传的是主端口 —— `get_robot_state`（`:1041`）、`set_darg_mode`（`:1363`）、`get_drag_thread_is_end`（`:1380`）。改成 aux 属行为变更，先验证再动。

## 点位数组长度

- `MoveCmd::targetPosValue` 是 **14 位**：前 7 位机器人本体，后 7 位外部轴（外部轴从 `pos[7]` 开始）；默认构造已 `targetPosValue(14)`（`lib/include/cpp/parameter/tl_define.h:39,54`）。
- 填充规则（SDK 注释原文）：**几轴就填前几位，其余置 0**（`tl_interface.h:727` robot_movej、`:739` robot_movel）。
- SDK 点位出参长度固定为 **7**（`tl_interface.h:128`、`:136`）。本仓库代码依赖这一点：`publish_joint_pose` 在 6 轴时用 `begin()..end() - 1` 截断（`tl_driver.cpp:2776`），servol 插值位姿固定构造 7 元素（`tl_driver.cpp:2704`）。

## 运动参数范围与单位

| 参数 | 范围 / 单位 |
|---|---|
| J 运动 `velocity` | (0, 100]，单位 % |
| L 运动 `velocity` | (0, 1000]，单位 mm/s |
| 外部轴 L 运动 `velocity` | (1, 9999] |
| `acc` / `dec` | (0, 100] |
| `coord` | 0 关节 / 1 直角 / 2 工具 / 3 用户 |
| `open_servoJ` 的 `vmax` / `amax` / `jmax` | 度/秒、度/秒²、度/秒³ |
| `set_servoJ_pos` 的 `q` | 度 |

## 单位约定（跨层，最易踩坑）

| 通道 | 单位 | 依据 |
|---|---|---|
| `/joint_states.position` | **rad**（SDK 返回度，驱动内批量转换） | `tl_driver.cpp:2751-2757` |
| `/tcp_pose.position` | **mm**（SDK coord=1 原值直接透传，**不是** ROS 惯例的 m） | `tl_driver.cpp:2786-2798`；下游 `tl_teleop.cpp:366` 亦按 mm 处理 |
| `/tcp_pose.rpy`、`arm_angle` | rad | 同上 |
| `/tl_driver/set_servoj_pos` | **度**（订阅方 tl_hardware 自行做弧度↔角度换算） | `tl_driver.cpp:2537-2547` |
| `ServolMove.target_pose` | 位置 **mm** + 姿态 rad；`step_size` **mm**（驱动内默认 2.0，传入 ≤0 取默认） | `tl_driver.cpp:2630-2677` |
| `MoveCommand.target_pos_value` | 14 维：前 7 位本体位姿 [X **mm**, Y **mm**, Z **mm**, RX rad, RY rad, RZ rad, 冗余臂角 rad]，后 7 位为外部轴点位（单位随外部轴类型，本仓库不做换算、按 SDK 原值透传）；`coord=0` 时整组为关节角（度） | 实现 `tl_driver.cpp:2496`、`2527` 等 MoveCmd 组包点；布局依据 `lib/include/cpp/parameter/tl_define.h:64`、`test/test_moveL.sh` |
| `ToolParam.x/y/z`、`payload_mass_center_x/y/z` | **mm**；`a/b/c` 现文档标 度（SDK 头未标注，待现场核对）；`payload_mass` kg；`payload_inertia` kg·m² | `tl_driver.cpp:1424-1447` |
| `SetUserCoord.pos.position` | **mm** + 姿态 rad | `tl_driver.cpp:1454-1465` |
| `CoordTransform.origin_pos`/`reference_pos`/`target_pos` | 对应 `coord = 0` 为关节度；`coord = 1/2/3` 为位置 **mm** + 姿态 rad | `tl_driver.cpp:1749-1782`、SDK 头 `tl_interface.h:497-507` |
| `GetPosReachable.pos`、`SetGlobalPos.pos_info`、`GetGlobalPos.pos` | **14 维点位容器**：`[0]`坐标系 `[1]`单位制(0=度/1=弧度) `[2]`形态 `[3]`工具 `[4]`用户 `[5][6]`备用 `[7..13]`点位信息（笛卡尔时位置 **mm**、姿态 rad） | `tl_driver.cpp:1785`、`2168`、`2200`；SDK 头 `tl_interface.h:238-243`、`366-382` |
| `GetCurrentLineJointSpeed.line_speed` | **mm/s**（`joint_speed` 为度/s） | `tl_driver.cpp:2440-2463`、SDK 头 `tl_interface.h:648-653` |
| `MoveCommand.velocity`、`acc`/`dec`、`SetSpeed` | 关节运动为 %，直角（MOVL）运动速度为 **mm/s**，`acc`/`dec` 为 % | SDK 头 `tl_interface.h:731-746` |
| `RobotDHParam`（`l1..l20`、`pitch`、导程、喷料距离、`sp`/`tl`） | **mm**（控制器机构常数） | `tl_driver.cpp:1819-1970`、说明书 DH 章节 |
| `OpenServoJ.vmax/amax/jmax` | 度/s、度/s²、度/s³ | SDK 头 `tl_interface.h:920-925` |
| `GetPosTransform.input/output`、`GetCurrentMotorTorque`、`RobotJointParam` | 旋转量 rad / 无量纲；力矩 %；关节量度、度/s | `tl_driver.cpp:773-926`、`2410-2431`、`1086-1170` |

> 单位口径是**有意选择**：笛卡尔位置保留 SDK 原生的 **mm**（与示教器/现场手册一致），不随 ROS 惯例改为 m。因此集成方**不得**把某个字段按 m 解释；`msg/srv` 字段注释、`tl_ros2_interface/README.md` 与 `tl_driver服务与话题说明书.md` §1.4 现已逐字段标注。若要改为 m，属破坏性变更：须同时改 `tl_driver` 全部接口边界、三个消费方（tl_teleop / tl_teleop_f710 / tl_example）、文档与现场脚本，并单独发一个破坏性版本。
> 历史上说明书 §4.2 曾把 `/tcp_pose.position` 标为 m（与实现不符），已于 2026-09-22 修正为 mm（`d0fd361`）；§8.2 `set_user_coord` 同类问题于 `29b7841` 修正。

## ROS2 节点结构不变量

- `tl_driver` 用 `MultiThreadedExecutor` 驱动 **3 个回调组**：`service_group_`（全部服务）、`topic_group_`（4 个订阅）、`timer_group_`（状态发布定时器），三组**均须为 `MutuallyExclusive`**（`tl_driver.cpp:87-93`）。状态发布频率 `publish_rate_` 默认 100.0（`tl_driver.h:350`，周期计算 `tl_driver.cpp:446-448`）。历史遗留：`timer_group_` 曾用 `Reentrant` 导致定时器回调并发，勿改回。
- moveJ / moveL 话题回调**不等待到位**：只透传 SDK 返回值并记日志（`tl_driver.cpp:2498-2501`、`2529-2532`）。需要到位判定的一方自行轮询 `/arm_status`。
- `init()` 有两处「失败即退进程」：`connect()` 失败（`tl_driver.cpp:496-497`）与切入示教模式失败（`:474-476`），均 `rclcpp::shutdown()` + `exit(0)`，不带病运行。
