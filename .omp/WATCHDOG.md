# Watchdog — tl_robot_ros2（dev 线）

控制器协议与单位约定。这些不变量横跨 `tl_driver` / `tl_teleop` / `tl_teleop_f710` / `tl_hardware`，
改动相关代码前先对照本节。本节只记录**代码里已实现、但容易被改错或改回去**的约定；
工作空间结构、命名规范、构建命令见 `AGENTS.md`，接口逐条说明见 `src/tl_driver/doc/tl_driver服务与话题说明书.md`。
本节与代码冲突时**以代码为准**，并同步修正文档。

> 本文件按 **dev 线（3.x）** 代码校准。`V2` 维护线的口径**不同**（该线 `/tcp_pose.position` 与 servol 输入一律保留 SDK 原生 mm，
> 且 SDK 头目录是 `lib/include/cpp/...`）—— 跨分支合并时以目标分支的消费方代码为准，勿照搬本文件。

## 双端口连接

| 端口 | 参数 | 职责 |
|---|---|---|
| 6001 | `arm_port` | 主端口：请求/响应式 SDK 调用（运动、IO、Modbus、作业、参数查询、外部轴运动、拖拽示教） |
| 7000 | `arm_port_aux` | 辅助端口：servoJ 全系列、伺服点位运动、**机器人状态的异步推送**、错误/告警回调 |

- 两个端口都必须连上：`socket_fd_ = connect_robot(arm_ip_, arm_port_)`（`tl_driver.cpp:590`）、
  `socket_fd_aux_ = connect_robot(arm_ip_, arm_port_aux_)`（`:591`）；`is_connected()` 另要求两个 fd 均 > 0（`:502-505`）。
- 端口参数声明与读取：`arm_port_aux` 默认 `"7000"`（`tl_driver.cpp:75`、`:82`）。
- servoJ 一律走 aux：`open_servoJ`（`:2229`）、`close_servoJ`（`:2245`）、`set_servoJ_pos`（`:2488`）。
- **状态推送回调只在 aux 注册**：新 SDK 用 `robot_state_callback(socket_fd_aux_, robot_state_callback_handler)`
  （`:615-616`），旧的 `recv_message` 机制已废弃，别照旧版本写法加回主端口。
- **错误/告警回调两个端口都注册**（`:611` 主端口 + `:613` aux）。该接口本身不属 7000 段，
  别因为「状态是 7000 端口的事」而删掉主端口那次注册。
- ⚠️ **仍与 SDK 文档不一致（未修正，改动前需真机验证）**：SDK 标注「需要连接 7000 端口」的
  `set_drag_mode`（`tl_driver.cpp:1366`）与 `get_drag_thread_is_end`（`:1383`）传的是**主端口**。
  同类问题中的 `get_robot_state` 已改走 aux（`:1048-1049`），可作参照；改成 aux 属行为变更，先验证再动。
- SDK 侧端口约定：`tl_interface.h:233-236`（7000 端口查询状态）、`:416-419`（7000 端口状态回调）、
  `tl_servo_ext.h:22-28`（servo 扩展需先连 7000，与 close 成对调用）。

## 点位数组长度

- `MoveCmd::targetPosValue` 默认 **14 位**：前 7 位机器人本体，后 7 位外部轴
  （`tl_types.h:183` 字段注释、`:199` 默认构造 `MoveCmd() : targetPosValue(14)`）。
- 填充规则（SDK 注释原文）：**几轴就填前几位，其余置 0**（`tl_interface.h:429` robot_movej、`:442` robot_movel）。
- `get_current_position`（Coord 重载）返回 **7 元素**：`Coord::JOINT` 为关节角（度）；
  `Coord::BASE/TOOL/USER` 为 `[X,Y,Z,RX,RY,RZ]`（mm, rad）（`tl_interface.h:518-522`）。
  驱动依赖这一点：`publish_joint_pose` 6 轴时截断 `end() - 1`（`tl_driver.cpp:2712-2728`），
  servol 插值位姿固定构造 7 元素（`:2626`）。
- GP 点位与可达性查询的容器同样是 **14 位**：`[0]`坐标系 `[1]`0=度/1=弧度 `[2]`形态 `[3]`工具
  `[4]`用户 `[5][6]`备用 `[7..13]`点位信息（`tl_interface.h:568-574` set_global_position、`:959-971` get_pos_reachable）。

## 运动参数范围与单位

| 参数 | 范围 / 单位 | 依据 |
|---|---|---|
| J 运动 `velocity` | (0, 100]，% | `tl_interface.h:430` |
| L 运动 `velocity` | (0, 1000]，mm/s | `tl_interface.h:443` |
| `acc` / `dec` | (0, 100] | `tl_interface.h:432-433`、`:445-446` |
| `coord` | 0 关节 / 1 直角 / 2 工具 / 3 用户 | 各服务入参校验，如 `tl_driver.cpp:1782-1794` |
| `open_servoJ` 的 `vmax`/`amax`/`jmax` | 7 元素向量，度/秒、度/秒²、度/秒³；6 轴第 7 元素补 0 | `tl_servo_ext.h:133-143` |
| `set_servoJ_pos` 的 `q` | 7 元素向量，度；与上同长度契约 | `tl_servo_ext.h:154-161` |

## 单位约定（跨层，最易踩坑）

dev 线的口径：**ROS 侧用 ROS 惯例（m / rad），SDK 与服务接口用控制器原生（mm / 度 / %）**，
换算发生在驱动内或各消费方。改任何一个接口前先看下表，别只改一端。

| 通道 | 单位 | 依据 |
|---|---|---|
| `/joint_states.position` | **rad**（SDK 返回度，驱动内批量转换） | `tl_driver.cpp:2695-2701` |
| `/tcp_pose.position` | **m**（驱动 mm→m，`kMmToM`） | `tl_driver.cpp:2732-2741` |
| `/tcp_pose.rpy`、`arm_angle` | rad | `tl_driver.cpp:2743-2750` |
| `/tl_driver/set_servoj_pos` | **度**（订阅回调原值透传 aux） | `tl_driver.cpp:2479-2490` |
| `/tl_driver/set_servol_pos` 的 `target_pose` | 位置 **mm** + 姿态 rad；`step_size` **mm**（传入 ≤0 取默认 2.0） | `tl_driver.cpp:2600-2617` |
| `MoveCommand.target_pos_value` | **原值透传**：`coord=0` 关节角（度）；`coord=1/2/3` 为 `[X mm, Y mm, Z mm, RX/RY/RZ rad]` | `tl_driver.cpp:2438-2440`、`:2469-2471` |
| `ToolParam.x/y/z`、`payload_mass_center_*` | **mm**；`a/b/c` **度**；`payload_mass` kg | `tl_types.h:202-216` |
| `SetUserCoord.pos.position` | **mm** + 姿态 rad（原值透传） | `tl_driver.cpp:1468-1473` |
| `CoordTransform.origin_pos`/`reference_pos`/`target_pos` | 按 coord 语义：0 关节度；1/2/3 位置 **mm** + 姿态 rad | `tl_driver.cpp:1800-1802` |
| `RobotDHParam` | `alpha`/`theta` **deg**、`a`/`d` **mm**、`mountingAngle` **deg** | `tl_types.h:225-233` |
| `GetCurrentLineJointSpeed.line_speed` / `joint_speed` | **mm/s** / **度/s** | `tl_interface.h:1281-1284` |
| `GetCurrentMotorTorque.motor_torque` | **%**（本体 7 元素 + 外部轴 5 元素） | `tl_interface.h:1255-1257` |
| `GetQuat2Rpy` 的 `rpy` | **rad**（本地实现为 XYZ 外旋） | `tl_interface.h:668-671`、`:710-714` |
| `GetPosReachable.pos`、`Set/GetGlobalPos.pos_info` | 14 位点位容器（见上节） | `tl_interface.h:959-971`、`:568-574` |

消费方换算（改单位时必须同步这一组）：

- `tl_teleop`：`/tcp_pose`(m) → 逆解请求(mm)，×1000（`tl_teleop.cpp:379-384`）
- `tl_teleop_f710`：FK 结果(m) ×1000 → servol `target_pose`(mm)（`tl_teleop_f710_node.cpp:331-333`、`:528-530`）；
  `/joint_states`(rad) → `set_servoJ_pos`(度)（`:453-457`）
- `tl_teleop_f710_sim_bridge`：servol(mm) → KDL(m)，÷1000（`tl_teleop_f710_sim_bridge.cpp:185-188`）

> 本线 `.msg/.srv` 字段**尚未标注单位**（现状），逐字段口径以本节为准；改接口时顺手补注释。

## ROS2 节点结构不变量

- `tl_driver` 用 `MultiThreadedExecutor` 驱动 **3 个回调组**：`service_group_`（全部服务）、`topic_group_`（4 个订阅）、
  `timer_group_`（状态发布定时器），三组**均须为 `MutuallyExclusive`**（`tl_driver.cpp:88`、`:90`、`:94`）。
  历史遗留：`timer_group_` 曾用 `Reentrant` 导致定时器回调并发，勿改回。
- 发布：`/joint_states`（`:425`）、`/tcp_pose`（`:427`）、`/arm_status`（`:429`）；
  订阅：moveJ（`:432`）、moveL（`:435`）、set_servoj_pos（`:438`）、set_servol_pos（`:442`）。
- 发布频率 `publish_rate_` 默认 100.0（`tl_driver.h:351`），定时器周期计算 `tl_driver.cpp:446-449`。
- moveJ / moveL 话题回调**不等待到位**：只透传 SDK 结果并记日志（`tl_driver.cpp:2440-2442`、`:2471-2473`）。
  需要到位判定的一方自行轮询 `/arm_status`。
- `init()` 有两处「失败即退进程」：连接失败（`tl_driver.cpp:497`）与切入示教模式失败（`:476`），
  均 `rclcpp::shutdown()` + exit，不带病运行。
- 轴数 `ndof_` 默认 6、关节名 `arm_joints_` 由配置注入（`tl_driver.h:350-352`）；
  6/7 轴分支集中在 `publish_joint_pose`（`tl_driver.cpp:2712`）与 `publish_tcp_pose`（`:2730`）。
