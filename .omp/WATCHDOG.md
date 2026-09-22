# Watchdog — tl_robot_ros2（dev 线）

给审查侧（advisor）的**检查清单**，不是实现文档。写法约定：只引用**文件与符号名**（行号会腐烂，一律不写）。
每条按「看到什么信号 → 去读哪里 → 确认什么」用。与代码冲突时以代码为准。

## 报与不报

| 级别 | 判据 | 例 |
|---|---|---|
| **blocker** | 真机会动错 / 进程崩 / 明确违反仓库硬规则 | 单位错导致千倍位移；长度契约破坏；动 `lib/` 下第三方产物 |
| **concern** | 契约单边修改、消费方或文档未同步、并发与生命周期缺陷 | 改了 `/tcp_pose` 单位没改 `tl_teleop`；launch 参数改了没改 README |
| **nit** | 其余可维护性问题 | 注释与代码不符 |

**不要报**：格式与 lint（CI 已管）、命名口味、行号/短哈希是否精确、纯风格重构意见、本次改动没碰到的既有问题、"建议补测试"（本仓库无自动化测试脚手架，只有 ament lint）。

**纪律**：提之前先 `read`/`grep` 落到具体文件与符号，拿不到证据就不说；同一问题别换措辞重提。

## 一、通用准则（任何改动先过这 6 条）

| 准则 | 问法 | 典型错误 |
|---|---|---|
| 对称性 | 改了这个字段，**另一端**改了吗？ | 只改发布方单位，消费方照旧 |
| 契约 | 长度、单位、坐标系、下标语义定义清楚了吗？ | 7 元素容器塞 6 个值 |
| 生命周期 | 连/断、开/关、上电/下电、注册/注销成对吗？ | 漏 `close_servoJ`；连接失败仍继续跑 |
| 并发 | 跑在哪个线程/回调组？会阻塞吗？共享状态有保护吗？ | 回调里做阻塞调用；控制循环里 future 析构 |
| 默认值 | 新参数有默认值吗？YAML/launch 同步了吗？ | 声明了参数但配置文件里没有 |
| 文档 | `AGENTS.md`「提交前必做」的文档同步逐条过了吗？ | 接口改了 `CHANGELOG.md` 没动 |

## 二、单位与量纲（本仓库第一号坑）

口径：**ROS 侧 m / rad；控制器与服务接口 mm / 度 / %**。换算发生在驱动内或各消费方，改任何一端都要同时看另一端。

| 通道 | 单位 | 位置 |
|---|---|---|
| `/joint_states.position` | rad（SDK 返回度，驱动内批量转） | `publish_joint_pose` |
| `/tcp_pose.position` | m（`kMmToM`）；`rpy` rad | `publish_tcp_pose` |
| `/tl_driver/set_servoj_pos` | **度**（原值透传 aux） | `handle_set_servoj_pos_topic` |
| `/tl_driver/set_servol_pos.target_pose` | 位置 mm + 姿态 rad；`step_size` mm | `handle_set_servol_pos_topic` |
| `MoveCommand.target_pos_value` | 原值透传：`coord=0` 度；`1/2/3` 为 `[X,Y,Z mm, RX,RY,RZ rad]` | `handle_movej_topic` / `handle_movel_topic` |
| `ToolParam` 的 `x/y/z`、`a/b/c`、`payload_mass` | mm / 度 / kg（单位注释在 SDK `tl_types.h`） | `handle_set_tool_param_service` |
| `SetUserCoord.pos.position` | mm + 姿态 rad（原值透传） | `handle_set_user_coord_service` |

消费方换算（改单位必须同步这一组）：

- `tl_teleop`：`/tcp_pose`(m) ×`kMToMm` → coord_transform `origin_pos`(mm)；VR 位移 ×1000
- `tl_teleop_f710_node`：KDL FK(m) ×1000 → `ServolMove.target_pose`(mm)；`/joint_states`(rad) ×180/π → `set_servoj_pos`(度)
- `tl_teleop_f710_sim_bridge`：servol(mm) ÷1000 → KDL(m)
- `tl_hardware_interface`：ros2_control(rad) ×180/π → `set_servoj_pos`(度)

**信号**：出现硬编码 `1000.0` / `180.0 / M_PI`、变量名带 `mm` 却参与 m 运算、新增跨层字段没有单位注释 → 逐一核对该通道两端。

**先例**：`/tcp_pose` 的 m↔mm 误用曾让遥操作 IK 目标点错 1000 倍，控制器报 **9754「目标位置不可达」**（`2c5b2e4`）。该链路出现"运动到离谱位置/报 9754"时，先怀疑单位。

## 三、长度与下标契约

- 关节向量的 6 与 7：`ndof_`（默认 6）、`arm_joints_`；6 轴时第 7 元素**补 0，不是丢掉**。
- `MoveCmd::targetPosValue` 14 位：前 7 本体 + 后 7 外部轴，几轴填几位、其余置 0。
- `get_current_position`（Coord 重载）返回 7 元素；6/7 轴分支分散在 `publish_joint_pose`、`publish_tcp_pose`。
- 点位容器 14 位：`[0]`坐标系 `[1]`单位 `[2]`形态 `[3]`工具 `[4]`用户 `[5][6]`备用 `[7..13]`点位（`set_global_position`、`get_pos_reachable`）。
- **信号**：`resize(7, 0.0)`、`end() - 1` 截断、`size() < 6` 早退 → 核对调用方实际长度。

## 四、线程与回调组

- `tl_driver` 三组均为 `MutuallyExclusive`：`service_group_`（全部 65 个服务）、`topic_group_`（4 个订阅）、`timer_group_`（状态发布定时器）；`MultiThreadedExecutor` 线程数 `max(4, hardware_concurrency)`。`timer_group_` 曾用 `Reentrant` 导致状态发布回调并发，`4757255` 改为互斥，**别改回**。
- 回调里不做阻塞：SDK 同步调用、`sleep`、等 future 都占住 executor 线程。
- 控制循环（f710 250Hz、`tl_teleop` 100Hz）：循环体不得超周期；**控制循环内 `std::async` 的 future 析构会阻塞到任务完成**，反复给同一 future 赋值 = 隐性停顿；跨线程标志用 `std::atomic`。
- 跨线程共享的 `std::vector`/`std::string`（如 `target_pose_`、`latest_joy_`）需锁或原子快照，禁止裸读写。

## 五、真机安全（最高优先级，宁误报不漏报）

任何让机械臂动起来的改动都要问：

- 速度/加速度上限是否被绕过（J 百分比、L mm/s）？增量是否在发指令前 clamp？
- 奇异点、关节跳变检测、IK 失败分支是否仍生效（`tl_teleop` 的 jump 检测、f710 的 IK 失败处理）？
- 失败路径：IK 失败 / SDK 非成功 / 超时 → **停止保持**，还是继续用旧值或零值发运动指令？后者 = blocker。
- 上电与示教时序：`is_powered_`、示教模式切换失败必须退进程（`init()` 的两处 `rclcpp::shutdown()`），不带病运行。
- 单位错在真机等价于千倍位移 —— 单位问题一律按 blocker 报。

## 六、错误处理

- SDK 返回码（`Result::SUCCESS` 一类）是否检查？`false`/负值是否被当成成功？
- try/catch 后是否只打日志就继续跑（吞错）？
- `rclcpp::shutdown()` 或节点析构之后，是否还有代码用 `this->`、继续发消息或调 SDK？
- 连接/断开、`open_servoJ`/`close_servoJ`、线程 `join` 是否成对？

## 七、双端口与连接生命周期

| 端口 | 参数 | 职责 |
|---|---|---|
| 6001 | `arm_port` | 请求/响应式 SDK 调用（运动、IO、Modbus、作业、参数查询、外部轴、拖拽示教） |
| 7000 | `arm_port_aux` | servoJ 全系列、伺服点位、**机器人状态异步推送**、错误/告警回调 |

- **两个 fd 都必须连上**：`connect()` 两次 `connect_robot`，任一侧 `<= 0` 即失败；`is_connected()` 另要求两个 fd 均 > 0。
- 状态推送只在 aux 注册：`robot_state_callback(socket_fd_aux_, robot_state_callback_handler)`；旧 `recv_message` 机制已废弃，别加回主端口。
- 错误/告警回调**两个端口都注册**（`set_receive_error_or_warnning_message_callback` 各一次）——别以「状态是 7000 的事」为由删掉主端口那次。
- ⚠️ 已知未修正项：`set_darg_mode`、`get_drag_thread_is_end` 传的是**主端口**（SDK 文档要求 7000）。**不要顺手「统一到 aux」**，属行为变更，需真机验证。

## 八、文档与仓库卫生（清单在 `AGENTS.md`，这里只留该盯的动作）

主会话上下文里已有 `AGENTS.md`（含「提交前必做」清单与硬约束），别复述其内容，只盯这几类漏做：

- 改了接口/launch/YAML/行为，却**只有代码没有文档**（`CHANGELOG.md`、包 `README.md`、说明书）→ concern。
- 动了 `src/*/lib/` 下的专有库或 SDK 头 → blocker（只读产物）。
- 改了 msg/srv 没重建、或没跑 `./scripts/format-cpp.sh` / `black .` → concern（CI 只查格式、不编译）。
- `CHANGELOG.md` 写成内部日志（构建配置、CI、`.omp/**`、目录重命名）→ nit；条目标准见 `.omp/rules/changelog-user-visible.md`。
