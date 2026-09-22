# Watchdog — tl_robot_ros2（V2 维护线）

给审查侧（advisor）的**检查清单**，不是实现文档。写法约定：只引用**文件与符号名**（行号会腐烂，一律不写）。
每条按「看到什么信号 → 去读哪里 → 确认什么」用。与代码冲突时以代码为准。

> 本文件按 **V2 线**校准：笛卡尔位置一律 **mm**（有意偏离 ROS 惯例 m）。`dev`(3.x) 线的 `/tcp_pose.position` 与
> servol 输入是 **m** —— 跨分支合并时以目标分支的消费方代码为准，**不要照搬**。

## 报与不报

| 级别 | 判据 | 例 |
|---|---|---|
| **blocker** | 真机会动错 / 进程崩 / 明确违反仓库硬规则 | 单位错导致千倍位移；长度契约破坏；动 `lib/` 下第三方产物 |
| **concern** | 契约单边修改、消费方或文档未同步、并发与生命周期缺陷 | 把 `/tcp_pose` 的 mm 改成 m 只改了一端；launch 参数改了没改 README |
| **nit** | 其余可维护性问题 | 注释与代码不符 |

**不要报**：格式与 lint（CI 已管）、命名口味、行号/短哈希是否精确、纯风格重构意见、本次改动没碰到的既有问题、"建议补测试"（本仓库无单元测试，只有 ament lint 与需真机的手工脚本）。

**纪律**：提之前先 `read`/`grep` 落到具体文件与符号，拿不到证据就不说；同一问题别换措辞重提。

## 一、通用准则（任何改动先过这 6 条）

| 准则 | 问法 | 典型错误 |
|---|---|---|
| 对称性 | 改了这个字段，**另一端**改了吗？ | 只改发布方单位，消费方照旧 |
| 契约 | 长度、单位、坐标系、下标语义定义清楚了吗？ | 两套 14 维布局混用 |
| 生命周期 | 连/断、开/关、上电/下电、注册/注销成对吗？ | 漏 `close_servoJ`；连接失败仍继续跑 |
| 并发 | 跑在哪个线程/回调组？会阻塞吗？共享状态有保护吗？ | 回调里做阻塞调用；控制循环里 future 析构 |
| 默认值 | 新参数有默认值吗？YAML/launch 同步了吗？ | 声明了参数但配置文件里没有 |
| 文档 | `AGENTS.md`「提交前必做」的文档同步逐条过了吗？ | 接口改了 `CHANGELOG.md` 没动 |

## 二、单位与量纲（本仓库第一号坑）

口径：**笛卡尔位置 mm**（SDK `coord=1` 原值透传，有意不用 ROS 惯例的 m）、姿态 rad、关节量度（`/joint_states` 是唯一换算点，度→rad）。

| 通道 | 单位 |
|---|---|
| `/joint_states.position` | rad（驱动内度→弧度） |
| `/tcp_pose.position` | **mm**（原值透传，**不是** m）；`.rpy`/`arm_angle` rad |
| `/tl_driver/set_servoj_pos` | 度（7 元素，末位补 0） |
| `/tl_driver/set_servol_pos` | `target_pose` 位置 mm + 姿态 rad；`step_size` mm（≤0 → 2.0） |
| `MoveCommand.target_pos_value` | `coord=0` 整组关节角（度）；`1/2/3` 为 mm + 姿态 rad |
| `set_user_coord` / `set_tool_param` | 位置 mm + 姿态 rad（`tool_param` 的 `a/b/c` 度） |

消费方（改单位必须同步这一组）：`tl_teleop` 按 mm 直用 `/tcp_pose`（**不要**引入 ×1000）；`tl_hardware_interface` 只做 rad↔度（`* 180/π`）后发 `set_servoj_pos`；`tl_teleop_f710` 的 ×1000 是**手柄位移 m → 机械臂 mm**，与话题单位无关。

**信号**：出现 ×1000/÷1000、`position` 被按 m 解释、msg/srv 字段注释未标单位 → 逐条核对该通道两端。
**先例**：把位置 `327.47`（mm）按 m 解释，IK 直接报 **9754「目标位置不可达」**；历史上说明书曾把 `/tcp_pose.position` 标为 m，属文档缺陷，已修正 —— 别改回去。

## 三、长度与下标契约

- 关节/位姿向量第 7 位对 6 轴**补 0，不是截断**；`publish_joint_pose` 等 6/7 轴分支要同步改。
- `MoveCmd::targetPosValue` **14 位**：前 7 本体 + 后 7 外部轴，几轴填几位、其余置 0。
- **两套 14 维布局必须区分**：`MoveCommand.target_pos_value`（`[0..6]` 本体位姿 + `[7..13]` 外部轴，无头部）vs 点位容器（`GetPosReachable.pos`、`Set/GetGlobalPos.pos_info`：`[0]`坐标系 `[1]`单位制 `[2]`形态 `[3]`工具 `[4]`用户 `[5][6]`备用 `[7..13]`点位）。
- **信号**：`resize(7, 0.0)`、`end() - 1` 截断、`size() < 6` 早退 → 核对调用方实际长度。

## 四、线程与回调组

- `tl_driver` 三组**均为 `MutuallyExclusive`**：`service_group_`（全部 65 个服务）、`topic_group_`（4 个订阅）、`timer_group_`（100 Hz 状态发布）；`MultiThreadedExecutor` 线程数 `max(4, hardware_concurrency)`。`timer_group_` 曾用 `Reentrant` 造成定时器回调并发，已改互斥，**勿改回**。
- 回调里不做阻塞：SDK 同步调用、`sleep`、等 future 都占住 executor 线程。
- 控制循环（f710 250 Hz、`tl_teleop` 100 Hz）：循环体不得超周期；**控制循环内 `std::async` 的 future 析构会阻塞到任务完成**，反复给同一 future 赋值 = 隐性停顿；跨线程标志用 `std::atomic`。
- 跨线程共享的 `std::vector`/`std::string`（如 `target_pose_`、`latest_tcp_pose_`）需锁或原子快照，禁止裸读写；`tl_hardware` 的 `read()` 在超时未收状态时只节流告警，不自动 shutdown（由上层控制器处理）—— 这属既定设计，别当缺陷报。

## 五、真机安全（最高优先级，宁误报不漏报）

任何让机械臂动起来的改动都要问：

- 速度/加速度上限是否被绕过（J 百分比、L mm/s）？增量是否在发指令前 clamp（`max_pos_delta_mm` 一类）？
- 奇异点、关节跳变检测、IK 失败分支是否仍生效？
- 失败路径：IK 失败 / SDK 非成功 / 超时 → **停止保持**，还是继续用旧值或零值发运动指令？后者 = blocker。
- 上电与示教时序：真机模式必须 `connect_arm → power_on → set_current_mode(2) → set_speed → open_servoj`，退出反向收尾；`tl_driver::init()` 连接失败或切入示教失败即 `rclcpp::shutdown()` + exit，不带病运行。
- `tl_hardware::on_activate()` 等首帧关节状态超时 5 s 必须返回 `CallbackReturn::ERROR`（激活失败），且用当前关节角播种命令接口 —— 改掉会先下发零位把机械臂拉向零点。
- 单位错在真机等价于千倍位移 —— 单位问题一律按 blocker 报。

## 六、错误处理

- SDK 返回码是否检查？`false`/负值是否被当成成功？
- try/catch 后是否只打日志就继续跑（吞错）？
- `rclcpp::shutdown()` 或节点析构之后，是否还有代码用 `this->`、继续发消息或调 SDK？
- 连接/断开、`open_servoJ`/`close_servoJ`、`open_servoj`/`close_servoj` 服务、线程 `join` 是否成对？

## 七、双端口与连接生命周期

| 端口 | 参数 | 职责 |
|---|---|---|
| 6001 | `arm_port` | 请求/响应式 SDK 调用（运动、IO、Modbus、作业、参数查询、外部轴运动） |
| 7000 | `arm_port_aux` | servoJ/servoP 全系列、伺服点位、独立轴、碰撞检测参数、拖拽示教、工具坐标范围与示教灵敏度、**机器人状态异步推送** |

- **两个 fd 都必须连上**：`is_connected()` 要求两者均 > 0。
- 状态推送只在 aux 注册：`recv_message(socket_fd_aux_, robot_state_recv_callback)` —— 别加回主端口。
- 错误/告警回调**两个端口都注册**（`set_receive_error_or_warnning_message_callback` 各一次）—— 别以「状态是 7000 的事」为由删掉主端口那次。
- ⚠️ 已知未修正项：`get_robot_state`、`set_darg_mode`、`get_drag_thread_is_end` 传的是**主端口**（SDK 文档要求 7000）。**不要顺手「统一到 aux」**，属行为变更，需真机验证。

## 八、文档与仓库卫生（清单在 `AGENTS.md`，这里只留该盯的动作）

主会话上下文里已有 `AGENTS.md`（含「提交前必做」清单与硬约束），别复述其内容，只盯这几类漏做：

- 改了接口/launch/YAML/行为，却**只有代码没有文档**（`CHANGELOG.md`、`tl_driver服务与话题说明书.md`、`tl_ros2_interface/README.md`、包 `README.md`）→ concern。
- 动了 `src/*/lib/` 下的专有库、Python 封装或 SDK 头 → blocker（只读产物）。
- 改了 msg/srv 没重建、或没跑 `./scripts/format-cpp.sh` / `black .` → concern（CI 只查格式、不编译）。
- 把 V2 的口径（mm）与 dev 线（m）混用 → blocker（跨分支合并最常见的错）。
- `CHANGELOG.md` 写成内部日志（构建配置、CI、`.omp/**`、目录重命名）→ nit；条目标准见 `.omp/rules/changelog-user-visible.md`。
