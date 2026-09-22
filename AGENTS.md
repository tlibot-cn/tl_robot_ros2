# AGENTS.md — 天链机器人 ROS2 工作空间（V2 维护线）

**本文件在会话启动时注入 agent 上下文，只写两类内容：不知道就会做错的硬约束、找东西的地图。**
使用方式（启动命令、参数配置）看 `README.md`；逐包结构与用途看 `src/README.md` 和各包 `README.md`；
接口逐条说明看 `src/tl_driver/doc/tl_driver服务与话题说明书.md`。文档与代码冲突时以代码为准，并同步修正文档。

> **本线为 V2 独立版本线**：接口口径与 `dev`(3.x) 线**不同** —— 笛卡尔位置是本仓库**有意保留的 mm**（dev 线为 m）。
> 跨线合并代码或文档时，先确认目标分支的消费方与接口口径，**不要照搬**。

## 项目速览

- `src/` 下 10 个顶层目录、共 **23 个功能包**（`package.xml`，其中 `tl_moveit2_config/` 含 14 个型号子包）+ `scripts/` 工具脚本；ROS2 Humble，C++17 / Python，colcon（ament_cmake + ament_python），无前端/Node.js。
- 构建：`colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` → `source install/setup.bash`（产物 `build/`、`install/`、`log/` 已 gitignore）。选择性构建必须先建 `tl_ros2_interface`（生成 msg/srv 头文件）。
- **无单元测试**。验证面两层：`ament_lint_auto` 代码风格检查（`tl_bringup`/`tl_description`/`tl_gazebo`/`tl_teleop`/`tl_teleop_f710` 的 `CMakeLists.txt` 中启用），以及 `src/tl_driver/test/` 下需真机在线的手工脚本（`test_moveJ.sh`、`test_moveL.sh`、`test_job_insert_*.sh`、`test_publisher.py`）。不要以"补测试"充当验证，验证靠编译 + 实机/仿真运行。
- 开发环境在 Docker 内（配置不在本仓库）。

| 包 | 作用 | 关键入口 |
| --- | --- | --- |
| `tl_ros2_interface` | 全部自定义接口（12 个 `.msg`、45 个 `.srv`，字段带单位注释） | `msg/`、`srv/`；**必须最先构建** |
| `tl_driver` | 机械臂驱动，TCP 连控制器（65 服务 / 4 订阅 / 3 发布） | `src/tl_driver.cpp`（约 2850 行单文件）、`include/tl_driver/tl_driver.h` |
| `tl_teleop` | VR 遥操作（PXREA Robot SDK，100 Hz 控制循环） | `src/tl_teleop.cpp` |
| `tl_teleop_f710` | F710 手柄遥操作（250 Hz）+ Gazebo 仿真桥接 | `src/tl_teleop_f710_node.cpp`、`src/tl_teleop_f710_sim_bridge.cpp` |
| `tl_hardware` | ros2_control 硬件插件，接 `joint_trajectory_controller` ↔ `tl_driver` | `src/tl_hardware_interface.cpp`（`tl_hardware::TLHardwareInterface`） |
| `tl_description` | URDF + STL 网格 + RViz（纯数据包，无编译代码） | `urdf/`、`meshes/`、`rviz/` |
| `tl_bringup` | 启动聚合（`tl_driver` + `tl_description`） | `launch/`（14 种臂型各一个） |
| `tl_gazebo` | Gazebo 仿真 | `launch/`、`config/` |
| `tl_moveit2_config` | MoveIt2 配置集合（14 个子包） | `tl_<arm_type>_config/` |
| `tl_example` | 示例节点（当前仅 `medical_demo`：检验科队列 MoveL） | `src/medical_demo.cpp` |

依赖（按 `package.xml`）：`tl_driver` / `tl_teleop` / `tl_teleop_f710` / `tl_hardware` / `tl_example` 依赖 `tl_ros2_interface`（须先构建）；`tl_teleop` / `tl_teleop_f710` / `tl_hardware` 另声明 `exec_depend: tl_driver`（运行期走 `/tl_driver/*` 话题与服务，不链接其库）；`tl_gazebo` 与各 `tl_<arm>_config` 声明 `tl_description`（后者另含 `tl_hardware`）；`tl_bringup` 只组合 launch，不声明包依赖。

## 硬约束（改代码前先读）

**① 专有库只读。** `src/tl_driver/lib/`（`arm/`、`x86/`：`_tl_host.so`、ARM 侧外围 `libtl_host.so`/`libservoJ_wrapper.so`/`libmodbus_wrapper.so`/`libmath_wrapper.so`、Python 封装 `tl_interface.py`、头文件 `include/c/`+`include/cpp/`）与 `src/tl_teleop/lib/`（`libPXREARobotSDK.so`）都是预编译第三方产物 —— 不改、不格式化（`scripts/format-cpp.sh` 已跳过 `lib/include/`）。CMake 按 `CMAKE_SYSTEM_PROCESSOR` 选目录，其他架构直接 `FATAL_ERROR`；C++ 只链接 `_tl_host.so`（`IMPORTED tl_host_lib`），安装时另建 `libnrc_host.so` 符号链接兼容旧 SONAME。

**② 单位口径（本线最易踩坑，且与 dev 线相反）。** 笛卡尔位置一律 **mm**（SDK `coord=1` 原值透传，**有意偏离 ROS 惯例的 m**）；姿态 rad；关节量度（`/joint_states` 是唯一换算点，SDK 度 → ROS rad）。

| 通道 | 单位 |
| --- | --- |
| `/joint_states.position` | rad（驱动内度→弧度） |
| `/tcp_pose.position` | **mm**（驱动原值透传，**不是** m）；`.rpy`/`arm_angle` rad |
| `/tl_driver/set_servoj_pos` | 度（7 元素，末位补 0） |
| `/tl_driver/set_servol_pos` | `target_pose` 位置 **mm** + 姿态 rad；`step_size` **mm**（≤0 → 2.0） |
| `MoveCommand.target_pos_value` | 原值透传：`coord=0` 整组关节角（度）；`coord=1/2/3` 为 mm + 姿态 rad |
| `set_user_coord` / `set_tool_param` | 位置 mm + 姿态 rad（`tool_param` 的 `a/b/c` 按度记录，但 SDK 头未标注、**待现场核对**） |

消费方**不要**自行 ×1000/÷1000：`tl_teleop` 直接按 mm 使用 `/tcp_pose`；`tl_hardware` 只做 rad↔度（`rad * 180/π` 后发 `set_servoj_pos`）；`tl_teleop_f710` 的 ×1000 是**手柄位移 m → 机械臂 mm**，与话题单位无关。
把 mm 当 m 会让 IK 直接失败（历史问题：位置 `327.47` 按 m 解释时报 **9754 目标位置不可达**）。欧拉角约定为 XYZ 内旋（scipy 中用大写 `'XYZ'`）。

**③ 长度契约。** 关节/位姿向量第 7 位对 6 轴**补 0**，不是截断（`publish_joint_pose` 用 `end() - 1`）；`MoveCmd::targetPosValue` 与全局点位容器都是 **14 位**，但**两套布局必须区分**：`target_pos_value` 是 `[0..6]` 本体位姿 + `[7..13]` 外部轴（无头部）；点位容器是 `[0]`坐标系 `[1]`单位制 `[2]`形态 `[3]`工具 `[4]`用户 `[5][6]`备用 `[7..13]`点位。

**④ 双端口。** 6001 `arm_port`：请求/响应式 SDK 调用；7000 `arm_port_aux`：servoJ/servoP 全系列、伺服点位、独立轴、碰撞检测参数、拖拽示教、工具坐标范围与示教灵敏度，以及**机器人状态的异步推送**。两个 fd 都必须连上（`is_connected()` 要求均 > 0）；状态推送只在 aux 注册（`recv_message(socket_fd_aux_, …)`），错误/告警回调**两个端口都注册**。
⚠️ 已知未修正项：`get_robot_state`、`set_darg_mode`、`get_drag_thread_is_end` 三个 SDK 文档标注需要 7000 端口的接口，代码传的是**主端口**。改成 aux 属行为变更，须真机验证后再动，**不要顺手改**。

**⑤ 线程模型。** `tl_driver` 三组回调组 `service_group_` / `topic_group_` / `timer_group_` **均为 `MutuallyExclusive`**（同组串行），由 `MultiThreadedExecutor` 驱动（线程数 `max(4, hardware_concurrency)`）；状态发布定时器 100 Hz（`publish_rate_`，硬编码 100.0）。`timer_group_` 曾为 `Reentrant` 导致定时器回调并发，已改互斥，**勿改回**。

**⑥ 失败即退与激活失败即报错。** `TL_Arm::init()` 中 `connect()` 失败、切入示教模式失败均 `rclcpp::shutdown()` + `exit(0)`；`tl_hardware` 的 `on_activate()` 等首帧关节状态超时 5 s 返回 `CallbackReturn::ERROR`（**激活失败，不是仅告警**），并用当前关节角播种命令接口，避免控制器接管前把机械臂拉向零点。

**⑦ 硬编码边界。** IP / 端口 / 关节名只出现在 `config/*.yaml`（默认 `192.168.1.13:6001`），不写进代码。`arm_type` 在配置 YAML 中大写（`TCB605`），启动参数小写（`tcb605`），全部 14 种。

## 关键话题

| 话题 | 方向与单位 |
| --- | --- |
| `/joint_states` | `tl_driver` 发（rad）；`tl_description`、`tl_hardware`、`tl_teleop_f710` 收 |
| `/tcp_pose` | `tl_driver` 发（**mm** / rad）；`tl_teleop` 收 |
| `/arm_status` | `tl_driver` 发；`tl_example` 收 |
| `/tl_driver/moveJ`、`/tl_driver/moveL` | `tl_driver` 收（`MoveCommand`，mm + rad；**不等到位**，需到位判定的一方自行轮询 `/arm_status`） |
| `/tl_driver/set_servoj_pos` | `tl_teleop`、`tl_teleop_f710`、`tl_hardware` 发（关节角，度；走 aux） |
| `/tl_driver/set_servol_pos` | `tl_teleop_f710` 仿真模式发（mm + rad）；`tl_driver` 与 `tl_teleop_f710_sim_bridge` 收 |
| `/joy` | `joy_node` 发；`tl_teleop_f710` 收 |
| `/tf`、`/tf_static` | `tl_description`（`robot_state_publisher`）发 |

其余话题与服务见 `src/tl_driver/doc/tl_driver服务与话题说明书.md`。

## 命名规范（只列非默认项）

- **C++**：文件 snake_case；类/枚举 PascalCase；枚举值与宏 UPPER_SNAKE；成员变量 `snake_case_`（下划线后缀）；成员函数 camelCase；服务回调 `handle_{name}_service`、话题回调 `handle_{topic}_topic`；句柄变量 `{name}_service_` / `_sub_` / `_pub_`；头文件保护 `包名__文件名_H_`；回调绑定用 `std::bind(&Class::method, this, ...)`。
- **Python**：文件/函数/变量 snake_case；类 PascalCase（ROS 节点类继承 `Node`）；私有方法 `_snake_case`；服务客户端 `_cli` 后缀、订阅者 `_sub` 后缀；入口函数与 `console_scripts` 同名。
- **话题/服务名**：snake_case，驱动侧统一 `/tl_driver/` 前缀。
- **已知例外（历史遗留，勿"顺手修正"）**：`connect_service_` → `/tl_driver/connect_arm`；`poweron_service_` / `poweroff_service_` → `/tl_driver/power_on` / `power_off`；`running_status_pub_` → `/arm_status`。

## 提交前必做

1. **格式**：`./scripts/format-cpp.sh` + `black .`（CI 与发版都会重跑这套检查，见下节）。`isort` 不进 CI，需要时自行跑。
2. **文档同步**：代码/配置变更命中下表任一项时，文档必须**在同一提交内**带上。逐项核对：

   | 变更 | 同步的文档 |
   | --- | --- |
   | msg / srv / 话题 / 服务增删改 | `CHANGELOG.md` + `src/tl_driver/doc/tl_driver服务与话题说明书.md`（接口包自身改 `src/tl_ros2_interface/README.md`） |
   | launch 文件、启动参数、YAML 参数 | 对应包 `README.md` |
   | 增删功能包、依赖变化 | `CHANGELOG.md` + 本文件包表与依赖行 + 该包 `README.md` |
   | 构建命令、命名规范、关键话题表、臂型表 | 本文件 |
   | 用户可见 Bug 修复、SDK 升级、新增臂型 | `CHANGELOG.md` |
   | 行为变更（协议、单位、上电时序、默认参数、回调组、公共接口签名） | `CHANGELOG.md` + 相关文档 |
   | 动到被文档引用的路径/名称/命令 | 修正所有引用处 |

   全部为否（格式化、注释、命名、`.omp/**` 等纯内部调整）才可跳过，且**不进 `CHANGELOG.md`**。条目标准、发布基线判定与格式见 `.omp/rules/changelog-user-visible.md`。
3. **自检**：`git diff --stat` 中代码与文档成对出现；文档引用的路径、launch 命令、话题/服务名、参数名与代码一致；`CHANGELOG.md` 里的短哈希真实存在于本分支历史。

## CI 与发版（`.github/workflows/`）

两个 workflow 都跑在 `ubuntu-24.04`，工具经 pip 固定版本（`clang-format==14.0.6`、`black==26.5.1`），与本地工具链一致，避免版本漂移误报。**两者都不编译**（MoveIt/RViz/ros2_control 依赖过重）—— 改了 C++ 必须本地 Docker 里 `colcon build` 过一遍。

**`ci.yml`（日常验证）**：push 到 `master`/`dev`/`V2` 与所有 PR 触发；`paths-ignore: **.md`、`docs/**`（纯文档提交**完全不触发**，改 `.github/**` 仍会触发）。只做格式检查：

- C++：`scripts/format-cpp.sh` 原地格式化后 `git diff --exit-code` 判断是否产生改动（自动跳过 `src/*/lib/include/` 下 SDK 头）
- Python：`black --check .`
- 同分支新推送会取消仍在跑的旧 CI（`concurrency`）

**`release.yml`（打标签触发，任意 tag 推送）**，四步：

1. **guard**：标签名须匹配 `[vV]?主.次.补[-(rc|beta)N]` **且**标签提交是远端 `master`/`dev`/`V2` 的祖先，两条都满足才发布；否则只留 notice、**静默跳过**（其余后缀如 alpha、里程碑标记一律不发布）。带 `-rc`/`-beta` 的标为 prerelease。**V2 为独立版本线，其标签只发布 V2 系列版本。**
2. **notes**：`scripts/release-notes.sh <tag>` 从 `CHANGELOG.md` 抽对应版本章节作正文（`V2.0.1` → `## [2.0.1]`，rc/beta 回退基础版本章节），末尾附完整变更日志链接，不用 GitHub 自动生成的提交/PR 列表。**找不到章节或章节为空 → 失败退出、不创建 Release**。
3. **format**：与 `ci.yml` 相同的格式检查。
4. **publish**：`gh release create`，标题即标签名，不附构建产物。

**发版顺序（必须）**：先把分支推上去并等 CI 绿 → `git tag Vx.y.z` → `git push origin Vx.y.z`。
tag 推送事件不携带分支信息，守卫靠提交祖先关系判断，**只推标签不推分支时远端分支引用仍在旧位置，守卫会静默跳过**（只剩标签、没有 Release）；误推时补推分支再重推标签即可。
章节缺失时：先把 `[Unreleased]` 并入版本章节并推分支，再把标签重新指向含该章节的提交：`git tag -f Vx.y.z <提交> && git push -f origin Vx.y.z`（仅删并重推同一标签仍指向旧提交，会再次失败）。
本仓库本地曾启用 `.githooks`：克隆后首次发版前如遇 hook 干扰，执行一次 `git config --unset core.hooksPath`。

## 协作约定

- 本文件每轮注入，**不要把使用说明、教程、历史写进来** —— 长文放 `README.md` / `src/README.md` / 各包 `README.md` / 说明书。
- 派子代理时任务要小且自包含：一个探索类子代理只查 1–2 个模式；已知路径直接用 `grep`/`read`，不委派；跨包或跨语言搜索拆成多个并行子代理。
- **跨线改动**：`V2` 与 `dev`(3.x) 的接口口径不同（位置单位 mm vs m 是最典型的一处），移植代码前先确认目标线口径。
