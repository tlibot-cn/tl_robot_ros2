# AGENTS.md — 天链机器人 ROS2 工作空间

**本文件在会话启动时注入 agent 上下文，只写两类内容：不知道就会做错的硬约束、找东西的地图。**
使用方式（启动命令、参数配置）看 `README.md`；逐包结构与用途看 `src/README.md` 和各包 `README.md`；
接口逐条说明看 `src/tl_driver/doc/tl_driver服务与话题说明书.md`。文档与代码冲突时以代码为准，并同步修正文档。

## 项目速览

- `src/` 下 10 个功能包 + `scripts/` 工具脚本；ROS2 Humble，C++17 / Python，colcon（ament_cmake + ament_python），无前端/Node.js。
- 构建：`colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` → `source install/setup.bash`（产物 `build/`、`install/`、`log/` 已 gitignore）。选择性构建必须先建 `tl_ros2_interface`（生成 msg/srv 头文件）。
- **无自动化测试**：`test/` 只有 ament lint 脚手架（`ament_copyright`/`flake8`/`pep257`）。不要假设有测试可跑，也不要以"补测试"充当验证 —— 验证靠编译 + 实机/仿真运行。
- 开发环境在 Docker 内（配置不在本仓库）。

| 包 | 作用 | 关键入口 |
| --- | --- | --- |
| `tl_ros2_interface` | 全部自定义接口（12 个 `.msg`、45 个 `.srv`） | `msg/`、`srv/`；**必须最先构建** |
| `tl_driver` | 机械臂驱动，TCP 连控制器（65 服务 / 4 订阅 / 3 发布） | `src/tl_driver.cpp`（约 2800 行单文件）、`include/tl_driver/tl_driver.h` |
| `tl_teleop` | VR 遥操作（PXREA Robot SDK，100 Hz 控制线程） | `src/tl_teleop.cpp`；SDK 头 `lib/include/PXREARobotSDK.h`（C 风格 API，用 `uint64_t` 需 `<stdint.h>`） |
| `tl_teleop_f710` | F710 手柄遥操作 + 仿真桥接（250 Hz ServoJ） | `src/tl_teleop_f710_node.cpp`、`src/tl_teleop_f710_sim_bridge.cpp` |
| `tl_hardware` | ros2_control 硬件插件，桥接 MoveIt2 ↔ `tl_driver` | `src/tl_hardware_interface.cpp`（`tl_hardware::TLHardwareInterface`） |
| `tl_description` | URDF + STL 网格 + RViz（纯数据包，无编译代码） | `urdf/`、`meshes/`、`rviz/` |
| `tl_bringup` | 启动聚合（`tl_driver` + `tl_description`） | `launch/`（14 种臂型各一个） |
| `tl_gazebo` | Gazebo 仿真 | `launch/`、`config/` |
| `tl_moveit2_config` | MoveIt2 配置集合（14 个子包） | `tl_<arm_type>_config/` |
| `tl_example` | 示例程序（医疗 demo、接口示例） | `src/` |

依赖：`tl_driver` / `tl_teleop` / `tl_teleop_f710` / `tl_hardware` / `tl_example` 依赖 `tl_ros2_interface`（须先构建）；`tl_teleop` / `tl_teleop_f710` / `tl_hardware` 另声明 `exec_depend: tl_driver`（运行时走话题/服务，不链接）；`tl_gazebo` 与各 `tl_<arm>_config` 声明 `tl_description`（后者另含 `tl_hardware`）；`tl_bringup` 只组合 launch，不声明包依赖。

## 硬约束（改代码前先读）

**① 专有库只读。** `src/tl_driver/lib/`（`libtl_host.so` + `include/*.h` 14 个头，V3.0.2）与 `src/tl_teleop/lib/`（`libPXREARobotSDK.so`）是预编译第三方产物 —— 不改、不格式化（`scripts/format-cpp.sh` 已跳过 `lib/include/`）。

**② 单位口径。** ROS 侧 m / rad；控制器与服务接口 mm / 度 / %。换算发生在驱动内或各消费方，**改任一端必须同时改另一端**：

| 通道 | 单位 |
| --- | --- |
| `/joint_states.position` | rad（驱动内度→弧度） |
| `/tcp_pose.position` / `.rpy` | m / rad（驱动内 mm→m） |
| `/tl_driver/set_servoj_pos` | 度（原值透传 aux；7 元素，末位补 0） |
| `/tl_driver/set_servol_pos.target_pose` | mm + 姿态 rad；`step_size` mm（≤0 → 2.0） |
| `MoveCommand.target_pos_value` | 原值透传：`coord=0` 为度；`1/2/3` 为 mm + 姿态 rad |
| `ToolParam` / `SetUserCoord` | mm + 姿态 rad（`ToolParam` 的 `a/b/c` 为度、`payload_mass` 为 kg） |

消费方换算共四处：`tl_teleop`（`/tcp_pose` m→mm ×1000）、`tl_teleop_f710_node`（KDL FK m→mm ×1000、rad→度 ×180/π）、`tl_teleop_f710_sim_bridge`（mm→m ÷1000）、`tl_hardware_interface`（rad→度 ×180/π）。
欧拉角约定为 **XYZ 内旋**（scipy 中用大写 `'XYZ'`）。
先例：`/tcp_pose` 的 m↔mm 误用曾让遥操作 IK 目标点错 1000 倍，控制器报 **9754「目标位置不可达」**（`2c5b2e4`）。

**③ 长度契约。** 关节/位姿向量第 7 位对 6 轴**补 0**，不是截断；`MoveCmd::targetPosValue` 与全局点位容器均为 **14 位**（前 7 本体 + 后 7 外部轴，几轴填几位、其余置 0）。

**④ 双端口。** 6001 `arm_port`：请求/响应式 SDK 调用；7000 `arm_port_aux`：servoJ 全系列 + **机器人状态异步推送**。两个 fd 都必须连上（`is_connected()` 要求均 > 0）；状态回调只在 aux 注册，错误/告警回调**两个端口都注册**。

**⑤ 线程模型。** `tl_driver` 三组回调组 `service_group_` / `topic_group_` / `timer_group_` **均为 `MutuallyExclusive`**（同组串行），由 `MultiThreadedExecutor` 驱动（线程数 `max(4, hardware_concurrency)`）。`timer_group_` 曾用 `Reentrant` 导致状态发布回调并发，`4757255` 改为互斥，**勿改回**。

**⑥ 失败即退。** `TL_Arm::init()` 中连接失败、切入示教模式失败均 `rclcpp::shutdown()` + exit —— 不带病运行。

**⑦ 硬编码边界。** IP / 端口 / 关节名只出现在 `config/*.yaml`（默认 `192.168.1.13:6001`），不写进代码。`arm_type` 在配置 YAML 中大写（`TCB605`），启动参数小写（`tcb605`），全部 14 种。

## 关键话题

| 话题 | 方向与单位 |
| --- | --- |
| `/joint_states` | `tl_driver` 发（rad）；`tl_description`、`tl_hardware` 收 |
| `/tcp_pose` | `tl_driver` 发（m / rad） |
| `/arm_status` | `tl_driver` 发（运行状态） |
| `/tl_driver/moveJ`、`/tl_driver/moveL` | `tl_driver` 收（`MoveCommand`；**不等到位**，需到位判定的一方自行轮询 `/arm_status`） |
| `/tl_driver/set_servoj_pos` | `tl_driver` 收（度，走 aux） |
| `/tl_driver/set_servol_pos` | `tl_driver` 收（mm + rad）；`tl_teleop_f710` 仿真模式发 |

其余话题与服务见 `src/tl_driver/doc/tl_driver服务与话题说明书.md`。

## 命名规范（只列非默认项）

- **C++**：文件 snake_case；类/枚举 PascalCase；枚举值与宏 UPPER_SNAKE；成员变量 `snake_case_`（下划线后缀）；成员函数 camelCase；服务回调 `handle_{name}_service`、话题回调 `handle_{topic}_topic`；句柄变量 `{name}_service_` / `_sub_` / `_pub_`；头文件保护 `包名__文件名_H_`；回调绑定用 `std::bind(&Class::method, this, ...)`。
- **Python**：文件/函数/变量 snake_case；类 PascalCase（ROS 节点类继承 `Node`）；私有方法 `_snake_case`；服务客户端 `_cli` 后缀、订阅者 `_sub` 后缀；入口函数与 `console_scripts` 同名。
- **话题/服务名**：snake_case，驱动侧统一 `/tl_driver/` 前缀。
- **已知例外（历史遗留，勿"顺手修正"）**：`connect_service_` → `/tl_driver/connect_arm`；`poweron_service_` / `poweroff_service_` → `/tl_driver/power_on` / `power_off`；`running_status_pub_` → `/arm_status`。

## 提交前必做

1. **格式**：`./scripts/format-cpp.sh` + `black .`（CI 与发版都会重跑这套检查，见下节）。`isort` 不进 CI，需要时自行跑。
2. **文档同步**：代码/配置变更必须**在同一提交内**带上文档。逐项核对：

   | 变更 | 同步的文档 |
   | --- | --- |
   | msg / srv / 话题 / 服务增删改 | `CHANGELOG.md` + 对应包说明书（如 `src/tl_driver/doc/`） |
   | launch 文件、启动参数、YAML 参数 | 对应包 `README.md` |
   | 增删功能包、依赖变化 | `CHANGELOG.md` + 本文件包表 + 该包 `README.md` |
   | 构建命令、命名规范、关键话题表、臂型表 | 本文件 |
   | 用户可见 Bug 修复、SDK 升级、新增臂型 | `CHANGELOG.md` |
   | 行为变更（协议、单位、上电时序、默认参数、回调组、公共 API 签名） | `CHANGELOG.md` + 相关文档 |
   | 动到被文档引用的路径/名称/命令 | 修正所有引用处 |

   全部为否（格式化、注释、命名、`.omp/**` 等纯内部调整）才可跳过，且**不进 `CHANGELOG.md`**。条目标准与发布基线判定见 `.omp/rules/changelog-user-visible.md`。
3. **自检**：`git diff --stat` 中代码与文档成对出现；文档引用的路径、launch 命令、话题/服务名与代码一致。

## CI 与发版（`.github/workflows/`）

两个 workflow 都跑在 `ubuntu-24.04`，工具经 pip 固定版本（`clang-format==14.0.6`、`black==26.5.1`），与本地工具链一致，避免版本漂移误报。**两者都不编译**（MoveIt/RViz/ros2_control 依赖过重）—— 改了 C++ 必须本地 Docker 里 `colcon build` 过一遍。

**`ci.yml`（日常验证）**：push 到 `master`/`dev` 与所有 PR 触发；`paths-ignore: **.md`、`docs/**`，即纯文档提交**完全不触发**（也就没有 CI 绿灯可等，发版前需自行确认）。只做格式检查：

- C++：`scripts/format-cpp.sh` 原地格式化后 `git diff --exit-code` 判断是否产生改动（自动跳过 `src/*/lib/include/` 下 SDK 头）
- Python：`black --check .`
- 同分支新推送会取消仍在跑的旧 CI（`concurrency`）

**`release.yml`（打标签触发，任意 tag 推送）**，四步：

1. **guard**：标签名须匹配 `[vV]?主.次.补[-(rc|beta)N]` **且**标签提交是远端 `master`/`dev` 的祖先，两条都满足才发布；否则只留 notice、**静默跳过**（其余后缀如 alpha、里程碑标记一律不发布）。带 `-rc`/`-beta` 的标为 prerelease。
2. **notes**：`scripts/release-notes.sh <tag>` 从 `CHANGELOG.md` 抽对应版本章节作正文（`V3.0.0` → `## [3.0.0]`，rc/beta 回退基础版本章节），末尾附完整变更日志链接，不用 GitHub 自动生成的提交/PR 列表。**找不到章节或章节为空 → 失败退出、不创建 Release**。
3. **format**：与 `ci.yml` 相同的格式检查。
4. **publish**：`gh release create`，标题即标签名，不附构建产物。

**发版顺序（必须）**：先把分支推上去并等 CI 绿 → `git tag Vx.y.z` → `git push origin Vx.y.z`。
tag 推送事件不携带分支信息，守卫靠提交祖先关系判断，**只推标签不推分支时远端分支引用仍在旧位置，守卫会静默跳过**（只剩标签、没有 Release）；误推时补推分支再重推标签即可。
章节缺失时：先把 `[Unreleased]` 并入版本章节并推分支，再把标签重新指向含该章节的提交：`git tag -f Vx.y.z <提交> && git push -f origin Vx.y.z`（仅删并重推同一标签仍指向旧提交，会再次失败）。

## 协作约定

- 本文件每轮注入，**不要把使用说明、教程、历史写进来** —— 长文放 `README.md` / `src/README.md` / 各包 `README.md` / 说明书。
- 派子代理时任务要小且自包含：一个探索类子代理只查 1–2 个模式；已知路径直接用 `grep`/`read`，不委派；跨包或跨语言搜索拆成多个并行子代理。
