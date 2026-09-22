# 变更日志

本文件记录天链机器人 ROS2 工作空间（tl_robot_ros2）的所有重要变更。

格式遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)：

- 已发布的版本章节按变更类型整理（新增/修复/变更/移除/文档/工程），不再按日期分组
- 未发布的变更记录在 `[Unreleased]` 下，按日期（新 → 旧）分组整理
- 正式发布时，将 `[Unreleased]` 内容合并进版本号章节（`## [x.y.z] - YYYY-MM-DD`），并重置 `[Unreleased]`
- 条目末尾括号内为提交短哈希，便于溯源

## [Unreleased]

（暂无）

## [2.0.1] - 2026-09-22

### 修复

- 修复 MoveIt2 / ros2_control 启动瞬间机械臂先被拉向零位、轨迹控制器接管后才停下的问题：硬件接口 `on_activate()` 改用当前关节角播种命令接口，启动阶段不再下发零位指令（`070b154`）

### 变更

- tl_teleop 包声明对 `tl_driver` 的依赖（`<exec_depend>`），保证服务端可用性（`2b5a56f`、`3265269`）
- tl_hardware、tl_teleop_f710 同样调用 `/tl_driver/*` 服务，补齐对 `tl_driver` 的 `<exec_depend>` 声明（`248e82f`、`3265269`）
- 功能包许可证统一为 Apache-2.0：`tl_teleop_f710` 由 `Proprietary`、`tl_gazebo` 与 14 个 MoveIt2 型号配置包由 `BSD` 改为 Apache-2.0，与仓库根 `LICENSE` 一致（`b682f00`）

### 文档

- tl_driver 服务与话题说明书：显式标注全部接口单位（笛卡尔位置 mm、姿态 rad、关节 度、速度 mm/s 或 %），明确本仓库笛卡尔位置有意采用 mm（非 ROS 惯例 m）（`c1c4f23`）
- 修正 `MoveCommand.target_pos_value` 的布局描述：实为 14 维 MoveCmd 本体位姿（`[0..6]` 本体 / `[7..13]` 外部轴），此前误按 GP 点位容器布局描述，与实现及 SDK 头注释矛盾（`c1c4f23`）
- 26 个 msg/srv 增加字段单位与布局注释，集成方无需再依赖外部文档判断单位（`fecff32`）
- tl_driver 服务与话题说明书：`/tcp_pose` 与 `/tl_driver/set_user_coord` 的位置单位由 m 修正为 mm（与实现、1.1 约定及同章工具参数口径一致），`/tl_driver/set_servol_pos` 的 `step_size` 默认值 5.0 → 2.0（代码实际值），1.4 单位制约定按节点/直角坐标各自的实际口径限定（`d0fd361`、`29b7841`）
- tl_ros2_interface README 补齐 `ServolMove` 消息说明（目录、文件总览与字段），修正 `CartesianPose.position` 单位标注（`d0fd361`）
- tl_teleop_f710 README 修正适用型号轴数：TCB610V 归入 6 轴，避免按 7 轴配置启动导致 FK/IK 初始化失败（`299482c`）
- tl_gazebo README 补全 F710 手柄仿真 launch / xacro / 控制器配置条目（`d0fd361`）

## [2.0.0] - 2026-08-21

### 新增

- f710 手柄遥操作功能包，支持 Gazebo 仿真遥操作及 6 轴、7 轴实体机械臂遥操作（`3f86c43`）
- tl_hardware 功能包（`8932fa0`），真实硬件接口支持并集成 servoj 控制服务（`8d0989e`），优化硬件接口性能并改进关节状态处理（`29da8e1`）
- 基于 servoj 的 servol 节点（`994f363`）
- MoveIt2 控制真实机械臂完成所有型号适配（`9ae0401`）
- 六轴遥操作适配（`87a930b`）
- 示例程序（`5e2519b`）
- TCB705 机械臂工作空间测量工具（`0a46541`）
- 控制循环性能计时功能（`5dd1fa4`）
- `GetCurrentMode` 服务，支持查询机械臂当前模式（`4b41369`）
- `/tl_driver/get_current_motor_torque` 与 `/tl_driver/get_current_line_joint_speed` 两个服务（`8bf9588`）
- tl_driver 新增 14 个独立 launch 文件（每种臂型一个），替代参数选择配置方式（`794ab48`）
- tl_driver 增加 ARM 平台支持（`0e4a536`）
- 605/610 系列 MoveIt2 + Gazebo 仿真（`f8657b6`）、705 系列（`82dd2ce`）、tcb710v、tcb710 仿真（`dd59d39`）
- Apache License 2.0（`8f4375b`）
- AGENTS.md 工作空间说明与手眼标定说明文档（`77af61a`）

### 修复

- `power_on` 各伺服状态下电逻辑（`1ce6ad6`）
- ros2_control Xacro YAML 加载函数调用：`load_yaml` → `xacro.load_yaml`（`e38e1ae`）
- 改为直接调用 SDK，逆解问题解决（`1b06d73`）
- tl_driver/lib 的动态库链接问题（`80cf401`、`fa77577`；`5443547` 曾回退）
- package.xml 补充 tl_ros2_interface 依赖声明（`56fb609`）
- 将 `T_tool_camera` 重命名为 `T_camera_tool` 以匹配实际变换方向（`fac0a78`）
- 移除硬编码路径，改为代码自动推导，提高可移植性（`dbf8ca5`）
- 统一 6 轴系列 gazebo_moveit_demo\* launch 文件名（去掉 tl 前缀）与 7 轴系列（`38c7428`）

### 变更

- tl_teleop 异步 IK + 控制循环实时性优化（`5aa2b12`）
- tl_teleop_f710 全面重构 C++ 遥操作节点，修复多项 Bug（`ea31815`）
- tl_teleop_f710 从 servol 改为 servoj 并修复 bug（`7efda43`）
- 遥操作用 ROS2 接口重构（`c601013`）
- tl_driver 使用回调组重构（`d564be7`）
- 定时器回调组多线程执行方式改为互斥（`4757255`）
- 优化上电时序（`a26e7d8`）
- tl_teleop 优化资源清理和退出顺序（`f11e958`）
- tl_driver 初始化上电后添加 2 s 延时（`6c0ff29`）
- tl_driver/lib 规范命名（`2af8842`）
- control_node.py 用 ROS2 接口重写（`9c4b7b2`）
- tl_vision calib_node.py 使用封装的 ROS2 接口重构（`ddfbc26`）
- tl_description 更新模型文件，同步变更 tl_gazebo 与 tl_moveit2 模型调用路径（`d704a2e`）
- tl_description 每种类型机械臂分别添加对应 launch 文件（`34193d8`）
- tl_vision 更新 launch 文件，control_node 与 calib_node 依赖 tl_driver（`6a72fd9`）
- 统一 cpp/python 格式化规则并添加到 githook（`2a6c984`）
- 补充注释（`daef348`）

### 移除

- 从远程仓库删除 tl_vision 功能包（`a58d072`）

### 文档

- 新增 tl_teleop README（`da562c9`）
- 更新 tl_teleop README（`1cc7217`）
- 新增 tl_teleop_f710 Readme（`d14c830`），更新 tl_teleop_f710 Readme（`9a896b4`）
- 更新 tl_driver 服务与话题说明书（`fc8bdda`、`24821f0`、`bc91dc9`）
- 完善 MoveIt2 控制真实机械臂的相关文档（`1a68afe`）
- 添加 tl_gazebo README（`906455a`）
- 添加 tl_moveit2_config 文档（`080eb3b`）
- 更新手眼标定说明文档（`1b564d1`），并移动至 `src/tl_vision/README.md`（`cc11ca8`）
- 按功能分类服务与话题，将简短功能描述改为详细描述（`372e118`）
- AGENTS.md 增加命名规范（`95c6929`）
- 更新 README.md（`8623d99`、`70151d5`）及各功能包文档（`6b216fb`、`f9e79e3`、`78c0e76`、`85f8ee3`、`ff855c9`、`ef5fa18`）

### 工程

- 更新 .gitignore（`16624b1`、`4c2c577`、`b9c163f`）
- 更新 AGENT.md（`4687476`）
- 更新 gitnexus 配置（`c4b779f`）

---

**备注**：
- `a9139ac`（保存工作）为临时保存，已被 `f0cb692`（Revert "保存工作"）回退，未计入本版本。
- `829c416`（Merge PR #3 from tlibot-cn/dev）为合并提交，无独立用户可见变更。
- 标签 `V2.0.0` 指向 `1ce6ad6`（2026-08-21）；`V2` 维护分支上 tag 之后的提交记入 `[2.0.1]`。
- 另有一条独立发布线（`master`/`dev` 分支，标签 `V3.0.0`，2026-09-02），历史与本文件所在的 V2 维护线已分叉，其变更不计入本文件的 V2 系列章节。
