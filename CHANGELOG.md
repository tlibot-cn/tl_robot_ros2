# 变更日志

本文件记录天链机器人 ROS2 工作空间（tl_robot_ros2）的所有重要变更。

格式遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)：

- 最新发布版本：`[3.0.0]`（2026-09-02，git tag `V3.0.0`）
- 已发布的版本章节按变更类型整理（新增/修复/变更/移除/文档/工程），不再按日期分组
- 未发布的变更记录在 `[Unreleased]` 下，按日期（新 → 旧）分组整理
- 正式发布时，将 `[Unreleased]` 内容合并进版本号章节（`## [x.y.z] - YYYY-MM-DD`），并重置 `[Unreleased]`
- 条目末尾括号内为提交短哈希，便于溯源

## [Unreleased]

### 2026-10-19

- **破坏性变更**：移除 ServoL 功能——删除 `/tl_driver/set_servol_pos` 话题与 `tl_ros2_interface/msg/ServolMove`；`tl_driver` 订阅数 4→3、接口包 msg 12→11。笛卡尔直线伺服改由消费方本地 IK 后经 `/tl_driver/set_servoj_pos` 下发
- tl_teleop_f710：仿真模式改为节点内 KDL IK，250 Hz 发布 `set_servoj_pos`；`sim_bridge` 仅转发关节角到 Gazebo position controller，不再依赖 ServoL
- 文档/测试：同步删除 ServoL 章节、单位表、接口测试与未使用导入；架构评审文档记录移除结论

### 2026-10-18

- tl_driver：所有服务回调统一加异常边界——回调内部抛异常时返回 `success=false` 与 `message`（`Internal error: ...`），不再越过回调终止驱动进程
- tl_driver：初始化连接或切入示教模式失败由 `exit(0)` 改为 `EXIT_FAILURE`，便于 systemd/launch/CI 识别启动失败
- tl_driver：修正服务结果语义——`get_robot_joint_param`、`get_drag_status` 查询失败不再返回 `success=true`；`set_axis_zero_pos` 改用自身文案；`set_robot_joint_param` 的上下电失败、`queue_motion_set_status` 的示教模式切换/下电失败均回写 `success=false`
- tl_driver：状态发布补齐 `switch` 默认分支（未知运行状态发布 `UNKNOWN`），按数据实际长度裁剪关节位姿并校验 TCP 位姿长度，避免发布空 `JointState` 或越界读取
- tl_driver：新增 `publish_rate` 参数（默认 100 Hz，范围 1–1000；越界或类型错误均以非零码启动失败，不再 SIGABRT）；`arm_joints` 数量限制为 1–8，非法配置启动即失败
- tl_driver：连接时任一端口失败会关闭另一已建立端口并清零 fd，避免重试时 fd 泄漏与控制器侧连接堆积

### 2026-10-17

- **破坏性变更**：tl_driver 移除 18 个不属于 ROS 驱动职责的服务，服务数量 65 → 47。移除范围：纯数学变换（`get_quat2rpy` / `get_rpy2quat` / `get_rpy2r` / `get_tr2r` / `get_r2tr`）、控制器作业文件（`get_all_job_filename` / `job_run` / `job_delete` / `job_insert_moveJ` / `job_insert_moveL` / `job_insert_imove` / `job_insert_moveC`）、全局路点（`get_global_pos` / `set_global_pos`）、`get_nexmotion_lib_version`、`set_dh_param`、`restore_default_dh_param`、`log_download`。`tl_ros2_interface` 中的 srv 定义暂保留，供外部工具按需引用
- tl_teleop：`get_rpy2quat` 改为本地数学转换，不再依赖驱动服务
- tl_example / `test_all_interfaces.py`：同步移除已下线服务的调用；说明书删除对应章节，章节号保持不变以避免破坏既有引用
- **破坏性变更**：全部 ROS 接口统一为 REP 103 单位——关节角 rad、笛卡尔位置 m、线速度 m/s、关节速度 rad/s；控制器 / SDK 的 mm / 度 / % 由 `tl_driver` 在边界换算。受影响接口：`MoveCommand`（关节 rad、位置 m、MOVL 速度 m/s）、`/tl_driver/set_servoj_pos`（rad）、`ServolMove`（m、step m）、`CartesianPose`（`SetUserCoord` 改 m）、`ToolParam`（m、rad）、`RobotDHParam`（rad、m）、`RobotJointParam`（rad、rad/s、rad/s²）、`OpenServoJ`（rad/s 系列）、`CoordTransform`（rad、m）、`GetCurrentLineJointSpeed`（m/s、rad/s）。`get_pos_reachable` 的 14 维协议包保留控制器 mm + 单位标志位口径
- tl_driver：新增 `src/convert/units.hpp` 集中单位换算，运动指令 / ServoJ / ServoL / 工具与用户坐标 / DH / 关节参数 / 线速度在边界转换；ServoJ 约束自动补齐 6 轴第 7 位
- tl_hardware：`write()` 直接发布 rad，`servoj_vmax/amax/jmax` 改用 rad 默认值（等价原 30/100/500 度制）
- tl_teleop / tl_teleop_f710 / tl_example：内部计算与发布统一 rad / m，配置参数保留度制并在读取时换算；`tl_teleop` 的 `max_pos_delta_mm` 更名为 `max_pos_delta_m`（默认 0.3 m）
- tl_ros2_interface：msg / srv 字段注释与 `README.md` 同步新单位；新增 `src/tl_driver/doc/tl_driver单位对照表.md`，说明书升至 V1.8
- tl_driver 测试：新增 `test_units.cpp`，更新 `test_msg_convert.cpp` 覆盖 rad/m 转换

### 2026-09-24

- tl_driver：启动日志新增驱动包自身版本行（`Driver Version: x.y.z`，取自 `package.xml`），与已有的 `SDK Version` 行并列输出（`c857ac0`）
- tl_driver：专有库更新到 V3.1.0（`lib/arm`、`lib/x86` 的 `libtl_host.so` 与 `lib/include/` 下 14 个 API 头），`README.md`、`AGENTS.md` 中的版本标注同步订正为 V3.1.0（`87a456e`）
- tl_ros2_interface：12 个 msg 头部标注用途（话题载荷 / 仅作 srv 嵌套参数 / 未接线）并在 README 补分类说明，避免把只作服务参数的结构误读为话题接口（`d29b9f1`）

### 2026-09-22

- tl_hardware：修复 MoveIt2 / ros2_control 启动瞬间机械臂先被拉向零位、轨迹控制器接管后才停下的问题：硬件接口 `on_activate()` 改用当前关节角播种命令接口，启动阶段不再下发零位指令（`46e8132`）
- tl_hardware、tl_teleop、tl_teleop_f710 补齐对 `tl_driver` 的 `<exec_depend>` 声明：三个包运行期都经 `/tl_driver/*` 服务与话题通信，此前缺该声明时 `rosdep install` 不会拉取 `tl_driver`（`89bef4d`）

### 2026-09-04

- tl_teleop：修复 /tcp_pose 单位误用（m 被当作 mm），遥操作 IK 目标点错误导致控制器报 9754「目标位置不可达」的问题（`2c5b2e4`）
- tl_teleop：修复 6 轴模式 ServoJ 速度/加速度/加加速度数组仅分配 6 元素、与 7 元素透传契约不一致的问题（`07230b8`）
- tl_teleop：PXREA 状态回调由逐帧 JSON 打印改为每 100 帧输出一次平均帧率，避免日志刷屏（`e60f924`）

### 2026-09-03

- `scripts/workspace_measure`：FK/IK 工作空间图由三视图改为四视图，新增 YZ 截面（切 `|X|<0.05`/`0.08`），布局调整为 2×2（3D + XZ/XY/YZ）

## [3.0.0] - 2026-09-02

### 新增

- tl_example 示例程序：数字 IO、信息查询、运动控制、队列运动、ServoJ、示教控制及医疗/实验室 moveL 队列示例（`b74b1e0`）
- `fk_workspace.py` 支持 `--arm-type` 指定臂型，覆盖全部 14 种型号，图表标题与输出文件名随臂型联动（`3b62410`）

### 修复

- tl_driver 专有库更新到 V3.0.2，修复控制器返回角度而非弧度的问题（`4b5dff0`）
- 清除 tl_driver 全部编译警告：SDK 弃用 int 接口迁移到枚举重载（RobotMode/ServoState/Coord/DragMode/IoLevel/RunState），SDK 返回值统一为 `Result` 类型（纯类型收紧，行为不变）（`8dca93f`）
- 修复 tl_example 构建失败：移除已删除的 ex_driver_quick_test 目标，ex_info_query 的 DH 参数打印适配新标准 DH 消息（`b4ad6ca`）

### 变更

- 全部功能包 `cmake_minimum_required` 统一升级到 3.16（`923080a`）
- 统一全部 14 种臂型默认通讯 IP 为 `192.168.1.13`（tl_tcb610v 配置同步更新）（`ffe0a3d`）
- 所有 tl_driver launch 文件新增 `arm_ip` 参数，启动时可覆盖默认 IP（优先于 config 文件）（`ffe0a3d`）
- 迁移接口测试脚本到 `tl_driver/test`，清理旧测试（`b0c5057`）
- 取消注释并测试全部接口（跳过危险与运动接口）（`0c00b3e`）
- 重新提交 `tl_driver.cpp`（`f9bcc3a`）
- tl_driver SDK 更新到 V3.0 版本（`2f8ac51`）
- tl_driver 补充遗漏的接口（`16980c7`）
- tl_driver 调整部分接口调用（`2930bd0`）
- 修复 `get/set_dh_param`，适配新 SDK 标准 DH 参数（`dda23c0`）

### 文档

- 新增 tl_driver ROS2 接口全面测试文档（`7d6ca97`）

### 工程

- 忽略 hetest 测试文件（`dd71277`）
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
- 标签 `V2.0.0` 指向 `1ce6ad6`（2026-08-21），master 分支 tip；tag 之后的分支提交记入 `[3.0.0]`。
- 当前发版分支为 `release`，历史经重放后部分提交哈希与旧 master 线不同，`[3.0.0]` 内哈希均以 `release` 分支实际提交为准。
