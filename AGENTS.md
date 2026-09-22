# AGENTS.md — 天链机器人 ROS2 工作空间

## 工作空间概述

天链（TianLian）机械臂 ROS2 工作空间。`src/` 下为 9 个顶层功能包，另有 `tl_moveit2_config/` 下 14 个型号子包（共 23 个 `package.xml`）与根目录 `scripts/` 工具脚本，使用标准 `colcon build` 构建流程。无 `package.json`、无 Node.js — 纯 ROS2（ament_cmake + ament_python）。

仓库根目录文件：

| 文件 / 目录 | 作用 |
|---|---|
| `README.md` | 环境搭建、编译、代码格式与运行入口 |
| `AGENTS.md` | 本文件 — 工作空间结构、命名规范与文档同步规则 |
| `CHANGELOG.md` | 全部用户可见变更的账本（维护规则见「文档与变更日志同步规则」） |
| `pyproject.toml`、`.clang-format` | Python（black/ruff/isort，100 列）与 C++（clang-format v14，Allman、2 空格、120 列）格式配置 |
| `.github/workflows/` | CI：push/PR 格式检查（clang-format + black）；打版本标签触发 GitHub Release（标题即标签名，正文取自 `CHANGELOG.md` 对应版本章节） |
| `scripts/format-cpp.sh` | clang-format 包装脚本，自动跳过 `lib/include/` 下三方 SDK 头文件 |
| `scripts/release-notes.sh` | Release 正文提取脚本：从 `CHANGELOG.md` 抽出指定标签的版本章节（`release.yml` 调用；本地预览 `./scripts/release-notes.sh V2.0.1`） |
| `scripts/workspace_measure` | 工作空间测量工具（FK/IK 可达空间可视化） |

## 构建命令

（colcon 自动解析拓扑顺序）：
```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

构建产物在 `build/`、`install/`、`log/` — 均已 gitignore。

## 功能包依赖关系

```
tl_ros2_interface  （基础：自定义 msg/srv，无依赖）
  └─► tl_driver        （C++ 节点，链接 _tl_host.so 专有库）
  └─► tl_teleop        （VR 遥操作 C++ 节点，PXREA Robot SDK；exec_depend: tl_driver）
  └─► tl_teleop_f710   （F710 手柄遥操作 C++ 节点，KDL IK；exec_depend: tl_driver）
  └─► tl_hardware      （ros2_control 硬件接口插件，桥接 MoveIt2 ↔ tl_driver；exec_depend: tl_driver）
  └─► tl_example       （示例节点：医疗检验科队列 MoveL）
tl_description     （独立：URDF + 网格 + RViz）
  └─► tl_gazebo       （Gazebo 仿真，依赖 tl_description）
  └─► tl_moveit2_config（MoveIt2 配置集合，依赖 tl_description + tl_hardware）
tl_bringup         （启动聚合器：包含 tl_driver + tl_description）
```

## 功能包说明

### tl_ros2_interface
- **构建类型**：ament_cmake
- **用途**：定义所有自定义 ROS2 接口（12 个 `.msg`，45 个 `.srv`）
- **关键消息**：`ObjectInfo`、`ArmStatus`、`CartesianPose`、`MoveCommand`
- **关键服务**：`GetCurrentCoord`、`SetSpeed`、`Jogging`、`ModbusRead/Write`、`JobRun`
- **必须最先构建** — 其他包依赖其生成的头文件（colcon 会自动处理构建顺序）

### tl_driver
- **构建类型**：ament_cmake（C++17）
- **用途**：机械臂驱动 — 通过 TCP 与实体机械臂通信
- **入口**：`src/tl_driver.cpp` → 单一 `tl_driver` 可执行文件；`main()` 使用 `MultiThreadedExecutor`（线程数 `max(4, hardware_concurrency)`）匹配回调组架构
- **专有库**：`lib/arm/`（ARM64）与 `lib/x86/`（x86_64）下的预编译 `.so`，不可修改；CMake 按 `CMAKE_SYSTEM_PROCESSOR` 自动选择目录，其他架构直接 `FATAL_ERROR`：
  - C++ 只链接 `_tl_host.so`（`IMPORTED` target `tl_host_lib`），安装到 `lib/tl_driver/` 并建 `libnrc_host.so` 符号链接兼容旧 SONAME
  - `lib/arm/` 额外含外围库：`libtl_host.so`、`libservoJ_wrapper.so`、`libmodbus_wrapper.so`、`libmath_wrapper.so`，由 `tl_interface.py` 经 ctypes 调用，仅 ARM64 安装
  - `lib/include/` 为 SDK 头文件：`cpp/interface/`、`cpp/parameter/`、`c/interface/`、`c/parameter/`（格式检查会跳过该目录）
- **Python API**：`lib/arm/tl_interface.py`、`lib/x86/tl_interface.py` — 运行时通过 `sys.path` 加载的 Python 封装
- **配置**：`config/` 下按臂型命名的 YAML（如 `tl_tcb605_config.yaml`）。关键参数：`arm_ip`、`arm_port`（TCP 主端口）、`arm_port_aux`（TCP 辅助端口）、`arm_type`、`arm_joints`
- **启动**：
  - 通用：`ros2 launch tl_driver tl_driver.launch.py arm_type:=<arm_type>`
  - 快捷：`ros2 launch tl_driver tl_tcb710_driver.launch.py`（每种臂型一个专用文件，如 `tl_tcbXXX_driver.launch.py`）
- **默认机械臂 IP**：`192.168.1.13`，端口 `6001` — 如需修改，改对应配置 YAML
- **回调组架构**（`TL_Arm` 构造函数中创建 3 组，全部为 `MutuallyExclusive` — 各组内回调串行，组间由 `MultiThreadedExecutor` 并行）：
  - `service_group_` — 全部 65 个服务，保证服务回调串行执行
  - `topic_group_` — 4 个话题订阅，保证话题回调串行执行
  - `timer_group_` — 状态发布定时器（100 Hz，`state_publish_timer_`），定时器回调同样串行
- **话题**：发布 `joint_states`、`tcp_pose`、`arm_status`；订阅 `moveJ`、`moveL`、`set_servoj_pos`、`set_servol_pos`（详见下方关键话题表）
- **安全行为**：`init()` 中若 `connect()` 失败，节点会调用 `rclcpp::shutdown()` 并 exit

### tl_teleop
- **构建类型**：ament_cmake（C++17）
- **用途**：遥操作节点 — 通过 PXREA Robot SDK（预编译 `.so`）与遥操作设备通信，同时在 ROS2 层面通过 `tl_ros2_interface` 的消息与服务与 `tl_driver` 交互
- **专有库**：`lib/arm/`（ARM 架构）和 `lib/x86/`（x86 架构）下的预编译 `libPXREARobotSDK.so`，不可修改
- **SDK 头文件**：`lib/include/PXREARobotSDK.h` — C 风格 API，使用 `uint64_t`（需 `#include <stdint.h>`）
- **依赖**：`rclcpp` + `tl_ros2_interface`，并以 `<exec_depend>` 声明 `tl_driver`（运行期经 `/tl_driver/*` 服务与话题通信）— 不链接 `tl_driver` 的库
- **实现状态**：已实现（双线程架构：ROS2 事件循环 + 100 Hz 控制循环；支持 6/7 轴自适应、握紧触发、摇杆死区、奇异点保护、关节跳变检测、控制循环分段计时）
- **文件组织**：
  ```
  tl_teleop/
  ├── src/tl_teleop.cpp           # 遥操作节点实现
  ├── include/tl_teleop/tl_teleop.h
  ├── launch/                     # tl_teleop_6axis.launch.py、tl_teleop_7axis.launch.py
  ├── config/                     # tl_teleop_6axis_config.yaml、tl_teleop_7axis_config.yaml
  ├── lib/
  │   ├── include/PXREARobotSDK.h # PXREA SDK C API 头文件
  │   ├── arm/libPXREARobotSDK.so # ARM 架构预编译库
  │   └── x86/libPXREARobotSDK.so # x86 架构预编译库
  ├── CMakeLists.txt
  └── package.xml
  ```

### tl_teleop_f710
- **构建类型**：ament_cmake（C++17）
- **用途**：Logitech F710 手柄遥操作 — `joy_node → /joy`，节点内自行做笛卡尔→关节 IK，以 250 Hz（4 ms）稳定输出关节角到 `/tl_driver/set_servoj_pos`，指令流不中断；真机 / Gazebo 仿真双模式
- **依赖**：`tl_ros2_interface`、`kdl_parser`/`orocos_kdl_vendor`（仿真模式 IK）、`ament_index_cpp`，并以 `<exec_depend>` 声明 `tl_driver`（运行期经 `/tl_driver/*` 服务与话题通信）— 不链接 `tl_driver` 的库
- **真机模式**：启动时自动执行 `connect_arm → power_on → set_current_mode(2) → set_speed → open_servoj`；退出时 `close_servoj → 切回示教模式 → power_off`；以首帧 `/joint_states` 作为初始指令，启动不移动到零位
- **仿真模式**：节点发布 `ServolMove` 到 `/tl_driver/set_servol_pos`，同包 `tl_teleop_f710_sim_bridge` 用 KDL `ChainIkSolverPos_LMA` 做 IK 并驱动 Gazebo position controller（仿真模式不依赖 tl_driver）
- **启动**：
  - 真机：`ros2 launch tl_teleop_f710 tl_teleop_f710_6axis.launch.py`（7 轴用 `tl_teleop_f710_7axis.launch.py`）— 同时拉起 `joy_node`
  - 仿真：`ros2 launch tl_teleop_f710 tl_teleop_f710_6axis_gazebo.launch.py arm_type:=<arm_type>` — 内部 include `tl_gazebo` 的 `gazebo_<n>axis_f710_sim.launch.py`，再启动 `joy_node` + 遥操作节点 + sim_bridge
- **配置**：`config/` 下 4 份 YAML（6/7 轴 × 真机/`_sim`）；`home_joints` 长度决定轴数
- **udev**：`udev/99-logitech-f710.rules` 需拷入 `/etc/udev/rules.d/` 并 reload rules；手柄拨到 **D（DirectInput）** 模式
- **手柄映射**：摇杆控制笛卡尔运动、十字键上下调速度、LB/RB 切姿态控制模式（偏航/翻滚/俯仰）、A 键回零、Back+Start 暂停/恢复

### tl_hardware
- **构建类型**：ament_cmake
- **用途**：ros2_control `SystemInterface` 硬件接口插件（插件名 `tl_hardware/TLHardwareInterface`，经 `tl_hardware_interface.xml` 导出），把 MoveIt2 的 `joint_trajectory_controller` 接到 `tl_driver`
- **依赖**：ros2_control 的 `hardware_interface`、`pluginlib`、`trajectory_msgs` 等，并以 `<exec_depend>` 声明 `tl_driver`（运行期经 `/tl_driver/*` 服务与话题通信）— 不链接 `tl_driver` 的库
- **数据通路**：读 `/joint_states`（best-effort QoS，位置差分算速度）；写 `/tl_driver/set_servoj_pos`（`std_msgs/Float64MultiArray`，单位为**角度**，与 ros2_control 的弧度制需转换）；`on_activate()` 调 `/tl_driver/open_servoj`（传 vmax/amax/jmax），关闭用 `/tl_driver/close_servoj`
- **生命周期**：`on_configure()` 创建内部节点 `tl_hardware` 与后台 `SingleThreadedExecutor` 线程；`on_activate()` 等待首帧关节状态，超时 5 s 返回 `CallbackReturn::ERROR`（**激活失败**，非仅告警），并用当前关节角播种命令接口（ros2_control 在控制器接管前就激活硬件，否则会先持续下发零位把机械臂拉向零点）；激活后 `read()` 若超过 `state_timeout_sec` 未收到状态，则每 5 s 节流告警一次，不自动 shutdown（由上层控制器处理）
- **启用方式**：`tl_moveit2_config` 各子包的 `config/tl_<arm_type>.ros2_control.xacro` 通过 `use_real_hardware` 开关选用该插件，参数为话题/服务名与 servoj 运动参数

### tl_example
- **构建类型**：ament_cmake（C++17）
- **用途**：示例节点；当前仅 `medical_demo` — 医学检验科自动化场景的队列 MoveL（`coord=1`）演示：切示教模式 → 显式上电 → 逐条下发队列运动 → 下电
- **入口**：`src/medical_demo.cpp`（`tl_example::MedicalDemo`），头文件 `include/tl_example/medical_demo.h`
- **运行**：`ros2 run tl_example medical_demo`（需 `tl_driver` 已连接机械臂）

### tl_description
- **构建类型**：ament_cmake
- **用途**：URDF 模型 + 网格文件 + robot_state_publisher + RViz 配置
- **无编译代码** — 纯数据包（URDF、STL 网格、.rviz 配置）
- **启动**：`ros2 launch tl_description tl_description.launch.py arm_type:=<arm_type> use_sim:=<true|false>`
- **按臂型启动**：`ros2 launch tl_description <arm_type>_description.launch.py use_sim:=<true|false>`（每种臂型一个文件，参数只有 `use_sim`）
- **use_sim=true**：启动 `joint_state_publisher_gui`，通过滑动条手动控制关节
- **use_sim=false**：订阅 `/joint_states`（需要 tl_driver 运行中）

### tl_bringup
- **构建类型**：ament_cmake
- **用途**：启动聚合器 — 同时启动 tl_driver + tl_description
- **无编译代码** — 仅启动文件
- **启动**：`ros2 launch tl_bringup tl_<arm_type>_bringup.launch.py`
- **每种臂型一个启动文件**（共 14 个，如 `tl_tcb605_bringup.launch.py`）

### tl_gazebo
- **构建类型**：ament_cmake
- **用途**：在 Gazebo 仿真环境中加载机械臂模型，通过 ros2_control 控制虚拟机械臂
- **启动**：`ros2 launch tl_gazebo gazebo_<arm_type>_demo.launch.py`
- **F710 仿真环境**：`gazebo_6axis_f710_sim.launch.py` / `gazebo_7axis_f710_sim.launch.py` — 只负责 Gazebo 与控制器（`ros2_controllers_f710_sim_*axis.yaml`），不含遥操作节点，由 `tl_teleop_f710` 的 `_gazebo` launch include
- **配合 MoveIt2**：`ros2 launch tl_<arm_type>_config gazebo_moveit_demo_<arm_type>.launch.py`

### tl_moveit2_config
- **构建类型**：ament_cmake（14 个子功能包集合，每个型号一套）
- **用途**：MoveIt2 运动规划配置，包含 SRDF、关节限位、运动学求解器（KDL）、控制器配置
- **启动**：
  - 虚拟控制：`ros2 launch tl_<arm_type>_config demo.launch.py`
  - Gazebo 仿真：`ros2 launch tl_<arm_type>_config gazebo_moveit_demo_<arm_type>.launch.py`
  - 真实机械臂：`ros2 launch tl_<arm_type>_config real_hardware_demo.launch.py` — xacro 传 `use_real_hardware:=true` 启用 `tl_hardware/TLHardwareInterface`，拉起 `ros2_control_node` + 控制器 spawner + `move_group` + RViz（**不含 tl_driver**，需另行启动）
- **配置**：每个子包包含 `config/`（initial_positions、joint_limits、kinematics、srdf、ros2_controllers、`tl_<arm_type>.ros2_control.xacro` 等）和 `launch/`（demo、real_hardware_demo、gazebo_moveit_demo、move_group、rviz 等）

## 支持的臂型

启动参数中全部小写：`tcb605`、`tcb605f`、`tcb605l`、`tcb605lv`、`tcb605v`、`tcb610`、`tcb610v`、`tcb705`、`tcb705f`、`tcb705l`、`tcb705lv`、`tcb705v`、`tcb710`、`tcb710v`

配置 YAML 中 `arm_type` 字段用大写：如 `TCB605`

## 关键话题

| 话题 | 发布者 | 订阅者 | 类型 |
|------|--------|--------|------|
| `/joint_states` | tl_driver（真实）/ joint_state_publisher_gui（仿真） | tl_description、tl_teleop_f710、tl_hardware | `sensor_msgs/JointState` |
| `/tcp_pose` | tl_driver | tl_teleop | `tl_ros2_interface/CartesianPose` |
| `/arm_status` | tl_driver | tl_example | `tl_ros2_interface/ArmStatus` |
| `/tl_driver/moveJ` | — | tl_driver | `tl_ros2_interface/MoveCommand` |
| `/tl_driver/moveL` | tl_example | tl_driver | `tl_ros2_interface/MoveCommand` |
| `/tl_driver/set_servoj_pos` | tl_teleop / tl_teleop_f710 / tl_hardware | tl_driver | `std_msgs/Float64MultiArray`（关节角，度） |
| `/tl_driver/set_servol_pos` | tl_teleop_f710（仿真模式） | tl_driver、tl_teleop_f710_sim_bridge | `tl_ros2_interface/ServolMove`（笛卡尔位姿） |
| `/joy` | joy_node（tl_teleop_f710） | tl_teleop_f710 | `sensor_msgs/Joy` |
| `/tf`、`/tf_static` | tl_description（robot_state_publisher） | — | `tf2_msgs/TFMessage` |

## 注意事项

- **`_tl_host.so`** 是预编译专有库，禁止尝试重新编译或修改。构建时链接，安装到 `lib/tl_driver/`。
- **tl_driver 使用 `MultiThreadedExecutor`** 驱动 3 个回调组（`service_group_`、`topic_group_`、`timer_group_`），三组均为 `MutuallyExclusive`：服务、话题、定时器回调各自串行，组间可并行。这是回调组架构正常工作的必要条件（历史上 `timer_group_` 曾为 `Reentrant`，已改为互斥）。
- **选择性构建时必须先构建 tl_ros2_interface**。不带 `--packages-select` 的 `colcon build` 会自动处理。
- **机械臂位置单位**：NRC SDK 以 mm 表示笛卡尔位置，ROS2 接口**有意沿用 mm**（偏离 ROS 惯例 m，已评估并保留）——`/tcp_pose.position`、`/tl_driver/set_servol_pos` 的 `target_pose`/`step_size`、`MoveCommand.target_pos_value`（`coord≠0` 时）、`set_user_coord`、`set_tool_param` 的 `x/y/z` 与负载质心、`coord_transform`、`get_pos_reachable`、`set/get_global_pos` 均按 mm 解释；姿态为 rad，关节量为度（`/joint_states` 已换算为 rad）。消费方**不要**自行 ×1000/÷1000；逐字段口径见本文件下方「关键话题」与 `src/tl_driver/doc/tl_driver服务与话题说明书.md` §1.4，跨层不变量见 `.omp/WATCHDOG.md`。欧拉角约定为 XYZ 内旋（scipy 中使用大写 `'XYZ'`）。
- **无单元测试**。测试面为两层：`ament_lint_auto` 代码风格检查（tl_bringup、tl_description、tl_gazebo、tl_teleop、tl_teleop_f710 的 `CMakeLists.txt` 中启用）与 `src/tl_driver/test/` 下的手工接口脚本（`test_moveJ.sh`、`test_moveL.sh`、`test_job_insert_*.sh`、`test_publisher.py`），后者需机械臂在线，手动运行。
- **提交前必须跑**：`./scripts/format-cpp.sh`（C++）与 `black .`（Python）；CI（`.github/workflows/ci.yml`）会对 push/PR 强制检查，不通过即失败。`isort`（import 排序）不纳入 CI——仓库历史 import 顺序存在漂移，需要时自行运行 `isort .`。
- **开发环境通过 Docker 搭建**（Docker 配置不在本仓库中）。构建和运行均在容器内进行。
- **发版**：在 `master`/`dev`/`V2` 分支上打 `V主.次.补`（可带 `-rc`/`-beta`）标签即触发 `.github/workflows/release.yml`——先校验发布条件，再由 `scripts/release-notes.sh` 从 `CHANGELOG.md` 抽出该标签的版本章节作 Release 正文（`V2.0.1` → `## [2.0.1]`，`-rc`/`-beta` 标签回退到基础版本章节），随后跑格式检查并创建 GitHub Release（标题即标签名，正文 = 章节内容 + 完整变更日志链接，不用 GitHub 自动生成的提交/PR 列表）。**找不到对应章节或章节为空时发布失败、不创建 Release**——须先把 `[Unreleased]` 内容合并进版本号章节并推送分支，再把标签**重新指向含该章节的提交**（`git tag -f Vx.y.z <提交> && git push -f origin Vx.y.z`）；仅删除并重推同一标签仍指向旧提交，会再次失败。`V2` 为独立版本线，其标签只发布 V2 系列版本。
- **发版顺序（必须）**：先把分支推上去并等 CI 绿，再打标签：`git push origin <分支>` → CI 通过 → `git tag Vx.y.z && git push origin Vx.y.z`。`release.yml` 的守卫用「标签提交是否为远端 `master`/`dev`/`V2` 的祖先」判定，**只推标签不推分支时远端分支引用还停在旧位置**，守卫会静默跳过发布（只有标签、没有 Release）。误推时补推分支后重推标签即可。首次在 V2 线发版前，本地曾启用过 `.githooks` 的克隆建议执行一次 `git config --unset core.hooksPath`。

## 文档与变更日志同步规则

**核心规则**：任何代码/配置变更，**凡命中下方检查清单任一项**，文档必须在**同一提交（或同一 PR）内**同步。禁止"代码先合、文档后续再说"。

### 提交前强制检查（起草 commit 前逐项核对）

1. 是否新增/删除/修改 ROS2 接口（msg/srv/话题/服务）？→ 更新 `CHANGELOG.md` + `src/tl_driver/doc/tl_driver服务与话题说明书.md`（接口包自身改 `src/tl_ros2_interface/README.md`）
2. 是否改变启动方式（launch 文件新增/改名/参数）或配置 YAML 参数？→ 更新对应包 `README.md` 的启动/配置章节
3. 是否新增/删除功能包或改变包依赖关系？→ 更新 `CHANGELOG.md` + 本文件依赖关系图与功能包说明 + 该包 `README.md`
4. 是否改变构建命令、命名规范、关键话题表、支持的臂型表？→ 更新 `AGENTS.md`（含本规则自身）
5. 是否修复用户可见 Bug、升级 SDK、新增臂型支持？→ 更新 `CHANGELOG.md`
6. 是否改变行为（通信协议、单位、上电时序、默认参数、回调组架构、公共接口签名）？→ 更新 `CHANGELOG.md` + 相关文档
7. 是否动到被文档引用的路径/名称/命令？→ 修正所有引用处

仅当**全部为否**（纯内部重构：格式化、注释、命名统一）才可跳过文档同步。此类纯内部变更——构建与工具配置（`.gitignore`、`.clang-format`、`pyproject.toml`、格式化脚本、`.github/workflows/**`）、agent/harness 配置（`.omp/**`、`.agents/**`）、目录重命名、文档润色——**不进 `CHANGELOG.md`**：该文件只记录使用者能感知的变化，内部调整写进去就是噪音。

**不得以任何理由跳过检查**。判为"无需文档"的变更必须能说出明确理由；说不出理由 = 漏了文档。

### CHANGELOG.md 条目标准

- 分类枚举固定为：新增 / 修复 / 变更 / 移除 / 文档 / 工程
- 「工程」类只写**用户可感知**的工程变更（依赖升级、构建链变化、SDK / ROS2 版本要求变化）；构建与工具配置、CI 工作流（`.github/workflows/**`）接入/迁移、格式化、agent/harness 配置、`package.xml` 元数据整理、目录重命名等内部调整一律不写（许可证变更除外——它影响用户能否再分发，写入「变更」）
- 未发布变更写在 `## [Unreleased]` 下，按日期分组（`### YYYY-MM-DD`，新 → 旧）
- 已发布版本章节写 `## [x.y.z] - YYYY-MM-DD`，只按类型分组，不再按日期分组
- 每条约一行，只描述**用户可见**变更（行为、接口、启动方式、依赖），不写内部实现细节；行尾附提交短哈希（如 `` `1ce6ad6` ``）便于溯源
- 发版时把 `[Unreleased]` 内容合并进版本号章节并按类型归类，然后重置 `[Unreleased]`；章节标题即为发版点（`## [x.y.z] - YYYY-MM-DD`），不另起「最新发布版本」说明行；该章节正文即 Release 正文来源（`release.yml` 找不到章节或章节为空时发布失败）
- 被回退的提交、合并提交等不产生用户可见变更的记录，写在文末「备注」中，不单列条目（例：`[2.0.0]` 备注中的 `a9139ac` 回退、`829c416` 合并提交）

### 文档对应关系速查

| 变更对象 | 必须同步的文档 |
|---|---|
| ROS2 接口（msg/srv/话题/服务） | `CHANGELOG.md` + `src/tl_driver/doc/tl_driver服务与话题说明书.md`、`src/tl_ros2_interface/README.md` |
| launch 文件 / 启动方式 / 配置参数 | 对应包 `README.md` |
| 功能包增减 / 依赖变化 | `CHANGELOG.md` + `AGENTS.md` + 该包 `README.md` |
| 构建命令 / 命名规范 / 臂型表 / 关键话题表 | `AGENTS.md` |
| 用户可见 Bug 修复 / SDK 升级 / 行为变更 | `CHANGELOG.md` |
| 手眼标定、工作空间测量等专项功能 | 对应专项文档（如 `scripts/workspace_measure/README.md`）+ `CHANGELOG.md` |

### 验收（提交前自检）

- 文档改动必须与代码改动**成对出现在同一提交**：检查清单有任一项为「是」而文档没跟上 = 变更未完成，禁止提交；判定为纯内部变更（全部为否）的提交不受此约束。
- 文档中引用的路径、launch 命令、话题/服务名、参数名必须与代码一致，不一致视为缺陷。
- `CHANGELOG.md` 条目里的短哈希必须真实存在于本分支历史（`git log --oneline | grep <hash>`）。

## 命名规范

### C++ 命名规范

| 元素 | 规范 | 示例 |
|------|------|------|
| **文件名** | snake_case | `tl_driver.cpp`、`tl_driver.h` |
| **类名** | PascalCase | `TL_Arm`、`MessageLists` |
| **枚举名** | PascalCase | `MessageLists` |
| **枚举值** | UPPER_SNAKE_CASE | `ROBOT_STATE`、`SUCCESS`、`RECEIVE_FAILED` |
| **成员变量** | snake_case + 下划线后缀 | `arm_ip_`、`socket_fd_`、`is_connected_`、`joint_state_pub_` |
| **普通变量** | snake_case | `arm_ip`、`socket_fd`、`state` |
| **成员函数** | camelCase | `handle_connect_service`、`power_on`、`publish_arm_state` |
| **ROS 服务回调** | `handle_` + `{service}` + `_service` | `handle_connect_service`、`handle_set_speed_service` |
| **ROS 话题回调** | `handle_` + `{topic}` + `_topic` | `handle_movej_topic`、`handle_movel_topic` |
| **命名空间** | ROS 标准（`::` 分隔） | `tl_ros2_interface::srv::SetSpeed` |
| **头文件宏保护** | `包名__文件名_H_` | `#ifndef TL_DRIVER__TL_DRIVER_H_` |
| **静态内联变量** | snake_case（下划线前缀可选） | `msg_id`、`msg`、`msg_received` |
| **ROS msg/srv 类型** | snake_case（自动生成） | `MoveCommand`、`CartesianPose`、`ArmStatus` |
| **参数默认值** | 小写字符串（ROS约定） | `"arm_ip"`、`"6001"`、`"TCB605"` |
| **回调函数指针** | lambda + bind 模式 | `std::bind(&TL_Arm::handle_..., this, ...)` |

**注意**：
- 服务句柄变量命名：`{service_name}_service_`（如 `connect_service_`、`set_speed_service_`）
- 话题订阅变量命名：`{topic_name}_sub_`（如 `movej_sub_`、`movel_sub_`）
- 话题发布变量命名：`{topic_name}_pub_`（如 `joint_state_pub_`、`tcp_pose_pub_`）
- 以下为已知例外（历史遗留，服务/话题名与变量名不完全对应）：
  - `connect_service_` 对应 `/tl_driver/connect_arm`（非 `connect_arm_service_`）
  - `poweron_service_` / `poweroff_service_` 对应 `/tl_driver/power_on` / `power_off`
  - `running_status_pub_` 发布到 `/arm_status`（非 `arm_status_pub_`）

### Python 命名规范

| 元素 | 规范 | 示例 |
|------|------|------|
| **文件名** | snake_case | `control_node.py`、`calib_node.py` |
| **类名** | PascalCase | `HandEyeCalibrationNode`、`TLDemoNode` |
| **ROS 节点类** | PascalCase，继承 `Node` | `class TLDemoNode(Node)` |
| **实例变量** | snake_case | `robot_ip`、`camera_object_topic`、`base_frame_id` |
| **私有方法** | 下划线前缀 + snake_case | `_tcp_pose_callback`、`_wait_for_services` |
| **公开方法** | snake_case | `safe_log_info`、`get_robot_pose`、`pose_to_tool_rt` |
| **ROS 参数键** | snake_case | `'arm_ip'`、`'camera_width'`、`'handeye_matrix'` |
| **ROS 话题名** | snake_case（小写） | `/joint_states`、`/tcp_pose` |
| **ROS 服务名** | snake_case（小写） | `/tl_driver/connect_arm`、`/tl_driver/power_on` |
| **入口点函数** | snake_case | `demo_node`、`control_node` |
| **console_scripts** | snake_case（与文件名对应） | `demo_node = tl_driver.demo_node:main` |
| **标准库导入** | 常用别名 | `import numpy as np`、`import cv2` |
| **ROS 客户端** | 下划线前缀 + `_cli` 后缀 | `self._connect_cli`、`self._power_on_cli` |
| **订阅者** | 下划线前缀 + `_sub` 后缀 | `self._tcp_pose_sub` |

### ROS 话题/服务命名规范

- 所有话题和服务名使用 **snake_case（小写+下划线）**
- 包名前缀：`/tl_driver/`
- 示例话题： `/joint_states`、`/tcp_pose`、`/arm_status`
- 示例服务： `/tl_driver/connect_arm`、`/tl_driver/set_speed`

### 文件组织规范

```
tl_driver/
├── src/tl_driver.cpp              # 主节点实现（camelCase 方法）
├── include/tl_driver/tl_driver.h  # 头文件（类定义）
├── launch/tl_driver.launch.py     # 通用启动文件
├── launch/tl_tcbXXX_driver.launch.py  # 各臂型快捷启动
└── config/*.yaml                  # 配置文件
```

## Sisyphus 后台任务超时规避

后台 explore/librarian 任务有 **30 分钟无活动超时限制**。超大代码库搜索时容易触发。规避方法：

- **每个 explore agent 只查 1-2 个具体模式**，不要塞 5+ 个搜索需求到一个 prompt
- **已知文件位置**（如已确定路径的文件）直接用 `grep`/`read`/`glob` 直接工具，不 delegation
- **大范围搜索拆成多个并行小任务**，每个小任务限定搜索范围（`path`、`include`、`globs` 参数）
- 如果需要跨包/跨语言搜索（如同时查 C++ 和 Python），必须拆成多个并行 agent

<!-- gitnexus:start -->
# GitNexus — Code Intelligence

This project is indexed by GitNexus as **tl_robot_ros2_cpp** (5344 symbols, 6830 relationships, 0 execution flows). Use the GitNexus MCP tools to understand code, assess impact, and navigate safely.

> If any GitNexus tool warns the index is stale, run `npx gitnexus analyze` in terminal first.

## Always Do

- **MUST run impact analysis before editing any symbol.** Before modifying a function, class, or method, run `gitnexus_impact({target: "symbolName", direction: "upstream"})` and report the blast radius (direct callers, affected processes, risk level) to the user.
- **MUST run `gitnexus_detect_changes()` before committing** to verify your changes only affect expected symbols and execution flows.
- **MUST warn the user** if impact analysis returns HIGH or CRITICAL risk before proceeding with edits.
- When exploring unfamiliar code, use `gitnexus_query({query: "concept"})` to find execution flows instead of grepping. It returns process-grouped results ranked by relevance.
- When you need full context on a specific symbol — callers, callees, which execution flows it participates in — use `gitnexus_context({name: "symbolName"})`.

## Never Do

- NEVER edit a function, class, or method without first running `gitnexus_impact` on it.
- NEVER ignore HIGH or CRITICAL risk warnings from impact analysis.
- NEVER rename symbols with find-and-replace — use `gitnexus_rename` which understands the call graph.
- NEVER commit changes without running `gitnexus_detect_changes()` to check affected scope.

## Resources

| Resource | Use for |
|----------|---------|
| `gitnexus://repo/tl_robot_ros2_cpp/context` | Codebase overview, check index freshness |
| `gitnexus://repo/tl_robot_ros2_cpp/clusters` | All functional areas |
| `gitnexus://repo/tl_robot_ros2_cpp/processes` | All execution flows |
| `gitnexus://repo/tl_robot_ros2_cpp/process/{name}` | Step-by-step execution trace |

## CLI

| Task | Read this skill file |
|------|---------------------|
| Understand architecture / "How does X work?" | `.opencode/skills/gitnexus/gitnexus-exploring/SKILL.md` |
| Blast radius / "What breaks if I change X?" | `.opencode/skills/gitnexus/gitnexus-impact-analysis/SKILL.md` |
| Trace bugs / "Why is X failing?" | `.opencode/skills/gitnexus/gitnexus-debugging/SKILL.md` |
| Rename / extract / split / refactor | `.opencode/skills/gitnexus/gitnexus-refactoring/SKILL.md` |
| Tools, resources, schema reference | `.opencode/skills/gitnexus/gitnexus-guide/SKILL.md` |
| Index, status, clean, wiki CLI commands | `.opencode/skills/gitnexus/gitnexus-cli/SKILL.md` |

<!-- gitnexus:end -->
