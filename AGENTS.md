# AGENTS.md — 天链机器人 ROS2 工作空间

## 工作空间概述

天链（TianLian）机械臂 ROS2 工作空间。`src/` 下包含 10 个功能包及 `scripts/` 工具脚本，使用标准 `colcon build` 构建流程。无 `package.json`、无 Node.js — 纯 ROS2（ament_cmake + ament_python）。

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
  └─► tl_driver       （C++ 节点，链接 libtl_host.so 专有库）
  └─► tl_teleop       （VR 遥操作 C++ 节点，PXREA Robot SDK，依赖接口消息）
  └─► tl_teleop_f710  （F710 手柄遥操作 C++ 节点，依赖接口消息）
  └─► tl_hardware     （ros2_control 硬件接口插件，依赖接口消息）
  └─► tl_example      （示例程序，依赖接口消息）
tl_description     （独立：URDF + 网格 + RViz）
  └─► tl_gazebo       （Gazebo 仿真，依赖 tl_description）
  └─► tl_moveit2_config（MoveIt2 配置集合，依赖 tl_description）
tl_bringup         （启动聚合器：包含 tl_driver + tl_description）
scripts/           （工作空间测量等工具脚本，不参与 colcon 构建）
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
- **专有库**：`lib/arm/`（ARM 架构）与 `lib/x86/`（x86 架构）下各一份预编译 `libtl_host.so`（V3.0.2），不可修改；`lib/include/` 为 C/C++ API 头文件（扁平结构，14 个 `.h`）
- **配置**：`config/` 下按臂型命名的 YAML（如 `tl_tcb605_config.yaml`）。关键参数：`arm_ip`、`arm_port`（TCP 主端口）、`arm_port_aux`（TCP 辅助端口）、`arm_type`、`arm_joints`
- **启动**：
  - 通用：`ros2 launch tl_driver tl_driver.launch.py arm_type:=<arm_type>`
  - 快捷：`ros2 launch tl_driver tl_tcb710_driver.launch.py`（每种臂型一个专用文件，如 `tl_tcbXXX_driver.launch.py`）
- **默认机械臂 IP**：`192.168.1.13`，端口 `6001` — 如需修改，改对应配置 YAML
- **回调组架构**（`TL_Arm` 构造函数中创建 3 组）：
  - `service_group_`（`MutuallyExclusive`）— 全部 65 个服务，保证服务回调串行执行
  - `topic_group_`（`MutuallyExclusive`）— 4 个话题订阅，保证话题回调串行执行
  - `timer_group_`（`Reentrant`）— 状态发布定时器（100 Hz），允许定时器回调并发
- **话题**：发布 `joint_states`、`tcp_pose`、`arm_status`；订阅 `moveJ`、`moveL`、`set_servoj_pos`、`set_servol_pos`（详见下方关键话题表）
- **安全行为**：`init()` 中若 `connect()` 失败，节点会调用 `rclcpp::shutdown()` 并 exit

### tl_teleop
- **构建类型**：ament_cmake（C++17）
- **用途**：遥操作节点 — 通过 PXREA Robot SDK（预编译 `.so`）与遥操作设备通信，同时在 ROS2 层面通过 `tl_ros2_interface` 的消息与服务与 `tl_driver` 交互
- **专有库**：`lib/arm/`（ARM 架构）和 `lib/x86/`（x86 架构）下的预编译 `libPXREARobotSDK.so`，不可修改
- **SDK 头文件**：`lib/include/PXREARobotSDK.h` — C 风格 API，使用 `uint64_t`（需 `#include <stdint.h>`）
- **依赖**：`rclcpp` + `tl_ros2_interface` — 不直接链接 `tl_driver`，运行时通过话题/服务通信
- **实现状态**：已实现（`tl_teleop.cpp` 双线程架构：ROS2 事件循环 + 100Hz 控制线程；支持 6/7 轴、握紧触发、死区滤波、奇异点保护、关节跳变检测）
- **文件组织**：
  ```
  tl_teleop/
  ├── src/tl_teleop.cpp           # 遥操作节点实现
  ├── include/tl_teleop/tl_teleop.h
  ├── config/                     # tl_teleop_6axis/7axis_config.yaml
  ├── launch/                     # tl_teleop_6axis/7axis.launch.py
  ├── lib/
  │   ├── include/PXREARobotSDK.h # PXREA SDK C API 头文件
  │   ├── arm/libPXREARobotSDK.so # ARM 架构预编译库
  │   └── x86/libPXREARobotSDK.so # x86 架构预编译库
  ├── CMakeLists.txt
  └── package.xml
  ```

### tl_description
- **构建类型**：ament_cmake
- **用途**：URDF 模型 + 网格文件 + robot_state_publisher + RViz 配置
- **无编译代码** — 纯数据包（URDF、STL 网格、.rviz 配置）
- **启动**：`ros2 launch tl_description tl_description.launch.py arm_type:=<arm_type> use_sim:=<true|false>`
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
- **配合 MoveIt2**：`ros2 launch tl_<arm_type>_config gazebo_moveit_demo_<arm_type>.launch.py`

### tl_moveit2_config
- **构建类型**：ament_cmake（14 个子功能包集合，每个型号一套）
- **用途**：MoveIt2 运动规划配置，包含 SRDF、关节限位、运动学求解器（KDL）、控制器配置
- **启动**：
  - 虚拟控制：`ros2 launch tl_<arm_type>_config demo.launch.py`
- **配置**：每个子包包含 `config/`（initial_positions、joint_limits、kinematics、srdf 等）和 `launch/`（demo、move_group、rviz 等）

## 支持的臂型

启动参数中全部小写：`tcb605`、`tcb605f`、`tcb605l`、`tcb605lv`、`tcb605v`、`tcb610`、`tcb610v`、`tcb705`、`tcb705f`、`tcb705l`、`tcb705lv`、`tcb705v`、`tcb710`、`tcb710v`

配置 YAML 中 `arm_type` 字段用大写：如 `TCB605`

## 关键话题

| 话题 | 发布者 | 订阅者 | 类型 |
|------|--------|--------|------|
| `/joint_states` | tl_driver（真实）/ joint_state_publisher_gui（仿真） | tl_description | `sensor_msgs/JointState` |
| `/tcp_pose` | tl_driver | — | `tl_ros2_interface/CartesianPose` |
| `/arm_status` | tl_driver | — | `tl_ros2_interface/ArmStatus` |
| `/tl_driver/moveJ` | — | tl_driver | `tl_ros2_interface/MoveCommand` |
| `/tl_driver/moveL` | — | tl_driver | `tl_ros2_interface/MoveCommand` |
| `/tl_driver/set_servoj_pos` | — | tl_driver | `std_msgs/Float64MultiArray` |
| `/tl_driver/set_servol_pos` | tl_teleop_f710（仿真模式） | tl_driver | `tl_ros2_interface/ServolMove` |

| `/tf`、`/tf_static` | tl_description（robot_state_publisher） | — | `tf2_msgs/TFMessage` |

## CI（GitHub Actions）

|文件|触发|内容|
|---|---|---|
|`.github/workflows/ci.yml`|push master/dev、pull_request（`**.md`、`docs/**`、`.github/**` 变更不触发）|格式检查：clang-format（C++，经 `scripts/format-cpp.sh`）+ black（Python，版本锁定 26.5.1）|
|`.github/workflows/release.yml`|任意标签推送|guard 校验（标签名 `v主.次.补[-rc/beta]` + 位于 master/dev）→ 格式检查 → 创建 GitHub Release（自动 notes，rc/beta 为 prerelease）|

- **CI 不做构建**：依赖环境过重（MoveIt/RViz/ros2_control 全量安装），编译验证在本地 Docker 开发环境完成
- 格式检查容器为 `ubuntu:22.04`（clang-format 14），与本地工具链版本一致，避免新版 clang-format 格式化结果漂移

## 注意事项

- **`_tl_host.so`** 是预编译专有库，禁止尝试重新编译或修改。构建时链接，安装到 `lib/tl_driver/`。
- **tl_driver 使用 `MultiThreadedExecutor`** 驱动 3 个回调组（`service_group_`、`topic_group_`、`timer_group_`），保证服务/话题串行、定时器并发。这是回调组架构正常工作的必要条件。
- **选择性构建时必须先构建 tl_ros2_interface**。不带 `--packages-select` 的 `colcon build` 会自动处理。
- **机械臂位置单位**：NRC API 返回 mm；ROS2 层使用时需注意单位转换。欧拉角约定为 XYZ 内旋（scipy 中使用大写 `'XYZ'`）。
- **无自动化测试**，仅有 ament 代码风格检查脚手架。`test/` 目录只包含 `ament_copyright`、`ament_flake8`、`ament_pep257`。
- **开发环境通过 Docker 搭建**（Docker 配置不在本仓库中）。构建和运行均在容器内进行。

## 文档同步规则

**核心规则**：任何代码/配置变更，文档必须在**同一提交（或同一 PR）内**同步。禁止"代码先合、文档后续再说"。

### 提交前强制检查（起草 commit 前逐项核对）

对本次变更依次回答：

1. 是否新增/删除/修改 ROS2 接口（msg/srv/话题/服务）？→ 更新 `CHANGELOG.md` + 对应包的服务与话题说明书
2. 是否改变启动方式（launch 文件新增/改名/参数）或配置 YAML 参数？→ 更新对应包 `README.md` 的启动/配置章节
3. 是否新增/删除功能包或改变包依赖关系？→ 更新 `CHANGELOG.md` + AGENTS.md 依赖关系图 + 创建/删除该包 `README.md`
4. 是否改变构建命令、命名规范、关键话题表、支持的臂型表？→ 更新 AGENTS.md（含本规则自身）
5. 是否修复用户可见 Bug、升级 SDK、新增臂型支持？→ 更新 `CHANGELOG.md`
6. 是否改变行为（协议、单位、上电时序、默认参数、回调组架构、公共 API 签名）？→ 更新 `CHANGELOG.md` + 相关文档
7. 是否动到被文档引用的路径/名称/命令？→ 修正所有引用处

仅当**全部为否**（纯内部重构：格式化、注释、命名统一、工具配置）才可跳过文档同步，但属"清理/迁移"类变更仍需在 `CHANGELOG.md` 记录一行。

**不得以任何理由跳过检查**。判定为"无需文档"的变更，必须能说出明确理由；说不出理由 = 漏了文档。

### CHANGELOG.md 条目标准

- 未发布的变更记录在 `## [Unreleased]` 下，按日期分组（`### YYYY-MM-DD`，新 → 旧）
- 已发布的版本章节（`## [x.y.z] - YYYY-MM-DD`）按变更类型整理（新增/修复/变更/移除/文档/工程），不再按日期分组
- 条目描述用户可见变更，不写内部实现细节；每条约一行，末尾附提交短哈希（如 `（`dda23c0`）`）便于溯源
- 正式发布时，将 `[Unreleased]` 内容合并进版本号章节，并按类型归类；然后重置 `[Unreleased]`

### 文档对应关系速查

| 变更对象 | 必须同步的文档 |
|------|------|
| ROS2 接口（msg/srv/话题/服务） | `CHANGELOG.md` + 对应包服务与话题说明书（如 `src/tl_driver/doc/`） |
| launch 文件 / 启动方式 / 配置参数 | 对应包 `README.md` |
| 功能包增减 / 依赖变化 | `CHANGELOG.md` + AGENTS.md + 该包 `README.md` |
| 构建命令 / 命名规范 / 臂型表 / 关键话题表 | AGENTS.md |
| 用户可见 Bug 修复 / SDK 升级 / 行为变更 | `CHANGELOG.md` |
| 手眼标定、MoveIt2 等专项功能 | 对应专项文档 + `CHANGELOG.md` |

### 验收标准（提交前用 `git diff --stat` 自检）

- 文档文件与代码文件必须**成对出现在同一提交**；只有代码没有文档 = 变更未完成，禁止提交。
- 文档中引用的路径、launch 命令、话题/服务名必须与代码一致，不一致视为缺陷。

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
