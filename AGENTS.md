# AGENTS.md — 天链机器人 ROS2 工作空间

## 工作空间概述

天链（TianLian）机械臂 ROS2 工作空间。`src/` 下包含 5 个功能包，使用标准 `colcon build` 构建流程。无 `package.json`、无 Node.js — 纯 ROS2（ament_cmake + ament_python）。

## 构建命令

（colcon 自动解析拓扑顺序）：
```bash
colcon build
source install/setup.bash
```

构建产物在 `build/`、`install/`、`log/` — 均已 gitignore。

## 功能包依赖关系

```
tl_ros2_interface  （基础：自定义 msg/srv，无依赖）
  └─► tl_driver       （C++ 节点，链接 _nrc_host.so 专有库）
  └─► tl_vision       （Python ament_python 包，同时依赖 tl_driver）
tl_description     （独立：URDF + 网格 + RViz）
tl_bringup         （启动聚合器：包含 tl_driver + tl_description）
```

## 功能包说明

### tl_ros2_interface
- **构建类型**：ament_cmake
- **用途**：定义所有自定义 ROS2 接口（12 个 `.msg`，41 个 `.srv`）
- **关键消息**：`ObjectInfo`、`ArmStatus`、`CartesianPose`、`MoveCommand`
- **关键服务**：`GetCurrentCoord`、`SetSpeed`、`Jogging`、`ModbusRead/Write`、`JobRun`
- **必须最先构建** — 其他包依赖其生成的头文件（colcon 会自动处理构建顺序）

### tl_driver
- **构建类型**：ament_cmake（C++17）
- **用途**：机械臂驱动 — 通过 TCP 与实体机械臂通信
- **入口**：`src/tl_driver.cpp` → 单一 `tl_driver` 可执行文件
- **专有库**：`lib/_nrc_host.so`（预编译共享库，不可修改）
- **Python API**：`lib/nrc_interface.py` — 运行时通过 `sys.path` 加载的 Python 封装
- **配置**：`config/` 下按臂型命名的 YAML（如 `tl_tcb605_config.yaml`）
- **启动**：
  - 通用：`ros2 launch tl_driver tl_driver.launch.py arm_type:=<arm_type>`
  - 快捷：`ros2 launch tl_driver tl_tcb710_driver.launch.py`（每种臂型一个专用文件，如 `tl_tcbXXX_driver.launch.py`）
- **默认机械臂 IP**：`192.168.1.13`，端口 `6001` — 如需修改，改对应配置 YAML

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

### tl_vision
- **构建类型**：ament_python（纯 Python，非 ament_cmake）
- **用途**：YOLO 目标检测 + 手眼标定 + 抓取控制
- **入口点**（来自 `setup.py`）：
  - `yolo_node` — 订阅 RGB+深度图，运行 YOLO，发布 `/tl_vision/object_3d_pos_camera`
  - `calib_node` — 手眼标定（在线用 RealSense，或离线从 `.npz` 计算）
  - `control_node` — 将相机坐标转换到基座坐标，通过 `nrc_interface` 控制机械臂
  - `fk_test_node` — 正运动学测试 UI
- **配置**：`config/yolo_node.yaml`、`config/control_node.yaml`、`config/calib_node.yaml`、`config/camera_params.yaml`
- **启动**：`ros2 launch tl_vision bringup.launch.py`（启动 RealSense + yolo_node + control_node）
- **hand_eye_calibration-main-1/**：独立标定脚本（非 ROS2 节点），直接用 `python` 运行

## 支持的臂型

启动参数中全部小写：`tcb605`、`tcb605f`、`tcb605l`、`tcb605lv`、`tcb605v`、`tcb610`、`tcb610v`、`tcb705`、`tcb705f`、`tcb705l`、`tcb705lv`、`tcb705v`、`tcb710`、`tcb710v`

配置 YAML 中 `arm_type` 字段用大写：如 `TCB605`

## 关键话题

| 话题 | 发布者 | 订阅者 | 类型 |
|------|--------|--------|------|
| `/joint_states` | tl_driver（真实）/ joint_state_publisher_gui（仿真） | tl_description | `sensor_msgs/JointState` |
| `/tl_vision/object_3d_pos_camera` | yolo_node | control_node | `tl_ros2_interface/ObjectInfo` |
| `/tl_vision/object_3d_pos_base` | control_node | — | `tl_ros2_interface/ObjectInfo` |
| `/tf`、`/tf_static` | tl_description（robot_state_publisher） | — | `tf2_msgs/TFMessage` |

## 注意事项

- **control_node 和 calib_node 均通过 `ament_index` 发现机制**加载 `nrc_interface`（`get_package_prefix('tl_driver')` + `lib/tl_driver`），开发/部署环境均适用。
- **`_nrc_host.so`** 是预编译专有库，禁止尝试重新编译或修改。构建时链接，安装到 `lib/tl_driver/`。
- **选择性构建时必须先构建 tl_ros2_interface**。不带 `--packages-select` 的 `colcon build` 会自动处理。
- **机械臂位置单位**：NRC API 返回 mm；control_node 除以 1000 转换为米。欧拉角约定为 XYZ 内旋（scipy 中使用大写 `'XYZ'`）。
- **control_node.yaml 中的 handeye_matrix** 是 16 个元素的扁平列表，表示 4×4 齐次变换矩阵（行优先）。默认为单位矩阵 — 必须替换为实际标定结果。
- **tl_vision/hand_eye_calibration-main-1/** 是独立 Python 工具，不属于 ROS2 包。直接用 `python compute_in_hand.py` 或 `python compute_to_hand.py` 运行。
- **无自动化测试**，仅有 ament 代码风格检查脚手架。`test/` 目录只包含 `ament_copyright`、`ament_flake8`、`ament_pep257`。
- **开发环境通过 Docker 搭建**（Docker 配置不在本仓库中）。构建和运行均在容器内进行。

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

### Python 命名规范

| 元素 | 规范 | 示例 |
|------|------|------|
| **文件名** | snake_case | `control_node.py`、`yolo_node.py`、`calib_node.py` |
| **类名** | PascalCase | `ObjectToBaseNode`、`YOLODemo`、`HandEyeCalibrationNode` |
| **ROS 节点类** | PascalCase，继承 `Node` | `class ObjectToBaseNode(Node)` |
| **实例变量** | snake_case | `robot_ip`、`camera_object_topic`、`base_frame_id` |
| **私有方法** | 下划线前缀 + snake_case | `_tcp_pose_callback`、`_wait_for_services` |
| **公开方法** | snake_case | `safe_log_info`、`get_robot_pose`、`pose_to_tool_rt` |
| **ROS 参数键** | snake_case | `'arm_ip'`、`'camera_width'`、`'handeye_matrix'` |
| **ROS 话题名** | snake_case（小写） | `/tl_vision/object_3d_pos_camera`、`/tcp_pose` |
| **ROS 服务名** | snake_case（小写） | `/tl_driver/connect_arm`、`/tl_driver/power_on` |
| **入口点函数** | snake_case | `yolo_node`、`calib_node`、`control_node`、`fk_test_node` |
| **console_scripts** | snake_case（与文件名对应） | `yolo_node = tl_vision.yolo_node:main` |
| **标准库导入** | 常用别名 | `import numpy as np`、`import cv2` |
| **ROS 客户端** | 下划线前缀 + `_cli` 后缀 | `self._connect_cli`、`self._power_on_cli` |
| **订阅者** | 下划线前缀 + `_sub` 后缀 | `self._tcp_pose_sub` |

### ROS 话题/服务命名规范

- 所有话题和服务名使用 **snake_case（小写+下划线）**
- 包名前缀：`/tl_driver/`、`/tl_vision/`、`/tl_driver/`
- 示例话题： `/joint_states`、`/tcp_pose`、`/arm_status`
- 示例服务： `/tl_driver/connect_arm`、`/tl_driver/set_speed`

### 文件组织规范

```
tl_driver/
├── src/tl_driver.cpp          # 主节点实现（camelCase 方法）
├── include/tl_driver/tl_driver.h  # 头文件（类定义）
├── launch/tl_driver.launch.py # 通用启动文件
├── launch/tl_tcbXXX_driver.launch.py # 各臂型快捷启动
└── config/*.yaml              # 配置文件

tl_vision/
├── tl_vision/
│   ├── control_node.py        # snake_case 文件名
│   ├── yolo_node.py
│   └── calib_node.py
└── launch/*.launch.py
```

## Sisyphus 后台任务超时规避

后台 explore/librarian 任务有 **30 分钟无活动超时限制**。超大代码库搜索时容易触发。规避方法：

- **每个 explore agent 只查 1-2 个具体模式**，不要塞 5+ 个搜索需求到一个 prompt
- **已知文件位置**（如已确定路径的文件）直接用 `grep`/`read`/`glob` 直接工具，不 delegation
- **大范围搜索拆成多个并行小任务**，每个小任务限定搜索范围（`path`、`include`、`globs` 参数）
- 如果需要跨包/跨语言搜索（如同时查 C++ 和 Python），必须拆成多个并行 agent

<!-- gitnexus:start -->
# GitNexus — Code Intelligence

This project is indexed by GitNexus as **demo_ws** (5464 symbols, 7513 relationships, 75 execution flows). Use the GitNexus MCP tools to understand code, assess impact, and navigate safely.

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
| `gitnexus://repo/demo_ws/context` | Codebase overview, check index freshness |
| `gitnexus://repo/demo_ws/clusters` | All functional areas |
| `gitnexus://repo/demo_ws/processes` | All execution flows |
| `gitnexus://repo/demo_ws/process/{name}` | Step-by-step execution trace |

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
