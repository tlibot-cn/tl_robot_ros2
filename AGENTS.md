# AGENTS.md — 天链机器人 ROS2 工作空间

## 工作空间概述

天链（TianLian）机械臂 ROS2 工作空间。`src/` 下包含 5 个功能包，使用标准 `colcon build` 构建流程。无 `package.json`、无 Node.js — 纯 ROS2（ament_cmake + ament_python）。

## 构建命令

**构建顺序很重要** — 必须先构建 `tl_ros2_interface`（自定义 msg/srv 定义）：

```bash
colcon build --packages-select tl_ros2_interface
source install/setup.bash
colcon build --packages-select tl_driver tl_description tl_bringup tl_vision
```

或一次性构建（colcon 自动解析拓扑顺序）：
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
- **必须最先构建** — 其他包依赖其生成的头文件

### tl_driver
- **构建类型**：ament_cmake（C++17）
- **用途**：机械臂驱动 — 通过 TCP 与实体机械臂通信
- **入口**：`src/tl_driver.cpp` → 单一 `tl_driver` 可执行文件
- **专有库**：`lib/_nrc_host.so`（预编译共享库，不可修改）
- **Python API**：`lib/nrc_interface.py` — 运行时通过 `sys.path` 加载的 Python 封装
- **配置**：`config/` 下按臂型命名的 YAML（如 `tl_tcb605_config.yaml`）
- **启动**：`ros2 launch tl_driver tl_driver.launch.py arm_type:=<arm_type>`
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
