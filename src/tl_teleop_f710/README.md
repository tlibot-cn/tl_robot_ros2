# 天链机械臂 F710 手柄遥操作功能包 (C++)

## 概述

`tl_teleop_f710` 是[天链机器人](https://www.tl-robot.com/) ROS2 工作空间的一个 **C++** 功能包，
通过 **Logitech F710 游戏手柄**远程控制天链机械臂运动。

基于笛卡尔空间伺服（`/tl_driver/set_servol_pos` 话题）实现直观的末端位置控制，
操作者无需理解关节空间，推摇杆机械臂就往对应方向运动。

支持真机控制和 Gazebo 仿真两种模式。

## 适用型号

通用启动文件按轴数（6 轴 / 7 轴）分类，兼容以下所有天链机械臂型号：

| 轴数 | 支持型号 | 通用配置文件 | 通用启动文件 |
|------|---------|-------------|-------------|
| **6 轴** | TCB605、TCB605F、TCB605L、TCB605LV、TCB605V、TCB610 | `tl_teleop_f710_6axis.yaml` / `_sim.yaml` | `tl_teleop_f710_6axis.launch.py` / `_gazebo.launch.py` |
| **7 轴** | TCB610V、TCB705、TCB705F、TCB705L、TCB705LV、TCB705V、TCB710、TCB710V | `tl_teleop_f710_7axis.yaml` / `_sim.yaml` | `tl_teleop_f710_7axis.launch.py` / `_gazebo.launch.py` |

> 真机启动无需指定型号参数，YAML 中 `arm_type` 仅用于标识；
> 仿真启动需通过 `arm_type:=` 参数指定具体型号（见下文）。

## 环境要求

- **操作系统**：Ubuntu 22.04
- **ROS2 发行版**：Humble Hawksbill
- **前置依赖**：`tl_driver` 功能包（真机需先启动 tl_driver 节点）
- **硬件**：Logitech F710 游戏手柄

## 安装依赖

```bash
# 自动安装所有 ROS 依赖
cd ~/tl_robot_ros2_cpp
rosdep install --from-paths src --ignore-src -r -y
```

本包依赖以下 ROS2 标准包（Humble 自带）：

| 包名 | 用途 |
|------|------|
| `joy` | F710 手柄驱动（ros-humble-joy） |
| `kdl_parser` | URDF → KDL 运动学树解析 |
| `orocos_kdl_vendor` | KDL 运动学库（FK/IK 求解） |

## 安装 udev 规则（免 root 权限）

```bash
sudo cp src/tl_teleop_f710/udev/99-logitech-f710.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

然后将 F710 手柄接收器插入 USB 口，手柄开关拨到 **"D"（DirectInput 模式）**。

## 编译

```bash
cd ~/tl_robot_ros2_cpp
colcon build --packages-select tl_teleop_f710
source install/setup.bash
```

## 使用

### 真机模式

终端 1 — 启动机械臂驱动（以 TCB605 为例）：

```bash
ros2 launch tl_driver tl_tcb605_driver.launch.py
```

终端 2 — 启动手柄遥操作（按轴数选择）：

```bash
# 6 轴机械臂通用
ros2 launch tl_teleop_f710 tl_teleop_f710_6axis.launch.py

# 7 轴机械臂通用
ros2 launch tl_teleop_f710 tl_teleop_f710_7axis.launch.py
```

真机模式下节点会自动完成以下初始化：
1. 等待 tl_driver 服务就绪
2. 设置运行模式为远程模式
3. 设置 ServoJ 运动速度
4. 开启关节跟踪模式（ServoJ）
5. 调用 coord_transform 服务获取 home 位姿
6. 输出 ✅ 提示，遥操作就绪
7. 有时候需要按一下手柄上的“START”键，开始手柄遥控机械臂

如果手柄在非默认路径，可指定设备：

```bash
ros2 launch tl_teleop_f710 tl_teleop_f710_6axis.launch.py joy_dev:=/dev/input/js1
```

### Gazebo 仿真模式（无需连接真机）

仿真模式下 IK 由桥接节点内部使用 **KDL** 库本地求解，无需 MoveIt2。

```bash
# 6 轴仿真
ros2 launch tl_teleop_f710 tl_teleop_f710_6axis_gazebo.launch.py arm_type:=tcb605
ros2 launch tl_teleop_f710 tl_teleop_f710_6axis_gazebo.launch.py arm_type:=tcb610
ros2 launch tl_teleop_f710 tl_teleop_f710_6axis_gazebo.launch.py arm_type:=tcb605f

# 7 轴仿真
ros2 launch tl_teleop_f710 tl_teleop_f710_7axis_gazebo.launch.py arm_type:=tcb710
ros2 launch tl_teleop_f710 tl_teleop_f710_7axis_gazebo.launch.py arm_type:=tcb705
```

**支持的 `arm_type` 值**（对应 `tl_description/urdf/` 下的 URDF 模型文件）：

| 轴数 | arm_type 可选值 |
|------|----------------|
| 6 轴 | `tcb605`、`tcb605f`、`tcb605l`、`tcb605lv`、`tcb605v`、`tcb610` |
| 7 轴 | `tcb610v`、`tcb705`、`tcb705f`、`tcb705l`、`tcb705lv`、`tcb705v`、`tcb710`、`tcb710v` |

`arm_type` 参数决定了 Gazebo 中加载的 URDF 模型、KDL 运动学模型以及 position controller 的关节数量（6 或 7）。

## 操作说明

| F710 输入 | 功能 | 说明 |
|-----------|------|------|
| **左摇杆** | X/Y 平移 | 末端在基座标系下沿 X/Y 方向移动 |
| **右摇杆上下** | Z 平移 | 末端沿 Z 方向（上下）移动 |
| **右摇杆左右** | 偏航 | 末端绕 Z 轴旋转 |
| **LB + 右摇杆左右** | 翻滚 | 末端绕 X 轴旋转 |
| **RB + 右摇杆左右** | 俯仰 | 末端绕 Y 轴旋转 |
| **十字键上** | 加速 | 增大运动速度 |
| **十字键下** | 减速 | 减小运动速度 |
| **A 键** | 回零 | 回到初始位姿（通过 FK 将 home_joints 转为笛卡尔位姿） |
| **B 键** | 停止 | 停止当前运动 |
| **START 键** | 开始 | 开始手柄控制 |

## 参数配置

真机和仿真使用独立的配置文件：

| 模式 | 6 轴配置文件 | 7 轴配置文件 |
|------|-------------|-------------|
| 真机 | `config/tl_teleop_f710_6axis.yaml` | `config/tl_teleop_f710_7axis.yaml` |
| 仿真 | `config/tl_teleop_f710_6axis_sim.yaml` | `config/tl_teleop_f710_7axis_sim.yaml` |

### 参数说明

| 参数 | 真机 6 轴 | 仿真 6 轴 | 真机 7 轴 | 仿真 7 轴 | 说明 |
|------|----------|----------|----------|----------|------|
| `control_rate` | 100.0 | 100.0 | 100.0 | 100.0 | 控制循环频率 (Hz) |
| `simulation_mode` | false | true | false | true | true 时跳过 ServoJ 初始化 |
| `arm_type` | tcb605 | tcb605 | tcb710 | tcb710 | 机械臂型号标识 |
| `speed_default` | 50.0 | 50.0 | 50.0 | 50.0 | 默认运动速度 (0-100) |
| `pos_sensitivity` | 80.0 | 200.0 | 80.0 | 200.0 | 位置灵敏度 (mm/s) |
| `rot_sensitivity` | 1.0 | 2.0 | 1.0 | 2.0 | 姿态灵敏度 (rad/s) |
| `step_size` | 5.0 | 1.0 | 5.0 | 1.0 | servol 插值步长 (mm) |
| `deadzone` | 0.15 | 0.15 | 0.15 | 0.15 | 摇杆死区 |
| `home_joints` (6) | [0,0,0,0,0,0] | [0,0,0,0,0,0] | — | — | 回零关节角度（度） |
| `home_joints` (7) | — | — | [0,0,0,0,0,0,0] | [0,0,0,0,0,0,0] | 回零关节角度（度） |
| `servo_speed` | 25.0 | — | 25.0 | — | ServoJ 速度（仅真机） |

> 仿真模式跳过 ServoJ 初始化，`servo_vmax`、`servo_amax`、`servo_jmax` 参数仅在真机配置中有效。

### 速度调优

末端运动速度估算公式：

```
速度 ≈ 摇杆值 × pos_sensitivity × (speed_value / 100)
```

- 调大 `pos_sensitivity` → 整体变快
- 调小 `pos_sensitivity` → 整体变慢
- 十字键上下调节 speed_value (0-100)，步长 5

## 坐标系

- 所有平移运动基于**基座标系（Base）**
- 姿态旋转使用欧拉角 (RPY)，单位 rad
- 位置单位 mm

## 真机架构

```
F710 手柄 → joy_node → /joy → tl_teleop_f710_node (C++)
                                   ↓
                         /tl_driver/set_servol_pos (ServolMove)
                                   ↓
                              tl_driver 节点
                    （_tl_host.so IK + servoj 执行）
                                   ↓
                              机械臂实际运动
```

## 仿真架构

```
F710 手柄 → joy_node → /joy → tl_teleop_f710_node (C++)
                                        ↓
                              /tl_driver/set_servol_pos 话题
                                        ↓
                         tl_teleop_f710_sim_bridge 节点 (C++)
                            （KDL ChainIkSolverPos_LMA 本地 IK）
                                        ↓
                     Gazebo（ros2_control position controller）
                                        ↓
                                   仿真机械臂运动
```

## 文件结构

```
tl_teleop_f710/
├── package.xml                         # ament_cmake 包定义
├── CMakeLists.txt                      # 编译规则
├── README.md                           # 本文件
├── udev/99-logitech-f710.rules         # 手柄权限规则
├── config/
│   ├── tl_teleop_f710_6axis.yaml       # 6 轴真机配置
│   ├── tl_teleop_f710_6axis_sim.yaml   # 6 轴仿真配置
│   ├── tl_teleop_f710_7axis.yaml       # 7 轴真机配置
│   └── tl_teleop_f710_7axis_sim.yaml   # 7 轴仿真配置
├── launch/
│   ├── tl_teleop_f710_6axis.launch.py      # 6 轴真机启动
│   ├── tl_teleop_f710_6axis_gazebo.launch.py  # 6 轴仿真启动
│   ├── tl_teleop_f710_7axis.launch.py      # 7 轴真机启动
│   └── tl_teleop_f710_7axis_gazebo.launch.py  # 7 轴仿真启动
├── include/tl_teleop_f710/
│   ├── tl_teleop_f710_node.h           # 遥操作节点类声明
│   └── tl_teleop_f710_sim_bridge.h     # 仿真桥接节点类声明
└── src/
    ├── tl_teleop_f710_node.cpp         # ★ 核心遥操作节点
    └── tl_teleop_f710_sim_bridge.cpp   # ★ KDL IK 仿真桥接
```

## 注意事项

1. 真机使用前请确保机械臂已上电并处于远程模式（节点会自动设置）
2. 回零前请确认周围无障碍物
3. 速度范围 0-100，建议从 50 开始适应后再调高
4. 本包不依赖 Python Pinocchio 库 — IK 由 C++ KDL 库求解
5. 长距离移动时建议使用较大 `step_size` 以提高响应
6. 真机与仿真参数独立配置在各自的 YAML 文件中，互不干扰
7. 仿真模式通过 `arm_type` 参数自动匹配对应 URDF 模型和关节数量（6/7 轴）
8. 仿真启动同时会启动 `tl_gazebo` 包的 `gazebo_6axis/7axis_f710_sim.launch.py`，自动拼接 URDF 和 ros2_control 配置
