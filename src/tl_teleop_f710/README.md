<div align="center">

# 天链机器人 tl_teleop_f710 使用说明书 (C++)

</div>

## 目录
* 1.[tl_teleop_f710 功能包说明](#1-tl_teleop_f710-功能包说明)
* 2.[tl_teleop_f710 功能包使用](#2-tl_teleop_f710-功能包使用)
* 3.[tl_teleop_f710 功能包架构说明](#3-tl_teleop_f710-功能包架构说明)
* 4.[tl_teleop_f710 功能包话题与服务说明](#4-tl_teleop_f710-功能包话题与服务说明)

## 1 tl_teleop_f710 功能包说明

`tl_teleop_f710` 功能包实现了通过 **Logitech F710 游戏手柄**远程控制天链机械臂运动。基于笛卡尔空间伺服（`/tl_driver/set_servol_pos` 话题）实现直观的末端位置控制，操作者无需理解关节空间，推摇杆机械臂就往对应方向运动。支持真机控制和 Gazebo 仿真两种模式。

通过以下四部分内容的介绍可以帮助大家：
* 1.了解该功能包的使用。
* 2.熟悉功能包中的文件构成及作用。
* 3.熟悉功能包相关的话题，方便开发和使用。

### 1.1 功能特性

- **F710 游戏手柄遥操作**：通过 Logitech F710 无线手柄（DirectInput 模式）控制机械臂末端在笛卡尔空间 6 自由度运动
- **笛卡尔空间伺服**：直接下发末端目标位姿（x, y, z, rx, ry, rz），由 tl_driver 内部 IK 或仿真 KDL IK 求解关节角
- **真机/仿真双模式**：真机模式连接实体机械臂通过 ServoJ 关节跟踪驱动；仿真模式通过 KDL 本地 IK + Gazebo position controller 虚拟驱动
- **回零 FK**：按下 A 键回零时，通过 FK 将配置的 `home_joints` 关节角度（度）转为笛卡尔位姿，不同臂型零位自动适配
- **6/7 轴自适应**：FK 和 IK 模型根据加载的 URDF 自动确定关节数量，无需手动指定
- **多种臂型兼容**：通过 `arm_type` 参数可切换任意天链机械臂型号，无需修改代码或创建新文件
- **速度实时调节**：十字键上下实时调整运动速度（0-100），支持 LB/RB 切换姿态控制模式（偏航/翻滚/俯仰）
- **摇杆死区滤波**：默认 0.15 死区阈值，手柄回中微小抖动被过滤
- **仿真 KDL IK**：仿真模式下使用 KDL `ChainIkSolverPos_LMA` 库本地求解逆运动学（Levenberg-Marquardt 迭代法），无需依赖 MoveIt2 或 Python Pinocchio

### 1.2 系统依赖关系

tl_teleop_f710 运行时依赖 **tl_driver** 功能包提供以下服务与话题：

| 依赖项 | 类型 | 说明 |
|--------|------|------|
| `/tl_driver/set_current_mode` | 服务 | 切换到远程模式（模式 2） |
| `/tl_driver/set_speed` | 服务 | 设置运动速度 |
| `/tl_driver/open_servoj` | 服务 | 开启 ServoJ 模式 |
| `/tl_driver/close_servoj` | 服务 | 关闭 ServoJ 模式 |
| `/tl_driver/coord_transform` | 服务 | 正运动学求解（关节→笛卡尔），用于真机回零 |
| `/tl_driver/set_servol_pos` | 话题 | 【发布】发送目标笛卡尔位姿（ServolMove） |

> 真机模式下依赖上述 tl_driver 接口；仿真模式下依赖 Gazebo + KDL，不依赖 tl_driver。

### 1.3 适用型号

| 轴数 | 支持型号 | 通用配置文件 | 通用启动文件 |
|------|---------|-------------|-------------|
| **6 轴** | TCB605、TCB605F、TCB605L、TCB605LV、TCB605V、TCB610 | `tl_teleop_f710_6axis.yaml` / `_sim.yaml` | `tl_teleop_f710_6axis.launch.py` / `_gazebo.launch.py` |
| **7 轴** | TCB610V、TCB705、TCB705F、TCB705L、TCB705LV、TCB705V、TCB710、TCB710V | `tl_teleop_f710_7axis.yaml` / `_sim.yaml` | `tl_teleop_f710_7axis.launch.py` / `_gazebo.launch.py` |

## 2 tl_teleop_f710 功能包使用

### 2.1 安装依赖

```bash
cd ~/tl_robot_ros2_cpp
rosdep install --from-paths src --ignore-src -r -y
```

本包使用 C++ KDL 库求解运动学，**无需安装 Python Pinocchio**。

### 2.2 安装 udev 规则

```bash
sudo cp src/tl_teleop_f710/udev/99-logitech-f710.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

F710 手柄拨到 **"D"（DirectInput 模式）**。

### 2.3 编译

```bash
cd ~/tl_robot_ros2_cpp
colcon build --packages-select tl_teleop_f710
source install/setup.bash
```

### 2.4 启动方式

**真机模式：**

```bash
# 终端 1：启动驱动
ros2 launch tl_driver tl_tcb605_driver.launch.py
# 终端 2：启动遥操作
ros2 launch tl_teleop_f710 tl_teleop_f710_6axis.launch.py
```

节点自动完成：服务等待 → 设远程模式 → 设速度 → 开 ServoJ → FK 初始位姿 → ✅ 就绪。

**Gazebo 仿真：**

```bash
ros2 launch tl_teleop_f710 tl_teleop_f710_6axis_gazebo.launch.py arm_type:=tcb605
```

`arm_type` 可选值：6 轴 `tcb605`、`tcb605f`、`tcb605l`、`tcb605lv`、`tcb605v`、`tcb610`；7 轴 `tcb610v`、`tcb705`、`tcb705f`、`tcb705l`、`tcb705lv`、`tcb705v`、`tcb710`、`tcb710v`。

### 2.5 操作说明

| F710 输入 | 功能 |
|-----------|------|
| **左摇杆** | X/Y 平移 |
| **右摇杆上下** | Z 平移 |
| **右摇杆左右** | 偏航 |
| **LB + 右摇杆左右** | 翻滚 |
| **RB + 右摇杆左右** | 俯仰 |
| **十字键上/下** | 加速/减速 |
| **A 键** | 回零 |
| **B 键** | 停止 |

### 2.6 参数配置

| 参数 | 6轴真机 | 6轴仿真 | 说明 |
|------|---------|---------|------|
| `control_rate` | 100.0 | 100.0 | 控制频率 (Hz) |
| `pos_sensitivity` | 80.0 | 200.0 | 位置灵敏度 (mm/s) |
| `rot_sensitivity` | 1.0 | 2.0 | 姿态灵敏度 (rad/s) |
| `step_size` | 5.0 | 1.0 | servol 步长 (mm) |
| `deadzone` | 0.15 | 0.15 | 摇杆死区 |
| `home_joints` | 6/7 零值 | 同 | 回零关节角（度）|
| `servo_speed` | 25.0 | — | ServoJ 速度 |

速度公式：`速度 ≈ 摇杆值 × pos_sensitivity × (speed_value / 100)`

## 3 功能包架构

```
tl_teleop_f710/
├── package.xml / CMakeLists.txt
├── config/                         # 4 套 YAML 配置
├── launch/                         # 4 个启动文件
├── include/tl_teleop_f710/         # 头文件
│   ├── tl_teleop_f710_node.h
│   └── tl_teleop_f710_sim_bridge.h
├── src/
│   ├── tl_teleop_f710_node.cpp     # 遥操作主节点
│   └── tl_teleop_f710_sim_bridge.cpp # KDL IK 桥接
└── udev/
```

**真机链路：** F710 → joy_node → /joy → tl_teleop_f710_node → /tl_driver/set_servol_pos → tl_driver → 机械臂

**仿真链路：** F710 → joy_node → /joy → tl_teleop_f710_node → set_servol_pos → sim_bridge (KDL IK) → Gazebo position controller → 仿真机械臂

## 4 话题与服务

| 话题/服务 | 类型 | 说明 |
|-----------|------|------|
| `/joy` | `sensor_msgs/Joy` | 手柄输入订阅 |
| `/tl_driver/set_servol_pos` | `ServolMove` | 发布笛卡尔目标位姿 |
| `/tl_driver/set_current_mode` | `SetCurrentMode` | 设远程模式 |
| `/tl_driver/open_servoj` | `OpenServoJ` | 开 ServoJ |
| `/tl_driver/coord_transform` | `CoordTransform` | 真机 FK |
| `/joint_states` | `JointState` | 仿真关节状态 |
| `${position_controller_topic}` | `Float64MultiArray` | 仿真关节指令 |

坐标系：基座标系、欧拉角 RPY (rad)、位置 mm。

## 5 注意事项

1. 真机需机械臂已上电并处于远程模式
2. 回零前确认无障碍物
3. 速度 0-100，建议从 50 开始适应
4. IK 由 C++ KDL 求解，无需 Python Pinocchio
5. 仿真自动启动 `tl_gazebo` 的 `gazebo_6axis/7axis_f710_sim.launch.py`
