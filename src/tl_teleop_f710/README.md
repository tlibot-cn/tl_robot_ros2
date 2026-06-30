<div align="center">

# 天链机器人 tl_teleop_f710 使用说明书 (C++)

</div>

## 目录
* 1.[tl_teleop_f710 功能包说明](#1-tl_teleop_f710-功能包说明)
* 2.[tl_teleop_f710 功能包使用](#2-tl_teleop_f710-功能包使用)
* 3.[tl_teleop_f710 功能包架构说明](#3-tl_teleop_f710-功能包架构说明)
* 4.[tl_teleop_f710 功能包话题与服务说明](#4-tl_teleop_f710-功能包话题与服务说明)

## 1 tl_teleop_f710 功能包说明

`tl_teleop_f710` 功能包实现了通过 **Logitech F710 游戏手柄**远程控制天链机械臂运动。采用**直接 ServoJ 方案**：节点内部自行做笛卡尔→关节 IK 求解，以 **250Hz（4ms）** 周期稳定输出关节角到 `/tl_driver/set_servoj_pos` 话题，指令流从不中断，运动平滑流畅。

支持真机控制和 Gazebo 仿真两种模式。

### 1.1 功能特性

- **F710 游戏手柄遥操作**：通过 Logitech F710 无线手柄（DirectInput 模式）控制机械臂末端在笛卡尔空间 6 自由度运动
- **直接 ServoJ 控制（250Hz）**：节点自行 IK + 每 4ms 稳定输出一帧关节角，指令流从不中断，松手即停
- **异步 IK 求解**：有摇杆输入时在独立线程中调用 `coord_transform` 服务做 IK，不阻塞 250Hz 控制循环
- **真机/仿真双模式**：真机模式连接实体机械臂通过 ServoJ 关节跟踪驱动；仿真模式通过 KDL 本地 IK + Gazebo position controller 虚拟驱动
- **完整上电流程**：自动执行 `connect_arm → power_on → set_mode → set_speed → open_servoj`，含伺服状态检测和报警清错
- **回零 FK**：按下 A 键回零时，通过 FK 将 `home_joints` 关节角度（度）转为笛卡尔位姿，不同臂型零位自动适配
- **6/7 轴自适应**：根据 `home_joints` 长度自动确定轴数
- **多种臂型兼容**：通过 `arm_type` 参数切换型号，无需修改代码
- **速度实时调节**：十字键上下实时调整运动速度（0-100），支持 LB/RB 切换姿态控制模式（偏航/翻滚/俯仰）
- **紧急停止**：Back+Start 组合键紧急停止，防误触
- **工作空间限位**：x/y/z 软限位保护，超出自动切断速度
- **仿真 KDL IK**：仿真模式下使用 KDL `ChainIkSolverPos_LMA` 库求解逆运动学，无需 MoveIt2 或 Python Pinocchio

### 1.2 系统依赖关系

tl_teleop_f710 运行时依赖 **tl_driver** 功能包提供以下服务与话题：

| 依赖项 | 类型 | 说明 |
|--------|------|------|
| `/tl_driver/connect_arm` | 服务 | 连接机械臂 |
| `/tl_driver/power_on` | 服务 | 伺服上电（含状态检测、报警清错） |
| `/tl_driver/set_current_mode` | 服务 | 切换到远程模式（模式 2） |
| `/tl_driver/set_speed` | 服务 | 设置运动速度 |
| `/tl_driver/open_servoj` | 服务 | 开启 ServoJ 模式 |
| `/tl_driver/close_servoj` | 服务 | 关闭 ServoJ 模式 |
| `/tl_driver/coord_transform` | 服务 | 正/逆运动学求解，用于 FK 和 IK |
| `/tl_driver/set_servoj_pos` | 话题 | 【发布】发送关节角指令（Float64MultiArray）|

> 真机模式下依赖上述全部 tl_driver 接口；仿真模式下不依赖 tl_driver，使用 KDL 本地求解。

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

本包使用 C++ KDL 库，**无需安装 Python Pinocchio**。

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

节点自动完成初始化流程：
```
连接 → 上电(状态检测/清错) → 设远程模式 → 设速度 → 开 ServoJ → FK 初始位姿 → ✅ 就绪
```

**Gazebo 仿真：**

```bash
ros2 launch tl_teleop_f710 tl_teleop_f710_6axis_gazebo.launch.py arm_type:=tcb605
```

`arm_type` 可选值：6 轴 `tcb605`、`tcb605f`、`tcb605l`、`tcb605lv`、`tcb605v`、`tcb610`；
7 轴 `tcb610v`、`tcb705`、`tcb705f`、`tcb705l`、`tcb705lv`、`tcb705v`、`tcb710`、`tcb710v`。

### 2.5 操作说明

| F710 输入 | 功能 | 说明 |
|-----------|------|------|
| **左摇杆** | X/Y 平移 | 末端沿基座标系 X/Y 方向移动 |
| **右摇杆上下** | Z 平移 | 末端沿 Z 方向（上下）移动 |
| **右摇杆左右** | 偏航 | 末端绕 Z 轴旋转（默认模式） |
| **LB + 右摇杆左右** | 翻滚 | 末端绕 X 轴旋转 |
| **RB + 右摇杆左右** | 俯仰 | 末端绕 Y 轴旋转 |
| **十字键上** | 加速 | 增大运动速度（+5） |
| **十字键下** | 减速 | 减小运动速度（-5） |
| **A 键** | 回零 | 回到初始位姿（FK + IK 计算 home 关节角） |
| **Back + Start** | ⚠️ 紧急停止 | 组合键防误触，立即停止运动 |

### 2.6 参数配置

以 6 轴真机配置为例（YAML 文件包含详细注释）：

```yaml
/**:
  ros__parameters:
    control_rate: 250.0          # 控制频率 (Hz)，推荐 250Hz
    arm_type: tcb605
    speed_default: 50.0          # 默认速度 (0-100)
    speed_min: 5.0
    speed_max: 100.0
    speed_step: 5.0
    pos_sensitivity: 160.0       # 平移灵敏度 (mm/s)
    rot_sensitivity: 1.0         # 旋转灵敏度 (rad/s)
    servo_speed: 25.0
    servo_vmax: 180.0            # 关节最大速度 (°/s)
    servo_amax: 3000.0           # 关节最大加速度 (°/s²)
    servo_jmax: 50000.0          # 关节最大加加速度 (°/s³)
    deadzone: 0.15               # 摇杆死区
    home_joints: [0,0,0,0,0,0]  # 回零关节角（度）
    workspace_limits: [-500,500,-500,500,0,800]  # 软限位 [xmin,xmax,ymin,ymax,zmin,zmax]
```

关键参数说明：

| 参数 | 6轴真机 | 6轴仿真 | 说明 |
|------|---------|---------|------|
| `control_rate` | 250.0 | 250.0 | 控制循环频率 (Hz)，4ms 周期 |
| `pos_sensitivity` | 160.0 | 200.0 | 平移灵敏度 (mm/s) |
| `rot_sensitivity` | 1.0 | 2.0 | 旋转灵敏度 (rad/s) |
| `servo_vmax` | 180.0 | — | 关节最大速度 (°/s)，上限 180 |
| `deadzone` | 0.15 | 0.15 | 摇杆死区 |
| `home_joints` | 6/7 零值 | 同 | 回零关节角（度）|
| `workspace_limits` | 6 个值 | 同 | 笛卡尔软限位 (mm) |

速度公式：`末端速度 ≈ 摇杆值 × sensitivity × (speed_value / 100)`

## 3 功能包架构

```
tl_teleop_f710/
├── package.xml / CMakeLists.txt
├── config/                         # 4 套 YAML 配置（含详细注释）
├── launch/                         # 4 个启动文件
├── include/tl_teleop_f710/
│   ├── tl_teleop_f710_node.h       # 遥操作节点类
│   └── tl_teleop_f710_sim_bridge.h # 仿真 IK 桥接节点类
├── src/
│   ├── tl_teleop_f710_node.cpp     # ★ 主节点：250Hz ServoJ 控制
│   └── tl_teleop_f710_sim_bridge.cpp # 仿真 KDL IK 桥接
└── udev/
```

**真机链路：**
```
F710 → joy_node → /joy
  → tl_teleop_f710_node (自做 IK, 250Hz)
  → /tl_driver/set_servoj_pos (Float64MultiArray, 关节角)
  → tl_driver → 机械臂
```

**仿真链路：**
```
F710 → joy_node → /joy
  → tl_teleop_f710_node (自做 IK)
  → /tl_driver/set_servoj_pos (关节角)
  → tl_teleop_f710_sim_bridge (KDL IK → position controller)
  → Gazebo → 仿真机械臂
```

### 节点内部架构

| 组件 | 频率 | 说明 |
|------|------|------|
| `/joy` 话题回调 | 事件驱动 | 缓存摇杆数据 |
| 主控制循环 | **250Hz (4ms)** | 读摇杆 → 笛卡尔速度积分 → 发上一帧关节角 → 异步 IK |
| IK 求解 | 异步 `std::async` | 有摇杆输入时调用 `coord_transform` 服务做 IK |
| 初始化定时器 | 1Hz | 状态机：connect → power_on → set_mode → set_speed → open_servoj |

### 控制循环流程（每 4ms）

```
1. 读摇杆
2. 检查 Back+Start 紧急停止
3. 十字键速度调节
4. A 键回零（异步 FK+IK）
5. 笛卡尔速度积分 → 更新 target_pose_
6. 工作空间限位保护
7. 发送上一帧关节角到 /tl_driver/set_servoj_pos  ← 每帧必发
8. 有摇杆输入 ? 异步 IK (std::async) : 跳过
```

## 4 话题与服务

### 4.1 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/joy` | `sensor_msgs/Joy` | 订阅 | 手柄输入 |
| `/tl_driver/set_servoj_pos` | `std_msgs/Float64MultiArray` | **发布** | 关节角指令（250Hz） |
| `/joint_states` | `sensor_msgs/JointState` | 订阅 | 仿真关节状态（sim_bridge）|

### 4.2 服务客户端

| 服务 | 类型 | 用途 |
|------|------|------|
| `/tl_driver/connect_arm` | `std_srvs/Trigger` | 连接机械臂 |
| `/tl_driver/power_on` | `std_srvs/Trigger` | 上电（含清错） |
| `/tl_driver/set_current_mode` | `SetCurrentMode` | 设远程模式 |
| `/tl_driver/set_speed` | `SetSpeed` | 设速度 |
| `/tl_driver/open_servoj` | `OpenServoJ` | 开 ServoJ 跟踪 |
| `/tl_driver/close_servoj` | `std_srvs/Trigger` | 关 ServoJ |
| `/tl_driver/coord_transform` | `CoordTransform` | FK 回零 + IK 关节角求解 |

### 4.3 坐标系

- 所有平移运动基于**基座标系（Base）**
- 姿态旋转使用欧拉角 RPY，单位 rad
- 位置单位 mm

## 5 注意事项

1. 真机需机械臂已上电（节点自动执行 `power_on`）
2. 回零前确认无障碍物
3. 速度 0-100，建议从 50 开始适应
4. 紧急停止请同时按 **Back + Start**（B 键已无功能）
5. IK 由 C++ KDL 求解，无需 Python Pinocchio
6. 仿真自动启动 `tl_gazebo` 的 `gazebo_6axis/7axis_f710_sim.launch.py`
7. 修改配置后需重新编译：`colcon build --packages-select tl_teleop_f710`
