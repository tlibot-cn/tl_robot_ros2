<div align="center">

# 天链机器人tl_teleop使用说明书

</div>

## 目录
* 1.[tl_teleop功能包说明](#tl_teleop功能包说明)
* 2.[tl_teleop功能包使用](#tl_teleop功能包使用)
* 3.[tl_teleop功能包架构说明](#tl_teleop功能包架构说明)
* 4.[tl_teleop功能包话题与服务说明](#tl_teleop功能包话题与服务说明)

## tl_teleop功能包说明
tl_teleop功能包实现了机械臂的遥操作（Teleoperation）功能，通过 PXREA Robot SDK 与 VR 遥操作手柄设备通信，将手柄的位姿映射为机械臂末端的运动指令，实现对机械臂的实时跟随控制。

* 1.功能包使用。
* 2.功能包架构说明。
* 3.功能包话题与服务说明。

通过这三部分内容的介绍可以帮助大家：
* 1.了解该功能包的使用。
* 2.熟悉功能包中的文件构成及作用。
* 3.熟悉功能包相关的话题，方便开发和使用

### 功能特性
- **VR 手柄遥操作**：通过 PXREA Robot SDK 读取 VR 手柄位姿（位置 + 四元数姿态），实时驱动机械臂末端运动
- **6/7 轴适配**：通过 `arm_axis_mode` 参数切换 6 轴模式（TCB605 系列）或 7 轴模式（TCB710 系列），自动匹配关节限位和奇异点检测
- **握紧触发机制**：只有握紧 VR 手柄扳机（握力 > 0.9）时才触发遥操作，松开即停止，握下瞬间自动记录当前机械臂位姿为基准
- **位置死区滤波**：手柄各轴微小位移（< 5mm）被单独过滤，避免手部抖动导致机械臂振荡
- **单步位置增量限幅**：每周期位置增量不超过 300mm，防止异常跳变
- **奇异点保护**：6 轴模式下 J5/J6 关节角度超过 160° 时自动减速至 20%，7 轴模式下检测 J6/J7，防止关节速度爆炸
- **关节跳变检测**：相邻两次指令的关节角度差超过 30° 时拒绝执行，防止异常指令或通信错误
- **A 键复位**：按下 VR 手柄 A 键将所有关节复位到零点
- **100Hz 控制循环**：独立控制线程以 100Hz 频率运行，通过 tl_driver 的 ServoJ 模式实现高速位置跟随
- **最短旋转路径**：四元数自动取最短路径，避免姿态反向绕远路

### 系统依赖关系

tl_teleop 运行时依赖 **tl_driver** 功能包提供以下服务：

| 依赖项 | 类型 | 说明 |
|--------|------|------|
| `/tl_driver/set_current_mode` | 服务 | 切换到运行模式（模式 2） |
| `/tl_driver/set_speed` | 服务 | 设置运动速度 |
| `/tl_driver/open_servoj` | 服务 | 开启 ServoJ 模式 |
| `/tl_driver/close_servoj` | 服务 | 关闭 ServoJ 模式 |
| `/tl_driver/coord_transform` | 服务 | 逆运动学求解（笛卡尔 → 关节） |
| `/tl_driver/get_rpy2quat` | 服务 | RPY → 四元数转换 |
| `/tcp_pose` | 话题 | 获取当前末端位姿 |
| `/tl_driver/set_servoj_pos` | 话题 | 发送目标关节位置 |

因此使用 tl_teleop 前应先启动 tl_driver。

### 外部环境依赖

tl_teleop 的 PXREA Robot SDK 依赖 **XRobotToolKit-PC_Service** 作为 PC 端后台服务，用于与 VR 遥操作设备建立连接并传输手柄数据。使用 tl_teleop 前必须安装并启动该服务。

**下载地址**：[XRoboToolkit-PC-Service v1.0.0](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/tag/v1.0.0)

根据平台架构选择对应的安装包：

| 架构 | 安装包 |
|------|--------|
| ARM64 | `XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb` |
| x86_64 | `XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb` |

安装并启动 XRobotToolKit-PC_Service 后，再启动 tl_teleop 节点，PXREA SDK 才能正常连接 VR 设备并接收手柄位姿数据。

## tl_teleop功能包使用
### tl_teleop功能包编译

```bash
cd ~/tl_robot_ros2_cpp
colcon build --packages-select tl_teleop
source install/setup.bash
```

> 若首次编译或接口包有更新，请使用 `colcon build`（不带 `--packages-select`）以确保 tl_ros2_interface 先被编译。

### 启动方式

tl_teleop 依赖 XRobotToolKit-PC_Service 和 tl_driver，启动前需**按顺序**依次启动以下服务与节点：

**第一步：启动 XRobotToolKit-PC_Service**

```bash
source /opt/apps/roboticsservice/runService.sh
```

该命令启动 PC 端后台服务，PXREA SDK 通过该服务与 VR 遥操作设备通信。

> 注意：不同平台的 XRobotToolKit-PC_Service 启动命令可能不同，具体查阅 XRobotToolKit-PC_Service 的相关说明。

**第二步：启动 tl_driver（以 TCB710 为例）**

```bash
ros2 launch tl_driver tl_tcb710_driver.launch.py
```

> 其他臂型的启动命令参见 [tl_driver 说明文档](../tl_driver/README.md)。

**第三步：启动 tl_teleop**

```bash
# 6 轴
ros2 launch tl_teleop tl_teleop_6axis.launch.py
# 7 轴
ros2 launch tl_teleop tl_teleop_7axis.launch.py
```

启动成功后，节点将依次执行：
1. 初始化 PXREA Robot SDK，连接 VR 遥操作设备
2. 等待 tl_driver 所有必要服务就绪（最长 30 秒）
3. 调用 tl_driver 服务完成模式切换（模式 2）、速度设置、开启 ServoJ（tl_driver 自动处理上电逻辑）
4. 进入遥操作控制循环（100Hz）

### 配置参数说明

配置文件位于 `config/` 目录，按轴数分为两份：

- `config/tl_teleop_6axis_config.yaml` — 6 轴机械臂（TCB605 系列）
- `config/tl_teleop_7axis_config.yaml` — 7 轴机械臂（TCB710 系列）

节点名称为 `tl_teleop_node`，参数在该命名空间下配置。示例（7 轴配置）：

```yaml
tl_teleop_node:
  ros__parameters:
    # ========== 轴数配置 ==========
    arm_axis_mode: 7                 # 机械臂关节数（6 / 7）

    # ========== 位置控制参数 ==========
    pos_scale: 0.5                   # 位置缩放系数（VR 位移 1m → 末端移动 0.5m）
    pos_deadzone: 0.005              # 位置死区（m），各轴独立判定
    max_pos_delta_mm: 300.0          # 单步最大位置增量（mm），防止异常跳变

    # ========== 奇异点防护参数 ==========
    joint_jump_threshold: 30.0       # 关节跳变阈值（度），超过此值拒绝执行
    singular_angle: 160.0            # 奇异点判定角度（度）
    singular_scale: 0.2              # 奇异点区域速度缩放系数
    # 关节硬限位—扁平数组，每两个值一组 [min1, max1, min2, max2, ...]（单位：度）
    joint_limits: [-180.0, 180.0, -180.0, 180.0, -180.0, 180.0,
                   -180.0, 180.0, -180.0, 180.0, -170.0, 170.0, -170.0, 170.0]

    # ========== ServoJ 初始化参数（标量值，自动广播到所有关节）==========
    servo_speed: 25.0                # ServoJ 运动速度
    servo_vmax: 80.0                 # ServoJ 最大速度（度/秒）
    servo_amax: 3000.0               # ServoJ 最大加速度（度/秒²）
    servo_jmax: 50000.0              # ServoJ 最大加加速度（度/秒³）
```

**参数说明**：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `arm_axis_mode` | int | — | 机械臂关节数，必须为 6 或 7 |
| `pos_scale` | double | 0.5 | 位置缩放系数。手柄位移 1m → 末端移动 `pos_scale` m |
| `pos_deadzone` | double | 0.005 | 位置死区（m），各轴独立判定，小于此值的位移被忽略 |
| `max_pos_delta_mm` | double | 300.0 | 单步最大位置增量（mm），防止异常跳变 |
| `singular_angle` | double | 160.0 | 奇异点判定角度（度）。6 轴检测 J5/J6，7 轴检测 J6/J7 |
| `singular_scale` | double | 0.2 | 奇异点区域速度缩放系数 |
| `joint_jump_threshold` | double | 30.0 | 关节跳变阈值（度），相邻帧超过此值拒绝执行 |
| `joint_limits` | double[] | — | 关节硬限位，扁平数组 `[min1, max1, min2, max2, ...]`（度） |
| `servo_speed` | double | 25.0 | ServoJ 运动速度 |
| `servo_vmax` | double | 80.0 | ServoJ 各关节最大速度（度/秒），标量值广播到所有关节 |
| `servo_amax` | double | 3000.0 | ServoJ 各关节最大加速度（度/秒²） |
| `servo_jmax` | double | 50000.0 | ServoJ 各关节最大加加速度（度/秒³） |

> 注意：`grip_threshold`（握紧阈值 0.9）和 `control_period_s`（控制周期 0.01s）为代码硬编码常量，不在 YAML 中配置。

## tl_teleop功能包架构说明
### 功能包文件总览
```
├── CMakeLists.txt                     # 编译规则文件（支持 ARM/x86 架构自动检测）
├── config                             # 配置文件
│   ├── tl_teleop_6axis_config.yaml    # 6 轴遥操作参数
│   └── tl_teleop_7axis_config.yaml    # 7 轴遥操作参数
├── include                            # 头文件
│   └── tl_teleop
│       └── tl_teleop.h                # TL_Teleop 类定义
├── launch                             # 启动文件
│   ├── tl_teleop_6axis.launch.py      # 6 轴启动
│   └── tl_teleop_7axis.launch.py      # 7 轴启动
├── lib                                # PXREA Robot SDK 专有库
│   ├── arm
│   │   └── libPXREARobotSDK.so        # ARM64 架构预编译库
│   ├── include
│   │   └── PXREARobotSDK.h            # PXREA SDK C API 头文件
│   └── x86
│       └── libPXREARobotSDK.so        # x86_64 架构预编译库
├── package.xml                        # 依赖说明文件
├── README.md                          # 说明文档
└── src                                # 遥操作源文件
    └── tl_teleop.cpp                  # 遥操作节点实现
```

### 代码架构说明

tl_teleop 节点基于 `rclcpp::Node` 实现，采用 **双线程架构**：

- **主线程**：ROS2 事件循环（`SingleThreadedExecutor::spin()`），负责处理服务调用异步响应、话题订阅回调
- **控制线程**：100Hz 固定频率控制循环（`control_loop()`），负责读取 VR 手柄位姿、计算笛卡尔偏差、调用逆运动学、发布关节目标

**核心类 `TL_Teleop` 主要成员：**

| 类别 | 成员 | 说明 |
|------|------|------|
| 服务客户端 | `set_speed_client_` 等 6 个 | 调用 tl_driver 各项服务（不含 power_on/power_off） |
| 话题发布 | `servoj_pos_pub_` | 发布 Float64MultiArray 到 `/tl_driver/set_servoj_pos` |
| 话题订阅 | `tcp_pose_sub_` | 订阅 `/tcp_pose` 获取当前末端位姿 |
| PXREA 回调 | `on_pxrea_client_cb()` | PXREA SDK 事件回调（设备连接、断开、状态更新） |
| VR 状态 | `VRState` | 线程安全的手柄姿态缓存结构体（含互斥锁） |
| 运动解算 | `get_inverse_kinematics()` | 通过 tl_driver 服务进行笛卡尔 → 关节逆解 |
| 安全保护 | `clamp_joints()` / `joints_safe()` | 关节限位裁剪 / 跳变检测 |
| 四元数运算 | `quat_multiply()` / `quat_inverse()` / `quat2rpy()` | 姿态差值运算 |
| 位姿缓存 | `latest_tcp_pose_` | 互斥锁保护的 `/tcp_pose` 最新值缓存 |

**控制流程**：
```
VR手柄位姿 → 握紧触发 → 记录基准位姿 → 计算偏差(位置+姿态) → 坐标映射(VR→机械臂)
→ 奇异点减速 → 逆运动学求解 → 关节跳变检测 → 关节限位裁剪
→ 发布到 /tl_driver/set_servoj_pos → tl_driver 执行
```

## tl_teleop功能包话题与服务说明

tl_teleop 作为客户端节点，主要使用（消费）tl_driver 提供的话题与服务，并将其转化为机械臂运动指令。

### 发布的话题
```
  Publishers:
    /tl_driver/set_servoj_pos: std_msgs/msg/Float64MultiArray
```
| 话题 | 类型 | 说明 |
|------|------|------|
| `/tl_driver/set_servoj_pos` | `std_msgs/Float64MultiArray` | 目标关节角度数组（单位：度），100Hz 频率发布 |

### 订阅的话题
```
  Subscribers:
    /tcp_pose: tl_ros2_interface/msg/CartesianPose
```
| 话题 | 类型 | 说明 |
|------|------|------|
| `/tcp_pose` | `tl_ros2_interface/msg/CartesianPose` | 获取机械臂当前末端位姿（position 单位 m，rpy 单位 rad），用于基准点标定 |

### 调用的服务（客户端）
```
  Service Clients:
    /tl_driver/set_speed: tl_ros2_interface/srv/SetSpeed
    /tl_driver/set_current_mode: tl_ros2_interface/srv/SetCurrentMode
    /tl_driver/open_servoj: tl_ros2_interface/srv/OpenServoJ
    /tl_driver/close_servoj: std_srvs/srv/Trigger
    /tl_driver/coord_transform: tl_ros2_interface/srv/CoordTransform
    /tl_driver/get_rpy2quat: tl_ros2_interface/srv/GetPosTransform
```

以上 **6 个服务**均为调用 tl_driver 提供的接口，tl_teleop 本身不提供服务端。

### 控制逻辑说明

1. **启动初始化** → 调用 `set_current_mode`（模式 2）、`set_speed`、`open_servoj`。各步骤均有 5 秒超时保护，失败则终止（tl_driver 自动处理上电）
2. **遥操作循环**（100Hz）：
   - 读取 VR 位姿 → A 键按下时复位到零点
   - 握下扳机时以当前末端位姿为基准（记录位置 + RPY → 四元数）
   - 手柄相对于基准的偏移映射为笛卡尔增量
   - VR 坐标系 → 机械臂坐标系映射（VR: X右 Y上 Z前 → 机械臂: X前 Y左 Z上）
   - 姿态使用四元数差值计算最短旋转路径
   - 奇异点检测 → 逆运动学求解 → 关节跳变/限位检查 → 发布关节目标
3. **安全退出** → 停止控制循环 → 异步关闭 ServoJ → PXREA SDK 反初始化
