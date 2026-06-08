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
- **握紧触发机制**：只有握紧 VR 手柄扳机（握力 > 0.9）时才触发遥操作，松开即停止，握下瞬间自动记录当前机械臂位姿为基准
- **位置死区滤波**：手柄微小位移（< 5mm）被过滤，避免手部抖动导致机械臂振荡
- **奇异点保护**：当 J6/J7 关节角度超过 160° 时自动减速至 20%，防止关节速度爆炸
- **关节跳变检测**：相邻两次指令的关节角度差超过 30° 时拒绝执行，防止异常指令或通信错误
- **A 键复位**：按下 VR 手柄 A 键将所有关节复位到零点
- **100Hz 控制循环**：独立控制线程以 100Hz 频率运行，通过 tl_driver 的 ServoJ 模式实现高速位置跟随

### 系统依赖关系

tl_teleop 运行时依赖 **tl_driver** 功能包提供以下服务：

| 依赖项 | 类型 | 说明 |
|--------|------|------|
| `/tl_driver/power_on` | 服务 | 上电 |
| `/tl_driver/power_off` | 服务 | 下电 |
| `/tl_driver/set_current_mode` | 服务 | 切换到运行模式 |
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
colcon build
source install/setup.bash
```

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
ros2 launch tl_teleop tl_teleop.launch.py
```

启动成功后，节点将依次执行：
1. 初始化 PXREA Robot SDK，连接 VR 遥操作设备
2. 等待 tl_driver 所有必要服务就绪
3. 调用 tl_driver 服务完成模式切换、速度设置、上电、开启 ServoJ
4. 进入遥操作控制循环

### 配置参数说明

配置文件位于 `config/tl_teleop_config.yaml`：

```
tl_teleop:
  ros__parameters:
    arm_ip: "192.168.1.13"          # 机械臂控制器 IP
    arm_port: "6001"                # 机械臂控制器端口
    arm_port_aux: "7000"            # 机械臂辅助端口
    arm_joints: 7                   # 机械臂关节数
    pos_scale: 0.5                  # 位置缩放系数（手柄位移 1m → 末端移动 0.5m）
    pos_deadzone: 0.005             # 位置死区（m），小于此值的位移被忽略
    max_pos_delta_mm: 300.0         # 单步最大位置增量（mm），防止异常跳变
    singular_angle: 160.0           # 奇异点判定角度（度）
    singular_scale: 0.2             # 奇异点区域速度缩放系数
    joint_jump_threshold: 30.0      # 关节跳变阈值（度），超过此值拒绝执行
    servo_speed: 25                 # ServoJ 运动速度
    publish_rate: 20.0              # 发布频率
```

> **注意**：`arm_ip`、`arm_port`、`arm_port_aux` 应与 tl_driver 中的配置保持一致，以确保 tl_teleop 通过 tl_driver 间接通信的控制器地址正确。

## tl_teleop功能包架构说明
### 功能包文件总览
```
├── CMakeLists.txt                     # 编译规则文件（支持 ARM/x86 架构自动检测）
├── config                             # 配置文件
│   └── tl_teleop_config.yaml          # 遥操作参数配置
├── include                            # 头文件
│   └── tl_teleop
│       └── tl_teleop.h                # TL_Teleop 类定义
├── launch                             # 启动文件
│   └── tl_teleop.launch.py            # 遥操作节点启动文件
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
| 服务客户端 | `power_on_client_` 等 9 个 | 调用 tl_driver 各项服务 |
| 话题发布 | `servoj_pos_pub_` | 发布 Float64MultiArray 到 `/tl_driver/set_servoj_pos` |
| 话题订阅 | `tcp_pose_sub_` | 订阅 `/tcp_pose` 获取当前末端位姿 |
| PXREA 回调 | `on_pxrea_client_cb()` | PXREA SDK 事件回调（设备连接、断开、状态更新） |
| VR 状态 | `VRState` | 线程安全的手柄姿态缓存结构体 |
| 运动解算 | `get_inverse_kinematics()` | 通过 tl_driver 服务进行笛卡尔 → 关节逆解 |
| 安全保护 | `clamp_joints()` / `joints_safe()` | 关节限位裁剪 / 跳变检测 |
| 四元数运算 | `quat_multiply()` / `quat_inverse()` / `quat2rpy()` | 姿态差值运算 |

**控制流程**：
```
VR手柄位姿 → 握紧触发 → 记录基准位姿 → 计算偏差(位置+姿态) → 坐标映射
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
| `/tcp_pose` | `tl_ros2_interface/msg/CartesianPose` | 获取机械臂当前末端位姿（position 单位 mm，rpy 单位 rad），用于基准点标定 |

### 调用的服务（客户端）
```
  Service Clients:
    /tl_driver/power_on: std_srvs/srv/Trigger
    /tl_driver/power_off: std_srvs/srv/Trigger
    /tl_driver/clear_error: std_srvs/srv/Trigger
    /tl_driver/set_speed: tl_ros2_interface/srv/SetSpeed
    /tl_driver/set_current_mode: tl_ros2_interface/srv/SetCurrentMode
    /tl_driver/open_servoj: tl_ros2_interface/srv/OpenServoJ
    /tl_driver/close_servoj: std_srvs/srv/Trigger
    /tl_driver/coord_transform: tl_ros2_interface/srv/CoordTransform
    /tl_driver/get_rpy2quat: tl_ros2_interface/srv/GetPosTransform
```

以上服务均为调用 tl_driver 提供的接口，tl_teleop 本身不提供服务端。

### 控制逻辑说明

1. **启动初始化** → 调用 `set_current_mode`（模式 2）、`set_speed`、`power_on`、`open_servoj`
2. **遥操作循环** → 读取 VR 位姿 → 握下扳机时以当前末端位姿为基准 → 手柄偏移映射为笛卡尔增 → 逆运动学 → 关节目标发布
3. **安全退出** → 关闭 ServoJ → 下电 → PXREA SDK 反初始化
