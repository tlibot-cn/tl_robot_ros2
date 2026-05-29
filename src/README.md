<div align="center">

# 天链机械臂 ROS2 功能包使用说明书

四川天链机器人股份有限公司

文件修订记录：

| 版本号 | 时间 | 备注 |
| :---: | :---- | :--- |
| V1.0 | 2026-5-22 | 拟制 |

</div>

## 目录

* 1 [功能包概览](#1-功能包概览)
* 2 [功能包说明](#2-功能包说明)
  * 2.1 [tl_bringup — 启动聚合](#21-tl_bringup--启动聚合)
  * 2.2 [tl_description — 模型描述](#22-tl_description--模型描述)
  * 2.3 [tl_driver — 硬件驱动](#23-tl_driver--硬件驱动)
  * 2.4 [tl_gazebo — Gazebo 仿真](#24-tl_gazebo--gazebo-仿真)
  * 2.5 [tl_moveit2_config — MoveIt2 配置](#25-tl_moveit2_config--moveit2-配置)
  * 2.6 [tl_ros2_interface — 消息与接口定义](#26-tl_ros2_interface--消息与接口定义)

## 1 功能包概览

`src/` 目录包含 **6 个功能包**，每个功能包（及子包）的作用如下：

```
src/
├── tl_bringup/              # 启动文件
│   ├── launch/              # 一键启动 launch 文件
│   └── doc/
├── tl_description/          # 模型描述
│   ├── config/              # 关节名称配置
│   ├── launch/              # 模型与 TF 发布 launch
│   ├── meshes/              # 14 套 STL 网格文件
│   ├── rviz/                # RViz 配置文件
│   └── urdf/                # 14 套 URDF 模型
├── tl_driver/               # 硬件驱动
│   ├── config/              # 14 套通信参数配置
│   ├── launch/              # 驱动节点 launch
│   ├── lib/                 # NexMotion SWIG 封装
│   └── src/                 # 驱动节点源码
├── tl_gazebo/               # Gazebo 仿真
│   ├── config/
│   └── launch/              # 14 套 Gazebo 仿真 launch
├── tl_moveit2_config/       # MoveIt2 配置（内含 14 个子包）
│   ├── tl_tcb605_config/    # 14 套 MoveIt2 配置（每型号一套）
│   │   ├── config/          # SRDF、限位、运动学等
│   │   └── launch/          # demo、move_group、rviz
│   ├── tl_tcb605f_config/
│   ├── ...
│   └── tl_tcb710v_config/
└── tl_ros2_interface/       # ROS2 消息与服务接口
    ├── msg/                 # 11 个 msg 定义文件
    └── srv/                 # 45 个 srv 定义文件
```

## 2 功能包说明

### 2.1 tl_bringup — 启动聚合

一键启动机械臂驱动和模型发布的 launch 文件。通过组合 `tl_driver` 和 `tl_description` 的 launch 文件，实现快速启动。

详细说明请参考 [tl_bringup/README.md](tl_bringup/README.md)。

### 2.2 tl_description — 模型描述

提供各型号机械臂的 URDF 模型文件和网格文件（STL），并为其他功能包提供机械臂关节间的坐标变换关系。

- 支持 14 种型号的 URDF 模型
- 包含碰撞检测网格和可视化网格
- 提供 RViz 配置文件
- 支持通过 `robot_state_publisher` 发布 TF 变换

详细说明请参考 [tl_description/README.md](tl_description/README.md)。

### 2.3 tl_driver — 硬件驱动

通过 TCP/IP 协议与机械臂控制器通信，提供机械臂的核心控制与状态监控功能：

- **状态反馈**：实时发布关节状态（位置、速度、力矩）、末端位姿以及机械臂整体运行状态，供上层模块订阅使用。
- **连接管理**：提供与机械臂控制器的建立连接与断开连接功能。
- **电源控制**：支持机械臂的上电与下电操作。
- **速度调节**：支持设定与读取机械臂运动速度。
- **点动控制**：提供关节/笛卡尔空间下的点动（Jogging）功能，可手动微调位姿。
- **状态监控**：支持查询机器人运行状态、关节温度、关节电压、电机电流、当前力矩、关节速度等实时状态参数。
- **模式切换**：支持设定与读取当前控制模式。
- **坐标系配置**：支持设置工具参数和用户坐标系。
- **Modbus 通信**：提供 Modbus 读写功能，用于与外部设备交互。
- **轨迹录制与回放**：支持录制机械臂运动轨迹并回放。
- **队列运动**：支持将运动指令加入队列依次执行。

驱动基于 NexMotion SDK 的 Python 封装（SWIG），通过 TCP 连接控制器（默认 IP：`192.168.1.13`，端口：`6001`）。

详细说明请参考 [tl_driver/README.md](tl_driver/README.md)。

### 2.4 tl_gazebo — Gazebo 仿真

在 Gazebo 仿真环境中加载机械臂模型，并可通过 MoveIt2 对仿真的机械臂进行规划控制。

详细说明请参考 [tl_gazebo/README.md](tl_gazebo/README.md)。

### 2.5 tl_moveit2_config — MoveIt2 配置

为各系列机械臂提供运动规划控制功能，包括虚拟和真实机械臂控制两部分。

- 运动学求解器：KDL
- 包含 14 套独立配置（每个型号一套）
- 提供关节限位、初始位姿、控制器配置等
- 支持 RViz 可视化规划

详细说明请参考 [tl_moveit2_config/README.md](tl_moveit2_config/README.md)。

### 2.6 tl_ros2_interface — 消息与接口定义

为 TL 系列机械臂在 ROS2 框架下定义统一的通信接口，是整个工作空间的**基础依赖包**，供上层驱动和应用功能包调用。

- **消息（msg）**：定义了机械臂状态、末端位姿、运动指令、Modbus 参数模版、DH 参数、关节参数、工具参数等数据格式，为话题通信提供统一的数据结构。
- **服务（srv）**：定义了坐标变换、作业文件管理、DH 参数读写、关节参数查询与设置、位姿转换、Modbus 读写、点动控制、队列运动、轨迹录制与回放等功能的请求/响应格式。

该包不包含可执行代码，仅提供接口定义（`.msg` 和 `.srv` 文件），其他功能包通过编译生成的头文件引用这些接口类型。

详细说明请参考 [tl_ros2_interface/README.md](tl_ros2_interface/README.md)。
