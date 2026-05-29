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

`src/` 目录包含 **7 个功能包目录**，每个功能包（及子包）的作用如下：

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
    └── srv/                 # 43 个 srv 定义文件
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

通过 TCP/IP 协议与机械臂控制器通信，实现以下功能：

- **发布的话题：**
  - `/joint_states`（`sensor_msgs/JointState`）：关节状态
  - `/tcp_pose`（`CartesianPose`）：末端位姿
  - `/arm_status`（`ArmStatus`）：机械臂运行状态

- **提供的服务：**
  - 连接/断开：`/tl_driver/connect_arm`、`/tl_driver/disconnect_arm`
  - 电源控制：`/tl_driver/power_on`、`/tl_driver/power_off`
  - 速度控制：`/tl_driver/set_speed`、`/tl_driver/get_speed`
  - 点动控制：`/tl_driver/start_jogging`、`/tl_driver/stop_jogging`
  - 状态查询：`/tl_driver/get_robot_state`、`/tl_driver/get_joint_temperature`、`/tl_driver/get_joint_voltage`、`/tl_driver/get_motor_current`、`/tl_driver/get_current_motor_torque`、`/tl_driver/get_current_line_joint_speed`
  - 模式控制：`/tl_driver/set_current_mode`、`/tl_driver/get_current_mode`
  - 坐标系设置：`/tl_driver/set_tool_param`、`/tl_driver/set_user_coord`
  - Modbus 通信：`/tl_driver/modbus_write`、`/tl_driver/modbus_read`
  - 轨迹录制与回放：`/tl_driver/track_save`、`/tl_driver/track_playback`
  - 队列运动：`/tl_driver/queue_motion_movej`、`/tl_driver/queue_motion_set_status`

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

为 TL 系列机械臂在 ROS2 框架下提供消息（msg）和服务（srv）接口定义，供上层驱动或应用调用。

- **消息（msg）**（共 11 个）：ArmStatus、CartesianPose、JobFileName、ModbusMasterParam、ModbusRTUParam、ModbusTCPParam、MoveCommand、ObjectInfo、RobotDHParam、RobotJointParam、ToolParam
- **服务（srv）**（共 43 个）：坐标转换、作业文件管理、DH 参数读写、关节参数查询与设置、位姿转换、Modbus 读写、点动控制、队列运动、轨迹录制与回放等

详细说明请参考 [tl_ros2_interface/README.md](tl_ros2_interface/README.md)。
