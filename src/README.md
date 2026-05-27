<div align="center">

# 天链机械臂 ROS2 功能包使用说明书

四川天链机器人股份有限公司

文件修订记录：

| 版本号 | 时间 | 备注 |
| :---: | :---- | :--- |
| V1.0 | 2026-5-22 | 拟制 |

TL 系列机械臂 ROS2 接口说明

</div>

## 目录
* 1 [功能包说明](#1-功能包说明)
* 2 [搭建环境](#2-搭建环境)
  * 2.1 [安装 ROS2](#21-安装-ros2)
  * 2.2 [编译](#22-编译)
* 3 [功能包概览](#3-功能包概览)
  * 3.1 [启动 tl_bringup](#31-启动-tl_bringup)
  * 3.2 [模型描述 tl_description](#32-模型描述-tl_description)
  * 3.3 [硬件驱动 tl_driver](#33-硬件驱动-tl_driver)
  * 3.4 [Gazebo 仿真 tl_gazebo](#34-gazebo-仿真-tl_gazebo)
  * 3.5 [MoveIt2 配置 tl_moveit2_config](#35-moveit2-配置-tl_moveit2_config)
  * 3.6 [ROS消息接口 tl_ros2_interface](#36-ros消息接口-tl_ros2_interface)
* 4 [功能运行](#4-功能运行)
  * 4.1 [运行真实机械臂](#41-运行真实机械臂)
  * 4.2 [运行 MoveIt2 + RViz 虚拟控制](#42-运行-moveit2--rviz-虚拟控制)
  * 4.3 [运行 Gazebo 仿真机械臂](#43-运行-gazebo-仿真机械臂)
* 5 [安全注意事项](#5-安全注意事项)
* 6 [许可与联系](#6-许可与联系)

---

## 1 功能包说明

该功能包的主要作用为提供 TL 系列机械臂的 ROS2 支持，以下为使用环境。

- **当前支持的机械臂型号：**
  - **6 轴系列**：TCB605、TCB605F、TCB605L、TCB605LV、TCB605V、TCB610、TCB610V（末端负载 5~10 kg）
  - **7 轴系列**：TCB705、TCB705F、TCB705L、TCB705LV、TCB705V、TCB710、TCB710V（末端负载 5~10 kg）
  - 型号命名规则：`TCB` + 轴数（6/7）+ 负载能力（05=5kg, 10=10kg）+ 后缀（F=力控, L=加长, V=视觉, LV=加长+视觉）
- **控制器版本**：2403
- **基于 Ubuntu 版本**：22.04
- **ROS2 版本**：Humble

## 2 搭建环境

在使用功能包之前，需要完成以下环境搭建。

### 2.1 安装 ROS2

请参考 ROS2 官方安装指南进行安装：

> [ROS2 Humble 安装教程](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### 2.2 编译

将功能包导入工作空间，编译全部功能包：

```bash
mkdir -p ~/tl_robot_ws/src
cp -r tl_robot_ros2_cpp/src ~/tl_robot_ws/src
cd ~/tl_robot_ws
colcon build
source install/setup.bash
```

编译完成后即可进行功能包的运行操作。

## 3 功能包概览

当前工作空间包含 **7 个功能包**，每个功能包的作用如下：

```
tl_robot_ros2_cpp/src/
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
├── tl_moveit2_config/       # MoveIt2 配置
│   ├── tl_tcb605_config/    # 14 套 MoveIt2 配置（每型号一套）
│   ├── tl_tcb605f_config/     ├── config/  (SRDF、限位、运动学等)
│   ├── ...                    └── launch/ (demo、move_group、rviz)
│   └── tl_tcb710v_config/
├── tl_ros2_interface/       # ROS2 消息与服务接口
    ├── msg/                 # 12 个 msg 定义文件
    └── srv/                 # 41 个 srv 定义文件
```

### 3.1 启动 tl_bringup

该功能包为机械臂的节点启动功能包，提供一键启动机械臂驱动和模型发布的 launch 文件。通过组合 `tl_driver` 和 `tl_description` 的 launch 文件，实现快速启动。

详细说明请参考 [tl_bringup/README.md](tl_bringup/README.md)。

### 3.2 模型描述 tl_description

该功能包为机械臂模型描述功能包，提供各型号机械臂的 URDF 模型文件和网格文件（STL），并为其他功能包提供机械臂关节间的坐标变换关系。

- 支持 14 种型号的 URDF 模型
- 包含碰撞检测网格和可视化网格
- 提供 RViz 配置文件
- 支持通过 `robot_state_publisher` 发布 TF 变换

详细说明请参考 [tl_description/README.md](tl_description/README.md)。

### 3.3 硬件驱动 tl_driver

该功能包为机械臂的 ROS2 底层驱动功能包，通过 TCP/IP 协议与机械臂控制器通信，实现以下功能：

- **发布的话题：**
  - `/joint_states`（`sensor_msgs/JointState`）：关节状态
  - `/tcp_pose`（`CartesianPose`）：末端位姿
  - `/arm_status`（`ArmStatus`）：机械臂运行状态

- **提供的服务：**
  - 连接/断开：`/tl_driver/connect_arm`、`/tl_driver/disconnect_arm`
  - 电源控制：`/tl_driver/power_on`、`/tl_driver/power_off`
  - 速度控制：`/tl_driver/set_speed`、`/tl_driver/get_speed`
  - 点动控制：`/tl_driver/start_jogging`、`/tl_driver/stop_jogging`
  - 状态查询：`/tl_driver/get_robot_state`、`/tl_driver/get_joint_temperature`、`/tl_driver/get_joint_voltage`、`/tl_driver/get_motor_current`
  - 坐标系设置：`/tl_driver/set_tool_param`、`/tl_driver/set_user_coord`
  - Modbus 通信：`/tl_driver/modbus_write`、`/tl_driver/modbus_read`
  - 轨迹录制与回放：`/tl_driver/track_save`、`/tl_driver/track_playback`
  - 队列运动：`/tl_driver/queue_motion_movej`、`/tl_driver/queue_motion_set_status`
  - 位姿转换：`/tl_driver/get_quat2rpy`、`/tl_driver/get_rpy2quat` 等

驱动基于 NexMotion SDK 的 Python 封装（SWIG），通过 TCP 连接控制器（默认 IP：`192.168.1.13`，端口：`6001`）。

详细说明请参考 [tl_driver/README.md](tl_driver/README.md)。

### 3.4 Gazebo 仿真 tl_gazebo

该功能包为 Gazebo 仿真机械臂功能包，主要功能为在 Gazebo 仿真环境中加载机械臂模型，并可通过 MoveIt2 对仿真的机械臂进行规划控制。

详细说明请参考 [tl_gazebo/README.md](tl_gazebo/README.md)。

### 3.5 MoveIt2 配置 tl_moveit2_config

该功能包为机械臂的 MoveIt2 适配功能包，为各系列机械臂提供运动规划控制功能，包括虚拟机械臂控制和真实机械臂控制两部分。

- 运动学求解器：KDL
- 包含 14 套独立配置（每个型号一套）
- 提供关节限位、初始位姿、控制器配置等
- 支持 RViz 可视化规划

详细说明请参考 [tl_moveit2_config/README.md](tl_moveit2_config/README.md)。

### 3.6 ROS消息接口 tl_ros2_interface

该功能包为 TL 系列机械臂在 ROS2 框架下提供消息（msg）和服务（srv）接口定义，供上层驱动或应用调用。包含以下内容：

- **消息（msg）**：ArmStatus、CartesianPose、JobFileName、JobInsertMove、ModbusMasterParam、ModbusRTUParam、ModbusTCPParam、MoveCommand、ObjectInfo、RobotDHParam、RobotJointParam、ToolParam
- **服务（srv）**：坐标转换、作业文件管理、DH 参数读写、关节参数查询与设置、位姿转换、Modbus 读写、点动控制、队列运动、轨迹录制与回放等 41 个服务接口

详细说明请参考 [tl_ros2_interface/README.md](tl_ros2_interface/README.md)。


详细说明请参考 [tl_vision/README.md](tl_vision/README.md)。

## 4 功能运行

### 4.1 运行真实机械臂

使用如下指令可以启动机械臂硬件驱动，连接真实机械臂：

```bash
source ~/tl_robot_ws/install/setup.bash
ros2 launch tl_bringup tl_tcb605_bringup.launch.py 
```

`<arm_type>` 需要使用实际机械臂型号替换，可选项为：
- **6 轴系列**：`tcb605`、`tcb605f`、`tcb605l`、`tcb605lv`、`tcb605v`、`tcb610`、`tcb610v`
- **7 轴系列**：`tcb705`、`tcb705f`、`tcb705l`、`tcb705lv`、`tcb705v`、`tcb710`、`tcb710v`

**修改通信参数：**

若机械臂 IP 地址被修改，需要修改 `tl_driver/config/` 下对应型号的 YAML 配置文件：

```yaml
tl_driver:
  ros__parameters:
    arm_ip: "192.168.1.13"            # TCP连接IP
    arm_port: "6001"                  # TCP连接端口
    arm_port_aux: "7000"              # 辅助端口
    arm_type: "TCB605"                # 机械臂型号
    arm_joints: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
```

### 4.2 运行 MoveIt2 + RViz 虚拟控制

使用如下指令启动 MoveIt2 虚拟机械臂控制（不连接真实硬件）：

```bash
source ~/tl_robot_ws/install/setup.bash
ros2 launch tl_tcb605_config demo.launch.py
```

将 `tl_tcb605_config` 替换为对应型号的配置包名称，例如 `tl_tcb710_config`、`tl_tcb705_config` 等。

### 4.3 运行 Gazebo 仿真机械臂

使用如下指令启动 Gazebo 仿真，并可通过 MoveIt2 进行规划控制：

```bash
source ~/tl_robot_ws/install/setup.bash
ros2 launch tl_gazebo gazebo_tcb605_demo.launch.py
```

## 5 安全注意事项
- 每次使用前检查机械臂安装情况，包括固定螺丝是否松动
- 机械臂运行过程中，人员不可处于机械臂工作范围内
- 不使用时将机械臂置于安全位置并断开电源

## 6 许可与联系

 **许可证**：

**公司官网**：[https://www.tlibot.com/](https://www.tlibot.com/)

**四川天链机器人股份有限公司**

如有问题，请在仓库 issue 提出或联系包维护者。
