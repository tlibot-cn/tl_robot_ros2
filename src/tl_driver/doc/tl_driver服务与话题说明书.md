<div align="center">

# tl_driver服务与话题说明书

文件修订记录：

|版本号 | 时间 | 备注 |
| :---: | :---- | :---: |
|V1.0 | 2026-4-29 | 拟制 |
|V1.1 | 2026-5-9  | 修订（添加[查询机械臂状态](#查询机械臂状态)、[查询库版本信息](#查询库版本信息)、[查询关节参数](#查询关节参数)、[设置关节参数](#设置关节参数)、[查询关节温度](#查询关节温度)、[查询关节电压](#查询关节电压)、[查询电机电流](#查询电机电流)、[查询关节软件版本号](#查询关节软件版本号)、<br>[查询算法库版本](#查询算法库版本)、[设置机械臂默认DH参数](#设置机械臂默认DH参数)、[设置机械臂默认笛卡尔参数](#设置机械臂默认笛卡尔参数)、[日志下载](#日志下载)、[查询运行速度](#查询运行速度)、[设置坐标系编号](#设置坐标系编号)、[设置机械臂DH参数](#设置机械臂DH参数)、<br>[四元数转欧拉角](#四元数转欧拉角)、[欧拉角转四元数](#欧拉角转四元数)、[欧拉角转旋转矩阵](#欧拉角转旋转矩阵)、[位姿转旋转矩阵](#位姿转旋转矩阵)、[旋转矩阵转位姿](#旋转矩阵转位姿)、[旋转矩阵转位姿](#旋转矩阵转位姿)、[设置控制器有线网口IP](#设置控制器有线网口IP)、<br>[查询控制器序列号ID](#查询控制器序列号ID)、[查询当前坐标系](#查询当前坐标系)等接口|
|V1.2 | 2026-5-27 | 修订（job_insert_* 接口从话题改为服务、移除 tool_hand_calib、新增 [删除作业文件](#删除作业文件) 服务、新增 [查询机械臂运行状态](#查询机械臂运行状态) 话题）|
|V1.3 | 2026-5-27 | 修订（新增 [查询当前运行模式](#查询当前运行模式) 服务，补漏 `get_current_mode`）|
|V1.4 | 2026-5-29 | 修订（新增 [查询当前电机力矩](#查询当前电机力矩)、[查询当前线速度和关节速度](#查询当前线速度和关节速度) 服务）|
|V1.5 | 2026-6-8  | 修订（详细化全部接口的输入输出参数说明，补全单位、范围、注意事项；优化参数表格式；修复锚点链接）|
|V1.6 | 2026-6-23 | 新增 15.4 发送跟踪笛卡尔位姿（已于 V1.10 删除） |
|V1.7 | 2026-10-17 | 单位对齐 SDK 修正：扭矩 %、用户坐标 mm、`MoveCommand` 布局/`pl`/`spin`/MoveL 速度、ServoJ 固定 7 元素、电机电流口径说明 |
|V1.8 | 2026-10-17 | ROS 接口统一 REP 103：关节 rad、笛卡尔 m、线速度 m/s；控制器 mm/度在驱动边界换算；协议包接口（可达性）保留单位制标志 |
|V1.9 | 2026-10-17 | 移除 18 个服务：纯数学变换（5）、作业文件（7）、全局路点（2）、`get_nexmotion_lib_version`、`set_dh_param`、`restore_default_dh_param`、`log_download`；对应章节已删除，章节号保留不复位以避免破坏既有引用 |
|V1.10 | 2026-10-19 | 移除 ServoL：删除 `/tl_driver/set_servol_pos` 话题、`ServolMove` 消息及 15.4 发送跟踪笛卡尔位姿章节；笛卡尔直线伺服改由消费方本地 IK 后经 ServoJ 下发 |
|V1.11 | 2026-10-20 | 移除 `/tl_driver/connect_arm` 与 `/tl_driver/disconnect_arm` 服务（服务数 47 → 45）：连接只在节点启动时建立、断开只在节点退出时执行；2.1/2.2 章节删除，章节号保留不复位 |
|V1.12 | 2026-10-20 | `get_robot_state` 补充入参范围与 `stop=true` 立即返回语义；服务回调组按域拆分，该服务使用独立回调组 |
|V1.13 | 2026-10-20 | `get_pos_reachable` 新增 `reachable` 字段（`success` 改为查询成功语义），`pos` 限 14 位 |
|V1.14 | 2026-10-21 | 状态话题化：`get_robot_state` 改为返回 ACS/MCS 缓存；新增 `joint_dynamics`/`joint_thermal`/`io_state` 话题与 `move_joint`/`move_linear`/`play_track` action（第 16 章） |
|V1.15 | 2026-10-21 | 新增 16.4 运动 Action 测试命令（MoveJ 10°、MoveL 5 cm）与回退命令 |

</div>

## 目录
* [1 通用约定](#1-通用约定)
* [2 连接管理接口](#2-连接管理接口)
  * [2.3 机械臂上电](#23-机械臂上电)
  * [2.4 机械臂下电](#24-机械臂下电)
* [4 信息查询接口](#4-信息查询接口)
  * [4.1 查询关节角度](#41-查询关节角度)
  * [4.2 查询末端位姿](#42-查询末端位姿)
  * [4.3 查询运行速度](#43-查询运行速度)
  * [4.4 查询控制器序列号ID](#44-查询控制器序列号id)
  * [4.5 查询机械臂状态](#45-查询机械臂状态)
  * [4.6 查询库版本信息](#46-查询库版本信息)
  * [4.7 查询关节参数](#47-查询关节参数)
  * [4.8 查询关节温度](#48-查询关节温度)
  * [4.9 查询关节电压](#49-查询关节电压)
  * [4.10 查询电机电流](#410-查询电机电流)
  * [4.11 查询关节软件版本号](#411-查询关节软件版本号)
  * [4.13 查询当前坐标系](#413-查询当前坐标系)
  * [4.14 查询坐标系编号](#414-查询坐标系编号)
  * [4.15 查询机械臂DH参数](#415-查询机械臂dh参数)
  * [4.17 查询目标位姿可达状态](#417-查询目标位姿可达状态)
  * [4.18 查询机械臂运行状态](#418-查询机械臂运行状态)
  * [4.19 查询当前电机力矩](#419-查询当前电机力矩)
  * [4.20 查询当前线速度和关节速度](#420-查询当前线速度和关节速度)
* [5 机械臂基础功能设置接口](#5-机械臂基础功能设置接口)
  * [5.1 设置运行速度](#51-设置运行速度)
  * [5.2 设置控制器有线网口IP](#52-设置控制器有线网口ip)
  * [5.3 设置关节参数](#53-设置关节参数)
  * [5.5 设置机械臂默认笛卡尔参数](#55-设置机械臂默认笛卡尔参数)
  * [5.6 坐标转换](#56-坐标转换)
* [6 MoveCommand 消息类型说明](#6-movecommand-消息类型说明)
* [7 实时运动控制接口](#7-实时运动控制接口)
  * [7.1 MoveJ运动控制](#71-movej运动控制)
  * [7.2 MoveL运动控制](#72-movel运动控制)
* [8 坐标系与工具管理接口](#8-坐标系与工具管理接口)
  * [8.1 设置工具手参数](#81-设置工具手参数)
  * [8.2 设置用户坐标系](#82-设置用户坐标系)
  * [8.3 设置坐标系编号](#83-设置坐标系编号)
  * [8.4 设置当前坐标系](#84-设置当前坐标系)
* [9 示教操作接口](#9-示教操作接口)
  * [9.1 开始点动](#91-开始点动)
  * [9.2 停止点动](#92-停止点动)
  * [9.3 设置拖拽模式](#93-设置拖拽模式)
  * [9.4 查看拖拽状态](#94-查看拖拽状态)
  * [9.5 拖拽轨迹保存](#95-拖拽轨迹保存)
  * [9.6 拖拽轨迹回放](#96-拖拽轨迹回放)
* [10 错误处理与零位标定接口](#10-错误处理与零位标定接口)
  * [10.1 清除错误](#101-清除错误)
  * [10.2 设置关节零点](#102-设置关节零点)
* [12 模式与IO控制接口](#12-模式与io控制接口)
  * [12.1 设置当前运行模式](#121-设置当前运行模式)
  * [12.2 查询当前运行模式](#122-查询当前运行模式)
  * [12.3 设置数字输出](#123-设置数字输出)
  * [12.4 查询数字输入输出状态](#124-查询数字输入输出状态)
* [13 Modbus通信接口](#13-modbus通信接口)
  * [13.1 写Modbus](#131-写modbus)
  * [13.2 读Modbus](#132-读modbus)
* [14 队列运动接口](#14-队列运动接口)
  * [14.1 设置MoveJ队列运动模式](#141-设置movej队列运动模式)
  * [14.2 MoveJ队列运动](#142-movej队列运动)
  * [14.3 停止MoveJ队列运动模式](#143-停止movej队列运动模式)
* [15 7000端口（关节跟踪）](#15-7000端口关节跟踪)
  * [15.1 打开关节跟踪模式](#151-打开关节跟踪模式)
  * [15.2 关闭关节跟踪模式](#152-关闭关节跟踪模式)
  * [15.3 发送跟踪关节位置](#153-发送跟踪关节位置)
* [16 状态诊断话题与 Action](#16-状态诊断话题与-action)
  * [16.1 状态流与回滚参数](#161-状态流与回滚参数)
  * [16.2 诊断话题](#162-诊断话题)
  * [16.3 运动 Action](#163-运动-action)
  * [16.4 运动 Action 测试命令](#164-运动-action-测试命令)


## 1 通用约定

### 1.1 坐标系编号
| 编号 | 含义 | 说明 |
| :---: | :--- | :--- |
| 0 | 关节坐标系（Joint） | 以各关节角度表示位置，单位：rad |
| 1 | 直角坐标系（Cartesian/Base） | 以基座为原点，位置(X,Y,Z)单位m，姿态(RX,RY,RZ)单位rad |
| 2 | 工具坐标系（Tool） | 以工具末端为参考的坐标系 |
| 3 | 用户坐标系（User） | 用户自定义的坐标系 |

### 1.2 运行模式编号
| 编号 | 含义 |
| :---: | :--- |
| 0 | 示教模式（手动操作） |
| 1 | 远程模式 |
| 2 | 运行模式（自动运行作业） |

### 1.3 返回值约定
- 所有 ROS2 服务的响应均包含 `bool success` 和 `string message` 字段
- `success = true`：操作成功（对应底层SDK返回值0）
- `success = false`：操作失败，`message` 中包含错误描述信息
- 对于 `std_srvs::srv::Trigger` 类型的服务，`success` 字段直接承载上述含义

### 1.4 单位制约定
> ROS 接口统一使用 REP 103 单位（m / rad / rad/s）。控制器与 SDK 的 mm / 度在 `tl_driver` 边界换算，本文档不再出现两套口径。
> 例外：`get_pos_reachable` 的 14 维控制器点位协议包自身携带单位制标志位，按协议原样透传、不做换算。

| 物理量 | 单位 | 说明 |
| :--- | :--- | :--- |
| 关节角 | rad | `/joint_states`、运动指令、坐标转换、DH 参数 |
| 笛卡尔位置 | m | `/tcp_pose`、运动指令、工具/用户坐标系 |
| 姿态 / 欧拉角 | rad | XYZ 内旋 |
| 线速度 | m/s | MOVL 指令、`get_current_line_joint_speed` |
| 关节速度 | rad/s | `get_current_line_joint_speed`、`RobotJointParam.*_rot_speed` |
| 角加速度 / 加加速度 | rad/s² / rad/s³ | ServoJ 约束、`RobotJointParam.max_acc` / `max_dec` |
| 百分比量 | % | 速度、加速度、扭矩（额定值百分比，取值见各接口） |
| 温度 / 电压 / 电流 | ℃ / V / A | SDK 头未标注温度、电压；电流 SDK 头标 ‰，与本仓 A 口径不符 |

- `MoveCommand.velocity`：MOVJ 为 %（0<v≤100）；MOVL 为 m/s（0<v≤1）。
- `target_pos_type` 是点位类型（PosType），**不携带单位制**；`target_pos_value` 的布局与单位换算规则见 `MoveCommand.msg` 文件头。
- `/tl_driver/set_servoj_pos` 的 `data` 为 rad，**固定 7 元素**（6 轴末位补 0）。

---
## 2 连接管理接口

> **连接生命周期**：驱动节点启动时自动连接控制器（6001 + 7000 双端口）并完成上电流程，退出时自动下电并断开；不提供运行时连接/断开服务。控制器掉线后需重启驱动节点恢复。

### 2.3 机械臂上电
| 功能描述 | 机械臂使能上电：先设置伺服状态为使能，再闭合强电接触器为电机供电 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/power_on` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 上电成功，`false` = 上电失败 |
| message | string | 失败时包含错误描述（如未清除报警） |

> **注意**：上电前需确保驱动已完成启动连接（`/arm_status` 正常发布），且无未清除的错误报警。若机械臂已处于上电状态，再次调用会返回成功。
#### 命令示例
```
ros2 service call /tl_driver/power_on std_srvs/srv/Trigger "{}"
```
### 2.4 机械臂下电
| 功能描述 | 机械臂使能下电：切断强电，电机抱闸锁死 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/power_off` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 下电成功，`false` = 下电失败 |
| message | string | 失败时包含错误描述 |

> **注意**：下电后机械臂会失去动力，抱闸锁死。若机械臂已处于下电状态，再次调用会返回成功。
#### 命令示例
```
ros2 service call /tl_driver/power_off std_srvs/srv/Trigger "{}"
```
## 4 信息查询接口
### 4.1 查询关节角度
| 功能描述 | 实时获取机械臂各关节当前角度位置（通过话题持续发布，频率100Hz） |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/joint_states` |
| 消息类型 | `sensor_msgs::msg::JointState` |

**输入参数**
无（话题订阅，无需请求参数）。

**输出/返回值**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| position | float64[] | rad | 各关节角度（ROS标准单位），数组长度 = 关节数 |
| velocity | float64[] | rad/s | 各关节角速度 |
| effort | float64[] | Nm | 各关节力矩 |
| name | string[] | — | 各关节名称 |
#### 命令示例
```
ros2 topic echo /joint_states
```
### 4.2 查询末端位姿
| 功能描述 | 实时获取机械臂末端（TCP）在直角坐标系下的位置和姿态（话题持续发布） |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/tcp_pose` |
| 消息类型 | `tl_ros2_interface/msg/CartesianPose` |

**输入参数**
无（话题订阅，无需请求参数）。

**输出/返回值**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| position.x | float64 | m | 末端位置X坐标（ROS标准单位） |
| position.y | float64 | m | 末端位置Y坐标 |
| position.z | float64 | m | 末端位置Z坐标 |
| rpy.x | float64 | rad | 末端姿态欧拉角 Roll |
| rpy.y | float64 | rad | 末端姿态欧拉角 Pitch |
| rpy.z | float64 | rad | 末端姿态欧拉角 Yaw |
| arm_angle | float64 | rad | 臂角（冗余自由度参数） |
#### 命令示例
```
ros2 topic echo /tcp_pose
```
### 4.3 查询运行速度
| 功能描述 | 查询机械臂当前全局运行速度百分比 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_speed` |
| 服务类型 | `tl_ros2_interface/srv/GetSpeed` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 单位 | 范围 | 说明 |
|--------|------|------|------|------|
| success | bool | — | — | `true` = 查询成功，`false` = 查询失败 |
| message | string | — | — | 失败时包含错误描述 |
| speed | float64 | % | 1～100 | 当前全局运行速度百分比 |
#### 命令示例
```
ros2 service call /tl_driver/get_speed tl_ros2_interface/srv/GetSpeed "{}"
```
### 4.4 查询控制器序列号ID
| 功能描述 | 查询机械臂控制器的唯一序列号标识 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_controller_id` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 成功时返回控制器序列号ID，失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/get_controller_id std_srvs/srv/Trigger "{}"
```
### 4.5 查询机械臂状态
| 功能描述 | 返回最近一帧缓存的机械臂状态报文（ACS/MCS），不再触发 SDK 查询 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_robot_state` |
| 服务类型 | `tl_ros2_interface/srv/GetRobotState` |

**输入参数**
| 参数名 | 类型 | 范围 | 说明 |
|--------|------|------|------|
| channel | int32 | — | 保留字段，当前忽略（缓存按 `position` 选择） |
| stop | bool | — | 必须为 `false`；`true` 返回失败 `"State stream is managed internally"` |
| mode | int32 | — | 保留字段，当前忽略 |
| interval | int32 | — | 保留字段，当前忽略 |
| io_state | bool | — | 保留字段，当前忽略 |
| position | int32 | 0 或 1 | `0` = 返回 ACS（关节）缓存，`1` = 返回 MCS（直角）缓存 |
| detail_motion_pos | bool | — | 保留字段，当前忽略 |
| pos_sum | int32 | — | 保留字段，当前忽略 |
| io_port | string[] | — | 保留字段，当前忽略 |
| optional | string[] | — | 保留字段，当前忽略 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 缓存可用，`false` = 未连接/无缓存/参数不支持 |
| message | string | 成功时为该通道最近一帧原始 JSON 报文；失败时为错误描述 |

> **注意**：状态流由驱动在连接时自动启动（`state_source=stream`），服务只读缓存，P99 < 10 ms，不再出现 5 s 超时。`position` 非 0/1 返回 `"Unsupported position"`；`io_state`/`detail_motion_pos`/`io_port` 不支持缓存模式；`state_source=poll` 返回 `"State stream disabled (state_source=poll)"`；缓存超过 3×`state_stream_interval_ms` 未更新返回 `"No fresh cached state"`。
#### 命令示例
```
ros2 service call /tl_driver/get_robot_state tl_ros2_interface/srv/GetRobotState \
"{
    channel: 1,
    stop: false,
    mode: 0,
    interval: 10,
    io_state: false,
    position: 0,
    detail_motion_pos: false,
    pos_sum: 1,
    io_port: ["DO1"],
    optional: ["ACS"]
}"
```
### 4.6 查询库版本信息
| 功能描述 | 查询本地API动态库（`_tl_host.so`）的版本信息 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_library_version` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 成功时返回动态库版本号（如 `"1.0.0"`），失败时包含错误描述 |

> **注意**：此接口查询的是本地SDK动态库版本，无需连接机械臂即可调用。
#### 命令示例
```
ros2 service call /tl_driver/get_library_version std_srvs/srv/Trigger "{}"
```
### 4.7 查询关节参数
| 功能描述 | 查询指定关节的减速比、限位、额定转速、最大加速度等硬件参数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_robot_joint_param` |
| 服务类型 | `tl_ros2_interface/srv/GetRobotJointParam` |

**输入参数**
| 参数名 | 类型 | 范围 | 说明 |
|--------|------|------|------|
| id | int32 | 1～6（或更多） | 关节轴序号 |

**输出/返回值**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| success | bool | — | `true` = 查询成功，`false` = 查询失败 |
| message | string | — | 失败时包含错误描述 |
| param.reduction_ratio | float64 | — | 关节减速比 |
| param.encoder_resolution | int32 | 位 | 编码器位数 |
| param.pos_sw_limit | float64 | rad | 轴正限位角度 |
| param.neg_sw_limit | float64 | rad | 轴反限位角度 |
| param.rated_rot_speed | float64 | rad/s | 电机额定正转速 |
| param.rated_derot_speed | float64 | rad/s | 电机额定反转速（新 SDK 无对应字段，当前恒为 0） |
| param.max_rot_speed | float64 | rad/s | 电机最大正转速 |
| param.max_derot_speed | float64 | rad/s | 电机最大反转速（新 SDK 无对应字段，当前恒为 0） |
| param.rated_vel | float64 | rad/s | 额定正速度（新 SDK 无对应字段，当前恒为 0） |
| param.rated_devel | float64 | rad/s | 额定反速度（新 SDK 无对应字段，当前恒为 0） |
| param.max_acc | float64 | rad/s² | 最大加速度 |
| param.max_dec | float64 | rad/s² | 最大减速度 |
| param.direction | int32 | — | 模型方向：`1` = 正向，`-1` = 反向 |

> **注意**：关节参数直接影响机械臂的运动范围和安全限制，修改前请确认参数正确性。
#### 命令示例
```
ros2 service call /tl_driver/get_robot_joint_param tl_ros2_interface/srv/GetRobotJointParam "{id: 1}"
```
### 4.8 查询关节温度
| 功能描述 | 查询各关节电机当前温度 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_joint_temperature` |
| 服务类型 | `tl_ros2_interface/srv/GetJointTemperature` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| success | bool | — | `true` = 查询成功，`false` = 查询失败 |
| message | string | — | 失败时包含错误描述 |
| temperatures | float64[] | ℃ | 各关节电机当前温度，数组长度 = 关节数 |
#### 命令示例
```
ros2 service call /tl_driver/get_joint_temperature tl_ros2_interface/srv/GetJointTemperature "{}"
```
### 4.9 查询关节电压
| 功能描述 | 查询各关节当前电压 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_joint_voltage` |
| 服务类型 | `tl_ros2_interface/srv/GetJointVoltage` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| success | bool | — | `true` = 查询成功，`false` = 查询失败 |
| message | string | — | 失败时包含错误描述 |
| joint_voltage | float64[] | V | 机械臂本体各关节电压 |
| positioner_voltage | float64[] | V | 外部轴各关节电压 |
#### 命令示例
```
ros2 service call /tl_driver/get_joint_voltage tl_ros2_interface/srv/GetJointVoltage "{}"
```
### 4.10 查询电机电流
| 功能描述 | 查询机器人本体 7 轴电机电流（外部轴同步电流未暴露） |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_motor_current` |
| 服务类型 | `tl_ros2_interface/srv/GetMotorCurrent` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| success | bool | — | `true` = 查询成功，`false` = 查询失败 |
| message | string | — | 失败时包含错误描述 |
| motor_current | float64[] | A | 机器人本体 7 轴电机电流（SDK 头标 ‰，与实际不符，此处按 A 口径） |
#### 命令示例
```
ros2 service call /tl_driver/get_motor_current tl_ros2_interface/srv/GetMotorCurrent "{}"
```
### 4.11 查询关节软件版本号
| 功能描述 | 查询指定关节（轴）的软件版本号 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_joint_software_version` |
| 服务类型 | `tl_ros2_interface/srv/GetJointSoftwareVersion` |

**输入参数**
| 参数名 | 类型 | 范围 | 说明 |
|--------|------|------|------|
| axis_num | int32 | 1～6（或更多） | 关节（轴）号 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 成功时返回版本号信息，失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/get_joint_software_version tl_ros2_interface/srv/GetJointSoftwareVersion "{axis_num: 1}"
```
### 4.13 查询当前坐标系
| 功能描述 | 查询当前激活的工作坐标系类型 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_current_coord` |
| 服务类型 | `tl_ros2_interface/srv/GetCurrentCoord` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 范围 | 说明 |
|--------|------|------|------|
| success | bool | — | `true` = 查询成功，`false` = 查询失败 |
| message | string | — | 失败时包含错误描述 |
| coord | int32 | 0～3 | 坐标系序号：0=关节，1=直角，2=工具，3=用户 |
#### 命令示例
```
ros2 service call /tl_driver/get_current_coord tl_ros2_interface/srv/GetCurrentCoord "{}"
```
### 4.14 查询坐标系编号
| 功能描述 | 查询当前工具坐标系和用户坐标系的编号 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_coord_num` |
| 服务类型 | `tl_ros2_interface/srv/GetCoordNum` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 失败时包含错误描述 |
| tool_num | int32 | 当前工具坐标系序号 |
| user_num | int32 | 当前用户坐标系序号 |
```
ros2 service call /tl_driver/get_coord_num tl_ros2_interface/srv/GetCoordNum "{}"
```
### 4.15 查询机械臂DH参数
| 功能描述 | 查询机械臂的标准DH几何参数（alpha/a/theta/d、安装角等） |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_dh_param` |
| 服务类型 | `tl_ros2_interface/srv/GetDHParam` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| success | bool | — | `true` = 查询成功，`false` = 查询失败 |
| message | string | — | 失败时包含错误描述 |
| param | RobotDHParam | — | 机械臂DH参数结构体，字段：alpha/a/theta/d（1-6轴标准DH参数，rad/m）、euler_angle（欧拉角模式）、mounting_angle（安装角，rad） |
#### 命令示例
```
ros2 service call /tl_driver/get_dh_param tl_ros2_interface/srv/GetDHParam
```
### 4.17 查询目标位姿可达状态
| 功能描述 | 检查指定目标位姿在给定运动方式下是否可达 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_pos_reachable` |
| 服务类型 | `tl_ros2_interface/srv/GetPosReachable` |

**输入参数**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| pos | float64[] | 按 [1] 标志 | 14维位姿数组：[0]坐标系 [1]单位制 [2]形态 [3]工具序号 [4]用户序号 [5][6]备用 [7~13]点位信息。控制器协议包（位置 mm、角度按 [1]），本接口原值透传 |
| move_type | string | — | 运动方式，如 `"MOVJ"` 或 `"MOVL"` |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | 查询本身是否成功；失败原因见 `message` |
| message | string | 查询结果或失败原因 |
| reachable | bool | `success=true` 时有效：`true` = 可达，`false` = 不可达 |
#### 命令示例
```
ros2 service call /tl_driver/get_pos_reachable tl_ros2_interface/srv/GetPosReachable \
'{
    pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -57.14, -32.93, 19.74, -89.89, -19.77, 0.0], move_type: "MOVJ"
}'
```
### 4.18 查询机械臂运行状态
| 功能描述 | 实时获取机械臂当前运行状态（停止/暂停/运行中），通过话题持续发布 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/arm_status` |
| 消息类型 | `tl_ros2_interface/msg/ArmStatus` |

**输入参数**
无（话题订阅，无需请求参数）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| stamp | Time | 时间戳 |
| run_state | string | 运行状态：`"STOP"` = 停止，`"PAUSE"` = 暂停，`"RUNNING"` = 运行中，`"UNKNOWN"` = SDK 返回未识别值 |
#### 命令示例
```
ros2 topic echo /arm_status
```

### 4.19 查询当前电机力矩
| 功能描述 | 查询机械臂当前各电机力矩 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_current_motor_torque` |
| 服务类型 | `tl_ros2_interface/srv/GetCurrentMotorTorque` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| success | bool | — | `true` = 查询成功，`false` = 查询失败 |
| message | string | — | 失败时包含错误描述 |
| motor_torque | int32[] | % | 机械臂本体当前各电机力矩（额定扭矩百分比，数组长度 7） |
| motor_torque_sync | int32[] | % | 同步轴当前各电机力矩（数组长度 5） |
#### 命令示例
```
ros2 service call /tl_driver/get_current_motor_torque tl_ros2_interface/srv/GetCurrentMotorTorque "{}"
```
### 4.20 查询当前线速度和关节速度
| 功能描述 | 查询机械臂当前末端的线速度和各关节角速度 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_current_line_joint_speed` |
| 服务类型 | `tl_ros2_interface/srv/GetCurrentLineJointSpeed` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| success | bool | — | `true` = 查询成功，`false` = 查询失败 |
| message | string | — | 失败时包含错误描述 |
| line_speed | float64 | m/s | 当前末端线速度 |
| joint_speed | float64[] | rad/s | 机械臂本体当前各关节速度，数组长度 7 |
| joint_speed_sync | float64[] | rad/s | 同步轴当前各关节速度，数组长度 5 |
#### 命令示例
```
ros2 service call /tl_driver/get_current_line_joint_speed tl_ros2_interface/srv/GetCurrentLineJointSpeed "{}"
```

## 5 机械臂基础功能设置接口
### 5.1 设置运行速度
| 功能描述 | 设置机械臂全局自动运行速度百分比 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_speed` |
| 服务类型 | `tl_ros2_interface/srv/SetSpeed` |

**输入参数**
| 参数名 | 类型 | 单位 | 范围 | 说明 |
|--------|------|------|------|------|
| speed | float64 | % | 1～100 | 全局运行速度百分比 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_speed tl_ros2_interface/srv/SetSpeed "{speed: 20.0}"
```
### 5.2 设置控制器有线网口IP
| 功能描述 | 配置机械臂控制器的有线网络参数（IP地址、网关、DNS） |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_controller_ip` |
| 服务类型 | `tl_ros2_interface/srv/SetControllerIP` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| name | string | 网络接口名称，如 `"eth0"` |
| addr | string | IP地址，如 `"192.168.1.13"` |
| gateway | string | 网关地址，可为空字符串 |
| dns | string | DNS服务器地址，可为空字符串 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |

> **注意**：修改IP后需重新连接机械臂。
#### 命令示例
```
ros2 service call /tl_driver/set_controller_ip tl_ros2_interface/srv/SetControllerIP \
"{
    name: 'eth0',
    addr: '192.168.1.13',
    gateway: '',
    dns: ''
}"
```
### 5.3 设置关节参数
| 功能描述 | 设置指定关节的减速比、限位、转速、加速度等硬件参数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_robot_joint_param` |
| 服务类型 | `tl_ros2_interface/srv/SetRobotJointParam` |

**输入参数**
| 参数名 | 类型 | 单位 | 范围 | 说明 |
|--------|------|------|------|------|
| id | int32 | — | 1～6（或更多） | 关节轴序号 |
| param | RobotJointParam | — | — | 关节参数结构体（详见 [查询关节参数](#47-查询关节参数)） |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |

> **注意**：修改关节参数会直接影响机械臂的运动范围和安全限制，请谨慎操作。参数字段含义见 [查询关节参数](#47-查询关节参数) 的输出说明。
#### 命令示例
```
ros2 service call /tl_driver/set_robot_joint_param tl_ros2_interface/srv/SetRobotJointParam \
"{
    id: 1,
    param:
    {
        reduction_ratio: 1.0,
        encoder_resolution: 19,
        pos_sw_limit: 179.0,
        neg_sw_limit: -179.0,
        rated_rot_speed: 30.0,
        rated_derot_speed: -30.0,
        max_rot_speed: 1.0,
        max_derot_speed: -1.0,
        rated_vel: 180.0,
        rated_devel: -180.0,
        max_acc: 1.5,
        max_deacc: -1.5,
        direction: -1
    }
}"

```
### 5.5 设置机械臂默认笛卡尔参数
| 功能描述 | 将机械臂笛卡尔空间参数恢复为出厂默认值 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_default_cartesian_param` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_default_cartesian_param std_srvs/srv/Trigger "{}"
```
### 5.6 坐标转换
| 功能描述 | 在不同坐标系之间进行空间坐标转换（纯数学计算，不触发运动） |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/coord_transform` |
| 服务类型 | `tl_ros2_interface/srv/CoordTransform` |

**输入参数**
| 参数名 | 类型 | 范围 | 说明 |
|--------|------|------|------|
| origin_coord | int32 | 0～3 | 源坐标系编号 |
| target_coord | int32 | 0～3 | 目标坐标系编号 |
| form | int32 | — | 转换形态/模式 |
| origin_pos | float64[] | — | 原始坐标系下的位姿数据 |
| reference_pos | float64[] | — | 参考位姿数据 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 转换成功，`false` = 转换失败（如奇异点、不可达） |
| message | string | 失败时包含错误描述 |
| target_pos | float64[] | 目标坐标系下的转换结果 |
#### 命令示例
```
ros2 service call /tl_driver/coord_transform tl_ros2_interface/srv/CoordTransform \
'{
    origin_coord: 0,
    target_coord: 1,
    form: 0,
    origin_pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    reference_pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}'
```
## 6 MoveCommand 消息类型说明

`MoveCommand` 消息类型字段说明如下：

| 字段名 | 类型 | 单位 | 范围 | 说明 |
|--------|------|------|------|------|
| target_pos_value | float64[] | rad / m | — | 14 维（与 SDK `MoveCmd::targetPosValue` 一致）：前 7 位本体、后 7 位外部轴。`coord=0` 时 [0..6] 为关节角（rad）；`coord=1/2/3` 时 [0]X(m) [1]Y(m) [2]Z(m) [3]RX(rad) [4]RY(rad) [5]RZ(rad) [6]冗余臂角(rad)；[7..13] 外部轴按 SDK 原值透传 |
| target_pos_name | string | — | — | 内置点位名，如 `"GP0001"`（`target_pos_type=5/6` 时使用） |
| target_pos_type | int32 | — | 0～6 | 点位类型 PosType：0=自定义数组 1=P 2=E 3=RP 4=AP 5=系统内置GP 6=含外部轴的GE |
| coord | int32 | — | 0～3 | 坐标系：0=关节（rad），1=直角，2=工具，3=用户（1/2/3 时位置 m、姿态 rad） |
| velocity | float64 | % / m/s | — | MOVJ 为 %（0<v≤100）；MOVL 为 m/s（0<v≤1）；队列接口同此口径 |
| velocity_sync | float64 | % / m/s | — | 外部轴同步速度：与 velocity 同口径（MOVJ %；MOVL m/s） |
| acc | float64 | % | (0,100] | 加速度百分比 |
| dec | float64 | % | (0,100] | 减速度百分比 |
| pl | int32 | — | 0～5 | 平滑过渡等级：0=精确到达，非百分比 |
| time | int32 | — | — | 提前执行时间（SDK 头未标注单位） |
| tool_num | int32 | — | — | 工具坐标系编号 |
| user_num | int32 | — | — | 用户坐标系编号 |
| posidtype | int32 | — | — | 变量类型：0=一级(P/GP) 1=二级(P[I001]/GP[I001]) |
| configuration | int32 | — | — | 形态配置 |
| spin | int32 | — | 0～2 | 仅MOVCA指令使用：0=姿态不变，1=六轴不转，2=六轴旋转 |
| para_sync | bool | — | — | `true` = 外部轴同步，`false` = 不同步 |

## 7 实时运动控制接口
### 7.1 MoveJ运动控制
| 功能描述 | 通过话题下发关节角度数据，驱动机器人实时关节空间运动到目标位置（不依赖作业文件） |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/tl_driver/moveJ` |
| 消息类型 | `tl_ros2_interface/msg/MoveCommand` |

**输入参数**
| 参数名 | 类型 | 单位 | 范围 | 说明 |
|--------|------|------|------|------|
| target_pos_value | float64[] | — | — | 14维位置数组（coord=0关节坐标系时为各关节目标角度） |
| target_pos_name | string | — | — | 目标位姿名称（通常留空） |
| target_pos_type | int32 | — | 1 | 目标位姿类型（实时运动固定为1） |
| coord | int32 | — | 0 | 坐标系（固定为关节坐标系） |
| velocity | float64 | % | 1～100 | 运动速度百分比 |
| velocity_sync | float64 | % | 1～100 | 同步轴速度百分比 |
| acc | float64 | % | 1～100 | 加速度百分比 |
| dec | float64 | % | 1～100 | 减速度百分比 |
| pl | int32 | — | 0～5 | 平滑过渡等级，0=精确到达，非百分比 |
| time | int32 | — | — | 提前执行时间（SDK 头未标注单位） |
| tool_num | int32 | — | — | 工具坐标系编号 |
| user_num | int32 | — | — | 用户坐标系编号 |
| posidtype | int32 | — | — | 变量类型 |
| configuration | int32 | — | — | 形态配置 |
| spin | int32 | — | 0～2 | 0=姿态不变，1=六轴不转，2=六轴旋转 |
| para_sync | bool | — | — | 外部轴是否同步 |

**输出/返回值**
无返回值（话题发布单向通信）。

> **注意**：实时运动控制不等同于作业文件运动，需确保机械臂已上电（power_on）后再调用。
#### 命令示例
```
ros2 topic pub --once /tl_driver/moveJ tl_ros2_interface/msg/MoveCommand \
"{
    target_pos_value: [1.5708, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    target_pos_name: '',
    target_pos_type: 0,
    coord: 0,
    velocity: 20.0,
    velocity_sync: 0.0,
    acc: 20.0,
    dec: 20.0,
    pl: 0,
    time: 0,
    tool_num: 0,
    user_num: 0,
    posidtype: 0,
    configuration: 0,
    spin: 0,
    para_sync: false
}"
```
### 7.2 MoveL运动控制
| 功能描述 | 通过话题下发直角坐标数据，驱动机器人末端沿直线实时运动到目标位姿 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/tl_driver/moveL` |
| 消息类型 | `tl_ros2_interface/msg/MoveCommand` |

**输入参数** 同 MoveJ，区别如下：
| 参数名 | 类型 | 单位 | 值 | 说明 |
|--------|------|------|------|------|
| coord | int32 | — | 1 | 坐标系（固定为直角坐标系） |
| target_pos_value | float64[] | m, rad | — | [X(m), Y(m), Z(m), RX(rad), RY(rad), RZ(rad), 0]（7维） |
| velocity | float64 | m/s | 0<v≤1 | 直线运动速度（与 MOVJ 的百分比口径不同） |

其余参数同 [MoveJ运动控制](#movej运动控制)。

**输出/返回值**
无返回值（话题发布单向通信）。

> **注意**：需确保机械臂已上电后再调用。
#### 命令示例
```
ros2 topic pub --once /tl_driver/moveL tl_ros2_interface/msg/MoveCommand \
"{
    target_pos_value: [0.010, 0.230, 0.245, -3.14, 0.0, -1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    target_pos_name: '',
    target_pos_type: 0,
    coord: 1,
    velocity: 0.02,
    velocity_sync: 0.0,
    acc: 20.0,
    dec: 20.0,
    pl: 0,
    time: 0,
    tool_num: 0,
    user_num: 0,
    posidtype: 0,
    configuration: 0,
    spin: 0,
    para_sync: false
}"
```
## 8 坐标系与工具管理接口
### 8.1 设置工具手参数
| 功能描述 | 配置指定编号的工具手坐标系偏移参数（TCP）、负载质量与质心 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_tool_param` |
| 服务类型 | `tl_ros2_interface/srv/SetToolParam` |

**输入参数**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| tool_num | int32 | — | 工具手编号 |
| param.x | float64 | m | 末端工具X轴向偏移量 |
| param.y | float64 | m | 末端工具Y轴向偏移量 |
| param.z | float64 | m | 末端工具Z轴向偏移量 |
| param.a | float64 | rad | 绕X轴旋转角度 |
| param.b | float64 | rad | 绕Y轴旋转角度 |
| param.c | float64 | rad | 绕Z轴旋转角度 |
| param.payload_mass | float64 | kg | 负载质量 |
| param.payload_inertia | float64 | kg·m² | 负载惯性 |
| param.payload_mass_center_x | float64 | m | 负载质心X坐标 |
| param.payload_mass_center_y | float64 | m | 负载质心Y坐标 |
| param.payload_mass_center_z | float64 | m | 负载质心Z坐标 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_tool_param tl_ros2_interface/srv/SetToolParam \
'{
    tool_num: 1,
    param:
    {
        x: 0.1,
        y: 0.02,
        z: 0.05,
        a: 0.1,
        b: 0.2,
        c: 0.3,
        payload_mass: 1.5,
        payload_inertia: 0.01,
        payload_mass_center_x: 0.005,
        payload_mass_center_y: 0.005,
        payload_mass_center_z: 0.01
    }
}'
```
### 8.2 设置用户坐标系
| 功能描述 | 配置指定编号的用户坐标系参数（相对基座的偏移和旋转） |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_user_coord` |
| 服务类型 | `tl_ros2_interface/srv/SetUserCoord` |

**输入参数**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| user_num | int32 | — | 用户坐标系编号（1～N） |
| pos.position.x | float64 | m | X方向偏移 |
| pos.position.y | float64 | m | Y方向偏移 |
| pos.position.z | float64 | m | Z方向偏移 |
| pos.rpy.x | float64 | rad | 绕X轴旋转（欧拉角） |
| pos.rpy.y | float64 | rad | 绕Y轴旋转（欧拉角） |
| pos.rpy.z | float64 | rad | 绕Z轴旋转（欧拉角） |
| pos.arm_angle | float64 | rad | 臂角 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_user_coord tl_ros2_interface/srv/SetUserCoord \
"{
    user_num: 1,
    pos:
    {
        header:
        {
            frame_id: 'base_link'
        },
        position:
        {
            x: 0.2,
            y: 0.1,
            z: 0.05
        },
        rpy:
        {
            x: 0.1,
            y: 0.2,
            z: 0.3
        },
        arm_angle: 0.0
    }
}"
```
### 8.3 设置坐标系编号
| 功能描述 | 设置当前激活的工具坐标系和用户坐标系编号（后续运动将基于这些坐标系） |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_coord_num` |
| 服务类型 | `tl_ros2_interface/srv/SetCoordNum` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| tool_num | int32 | 工具坐标系序号 |
| user_num | int32 | 用户坐标系序号 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
```
ros2 service call /tl_driver/set_coord_num tl_ros2_interface/srv/SetCoordNum "{tool_num: 1, user_num: 2}"
```
### 8.4 设置当前坐标系
| 功能描述 | 切换当前点动（Jogging）时使用的坐标系 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_current_coord` |
| 服务类型 | `tl_ros2_interface/srv/SetCurrentCoord` |

**输入参数**
| 参数名 | 类型 | 范围 | 说明 |
|--------|------|------|------|
| coord | int32 | 0～3 | 坐标系序号：0=关节，1=直角，2=工具，3=用户 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_current_coord tl_ros2_interface/srv/SetCurrentCoord "{coord: 0}"
```
## 9 示教操作接口
### 9.1 开始点动
| 功能描述 | 使指定关节轴沿指定方向连续运动（点动/微调） |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/start_jogging` |
| 服务类型 | `tl_ros2_interface/srv/Jogging` |

**输入参数**
| 参数名 | 类型 | 范围 | 说明 |
|--------|------|------|------|
| axis | int32 | 1～6（或更多） | 关节轴号 |
| direction | bool | `true` / `false` | 运动方向：`true` = 正方向，`false` = 负方向 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 点动成功，`false` = 点动失败 |
| message | string | 失败时包含错误描述 |

> **注意**：点动指令需周期性调用才能维持运动（通常每40ms调用一次），或与 [停止点动](#停止点动) 配合使用。
#### 命令示例
```
ros2 service call /tl_driver/start_jogging tl_ros2_interface/srv/Jogging "{axis: 3, direction: 1}"
```
### 9.2 停止点动
| 功能描述 | 停止指定关节轴的点动运动 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/stop_jogging` |
| 服务类型 | `tl_ros2_interface/srv/Jogging` |

**输入参数**
| 参数名 | 类型 | 范围 | 说明 |
|--------|------|------|------|
| axis | int32 | 1～6（或更多） | 要停止的关节轴号 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 停止成功，`false` = 停止失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/stop_jogging tl_ros2_interface/srv/Jogging "{axis: 3}"
```
### 9.3 设置拖拽模式
| 功能描述 | 设置机械臂拖拽（示教）模式 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_drag_mode` |
| 服务类型 | `tl_ros2_interface/srv/SetDragMode` |

**输入参数**
| 参数名 | 类型 | 范围 | 说明 |
|--------|------|------|------|
| mode | int32 | 0～3 | 拖拽模式：0=关闭（无），1=3D鼠标，2=力矩模式，3=位置模式 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_drag_mode tl_ros2_interface/srv/SetDragMode "{mode: 3}"
```
### 9.4 查看拖拽状态
| 功能描述 | 查询拖拽操作是否已结束 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_drag_status` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 拖拽已结束，`false` = 拖拽未结束或查询失败 |
| message | string | 包含状态信息；查询失败时含 `ret=` 错误码 |
> **注意**：`success` 同时承担业务状态与调用状态。需要区分"未结束"与"查询失败"时，检查 `message` 是否以 `Failed to get drag status` 开头。
#### 命令示例
```
ros2 service call /tl_driver/get_drag_status std_srvs/srv/Trigger "{}"
```
### 9.5 拖拽轨迹保存
| 功能描述 | 将拖拽示教的轨迹保存到指定文件 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/track_save` |
| 服务类型 | `tl_ros2_interface/srv/TrackSave` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| traj_name | string | 保存的轨迹文件名称，如 `"traj_test"` |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 保存成功，`false` = 保存失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/track_save tl_ros2_interface/srv/TrackSave "{traj_name: 'traj_test'}"
```
### 9.6 拖拽轨迹回放
| 功能描述 | 回放已保存的拖拽轨迹 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/track_playback` |
| 服务类型 | `tl_ros2_interface/srv/TrackPlayback` |

**输入参数**
| 参数名 | 类型 | 单位 | 范围 | 说明 |
|--------|------|------|------|------|
| vel | int32 | % | 1～100 | 轨迹回放速度百分比 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 回放成功，`false` = 回放失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/track_playback tl_ros2_interface/srv/TrackPlayback "{vel: 20}"
```
## 10 错误处理与零位标定接口
### 10.1 清除错误
| 功能描述 | 复位控制器的当前报警状态（清除红色错误）。在机械臂发生碰撞、超限位等错误后，排除物理故障后调用 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/clear_error` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 清除成功，`false` = 清除失败（如故障未排除） |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/clear_error std_srvs/srv/Trigger "{}"
```
### 10.2 设置关节零点
| 功能描述 | 将当前关节位置记录为零位。axis=1表示所有轴，axis=2～7表示单轴 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_axis_zero_pos` |
| 服务类型 | `tl_ros2_interface/srv/SetAxisZeroPos` |

**输入参数**
| 参数名 | 类型 | 范围 | 说明 |
|--------|------|------|------|
| axis | int32 | 1～7 | 轴号：1=全轴，2～7=单轴 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |

> **警告**：仅在机械臂物理对齐零刻度线后调用，否则将导致坐标系错乱，引发撞机风险！设置关节零点前需先执行 `power_off`（下电），完成设置后再 `power_on`（上电）。
#### 命令示例
```
ros2 service call /tl_driver/set_axis_zero_pos tl_ros2_interface/srv/SetAxisZeroPos "{axis: 1}"
```
## 12 模式与IO控制接口
### 12.1 设置当前运行模式
| 功能描述 | 切换控制器运行模式 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_current_mode` |
| 服务类型 | `tl_ros2_interface/srv/SetCurrentMode` |

**输入参数**
| 参数名 | 类型 | 范围 | 说明 |
|--------|------|------|------|
| mode | int32 | 0～2 | 模式序号：0=示教模式，1=远程模式，2=运行模式 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_current_mode tl_ros2_interface/srv/SetCurrentMode "{mode: 1}"
```
### 12.2 查询当前运行模式
| 功能描述 | 查询控制器当前运行模式 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_current_mode` |
| 服务类型 | `tl_ros2_interface/srv/GetCurrentMode` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 范围 | 说明 |
|--------|------|------|------|
| success | bool | — | `true` = 查询成功，`false` = 查询失败 |
| message | string | — | 失败时包含错误描述 |
| mode | int32 | 0～2 | 当前模式：0=示教，1=远程，2=运行 |
#### 命令示例
```
ros2 service call /tl_driver/get_current_mode tl_ros2_interface/srv/GetCurrentMode "{}"
```
### 12.3 设置数字输出
| 功能描述 | 设置指定数字输出端口的高低电平状态 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_digital_output` |
| 服务类型 | `tl_ros2_interface/srv/SetDigitalOutput` |

**输入参数**
| 参数名 | 类型 | 范围 | 说明 |
|--------|------|------|------|
| port | int32 | — | 数字输出端口号 |
| value | int32 | 0 或 1 | 输出状态：0=低电平，1=高电平 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_digital_output tl_ros2_interface/srv/SetDigitalOutput "{port: 1, value: 1}"
```
### 12.4 查询数字输入输出状态
| 功能描述 | 查询所有数字输入（DI）和数字输出（DO）端口状态 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_digital_input_output` |
| 服务类型 | `tl_ros2_interface/srv/GetDigitalInputOutput` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 失败时包含错误描述 |
| input | int32[] | 各数字输入端口状态（0=低电平，1=高电平） |
| output | int32[] | 各数字输出端口状态（0=低电平，1=高电平） |
#### 命令示例
```
ros2 service call /tl_driver/get_digital_input_output tl_ros2_interface/srv/GetDigitalInputOutput "{}"
```
## 13 modbus通信接口
### 13.1 写Modbus
| 功能描述 | 通过Modbus协议向从站批量写入寄存器数据（功能码16，0x10） |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/modbus_write` |
| 服务类型 | `tl_ros2_interface/srv/ModbusWrite` |

**输入参数**
| 参数名 | 类型 | 范围 | 说明 |
|--------|------|------|------|
| master_id | int32 | 0～8 | 主站通道编号 |
| addr | int32 | — | 起始寄存器地址 |
| data | int32[] | — | 待写入的16位整型数据列表 |
| master_param.type | string | — | 通信协议：`"RTU"` 或 `"TCP"` |
| master_param.start_addr | bool | — | `true` = 从起始地址开始 |
| rtu.slave_id | int32 | — | 从站ID（RTU模式） |
| rtu.port | int32 | — | 控制器物理串口号（RTU模式） |
| rtu.baudrate | int32 | — | 波特率（常见值：9600, 115200） |
| rtu.data_bit | int32 | 7 或 8 | 数据位（通常为8） |
| rtu.stop_bit | int32 | 1 或 2 | 停止位（通常为1） |
| rtu.check_bit | string | — | 校验方式：`"None"`/`"Odd"`/`"Even"`/`"N"`/`"O"`/`"E"` |
| tcp.ip | string | — | 目标IP地址（TCP模式） |
| tcp.port | int32 | — | 目标端口号（TCP模式） |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 写入成功，`false` = 写入失败（从站无响应、地址越界等） |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/modbus_write tl_ros2_interface/srv/ModbusWrite \
"{
    master_id: 1,
    addr: 1135,
    data: [0, 0, 0, 0, 0],
    master_param:
    {
        type: 'RTU',
        start_addr: true,
        rtu:
        {
            slave_id: 2,
            port: 2,
            baudrate: 115200,
            data_bit: 8,
            stop_bit: 1,
            check_bit: 'N'
        }
    }
}"
```
### 13.2 读Modbus
| 功能描述 | 通过Modbus协议从从站读取保持寄存器数据（功能码03） |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/modbus_read` |
| 服务类型 | `tl_ros2_interface/srv/ModbusRead` |

**输入参数**
| 参数名 | 类型 | 范围 | 说明 |
|--------|------|------|------|
| master_id | int32 | 0～8 | 主站通道编号 |
| addr | int32 | — | 起始寄存器地址 |
| quantity | int32 | ≥1 | 要读取的寄存器数量 |
| master_param | ModbusMasterParam | — | 主站参数（参见 [写Modbus](#131-写modbus) 的参数说明） |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 读取成功，`false` = 读取失败 |
| message | string | 失败时包含错误描述 |
| data | int32[] | 读取到的寄存器数据 |
#### 命令示例
```
ros2 service call /tl_driver/modbus_read tl_ros2_interface/srv/ModbusRead \
"{
    master_id: 1,
    addr: 1135,
    quantity: 5,
    master_param:
    {
        type: 'RTU',
        start_addr: true,
        rtu:
        {
            slave_id: 2,
            port: 2,
            baudrate: 115200,
            data_bit: 8,
            stop_bit: 1,
            check_bit: 'N'
        }
    }
}"
```
## 14 队列运动接口
### 14.1 设置MoveJ队列运动模式
| 功能描述 | 开启或关闭队列运动模式，开启后运动指令将加入缓冲队列依次执行 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/queue_motion_set_status` |
| 服务类型 | `tl_ros2_interface/srv/QueueMotionSetStatus` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| status | bool | `true` = 打开队列模式，`false` = 关闭 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
> **注意**：`status: false` 时驱动还会切换到示教模式并下电；队列状态已设置但这两步失败时，`success` 也返回 `false`，`message` 指明具体失败步骤。
#### 命令示例
```
ros2 service call /tl_driver/queue_motion_set_status tl_ros2_interface/srv/QueueMotionSetStatus "{status: true}"
```
### 14.2 MoveJ队列运动
| 功能描述 | 向队列中添加一条MoveJ运动指令并下发执行 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/queue_motion_movej` |
| 服务类型 | `tl_ros2_interface/srv/QueueMotionMoveJ` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| is_continue | bool | `false` = 新队列运动，`true` = 从断点继续执行 |
| cmd | MoveCommand | 运动指令（参见 [MoveCommand消息类型说明](#movecommand消息类型说明)） |

> **速度口径**：队列接口的 `velocity` 与 `MoveCommand` 同口径（MOVJ 为 %、MOVL 为 m/s）。SDK 队列头注释写作「度/s」有误，不要照抄。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 运动成功，`false` = 运动失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/queue_motion_movej tl_ros2_interface/srv/QueueMotionMoveJ \
"{
    is_continue: false,
    cmd:
    {
        target_pos_value: [0.0, 0.0, 0.1745, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        velocity: 20.0,
        acc: 20.0,
        dec: 20.0
    }
}"
```
### 14.3 停止MoveJ队列运动模式
| 功能描述 | 停止队列运动，清空缓冲队列 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/queue_motion_stop` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 停止成功，`false` = 停止失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/queue_motion_stop std_srvs/srv/Trigger "{}"
```
## 15 7000端口
### 15.1 打开关节跟踪模式
| 功能描述 | 激活高速流式伺服跟踪模式（ServoJ），需提供各轴最大速度/加速度/加加速度限制 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/open_servoj` |
| 服务类型 | `tl_ros2_interface/srv/OpenServoJ` |

**输入参数**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| vmax | float64[] | rad/s | 各轴最大速度，**固定 7 元素**（6 轴末位补 0；7 轴第 7 位为真实 J7） |
| amax | float64[] | rad/s² | 各轴最大加速度，**固定 7 元素** |
| jmax | float64[] | rad/s³ | 各轴最大加加速度，**固定 7 元素** |

> **注意**：三个约束向量必须固定 7 元素。传 6 元素会导致控制器协议解析错乱并断开 7000 端口连接（SDK 警告）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 打开成功，`false` = 打开失败 |
| message | string | 失败时包含错误描述 |

> **注意**：该模式只能在连接端口7000后使用。使用前建议先通过 [设置运行速度](#设置运行速度) 增大机械臂运行速度，如果运行速度太小会出现关节不动或运行缓慢的情况。
#### 命令示例
```
ros2 service call /tl_driver/open_servoj tl_ros2_interface/srv/OpenServoJ \
"{
    vmax: [5.236, 5.236, 5.236, 5.236, 5.236, 5.236, 5.236],
    amax: [52.360, 52.360, 52.360, 52.360, 52.360, 52.360, 52.360],
    jmax: [872.665, 872.665, 872.665, 872.665, 872.665, 872.665, 872.665]
}"
```
* 注意: 该模式只能在连接端口7000后使用。在使用该功能前最好先增加机械臂运行速度，如果运行速度太小的话会出现关节不动或者关节运行较慢的情况。
### 15.2 关闭关节跟踪模式
| 功能描述 | 退出伺服跟踪模式，恢复常规轨迹规划 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/close_servoj` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 关闭成功，`false` = 关闭失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/close_servoj std_srvs/srv/Trigger "{}"
```
### 15.3 发送跟踪关节位置
| 功能描述 | 在跟踪模式下实时发送目标关节角度，需以固定周期持续调用（如10ms或20ms） |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/tl_driver/set_servoj_pos` |
| 消息类型 | `std_msgs::msg::Float64MultiArray` |

**输入参数**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| data | float64[] | rad | 目标关节角度数组，**固定 7 元素**：6 轴机器人在第 7 元素补 0；7 轴机器人第 7 元素为真实 J7 关节角 |

**输出/返回值**
无返回值（话题发布单向通信）。

> **注意**：该话题仅在 [打开关节跟踪模式](#打开关节跟踪模式) 后有效。需以固定周期持续调用以维持跟踪。
#### 命令示例
```
ros2 topic pub /tl_driver/set_servoj_pos std_msgs/msg/Float64MultiArray \
"{
    layout:
    {
        dim: [],
        data_offset: 0
    },
    data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.349, 0.0]
}"
```

## 16 状态诊断话题与 Action

### 16.1 状态流与回滚参数

驱动连接成功后，在 7000 端口启动两个连续反馈通道，回调只解析并写快照；`/joint_states`、`/tcp_pose` 由帧通知唤醒的独立发布线程直接发布，不再经过 100 Hz 定时器，因此不受 6001 端口同步查询阻塞。`/arm_status` 由 10 Hz 查询缓存、100 Hz 定时器只发布缓存值。

| 通道 | 请求 | 数据源 | 用途 |
| :---- | :---- | :---- | :---- |
| 1 | `mode=1, position=0, interval=10` | `realPosACS`（度） | `/joint_states` |
| 2 | `mode=1, position=1, interval=10` | `realPosMCS`（位置 mm、姿态度） | `/tcp_pose`、action feedback |

| 参数 | 默认 | 范围 | 说明 |
| :---- | ----: | :---- | :---- |
| `state_source` | `stream` | `stream` / `poll` | `poll` 回退旧的同步查询路径（启动时生效） |
| `state_stream_interval_ms` | 10 | 10–60000 | 连续反馈间隔 |
| `joint_dynamics_rate` | 10.0 | 0–100 | `/tl_driver/joint_dynamics` 频率，0 关闭 |
| `joint_thermal_rate` | 1.0 | 0–1 | `/tl_driver/joint_thermal` 频率，0 关闭 |
| `io_state_rate` | 10.0 | 0–100 | `/tl_driver/io_state` 频率，0 关闭 |
| `motion_action_timeout` | 60.0 | (0, 3600] | 运动 action 默认超时（秒） |
| `track_action_timeout` | 600.0 | (0, 3600] | 回放 action 默认超时（秒） |

> 两个通道独立降级：某通道连续失效 3 s 后只有该通道改用同步查询（20 Hz），恢复连续 1 s 后切回状态流，另一通道不受影响。1 Hz 看门狗在通道超过 3×`state_stream_interval_ms` 无新帧时重发 `mode=1`（前 3 s 每秒一次，之后每 5 s 一次），3 次仍无新帧记录故障告警。`state_source=poll` 时两个通道都走同步查询，发布频率为 20 Hz。运行状态（`/arm_status`）由独立 10 Hz 定时器查询并缓存，100 Hz 定时器只发布缓存，查询不再进入 100 Hz 路径。

### 16.2 诊断话题

| 话题 | 类型 | 默认频率 | 内容 |
| :---- | :---- | ----: | :---- |
| `/tl_driver/joint_dynamics` | `tl_ros2_interface/msg/JointDynamics` | 10 Hz | 线速度 m/s、关节速度 rad/s、电机电流 A、力矩 ‰ 额定扭矩（千分之一，不是 %） |
| `/tl_driver/joint_thermal` | `tl_ros2_interface/msg/JointThermal` | 1 Hz | 关节温度 °C、关节/外部轴电压 V |
| `/tl_driver/io_state` | `tl_ros2_interface/msg/IoState` | 10 Hz | 数字输入/输出数组 |

> 三个话题均为 `reliable` + `transient_local` + depth 1。`joint_thermal` 单次查询约 150–350 ms，频率上限 1 Hz；每次发布单独占用热状态回调组。/joint_states 与 /tcp_pose 已由独立发布线程发布，热状态查询不再阻塞它们；`/arm_status` 的 100 Hz 发布只读 10 Hz 查询缓存。原有查询服务全部保留。

### 16.3 运动 Action

| Action | 类型 | 说明 |
| :---- | :---- | :---- |
| `/tl_driver/move_joint` | `tl_ros2_interface/action/Move` | 关节运动（仅 `coord=0`，关节 rad） |
| `/tl_driver/move_linear` | `tl_ros2_interface/action/Move` | 直线运动（仅 `coord=1`，位置 m、姿态 rad） |
| `/tl_driver/play_track` | `tl_ros2_interface/action/PlayTrack` | 轨迹回放（`vel` 1–100 %） |

**Move goal**
| 字段 | 类型 | 说明 |
| :---- | :---- | :---- |
| command | `MoveCommand` | 与话题同结构；`target_pos_value` 必须为 14 位，`target_pos_type` 必须为 0 |
| timeout_sec | float64 | 超时秒数；<=0 时使用 `motion_action_timeout` |

**Move feedback / result**
| 字段 | 类型 | 说明 |
| :---- | :---- | :---- |
| progress | float64 | 0.0–1.0；位姿数据不可用时为 -1 |
| current_pose | `CartesianPose` | MCS 状态流换算后的当前位姿；无流时为零值 |
| state | string | `RUNNING` / `STOP` / `PAUSE` |
| success / message | bool / string | result 中的最终结果与说明 |

**PlayTrack goal / feedback / result**
| 字段 | 类型 | 说明 |
| :---- | :---- | :---- |
| vel | int32 | 回放速度百分比 1–100 |
| timeout_sec | float64 | 超时秒数；<=0 时使用 `track_action_timeout` |
| progress | float64 | 控制器不提供进度，固定 -1 |
| state | string | `RUNNING` / `STOP` / `PAUSE` |
| success / message | bool / string | result 中的最终结果与说明 |

> **cancel 不支持**：控制器 SDK 没有通用运动停止接口（仅有 `robot_stop_jogging`、`queue_motion_stop_not_power_off`、`job_stop`、`track_record_stop` 录制停止），因此三个 action 的 cancel 请求一律返回 `REJECT`，不会出现“取消成功但机械臂仍在运动”。同一时刻只允许一个运动 action goal；已有 goal 运行或 `run_state` 非 `STOP`（含 `PAUSE`）时新 goal 被拒绝。action 执行期间，`moveJ`/`moveL` 话题、`set_servoj_pos`、`track_playback`、队列运动与 `open_servoj` 会被拒绝/忽略并记录告警。现有 `moveJ`/`moveL` 话题与 `track_playback` 服务保持不变。刚通过 `moveJ`/`moveL` 话题成功下发运动后 500 ms 内到达的 action goal 会被拒绝，避免控制器 `run_state` 更新延迟导致并发运动。完成判定对 `coord=0` 用 ACS、`coord=1` 用 MCS 校验目标，到达容差为 1° / 2 mm；`coord=2/3` 的 action goal 会被拒绝。

### 16.4 运动 Action 测试命令

前置条件：

- `tl_driver` 已启动并连接控制器，`/arm_status` 为 `STOP`，机械臂已上电且工作区安全。
- 以下示例以 6 轴零位为基准：`/tcp_pose` = `(0.2793, 0, 0.4216)` m，rpy = `(3.1414, 0, 0)` rad。换用其他初始位姿前，先读当前位姿并替换 `target_pos_value`。

```bash
ros2 topic echo /tcp_pose --once
ros2 topic echo /arm_status --once
```

#### MoveJ：J6 转 10°

10° = 0.174533 rad。`target_pos_value` 必须为 14 位：前 7 位是 J1–J7（rad），后 7 位是外部轴。

```bash
ros2 action send_goal /tl_driver/move_joint tl_ros2_interface/action/Move \
"{command: {
  target_pos_value: [0.0, 0.0, 0.0, 0.0, 0.0, 0.174533, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  target_pos_name: '',
  target_pos_type: 0,
  coord: 0,
  velocity: 20.0,
  velocity_sync: 0.0,
  acc: 20.0,
  dec: 20.0,
  pl: 0,
  time: 0,
  tool_num: 0,
  user_num: 0,
  posidtype: 0,
  configuration: 0,
  spin: 0,
  para_sync: false
}, timeout_sec: 10.0}" --feedback
```

回零：

```bash
ros2 action send_goal /tl_driver/move_joint tl_ros2_interface/action/Move \
"{command: {
  target_pos_value: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  target_pos_name: '', target_pos_type: 0, coord: 0,
  velocity: 20.0, velocity_sync: 0.0, acc: 20.0, dec: 20.0,
  pl: 0, time: 0, tool_num: 0, user_num: 0,
  posidtype: 0, configuration: 0, spin: 0, para_sync: false
}, timeout_sec: 10.0}" --feedback
```

#### MoveL：沿 +Z 平移 5 cm

基准位姿 z = 0.4216 m，目标 z = 0.4716 m。`coord=1`，位置 m、姿态 rad；`velocity` 单位为 m/s。

```bash
ros2 action send_goal /tl_driver/move_linear tl_ros2_interface/action/Move \
"{command: {
  target_pos_value: [0.2793, 0.0, 0.4716, 3.1414, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  target_pos_name: '',
  target_pos_type: 0,
  coord: 1,
  velocity: 0.01,
  velocity_sync: 0.0,
  acc: 20.0,
  dec: 20.0,
  pl: 0,
  time: 0,
  tool_num: 0,
  user_num: 0,
  posidtype: 0,
  configuration: 0,
  spin: 0,
  para_sync: false
}, timeout_sec: 10.0}" --feedback
```

回原位：

```bash
ros2 action send_goal /tl_driver/move_linear tl_ros2_interface/action/Move \
"{command: {
  target_pos_value: [0.2793, 0.0, 0.4216, 3.1414, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  target_pos_name: '', target_pos_type: 0, coord: 1,
  velocity: 0.01, velocity_sync: 0.0, acc: 20.0, dec: 20.0,
  pl: 0, time: 0, tool_num: 0, user_num: 0,
  posidtype: 0, configuration: 0, spin: 0, para_sync: false
}, timeout_sec: 10.0}" --feedback
```

> `move_joint` 的 `velocity` 是百分比，`move_linear` 的 `velocity` 是 m/s；`target_pos_type` 必须为 0，数组长度不是 14 时 goal 会被直接拒绝。两个 action 的 cancel 都会被拒绝，测试时如遇异常直接拍急停。
