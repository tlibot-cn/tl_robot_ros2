<div align="center">

# tl_driver接口说明书

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
|V1.16 | 2026-10-22 | 查询类接口话题化：删除 `get_speed`、`get_robot_state`、`get_joint_temperature`、`get_joint_voltage`、`get_motor_current`、`get_joint_software_version`、`get_current_motor_torque`、`get_current_line_joint_speed`；删除 `io_state` 话题（数字 IO 读写统一走服务）；删除 `set_robot_joint_param`；力矩单位统一为额定扭矩千分之一（‰）|
|V1.17 | 2026-10-23 | 文档更名《tl_driver接口说明书》：第 16 章解散，状态流与回滚参数并入 15.4、诊断话题说明并入 4.8/4.10、运动 Action 并入第 7 章 |
|V1.18 | 2026-10-23 | 第 7 章由「实时运动控制接口」更名为「运动控制接口」；删除 `/tl_driver/moveJ`、`/tl_driver/moveL` 话题及 7.1/7.2 章节，运动 Action 章节改为 7.1/7.2 |
|V1.19 | 2026-10-23 | `MoveCommand` 裁剪 6 个未使用字段并改定长 14 维；`JointDynamics`/`JointThermal` 定长数组 + NaN 语义（力矩改 float64）；删除 `state_source=poll` 与 `publish_rate`，`/arm_status` 随 10 Hz 查询发布；新增 `executor_threads`；补充接口矩阵与移除清单（第 16 章） |
|V1.20 | 2026-10-23 | 删除 `/tl_driver/track_playback` 服务与 `TrackPlayback.srv`，轨迹回放统一走 `/tl_driver/play_track` action（服务数 36 → 35） |
|V1.21 | 2026-10-23 | 删除 1 Hz 状态流看门狗；状态流改在上下电完成后启动，启动失败即退出（回调组 8 → 7） |
|V1.22 | 2026-10-29 | `ArmStatus` 去掉 `speed`/`speed_valid`，恢复 `get_speed` 服务；`JointDynamics`/`JointThermal` 合并为 `JointMonitor`（默认 10 Hz，温度/电压按 `joint_thermal_rate` 刷新；删除外部轴电流/力矩与外部轴电压）；新增 `fk`/`ik` 正逆解服务 |
|V1.23 | 2026-10-29 | 诊断话题按查询拆分：力矩拆到 `/tl_driver/joint_torque`（`JointTorque`），线速度拆到 `/tl_driver/line_speed`（`LineSpeed`），关节速度保留在 `/tl_driver/joint_speed`（`JointSpeed`，删除 `joint_speed_sync`）；新增 `joint_speed_rate`、`joint_torque_rate`；`/tl_driver/joint_monitor` 只保留电流/温度/电压，默认 2 Hz，电流、温度、电压在同一回调内查询且不缓存；删除 `joint_thermal_rate` 与独立热查询定时器 |
|V1.24 | 2026-10-30 | `/arm_status` 话题改为按需服务 `get_arm_status`（`GetArmStatus.srv`），删除 10 Hz 状态查询定时器与 `ArmStatus.msg`；回调组 7 → 6 |
|V1.25 | 2026-11-02 | 新增 `/tl_driver/error_info`（`ErrorInfo`，控制器错误/告警异步推送，事件驱动不节流）；`/tl_driver/joint_speed`、`/tl_driver/line_speed`、`/tl_driver/joint_torque` 改为在同一个定时器内先后发布；参数 `joint_speed_rate`、`joint_torque_rate` 合并为 `feedback_rate`（默认 100 Hz） |
|V1.26 | 2026-11-03 | 新增 9.7 [设置示教模式类型](#97-设置示教模式类型)、9.8 [查询示教模式类型](#98-查询示教模式类型) 服务（`SetTeachType.srv` / `GetTeachType.srv`）；开启力引导拖拽需 `set_drag_mode(1)` + `set_teach_type(1)` 两步且顺序不可颠倒；服务数 39 → 41 |
|V1.27 | 2026-11-03 | 新增 15.5–15.10：周期同步速度模式（CSV）与周期同步扭矩模式（CST）的开关服务与下发话题（服务数 41 → 45，订阅 1 → 3）；两个话题采用 `Float64MultiArray`，速度 rad/s、扭矩 ‰ 额定扭矩；力矩补偿尚未实现 |
|V1.28 | 2026-11-03 | `fk`/`ik` 新增 `source`（0=控制器 1=本地模型）；`reference_pos` 改为必填；`CartesianPose.arm_angle` 明确为冗余臂角 ψ；启动时校验控制器机型轴数与 `arm_joints`，不一致即退出；删除 `set_axis_zero_pos` 服务（服务数 45 → 44） |

</div>

## 目录

> 编号说明：章号与小节号沿用历史发布，删除的接口不再复用编号（2.1/2.2、3、4.12/4.16、5.3/5.4、11 等空缺即为已下线项），以免破坏外部既有引用。完整的接口总表与移除清单见 [16 接口矩阵与移除清单](#16-接口矩阵与移除清单)。

* [1 通用约定](#1-通用约定)
  * [1.5 接口条目与 QoS 约定](#15-接口条目与-qos-约定)
  * [1.6 启动校验与本地运动学](#16-启动校验与本地运动学)
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
  * [4.21 正运动学](#421-正运动学)
  * [4.22 逆运动学](#422-逆运动学)
  * [4.23 控制器错误与告警](#423-控制器错误与告警)
* [5 机械臂基础功能设置接口](#5-机械臂基础功能设置接口)
  * [5.1 设置运行速度](#51-设置运行速度)
  * [5.2 设置控制器有线网口IP](#52-设置控制器有线网口ip)
  * [5.5 设置机械臂默认笛卡尔参数](#55-设置机械臂默认笛卡尔参数)
  * [5.6 坐标转换](#56-坐标转换)
* [6 MoveCommand 消息类型说明](#6-movecommand-消息类型说明)
* [7 运动控制接口](#7-运动控制接口)
  * [7.1 运动 Action](#71-运动-action)
  * [7.2 运动 Action 测试命令](#72-运动-action-测试命令)
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
  * [9.7 设置示教模式类型](#97-设置示教模式类型)
  * [9.8 查询示教模式类型](#98-查询示教模式类型)
* [10 错误处理接口](#10-错误处理接口)
  * [10.1 清除错误](#101-清除错误)
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
* [15 7000端口](#15-7000端口)
  * [15.1 打开关节跟踪模式](#151-打开关节跟踪模式)
  * [15.2 关闭关节跟踪模式](#152-关闭关节跟踪模式)
  * [15.3 发送跟踪关节位置](#153-发送跟踪关节位置)
  * [15.4 状态流参数](#154-状态流参数)
  * [15.5 打开周期同步速度模式](#155-打开周期同步速度模式)
  * [15.6 关闭周期同步速度模式](#156-关闭周期同步速度模式)
  * [15.7 发送周期同步速度](#157-发送周期同步速度)
  * [15.8 打开周期同步扭矩模式](#158-打开周期同步扭矩模式)
  * [15.9 关闭周期同步扭矩模式](#159-关闭周期同步扭矩模式)
  * [15.10 发送周期同步扭矩](#1510-发送周期同步扭矩)
* [16 接口矩阵与移除清单](#16-接口矩阵与移除清单)
  * [16.1 接口总矩阵](#161-接口总矩阵)
  * [16.2 已移除接口](#162-已移除接口)


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
| 姿态 / 欧拉角 | rad | XYZ 外旋（scipy `xyz`） |
| 线速度 | m/s | MOVL 指令、`/tl_driver/line_speed.line_speed` |
| 关节速度 | rad/s | `/tl_driver/joint_speed.joint_speed`、`RobotJointParam.*_rot_speed` |
| 角加速度 / 加加速度 | rad/s² / rad/s³ | ServoJ 约束、`RobotJointParam.max_acc` / `max_dec` |
| 百分比量 | % | 速度、加速度（额定值百分比，取值见各接口） |
| 电机力矩 | ‰（额定扭矩千分之一） | `JointTorque.motor_torque`，取值 ±1000 对应 ±100% 额定扭矩；不是 N·m，也不是 % |
| 温度 / 电压 / 电流 | ℃ / V / A | SDK 头未标注温度、电压；电流 SDK 头标 ‰，与本仓 A 口径不符 |

- `MoveCommand.velocity`：MOVJ 为 %（0<v≤100）；MOVL 为 m/s（0<v≤1）。
- `target_pos_value` 为定长 14 维，布局与单位换算规则见 `MoveCommand.msg` 文件头；点位类型固定为 `PosType::data`，消息中不再暴露 `target_pos_type`。
- `/tl_driver/set_servoj_pos` 的 `data` 为 rad，**固定 7 元素**（6 轴末位补 0）。

### 1.5 接口条目与 QoS 约定

每个接口条目按固定顺序描述：名称与方向（话题/服务/action）、频率（静态话题标注，服务为事件驱动）、QoS、字段与单位、成功/失败语义、替代或移除关系、可复制命令。QoS 统一如下，条目内不再重复：

| 类别 | QoS |
| :---- | :---- |
| 服务与 action | `rmw_qos_profile_services_default` |
| `/joint_states`、`/tcp_pose` | `KeepLast(10)`，reliable |
| `/tl_driver/joint_monitor`、`/tl_driver/joint_torque`、`/tl_driver/joint_speed`、`/tl_driver/line_speed` | `KeepLast(1)`，reliable，`transient_local` |
| `/tl_driver/error_info` | `KeepLast(100)`，reliable（事件逐条发布，不用 `transient_local`） |

> 服务失败统一返回 `success=false` + `message`，不抛异常；话题无帧时停止发布，不重复发送过期值。

### 1.6 启动校验与本地运动学

- **机型轴数校验**：驱动启动时查询控制器机型，与 `arm_joints` 长度比对；两者不一致直接退出（`EXIT_FAILURE`）。机型查询失败或机型不是串联系列 6/7 轴时仅告警，不阻断启动。
- **本地运动学模型**：`fk`/`ik` 的 `source=1` 使用本地模型，启动时由控制器回读的 DH、关节限位与当前工具手构建**一次**；运行期修改 DH、零点、限位或工具后不重建，需要重启驱动。构建失败不影响驱动启动，仅使 `source=1` 请求返回失败原因。
- 本地模型只支持基坐标系（`frame=1`）与 6/7 轴；`ik` 的 `reference_pos` 必填。

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

> **注意**：上电前需确保驱动已完成启动连接（`get_arm_status` 服务可正常返回），且无未清除的错误报警。若机械臂已处于上电状态，再次调用会返回成功。
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

**请求参数**
无。

**响应参数**
| 参数名 | 类型 | 单位 | 范围 | 说明 |
|--------|------|------|------|------|
| success | bool | — | — | 查询是否成功 |
| message | string | — | — | 成功/失败描述 |
| speed | float64 | % | 0～100 | 当前全局运行速度 |

> 运行速度不再随运行状态周期发布；本接口事件驱动，每次调用直接查询控制器。
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
| 功能描述 | 通过 `get_arm_status` 服务与 `/joint_states`、`/tcp_pose` 话题获取机械臂当前状态 |
| :---: | :---- |
| 通信机制 | ROS2服务 + 话题 |
| 接口名 | `/tl_driver/get_arm_status`（运行状态）、`/joint_states`（关节）、`/tcp_pose`（末端位姿） |
| 类型 | `tl_ros2_interface/srv/GetArmStatus`、`sensor_msgs/msg/JointState`、`tl_ros2_interface/msg/CartesianPose` |

**输入参数**
`get_arm_status` 无请求参数；两个话题无需请求参数。

**输出/返回值**
| 接口 | 字段 | 说明 |
|--------|------|------|
| `get_arm_status` | `success` | 查询是否成功 |
| `get_arm_status` | `message` | 成功/失败描述 |
| `get_arm_status` | `run_state` | `"STOP"` / `"PAUSE"` / `"RUNNING"` / `"UNKNOWN"` |
| `/joint_states` | `position` | 关节角 rad（6 轴只发 6 个） |
| `/tcp_pose` | `position` / `rpy` | 末端位姿 m / rad |

> 服务 `/tl_driver/get_robot_state` 已删除：它只返回内部缓存报文，话题与服务已完整覆盖；驱动不再对外暴露 ACS/MCS 原始 JSON。
#### 命令示例
```
ros2 service call /tl_driver/get_arm_status tl_ros2_interface/srv/GetArmStatus "{}"
ros2 topic echo /joint_states --once
ros2 topic echo /tcp_pose --once
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
| 功能描述 | 查询各关节电机当前温度（随 `/tl_driver/joint_monitor` 发布） |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/tl_driver/joint_monitor` |
| 消息类型 | `tl_ros2_interface/msg/JointMonitor` |

**输入参数**
无（话题订阅，无需请求参数）。

**输出/返回值**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| temperature | float64[7] | ℃ | 各关节电机当前温度，定长 7；缺失元素为 NaN |
| joint_voltage | float64[7] | V | 机械臂本体各关节电压，定长 7；缺失元素为 NaN |

> 数组定长，查询失败或数据不足时缺失元素为 `quiet_NaN()`，消费方用 `std::isnan()` 判定，不再用空数组表示失败。

> 电流、温度、电压在同一监测回调内顺序查询，按 `joint_monitor_rate`（默认 2 Hz，取 0 关闭）发布；任一子查询失败时对应字段整段 NaN，不跨帧复用旧值。温度/电压查询较慢（单次约 150–350 ms），因此监测频率上限为 2 Hz。服务 `/tl_driver/get_joint_temperature`、`/tl_driver/get_joint_voltage` 已删除。
#### 命令示例
```
ros2 topic echo /tl_driver/joint_monitor --field temperature --once
```
### 4.9 查询关节电压
| 功能描述 | 查询各关节当前电压（与 4.8 同一话题、同一监测回调） |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/tl_driver/joint_monitor` |
| 消息类型 | `tl_ros2_interface/msg/JointMonitor` |

**输出/返回值**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| joint_voltage | float64[7] | V | 机械臂本体各关节电压，定长 7；缺失元素为 NaN |

> 外部轴电压（`positioner_voltage`）本版本不再发布。

#### 命令示例
```
ros2 topic echo /tl_driver/joint_monitor --field joint_voltage
```
### 4.10 查询电机电流
| 功能描述 | 查询机器人本体 7 轴电机电流（随 `/tl_driver/joint_monitor` 发布） |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/tl_driver/joint_monitor` |
| 消息类型 | `tl_ros2_interface/msg/JointMonitor` |

**输入参数**
无（话题订阅，无需请求参数）。

**输出/返回值**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| motor_current | float64[7] | A | 机器人本体 7 轴电机电流；缺失元素为 NaN |

> 监测话题默认 2 Hz（`joint_monitor_rate`，取 0 关闭；范围 0–2），`reliable` + `transient_local` + depth 1；电流、温度、电压在同一回调内顺序查询，不跨帧缓存。本话题只保留这三项，力矩见 [4.19](#419-查询当前电机力矩)，速度见 [4.20](#420-查询当前线速度和关节速度)。外部轴电流（`motor_current_sync`）本版本不再发布。服务 `/tl_driver/get_motor_current` 已删除；数字 IO 不发布话题，读写分别使用 `get_digital_input_output` / `set_digital_output` 服务。
#### 命令示例
```
ros2 topic echo /tl_driver/joint_monitor --field motor_current --once
```
### 4.11 查询关节软件版本号
| 功能描述 | 驱动启动时逐轴查询并打印关节软件版本 |
| :---: | :---- |
| 通信机制 | ROS2 日志 |
| 输出位置 | `[Init]: joint N software version: ...` |

**说明**
连接并上电成功后，`DriverNode::init()` 对 1..NDOF 逐轴调用 `tl::query_joint_software_version`，成功打 `INFO`，失败打 `WARN` 并继续启动；不再提供 `/tl_driver/get_joint_software_version` 服务。

#### 命令示例
```
ros2 launch tl_bringup tl_tcb610_bringup.launch.py 2>&1 | grep 'software version'
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
| 功能描述 | 按需查询机械臂当前运行状态（停止/暂停/运行中） |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_arm_status` |
| 服务类型 | `tl_ros2_interface/srv/GetArmStatus` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 成功/失败描述 |
| run_state | string | 运行状态：`"STOP"` = 停止，`"PAUSE"` = 暂停，`"RUNNING"` = 运行中，`"UNKNOWN"` = SDK 返回未识别值 |

> 事件驱动，每次调用直接查询控制器，不周期发布话题；查询失败返回 `success=false`。运行速度改由 [4.3](#43-查询运行速度) 的 `get_speed` 服务查询。
#### 命令示例
```
ros2 service call /tl_driver/get_arm_status tl_ros2_interface/srv/GetArmStatus "{}"
```

### 4.19 查询当前电机力矩
| 功能描述 | 查询机械臂当前各电机力矩（话题 `/tl_driver/joint_torque`） |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/tl_driver/joint_torque` |
| 消息类型 | `tl_ros2_interface/msg/JointTorque` |

**输入参数**
无（话题订阅，无需请求参数）。

**输出/返回值**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| motor_torque | float64[7] | ‰ 额定扭矩 | 机械臂本体当前各电机力矩（千分之一额定扭矩，定长 7）；缺失元素为 NaN |

> **单位**：力矩是额定扭矩的千分之一（‰），±1000 对应 ±100% 额定扭矩；不是 N·m，也不是百分比。外部轴力矩（`motor_torque_sync`）本版本不再发布。默认 100 Hz（`feedback_rate`，与 4.20 的速度话题共用同一个定时器，各自独立查询与时间戳，取 0 关闭），`reliable` + `transient_local` + depth 1；查询失败时该周期不发布。服务 `/tl_driver/get_current_motor_torque` 已删除。
#### 命令示例
```
ros2 topic echo /tl_driver/joint_torque --once
```
### 4.20 查询当前线速度和关节速度
| 功能描述 | 查询机械臂当前末端的线速度（`/tl_driver/line_speed`）和各关节角速度（`/tl_driver/joint_speed`） |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/tl_driver/line_speed`、`/tl_driver/joint_speed` |
| 消息类型 | `tl_ros2_interface/msg/LineSpeed`、`tl_ros2_interface/msg/JointSpeed` |

**输入参数**
无（话题订阅，无需请求参数）。

**输出/返回值**
`LineSpeed`：

| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| line_speed | float64 | m/s | 当前末端线速度；查询失败为 NaN |

`JointSpeed`：

| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| joint_speed | float64[7] | rad/s | 机械臂本体当前各关节速度，定长 7；缺失元素为 NaN |

> 两个话题由同一次 SDK 查询产生，共用 `feedback_rate`（默认 100 Hz，取 0 关闭；与 4.19 的力矩话题共用同一个定时器，各自独立查询与时间戳）；外部轴同步速度（`joint_speed_sync`）本版本不再发布。查询失败时该周期两个话题都不发布。服务 `/tl_driver/get_current_line_joint_speed` 已删除。
#### 命令示例
```
ros2 topic echo /tl_driver/line_speed --once
ros2 topic echo /tl_driver/joint_speed --once
```

### 4.21 正运动学
| 功能描述 | 由关节角解算指定坐标系下的 TCP 位姿（正解） |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/fk` |
| 服务类型 | `tl_ros2_interface/srv/Fk` |

**请求参数**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| frame | int32 | — | 输出位姿坐标系：1=基坐标 2=工具 3=用户；0=关节，不支持 |
| joints | float64[] | rad | 关节角，6 或 7 元素（不少于本体轴数；6 轴第 7 位为外部轴，本地模型忽略） |
| source | int32 | — | 求解来源：0=控制器（默认）1=本地模型（仅 `frame=1`） |

**响应参数**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| success | bool | — | 解算是否成功 |
| message | string | — | 成功/失败描述 |
| pose | CartesianPose | m + rad | TCP 位姿；`arm_angle` 为冗余臂角 ψ（`source=1` 的 7 轴由本地解算给出；6 轴为 0；7 轴 ψ 无定义时为 NaN），`header.frame_id` 为 `base_link` / `tool0` / `user_coord` |

> 输入 `joints` 由驱动换算为度后交 SDK；输出位置由 mm 换算为 m，姿态保持 rad。逆解失败（奇异点/不可达）等错误通过 `success=false` 与 `message` 返回。
> `source=1` 的本地模型在驱动启动时由控制器回读的 DH、关节限位与当前工具手构建一次，运行期不重建；模型不可用或 `frame≠1` 时直接返回失败，不回退控制器。
#### 命令示例
```
ros2 service call /tl_driver/fk tl_ros2_interface/srv/Fk "{
  frame: 1,
  joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  source: 0
}"
```

### 4.22 逆运动学
| 功能描述 | 由指定坐标系下的 TCP 位姿解算关节角（逆解） |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/ik` |
| 服务类型 | `tl_ros2_interface/srv/Ik` |

**请求参数**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| frame | int32 | — | 输入位姿坐标系：1=基坐标 2=工具 3=用户；0=关节，不支持 |
| pose | CartesianPose | m + rad | 目标位姿；`arm_angle` 为冗余臂角 ψ：`source=1` 的 7 轴作为硬约束（不指定传 NaN），`source=0` 透传到 SDK 第 7 位，6 轴忽略 |
| form | int32 | — | 形态（0=默认/自动；1~8 指定分支，控制器与本地模型语义一致） |
| reference_pos | float64[] | rad | 逆解参考关节角（**必填**，长度与关节数一致，6 或 7；用于选择离参考点最近的形态） |
| source | int32 | — | 求解来源：0=控制器（默认）1=本地模型（仅 `frame=1`） |

**响应参数**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| success | bool | — | 解算是否成功 |
| message | string | — | 成功/失败描述 |
| joints | float64[7] | rad | 解算关节角，定长 7；失败时为 NaN；6 轴第 7 位为 0 |

> 输入位置由 m 换算为 mm，姿态保持 rad；输出关节角由度换算为 rad。TOOL 坐标系位姿锚定调用时刻的当前 TCP，仅适用于机械臂静止或对锚定时刻不敏感的场景。
> `source=1` 仅支持基坐标系与 6/7 轴；`arm_angle` 对 7 轴是 ψ 硬约束（不指定时传 NaN），对 6 轴忽略。本地模型不可用时返回失败，不回退控制器。
#### 命令示例
```
ros2 service call /tl_driver/ik tl_ros2_interface/srv/Ik "{
  frame: 1,
  pose: {
    header: {frame_id: 'base_link'},
    position: {x: 0.28, y: 0.0, z: 0.47},
    rpy: {x: 3.14, y: 0.0, z: 0.0},
    arm_angle: 0.0
  },
  form: 0,
  reference_pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  source: 0
}"
```

### 4.23 控制器错误与告警
| 功能描述 | 接收控制器主动推送的错误/告警信息（6001 主端口异步回调） |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/tl_driver/error_info` |
| 消息类型 | `tl_ros2_interface/msg/ErrorInfo` |

**输入参数**
无（话题订阅，无需请求参数）。

**输出/返回值**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| message_type | int32 | — | SDK 回调 `messageType`，控制器原样透传（错误/告警类别） |
| message_code | int32 | — | SDK 回调 `messageCode`，控制器错误码 |
| message | string | — | SDK 回调文本 |
| header.stamp | builtin_interfaces/Time | — | 驱动收到回调的时刻 |

> 事件驱动，不设发布频率参数。驱动对每条推送都发布一帧，不在话题侧节流或去重；仅日志按 1 s 节流 WARN，避免刷屏。QoS 为 `KeepLast(100)` + reliable，不使用 `transient_local`（新订阅者不应收到陈旧故障）。`message_type` / `message_code` 的取值含义见控制器错误码表。
#### 命令示例
```
ros2 topic echo /tl_driver/error_info
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
| target_pos_value | float64[14] | rad / m | — | 定长 14 维（与 SDK `MoveCmd::targetPosValue` 一致）：前 7 位本体、后 7 位外部轴。`coord=0` 时 [0..6] 为关节角（rad）；`coord=1/2/3` 时 [0]X(m) [1]Y(m) [2]Z(m) [3]RX(rad) [4]RY(rad) [5]RZ(rad) [6]冗余臂角(rad)；[7..13] 外部轴按 SDK 原值透传 |
| coord | int32 | — | 0～3 | 坐标系：0=关节（rad），1=直角，2=工具，3=用户（1/2/3 时位置 m、姿态 rad） |
| velocity | float64 | % / m/s | — | MOVJ 为 %（0<v≤100）；MOVL 为 m/s（0<v≤1）；队列接口同此口径 |
| velocity_sync | float64 | % / m/s | — | 外部轴同步速度：与 velocity 同口径（MOVJ %；MOVL m/s） |
| acc | float64 | % | (0,100] | 加速度百分比 |
| dec | float64 | % | (0,100] | 减速度百分比 |
| pl | int32 | — | 0～5 | 平滑过渡等级：0=精确到达，非百分比 |
| tool_num | int32 | — | — | 工具坐标系编号 |
| user_num | int32 | — | — | 用户坐标系编号 |
| para_sync | bool | — | — | `true` = 外部轴同步，`false` = 不同步 |

> 点位类型固定为自定义数组（`PosType::data`）；`target_pos_name`、`target_pos_type`、`time`、`posidtype`、`configuration`、`spin` 已从消息中删除，驱动按控制器默认值硬编码（0 / 空串）。消息为定长数组，不再有长度校验。

## 7 运动控制接口
### 7.1 运动 Action

| Action | 类型 | 说明 |
| :---- | :---- | :---- |
| `/tl_driver/move_joint` | `tl_ros2_interface/action/Move` | 关节运动（仅 `coord=0`，关节 rad） |
| `/tl_driver/move_linear` | `tl_ros2_interface/action/Move` | 直线运动（仅 `coord=1`，位置 m、姿态 rad） |
| `/tl_driver/play_track` | `tl_ros2_interface/action/PlayTrack` | 轨迹回放（`vel` 1–100 %） |

**Move goal**
| 字段 | 类型 | 说明 |
| :---- | :---- | :---- |
| command | `MoveCommand` | 运动指令；`target_pos_value` 为定长 14 维 |
| timeout_sec | float64 | 超时秒数；<=0 时使用参数 `motion_action_timeout` |

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
| timeout_sec | float64 | 超时秒数；<=0 时使用参数 `track_action_timeout` |
| progress | float64 | 控制器不提供进度，固定 -1 |
| state | string | `RUNNING` / `STOP` / `PAUSE` |
| success / message | bool / string | result 中的最终结果与说明 |

> **超时参数**：`move_joint` / `move_linear` 使用 `motion_action_timeout`（默认 60 s），`play_track` 使用 `track_action_timeout`（默认 600 s）；goal 的 `timeout_sec` <= 0 时取节点参数。

> **cancel 不支持**：控制器 SDK 没有通用运动停止接口（仅有 `robot_stop_jogging`、`queue_motion_stop_not_power_off`、`job_stop`、`track_record_stop` 录制停止），因此三个 action 的 cancel 请求一律返回 `REJECT`，不会出现“取消成功但机械臂仍在运动”。同一时刻只允许一个运动 action goal；已有 goal 运行或 `run_state` 非 `STOP`（含 `PAUSE`）时新 goal 被拒绝。action 执行期间，`set_servoj_pos`、`track_playback`、队列运动与 `open_servoj` 会被拒绝/忽略并记录告警。刚通过队列运动或轨迹回放等服务成功下发运动后 500 ms 内到达的 action goal 会被拒绝，避免控制器 `run_state` 更新延迟导致并发运动。完成判定对 `coord=0` 用 ACS、`coord=1` 用 MCS 校验目标，到达容差为 1° / 2 mm；`coord=2/3` 的 action goal 会被拒绝。

### 7.2 运动 Action 测试命令

前置条件：

- `tl_driver` 已启动并连接控制器，`get_arm_status` 返回 `STOP`，机械臂已上电且工作区安全。
- 以下示例以 6 轴零位为基准：`/tcp_pose` = `(0.2793, 0, 0.4216)` m，rpy = `(3.1414, 0, 0)` rad。换用其他初始位姿前，先读当前位姿并替换 `target_pos_value`。

```bash
ros2 topic echo /tcp_pose --once
ros2 service call /tl_driver/get_arm_status tl_ros2_interface/srv/GetArmStatus "{}"
```

#### MoveJ：J6 转 10°

10° = 0.174533 rad。`target_pos_value` 必须为 14 位：前 7 位是 J1–J7（rad），后 7 位是外部轴。

```bash
ros2 action send_goal /tl_driver/move_joint tl_ros2_interface/action/Move \
"{command: {
  target_pos_value: [0.0, 0.0, 0.0, 0.0, 0.0, 0.174533, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  coord: 0,
  velocity: 20.0,
  velocity_sync: 0.0,
  acc: 20.0,
  dec: 20.0,
  pl: 0,
  tool_num: 0,
  user_num: 0,
  para_sync: false
}, timeout_sec: 10.0}" --feedback
```

回零：

```bash
ros2 action send_goal /tl_driver/move_joint tl_ros2_interface/action/Move \
"{command: {
  target_pos_value: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  coord: 0,
  velocity: 20.0, velocity_sync: 0.0, acc: 20.0, dec: 20.0,
  pl: 0, tool_num: 0, user_num: 0, para_sync: false
}, timeout_sec: 10.0}" --feedback
```

#### MoveL：沿 +Z 平移 5 cm

基准位姿 z = 0.4216 m，目标 z = 0.4716 m。`coord=1`，位置 m、姿态 rad；`velocity` 单位为 m/s。

```bash
ros2 action send_goal /tl_driver/move_linear tl_ros2_interface/action/Move \
"{command: {
  target_pos_value: [0.2793, 0.0, 0.4716, 3.1414, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  coord: 1,
  velocity: 0.01,
  velocity_sync: 0.0,
  acc: 20.0,
  dec: 20.0,
  pl: 0,
  tool_num: 0,
  user_num: 0,
  para_sync: false
}, timeout_sec: 10.0}" --feedback
```

回原位：

```bash
ros2 action send_goal /tl_driver/move_linear tl_ros2_interface/action/Move \
"{command: {
  target_pos_value: [0.2793, 0.0, 0.4216, 3.1414, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  coord: 1,
  velocity: 0.01, velocity_sync: 0.0, acc: 20.0, dec: 20.0,
  pl: 0, tool_num: 0, user_num: 0, para_sync: false
}, timeout_sec: 10.0}" --feedback
```

> `move_joint` 的 `velocity` 是百分比，`move_linear` 的 `velocity` 是 m/s；`target_pos_value` 为定长 14 维，`move_joint` 要求 `coord=0`、`move_linear` 要求 `coord=1`，非有限值会被拒绝。两个 action 的 cancel 都会被拒绝，测试时如遇异常直接拍急停。

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
| 功能描述 | 回放已保存的拖拽轨迹（已迁移到 `/tl_driver/play_track` action） |
| :---: | :---- |
| 通信机制 | ROS2 Action |
| Action 名 | `/tl_driver/play_track` |
| Action 类型 | `tl_ros2_interface/action/PlayTrack` |
| 服务状态 | `/tl_driver/track_playback`（`TrackPlayback.srv`）已删除 |

**输入参数**
| 参数名 | 类型 | 单位 | 范围 | 说明 |
|--------|------|------|------|------|
| vel | int32 | % | 1～100 | 轨迹回放速度百分比 |
| timeout_sec | float64 | s | ≤3600 | 超时；<=0 时取参数 `track_action_timeout`（默认 600 s） |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | result 中的最终结果 |
| message | string | 失败原因或完成说明 |

> 回放是秒级到分钟级长任务，Action 额外提供超时、进度反馈与运动互斥；cancel 因 SDK 无回放停止接口而显式拒绝。
#### 命令示例
```
ros2 action send_goal /tl_driver/play_track tl_ros2_interface/action/PlayTrack "{vel: 20, timeout_sec: 600.0}" --feedback
```
### 9.7 设置示教模式类型
| 功能描述 | 设置机械臂示教模式类型（点动 / 拖拽） |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_teach_type` |
| 服务类型 | `tl_ros2_interface/srv/SetTeachType` |

**输入参数**
| 参数名 | 类型 | 范围 | 说明 |
|--------|------|------|------|
| type | int32 | 0～1 | 示教类型：0=点动，1=拖拽；其他值直接返回 `Invalid teach type`，不下发控制器 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |

> **开启力引导拖拽需两步，顺序不可颠倒**：先 `set_drag_mode(mode=1)` 进入力矩拖拽，再 `set_teach_type(type=1)` 切到拖拽示教；只调 `set_teach_type` 不会产生可拖拽的力矩。用 `get_teach_type` 返回 `type=1` 才算真正生效。
#### 命令示例
```
ros2 service call /tl_driver/set_teach_type tl_ros2_interface/srv/SetTeachType "{type: 1}"
```
### 9.8 查询示教模式类型
| 功能描述 | 查询机械臂当前的示教模式类型 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_teach_type` |
| 服务类型 | `tl_ros2_interface/srv/GetTeachType` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| type | int32 | 0=点动，1=拖拽；查询失败时保持 `-1`（哨兵值，避免误判为点动） |
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/get_teach_type tl_ros2_interface/srv/GetTeachType "{}"
```
## 10 错误处理接口
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
| 功能描述 | 查询所有数字输入（DI）和数字输出（DO）端口状态；这是数字 IO 的唯一读取路径，不再发布 `io_state` 话题 |
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

### 15.4 状态流参数

驱动连接成功后，在 7000 端口启动两个连续反馈通道，回调只解析并写快照；`/joint_states`、`/tcp_pose` 由帧通知唤醒的独立发布线程直接发布，不经过执行器或 6001 端口的同步查询。运行状态改由 `get_arm_status` 服务按需查询，不再周期发布。

| 通道 | 请求 | 数据源 | 用途 |
| :---- | :---- | :---- | :---- |
| 1 | `mode=1, position=0, interval=10` | `realPosACS`（度） | `/joint_states` |
| 2 | `mode=1, position=1, interval=10` | `realPosMCS`（位置 mm、姿态度） | `/tcp_pose`、action feedback |

| 参数 | 默认 | 范围 | 说明 |
| :---- | ----: | :---- | :---- |
| `state_stream_interval_ms` | 10 | 10–60000 | 连续反馈间隔 |
| `executor_threads` | 8 | 1–64 | 执行器线程数；默认 `max(4, min(hardware_concurrency, 8))`，阻塞 I/O 场景可调到 16 |
| `joint_monitor_rate` | 2.0 | 0–2 | `/tl_driver/joint_monitor` 发布频率（Hz），含电流/温度/电压查询，0=关闭 |
| `feedback_rate` | 100.0 | 0–100 | `/tl_driver/joint_speed`、`/tl_driver/line_speed`、`/tl_driver/joint_torque` 共用定时器的发布频率（Hz），0=关闭；定时器回调内先后调用速度与力矩两个发布函数 |

> 状态流没有同步查询回退路径：某通道无新帧时对应话题停止发布（保持静默而不是发布过期值）。状态流在上下电与模式切换完成后启动，任一通道请求失败则驱动启动失败；运行期控制器停止推帧后需要重启驱动恢复（不设看门狗重试）。运行状态由 10 Hz 定时器查询，查询成功即发布 `/arm_status`；查询失败时该周期不发布（节流 WARN），不会用缓存值兜底。运行速度改由 `get_speed` 服务按需查询，不再由定时器轮询。
>
> 话题频率参数 `joint_monitor_rate` / `feedback_rate` 见 [4.10](#410-查询电机电流)、[4.19](#419-查询当前电机力矩)、[4.20](#420-查询当前线速度和关节速度)；运动 action 超时参数见 [7.1](#71-运动-action)。

### 15.5 打开周期同步速度模式
| 功能描述 | 进入周期同步速度模式（CSV），之后由 `/tl_driver/set_servo_csv_velocity` 按周期下发各关节目标速度 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/open_servo_csv` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 进入成功，`false` = 进入失败 |
| message | string | 失败时包含错误描述；运动 action 运行中会返回 `A motion action is active` |

> **注意（模式互斥）**：CSV / CST / ServoJ 共用 7000 端口的 servoJ 会话，SDK **不做互斥检测**，同一时刻只能开启其中一种；切换前先关闭当前模式（如先 `close_servoj` 再 `open_servo_csv`）。
> **注意（安全）**：进入该模式后机械臂由速度指令驱动，开启前需确认工作空间内无人、自身有支撑或已限速；停止时先发零速度再 `close_servo_csv`。
#### 命令示例
```
ros2 service call /tl_driver/open_servo_csv std_srvs/srv/Trigger "{}"
```
### 15.6 关闭周期同步速度模式
| 功能描述 | 退出周期同步速度模式，恢复常规轨迹规划 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/close_servo_csv` |
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
ros2 service call /tl_driver/close_servo_csv std_srvs/srv/Trigger "{}"
```
### 15.7 发送周期同步速度
| 功能描述 | 在 CSV 模式下按周期下发各关节目标速度 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/tl_driver/set_servo_csv_velocity` |
| 消息类型 | `std_msgs::msg::Float64MultiArray` |

**输入参数**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| data | float64[] | rad/s | 各关节目标速度，6 或 7 元素；6 轴时末尾补 0（外部轴保持 0）；驱动内部换算为度/秒下发 |

**输出/返回值**
无返回值（话题发布单向通信）。

> **注意**：需先 [打开周期同步速度模式](#155-打开周期同步速度模式)，否则 SDK 返回 `OPERATION_NOT_ALLOWED`。需以固定周期持续发布（建议 ≥100 Hz）；元素数非 6/7 或含 NaN/Inf 的帧直接丢弃并节流告警。成功时日志为 DEBUG，失败为每秒最多一条 WARN。
#### 命令示例
```
ros2 topic pub -r 100 /tl_driver/set_servo_csv_velocity std_msgs/msg/Float64MultiArray \
"{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```
### 15.8 打开周期同步扭矩模式
| 功能描述 | 进入周期同步扭矩模式（CST），之后由 `/tl_driver/set_servo_cst_torque` 按周期下发各关节目标扭矩 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/open_servo_cst` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 进入成功，`false` = 进入失败 |
| message | string | 失败时包含错误描述；运动 action 运行中会返回 `A motion action is active` |

> **注意（模式互斥）**：与 CSV / ServoJ 同样共用 7000 端口的 servoJ 会话，SDK 不做互斥检测，切换前先关闭当前模式。
> **注意（安全）**：CST 为纯力矩透传，机械臂**不受位置/重力补偿保护**，开启后若不下发保持力矩会发生重力坠落——必须有支撑或人工扶持。
> **注意（能力边界）**：力矩补偿（重力/摩擦/负载）尚未实现，本模式只做原始力矩透传。
#### 命令示例
```
ros2 service call /tl_driver/open_servo_cst std_srvs/srv/Trigger "{}"
```
### 15.9 关闭周期同步扭矩模式
| 功能描述 | 退出周期同步扭矩模式，恢复常规轨迹规划 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/close_servo_cst` |
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
ros2 service call /tl_driver/close_servo_cst std_srvs/srv/Trigger "{}"
```
### 15.10 发送周期同步扭矩
| 功能描述 | 在 CST 模式下按周期下发各关节目标扭矩 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/tl_driver/set_servo_cst_torque` |
| 消息类型 | `std_msgs::msg::Float64MultiArray` |

**输入参数**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| data | float64[] | ‰ 额定扭矩 | 各关节目标扭矩，6 或 7 元素；6 轴时末尾补 0（外部轴保持 0）；范围 [-1000, 1000]，与 SDK 单位（额定扭矩 0.1%）一致，不做换算 |

**输出/返回值**
无返回值（话题发布单向通信）。

> **注意**：需先 [打开周期同步扭矩模式](#158-打开周期同步扭矩模式)。元素数非 6/7、含 NaN/Inf 或超出 [-1000, 1000] 的帧**整帧丢弃**并节流告警（不做裁剪，避免误因限幅产生不可预期的转矩）。需以固定周期持续发布；停止时发零扭矩再关闭模式。
#### 命令示例
```
ros2 topic pub -r 100 /tl_driver/set_servo_cst_torque std_msgs/msg/Float64MultiArray \
"{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

## 16 接口矩阵与移除清单

### 16.1 接口总矩阵

**话题**

| 接口 | 类型 | 频率 | 字段单位 | 状态 |
| :---- | :---- | :---- | :---- | :---- |
| `/joint_states` | `sensor_msgs/msg/JointState` | 7000 帧率（默认 100 Hz） | position rad | 稳定 |
| `/tcp_pose` | `CartesianPose` | 7000 帧率（默认 100 Hz） | position m、rpy rad | 稳定 |
| `/tl_driver/joint_monitor` | `JointMonitor` | `joint_monitor_rate`（默认 2 Hz，范围 0–2；电流/温度/电压同帧查询） | A、℃、V | 稳定 |
| `/tl_driver/joint_torque` | `JointTorque` | `feedback_rate`（默认 100 Hz，与速度话题同一定时器） | ‰ 额定扭矩 | 稳定 |
| `/tl_driver/joint_speed` | `JointSpeed` | `feedback_rate`（默认 100 Hz，与 line_speed 同一次查询） | rad/s | 稳定 |
| `/tl_driver/line_speed` | `LineSpeed` | `feedback_rate`（默认 100 Hz，与 joint_speed 同一次查询） | m/s | 稳定 |
| `/tl_driver/error_info` | `ErrorInfo` | 事件驱动（不节流） | int32 / string | 稳定 |
| `/tl_driver/set_servoj_pos` | `std_msgs/msg/Float64MultiArray` | 事件驱动（≤250 Hz） | rad（6 或 7 元素） | 稳定 |
| `/tl_driver/set_servo_csv_velocity` | `std_msgs/msg/Float64MultiArray` | 事件驱动（建议 ≥100 Hz） | rad/s（6 或 7 元素） | 稳定 |
| `/tl_driver/set_servo_cst_torque` | `std_msgs/msg/Float64MultiArray` | 事件驱动（建议 ≥100 Hz） | ‰ 额定扭矩（6 或 7 元素，[-1000, 1000]） | 稳定 |

**Action**

| 接口 | 类型 | 频率 | 参数单位 | 状态 |
| :---- | :---- | :---- | :---- | :---- |
| `/tl_driver/move_joint` | `Move` | 事件驱动 | 关节 rad、速度 % | 稳定 |
| `/tl_driver/move_linear` | `Move` | 事件驱动 | 位置 m、姿态 rad、速度 m/s | 稳定 |
| `/tl_driver/play_track` | `PlayTrack` | 事件驱动 | 速度 % | 稳定 |

**服务（44 个）**

| 接口 | 类型 | 领域 | 单位/说明 | 状态 |
| :---- | :---- | :---- | :---- | :---- |
| `/tl_driver/power_on` | `Trigger` | session | 上电 | 稳定 |
| `/tl_driver/power_off` | `Trigger` | session | 下电 | 稳定 |
| `/tl_driver/get_controller_id` | `Trigger` | session | 控制器序列号 | 稳定 |
| `/tl_driver/get_library_version` | `Trigger` | session | SDK 版本 | 稳定 |
| `/tl_driver/set_controller_ip` | `SetControllerIP` | session | IP | 稳定 |
| `/tl_driver/set_default_cartesian_param` | `Trigger` | config | 默认笛卡尔参数 | 稳定 |
| `/tl_driver/set_drag_mode` | `SetDragMode` | config | 拖拽模式 | 稳定 |
| `/tl_driver/set_teach_type` | `SetTeachType` | config | 示教类型（0 点动 / 1 拖拽） | 稳定 |
| `/tl_driver/get_teach_type` | `GetTeachType` | config | 示教类型（失败返回 -1） | 稳定 |
| `/tl_driver/get_dh_param` | `GetDHParam` | config | rad、m | 稳定 |
| `/tl_driver/get_robot_joint_param` | `GetRobotJointParam` | config | rad、m、rad/s、rad/s² | 稳定 |
| `/tl_driver/set_tool_param` | `SetToolParam` | config | m、rad、kg | 稳定 |
| `/tl_driver/set_user_coord` | `SetUserCoord` | config | m、rad | 稳定 |
| `/tl_driver/set_coord_num` | `SetCoordNum` | config | 编号 | 稳定 |
| `/tl_driver/get_coord_num` | `GetCoordNum` | config | 编号 | 稳定 |
| `/tl_driver/get_pos_reachable` | `GetPosReachable` | config | 14 维协议包，原样透传 | 稳定 |
| `/tl_driver/set_digital_output` | `SetDigitalOutput` | io | 数字输出 | 稳定 |
| `/tl_driver/get_digital_input_output` | `GetDigitalInputOutput` | io | 数字输入输出 | 稳定 |
| `/tl_driver/modbus_write` | `ModbusWrite` | io | Modbus | 稳定 |
| `/tl_driver/modbus_read` | `ModbusRead` | io | Modbus | 稳定 |
| `/tl_driver/set_speed` | `SetSpeed` | motion | % | 稳定 |
| `/tl_driver/get_speed` | `GetSpeed` | motion | % | 稳定 |
| `/tl_driver/start_jogging` | `Jogging` | motion | rad/s、rad/s² | 稳定 |
| `/tl_driver/stop_jogging` | `Jogging` | motion | — | 稳定 |
| `/tl_driver/track_save` | `TrackSave` | motion | — | 稳定 |
| `/tl_driver/open_servoj` | `OpenServoJ` | motion | rad/s、rad/s²、rad/s³ | 稳定 |
| `/tl_driver/close_servoj` | `Trigger` | motion | — | 稳定 |
| `/tl_driver/open_servo_csv` | `Trigger` | motion | 速度透传模式开关 | 稳定 |
| `/tl_driver/close_servo_csv` | `Trigger` | motion | 速度透传模式开关 | 稳定 |
| `/tl_driver/open_servo_cst` | `Trigger` | motion | 扭矩透传模式开关 | 稳定 |
| `/tl_driver/close_servo_cst` | `Trigger` | motion | 扭矩透传模式开关 | 稳定 |
| `/tl_driver/queue_motion_set_status` | `QueueMotionSetStatus` | motion | — | 稳定 |
| `/tl_driver/queue_motion_movej` | `QueueMotionMoveJ` | motion | 嵌套 `MoveCommand` | 稳定 |
| `/tl_driver/queue_motion_stop` | `Trigger` | motion | — | 稳定 |
| `/tl_driver/set_current_mode` | `SetCurrentMode` | state | 模式 | 稳定 |
| `/tl_driver/get_current_mode` | `GetCurrentMode` | state | 模式 | 稳定 |
| `/tl_driver/get_arm_status` | `GetArmStatus` | state | 运行状态 | 稳定 |
| `/tl_driver/get_drag_status` | `Trigger` | state | 拖拽状态 | 稳定 |
| `/tl_driver/clear_error` | `Trigger` | state | 清错 | 稳定 |
| `/tl_driver/coord_transform` | `CoordTransform` | transform | m、rad | 稳定 |
| `/tl_driver/fk` | `Fk` | transform | joints rad；pose m + rad；`source` 0=控制器 1=本地 | 稳定 |
| `/tl_driver/ik` | `Ik` | transform | pose m + rad；joints rad；`reference_pos` 必填；`source` 0=控制器 1=本地 | 稳定 |
| `/tl_driver/set_current_coord` | `SetCurrentCoord` | transform | 坐标系类型 | 稳定 |
| `/tl_driver/get_current_coord` | `GetCurrentCoord` | transform | 坐标系类型 | 稳定 |

> 服务均为请求/响应式，无固定频率；返回统一为 `success` + `message`，业务失败不抛异常。

**消息与 Action 类型（13 个 msg + 2 个 action）**

| 消息 | 用途 | 状态 |
| :---- | :---- | :---- |
| `MoveCommand` | `Move` action goal 与 `queue_motion_movej` 的指令载荷 | 稳定 |
| `Move`（action） | `move_joint` / `move_linear` | 稳定 |
| `PlayTrack`（action） | `play_track` | 稳定 |
| `CartesianPose` | `/tcp_pose` | 稳定 |
| `JointMonitor` | `/tl_driver/joint_monitor` 诊断话题（电流/温度/电压同帧查询，定长数组 + NaN） | 稳定 |
| `JointSpeed` / `LineSpeed` | `/tl_driver/joint_speed` / `/tl_driver/line_speed` | 稳定 |
| `JointTorque` | `/tl_driver/joint_torque` | 稳定 |
| `ErrorInfo` | `/tl_driver/error_info` 控制器错误/告警（事件驱动，不节流） | 稳定 |
| `RobotDHParam` / `RobotJointParam` / `ToolParam` / `ModbusMasterParam` / `ModbusRTUParam` / `ModbusTCPParam` | 服务嵌套参数 | 稳定 |

### 16.2 已移除接口

| 接口 | 类型 | 替代 | 移除版本 |
| :---- | :---- | :---- | :---- |
| `get_quat2rpy`、`get_rpy2quat`、`get_rpy2r`、`get_tr2r`、`get_r2tr` | srv | 消费方本地数学库 | V1.9 |
| `get_all_job_filename`、`job_run`、`job_delete`、`job_insert_moveJ`、`job_insert_moveL`、`job_insert_imove`、`job_insert_moveC` | srv | 无（非 ROS 驱动职责） | V1.9 |
| `get_global_pos`、`set_global_pos` | srv | 无 | V1.9 |
| `get_nexmotion_lib_version`、`set_dh_param`、`restore_default_dh_param`、`log_download` | srv | `get_library_version` / 无 | V1.9 |
| `connect_arm`、`disconnect_arm` | srv | 节点启动自动连接、退出自动断开 | V1.11 |
| `get_robot_state` | srv | `/joint_states`、`/tcp_pose`、`get_arm_status` | V1.16 |
| `get_joint_temperature`、`get_joint_voltage` | srv | `/tl_driver/joint_monitor` | V1.16 |
| `get_motor_current` | srv | `/tl_driver/joint_monitor` | V1.16 |
| `get_current_motor_torque` | srv | `/tl_driver/joint_torque` | V1.16 |
| `get_current_line_joint_speed` | srv | `/tl_driver/line_speed` + `/tl_driver/joint_speed` | V1.16 |
| `get_joint_software_version` | srv | 启动日志逐轴打印 | V1.16 |
| `set_robot_joint_param` | srv | 无 | V1.16 |
| `io_state` | topic | `get_digital_input_output` / `set_digital_output` | V1.16 |
| `IoState`、`ServolMove`、`JobFileName` | msg | 无 / 本地 IK / 无 | V1.10–V1.16 |
| `set_servol_pos` | topic | 消费方本地 IK 后走 `set_servoj_pos` | V1.10 |
| `moveJ`、`moveL` | topic | `move_joint` / `move_linear` action | V1.18 |
| `track_playback` | srv | `/tl_driver/play_track` action | V1.20 |
| `arm_status.speed` / `speed_valid` | 字段 | `/tl_driver/get_speed` 服务 | V1.22 |
| `JointDynamics`、`JointThermal` | msg | `JointMonitor`（删除外部轴电流/力矩与外部轴电压字段） | V1.22 |
| `JointMonitor.line_speed` / `.joint_speed` / `.motor_torque` | 字段 | `/tl_driver/line_speed`、`/tl_driver/joint_speed`、`/tl_driver/joint_torque` | V1.23 |
| `JointSpeed.joint_speed_sync` | 字段 | 无（外部轴同步速度不再发布） | V1.23 |
| `joint_dynamics_rate` | 参数 | `joint_monitor_rate` | V1.22 |
| `joint_thermal_rate` | 参数 | 无（温度/电压并入 `joint_monitor_rate`，2 Hz） | V1.23 |
| `joint_speed_rate`、`joint_torque_rate` | 参数 | `feedback_rate`（速度与力矩共用同一定时器） | V1.25 |
| `state_source`（`poll` 回退） | 参数 | 仅保留 7000 状态流 | 本版本 |
| `publish_rate` | 参数 | 无（话题发布频率由各 rate 参数控制） | 本版本 |
| `/arm_status` | topic | `get_arm_status` 服务 | V1.24 |
| `ArmStatus` | msg | `GetArmStatus.srv` 响应字段 | V1.24 |

> `get_speed` 曾在 V1.16 移除，V1.22 恢复为独立服务；同时删除 `arm_status.speed` / `speed_valid` 字段。
