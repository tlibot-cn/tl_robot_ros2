<div align="center">

# 天链机械臂接口函数使用说明书（ROS2）

四川天链机器人股份有限公司

文件修订记录：

|版本号 | 时间 | 备注 |
| :---: | :---- | :---: |
|V1.0 | 2026-5-22 | 拟制 |

TL 系列机械臂 ROS2 接口说明

</div>

## 目录
* 1 [tl_ros2_interface功能包说明](#tl_ros2_interface功能包说明)
* 2 [tl_ros2_interface使用说明](#tl_ros2_interface使用说明)
* 3 [tl_ros2_interface文件总览](#tl_ros2_interface文件总览)
* 4 [tl_ros2_interface消息（msg）说明](#tl_ros2_interface消息msg说明)
* 4.1 [机械臂运行状态ArmStatus_msg](#机械臂运行状态ArmStatus_msg)
* 4.2 [直角坐标系位姿CartesianPose_msg](#直角坐标系位姿CartesianPose_msg)
* 4.3 [ModbusMaster参数ModbusMasterParam_msg](#ModbusMaster参数ModbusMasterParam_msg)
* 4.4 [ModbusRTU参数ModbusRTUParam_msg](#ModbusRTU参数ModbusRTUParam_msg)
* 4.5 [ModbusTCP参数ModbusTCPParam_msg](#ModbusTCP参数ModbusTCPParam_msg)
* 4.6 [运动指令MoveCommand_msg](#运动指令MoveCommand_msg)
* 4.7 [机器人DH参数RobotDHParam_msg](#机器人DH参数RobotDHParam_msg)
* 4.8 [机械臂关节参数RobotJointParam_msg](#机械臂关节参数RobotJointParam_msg)
* 4.9 [工具参数ToolParam_msg](#工具参数toolparam_msg)
* 4.10 [关节动力学JointDynamics_msg](#关节动力学jointdynamics_msg)
* 4.11 [关节热状态JointThermal_msg](#关节热状态jointthermal_msg)
* 5 [tl_ros2_interface服务（srv）说明](#tl_ros2_interface服务srv说明)
* 5.1 [坐标转换CoordTransform_srv](#坐标转换CoordTransform_srv)
* 5.2 [坐标系编号GetCoordNum_srv](#坐标系编号GetCoordNum_srv)
* 5.3 [当前坐标系GetCurrentCoord_srv](#当前坐标系GetCurrentCoord_srv)
* 5.4 [机器人DH参数GetDHParam_srv](#机器人DH参数GetDHParam_srv)
* 5.5 [数字输入输出状态GetDigitalInputOutput_srv](#数字输入输出状态GetDigitalInputOutput_srv)
* 5.6 [目标位姿可达状态GetPosReachable_srv](#目标位姿可达状态GetPosReachable_srv)
* 5.7 [关节参数GetRobotJointParam_srv](#关节参数GetRobotJointParam_srv)
* 5.8 [机械臂点动Jogging_srv](#机械臂点动Jogging_srv)
* 5.9 [写ModbusModbusWrite_srv](#写ModbusModbusWrite_srv)
* 5.10 [读ModbusModbusRead_srv](#读ModbusModbusRead_srv)
* 5.11 [关节跟踪OpenServoJ_srv](#关节跟踪OpenServoJ_srv)
* 5.12 [MoveJ队列运动QueueMotionMoveJ_srv](#MoveJ队列运动QueueMotionMoveJ_srv)
* 5.13 [MoveJ队列运动模式QueueMotionSetStatus_srv](#MoveJ队列运动模式QueueMotionSetStatus_srv)
* 5.14 [设置关节零点SetAxisZeroPos_srv](#设置关节零点SetAxisZeroPos_srv)
* 5.15 [控制器有线网口IP设置SetControllerIP_srv](#控制器有线网口IP设置SetControllerIP_srv)
* 5.16 [设置坐标系编号SetCoordNum_srv](#设置坐标系编号SetCoordNum_srv)
* 5.17 [设置当前坐标系SetCurrentCoord_srv](#设置当前坐标系SetCurrentCoord_srv)
* 5.18 [设置当前运行模式SetCurrentMode_srv](#设置当前运行模式SetCurrentMode_srv)
* 5.19 [设置数字输出SetDigitalOutput_srv](#设置数字输出SetDigitalOutput_srv)
* 5.20 [设置拖拽模式SetDragMode_srv](#设置拖拽模式SetDragMode_srv)
* 5.21 [设置运行速度SetSpeed_srv](#设置运行速度SetSpeed_srv)
* 5.22 [设置工具手参数SetToolParam_srv](#设置工具手参数SetToolParam_srv)
* 5.23 [设置用户坐标系SetUserCoord_srv](#设置用户坐标系SetUserCoord_srv)
* 5.24 [拖拽轨迹回放TrackPlayback_srv](#拖拽轨迹回放TrackPlayback_srv)
* 5.25 [拖拽轨迹保存TrackSave_srv](#拖拽轨迹保存TrackSave_srv)
* 5.26 [查询当前运行模式GetCurrentMode_srv](#查询当前运行模式GetCurrentMode_srv)

## tl_ros2_interface功能包说明
tl_ros2_interface 功能包为 TL 系列机械臂在 ROS2 框架下提供消息（msg）和服务（srv）接口定义，供上层驱动或应用调用。该包本身没有可执行程序，主要作用为定义协议数据结构和服务接口。

> **单位约定**：全部 ROS 接口使用 REP 103 单位（关节 rad、笛卡尔位置 m、线速度 m/s）。控制器/SDK 的 mm/度在 `tl_driver` 边界换算。
> 例外：`GetPosReachable` 的 14 位点位数组是控制器协议包，自带单位制标志位，按协议原样透传。

## tl_ros2_interface使用说明
该功能包并没有可执行的使用命令，其主要作用为为其他功能包提供必须的消息文件。

## tl_ros2_interface文件总览
```
tl_ros2_interface/
├── CMakeLists.txt        # 编译规则
├── package.xml           # 依赖声明
├── msg/                  # 消息定义
│   ├── ArmStatus.msg
│   ├── CartesianPose.msg
│   ├── ModbusMasterParam.msg
│   ├── ModbusRTUParam.msg
│   ├── ModbusTCPParam.msg
│   ├── MoveCommand.msg
│   ├── RobotDHParam.msg
│   ├── RobotJointParam.msg
│   ├── ToolParam.msg
│   ├── JointDynamics.msg
│   └── JointThermal.msg
├── action/               # Action 定义
│   ├── Move.action
│   └── PlayTrack.action
├── srv/                  # 服务定义
│   ├── CoordTransform.srv
│   ├── GetCoordNum.srv
│   ├── GetCurrentCoord.srv
│   ├── GetCurrentMode.srv
│   ├── GetDHParam.srv
│   ├── GetDigitalInputOutput.srv
│   ├── GetPosReachable.srv
│   ├── GetRobotJointParam.srv
│   ├── Jogging.srv
│   ├── ModbusRead.srv
│   ├── ModbusWrite.srv
│   ├── OpenServoJ.srv
│   ├── QueueMotionMoveJ.srv
│   ├── QueueMotionSetStatus.srv
│   ├── SetAxisZeroPos.srv
│   ├── SetControllerIP.srv
│   ├── SetCoordNum.srv
│   ├── SetCurrentCoord.srv
│   ├── SetCurrentMode.srv
│   ├── SetDigitalOutput.srv
│   ├── SetDragMode.srv
│   ├── SetSpeed.srv
│   ├── SetToolParam.srv
│   ├── SetUserCoord.srv
│   ├── TrackPlayback.srv
│   └── TrackSave.srv
```

## tl_ros2_interface消息（msg）说明
以下列出 msg 文件定义及字段说明。**11 个 msg 与 2 个 action，每个文件头部注释里都标了用途，读之前先看那一行：**

| 用途 | 数量 | 定义 |
| :-- | :--: | :-- |
| 话题载荷（在 publisher / subscriber 上出现） | 4 | `ArmStatus`、`CartesianPose`、`JointDynamics`、`JointThermal` |
| Action goal 载荷 / srv 嵌套参数 | 1 | `MoveCommand` |
| 仅作 srv 嵌套参数（不发布话题） | 6 | `ModbusMasterParam`、`ModbusRTUParam`、`ModbusTCPParam`、`RobotDHParam`、`RobotJointParam`、`ToolParam` |
| Action | 2 | `Move`（`move_joint`/`move_linear`）、`PlayTrack`（`play_track`） |

其中 `CartesianPose` 既发布/订阅话题，也作为 srv 嵌套参数（`SetUserCoord.pos`）；`MoveCommand` 作为 Move action goal 载荷，同时作为 srv 嵌套参数（`QueueMotionMoveJ.cmd`）。

> **「仅在 srv 中出现的结构为什么放在 `msg/`」**：ROS2 的 `.srv` 只能声明内建类型或消息类型字段，无法内联结构体，也没有「仅供服务使用的结构」这一类——任何被 srv 复用的结构都必须定义成 `.msg`，生成出的类型名也就必然是 `<包>/msg/<名字>`。所以「`msg/` 目录里有不发布话题的类型」是 ROS2 类型系统的必然结果，不代表这些类型接口可以发布。判断一个类型是否发布，看驱动里有没有对应的 `create_publisher` / `create_subscription`。

### 机械臂运行状态ArmStatus_msg
```
builtin_interfaces/Time stamp
string run_state
float64 speed
bool speed_valid
```
__msg成员__
- stamp: 采样时间戳
- run_state: 运行状态描述字符串
- speed: 全局运行速度百分比 %（1 Hz 查询缓存）
- speed_valid: 最近一次速度查询是否成功

### 直角坐标系位姿CartesianPose_msg
```
std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Vector3 rpy
float64 arm_angle
```
__msg成员__
- header: 标准消息头
- position: x,y,z 位置，单位 **m**（`/tcp_pose` 与 `SetUserCoord.pos` 一致）
- rpy: 姿态欧拉角（弧度，XYZ 内旋）
- arm_angle: 冗余臂角（弧度；6 轴为 0，`SetUserCoord` 未使用）

### ModbusMaster参数ModbusMasterParam_msg
```
string type
bool start_addr
tl_ros2_interface/ModbusTCPParam tcp
tl_ros2_interface/ModbusRTUParam rtu
```
__msg成员__
- type: "TCP" 或 "RTU"
- start_addr: 是否从起始地址开始
- tcp: Modbus TCP 参数
- rtu: Modbus RTU 参数

### ModbusRTU参数ModbusRTUParam_msg
```
int32 slave_id
int32 port
int32 baudrate
int32 data_bit
int32 stop_bit
string check_bit
```
__msg成员__
- slave_id: 从站 ID
- port: 串口号
- baudrate: 波特率
- data_bit: 数据位
- stop_bit: 停止位
- check_bit: 校验方式

### ModbusTCP参数ModbusTCPParam_msg
```
string ip
int32 port
```
__msg成员__
- ip: 目标 IP
- port: 目标端口

### 运动指令MoveCommand_msg
```
float64[] target_pos_value
string target_pos_name
int32 target_pos_type
int32 coord
float64 velocity
float64 velocity_sync
float64 acc
float64 dec
int32 pl
int32 time
int32 tool_num
int32 user_num
int32 posidtype
int32 configuration
int32 spin
bool para_sync
```
__msg成员__
- target_pos_value: 14 维目标点位。前 7 位本体、后 7 位外部轴；`coord=0` 时 [0..6] 为关节角（rad）；`coord=1/2/3` 时位置 m、姿态 rad，[7..13] 外部轴按 SDK 原值透传
- target_pos_name: 内置点位名（如 `"GP0001"`，`target_pos_type=5/6` 时使用）
- target_pos_type: PosType：0=自定义数组 1=P 2=E 3=RP 4=AP 5=系统内置GP 6=含外部轴的GE
- coord: 0=关节（rad）1=直角 2=工具 3=用户（1/2/3 时位置 m、姿态 rad）
- velocity/velocity_sync: MOVJ 为 %（0<v≤100）；MOVL 为 m/s（0<v≤1）；队列接口同此口径
- acc/dec: 加速度/减速度百分比，(0,100]
- pl: 平滑过渡等级 0～5，非百分比
- time: 提前执行时间（SDK 头未标注单位）
- tool_num/user_num: 工具/用户坐标编号
- posidtype: 0=一级(P/GP) 1=二级(P[I001]/GP[I001])
- configuration: 形态
- spin: MOVCA 使用：0=姿态不变 1=六轴不转 2=六轴旋转
- para_sync: 外部轴是否同步

### 机器人DH参数RobotDHParam_msg
```
# 对应 SDK tl::RobotDHParam：alpha[6]/a[6]/theta[6]/d[6]/eulerAngle/mountingAngle
float64[] alpha           # 1-6轴DH参数alpha，单位：rad
float64[] a               # 1-6轴DH参数a，单位：m
float64[] theta           # 1-6轴DH参数theta，单位：rad
float64[] d               # 1-6轴DH参数d，单位：m
int32 euler_angle         # 欧拉角模式
float64 mounting_angle    # 机器人安装角度，单位：rad
```
__msg成员__
- alpha: 1-6轴DH参数alpha（rad）
- a: 1-6轴DH参数a（m）
- theta: 1-6轴DH参数theta（rad）
- d: 1-6轴DH参数d（m）
- euler_angle: 欧拉角模式
- mounting_angle: 机器人安装角度（rad）

### 机械臂关节参数RobotJointParam_msg
```
float64 reduction_ratio
int32 encoder_resolution
float64 pos_sw_limit
float64 neg_sw_limit
float64 rated_rot_speed
float64 rated_derot_speed
float64 max_rot_speed
float64 max_derot_speed
float64 rated_vel
float64 rated_devel
float64 max_acc
float64 max_dec
int32 direction
```
__msg成员__
- reduction_ratio: 减速比
- encoder_resolution: 编码器位数
- pos_sw_limit: 轴正限位（rad）
- neg_sw_limit: 轴反限位（rad）
- rated_rot_speed: 电机额定正转速（rad/s）
- rated_derot_speed: 电机额定反转速（rad/s；新 SDK 无对应字段，当前恒为 0）
- max_rot_speed: 电机最大正转速（rad/s）
- max_derot_speed: 电机最大反转速（rad/s；新 SDK 无对应字段，当前恒为 0）
- rated_vel: 额定正速度（rad/s；新 SDK 无对应字段，当前恒为 0）
- rated_devel: 额定反速度（rad/s；新 SDK 无对应字段，当前恒为 0）
- max_acc: 最大加速度（rad/s²）
- max_dec: 最大减速度（rad/s²）
- direction: 模型方向，1：正向，-1：反向

### 工具参数ToolParam_msg
```
float64 x
float64 y
float64 z
float64 a
float64 b
float64 c
float64 payload_mass
float64 payload_inertia
float64 payload_mass_center_x
float64 payload_mass_center_y
float64 payload_mass_center_z
```
__msg成员__
- x: X 轴偏移（m）
- y: Y 轴偏移（m）
- z: Z 轴偏移（m）
- a: 绕 A 轴旋转（rad）
- b: 绕 B 轴旋转（rad）
- c: 绕 C 轴旋转（rad）
- payload_mass: 负载质量（kg）
- payload_inertia: 负载惯量（SDK 未标注单位）
- payload_mass_center_x: 负载质心 X（m）
- payload_mass_center_y: 负载质心 Y（m）
- payload_mass_center_z: 负载质心 Z（m）

### 关节动力学JointDynamics_msg
```
std_msgs/Header header
float64 line_speed
float64[] joint_speed
float64[] joint_speed_sync
float64[] motor_current
float64[] motor_current_sync
int32[] motor_torque
int32[] motor_torque_sync
```
- `/tl_driver/joint_dynamics` 载荷，默认 10 Hz
- 单位：`line_speed` m/s、速度 rad/s、电流 A、力矩为额定扭矩千分之一（‰，±1000 对应 ±100%；不是 N·m，也不是 %）

### 关节热状态JointThermal_msg
```
std_msgs/Header header
float64[] temperature
float64[] joint_voltage
float64[] positioner_voltage
```
- `/tl_driver/joint_thermal` 载荷，默认 0.5 Hz（可关闭）
- 单位：°C / V

## tl_ros2_interface服务（srv）说明
下面列出 srv 文件的请求/响应字段及简要说明。

### 坐标转换CoordTransform_srv
```
int32 origin_coord
int32 target_coord
int32 form
float64[] origin_pos
float64[] reference_pos
---
bool success
string message
float64[] target_pos
```
- origin_coord/target_coord: 源/目标坐标系编号（0=关节 1=基坐标 2=工具 3=用户）
- form: 形态
- origin_pos/reference_pos: 输入坐标与逆解参考点。关节坐标为 rad；直角/工具/用户坐标为位置 m + 姿态 rad
- 返回: success/message/target_pos（转换结果，单位同上）

### 坐标系编号GetCoordNum_srv
```
---
bool success
string message
int32 tool_num
int32 user_num
```
- 返回工具与用户坐标系编号
- 返回: success/message/tool_num（工具坐标系编号）/user_num（用户坐标系编号）

### 当前坐标系GetCurrentCoord_srv
```
---
bool success
string message
int32 coord
```
- 返回当前坐标系编号
- 返回: success/message/coord（当前坐标系编号）

### 机器人DH参数GetDHParam_srv
```
---
bool success
string message
tl_ros2_interface/RobotDHParam param
```
- 返回机器人DH参数
- 返回: success/message/param（DH 参数）

### 数字输入输出状态GetDigitalInputOutput_srv
```
---
bool success
string message
int32[] input
int32[] output
```
- 返回数字输入输出状态；数字 IO 的唯一读取接口（不发布 `io_state` 话题）
- 返回: success/message/input（数字输入状态数组）/output（数字输出状态数组）

### 目标位姿可达状态GetPosReachable_srv
```
float64[] pos
string move_type
---
bool success
string message
bool reachable
```
- 输入目标位姿，返回是否可达
- pos: 长度 14，布局同全局点位（控制器协议包，位置 mm、角度按 [1]）
- move_type: 运动类型："MOVJ" 或 "MOVL"
- 返回: success/reachable/message；`success` 表示查询是否成功，`reachable` 仅在查询成功时有效

### 关节参数GetRobotJointParam_srv
```
int32 id
---
bool success
string message
tl_ros2_interface/RobotJointParam param
```
- 查询并返回指定关节参数
- id: 关节编号
- 返回: success/message/param（关节参数）

### 机械臂点动Jogging_srv
```
int32 axis
bool direction
---
bool success
string message
```
- 点动控制指定轴及方向
- axis: 轴编号
- direction: 运动方向（true 为正方向，false 为负方向）
- 返回: success/message

### 写ModbusModbusWrite_srv
```
int32 master_id
int32 addr
int32[] data
tl_ros2_interface/ModbusMasterParam master_param
---
bool success
string message
```
- 向Modbus从站写入数据
- master_id: 主站 ID
- addr: 寄存器地址
- data: 要写入的数据数组
- master_param: Modbus 主站通信参数（TCP 或 RTU）
- 返回: success/message

### 读ModbusModbusRead_srv
```
int32 master_id
int32 addr
int32 quantity
tl_ros2_interface/ModbusMasterParam master_param
---
bool success
string message
int32[] data
```
- 从Modbus读取数据
- master_id: 主站 ID
- addr: 寄存器起始地址
- quantity: 读取的寄存器数量
- master_param: Modbus 主站通信参数（TCP 或 RTU）
- 返回: success/message/data

### 关节跟踪OpenServoJ_srv
```
float64[] vmax
float64[] amax
float64[] jmax
---
bool success
string message
```
- 设置并打开关节伺服的最大速度/加速度
- vmax: 最大速度数组，固定 7 元素（6 轴末位补 0；7 轴第 7 位为真实 J7），单位 rad/s
- amax: 最大加速度数组，固定 7 元素，单位 rad/s²
- jmax: 最大加加速度数组，固定 7 元素，单位 rad/s³
- 注意：传 6 元素会导致控制器协议解析错乱并断开 7000 端口连接（SDK 警告）
- 返回: success/message

### MoveJ队列运动QueueMotionMoveJ_srv
```
bool is_continue
tl_ros2_interface/MoveCommand cmd
---
bool success
string message
```
- 将 MoveJ 命令加入队列或继续队列执行
- is_continue: 是否继续队列执行（false 为新队列，true 为继续）
- cmd: 运动指令
- 返回: success/message

### MoveJ队列运动模式QueueMotionSetStatus_srv
```
bool status
---
bool success
string message
```
- 设置队列运动模式开关
- status: true 开启队列运动，false 关闭队列运动
- 返回: success/message

### 设置关节零点SetAxisZeroPos_srv
```
int32 axis
---
bool success
string message
```
- 设置轴零位
- axis: 轴编号
- 返回: success/message

### 控制器有线网口IP设置SetControllerIP_srv
```
string name
string addr
string gateway
string dns
---
bool success
string message
```
- 配置控制器网络参数
- name: 网络接口名称
- addr: IP 地址
- gateway: 网关地址
- dns: DNS 服务器地址
- 返回: success/message

### 设置坐标系编号SetCoordNum_srv
```
int32 tool_num
int32 user_num
---
bool success
string message
```
- 设置工具坐标系和用户坐标系编号
- tool_num: 工具坐标系编号
- user_num: 用户坐标系编号
- 返回: success/message

### 设置当前坐标系SetCurrentCoord_srv
```
int32 coord
---
bool success
string message
```
- 设置当前坐标系
- coord: 0=关节 1=基坐标 2=工具 3=用户
- 返回: success/message

### 设置当前运行模式SetCurrentMode_srv
```
int32 mode
---
bool success
string message
```
- 设置当前工作模式
- mode: 工作模式
- 返回: success/message

### 设置数字输出SetDigitalOutput_srv
```
int32 port
int32 value
---
bool success
string message
```
- 设置数字输出口值
- port: 输出端口号
- value: 输出值（0 或 1）
- 返回: success/message

### 设置拖拽模式SetDragMode_srv
```
int32 mode
---
bool success
string message
```
- 设置拖拽（示教）模式
- mode: 0=无 1=3D鼠标 2=力矩模式 3=位置模式
- 返回: success/message

### 设置运行速度SetSpeed_srv
```
float64 speed
---
bool success
string message
```
- 设置全局速度
- speed: 全局运行速度百分比，单位 %（0<speed≤100）
- 返回: success/message

### 设置工具手参数SetToolParam_srv
```
int32 tool_num
tl_ros2_interface/ToolParam param
---
bool success
string message
```
- 设置工具参数（TCP、负载）
- tool_num: 工具编号
- param: 工具参数（位姿、负载等）
- 返回: success/message

### 设置用户坐标系SetUserCoord_srv
```
int32 user_num
tl_ros2_interface/CartesianPose pos
---
bool success
string message
```
- 设置用户坐标系的笛卡尔位姿
- user_num: 用户坐标系编号
- pos: 笛卡尔位姿（position m、rpy rad；arm_angle 未使用）
- 返回: success/message

### 拖拽轨迹回放TrackPlayback_srv
```
int32 vel
---
bool success
string message
```
- 回放轨迹
- vel: 回放速度百分比（1～100；SDK 未标注单位）
- 返回: success/message

### TrackSave_srv
```
string traj_name
---
bool success
string message
```
- 保存轨迹
- traj_name: 轨迹名称
- 返回: success/message

### 查询当前运行模式GetCurrentMode_srv
```

---
bool success
string message
int32 mode
```
- 查询当前运行模式
- 返回: success/message/mode（当前模式：0-示教 1-远程 2-运行）

## tl_ros2_interface动作（action）说明

### 运动Move_action
```
tl_ros2_interface/MoveCommand command
float64 timeout_sec
---
bool success
string message
---
float64 progress
tl_ros2_interface/CartesianPose current_pose
string state
```
- goal: `command` 为 `MoveCommand`，`target_pos_value` 必须 14 位、`target_pos_type` 必须 0；`timeout_sec<=0` 使用节点默认超时
- feedback: `progress`（0–1，坐标系 2/3 无法估算时为 -1）、`current_pose`（MCS 状态流）、`state`（RUNNING/STOP/PAUSE）
- result: `success`/`message`
- cancel：控制器 SDK 无通用停止接口，驱动一律拒绝，不会返回“取消成功”
- 并发：刚通过队列运动、轨迹回放等服务下发运动后 500 ms 内的 goal 会被拒绝
- 完成校验：`move_joint` 仅接受 `coord=0`、`move_linear` 仅接受 `coord=1`；前者用 ACS、后者用 MCS 校验，到达容差 1° / 2 mm

### 轨迹回放PlayTrack_action
```
int32 vel
float64 timeout_sec
---
bool success
string message
---
float64 progress
string state
```
- goal: `vel` 回放速度百分比 1–100
- feedback: `progress` 固定 -1（控制器不提供回放进度）、`state`
- result: `success`/`message`
- cancel：同样拒绝（无可用的回放停止接口）

## 说明与后续
- 若需更详细的字段含义或示例用法，可提交 issue 或说明需要的具体消息/服务，文档将补充字段示例与使用场景。

## 联系
如有问题，请在仓库 issue 提出或联系包维护者。
