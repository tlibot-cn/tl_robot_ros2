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
* 4.3 [作业文件名JobFileName_msg](#作业文件名JobFileName_msg)
* 4.4 [ModbusMaster参数ModbusMasterParam_msg](#ModbusMaster参数ModbusMasterParam_msg)
* 4.5 [ModbusRTU参数ModbusRTUParam_msg](#ModbusRTU参数ModbusRTUParam_msg)
* 4.6 [ModbusTCP参数ModbusTCPParam_msg](#ModbusTCP参数ModbusTCPParam_msg)
* 4.7 [运动指令MoveCommand_msg](#运动指令MoveCommand_msg)
* 4.8 [目标检测结果ObjectInfo_msg](#目标检测结果ObjectInfo_msg)
* 4.9 [机器人DH参数RobotDHParam_msg](#机器人DH参数RobotDHParam_msg)
* 4.10 [机械臂关节参数RobotJointParam_msg](#机械臂关节参数RobotJointParam_msg)
* 4.11 [工具参数ToolParam_msg](#工具参数ToolParam_msg)
* 5 [tl_ros2_interface服务（srv）说明](#tl_ros2_interface服务srv说明)
* 5.1 [坐标转换CoordTransform_srv](#坐标转换CoordTransform_srv)
* 5.2 [所有作业文件名GetAllJobFileName_srv](#所有作业文件名GetAllJobFileName_srv)
* 5.3 [坐标系编号GetCoordNum_srv](#坐标系编号GetCoordNum_srv)
* 5.4 [当前坐标系GetCurrentCoord_srv](#当前坐标系GetCurrentCoord_srv)
* 5.5 [机器人DH参数GetDHParam_srv](#机器人DH参数GetDHParam_srv)
* 5.6 [数字输入输出状态GetDigitalInputOutput_srv](#数字输入输出状态GetDigitalInputOutput_srv)
* 5.7 [全局路点GetGlobalPos_srv](#全局路点GetGlobalPos_srv)
* 5.8 [关节软件版本号GetJointSoftwareVersion_srv](#关节软件版本号GetJointSoftwareVersion_srv)
* 5.9 [关节温度GetJointTemperature_srv](#关节温度GetJointTemperature_srv)
* 5.10 [关节电压GetJointVoltage_srv](#关节电压GetJointVoltage_srv)
* 5.11 [电机温度GetMotorCurrent_srv](#电机温度GetMotorCurrent_srv)
* 5.12 [目标位姿可达状态GetPosReachable_srv](#目标位姿可达状态GetPosReachable_srv)
* 5.13 [位姿转换GetPosTransform_srv](#位姿转换GetPosTransform_srv)
* 5.14 [关节参数GetRobotJointParam_srv](#关节参数GetRobotJointParam_srv)
* 5.15 [机械臂运行状态GetRobotState_srv](#机械臂运行状态GetRobotState_srv)
* 5.16 [获取机械臂运行速度GetSpeed_srv](#获取机械臂运行速度GetSpeed_srv)
* 5.17 [插入运动指令JobInsertMove_srv](#插入运动指令JobInsertMove_srv)
* 5.18 [运行指定作业文件JobRun_srv](#运行指定作业文件JobRun_srv)
* 5.19 [机械臂点动Jogging_srv](#机械臂点动Jogging_srv)
* 5.20 [下载日志LogDownload_srv](#下载日志LogDownload_srv)
* 5.21 [写ModbusModbusWrite_srv](#写ModbusModbusWrite_srv)
* 5.22 [读ModbusModbusRead_srv](#读ModbusModbusRead_srv)
* 5.23 [关节跟踪OpenServoJ_srv](#关节跟踪OpenServoJ_srv)
* 5.24 [MoveJ队列运动QueueMotionMoveJ_srv](#MoveJ队列运动QueueMotionMoveJ_srv)
* 5.25 [MoveJ队列运动模式QueueMotionSetStatus_srv](#MoveJ队列运动模式QueueMotionSetStatus_srv)
* 5.26 [机械臂默认DH参数RestoreDefaultDHParam_srv](#机械臂默认DH参数RestoreDefaultDHParam_srv)
* 5.27 [设置关节零点SetAxisZeroPos_srv](#设置关节零点SetAxisZeroPos_srv)
* 5.28 [控制器有线网口IP设置SetControllerIP_srv](#控制器有线网口IP设置SetControllerIP_srv)
* 5.29 [设置坐标系编号SetCoordNum_srv](#设置坐标系编号SetCoordNum_srv)
* 5.30 [设置当前坐标系SetCurrentCoord_srv](#设置当前坐标系SetCurrentCoord_srv)
* 5.31 [设置当前运行模式SetCurrentMode_srv](#设置当前运行模式SetCurrentMode_srv)
* 5.32 [设置机械臂DH参数SetDHParam_srv](#设置机械臂DH参数SetDHParam_srv)
* 5.33 [设置数字输出SetDigitalOutput_srv](#设置数字输出SetDigitalOutput_srv)
* 5.34 [设置拖拽模式SetDragMode_srv](#设置拖拽模式SetDragMode_srv)
* 5.35 [设置全局位点SetGlobalPos_srv](#设置全局位点SetGlobalPos_srv)
* 5.36 [设置关节参数SetRobotJointParam_srv](#设置关节参数SetRobotJointParam_srv)
* 5.37 [设置运行速度SetSpeed_srv](#设置运行速度SetSpeed_srv)
* 5.38 [设置工具手参数SetToolParam_srv](#设置工具手参数SetToolParam_srv)
* 5.39 [设置用户坐标系SetUserCoord_srv](#设置用户坐标系SetUserCoord_srv)
* 5.40 [工具手参数标定ToolHandCalib_srv](#工具手参数标定ToolHandCalib_srv)
* 5.41 [拖拽轨迹回放TrackPlayback_srv](#拖拽轨迹回放TrackPlayback_srv)
* 5.42 [拖拽轨迹保存TrackSave_srv](#拖拽轨迹保存TrackSave_srv)
* 5.43 [当前电机力矩GetCurrentMotorTorque_srv](#当前电机力矩GetCurrentMotorTorque_srv)
* 5.44 [当前线速度和关节速度GetCurrentLineJointSpeed_srv](#当前线速度和关节速度GetCurrentLineJointSpeed_srv)
* 5.45 [查询当前运行模式GetCurrentMode_srv](#查询当前运行模式GetCurrentMode_srv)

## tl_ros2_interface功能包说明
tl_ros2_interface 功能包为 TL 系列机械臂在 ROS2 框架下提供消息（msg）和服务（srv）接口定义，供上层驱动或应用调用。该包本身没有可执行程序，主要作用为定义协议数据结构和服务接口。

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
│   ├── JobFileName.msg
│   ├── ModbusMasterParam.msg
│   ├── ModbusRTUParam.msg
│   ├── ModbusTCPParam.msg
│   ├── MoveCommand.msg
│   ├── ObjectInfo.msg
│   ├── RobotDHParam.msg
│   ├── RobotJointParam.msg
│   └── ToolParam.msg
├── srv/                  # 服务定义
│   ├── CoordTransform.srv
│   ├── GetAllJobFileName.srv
│   ├── GetCoordNum.srv
│   ├── GetCurrentCoord.srv
│   ├── GetCurrentLineJointSpeed.srv
│   ├── GetCurrentMode.srv
│   ├── GetCurrentMotorTorque.srv
│   ├── GetDHParam.srv
│   ├── GetDigitalInputOutput.srv
│   ├── GetGlobalPos.srv
│   ├── GetJointSoftwareVersion.srv
│   ├── GetJointTemperature.srv
│   ├── GetJointVoltage.srv
│   ├── GetMotorCurrent.srv
│   ├── GetPosReachable.srv
│   ├── GetPosTransform.srv
│   ├── GetRobotJointParam.srv
│   ├── GetRobotState.srv
│   ├── GetSpeed.srv
│   ├── JobRun.srv
│   ├── JobInsertMove.srv
│   ├── Jogging.srv
│   ├── LogDownload.srv
│   ├── ModbusRead.srv
│   ├── ModbusWrite.srv
│   ├── OpenServoJ.srv
│   ├── QueueMotionMoveJ.srv
│   ├── QueueMotionSetStatus.srv
│   ├── RestoreDefaultDHParam.srv
│   ├── SetAxisZeroPos.srv
│   ├── SetControllerIP.srv
│   ├── SetCoordNum.srv
│   ├── SetCurrentCoord.srv
│   ├── SetCurrentMode.srv
│   ├── SetDHParam.srv
│   ├── SetDigitalOutput.srv
│   ├── SetDragMode.srv
│   ├── SetGlobalPos.srv
│   ├── SetRobotJointParam.srv
│   ├── SetSpeed.srv
│   ├── SetToolParam.srv
│   ├── SetUserCoord.srv
│   ├── ToolHandCalib.srv
│   ├── TrackPlayback.srv
│   ├── TrackSave.srv
```

## tl_ros2_interface消息（msg）说明
以下列出 msg 文件定义及字段说明。

### 机械臂运行状态ArmStatus_msg
```
builtin_interfaces/Time stamp
string run_state
```
__msg成员__
- stamp: 时间戳
- run_state: 运行状态描述字符串

### 直角坐标系位姿CartesianPose_msg
```
std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Vector3 rpy
float64 arm_angle
```
__msg成员__
- header: 标准消息头
- position: x,y,z 位置（米）
- rpy: 姿态欧拉角（弧度）
- arm_angle: 机械臂额外角度（弧度）

### 作业文件名JobFileName_msg
```
string[] file_name
```
__msg成员__
- file_name: 作业文件名数组

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
- target_pos_value: 目标位置数值数组（关节或笛卡尔）
- target_pos_name: 目标位置名称
- target_pos_type: 位置类型标识
- coord: 坐标系编号
- velocity/velocity_sync/acc/dec: 速度与加减速参数
- pl/time: 轨迹/时间相关参数
- tool_num/user_num: 工具/用户坐标编号
- posidtype/configuration/spin/para_sync: 其他标志位或配置

### 目标检测结果ObjectInfo_msg
```
string type
geometry_msgs/PointStamped pos
```
__msg成员__
- type: 目标类型
- pos: 检测到的目标位置（带时间戳）

### 机器人DH参数RobotDHParam_msg
```
float64 l1
float64 l2
float64 l3
float64 l4
float64 l5
float64 l6
float64 l7
float64 l8
float64 l9
float64 l10
float64 l11
float64 l12
float64 l13
float64 l14
float64 l15
float64 l16
float64 l17
float64 l18
float64 l19
float64 l20

float64 couple_coe_1_2
float64 couple_coe_2_3
float64 couple_coe_3_2
float64 couple_coe_3_4
float64 couple_coe_4_5
float64 couple_coe_4_6
float64 couple_coe_5_6

float64 dynamic_limit_max
float64 dynamic_limit_min

float64 pitch
float64 sliding_lead_value
float64 uplift_lead_value
float64 spray_distance

float64 three_axis_direction
float64 five_axis_direction

float64	two_axis_convertion_ratio
float64 three_axis_convertion_ratio
float64 amplification_ratio

float64 convertion_ratio_x
float64 convertion_ratio_y
float64 convertion_ratio_z

float64 convertion_ratio_j1
float64 convertion_ratio_j2
float64 convertion_ratio_j3

int32 upside_down

float64 pc
float64[] sp
float64[] tl
```
__msg成员__
- l1..l20: 连杆长度参数
- couple_coe_*: 联动系数
- dynamic_limit_*: 动态限制
- pitch: 螺距
- sliding_lead_value: 滑动电动缸导程，酒槽机型用
- uplift_lead_value: 顶升电动缸导程，酒槽机型用
- spray_distance: 喷料距离，酒槽机型用
- three_axis_direction: 3轴方向
- five_axis_direction: 5轴方向
- two_axis_convertion_ratio/three_axis_convertion_ratio/amplification_ratio: 转换比
- convertion_ratio_x/convertion_ratio_y/convertion_ratio_z: 三轴转换比
- convertion_ratio_j1/convertion_ratio_j2/convertion_ratio_j3: 关节转换比
- upside_down: 反向标志
- pc/sp/tl: 其他参数数组

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
- pos_sw_limit: 轴正限位
- neg_sw_limit: 轴反限位
- rated_rot_speed: 电机额定正转速
- rated_derot_speed: 电机额定反转速
- max_rot_speed: 电机最大正转速
- max_derot_speed: 电机最大反转速
- rated_vel: 额定正速度
- rated_devel: 额定反速度
- max_acc: 最大加速度
- max_dec: 最大减速度
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
- x: X轴偏移方向
- y: Y轴偏移方向
- z: Z轴偏移方向
- a: 绕A轴旋转
- b: 绕B轴旋转
- c: 绕C轴旋转
- payload_mass: 负载质量
- payload_inertia: 负载惯性
- payload_mass_center_x: 负载质心X
- payload_mass_center_y: 负载质心Y
- payload_mass_center_z: 负载质心Z

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
- origin_coord/target_coord: 源/目标坐标系编号
- form: 转换模式
- origin_pos/reference_pos: 输入位置与参考位置
- 返回: success/message/target_pos

### 所有作业文件名GetAllJobFileName_srv
```
---
bool success
string message
tl_ros2_interface/JobFileName[] robots_file
```
- 返回所有作业文件名列表
- 返回: success/message/robots_file（作业文件名列表）

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
- 返回数字输入输出状态
- 返回: success/message/input（数字输入状态数组）/output（数字输出状态数组）

### 全局路点GetGlobalPos_srv
```
string pos_name
---
bool success
string message
float64[] pos
```
- 输入 pos_name，返回对应全局路点
- pos_name: 全局位置点名称
- 返回: success/message/pos（位置坐标数组）

### 关节软件版本号GetJointSoftwareVersion_srv
```
int32 axis_num
---
bool success
string message
```
- 查询指定轴的软件版本（返回 message）
- axis_num: 轴编号
- 返回: success/message（软件版本信息）

### 关节温度GetJointTemperature_srv
```
---
bool success
string message
float64[] temperatures
```
- 返回各关节温度数组
- 返回: success/message/temperatures（各关节温度数组）

### 关节电压GetJointVoltage_srv
```
---
bool success
string message
float64[] joint_voltage
float64[] positioner_voltage
```
- 返回关节电压数组
- 返回: success/message/joint_voltage（关节电压数组）/positioner_voltage（定位器电压数组）

### 电机温度GetMotorCurrent_srv
```
---
bool success
string message
float64[] motor_current
```
- 返回电机电流数组
- 返回: success/message/motor_current（电机电流数组）

### 目标位姿可达状态GetPosReachable_srv
```
float64[] pos
string move_type
---
bool success
string message
```
- 输入目标位姿，返回是否可达
- pos: 目标位姿数组
- move_type: 运动类型（如关节运动、直线运动等）
- 返回: success/message

### 位姿转换GetPosTransform_srv
```
float64[] input
---
bool success
string message
float64[] output
```
- 输入位姿，返回转换后的位姿
- input: 输入位姿数组
- 返回: success/message/output（转换后的位姿数组）

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

### 机械臂运行状态GetRobotState_srv
```
int32 channel
bool stop
int32 mode
int32 interval
bool io_state
int32 position
bool detail_motion_pos
int32 pos_sum
string[] io_port
string[] optional
---
bool success
string message
```
- 获取机械臂运行相关状态，包含 IO 与运动详情
- channel: 通道号
- stop: 是否停止
- mode: 模式
- interval: 查询间隔
- io_state: 是否返回 IO 状态
- position: 位置类型
- detail_motion_pos: 是否返回详细运动位置
- pos_sum: 位置数量
- io_port: 指定 IO 端口列表
- optional: 其他可选参数
- 返回: success/message

### 获取机械臂运行速度GetSpeed_srv
```
---
bool success
string message
float64 speed
```
- 返回当前全局速度设置

### 插入运动指令JobInsertMove_srv
```
int32 line
tl_ros2_interface/MoveCommand cmd
---
bool success
string message
```
- 向作业文件插入一条运动指令
- line: 插入的行序号
- cmd: 运动指令（MoveCommand 类型）
- 返回: success/message

### 运行指定作业文件JobRun_srv
```
string job_name
---
bool success
string message
```
- 请求运行指定作业文件
- job_name: 作业文件名
- 返回: success/message

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

### 下载日志LogDownload_srv
```
int32 count
string directory_path
---
bool success
string message
```
- 请求下载日志到目录
- count: 日志条数
- directory_path: 下载保存路径
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
- vmax: 最大速度数组
- amax: 最大加速度数组
- jmax: 最大加加速度数组
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

### 机械臂默认DH参数RestoreDefaultDHParam_srv
```
int32 robot_num
---
bool success
string message
```
- 恢复默认 DH 参数
- robot_num: 机器人编号
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
- coord: 坐标系编号
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

### 设置机械臂DH参数SetDHParam_srv
```
tl_ros2_interface/RobotDHParam param
---
bool success
string message
```
- 设置机器人 DH 参数
- param: DH 参数
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
- mode: 拖拽模式（0 关闭，1 开启）
- 返回: success/message

### 设置全局位点SetGlobalPos_srv
```
string pos_name
float64[] pos_info
---
bool success
string message
```
- 设置/保存全局位置点
- pos_name: 位置点名称
- pos_info: 位置坐标数组
- 返回: success/message

### 设置关节参数SetRobotJointParam_srv
```
int32 id
tl_ros2_interface/RobotJointParam param
---
bool success
string message
```
- 设置指定关节参数
- id: 关节编号
- param: 关节参数
- 返回: success/message

### 设置运行速度SetSpeed_srv
```
float64 speed
---
bool success
string message
```
- 设置全局速度
- speed: 速度值（百分比或比例）
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
- pos: 笛卡尔位姿
- 返回: success/message

### 工具手参数标定ToolHandCalib_srv
```
int32 tool_num
int32 point_num
---
bool success
string message
```
- 工具手标定
- tool_num: 工具编号
- point_num: 标定点数
- 返回: success/message

### 拖拽轨迹回放TrackPlayback_srv
```
int32 vel
---
bool success
string message
```
- 回放轨迹
- vel: 回放速度
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

### 当前电机力矩GetCurrentMotorTorque_srv
```

---
bool success
string message
int32[] motor_torque
int32[] motor_torque_sync
```
- 查询当前电机力矩
- 返回: success/message/motor_torque（当前电机力矩）/motor_torque_sync（同步轴电机力矩）

### 当前线速度和关节速度GetCurrentLineJointSpeed_srv
```

---
bool success
string message
float64 line_speed
float64[] joint_speed
float64[] joint_speed_sync
```
- 查询当前线速度和关节速度
- 返回: success/message/line_speed（当前线速度）/joint_speed（关节速度）/joint_speed_sync（同步轴关节速度）

### 查询当前运行模式GetCurrentMode_srv
```

---
bool success
string message
int32 mode
```
- 查询当前运行模式
- 返回: success/message/mode（当前模式：0-示教 1-远程 2-运行）

## 说明与后续
- 若需更详细的字段含义或示例用法，可提交 issue 或说明需要的具体消息/服务，文档将补充字段示例与使用场景。

## 联系
如有问题，请在仓库 issue 提出或联系包维护者。
