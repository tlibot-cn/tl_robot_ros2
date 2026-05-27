<div align="center">

# tl_driver服务与话题说明书

文件修订记录：

|版本号 | 时间 | 备注 |
| :---: | :---- | :---: |
|V1.0 | 2026-4-29 | 拟制 |
|V1.1 | 2024-5-9  | 修订（添加[查询机械臂状态](#查询机械臂状态)、[查询库版本信息](#查询库版本信息)、[查询关节参数](#查询关节参数)、[设置关节参数](#设置关节参数)、[查询关节温度](#查询关节温度)、[查询关节电压](#查询关节电压)、[查询电机电流](#查询电机电流)、[查询关节软件版本号](#查询关节软件版本号)、<br>[查询算法库版本](#查询算法库版本)、[设置机械臂默认DH参数](#设置机械臂默认DH参数)、[设置机械臂默认笛卡尔参数](#设置机械臂默认笛卡尔参数)、[日志下载](#日志下载)、[查询运行速度](#查询运行速度)、[设置坐标系编号](#设置坐标系编号)、[设置机械臂DH参数](#设置机械臂DH参数)、<br>[四元数转欧拉角](#四元数转欧拉角)、[欧拉角转四元数](#欧拉角转四元数)、[欧拉角转旋转矩阵](#欧拉角转旋转矩阵)、[位姿转旋转矩阵](#位姿转旋转矩阵)、[旋转矩阵转位姿](#旋转矩阵转位姿)、[旋转矩阵转位姿](#旋转矩阵转位姿)、[设置控制器有线网口IP](#设置控制器有线网口IP)、<br>[查询控制器序列号ID](#查询控制器序列号ID)、[查询当前坐标系](#查询当前坐标系)等接口|

</div>

## 目录
* 1[连接管理接口](#连接管理接口)
* 1.1[机械臂连接](#机械臂连接)
* 1.2[机械臂断开连接](#机械臂断开连接)
* 1.3[机械臂上电](#机械臂上电)
* 1.4[机械臂下电](#机械臂下电)
* 2[日志管理接口](#日志管理接口)
* 2.1[日志下载](#日志下载)
* 3[信息查询接口](#信息查询接口)
* 3.1[查询关节角度](#查询关节角度)
* 3.2[查询末端位姿](#查询末端位姿)
* 3.3[查询运行速度](#查询运行速度)
* 3.4[查询控制器序列号ID](#查询控制器序列号ID)
* 3.5[查询机械臂状态](#查询机械臂状态)
* 3.6[查询库版本信息](#查询库版本信息)
* 3.7[查询关节参数](#查询关节参数)
* 3.8[查询关节温度](#查询关节温度)
* 3.9[查询关节电压](#查询关节电压)
* 3.10[查询电机电流](#查询电机电流)
* 3.11[查询关节软件版本号](#查询关节软件版本号)
* 3.12[查询算法库版本](#查询算法库版本)
* 3.13[查询当前坐标系](#查询当前坐标系)
* 3.14[查询坐标系编号](#查询坐标系编号)
* 3.15[查询机械臂DH参数](#查询机械臂DH参数)
* 3.16[查询所有作业文件名称](#查询所有作业文件名称)
* 3.17[查询目标位姿可达状态](#查询目标位姿可达状态)
* 4[机械臂基础功能设置接口](#机械臂基础功能设置接口)
* 4.1[设置运行速度](#设置运行速度)
* 4.2[设置控制器有线网口IP](#设置控制器有线网口IP)
* 4.3[设置关节参数](#设置关节参数)
* 4.4[设置机械臂默认DH参数](#设置机械臂默认DH参数)
* 4.5[设置机械臂默认笛卡尔参数](#设置机械臂默认笛卡尔参数)
* 4.6[坐标转换](#坐标转换)
* 4.7[设置机械臂DH参数](#设置机械臂DH参数)
* 5[作业运动控制接口](#作业运动控制接口)
* 5.1[向作业文件插入一条moveJ关节运动](#向作业文件插入一条moveJ关节运动)
* 5.2[向作业文件插入一条moveL关节运动](#向作业文件插入一条moveL关节运动)
* 5.3[向作业文件插入一条增量指令IMove](#向作业文件插入一条增量指令IMove)
* 5.4[向作业文件插入一条moveC关节运动](#向作业文件插入一条moveC关节运动)
* 5.5[运行作业文件](#运行作业文件)
* 6[实时运动控制接口](#实时运动控制接口)
* 6.1[MoveJ运动控制](#MoveJ运动控制)
* 6.2[MoveL运动控制](#MoveL运动控制)
* 7[坐标系与工具管理接口](#坐标系与工具管理接口)
* 7.1[设置工具手参数](#设置工具手参数)
* 7.2[工具手参数标定](#工具手参数标定)
* 7.3[设置用户坐标系](#设置用户坐标系)
* 7.4[设置坐标系编号](#设置坐标系编号)
* 7.5[设置当前坐标系](#设置当前坐标系)
* 8[示教操作接口](#示教操作接口)
* 8.1[开始点动](#开始点动)
* 8.2[停止点动](#停止点动)
* 8.3[设置拖拽模式](#设置拖拽模式)
* 8.4[查看拖拽状态](#查看拖拽状态)
* 8.5[拖拽轨迹保存](#拖拽轨迹保存)
* 8.6[拖拽轨迹回放](#拖拽轨迹回放)
* 9[错误处理与零位标定接口](#错误处理与零位标定接口)
* 9.1[清除错误](#清除错误)
* 9.2[设置关节零点](#设置关节零点)
* 10[全局路点管理接口](#全局路点管理接口)
* 10.1[查询全局位点](#查询全局位点)
* 10.2[设置全局位点](#设置全局位点)
* 11[模式与IO控制接口](#模式与IO控制接口)
* 11.1[设置当前运行模式](#设置当前运行模式)
* 11.2[设置数字输出](#设置数字输出)
* 11.3[查询数字输入输出状态](#查询数字输入输出状态)
* 12[modbus通信接口](#modbus通信接口)
* 12.1[写Modbus](#写Modbus)
* 12.2[读Modbus](#读Modbus)
* 13[队列运动接口](#队列运动接口)
* 13.1[设置MoveJ队列运动模式](#设置MoveJ队列运动模式)
* 13.2[MoveJ队列运动](#MoveJ队列运动)
* 13.3[停止MoveJ队列运动模式](#停止MoveJ队列运动模式)
* 14[7000端口](#7000端口)
* 14.1[打开关节跟踪模式](#打开关节跟踪模式)
* 14.2[关闭关节跟踪模式](#关闭关节跟踪模式)
* 14.3[发送跟踪关节位置](#发送跟踪关节位置)
* 15[位姿转换工具接口](#矩阵转换工具接口)
* 15.1[四元数转欧拉角](#四元数转欧拉角)
* 15.2[欧拉角转四元数](#欧拉角转四元数)
* 15.3[欧拉角转旋转矩阵](#欧拉角转旋转矩阵)
* 15.4[位姿转旋转矩阵](#位姿转旋转矩阵)
* 15.5[旋转矩阵转位姿](#旋转矩阵转位姿)

## 连接管理接口
### 机械臂连接
| 功能描述 | 建立机械臂网络通信连接 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-连接成功，false-连接失败 |
#### 命令示例
```
ros2 service call /tl_driver/connect_arm std_srvs/srv/Trigger "{}"
```
### 机械臂断开连接
| 功能描述 | 断开机械臂网络通信连接 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-断开连接成功，false-断开连接失败 |
#### 命令示例
```
ros2 service call /tl_driver/disconnect_arm std_srvs/srv/Trigger "{}" 
```
### 机械臂上电
| 功能描述 | 机械臂使能上电 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-上电成功，false-上电失败 |
#### 命令示例
```
ros2 service call /tl_driver/power_on std_srvs/srv/Trigger "{}"
```
### 机械臂下电
| 功能描述 | 机械臂使能下电 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-下电成功，false-下电失败 |
#### 命令示例
```
ros2 service call /tl_driver/power_off std_srvs/srv/Trigger "{}"
```
## 日志管理接口
### 日志下载
| 功能描述 | 下载指定数量的日志到指定文件夹 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | LogDownload.srv<br>int32 count：日志数量<br>string directory_path：保存目录路径 |
| 返回值 | true-下载成功，false-下载失败 |
#### 命令示例
```
ros2 service call /tl_driver/log_download tl_ros2_interface/srv/LogDownload "{count: 1, directory_path: '/home/ubuntu/桌面'}"
```
## 信息查询接口
### 查询关节角度
| 功能描述 | 查询关节角度 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 参数说明 | 无参数 |
| 返回值 | 关节角度（sensor_msgs::msg::JointState） |
#### 命令示例
```
ros2 topic echo /joint_states
```
### 查询末端位姿
| 功能描述 | 查询机械臂末端位置 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 参数说明 | 无参数 |
| 返回值 | 末端位姿（CartesianPose.msg） |
#### 命令示例
```
ros2 topic echo /tcp_pose
```
### 查询运行速度
| 功能描述 | 查询机械臂运行速度 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetSpeed.srv<br> |
| 返回值 | true-查询成功，false-查询失败<br>float64 speed：运行速度 |
#### 命令示例
```
ros2 service call /tl_driver/get_speed tl_ros2_interface/srv/GetSpeed "{}"
```
### 查询控制器序列号ID
| 功能描述 | 查询控制器序列号ID |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-查询成功，false-查询失败<br>查询成功时返回控制器序列号ID|
#### 命令示例
```
ros2 service call /tl_driver/get_controller_id std_srvs/srv/Trigger "{}" 
```
### 查询机械臂状态
| 功能描述 | 查询机械臂详细状态 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetRobotState.srv<br>int32 channel：查询通道<br>bool stop：是否停止发送<br>int32 mode：查询模式（0-只回复一次  1-持续回复）<br>int32 interval：仅mode = 1时有效，回复时间范围 [10,60000] ms<br>bool io_state：查询IO<br>int32 position：0-关节坐标  1-直角坐标<br>bool detail_motion_pos：机械臂的运动点位<br>int32 pos_sum：当查询机械臂运动点位时，posSum为每帧数据回复的点位数目<br>string[] io_port：IO端口，可查询的最大数量不可大于IO实际个数 例子:[ “DI1”, “DI16”, “DO1”, “DO3”, “DO17”]<br>string[] optional：查询运动点位返回的坐标类型  "ACS"-关节参数 "MCS"-直角参数 "time"-时间戳 "reset"-重置点位记录|
| 返回值 | true-查询成功，false-查询失败<br>查询成功时返回对应的查询信息|
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
### 查询库版本信息
| 功能描述 | 查询API库版本相关信息 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-查询成功，false-查询失败<br>查询成功时返回库版本信息 |
#### 命令示例
```
ros2 service call /tl_driver/get_library_version std_srvs/srv/Trigger "{}"
```
### 查询关节参数
| 功能描述 | 查询机械臂关节参数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetRobotJointParam.srv<br>int32 id：关节序号 |
| 返回值 | true-查询成功，false-查询失败<br>查询成功时返回关节参数<br>RobotJointParam.msg（机械臂关节信息）<br>float64 reduction_ratio：关节减速比<br>int32 encoder_resolution：编码器位数<br>float64 pos_sw_limit：轴正限位<br>float64 neg_sw_limit：轴反限位<br>float64 rated_rot_speed：电机额定正转速<br>float64 rated_derot_speed：电机额定反转速<br>float64 max_rot_speed：电机最大正转速<br>float64 max_derot_speed：电机最大反转速<br>float64 rated_vel：额定正速度<br>float64 rated_devel：额定反速度<br>float64 max_acc：最大加速度<br>float64 max_deacc：最大减速度<br>int32 direction：模型方向 |
#### 命令示例
```
ros2 service call /tl_driver/get_robot_joint_param tl_ros2_interface/srv/GetRobotJointParam "{id: 1}"
```
### 查询关节温度
| 功能描述 | 查询关节温度 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetJointTemperature.srv |
| 返回值 | true-查询成功，false-查询失败<br>查询成功时返回各个关节温度<br>float64[] temperatures：各个关节温度 |
#### 命令示例
```
ros2 service call /tl_driver/get_joint_temperature tl_ros2_interface/srv/GetJointTemperature "{}"
```
### 查询关节电压
| 功能描述 | 查询关节电压 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetJointVoltage.srv |
| 返回值 | true-查询成功，false-查询失败<br>查询成功时返回各个关节电压<br>float64[] joint_voltage：机械臂本体各关节电压<br>float64[] positioner_voltage：外部轴各关节电压 |
#### 命令示例
```
ros2 service call /tl_driver/get_joint_voltage tl_ros2_interface/srv/GetJointVoltage "{}"
```
### 查询电机电流
| 功能描述 | 查询独立轴当前电机电流 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetMotorCurrent.srv |
| 返回值 | true-查询成功，false-查询失败<br>查询成功时返回各个独立轴电机电流<br>float64[] current_motor：机械臂独立轴电机电流 |
#### 命令示例
```
ros2 service call /tl_driver/get_motor_current tl_ros2_interface/srv/GetMotorCurrent "{}"
```
### 查询关节软件版本号
| 功能描述 | 查询指定关节（轴）软件版本号 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetJointSoftwareVersion.srv<br>int32 axis_num：关节（轴）号 |
| 返回值 | true-查询成功，false-查询失败<br>查询成功时返回指定关节软件版本号 |
#### 命令示例
```
ros2 service call /tl_driver/get_joint_software_version tl_ros2_interface/srv/GetJointSoftwareVersion "{axis_num: 1}"
```
### 查询算法库版本
| 功能描述 | 查询算法库版本信息 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-查询成功，false-查询失败<br>查询成功时返回算法库版本信息 |
#### 命令示例
```
ros2 service call /tl_driver/get_nexmotion_lib_version std_srvs/srv/Trigger "{}"
```
### 查询当前坐标系
| 功能描述 | 查询当前坐标系 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetCurrentCoord.srv<br> |
| 返回值 | true-查询成功，false-查询失败<br>int32 coord：坐标系序号（0-关节  1-直角  2-工具  3-用户） |
#### 命令示例
```
ros2 service call /tl_driver/get_current_coord tl_ros2_interface/srv/GetCurrentCoord "{}"
```
### 查询坐标系编号
| 功能描述 | 查询工具坐标系和用户坐标系编号 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetCoordNum.srv |
| 返回值 | true-查询成功，false-查询失败<br>int32 tool_num：工具坐标系序号<br>int32 user_num：用户坐标系序号|
```
ros2 service call /tl_driver/get_coord_num tl_ros2_interface/srv/GetCoordNum "{}"
```
### 查询机械臂DH参数
| 功能描述 | 查询机械臂DH参数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetDHParam.srv|
| 返回值 | true-查询成功，false-查询失败<br>RobotDHParam.msg|
#### 命令示例
```
ros2 service call /tl_driver/get_dh_param tl_ros2_interface/srv/GetDHParam
```
### 查询所有作业文件名称
| 功能描述 | 查询所有作业文件名称 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetAllJobFileName.srv|
| 返回值 | true-查询成功，false-查询失败<br>JobFileName.msg（作业文件名称参数）<br>string[] file_name：作业文件名称|
#### 命令示例
```
ros2 service call /tl_driver/get_all_job_filename tl_ros2_interface/srv/GetAllJobFileName "{}"
```
### 查询目标位姿可达状态
| 功能描述 | 查询目标位姿可达状态 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetPosReachable.srv<br>float64[] pos：查询位姿<br>string move_type：运动方式|
| 返回值 | true-目标位姿可达，false-目标位姿不可达|
#### 命令示例
```
ros2 service call /tl_driver/get_pos_reachable tl_ros2_interface/srv/GetPosReachable \
'{
    pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -57.14, -32.93, 19.74, -89.89, -19.77, 0.0], move_type: "MOVJ"
}'
```
## 机械臂基础功能设置接口
### 设置运行速度
| 功能描述 | 设置机械臂运行速度 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetSpeed.srv<br>float64 speed：运行速度 |
| 返回值 | true-设置成功，false-设置失败 |
#### 命令示例
```
ros2 service call /tl_driver/set_speed tl_ros2_interface/srv/SetSpeed "{speed: 20.0}"
```
### 设置控制器有线网口IP
| 功能描述 | 设置控制器有线网口IP |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetControllerIP.srv<br>string name：配置名称<br>string addr：IP地址<br>string gateway：网关<br>string dns：DNS域名 |
| 返回值 | true-设置成功，false-设置失败|
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
### 设置关节参数
| 功能描述 | 设置机械臂关节参数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetRobotJointParam.srv<br>int32 id：关节序号<br>RobotJointParam.msg<br>相关参数查阅[查询关节参数](#查询关节参数)返回值部分 |
| 返回值 | true-设置成功，false-设置失败 |
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
### 设置机械臂默认DH参数
| 功能描述 | 设置机械臂默认DH参数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | RestoreDefaultDHParam.srv |
| 返回值 | true-恢复成功，false-恢复失败 |
#### 命令示例
```
ros2 service call /tl_driver/restore_default_dh_param tl_ros2_interface/srv/RestoreDefaultDHParam "{robot_num: 1}"
```
### 设置机械臂默认笛卡尔参数
| 功能描述 | 设置机械臂默认笛卡尔参数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-设置成功，false-设置失败 |
#### 命令示例
```
ros2 service call /tl_driver/set_default_cartesian_param std_srvs/srv/Trigger "{}" 
```
### 坐标转换
| 功能描述 | 坐标系数据转换 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | CoordTransform.srv<br>int32 origin_coord：原始坐标系序号<br>int32 target_coord：目标坐标系序号<br>int32 form：形态<br>float64[] origin_pos：原始坐标系位姿<br>float64[] reference_pos：参考位姿|
| 返回值 | true-写入成功，false-写入失败<br>float64[] target_pos：目标坐标系位姿|
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
### 设置机械臂DH参数
| 功能描述 | 设置机械臂DH参数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetDHParam.srv<br>RobotDHParam.msg（机械臂DH参数）|
| 返回值 | true-设置成功，false-设置失败|
#### 命令示例
```
ros2 service call /tl_driver/set_dh_param tl_ros2_interface/srv/SetDHParam \
"{
    param:
    {
        l1: 127.5
    }
}"
```
## 作业运动控制接口
### 向作业文件插入一条moveJ关节运动
| 功能描述 | 向作业文件插入一条moveJ关节运动 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 参数说明 | JobInsertMove.msg（作业文件插入运动控制参数）<br>int32 line：插入行序号<br>MoveCommand cmd：运动指令 |
| 返回值 | 无返回值 |
#### 命令示例
targetPosType=0为自定义数组 posInfo[14] [0]坐标系 0：关节 1：直角 2：工具 3：用户 [1]:0 角度制 1弧度制 [2]形态 [3]工具手坐标序号 [4]用户坐标序号 [5][6] 备用 [7-13] 点位信息
```
ros2 topic pub --once /tl_driver/job_insert_moveJ tl_ros2_interface/msg/JobInsertMove "{
  line: 1,
  cmd: {
    target_pos_value: [
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    ],
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
  }
}"
```
targetPosType=1,需要设置targetPosName为"P0001",默认中间三个0
此时target_pos_value根据实际机械臂轴数来设定参数数量，输入关节角度
```
ros2 topic pub --once /tl_driver/job_insert_moveJ tl_ros2_interface/msg/JobInsertMove "{
  line: 1,
  cmd: {
    target_pos_value: [
      10.0, 20.0, 0.0, 0.0, 0.0, 0.0
    ],
    target_pos_name: 'P0001',
    target_pos_type: 1,
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
  }
}"

```
### 向作业文件插入一条moveL关节运动
| 功能描述 | 向作业文件插入一条moveL关节运动 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 参数说明 | JobInsertMove.msg<br>int32 line：插入行序号<br>MoveCommand cmd：运动指令 |
| 返回值 | 无返回值 |
#### 命令示例
```
ros2 topic pub --once /tl_driver/job_insert_moveL tl_ros2_interface/msg/JobInsertMove "{
  line: 1,
  cmd: {
    target_pos_value: [
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    ],
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
  }
}"
```
### 向作业文件插入一条增量指令IMove
| 功能描述 | 向作业文件插入一条增量指令IMove |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 参数说明 | JobInsertMove.msg<br>int32 line：插入行序号<br>MoveCommand cmd：运动指令 |
| 返回值 | 无返回值 |
#### 命令示例
```
ros2 topic pub --once /tl_driver/job_insert_imove tl_ros2_interface/msg/JobInsertMove "{
  line: 1,
  cmd: {
    target_pos_value: [
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    ],
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
  }
}"
```
### 向作业文件插入一条moveC关节运动
| 功能描述 | 向作业文件插入一条moveC关节运动 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 参数说明 | JobInsertMove.msg<br>int32 line：插入行序号<br>MoveCommand cmd：运动指令 |
| 返回值 | 无返回值 |
#### 命令示例
```
ros2 topic pub --once /tl_driver/job_insert_moveC tl_ros2_interface/msg/JobInsertMove "{
  line: 1,
  cmd: {
    target_pos_value: [
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    ],
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
  }
}"
```
### 运行作业文件
| 功能描述 | 运行作业文件 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | JobRun.srv<br>string job_name：作业名称|
| 返回值 | true-运行成功，false-运行失败|
#### 命令示例
```
ros2 service call /tl_driver/job_run tl_ros2_interface/srv/JobRun "{job_name: '回零点'}"
```
## 实时运动控制接口
### MoveJ运动控制
| 功能描述 | 机械臂MoveJ运动控制 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 参数说明 | MoveCommand.msg<br>float64[] target_pos_value：目标位姿<br>string target_pos_name：目标位姿名称<br>int32 target_pos_type：目标位姿类型<br>int32 coord：坐标系序号<br>float64 velocity：运行速度<br>float64 velocity_sync：速度同步<br>float64 acc：加速度<br>float64 dec：减速度<br>int32 pl：平滑度<br>int32 time：提前执行时间<br>int32 tool_num：工具坐标系编号<br>int32 user_num：用户坐标系编号<br>int32 posidtype：变量类型<br>int32 configuration：形态<br>int32 spin：MOVCA指令使用（0-姿态不变  1-六轴不转  2-六轴旋转）<br>bool para_sync：外部轴是否同步 |
| 返回值 | 无返回值 |
#### 命令示例
```
ros2 topic pub --once /tl_driver/moveJ tl_ros2_interface/msg/MoveCommand \
"{
    target_pos_value: [90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
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
### MoveL运动控制
| 功能描述 | 机械臂MoveL运动控制 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 参数说明 | MoveCommand.msg |
| 返回值 | 无返回值 |
#### 命令示例
```
ros2 topic pub --once /tl_driver/moveL tl_ros2_interface/msg/MoveCommand \
"{
    target_pos_value: [10.0, 230.0, 245.0, -3.14, 0.0, -1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    target_pos_name: '',
    target_pos_type: 0,
    coord: 1,
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
## 坐标系与工具管理接口
### 设置工具手参数
| 功能描述 | 设置工具手相关参数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetToolParam.srv<br>int32 tool_num：工具手编号<br>ToolParam.msg（工具手参数）<br>float64 x：x轴方向偏移<br>float64 y：y轴方向偏移<br>float64 z：z轴方向偏移<br>float64 a：绕a轴旋转<br>float64 b：绕b轴旋转<br>float64 c：绕c轴旋转<br>payload_mass：负载质量<br>payload_inertia：负载惯性<br>payload_mass_center_x：负载质心x<br>payload_mass_center_y：负载质心y<br>payload_mass_center_z：负载质心z|
| 返回值 | true-设置成功，false-设置失败 |
#### 命令示例
```
ros2 service call /tl_driver/set_tool_param tl_ros2_interface/srv/SetToolParam \
'{
    tool_num: 1, 
    param: 
    {
        x: 100.0, 
        y: 20.0, 
        z: 50.0, 
        a: 0.1, 
        b: 0.2, 
        c: 0.3, 
        payload_mass: 1.5, 
        payload_inertia: 0.01, 
        payload_mass_center_x: 5.0, 
        payload_mass_center_y: 5.0, 
        payload_mass_center_z: 10.0
    }
}'
```
### 工具手参数标定
| 功能描述 | 工具手参数标定 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | ToolHandCalib.srv<br>int32 tool_num：工具坐标系序号<br>int32 point_num：标定点数 |
| 返回值 | true-标定成功，false-标定失败|
#### 命令示例
```
ros2 service call /tl_driver/tool_hand_calib tl_ros2_interface/srv/ToolHandCalib "{tool_num: 3}"
```
### 设置用户坐标系
| 功能描述 | 设置用户坐标系相关参数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetUserCoord.srv<br>int32 user_num：用户坐标系编号<br>CartesianPose.msg（直接坐标系参数）<br>header：坐标系和时间戳<br>geometry_msgs/Point/position：直接坐标系位置<br>geometry_msgs/Vector3/rpy：欧拉角<br>float64 arm_angle：臂角 |
| 返回值 | true-设置成功，false-设置失败 |
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
            x: 200.0, 
            y: 100.0, 
            z: 50.0
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
### 设置坐标系编号
| 功能描述 | 设置工具坐标系和用户坐标系编号 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetCoordNum.srv<br>int32 tool_num：工具坐标系序号<br>int32 user_num：用户坐标系序号 |
| 返回值 | true-设置成功，false-设置失败 |
```
ros2 service call /tl_driver/set_coord_num tl_ros2_interface/srv/SetCoordNum "{tool_num: 1, user_num: 2}"
```
### 设置当前坐标系
| 功能描述 | 设置当前坐标系 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetCurrentCoord.srv<br>int32 coord：坐标系序号（0-关节  1-直角  2-工具  3-用户） |
| 返回值 | true-设置成功，false-设置失败 |
#### 命令示例
```
ros2 service call /tl_driver/set_current_coord tl_ros2_interface/srv/SetCurrentCoord "{coord: 1}"
```
## 示教操作接口
### 开始点动
| 功能描述 | 开始点动机械臂 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | Jogging.srv<br>int32 axis：关节轴号<br>bool direction：关节运动方向（true-正方向  false-反方向） |
| 返回值 | true-点动成功，false-点动失败 |
| 注意 | direction共有两种取值，direction == 1和direction == 0 |
#### 命令示例
```
ros2 service call /tl_driver/start_jogging tl_ros2_interface/srv/Jogging "{axis: 3, direction: 1}"
```
### 停止点动
| 功能描述 | 停止点动机械臂 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | Jogging.srv<br>int32 axis：关节轴号 |
| 返回值 | true-停止点动成功，false-停止点动失败 |
#### 命令示例
```
ros2 service call /tl_driver/stop_jogging tl_ros2_interface/srv/Jogging "{axis: 3}"
```
### 设置拖拽模式
| 功能描述 | 设置机械臂拖拽模式 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetDragMode.srv<br>int32 mode：拖拽模式（0-无  1-3D鼠标  2-力矩模式 3-位置） |
| 返回值 | true-停止点动成功，false-停止点动失败 |
#### 命令示例
```
ros2 service call /tl_driver/set_drag_mode tl_ros2_interface/srv/SetDragMode "{mode: 3}"
```
### 查看拖拽状态
| 功能描述 | 查询拖拽是否结束 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-拖拽结束，false-拖拽未结束 |
#### 命令示例
```
ros2 service call /tl_driver/get_drag_status std_srvs/srv/Trigger "{}"
```
### 拖拽轨迹保存
| 功能描述 | 拖拽轨迹保存 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | TrackSave.srv<br>string traj_name：保存轨迹名称 |
| 返回值 | true-保存成功，false-保存失败 |
#### 命令示例
```
ros2 service call /tl_driver/track_save tl_ros2_interface/srv/TrackSave "{traj_name: 'traj_test'}"
```
### 拖拽轨迹回放
| 功能描述 | 拖拽轨迹回放 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | TrackPlayback.srv<br>int32 vel：轨迹回放速度 |
| 返回值 | true-轨迹回放成功，false-轨迹回放失败 |
#### 命令示例
```
ros2 service call /tl_driver/track_playback tl_ros2_interface/srv/TrackPlayback "{vel: 20}"
```
## 错误处理与零位标定接口
### 清除错误
| 功能描述 | 清除错误 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-清除错误成功，false-清除错误失败 |
#### 命令示例
```
ros2 service call /tl_driver/clear_error std_srvs/srv/Trigger "{}"
```
### 设置关节零点
| 功能描述 | 设置关节零点 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetAxisZeroPos.srv<br>int32 axis：关节轴号 |
| 返回值 | true-设置成功，false-设置失败 |
| 注意 | 设置关节零点需要先进行power_off，完成设置后再power_on |
#### 命令示例
```
ros2 service call /tl_driver/set_axis_zero_pos tl_ros2_interface/srv/SetAxisZeroPos "{axis: 1}"
```
## 全局路点管理接口
### 查询全局位点
| 功能描述 | 查询全局位点 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetGlobalPos.srv<br>string pos_name：位点名称|
| 返回值 | true-查询成功，false-查询失败<br>float64[] pos_info：位点信息|
#### 命令示例
```
ros2 service call /tl_driver/get_global_pos tl_ros2_interface/srv/GetGlobalPos "{pos_name: 'GP0002'}"
```
### 设置全局位点
| 功能描述 | 设置全局位点 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetGlobalPos.srv<br>string pos_name：位点名称<br>float64[] pos_info：位点信息|
| 返回值 | true-设置成功，false-设置失败|
#### 命令示例
```
ros2 service call /tl_driver/set_global_pos tl_ros2_interface/srv/SetGlobalPos \
'{
    pos_name: "GP0002", 
    pos_info: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 57.14, -32.93, 19.74, -89.89, -19.77, 0.0]
}'
```
## 模式与IO控制接口
### 设置当前运行模式
| 功能描述 | 设置当前运行模式 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetCurrentMode.srv<br>int32 mode：模式序号(0-示教  1-远程  2-运行)|
| 返回值 | true-设置成功，false-设置失败|
#### 命令示例
```
ros2 service call /tl_driver/set_current_mode tl_ros2_interface/srv/SetCurrentMode "{mode: 2}"
```
### 设置数字输出
| 功能描述 | 设置数字输出 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetDigitalOutput.srv<br>int32 port：端口号<br>int32 value：端口输出状态（0或1）|
| 返回值 | true-设置成功，false-设置失败|
#### 命令示例
```
ros2 service call /tl_driver/set_digital_output tl_ros2_interface/srv/SetDigitalOutput "{port: 1, value: 1}"
```
### 查询数字输入输出状态
| 功能描述 | 查看数字输入输出状态 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetDigitalInputOutput.srv|
| 返回值 | true-查询成功，false-查询失败<br>int32[] input：数字输入状态<br>int32[] output：数字输出状态|
#### 命令示例
```
ros2 service call /tl_driver/get_digital_input_output tl_ros2_interface/srv/GetDigitalInputOutput "{}"
```
## modbus通信接口
### 写Modbus
| 功能描述 | Modbus数据写入 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | ModbusWrite.srv<br>int32 master_id：主站ID<br>int32 addr：主站地址<br>int32[] data：写入数据<br>ModbusMasterParam.msg（Modbus主站参数）<br>string type：主站类型<br>bool start_addr：起始地址<br>ModbusTCPParam.msg（ModbusTCP通信参数）<br>string ip：IP地址<br>int32 port：端口号<br>ModbusRTUParam.msg（ModbusRTU通信参数）<br>int32 slave_id：从站ID<br>int32 port：从站端口<br>int32 baudrate：波特率<br>int32 data_bit：数据位<br>int32 stop_bit：停止位<br>string check_bit：校验位|
| 返回值 | true-写入成功，false-写入失败|
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
### 读Modbus
| 功能描述 | Modbus数据读取 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | ModbusRead.srv<br>int32 master_id：主站ID<br>int32 addr：主站地址<br>int32 quantity：读取数量<br>tl_ros2_interface/ModbusMasterParam master_param：Modbus主站参数<br>其它相关参数可参考写Modbus部分|
| 返回值 | true-写入成功，false-写入失败|
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
## 队列运动接口
### 设置MoveJ队列运动模式
| 功能描述 | 设置MoveJ队列运动模式 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | QueueMotionSetStatus.srv<br>bool status：队列运动模型状态开关（true-打开  false-关闭）|
| 返回值 | true-设置成功，false-设置失败|
#### 命令示例
```
ros2 service call /tl_driver/queue_motion_set_status tl_ros2_interface/srv/QueueMotionSetStatus "{status: true}"
```
### MoveJ队列运动
| 功能描述 | MoveJ队列运动 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | QueueMotionMoveJ.srv<br>bool is_continue：是否连续运动<br>MoveCommand.msg（运动控制命令参数）<br>cmd的参数较多，此处不一一列举，相关参数请查询API函数接口|
| 返回值 | true-运动成功，false-运动失败|
#### 命令示例
```
ros2 service call /tl_driver/queue_motion_movej tl_ros2_interface/srv/QueueMotionMoveJ \
"{
    is_continue: false,
    cmd: 
    {
        target_pos_value: [0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0], 
        velocity: 20.0, 
        acc: 20.0, 
        dec: 20.0
    }
}"
```
### 停止MoveJ队列运动模式
| 功能描述 | 停止MoveJ队列运动模式 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger|
| 返回值 | true-停止成功，false-停止失败|
#### 命令示例
```
ros2 service call /tl_driver/queue_motion_stop std_srvs/srv/Trigger "{}"
```
## 7000端口
### 打开关节跟踪模式
| 功能描述 | 打开关节跟踪模式 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | OpenServoJ.srv<br>float64[] vmax：最大速度<br>float64[] amax：最大加速度<br>float64[] jmax：最大加加速度|
| 返回值 | true-打开成功，false-打开失败|
#### 命令示例
```
ros2 service call /tl_driver/open_servoj tl_ros2_interface/srv/OpenServoJ \
"{
    vmax: [300, 300, 300, 300, 300, 300, 300],
    amax: [3000, 3000, 3000, 3000, 3000, 3000, 3000],
    jmax: [50000, 50000, 50000, 50000, 50000, 50000, 50000]
}"
```
* 注意: 该模式只能在连接端口7000后使用。在使用该功能前最好先增加机械臂运行速度，如果运行速度太小的话会出现关节不动或者关节运行较慢的情况。
### 关闭关节跟踪模式
| 功能描述 | 关闭关节跟踪模式 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger|
| 返回值 | true-关闭成功，false-关闭失败|
#### 命令示例
```
ros2 service call /tl_driver/close_servoj std_srvs/srv/Trigger "{}"
```
### 发送跟踪关节位置
| 功能描述 | 发送跟踪关节位置 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 参数说明 | std_msgs::msg::Float64MultiArray<br>float64[] data：目标关节角度 |
| 返回值 | 无返回值 |
#### 命令示例
```
ros2 topic pub /tl_driver/set_servoj_pos std_msgs/msg/Float64MultiArray \ 
"{
    layout: 
    {
        dim: [], 
        data_offset: 0
    }, 
    data: [0.0, 0.0, 0.0, 0.0, 0.0, 20.0, 0.0]
}"
```
## 位姿转换工具接口
### 四元数转欧拉角
| 功能描述 | 四元数转欧拉角 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetPosTransform.srv<br>float64[] input：四元数输入(长度为4，顺序为wxyz) |
| 返回值 | true-转换成功，false-转换失败<br>float64[] output：欧拉角输出(长度为3) |
#### 命令示例
```
ros2 service call /tl_driver/get_quat2rpy tl_ros2_interface/srv/GetPosTransform \
"{
    input: [-0.080, 0.919, 0.365, 0.122]
}"
```
### 欧拉角转四元数
| 功能描述 | 欧拉角转四元数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetPosTransform.srv<br>float64[] input：欧拉角输入(长度为3) |
| 返回值 | true-转换成功，false-转换失败<br>float64[] output：四元数输出(长度为4) |
#### 命令示例
```
ros2 service call /tl_driver/get_rpy2quat tl_ros2_interface/srv/GetPosTransform \
"{
    input: [-2.9, 0.167, -0.777]
}"
```
### 欧拉角转旋转矩阵
| 功能描述 | 欧拉角转旋转矩阵 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetPosTransform.srv<br>float64[] input：欧拉角输入(长度为3) |
| 返回值 | true-转换成功，false-转换失败<br>float64[] output：旋转矩阵输出(长度为9) |
#### 命令示例
```
ros2 service call /tl_driver/get_rpy2r tl_ros2_interface/srv/GetPosTransform \
"{
    input: [-2.9, 0.167, -0.777]
}"
```
### 位姿转旋转矩阵
| 功能描述 | 位姿转旋转矩阵 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetPosTransform.srv<br>float64[] input：位姿输入(长度为16) |
| 返回值 | true-转换成功，false-转换失败<br>float64[] output：旋转矩阵输出(长度为9) |
#### 命令示例
```
ros2 service call /tl_driver/get_tr2r tl_ros2_interface/srv/GetPosTransform \
"{
    input: [0.703, 0.691, 0.166, 0.000, 0.652, -0.720, 0.236, 0.000, 0.283, -0.057, -0.957, 0.000, 0.000, 0.000, 0.000, 1.000]
}"
```
### 旋转矩阵转位姿
| 功能描述 | 旋转矩阵转位姿 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetPosTransform.srv<br>float64[] input：旋转矩阵输入(长度为9) |
| 返回值 | true-转换成功，false-转换失败<br>float64[] output：位姿输出(长度为16) |
#### 命令示例
```
ros2 service call /tl_driver/get_r2tr tl_ros2_interface/srv/GetPosTransform \
"{
    input: [0.703, 0.691, 0.166, 0.652, -0.720, 0.236, 0.283, -0.057, -0.957]
}"
```
