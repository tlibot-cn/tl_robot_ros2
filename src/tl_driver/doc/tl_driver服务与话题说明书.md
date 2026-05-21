<div align="center">

# tl_driver 功能说明书

文件修订记录：

|版本号 | 时间 | 备注 |
| :---: | :---- | :---: |
|V1.0 | 2026-4-29 | 拟制 |
|V1.1 | 2024-5-9  | 修订（添加[查询机械臂状态](#四状态查询)、[查询库版本信息](#四状态查询)、[查询关节参数](#三速度与参数设置)、[设置关节参数](#三速度与参数设置)、[查询关节温度](#四状态查询)、[查询关节电压](#四状态查询)、[查询电机电流](#四状态查询)、[查询关节软件版本号](#四状态查询)、[查询算法库版本](#四状态查询)、[设置机械臂默认DH参数](#三速度与参数设置)、[设置机械臂默认笛卡尔参数](#三速度与参数设置)、[日志下载](#十其他)、[查询运行速度](#三速度与参数设置)、[设置坐标系编号](#五坐标系与工具)、[设置机械臂DH参数](#三速度与参数设置)、[四元数转欧拉角](#六位姿变换)、[欧拉角转四元数](#六位姿变换)、[欧拉角转旋转矩阵](#六位姿变换)、[位姿转旋转矩阵](#六位姿变换)、[旋转矩阵转位姿](#六位姿变换)、[设置控制器有线网口IP](#八io与通信)、[查询控制器序列号ID](#四状态查询)、[查询当前坐标系](#五坐标系与工具)等接口）|
|V2.0 | 2026-5-21 | 重构：按功能分类重组文档 |

</div>

## 目录

### 一、机械臂连接与电源管理
* [1.1 机械臂连接](#11-机械臂连接)
* [1.2 机械臂断开连接](#12-机械臂断开连接)
* [1.3 机械臂上电](#13-机械臂上电)
* [1.4 机械臂下电](#14-机械臂下电)
* [1.5 清除错误](#15-清除错误)

### 二、运动控制
* [2.1 MoveJ运动控制](#21-movej运动控制)
* [2.2 MoveL运动控制](#22-movel运动控制)
* [2.3 开始点动](#23-开始点动)
* [2.4 停止点动](#24-停止点动)
* [2.5 设置MoveJ队列运动模式](#25-设置movej队列运动模式)
* [2.6 MoveJ队列运动](#26-movej队列运动)
* [2.7 停止MoveJ队列运动模式](#27-停止movej队列运动模式)
* [2.8 打开关节跟踪模式](#28-打开关节跟踪模式)
* [2.9 关闭关节跟踪模式](#29-关闭关节跟踪模式)
* [2.10 发送跟踪关节位置](#210-发送跟踪关节位置)

### 三、速度与参数设置
* [3.1 设置运行速度](#31-设置运行速度)
* [3.2 查询运行速度](#32-查询运行速度)
* [3.3 查询关节参数](#33-查询关节参数)
* [3.4 设置关节参数](#34-设置关节参数)
* [3.5 设置机械臂DH参数](#35-设置机械臂dh参数)
* [3.6 查询机械臂DH参数](#36-查询机械臂dh参数)
* [3.7 设置机械臂默认DH参数](#37-设置机械臂默认dh参数)
* [3.8 设置机械臂默认笛卡尔参数](#38-设置机械臂默认笛卡尔参数)

### 四、状态查询
* [4.1 查询机械臂状态](#41-查询机械臂状态)
* [4.2 查询关节角度](#42-查询关节角度)
* [4.3 查询末端位姿](#43-查询末端位姿)
* [4.4 查询关节温度](#44-查询关节温度)
* [4.5 查询关节电压](#45-查询关节电压)
* [4.6 查询电机电流](#46-查询电机电流)
* [4.7 查询关节软件版本号](#47-查询关节软件版本号)
* [4.8 查询库版本信息](#48-查询库版本信息)
* [4.9 查询算法库版本](#49-查询算法库版本)
* [4.10 查询控制器序列号ID](#410-查询控制器序列号id)
* [4.11 查询目标位姿可达状态](#411-查询目标位姿可达状态)

### 五、坐标系
* [5.1 设置当前坐标系](#51-设置当前坐标系)
* [5.2 查询当前坐标系](#52-查询当前坐标系)
* [5.3 设置坐标系编号](#53-设置坐标系编号)
* [5.4 查询坐标系编号](#54-查询坐标系编号)
* [5.5 设置用户坐标系](#55-设置用户坐标系)

### 六、坐标与位姿变换
* [6.1 坐标转换](#61-坐标转换)
* [6.2 四元数转欧拉角](#62-四元数转欧拉角)
* [6.3 欧拉角转四元数](#63-欧拉角转四元数)
* [6.4 欧拉角转旋转矩阵](#64-欧拉角转旋转矩阵)
* [6.5 位姿转旋转矩阵](#65-位姿转旋转矩阵)
* [6.6 旋转矩阵转位姿](#66-旋转矩阵转位姿)

### 七、工具手
* [7.1 设置工具手参数](#71-设置工具手参数)
* [7.2 工具手参数标定](#72-工具手参数标定)

### 八、拖拽功能
* [8.1 设置拖拽模式](#81-设置拖拽模式)
* [8.2 查看拖拽状态](#82-查看拖拽状态)
* [8.3 拖拽轨迹保存](#83-拖拽轨迹保存)
* [8.4 拖拽轨迹回放](#84-拖拽轨迹回放)

### 九、I/O与通信
* [9.1 设置数字输出](#91-设置数字输出)
* [9.2 查询数字输入输出状态](#92-查询数字输入输出状态)
* [9.3 写Modbus](#93-写modbus)
* [9.4 读Modbus](#94-读modbus)
* [9.5 设置控制器有线网口IP](#95-设置控制器有线网口ip)

### 十、作业文件管理
* [10.1 查询所有作业文件名称](#101-查询所有作业文件名称)
* [10.2 运行作业文件](#102-运行作业文件)
* [10.3 设置全局位点](#103-设置全局位点)
* [10.4 查询全局位点](#104-查询全局位点)
* [10.5 向作业文件插入一条moveJ关节运动](#105-向作业文件插入一条movej关节运动)
* [10.6 向作业文件插入一条moveL关节运动](#106-向作业文件插入一条movel关节运动)
* [10.7 向作业文件插入一条增量指令IMove](#107-向作业文件插入一条增量指令imove)
* [10.8 向作业文件插入一条moveC关节运动](#108-向作业文件插入一条movec关节运动)
* [10.9 设置当前运行模式](#109-设置当前运行模式)

### 十一、其他
* [11.1 设置关节零点](#111-设置关节零点)
* [11.2 日志下载](#112-日志下载)

---

## 一、机械臂连接与电源管理

### 1.1 机械臂连接
| 功能描述 | 通过TCP/IP与机械臂控制器建立网络通信连接。连接成功后即可进行后续的上电、运动控制等所有操作。连接前需确保控制器IP地址和端口配置正确（默认为192.168.1.13:6001）。 |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-连接成功，false-连接失败 |
#### 命令示例
```
ros2 service call /tl_driver/connect_arm std_srvs/srv/Trigger "{}"
```

### 1.2 机械臂断开连接
| 功能描述 | 断开与机械臂控制器的TCP/IP网络通信连接。断开后机械臂将保持当前状态，但无法再接收新的控制指令。建议在程序退出前主动断开连接。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-断开连接成功，false-断开连接失败 |
#### 命令示例
```
ros2 service call /tl_driver/disconnect_arm std_srvs/srv/Trigger "{}" 
```

### 1.3 机械臂上电
| 功能描述 | 对机械臂进行使能上电操作，为各关节电机通电并进入伺服就绪状态。上电成功后机械臂将保持当前位置（带自锁力），此时方可执行运动指令。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-上电成功，false-上电失败 |
#### 命令示例
```
ros2 service call /tl_driver/power_on std_srvs/srv/Trigger "{}"
```

### 1.4 机械臂下电
| 功能描述 | 对机械臂进行使能下电操作，关闭各关节电机使能。下电后机械臂处于自由状态（无自锁力），关节可被动转动。下电前应先停止所有运动任务。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-下电成功，false-下电失败 |
#### 命令示例
```
ros2 service call /tl_driver/power_off std_srvs/srv/Trigger "{}"
```

### 1.5 清除错误
| 功能描述 | 清除机械臂控制器中当前报错状态。当机械臂因碰撞、过载、通信异常等原因触发保护性报错时，需先排除故障原因后再调用此服务清除错误，使机械臂恢复可操作状态。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-清除错误成功，false-清除错误失败 |
#### 命令示例
```
ros2 service call /tl_driver/clear_error std_srvs/srv/Trigger "{}"
```

---

## 二、运动控制

### 2.1 MoveJ运动控制
| 功能描述 | 通过话题发布MoveCommand消息，控制机械臂以关节空间插补方式（MoveJ）运动到目标位姿。各关节同时从当前角度运动到目标角度，运动路径为关节空间中的非线性轨迹，末端走弧线。适用于对路径无严格要求的点到点运动。 |
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

### 2.2 MoveL运动控制
| 功能描述 | 通过话题发布MoveCommand消息，控制机械臂以笛卡尔空间直线插补方式（MoveL）运动到目标位姿。机械臂末端沿直线轨迹运动，姿态在起止点间线性变化。适用于对路径有严格直线要求的应用场景，如焊接、涂胶、搬运等。 |
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

### 2.3 开始点动
| 功能描述 | 以点动方式（Jogging）使指定关节沿正方向或反方向持续运动，直到收到[停止点动](#24-停止点动)指令。点动常用于手动示教、调试和姿态微调，每次只能点动一个关节。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | Jogging.srv<br>int32 axis：关节轴号<br>bool direction：关节运动方向（true-正方向  false-反方向） |
| 返回值 | true-点动成功，false-点动失败 |
#### 命令示例
```
ros2 service call /tl_driver/start_jogging tl_ros2_interface/srv/Jogging "{axis: 3, direction: 1}"
```

### 2.4 停止点动
| 功能描述 | 停止指定关节的点动运动。通常在[开始点动](#23-开始点动)将关节移动到目标位置后调用。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | Jogging.srv<br>int32 axis：关节轴号 |
| 返回值 | true-停止点动成功，false-停止点动失败 |
#### 命令示例
```
ros2 service call /tl_driver/stop_jogging tl_ros2_interface/srv/Jogging "{axis: 3}"
```

### 2.5 设置MoveJ队列运动模式
| 功能描述 | 打开或关闭MoveJ队列运动模式（Queue Motion）。开启后可通过[MoveJ队列运动](#26-movej队列运动)依次添加多条运动指令到队列中，机械臂将按添加顺序连续执行，实现多段连续运动而无须等待每段运动结束后再下发下一条指令。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | QueueMotionSetStatus.srv<br>bool status：队列运动模型状态开关（true-打开  false-关闭）|
| 返回值 | true-设置成功，false-设置失败|
#### 命令示例
```
ros2 service call /tl_driver/queue_motion_set_status tl_ros2_interface/srv/QueueMotionSetStatus "{status: true}"
```

### 2.6 MoveJ队列运动
| 功能描述 | 在队列运动模式已开启的状态下，向运动队列中添加一条MoveJ运动指令。支持连续运动模式（is_continue=true），使各段运动之间平滑过渡不停顿。 |
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

### 2.7 停止MoveJ队列运动模式
| 功能描述 | 停止当前MoveJ队列运动，清空队列中尚未执行的运动指令。机械臂将在完成当前正在执行的单段运动后停止。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger|
| 返回值 | true-停止成功，false-停止失败|
#### 命令示例
```
ros2 service call /tl_driver/queue_motion_stop std_srvs/srv/Trigger "{}"
```

### 2.8 打开关节跟踪模式
| 功能描述 | 打开关节跟踪模式（ServoJ），允许通过[发送跟踪关节位置](#210-发送跟踪关节位置)话题实时下发目标关节角度，机械臂将高速跟踪该角度序列进行连续运动。常用于视觉伺服、力控跟踪等需要实时轨迹调整的场景。注意：该模式只能在连接辅助端口7000后使用，建议使用前适当增大运行速度以避免关节响应滞后。 |
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

### 2.9 关闭关节跟踪模式
| 功能描述 | 关闭关节跟踪模式（ServoJ），退出实时关节角度控制状态。关闭后机械臂将保持当前角度位置，恢复对常规运动指令（MoveJ/MoveL）的响应。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger|
| 返回值 | true-关闭成功，false-关闭失败|
#### 命令示例
```
ros2 service call /tl_driver/close_servoj std_srvs/srv/Trigger "{}"
```

### 2.10 发送跟踪关节位置
| 功能描述 | 在关节跟踪模式（ServoJ）已开启的状态下，通过话题实时发布目标关节角度数组。机械臂将高速跟踪该角度值进行连续运动，发布频率越高跟踪效果越好。需先调用[打开关节跟踪模式](#28-打开关节跟踪模式)开启ServoJ模式后方可使用。 |
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

---

## 三、速度与参数设置

### 3.1 设置运行速度
| 功能描述 | 设置机械臂全局运动速度百分比（范围0~100），该值将作为后续所有MoveJ/MoveL等运动指令的速度倍率。例如设为20表示以最大速度的20%运行。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetSpeed.srv<br>float64 speed：运行速度 |
| 返回值 | true-设置成功，false-设置失败 |
#### 命令示例
```
ros2 service call /tl_driver/set_speed tl_ros2_interface/srv/SetSpeed "{speed: 20.0}"
```

### 3.2 查询运行速度
| 功能描述 | 查询当前机械臂的全局运动速度百分比设置值，返回的速度值即此前通过[设置运行速度](#31-设置运行速度)设定的值。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetSpeed.srv<br> |
| 返回值 | true-查询成功，false-查询失败<br>float64 speed：运行速度 |
#### 命令示例
```
ros2 service call /tl_driver/get_speed tl_ros2_interface/srv/GetSpeed "{}"
```

### 3.3 查询关节参数
| 功能描述 | 查询指定关节（轴）的详细运动学与动力学参数，包括减速比、编码器位数、正/反方向软限位角度、额定/最大转速、额定速度、最大加速度/减速度以及模型方向等。这些参数定义了关节的运动范围和能力边界。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetRobotJointParam.srv<br>int32 id：关节序号 |
| 返回值 | true-查询成功，false-查询失败<br>查询成功时返回关节参数<br>RobotJointParam.msg（机械臂关节信息）<br>float64 reduction_ratio：关节减速比<br>int32 encoder_resolution：编码器位数<br>float64 pos_sw_limit：轴正限位<br>float64 neg_sw_limit：轴反限位<br>float64 rated_rot_speed：电机额定正转速<br>float64 rated_derot_speed：电机额定反转速<br>float64 max_rot_speed：电机最大正转速<br>float64 max_derot_speed：电机最大反转速<br>float64 rated_vel：额定正速度<br>float64 rated_devel：额定反速度<br>float64 max_acc：最大加速度<br>float64 max_deacc：最大减速度<br>int32 direction：模型方向 |
#### 命令示例
```
ros2 service call /tl_driver/get_robot_joint_param tl_ros2_interface/srv/GetRobotJointParam "{id: 1}"
```

### 3.4 设置关节参数
| 功能描述 | 设置指定关节（轴）的运动学与动力学参数，包括减速比、限位角度、额定/最大速度、加速度/减速度等。修改后的参数将替换该关节的原有配置，需谨慎调整以避免超出机械臂硬件能力范围。参数字段详见[查询关节参数](#33-查询关节参数)的返回值说明。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetRobotJointParam.srv<br>int32 id：关节序号<br>RobotJointParam.msg<br>相关参数查阅[查询关节参数](#33-查询关节参数)返回值部分 |
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

### 3.5 设置机械臂DH参数
| 功能描述 | 设置机械臂的DH（Denavit-Hartenberg）运动学参数。DH参数定义了各连杆之间的几何关系（连杆长度、扭转角、关节距离、关节角偏移），是机械臂正逆运动学计算的基础。修改后会影响所有运动学相关的计算精度。 |
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

### 3.6 查询机械臂DH参数
| 功能描述 | 查询当前机械臂的DH（Denavit-Hartenberg）运动学参数，返回各连杆的几何定义数据。可用于校验当前参数配置是否正确或备份当前参数。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetDHParam.srv|
| 返回值 | true-查询成功，false-查询失败<br>RobotDHParam.msg|
#### 命令示例
```
ros2 service call /tl_driver/get_dh_param tl_ros2_interface/srv/GetDHParam
```

### 3.7 设置机械臂默认DH参数
| 功能描述 | 将机械臂的DH参数恢复为出厂默认值。当DH参数被修改导致运动学计算异常时，可使用此服务快速恢复至原始配置。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | RestoreDefaultDHParam.srv |
| 返回值 | true-恢复成功，false-恢复失败 |
#### 命令示例
```
ros2 service call /tl_driver/restore_default_dh_param tl_ros2_interface/srv/RestoreDefaultDHParam "{robot_num: 1}"
```

### 3.8 设置机械臂默认笛卡尔参数
| 功能描述 | 将机械臂的笛卡尔空间运动参数恢复为出厂默认值，包括直线运动、圆弧运动等笛卡尔轨迹规划的相关参数配置。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-设置成功，false-设置失败 |
#### 命令示例
```
ros2 service call /tl_driver/set_default_cartesian_param std_srvs/srv/Trigger "{}" 
```

---

## 四、状态查询

### 4.1 查询机械臂状态
| 功能描述 | 查询机械臂的综合状态信息，支持丰富的查询选项：可获取IO状态、关节坐标或直角坐标、运动点位等。支持单次查询（mode=0）和持续回传（mode=1，可设置回传间隔）。可同时查询多个IO端口状态，以及选择运动点位返回的坐标类型。 |
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
    io_port: [\"DO1\"], 
    optional: [\"ACS\"]
}"
```

### 4.2 查询关节角度
| 功能描述 | 通过ROS2话题实时订阅机械臂各关节的角度值，话题类型为sensor_msgs/JointState。数据以固定频率持续发布，可用于实时监控机械臂姿态或在外部程序中获取当前关节角度用于运动规划。 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 参数说明 | 无参数 |
| 返回值 | 关节角度（sensor_msgs::msg::JointState） |
#### 命令示例
```
ros2 topic echo /joint_states
```

### 4.3 查询末端位姿
| 功能描述 | 通过ROS2话题实时订阅机械臂末端执行器在笛卡尔空间中的位姿数据（位置+姿态），话题类型为CartesianPose.msg。包含末端在基坐标系下的三维坐标(x,y,z)和欧拉角(r,p,y)，可用于闭环控制或轨迹监控。 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 参数说明 | 无参数 |
| 返回值 | 末端位姿（CartesianPose.msg） |
#### 命令示例
```
ros2 topic echo /tcp_pose
```

### 4.4 查询关节温度
| 功能描述 | 查询机械臂各关节的当前温度值（单位：摄氏度），返回数组中每个元素对应一个关节的温度。用于实时监控关节是否过热，当温度超过安全阈值时应暂停运动让关节冷却。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetJointTemperature.srv |
| 返回值 | true-查询成功，false-查询失败<br>查询成功时返回各个关节温度<br>float64[] temperatures：各个关节温度 |
#### 命令示例
```
ros2 service call /tl_driver/get_joint_temperature tl_ros2_interface/srv/GetJointTemperature "{}"
```

### 4.5 查询关节电压
| 功能描述 | 查询机械臂本体各关节及外部轴（如变位机、导轨等）的当前电压值。电压异常可反映供电或驱动电路问题，用于故障诊断和预防性维护。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetJointVoltage.srv |
| 返回值 | true-查询成功，false-查询失败<br>查询成功时返回各个关节电压<br>float64[] joint_voltage：机械臂本体各关节电压<br>float64[] positioner_voltage：外部轴各关节电压 |
#### 命令示例
```
ros2 service call /tl_driver/get_joint_voltage tl_ros2_interface/srv/GetJointVoltage "{}"
```

### 4.6 查询电机电流
| 功能描述 | 查询机械臂各独立轴电机的当前工作电流，用于监控电机负载情况。电流异常升高可能预示机械卡死、碰撞或电机故障。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetMotorCurrent.srv |
| 返回值 | true-查询成功，false-查询失败<br>查询成功时返回各个独立轴电机电流<br>float64[] current_motor：机械臂独立轴电机电流 |
#### 命令示例
```
ros2 service call /tl_driver/get_joint_voltage tl_ros2_interface/srv/GetJointVoltage "{}"
```
> ⚠️ 注意：命令示例中的服务名似应为 `/tl_driver/get_motor_current`，此处沿用原始文档内容。

### 4.7 查询关节软件版本号
| 功能描述 | 查询指定关节（轴）的固件软件版本号。用于确认关节控制器的固件版本是否与驱动程序兼容，在固件升级后可用于验证升级是否成功。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetJointSoftwareVersion.srv<br>int32 axis_num：关节（轴）号 |
| 返回值 | true-查询成功，false-查询失败<br>查询成功时返回指定关节软件版本号 |
#### 命令示例
```
ros2 service call /tl_driver/get_joint_software_version tl_ros2_interface/srv/GetJointSoftwareVersion "{axis_num: 1}"
```

### 4.8 查询库版本信息
| 功能描述 | 查询当前使用的NRC API接口库的版本号及相关信息。用于确认驱动库版本与机械臂控制器固件是否匹配，以及在技术支持和问题排查时提供版本依据。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-查询成功，false-查询失败<br>查询成功时返回库版本信息 |
#### 命令示例
```
ros2 service call /tl_driver/get_library_version std_srvs/srv/Trigger "{}"
```

### 4.9 查询算法库版本
| 功能描述 | 查询NexMotion算法库的版本信息。该算法库负责机械臂的运动学正逆解、轨迹规划等核心计算，版本信息在排查运动学相关问题时具有重要参考价值。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-查询成功，false-查询失败<br>查询成功时返回算法库版本信息 |
#### 命令示例
```
ros2 service call /tl_driver/get_nexmotion_lib_version std_srvs/srv/Trigger "{}"
```

### 4.10 查询控制器序列号ID
| 功能描述 | 查询机械臂控制器的硬件序列号（ID），该序列号为每台控制器的唯一标识。在设备管理、售后服务和配置归档时用于精确识别具体的硬件单元。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-查询成功，false-查询失败<br>查询成功时返回控制器序列号ID|
#### 命令示例
```
ros2 service call /tl_driver/get_controller_id std_srvs/srv/Trigger "{}" 
```

### 4.11 查询目标位姿可达状态
| 功能描述 | 在发送运动指令前预检目标位姿是否在机械臂的可达工作空间内，以及指定的运动方式（如MoveJ/MoveL）能否使机械臂无碰撞地到达该位姿。用于运动规划前的可行性验证，避免执行不可达的运动指令。 |
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

---

## 五、坐标系

### 5.1 设置当前坐标系
| 功能描述 | 设置机械臂当前使用的坐标系类型。支持四种坐标系：关节坐标系（0）用于关节角度控制、直角坐标系（1）用于笛卡尔位置控制、工具坐标系（2）以工具末端为参考、用户坐标系（3）以用户自定义坐标系为参考。坐标系类型影响后续运动指令中目标位姿的解释方式。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetCurrentCoord.srv<br>int32 coord：坐标系序号（0-关节  1-直角  2-工具  3-用户） |
| 返回值 | true-设置成功，false-设置失败 |
#### 命令示例
```
ros2 service call /tl_driver/set_current_coord tl_ros2_interface/srv/SetCurrentCoord "{coord: 1}"
```

### 5.2 查询当前坐标系
| 功能描述 | 查询当前机械臂使用的坐标系类型，返回值0~3分别对应关节、直角、工具、用户坐标系。用于在切换坐标系前确认当前状态。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetCurrentCoord.srv<br> |
| 返回值 | true-查询成功，false-查询失败<br>int32 coord：坐标系序号（0-关节  1-直角  2-工具  3-用户） |
#### 命令示例
```
ros2 service call /tl_driver/get_current_coord tl_ros2_interface/srv/GetCurrentCoord "{}"
```

### 5.3 设置坐标系编号
| 功能描述 | 设置系统当前使用的工具坐标系编号和用户坐标系编号。在定义了多个工具或用户坐标系的情况下，通过此服务切换当前生效的坐标系编号。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetCoordNum.srv<br>int32 tool_num：工具坐标系序号<br>int32 user_num：用户坐标系序号 |
| 返回值 | true-设置成功，false-设置失败 |
#### 命令示例
```
ros2 service call /tl_driver/set_coord_num tl_ros2_interface/srv/SetCoordNum "{tool_num: 1, user_num: 2}"
```

### 5.4 查询坐标系编号
| 功能描述 | 查询系统当前使用的工具坐标系编号和用户坐标系编号。用于确认当前运动指令所参考的工具和用户坐标系配置。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetCoordNum.srv |
| 返回值 | true-查询成功，false-查询失败<br>int32 tool_num：工具坐标系序号<br>int32 user_num：用户坐标系序号|
#### 命令示例
```
ros2 service call /tl_driver/get_coord_num tl_ros2_interface/srv/GetCoordNum "{}"
```

### 5.5 设置用户坐标系
| 功能描述 | 设置用户自定义坐标系的原点位置（x, y, z）和姿态（r, p, y 欧拉角）以及臂角参数。用户坐标系常用于将工件坐标系定义为参考系，使在该工件上的编程点位不随工件位置变化而失效。 |
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

---

## 六、坐标与位姿变换

### 6.1 坐标转换
| 功能描述 | 在不同坐标系类型之间进行位姿数据转换。支持关节坐标与直角坐标互转、不同工具坐标系之间的转换、用户坐标系与世界坐标系之间的转换等。通过指定原始坐标系和目标坐标系的序号，以及参考位姿，将输入的原始位姿转换到目标坐标系下。 |
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

### 6.2 四元数转欧拉角
| 功能描述 | 将四元数（w, x, y, z）表示的刚体姿态转换为欧拉角（Roll, Pitch, Yaw）表示。四元数常用于姿态插补和避免万向节锁，欧拉角更直观易于人工理解。输入四元数顺序为wxyz。 |
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

### 6.3 欧拉角转四元数
| 功能描述 | 将欧拉角（Roll, Pitch, Yaw）表示的姿态转换为四元数（w, x, y, z）表示。常用于将外部输入的欧拉角姿态转换为四元数用于运动规划和姿态插补。 |
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

### 6.4 欧拉角转旋转矩阵
| 功能描述 | 将欧拉角（Roll, Pitch, Yaw）表示的姿态转换为3×3旋转矩阵。旋转矩阵可用于坐标变换计算，是机器人运动学中的基础转换之一。 |
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

### 6.5 位姿转旋转矩阵
| 功能描述 | 从4×4齐次变换矩阵（位姿矩阵，包含旋转和平移信息）中提取前3×3子矩阵作为旋转矩阵输出。常用于分离位姿中的旋转分量进行单独分析或运算。 |
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

### 6.6 旋转矩阵转位姿
| 功能描述 | 将3×3旋转矩阵扩展为4×4齐次变换矩阵（位姿矩阵），旋转部分填入前3×3子矩阵，平移部分设为零。用于将旋转矩阵还原为可用于运动学计算的齐次变换矩阵格式。 |
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

---

## 七、工具手

### 7.1 设置工具手参数
| 功能描述 | 设置工具手的几何参数和负载参数。几何参数包括TCP（工具中心点）在法兰坐标系下的位置偏移（x, y, z）和姿态旋转（a, b, c）；负载参数包括工具质量、惯性及质心位置。正确的工具参数是保证末端定位精度的前提。 |
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

### 7.2 工具手参数标定
| 功能描述 | 通过机械臂自动运动对指定工具坐标系进行参数标定。标定过程通过多个参考点的测量精确计算工具末端TCP的位置和姿态偏移量，使工具参数更接近实际值，提高末端定位精度。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | ToolHandCalib.srv<br>int32 tool_num：工具坐标系序号|
| 返回值 | true-标定成功，false-标定失败|
#### 命令示例
```
ros2 service call /tl_driver/tool_hand_calib tl_ros2_interface/srv/ToolHandCalib "{tool_num: 3}"
```

---

## 八、拖拽功能

### 8.1 设置拖拽模式
| 功能描述 | 设置机械臂的拖拽示教模式。在拖拽模式下，操作人员可以直接手持机械臂末端进行拖动示教。支持四种模式：0-关闭拖拽、1-3D鼠标模式、2-力矩模式（零力拖动/柔顺控制）、3-位置模式。力矩模式下机械臂处于重力补偿状态，操作最轻便。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetDragMode.srv<br>int32 mode：拖拽模式（0-无  1-3D鼠标  2-力矩模式 3-位置） |
| 返回值 | true-停止点动成功，false-停止点动失败 |
#### 命令示例
```
ros2 service call /tl_driver/set_drag_mode tl_ros2_interface/srv/SetDragMode "{mode: 3}"
```

### 8.2 查看拖拽状态
| 功能描述 | 查询当前拖拽操作是否已结束。当在拖拽示教模式下进行拖动操作时，操作完成后通过此服务确认拖拽是否已停止，以便进行后续的轨迹保存或回放操作。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | std_srvs::srv::Trigger |
| 返回值 | true-拖拽结束，false-拖拽未结束 |
#### 命令示例
```
ros2 service call /tl_driver/get_drag_statu std_srvs/srv/Trigger "{}"
```

### 8.3 拖拽轨迹保存
| 功能描述 | 将拖拽示教过程中记录的机械臂运动轨迹保存到控制器中，并指定轨迹名称。保存后的轨迹可在后续通过[拖拽轨迹回放](#84-拖拽轨迹回放)功能复现，实现"示教-回放"的编程模式。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | TrackSave.srv<br>string traj_name：保存轨迹名称 |
| 返回值 | true-保存成功，false-保存失败 |
#### 命令示例
```
ros2 service call /tl_driver/track_save tl_ros2_interface/srv/TrackSave "{traj_name: 'traj_test'}"
```

### 8.4 拖拽轨迹回放
| 功能描述 | 以指定速度回放此前通过[拖拽轨迹保存](#83-拖拽轨迹保存)保存的运动轨迹。机械臂将沿保存的轨迹路径以设定的速度百分比自动运动，复现示教时的运动过程。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | TrackPlayback.srv<br>int32 vel：轨迹回放速度 |
| 返回值 | true-轨迹回放成功，false-轨迹回放失败 |
#### 命令示例
```
ros2 service call /tl_driver/track_playback tl_ros2_interface/srv/TrackPlayback "{vel: 20}"
```

---

## 九、I/O与通信

### 9.1 设置数字输出
| 功能描述 | 设置机械臂控制器指定端口的数字输出信号状态（0或1）。数字输出可用于控制外部设备如电磁阀、报警灯、夹具等，是机械臂与外部设备交互的基础接口。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetDigitalOutput.srv<br>int32 port：端口号<br>int32 value：端口输出状态（0或1）|
| 返回值 | true-设置成功，false-设置失败|
#### 命令示例
```
ros2 service call /tl_driver/set_digital_output tl_ros2_interface/srv/SetDigitalOutput "{port: 1, value: 1}"
```

### 9.2 查询数字输入输出状态
| 功能描述 | 查询机械臂控制器所有数字输入端口和输出端口的当前状态。数字输入可用于读取外部传感器信号（如限位开关、启动按钮等），数字输出状态反映当前控制器的输出电平配置。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetDigitalInputOutput.srv|
| 返回值 | true-查询成功，false-查询失败<br>int32[] input：数字输入状态<br>int32[] output：数字输出状态|
#### 命令示例
```
ros2 service call /tl_driver/get_digital_input_output tl_ros2_interface/srv/GetDigitalInputOutput "{}"
```

### 9.3 写Modbus
| 功能描述 | 通过Modbus协议向指定主站的从站设备写入数据。支持Modbus TCP（以太网）和Modbus RTU（串口）两种通信方式。可配置主站参数（类型、起始地址）和从站参数（ID、端口、波特率、数据位、校验位等），适用于与PLC、传感器等Modbus设备通信。 |
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

### 9.4 读Modbus
| 功能描述 | 通过Modbus协议从指定主站的从站设备读取数据。支持Modbus TCP和Modbus RTU两种通信方式，需指定读取数据的数量及相关主站/从站通信参数。常用于读取外部传感器数据或设备状态。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | ModbusRead.srv<br>int32 quantity：读取数量<br>其它相关参数可参考[写Modbus](#83-写modbus)部分|
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

### 9.5 设置控制器有线网口IP
| 功能描述 | 设置机械臂控制器有线网口的网络参数，包括配置名称（如eth0）、IP地址、网关和DNS。当机械臂所在网络环境变更时，可通过此服务重新配置控制器的网络连接参数，确保控制器与上位机之间的通信正常。 |
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

---

## 十、作业文件管理

### 10.1 查询所有作业文件名称
| 功能描述 | 查询机械臂控制器中存储的所有作业文件（Job）的名称列表。作业文件是预先编制好的运动程序，包含一系列运动指令和逻辑控制。查询结果可用于选择需要运行的作业文件。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetAllJobFileName.srv|
| 返回值 | true-查询成功，false-查询失败<br>JobFileName.msg（作业文件名称参数）<br>string[] file_name：作业文件名称|
#### 命令示例
```
ros2 service call /tl_driver/get_all_job_filename tl_ros2_interface/srv/GetAllJobFileName "{}"
```

### 10.2 运行作业文件
| 功能描述 | 运行指定的作业文件（Job）。作业文件是预先存储在控制器中的运动程序，包含预设的运动指令序列。通过指定作业名称即可让机械臂自动执行该作业中编排的全部动作序列。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | JobRun.srv<br>string job_name：作业名称|
| 返回值 | true-运行成功，false-运行失败|
#### 命令示例
```
ros2 service call /tl_driver/job_run tl_ros2_interface/srv/JobRun "{job_name: '回零点'}"
```

### 10.3 设置全局位点
| 功能描述 | 在控制器中设置一个全局位点（Global Position），指定位点名称和位姿信息。全局位点可在作业文件中引用，用于定义关键位置点（如回零位、抓取位、放置位等），便于作业编程中重复调用。 |
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

### 10.4 查询全局位点
| 功能描述 | 根据位点名称查询已存储在控制器中的全局位点的位姿信息。返回该位点保存的全部关节角度或笛卡尔坐标值，用于确认位点数据或在外部程序中使用该位姿数据。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetGlobalPos.srv<br>string pos_name：位点名称|
| 返回值 | true-查询成功，false-查询失败<br>float64[] pos_info：位点信息|
#### 命令示例
```
ros2 service call /tl_driver/get_global_pos tl_ros2_interface/srv/GetGlobalPos "{pos_name: 'GP0002'}"
```

### 10.5 向作业文件插入一条moveJ关节运动
| 功能描述 | 向指定的作业文件中插入一条MoveJ（关节空间插补）运动指令。通过指定插入行号和MoveCommand运动参数，可在作业文件的指定位置添加一段关节运动，用于在线编辑和补充作业程序。 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 参数说明 | JobInsertMove.msg（作业文件插入运动控制参数）<br>int32 line：插入行序号<br>MoveCommand cmd：运动指令 |
| 返回值 | 无返回值 |
#### 命令示例
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

### 10.6 向作业文件插入一条moveL关节运动
| 功能描述 | 向指定的作业文件中插入一条MoveL（笛卡尔直线插补）运动指令。通过指定插入行号和MoveCommand运动参数，在作业文件的指定位置添加一段直线运动，用于在线编辑和补充作业程序。 |
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

### 10.7 向作业文件插入一条增量指令IMove
| 功能描述 | 向指定的作业文件中插入一条增量运动指令（IMove）。增量指令以当前位姿为基准进行相对运动，而非绝对定位。常用于需要重复偏移的场景，如码垛、阵列搬运等。通过指定插入行号和运动参数完成插入。 |
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

### 10.8 向作业文件插入一条moveC关节运动
| 功能描述 | 向指定的作业文件中插入一条MoveC（圆弧插补）运动指令。MoveC通过指定中间点和终点实现圆弧轨迹运动，常用于焊接、打磨等需要沿圆弧路径运动的场景。通过指定插入行号和运动参数完成插入。 |
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

### 10.9 设置当前运行模式
| 功能描述 | 设置机械臂的当前运行模式。支持三种模式：0-示教模式（用于手动编程和点位示教）、1-远程模式（由外部上位机通过API控制）、2-运行模式（自动执行已加载的作业文件）。在远程控制前需确保模式设置为1。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetCurrentMode.srv<br>int32 mode：模式序号(0-示教  1-远程  2-运行)|
| 返回值 | true-设置成功，false-设置失败|
#### 命令示例
```
ros2 service call /tl_driver/set_current_mode tl_ros2_interface/srv/SetCurrentMode "{mode: 2}"
```

---

## 十一、其他

### 11.1 设置关节零点
| 功能描述 | 将指定关节（轴）的当前位置设为该关节的零点位置。此操作会重新校准关节的编码器零位偏移，一般在更换关节电机、编码器或机械维护后执行，以恢复关节角度测量的准确性。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetAxisZeroPos.srv<br>int32 axis：关节轴号 |
| 返回值 | true-设置成功，false-设置失败 |
#### 命令示例
```
ros2 service call /tl_driver/set_axis_zero_pos tl_ros2_interface/srv/SetAxisZeroPos "{axis: 1}"
```

### 11.2 日志下载
| 功能描述 | 从机械臂控制器下载指定数量的日志文件到本地的指定目录。日志文件记录了控制器的运行状态、报警信息、操作记录等历史数据，用于故障排查、运行分析和维护保养。 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | LogDownload.srv<br>int32 count：日志数量<br>string directory_path：保存目录路径 |
| 返回值 | true-下载成功，false-下载失败 |
#### 命令示例
```
ros2 service call /tl_driver/log_download tl_ros2_interface/srv/LogDownload "{count: 1, directory_path: '/home/ubuntu/桌面'}"
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
### 开始点动
| 功能描述 | 开始点动机械臂 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | Jogging.srv<br>int32 axis：关节轴号<br>bool direction：关节运动方向（true-正方向  false-反方向） |
| 返回值 | true-点动成功，false-点动失败 |
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
### 查询机械臂状态
| 功能描述 | 查询机械臂详细状态 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetRobotState.srv<br>int32 channel：查询通道<br>bool stop：是否停止发送<br>int32 mode：查询模式（0-只回复一次  1-持续回复）<br>int32 interval：仅mode = 1时有效，回复时间范围 [10,60000] ms<br>bool io_state：查询IO<br>int32 position：0-关节坐标  1-直角坐标<br>bool detail_motion_pos：机械臂的运动点位<br>int32 pos_num：当查询机械臂运动点位时，posNum为每帧数据回复的点位数目<br>string[] io_port：IO端口，可查询的最大数量不可大于IO实际个数 例子:[ “DI1”, “DI16”, “DO1”, “DO3”, “DO17”]<br>string[] optional：查询运动点位返回的坐标类型  "ACS"-关节参数 "MCS"-直角参数 "time"-时间戳 "reset"-重置点位记录|
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
### 设置关节参数
| 功能描述 | 设置机械臂关节参赛 |
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
ros2 service call /tl_driver/get_joint_voltage tl_ros2_interface/srv/GetJointVoltage "{}"
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
ros2 service call /tl_driver/get_drag_statu std_srvs/srv/Trigger "{}"
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
### 设置关节零点
| 功能描述 | 设置关节零点 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | SetAxisZeroPos.srv<br>int32 axis：关节轴号 |
| 返回值 | true-设置成功，false-设置失败 |
#### 命令示例
```
ros2 service call /tl_driver/set_axis_zero_pos tl_ros2_interface/srv/SetAxisZeroPos "{axis: 1}"
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
### 工具手参数标定
| 功能描述 | 工具手参数标定 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | ToolHandCalib.srv<br>int32 tool_num：工具坐标系序号|
| 返回值 | true-标定成功，false-标定失败|
#### 命令示例
```
ros2 service call /tl_driver/tool_hand_calib tl_ros2_interface/srv/ToolHandCalib "{tool_num: 3}"
```
### 设置坐标系编号
| 功能描述 | 设置工具坐标系和用户坐标系编号 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetCoordNum.srv<br>int32 tool_num：工具坐标系序号<br>int32 user_num：用户坐标系序号 |
| 返回值 | true-设置成功，false-设置失败 |
```
ros2 service call /tl_driver/set_coord_num tl_ros2_interface/srv/SetCoordNum "{tool_num: 1, user_num: 2}"
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
### 工具手参数标定
| 功能描述 | 工具手参数标定 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | ToolHandCalib.srv<br>int32 tool_num：工具坐标系序号|
| 返回值 | true-标定成功，false-标定失败|
#### 命令示例
```
ros2 service call /tl_driver/tool_hand_calib tl_ros2_interface/srv/ToolHandCalib "{tool_num: 3}"
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
| 参数说明 | ModbusRead.srv<br>int32 quantity：读取数量<br>其它相关参数可参考读Modbus部分|
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
### 设置机械臂DH参数
| 功能描述 | 查询机械臂DH参数 |
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
### 查询机械臂DH参数
| 功能描述 | 设置机械臂DH参数 |
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
### 运行作业文件
| 功能描述 | 运行作业文件 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | JobRun.srv<br>string job_name：作业名称|
| 返回值 | true-运行成功，false-运行失败|
```
ros2 service call /tl_driver/job_run tl_ros2_interface/srv/JobRun "{job_name: '回零点'}"
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
### 查询全局位点
| 功能描述 | 设置全局位点 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 参数说明 | GetGlobalPos.srv<br>string pos_name：位点名称|
| 返回值 | true-查询成功，false-查询失败<br>float64[] pos_info：位点信息|
#### 命令示例
```
ros2 service call /tl_driver/get_global_pos tl_ros2_interface/srv/GetGlobalPos "{pos_name: 'GP0002'}"
```
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
### 向作业文件插入一条moveJ关节运动
| 功能描述 | 向作业文件插入一条moveJ关节运动 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 参数说明 | JobInsertMove.msg（作业文件插入运动控制参数）<br>int32 line：插入行序号<br>MoveCommand cmd：运动指令 |
| 返回值 | 无返回值 |
#### 命令示例
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
### 发送跟踪关节位置
| 功能描述 | 机械臂MoveL运动控制 |
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
