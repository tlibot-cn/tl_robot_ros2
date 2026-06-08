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
|V1.5 | 2026-6-8  | 修订（详细化全部接口的输入输出参数说明，补全单位、范围、注意事项；优化参数表格式；修复锚点链接；编号统一改为从1开始）|

</div>

## 目录
* [1 通用约定](#1-通用约定)
* [2 连接管理接口](#2-连接管理接口)
  * [2.1 机械臂连接](#21-机械臂连接)
  * [2.2 机械臂断开连接](#22-机械臂断开连接)
  * [2.3 机械臂上电](#23-机械臂上电)
  * [2.4 机械臂下电](#24-机械臂下电)
* [3 日志管理接口](#3-日志管理接口)
  * [3.1 日志下载](#31-日志下载)
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
  * [4.12 查询算法库版本](#412-查询算法库版本)
  * [4.13 查询当前坐标系](#413-查询当前坐标系)
  * [4.14 查询坐标系编号](#414-查询坐标系编号)
  * [4.15 查询机械臂DH参数](#415-查询机械臂dh参数)
  * [4.16 查询所有作业文件名称](#416-查询所有作业文件名称)
  * [4.17 查询目标位姿可达状态](#417-查询目标位姿可达状态)
  * [4.18 查询机械臂运行状态](#418-查询机械臂运行状态)
  * [4.19 查询当前电机力矩](#419-查询当前电机力矩)
  * [4.20 查询当前线速度和关节速度](#420-查询当前线速度和关节速度)
* [5 机械臂基础功能设置接口](#5-机械臂基础功能设置接口)
  * [5.1 设置运行速度](#51-设置运行速度)
  * [5.2 设置控制器有线网口IP](#52-设置控制器有线网口ip)
  * [5.3 设置关节参数](#53-设置关节参数)
  * [5.4 设置机械臂默认DH参数](#54-设置机械臂默认dh参数)
  * [5.5 设置机械臂默认笛卡尔参数](#55-设置机械臂默认笛卡尔参数)
  * [5.6 坐标转换](#56-坐标转换)
  * [5.7 设置机械臂DH参数](#57-设置机械臂dh参数)
* [6 作业运动控制接口](#6-作业运动控制接口)
  * [6.1 向作业文件插入一条moveJ关节运动](#61-向作业文件插入一条movej关节运动)
  * [6.2 向作业文件插入一条moveL直线运动](#62-向作业文件插入一条movel直线运动)
  * [6.3 向作业文件插入一条增量指令IMove](#63-向作业文件插入一条增量指令imove)
  * [6.4 向作业文件插入一条moveC圆弧运动](#64-向作业文件插入一条movec圆弧运动)
  * [6.5 运行作业文件](#65-运行作业文件)
  * [6.6 删除作业文件](#66-删除作业文件)
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
* [11 全局路点管理接口](#11-全局路点管理接口)
  * [11.1 查询全局位点](#111-查询全局位点)
  * [11.2 设置全局位点](#112-设置全局位点)
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
* [16 位姿转换工具接口](#16-位姿转换工具接口)
  * [16.1 四元数转欧拉角](#161-四元数转欧拉角)
  * [16.2 欧拉角转四元数](#162-欧拉角转四元数)
  * [16.3 欧拉角转旋转矩阵](#163-欧拉角转旋转矩阵)
  * [16.4 位姿转旋转矩阵](#164-位姿转旋转矩阵)
  * [16.5 旋转矩阵转位姿](#165-旋转矩阵转位姿)


## 1 通用约定

### 1.1 坐标系编号
| 编号 | 含义 | 说明 |
| :---: | :--- | :--- |
| 1 | 关节坐标系（Joint） | 以各关节角度表示位置，单位：度（°）或弧度（rad） |
| 2 | 直角坐标系（Cartesian/Base） | 以基座为原点，位置(X,Y,Z)单位mm，姿态(RX,RY,RZ)单位rad |
| 3 | 工具坐标系（Tool） | 以工具末端为参考的坐标系 |
| 4 | 用户坐标系（User） | 用户自定义的坐标系 |

### 1.2 运行模式编号
| 编号 | 含义 |
| :---: | :--- |
| 1 | 示教模式（手动操作） |
| 2 | 远程模式 |
| 3 | 运行模式（自动运行作业） |

### 1.3 返回值约定
- 所有 ROS2 服务的响应均包含 `bool success` 和 `string message` 字段
- `success = true`：操作成功（对应底层SDK返回值0）
- `success = false`：操作失败，`message` 中包含错误描述信息
- 对于 `std_srvs::srv::Trigger` 类型的服务，`success` 字段直接承载上述含义

### 1.4 单位制约定
- `target_pos_type` 字段中携带单位制信息：`1` = 角度制（度），`2` = 弧度制（rad）
- ROS标准消息（如 `sensor_msgs/JointState`、`CartesianPose`）中的角度/位置遵循ROS标准单位制
- SDK底层API中 `get_current_position` 等接口返回的角度单位为度（°）

---
## 2 连接管理接口
### 2.1 机械臂连接
| 功能描述 | 建立与机械臂控制器的TCP网络通信连接（端口6001和7000） |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/connect_arm` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 连接成功（双端口均已建立），`false` = 连接失败 |
| message | string | 失败时包含错误描述 |

> **注意**：该服务同时连接端口6001（指令通道）和端口7000（实时数据通道）。连接前需确保机械臂控制器IP和端口配置正确（默认IP：`192.168.1.13`，指令端口：`6001`，实时端口：`7000`）。
#### 命令示例
```
ros2 service call /tl_driver/connect_arm std_srvs/srv/Trigger "{}"
```
### 2.2 机械臂断开连接
| 功能描述 | 断开与机械臂控制器的TCP网络通信连接（双端口），释放网络资源 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/disconnect_arm` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 断开成功，`false` = 断开失败 |
| message | string | 失败时包含错误描述 |

> **注意**：断开连接前会自动执行机械臂下电和伺服关闭操作，确保安全退出。
#### 命令示例
```
ros2 service call /tl_driver/disconnect_arm std_srvs/srv/Trigger "{}" 
```
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

> **注意**：上电前需确保机械臂已连接（`connect_arm`成功），且无未清除的错误报警。若机械臂已处于上电状态，再次调用会返回成功。
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
## 3 日志管理接口
### 3.1 日志下载
| 功能描述 | 从机械臂控制器下载指定数量的日志条目并保存到本地文件 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/log_download` |
| 服务类型 | `tl_ros2_interface/srv/LogDownload` |

**输入参数**
| 参数名 | 类型 | 单位 | 范围 | 说明 |
|--------|------|------|------|------|
| count | int32 | — | ≥1 | 要下载的日志条数 |
| directory_path | string | — | — | 日志保存目录的绝对路径，如 `"/home/ubuntu/桌面"` |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 下载成功，`false` = 下载失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/log_download tl_ros2_interface/srv/LogDownload "{count: 1, directory_path: '/home/ubuntu/桌面'}"
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
| 功能描述 | 查询机械臂详细运行状态，包括IO状态、运动点位等综合信息 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_robot_state` |
| 服务类型 | `tl_ros2_interface/srv/GetRobotState` |

**输入参数**
| 参数名 | 类型 | 范围 | 说明 |
|--------|------|------|------|
| channel | int32 | — | 查询通道编号 |
| stop | bool | — | `true` = 停止持续发送，`false` = 不停止 |
| mode | int32 | 1 或 2 | 查询模式：`1` = 只回复一次，`2` = 持续回复 |
| interval | int32 | 10～60000 | 持续回复时间间隔（mode=2时有效），单位：ms |
| io_state | bool | — | `true` = 查询IO状态，`false` = 不查询 |
| position | int32 | 1 或 2 | 位置类型：`1` = 关节坐标，`2` = 直角坐标 |
| detail_motion_pos | bool | — | `true` = 查询运动点位详情，`false` = 不查询 |
| pos_sum | int32 | — | 每帧数据回复的点位数目（detail_motion_pos=true时有效） |
| io_port | string[] | — | 指定IO端口列表，如 `["DI1","DO1"]`，数量不可大于IO实际个数 |
| optional | string[] | — | 坐标类型：`"ACS"`=关节参数，`"MCS"`=直角参数，`"time"`=时间戳，`"reset"`=重置点位记录 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 成功时包含查询信息，失败时包含错误描述 |
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

> **注意**：此接口查询的是本地SDK动态库版本，无需连接机械臂即可调用。与 [查询算法库版本](#查询算法库版本)（查询控制器端算法库版本）不同。
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
| param.pos_sw_limit | float64 | ° | 轴正限位角度 |
| param.neg_sw_limit | float64 | ° | 轴反限位角度 |
| param.rated_rot_speed | float64 | rpm | 电机额定正转速 |
| param.rated_derot_speed | float64 | rpm | 电机额定反转速 |
| param.max_rot_speed | float64 | rpm | 电机最大正转速 |
| param.max_derot_speed | float64 | rpm | 电机最大反转速 |
| param.rated_vel | float64 | °/s | 额定正速度 |
| param.rated_devel | float64 | °/s | 额定反速度 |
| param.max_acc | float64 | °/s² | 最大加速度 |
| param.max_dec | float64 | °/s² | 最大减速度 |
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
| 功能描述 | 查询独立轴当前电机电流 |
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
| motor_current | float64[] | A | 机械臂独立轴电机电流 |
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
### 4.12 查询算法库版本
| 功能描述 | 查询机械臂控制器端算法库版本信息 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_nexmotion_lib_version` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 成功时返回算法库版本信息，失败时包含错误描述 |

> **注意**：此接口查询的是控制器端算法库版本，与 [查询库版本信息](#查询库版本信息)（查询本地动态库版本）不同。
#### 命令示例
```
ros2 service call /tl_driver/get_nexmotion_lib_version std_srvs/srv/Trigger "{}"
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
| coord | int32 | 1～4 | 坐标系序号：1=关节，2=直角，3=工具，4=用户 |
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
| 功能描述 | 查询机械臂的DH几何参数（连杆长度、耦合系数、动态限制等） |
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
| param | RobotDHParam | — | 机械臂DH参数结构体，主要字段：l1~l20（连杆长度，mm）、couple_coe_*（联动系数）、dynamic_limit_max/min（动态限制）等 |
#### 命令示例
```
ros2 service call /tl_driver/get_dh_param tl_ros2_interface/srv/GetDHParam
```
### 4.16 查询所有作业文件名称
| 功能描述 | 查询控制器内所有作业文件的名称列表 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_all_job_filename` |
| 服务类型 | `tl_ros2_interface/srv/GetAllJobFileName` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 失败时包含错误描述 |
| robots_file | JobFileName[] | 作业文件列表，每组包含 file_name 字符串数组 |
#### 命令示例
```
ros2 service call /tl_driver/get_all_job_filename tl_ros2_interface/srv/GetAllJobFileName "{}"
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
| pos | float64[] | ° 或 mm/rad | 14维位姿数组：[0]坐标系 [1]单位制 [2]形态 [3]工具序号 [4]用户序号 [5][6]备用 [7~13]点位信息 |
| move_type | string | — | 运动方式，如 `"MOVJ"` 或 `"MOVL"` |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 目标位姿可达，`false` = 不可达 |
| message | string | 不可达时包含原因描述 |
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
| run_state | string | 运行状态：`"STOP"` = 停止，`"PAUSE"` = 暂停，`"RUNNING"` = 运行中 |
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
| motor_torque | int32[] | Nm | 机械臂本体当前各电机力矩 |
| motor_torque_sync | int32[] | Nm | 同步轴当前各电机力矩 |
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
| line_speed | float64 | mm/s | 当前末端线速度 |
| joint_speed | float64[] | °/s | 机械臂本体当前各关节速度 |
| joint_speed_sync | float64[] | °/s | 同步轴当前各关节速度 |
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
### 5.4 设置机械臂默认DH参数
| 功能描述 | 将机械臂DH参数恢复为出厂默认值 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/restore_default_dh_param` |
| 服务类型 | `tl_ros2_interface/srv/RestoreDefaultDHParam` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| robot_num | int32 | 机器人编号，通常为 `1` |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 恢复成功，`false` = 恢复失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/restore_default_dh_param tl_ros2_interface/srv/RestoreDefaultDHParam "{robot_num: 1}"
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
| origin_coord | int32 | 1～4 | 源坐标系编号 |
| target_coord | int32 | 1～4 | 目标坐标系编号 |
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
### 5.7 设置机械臂DH参数
| 功能描述 | 设置机械臂的DH几何参数（连杆长度等） |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_dh_param` |
| 服务类型 | `tl_ros2_interface/srv/SetDHParam` |

**输入参数**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| param | RobotDHParam | — | 机械臂DH参数结构体，主要字段：l1~l20（连杆长度，mm）、couple_coe_*（联动系数）等 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |

> **注意**：修改DH参数会影响机械臂运动学模型，请谨慎操作。
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
## 6 作业运动控制接口

### MoveCommand消息类型说明

作业运动控制接口共用 `MoveCommand` 消息类型，其字段说明如下：

| 字段名 | 类型 | 单位 | 范围 | 说明 |
|--------|------|------|------|------|
| target_pos_value | float64[] | — | — | 14维位置数组：[0]坐标系 [1]单位制(1=度,2=弧度) [2]形态 [3]工具序号 [4]用户序号 [5][6]备用 [7~13]点位信息 |
| target_pos_name | string | — | — | 目标位姿变量名，如 `"P0001"`（target_pos_type=1时使用） |
| target_pos_type | int32 | — | 1 或 2 | 位姿类型：`1` = 自定义数组，`2` = 变量模式（使用target_pos_name） |
| coord | int32 | — | 1～4 | 坐标系：1=关节，2=直角，3=工具，4=用户 |
| velocity | float64 | % | 1～100 | 运动速度百分比 |
| velocity_sync | float64 | % | 1～100 | 同步轴速度百分比 |
| acc | float64 | % | 1～100 | 加速度百分比 |
| dec | float64 | % | 1～100 | 减速度百分比 |
| pl | int32 | — | ≥0 | 平滑度（过渡等级），0=精确到达 |
| time | int32 | — | — | 提前执行时间 |
| tool_num | int32 | — | — | 工具坐标系编号 |
| user_num | int32 | — | — | 用户坐标系编号 |
| posidtype | int32 | — | — | 变量类型 |
| configuration | int32 | — | — | 形态配置 |
| spin | int32 | — | 1～3 | 仅MOVCA指令使用：1=姿态不变，2=六轴不转，3=六轴旋转 |
| para_sync | bool | — | — | `true` = 外部轴同步，`false` = 不同步 |

### 6.1 向作业文件插入一条moveJ关节运动
| 功能描述 | 在作业文件指定行插入一条关节空间运动（MoveJ）指令 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/job_insert_moveJ` |
| 服务类型 | `tl_ros2_interface/srv/JobInsertMove` |

**输入参数**
| 参数名 | 类型 | 范围 | 说明 |
|--------|------|------|------|
| line | int32 | ≥1 | 插入行序号（从1开始） |
| cmd | MoveCommand | — | 运动指令，字段详见 [MoveCommand消息类型说明](#movecommand消息类型说明) |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 插入成功，`false` = 插入失败 |
| message | string | 失败时包含错误描述 |

> **注意**：target_pos_type=0时使用target_pos_value自定义数组；target_pos_type=1时需设置target_pos_name为 `"P0001"` 格式，target_pos_value仅填关节角度。
#### 命令示例
targetPosType=0为自定义数组，posInfo[14]：[0]坐标系 1=关节 2=直角 3=工具 4=用户，[1] 1=角度制 2=弧度制，[2]形态，[3]工具手坐标序号，[4]用户坐标序号，[5][6]备用，[7-13]点位信息
```
ros2 service call /tl_driver/job_insert_moveJ tl_ros2_interface/srv/JobInsertMove "{
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
ros2 service call /tl_driver/job_insert_moveJ tl_ros2_interface/srv/JobInsertMove "{
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
### 6.2 向作业文件插入一条moveL直线运动
| 功能描述 | 在作业文件指定行插入一条笛卡尔空间直线运动（MoveL）指令 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/job_insert_moveL` |
| 服务类型 | `tl_ros2_interface/srv/JobInsertMove` |

**输入参数** 同 §6.1，区别：`cmd.coord=2`（直角坐标系），target_pos_value为[X(mm), Y, Z, RX(rad), RY, RZ]。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 插入成功，`false` = 插入失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/job_insert_moveL tl_ros2_interface/srv/JobInsertMove "{
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
### 6.3 向作业文件插入一条增量指令IMove
| 功能描述 | 在作业文件指定行插入一条增量运动（IMove）指令，以当前位姿为基准进行相对运动 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/job_insert_imove` |
| 服务类型 | `tl_ros2_interface/srv/JobInsertMove` |

**输入参数** 同 §6.1，`cmd` 中的 `target_pos_value` 表示相对增量。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 插入成功，`false` = 插入失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/job_insert_imove tl_ros2_interface/srv/JobInsertMove "{
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
### 6.4 向作业文件插入一条moveC圆弧运动
| 功能描述 | 在作业文件指定行插入一条笛卡尔空间圆弧运动（MoveC）指令。需连续插入两条：第一条为圆弧中间点，第二条为终点 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/job_insert_moveC` |
| 服务类型 | `tl_ros2_interface/srv/JobInsertMove` |

**输入参数** 同 §6.2（coord=2，直角坐标系）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 插入成功，`false` = 插入失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/job_insert_moveC tl_ros2_interface/srv/JobInsertMove "{
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
### 6.5 运行作业文件
| 功能描述 | 启动运行指定的作业文件 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/job_run` |
| 服务类型 | `tl_ros2_interface/srv/JobRun` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| job_name | string | 作业文件名，如 `"回零点"` |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 运行成功，`false` = 运行失败（如文件不存在、未使能等） |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/job_run tl_ros2_interface/srv/JobRun "{job_name: '回零点'}"
```
### 6.6 删除作业文件
| 功能描述 | 删除指定的作业文件 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/job_delete` |
| 服务类型 | `tl_ros2_interface/srv/JobRun` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| job_name | string | 要删除的作业文件名，如 `"回零点"` |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 删除成功，`false` = 删除失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/job_delete tl_ros2_interface/srv/JobRun "{job_name: '回零点'}"
```
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
| target_pos_value | float64[] | — | — | 14维位置数组（coord=1关节坐标系时为各关节目标角度） |
| target_pos_name | string | — | — | 目标位姿名称（通常留空） |
| target_pos_type | int32 | — | 1 | 目标位姿类型（实时运动固定为1） |
| coord | int32 | — | 1 | 坐标系（固定为关节坐标系） |
| velocity | float64 | % | 1～100 | 运动速度百分比 |
| velocity_sync | float64 | % | 1～100 | 同步轴速度百分比 |
| acc | float64 | % | 1～100 | 加速度百分比 |
| dec | float64 | % | 1～100 | 减速度百分比 |
| pl | int32 | — | ≥0 | 平滑度，0=精确到达 |
| time | int32 | — | — | 提前执行时间 |
| tool_num | int32 | — | — | 工具坐标系编号 |
| user_num | int32 | — | — | 用户坐标系编号 |
| posidtype | int32 | — | — | 变量类型 |
| configuration | int32 | — | — | 形态配置 |
| spin | int32 | — | 1～3 | 1=姿态不变，2=六轴不转，3=六轴旋转 |
| para_sync | bool | — | — | 外部轴是否同步 |

**输出/返回值**
无返回值（话题发布单向通信）。

> **注意**：实时运动控制不等同于作业文件运动，需确保机械臂已上电（power_on）后再调用。
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
### 7.2 MoveL运动控制
| 功能描述 | 通过话题下发直角坐标数据，驱动机器人末端沿直线实时运动到目标位姿 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/tl_driver/moveL` |
| 消息类型 | `tl_ros2_interface/msg/MoveCommand` |

**输入参数** 同 MoveJ，区别如下：
| 参数名 | 类型 | 单位 | 值 | 说明 |
|--------|------|------|------|------|
| coord | int32 | — | 2 | 坐标系（固定为直角坐标系） |
| target_pos_value | float64[] | mm, rad | — | [X(mm), Y(mm), Z(mm), RX(rad), RY(rad), RZ(rad), 0]（7维） |

其余参数同 [MoveJ运动控制](#movej运动控制)。

**输出/返回值**
无返回值（话题发布单向通信）。

> **注意**：需确保机械臂已上电后再调用。
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
| param.x | float64 | mm | 末端工具X轴向偏移量 |
| param.y | float64 | mm | 末端工具Y轴向偏移量 |
| param.z | float64 | mm | 末端工具Z轴向偏移量 |
| param.a | float64 | ° | 绕X轴旋转角度 |
| param.b | float64 | ° | 绕Y轴旋转角度 |
| param.c | float64 | ° | 绕Z轴旋转角度 |
| param.payload_mass | float64 | kg | 负载质量 |
| param.payload_inertia | float64 | kg·m² | 负载惯性 |
| param.payload_mass_center_x | float64 | mm | 负载质心X坐标 |
| param.payload_mass_center_y | float64 | mm | 负载质心Y坐标 |
| param.payload_mass_center_z | float64 | mm | 负载质心Z坐标 |

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
| pos.position.x | float64 | m | X方向偏移（ROS标准单位） |
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
| coord | int32 | 1～4 | 坐标系序号：1=关节，2=直角，3=工具，4=用户 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_current_coord tl_ros2_interface/srv/SetCurrentCoord "{coord: 1}"
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
| mode | int32 | 1～4 | 拖拽模式：1=关闭，2=3D鼠标，3=力矩模式，4=位置模式 |

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
| success | bool | `true` = 拖拽已结束，`false` = 拖拽未结束 |
| message | string | 包含状态信息 |
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
## 11 全局路点管理接口
### 11.1 查询全局位点
| 功能描述 | 读取指定名称的全局路点数据，返回14维位置向量 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_global_pos` |
| 服务类型 | `tl_ros2_interface/srv/GetGlobalPos` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| pos_name | string | 全局路点名称，如 `"GP0002"` |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 失败时包含错误描述 |
| pos_info | float64[] | 14维位点数据：[0]坐标系 [1]单位制(1=度,2=弧度) [2]形态 [3]工具序号 [4]用户序号 [5][6]备用 [7~13]位点信息 |
#### 命令示例
```
ros2 service call /tl_driver/get_global_pos tl_ros2_interface/srv/GetGlobalPos "{pos_name: 'GP0002'}"
```
### 11.2 设置全局位点
| 功能描述 | 创建或修改一个持久化的全局位置变量（路点），可通过名称直接调用 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_global_pos` |
| 服务类型 | `tl_ros2_interface/srv/SetGlobalPos` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| pos_name | string | 全局路点名称，如 `"GP0002"` |
| pos_info | float64[] | 14维位点数据（格式同 [查询全局位点](#查询全局位点) 的 `pos_info`） |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_global_pos tl_ros2_interface/srv/SetGlobalPos \
'{
    pos_name: "GP0002", 
    pos_info: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 57.14, -32.93, 19.74, -89.89, -19.77, 0.0]
}'
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
| mode | int32 | 1～3 | 模式序号：1=示教模式，2=远程模式，3=运行模式 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_current_mode tl_ros2_interface/srv/SetCurrentMode "{mode: 2}"
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
| mode | int32 | 1～3 | 当前模式：1=示教，2=远程，3=运行 |
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
        target_pos_value: [0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0], 
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
| vmax | float64[] | °/s | 各轴最大速度，数组长度 ≥ 关节数 |
| amax | float64[] | °/s² | 各轴最大加速度 |
| jmax | float64[] | °/s³ | 各轴最大加加速度 |

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
    vmax: [300, 300, 300, 300, 300, 300, 300],
    amax: [3000, 3000, 3000, 3000, 3000, 3000, 3000],
    jmax: [50000, 50000, 50000, 50000, 50000, 50000, 50000]
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
| data | float64[] | ° | 目标关节角度数组（7维，前6轴角度，第7轴补0） |

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
    data: [0.0, 0.0, 0.0, 0.0, 0.0, 20.0, 0.0]
}"
```
## 16 位姿转换工具接口
### 16.1 四元数转欧拉角
| 功能描述 | 将四元数姿态转换为欧拉角（RPY）姿态 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_quat2rpy` |
| 服务类型 | `tl_ros2_interface/srv/GetPosTransform` |

**输入参数**
| 参数名 | 类型 | 长度 | 说明 |
|--------|------|------|------|
| input | float64[] | 4 | 四元数 [w, x, y, z]（无单位） |

**输出/返回值**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| success | bool | — | `true` = 转换成功，`false` = 转换失败 |
| message | string | — | 失败时包含错误描述 |
| output | float64[] | rad | 欧拉角 [rx, ry, rz] |
#### 命令示例
```
ros2 service call /tl_driver/get_quat2rpy tl_ros2_interface/srv/GetPosTransform \
"{
    input: [-0.080, 0.919, 0.365, 0.122]
}"
```
### 16.2 欧拉角转四元数
| 功能描述 | 将欧拉角（RPY）姿态转换为四元数姿态 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_rpy2quat` |
| 服务类型 | `tl_ros2_interface/srv/GetPosTransform` |

**输入参数**
| 参数名 | 类型 | 单位 | 长度 | 说明 |
|--------|------|------|------|------|
| input | float64[] | rad | 3 | 欧拉角 [rx, ry, rz] |

**输出/返回值**
| 参数名 | 类型 | 长度 | 说明 |
|--------|------|------|------|
| success | bool | — | `true` = 转换成功，`false` = 转换失败 |
| message | string | — | 失败时包含错误描述 |
| output | float64[] | 4 | 四元数 [w, x, y, z]（无单位） |
#### 命令示例
```
ros2 service call /tl_driver/get_rpy2quat tl_ros2_interface/srv/GetPosTransform \
"{
    input: [-2.9, 0.167, -0.777]
}"
```
### 16.3 欧拉角转旋转矩阵
| 功能描述 | 将欧拉角转换为3×3旋转矩阵（输出9个元素，行主序） |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_rpy2r` |
| 服务类型 | `tl_ros2_interface/srv/GetPosTransform` |

**输入参数**
| 参数名 | 类型 | 单位 | 长度 | 说明 |
|--------|------|------|------|------|
| input | float64[] | rad | 3 | 欧拉角 [rx, ry, rz] |

**输出/返回值**
| 参数名 | 类型 | 长度 | 说明 |
|--------|------|------|------|
| success | bool | — | `true` = 转换成功，`false` = 转换失败 |
| message | string | — | 失败时包含错误描述 |
| output | float64[] | 9 | 3×3旋转矩阵（行主序，无单位） |
#### 命令示例
```
ros2 service call /tl_driver/get_rpy2r tl_ros2_interface/srv/GetPosTransform \
"{
    input: [-2.9, 0.167, -0.777]
}"
```
### 16.4 位姿转旋转矩阵
| 功能描述 | 从4×4齐次变换矩阵（16个元素，行主序）中提取3×3旋转矩阵 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_tr2r` |
| 服务类型 | `tl_ros2_interface/srv/GetPosTransform` |

**输入参数**
| 参数名 | 类型 | 长度 | 说明 |
|--------|------|------|------|
| input | float64[] | 16 | 4×4齐次变换矩阵（行主序，无单位） |

**输出/返回值**
| 参数名 | 类型 | 长度 | 说明 |
|--------|------|------|------|
| success | bool | — | `true` = 转换成功，`false` = 转换失败 |
| message | string | — | 失败时包含错误描述 |
| output | float64[] | 9 | 3×3旋转矩阵（行主序，无单位） |
#### 命令示例
```
ros2 service call /tl_driver/get_tr2r tl_ros2_interface/srv/GetPosTransform \
"{
    input: [0.703, 0.691, 0.166, 0.000, 0.652, -0.720, 0.236, 0.000, 0.283, -0.057, -0.957, 0.000, 0.000, 0.000, 0.000, 1.000]
}"
```
### 16.5 旋转矩阵转位姿
| 功能描述 | 将3×3旋转矩阵扩展为4×4齐次变换矩阵（平移部分为零） |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_r2tr` |
| 服务类型 | `tl_ros2_interface/srv/GetPosTransform` |

**输入参数**
| 参数名 | 类型 | 长度 | 说明 |
|--------|------|------|------|
| input | float64[] | 9 | 3×3旋转矩阵（行主序，无单位） |

**输出/返回值**
| 参数名 | 类型 | 长度 | 说明 |
|--------|------|------|------|
| success | bool | — | `true` = 转换成功，`false` = 转换失败 |
| message | string | — | 失败时包含错误描述 |
| output | float64[] | 16 | 4×4齐次变换矩阵（行主序，无单位） |
#### 命令示例
```
ros2 service call /tl_driver/get_r2tr tl_ros2_interface/srv/GetPosTransform \
"{
    input: [0.703, 0.691, 0.166, 0.652, -0.720, 0.236, 0.283, -0.057, -0.957]
}"
```
