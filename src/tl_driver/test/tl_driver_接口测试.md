# tl_driver 接口测试清单

> 本文档列出 `tl_driver` 节点对外暴露的全部 ROS2 接口（服务/话题），并给出对应的测试指令。
> 启动前提：
> ```bash
> cd ~/Desktop/test/ros2/tl_robot_ros2
> source /opt/ros/humble/setup.bash
> source install/setup.bash
> ros2 launch tl_driver tl_tcb610_driver.launch.py
> ```
> 测试前先 `ros2 service call /tl_driver/connect_arm std_srvs/srv/Trigger "{}"` 连接机械臂。

---

## 一、连接 / 伺服（Trigger 服务，无参数）

| # | 接口名 | 类型 | 说明 |
|---|--------|------|------|
| 1 | `/tl_driver/connect_arm` | `std_srvs/srv/Trigger` | 连接机械臂 |
| 2 | `/tl_driver/disconnect_arm` | `std_srvs/srv/Trigger` | 断开机械臂 |
| 3 | `/tl_driver/power_on` | `std_srvs/srv/Trigger` | 上电使能 |
| 4 | `/tl_driver/power_off` | `std_srvs/srv/Trigger` | 下电 |
| 5 | `/tl_driver/clear_error` | `std_srvs/srv/Trigger` | 清除报警/错误 |
| 6 | `/tl_driver/get_controller_id` | `std_srvs/srv/Trigger` | 获取控制器序列号 |
| 7 | `/tl_driver/get_library_version` | `std_srvs/srv/Trigger` | 获取 SDK 库版本 |
| 8 | `/tl_driver/get_nexmotion_lib_version` | `std_srvs/srv/Trigger` | 获取算法库版本 |
| 9 | `/tl_driver/set_default_cartesian_param` | `std_srvs/srv/Trigger` | 恢复默认笛卡尔参数 |
| 10 | `/tl_driver/close_servoj` | `std_srvs/srv/Trigger` | 关闭关节跟踪模式 |
| 11 | `/tl_driver/queue_motion_stop` | `std_srvs/srv/Trigger` | 队列运动停止 |
| 12 | `/tl_driver/get_drag_status` | `std_srvs/srv/Trigger` | ⚠️ 获取拖拽状态（源码标注"用不了"） |

**测试指令（无参数服务通用格式）：**
```bash
ros2 service call /tl_driver/connect_arm std_srvs/srv/Trigger "{}"
ros2 service call /tl_driver/power_on std_srvs/srv/Trigger "{}"
ros2 service call /tl_driver/power_off std_srvs/srv/Trigger "{}"
ros2 service call /tl_driver/clear_error std_srvs/srv/Trigger "{}"
ros2 service call /tl_driver/get_library_version std_srvs/srv/Trigger "{}"
ros2 service call /tl_driver/get_controller_id std_srvs/srv/Trigger "{}"
ros2 service call /tl_driver/get_nexmotion_lib_version std_srvs/srv/Trigger "{}"
ros2 service call /tl_driver/disconnect_arm std_srvs/srv/Trigger "{}"
```

---

## 二、速度 / 模式

### `/tl_driver/set_speed` — `tl_ros2_interface/srv/SetSpeed`
请求：`float64 speed`
```bash
ros2 service call /tl_driver/set_speed tl_ros2_interface/srv/SetSpeed "{speed: 30.0}"
```

### `/tl_driver/get_speed` — `tl_ros2_interface/srv/GetSpeed`
无请求参数
```bash
ros2 service call /tl_driver/get_speed tl_ros2_interface/srv/GetSpeed "{}"
```

### `/tl_driver/set_current_mode` — `tl_ros2_interface/srv/SetCurrentMode`
请求：`int32 mode`（0=示教 1=远程 2=运行）
```bash
ros2 service call /tl_driver/set_current_mode tl_ros2_interface/srv/SetCurrentMode "{mode: 1}"
```

### `/tl_driver/get_current_mode` — `tl_ros2_interface/srv/GetCurrentMode`
无请求参数
```bash
ros2 service call /tl_driver/get_current_mode tl_ros2_interface/srv/GetCurrentMode "{}"
```

---

## 三、运动（话题）

### 订阅话题（发指令给机械臂）

**`/tl_driver/moveJ` — `tl_ros2_interface/msg/MoveCommand`（关节运动）**
```bash
ros2 topic pub -1 /tl_driver/moveJ tl_ros2_interface/msg/MoveCommand "{
  target_pos_type: 0,
  target_pos_value: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  coord: 0,
  velocity: 30.0,
  acc: 20.0,
  dec: 20.0,
  pl: 0
}"
```

**`/tl_driver/moveL` — `tl_ros2_interface/msg/MoveCommand`（直线运动）**
```bash
ros2 topic pub -1 /tl_driver/moveL tl_ros2_interface/msg/MoveCommand "{
  target_pos_type: 0,
  target_pos_value: [300.0, 0.0, 200.0, 3.14, 0.0, 0.0, 0.0],
  coord: 1,
  velocity: 100.0,
  acc: 20.0,
  dec: 20.0,
  pl: 0
}"
```

**`/tl_driver/set_servoj_pos` — `std_msgs/msg/Float64MultiArray`（servoJ 关节角跟踪）**
```bash
ros2 topic pub -1 /tl_driver/set_servoj_pos std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

**`/tl_driver/set_servol_pos` — `tl_ros2_interface/msg/ServolMove`（servoL 笛卡尔直线伺服）**
```bash
ros2 topic pub -1 /tl_driver/set_servol_pos tl_ros2_interface/msg/ServolMove "{
  target_pose: [300.0, 0.0, 200.0, 3.14, 0.0, 0.0],
  step_size: 2.0,
  coord: 1
}"
```

### 发布话题（读取机械臂状态）

| # | 话题 | 类型 | 说明 |
|---|------|------|------|
| 1 | `/joint_states` | `sensor_msgs/msg/JointState` | 关节状态（关节角等） |
| 2 | `/tcp_pose` | `tl_ros2_interface/msg/CartesianPose` | TCP 位姿（位置+RPY+臂角） |
| 3 | `/arm_status` | `tl_ros2_interface/msg/ArmStatus` | 机械臂运行状态字符串 |

**测试指令：**
```bash
ros2 topic echo /joint_states
ros2 topic echo /tcp_pose
ros2 topic echo /arm_status
```

---

## 四、点动 / 拖拽

### `/tl_driver/start_jogging` — `tl_ros2_interface/srv/Jogging`
请求：`int32 axis`（轴号 1-6）、`bool direction`（true=正方向）
```bash
ros2 service call /tl_driver/start_jogging tl_ros2_interface/srv/Jogging "{axis: 1, direction: true}"
```

### `/tl_driver/stop_jogging` — `tl_ros2_interface/srv/Jogging`
```bash
ros2 service call /tl_driver/stop_jogging tl_ros2_interface/srv/Jogging "{axis: 1, direction: false}"
```

### `/tl_driver/set_drag_mode` — `tl_ros2_interface/srv/SetDragMode`
请求：`int32 mode`（0=无 1=3D鼠标 2=力矩 3=位置）
```bash
ros2 service call /tl_driver/set_drag_mode tl_ros2_interface/srv/SetDragMode "{mode: 2}"
```

---

## 五、坐标系 / 坐标转换

### `/tl_driver/set_current_coord` — `SetCurrentCoord`（coord: 0=关节 1=直角 2=工具 3=用户）
```bash
ros2 service call /tl_driver/set_current_coord tl_ros2_interface/srv/SetCurrentCoord "{coord: 1}"
```

### `/tl_driver/get_current_coord` — `GetCurrentCoord`
```bash
ros2 service call /tl_driver/get_current_coord tl_ros2_interface/srv/GetCurrentCoord "{}"
```

### `/tl_driver/set_coord_num` — `SetCoordNum`（tool_num/user_num）
```bash
ros2 service call /tl_driver/set_coord_num tl_ros2_interface/srv/SetCoordNum "{tool_num: 1, user_num: 1}"
```

### `/tl_driver/get_coord_num` — `GetCoordNum`
```bash
ros2 service call /tl_driver/get_coord_num tl_ros2_interface/srv/GetCoordNum "{}"
```

### `/tl_driver/coord_transform` — `CoordTransform`
请求：`origin_coord`、`target_coord`、`form`、`origin_pos`、`reference_pos`
```bash
ros2 service call /tl_driver/coord_transform tl_ros2_interface/srv/CoordTransform "{
  origin_coord: 0,
  target_coord: 1,
  form: 0,
  origin_pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  reference_pos: []
}"
```

### 位姿转换 5 个服务（均用 `GetPosTransform`，请求 `float64[] input`）

| # | 接口名 | 说明 |
|---|--------|------|
| 1 | `/tl_driver/get_quat2rpy` | 四元数→欧拉角 |
| 2 | `/tl_driver/get_rpy2quat` | 欧拉角→四元数 |
| 3 | `/tl_driver/get_rpy2r` | 欧拉角→旋转矩阵 |
| 4 | `/tl_driver/get_tr2r` | 位姿矩阵→旋转矩阵 |
| 5 | `/tl_driver/get_r2tr` | 旋转矩阵→位姿矩阵 |

```bash
ros2 service call /tl_driver/get_quat2rpy tl_ros2_interface/srv/GetPosTransform "{input: [0.0, 0.0, 0.0, 1.0]}"
ros2 service call /tl_driver/get_rpy2quat tl_ros2_interface/srv/GetPosTransform "{input: [0.0, 0.0, 0.0]}"
ros2 service call /tl_driver/get_rpy2r tl_ros2_interface/srv/GetPosTransform "{input: [0.0, 0.0, 0.0]}"
ros2 service call /tl_driver/get_tr2r tl_ros2_interface/srv/GetPosTransform "{input: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]}"
ros2 service call /tl_driver/get_r2tr tl_ros2_interface/srv/GetPosTransform "{input: [1,0,0, 0,1,0, 0,0,1]}"
```

---

## 六、状态查询

| # | 接口名 | 类型 | 请求参数 | 说明 |
|---|--------|------|----------|------|
| 1 | `/tl_driver/get_robot_state` | `GetRobotState` | `channel/stop/mode/interval/io_state/position/detail_motion_pos/pos_sum/io_port/optional` | 7000 端口状态 |
| 2 | `/tl_driver/get_joint_temperature` | `GetJointTemperature` | 无 | 关节温度 |
| 3 | `/tl_driver/get_joint_voltage` | `GetJointVoltage` | 无 | 关节电压 |
| 4 | `/tl_driver/get_motor_current` | `GetMotorCurrent` | 无 | 电机电流 |
| 5 | `/tl_driver/get_current_motor_torque` | `GetCurrentMotorTorque` | 无 | 电机力矩 |
| 6 | `/tl_driver/get_current_line_joint_speed` | `GetCurrentLineJointSpeed` | 无 | 末端线速度+关节速度 |
| 7 | `/tl_driver/get_joint_software_version` | `GetJointSoftwareVersion` | `axis_num` | 关节软件版本 |
| 8 | `/tl_driver/get_pos_reachable` | `GetPosReachable` | `pos/move_type` | 点位是否可达 |

**测试指令：**
```bash
ros2 service call /tl_driver/get_robot_state tl_ros2_interface/srv/GetRobotState "{channel: 0, stop: false, mode: 0, interval: 100, io_state: false, position: 0, detail_motion_pos: false, pos_sum: 0, io_port: [], optional: []}"
ros2 service call /tl_driver/get_joint_temperature tl_ros2_interface/srv/GetJointTemperature "{}"
ros2 service call /tl_driver/get_joint_voltage tl_ros2_interface/srv/GetJointVoltage "{}"
ros2 service call /tl_driver/get_motor_current tl_ros2_interface/srv/GetMotorCurrent "{}"
ros2 service call /tl_driver/get_current_motor_torque tl_ros2_interface/srv/GetCurrentMotorTorque "{}"
ros2 service call /tl_driver/get_current_line_joint_speed tl_ros2_interface/srv/GetCurrentLineJointSpeed "{}"
ros2 service call /tl_driver/get_joint_software_version tl_ros2_interface/srv/GetJointSoftwareVersion "{axis_num: 1}"
ros2 service call /tl_driver/get_pos_reachable tl_ros2_interface/srv/GetPosReachable "{pos: [0.0,0.0,0.0,0.0,0.0,0.0,0.0], move_type: 'MOVJ'}"
```

---

## 七、参数（DH / 关节 / 控制器）

| # | 接口名 | 类型 | 说明 |
|---|--------|------|------|
| 1 | `/tl_driver/get_dh_param` | `GetDHParam` | 获取 DH 参数 |
| 2 | `/tl_driver/set_dh_param` | `SetDHParam`（`RobotDHParam param`） | 设置 DH 参数 |
| 3 | `/tl_driver/restore_default_dh_param` | `RestoreDefaultDHParam`（`robot_num`） | 恢复默认 DH |
| 4 | `/tl_driver/get_robot_joint_param` | `GetRobotJointParam`（`id`） | 获取关节参数 |
| 5 | `/tl_driver/set_robot_joint_param` | `SetRobotJointParam`（`id`+`RobotJointParam`） | 设置关节参数 |
| 6 | `/tl_driver/set_controller_ip` | `SetControllerIP` | 配置控制器网口 IP |

**测试指令：**
```bash
ros2 service call /tl_driver/get_dh_param tl_ros2_interface/srv/GetDHParam "{}"
ros2 service call /tl_driver/restore_default_dh_param tl_ros2_interface/srv/RestoreDefaultDHParam "{robot_num: 1}"
ros2 service call /tl_driver/get_robot_joint_param tl_ros2_interface/srv/GetRobotJointParam "{id: 1}"
ros2 service call /tl_driver/set_controller_ip tl_ros2_interface/srv/SetControllerIP "{name: 'eth0', addr: '192.168.1.13', gateway: '192.168.1.1', dns: '8.8.8.8'}"
```

> ⚠️ `set_dh_param` / `set_robot_joint_param` 涉及结构体参数，改动有风险，测试请谨慎（先 `get` 备份原值）。

---

## 八、IO / Modbus

### `/tl_driver/set_digital_output` — `SetDigitalOutput`（port/value）
```bash
ros2 service call /tl_driver/set_digital_output tl_ros2_interface/srv/SetDigitalOutput "{port: 1, value: 1}"
```

### `/tl_driver/get_digital_input_output` — `GetDigitalInputOutput`
```bash
ros2 service call /tl_driver/get_digital_input_output tl_ros2_interface/srv/GetDigitalInputOutput "{}"
```

### `/tl_driver/modbus_read` — `ModbusRead`
```bash
ros2 service call /tl_driver/modbus_read tl_ros2_interface/srv/ModbusRead "{master_id: 1, addr: 0, quantity: 1, master_param: {type: 'tcp', start_addr: false, tcp: {ip: '192.168.1.100', port: 502}, rtu: {serial_port: '/dev/ttyUSB0', baudrate: 9600}}}"
```

### `/tl_driver/modbus_write` — `ModbusWrite`
```bash
ros2 service call /tl_driver/modbus_write tl_ros2_interface/srv/ModbusWrite "{master_id: 1, addr: 0, data: [1], master_param: {type: 'tcp', start_addr: false, tcp: {ip: '192.168.1.100', port: 502}, rtu: {serial_port: '/dev/ttyUSB0', baudrate: 9600}}}"
```

---

## 九、作业文件

| # | 接口名 | 类型 | 说明 |
|---|--------|------|------|
| 1 | `/tl_driver/get_all_job_filename` | `GetAllJobFileName` | 获取所有作业文件名 |
| 2 | `/tl_driver/job_run` | `JobRun`（`job_name`） | 运行作业 |
| 3 | `/tl_driver/job_delete` | `JobRun`（`job_name`） | 删除作业 |
| 4 | `/tl_driver/job_insert_moveJ` | `JobInsertMove` | 插入 MoveJ 指令 |
| 5 | `/tl_driver/job_insert_moveL` | `JobInsertMove` | 插入 MoveL 指令 |
| 6 | `/tl_driver/job_insert_moveC` | `JobInsertMove` | 插入 MoveC 指令 |
| 7 | `/tl_driver/job_insert_imove` | `JobInsertMove` | 插入 iMove 指令 |

**测试指令：**
```bash
ros2 service call /tl_driver/get_all_job_filename tl_ros2_interface/srv/GetAllJobFileName "{}"
ros2 service call /tl_driver/job_run tl_ros2_interface/srv/JobRun "{job_name: 'test.job'}"
ros2 service call /tl_driver/job_delete tl_ros2_interface/srv/JobRun "{job_name: 'test.job'}"
```

---

## 十、全局点位 / 工具 / 用户坐标 / 零位

### `/tl_driver/set_global_pos` — `SetGlobalPos`（pos_name/pos_info）
```bash
ros2 service call /tl_driver/set_global_pos tl_ros2_interface/srv/SetGlobalPos "{pos_name: 'GP0001', pos_info: [1,0,0,1,1,0,0, 300.0,0.0,200.0,3.14,0.0,0.0,0.0]}"
```

### `/tl_driver/get_global_pos` — `GetGlobalPos`（pos_name）
```bash
ros2 service call /tl_driver/get_global_pos tl_ros2_interface/srv/GetGlobalPos "{pos_name: 'GP0001'}"
```

### `/tl_driver/set_tool_param` — `SetToolParam`（tool_num + ToolParam）
```bash
ros2 service call /tl_driver/set_tool_param tl_ros2_interface/srv/SetToolParam "{tool_num: 1, param: {x: 0.0, y: 0.0, z: 0.0, a: 0.0, b: 0.0, c: 0.0, payload_mass: 0.0, payload_inertia: 0.0, payload_mass_center_x: 0.0, payload_mass_center_y: 0.0, payload_mass_center_z: 0.0}}"
```

### `/tl_driver/set_user_coord` — `SetUserCoord`（user_num + CartesianPose）
```bash
ros2 service call /tl_driver/set_user_coord tl_ros2_interface/srv/SetUserCoord "{user_num: 1, pos: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, position: {x: 0.0, y: 0.0, z: 0.0}, rpy: {x: 0.0, y: 0.0, z: 0.0}, arm_angle: 0.0}}"
```

### `/tl_driver/set_axis_zero_pos` — `SetAxisZeroPos`（axis）
```bash
ros2 service call /tl_driver/set_axis_zero_pos tl_ros2_interface/srv/SetAxisZeroPos "{axis: 1}"
```

---

## 十一、轨迹记录 / 回放

### `/tl_driver/track_save` — `TrackSave`（traj_name）
```bash
ros2 service call /tl_driver/track_save tl_ros2_interface/srv/TrackSave "{traj_name: 'traj1'}"
```

### `/tl_driver/track_playback` — `TrackPlayback`（vel）
```bash
ros2 service call /tl_driver/track_playback tl_ros2_interface/srv/TrackPlayback "{vel: 50}"
```

---

## 十二、servoJ / 队列运动

### `/tl_driver/open_servoj` — `OpenServoJ`（vmax/amax/jmax）
```bash
ros2 service call /tl_driver/open_servoj tl_ros2_interface/srv/OpenServoJ "{vmax: [90.0,90.0,90.0,90.0,90.0,90.0,90.0], amax: [100.0,100.0,100.0,100.0,100.0,100.0,100.0], jmax: [200.0,200.0,200.0,200.0,200.0,200.0,200.0]}"
```

### `/tl_driver/close_servoj` — `std_srvs/srv/Trigger`
```bash
ros2 service call /tl_driver/close_servoj std_srvs/srv/Trigger "{}"
```

### `/tl_driver/queue_motion_set_status` — `QueueMotionSetStatus`（status）
```bash
ros2 service call /tl_driver/queue_motion_set_status tl_ros2_interface/srv/QueueMotionSetStatus "{status: true}"
```

### `/tl_driver/queue_motion_movej` — `QueueMotionMoveJ`（is_continue + MoveCommand）
```bash
ros2 service call /tl_driver/queue_motion_movej tl_ros2_interface/srv/QueueMotionMoveJ "{
  is_continue: false,
  cmd: {target_pos_type: 0, target_pos_value: [20.0,10.0,-10.0,0.0,0.0,0.0,0.0], coord: 0, velocity: 30.0, acc: 20.0, dec: 20.0, pl: 0}
}"
```

### `/tl_driver/queue_motion_stop` — `std_srvs/srv/Trigger`
```bash
ros2 service call /tl_driver/queue_motion_stop std_srvs/srv/Trigger "{}"
```

---

## 十三、日志下载

### `/tl_driver/log_download` — `LogDownload`（count/directory_path）
```bash
ros2 service call /tl_driver/log_download tl_ros2_interface/srv/LogDownload "{count: 100, directory_path: '/tmp'}"
```

---

## 附：常用检查命令

```bash
# 查看节点与全部接口
ros2 node info /tl_driver
# 列出服务
ros2 service list
# 列出话题
ros2 topic list
# 查看接口类型定义
ros2 interface show tl_ros2_interface/msg/MoveCommand
ros2 interface show tl_ros2_interface/srv/SetSpeed
```

## 测试顺序建议（安全优先）

1. **连接类**：`connect_arm` → `get_library_version` → `get_controller_id` → `get_speed` → `get_current_mode` → `get_current_coord`
2. **状态类**：`get_joint_temperature` / `get_joint_voltage` / `get_motor_current` / `get_current_motor_torque` / `get_current_line_joint_speed`（只读，安全）
3. **配置类**：`set_speed` / `set_current_mode` / `set_current_coord` / `set_drag_mode`
4. **运动类**（⚠️ 先确认周围安全、上电后）：`power_on` → `moveJ`/`moveL` 话题 → `power_off`
5. **IO/Modbus/作业/轨迹**：按需单独测试
