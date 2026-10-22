# tl_driver 接口测试清单

> 本文档列出 `tl_driver` 节点对外暴露的全部 ROS2 接口（服务/话题），并给出对应的测试指令。
> 启动前提：
> ```bash
> cd ~/Desktop/test/ros2/tl_robot_ros2
> source /opt/ros/humble/setup.bash
> source install/setup.bash
> ros2 launch tl_driver tl_tcb610_driver.launch.py
> ```
> 连接由驱动节点启动时自动建立、退出时自动断开，无需手工调用连接/断开服务。

---

## 一、连接 / 伺服（Trigger 服务，无参数）

| # | 接口名 | 类型 | 说明 |
|---|--------|------|------|
| 1 | `/tl_driver/power_on` | `std_srvs/srv/Trigger` | 上电使能 |
| 2 | `/tl_driver/power_off` | `std_srvs/srv/Trigger` | 下电 |
| 3 | `/tl_driver/clear_error` | `std_srvs/srv/Trigger` | 清除报警/错误 |
| 4 | `/tl_driver/get_controller_id` | `std_srvs/srv/Trigger` | 获取控制器序列号 |
| 5 | `/tl_driver/get_library_version` | `std_srvs/srv/Trigger` | 获取 SDK 库版本 |
| 6 | `/tl_driver/set_default_cartesian_param` | `std_srvs/srv/Trigger` | 恢复默认笛卡尔参数 |
| 7 | `/tl_driver/close_servoj` | `std_srvs/srv/Trigger` | 关闭关节跟踪模式 |
| 8 | `/tl_driver/queue_motion_stop` | `std_srvs/srv/Trigger` | 队列运动停止 |
| 9 | `/tl_driver/get_drag_status` | `std_srvs/srv/Trigger` | ⚠️ 获取拖拽状态（源码标注"用不了"） |

**测试指令（无参数服务通用格式）：**
```bash
ros2 service call /tl_driver/power_on std_srvs/srv/Trigger "{}"
ros2 service call /tl_driver/power_off std_srvs/srv/Trigger "{}"
ros2 service call /tl_driver/clear_error std_srvs/srv/Trigger "{}"
ros2 service call /tl_driver/get_library_version std_srvs/srv/Trigger "{}"
ros2 service call /tl_driver/get_controller_id std_srvs/srv/Trigger "{}"
```

---

## 二、速度 / 模式

### `/tl_driver/set_speed` — `tl_ros2_interface/srv/SetSpeed`
请求：`float64 speed`
```bash
ros2 service call /tl_driver/set_speed tl_ros2_interface/srv/SetSpeed "{speed: 30.0}"
```

### 运行速度 — `/arm_status`
话题，`speed` / `speed_valid` 字段（1 Hz 查询缓存）
```bash
ros2 topic echo /arm_status --field speed
ros2 topic echo /arm_status --field speed_valid
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

## 六、状态查询

| # | 接口名 | 类型 | 请求参数 | 说明 |
|---|--------|------|----------|------|
| 1 | `/arm_status` | `ArmStatus` | — | `run_state` / `speed` / `speed_valid` |
| 2 | `/tl_driver/joint_dynamics` | `JointDynamics` | — | 线速度、关节速度、电流、力矩（‰ 额定扭矩） |
| 3 | `/tl_driver/joint_thermal` | `JointThermal` | — | 温度、本体/外部轴电压 |
| 4 | `/tl_driver/get_pos_reachable` | `GetPosReachable` | `pos/move_type` | 点位是否可达 |

**测试指令：**
```bash
ros2 topic echo /arm_status --once
ros2 topic echo /tl_driver/joint_dynamics --once
ros2 topic echo /tl_driver/joint_thermal --once
ros2 service call /tl_driver/get_pos_reachable tl_ros2_interface/srv/GetPosReachable "{pos: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0], move_type: 'MOVJ'}"
```

---

## 七、参数（DH / 关节 / 控制器）

| # | 接口名 | 类型 | 说明 |
|---|--------|------|------|
| 1 | `/tl_driver/get_dh_param` | `GetDHParam` | 获取 DH 参数 |
| 2 | `/tl_driver/get_robot_joint_param` | `GetRobotJointParam`（`id`） | 获取关节参数 |
| 3 | `/tl_driver/set_controller_ip` | `SetControllerIP` | 配置控制器网口 IP |

**测试指令：**
```bash
ros2 service call /tl_driver/get_dh_param tl_ros2_interface/srv/GetDHParam "{}"
ros2 service call /tl_driver/get_robot_joint_param tl_ros2_interface/srv/GetRobotJointParam "{id: 1}"
ros2 service call /tl_driver/set_controller_ip tl_ros2_interface/srv/SetControllerIP "{name: 'eth0', addr: '192.168.1.13', gateway: '192.168.1.1', dns: '8.8.8.8'}"
```

> ⚠️ `set_controller_ip` 会修改控制器网络，测试后需重新连接。

---

## 八、IO / Modbus

### `/tl_driver/set_digital_output` — `SetDigitalOutput`（port/value）
```bash
ros2 service call /tl_driver/set_digital_output tl_ros2_interface/srv/SetDigitalOutput "{port: 1, value: 1}"
```

### `/tl_driver/get_digital_input_output` — `GetDigitalInputOutput`
数字 IO 的唯一读取路径（不发布 `io_state` 话题）
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

## 十、工具 / 用户坐标 / 零位

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

1. **连接类**：`get_library_version` → `get_controller_id` → `get_current_mode` → `get_current_coord`
2. **状态类**：`/arm_status`、`/tl_driver/joint_dynamics`、`/tl_driver/joint_thermal` 话题（只读，安全）
3. **配置类**：`set_speed` / `set_current_mode` / `set_current_coord` / `set_drag_mode`
4. **运动类**（⚠️ 先确认周围安全、上电后）：`power_on` → `moveJ`/`moveL` 话题 → `power_off`
5. **IO/Modbus/轨迹**：按需单独测试
