# 天链机械臂 ROS2 功能包使用说明书

四川天链机器人股份有限公司

文件修订记录：

| 版本号 | 时间      | 备注                        |
| :----: | :-------- | :-------------------------- |
|  V1.0  | 2026-6-24 | 拟制                        |
|  V1.1  | 2026-7-08 | 补充多方块/循环转移模式说明 |

# TL_Vision — 视觉抓取系统

基于 YOLOv8 + RealSense D435 + 天链机械臂的视觉伺服抓取系统。支持单块视觉跟随、多块连续抓取、A↔B 循环转移等模式。

---

## 系统要求

| 依赖          | 版本/说明                             |
| ------------- | ------------------------------------- |
| ROS2          | Humble                                |
| Python        | 3.10+                                 |
| CUDA          | 11.8+（推荐 12.x，YOLO 推理加速）     |
| RealSense SDK | librealsense2                         |
| 相机          | Intel RealSense D435                  |
| 夹爪          | 通过机械臂 RS485 口通信（Modbus RTU） |

### Python 依赖

- `ultralytics`（YOLOv8）
- `opencv-python`
- `pyrealsense2`
- `scipy`
- `cv_bridge`

---

## 启动步骤

> 启动前确保已 `source install/setup.bash`

### 1️⃣ 启动 RealSense 相机

```bash
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args --params-file src/tl_vision/config/camera_params.yaml
```

### 2️⃣ 启动 YOLO 检测

```bash
ros2 run tl_vision yolo_node \
  --ros-args --params-file src/tl_vision/config/yolo_node.yaml
```

### 3️⃣ 启动机械臂驱动

```bash
ros2 launch tl_driver tl_tcb610_driver.launch.py
```

### 4️⃣ 启动主控节点

```bash
ros2 launch tl_vision control_node.launch.py
```

### 一键启动

```bash
ros2 launch tl_vision bringup.launch.py arm_type:=tcb610
```

一键启动同时启动以下节点：RealSense 相机、YOLO 检测、机械臂驱动、主控节点。

---

## 操作说明

| 按键  | 功能                                           |
| ----- | ---------------------------------------------- |
| `c` | 切换视觉跟随模式（物体静止 2 秒后自动抓取）    |
| `m` | 多块连续抓取模式（自动计数 + 逐个抓取 + 放置） |
| `r` | 循环转移模式（A↔B 堆叠交换）                  |
| `z` | 回零点                                         |
| `o` | 夹爪张开                                       |
| `p` | 夹爪闭合                                       |
| `3` | 查看夹爪参数（端口、速度、力度、开口度等）     |
| `q` | 退出程序                                       |

---

## 节点说明

### yolo_node

YOLOv8 物体检测节点。

- 订阅：RealSense RGB + 深度图
- 发布：`/tl_vision/object_3d_pos_camera`（`ObjectInfo`，相机坐标系下的物体 3D 位置）
- 通过深度图获取物体深度，计算 3D 坐标
- 支持按物体类型过滤（YOLO class_id）

### control_node

主控节点，集成了坐标变换、轨迹规划、抓取控制。

**订阅话题：**

| 话题                                | 类型                                | 说明           |
| ----------------------------------- | ----------------------------------- | -------------- |
| `/tl_vision/object_3d_pos_camera` | `tl_ros2_interface/ObjectInfo`    | YOLO 检测结果  |
| `/joint_states`                   | `sensor_msgs/JointState`          | 机械臂关节状态 |
| `/tcp_pose`                       | `tl_ros2_interface/CartesianPose` | TCP 位姿       |
| `/tf`                             | `tf2_msgs/TFMessage`              | 坐标变换       |

**发布话题：**

| 话题                              | 类型                             | 说明                |
| --------------------------------- | -------------------------------- | ------------------- |
| `/tl_vision/object_3d_pos_base` | `tl_ros2_interface/ObjectInfo` | 基坐标系物体位置    |
| `/tl_driver/set_servoj_pos`     | `std_msgs/Float64MultiArray`   | ServoJ 关节角度指令 |

### calib_node

手眼标定节点（hand-eye calibration）。

- 支持在线/离线标定
- 使用 TSAI 等方法计算相机到工具（TCP）的变换矩阵
- 标定结果写入 `control_node.yaml` 的 `handeye_matrix` 参数

```bash
ros2 run tl_vision calib_node
```

### fk_test_node

正运动学测试节点。使用 Tkinter 提供 GUI 界面，可手动拖动滑块控制各关节角度，验证运动学解算是否正确。

```bash
ros2 run tl_vision fk_test_node
```

---

## 配置说明

### 主控参数（`config/control_node.yaml`）

| 参数                        | 默认值 | 说明                                      |
| --------------------------- | ------ | ----------------------------------------- |
| `fixed_approach_height`   | 302.26 | 视觉跟踪高度（mm），此高度跟踪物体 XY     |
| `fixed_grasp_height`      | 166.33 | 抓取高度（mm），跟踪到位后下到此高度抓取  |
| `grasp_forward_offset`    | 0.004  | 抓取前向偏移（米），补偿相机与 TCP 的偏差 |
| `servoj_interp_points`    | 500    | ServoJ 插值点数（越大运动越慢越平滑）     |
| `tracking_interp_points`  | 30     | 视觉跟踪插值点数                          |
| `object_stable_threshold` | 2.0    | 物体静止多久（秒）后触发抓取              |
| `cam_stable_frames`       | 10     | 连续稳定帧数阈值                          |
| `cam_stable_threshold`    | 0.02   | 基坐标变化 < 此值（米）算稳定             |
| `servoj_loss_timeout`     | 3.0    | YOLO 目标丢失超时（秒），超时回零         |
| `handeye_matrix`          | —     | 手眼标定矩阵（4×4，相机→TCP）           |

### 夹爪参数（`config/gripper.yaml`）

| 参数                            | 默认值 | 说明                           |
| ------------------------------- | ------ | ------------------------------ |
| `gripper_port`                | 2      | 机械臂 RS485 端口号            |
| `gripper_slave_id`            | 1      | Modbus 从站 ID                 |
| `gripper_baudrate`            | 115200 | 串口波特率                     |
| `gripper_speed`               | 700    | 夹爪运动速度 (10-1000)         |
| `gripper_force`               | 400    | 夹持力 (100-1000g)             |
| `gripper_open_position`       | 1000   | 张开位置（0-1000，1000≈70mm） |
| `gripper_close_position`      | 0      | 闭合位置                       |
| `gripper_grasp_wait_duration` | 1.5    | 抓取闭合后等待回零的时间（秒） |

### YOLO 参数（`config/yolo_node.yaml`）

| 参数                     | 说明                |
| ------------------------ | ------------------- |
| `model_path`           | YOLO 模型权重路径   |
| `confidence_threshold` | 检测置信度阈值      |
| `object_class_id`      | 目标物体的 class_id |
| `camera_width/height`  | 输入图像分辨率      |

### 相机参数（`config/camera_params.yaml`）

| 参数                              | 说明              |
| --------------------------------- | ----------------- |
| `camera.color.width/height/fps` | RGB 流分辨率/帧率 |
| `camera.depth.width/height/fps` | 深度流分辨率/帧率 |
| `enable_ir`                     | 是否启用红外      |

---

## 工作模式详解

### 单块跟随模式（按 `c`）

1. 按下 `c` 后启用视觉跟随
2. 机械臂在 `fixed_approach_height` 高度跟随物体 XY 运动
3. `object_stable_threshold` 秒后物体静止 → 自动触发抓取
4. 下到 `fixed_grasp_height` → 闭合夹爪 → 等待 `gripper_grasp_wait_duration` → 回零

### 多块连续抓取模式（按 `m`）

自动检测桌面上所有同类型方块，逐个抓取并放置到预定位置。

1. **计数阶段：** 启动后 3 秒内统计 YOLO 检测到的方块数量
2. **逐个抓取：** 每次锁定一个方块 → 跟踪 → 抓取 → 放置 → 返回
3. **验证机制：** 抓取后再次检测，若方块数量减少则抓取成功
4. **完成：** 所有方块抓完或连续 3 次失败后退出

```yaml
# 放置位置（6个点，每块一个，格式 [x, y, z, rx, ry, rz, posture]）
place_positions: [
  227.67, 353.11, 178.31, 3.14, 0.0, 0.919, 0.0,
  252.45, 322.77, 178.31, 3.14, 0.0, 0.919, 0.0,
  ...
]
```

### 循环转移模式（按 `r`，A↔B 堆叠交换）

适用于流水线场景：将 A 侧 6 个方块逐个转移到 B 侧，再从 B 侧转移到 A 侧，循环往复。

**每个方块 4 个阶段：**

```
取堆叠块 ──► 散落区释放 ──► 视觉跟踪抓取 ──► 放置到目标
（预设位置）   （空爪松爪）   （YOLO找块抓取）   （放到对面）
```

1. **取堆叠块**（`_cycle_pick_from_stack`）：

   - 移动到堆叠位置（Z-2mm），闭合夹爪
   - 若夹爪读数 < `gripper_grasp_threshold` 则视为成功
   - 失败则回零结束进程
2. **散落区释放**（`_cycle_drop_at_zone`）：

   - 移动到 `drop_zone_pose` 位置
   - 张开夹爪释放抓取的方块（用于清空夹爪中的堆叠块）
   - 回零
3. **视觉跟踪抓取**（`_cycle_visual_track_grasp`）：

   - 等 YOLO 数据 → 跟踪 → 稳定检测触发 → 下到抓取高度 → 闭爪
   - 最多重试 3 次，单次 5 秒无数据则结束进程
4. **放置到目标**（`_cycle_place_at_target`）：

   - IK 解算 → ServoJ 移动到对面侧对应位置
   - 松爪 → 回零

**取块顺序：**

| 方向 | 取块顺序                                       |
| ---- | ---------------------------------------------- |
| A→B | 先取 A1 三角区：索引 2→1→0，再取 A2：5→4→3 |
| B→A | 先取 B2 三角区：索引 5→4→3，再取 B1：2→1→0 |

**完成后：** 切换方向（A↔B），循环往复。

---

## 夹爪控制

### 通过机械臂 RS485（推荐）

夹爪通过机械臂控制器的 RS485 端口 2 通信，使用 Modbus RTU 协议。

- **驱动方式：** 无需 USB 串口线，通过 `tl_driver` 的 Modbus 服务读写夹爪寄存器
- **Python 接口：** `test_gripper_arm.py` 中的 `ArmGripper` 类
- **测试脚本：**

```bash
# 终端1：启动驱动
ros2 launch tl_driver tl_tcb610_driver.launch.py

# 终端2：运行测试
cd src/tl_vision/tl_vision
python3 test_gripper_arm.py
```

### USB 直连（备用）

使用 USB 转 485 串口线直接连接夹爪，适用于调试。

```bash
# 需要先配置串口权限
sudo python3 src/tl_vision/tl_vision/zhua.py
```

### Modbus 寄存器说明

| 寄存器     | 地址 | 说明                     |
| ---------- | ---- | ------------------------ |
| 开口度设置 | 10   | 写入目标开口度（0-1000） |
| 速度设置   | 11   | 写入运动速度（10-1000）  |
| 力度设置   | 12   | 写入夹持力               |
| 状态读取   | 65   | 读取当前开口度（只读）   |

---

## 坐标系说明

```
相机坐标 ──[手眼标定矩阵]──► TCP坐标 ──[正运动学]──► 基坐标
      T_tool_camera                 T_base_tool
```

- `handeye_matrix`：相机→TCP 的变换矩阵（4×4），通过 `calib_node` 标定
- `T_base_tool`：根据机械臂当前关节角度实时计算
- 物体最终位置：`P_base = T_base_tool @ T_tool_camera @ P_camera`

---

## 抓取参数调优建议

| 现象               | 调优方向                                                        |
| ------------------ | --------------------------------------------------------------- |
| 抓取时夹爪撞到物体 | 降低 `servoj_interp_points` 或增大 `fixed_grasp_height`     |
| 跟踪抖动           | 降低 `tracking_interp_points` 或增大指数平滑 alpha            |
| 跟踪跟不上         | 增大 `tracking_interp_points` 或减小 `servoj_interp_points` |
| 抓取位置偏左/右    | 调整 `grasp_forward_offset`（X 方向偏移）                     |
| 稳定检测不触发     | 减小 `object_stable_threshold` 或 `cam_stable_frames`       |
| 稳定检测触发太快   | 增大 `object_stable_threshold` 或 `cam_stable_threshold`    |
| 夹爪夹不紧         | 增大 `gripper_force`（夹持力）                                |
| YOLO 检测不到物体  | 调整 `confidence_threshold` 或重新训练模型                    |

---

## 系统架构

```
RealSense D435                     tl_driver
     │                                │
     ├── RGB ──► YOLO Node            ├── joint_states ──┐
     │              │                 │                  │
     │              ├─ 3D定位          ├── tcp_pose ──────┤
     │              ├─ ObjectInfo      │                  ▼
     └── Depth ─────┘                 │           Control Node
           │                          │              │
           │                          │     ┌────────┴────────┐
           │                          │     │                 │
           ▼                          ▼     ▼                 ▼
    /object_3d_pos_camera       /set_servoj_pos        夹爪(Modbus)
                                                        │
                                                   RS485 端口 2
                                                   └── ArmGripper
```

---

## 文件结构

```
tl_vision/
├── config/
│   ├── control_node.yaml         # 主控参数
│   ├── gripper.yaml              # 夹爪参数
│   ├── yolo_node.yaml            # YOLO 参数
│   └── camera_params.yaml        # 相机参数
├── launch/
│   ├── control_node.launch.py    # 主控启动
│   ├── bringup.launch.py         # 一键启动
│   ├── hand_in_eye_calib.launch.py
│   └── fk_test.launch.py
├── tl_vision/
│   ├── control_node.py           # 主控节点
│   ├── yolo_node.py              # YOLO 检测节点
│   ├── calib_node.py             # 手眼标定节点
│   ├── fk_test_node.py           # 正运动学测试
│   ├── zhua.py                   # 夹爪控制库 (EG2Gripper + ArmGripper)
│   ├── test_gripper.py           # USB 夹爪测试
│   ├── test_gripper_feedback.py  # 夹爪反馈测试
│   ├── test_gripper_arm.py       # Modbus 夹爪测试（推荐）
│   └── Readme.md                 # 本文件
├── package.xml
├── setup.py
└── setup.cfg
```

---

## 常见问题

### Q: 相机启动后收不到数据

- 检查 USB 3.0 连接
- 禁用 USB 自动挂起：`sudo udevadm control --reload-rules`
- 降低分辨率/帧率（`camera_params.yaml` 中设置 640×480@15fps）

### Q: YOLO 检测不到物体

- 检查 `yolo_node.yaml` 中的 `object_class_id` 是否正确
- 降低 `confidence_threshold`
- 检查 RGB 图像是否正常显示

### Q: 夹爪不动（Modbus 方式）

- 确认夹爪连接到机械臂 RS485 端口 2（不是端口 1）
- 检查 `gripper.yaml` 中的 `gripper_port: 2`
- 检查 `gripper_slave_id` 是否为 1
- 尝试运行 `test_gripper_arm.py` 诊断

### Q: 视觉跟踪时夹爪提前闭合

- 检查是否在多方块/循环模式下且稳定检测触发过早
- 增大 `object_stable_threshold`
- 或检查 `_update_trajectory_from_detection()` 中轨迹更新节流逻辑

### Q: 构建失败

```bash
# 确保先构建接口包
colcon build --packages-select tl_ros2_interface
colcon build --packages-select tl_vision
```
