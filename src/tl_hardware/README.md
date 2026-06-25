# tl_hardware — ros2_control 硬件接口插件

## 概述

`tl_hardware` 包实现了一个自定义 [ros2_control](https://control.ros.org/humble/) `SystemInterface` 硬件接口插件，用于桥接 MoveIt2 的 ros2_control 控制器层与机械臂驱动节点 `tl_driver`。

插件名：`tl_hardware/TLHardwareInterface`

## 架构

```
joint_trajectory_controller (ros2_control)
  │
  │  write():  position commands (rad)
  ▼
TLHardwareInterface
  │  ┌─ subscriber: /joint_states         ◄── tl_driver 发布
  │  ├─ publisher:  /tl_driver/set_servoj_pos  ──► tl_driver 接收 (deg)
  │  ├─ client:     /tl_driver/open_servoj     ──► 开启 servoj 流
  │  └─ client:     /tl_driver/close_servoj    ──► 关闭 servoj 流
  ▼  TCP/IP
机械臂控制器
```

插件通过以下方式实现 ros2_control 与真实机械臂的通信：

| 方向 | 方式 | 话题 / 服务 |
|------|------|-------------|
| **读取状态** | Subscriber | `/joint_states`（sensor_msgs/JointState） |
| **写入指令** | Publisher | `/tl_driver/set_servoj_pos`（std_msgs/Float64MultiArray，**角度**） |
| **开启流模式** | Service Client | `/tl_driver/open_servoj`（tl_ros2_interface/srv/OpenServoJ） |
| **关闭流模式** | Service Client | `/tl_driver/close_servoj`（std_srvs/srv/Trigger） |

## 生命周期

### `on_init(info)`

解析 `HardwareInfo` 中的硬件参数，包括话题名、服务名、servoj 运动参数、超时时间。建立关节名到索引的映射表，为后续 O(1) 查找做准备。

### `on_configure()`

- 创建一个内部 ROS2 节点 `tl_hardware`
- 创建 Publisher（`/tl_driver/set_servoj_pos`）
- 创建 Service Clients（`open_servoj`、`close_servoj`）
- 创建 Subscriber（`/joint_states`，best-effort QoS）
- 启动 `SingleThreadedExecutor` 后台线程处理回调

### `on_activate()`

1. 调用 `/tl_driver/open_servoj` 服务，传入 `vmax`/`amax`/`jmax` 参数
2. 等待首帧 `/joint_states` 消息（超时 5 秒）
3. 将接收到的关节位置/力矩数据写入状态接口，初始化速度计算状态
4. 标记 `hardware_active_ = true`

### `read()`

- 从 `/joint_states` 回调中更新位置、速度、力矩数据到状态接口
- 速度由位置差分计算
- 若超过 `state_timeout_sec` 未收到关节状态，输出告警日志（5 秒节流）

### `write()`

- 读取 `joint_position_commands_`（弧度）
- 转换为**角度**（× 180/π）
- 发布 `Float64MultiArray` 到 `/tl_driver/set_servoj_pos`

### `on_deactivate()`

- 调用 `/tl_driver/close_servoj` 服务关闭流模式
- 标记 `hardware_active_ = false`

### `on_cleanup()`

- 停止后台 executor 线程
- 重置所有状态标记

## 硬件参数

这些参数在 `<arm>.ros2_control.xacro` 中通过 `<param>` 标签配置：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `joint_states_topic` | `/joint_states` | `tl_driver` 发布的关节状态话题 |
| `servoj_topic` | `/tl_driver/set_servoj_pos` | 位置指令发布话题 |
| `open_servoj_service` | `/tl_driver/open_servoj` | 开启 servoj 流服务 |
| `close_servoj_service` | `/tl_driver/close_servoj` | 关闭 servoj 流服务 |
| `state_timeout_sec` | `1.0` | 关节状态超时告警阈值（秒） |
| `servoj_vmax` | `30.0` | 各关节最大速度（deg/s），支持单值或逗号分隔的逐关节值 |
| `servoj_amax` | `100.0` | 各关节最大加速度（deg/s²），支持单值或逗号分隔的逐关节值 |
| `servoj_jmax` | `500.0` | 各关节最大加加速度（deg/s³），支持单值或逗号分隔的逐关节值 |

示例配置片段：

```xml
<plugin>tl_hardware/TLHardwareInterface</plugin>
<param name="joint_states_topic">/joint_states</param>
<param name="servoj_topic">/tl_driver/set_servoj_pos</param>
<param name="open_servoj_service">/tl_driver/open_servoj</param>
<param name="close_servoj_service">/tl_driver/close_servoj</param>
<param name="state_timeout_sec">1.0</param>
<param name="servoj_vmax">30.0,30.0,30.0,30.0,30.0,30.0,30.0</param>
<param name="servoj_amax">100.0</param>
<param name="servoj_jmax">500.0</param>
```

## 插件注册

通过 `tl_hardware_interface.xml` 注册为 Plugin：

```xml
<library path="tl_hardware">
  <class name="tl_hardware/TLHardwareInterface"
         type="tl_hardware::TLHardwareInterface"
         base_class_type="hardware_interface::SystemInterface">
    <description>tlrobot hardware interface for ROS2 Control.</description>
  </class>
</library>
```

并在 `package.xml` 中导出：

```xml
<export>
  <hardware_interface plugin="${prefix}/tl_hardware_interface.xml"/>
</export>
```

## 与 MoveIt2 集成

在 MoveIt2 配置包的 `<arm>.ros2_control.xacro` 中通过 `use_real_hardware` 参数切换：

```xml
<xacro:macro name="tl_tcb710_ros2_control" params="name initial_positions_file use_real_hardware:=false">
    <ros2_control name="${name}" type="system">
        <hardware>
            <xacro:if value="${use_real_hardware}">
                <plugin>tl_hardware/TLHardwareInterface</plugin>
                <!-- 参数配置 -->
            </xacro:if>
            <xacro:unless value="${use_real_hardware}">
                <plugin>mock_components/GenericSystem</plugin>
            </xacro:unless>
        </hardware>
        <!-- 关节定义 -->
    </ros2_control>
</xacro:macro>
```

启动时通过 `real_hardware_demo.launch.py` 传入 `use_real_hardware:=true`：

```bash
ros2 launch tl_tcb710_config real_hardware_demo.launch.py
```

## 构建与依赖

```bash
colcon build --packages-select tl_hardware
source install/setup.bash
```

依赖的 ROS2 包：

- `hardware_interface` — ros2_control 硬件接口基类
- `pluginlib` — 插件加载机制
- `rclcpp` / `rclcpp_lifecycle` — ROS2 节点与生命周期
- `sensor_msgs` — JointState 消息
- `std_msgs` — Float64MultiArray 消息
- `std_srvs` — Trigger 服务
- `tl_ros2_interface` — OpenServoJ 服务定义
- `trajectory_msgs` — 轨迹消息

## 注意事项

- 该插件作为 `tl_driver` 的 **数据传输层**，本身不直接与机械臂 TCP 通信——所有数据通过 ROS2 话题/服务与 `tl_driver` 交互。
- 位置指令的单位转换（rad → deg）在 `write()` 中自动完成，无需额外配置。
- 速度由位置差分计算，不依赖 `tl_driver` 发布的速度值。
- 若长时间收不到 `/joint_states`，插件会持续告警但不会自动 shutdown——此行为由上层控制器管理。
