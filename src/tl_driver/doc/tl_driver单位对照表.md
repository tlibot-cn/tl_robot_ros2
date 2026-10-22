# tl_driver 接口单位对照表

> 基线：ROS 接口统一 REP 103（m / rad / rad/s），控制器与 SDK 的 mm / 度 / % 由 `tl_driver` 在边界换算。
> 修改任一接口单位时，必须同步驱动换算、`tl_ros2_interface` 注释、本文与消费方。

## 话题

| 话题 | 字段 | ROS 单位 | 控制器 / SDK 单位 |
| :--- | :--- | :--- | :--- |
| `/joint_states` | `position` / `velocity` | rad / rad·s⁻¹ | ACS 流：度；同步查询：度 |
| `/tcp_pose` | `position` / `rpy` | m / rad | MCS 流：mm / 度；同步查询：mm / rad |
| `/tl_driver/joint_dynamics` | `line_speed` / `joint_speed` / 电流 / 力矩 | m/s / rad/s / A / ‰ 额定扭矩 | mm/s / 度/s / A / ‰ |
| `/tl_driver/joint_thermal` | `temperature` / `joint_voltage` / `positioner_voltage` | °C / V / V | °C / V / V |
| `/tl_driver/moveJ` | `MoveCommand`，`coord=0` | 关节 rad | 度 |
| `/tl_driver/moveL` | `MoveCommand`，`coord=1/2/3` | m + rad；`velocity` m/s | mm + rad；`velocity` mm/s |
| `/tl_driver/set_servoj_pos` | `data[7]` | rad（6 轴末位补 0） | 度 |

## MoveCommand

| 字段 | 说明 |
| :--- | :--- |
| `target_pos_value` | 14 位：`coord=0` 前 7 位为关节 rad；`1/2/3` 前 3 位 m、姿态 rad；后 7 位外部轴按 SDK 原值 |
| `velocity` / `velocity_sync` | MOVJ 为 %；MOVL 为 m/s |
| `acc` / `dec` | % |

## 服务

| 服务 | 字段 | ROS 单位 |
| :--- | :--- | :--- |
| `coord_transform` | `origin_pos` / `reference_pos` / `target_pos` | 关节 rad；笛卡尔 m + rad |
| `get_robot_joint_param` | 软限位 rad；`*_rot_speed` rad/s；`max_acc` / `max_dec` rad/s² | 驱动内换算 rpm / 度 |
| `set_tool_param` | `x/y/z`、质心 m；`a/b/c` rad；质量 kg | 驱动内换算 mm / 度 |
| `set_user_coord` | `position` m；`rpy` rad | 驱动内换算 mm |
| `open_servoj` | `vmax` / `amax` / `jmax` | rad/s、rad/s²、rad/s³（固定 7 元素） |
| `get_dh_param` | `alpha/theta/mounting_angle` rad；`a/d` m | 驱动内换算 度 / mm |

## 协议包例外（不换算）

以下接口的数组是控制器协议包，自带单位制标志位，保留 mm + 标志位口径：

- `get_pos_reachable.pos`

## 配置参数

控制器内部的配置参数（如 `tl_teleop` / `tl_teleop_f710` 的 `servo_vmax`、`tl_hardware` 的 `servoj_vmax`）沿用历史度制，由各节点在调用 ROS 接口前转成 rad；新增配置项优先直接使用 REP 103 单位。
