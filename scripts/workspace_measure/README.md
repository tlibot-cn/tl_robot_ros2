# TCB705 机械臂工作空间测量工具

两个脚本，两种方法测量 TCB705 七轴机械臂的工作范围。

```
workspace_measure/
├── README.md                    # 本文件
├── fk_workspace.py              # FK 蒙特卡洛采样（快速，理论边界）
├── ik_workspace.py              # IK 服务调用（精确，实际可达范围）
└── results/
    ├── tcb705_workspace.png      # FK 结果图
    ├── tcb705_workspace_ik.png   # IK 结果图
    ├── tcb705_workspace_stats.txt
    └── tcb705_workspace_ik_stats.txt
```

---

## 1. FK 理论工作空间 — `fk_workspace.py`

**原理**：从 URDF 读取运动学参数，在关节空间做 15 万次蒙特卡洛采样，用正向运动学计算末端（link7）位置。**不约束姿态**，给出理论最大可达范围。

```bash
cd workspace_measure
python3 fk_workspace.py
```

输出：`results/tcb705_workspace.png` + `results/tcb705_workspace_stats.txt`

**运行时间**：约 30 秒。

---

## 2. IK 实测工作空间 — `ik_workspace.py`

**原理**：启动 MoveIt2 + KDL IK 求解器，在球坐标网格上逐方向调用 `/compute_ik` 服务，用二分法搜索可达边界。**同时约束位置和末端姿态**，反映实际 IK 求解器能力。

### 前置条件

需要 MoveIt2 demo 已启动：

```bash
cd ~/tl_robot_ros2_cpp
source install/setup.bash
ros2 launch tl_tcb705_config demo.launch.py
```

### 修改末端姿态

打开 `ik_workspace.py`，在第 36-41 行修改四元数：

```python
# RPY(0, 0, -90°): 绕基座 Z 轴转 -90°
self.default_orient = Pose().orientation
self.default_orient.x = 0.0
self.default_orient.y = 0.0
self.default_orient.z = -0.7071068   # sin(-45°)
self.default_orient.w = 0.7071068    # cos(-45°)
```

常见姿态示例：

| 末端朝向 | RPY (°) | 四元数 (x, y, z, w) |
|---|---|---|
| TCP 水平朝 X | (0, 0, 0) | (0, 0, 0, 1) |
| TCP 朝下 | (π, 0, 0) | (1, 0, 0, 0) |
| 绕 Z 转 -90° | (0, 0, -90) | (0, 0, -0.707, 0.707) |

### 运行

```bash
cd workspace_measure
python3 ik_workspace.py
```

输出：`results/tcb705_workspace_ik.png` + `results/tcb705_workspace_ik_stats.txt`

**运行时间**：约 30-60 秒（512 个方向 × 每方向 IK 调用十几次）。

### 调整采样密度

`main()` 中的参数：

```python
result = measurer.measure_workspace(
    n_theta=32,     # 水平方向划分数
    n_phi=16,       # 垂直方向划分数
    max_radius=0.95 # 搜索最大半径 (m)
)
```

增大 `n_theta` / `n_phi` → 更精细，但耗时更长。

---

## 3. 两种方法对比

| 方法 | 约束 | 运行时间 | X 跨度 | Y 跨度 | Z 跨度 | 最大半径 |
|---|---|---|---|---|---|---|
| FK | 无姿态约束 | ~30s | 1297 mm | 1303 mm | 1219 mm | 895 mm |
| IK | 位置+姿态 | ~45s | ~990 mm | ~990 mm | ~770 mm | ~890 mm |

- **FK** 给出理论边界，用于设计验证、安装空间评估
- **IK** 给出特定姿态下的实际可达范围，贴近真实使用场景

---

## 4. 支持其他臂型

修改 `fk_workspace.py` 中的 URDF 路径：

```python
urdf_path = script_dir.parent / 'src' / 'tl_description' / 'urdf' / 'tcb705.urdf'
```

`ik_workspace.py` 通过 MoveIt2 加载模型（启动对应臂型的 demo.launch.py 即可），不需要改脚本路径。
