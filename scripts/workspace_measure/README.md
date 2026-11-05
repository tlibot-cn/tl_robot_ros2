# 机械臂工作空间测量工具

三个脚本，三种方法测量七轴机械臂的工作范围。FK 与灵巧空间（蒙特卡洛）脚本只读 URDF、无需 ROS；IK 脚本通过 MoveIt2 加载模型。

```
workspace_measure/
├── README.md                    # 本文件
├── fk_workspace.py              # FK 蒙特卡洛采样（快速，理论边界）
├── ik_workspace.py              # IK 服务调用（精确，单一姿态实际可达范围）
├── dexterous_workspace.py       # FK 蒙特卡洛 + 姿态覆盖（灵巧工作空间，无需 ROS）
└── results/
    ├── <arm>_workspace.png / _stats.txt           # FK 结果
    ├── <arm>_workspace_ik.png / _stats.txt        # IK 结果
    └── <arm>_dexterous_workspace.png / _stats.txt # 灵巧空间（蒙特卡洛）
```

---

## 1. FK 理论工作空间 — `fk_workspace.py`

**原理**：从 URDF 读取运动学参数，在关节空间做 15 万次蒙特卡洛采样，用正向运动学计算末端（link7）位置。**不约束姿态**，给出理论最大可达范围。

```bash
cd workspace_measure
python3 fk_workspace.py --arm-type tcb705
```

输出：`results/<arm_type>_workspace.png` + `results/<arm_type>_workspace_stats.txt`

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

## 3. 灵巧工作空间 — `dexterous_workspace.py`（蒙特卡洛，推荐）

**原理**：灵巧工作空间（dexterous workspace）是「末端能以**任意姿态**到达」的位置集合，是可达工作空间的子集。脚本不依赖 ROS / MoveIt：

1. 从 URDF 读取关节链与限位，在关节空间均匀采样 `--samples` 个构型；
2. 批量 FK 得到末端位置与四元数，按位置分箱成 `--step` 体素；
3. 对每个体素，计算「最坏方向的姿态覆盖半径」：

$$\theta_{max}(v)=\max_{r\in\text{探针}}\ \min_{i\in v}\ \arccos\!\big(2(q_i\cdot q_r)^2-1\big)$$

4. 体素灵巧度 $d = 1-\theta_{max}/\pi$，$\theta_{max}\le$ `--tau-deg` 的体素构成灵巧空间。

```bash
cd workspace_measure
python3 dexterous_workspace.py --arm-type tcb705
```

输出：`results/<arm_type>_dexterous_workspace.png`（3D 灵巧空间 + XZ/YZ 截面 + 多层 XY 截面）+ `results/<arm_type>_dexterous_workspace_stats.txt`

**运行时间**：默认配置（500 万构型、0.1 m 体素）约 30 秒，tcb705 实测 27 s。

### 关键参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--arm-type` | 必填 | 读取 `src/tl_description/urdf/<arm_type>.urdf`，取值即 14 种臂型 |
| `--samples` | `5000000` | 关节空间采样数；体素内样本数决定姿态覆盖的分辨率 |
| `--step` | `0.1` | 体素边长 (m)，越小越精细、每体素样本越少 |
| `--r-max` / `--z-min` | `1.0` / `-1.0` | 搜索球半径与最低 Z (m) |
| `--tau-deg` | `20` | 灵巧判定角容差；越小越严格 |
| `--n-probe` | `100` | 姿态覆盖探针数，用于估计覆盖半径 |
| `--min-samples` | `0` | 体素最少样本数；0 = 由 τ 自动推导 `⌈100/τ³⌉` |
| `--seed` | `42` | 随机种子，结果可复现 |

### 分辨率与偏差（必读）

- 覆盖半径随体素样本数 $K$ 存在量化下限，实测约 $\theta_{floor}=(100/K)^{1/3}$ rad：$K=4700$ 时约 16°，$K=800$ 时约 29°；
- 体素内的样本来自不同位置（最大偏移 `step`/2），体素越大越容易「拼出」全姿态，会高估灵巧度；`step` 建议不超过 0.1 m；
- 两个方向相反的偏差：样本有限使 $\theta_{max}$ 偏大（保守、漏判），体素内位置混合使 $\theta_{max}$ 偏小（乐观、误判）；净效应取决于 `step` 与 $K$；
- 样本不足的体素用 `--min-samples` 直接剔除；关节空间均匀采样 ≠ 工作空间均匀，稀疏体素被剔除后按「不可达」处理，工作空间边界外侧的薄壳可能被低估；
- 结果给的是统计意义下的灵巧空间，不是逐位姿 IK 可解性证明。

tcb705 默认参数实测（500 万构型，5.0 m³ 采样空间）：829 个可达体素、42 个灵巧体素，灵巧体积 0.042 m³（可达体积 0.829 m³，占 5.1%），跨度 500×500×500 mm，最大半径 572 mm；换随机种子（42 → 7）结果稳定在 42–43 个体素。

---

## 4. 方法对比

| 方法 | 姿态约束 | 依赖 | 运行时间 | X 跨度 | Y 跨度 | Z 跨度 | 最大半径 |
|---|---|---|---|---|---|---|---|
| FK | 无 | 无 | ~30s | 1297 mm | 1303 mm | 1219 mm | 895 mm |
| IK | 单一姿态 | MoveIt2 | ~45s | ~990 mm | ~990 mm | ~770 mm | ~890 mm |
| 灵巧空间（MC，tcb705） | 全姿态（20° 容差） | 无 | ~30s | 500 mm | 500 mm | 500 mm | 572 mm |

- **FK** 给出理论边界，用于设计验证、安装空间评估；
- **IK** 给出特定姿态下的实际可达范围，贴近真实使用场景；
- **灵巧空间（MC）** 适合批量扫全部臂型、离线出图；结果依赖采样数与角容差。

---

## 5. 支持其他臂型

FK 与灵巧空间（MC）脚本：`--arm-type` 取值即 `src/tl_description/urdf/` 下的 URDF 文件名（14 种臂型全部支持），例如：

```bash
python3 fk_workspace.py --arm-type tcb605
python3 dexterous_workspace.py --arm-type tcb605
```

`ik_workspace.py` 通过 MoveIt2 加载模型（启动对应臂型的 demo.launch.py 即可），不需要改脚本路径。
