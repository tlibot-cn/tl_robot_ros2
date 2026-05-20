# 手眼标定说明文档

## 概述

手眼标定用于确定相机与机械臂之间的坐标变换关系，使机械臂能够根据相机识别到的物体位置进行精确抓取。

**本机器人相机固定在机械臂末端，采用「眼在手上（eye-in-hand）」模式**。

标定完成后，**除非相机和机械臂的相对位置发生变动，否则无需重新标定**。

### 眼在手上（eye-in-hand）原理

```
相机固定在机械臂末端（法兰盘），随机械臂一起运动。
标定目标：计算出 相机坐标系 相对于 机械臂末端坐标系 的变换 H_end^camera
```

核心方程（AX = XB）：

$$A_2^{-1} \cdot A_1 \cdot X = X \cdot B_2 \cdot B_1^{-1}$$

- **A**：相邻两次运动时机械臂末端的变换关系（通过 API 获取，已知）
- **B**：相邻两次运动时标定板在相机中的位姿变化（通过相机标定求出，已知）
- **X**：相机到机械臂末端的变换矩阵（**求解目标**）

坐标系关系（标定结果在抓取运行时的使用方式）：

$$P_{base} = T_{base}^{tool} \cdot T_{tool}^{camera} \cdot P_{camera}$$

- $T_{base}^{tool}$：机械臂末端在基座坐标系下的位姿（通过 `get_current_position` 实时获取）
- $T_{tool}^{camera}$：手眼标定结果（即 `handeye_matrix`）
- $P_{camera}$：YOLO 检测到的物体在相机坐标系下的坐标

## 标定方式

使用 ROS2 节点 `calib_node`，支持两种模式：

| 模式 | 说明 |
|------|------|
| **online** | 连接相机和机械臂，实时采集图像和位姿，交互式计算标定 |
| **offline** | 从已有 `.npz` 数据文件直接计算标定结果 |

### 配置参数

编辑 `src/tl_vision/config/calib_node.yaml`：

```yaml
calib_node:
  ros__parameters:
    robot_ip: "192.168.1.13"    # 机械臂 IP （弃用）
    robot_port: "6001"          # 机械臂端口 （弃用）
    camera_width: 640           # 相机分辨率宽
    camera_height: 480          # 相机分辨率高
    camera_fps: 30              # 相机帧率
    chessboard_xx: 11           # 标定板横向内角点数（格子数 - 1）
    chessboard_yy: 8            # 标定板纵向内角点数（格子数 - 1）
    chessboard_L: 0.02          # 单个方格边长（单位：米）
    save_path: ""               # 数据保存路径（默认自动生成）
    save_result_file: true      # 是否保存标定结果 YAML
    calculation_mode: "online"  # "online" 实时采集 | "offline" 从 npz 文件计算
    data_file: ""               # offline 模式下的 .npz 数据文件路径
    handeye_method: "TSAI"      # 手眼标定算法：TSAI / PARK / HORAUD / ANDREFF / DANIILIDIS
    display_scale: 2.0          # 显示缩放比例
```

### 启动标定节点

```bash
ros2 launch tl_vision hand_in_eye_calib.launch.py
```

### 在线标定（calculation_mode: "online"）

1. **连接相机和机械臂**：确保 RealSense D435 相机和机械臂均已连接
2. **固定标定板**：将标定板**固定放置**在工作台上，确保机械臂末端相机能从不同视角观测到它。标定板在标定期间**保持固定，不得移动**
3. **调整角度**：移动机械臂使标定板清晰出现在相机视野中，**标定板与相机镜面保持一定角度**（不要正对）
4. **采集数据**：
   - 按键盘 `s` 键采集一组数据（当前帧角点 + 机械臂位姿）
   - 移动**机械臂末端旋转轴**，**每次旋转角度尽量大（>30°）**，确保 X、Y、Z 三轴都有足够的旋转变化
   - 可以先绕末端 Z 轴旋转拍摄多张，再绕 X 轴旋转拍摄
   - 采集 **15-20 组**不同姿态的数据
5. **计算标定**：按键盘 `c` 键计算手眼标定结果
6. **结果输出**：终端打印旋转矩阵和平移向量，同时保存 `calibration_result.yaml`

#### 操作按键

| 按键 | 功能 |
|------|------|
| `s` | 采集当前帧角点 + 机械臂位姿（自动保存到 .npz 文件） |
| `c` | 使用已采集数据计算手眼标定 |
| `r` | 清空已采集数据 |
| `q` | 退出 |

### 离线标定（calculation_mode: "offline"）

1. 准备 `.npz` 数据文件（可来自之前的 online 采集，或使用独立脚本采集的数据）
2. 在 `calib_node.yaml` 中设置：
   ```yaml
   calculation_mode: "offline"
   data_file: "/path/to/handeye_samples.npz"
   ```
3. 启动节点，自动计算并输出结果

### 标定结果格式

```yaml
rotation_matrix:            # 3×3 旋转矩阵
translation_vector:         # 3×1 平移向量（单位：米）
quaternion_xyzw:            # 四元数表示
camera_matrix:              # 相机内参矩阵
distortion_coefficients:    # 畸变系数
valid_count: 15             # 有效数据数量
handeye_method: "TSAI"      # 使用的算法
```

---

## 如何应用标定结果

### 在 control_node 中使用

将标定得到的 4×4 齐次变换矩阵填入 `src/tl_vision/config/control_node.yaml` 的 `handeye_matrix` 字段：

```yaml
control_node:
  ros__parameters:
    handeye_matrix:
      [
        R11, R12, R13, tx,
        R21, R22, R23, ty,
        R31, R32, R33, tz,
        0.0, 0.0, 0.0, 1.0
      ]
```

- **格式**：16 个元素扁平列表，行优先（row-major）
- **内容**：上半部分 3×3 为旋转矩阵 R，最后一列前 3 个为平移向量 t
- **默认值**：单位矩阵（未标定状态），**必须替换为实际标定结果**

### 标定结果如何被使用

标定完成后，`handeye_matrix` 在 control_node 中的作用链路如下：

```
yolo_node  ──(物体3D坐标)──►  control_node  ──(基座坐标)──►  机械臂运动
                                  │
                           使用 handeye_matrix
                           做坐标转换
```

$$P_{base} = T_{base}^{tool} \cdot T_{tool}^{camera} \cdot P_{camera}$$

| 变量 | 含义 | 来源 |
|------|------|------|
| $$P_{camera}$$ | 物体在相机坐标系下的 3D 坐标 | yolo_node 发布 `/tl_vision/object_3d_pos_camera` |
| $$T_{tool}^{camera}$$ | 相机 → 机械臂末端 的齐次变换 | 手眼标定结果，填入 `handeye_matrix` |
| $$T_{base}^{tool}$$ | 机械臂末端在基座坐标下的位姿 | 机械臂 API `get_current_position` 实时获取 |
| $$P_{base}$$ | 物体在基座坐标系下的 3D 坐标 | control_node 发布 `/tl_vision/object_3d_pos_base` |

> 注意：calib_node 本身不依赖 yolo_node。标定时直接用 RealSense 相机采集棋盘格图像，用 nrc_interface 获取机械臂位姿。yolo_node 只在标定完成后的抓取运行阶段才参与。

---

## 常见问题

### 1. 标定计算失败

**错误信息**：`calibrateHandEyeTsai Hand-eye calibration failed! Not enough informative motions`

**原因**：采集的图片旋转量不足

**解决**：
- 每次移动机械臂末端时，**旋转角度尽量大于 30°**
- 确保 X、Y、Z 三轴都有足够的旋转变化
- 采集 15 组以上不同姿态的数据
- 避免只在一个轴向上小幅运动

### 2. 棋盘格角点检测不到

- 确保标定板在相机视野中完整可见
- 调整光照，避免反光
- 确认 `chessboard_xx` 和 `chessboard_yy` 参数与实际标定板一致（注意是**内角点数**，即格子数减 1）

### 3. 标定精度低（平移误差 > 1cm）

- 增加采集数据量（建议 15-20 组）
- 确保机械臂在采集过程中覆盖更大的工作空间
- 检查标定板是否在采集过程中发生移动
- 确认 `chessboard_L`（方格边长）参数与实际标定板一致

### 4. control_node 机械臂运动位置不准

- 检查 `handeye_matrix` 是否正确填入（注意是 `T_tool_camera`，不是 `T_camera_tool`）
- 确认 `handeye_matrix` 最后一行是 `[0, 0, 0, 1]`
- 验证 NRC API 返回的位置单位（mm），control_node 会自动除以 1000 转为米
- 检查 `grasp_offset` 参数是否合理

### 5. calib_node 无法连接机械臂

- 确认机械臂 IP 和端口配置正确
- 确认机械臂已上电且处于就绪状态
- 检查网络连通性：`ping 192.168.1.13`

---

## 注意事项

- **标定板必须固定**：采集过程中标定板不能有任何移动，否则结果无效
- **单位一致性**：`chessboard_L` 单位为米，calib_node 输出平移向量单位也为米
- **算法选择**：`TSAI` 算法是工程中最常用的，通常无需更改
- **重新标定时机**：仅当相机与机械臂的相对位置发生变动时才需重新标定
- **数据保存**：online 模式每采集一组即自动保存到 `.npz` 文件，避免中途退出丢失数据
