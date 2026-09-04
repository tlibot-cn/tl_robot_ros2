# 天链机器人 ROS2 机械臂工作空间

TL 系列机械臂 ROS2 接口与功能包集 — 四川天链机器人股份有限公司

## 环境要求

- Ubuntu 22.04
- ROS2 Humble
- 控制器版本 2403

## 支持机械臂型号

- **6 轴系列**：TCB605、TCB605F、TCB605L、TCB605LV、TCB605V、TCB610、TCB610V（末端负载 5~10 kg）
- **7 轴系列**：TCB705、TCB705F、TCB705L、TCB705LV、TCB705V、TCB710、TCB710V（末端负载 5~10 kg）
- 命名规则：`TCB` + 轴数（6/7）+ 负载能力（05=5kg, 10=10kg）+ 后缀（F=力控, L=加长, V=视觉, LV=加长+视觉）

## 搭建环境

### 安装 ROS2

参考 ROS2 官方安装指南：

> [ROS2 Humble 安装教程](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### 编译

```bash
# 克隆仓库
git clone https://github.com/tlibot-cn/tl_robot_ros2_cpp.git

# 编译
cd ~/tl_robot_ros2_cpp
colcon build
source install/setup.bash
```

编译完成后即可进行各功能包的运行操作。

### 安装格式化工具

```bash
sudo apt install clang-format
pip install black isort ruff
```

## 代码规范

项目统一使用以下格式化配置，所有成员在提交前需确保代码格式一致。

### CI 格式检查

仓库已接入 GitHub Actions（`.github/workflows/ci.yml`）：push 到 master/dev、
提交 PR 时自动执行格式检查（纯文档变更不触发），格式不合格会阻断合并。
发布流程见 `.github/workflows/release.yml`（打版本标签触发）。

### C++ 格式

- 配置：`.clang-format`（clang-format v14）
- Allman 大括号风格，2 空格缩进，120 列宽限制
- CI 强制格式检查，提交时请先本地格式化

手动格式化（自动跳过三方 SDK 头文件）：
```bash
./scripts/format-cpp.sh                          # 格式化所有 C++ 文件
./scripts/format-cpp.sh src/tl_driver/src/       # 格式化指定目录
./scripts/format-cpp.sh src/tl_driver/src/tl_driver.cpp  # 格式化指定文件
```

### Python 格式

- 配置：`pyproject.toml`（Black / Ruff / isort）
- 4 空格缩进，100 列宽限制

手动格式化：
```bash
ruff format src/
# 或
black src/ && isort src/
```

## 运行

### 真实机械臂

```bash
source ~/tl_robot_ros2_cpp/install/setup.bash
ros2 launch tl_bringup tl_<arm_type>_bringup.launch.py
```

`<arm_type>` 可选：`tcb605`、`tcb605f`、`tcb605l`、`tcb605lv`、`tcb605v`、`tcb610`、`tcb610v`、`tcb705`、`tcb705f`、`tcb705l`、`tcb705lv`、`tcb705v`、`tcb710`、`tcb710v`

**修改通信参数**：编辑 `src/tl_driver/config/` 下对应型号的 YAML，修改 `arm_ip`、`arm_port` 等参数。

### MoveIt2 + RViz 虚拟控制

```bash
source ~/tl_robot_ros2_cpp/install/setup.bash
ros2 launch tl_<arm_type>_config demo.launch.py
```

替换 `<arm_type>` 为对应型号名称。

### Gazebo 仿真

```bash
source ~/tl_robot_ros2_cpp/install/setup.bash
ros2 launch tl_gazebo gazebo_<arm_type>_demo.launch.py
```

## 工作空间结构

```
tl_robot_ros2_cpp/
├── src/           # ROS2 功能包源码
├── build/         # 编译中间产物（已 gitignore）
├── install/       # 编译输出（已 gitignore）
└── log/           # 编译日志（已 gitignore）
```

各功能包的详细用途请参见 [`src/README.md`](src/README.md)。

## 安全注意事项

- 每次使用前检查机械臂安装情况，包括固定螺丝是否松动
- 机械臂运行过程中，人员不可处于机械臂工作范围内
- 不使用时将机械臂置于安全位置并断开电源

## 许可与联系

**许可证**： Apache License 2.0

**公司官网**：[https://www.tlibot.com/](https://www.tlibot.com/)

**四川天链机器人股份有限公司**

如有问题，请在仓库 issue 提出或联系包维护者。
