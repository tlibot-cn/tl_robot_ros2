# 天链机器人tl_gazebo使用说明书V1.0 天链机器人

文件修订记录：

| 版本号 | 时间 | 备注 |
| :---: | :---: | :---: |
| V1.0 | 2026-5-22 | 拟制 |

## 目录
* 1.[tl_gazebo功能包说明](#tl_gazebo功能包说明)
* 2.[tl_gazebo功能包运行](#tl_gazebo功能包运行)
  * 2.1[启动Gazebo仿真机械臂](#启动Gazebo仿真机械臂)
  * 2.2[控制仿真机械臂](#控制仿真机械臂)
* 3.[tl_gazebo功能包架构说明](#tl_gazebo功能包架构说明)
  * 3.1[功能包文件总览](#功能包文件总览)

## tl_gazebo功能包说明

tl_gazebo的主要作用为帮助我们实现机械臂的Gazebo仿真功能，我们将在Gazebo的仿真环境中搭建一个虚拟机械臂，然后通过ros2_control控制Gazebo中的虚拟机械臂。

* 1.功能包使用。
* 2.功能包架构说明。

通过这两部分内容的介绍可以帮助大家：
* 1.了解该功能包的使用。
* 2.熟悉功能包中的文件构成及作用。

### 支持的臂型

tl_gazebo支持天链机器人全系列臂型，共14种：

`tcb605`、`tcb605f`、`tcb605l`、`tcb605lv`、`tcb605v`、`tcb610`、`tcb610v`、`tcb705`、`tcb705f`、`tcb705l`、`tcb705lv`、`tcb705v`、`tcb710`、`tcb710v`

每种臂型均有对应的Gazebo模型描述文件（`.urdf.xacro`）和启动文件（`.launch.py`）。

## tl_gazebo功能包运行

### 启动Gazebo仿真机械臂

在完成环境安装和功能包编译后，我们可以进行tl_gazebo功能包的运行。

使用如下指令启动Gazebo虚拟空间和虚拟机械臂：

```bash
ros2 launch tl_gazebo gazebo_<arm_type>_demo.launch.py
```

在实际使用时需要将 `<arm_type>` 更换为实际的机械臂型号，可选择的机械臂型号有 `tcb605`、`tcb605f`、`tcb605l`、`tcb605lv`、`tcb605v`、`tcb610`、`tcb610v`、`tcb705`、`tcb705f`、`tcb705l`、`tcb705lv`、`tcb705v`、`tcb710`、`tcb710v`，例如 tcb710 机械臂的启动命令：

```bash
ros2 launch tl_gazebo gazebo_tcb710_demo.launch.py
```

启动成功后，将弹出如下界面：

![image](doc/image1.png)

### 控制仿真机械臂

Gazebo中的仿真机械臂启动后，可以通过MoveIt2对其进行运动规划和控制。

使用如下指令启动MoveIt2控制界面：

```bash
ros2 launch tl_<arm_type>_config gazebo_moveit_demo_<arm_type>.launch.py
```

例如 tcb710 机械臂的控制命令：

```bash
ros2 launch tl_tcb710_config gazebo_moveit_demo_tcb710.launch.py
```

启动成功后弹出RViz2的控制界面后就可以进行MoveIt2和Gazebo的仿真控制了。

![image](doc/image2.png)

## tl_gazebo功能包架构说明

### 功能包文件总览

当前tl_gazebo功能包的文件构成如下：

```
tl_gazebo/
├── CMakeLists.txt                          # 编译规则文件
├── config
│   ├── gazebo_tcb605_description.urdf.xacro    # TCB605 Gazebo模型描述文件
│   ├── gazebo_tcb605f_description.urdf.xacro   # TCB605F Gazebo模型描述文件
│   ├── gazebo_tcb605l_description.urdf.xacro   # TCB605L Gazebo模型描述文件
│   ├── gazebo_tcb605lv_description.urdf.xacro  # TCB605LV Gazebo模型描述文件
│   ├── gazebo_tcb605v_description.urdf.xacro   # TCB605V Gazebo模型描述文件
│   ├── gazebo_tcb610_description.urdf.xacro    # TCB610 Gazebo模型描述文件
│   ├── gazebo_tcb610v_description.urdf.xacro   # TCB610V Gazebo模型描述文件
│   ├── gazebo_tcb705_description.urdf.xacro    # TCB705 Gazebo模型描述文件
│   ├── gazebo_tcb705f_description.urdf.xacro   # TCB705F Gazebo模型描述文件
│   ├── gazebo_tcb705l_description.urdf.xacro   # TCB705L Gazebo模型描述文件
│   ├── gazebo_tcb705lv_description.urdf.xacro  # TCB705LV Gazebo模型描述文件
│   ├── gazebo_tcb705v_description.urdf.xacro   # TCB705V Gazebo模型描述文件
│   ├── gazebo_tcb710_description.urdf.xacro    # TCB710 Gazebo模型描述文件
│   ├── gazebo_tcb710v_description.urdf.xacro   # TCB710V Gazebo模型描述文件
│   ├── gazebo_6axis_f710_sim.urdf.xacro        # 6 轴 F710 手柄仿真模型描述文件
│   ├── gazebo_7axis_f710_sim.urdf.xacro        # 7 轴 F710 手柄仿真模型描述文件
│   ├── ros2_controllers_f710_sim_6axis.yaml    # 6 轴 F710 仿真控制器配置
│   └── ros2_controllers_f710_sim_7axis.yaml    # 7 轴 F710 仿真控制器配置
├── doc
│   ├── image1.png                          # Gazebo仿真界面截图
│   └── image2.png                          # RViz2+MoveIt2控制界面截图
├── launch
│   ├── gazebo_tcb605_demo.launch.py        # TCB605 Gazebo启动文件
│   ├── gazebo_tcb605f_demo.launch.py       # TCB605F Gazebo启动文件
│   ├── gazebo_tcb605l_demo.launch.py       # TCB605L Gazebo启动文件
│   ├── gazebo_tcb605lv_demo.launch.py      # TCB605LV Gazebo启动文件
│   ├── gazebo_tcb605v_demo.launch.py       # TCB605V Gazebo启动文件
│   ├── gazebo_tcb610_demo.launch.py        # TCB610 Gazebo启动文件
│   ├── gazebo_tcb610v_demo.launch.py       # TCB610V Gazebo启动文件
│   ├── gazebo_tcb705_demo.launch.py        # TCB705 Gazebo启动文件
│   ├── gazebo_tcb705f_demo.launch.py       # TCB705F Gazebo启动文件
│   ├── gazebo_tcb705l_demo.launch.py       # TCB705L Gazebo启动文件
│   ├── gazebo_tcb705lv_demo.launch.py      # TCB705LV Gazebo启动文件
│   ├── gazebo_tcb705v_demo.launch.py       # TCB705V Gazebo启动文件
│   ├── gazebo_tcb710_demo.launch.py        # TCB710 Gazebo启动文件
│   ├── gazebo_tcb710v_demo.launch.py       # TCB710V Gazebo启动文件
│   ├── gazebo_6axis_f710_sim.launch.py     # 6 轴 F710 手柄仿真启动文件
│   └── gazebo_7axis_f710_sim.launch.py     # 7 轴 F710 手柄仿真启动文件
├── package.xml                             # 依赖说明文件
└── README.md                               # 说明文档
```
