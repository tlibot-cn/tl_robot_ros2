<div align="center">

# 天链机器人tl_driver使用说明书

</div>

## 目录
* 1.[tl_driver功能包说明](#tl_driver功能包说明)
* 2.[tl_driver功能包使用](#tl_driver功能包使用)
* 3.[tl_driver功能包架构说明](#tl_driver功能包架构说明)
* 4.[tl_driver功能包话题与服务说明](#tl_driver功能包话题与服务说明)

## tl_driver功能包说明
tl_driver功能包在机械臂ROS2功能包中是十分重要的，该功能包实现了通过ROS与机械臂进行通信控制机械臂的功能，在下文中将通过以下几个方面详细介绍该功能包。  
* 1.功能包使用。  
* 2.功能包架构说明。  
* 3.功能包话题说明。

通过这三部分内容的介绍可以帮助大家：  
* 1.了解该功能包的使用。  
* 2.熟悉功能包中的文件构成及作用。  
* 3.熟悉功能包相关的话题，方便开发和使用  
## tl_driver功能包使用
### tl_driver功能包编译

```bash
cd ~/tl_robot
colcon build
source install/setup.bash
```
编译完成后我们可以通过以下命令直接启动节点，连接机械臂。

### 启动方式

tl_driver 提供两种启动方式：

**方式一：通用启动（推荐开发调试使用）**

通过 `tl_driver.launch.py` 传入 `arm_type` 参数，适用于快速切换不同臂型：

```bash
ros2 launch tl_driver tl_driver.launch.py arm_type:=<arm_type>
```

**方式二：快捷启动（推荐日常使用）**

每种臂型都有对应的专用 launch 文件，无需传参，直接启动：

| 臂型 | 启动命令 |
|------|----------|
| TCB605 | `ros2 launch tl_driver tl_tcb605_driver.launch.py` |
| TCB605F | `ros2 launch tl_driver tl_tcb605f_driver.launch.py` |
| TCB605L | `ros2 launch tl_driver tl_tcb605l_driver.launch.py` |
| TCB605LV | `ros2 launch tl_driver tl_tcb605lv_driver.launch.py` |
| TCB605V | `ros2 launch tl_driver tl_tcb605v_driver.launch.py` |
| TCB610 | `ros2 launch tl_driver tl_tcb610_driver.launch.py` |
| TCB610V | `ros2 launch tl_driver tl_tcb610v_driver.launch.py` |
| TCB705 | `ros2 launch tl_driver tl_tcb705_driver.launch.py` |
| TCB705F | `ros2 launch tl_driver tl_tcb705f_driver.launch.py` |
| TCB705L | `ros2 launch tl_driver tl_tcb705l_driver.launch.py` |
| TCB705LV | `ros2 launch tl_driver tl_tcb705lv_driver.launch.py` |
| TCB705V | `ros2 launch tl_driver tl_tcb705v_driver.launch.py` |
| TCB710 | `ros2 launch tl_driver tl_tcb710_driver.launch.py` |
| TCB710V | `ros2 launch tl_driver tl_tcb710v_driver.launch.py` |

> 以下文档示例均以 **TCB710** 为例进行说明，其他臂型操作方式相同。

### 示例：启动 TCB710 机械臂

```bash
ros2 launch tl_driver tl_tcb710_driver.launch.py
```

启动成功后，将显示以下画面:

![image](doc/tl_driver1.png)  
当机械臂IP被改变后我们的启动指令就失效了，再直接使用如上指令就无法成功连接到机械臂了，我们可以通过修改如下配置文件，重新建立连接。
该配置文件位于我们的tl_driver功能包下的config文件夹下，需要根据不同的机械臂型号修改相应的配置文件。
以 TCB710 为例，配置文件为 `config/tl_tcb710_config.yaml`：

![image](doc/tl_driver2.png)  
其配置文件内容如下:
```
tl_driver:   
  ros__parameters:  
    #robot param  
    arm_ip: "192.168.1.13"          # 设置TCP连接时的IP  
    arm_port: "6001"                # 设置TCP连接时的端口  
    arm_port_aux: "7000"            # 设置机械臂连接时的辅助端口
    arm_type: "TCB710"              # 机械臂型号
    arm_joints: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"] # 关节名称
```
## tl_driver功能包架构说明
### 功能包文件总览
```
├── CMakeLists.txt                 # 编译规则文件
├── config                         # 机械臂配置文件
│   ├── tl_tcb605_config.yaml
│   ├── tl_tcb605f_config.yaml
│   ├── tl_tcb605l_config.yaml
│   ├── tl_tcb605lv_config.yaml
│   ├── tl_tcb605v_config.yaml
│   ├── tl_tcb610_config.yaml
│   ├── tl_tcb610v_config.yaml
│   ├── tl_tcb705_config.yaml
│   ├── tl_tcb705f_config.yaml
│   ├── tl_tcb705l_config.yaml
│   ├── tl_tcb705lv_config.yaml
│   ├── tl_tcb705v_config.yaml
│   ├── tl_tcb710_config.yaml
│   └── tl_tcb710v_config.yaml
├── doc                            # 相关文档与图片
│   ├── tl_driver1.png
│   └── tl_driver2.png
├── include                        # 头文件
│   └── tl_driver
│       └── tl_driver.h
├── launch                         # 启动文件
│   ├── tl_driver.launch.py        # 通用启动（需传arm_type参数）
│   ├── tl_tcb605_driver.launch.py
│   ├── tl_tcb605f_driver.launch.py
│   ├── tl_tcb605l_driver.launch.py
│   ├── tl_tcb605lv_driver.launch.py
│   ├── tl_tcb605v_driver.launch.py
│   ├── tl_tcb610_driver.launch.py
│   ├── tl_tcb610v_driver.launch.py
│   ├── tl_tcb705_driver.launch.py
│   ├── tl_tcb705f_driver.launch.py
│   ├── tl_tcb705l_driver.launch.py
│   ├── tl_tcb705lv_driver.launch.py
│   ├── tl_tcb705v_driver.launch.py
│   ├── tl_tcb710_driver.launch.py
│   └── tl_tcb710v_driver.launch.py
├── lib                            # API依赖库
│   ├── include
│   │   ├── c
│   │   │   ├── interface
│   │   │   │   ├── tl_c_craft_pallet.h
│   │   │   │   ├── tl_c_interface.h
│   │   │   │   ├── tl_c_io.h
│   │   │   │   ├── tl_c_job_operate.h
│   │   │   │   └── tl_c_queue_operate.h
│   │   │       ├── tl_define.h
│   │   │       └── tl_interface_parameter.h
│   │       │   ├── tl_api.h
│   │       │   ├── tl_craft_conveyor_belt_track.h
│   │       │   ├── tl_craft_laser_cutting.h
│   │       │   ├── tl_craft_pallet.h
│   │       │   ├── tl_craft_track.h
│   │       │   ├── tl_craft_vision.h
│   │       │   ├── tl_craft_weld.h
│   │       │   ├── tl_dual_arm.h
│   │       │   ├── tl_interface.h
│   │       │   ├── tl_io.h
│   │       │   ├── tl_job_operate.h
│   │       │   ├── tl_modbus.h
│   │       │   ├── tl_queue_operate.h
│   │       │   ├── tl_track.h
│   │       │   └── tl_vfd_ctr.h
│   │           ├── tl_craft_conveyor_belt_track_parameter.h
│   │           ├── tl_craft_laser_cutting_parameter.h
│   │           ├── tl_craft_track_parameter.h
│   │           ├── tl_craft_vision_parameter.h
│   │           ├── tl_craft_weld_parameter.h
│   │           ├── tl_define.h
│   │           ├── tl_interface_parameter.h
│   │           ├── tl_io_parameter.h
│   │           ├── tl_modbus_parameter.h
│   │           └── tl_parameter.h
│   ├── _tl_host.so
│   └── tl_interface.py
├── package.xml
├── README.md
├── README_update.md             
├── src                        # 驱动代码源文件
│   └── tl_driver.cpp
└── test                       # 测试脚本
    ├── test_job_insert_moveJ.sh
    ├── test_job_insert_moveL.sh
    ├── test_moveJ.sh
    ├── test_moveL.sh
    └── test_publisher.py
```
## tl_driver功能包话题与服务说明
tl_driver功能包的服务和话题较多，可以通过如下指令了解其话题信息。

![image](doc/tl_driver3.png)  
![image](doc/tl_driver4.png)  
主要为套用API实现的一些机械臂本体的功能，其详细介绍和使用在此不详细展开，可以通过专门的文档《[tl_driver服务与话题说明书](doc/tl_driver服务与话题说明书.md)》进行查看。
