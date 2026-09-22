# ServoL 笛卡尔空间直线伺服运动指令
# 前置条件：需先调用 open_servoj 打开关节跟踪模式
# 每次收到该话题消息，节点自动获取当前位姿，插值并 IK 转为关节角后通过 servoj 发送
# 单位约定：位置为 mm（本仓库采用 mm，非 ROS 惯例的 m）；姿态为 rad
float64[] target_pose     # [x, y, z, rx, ry, rz] 目标笛卡尔位姿：x/y/z 单位: mm，rx/ry/rz 单位: rad
float64 step_size          # 插值步长 单位: mm；相邻插值点之间的最大笛卡尔距离，驱动内默认 2.0（传入 ≤0 时取默认）
int32 coord                # 坐标系：1=基座标系(Base) 2=工具坐标系(Tool) 3=用户坐标系(User)；target_pose 位置 mm、姿态 rad
