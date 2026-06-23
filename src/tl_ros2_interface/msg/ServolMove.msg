# ServoL 笛卡尔空间直线伺服运动指令
# 前置条件：需先调用 open_servoj 打开关节跟踪模式
# 每次收到该话题消息，节点自动获取当前位姿，插值并 IK 转为关节角后通过 servoj 发送
float64[] target_pose     # [x, y, z, rx, ry, rz] 目标笛卡尔位姿
float64 step_size          # 插值步长 (mm)，相邻插值点之间的最大笛卡尔距离，默认 2.0
int32 coord                # 坐标系：1=基座标系(Base) 2=工具坐标系(Tool) 3=用户坐标系(User)
