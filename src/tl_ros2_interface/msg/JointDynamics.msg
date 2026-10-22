# 关节动力学状态（/tl_driver/joint_dynamics，默认 10 Hz）
# 数据源为控制器 6001 端口的同步查询；驱动边界换算为 REP 103。
std_msgs/Header header
float64 line_speed              # 线速度，单位：m/s
float64[] joint_speed           # 关节速度，单位：rad/s
float64[] joint_speed_sync      # 外部轴同步速度，单位：rad/s
float64[] motor_current         # 电机电流，单位：A
float64[] motor_current_sync    # 外部轴同步电机电流，单位：A
int32[] motor_torque            # 电机力矩，单位：额定扭矩千分之一（‰，±1000 对应 ±100%；不是 N·m，也不是 %）
int32[] motor_torque_sync       # 外部轴同步电机力矩，单位：额定扭矩千分之一（‰）
