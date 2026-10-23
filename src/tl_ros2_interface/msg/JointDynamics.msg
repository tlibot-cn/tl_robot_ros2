# 关节动力学状态（/tl_driver/joint_dynamics，默认 10 Hz）
# 数据源为控制器 6001 端口的同步查询；驱动边界换算为 REP 103。
# 数组定长，元素顺序与关节一致；查询失败或数据不足时缺失元素为 NaN（quiet_NaN），消费方用 std::isnan() 判定。
std_msgs/Header header
float64 line_speed              # 线速度，单位：m/s；查询失败时为 NaN
float64[7] joint_speed          # 关节速度，单位：rad/s；最多 7 轴，缺失为 NaN
float64[5] joint_speed_sync     # 外部轴同步速度，单位：rad/s；缺失为 NaN
float64[7] motor_current        # 电机电流，单位：A（SDK 头标 ‰，驱动未换算，沿用现有 ROS 口径）；缺失为 NaN
float64[5] motor_current_sync   # 外部轴同步电机电流，单位：A（同上）；缺失为 NaN
float64[7] motor_torque         # 电机力矩，单位：额定扭矩千分之一（‰，±1000 对应 ±100%；不是 N·m，也不是 %）
float64[5] motor_torque_sync    # 外部轴同步电机力矩，单位：额定扭矩千分之一（‰）
