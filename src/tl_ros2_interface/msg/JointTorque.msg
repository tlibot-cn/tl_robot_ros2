# 关节力矩（/tl_driver/joint_torque，默认 100 Hz，feedback_rate 可配）
# 数据源为控制器 6001 端口的同步查询。
# 单位：额定扭矩千分之一（‰，±1000 对应 ±100%；不是 N·m，也不是 %）。
# 定长数组；超出实际轴数或查询失败时缺失元素为 NaN（quiet_NaN）。
std_msgs/Header header
float64[7] motor_torque         # 本体关节力矩，单位：‰ 额定扭矩；缺失为 NaN
