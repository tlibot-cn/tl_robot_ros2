# 关节监测（/tl_driver/joint_monitor，默认 2 Hz，joint_monitor_rate 可配，0=关闭）
# 同一回调内依次查询电流、温度、电压，整帧覆盖，不跨帧复用旧值。
# 数组定长，元素顺序与关节一致；查询失败或数据不足时缺失元素为 NaN（quiet_NaN）。
# header.stamp 为本帧查询完成后的采样时刻。
std_msgs/Header header
float64[7] motor_current        # 电机电流，单位：A（SDK 头标 ‰，驱动未换算，沿用现有 ROS 口径）；缺失为 NaN
float64[7] temperature          # 关节温度，单位：°C；缺失为 NaN
float64[7] joint_voltage        # 关节电压，单位：V；缺失为 NaN
