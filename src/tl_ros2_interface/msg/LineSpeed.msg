# 末端线速度（/tl_driver/line_speed，默认 100 Hz，feedback_rate 可配）
# 数据源为控制器 6001 端口的同步查询；单位 m/s。
# 与 /tl_driver/joint_speed 由同一次查询产生，共享 feedback_rate。
std_msgs/Header header
float64 line_speed              # 末端线速度，单位：m/s；查询失败时为 NaN
